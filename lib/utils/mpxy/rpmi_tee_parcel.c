/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * RPMI TEE memory parcel manager.
 *
 * Framework-side bookkeeping and validation state machine for the RPMI TEE
 * memory parcel lifecycle. A parcel is a firmware-tracked handle describing a
 * scatter-gather list of 4kB-page blocks plus per-receiver access rights and a
 * lifecycle: created -> accepted -> released -> reclaimed.
 *
 * Ownership of every described block is validated against the calling (REE)
 * domain via sbi_domain_check_addr_range(). No PMP or domain region is mutated
 * at runtime: in this model the trusted domain already holds full memory
 * access, so the manager enforces the protocol and validates ownership without
 * altering the physical isolation.
 */

#include <sbi/sbi_error.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_string.h>
#include <sbi/riscv_locks.h>
#include <sbi/riscv_encoding.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>
#include <sbi_utils/mpxy/rpmi_tee_parcel.h>

#define RPMI_PARCEL_POOL_SIZE		16
#define RPMI_PARCEL_MAX_RECEIVERS	4
#define RPMI_PARCEL_MAX_BLOCKS		64
#define RPMI_PARCEL_LABEL_LEN		RPMI_TEE_PARCEL_BLOCK_LABEL_LEN

enum rpmi_parcel_state {
	RPMI_PARCEL_FREE = 0,
	RPMI_PARCEL_CONSTRUCTING,
	RPMI_PARCEL_CREATED,
	RPMI_PARCEL_ACCEPTED,
	RPMI_PARCEL_RELEASED,
	RPMI_PARCEL_DESTROYED,
};

struct rpmi_parcel_block {
	u64 base;		/* byte address, host order */
	u32 page_count;
};

struct rpmi_parcel {
	enum rpmi_parcel_state state;
	u32 id;			/* (gen << 8) | slot ; 0 = invalid */
	u32 gen;
	u32 creator_id;
	u32 creator_access;
	u32 flags;
	u32 nonce;
	u8 label[RPMI_PARCEL_LABEL_LEN];
	u32 receiver_cnt;
	u32 receiver_id[RPMI_PARCEL_MAX_RECEIVERS];
	u32 receiver_access[RPMI_PARCEL_MAX_RECEIVERS];
	bool accepted[RPMI_PARCEL_MAX_RECEIVERS];
	bool released[RPMI_PARCEL_MAX_RECEIVERS];
	u32 block_cnt;
	struct rpmi_parcel_block blocks[RPMI_PARCEL_MAX_BLOCKS];
	u64 total_pages;
	u32 next_segment_idx;	/* multi-segment construction cursor */
	u32 next_receive_idx;	/* multi-segment receive cursor */
};

static struct rpmi_parcel parcel_pool[RPMI_PARCEL_POOL_SIZE];
static DEFINE_SPIN_LOCK(parcel_lock);
static bool parcel_pool_ready;

void rpmi_tee_parcel_init(void)
{
	int i;

	spin_lock(&parcel_lock);
	if (parcel_pool_ready) {
		spin_unlock(&parcel_lock);
		return;
	}

	for (i = 0; i < RPMI_PARCEL_POOL_SIZE; i++) {
		sbi_memset(&parcel_pool[i], 0, sizeof(parcel_pool[i]));
		parcel_pool[i].state = RPMI_PARCEL_FREE;
	}

	parcel_pool_ready = true;
	spin_unlock(&parcel_lock);
}

/* Build a stored handle from slot index and generation; id 0 is never valid. */
static u32 parcel_make_id(u32 slot, u32 gen)
{
	u32 id = (gen << 8) | (slot & 0xff);

	return id ? id : ((1u << 8) | (slot & 0xff));
}

/* Resolve a handle to a live slot via exact stored-id equality. */
static struct rpmi_parcel *parcel_lookup(u32 id)
{
	u32 slot;

	if (!id)
		return NULL;

	slot = id & 0xff;
	if (slot >= RPMI_PARCEL_POOL_SIZE)
		return NULL;

	if (parcel_pool[slot].state == RPMI_PARCEL_FREE ||
	    parcel_pool[slot].id != id)
		return NULL;

	return &parcel_pool[slot];
}

/* Recycle a slot: bump generation so stale handles fail lookup. */
static void parcel_recycle(struct rpmi_parcel *p)
{
	u32 gen = p->gen + 1;

	sbi_memset(p, 0, sizeof(*p));
	p->state = RPMI_PARCEL_FREE;
	p->gen = gen;
	p->id = 0;
}

/* True once every accepted receiver has released its access. */
static bool parcel_all_released(const struct rpmi_parcel *p)
{
	u32 i;

	for (i = 0; i < p->receiver_cnt; i++) {
		/* If not ACCEPTED yet, then considered released. */
		if (p->accepted[i] && !p->released[i])
			return false;
		}
	return true;
}

/*
 * Validate ownership of and append a batch of blocks to a parcel under
 * construction. blk_high[n]/blk_low[n] are the block-list arrays in wire order.
 * Returns an RPMI_* status; the parcel lock must be held by the caller.
 */
static int parcel_append_blocks(struct rpmi_parcel *p, const u32 *blk_high,
				const u32 *blk_low, u32 n)
{
	u32 i;

	if (p->block_cnt + n > RPMI_PARCEL_MAX_BLOCKS)
		return RPMI_ERR_INVALID_PARAM;

	for (i = 0; i < n; i++) {
		u32 high = le32_to_cpu(blk_high[i]);
		u32 low = le32_to_cpu(blk_low[i]);
		u64 page = RPMI_TEE_PARCEL_BLOCK_PAGE_NUM(high, low);
		u32 pages = RPMI_TEE_PARCEL_BLOCK_PAGES(low);
		u64 base = page << 12;
		u64 size = (u64)pages << 12;

		if (!sbi_domain_check_addr_range(sbi_domain_thishart_ptr(),
						 base, size, PRV_S,
						 SBI_DOMAIN_READ |
						 SBI_DOMAIN_WRITE))
			return RPMI_ERR_DENIED;

		p->blocks[p->block_cnt].base = base;
		p->blocks[p->block_cnt].page_count = pages;
		p->block_cnt++;
		p->total_pages += pages;
	}

	return RPMI_SUCCESS;
}

int rpmi_tee_parcel_create(void *msgbuf, u32 msg_len,
			   void *respbuf, u32 resp_max_len,
			   unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_create_req *req = msgbuf;
	struct rpmi_tee_mem_parcel_create_resp *resp = respbuf;
	u32 receiver_cnt, block_cnt, flags, creator_access, creator_id;
	const u32 *rx_id, *rx_access, *blk_high, *blk_low;
	u32 parcel_access, dom_access;
	struct rpmi_parcel *p = NULL;
	u64 total_pages = 0;
	u32 expected, i, slot;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	if (msg_len < sizeof(*req))
		goto invalid;

	creator_id = le32_to_cpu(req->creator_id);
	creator_access = le32_to_cpu(req->creator_access);
	receiver_cnt = le32_to_cpu(req->receiver_cnt);
	flags = le32_to_cpu(req->flags);
	block_cnt = le32_to_cpu(req->block_cnt);

	/* Reject deferred multi-segment operation. */
	if (creator_id != RPMI_TEE_ENDPOINT_REE)
		goto invalid;
	if (!receiver_cnt || receiver_cnt > RPMI_PARCEL_MAX_RECEIVERS)
		goto invalid;
	if (block_cnt > RPMI_PARCEL_MAX_BLOCKS)
		goto invalid;
	/*
	 * A MULTI_SEGMENT parcel is built incrementally: CREATE carries the
	 * receiver metadata plus an optional first batch of blocks (possibly
	 * none) and leaves the parcel "constructing"; SEGMENT_SEND appends the
	 * remaining blocks and the LAST segment finalizes it. A single-shot
	 * CREATE must carry at least one block.
	 */
	if (!(flags & RPMI_TEE_PARCEL_CREATE_FLAG_MULTI_SEGMENT) && !block_cnt)
		goto invalid;

	/* data[] = receiver_id[N] access[N] block_high[M] block_low[M] */
	expected = sizeof(*req) +
		   (2 * receiver_cnt + 2 * block_cnt) * sizeof(u32);
	if (msg_len < expected)
		goto invalid;

	/* Owner-transfer (donate): the creator must keep no access, and the
	 * transfer targets exactly one receiver.
	 */
	if ((flags & RPMI_TEE_PARCEL_CREATE_FLAG_OWNER_XFER) &&
	    ((creator_access & RPMI_TEE_PARCEL_ACCESS_MASK) ||
	     receiver_cnt != 1))
		goto invalid;

	rx_id = &req->data[0];
	rx_access = &req->data[receiver_cnt];
	blk_high = &req->data[2 * receiver_cnt];
	blk_low = &req->data[2 * receiver_cnt + block_cnt];

	/*
	 * The spec bounds each receiver's grant by the creator's actual
	 * access to the memory, not by creator_access (the residual access
	 * it elects to keep for itself afterward) - this manager has no way
	 * to query a domain's actual access rights (only fixed-flag boolean
	 * checks), so receiver access is trusted as supplied and enforced
	 * only via the domain-ownership check below.
	 */
	parcel_access = creator_access & RPMI_TEE_PARCEL_ACCESS_MASK;
	for (i = 0; i < receiver_cnt; i++) {
		u32 acc = le32_to_cpu(rx_access[i]) & RPMI_TEE_PARCEL_ACCESS_MASK;

		parcel_access |= acc;
	}

	/* Only require domain ownership for the access actually granted. */
	dom_access = 0;
	if (parcel_access & RPMI_TEE_PARCEL_ACCESS_R)
		dom_access |= SBI_DOMAIN_READ;
	if (parcel_access & RPMI_TEE_PARCEL_ACCESS_W)
		dom_access |= SBI_DOMAIN_WRITE;
	if (parcel_access & RPMI_TEE_PARCEL_ACCESS_X)
		dom_access |= SBI_DOMAIN_EXECUTE;

	/* Validate that the REE domain owns every described block. */
	for (i = 0; i < block_cnt; i++) {
		u32 high = le32_to_cpu(blk_high[i]);
		u32 low = le32_to_cpu(blk_low[i]);
		u64 page = RPMI_TEE_PARCEL_BLOCK_PAGE_NUM(high, low);
		u32 pages = RPMI_TEE_PARCEL_BLOCK_PAGES(low);
		u64 base = page << 12;
		u64 size = (u64)pages << 12;

		if (!sbi_domain_check_addr_range(sbi_domain_thishart_ptr(),
						 base, size, PRV_S,
						 dom_access)) {
			resp->status = cpu_to_le32(RPMI_ERR_DENIED);
			resp->mem_parcel_id = 0;
			*resp_len = sizeof(*resp);
			return SBI_OK;
		}
		total_pages += pages;
	}

	/* Allocate a free slot. */
	spin_lock(&parcel_lock);
	for (slot = 0; slot < RPMI_PARCEL_POOL_SIZE; slot++) {
		if (parcel_pool[slot].state == RPMI_PARCEL_FREE) {
			p = &parcel_pool[slot];
			break;
		}
	}
	if (!p) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_FAILED);
		resp->mem_parcel_id = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	p->state = (flags & RPMI_TEE_PARCEL_CREATE_FLAG_MULTI_SEGMENT) ?
		   RPMI_PARCEL_CONSTRUCTING : RPMI_PARCEL_CREATED;
	p->id = parcel_make_id(slot, p->gen);
	p->creator_id = creator_id;
	p->creator_access = creator_access;
	p->flags = flags;
	p->nonce = le32_to_cpu(req->nonce);
	sbi_memcpy(p->label, req->label, RPMI_PARCEL_LABEL_LEN);
	p->receiver_cnt = receiver_cnt;
	for (i = 0; i < receiver_cnt; i++) {
		p->receiver_id[i] = le32_to_cpu(rx_id[i]);
		p->receiver_access[i] =
			le32_to_cpu(rx_access[i]) & RPMI_TEE_PARCEL_ACCESS_MASK;
		p->accepted[i] = false;
		p->released[i] = false;
	}
	p->block_cnt = block_cnt;
	for (i = 0; i < block_cnt; i++) {
		u32 high = le32_to_cpu(blk_high[i]);
		u32 low = le32_to_cpu(blk_low[i]);

		p->blocks[i].base =
			RPMI_TEE_PARCEL_BLOCK_PAGE_NUM(high, low) << 12;
		p->blocks[i].page_count = RPMI_TEE_PARCEL_BLOCK_PAGES(low);
	}
	p->total_pages = total_pages;
	p->next_segment_idx = 0;

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->mem_parcel_id = cpu_to_le32(p->id);
	*resp_len = sizeof(*resp);
	spin_unlock(&parcel_lock);
	return SBI_OK;

invalid:
	resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
	resp->mem_parcel_id = 0;
	*resp_len = sizeof(*resp);
	return SBI_OK;
}

/* Find a receiver slot by endpoint id; returns index or -1 if not listed. */
static int parcel_receiver_index(const struct rpmi_parcel *p, u32 endpoint_id)
{
	u32 i;

	for (i = 0; i < p->receiver_cnt; i++)
		if (p->receiver_id[i] == endpoint_id)
			return (int)i;
	return -1;
}

/* Fill a fixed-header ACCEPT error response; always returns SBI_OK. */
static int parcel_accept_error(struct rpmi_tee_mem_parcel_accept_resp *resp,
			       unsigned long *resp_len, u32 status)
{
	resp->status = cpu_to_le32(status);
	resp->flags = 0;
	resp->page_cnt = 0;
	resp->block_cnt = 0;
	*resp_len = sizeof(*resp);
	return SBI_OK;
}

int rpmi_tee_parcel_accept(void *msgbuf, u32 msg_len,
			   void *respbuf, u32 resp_max_len,
			   unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_accept_req *req = msgbuf;
	struct rpmi_tee_mem_parcel_accept_resp *resp = respbuf;
	u32 id, acceptor_id, access, nonce, creator_id, creator_access;
	u32 flags, max_pages, other_cnt, expected, resp_bytes, i;
	struct rpmi_parcel *p;
	bool segmented = false;
	int idx;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	if (msg_len < sizeof(*req))
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);

	acceptor_id = le32_to_cpu(req->acceptor_id);
	access = le32_to_cpu(req->access) & RPMI_TEE_PARCEL_ACCESS_MASK;
	id = le32_to_cpu(req->mem_parcel_id);
	nonce = le32_to_cpu(req->nonce);
	creator_id = le32_to_cpu(req->creator_id);
	creator_access = le32_to_cpu(req->creator_access);
	flags = le32_to_cpu(req->flags);
	max_pages = le32_to_cpu(req->max_pages);
	other_cnt = le32_to_cpu(req->other_cnt);

	/* Deferred multi-segment accept is rejected. */
	if (flags & RPMI_TEE_PARCEL_ACCEPT_RESP_FLAG_MULTI_SEGMENT)
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);

	if (other_cnt > RPMI_PARCEL_MAX_RECEIVERS)
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);

	/* data[] = other_id[other_cnt] other_access[other_cnt] */
	expected = sizeof(*req) + 2 * other_cnt * sizeof(u32);
	if (msg_len < expected)
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);

	spin_lock(&parcel_lock);
	p = parcel_lookup(id);
	if (!p || (p->state != RPMI_PARCEL_CREATED &&
		   p->state != RPMI_PARCEL_ACCEPTED)) {
		spin_unlock(&parcel_lock);
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);
	}

	/* The accept must match the parcel's creator identity and nonce. */
	if (nonce != p->nonce || creator_id != p->creator_id ||
	    creator_access != p->creator_access) {
		spin_unlock(&parcel_lock);
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);
	}

	/* The acceptor must be a listed receiver. */
	idx = parcel_receiver_index(p, acceptor_id);
	if (idx < 0) {
		spin_unlock(&parcel_lock);
		return parcel_accept_error(resp, resp_len, RPMI_ERR_DENIED);
	}

	/* Requested access must be a subset of the receiver's grant. */
	if (access & ~p->receiver_access[idx]) {
		spin_unlock(&parcel_lock);
		return parcel_accept_error(resp, resp_len, RPMI_ERR_DENIED);
	}

	/* The acceptor's advertised capacity must cover the parcel. */
	if (max_pages && p->total_pages > max_pages) {
		spin_unlock(&parcel_lock);
		return parcel_accept_error(resp, resp_len,
					   RPMI_ERR_INVALID_PARAM);
	}

	/*
	 * Variable-length response: fixed header + block_high[K] block_low[K],
	 * where K is as many blocks as fit in resp_max_len. If the parcel has
	 * more blocks than fit, set the MULTI_SEGMENT response flag; the
	 * acceptor then pulls the remainder via SEGMENT_RECEIVE. block_cnt here
	 * counts the blocks returned in THIS response (equal to the total in the
	 * common single-response case, so existing single-shot readers are
	 * unaffected).
	 */
	{
	u32 max_fit = (resp_max_len - sizeof(*resp)) / (2 * sizeof(u32));
	u32 ret_blocks = p->block_cnt;
	bool more = false;

	if (ret_blocks > max_fit) {
		ret_blocks = max_fit;
		more = true;
	}
	resp_bytes = sizeof(*resp) + 2 * ret_blocks * sizeof(u32);

	for (i = 0; i < ret_blocks; i++) {
		u64 page = p->blocks[i].base >> 12;
		u32 pages = p->blocks[i].page_count;
		u32 high = (u32)(page >> 20);
		u32 low = (u32)(((page & 0xFFFFFU) << 12) | (pages - 1));

		resp->data[i] = cpu_to_le32(high);
		resp->data[ret_blocks + i] = cpu_to_le32(low);
	}

	p->accepted[idx] = true;
	p->state = RPMI_PARCEL_ACCEPTED;
	/*
	 * Reset the shared SEGMENT_RECEIVE cursor for this accept. One cursor
	 * (not per-acceptor) is sufficient: the block list is shared/immutable
	 * and nothing in this design has multiple acceptors pulling segments
	 * from the same parcel concurrently.
	 */
	p->next_receive_idx = 0;

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = more ?
		cpu_to_le32(RPMI_TEE_PARCEL_ACCEPT_RESP_FLAG_MULTI_SEGMENT) : 0;
	resp->page_cnt = cpu_to_le32((u32)p->total_pages);
	resp->block_cnt = cpu_to_le32(ret_blocks);
	*resp_len = resp_bytes;
	segmented = more;
	}

	/*
	 * Owner-transfer (donate): ownership moves to the acceptor, so the
	 * creator can never reclaim it. Destroy the handle on accept; a later
	 * reclaim of the same id then fails lookup.
	 *
	 * If the block list did not fit in this response (segmented accept),
	 * the acceptor must still pull the remainder via SEGMENT_RECEIVE, so
	 * the parcel has to survive until the LAST segment is delivered. In
	 * that case defer the destroy to SEGMENT_RECEIVE; destroying here would
	 * make the follow-up SEGMENT_RECEIVE fail lookup and strand the accept.
	 */
	if ((p->flags & RPMI_TEE_PARCEL_CREATE_FLAG_OWNER_XFER) && !segmented) {
		p->state = RPMI_PARCEL_DESTROYED;
		parcel_recycle(p);
	}

	spin_unlock(&parcel_lock);
	return SBI_OK;
}

int rpmi_tee_parcel_release(void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_release_req *req = msgbuf;
	struct rpmi_tee_mem_parcel_release_resp *resp = respbuf;
	u32 id, endpoint_cnt, expected, i;
	struct rpmi_parcel *p;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	if (msg_len < sizeof(*req)) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	id = le32_to_cpu(req->mem_parcel_id);
	endpoint_cnt = le32_to_cpu(req->endpoint_cnt);

	if (!endpoint_cnt || endpoint_cnt > RPMI_PARCEL_MAX_RECEIVERS) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	expected = sizeof(*req) + endpoint_cnt * sizeof(u32);
	if (msg_len < expected) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	spin_lock(&parcel_lock);
	p = parcel_lookup(id);
	if (!p) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	/* Every listed endpoint must currently hold an accepted, unreleased grant. */
	for (i = 0; i < endpoint_cnt; i++) {
		u32 ep = le32_to_cpu(req->endpoint_id[i]);
		int idx = parcel_receiver_index(p, ep);

		if (idx < 0 || !p->accepted[idx] || p->released[idx]) {
			spin_unlock(&parcel_lock);
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_STATE);
			*resp_len = sizeof(*resp);
			return SBI_OK;
		}
	}

	for (i = 0; i < endpoint_cnt; i++) {
		u32 ep = le32_to_cpu(req->endpoint_id[i]);
		int idx = parcel_receiver_index(p, ep);

		p->released[idx] = true;
	}

	if (parcel_all_released(p))
		p->state = RPMI_PARCEL_RELEASED;

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	*resp_len = sizeof(*resp);
	spin_unlock(&parcel_lock);
	return SBI_OK;
}

int rpmi_tee_parcel_reclaim(void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_reclaim_req *req = msgbuf;
	struct rpmi_tee_mem_parcel_reclaim_resp *resp = respbuf;
	struct rpmi_parcel *p;
	u32 id;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	if (msg_len < sizeof(*req)) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		resp->flags = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	id = le32_to_cpu(req->mem_parcel_id);

	spin_lock(&parcel_lock);
	p = parcel_lookup(id);
	if (!p) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		resp->flags = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	/* Reclaim is only allowed once every accepted receiver has released. */
	if (!parcel_all_released(p)) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_DENIED);
		resp->flags = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	parcel_recycle(p);
	spin_unlock(&parcel_lock);

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = 0;
	*resp_len = sizeof(*resp);
	return SBI_OK;
}

/*
 * SEGMENT_SEND (0x0D): the REE streams the remaining block-list segments of a
 * MULTI_SEGMENT parcel still under construction. Each carries up to
 * RPMI_TEE_PARCEL_SEGMENT_MAX_BLOCKS blocks so the request stays within the
 * transport slot; ordering is enforced implicitly by appending at the
 * server's own next_segment_idx cursor (the spec carries no segment index on
 * the wire). The LAST-flagged segment finalizes the parcel to CREATED.
 * data[] = block_high[block_cnt] block_low[block_cnt].
 */
int rpmi_tee_parcel_segment_send(void *msgbuf, u32 msg_len,
				 void *respbuf, u32 resp_max_len,
				 unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_segment_send_req *req = msgbuf;
	struct rpmi_tee_mem_parcel_segment_send_resp *resp = respbuf;
	u32 id, flags, block_cnt, expected;
	const u32 *blk_high, *blk_low;
	struct rpmi_parcel *p;
	int rc;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	if (msg_len < sizeof(*req)) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	id = le32_to_cpu(req->mem_parcel_id);
	flags = le32_to_cpu(req->flags);
	block_cnt = le32_to_cpu(req->block_cnt);

	if (block_cnt > RPMI_TEE_PARCEL_SEGMENT_MAX_BLOCKS) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	/* data[] = block_high[block_cnt] block_low[block_cnt] */
	expected = sizeof(*req) + 2 * block_cnt * sizeof(u32);
	if (msg_len < expected) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	blk_high = &req->data[0];
	blk_low = &req->data[block_cnt];

	spin_lock(&parcel_lock);
	p = parcel_lookup(id);
	if (!p || p->state != RPMI_PARCEL_CONSTRUCTING) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_STATE);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	rc = parcel_append_blocks(p, blk_high, blk_low, block_cnt);
	if (rc != RPMI_SUCCESS) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(rc);
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	p->next_segment_idx++;

	/* LAST finalizes: a MULTI_SEGMENT parcel must end with >=1 block. */
	if (flags & RPMI_TEE_PARCEL_SEGMENT_FLAG_LAST) {
		if (!p->block_cnt) {
			spin_unlock(&parcel_lock);
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			return SBI_OK;
		}
		p->state = RPMI_PARCEL_CREATED;
	}

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	*resp_len = sizeof(*resp);
	spin_unlock(&parcel_lock);
	return SBI_OK;
}

/*
 * SEGMENT_RECEIVE (0x0E): an acceptor pulls the block list back in segments
 * when it did not fit in the ACCEPT response. The server tracks the receive
 * cursor itself (p->next_receive_idx, reset on ACCEPT); the acceptor supplies
 * only its identity, which must match a registered, already-accepted
 * receiver of this parcel. Each call returns as many blocks as fit in the
 * response, setting the LAST flag once the final block is included.
 * data[] = block_high[block_cnt] block_low[block_cnt].
 */
int rpmi_tee_parcel_segment_receive(void *msgbuf, u32 msg_len,
				    void *respbuf, u32 resp_max_len,
				    unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_segment_receive_req *req = msgbuf;
	struct rpmi_tee_mem_parcel_segment_receive_resp *resp = respbuf;
	u32 id, acceptor_id, max_fit, ret_blocks, i;
	struct rpmi_parcel *p;
	int idx;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	if (msg_len < sizeof(*req)) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		resp->flags = 0;
		resp->block_cnt = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	id = le32_to_cpu(req->mem_parcel_id);
	acceptor_id = le32_to_cpu(req->acceptor_id);

	spin_lock(&parcel_lock);
	p = parcel_lookup(id);
	if (!p || (p->state != RPMI_PARCEL_CREATED &&
		   p->state != RPMI_PARCEL_ACCEPTED)) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_STATE);
		resp->flags = 0;
		resp->block_cnt = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	/* The acceptor must be a listed, already-accepted receiver. */
	idx = parcel_receiver_index(p, acceptor_id);
	if (idx < 0 || !p->accepted[idx]) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_DENIED);
		resp->flags = 0;
		resp->block_cnt = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	if (p->next_receive_idx >= p->block_cnt) {
		spin_unlock(&parcel_lock);
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		resp->flags = 0;
		resp->block_cnt = 0;
		*resp_len = sizeof(*resp);
		return SBI_OK;
	}

	max_fit = (resp_max_len - sizeof(*resp)) / (2 * sizeof(u32));
	if (max_fit > RPMI_TEE_PARCEL_SEGMENT_MAX_BLOCKS)
		max_fit = RPMI_TEE_PARCEL_SEGMENT_MAX_BLOCKS;
	ret_blocks = p->block_cnt - p->next_receive_idx;
	if (ret_blocks > max_fit)
		ret_blocks = max_fit;

	for (i = 0; i < ret_blocks; i++) {
		u64 page = p->blocks[p->next_receive_idx + i].base >> 12;
		u32 pages = p->blocks[p->next_receive_idx + i].page_count;
		u32 high = (u32)(page >> 20);
		u32 low = (u32)(((page & 0xFFFFFU) << 12) | (pages - 1));

		resp->data[i] = cpu_to_le32(high);
		resp->data[ret_blocks + i] = cpu_to_le32(low);
	}

	p->next_receive_idx += ret_blocks;

	{
	bool last = (p->next_receive_idx >= p->block_cnt);

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = last ?
		cpu_to_le32(RPMI_TEE_PARCEL_SEGMENT_FLAG_LAST) : 0;
	resp->block_cnt = cpu_to_le32(ret_blocks);
	*resp_len = sizeof(*resp) + 2 * ret_blocks * sizeof(u32);

	/*
	 * A donate parcel's destroy was deferred by ACCEPT when its block list
	 * had to be segmented (see rpmi_tee_parcel_accept). Ownership has now
	 * fully transferred once the acceptor pulls the LAST segment, so retire
	 * the handle here; a later reclaim of the same id then fails lookup.
	 */
	if (last && (p->flags & RPMI_TEE_PARCEL_CREATE_FLAG_OWNER_XFER)) {
		p->state = RPMI_PARCEL_DESTROYED;
		parcel_recycle(p);
	}
	}
	spin_unlock(&parcel_lock);
	return SBI_OK;
}

/*
 * Minimal CBOR encoder for the fixed PROBE_SYSTEM system-info map. Only the
 * subset needed here is implemented: short definite-length maps, short
 * text-string keys, and unsigned integers. Each helper appends to buf[] at
 * *off, never writing past cap; on overflow it keeps advancing *off so the
 * caller can detect truncation by comparing the returned length against cap.
 * No general CBOR support is intended.
 */
static void cbor_put(u8 *buf, u32 cap, u32 *off, u8 b)
{
	if (*off < cap)
		buf[*off] = b;
	(*off)++;
}

/* CBOR unsigned integer (major type 0). */
static void cbor_uint(u8 *buf, u32 cap, u32 *off, u32 v)
{
	if (v < 24) {
		cbor_put(buf, cap, off, (u8)v);
	} else if (v < 256) {
		cbor_put(buf, cap, off, 0x18);
		cbor_put(buf, cap, off, (u8)v);
	} else if (v < 65536) {
		cbor_put(buf, cap, off, 0x19);
		cbor_put(buf, cap, off, (u8)(v >> 8));
		cbor_put(buf, cap, off, (u8)v);
	} else {
		cbor_put(buf, cap, off, 0x1a);
		cbor_put(buf, cap, off, (u8)(v >> 24));
		cbor_put(buf, cap, off, (u8)(v >> 16));
		cbor_put(buf, cap, off, (u8)(v >> 8));
		cbor_put(buf, cap, off, (u8)v);
	}
}

/* CBOR map header (major type 5) for n <= 23 pairs. */
static void cbor_map(u8 *buf, u32 cap, u32 *off, u32 n)
{
	cbor_put(buf, cap, off, (u8)(0xa0 | (n & 0x1f)));
}

/* CBOR text string (major type 3) for a short (< 24 byte) ASCII key. */
static void cbor_key(u8 *buf, u32 cap, u32 *off, const char *s)
{
	u32 i, len = 0;

	while (s[len])
		len++;
	cbor_put(buf, cap, off, (u8)(0x60 | (len & 0x1f)));
	for (i = 0; i < len; i++)
		cbor_put(buf, cap, off, (u8)s[i]);
}

/*
 * Encode the parcel-manager system-info map into buf[] and return its encoded
 * length (which may exceed cap if it did not fit). The map advertises the
 * framework's fixed parcel capacities and supported modes so a discovering REE
 * need not hard-code them.
 */
static u32 parcel_encode_system_info(u8 *buf, u32 cap)
{
	u32 off = 0;

	cbor_map(buf, cap, &off, 5);
	cbor_key(buf, cap, &off, "pool_size");
	cbor_uint(buf, cap, &off, RPMI_PARCEL_POOL_SIZE);
	cbor_key(buf, cap, &off, "max_receivers");
	cbor_uint(buf, cap, &off, RPMI_PARCEL_MAX_RECEIVERS);
	cbor_key(buf, cap, &off, "max_blocks");
	cbor_uint(buf, cap, &off, RPMI_PARCEL_MAX_BLOCKS);
	cbor_key(buf, cap, &off, "seg_max_blocks");
	cbor_uint(buf, cap, &off, RPMI_TEE_PARCEL_SEGMENT_MAX_BLOCKS);
	/* modes bitmask: donate(1) | lend(2) | share(4). */
	cbor_key(buf, cap, &off, "modes");
	cbor_uint(buf, cap, &off, 0x7);

	return off;
}

/*
 * PROBE_SYSTEM (0x03): return a CBOR-encoded system-info blob describing the
 * parcel-manager capacities. Framework-answered; no TEE domain involvement.
 * The response carries the encoding format and the blob inline after the
 * fixed header.
 */
int rpmi_tee_parcel_probe_system(void *msgbuf, u32 msg_len,
				 void *respbuf, u32 resp_max_len,
				 unsigned long *resp_len)
{
	struct rpmi_tee_probe_system_resp *resp = respbuf;
	u32 cap, info_len;

	(void)msgbuf;
	(void)msg_len;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	cap = resp_max_len - sizeof(*resp);
	info_len = parcel_encode_system_info(resp->data, cap);
	if (info_len > cap)
		return SBI_ENOMEM;

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->format = cpu_to_le32(RPMI_TEE_SYSINFO_FORMAT_CBOR);
	resp->info_len = cpu_to_le32(info_len);
	*resp_len = sizeof(*resp) + info_len;
	return SBI_OK;
}
