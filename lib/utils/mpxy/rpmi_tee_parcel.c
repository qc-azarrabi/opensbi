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
	if (!block_cnt || block_cnt > RPMI_PARCEL_MAX_BLOCKS)
		goto invalid;
	if (flags & RPMI_TEE_PARCEL_CREATE_FLAG_MULTI_SEGMENT)
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

	p->state = RPMI_PARCEL_CREATED;
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

int rpmi_tee_parcel_accept(void *msgbuf, u32 msg_len,
			   void *respbuf, u32 resp_max_len,
			   unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_accept_resp *resp = respbuf;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	resp->status = cpu_to_le32(RPMI_ERR_NOTSUPP);
	resp->flags = 0;
	resp->page_cnt = 0;
	resp->block_cnt = 0;
	*resp_len = sizeof(*resp);
	return SBI_OK;
}

int rpmi_tee_parcel_release(void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *resp_len)
{
	struct rpmi_tee_mem_parcel_release_resp *resp = respbuf;

	if (resp_max_len < sizeof(*resp))
		return SBI_ENOMEM;

	resp->status = cpu_to_le32(RPMI_ERR_NOTSUPP);
	*resp_len = sizeof(*resp);
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
