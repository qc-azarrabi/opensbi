/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

#define TEE_MAX_PARCELS		64
#define TEE_MAX_PAGES		4096

struct tee_block {
	unsigned long address;
	unsigned long pages;
};

/* Blocks supplied by one CREATE or SEGMENT_SEND request. */
struct tee_block_batch {
	struct sbi_dlist node;
	u32 count;
	u32 page_count;
	struct tee_block blocks[];
};

/*
 * Parcel lifecycle. NONE means the parcel is absent from the registry.
 *
 * State     Successful operation  Next
 * --------  --------------------  --------
 * NONE      CREATE                CREATED
 * CREATED   ACCEPT                ACCEPTED
 * ACCEPTED  RELEASE               CREATED
 * CREATED   RECLAIM               NONE
 *
 * ''busy'' reserves a parcel for one operation.
 *   - CREATE sets it before publishing the provisional entry;
 *   - rpmi_tee_memory_request() sets it before dispatching requests for
 *     an existing parcel.
 *
 * Protection hooks temporarily release parcel_lock, but busy remains set.
 * Another request, including RECLAIM, returns BUSY rather than modifying or
 * freeing the parcel during that gap.
 *
 * CREATE clears busy on success and removes the entry on failure. For other
 * operations, the request wrapper clears busy after dispatch; successful
 * RECLAIM instead removes and frees the parcel.
 *
 * ''disconnected'' marks a parcel whose receiver has failed. Only RECLAIM is
 * allowed afterward.
 *   - If busy is clear, disconnect changes ACCEPTED to CREATED;
 *   - If busy is set, the request wrapper makes that change when the
 *     operation finishes.
 *
 * The parcel remains registered and disconnected until its creator
 * reclaims it.
 *
 * ''parcel_lock'' protects registry membership, lifecycle state and both flags.
 * Protection hooks run outside the lock to allow lengthy permission updates
 * and cross-hart synchronization without holding the global spinlock.
 */
enum tee_parcel_state {
	TEE_PARCEL_CREATED,
	TEE_PARCEL_ACCEPTED,
};

struct tee_parcel {
	struct sbi_dlist node;
	u32 id;
	u32 nonce;
	struct rpmi_tee_endpoint *creator;
	struct rpmi_tee_endpoint *receiver;
	u32 creator_access;
	u32 access;
	u32 page_count;
	u32 block_count;
	/* Block batches from CREATE and subsequent SEGMENT_SEND requests. */
	struct sbi_dlist batches;
	enum tee_parcel_state state;
	bool busy;
	bool disconnected;
};

/* Protects the registry, reservations and parcel lifecycle fields. */
static spinlock_t parcel_lock = SPIN_LOCK_INITIALIZER;
static SBI_LIST_HEAD(tee_parcels);
static u32 parcel_count;
static u32 parcel_next_id = 1;

#define tee_parcel_for_each(_parcel) \
	sbi_list_for_each_entry(_parcel, &tee_parcels, node)

#define tee_parcel_for_each_batch(_batch, _parcel) \
	sbi_list_for_each_entry(_batch, &(_parcel)->batches, node)

/* Memory-protection backend. */

#ifdef CONFIG_FDT_MPXY_RPMI_TEE_TEST_SHARED_RAM

/* Check that both endpoint domains have read/write access to every block. */
static int parcel_protect_check(struct tee_parcel *parcel)
{
	struct sbi_domain *creator, *receiver;
	unsigned long access = SBI_DOMAIN_READ | SBI_DOMAIN_WRITE;
	struct tee_block_batch *batch;
	u32 i;

	creator = rpmi_tee_endpoint_domain(parcel->creator);
	receiver = rpmi_tee_endpoint_domain(parcel->receiver);
	if (!creator || !receiver)
		return SBI_ENODEV;

	/* Sharing executable memory is unsupported for now. */
	if ((parcel->creator_access | parcel->access) & RPMI_TEE_ACCESS_EXEC)
		return SBI_ENOTSUPP;

	/* Check both domains against the trusted static access policy. */
	tee_parcel_for_each_batch(batch, parcel) {
		for (i = 0; i < batch->count; i++) {
			const struct tee_block *block = &batch->blocks[i];
			unsigned long size = block->pages <<
					     RPMI_TEE_PAGE_SHIFT;

			if (!sbi_domain_check_addr_range(creator,
						 block->address, size, PRV_S,
						 access) ||
			    !sbi_domain_check_addr_range(receiver,
						 block->address, size, PRV_S,
						 access))
				return SBI_EDENIED;
		}
	}
	return 0;
}

static int parcel_protect_create(struct tee_parcel *parcel)
{
	return 0;
}

static int parcel_protect_accept(struct tee_parcel *parcel)
{
	return 0;
}

static int parcel_protect_release(struct tee_parcel *parcel)
{
	return 0;
}

static int parcel_protect_reclaim(struct tee_parcel *parcel)
{
	return 0;
}

static int parcel_protect_disconnect(struct rpmi_tee_endpoint *ep)
{
	return 0;
}
#else
#error "RPMI TEE memory requires CONFIG_FDT_MPXY_RPMI_TEE_TEST_SHARED_RAM"
#endif

static int parcel_protect_run(struct tee_parcel *parcel,
			      int (*operation)(struct tee_parcel *parcel))
{
	int ret;

	/* After receiver failure (disconnected), reclamation is allowed. */
	if (operation != parcel_protect_reclaim &&
	    (parcel->disconnected ||
	     !rpmi_tee_endpoint_available(parcel->receiver)))
		return SBI_EDENIED;

	spin_unlock(&parcel_lock);
	ret = operation(parcel);
	spin_lock(&parcel_lock);

	return ret;
}

/* Registry and block helpers. */

static s32 parcel_error(int error)
{
	switch (error) {
	case SBI_ENOTSUPP:
		return RPMI_ERR_NOTSUPP;
	case SBI_EINVAL:
		return RPMI_ERR_INVALID_PARAM;
	case SBI_EDENIED:
		return RPMI_ERR_DENIED;
	case SBI_EALREADY:
	case SBI_ENOSPC:
		return RPMI_ERR_BUSY;
	case SBI_ENOMEM:
		return RPMI_ERR_FAILED;
	default:
		return RPMI_ERR_HW_FAULT;
	}
}

static struct tee_parcel *parcel_find(u32 id)
{
	struct tee_parcel *p;

	tee_parcel_for_each(p)
		if (p->id == id)
			return p;

	return NULL;
}

static u32 parcel_alloc_id(void)
{
	while (parcel_find(parcel_next_id))
		parcel_next_id++;
	return parcel_next_id++;
}

/* Check overlap with blocks whose address ranges are already validated. */
static bool parcel_blocks_overlap(const struct tee_block *blocks, u32 count,
				  unsigned long address, unsigned long size)
{
	u32 i;

	for (i = 0; i < count; i++) {
		unsigned long start = blocks[i].address;
		unsigned long length = blocks[i].pages << RPMI_TEE_PAGE_SHIFT;

		if (address < start + length && start < address + size)
			return true;
	}

	return false;
}

/* Decode one request into an independent batch; publish only on success. */
static int parcel_decode_blocks(const u32 *high, const u32 *low, u32 count,
				struct tee_block_batch **result)
{
	struct tee_block_batch *batch, *other_batch;
	struct tee_parcel *other;
	int ret = SBI_EINVAL;
	u32 i;

	if (count > TEE_MAX_PAGES)
		return SBI_EINVAL;

	batch = sbi_zalloc(sizeof(*batch) + count * sizeof(batch->blocks[0]));
	if (!batch)
		return SBI_ENOMEM;

	for (i = 0; i < count; i++) {
		u32 lo = le32_to_cpu(low[i]);
		u32 hi = le32_to_cpu(high[i]);
		u32 pages = rpmi_tee_block_pages(lo);
		/* Current block's address:size. */
		unsigned long address = rpmi_tee_block_address(hi, lo);
		unsigned long size = pages << RPMI_TEE_PAGE_SHIFT;

		/* Reject address overflow or exceeding the page limit. */
		if (address > ~0UL - size ||
		    pages > TEE_MAX_PAGES - batch->page_count)
			goto out_free;

		/* Reject overlap with earlier blocks in this request. */
		ret = SBI_EDENIED;
		if (parcel_blocks_overlap(batch->blocks, i, address, size))
			goto out_free;

		/* Registered batches reserve their ranges until reclamation. */
		tee_parcel_for_each(other) {
			tee_parcel_for_each_batch(other_batch, other) {
				if (parcel_blocks_overlap(other_batch->blocks,
							  other_batch->count,
							  address, size))
					goto out_free;
			}
		}

		batch->blocks[i].address = address;
		batch->blocks[i].pages = pages;
		batch->page_count += pages;
		ret = SBI_EINVAL;
	}

	batch->count = count;
	*result = batch;

	return 0;

out_free:
	sbi_free(batch);

	return ret;
}

/* Free every block batch owned by a parcel. */
static void parcel_free_blocks(struct tee_parcel *p)
{
	struct tee_block_batch *batch, *next;

	sbi_list_for_each_entry_safe(batch, next, &p->batches, node) {
		sbi_list_del(&batch->node);
		sbi_free(batch);
	}
}

/* Encode selected parcel blocks into BLOCK_HIGH and BLOCK_LOW arrays. */
static void parcel_put_blocks(u32 *data, const struct tee_parcel *p,
			      u32 first, u32 count)
{
	struct tee_block_batch *batch;
	u32 i, written = 0;

	tee_parcel_for_each_batch(batch, p) {
		/* Skip batched before the first requested block. */
		if (first >= batch->count) {
			first -= batch->count;
			continue;
		}

		for (i = first; i < batch->count && written < count; i++) {
			const struct tee_block *block = &batch->blocks[i];

			u32 high = rpmi_tee_block_high(block->address);
			u32 low = rpmi_tee_block_low(block->address,
						     block->pages);

			data[written] = cpu_to_le32(high);
			data[count + written] = cpu_to_le32(low);
			written++;
		}

		if (written == count)
			return;

		first = 0;
	}
}

/* Parcel lifecycle handlers. */

/* Create a SHARE parcel, reserving its blocks until reclamation. */
static int parcel_create(struct rpmi_tee_endpoint *caller, const void *data,
			 u32 len, void *response, u32 maxlen,
			 unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_parcel_create_req *req = data;
	struct rpmi_tee_parcel_create_resp *resp = response;
	struct rpmi_tee_endpoint *receiver;
	struct tee_parcel *p;
	struct tee_block_batch *batch;
	u32 creator_access, access, count;
	s32 status = RPMI_ERR_INVALID_PARAM;
	int ret;

	/* Require the prefix, receiver ID/access pair and response space. */
	if (len < sizeof(*req) + 2 * sizeof(u32) || maxlen < sizeof(*resp))
		goto out_status;

	if (le32_to_cpu(req->creator_id) != caller->id)
		goto out_status;

	if (le32_to_cpu(req->receiver_count) != 1)
		goto out_status;

	count = le32_to_cpu(req->block_count);
	if (!count ||
	    /* check block pairs after the receiver ID/access pair. */
	    count != (len - sizeof(*req) - 2 * sizeof(u32)) /
		     (2 * sizeof(u32)) ||
	    (len - sizeof(*req) - 2 * sizeof(u32)) % (2 * sizeof(u32)))
		goto out_status;

	receiver = rpmi_tee_endpoint_by_id(le32_to_cpu(req->data[0]));
	creator_access = le32_to_cpu(req->creator_access);
	access = le32_to_cpu(req->data[1]);
	if (!receiver ||
	    !rpmi_tee_endpoint_available(receiver) ||
	    !creator_access || !access ||
	    ((creator_access | access) & ~RPMI_TEE_ACCESS_MASK))
		goto out_status;

	if (parcel_count >= TEE_MAX_PARCELS) {
		status = RPMI_ERR_BUSY;
		goto out_status;
	}

	p = sbi_zalloc(sizeof(*p));
	if (!p) {
		status = RPMI_ERR_FAILED;
		goto out_status;
	}

	SBI_INIT_LIST_HEAD(&p->batches);
	p->creator = caller;
	p->receiver = receiver;
	p->creator_access = creator_access;
	p->access = access;
	p->nonce = le32_to_cpu(req->nonce);
	p->state = TEE_PARCEL_CREATED;
	/* BLOCK_HIGH/LOW arrays follow the receiver ID/access pair. */
	ret = parcel_decode_blocks(req->data + 2, req->data + 2 + count,
				   count, &batch);
	if (ret) {
		status = parcel_error(ret);
		goto out_free;
	}

	sbi_list_add_tail(&batch->node, &p->batches);
	p->block_count = batch->count;
	p->page_count = batch->page_count;

	p->id = parcel_alloc_id();
	p->busy = true;

	sbi_list_add_tail(&p->node, &tee_parcels);
	parcel_count++;

	ret = parcel_protect_run(p, parcel_protect_check);
	if (!ret && p->state == TEE_PARCEL_CREATED)
		/* For !segmented run parcel_protect_create. */
		ret = parcel_protect_run(p, parcel_protect_create);
	if (ret) {
		sbi_list_del(&p->node);
		parcel_count--;
		status = parcel_error(ret);
		goto out_free;
	}

	p->busy = false;

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->parcel_id = cpu_to_le32(p->id);
	*response_len = sizeof(*resp);

	return 0;

out_free:
	parcel_free_blocks(p);
	sbi_free(p);

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
}

static int parcel_accept(struct rpmi_tee_endpoint *caller, const void *data,
			 u32 len, void *response, u32 maxlen,
			 unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_parcel_accept_req *req = data;
	struct rpmi_tee_parcel_accept_resp *resp = response;
	struct tee_parcel *p;
	u32 count;
	s32 status = RPMI_ERR_INVALID_PARAM;
	int ret;

	if (len != sizeof(*req))
		goto out_status;

	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->receiver != caller)
		goto out_status;

	if (le32_to_cpu(req->acceptor_id) != caller->id ||
	    le32_to_cpu(req->creator_id) != p->creator->id ||
	    le32_to_cpu(req->nonce) != p->nonce)
		goto out_status;

	if (le32_to_cpu(req->creator_access) != p->creator_access ||
	    le32_to_cpu(req->access) != p->access)
		goto out_status;

	/* Require default flags, no address hint and no other endpoints. */
	if (req->flags || req->address_high || req->address_low ||
	    req->other_count)
		goto out_status;

	if (le32_to_cpu(req->max_pages) < p->page_count)
		goto out_status;

	if (p->state == TEE_PARCEL_ACCEPTED) {
		status = RPMI_ERR_ALREADY;
		goto out_status;
	}

	if (maxlen < sizeof(*resp) + rpmi_tee_block_array_size(1))
		return SBI_ENOSPC;

	count = p->block_count;
	if (maxlen < sizeof(*resp) + rpmi_tee_block_array_size(count))
		return SBI_ENOSPC;

	ret = parcel_protect_run(p, parcel_protect_accept);
	if (ret) {
		status = parcel_error(ret);
		goto out_status;
	}
	p->state = TEE_PARCEL_ACCEPTED;
	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = 0;
	resp->page_count = cpu_to_le32(p->page_count);
	resp->block_count = cpu_to_le32(count);
	parcel_put_blocks(resp->blocks, p, 0, count);
	*response_len = sizeof(*resp) + rpmi_tee_block_array_size(count);

	return 0;

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
}

static int parcel_release(struct rpmi_tee_endpoint *caller, const void *data,
			  u32 len, void *response, u32 maxlen,
			  unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_parcel_release_req *req = data;
	struct tee_parcel *p;
	s32 status = RPMI_ERR_INVALID_PARAM;
	int ret;

	if (len != sizeof(*req) + sizeof(u32))
		goto out_status;

	if (req->flags)
		goto out_status;

	if (le32_to_cpu(req->endpoint_count) != 1 ||
	    le32_to_cpu(req->endpoints[0]) != caller->id)
		goto out_status;

	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->receiver != caller)
		goto out_status;

	if (p->state != TEE_PARCEL_ACCEPTED)
		goto out_status;

	if (p->state == TEE_PARCEL_ACCEPTED) {
		ret = parcel_protect_run(p, parcel_protect_release);
		if (ret) {
			status = parcel_error(ret);
			goto out_status;
		}
	}

	p->state = TEE_PARCEL_CREATED;
	status = RPMI_SUCCESS;

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
}

static int parcel_reclaim(struct rpmi_tee_endpoint *caller, const void *data,
			  u32 len, void *response, u32 maxlen,
			  unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_parcel_reclaim_req *req = data;
	struct rpmi_tee_parcel_reclaim_resp *resp = response;
	struct tee_parcel *p;
	s32 status = RPMI_ERR_INVALID_PARAM;
	int ret;

	if (len != sizeof(*req) || maxlen < sizeof(*resp))
		goto out_status;

	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->creator != caller)
		goto out_status;

	if (p->state == TEE_PARCEL_ACCEPTED) {
		status = RPMI_ERR_DENIED;
		goto out_status;
	}

	ret = parcel_protect_run(p, parcel_protect_reclaim);
	if (ret) {
		status = parcel_error(ret);
		goto out_status;
	}
	sbi_list_del(&p->node);
	parcel_count--;
	parcel_free_blocks(p);
	sbi_free(p);

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = 0;
	*response_len = sizeof(*resp);

	return 0;

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
}

/* Dispatch and public entry points. */

static int parcel_dispatch(struct rpmi_tee_endpoint *caller, u32 service,
			   const void *data, u32 len, void *response,
			   u32 maxlen, unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;

	switch (service) {
	case RPMI_TEE_MEMORY_PARCEL_CREATE:
		return parcel_create(caller, data, len, response, maxlen,
				     response_len);
	case RPMI_TEE_MEMORY_PARCEL_ACCEPT:
		return parcel_accept(caller, data, len, response, maxlen,
				     response_len);
	case RPMI_TEE_MEMORY_PARCEL_RELEASE:
		return parcel_release(caller, data, len, response, maxlen,
				      response_len);
	case RPMI_TEE_MEMORY_PARCEL_RECLAIM:
		return parcel_reclaim(caller, data, len, response, maxlen,
				      response_len);
	default:
		break;
	}

	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(RPMI_ERR_NOTSUPP);
	*response_len = sizeof(*status_resp);

	return 0;
}

int rpmi_tee_memory_request(struct rpmi_tee_endpoint *caller, u32 service,
			    const void *data, u32 len, void *response,
			    u32 maxlen, unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	struct tee_parcel *parcel = NULL;
	u32 id = 0;
	bool lookup = false;
	int ret;
	s32 status = RPMI_ERR_INVALID_PARAM;

	switch (service) {
	case RPMI_TEE_MEMORY_PARCEL_CREATE:
		break;
	case RPMI_TEE_MEMORY_PARCEL_ACCEPT: {
		const struct rpmi_tee_parcel_accept_req *req = data;

		if (len < sizeof(*req))
			goto out_status;
		id = le32_to_cpu(req->parcel_id);
		lookup = true;
		break;
	}
	case RPMI_TEE_MEMORY_PARCEL_RELEASE: {
		const struct rpmi_tee_parcel_release_req *req = data;

		if (len < sizeof(*req))
			goto out_status;
		id = le32_to_cpu(req->parcel_id);
		lookup = true;
		break;
	}
	case RPMI_TEE_MEMORY_PARCEL_RECLAIM: {
		const struct rpmi_tee_parcel_reclaim_req *req = data;

		if (len < sizeof(*req))
			goto out_status;
		id = le32_to_cpu(req->parcel_id);
		lookup = true;
		break;
	}
	default:
		status = RPMI_ERR_NOTSUPP;
		goto out_status;
	}

	spin_lock(&parcel_lock);
	if (!rpmi_tee_endpoint_available(caller)) {
		status = RPMI_ERR_DENIED;
		goto unlock_status;
	}

	if (lookup) {
		parcel = parcel_find(id);
		if (parcel && parcel->busy) {
			status = RPMI_ERR_BUSY;
			goto unlock_status;
		}
		if (parcel && parcel->disconnected &&
		    service != RPMI_TEE_MEMORY_PARCEL_RECLAIM) {
			status = RPMI_ERR_DENIED;
			goto unlock_status;
		}
		if (parcel)
			parcel->busy = true;
	}

	ret = parcel_dispatch(caller, service, data, len, response, maxlen,
			      response_len);

	/* A successful reclaim may have removed and freed the reservation. */
	if (lookup) {
		parcel = parcel_find(id);
		if (parcel) {
			parcel->busy = false;
			if (parcel->disconnected)
				parcel->state = TEE_PARCEL_CREATED;
		}
	}

	if (!ret && !rpmi_tee_endpoint_available(caller)) {
		status = RPMI_ERR_DENIED;
		goto unlock_status;
	}
	spin_unlock(&parcel_lock);

	return ret;

unlock_status:
	spin_unlock(&parcel_lock);

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
}

u32 rpmi_tee_memory_feature(u32 feature)
{
	if (feature == RPMI_TEE_FEATURE_MEMORY_SHARE)
		return RPMI_TEE_MEMORY_FULLY_SUPPORTED;

	return 0;
}

void rpmi_tee_memory_disconnect(struct rpmi_tee_endpoint *ep)
{
	struct tee_parcel *parcel;
	bool update = false;

	spin_lock(&parcel_lock);
	tee_parcel_for_each(parcel) {
		if (parcel->receiver != ep)
			continue;

		parcel->disconnected = true;
		update = true;
	}
	spin_unlock(&parcel_lock);

	/* Endpoint retirement must fence commits as well as existing grants. */
	if (update && parcel_protect_disconnect(ep))
		sbi_hart_hang();

	spin_lock(&parcel_lock);
	tee_parcel_for_each(parcel) {
		if (parcel->receiver != ep || parcel->busy)
			continue;

		parcel->state = TEE_PARCEL_CREATED;
	}
	spin_unlock(&parcel_lock);
}
