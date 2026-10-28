/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

#define TEE_MAX_PARCELS		64
#define TEE_MAX_PAGES		4096
#define TEE_MAX_SEGMENTED	4

struct tee_block {
	unsigned long address;
	u32 pages;
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
	struct tee_block *blocks;
	bool accepted;
	bool creating;
	bool accepting;
	u32 receive_next;
	bool busy;
	bool disconnected;
};

/* Protects the registry, reservations and parcel lifecycle fields. */
static spinlock_t parcel_lock = SPIN_LOCK_INITIALIZER;
static SBI_LIST_HEAD(tee_parcels);
static u32 parcel_next_id = 1;

/* Memory-protection backend. */

#ifdef CONFIG_FDT_MPXY_RPMI_TEE_TEST_SHARED_RAM
/*
 * Trusted domain policy must limit shared access to suitable normal RAM.
 * This test backend checks permissions but never changes or revokes access.
 */
static int parcel_protect_check(struct tee_parcel *parcel)
{
	struct sbi_domain *creator, *receiver;
	unsigned long access = SBI_DOMAIN_READ | SBI_DOMAIN_WRITE;
	u32 i;

	creator = rpmi_tee_endpoint_domain(parcel->creator);
	receiver = rpmi_tee_endpoint_domain(parcel->receiver);
	if (!creator || !receiver)
		return SBI_ENODEV;
	if ((parcel->creator_access | parcel->access) & RPMI_TEE_ACCESS_EXEC)
		return SBI_ENOTSUPP;
	/* Check both domains against the trusted static access policy. */
	for (i = 0; i < parcel->block_count; i++) {
		const struct tee_block *block = &parcel->blocks[i];
		unsigned long size = (unsigned long)block->pages <<
				     RPMI_TEE_PAGE_SHIFT;

		if (!sbi_domain_check_addr_range(creator, block->address, size,
						 PRV_S, access) ||
		    !sbi_domain_check_addr_range(receiver, block->address, size,
						 PRV_S, access))
			return SBI_EDENIED;
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

/* The busy reservation pins the descriptor while enforcement runs unlocked. */
static int parcel_protect_run(struct tee_parcel *parcel,
			      int (*operation)(struct tee_parcel *parcel))
{
	int ret;

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

	sbi_list_for_each_entry(p, &tee_parcels, node)
		if (p->id == id)
			return p;
	return NULL;
}

static u32 pending_segments(struct rpmi_tee_endpoint *ep)
{
	struct tee_parcel *p;
	u32 count = 0;

	sbi_list_for_each_entry(p, &tee_parcels, node) {
		if (p->creator == ep && p->creating)
			count++;
		if (p->receiver == ep && p->accepting)
			count++;
	}
	return count;
}

static int parcel_decode(struct tee_parcel *p, const u32 *high,
			 const u32 *low, u32 count)
{
	u32 i, j;

	if (!count || count > TEE_MAX_PAGES)
		return SBI_EINVAL;
	p->blocks = sbi_zalloc(count * sizeof(*p->blocks));
	if (!p->blocks)
		return SBI_ENOMEM;
	for (i = 0; i < count; i++) {
		u32 lo = le32_to_cpu(low[i]);
		u64 address = ((u64)le32_to_cpu(high[i]) << 32) | (lo & ~4095U);
		unsigned long size;
		struct tee_parcel *other;

		p->blocks[i].pages = (lo & 4095) + 1;
		size = (unsigned long)p->blocks[i].pages << RPMI_TEE_PAGE_SHIFT;
		if ((unsigned long)address != address ||
		    address > ~0UL - size ||
		    p->blocks[i].pages > TEE_MAX_PAGES - p->page_count)
			return SBI_EINVAL;
		p->blocks[i].address = address;
		p->page_count += p->blocks[i].pages;
		for (j = 0; j < i; j++) {
			unsigned long a = p->blocks[j].address;
			unsigned long n = (unsigned long)p->blocks[j].pages <<
					  RPMI_TEE_PAGE_SHIFT;

			if (address < a + n && a < address + size)
				return SBI_EDENIED;
		}
		sbi_list_for_each_entry(other, &tee_parcels, node) {
			for (j = 0; j < other->block_count; j++) {
				unsigned long a = other->blocks[j].address;
				unsigned long n =
					(unsigned long)other->blocks[j].pages <<
					RPMI_TEE_PAGE_SHIFT;

				if (address < a + n && a < address + size)
					return SBI_EDENIED;
			}
		}
	}
	p->block_count = count;
	return 0;
}

static void parcel_put_blocks(u32 *data, const struct tee_parcel *p,
			      u32 first, u32 count)
{
	u32 i;

	for (i = 0; i < count; i++) {
		const struct tee_block *block = &p->blocks[first + i];

		data[i] = cpu_to_le32((u64)block->address >> 32);
		data[count + i] = cpu_to_le32((u32)block->address |
					      (block->pages - 1));
	}
}

/* Parcel lifecycle handlers. */

static int parcel_create(struct rpmi_tee_endpoint *caller, const void *data,
			 u32 len, void *response, u32 maxlen,
			 unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_parcel_create_req *req = data;
	struct rpmi_tee_parcel_create_resp *resp = response;
	struct tee_parcel *p, *other;
	u32 count, live = 0;
	int ret;
	s32 status;

	if (caller->tee || len < sizeof(*req) + 2 * sizeof(u32) ||
	    maxlen < sizeof(*resp))
		goto invalid;
	count = le32_to_cpu(req->block_count);
	if (le32_to_cpu(req->creator_id) != caller->id ||
	    (le32_to_cpu(req->flags) & ~RPMI_TEE_MULTI_SEGMENT) ||
	    le32_to_cpu(req->receiver_count) != 1 || !count ||
	    count != (len - sizeof(*req) - 8) / 8 ||
	    (len - sizeof(*req) - 8) % 8)
		goto invalid;
	sbi_list_for_each_entry(other, &tee_parcels, node)
		live++;
	if (live == TEE_MAX_PARCELS) {
		status = RPMI_ERR_BUSY;
		goto out_status;
	}
	p = sbi_zalloc(sizeof(*p));
	if (!p) {
		status = RPMI_ERR_FAILED;
		goto out_status;
	}
	p->creator = caller;
	p->receiver = rpmi_tee_endpoint_by_id(le32_to_cpu(req->data[0]));
	p->creator_access = le32_to_cpu(req->creator_access);
	p->access = le32_to_cpu(req->data[1]);
	p->nonce = le32_to_cpu(req->nonce);
	p->creating = !!(le32_to_cpu(req->flags) & RPMI_TEE_MULTI_SEGMENT);
	if (p->creating && pending_segments(caller) >= TEE_MAX_SEGMENTED) {
		ret = SBI_ENOSPC;
		goto failed;
	}
	if (!p->receiver || !p->receiver->tee ||
	    !rpmi_tee_endpoint_available(p->receiver) ||
	    !p->creator_access || !p->access ||
	    ((p->creator_access | p->access) & ~RPMI_TEE_ACCESS_MASK)) {
		ret = SBI_EINVAL;
		goto failed;
	}
	ret = parcel_decode(p, req->data + 2, req->data + 2 + count, count);
	if (ret)
		goto failed;
	while (parcel_find(parcel_next_id))
		parcel_next_id++;
	p->id = parcel_next_id++;
	p->busy = true;
	sbi_list_add_tail(&p->node, &tee_parcels);
	ret = parcel_protect_run(p, parcel_protect_check);
	if (!ret && !p->creating)
		ret = parcel_protect_run(p, parcel_protect_create);
	if (ret) {
		sbi_list_del(&p->node);
		goto failed;
	}
	p->busy = false;
	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->parcel_id = cpu_to_le32(p->id);
	*response_len = sizeof(*resp);
	return 0;
failed:
	sbi_free(p->blocks);
	sbi_free(p);
	status = parcel_error(ret);
	goto out_status;
invalid:
	status = RPMI_ERR_INVALID_PARAM;
out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;
	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);
	return 0;
}

static int parcel_segment_send(struct rpmi_tee_endpoint *caller,
			       const void *data, u32 len, void *response,
			       u32 maxlen, unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_segment_send_req *req = data;
	struct tee_parcel *p;
	struct tee_parcel extra = { 0 };
	struct tee_block *joined, *old;
	u32 count, flags, old_count, old_pages;
	int ret;
	s32 status;

	if (len < sizeof(*req))
		goto invalid;
	p = parcel_find(le32_to_cpu(req->parcel_id));
	count = le32_to_cpu(req->block_count);
	flags = le32_to_cpu(req->flags);
	if (!p || p->creator != caller || !p->creating || !count ||
	    flags & ~RPMI_TEE_LAST_SEGMENT ||
	    count != (len - sizeof(*req)) / 8 || (len - sizeof(*req)) % 8 ||
	    count > TEE_MAX_PAGES - p->block_count)
		goto invalid;
	extra.creator = p->creator;
	extra.receiver = p->receiver;
	extra.creator_access = p->creator_access;
	extra.access = p->access;
	ret = parcel_decode(&extra, req->blocks, req->blocks + count, count);
	if (ret || extra.page_count > TEE_MAX_PAGES - p->page_count) {
		sbi_free(extra.blocks);
		if (!ret)
			ret = SBI_EINVAL;
		status = parcel_error(ret);
		goto out_status;
	}
	joined = sbi_malloc((p->block_count + count) * sizeof(*joined));
	if (!joined) {
		sbi_free(extra.blocks);
		status = RPMI_ERR_FAILED;
		goto out_status;
	}
	sbi_memcpy(joined, p->blocks, p->block_count * sizeof(*joined));
	sbi_memcpy(joined + p->block_count, extra.blocks,
		   count * sizeof(*joined));
	sbi_free(extra.blocks);
	old = p->blocks;
	old_count = p->block_count;
	old_pages = p->page_count;
	p->blocks = joined;
	p->block_count += count;
	p->page_count += extra.page_count;
	ret = parcel_protect_run(p, parcel_protect_check);
	if (!ret && (flags & RPMI_TEE_LAST_SEGMENT))
		ret = parcel_protect_run(p, parcel_protect_create);
	if (ret) {
		p->blocks = old;
		p->block_count = old_count;
		p->page_count = old_pages;
		p->creating = true;
		sbi_free(joined);
		status = parcel_error(ret);
		goto out_status;
	}
	p->creating = !(flags & RPMI_TEE_LAST_SEGMENT);
	sbi_free(old);
	status = RPMI_SUCCESS;
	goto out_status;
invalid:
	status = RPMI_ERR_INVALID_PARAM;
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
	int ret;
	s32 status;

	if (len != sizeof(*req) || !caller->tee)
		goto invalid;
	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->creating || p->receiver != caller ||
	    le32_to_cpu(req->acceptor_id) != caller->id ||
	    le32_to_cpu(req->creator_id) != p->creator->id ||
	    le32_to_cpu(req->nonce) != p->nonce ||
	    le32_to_cpu(req->creator_access) != p->creator_access ||
	    le32_to_cpu(req->access) != p->access || req->flags ||
	    req->address_high || req->address_low || req->other_count ||
	    le32_to_cpu(req->max_pages) < p->page_count)
		goto invalid;
	if (p->accepted || p->accepting) {
		status = RPMI_ERR_ALREADY;
		goto out_status;
	}
	if (maxlen < sizeof(*resp) + 8)
		return SBI_ENOSPC;
	count = MIN(p->block_count, (maxlen - sizeof(*resp)) / 8);
	if (count < p->block_count) {
		if (pending_segments(caller) >= TEE_MAX_SEGMENTED) {
			status = RPMI_ERR_BUSY;
			goto out_status;
		}
		p->accepting = true;
		p->receive_next = count;
	} else {
		ret = parcel_protect_run(p, parcel_protect_accept);
		if (ret) {
			status = parcel_error(ret);
			goto out_status;
		}
		p->accepted = true;
	}
	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = cpu_to_le32(p->accepting ? RPMI_TEE_MULTI_SEGMENT : 0);
	resp->page_count = cpu_to_le32(p->page_count);
	resp->block_count = cpu_to_le32(count);
	parcel_put_blocks(resp->blocks, p, 0, count);
	*response_len = sizeof(*resp) + count * 8;
	return 0;
invalid:
	status = RPMI_ERR_INVALID_PARAM;
out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;
	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);
	return 0;
}

static int parcel_segment_receive(struct rpmi_tee_endpoint *caller,
				  const void *data, u32 len, void *response,
				  u32 maxlen, unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_segment_receive_req *req = data;
	struct rpmi_tee_segment_receive_resp *resp = response;
	struct tee_parcel *p;
	u32 count;
	bool last;
	int ret;
	s32 status;

	if (len != sizeof(*req) || le32_to_cpu(req->acceptor_id) != caller->id)
		goto invalid;
	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->receiver != caller || !p->accepting)
		goto invalid;
	if (maxlen < sizeof(*resp) + 8)
		return SBI_ENOSPC;
	count = MIN(p->block_count - p->receive_next,
		    (maxlen - sizeof(*resp)) / 8);
	last = p->receive_next + count == p->block_count;
	if (last) {
		ret = parcel_protect_run(p, parcel_protect_accept);
		if (ret) {
			status = parcel_error(ret);
			goto out_status;
		}
		p->accepting = false;
		p->accepted = true;
	}
	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = cpu_to_le32(last ? RPMI_TEE_LAST_SEGMENT : 0);
	resp->block_count = cpu_to_le32(count);
	parcel_put_blocks(resp->blocks, p, p->receive_next, count);
	p->receive_next += count;
	*response_len = sizeof(*resp) + count * 8;
	return 0;
invalid:
	status = RPMI_ERR_INVALID_PARAM;
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
	int ret;
	s32 status;

	if (len != sizeof(*req) + sizeof(u32) || req->flags ||
	    le32_to_cpu(req->endpoint_count) != 1 ||
	    le32_to_cpu(req->endpoints[0]) != caller->id)
		goto invalid;
	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->receiver != caller || (!p->accepted && !p->accepting))
		goto invalid;
	if (p->accepted) {
		ret = parcel_protect_run(p, parcel_protect_release);
		if (ret) {
			status = parcel_error(ret);
			goto out_status;
		}
	}
	p->accepted = false;
	p->accepting = false;
	status = RPMI_SUCCESS;
	goto out_status;
invalid:
	status = RPMI_ERR_INVALID_PARAM;
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
	int ret;
	s32 status;

	if (len != sizeof(*req) || maxlen < sizeof(*resp))
		goto invalid;
	p = parcel_find(le32_to_cpu(req->parcel_id));
	if (!p || p->creator != caller)
		goto invalid;
	if (p->accepted || p->accepting) {
		status = RPMI_ERR_DENIED;
		goto out_status;
	}
	if (!p->creating) {
		ret = parcel_protect_run(p, parcel_protect_reclaim);
		if (ret) {
			status = parcel_error(ret);
			goto out_status;
		}
	}
	sbi_list_del(&p->node);
	sbi_free(p->blocks);
	sbi_free(p);
	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = 0;
	*response_len = sizeof(*resp);
	return 0;
invalid:
	status = RPMI_ERR_INVALID_PARAM;
out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;
	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);
	return 0;
}

/* Dispatch and public entry points. */

static int parcel_request(struct rpmi_tee_endpoint *caller, u32 service,
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
	case RPMI_TEE_MEMORY_SEGMENT_SEND:
		return parcel_segment_send(caller, data, len, response, maxlen,
					   response_len);
	case RPMI_TEE_MEMORY_SEGMENT_RECEIVE:
		return parcel_segment_receive(caller, data, len, response,
					      maxlen, response_len);
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
	const u32 *words = data;
	struct rpmi_tee_status_resp *status_resp = response;
	struct tee_parcel *parcel = NULL;
	u32 id = 0, offset = 0;
	bool lookup = false;
	int ret;
	s32 status;

	switch (service) {
	case RPMI_TEE_MEMORY_PARCEL_CREATE:
		break;
	case RPMI_TEE_MEMORY_PARCEL_ACCEPT:
		offset = 2;
		lookup = true;
		break;
	case RPMI_TEE_MEMORY_SEGMENT_RECEIVE:
		offset = 1;
		lookup = true;
		break;
	case RPMI_TEE_MEMORY_PARCEL_RELEASE:
	case RPMI_TEE_MEMORY_PARCEL_RECLAIM:
	case RPMI_TEE_MEMORY_SEGMENT_SEND:
		lookup = true;
		break;
	default:
		status = RPMI_ERR_NOTSUPP;
		goto out_status;
	}
	if (lookup && len < (offset + 1) * sizeof(u32)) {
		status = RPMI_ERR_INVALID_PARAM;
		goto out_status;
	}
	spin_lock(&parcel_lock);
	if (!rpmi_tee_endpoint_available(caller)) {
		status = RPMI_ERR_DENIED;
		goto unlock_status;
	}
	if (lookup) {
		id = le32_to_cpu(words[offset]);
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
	ret = parcel_request(caller, service, data, len, response, maxlen,
			     response_len);
	/* A successful reclaim may have removed and freed the reservation. */
	if (lookup) {
		parcel = parcel_find(id);
		if (parcel) {
			parcel->busy = false;
			if (parcel->disconnected) {
				parcel->accepted = false;
				parcel->accepting = false;
			}
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
	if (feature == RPMI_TEE_FEATURE_MULTISEGMENT_OPS)
		return TEE_MAX_SEGMENTED;
	if (feature == RPMI_TEE_FEATURE_MEMORY_SHARE)
		return RPMI_TEE_MEMORY_FULLY_SUPPORTED;
	return 0;
}

void rpmi_tee_memory_disconnect(struct rpmi_tee_endpoint *ep)
{
	struct tee_parcel *parcel;
	bool update = false;

	spin_lock(&parcel_lock);
	sbi_list_for_each_entry(parcel, &tee_parcels, node) {
		if (parcel->receiver != ep)
			continue;
		parcel->disconnected = true;
		if (parcel->busy || !parcel->creating)
			update = true;
	}
	spin_unlock(&parcel_lock);
	/* Endpoint retirement must fence commits as well as existing grants. */
	if (update && parcel_protect_disconnect(ep))
		sbi_hart_hang();
	spin_lock(&parcel_lock);
	sbi_list_for_each_entry(parcel, &tee_parcels, node) {
		if (parcel->receiver != ep || parcel->busy)
			continue;
		parcel->accepted = false;
		parcel->accepting = false;
	}
	spin_unlock(&parcel_lock);
}
