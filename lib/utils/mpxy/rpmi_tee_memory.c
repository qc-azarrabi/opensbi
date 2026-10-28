/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <libfdt.h>
#include <sbi/riscv_asm.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

#define TEE_MAX_PARCELS	64
#define TEE_MAX_PAGES	4096

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
#define TEE_MAX_RAM_RANGES 16
static struct {
	u64 base;
	u64 size;
} ram_ranges[TEE_MAX_RAM_RANGES];
static u32 ram_range_count;
static bool ram_initialized;

void rpmi_tee_memory_set_fdt(const void *fdt)
{
	int node = -1, i;

	if (ram_initialized)
		return;
	ram_initialized = true;
	/* Copy policy before either endpoint can modify its boot DT memory. */
	while ((node = fdt_node_offset_by_prop_value(fdt, node,
						     "device_type",
						     "memory", 7)) >= 0) {
		for (i = 0; ; i++) {
			u64 base, size;

			if (fdt_get_node_addr_size(fdt, node, i, &base, &size))
				break;
			if (!size || base > ~0ULL - size ||
			    ram_range_count == TEE_MAX_RAM_RANGES) {
				ram_range_count = 0;
				return;
			}
			ram_ranges[ram_range_count].base = base;
			ram_ranges[ram_range_count++].size = size;
		}
	}
}

/* Query backend readiness without publishing unsupported memory features. */
static bool parcel_protection_available(void)
{
	/*
	 * TODO: Require initialized domain protection and transactional updates
	 * with revocation synchronized across every affected hart. Reuse the
	 * Smsdid/Smmpt infrastructure rather than rebuilding PMP region arrays.
	 */
	return false;
}

static int memory_init(void)
{
	if (!ram_range_count || !parcel_protection_available())
		return SBI_ENOTSUPP;
	return 0;
}

/* Validate effective permissions without changing either domain's access. */
static int parcel_protect_check(struct tee_parcel *parcel)
{
	/*
	 * TODO: Query effective ownership and permissions for every block.
	 * Check creator authority, receiver isolation and firmware/private RAM
	 * exclusions using the protection backend's policy, not PMP encoding.
	 */
	return SBI_ENOTSUPP;
}

static int parcel_protect_create(struct tee_parcel *parcel)
{
	/*
	 * TODO: Save original permissions and apply retained creator access.
	 * Reserve resources before changing access; on failure roll back every
	 * block. Success requires completed synchronization on affected harts.
	 */
	return SBI_ENOTSUPP;
}

static int parcel_protect_accept(struct tee_parcel *parcel)
{
	/*
	 * TODO: Grant receiver access transactionally through the existing MPT
	 * infrastructure. Complete remote invalidation before success, and
	 * prevent an endpoint disconnect from racing a new grant.
	 */
	return SBI_ENOTSUPP;
}

static int parcel_protect_release(struct tee_parcel *parcel)
{
	/*
	 * TODO: Revoke receiver access on all affected harts before returning.
	 * Preserve receiver interest on failure; stale cached permissions must
	 * not permit access after successful release or later reclamation.
	 */
	return SBI_ENOTSUPP;
}

static int parcel_protect_reclaim(struct tee_parcel *parcel)
{
	/*
	 * TODO: Restore the saved creator permissions and retire backend state
	 * only after all updates complete. A failed reclaim must keep the
	 * parcel and its original protection state available for recovery.
	 */
	return SBI_ENOTSUPP;
}

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

u32 rpmi_tee_memory_feature(u32 feature)
{
	if (memory_init())
		return 0;
	if (feature == RPMI_TEE_FEATURE_MEMORY_SHARE)
		return RPMI_TEE_MEMORY_FULLY_SUPPORTED;
	return 0;
}

static bool memory_is_ram(unsigned long address, unsigned long size)
{
	u32 i;

	if (!size || address > ~0UL - size)
		return false;
	for (i = 0; i < ram_range_count; i++) {
		u64 base = ram_ranges[i].base;
		u64 len = ram_ranges[i].size;

		if (address >= base && size <= len &&
		    address - base <= len - size)
			return true;
	}
	return false;
}

static struct tee_parcel *parcel_find(u32 id)
{
	struct tee_parcel *p;

	sbi_list_for_each_entry(p, &tee_parcels, node)
		if (p->id == id)
			return p;
	return NULL;
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
		    !memory_is_ram(address, size) ||
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

static int parcel_create(struct rpmi_tee_endpoint *caller, const void *data,
			 u32 len, void *response, u32 maxlen,
			 unsigned long *response_len)
{
	const struct rpmi_tee_parcel_create_req *req = data;
	struct tee_parcel *p, *other;
	u32 count, live = 0;
	int ret;

	if (caller->tee || len < sizeof(*req) + 2 * sizeof(u32) || maxlen < 8)
		goto invalid;
	count = le32_to_cpu(req->block_count);
	if (le32_to_cpu(req->creator_id) != caller->id ||
	    req->flags ||
	    le32_to_cpu(req->receiver_count) != 1 || !count ||
	    count != (len - sizeof(*req) - 8) / 8 ||
	    (len - sizeof(*req) - 8) % 8)
		goto invalid;
	sbi_list_for_each_entry(other, &tee_parcels, node)
		live++;
	if (live == TEE_MAX_PARCELS)
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_BUSY);
	p = sbi_zalloc(sizeof(*p));
	if (!p)
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_FAILED);
	p->creator = caller;
	p->receiver = rpmi_tee_endpoint_by_id(le32_to_cpu(req->data[0]));
	p->creator_access = le32_to_cpu(req->creator_access);
	p->access = le32_to_cpu(req->data[1]);
	p->nonce = le32_to_cpu(req->nonce);
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
	((u32 *)response)[0] = cpu_to_le32(RPMI_SUCCESS);
	((u32 *)response)[1] = cpu_to_le32(p->id);
	*response_len = 8;
	return 0;
failed:
	sbi_free(p->blocks);
	sbi_free(p);
	return rpmi_tee_status(response, maxlen, response_len,
			       parcel_error(ret));
invalid:
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_INVALID_PARAM);
}

static int parcel_accept(struct rpmi_tee_endpoint *caller, const void *data,
			 u32 len, void *response, u32 maxlen,
			 unsigned long *response_len)
{
	const struct rpmi_tee_parcel_accept_req *req = data;
	struct rpmi_tee_parcel_accept_resp *resp = response;
	struct tee_parcel *p;
	u32 count;
	int ret;

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
	if (p->accepted || p->accepting)
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_ALREADY);
	if (maxlen < sizeof(*resp) + 8)
		return SBI_ENOSPC;
	count = p->block_count;
	if (count > (maxlen - sizeof(*resp)) / 8)
		return SBI_ENOSPC;
	ret = parcel_protect_run(p, parcel_protect_accept);
	if (ret)
		return rpmi_tee_status(response, maxlen, response_len,
				       parcel_error(ret));
	p->accepted = true;
	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->flags = 0;
	resp->page_count = cpu_to_le32(p->page_count);
	resp->block_count = cpu_to_le32(count);
	parcel_put_blocks(resp->blocks, p, 0, count);
	*response_len = sizeof(*resp) + count * 8;
	return 0;
invalid:
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_INVALID_PARAM);
}

static int parcel_release(struct rpmi_tee_endpoint *caller, const void *data,
			  u32 len, void *response, u32 maxlen,
			  unsigned long *response_len)
{
	const u32 *words = data;
	struct tee_parcel *p;
	int ret;

	if (len != 16 || words[1] || le32_to_cpu(words[2]) != 1 ||
	    le32_to_cpu(words[3]) != caller->id)
		goto invalid;
	p = parcel_find(le32_to_cpu(words[0]));
	if (!p || p->receiver != caller || (!p->accepted && !p->accepting))
		goto invalid;
	if (p->accepted) {
		ret = parcel_protect_run(p, parcel_protect_release);
		if (ret)
			return rpmi_tee_status(response, maxlen, response_len,
					       parcel_error(ret));
	}
	p->accepted = false;
	p->accepting = false;
	return rpmi_tee_status(response, maxlen, response_len, RPMI_SUCCESS);
invalid:
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_INVALID_PARAM);
}

static int parcel_reclaim(struct rpmi_tee_endpoint *caller, const void *data,
			  u32 len, void *response, u32 maxlen,
			  unsigned long *response_len)
{
	struct tee_parcel *p;
	int ret;

	if (len != 4 || maxlen < 8)
		goto invalid;
	p = parcel_find(le32_to_cpu(*(const u32 *)data));
	if (!p || p->creator != caller)
		goto invalid;
	if (p->accepted || p->accepting)
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_DENIED);
	if (!p->creating) {
		ret = parcel_protect_run(p, parcel_protect_reclaim);
		if (ret)
			return rpmi_tee_status(response, maxlen, response_len,
					       parcel_error(ret));
	}
	sbi_list_del(&p->node);
	sbi_free(p->blocks);
	sbi_free(p);
	((u32 *)response)[0] = cpu_to_le32(RPMI_SUCCESS);
	((u32 *)response)[1] = 0;
	*response_len = 8;
	return 0;
invalid:
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_INVALID_PARAM);
}

static int parcel_request(struct rpmi_tee_endpoint *caller, u32 service,
			  const void *data, u32 len, void *response,
			  u32 maxlen, unsigned long *response_len)
{
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
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_NOTSUPP);
	}
}

int rpmi_tee_memory_request(struct rpmi_tee_endpoint *caller, u32 service,
			    const void *data, u32 len, void *response,
			    u32 maxlen, unsigned long *response_len)
{
	const u32 *words = data;
	struct tee_parcel *parcel = NULL;
	u32 id = 0, offset = 0;
	bool lookup = false;
	int ret;

	if (memory_init())
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_NOTSUPP);
	switch (service) {
	case RPMI_TEE_MEMORY_PARCEL_CREATE:
		break;
	case RPMI_TEE_MEMORY_PARCEL_ACCEPT:
		offset = 2;
		lookup = true;
		break;
	case RPMI_TEE_MEMORY_PARCEL_RELEASE:
	case RPMI_TEE_MEMORY_PARCEL_RECLAIM:
		lookup = true;
		break;
	default:
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_NOTSUPP);
	}
	if (lookup && len < (offset + 1) * sizeof(u32))
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_INVALID_PARAM);
	spin_lock(&parcel_lock);
	if (!rpmi_tee_endpoint_available(caller)) {
		ret = rpmi_tee_status(response, maxlen, response_len,
				      RPMI_ERR_DENIED);
		goto out;
	}
	if (lookup) {
		id = le32_to_cpu(words[offset]);
		parcel = parcel_find(id);
		if (parcel && parcel->busy) {
			ret = rpmi_tee_status(response, maxlen, response_len,
					      RPMI_ERR_BUSY);
			goto out;
		}
		if (parcel && parcel->disconnected &&
		    service != RPMI_TEE_MEMORY_PARCEL_RECLAIM) {
			ret = rpmi_tee_status(response, maxlen, response_len,
					      RPMI_ERR_DENIED);
			goto out;
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
	if (!ret && !rpmi_tee_endpoint_available(caller))
		ret = rpmi_tee_status(response, maxlen, response_len,
				      RPMI_ERR_DENIED);
out:
	spin_unlock(&parcel_lock);
	return ret;
}
