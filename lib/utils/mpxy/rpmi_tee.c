/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <libfdt.h>
#include <sbi/riscv_asm.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_hart_pmp.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/mpxy/rpmi_tee.h>
#include <sbi_utils/mpxy/fdt_mpxy_bridge.h>

static SBI_LIST_HEAD(tee_endpoints);

struct sbi_domain *rpmi_tee_endpoint_domain(struct rpmi_tee_endpoint *ep)
{
	return sbi_domain_find_by_name(ep->domain_name);
}

struct rpmi_tee_endpoint *rpmi_tee_endpoint_by_id(u32 id)
{
	struct rpmi_tee_endpoint *ep;

	sbi_list_for_each_entry(ep, &tee_endpoints, node)
		if (ep->id == id)
			return ep;
	return NULL;
}

struct rpmi_tee_endpoint *rpmi_tee_endpoint_by_domain(struct sbi_domain *dom)
{
	struct rpmi_tee_endpoint *ep;

	if (!dom)
		return NULL;
	sbi_list_for_each_entry(ep, &tee_endpoints, node)
		if (!sbi_strcmp(ep->domain_name, dom->name))
			return ep;
	return NULL;
}

struct rpmi_tee_endpoint *rpmi_tee_endpoint_by_role(bool tee)
{
	struct rpmi_tee_endpoint *ep;

	sbi_list_for_each_entry(ep, &tee_endpoints, node)
		if (ep->tee == tee)
			return ep;
	return NULL;
}

int rpmi_tee_register_endpoint(const void *fdt, int nodeoff)
{
	struct rpmi_tee_endpoint *ep, *other;
	const fdt32_t *id, *owner;
	const char *domain_name, *name;
	const void *services;
	int len, domain_node, service_len;
	bool tee = !!fdt_getprop(fdt, nodeoff, "riscv,tee", NULL);

	id = fdt_getprop(fdt, nodeoff, "riscv,endpoint-id", &len);
	if (!id || len != sizeof(*id))
		return SBI_EINVAL;
	owner = fdt_getprop(fdt, nodeoff, "riscv,domain", &len);
	if (!owner || len != sizeof(*owner))
		return SBI_EINVAL;
	domain_node = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*owner));
	if (domain_node < 0)
		return SBI_EINVAL;
	domain_name = fdt_get_name(fdt, domain_node, NULL);
	if (!domain_name || sbi_strlen(domain_name) >= sizeof(ep->domain_name))
		return SBI_EINVAL;
	sbi_list_for_each_entry(other, &tee_endpoints, node) {
		if (other->tee == tee || other->id == fdt32_to_cpu(*id) ||
		    !sbi_strcmp(other->domain_name, domain_name))
			return SBI_EALREADY;
	}
	services = fdt_getprop(fdt, nodeoff, "riscv,services", &service_len);
	if (!services)
		service_len = 0;
	if (service_len % RPMI_TEE_UUID_SIZE || service_len > 4000)
		return SBI_EINVAL;

	ep = sbi_zalloc(sizeof(*ep));
	if (!ep)
		return SBI_ENOMEM;
	if (service_len) {
		ep->services = sbi_malloc(service_len);
		if (!ep->services) {
			sbi_free(ep);
			return SBI_ENOMEM;
		}
		sbi_memcpy(ep->services, services, service_len);
	}
	ep->id = fdt32_to_cpu(*id);
	ep->tee = tee;
	ATOMIC_INIT(&ep->available, 1);
	ep->service_count = service_len / RPMI_TEE_UUID_SIZE;
	sbi_strcpy(ep->domain_name, domain_name);
	name = fdt_get_name(fdt, nodeoff, NULL);
	sbi_strncpy(ep->name, name, sizeof(ep->name) - 1);
	sbi_list_add_tail(&ep->node, &tee_endpoints);
	rpmi_tee_memory_set_fdt(fdt);
	return 0;
}

int rpmi_tee_status(void *response, u32 maxlen, unsigned long *response_len,
		    s32 status)
{
	if (maxlen < sizeof(u32))
		return SBI_ENOSPC;
	*(u32 *)response = cpu_to_le32(status);
	*response_len = sizeof(u32);
	return 0;
}

static int tee_probe(struct rpmi_tee_endpoint *caller, u32 service,
		     const void *data, u32 len, void *response, u32 maxlen,
		     unsigned long *response_len)
{
	struct rpmi_tee_endpoint *ep;
	u32 id = 0, value = 0;

	if (service == RPMI_TEE_PROBE_FEATURES) {
		if (len != sizeof(u32) || maxlen < 2 * sizeof(u32))
			return rpmi_tee_status(response, maxlen, response_len,
					       RPMI_ERR_INVALID_PARAM);
		id = le32_to_cpu(*(const u32 *)data);
		if (id > RPMI_TEE_FEATURE_MULTISEGMENT_OPS)
			return rpmi_tee_status(response, maxlen, response_len,
					       RPMI_ERR_INVALID_PARAM);
		value = rpmi_tee_memory_feature(id);
		((u32 *)response)[0] = cpu_to_le32(RPMI_SUCCESS);
		((u32 *)response)[1] = cpu_to_le32(value);
		*response_len = 2 * sizeof(u32);
		return 0;
	}
	if (service == RPMI_TEE_PROBE_SYSTEM) {
		struct rpmi_tee_probe_system_resp *resp = response;
		u32 count = 0;

		if (len || maxlen < sizeof(*resp))
			goto invalid;
		sbi_list_for_each_entry(ep, &tee_endpoints, node) {
			struct sbi_domain *dom = rpmi_tee_endpoint_domain(ep);

			if (!dom || !rpmi_tee_endpoint_available(ep))
				continue;
			if (count >= (maxlen - sizeof(*resp)) / sizeof(u32))
				return SBI_ENOSPC;
			resp->domains[count++] = cpu_to_le32(dom->index);
		}
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->caller_domain =
			cpu_to_le32(rpmi_tee_endpoint_domain(caller)->index);
		resp->caller_endpoint = cpu_to_le32(caller->id);
		resp->domain_count = cpu_to_le32(count);
		*response_len = sizeof(*resp) + count * sizeof(u32);
		return 0;
	}
	if (len != sizeof(u32))
		goto invalid;
	id = le32_to_cpu(*(const u32 *)data);
	if (service == RPMI_TEE_PROBE_DOMAIN) {
		struct rpmi_tee_probe_domain_resp *resp = response;

		if (maxlen < sizeof(*resp) + sizeof(u32))
			return SBI_ENOSPC;
		sbi_list_for_each_entry(ep, &tee_endpoints, node) {
			struct sbi_domain *dom = rpmi_tee_endpoint_domain(ep);

			if (!dom || dom->index != id ||
			    !rpmi_tee_endpoint_available(ep))
				continue;
			resp->status = cpu_to_le32(RPMI_SUCCESS);
			resp->flags = cpu_to_le32(ep->tee ?
						 RPMI_TEE_DOMAIN_TEE : 0);
			resp->endpoint_count = cpu_to_le32(1);
			resp->endpoints[0] = cpu_to_le32(ep->id);
			*response_len = sizeof(*resp) + sizeof(u32);
			return 0;
		}
		goto invalid;
	}
	if (service == RPMI_TEE_PROBE_ENDPOINT) {
		struct rpmi_tee_probe_endpoint_resp *resp = response;
		u32 size;

		ep = rpmi_tee_endpoint_by_id(id);
		if (!ep || !rpmi_tee_endpoint_available(ep) ||
		    !rpmi_tee_endpoint_domain(ep))
			goto invalid;
		size = sizeof(*resp) + ep->service_count * RPMI_TEE_UUID_SIZE;
		if (size > maxlen)
			return SBI_ENOSPC;
		sbi_memset(resp, 0, sizeof(*resp));
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->flags = cpu_to_le32(RPMI_TEE_ENDPOINT_PHYSICAL |
					 (ep->tee ? RPMI_TEE_ENDPOINT_TEE : 0));
		sbi_memcpy(resp->name, ep->name, sizeof(resp->name));
		resp->service_count = cpu_to_le32(ep->service_count);
		if (ep->service_count)
			sbi_memcpy(resp->data, ep->services,
				   ep->service_count * RPMI_TEE_UUID_SIZE);
		*response_len = size;
		return 0;
	}
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_NOTSUPP);
invalid:
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_INVALID_PARAM);
}

static bool tee_has_service(struct rpmi_tee_endpoint *ep, const u8 *uuid)
{
	u32 i;

	for (i = 0; i < ep->service_count; i++)
		if (!sbi_memcmp(ep->services + i * RPMI_TEE_UUID_SIZE, uuid,
				RPMI_TEE_UUID_SIZE))
			return true;
	return false;
}

int rpmi_tee_check_domains(struct sbi_domain *source, struct sbi_domain *dest)
{
	struct rpmi_tee_endpoint *src = rpmi_tee_endpoint_by_domain(source);
	struct rpmi_tee_endpoint *target = rpmi_tee_endpoint_by_domain(dest);

	if (!src || !target || !target->tee)
		return SBI_EINVAL;
	/* OP-TEE must initialize before the REE takes over the boot hart. */
	if (source != dest && (source->boot_hartid != dest->boot_hartid ||
			       source->init_order <= dest->init_order))
		return SBI_EINVAL;
	/* The initial profile excludes architectural state not yet isolated. */
	if (misa_extension('H') || misa_extension('V') ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SSTC) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SMSTATEEN) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SSSTATEEN) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SMCDELEG) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SSCCFG) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SMCTR) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SSCTR))
		return SBI_ENOTSUPP;
	/* Domain migration needs synchronized protection updates. */
	if (__riscv_xlen != 64 || sbi_hart_count() != 1 ||
	    !sbi_hart_pmp_count(sbi_scratch_thishart_ptr()) ||
	    sbi_hart_has_extension(sbi_scratch_thishart_ptr(),
				   SBI_HART_EXT_SMEPMP))
		return SBI_ENOTSUPP;
	return 0;
}

static int tee_call(struct rpmi_tee_endpoint *caller, const void *data,
		    u32 len, void *response, u32 maxlen,
		    unsigned long *response_len)
{
	const struct rpmi_tee_call_req *req = data;
	struct rpmi_tee_endpoint *target;

	if (caller->tee)
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_NOTSUPP);
	if (len < sizeof(*req) ||
	    le32_to_cpu(req->sender_id) != caller->id ||
	    le32_to_cpu(req->service_data_len) != len - sizeof(*req))
		goto invalid;
	target = rpmi_tee_endpoint_by_id(le32_to_cpu(req->target_id));
	if (!target || !target->tee ||
	    !rpmi_tee_endpoint_available(target) ||
	    !tee_has_service(target, req->service))
		goto invalid;
	/* The single-hart caller cannot overwrite its pending request. */
	return SBI_ENOTSUPP;
invalid:
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_INVALID_PARAM);
}

static int tee_complete_response(struct mpxy_bridge_src_ctx *src,
				 const void *data, u32 len)
{
	if (len < sizeof(u32) || len > src->resp_bufsize || len % sizeof(u32))
		return SBI_EINVAL;

	sbi_memcpy(src->respbuf, data, len);
	src->resp_length = len;
	return 0;
}

int rpmi_tee_complete(struct mpxy_bridge_src_ctx *src, void *data, u32 len)
{
	const struct rpmi_tee_call_resp *resp = data;
	u32 error = cpu_to_le32(RPMI_ERR_IO);

	if (src->header.service_id != RPMI_TEE_CALL ||
	    len < sizeof(*resp) || le32_to_cpu(resp->status) != RPMI_SUCCESS ||
	    le32_to_cpu(resp->service_data_len) != len - sizeof(*resp))
		return tee_complete_response(src, &error, sizeof(error));
	return tee_complete_response(src, data, len);
}

int rpmi_tee_request(struct sbi_domain *dom, u32 service, const void *data,
		     u32 len, void *response, u32 maxlen,
		     unsigned long *response_len)
{
	struct rpmi_tee_endpoint *caller = rpmi_tee_endpoint_by_domain(dom);

	if (!caller || !rpmi_tee_endpoint_available(caller))
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_DENIED);
	if (service == RPMI_TEE_ENABLE_NOTIFICATION) {
		const struct rpmi_enable_notification_req *req = data;

		if (len != sizeof(*req) ||
		    le32_to_cpu(req->req_state) >= RPMI_EVENT_NOTIF_MAX_COUNT)
			return rpmi_tee_status(response, maxlen, response_len,
					       RPMI_ERR_INVALID_PARAM);
		/* The TEE service group defines no notification events. */
		return rpmi_tee_status(response, maxlen, response_len,
				       RPMI_ERR_NOTSUPP);
	}
	if (service >= RPMI_TEE_MEMORY_PARCEL_CREATE &&
	    service <= RPMI_TEE_MEMORY_SEGMENT_RECEIVE)
		return rpmi_tee_memory_request(caller, service, data, len,
					       response, maxlen, response_len);
	if (service == RPMI_TEE_CALL)
		return tee_call(caller, data, len, response, maxlen,
				response_len);
	if (service >= RPMI_TEE_PROBE_FEATURES &&
	    service <= RPMI_TEE_PROBE_ENDPOINT)
		return tee_probe(caller, service, data, len, response, maxlen,
				 response_len);
	return rpmi_tee_status(response, maxlen, response_len,
			       RPMI_ERR_NOTSUPP);
}
