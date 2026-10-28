/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <libfdt.h>
#include <sbi/sbi_error.h>
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
	/* TEE vs. REE. */
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
		/* Allow one endpoint per role, with unique IDs and domains. */
		if (other->tee == tee || other->id == fdt32_to_cpu(*id) ||
		    !sbi_strcmp(other->domain_name, domain_name))
			return SBI_EALREADY;
	}

	services = fdt_getprop(fdt, nodeoff, "riscv,services", &service_len);
	if (!services)
		service_len = 0;
	/* Leave room for the 56-byte PROBE_ENDPOINT response prefix in page. */
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
	/* Publish the initialized endpoint in the registry. */
	sbi_list_add_tail(&ep->node, &tee_endpoints);

	return 0;
}

/* Handle feature and topology queries using the endpoint registry. */
static int tee_handle_probe(struct rpmi_tee_endpoint *caller, u32 service,
			    const void *data, u32 len, void *response, u32 maxlen,
			    unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	s32 status = RPMI_ERR_INVALID_PARAM;

	switch (service) {
	case RPMI_TEE_PROBE_FEATURES: {
		const struct rpmi_tee_probe_features_req *req = data;
		struct rpmi_tee_probe_features_resp *resp = response;
		u32 id, value;

		if (len != sizeof(*req) || maxlen < sizeof(*resp))
			goto out_status;

		id = le32_to_cpu(req->feature_id);
		if (id > RPMI_TEE_FEATURE_MULTISEGMENT_OPS)
			goto out_status;
		if (id == RPMI_TEE_FEATURE_SIGNAL_BUS)
			value = rpmi_tee_signal_feature(caller);
		else
			value = rpmi_tee_memory_feature(id);

		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->value = cpu_to_le32(value);
		*response_len = sizeof(*resp);

		return 0;
	}

	case RPMI_TEE_PROBE_SYSTEM: {
		struct rpmi_tee_probe_system_resp *resp = response;
		struct rpmi_tee_endpoint *ep;
		u32 count = 0;

		/* No req for PROBE_SYSTEM. */
		if (len || maxlen < sizeof(*resp))
			goto out_status;

		sbi_list_for_each_entry(ep, &tee_endpoints, node) {
			struct sbi_domain *dom = rpmi_tee_endpoint_domain(ep);
			/* Skip unavailable endpoints or missing domains. */
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

	case RPMI_TEE_PROBE_DOMAIN: {
		const struct rpmi_tee_probe_domain_req *req = data;
		struct rpmi_tee_probe_domain_resp *resp = response;
		struct rpmi_tee_endpoint *ep;
		u32 id;

		if (len != sizeof(*req))
			goto out_status;
		/* Each domain has one endpoint; reserve room for its ID. */
		if (maxlen < sizeof(*resp) + sizeof(u32))
			return SBI_ENOSPC;

		id = le32_to_cpu(req->domain_id);

		sbi_list_for_each_entry(ep, &tee_endpoints, node) {
			struct sbi_domain *dom = rpmi_tee_endpoint_domain(ep);
			/* Find an available endpoint in the requested domain. */
			if (!dom || dom->index != id ||
			    !rpmi_tee_endpoint_available(ep))
				continue;

			resp->status = cpu_to_le32(RPMI_SUCCESS);
			resp->flags = cpu_to_le32(ep->tee ?
						  RPMI_TEE_DOMAIN_TEE : 0);
			resp->endpoint_count = cpu_to_le32(1); /* Only 1 ep.*/
			resp->endpoints[0] = cpu_to_le32(ep->id);
			*response_len = sizeof(*resp) + sizeof(u32);

			return 0;
		}
		goto out_status;
	}

	case RPMI_TEE_PROBE_ENDPOINT: {
		const struct rpmi_tee_probe_endpoint_req *req = data;
		struct rpmi_tee_probe_endpoint_resp *resp = response;
		struct rpmi_tee_endpoint *ep;
		u32 size;

		if (len != sizeof(*req))
			goto out_status;

		ep = rpmi_tee_endpoint_by_id(le32_to_cpu(req->endpoint_id));
		if (!ep || !rpmi_tee_endpoint_available(ep) ||
		    !rpmi_tee_endpoint_domain(ep))
			goto out_status;

		/* The fixed response fields are followed by service UUIDs. */
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

	default:
		status = RPMI_ERR_NOTSUPP;
		break;
	}

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
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

/* Validate TEE_CALL and select forwarding or prepare an error response. */
static int tee_call(struct rpmi_tee_endpoint *caller, const void *data,
		    u32 len, void *response, u32 maxlen,
		    unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	const struct rpmi_tee_call_req *req = data;
	struct rpmi_tee_endpoint *target;
	s32 status = RPMI_ERR_INVALID_PARAM;

	/* Only REE-to-TEE calls are supported, for now. */
	if (caller->tee) {
		status = RPMI_ERR_NOTSUPP;
		goto out_status;
	}

	if (len < sizeof(*req) ||
	    le32_to_cpu(req->sender_id) != caller->id ||
	    le32_to_cpu(req->service_data_len) != len - sizeof(*req))
		goto out_status;

	target = rpmi_tee_endpoint_by_id(le32_to_cpu(req->target_id));
	if (!target || !target->tee ||
	    !rpmi_tee_endpoint_available(target) ||
	    !tee_has_service(target, req->service))
		goto out_status;

	/* Ask the bridge to forward the validated call to the target TEE. */
	return SBI_ENOTSUPP;

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
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

	if (src->header.service_id != RPMI_TEE_CALL)
		return tee_complete_response(src, &error, sizeof(error));

	if (len < sizeof(*resp))
		return tee_complete_response(src, &error, sizeof(error));

	/* Outer RPMI errors -> RPMI_ERR_IO as aborted request by TEE. */
	if (le32_to_cpu(resp->status) != RPMI_SUCCESS)
		return tee_complete_response(src, &error, sizeof(error));

	if (le32_to_cpu(resp->service_data_len) != len - sizeof(*resp))
		return tee_complete_response(src, &error, sizeof(error));

	return tee_complete_response(src, data, len);
}

/* Handle requests locally; return SBI_ENOTSUPP to request forwarding. */
int rpmi_tee_request(struct sbi_domain *dom, u32 service, const void *data,
		     u32 len, void *response, u32 maxlen,
		     unsigned long *response_len)
{
	struct rpmi_tee_status_resp *status_resp = response;
	struct rpmi_tee_endpoint *caller = rpmi_tee_endpoint_by_domain(dom);
	s32 status;

	if (!caller || !rpmi_tee_endpoint_available(caller)) {
		status = RPMI_ERR_DENIED;
		goto out_status;
	}

	if (service >= RPMI_TEE_MEMORY_PARCEL_CREATE &&
	    service <= RPMI_TEE_MEMORY_SEGMENT_RECEIVE)
		return rpmi_tee_memory_request(caller, service, data, len,
					       response, maxlen, response_len);

	/* A validated call returns SBI_ENOTSUPP to request forwarding. */
	if (service == RPMI_TEE_CALL)
		return tee_call(caller, data, len, response, maxlen,
				response_len);

	if (service >= RPMI_TEE_PROBE_FEATURES &&
	    service <= RPMI_TEE_PROBE_ENDPOINT)
		return tee_handle_probe(caller, service, data, len, response, maxlen,
				 response_len);

	status = RPMI_ERR_NOTSUPP;

out_status:
	if (maxlen < sizeof(*status_resp))
		return SBI_ENOSPC;

	status_resp->status = cpu_to_le32(status);
	*response_len = sizeof(*status_resp);

	return 0;
}
