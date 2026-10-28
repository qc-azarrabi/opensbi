/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <libfdt.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

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
