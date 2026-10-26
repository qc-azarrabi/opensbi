/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#ifndef __MPXY_RPMI_TEE_H__
#define __MPXY_RPMI_TEE_H__

#include <sbi/riscv_atomic.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_list.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>

struct mpxy_bridge_src_ctx;
struct mpxy_rpmi_data;

/* Endpoint registry */

/** Physical endpoint provisioned by trusted firmware configuration. */
struct rpmi_tee_endpoint {
	struct sbi_dlist node;
	u32 id;
	bool tee;
	/* Cleared when the endpoint can no longer serve requests. */
	atomic_t available;
	char name[RPMI_TEE_NAME_SIZE];
	char domain_name[64];
	u32 service_count;
	u8 *services;
};

/** Endpoint storage is boot-time owned; availability can change at runtime. */
static inline bool rpmi_tee_endpoint_available(struct rpmi_tee_endpoint *ep)
{
	return atomic_read(&ep->available) != 0;
}

int rpmi_tee_register_endpoint(const void *fdt, int nodeoff);
struct rpmi_tee_endpoint *rpmi_tee_endpoint_by_id(u32 id);
struct rpmi_tee_endpoint *rpmi_tee_endpoint_by_domain(struct sbi_domain *dom);
struct rpmi_tee_endpoint *rpmi_tee_endpoint_by_role(bool tee);
struct sbi_domain *rpmi_tee_endpoint_domain(struct rpmi_tee_endpoint *ep);

/* MPXY channel configuration */

extern const struct mpxy_rpmi_data rpmi_tee_local_data;
int rpmi_tee_channel_get_attribute(u32 attr_id, u32 *value);

/* Request dispatch and bridge integration */

/** Process local services; SBI_ENOTSUPP requests validated forwarding. */
int rpmi_tee_request(struct sbi_domain *dom, u32 service, const void *data,
		     u32 len, void *response, u32 maxlen,
		     unsigned long *response_len);

/* Complete forwarded SIGNAL_BUS_SETUP, SIGNAL_BUS_TEARDOWN and TEE_CALL. */
int rpmi_tee_complete(struct mpxy_bridge_src_ctx *src, void *data, u32 len);

/* Memory parcels */

u32 rpmi_tee_memory_feature(u32 feature);
int rpmi_tee_memory_request(struct rpmi_tee_endpoint *caller, u32 service,
			    const void *data, u32 len, void *response,
			    u32 maxlen, unsigned long *response_len);
void rpmi_tee_memory_disconnect(struct rpmi_tee_endpoint *ep);

/* Signal buses */

void rpmi_tee_signal_set_notify(void (*notify)(u32 endpoint_id));
u32 rpmi_tee_signal_feature(struct rpmi_tee_endpoint *ep);
bool rpmi_tee_signal_pending(u32 endpoint_id);

/* Handle signal requests; return SBI_ENOTSUPP to forward SETUP/TEARDOWN. */
int rpmi_tee_signal_request(struct rpmi_tee_endpoint *caller, u32 service,
			    const void *data, u32 len, void *response,
			    u32 maxlen, unsigned long *response_len);

s32 rpmi_tee_signal_complete(u32 service, const void *data, u32 len);
void rpmi_tee_signal_cancel(u32 service);

#endif
