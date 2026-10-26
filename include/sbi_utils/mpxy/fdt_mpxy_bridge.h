/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2026 Qualcomm, Inc
 *
 * Authors:
 *   Pawandeep Oza <pawandeep.oza at oss.qualcomm.com>
 */

#ifndef __FDT_MPXY_BRIDGE_H__
#define __FDT_MPXY_BRIDGE_H__

#include <sbi/sbi_console.h>
#include <sbi/sbi_types.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi.h>

#define SBI_MPXY_BRIDGE_DEFAULT_MSG_DATA_MAXLEN  4096
#define SBI_MPXY_BRIDGE_DOMAIN_NAME_MAXLEN       64

struct mpxy_bridge_source;

struct mpxy_bridge_dst_ctx {
	void          *respbuf;
	u32            resp_bufsize;
	unsigned long resp_len;
	bool waiting;
	u32            start_index;
};

struct mpxy_bridge_src_ctx {
	struct rpmi_message_header header;;
	bool occupied;
	bool retrieved;
	bool completed;
	void *msgbuf;
	void *respbuf;
	u32  msg_len;
	u32  resp_bufsize;
	u32  resp_length;
};

struct mpxy_bridge_source_driver {
    const struct fdt_match *match_table;
    int (*probe)(const void *fdt,
			int nodeoff, const struct fdt_match *match);
};

struct mpxy_bridge_source_ops {
	/** Require a pending destination retrieval before forwarding. */
	bool require_waiting_retrieval;
	/** Roll back local preparation when forwarding fails. */
	void (*cancel)(u32 service);
	/** Validate the source and destination domain association at boot. */
	int (*check_domains)(struct sbi_domain *source,
			     struct sbi_domain *dest);
	/** Handle a local service; SBI_ENOTSUPP selects forwarding. */
	int (*request)(struct sbi_domain *dom, u32 service, const void *data,
		       u32 len, void *response, u32 maxlen,
		       unsigned long *response_len);
	int (*xfer) (struct mpxy_bridge_src_ctx *src,
				struct mpxy_bridge_dst_ctx *dst);
	int (*complete) (struct mpxy_bridge_src_ctx *src,
				void *msgbuf, u32 msg_len);
};

int mpxy_bridge_register_source(const void *fdt, int nodeoff,
			    const struct mpxy_bridge_source_ops *ops,
			    struct mpxy_rpmi_service_data *srvcdata,
			    u32 num_services,
			    enum rpmi_servicegroup_id srvgrpid);

#endif /* __FDT_MPXY_BRIDGE_H__ */
