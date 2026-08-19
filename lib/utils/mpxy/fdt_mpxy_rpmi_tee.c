/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Andes Technology Corporation.
 * Copyright (c) 2026 SiFive Inc.
 *
 * Generic TEE Service Group for MPXY RPMI
 *
 * This implementation follows the RPMI TEE Service Group specification
 * and delegates TEE-specific operations to registered dispatchers.
 */

#include <libfdt.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_mpxy.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_mbox.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_tee.h>
#include <sbi_utils/mailbox/rpmi_mailbox.h>

/**
 * TEE MPXY channel context
 *
 * This structure represents a per-hart TEE Service Group MPXY channel.
 * Each hart has its own mpxy_tee instance that routes TEE_COMMUNICATE
 * requests to the appropriate TEE dispatcher.
 */
struct mpxy_tee {
	/** TEE dispatcher shared across harts for same TEE Implementation */
	struct tee_dispatcher *dispatcher;
	/** TEE attributes */
	struct tee_attributes attrs;
	/** RPMI channel attributes for MPXY */
	struct mpxy_rpmi_channel_attrs msgprot_attrs;
	/** MPXY channel instance */
	struct sbi_mpxy_channel channel;
	/** Owner Hart ID of this channel */
	u32 hartid;
	/** List node for TEE instance tracking */
	struct sbi_dlist node;
};

/**
 * Read RPMI message protocol attributes
 */
static int mpxy_tee_read_attributes(struct sbi_mpxy_channel *channel,
				    u32 *outmem, u32 base_attr_id,
				    u32 attr_count)
{
	struct mpxy_tee *tee =
		container_of(channel, struct mpxy_tee, channel);
	u32 end_id = base_attr_id + attr_count - 1;

	if (end_id >= MPXY_MSGPROT_RPMI_ATTR_MAX_ID ||
	    base_attr_id < MPXY_MSGPROT_RPMI_ATTR_SERVICEGROUP_ID)
		return SBI_EBAD_RANGE;

	sbi_memcpy(outmem,
		   (void *)&tee->msgprot_attrs + attr_id2index(base_attr_id) *
		   sizeof(u32), attr_count * sizeof(u32));
	return SBI_OK;
}

/**
 * Send TEE message with response
 */
static int mpxy_tee_send_message_with_response(struct sbi_mpxy_channel *channel,
					       u32 msg_id, void *msgbuf,
					       u32 msg_len, void *respbuf,
					       u32 resp_max_len,
					       unsigned long *resp_len)
{
	struct mpxy_tee *tee =
		container_of(channel, struct mpxy_tee, channel);
	s32 *status = (s32 *)respbuf;
	int rc = SBI_OK;

	if (!tee->dispatcher)
		return SBI_ENODEV;

	switch (msg_id) {
	case RPMI_TEE_SRV_ENABLE_NOTIFICATION:
		/*
		 * The TEE service group defines no events, so notification
		 * enable is answered directly by the framework as
		 * not-supported. No TEE domain involvement.
		 */
		if (resp_max_len < sizeof(s32))
			return SBI_ENOMEM;
		*status = cpu_to_le32(RPMI_ERR_NOTSUPP);
		*resp_len = sizeof(s32);
		break;

	case RPMI_TEE_SRV_PROBE_FEATURES: {
		/*
		 * TEE_PROBE_FEATURES is answered directly by the framework
		 * (no TEE domain involvement). None of the optional features
		 * are supported by this prototype; an unknown feature id is
		 * rejected.
		 */
		struct rpmi_tee_probe_features_req *feat_req = msgbuf;
		struct rpmi_tee_probe_features_resp *feat_resp = respbuf;
		u32 feature_id;

		if (resp_max_len < sizeof(*feat_resp))
			return SBI_ENOMEM;
		if (msg_len < sizeof(*feat_req)) {
			feat_resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			feat_resp->value = 0;
			*resp_len = sizeof(*feat_resp);
			break;
		}

		feature_id = le32_to_cpu(feat_req->feature_id);
		switch (feature_id) {
		case RPMI_TEE_FEAT_MEMORY_DONATE:
		case RPMI_TEE_FEAT_MEMORY_LEND:
		case RPMI_TEE_FEAT_MEMORY_SHARE:
		case RPMI_TEE_FEAT_SIGNAL_BUS:
		case RPMI_TEE_FEAT_MULTISEGMENT_OPS:
		case RPMI_TEE_FEAT_SYSINFO_FORMAT:
			feat_resp->status = cpu_to_le32(RPMI_SUCCESS);
			feat_resp->value = 0;
			break;
		default:
			feat_resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			feat_resp->value = 0;
			break;
		}
		*resp_len = sizeof(*feat_resp);
		break;
	}

	case RPMI_TEE_SRV_COMMUNICATE:
		/*
		 * TEE_COMMUNICATE response format:
		 *   Word 0:    STATUS (RPMI error code)
		 *   Word 1..M: Implementation-specific data (XLEN-sized registers)
		 *
		 * Reserve space for STATUS, pass remaining buffer to dispatcher.
		 * The response data size is determined by comm_resp_regs from
		 * TEE attributes (e.g., 4 registers for OP-TEE = 32 bytes on RV64).
		 */
		if (resp_max_len < sizeof(s32)) {
			rc = SBI_ENOMEM;
			break;
		}

		if (tee->dispatcher->ops->communicate) {
			unsigned long data_len = 0;
			void *data_buf = (u8 *)respbuf + sizeof(u32);
			u32 data_max_len = resp_max_len - sizeof(u32);

			rc = tee->dispatcher->ops->communicate(
				tee->dispatcher,
				msgbuf, msg_len,
				data_buf, data_max_len,
				&data_len);

			/* Set RPMI status based on dispatcher result */
			if (rc) {
				*status = cpu_to_le32(RPMI_ERR_FAILED);
				*resp_len = sizeof(s32);
			} else {
				/*
				 * Enter TEE domain if supported. This blocks
				 * until TEE processing completes and the domain
				 * switches back. After this returns, the response
				 * data has been written to data_buf by the reqfwd
				 * COMPLETE_CURRENT_MESSAGE handler.
				 */
				if (tee->dispatcher->ops->domain_enter) {
					rc = tee->dispatcher->ops->domain_enter(
						tee->dispatcher);
					if (rc) {
						*status = cpu_to_le32(RPMI_ERR_FAILED);
						*resp_len = sizeof(s32);
						break;
					}
				}

				*status = cpu_to_le32(RPMI_SUCCESS);
				/*
				 * data_len may be the REQFWD RETRIEVE
				 * response length when resuming a waiting
				 * OP-TEE domain.
				 * It already includes the RPMI status/header.
				 * Should not add the TEE status word here.
				 */
				*resp_len = data_len;
			}
		} else {
			*status = cpu_to_le32(RPMI_ERR_NOTSUPP);
			*resp_len = sizeof(s32);
		}
		break;

	default:
		*status = cpu_to_le32(RPMI_ERR_NOTSUPP);
		*resp_len = sizeof(u32);
		break;
	}

	return rc;
}

/** List to track all registered TEE instances for hartid lookup */
static SBI_LIST_HEAD(mpxy_tee_list);

/**
 * Find TEE MPXY channel by hartid
 *
 * Each TEE MPXY channel is associated with a specific hart. This function
 * searches the registered TEE channels and returns the MPXY channel
 * for the specified hartid.
 *
 * @param hartid: The hart ID to search for
 * @return Pointer to the MPXY channel, or NULL if not found
 */
struct sbi_mpxy_channel *mpxy_tee_find_channel_by_hartid(u32 hartid)
{
	struct mpxy_tee *tee;

	sbi_list_for_each_entry(tee, &mpxy_tee_list, node)
		if (tee->hartid == hartid)
			return &tee->channel;

	return NULL;
}

/**
 * Get TEE dispatcher from TEE MPXY channel
 *
 * This function extracts the TEE dispatcher from a TEE MPXY channel.
 * Used by the reqfwd driver for lazy callback registration.
 *
 * @param channel: Pointer to TEE MPXY channel
 * @return Pointer to TEE dispatcher, or NULL if not found
 */
struct tee_dispatcher *mpxy_tee_get_dispatcher(struct sbi_mpxy_channel *channel)
{
	struct mpxy_tee *tee;

	if (!channel)
		return NULL;

	tee = container_of(channel, struct mpxy_tee, channel);
	return tee->dispatcher;
}

/**
 * Initialize TEE MPXY channel
 *
 * Each hart has its own TEE MPXY channel. The dispatcher handles any
 * TEE-specific per-hart setup (e.g., OP-TEE's reqfwd channel binding).
 */
static int mpxy_tee_init(const void *fdt, int nodeoff,
			 const struct fdt_match *match)
{
	struct mpxy_tee *tee;
	const fdt32_t *val;
	u32 channel_id, hartid;
	int rc, len, cpu_offset;

	/* Allocate context for TEE MPXY */
	tee = sbi_zalloc(sizeof(*tee));
	if (!tee)
		return SBI_ENOMEM;

	/* Get channel ID from DT */
	val = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (len > 0 && val)
		channel_id = fdt32_to_cpu(*val);
	else {
		rc = SBI_EINVAL;
		goto fail_free;
	}

	/* Get parent CPU node to extract hartid from its reg property */
	cpu_offset = fdt_parent_offset(fdt, nodeoff);
	if (cpu_offset < 0) {
		rc = SBI_EINVAL;
		goto fail_free;
	}

	rc = fdt_parse_hart_id(fdt, cpu_offset, &hartid);
	if (rc)
		goto fail_free;

	tee->hartid = hartid;

	/* Setup TEE dispatcher from device tree */
	rc = tee_dispatcher_setup(fdt, nodeoff, &tee->dispatcher);
	if (rc)
		goto fail_free;

	/* Get TEE attributes */
	if (tee->dispatcher->ops->get_attributes) {
		rc = tee->dispatcher->ops->get_attributes(
			tee->dispatcher, &tee->attrs);
		if (rc)
			goto fail_free;
	}

	/* Setup MPXY channel */
	tee->channel.channel_id = channel_id;
	tee->channel.attrs.msg_proto_id = SBI_MPXY_MSGPROTO_RPMI_ID;
	tee->channel.attrs.msg_proto_version = 1;
	tee->channel.attrs.msg_data_maxlen = PAGE_SIZE;
	tee->channel.attrs.msg_send_timeout = 0;
	tee->channel.attrs.msg_completion_timeout = 0;
	tee->channel.read_attributes = mpxy_tee_read_attributes;
	tee->channel.send_message_with_response =
		mpxy_tee_send_message_with_response;

	/* Setup RPMI service group attributes */
	tee->msgprot_attrs.servicegrp_id = RPMI_SRVGRP_TEE;
	tee->msgprot_attrs.servicegrp_ver = 1;

	/* Register MPXY channel */
	rc = sbi_mpxy_register_channel(&tee->channel);
	if (rc)
		goto fail_free;

	/* Add to TEE list for hartid lookup */
	sbi_list_add_tail(&tee->node, &mpxy_tee_list);

	return SBI_OK;

fail_free:
	sbi_free(tee);
	return rc;
}

/** Device tree match table */
static const struct fdt_match tee_match[] = {
	{
		.compatible = "riscv,rpmi-mpxy-tee",
		.data = NULL,
	},
	{},
};

/** TEE MPXY driver */
const struct fdt_driver fdt_mpxy_rpmi_tee = {
	.experimental = true,
	.match_table = tee_match,
	.init = mpxy_tee_init,
};
