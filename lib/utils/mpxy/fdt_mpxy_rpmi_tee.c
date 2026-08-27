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
#include <sbi_utils/mailbox/fdt_mailbox_rpmi_sysmsi.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_tee.h>
#include <sbi_utils/mpxy/rpmi_tee_parcel.h>
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

#define TEE_SIGNAL_BUS_MAX_WIDTH	32
#define TEE_SIGNAL_MSI_INDEX		0

static struct {
	bool active;
	u32 target_id;		/* target physical endpoint (OP-TEE) */
	u32 width;		/* M: total signals */
	u32 sender_signals;	/* N: signals OP-TEE raises / REE reads */
	u32 pending_for_ree;	/* raised by OP-TEE, awaiting REE retrieve */
	u32 pending_for_tee;	/* raised by REE, awaiting OP-TEE retrieve */
} tee_signal_bus;

/* True if signal index x is in the range the REE is allowed to raise: [N,M). */
static inline bool sig_in_ree_raise_range(u32 x)
{
	return x >= tee_signal_bus.sender_signals && x < tee_signal_bus.width;
}

/* True if signal index x is in the range OP-TEE is allowed to raise: [0,N). */
static inline bool sig_in_tee_raise_range(u32 x)
{
	return x < tee_signal_bus.sender_signals;
}

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
		 * The TEE service group (section 4.16.2) defines no events, so
		 * notification enable is answered directly by the framework
		 * as not-supported. No TEE domain involvement.
		 */
		if (resp_max_len < sizeof(s32))
			return SBI_ENOMEM;
		*status = cpu_to_le32(RPMI_ERR_NOTSUPP);
		*resp_len = sizeof(s32);
		break;

	case RPMI_TEE_SRV_PROBE_FEATURES: {
		/*
		 * TEE_PROBE_FEATURES is answered directly by the framework
		 * (no TEE domain involvement). The memory-parcel modes
		 * (donate/lend/share) are supported for both REE and TEE and
		 * report value 2; the remaining optional features are not yet
		 * supported and report value 0. An unknown feature id is
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
		case RPMI_TEE_FEAT_MULTISEGMENT_OPS:
			feat_resp->status = cpu_to_le32(RPMI_SUCCESS);
			feat_resp->value =
				cpu_to_le32(RPMI_TEE_FEAT_VAL_FULL_REE_TEE);
			break;
		case RPMI_TEE_FEAT_SIGNAL_BUS:
			/*
			 * Signal bus supported, signals delivered as System MSI
			 * (delivery mode 1). Under polling the MSI index is
			 * informational; the SYSIRQ prototype later reports mode 2
			 * with the same services. Encode index/width/mode per spec.
			 */
			feat_resp->status = cpu_to_le32(RPMI_SUCCESS);
			feat_resp->value = cpu_to_le32(RPMI_TEE_SIGNAL_BUS_VALUE(
				TEE_SIGNAL_MSI_INDEX, TEE_SIGNAL_BUS_MAX_WIDTH,
				RPMI_TEE_SIGNAL_DELIVERY_MSI));
			break;
		case RPMI_TEE_FEAT_SYSINFO_FORMAT:
			/*
			 * SYSINFO_FORMAT's value is the encoding format of the
			 * PROBE_SYSTEM blob, not a REE/TEE support level. We
			 * emit CBOR.
			 */
			feat_resp->status = cpu_to_le32(RPMI_SUCCESS);
			feat_resp->value =
				cpu_to_le32(RPMI_TEE_SYSINFO_FORMAT_CBOR);
			break;
		default:
			feat_resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			feat_resp->value = 0;
			break;
		}
		*resp_len = sizeof(*feat_resp);
		break;
	}

	case RPMI_TEE_SRV_TEE_CALL: {
		/*
		 * TEE_CALL (RPMI spec section 4.16, Tables 218/219).
		 *
		 * The request carries a fixed REE->OP-TEE identity and a
		 * well-known service UUID; its SERVICE_DATA is the SMC-style
		 * a0-a7 register block. After validation the register block is
		 * forwarded to the TEE dispatcher unchanged (identical to the
		 * legacy COMMUNICATE payload), so OP-TEE needs no change.
		 *
		 * Response layout:
		 *   Word 0:    STATUS (s32)
		 *   Word 1:    SERVICE_RSP_LEN (u32)
		 *   Bytes 8..: SERVICE_RSP (comm_resp_regs XLEN-sized registers)
		 */
		static const u8 optee_service_uuid[16] =
			RPMI_TEE_OPTEE_SERVICE_UUID;
		struct rpmi_tee_call_req *call_req = msgbuf;
		struct rpmi_tee_call_resp *call_resp = respbuf;
		u32 service_data_len;

		if (resp_max_len < sizeof(*call_resp)) {
			rc = SBI_ENOMEM;
			break;
		}

		/* Validate framing: header present, fixed identity + UUID */
		if (msg_len < sizeof(*call_req) ||
		    le32_to_cpu(call_req->sender_id) != RPMI_TEE_ENDPOINT_REE ||
		    le32_to_cpu(call_req->target_id) != RPMI_TEE_ENDPOINT_OPTEE ||
		    sbi_memcmp(call_req->service, optee_service_uuid,
			       sizeof(optee_service_uuid))) {
			call_resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			call_resp->service_rsp_len = 0;
			*resp_len = sizeof(*call_resp);
			break;
		}

		service_data_len = le32_to_cpu(call_req->service_data_len);
		if (!service_data_len ||
		    service_data_len > msg_len - sizeof(*call_req)) {
			call_resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			call_resp->service_rsp_len = 0;
			*resp_len = sizeof(*call_resp);
			break;
		}

		if (tee->dispatcher->ops->communicate) {
			unsigned long data_len = 0;
			void *data_buf = call_resp->service_rsp;
			u32 data_max_len = resp_max_len - sizeof(*call_resp);
			u32 rsp_len = tee->attrs.comm_resp_regs *
				      sizeof(unsigned long);

			rc = tee->dispatcher->ops->communicate(
				tee->dispatcher,
				call_req->service_data, service_data_len,
				data_buf, data_max_len,
				&data_len);
			if (rc) {
				call_resp->status = cpu_to_le32(RPMI_ERR_FAILED);
				call_resp->service_rsp_len = 0;
				*resp_len = sizeof(*call_resp);
				break;
			}

			/*
			 * Enter TEE domain. Blocks until OP-TEE processing
			 * completes and the domain switches back; the reqfwd
			 * COMPLETE handler writes the stripped a0-a3 register
			 * block into data_buf. The response register count is
			 * fixed by the dispatcher (comm_resp_regs), so the
			 * SERVICE_RSP length is deterministic and must NOT be
			 * taken from the reqfwd retrieve length in data_len.
			 */
			if (tee->dispatcher->ops->domain_enter) {
				rc = tee->dispatcher->ops->domain_enter(
					tee->dispatcher);
				if (rc) {
					call_resp->status =
						cpu_to_le32(RPMI_ERR_FAILED);
					call_resp->service_rsp_len = 0;
					*resp_len = sizeof(*call_resp);
					break;
				}
			}

			call_resp->status = cpu_to_le32(RPMI_SUCCESS);
			call_resp->service_rsp_len = cpu_to_le32(rsp_len);
			*resp_len = sizeof(*call_resp) + rsp_len;
		} else {
			call_resp->status = cpu_to_le32(RPMI_ERR_NOTSUPP);
			call_resp->service_rsp_len = 0;
			*resp_len = sizeof(*call_resp);
		}
		break;
	}

	case RPMI_TEE_SRV_MEM_PARCEL_CREATE:
		rc = rpmi_tee_parcel_create(msgbuf, msg_len, respbuf,
					    resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_MEM_PARCEL_ACCEPT:
		rc = rpmi_tee_parcel_accept(msgbuf, msg_len, respbuf,
					    resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_MEM_PARCEL_RELEASE:
		rc = rpmi_tee_parcel_release(msgbuf, msg_len, respbuf,
					     resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_MEM_PARCEL_RECLAIM:
		rc = rpmi_tee_parcel_reclaim(msgbuf, msg_len, respbuf,
					     resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_MEM_PARCEL_SEGMENT_SEND:
		rc = rpmi_tee_parcel_segment_send(msgbuf, msg_len, respbuf,
						  resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_MEM_PARCEL_SEGMENT_RECEIVE:
		rc = rpmi_tee_parcel_segment_receive(msgbuf, msg_len, respbuf,
						     resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_PROBE_SYSTEM:
		rc = rpmi_tee_parcel_probe_system(msgbuf, msg_len, respbuf,
						  resp_max_len, resp_len);
		break;

	case RPMI_TEE_SRV_SIGNAL_BUS_SETUP: {
		/*
		 * REE-initiated setup of the REE<->TEE signal bus (spec 4.16.7).
		 * The framework validates parameters; the target (OP-TEE) would
		 * accept/deny. For the prototype the framework accepts on the
		 * target's behalf (single well-known OP-TEE endpoint).
		 */
		struct rpmi_tee_signal_bus_setup_req *req = msgbuf;
		struct rpmi_tee_signal_bus_setup_resp *resp = respbuf;
		u32 target_id, width, sender_signals;

		if (resp_max_len < sizeof(*resp))
			return SBI_ENOMEM;
		if (msg_len < sizeof(*req)) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			break;
		}

		target_id = le32_to_cpu(req->target_id);
		width = le32_to_cpu(req->bus_width);
		sender_signals = le32_to_cpu(req->sender_signals);

		if (target_id != RPMI_TEE_ENDPOINT_OPTEE)
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		else if (!width || width > TEE_SIGNAL_BUS_MAX_WIDTH ||
			 sender_signals > width)
			resp->status = cpu_to_le32(RPMI_ERR_BAD_RANGE);
		else if (tee_signal_bus.active)
			resp->status = cpu_to_le32(RPMI_ERR_ALREADY);
		else {
			tee_signal_bus.active = true;
			tee_signal_bus.target_id = target_id;
			tee_signal_bus.width = width;
			tee_signal_bus.sender_signals = sender_signals;
			tee_signal_bus.pending_for_ree = 0;
			tee_signal_bus.pending_for_tee = 0;
			resp->status = cpu_to_le32(RPMI_SUCCESS);
		}
		*resp_len = sizeof(*resp);
		break;
	}

	case RPMI_TEE_SRV_SIGNAL_BUS_TEARDOWN: {
		/* REE-initiated teardown (spec 4.16.8). */
		struct rpmi_tee_signal_bus_teardown_req *req = msgbuf;
		struct rpmi_tee_signal_bus_teardown_resp *resp = respbuf;
		u32 target_id;

		if (resp_max_len < sizeof(*resp))
			return SBI_ENOMEM;
		if (msg_len < sizeof(*req)) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			break;
		}

		target_id = le32_to_cpu(req->target_id);
		if (target_id != RPMI_TEE_ENDPOINT_OPTEE)
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		else if (!tee_signal_bus.active ||
			 tee_signal_bus.target_id != target_id)
			resp->status = cpu_to_le32(RPMI_ERR_ALREADY);
		else {
			/* Active, not-yet-retrieved signals are lost on teardown. */
			sbi_memset(&tee_signal_bus, 0, sizeof(tee_signal_bus));
			resp->status = cpu_to_le32(RPMI_SUCCESS);
		}
		*resp_len = sizeof(*resp);
		break;
	}

	case RPMI_TEE_SRV_SIGNAL_RAISE: {
		/*
		 * A single logical REE<->OP-TEE bus, reached over one TEE MPXY
		 * channel by both endpoints (MPXY channel access is not
		 * domain-gated, so the trusted domain issues this service on the
		 * same channel the REE uses - mirroring how OP-TEE issues the
		 * standard MEM_PARCEL services directly). The direction is taken
		 * from target_id, and the raised indices are validated against
		 * the range the *sender* owns:
		 *   target == OP-TEE  -> sender is the REE, indices in [N,M),
		 *                        set pending_for_tee.
		 *   target == REE     -> sender is OP-TEE, indices in [0,N),
		 *                        set pending_for_ree (the async-notif
		 *                        doorbell the REE later retrieves).
		 */
		struct rpmi_tee_signal_raise_req *req = msgbuf;
		struct rpmi_tee_signal_raise_resp *resp = respbuf;
		u32 target_id, signal_len, i;
		bool to_tee;
		bool bad = false;

		if (resp_max_len < sizeof(*resp))
			return SBI_ENOMEM;
		if (msg_len < sizeof(*req)) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			break;
		}

		target_id = le32_to_cpu(req->target_id);
		signal_len = le32_to_cpu(req->signal_len);

		if (target_id == RPMI_TEE_ENDPOINT_OPTEE)
			to_tee = true;
		else if (target_id == RPMI_TEE_ENDPOINT_REE)
			to_tee = false;
		else {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			break;
		}

		if (!tee_signal_bus.active || !signal_len ||
		    signal_len > tee_signal_bus.width ||
		    msg_len < sizeof(*req) + signal_len * sizeof(u32)) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			break;
		}

		for (i = 0; i < signal_len; i++) {
			u32 sig = le32_to_cpu(req->signal[i]);

			if (to_tee ? !sig_in_ree_raise_range(sig)
				   : !sig_in_tee_raise_range(sig)) {
				bad = true;
				break;
			}
		}
		if (bad) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			*resp_len = sizeof(*resp);
			break;
		}

		for (i = 0; i < signal_len; i++) {
			u32 sig = le32_to_cpu(req->signal[i]);

			if (to_tee)
				tee_signal_bus.pending_for_tee |= (1U << sig);
			else
				tee_signal_bus.pending_for_ree |= (1U << sig);
		}

		/*
		 * Doorbell the REE: signals it can read are now pending.
		 * Delivered as a System MSI at TEE_SIGNAL_MSI_INDEX (the
		 * index reported in the SIGNAL_BUS feature word). The REE
		 * still drains the actual pending bits via SIGNAL_RETRIEVE;
		 * the MSI carries no signal values. No-op until the REE has
		 * programmed the MSI target (SET_MSI_TARGET/SET_MSI_STATE),
		 * so firing unconditionally on this path is safe.
		 */
		if (!to_tee)
			rpmi_sysmsi_send(TEE_SIGNAL_MSI_INDEX);

		resp->status = cpu_to_le32(RPMI_SUCCESS);
		*resp_len = sizeof(*resp);
		break;
	}

	case RPMI_TEE_SRV_SIGNAL_RETRIEVE: {
		/*
		 * REE retrieves the signals readable by it: those OP-TEE raised
		 * (pending_for_ree). Clears them (spec 4.16.10). Single bus, so
		 * MORE_AVAILABLE is always 0.
		 */
		struct rpmi_tee_signal_retrieve_resp *resp = respbuf;
		u32 pending, n = 0, i;

		if (resp_max_len < sizeof(*resp))
			return SBI_ENOMEM;

		pending = tee_signal_bus.active ?
			  tee_signal_bus.pending_for_ree : 0;
		if (!pending) {
			resp->status = cpu_to_le32(RPMI_ERR_NO_DATA);
			*resp_len = sizeof(*resp);
			break;
		}

		for (i = 0; i < TEE_SIGNAL_BUS_MAX_WIDTH; i++) {
			if (!(pending & (1U << i)))
				continue;
			if (resp_max_len < sizeof(*resp) + (n + 1) * sizeof(u32))
				break;
			resp->signal[n++] = cpu_to_le32(i);
		}

		/* Clear only the signals we actually reported. */
		for (i = 0; i < n; i++)
			tee_signal_bus.pending_for_ree &=
				~(1U << le32_to_cpu(resp->signal[i]));

		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->flags = cpu_to_le32(0);
		resp->target_id = cpu_to_le32(tee_signal_bus.target_id);
		resp->signal_len = cpu_to_le32(n);
		*resp_len = sizeof(*resp) + n * sizeof(u32);
		break;
	}

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

	/* Initialize the shared memory parcel pool (idempotent). */
	rpmi_tee_parcel_init();

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
