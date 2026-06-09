/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Andes Technology Corporation. All rights reserved.
 */

#include <libfdt.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_fifo.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_mpxy.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_mbox.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_reqfwd.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_tee.h>
#include <sbi_utils/fdt/fdt_helper.h>

/** RPMI Message */
struct rpmi_message_slot {
	struct rpmi_message_header header;
	u8 data[RPMI_MSG_DATA_SIZE(RPMI_REQFWD_FIFO_SLOT_SIZE)];

	/* Sender RX address. Should be MPXY shared memory */
	void *sender_rx;
	/* Maximum sender RX length */
	u32 sender_rx_max_len;
	/* Optional response transformation callback (TEE-specific) */
	mpxy_reqfwd_transform_fn transform_fn;
};

#define REQFWD_MSG_FIFO_ENTRIES		4

/**
 * MPXY ReqFwd instance per MPXY channel.
 */
struct mpxy_reqfwd {
	struct mpxy_rpmi_channel_attrs msgprot_attrs;
	struct sbi_mpxy_channel channel;
	/* Owner Hart ID of this channel */
	u32 hartid;
	/* FIFO to store forwarded RPMI request message */
	struct sbi_fifo msg_fifo;
	/* Current forwarded RPMI request message */
	struct rpmi_message_slot current_msg;
	/* Flag indicating if current_msg contains valid data being retrieved */
	bool has_current_msg;

	/* Receiver side is waiting for message */
	bool is_waiting_message;
	/* Receiver RX address. Should be MPXY shared memory */
	void *rx;
	/* Maximum receiver RX length */
	u32 rx_max_len;
	/* Length of response of current message */
	unsigned long ack_len;
	/* List node for reqfwd instance tracking */
	struct sbi_dlist node;
};

static int retrieve_message(struct mpxy_reqfwd *reqfwd,
			    void *tx, u32 tx_len,
			    void *rx, u32 rx_max_len, unsigned long *ack_len)
{
	struct rpmi_message_slot *current_msg = &reqfwd->current_msg;
	struct rpmi_reqfwd_retrieve_current_message_req *req = tx;
	struct rpmi_reqfwd_retrieve_current_message_resp *resp = rx;
	u32 start_index, datalen, available_space, chunk_size, remaining;
	int rc;

	if (tx_len < sizeof(*req)) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*ack_len = sizeof(resp->status);
		return SBI_OK;
	}

	start_index = le32_to_cpu(req->start_index);

	/*
	 * Chunked Retrieval Logic:
	 * - START_INDEX=0: Dequeue new message (first call or new message)
	 * - START_INDEX>0: Continue retrieving current message
	 */
	if (start_index == 0) {
		/*
		 * First call or new message request.
		 * Dequeue oldest forwarded RPMI request message from FIFO.
		 */
		rc = sbi_fifo_dequeue(&reqfwd->msg_fifo, current_msg);
		if (rc)
			return rc;

		/* Mark that we now have a valid current message */
		reqfwd->has_current_msg = true;
		reqfwd->is_waiting_message = false;
	} else {
		if (!reqfwd->has_current_msg) {
			resp->status = cpu_to_le32(RPMI_ERR_NO_DATA);
			*ack_len = sizeof(resp->status);
			return SBI_OK;
		}
	}

	datalen = le16_to_cpu(current_msg->header.datalen);

	/* Validate START_INDEX within message bounds */
	if (start_index >= datalen) {
		resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
		*ack_len = sizeof(resp->status);
		return SBI_OK;
	}

	/* Calculate chunk size based on available buffer space */
	available_space = RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) - offsetof(struct rpmi_reqfwd_retrieve_current_message_resp, request_message);
	chunk_size = datalen - start_index;
	if (chunk_size > available_space)
		chunk_size = available_space;

	remaining = datalen - start_index - chunk_size;

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->remaining = cpu_to_le32(remaining);
	resp->returned = cpu_to_le32(chunk_size);
	sbi_memcpy(resp->request_message, &current_msg->data[start_index], chunk_size);

	*ack_len = offsetof(struct rpmi_reqfwd_retrieve_current_message_resp,
			    request_message) + chunk_size;

	/* Clear flag when message fully retrieved */
	if (remaining == 0)
		reqfwd->has_current_msg = false;

	return SBI_OK;
}

/** List to track all registered reqfwd instances for hartid lookup */
static SBI_LIST_HEAD(mpxy_reqfwd_list);

/**
 * Find request forward channel by hartid
 *
 * Each reqfwd channel is associated with a specific hart. This function
 * searches the registered reqfwd channels and returns the MPXY channel
 * for the specified hartid.
 *
 * @param hartid: The hart ID to search for
 * @return Pointer to the MPXY channel, or NULL if not found
 */
struct sbi_mpxy_channel *mpxy_reqfwd_find_channel_by_hartid(u32 hartid)
{
	struct mpxy_reqfwd *reqfwd;

	sbi_list_for_each_entry(reqfwd, &mpxy_reqfwd_list, node)
		if (reqfwd->hartid == hartid)
			return &reqfwd->channel;

	return NULL;
}

int mpxy_reqfwd_forward_message(struct sbi_mpxy_channel *channel,
				struct rpmi_message_header *header,
				void *tx, u32 tx_len,
				void *rx, u32 rx_max_len,
				unsigned long *ack_len,
				mpxy_reqfwd_transform_fn transform_fn)
{
	struct mpxy_reqfwd *reqfwd;
	struct rpmi_message_slot msg;

	if (!tx || tx_len > RPMI_MSG_DATA_SIZE(RPMI_REQFWD_FIFO_SLOT_SIZE))
		return SBI_EINVAL;

	reqfwd = container_of(channel, struct mpxy_reqfwd, channel);

	/* Prepare and enqueue message into per-channel FIFO */
	sbi_memset(&msg, 0, sizeof(struct rpmi_message_slot));
	sbi_memcpy(&msg.header, header, RPMI_MSG_HDR_SIZE);
	sbi_memcpy(msg.data, tx, tx_len);
	/* Record RX information so that we can copy response into it later */
	msg.sender_rx = rx;
	msg.sender_rx_max_len = rx_max_len;
	/* Store optional transformation callback for response processing */
	msg.transform_fn = transform_fn;
	sbi_fifo_enqueue(&reqfwd->msg_fifo, &msg, true);

	if (reqfwd->is_waiting_message) {
		int rc;
		u32 start_index = 0;
		/*
		 * The callee domain is waiting for message.
		 * We immediately retrieve message and switch into it.
		 */
		rc = retrieve_message(reqfwd, &start_index, sizeof(start_index),
				      reqfwd->rx, reqfwd->rx_max_len,
				      ack_len);
		if (rc)
			return rc;
	}

	return SBI_OK;
}

/** Copy attributes word size */
static void mpxy_copy_attrs(u32 *outmem, u32 *inmem, u32 count)
{
	u32 idx;
	for (idx = 0; idx < count; idx++)
		outmem[idx] = cpu_to_le32(inmem[idx]);
}

static int mpxy_reqfwd_read_attributes(struct sbi_mpxy_channel *channel,
				       u32 *outmem, u32 base_attr_id,
				       u32 attr_count)
{
	struct mpxy_reqfwd *reqfwd =
		container_of(channel, struct mpxy_reqfwd, channel);
	u32 *attr_array = (u32 *)&reqfwd->msgprot_attrs;
	u32 end_id = base_attr_id + attr_count - 1;

	if (end_id >= MPXY_MSGPROT_RPMI_ATTR_MAX_ID)
		return SBI_EBAD_RANGE;

	mpxy_copy_attrs(outmem, &attr_array[attr_id2index(base_attr_id)],
			attr_count);

	return SBI_OK;
}

static int mpxy_reqfwd_send_message_withresp(struct sbi_mpxy_channel *channel,
					     u32 message_id,
					     void *tx, u32 tx_len,
					     void *rx, u32 rx_max_len,
					     unsigned long *ack_len)
{
	struct mpxy_reqfwd *reqfwd =
		container_of(channel, struct mpxy_reqfwd, channel);
	struct rpmi_message_slot *current_msg = &reqfwd->current_msg;
	struct sbi_mpxy_channel *tee_channel;
	struct tee_dispatcher *dispatcher;
	int rc;

	if (RPMI_REQFWD_SRV_RETRIEVE_CURRENT_MESSAGE == message_id) {
		rc = retrieve_message(reqfwd, tx, tx_len, rx, rx_max_len, ack_len);
		if (rc == SBI_OK || rc == SBI_EINVAL)
			return rc;

		/* No message available */
		if (rc == SBI_ENOENT) {
			struct rpmi_reqfwd_retrieve_current_message_resp *resp = rx;

			/* Try domain_exit if TEE channel available */
			tee_channel = mpxy_tee_find_channel_by_hartid(reqfwd->hartid);
			if (tee_channel) {
				dispatcher = mpxy_tee_get_dispatcher(tee_channel);
				if (dispatcher && dispatcher->ops &&
				    dispatcher->ops->domain_exit) {
					*ack_len = reqfwd->ack_len;
					reqfwd->ack_len = 0;
					/* TEE use case: save state before domain_exit */
					reqfwd->is_waiting_message = true;
					reqfwd->rx = rx;
					reqfwd->rx_max_len = rx_max_len;

					dispatcher->ops->domain_exit(dispatcher);
					return SBI_OK;
				}
			}

			/* Non-TEE use case */
			resp->status = cpu_to_le32(RPMI_ERR_NO_DATA);
			resp->remaining = cpu_to_le32(0);
			resp->returned = cpu_to_le32(0);
			*ack_len = offsetof(struct rpmi_reqfwd_retrieve_current_message_resp,
					    request_message);
			return SBI_OK;
		}

		return rc;
	} else if (RPMI_REQFWD_SRV_COMPLETE_CURRENT_MESSAGE == message_id) {
		struct rpmi_reqfwd_complete_current_message_req *req = tx;
		struct rpmi_reqfwd_complete_current_message_resp *resp = rx;
		u32 num_messages;

		if (current_msg->header.servicegroup_id) {
			/* Apply transformation callback if provided */
			if (current_msg->transform_fn) {
				rc = current_msg->transform_fn(
					req->response_data, tx_len,
					current_msg->sender_rx,
					current_msg->sender_rx_max_len,
					&reqfwd->ack_len);
				if (rc) {
					resp->status = cpu_to_le32(RPMI_ERR_FAILED);
					resp->num_messages = cpu_to_le32(0);
					*ack_len = sizeof(*resp);
					return SBI_OK;
				}
			} else {
				/* No transformation - copy response as-is */
				sbi_memcpy(current_msg->sender_rx, req->response_data, tx_len);
				reqfwd->ack_len = tx_len;
			}
			sbi_memset(current_msg, 0, sizeof(*current_msg));

			num_messages = sbi_fifo_avail(&reqfwd->msg_fifo);

			resp->status = cpu_to_le32(RPMI_SUCCESS);
			resp->num_messages = cpu_to_le32(num_messages);
		} else {
			resp->status = cpu_to_le32(RPMI_ERR_NO_DATA);
			resp->num_messages = cpu_to_le32(0);
		}
		*ack_len = sizeof(*resp);
	} else {
		return SBI_EFAIL;
	}

	return SBI_OK;
}

static int mpxy_reqfwd_init(const void *fdt, int nodeoff,
			    const struct fdt_match *match)
{
	struct rpmi_message_slot *msg_buf;
	struct mpxy_reqfwd *reqfwd;
	const fdt32_t *val;
	u32 channel_id, hartid;
	int rc, len, cpu_offset;

	/* Allocate context for Request Forward */
	reqfwd = sbi_zalloc(sizeof(*reqfwd));
	if (!reqfwd)
		return SBI_ENOMEM;

	val = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (len > 0 && val)
		channel_id = fdt32_to_cpu(*val);
	else
		sbi_panic("Failed to get riscv,sbi-mpxy-channel-id");

	/* Get parent CPU node to extract hartid from its reg property */
	cpu_offset = fdt_parent_offset(fdt, nodeoff);
	if (cpu_offset < 0)
		sbi_panic("Failed to get parent CPU node");

	rc = fdt_parse_hart_id(fdt, cpu_offset, &hartid);
	if (rc)
		sbi_panic("Failed to parse hartid from parent CPU node");

	reqfwd->hartid = hartid;
	reqfwd->channel.channel_id = channel_id;
	reqfwd->channel.read_attributes = mpxy_reqfwd_read_attributes;
	reqfwd->channel.send_message_with_response =
		mpxy_reqfwd_send_message_withresp;
	reqfwd->channel.attrs.msg_data_maxlen = PAGE_SIZE;

	/* RPMI service group attributes */
	reqfwd->msgprot_attrs.servicegrp_id = RPMI_SRVGRP_REQFWD;
	reqfwd->msgprot_attrs.servicegrp_ver = 1;

	/* Allocate per-channel FIFO */
	msg_buf = sbi_calloc(REQFWD_MSG_FIFO_ENTRIES, sizeof(*msg_buf));
	if (!msg_buf) {
		sbi_free(reqfwd);
		return SBI_ENOMEM;
	}

	sbi_fifo_init(&reqfwd->msg_fifo, msg_buf, REQFWD_MSG_FIFO_ENTRIES,
		      sizeof(struct rpmi_message_slot));

	rc = sbi_mpxy_register_channel(&reqfwd->channel);
	if (rc) {
		sbi_free(reqfwd);
		return rc;
	}

	/* Add to reqfwd list for hartid lookup */
	sbi_list_add_tail(&reqfwd->node, &mpxy_reqfwd_list);

	return SBI_OK;
}

static const struct fdt_match reqfwd_match[] = {
	{ .compatible = "riscv,rpmi-mpxy-reqfwd", .data = NULL },
	{},
};

const struct fdt_driver fdt_mpxy_rpmi_reqfwd = {
	.match_table = reqfwd_match,
	.init = mpxy_reqfwd_init,
	.experimental = true,
};
