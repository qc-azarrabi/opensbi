/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 SiFive Inc.
 */

#ifndef __FDT_MPXY_RPMI_REQFWD_H__
#define __FDT_MPXY_RPMI_REQFWD_H__

#include <sbi/sbi_types.h>
#include <sbi/sbi_mpxy.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>

/**
 * Response transformation callback type for reqfwd
 *
 * This callback allows TEE-specific transformation of response data before
 * it is copied to the sender's RX buffer.
 *
 * @param tx: Raw response data from TEE domain
 * @param tx_len: Length of raw response data
 * @param rx: Destination buffer for transformed response
 * @param rx_max_len: Maximum size of destination buffer
 * @param rx_len: Output - actual length written to rx
 * @return 0 on success, negative error code on failure
 */
typedef int (*mpxy_reqfwd_transform_fn)(void *tx, u32 tx_len,
					void *rx, u32 rx_max_len,
					unsigned long *rx_len);

/**
 * Forward message by MPXY RPMI Request Forward service group
 *
 * @param channel: MPXY channel for request forwarding
 * @param header: RPMI message header
 * @param tx: Request data to forward
 * @param tx_len: Length of request data
 * @param rx: Buffer for response data
 * @param rx_max_len: Maximum response buffer size
 * @param ack_len: Output - actual response length
 * @param transform_fn: Optional callback to transform response data (can be NULL)
 * @return 0 on success, negative error code on failure
 */
#ifdef CONFIG_FDT_MPXY_RPMI_REQFWD
int mpxy_reqfwd_forward_message(struct sbi_mpxy_channel *channel,
				struct rpmi_message_header *header,
				void *tx, u32 tx_len,
				void *rx, u32 rx_max_len,
				unsigned long *ack_len,
				mpxy_reqfwd_transform_fn transform_fn);
#else
static inline int mpxy_reqfwd_forward_message(struct sbi_mpxy_channel *channel,
					      struct rpmi_message_header *header,
					      void *tx, u32 tx_len,
					      void *rx, u32 rx_max_len,
					      unsigned long *ack_len,
					      mpxy_reqfwd_transform_fn transform_fn)
{
	return SBI_ENODEV;
}
#endif

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
#ifdef CONFIG_FDT_MPXY_RPMI_REQFWD
struct sbi_mpxy_channel *mpxy_reqfwd_find_channel_by_hartid(u32 hartid);
#else
static inline struct sbi_mpxy_channel *mpxy_reqfwd_find_channel_by_hartid(u32 hartid)
{
	return NULL;
}
#endif

#endif

