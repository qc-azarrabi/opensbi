/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 SiFive Inc.
 *
 * OP-TEE Dispatcher Implementation
 */

#include <libfdt.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_mbox.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi_reqfwd.h>
#include <sbi_utils/tee/tee_dispatcher.h>

/* OP-TEE specific context */
struct optee_context {
	/* Pointer to OP-TEE domain */
	struct sbi_domain *domain;
};

static int optee_domain_setup(const void *fdt, int nodeoff,
			      struct optee_context *ctx)
{
	struct sbi_domain *dom = NULL;
	const u32 *prop_instance;
	int len, offset;
	const char *domain_name;

	prop_instance = fdt_getprop(fdt, nodeoff, "opensbi-domain-instance",
				    &len);
	if (!prop_instance || len < 4)
		return SBI_EINVAL;

	offset = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*prop_instance));
	if (offset < 0)
		return SBI_EINVAL;

	domain_name = fdt_get_name(fdt, offset, NULL);
	if (!domain_name)
		return SBI_EINVAL;

	/* Find and bind the domain */
	sbi_domain_for_each(dom) {
		if (!sbi_strcmp(dom->name, domain_name)) {
			ctx->domain = dom;
			return SBI_OK;
		}
	}

	/*
	 * Domain not found - this is a configuration error.
	 * Either the domain name in DT is wrong, or sbi_mpxy_init() was
	 * called before sbi_domain_finalize() (which violates the expected
	 * initialization order in sbi_init.c).
	 */
	return SBI_ENOENT;
}

/**
 * Get OP-TEE attributes
 *
 * OP-TEE uses SMC-style communication with 8 input registers (a0-a7)
 * and 4 output registers (a0-a3).
 */
static int optee_get_attributes(const struct tee_dispatcher *dispatcher,
				struct tee_attributes *attr)
{
	if (!attr)
		return SBI_EINVAL;

	/* TEE implementation ID */
	attr->tee_impl_id = RPMI_TEE_IMPL_ID_OPTEE;

	/* OP-TEE uses 8 registers for request (a0-a7) */
	attr->comm_req_regs = RPMI_TEE_OPTEE_COMM_REQ_REGS;

	/* OP-TEE uses 4 registers for response (a0-a3) */
	attr->comm_resp_regs = RPMI_TEE_OPTEE_COMM_RESP_REGS;

	return SBI_OK;
}

/**
 * OP-TEE response transformation callback
 *
 * OP-TEE returns 5 unsigned longs (a0-a4) where:
 *
 *   a0 = TEEABI_OPTEED_RETURN_* code (internal signal to dispatcher)
 *        This register contains an internal OP-TEE ABI code that signals
 *        what type of event completed. On RISC-V, only these are used:
 *
 *          - TEEABI_OPTEED_RETURN_ENTRY_DONE  (0xBE000001)
 *            OP-TEE primary hart initialization complete.
 *
 *          - TEEABI_OPTEED_RETURN_ON_DONE     (0xBE000002)
 *            OP-TEE secondary hart boot complete.
 *
 *          - TEEABI_OPTEED_RETURN_CALL_DONE   (0xBE000005)
 *            A standard SMC call to OP-TEE has completed.
 *            This is the most common code for normal TEE operations.
 *
 *          - TEEABI_OPTEED_RETURN_FIQ_DONE    (0xBE000006)
 *            OP-TEE has finished handling a forwarded FIQ.
 *
 *        Note: OFF_DONE, SUSPEND_DONE, RESUME_DONE, SYSTEM_OFF_DONE,
 *        SYSTEM_RESET_DONE are defined but NOT used on RISC-V because
 *        power management is handled directly by OpenSBI SBI extensions
 *        (HSM, SRST), not through OP-TEE.
 *
 *        These codes are internal signals between OP-TEE and OpenSBI.
 *        They are NOT part of the SMC calling convention that Linux uses.
 *
 *   a1 = SMC return value (e.g., OPTEE_SMC_RETURN_OK)
 *        This becomes the caller's a0.
 *
 *   a2-a4 = Additional return values
 *        These become the caller's a1-a3.
 *
 * This callback ALWAYS strips a0, regardless of which TEEABI_OPTEED_RETURN_*
 * code it contains, because all return codes are internal dispatcher signals.
 * Linux expects SMC results in a0-a3 per the SMC Calling Convention.
 */
static int optee_transform_response(void *tx, u32 tx_len,
				    void *rx, u32 rx_max_len,
				    unsigned long *rx_len)
{
	u32 copy_len;

	/* Must have at least one register (a0) to skip */
	if (tx_len < sizeof(ulong))
		return SBI_EINVAL;

	/* Calculate length after skipping a0 */
	copy_len = tx_len - sizeof(ulong);

	/* Check destination buffer size */
	if (copy_len > rx_max_len)
		return SBI_ENOMEM;

	/* Copy a1-a4 to rx, skipping a0 (TEEABI_OPTEED_RETURN_*) */
	sbi_memcpy(rx, &(((ulong *)tx)[1]), copy_len);
	*rx_len = copy_len;

	return SBI_OK;
}

/**
 * OP-TEE communicate - forward message and enter domain
 *
 * OP-TEE uses SMC-style parameters (a0-a7) for communication.
 * The request data contains 8 unsigned long values representing
 * the SMC parameters, and the response contains 4 return values.
 *
 * The per-hart reqfwd channel is looked up at runtime using
 * current_hartid(). The device tree binding guarantees that
 * reqfwd nodes are siblings of TEE nodes under the same CPU.
 *
 * @param dispatcher: TEE dispatcher instance
 * @param tx_data: Request data buffer
 * @param tx_len: Size of request data
 * @param rx_data: Response data buffer
 * @param rx_max_len: Maximum size of response buffer
 * @param rx_len: Actual size of response data written
 * @return 0 on success, negative error code on failure
 */
static int optee_communicate(const struct tee_dispatcher *dispatcher,
			     void *tx_data, u32 tx_len,
			     void *rx_data, u32 rx_max_len,
			     unsigned long *rx_len)
{
	struct optee_context *ctx = dispatcher->context;
	struct sbi_mpxy_channel *recv_channel;
	struct rpmi_message_header header;
	int rc;

	if (!ctx)
		return SBI_EINVAL;

	/* Find per-hart request forward channel by current hartid */
	recv_channel = mpxy_reqfwd_find_channel_by_hartid(current_hartid());
	if (!recv_channel)
		return SBI_ENODEV;

	/* Prepare the header for forwarding to OP-TEE domain */
	header.servicegroup_id = cpu_to_le16(RPMI_SRVGRP_TEE);
	header.service_id = RPMI_TEE_SRV_COMMUNICATE;
	header.flags = RPMI_MSG_NORMAL_REQUEST;
	header.datalen = cpu_to_le16(tx_len);
	header.token = cpu_to_le16(0);

	/*
	 * Forward message to request forward channel.
	 * Pass OP-TEE specific response transformation callback to skip a0
	 * (internal TEEABI return code) when copying response to caller.
	 */
	rc = mpxy_reqfwd_forward_message(recv_channel, &header,
					 tx_data, tx_len,
					 rx_data, rx_max_len,
					 rx_len,
					 optee_transform_response);

	return rc;
}

/**
 * Enter OP-TEE domain
 *
 * This function switches execution context to the OP-TEE domain.
 * It blocks until OP-TEE processing completes and domain_exit is called.
 */
static int optee_domain_enter(const struct tee_dispatcher *dispatcher)
{
	struct optee_context *ctx = dispatcher->context;

	if (!ctx)
		return SBI_EINVAL;

	if (!ctx->domain)
		return SBI_ENOENT;

	return sbi_domain_context_enter(ctx->domain);
}

/**
 * Exit OP-TEE domain
 */
static int optee_domain_exit(const struct tee_dispatcher *dispatcher)
{
	struct optee_context *ctx = dispatcher->context;

	if (!ctx)
		return SBI_EINVAL;

	if (!ctx->domain)
		return SBI_ENOENT;

	return sbi_domain_context_exit();
}

/** OP-TEE dispatcher operations */
static const struct tee_dispatcher_ops optee_ops = {
	.get_attributes = optee_get_attributes,
	.communicate = optee_communicate,
	.domain_enter = optee_domain_enter,
	.domain_exit = optee_domain_exit,
};

/**
 * Setup OP-TEE dispatcher from device tree
 *
 * This function creates an OP-TEE dispatcher context with domain reference.
 * Per-hart reqfwd channels are looked up at runtime by hartid using
 * mpxy_reqfwd_find_channel_by_hartid(), so no channel setup is needed here.
 * The device tree binding guarantees that reqfwd nodes are siblings of TEE
 * nodes under the same CPU.
 *
 * Setup steps:
 *   1. Allocate OP-TEE context
 *   2. Setup OP-TEE domain reference
 *   3. Initialize dispatcher structure
 */
int optee_dispatcher_setup(const void *fdt, int nodeoff,
			   struct tee_dispatcher *dispatcher)
{
	struct optee_context *ctx;
	int rc;

	/* Allocate OP-TEE context */
	ctx = sbi_zalloc(sizeof(*ctx));
	if (!ctx)
		return SBI_ENOMEM;

	/* Setup OP-TEE domain */
	rc = optee_domain_setup(fdt, nodeoff, ctx);
	if (rc) {
		sbi_free(ctx);
		return rc;
	}

	/* Setup dispatcher */
	dispatcher->impl_id = RPMI_TEE_IMPL_ID_OPTEE;
	dispatcher->name = "OP-TEE";
	dispatcher->ops = &optee_ops;
	dispatcher->context = ctx;

	return SBI_OK;
}
