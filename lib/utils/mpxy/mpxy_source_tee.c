/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <sbi/riscv_asm.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_hart_pmp.h>
#include <sbi/sbi_scratch.h>
#include <sbi_utils/mpxy/fdt_mpxy.h>
#include <sbi_utils/mpxy/fdt_mpxy_bridge.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

/* Format a packet-relative chunk of the current TEE request. */
static int tee_xfer(struct mpxy_bridge_src_ctx *src,
		    struct mpxy_bridge_dst_ctx *dst)
{
	struct rpmi_request_forward_resp *resp = dst->respbuf;
	u32 header_len = sizeof(src->header);
	u32 total = header_len + src->msg_len;
	u32 offset = dst->start_index;
	u32 returned, copied = 0;

	if (dst->resp_bufsize < sizeof(*resp) || offset >= total)
		return SBI_EINVAL;
	returned = MIN(dst->resp_bufsize - sizeof(*resp), total - offset);
	if (!returned)
		return SBI_EINVAL;

	if (offset < header_len) {
		copied = MIN(returned, header_len - offset);
		sbi_memcpy(resp->data, (u8 *)&src->header + offset, copied);
		offset += copied;
	}
	if (copied < returned)
		sbi_memcpy(resp->data + copied,
			   (u8 *)src->msgbuf + offset - header_len,
			   returned - copied);

	resp->status = cpu_to_le32(RPMI_SUCCESS);
	resp->returned = cpu_to_le32(returned);
	resp->remaining = cpu_to_le32(total - dst->start_index - returned);
	dst->resp_len = sizeof(*resp) + returned;
	return 0;
}

static const struct mpxy_bridge_source_ops tee_ops = {
	.require_waiting_retrieval = true,
	.request = rpmi_tee_request,
	.xfer = tee_xfer,
	.complete = rpmi_tee_complete,
};

/* Reject platforms outside the initial TEE transport profile. */
static int tee_check_platform(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();

	if (__riscv_xlen != 64 || sbi_hart_count() != 1 ||
	    !sbi_hart_pmp_count(scratch) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SMEPMP))
		return SBI_ENOTSUPP;

	if (misa_extension('H') || misa_extension('V') ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SSTC) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SMSTATEEN) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SSSTATEEN) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SMCDELEG) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SSCCFG) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SMCTR) ||
	    sbi_hart_has_extension(scratch, SBI_HART_EXT_SSCTR))
		return SBI_ENOTSUPP;

	return 0;
}

static int tee_source_probe(const void *fdt, int nodeoff,
			    const struct fdt_match *match)
{
	int ret;

	ret = tee_check_platform();
	if (ret)
		return ret;

	ret = rpmi_tee_register_endpoint(fdt, nodeoff);
	if (ret)
		return ret;

	return mpxy_bridge_register_source(fdt, nodeoff, &tee_ops,
		rpmi_tee_local_data.service_data,
		rpmi_tee_local_data.num_services, RPMI_SRVGRP_TEE);
}

static const struct fdt_match tee_matches[] = {
	{ .compatible = "opensbi,rpmi-mpxy-reqfwd-tee" },
	{ }
};

const struct mpxy_bridge_source_driver mpxy_rpmi_tee_driver = {
	.match_table = tee_matches,
	.probe = tee_source_probe,
};
