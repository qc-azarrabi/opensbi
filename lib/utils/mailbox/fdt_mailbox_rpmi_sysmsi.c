/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * In-framework RPMI SYSTEM_MSI mailbox controller.
 *
 * The RPMI SYSTEM_MSI MPXY driver (lib/utils/mpxy/fdt_mpxy_rpmi_sysmsi.c) is a
 * forwarding proxy that relays the SYSTEM_MSI services to the mailbox
 * controller named by its "mboxes" property. On real hardware that mailbox is
 * the shared-memory transport (riscv,rpmi-shmem-mbox) to an external platform
 * microcontroller which owns the System MSIs. Under QEMU virt there is no such
 * microcontroller, so this controller answers the SYSTEM_MSI services directly
 * in the framework: it keeps per-index MSI target+state and, on request from
 * the TEE signal bus, emits the System MSI with a raw write to the configured
 * target (an S-mode IMSIC interrupt file).
 *
 * Only spec-defined RPMI SYSTEM_MSI (group 0x0002) services are answered here;
 * no non-spec behavior is added. The MPXY proxy above is reused unchanged - the
 * only change is which mailbox controller sits behind it.
 */

#include <libfdt.h>
#include <sbi/sbi_byteorder.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_string.h>
#include <sbi/riscv_io.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/mailbox/fdt_mailbox.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>
#include <sbi_utils/mailbox/fdt_mailbox_rpmi_sysmsi.h>

/*
 * Number of System MSIs this controller exposes. Index 0 is the TEE signal-bus
 * "signals available" doorbell (matches TEE_SIGNAL_MSI_INDEX on the TEE side).
 */
#define SYSMSI_NUM_MSI			1

/*
 * Maximum RPMI message data length reported to the MPXY client and used to
 * bound transfers. The SYSTEM_MSI messages are tiny (<= 4 words); this is a
 * comfortable ceiling.
 */
#define SYSMSI_MBOX_MAX_XFER_LEN	0x200

/* Reported service group / protocol versions. */
#define SYSMSI_SRVGRP_VERSION		1
#define SYSMSI_PROTOCOL_VERSION		1

/*
 * P2A doorbell index reported to the proxy. The proxy marks this index (and any
 * index flagged PREF_PRIV) as "denied" so the REE cannot reprogram it. Here the
 * framework itself emits the MSI to the target the REE programs, so no index is
 * reserved: report an out-of-range value so nothing is denied and the REE can
 * program index 0 (the TEE signal doorbell).
 */
#define SYSMSI_NO_P2A_DOORBELL		SYSMSI_NUM_MSI

/** Per-index System MSI state programmed by SET_MSI_TARGET / SET_MSI_STATE. */
struct sysmsi_msi_state {
	bool enabled;
	u32 addr_lo;
	u32 addr_hi;
	u32 data;
};

/** In-framework SYSTEM_MSI mailbox controller. */
struct rpmi_sysmsi_mbox_controller {
	struct mbox_controller controller;
	u32 sys_num_msi;
	struct sysmsi_msi_state msi[SYSMSI_NUM_MSI];
};

/*
 * Single controller instance. The System MSIs are process-global platform
 * resources (not per-hart), so a lone controller backs them and the emit
 * helper reaches it through this pointer.
 */
static struct rpmi_sysmsi_mbox_controller *sysmsi_singleton;

int rpmi_sysmsi_send(u32 index)
{
	struct rpmi_sysmsi_mbox_controller *mc = sysmsi_singleton;
	struct sysmsi_msi_state *m;

	if (!mc || index >= mc->sys_num_msi)
		return SBI_EINVAL;

	m = &mc->msi[index];
	if (!m->enabled || (!m->addr_lo && !m->addr_hi))
		return SBI_EINVALID_STATE;

	/*
	 * Emit the MSI: write the message data to the configured target
	 * address (an IMSIC S-file SETEIPNUM register). Same primitive the
	 * IMSIC driver uses for IPIs (lib/utils/irqchip/imsic.c).
	 */
	writel(m->data, (void *)(((u64)m->addr_hi << 32) | m->addr_lo));
	return SBI_OK;
}

static int rpmi_sysmsi_mbox_xfer(struct mbox_chan *chan, struct mbox_xfer *xfer)
{
	struct rpmi_sysmsi_mbox_controller *mc =
		container_of(chan->mbox, struct rpmi_sysmsi_mbox_controller,
			     controller);
	struct rpmi_message_args *args = xfer->args;
	u32 *tx = xfer->tx;
	u32 index;

	/* Posted requests (no response buffer) have nothing to answer. */
	if (!xfer->rx || args->type != RPMI_MSG_NORMAL_REQUEST)
		return 0;

	switch (args->service_id) {
	case RPMI_SYSMSI_SRV_GET_ATTRIBUTES: {
		struct rpmi_sysmsi_get_attributes_resp *resp = xfer->rx;

		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->sys_num_msi = cpu_to_le32(mc->sys_num_msi);
		resp->flag0 = 0;
		resp->flag1 = 0;
		args->rx_data_len = sizeof(*resp);
		break;
	}

	case RPMI_SYSMSI_SRV_GET_MSI_ATTRIBUTES: {
		struct rpmi_sysmsi_get_msi_attributes_resp *resp = xfer->rx;
		static const char msi_name[] = "tee-signal";

		index = le32_to_cpu(tx[0]);
		if (index >= mc->sys_num_msi) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			args->rx_data_len = sizeof(u32);
			break;
		}
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->flag0 = 0;
		resp->flag1 = 0;
		sbi_memset(resp->name, 0, sizeof(resp->name));
		sbi_memcpy(resp->name, msi_name, sizeof(msi_name) - 1);
		args->rx_data_len = sizeof(*resp);
		break;
	}

	case RPMI_SYSMSI_SRV_SET_MSI_STATE: {
		struct rpmi_sysmsi_set_msi_state_resp *resp = xfer->rx;
		u32 state;

		index = le32_to_cpu(tx[0]);
		if (index >= mc->sys_num_msi) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			args->rx_data_len = sizeof(u32);
			break;
		}
		state = le32_to_cpu(tx[1]);
		mc->msi[index].enabled =
			!!(state & RPMI_SYSMSI_MSI_STATE_ENABLE);
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		args->rx_data_len = sizeof(*resp);
		break;
	}

	case RPMI_SYSMSI_SRV_GET_MSI_STATE: {
		struct rpmi_sysmsi_get_msi_state_resp *resp = xfer->rx;

		index = le32_to_cpu(tx[0]);
		if (index >= mc->sys_num_msi) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			args->rx_data_len = sizeof(u32);
			break;
		}
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->sys_msi_state = cpu_to_le32(mc->msi[index].enabled ?
						  RPMI_SYSMSI_MSI_STATE_ENABLE : 0);
		args->rx_data_len = sizeof(*resp);
		break;
	}

	case RPMI_SYSMSI_SRV_SET_MSI_TARGET: {
		struct rpmi_sysmsi_set_msi_target_req *req = xfer->tx;
		struct rpmi_sysmsi_set_msi_target_resp *resp = xfer->rx;
		u64 addr;

		index = le32_to_cpu(req->sys_msi_index);
		if (index >= mc->sys_num_msi) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			args->rx_data_len = sizeof(u32);
			break;
		}
		addr = le32_to_cpu(req->sys_msi_address_low);
		addr |= ((u64)le32_to_cpu(req->sys_msi_address_high)) << 32;
		/*
		 * The proxy already range-checks this on the forwarding path;
		 * re-validate so the controller is self-contained.
		 */
		if (!sbi_domain_check_addr_range(sbi_domain_thishart_ptr(),
						 addr, 0x4, PRV_S,
						 SBI_DOMAIN_READ | SBI_DOMAIN_WRITE |
						 SBI_DOMAIN_MMIO)) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_ADDR);
			args->rx_data_len = sizeof(u32);
			break;
		}
		mc->msi[index].addr_lo = le32_to_cpu(req->sys_msi_address_low);
		mc->msi[index].addr_hi = le32_to_cpu(req->sys_msi_address_high);
		mc->msi[index].data = le32_to_cpu(req->sys_msi_data);
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		args->rx_data_len = sizeof(*resp);
		break;
	}

	case RPMI_SYSMSI_SRV_GET_MSI_TARGET: {
		struct rpmi_sysmsi_get_msi_target_resp *resp = xfer->rx;

		index = le32_to_cpu(tx[0]);
		if (index >= mc->sys_num_msi) {
			resp->status = cpu_to_le32(RPMI_ERR_INVALID_PARAM);
			args->rx_data_len = sizeof(u32);
			break;
		}
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->sys_msi_address_low = cpu_to_le32(mc->msi[index].addr_lo);
		resp->sys_msi_address_high = cpu_to_le32(mc->msi[index].addr_hi);
		resp->sys_msi_data = cpu_to_le32(mc->msi[index].data);
		args->rx_data_len = sizeof(*resp);
		break;
	}

	default:
		((u32 *)xfer->rx)[0] = cpu_to_le32(RPMI_ERR_NOTSUPP);
		args->rx_data_len = sizeof(u32);
		break;
	}

	return 0;
}

static int rpmi_sysmsi_mbox_get_attribute(struct mbox_chan *chan, int attr_id,
					  void *out_value)
{
	switch (attr_id) {
	case RPMI_CHANNEL_ATTR_PROTOCOL_VERSION:
		*((u32 *)out_value) = SYSMSI_PROTOCOL_VERSION;
		break;
	case RPMI_CHANNEL_ATTR_MAX_DATA_LEN:
		*((u32 *)out_value) = SYSMSI_MBOX_MAX_XFER_LEN;
		break;
	case RPMI_CHANNEL_ATTR_P2A_DOORBELL_SYSMSI_INDEX:
		*((u32 *)out_value) = SYSMSI_NO_P2A_DOORBELL;
		break;
	case RPMI_CHANNEL_ATTR_TX_TIMEOUT:
		*((u32 *)out_value) = RPMI_DEF_TX_TIMEOUT;
		break;
	case RPMI_CHANNEL_ATTR_RX_TIMEOUT:
		*((u32 *)out_value) = RPMI_DEF_RX_TIMEOUT;
		break;
	case RPMI_CHANNEL_ATTR_SERVICEGROUP_ID:
		*((u32 *)out_value) = RPMI_SRVGRP_SYSTEM_MSI;
		break;
	case RPMI_CHANNEL_ATTR_SERVICEGROUP_VERSION:
		*((u32 *)out_value) = SYSMSI_SRVGRP_VERSION;
		break;
	case RPMI_CHANNEL_ATTR_IMPL_ID:
		*((u32 *)out_value) = 0;
		break;
	case RPMI_CHANNEL_ATTR_IMPL_VERSION:
		*((u32 *)out_value) = 0;
		break;
	default:
		return SBI_ENOTSUPP;
	}

	return 0;
}

static struct mbox_chan *rpmi_sysmsi_mbox_request_chan(
					struct mbox_controller *mbox,
					u32 *chan_args)
{
	struct mbox_chan *chan;

	/* This controller only serves the SYSTEM_MSI service group. */
	if (chan_args[0] != RPMI_SRVGRP_SYSTEM_MSI)
		return NULL;

	chan = sbi_zalloc(sizeof(*chan));
	if (!chan)
		return NULL;

	return chan;
}

static void rpmi_sysmsi_mbox_free_chan(struct mbox_controller *mbox,
				       struct mbox_chan *chan)
{
	sbi_free(chan);
}

extern struct fdt_mailbox fdt_mailbox_rpmi_sysmsi;

static int rpmi_sysmsi_mbox_init(const void *fdt, int nodeoff,
				 const struct fdt_match *match)
{
	struct rpmi_sysmsi_mbox_controller *mc;
	int rc;

	/* Only one controller instance is meaningful. */
	if (sysmsi_singleton)
		return SBI_EALREADY;

	mc = sbi_zalloc(sizeof(*mc));
	if (!mc)
		return SBI_ENOMEM;

	mc->sys_num_msi = SYSMSI_NUM_MSI;

	mc->controller.id = nodeoff;
	mc->controller.max_xfer_len = SYSMSI_MBOX_MAX_XFER_LEN;
	mc->controller.driver = &fdt_mailbox_rpmi_sysmsi;
	mc->controller.request_chan = rpmi_sysmsi_mbox_request_chan;
	mc->controller.free_chan = rpmi_sysmsi_mbox_free_chan;
	mc->controller.xfer = rpmi_sysmsi_mbox_xfer;
	mc->controller.get_attribute = rpmi_sysmsi_mbox_get_attribute;

	rc = mbox_controller_add(&mc->controller);
	if (rc) {
		sbi_free(mc);
		return rc;
	}

	sysmsi_singleton = mc;
	return 0;
}

static const struct fdt_match rpmi_sysmsi_mbox_match[] = {
	{ .compatible = "riscv,rpmi-sysmsi-mbox" },
	{ },
};

struct fdt_mailbox fdt_mailbox_rpmi_sysmsi = {
	.driver = {
		.match_table = rpmi_sysmsi_mbox_match,
		.init = rpmi_sysmsi_mbox_init,
	},
	.xlate = fdt_mailbox_simple_xlate,
};
