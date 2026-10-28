/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Firmware-owned MSI sources for the single-hart RPMI TEE framework.
 * The existing RPMI System MSI MPXY driver exposes these to each domain.
 */

#include <libfdt.h>
#include <sbi/riscv_io.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/irqchip/imsic.h>
#include <sbi_utils/mailbox/fdt_mailbox.h>
#include <sbi_utils/mailbox/rpmi_mailbox.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

extern const struct fdt_mailbox fdt_mailbox_rpmi_tee_msi;

#define TEE_MSI_COUNT 2

static struct {
	struct mbox_controller mbox;
	struct mbox_chan channels[TEE_MSI_COUNT];
	struct imsic_data imsic;
	/* Serializes target programming, disabling and MSI writes. */
	spinlock_t lock;
	struct {
		unsigned long address;
		u32 data;
		bool enabled;
	} routes[TEE_MSI_COUNT];
} tee_msi = {
	.lock = __SPIN_LOCK_UNLOCKED,
};

static void tee_msi_notify(u32 endpoint_id)
{
	struct rpmi_tee_endpoint *ep;
	u32 i;

	spin_lock(&tee_msi.lock);
	for (i = 0; i < TEE_MSI_COUNT; i++) {
		ep = rpmi_tee_endpoint_by_role(i != 0);
		if (!ep || ep->id != endpoint_id || !tee_msi.routes[i].enabled)
			continue;
		if (rpmi_tee_signal_pending(endpoint_id))
			writel(cpu_to_le32(tee_msi.routes[i].data),
			       (void *)tee_msi.routes[i].address);
	}
	spin_unlock(&tee_msi.lock);
}

static bool tee_msi_target_valid(u64 address, u32 data)
{
	struct imsic_data *imsic = &tee_msi.imsic;
	struct sbi_domain *dom = sbi_domain_thishart_ptr();
	u32 i;

	if ((unsigned long)address != address ||
	    !data || data > imsic->num_ids ||
	    (address & (IMSIC_MMIO_PAGE_SZ - 1)))
		return false;

	for (i = 0; i < array_size(imsic->regs); i++) {
		const struct imsic_regs *regs = &imsic->regs[i];

		if (!regs->size || address < regs->addr ||
		    address - regs->addr >= regs->size)
			continue;

		return sbi_domain_check_addr_range(dom, address,
			sizeof(u32), PRV_S, SBI_DOMAIN_MMIO |
			SBI_DOMAIN_READ | SBI_DOMAIN_WRITE);
	}

	return false;
}

static u32 tee_msi_tx(struct mbox_xfer *xfer, u32 index)
{
	struct rpmi_message_args *args = xfer->args;
	u32 word = ((u32 *)xfer->tx)[index];

	if (index < args->tx_endian_words)
		return word;
	return le32_to_cpu(word);
}

static int tee_msi_xfer(struct mbox_chan *chan, struct mbox_xfer *xfer)
{
	struct rpmi_message_args *args = xfer->args;
	struct rpmi_tee_endpoint *ep = NULL;
	u32 owner = chan->chan_args[1], index = 0, count = 1, need = 0;
	u32 out[7] = { RPMI_SUCCESS }, i, value;
	u64 address;
	bool notify = false;

	if (!args || args->type != RPMI_MSG_NORMAL_REQUEST || !xfer->rx ||
	    xfer->rx_len < sizeof(u32))
		return SBI_EINVAL;
	switch (args->service_id) {
	case RPMI_SYSMSI_SRV_GET_ATTRIBUTES:
		break;
	case RPMI_SYSMSI_SRV_GET_MSI_ATTRIBUTES:
	case RPMI_SYSMSI_SRV_GET_MSI_STATE:
	case RPMI_SYSMSI_SRV_GET_MSI_TARGET:
		need = sizeof(u32);
		break;
	case RPMI_SYSMSI_SRV_SET_MSI_STATE:
		need = 2 * sizeof(u32);
		break;
	case RPMI_SYSMSI_SRV_SET_MSI_TARGET:
		need = 4 * sizeof(u32);
		break;
	default:
		out[0] = RPMI_ERR_NOTSUPP;
		goto respond;
	}
	if (xfer->tx_len != need || (need && !xfer->tx)) {
		out[0] = RPMI_ERR_INVALID_PARAM;
		goto respond;
	}
	if (need) {
		index = tee_msi_tx(xfer, 0);
		if (index >= TEE_MSI_COUNT) {
			out[0] = RPMI_ERR_INVALID_PARAM;
			goto respond;
		}
	}
	/* Attribute queries also run while the proxy is being initialized. */
	if (args->service_id != RPMI_SYSMSI_SRV_GET_ATTRIBUTES &&
	    args->service_id != RPMI_SYSMSI_SRV_GET_MSI_ATTRIBUTES) {
		ep = rpmi_tee_endpoint_by_domain(sbi_domain_thishart_ptr());
		if (!ep || index != owner || ep->tee != !!owner) {
			out[0] = RPMI_ERR_DENIED;
			goto respond;
		}
	}
	/* Check the complete response size before changing a route. */
	if (args->service_id == RPMI_SYSMSI_SRV_GET_ATTRIBUTES ||
	    args->service_id == RPMI_SYSMSI_SRV_GET_MSI_TARGET)
		count = 4;
	else if (args->service_id == RPMI_SYSMSI_SRV_GET_MSI_ATTRIBUTES)
		count = 7;
	else if (args->service_id == RPMI_SYSMSI_SRV_GET_MSI_STATE)
		count = 2;
	if (xfer->rx_len < count * sizeof(u32))
		return SBI_ENOSPC;
	spin_lock(&tee_msi.lock);
	switch (args->service_id) {
	case RPMI_SYSMSI_SRV_GET_ATTRIBUTES:
		out[1] = TEE_MSI_COUNT;
		break;
	case RPMI_SYSMSI_SRV_GET_MSI_ATTRIBUTES:
		if (index != owner)
			out[1] = RPMI_SYSMSI_MSI_ATTRIBUTES_FLAG0_PREF_PRIV;
		break;
	case RPMI_SYSMSI_SRV_SET_MSI_STATE:
		value = tee_msi_tx(xfer, 1);
		if (value & ~RPMI_SYSMSI_MSI_STATE_ENABLE) {
			out[0] = RPMI_ERR_INVALID_PARAM;
			break;
		}
		if (value && !tee_msi.routes[index].address) {
			out[0] = RPMI_ERR_INVALID_ADDR;
			break;
		}
		tee_msi.routes[index].enabled = value != 0;
		notify = value != 0;
		break;
	case RPMI_SYSMSI_SRV_GET_MSI_STATE:
		out[1] = tee_msi.routes[index].enabled;
		if (rpmi_tee_signal_pending(ep->id))
			out[1] |= RPMI_SYSMSI_MSI_STATE_PENDING;
		break;
	case RPMI_SYSMSI_SRV_SET_MSI_TARGET:
		address = tee_msi_tx(xfer, 1);
		address |= (u64)tee_msi_tx(xfer, 2) << 32;
		value = tee_msi_tx(xfer, 3);
		if (!tee_msi_target_valid(address, value)) {
			out[0] = RPMI_ERR_INVALID_ADDR;
			break;
		}
		tee_msi.routes[index].address = address;
		tee_msi.routes[index].data = value;
		notify = true;
		break;
	case RPMI_SYSMSI_SRV_GET_MSI_TARGET:
		out[1] = tee_msi.routes[index].address;
		out[2] = (u64)tee_msi.routes[index].address >> 32;
		out[3] = tee_msi.routes[index].data;
		break;
	}
	spin_unlock(&tee_msi.lock);
	if (notify)
		tee_msi_notify(ep->id);
respond:
	if ((s32)out[0] != RPMI_SUCCESS)
		count = 1;
	for (i = 0; i < count; i++)
		((u32 *)xfer->rx)[i] = i < args->rx_endian_words ? out[i] :
					cpu_to_le32(out[i]);
	args->rx_data_len = count * sizeof(u32);
	return 0;
}

static struct mbox_chan *tee_msi_request_chan(struct mbox_controller *mbox,
					      u32 *args)
{
	if (args[0] != RPMI_SRVGRP_SYSTEM_MSI || args[1] >= TEE_MSI_COUNT)
		return NULL;
	return &tee_msi.channels[args[1]];
}

static int tee_msi_get_attribute(struct mbox_chan *chan, int id, void *out)
{
	u32 value;

	switch (id) {
	case RPMI_CHANNEL_ATTR_PROTOCOL_VERSION:
		value = RPMI_VERSION(2, 0);
		break;
	case RPMI_CHANNEL_ATTR_SERVICEGROUP_VERSION:
		value = RPMI_VERSION(1, 0);
		break;
	case RPMI_CHANNEL_ATTR_SERVICEGROUP_ID:
		value = RPMI_SRVGRP_SYSTEM_MSI;
		break;
	case RPMI_CHANNEL_ATTR_MAX_DATA_LEN:
		value = 4096;
		break;
	case RPMI_CHANNEL_ATTR_P2A_DOORBELL_SYSMSI_INDEX:
		value = ~0U;
		break;
	case RPMI_CHANNEL_ATTR_TX_TIMEOUT:
		value = RPMI_DEF_TX_TIMEOUT;
		break;
	case RPMI_CHANNEL_ATTR_RX_TIMEOUT:
		value = RPMI_DEF_RX_TIMEOUT;
		break;
	case RPMI_CHANNEL_ATTR_IMPL_ID:
	case RPMI_CHANNEL_ATTR_IMPL_VERSION:
		value = 0;
		break;
	default:
		return SBI_ENOTSUPP;
	}
	*(u32 *)out = value;
	return 0;
}

static int tee_msi_xlate(struct mbox_controller *mbox,
			 const struct fdt_phandle_args *args, u32 *out)
{
	if (args->args_count != 2)
		return SBI_EINVAL;
	out[0] = args->args[0];
	out[1] = args->args[1];
	return 0;
}

static int tee_msi_init(const void *fdt, int nodeoff,
			const struct fdt_match *match)
{
	const fdt32_t *prop;
	int len, parent, rc;

	if (tee_msi.mbox.max_xfer_len)
		return SBI_EALREADY;
	prop = fdt_getprop(fdt, nodeoff, "msi-parent", &len);
	if (!prop || len != sizeof(*prop))
		return SBI_EINVAL;
	parent = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*prop));
	if (parent < 0 ||
	    fdt_node_check_compatible(fdt, parent, "riscv,imsics"))
		return SBI_EINVAL;
	rc = fdt_parse_imsic_node(fdt, parent, &tee_msi.imsic);
	if (rc || imsic_data_check(&tee_msi.imsic) ||
	    tee_msi.imsic.targets_mmode || tee_msi.imsic.guest_index_bits)
		return SBI_EINVAL;
	tee_msi.mbox.driver = (void *)&fdt_mailbox_rpmi_tee_msi;
	tee_msi.mbox.id = nodeoff;
	tee_msi.mbox.max_xfer_len = 4096;
	tee_msi.mbox.request_chan = tee_msi_request_chan;
	tee_msi.mbox.get_attribute = tee_msi_get_attribute;
	tee_msi.mbox.xfer = tee_msi_xfer;
	rc = mbox_controller_add(&tee_msi.mbox);
	if (rc) {
		tee_msi.mbox.max_xfer_len = 0;
		return rc;
	}
	rpmi_tee_signal_set_notify(tee_msi_notify);
	return 0;
}

static const struct fdt_match tee_msi_match[] = {
	{ .compatible = "opensbi,rpmi-tee-msi" },
	{ },
};

const struct fdt_mailbox fdt_mailbox_rpmi_tee_msi = {
	.driver = {
		.match_table = tee_msi_match,
		.init = tee_msi_init,
	},
	.xlate = tee_msi_xlate,
};
