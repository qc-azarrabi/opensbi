/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <libfdt.h>
#include <sbi/sbi_byteorder.h>
#include <sbi/sbi_error.h>
#include <sbi_utils/fdt/fdt_driver.h>
#include <sbi_utils/mailbox/rpmi_mailbox.h>
#include <sbi_utils/mpxy/fdt_mpxy_rpmi.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

#define RPMI_TEE_CHANNEL_MAX_DATA_LEN 4096

#define TEE_SERVICE(srv) { .id = (srv), .min_tx_len = 0, \
	.max_tx_len = RPMI_TEE_CHANNEL_MAX_DATA_LEN, \
	.min_rx_len = sizeof(u32), \
	.max_rx_len = RPMI_TEE_CHANNEL_MAX_DATA_LEN }

static struct mpxy_rpmi_service_data tee_services[] = {
	TEE_SERVICE(RPMI_TEE_ENABLE_NOTIFICATION),
	TEE_SERVICE(RPMI_TEE_PROBE_FEATURES),
	TEE_SERVICE(RPMI_TEE_PROBE_SYSTEM),
	TEE_SERVICE(RPMI_TEE_PROBE_DOMAIN),
	TEE_SERVICE(RPMI_TEE_PROBE_ENDPOINT),
	TEE_SERVICE(RPMI_TEE_PROBE_MEMORY_PARCEL),
	TEE_SERVICE(RPMI_TEE_PROBE_MEMORY_SEGMENT),
	TEE_SERVICE(RPMI_TEE_PROBE_SERVICE),
	TEE_SERVICE(RPMI_TEE_EXIT),
	TEE_SERVICE(RPMI_TEE_SIGNAL_BUS_SETUP),
	TEE_SERVICE(RPMI_TEE_SIGNAL_BUS_TEARDOWN),
	TEE_SERVICE(RPMI_TEE_SIGNAL_RAISE),
	TEE_SERVICE(RPMI_TEE_SIGNAL_RETRIEVE),
	TEE_SERVICE(RPMI_TEE_MEMORY_PARCEL_CREATE),
	TEE_SERVICE(RPMI_TEE_MEMORY_PARCEL_ACCEPT),
	TEE_SERVICE(RPMI_TEE_MEMORY_PARCEL_RELEASE),
	TEE_SERVICE(RPMI_TEE_MEMORY_PARCEL_RECLAIM),
	TEE_SERVICE(RPMI_TEE_MEMORY_SEGMENT_SEND),
	TEE_SERVICE(RPMI_TEE_MEMORY_SEGMENT_RECEIVE),
	TEE_SERVICE(RPMI_TEE_REGISTER_FOR_LIFECYCLE_INFO),
	TEE_SERVICE(RPMI_TEE_ON_START_INFO),
	TEE_SERVICE(RPMI_TEE_ON_SHUTDOWN_INFO),
	TEE_SERVICE(RPMI_TEE_ON_CHANGED_INFO),
	TEE_SERVICE(RPMI_TEE_CALL),
};

int rpmi_tee_channel_get_attribute(u32 attr_id, u32 *value)
{
	switch (attr_id) {
	case RPMI_CHANNEL_ATTR_PROTOCOL_VERSION:
	case RPMI_CHANNEL_ATTR_SERVICEGROUP_VERSION:
		*value = RPMI_VERSION(2, 0);
		break;
	case RPMI_CHANNEL_ATTR_SERVICEGROUP_ID:
		*value = RPMI_SRVGRP_TEE;
		break;
	case RPMI_CHANNEL_ATTR_MAX_DATA_LEN:
		*value = RPMI_TEE_CHANNEL_MAX_DATA_LEN;
		break;
	case RPMI_CHANNEL_ATTR_TX_TIMEOUT:
		*value = RPMI_DEF_TX_TIMEOUT;
		break;
	case RPMI_CHANNEL_ATTR_RX_TIMEOUT:
		*value = RPMI_DEF_RX_TIMEOUT;
		break;
	default:
		*value = 0;
		break;
	}
	return 0;
}

static int tee_channel_get_attribute(void *context, struct mbox_chan *chan,
				    enum rpmi_channel_attribute_id attr_id,
				    u32 *value)
{
	return rpmi_tee_channel_get_attribute(attr_id, value);
}

static int tee_channel_setup(void **context, struct mbox_chan *chan,
			     struct sbi_domain *dom,
			     const struct mpxy_rpmi_data *data)
{
	struct rpmi_tee_endpoint *ep = rpmi_tee_endpoint_by_domain(dom);

	if (!ep || !ep->tee)
		return SBI_EINVAL;
	*context = dom;
	return 0;
}

static int tee_channel_xfer(void *context, struct mbox_chan *chan,
			    struct mbox_xfer *xfer)
{
	struct rpmi_message_args *args = xfer->args;
	unsigned long response_len = 0;
	int ret;

	if (args->flags & RPMI_MSG_FLAGS_NO_RX)
		return SBI_ENOTSUPP;
	ret = rpmi_tee_request(context, args->service_id, xfer->tx,
			       xfer->tx_len, xfer->rx, xfer->rx_len,
			       &response_len);
	/* This channel has no route for forwarding requests. */
	if (ret == SBI_ENOTSUPP) {
		rpmi_tee_signal_cancel(args->service_id);
		ret = rpmi_tee_status(xfer->rx, xfer->rx_len, &response_len,
				      RPMI_ERR_NOTSUPP);
	}
	if (ret)
		return ret;
	args->rx_data_len = response_len;
	xfer->rx_len = response_len;
	return 0;
}

const struct mpxy_rpmi_data rpmi_tee_local_data = {
	.servicegrp_id = RPMI_SRVGRP_TEE,
	.num_services = array_size(tee_services),
	.service_data = tee_services,
	.setup_group = tee_channel_setup,
	.get_attribute_group = tee_channel_get_attribute,
	.xfer_group = tee_channel_xfer,
};

static int tee_channel_probe(const void *fdt, int nodeoff,
			     const struct fdt_match *match)
{
	const fdt32_t *channel_id;
	struct rpmi_tee_endpoint *ep;
	int len, ret;

	if (!fdt_getprop(fdt, nodeoff, "riscv,tee", NULL))
		return SBI_EINVAL;
	channel_id = fdt_getprop(fdt, nodeoff, "riscv,sbi-mpxy-channel-id", &len);
	if (!channel_id || len != sizeof(*channel_id))
		return SBI_EINVAL;
	ret = rpmi_tee_register_endpoint(fdt, nodeoff);
	if (ret)
		return ret;
	ep = rpmi_tee_endpoint_by_role(true);
	return mpxy_rpmi_no_mbox_init(&rpmi_tee_local_data,
				     fdt32_to_cpu(*channel_id), ep->domain_name);
}

static const struct fdt_match tee_channel_match[] = {
	{ .compatible = "opensbi,rpmi-tee-channel" },
	{ }
};

const struct fdt_driver fdt_mpxy_rpmi_tee = {
	.match_table = tee_channel_match,
	.init = tee_channel_probe,
};
