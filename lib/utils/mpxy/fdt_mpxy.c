/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Anup Patel <apatel@ventanamicro.com>
 */

#include <sbi_utils/mpxy/fdt_mpxy.h>

/* List of FDT MPXY drivers generated at compile time */
extern const struct fdt_driver *const fdt_mpxy_drivers[];

#ifdef CONFIG_FDT_MPXY_RPMI_TEE
extern const struct fdt_driver fdt_mpxy_rpmi_tee;

static const struct fdt_driver *const tee_channel_drivers[] = {
	&fdt_mpxy_rpmi_tee,
	NULL,
};
#endif

int fdt_mpxy_init(const void *fdt)
{
#ifdef CONFIG_FDT_MPXY_RPMI_TEE
	int ret;

	/* Publish TEE endpoints before bridges validate their destination. */
	ret = fdt_driver_init_all(fdt, tee_channel_drivers);
	if (ret)
		return ret;
#endif
	return fdt_driver_init_all(fdt, fdt_mpxy_drivers);
}
