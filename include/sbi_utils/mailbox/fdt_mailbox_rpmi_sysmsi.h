/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * In-framework RPMI SYSTEM_MSI mailbox backend interface.
 *
 * The RPMI SYSTEM_MSI MPXY driver (lib/utils/mpxy/fdt_mpxy_rpmi_sysmsi.c) is a
 * forwarding proxy: it validates and relays SYSTEM_MSI services to whatever
 * mailbox controller its "mboxes" property points at. On real hardware that is
 * the shared-memory transport to an external platform microcontroller. Under
 * QEMU virt there is no such microcontroller, so an in-framework mailbox
 * controller (lib/utils/mailbox/fdt_mailbox_rpmi_sysmsi.c) answers the
 * SYSTEM_MSI services directly and keeps the per-index MSI target+state.
 *
 * This header exposes only the raw System MSI emission helper so the TEE
 * signal bus can poke the REE at its configured SYS_MSI_INDEX after a raise.
 */

#ifndef __FDT_MAILBOX_RPMI_SYSMSI_H__
#define __FDT_MAILBOX_RPMI_SYSMSI_H__

#include <sbi/sbi_types.h>
#include <sbi/sbi_error.h>

/**
 * Emit the System MSI at the given SYS_MSI_INDEX.
 *
 * If the in-framework SYSTEM_MSI mailbox controller is registered and the
 * index has been enabled (SET_MSI_STATE) with a valid target (SET_MSI_TARGET),
 * performs the raw MSI write to the configured target address. Otherwise a
 * no-op returning an error, so callers can fire unconditionally on the raise
 * path.
 *
 * @param index: SYS_MSI_INDEX to emit
 * @return SBI_OK on emission, SBI_ERR_* otherwise
 */
#ifdef CONFIG_FDT_MAILBOX_RPMI_SYSMSI
int rpmi_sysmsi_send(u32 index);
#else
static inline int rpmi_sysmsi_send(u32 index)
{
	return SBI_ENODEV;
}
#endif

#endif
