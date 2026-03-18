/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 SiFive Inc.
 *
 * TEE MPXY Channel Interface
 */

#ifndef __FDT_MPXY_RPMI_TEE_H__
#define __FDT_MPXY_RPMI_TEE_H__

#include <sbi/sbi_types.h>
#include <sbi/sbi_mpxy.h>
#include <sbi_utils/tee/tee_dispatcher.h>

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
#ifdef CONFIG_FDT_MPXY_RPMI_TEE
struct sbi_mpxy_channel *mpxy_tee_find_channel_by_hartid(u32 hartid);
#else
static inline struct sbi_mpxy_channel *mpxy_tee_find_channel_by_hartid(u32 hartid)
{
	return NULL;
}
#endif

/**
 * Get TEE dispatcher from TEE MPXY channel
 *
 * This function extracts the TEE dispatcher from a TEE MPXY channel.
 * Used by the reqfwd driver for lazy callback registration.
 *
 * @param channel: Pointer to TEE MPXY channel
 * @return Pointer to TEE dispatcher, or NULL if not found
 */
#ifdef CONFIG_FDT_MPXY_RPMI_TEE
struct tee_dispatcher *mpxy_tee_get_dispatcher(struct sbi_mpxy_channel *channel);
#else
static inline struct tee_dispatcher *mpxy_tee_get_dispatcher(struct sbi_mpxy_channel *channel)
{
	return NULL;
}
#endif

#endif

