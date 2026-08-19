/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * RPMI TEE memory parcel manager.
 *
 * Implements the framework-answered memory parcel lifecycle services of the
 * RPMI TEE Service Group (MEM_PARCEL_CREATE 0x09, MEM_PARCEL_ACCEPT 0x0A,
 * MEM_PARCEL_RELEASE 0x0B, MEM_PARCEL_RECLAIM 0x0C). These services are
 * handled entirely inside the REE-facing TEE MPXY driver: no dispatcher call,
 * no request forwarding, and no domain switch.
 *
 * Each entry point mirrors the framework-answered contract: the return value
 * is the SBI transport status; the RPMI STATUS word and any payload are
 * written into respbuf, and *resp_len is set on success.
 */

#ifndef __RPMI_TEE_PARCEL_H__
#define __RPMI_TEE_PARCEL_H__

#include <sbi/sbi_types.h>

/**
 * Initialize the parcel pool.
 * Idempotent; safe across per-hart channel setup.
 */
void rpmi_tee_parcel_init(void);

int rpmi_tee_parcel_create(void *msgbuf, u32 msg_len,
			   void *respbuf, u32 resp_max_len,
			   unsigned long *resp_len);

int rpmi_tee_parcel_accept(void *msgbuf, u32 msg_len,
			   void *respbuf, u32 resp_max_len,
			   unsigned long *resp_len);

int rpmi_tee_parcel_release(void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *resp_len);

int rpmi_tee_parcel_reclaim(void *msgbuf, u32 msg_len,
			    void *respbuf, u32 resp_max_len,
			    unsigned long *resp_len);

int rpmi_tee_parcel_segment_send(void *msgbuf, u32 msg_len,
				 void *respbuf, u32 resp_max_len,
				 unsigned long *resp_len);

int rpmi_tee_parcel_segment_receive(void *msgbuf, u32 msg_len,
				    void *respbuf, u32 resp_max_len,
				    unsigned long *resp_len);

#endif /* __RPMI_TEE_PARCEL_H__ */
