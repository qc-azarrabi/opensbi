/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <sbi/riscv_locks.h>
#include <sbi/sbi_bitmap.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/mpxy/rpmi_tee.h>

#define TEE_SIGNAL_WIDTH 64

enum tee_bus_state {
	TEE_BUS_NONE,
	TEE_BUS_SETUP,
	TEE_BUS_ACTIVE,
	TEE_BUS_TEARDOWN,
};

static struct {
	/* Protects lifecycle, partition and pending state. */
	spinlock_t lock;
	enum tee_bus_state state;
	struct rpmi_tee_endpoint *sender;
	struct rpmi_tee_endpoint *target;
	u32 width;
	u32 sender_signals;
	DECLARE_BITMAP(pending, TEE_SIGNAL_WIDTH);
	void (*notify)(u32 endpoint_id);
} bus = {
	.lock = __SPIN_LOCK_UNLOCKED,
};

void rpmi_tee_signal_set_notify(void (*notify)(u32 endpoint_id))
{
	bus.notify = notify;
}

u32 rpmi_tee_signal_feature(struct rpmi_tee_endpoint *ep)
{
	if (!bus.notify)
		return 0;
	return RPMI_TEE_SIGNAL_SYSTEM_MSI | (TEE_SIGNAL_WIDTH << 2) |
		((ep->tee ? 1U : 0U) << 12);
}

bool rpmi_tee_signal_pending(u32 endpoint_id)
{
	u32 first, last, i;
	bool pending = false;

	spin_lock(&bus.lock);
	if (!bus.sender || !bus.target || bus.state == TEE_BUS_NONE)
		goto out;
	if (endpoint_id == bus.sender->id) {
		first = 0;
		last = bus.sender_signals;
	} else if (endpoint_id == bus.target->id) {
		first = bus.sender_signals;
		last = bus.width;
	} else {
		goto out;
	}
	for (i = first; i < last; i++)
		if (bitmap_test(bus.pending, i)) {
			pending = true;
			break;
		}
out:
	spin_unlock(&bus.lock);
	return pending;
}

static s32 bus_control(struct rpmi_tee_endpoint *caller, u32 service,
		       const void *data, u32 len)
{
	const struct rpmi_tee_signal_bus_setup_req *req = data;
	struct rpmi_tee_endpoint *target;
	u32 width, split;

	if (caller->tee || len != (service == RPMI_TEE_SIGNAL_BUS_SETUP ?
	    sizeof(*req) : sizeof(struct rpmi_tee_signal_bus_teardown_req)) ||
	    le32_to_cpu(req->sender_id) != caller->id)
		return RPMI_ERR_INVALID_PARAM;
	target = rpmi_tee_endpoint_by_id(le32_to_cpu(req->target_id));
	if (!target || !target->tee || !rpmi_tee_endpoint_available(target))
		return RPMI_ERR_INVALID_PARAM;
	if (service == RPMI_TEE_SIGNAL_BUS_TEARDOWN) {
		if (bus.state != TEE_BUS_ACTIVE || bus.sender != caller ||
		    bus.target != target)
			return RPMI_ERR_ALREADY;
		bus.state = TEE_BUS_TEARDOWN;
		return RPMI_SUCCESS;
	}
	if (!bus.notify)
		return RPMI_ERR_NOTSUPP;
	width = le32_to_cpu(req->width);
	split = le32_to_cpu(req->sender_signals);
	if (!width || width > TEE_SIGNAL_WIDTH || split > width)
		return RPMI_ERR_BAD_RANGE;
	if (bus.state != TEE_BUS_NONE)
		return RPMI_ERR_ALREADY;
	bus.sender = caller;
	bus.target = target;
	bus.width = width;
	bus.sender_signals = split;
	bitmap_zero(bus.pending, TEE_SIGNAL_WIDTH);
	bus.state = TEE_BUS_SETUP;
	return RPMI_SUCCESS;
}

int rpmi_tee_signal_request(struct rpmi_tee_endpoint *caller, u32 service,
			    const void *data, u32 len, void *response,
			    u32 maxlen, unsigned long *response_len)
{
	const struct rpmi_tee_signal_raise_req *req = data;
	struct rpmi_tee_signal_retrieve_resp *resp = response;
	struct rpmi_tee_endpoint *peer = NULL;
	u32 first = 0, last = 0, count = 0, i, notify_id = 0;
	s32 status = RPMI_ERR_INVALID_PARAM;
	bool notify = false;

	spin_lock(&bus.lock);
	if (service == RPMI_TEE_SIGNAL_BUS_SETUP ||
	    service == RPMI_TEE_SIGNAL_BUS_TEARDOWN) {
		status = bus_control(caller, service, data, len);
		spin_unlock(&bus.lock);
		if (status == RPMI_SUCCESS)
			return SBI_ENOTSUPP;
		return rpmi_tee_status(response, maxlen, response_len, status);
	}
	if (bus.state == TEE_BUS_NONE || bus.state == TEE_BUS_SETUP)
		goto out;
	if (caller == bus.sender) {
		peer = bus.target;
		first = 0;
		last = bus.sender_signals;
	} else if (caller == bus.target) {
		peer = bus.sender;
		first = bus.sender_signals;
		last = bus.width;
	} else {
		goto out;
	}
	if (service == RPMI_TEE_SIGNAL_RAISE) {
		if (bus.state == TEE_BUS_TEARDOWN) {
			status = RPMI_ERR_DENIED;
			goto out;
		}
		if (len < sizeof(*req) ||
		    le32_to_cpu(req->target_id) != peer->id)
			goto out;
		count = le32_to_cpu(req->signal_count);
		if (!count || count != (len - sizeof(*req)) / sizeof(u32) ||
		    (len - sizeof(*req)) % sizeof(u32))
			goto out;
		/* A sender raises only IDs in the peer's readable partition. */
		for (i = 0; i < count; i++) {
			u32 id = le32_to_cpu(req->signals[i]);

			if (id >= bus.width || (id >= first && id < last))
				goto out;
		}
		for (i = 0; i < count; i++)
			bitmap_set(bus.pending, le32_to_cpu(req->signals[i]),
				   1);
		notify = true;
		notify_id = peer->id;
		status = RPMI_SUCCESS;
	} else if (service == RPMI_TEE_SIGNAL_RETRIEVE) {
		if (len)
			goto out;
		for (i = first; i < last; i++)
			if (bitmap_test(bus.pending, i))
				count++;
		if (!count) {
			status = RPMI_ERR_NO_DATA;
			goto out;
		}
		if (maxlen < sizeof(*resp) ||
		    count > (maxlen - sizeof(*resp)) / sizeof(u32)) {
			spin_unlock(&bus.lock);
			return SBI_ENOSPC;
		}
		count = 0;
		for (i = first; i < last; i++) {
			if (!bitmap_test(bus.pending, i))
				continue;
			resp->signals[count++] = cpu_to_le32(i);
			bitmap_clear(bus.pending, i, 1);
		}
		resp->status = cpu_to_le32(RPMI_SUCCESS);
		resp->flags = 0;
		resp->target_id = cpu_to_le32(peer->id);
		resp->signal_count = cpu_to_le32(count);
		*response_len = sizeof(*resp) + count * sizeof(u32);
		spin_unlock(&bus.lock);
		return 0;
	}
out:
	spin_unlock(&bus.lock);
	/* Deliver outside the bus lock without waiting for the receiver. */
	if (notify && bus.notify)
		bus.notify(notify_id);
	return rpmi_tee_status(response, maxlen, response_len, status);
}

s32 rpmi_tee_signal_complete(u32 service, const void *data, u32 len)
{
	s32 status = RPMI_ERR_IO;
	struct rpmi_tee_endpoint *failed = NULL;

	if (len == sizeof(u32))
		status = (s32)le32_to_cpu(*(const u32 *)data);
	spin_lock(&bus.lock);
	if (service == RPMI_TEE_SIGNAL_BUS_SETUP &&
	    bus.state == TEE_BUS_SETUP) {
		if (status == RPMI_SUCCESS) {
			bus.state = TEE_BUS_ACTIVE;
		} else {
			bus.state = TEE_BUS_NONE;
			if (status != RPMI_ERR_DENIED)
				failed = bus.target;
		}
	} else if (service == RPMI_TEE_SIGNAL_BUS_TEARDOWN &&
		   bus.state == TEE_BUS_TEARDOWN) {
		bus.state = TEE_BUS_NONE;
		bitmap_zero(bus.pending, TEE_SIGNAL_WIDTH);
		if (status != RPMI_SUCCESS)
			failed = bus.target;
	} else {
		status = RPMI_ERR_IO;
	}
	spin_unlock(&bus.lock);
	if (failed) {
		atomic_write(&failed->available, 0);
		rpmi_tee_memory_disconnect(failed);
		status = RPMI_ERR_IO;
	}
	return status;
}

void rpmi_tee_signal_cancel(u32 service)
{
	spin_lock(&bus.lock);
	if (service == RPMI_TEE_SIGNAL_BUS_SETUP && bus.state == TEE_BUS_SETUP)
		bus.state = TEE_BUS_NONE;
	else if (service == RPMI_TEE_SIGNAL_BUS_TEARDOWN &&
		 bus.state == TEE_BUS_TEARDOWN)
		bus.state = TEE_BUS_ACTIVE;
	spin_unlock(&bus.lock);
}
