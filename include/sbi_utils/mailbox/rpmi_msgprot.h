/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Rahul Pathak <rpathak@ventanamicro.com>
 *   Subrahmanya Lingappa <slingappa@ventanamicro.com>
 */

#ifndef __RPMI_MSGPROT_H__
#define __RPMI_MSGPROT_H__

#include <sbi/sbi_byteorder.h>
#include <sbi/sbi_error.h>

/*
 * 31                                            0
 * +---------------------+-----------------------+
 * | FLAGS | SERVICE_ID  |   SERVICEGROUP_ID     |
 * +---------------------+-----------------------+
 * |        TOKEN        |     DATA LENGTH       |
 * +---------------------+-----------------------+
 * |                 DATA/PAYLOAD                |
 * +---------------------------------------------+
 */

/** Message Header byte offset */
#define RPMI_MSG_HDR_OFFSET			(0x0)
/** Message Header Size in bytes */
#define RPMI_MSG_HDR_SIZE			(8)

/** ServiceGroup ID field byte offset */
#define RPMI_MSG_SERVICEGROUP_ID_OFFSET		(0x0)
/** ServiceGroup ID field size in bytes */
#define RPMI_MSG_SERVICEGROUP_ID_SIZE		(2)

/** Service ID field byte offset */
#define RPMI_MSG_SERVICE_ID_OFFSET		(0x2)
/** Service ID field size in bytes */
#define RPMI_MSG_SERVICE_ID_SIZE		(1)

/** Flags field byte offset */
#define RPMI_MSG_FLAGS_OFFSET			(0x3)
/** Flags field size in bytes */
#define RPMI_MSG_FLAGS_SIZE			(1)

#define RPMI_MSG_FLAGS_TYPE_POS			(0U)
#define RPMI_MSG_FLAGS_TYPE_MASK		0x7
#define RPMI_MSG_FLAGS_TYPE			\
	((0x7) << RPMI_MSG_FLAGS_TYPE_POS)

#define RPMI_MSG_FLAGS_DOORBELL_POS		(3U)
#define RPMI_MSG_FLAGS_DOORBELL_MASK		0x1
#define RPMI_MSG_FLAGS_DOORBELL			\
	((0x1) << RPMI_MSG_FLAGS_DOORBELL_POS)

/** Data length field byte offset */
#define RPMI_MSG_DATALEN_OFFSET			(0x4)
/** Data length field size in bytes */
#define RPMI_MSG_DATALEN_SIZE			(2)

/** Token field byte offset */
#define RPMI_MSG_TOKEN_OFFSET			(0x6)
/** Token field size in bytes */
#define RPMI_MSG_TOKEN_SIZE			(2)
/** Token field mask */
#define RPMI_MSG_TOKEN_MASK			(0xffffU)

/** Data field byte offset */
#define RPMI_MSG_DATA_OFFSET			(RPMI_MSG_HDR_SIZE)
/** Data field size in bytes */
#define RPMI_MSG_DATA_SIZE(__slot_size)		((__slot_size) - RPMI_MSG_HDR_SIZE)

/** Minimum slot size in bytes */
#define RPMI_SLOT_SIZE_MIN			(64)

#define RPMI_REQFWD_FIFO_SLOT_SIZE		(128)

/** Name length of 16 characters */
#define RPMI_NAME_CHARS_MAX			(16)

/** Queue layout */
#define RPMI_QUEUE_HEAD_SLOT		0
#define RPMI_QUEUE_TAIL_SLOT		1
#define RPMI_QUEUE_HEADER_SLOTS		2

/** Default timeout values */
#define RPMI_DEF_TX_TIMEOUT			20
#define RPMI_DEF_RX_TIMEOUT			20

/**
 * Common macro to generate composite version from major
 * and minor version numbers.
 *
 * RPMI has Specification version, Implementation version
 * Service group versions which follow the same versioning
 * encoding as below.
 */
#define RPMI_VERSION(__major, __minor) (((__major) << 16) | (__minor))

/** RPMI Message Header */
struct rpmi_message_header {
	le16_t servicegroup_id;
	uint8_t service_id;
	uint8_t flags;
	le16_t datalen;
	le16_t token;
} __packed;

/** RPMI Message */
struct rpmi_message {
	struct rpmi_message_header header;
	u8 data[0];
} __packed;

/** RPMI Messages Types */
enum rpmi_message_type {
	/* Normal request backed with ack */
	RPMI_MSG_NORMAL_REQUEST = 0x0,
	/* Request without any ack */
	RPMI_MSG_POSTED_REQUEST = 0x1,
	/* Acknowledgment for normal request message */
	RPMI_MSG_ACKNOWLDGEMENT = 0x2,
	/* Notification message */
	RPMI_MSG_NOTIFICATION = 0x3,
};

/** RPMI Error Types */
enum rpmi_error {
	/* Success */
	RPMI_SUCCESS		= 0,
	/* General failure  */
	RPMI_ERR_FAILED		= -1,
	/* Service or feature not supported */
	RPMI_ERR_NOTSUPP	= -2,
	/* Invalid Parameter  */
	RPMI_ERR_INVALID_PARAM    = -3,
	/*
	 * Denied to insufficient permissions
	 * or due to unmet prerequisite
	 */
	RPMI_ERR_DENIED		= -4,
	/* Invalid address or offset */
	RPMI_ERR_INVALID_ADDR	= -5,
	/*
	 * Operation failed as it was already in
	 * progress or the state has changed already
	 * for which the operation was carried out.
	 */
	RPMI_ERR_ALREADY	= -6,
	/*
	 * Error in implementation which violates
	 * the specification version
	 */
	RPMI_ERR_EXTENSION	= -7,
	/* Operation failed due to hardware issues */
	RPMI_ERR_HW_FAULT	= -8,
	/* System, device or resource is busy */
	RPMI_ERR_BUSY		= -9,
	/* System or device or resource in invalid state */
	RPMI_ERR_INVALID_STATE	= -10,
	/* Index, offset or address is out of range */
	RPMI_ERR_BAD_RANGE	= -11,
	/* Operation timed out */
	RPMI_ERR_TIMEOUT	= -12,
	/*
	 * Error in input or output or
	 * error in sending or receiving data
	 * through communication medium
	 */
	RPMI_ERR_IO		= -13,
	/* No data available */
	RPMI_ERR_NO_DATA	= -14,
	RPMI_ERR_RESERVED_START	= -15,
	RPMI_ERR_RESERVED_END	= -127,
	RPMI_ERR_VENDOR_START	= -128,
};

/** RPMI Mailbox Message Arguments */
struct rpmi_message_args {
	u32 flags;
#define RPMI_MSG_FLAGS_NO_TX		(1U << 0)
#define RPMI_MSG_FLAGS_NO_RX		(1U << 1)
#define RPMI_MSG_FLAGS_NO_RX_TOKEN	(1U << 2)
	enum rpmi_message_type type;
	u8 service_id;
	u32 tx_endian_words;
	u32 rx_endian_words;
	u16 rx_token;
	u32 rx_data_len;
};

/** RPMI Mailbox Channel Attribute IDs */
enum rpmi_channel_attribute_id {
	RPMI_CHANNEL_ATTR_PROTOCOL_VERSION = 0,
	RPMI_CHANNEL_ATTR_MAX_DATA_LEN,
	RPMI_CHANNEL_ATTR_P2A_DOORBELL_SYSMSI_INDEX,
	RPMI_CHANNEL_ATTR_TX_TIMEOUT,
	RPMI_CHANNEL_ATTR_RX_TIMEOUT,
	RPMI_CHANNEL_ATTR_SERVICEGROUP_ID,
	RPMI_CHANNEL_ATTR_SERVICEGROUP_VERSION,
	RPMI_CHANNEL_ATTR_IMPL_ID,
	RPMI_CHANNEL_ATTR_IMPL_VERSION,
	RPMI_CHANNEL_ATTR_MAX,
};

/*
 * RPMI SERVICEGROUPS AND SERVICES
 */

/** RPMI ServiceGroups IDs */
enum rpmi_servicegroup_id {
	RPMI_SRVGRP_ID_MIN = 0,
	RPMI_SRVGRP_BASE = 0x0001,
	RPMI_SRVGRP_SYSTEM_MSI = 0x0002,
	RPMI_SRVGRP_SYSTEM_RESET = 0x0003,
	RPMI_SRVGRP_SYSTEM_SUSPEND = 0x0004,
	RPMI_SRVGRP_HSM = 0x0005,
	RPMI_SRVGRP_CPPC = 0x0006,
	RPMI_SRVGRP_VOLTAGE = 0x00007,
	RPMI_SRVGRP_CLOCK = 0x0008,
	RPMI_SRVGRP_DEVICE_POWER = 0x0009,
	RPMI_SRVGRP_PERFORMANCE = 0x0000A,
	RPMI_SRVGRP_MANAGEMENT_MODE = 0x000B,
	RPMI_SRVGRP_REQFWD = 0x000D,
	RPMI_SRVGRP_TEE = 0x0010,
	RPMI_SRVGRP_ID_MAX_COUNT,

	/* Reserved range for service groups */
	RPMI_SRVGRP_RESERVE_START = RPMI_SRVGRP_ID_MAX_COUNT,
	RPMI_SRVGRP_RESERVE_END = 0x7FFF,

	/* Vendor/Implementation-specific service groups range */
	RPMI_SRVGRP_VENDOR_START = 0x8000,
	RPMI_SRVGRP_VENDOR_END = 0xFFFF,
};

/** RPMI enable notification request */
struct rpmi_enable_notification_req {
	u32 eventid;
};

/** RPMI enable notification response */
struct rpmi_enable_notification_resp {
	s32 status;
};

/** RPMI Base ServiceGroup Service IDs */
enum rpmi_base_service_id {
	RPMI_BASE_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_BASE_SRV_GET_IMPLEMENTATION_VERSION = 0x02,
	RPMI_BASE_SRV_GET_IMPLEMENTATION_IDN = 0x03,
	RPMI_BASE_SRV_GET_SPEC_VERSION = 0x04,
	RPMI_BASE_SRV_GET_PLATFORM_INFO = 0x05,
	RPMI_BASE_SRV_PROBE_SERVICE_GROUP = 0x06,
	RPMI_BASE_SRV_GET_ATTRIBUTES = 0x07,
};

#define RPMI_BASE_FLAGS_F0_PRIVILEGE		(1U << 1)
#define RPMI_BASE_FLAGS_F0_EV_NOTIFY		(1U << 0)

enum rpmi_base_context_priv_level {
	RPMI_BASE_CONTEXT_PRIV_S_MODE,
	RPMI_BASE_CONTEXT_PRIV_M_MODE,
};

struct rpmi_base_get_attributes_resp {
	s32 status_code;
	u32 f0;
	u32 f1;
	u32 f2;
	u32 f3;
};

struct rpmi_base_get_platform_info_resp {
	s32 status;
	u32 plat_info_len;
	char plat_info[];
};

/** RPMI System MSI ServiceGroup Service IDs */
enum rpmi_sysmsi_service_id {
	RPMI_SYSMSI_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_SYSMSI_SRV_GET_ATTRIBUTES = 0x2,
	RPMI_SYSMSI_SRV_GET_MSI_ATTRIBUTES = 0x3,
	RPMI_SYSMSI_SRV_SET_MSI_STATE = 0x4,
	RPMI_SYSMSI_SRV_GET_MSI_STATE = 0x5,
	RPMI_SYSMSI_SRV_SET_MSI_TARGET = 0x6,
	RPMI_SYSMSI_SRV_GET_MSI_TARGET = 0x7,
	RPMI_SYSMSI_SRV_ID_MAX_COUNT,
};

/** Response for system MSI service group attributes */
struct rpmi_sysmsi_get_attributes_resp {
	s32 status;
	u32 sys_num_msi;
	u32 flag0;
	u32 flag1;
};

/** Request for system MSI attributes */
struct rpmi_sysmsi_get_msi_attributes_req {
	u32 sys_msi_index;
};

/** Response for system MSI attributes */
struct rpmi_sysmsi_get_msi_attributes_resp {
	s32 status;
	u32 flag0;
	u32 flag1;
	u8 name[16];
};

#define RPMI_SYSMSI_MSI_ATTRIBUTES_FLAG0_PREF_PRIV	(1U << 0)

/** Request for system MSI set state */
struct rpmi_sysmsi_set_msi_state_req {
	u32 sys_msi_index;
	u32 sys_msi_state;
};

#define RPMI_SYSMSI_MSI_STATE_ENABLE			(1U << 0)
#define RPMI_SYSMSI_MSI_STATE_PENDING			(1U << 1)

/** Response for system MSI set state */
struct rpmi_sysmsi_set_msi_state_resp {
	s32 status;
};

/** Request for system MSI get state */
struct rpmi_sysmsi_get_msi_state_req {
	u32 sys_msi_index;
};

/** Response for system MSI get state */
struct rpmi_sysmsi_get_msi_state_resp {
	s32 status;
	u32 sys_msi_state;
};

/** Request for system MSI set target */
struct rpmi_sysmsi_set_msi_target_req {
	u32 sys_msi_index;
	u32 sys_msi_address_low;
	u32 sys_msi_address_high;
	u32 sys_msi_data;
};

/** Response for system MSI set target */
struct rpmi_sysmsi_set_msi_target_resp {
	s32 status;
};

/** Request for system MSI get target */
struct rpmi_sysmsi_get_msi_target_req {
	u32 sys_msi_index;
};

/** Response for system MSI get target */
struct rpmi_sysmsi_get_msi_target_resp {
	s32 status;
	u32 sys_msi_address_low;
	u32 sys_msi_address_high;
	u32 sys_msi_data;
};

/** RPMI System Reset ServiceGroup Service IDs */
enum rpmi_system_reset_service_id {
	RPMI_SYSRST_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_SYSRST_SRV_GET_ATTRIBUTES = 0x02,
	RPMI_SYSRST_SRV_SYSTEM_RESET = 0x03,
	RPMI_SYSRST_SRV_ID_MAX_COUNT,
};

/** RPMI System Reset types */
enum rpmi_sysrst_reset_type {
	RPMI_SYSRST_TYPE_SHUTDOWN = 0x0,
	RPMI_SYSRST_TYPE_COLD_REBOOT = 0x1,
	RPMI_SYSRST_TYPE_WARM_REBOOT = 0x2,
	RPMI_SYSRST_TYPE_MAX,
};

#define RPMI_SYSRST_ATTRS_FLAGS_RESETTYPE_POS		(1)
#define RPMI_SYSRST_ATTRS_FLAGS_RESETTYPE_MASK		\
			(1U << RPMI_SYSRST_ATTRS_FLAGS_RESETTYPE_POS)

/** Response for system reset attributes */
struct rpmi_sysrst_get_reset_attributes_resp {
	s32 status;
	u32 flags;
};

/** RPMI System Suspend ServiceGroup Service IDs */
enum rpmi_system_suspend_service_id {
	RPMI_SYSSUSP_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_SYSSUSP_SRV_GET_ATTRIBUTES = 0x02,
	RPMI_SYSSUSP_SRV_SYSTEM_SUSPEND = 0x03,
	RPMI_SYSSUSP_SRV_ID_MAX_COUNT,
};

/** Request for system suspend attributes */
struct rpmi_syssusp_get_attr_req {
	u32 susp_type;
};

#define RPMI_SYSSUSP_ATTRS_FLAGS_RESUMEADDR	(1U << 1)
#define RPMI_SYSSUSP_ATTRS_FLAGS_SUSPENDTYPE	1U

/** Response for system suspend attributes */
struct rpmi_syssusp_get_attr_resp {
	s32 status;
	u32 flags;
};

struct rpmi_syssusp_suspend_req {
	u32 hartid;
	u32 suspend_type;
	u32 resume_addr_lo;
	u32 resume_addr_hi;
};

struct rpmi_syssusp_suspend_resp {
	s32 status;
};

/** RPMI HSM State Management ServiceGroup Service IDs */
enum rpmi_hsm_service_id {
	RPMI_HSM_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_HSM_SRV_GET_HART_STATUS = 0x02,
	RPMI_HSM_SRV_GET_HART_LIST = 0x03,
	RPMI_HSM_SRV_GET_SUSPEND_TYPES = 0x04,
	RPMI_HSM_SRV_GET_SUSPEND_INFO = 0x05,
	RPMI_HSM_SRV_HART_START = 0x06,
	RPMI_HSM_SRV_HART_STOP = 0x07,
	RPMI_HSM_SRV_HART_SUSPEND = 0x08,
	RPMI_HSM_SRV_ID_MAX = 0x09,
};

/* HSM service group request and response structs */
struct rpmi_hsm_hart_start_req {
	u32 hartid;
	u32 start_addr_lo;
	u32 start_addr_hi;
};

struct rpmi_hsm_hart_start_resp {
	s32 status;
};

struct rpmi_hsm_hart_stop_req {
	u32 hartid;
};

struct rpmi_hsm_hart_stop_resp {
	s32 status;
};

struct rpmi_hsm_hart_susp_req {
	u32 hartid;
	u32 suspend_type;
	u32 resume_addr_lo;
	u32 resume_addr_hi;
};

struct rpmi_hsm_hart_susp_resp {
	s32 status;
};

struct rpmi_hsm_get_hart_status_req {
	u32 hartid;
};

struct rpmi_hsm_get_hart_status_resp {
	s32 status;
	u32 hart_status;
};

struct rpmi_hsm_get_hart_list_req {
	u32 start_index;
};

struct rpmi_hsm_get_hart_list_resp {
	s32 status;
	u32 remaining;
	u32 returned;
	/* remaining space need to be adjusted for the above 3 u32's */
	u32 hartid[(RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) - (sizeof(u32) * 3)) / sizeof(u32)];
};

struct rpmi_hsm_get_susp_types_req {
	u32 start_index;
};

struct rpmi_hsm_get_susp_types_resp {
	s32 status;
	u32 remaining;
	u32 returned;
	/* remaining space need to be adjusted for the above 3 u32's */
	u32 types[(RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) - (sizeof(u32) * 3)) / sizeof(u32)];
};

struct rpmi_hsm_get_susp_info_req {
	u32 suspend_type;
};

#define RPMI_HSM_SUSPEND_INFO_FLAGS_TIMER_STOP		1U

struct rpmi_hsm_get_susp_info_resp {
	s32 status;
	u32 flags;
	u32 entry_latency_us;
	u32 exit_latency_us;
	u32 wakeup_latency_us;
	u32 min_residency_us;
};

/** RPMI CPPC ServiceGroup Service IDs */
enum rpmi_cppc_service_id {
	RPMI_CPPC_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_CPPC_SRV_PROBE_REG = 0x02,
	RPMI_CPPC_SRV_READ_REG = 0x03,
	RPMI_CPPC_SRV_WRITE_REG = 0x04,
	RPMI_CPPC_SRV_GET_FAST_CHANNEL_REGION = 0x05,
	RPMI_CPPC_SRV_GET_FAST_CHANNEL_OFFSET = 0x06,
	RPMI_CPPC_SRV_GET_HART_LIST = 0x07,
	RPMI_CPPC_SRV_MAX_COUNT,
};

struct rpmi_cppc_probe_req {
	u32 hart_id;
	u32 reg_id;
};

struct rpmi_cppc_probe_resp {
	s32 status;
	u32 reg_len;
};

struct rpmi_cppc_read_reg_req {
	u32 hart_id;
	u32 reg_id;
};

struct rpmi_cppc_read_reg_resp {
	s32 status;
	u32 data_lo;
	u32 data_hi;
};

struct rpmi_cppc_write_reg_req {
	u32 hart_id;
	u32 reg_id;
	u32 data_lo;
	u32 data_hi;
};

struct rpmi_cppc_write_reg_resp {
	s32 status;
};

struct rpmi_cppc_get_fastchan_offset_req {
	u32 hart_id;
};

struct rpmi_cppc_get_fastchan_offset_resp {
	s32 status;
	u32 fc_perf_request_offset_lo;
	u32 fc_perf_request_offset_hi;
	u32 fc_perf_feedback_offset_lo;
	u32 fc_perf_feedback_offset_hi;
};

#define RPMI_CPPC_FAST_CHANNEL_CPPC_MODE_POS		3
#define RPMI_CPPC_FAST_CHANNEL_CPPC_MODE_MASK		\
			(3U << RPMI_CPPC_FAST_CHANNEL_CPPC_MODE_POS)
#define RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_WIDTH_POS	1
#define RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_WIDTH_MASK	\
			(3U << RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_WIDTH_POS)
#define RPMI_CPPC_FAST_CHANNEL_FLAGS_DB_SUPPORTED	(1U << 0)

struct rpmi_cppc_get_fastchan_region_resp {
	s32 status;
	u32 flags;
	u32 region_addr_lo;
	u32 region_addr_hi;
	u32 region_size_lo;
	u32 region_size_hi;
	u32 db_addr_lo;
	u32 db_addr_hi;
	u32 db_setmask_lo;
	u32 db_setmask_hi;
	u32 db_preservemask_lo;
	u32 db_preservemask_hi;
};

enum rpmi_cppc_fast_channel_db_width {
	RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_8 = 0x0,
	RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_16 = 0x1,
	RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_32 = 0x2,
	RPMI_CPPC_FAST_CHANNEL_DB_WIDTH_64 = 0x3,
};

enum rpmi_cppc_fast_channel_cppc_mode {
	RPMI_CPPC_FAST_CHANNEL_CPPC_MODE_PASSIVE = 0x0,
	RPMI_CPPC_FAST_CHANNEL_CPPC_MODE_ACTIVE = 0x1,
	RPMI_CPPC_FAST_CHANNEL_CPPC_MODE_MAX_IDX,
};

struct rpmi_cppc_hart_list_req {
	u32 start_index;
};

struct rpmi_cppc_hart_list_resp {
	s32 status;
	u32 remaining;
	u32 returned;
	/* remaining space need to be adjusted for the above 3 u32's */
	u32 hartid[(RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) - (sizeof(u32) * 3)) / sizeof(u32)];
};

/** RPMI Voltage ServiceGroup Service IDs */
enum rpmi_voltage_service_id {
	RPMI_VOLTAGE_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_VOLTAGE_SRV_GET_NUM_DOMAINS = 0x02,
	RPMI_VOLTAGE_SRV_GET_ATTRIBUTES = 0x03,
	RPMI_VOLTAGE_SRV_GET_SUPPORTED_LEVELS = 0x04,
	RPMI_VOLTAGE_SRV_SET_CONFIG = 0x05,
	RPMI_VOLTAGE_SRV_GET_CONFIG = 0x06,
	RPMI_VOLTAGE_SRV_SET_LEVEL = 0x07,
	RPMI_VOLTAGE_SRV_GET_LEVEL = 0x08,
	RPMI_VOLTAGE_SRV_MAX_COUNT,
};

struct rpmi_voltage_get_num_domains_resp {
       s32 status;
       u32 num_domains;
};

struct rpmi_voltage_get_attributes_req {
        u32 domain_id;
};

struct rpmi_voltage_get_attributes_resp {
        s32 status;
        u32 flags;
        u32 num_levels;
        u32 transition_latency;
        u8 name[16];
};

struct rpmi_voltage_get_supported_rate_req {
        u32 domain_id;
        u32 index;
};

struct rpmi_voltage_get_supported_rate_resp {
        s32 status;
        u32 flags;
        u32 remaining;
        u32 returned;
	u32 level[0];
};

struct rpmi_voltage_set_config_req {
        u32 domain_id;
#define RPMI_CLOCK_CONFIG_ENABLE                (1U << 0)
        u32 config;
};

struct rpmi_voltage_set_config_resp {
        s32 status;
};

struct rpmi_voltage_get_config_req {
        u32 domain_id;
};

struct rpmi_voltage_get_config_resp {
        s32 status;
        u32 config;
};

struct rpmi_voltage_set_level_req {
        u32 domain_id;
        s32 level;
};

struct rpmi_voltage_set_level_resp {
        s32 status;
};

struct rpmi_voltage_get_level_req {
        u32 domain_id;
};

struct rpmi_voltage_get_level_resp {
        s32 status;
        s32 level;
};

/** RPMI Clock ServiceGroup Service IDs */
enum rpmi_clock_service_id {
	RPMI_CLOCK_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_CLOCK_SRV_GET_NUM_CLOCKS = 0x02,
	RPMI_CLOCK_SRV_GET_ATTRIBUTES = 0x03,
	RPMI_CLOCK_SRV_GET_SUPPORTED_RATES = 0x04,
	RPMI_CLOCK_SRV_SET_CONFIG = 0x05,
	RPMI_CLOCK_SRV_GET_CONFIG = 0x06,
	RPMI_CLOCK_SRV_SET_RATE = 0x07,
	RPMI_CLOCK_SRV_GET_RATE = 0x08,
	RPMI_CLOCK_SRV_MAX_COUNT,
};

struct rpmi_clock_get_num_clocks_resp {
	s32 status;
	u32 num_clocks;
};

struct rpmi_clock_get_attributes_req {
	u32 clock_id;
};

struct rpmi_clock_get_attributes_resp {
	s32 status;
#define RPMI_CLOCK_FLAGS_FORMAT_POS		30
#define RPMI_CLOCK_FLAGS_FORMAT_MASK		\
			(3U << RPMI_CLOCK_FLAGS_CLOCK_FORMAT_POS)
#define RPMI_CLOCK_FLAGS_FORMAT_DISCRETE	0
#define RPMI_CLOCK_FLAGS_FORMAT_LINEAR		1
	u32 flags;
	u32 num_rates;
	u32 transition_latency;
	u8 name[16];
};

struct rpmi_clock_get_supported_rates_req {
	u32 clock_id;
	u32 clock_rate_index;
};

struct rpmi_clock_get_supported_rates_resp {
	s32 status;
	u32 flags;
	u32 remaining;
	u32 returned;
	u32 clock_rate[0];
};

struct rpmi_clock_set_config_req {
	u32 clock_id;
#define RPMI_CLOCK_CONFIG_ENABLE		(1U << 0)
	u32 config;
};

struct rpmi_clock_set_config_resp {
	s32 status;
};

struct rpmi_clock_get_config_req {
	u32 clock_id;
};

struct rpmi_clock_get_config_resp {
	s32 status;
	u32 config;
};

struct rpmi_clock_set_rate_req {
	u32 clock_id;
#define RPMI_CLOCK_SET_RATE_FLAGS_MASK		(3U << 0)
#define RPMI_CLOCK_SET_RATE_FLAGS_ROUND_DOWN	0
#define RPMI_CLOCK_SET_RATE_FLAGS_ROUND_UP	1
#define RPMI_CLOCK_SET_RATE_FLAGS_ROUND_PLAT	2
	u32 flags;
	u32 clock_rate_low;
	u32 clock_rate_high;
};

struct rpmi_clock_set_rate_resp {
	s32 status;
};

struct rpmi_clock_get_rate_req {
	u32 clock_id;
};

struct rpmi_clock_get_rate_resp {
	s32 status;
	u32 clock_rate_low;
	u32 clock_rate_high;
};

/** RPMI Device Power ServiceGroup Service IDs */
enum rpmi_dpwr_service_id {
	RPMI_DPWR_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_DPWR_SRV_GET_NUM_DOMAINS = 0x02,
	RPMI_DPWR_SRV_GET_ATTRIBUTES = 0x03,
	RPMI_DPWR_SRV_SET_STATE = 0x04,
	RPMI_DPWR_SRV_GET_STATE = 0x05,
	RPMI_DPWR_SRV_MAX_COUNT,
};

struct rpmi_dpwr_get_num_domain_resp {
	s32 status;
	u32 num_domain;
};

struct rpmi_dpwr_get_attrs_req {
	u32 domain_id;
};

struct rpmi_dpwr_get_attrs_resp {
	s32 status;
	u32 flags;
	u32 transition_latency;
	u8 name[16];
};

struct rpmi_dpwr_set_state_req {
	u32 domain_id;
	u32 state;
};

struct rpmi_dpwr_set_state_resp {
	s32 status;
};

struct rpmi_dpwr_get_state_req {
	u32 domain_id;
};

struct rpmi_dpwr_get_state_resp {
	s32 status;
	u32 state;
};

/** RPMI Performance ServiceGroup Service IDs */
enum rpmi_performance_service_id {
	RPMI_PERF_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_PERF_SRV_GET_NUM_DOMAINS = 0x02,
	RPMI_PERF_SRV_GET_ATTRIBUTES = 0x03,
	RPMI_PERF_SRV_GET_SUPPORTED_LEVELS = 0x04,
	RPMI_PERF_SRV_GET_LEVEL = 0x05,
	RPMI_PERF_SRV_SET_LEVEL = 0x06,
	RPMI_PERF_SRV_GET_LIMIT = 0x07,
	RPMI_PERF_SRV_SET_LIMIT = 0x08,
	RPMI_PERF_SRV_GET_FAST_CHANNEL_REGION = 0x09,
	RPMI_PERF_SRV_GET_FAST_CHANNEL_ATTRIBUTES = 0x0A,
	RPMI_PERF_SRV_MAX_COUNT,
};

struct rpmi_perf_get_num_domain_resp {
	s32 status;
	u32 num_domains;
};

struct rpmi_perf_get_attrs_req {
	u32 domain_id;
};

struct rpmi_perf_get_attrs_resp {
	s32 status;
	u32 flags;
	u32 num_level;
	u32 latency;
	u8 name[16];
};

struct rpmi_perf_get_supported_level_req {
	u32 domain_id;
	u32 perf_level_index;
};

struct rpmi_perf_domain_level {
	u32 level_index;
	u32 opp_level;
	u32 power_cost_uw;
	u32 transition_latency_us;
};

struct rpmi_perf_get_supported_level_resp {
	s32 status;
	u32 reserve;
	u32 remaining;
	u32 returned;
	struct rpmi_perf_domain_level level[0];
};

struct rpmi_perf_get_level_req {
	u32 domain_id;
};

struct rpmi_perf_get_level_resp {
	s32 status;
	u32 level_index;
};

struct rpmi_perf_set_level_req {
	u32 domain_id;
	u32 level_index;
};

struct rpmi_perf_set_level_resp {
	s32 status;
};

struct rpmi_perf_get_limit_req {
	u32 domain_id;
};

struct rpmi_perf_get_limit_resp {
	s32 status;
	u32 level_index_max;
	u32 level_index_min;
};

struct rpmi_perf_set_limit_req {
	u32 domain_id;
	u32 level_index_max;
	u32 level_index_min;
};

struct rpmi_perf_set_limit_resp {
	s32 status;
};

struct rpmi_perf_get_fast_chn_region_resp {
	s32 status;
	u32 region_phy_addr_low;
	u32 region_phy_addr_high;
	u32 region_size_low;
	u32 region_size_high;
};

struct rpmi_perf_get_fast_chn_attr_req {
	u32 domain_id;
	u32 service_id;
};

struct rpmi_perf_get_fast_chn_attr_resp {
	s32 status;
	u32 flags;
	u32 region_offset_low;
	u32 region_offset_high;
	u32 region_size;
	u32 db_addr_low;
	u32 db_addr_high;
	u32 db_id_low;
	u32 db_id_high;
	u32 db_perserved_low;
	u32 db_perserved_high;
};

/** RPMI MM ServiceGroup Service IDs */
enum rpmi_mm_service_id {
	RPMI_MM_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_MM_SRV_GET_ATTRIBUTES = 0x02,
	RPMI_MM_SRV_COMMUNICATE = 0x03,
	RPMI_MM_SRV_MAX_COUNT,
};

/** RPMI MM ServiceGroup Get Attributes main struct */
struct rpmi_mm_attributes {
	u32 mm_version;
	u32 shmem_addr_lo;
	u32 shmem_addr_hi;
	u32 shmem_size;
};

/** RPMI MM ServiceGroup Get Attributes response struct */
struct rpmi_mm_get_attributes_rsp {
	s32 status;
	struct rpmi_mm_attributes mma;
};

/** RPMI MM ServiceGroup Communicate request struct */
struct rpmi_mm_communicate_req {
	u32 mm_comm_ipdata_off;
	u32 mm_comm_ipdata_size;
	u32 mm_comm_opdata_off;
	u32 mm_comm_opdata_size;
};

/** RPMI MM ServiceGroup Communicate response struct */
struct rpmi_mm_communicate_rsp {
	s32 status;
	u32 mm_comm_retdata_size;
};

/** RPMI TEE ServiceGroup Service IDs (RPMI spec section 4.16) */
enum rpmi_tee_service_id {
	RPMI_TEE_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_TEE_SRV_PROBE_FEATURES = 0x02,
	RPMI_TEE_SRV_PROBE_SYSTEM = 0x03,
	RPMI_TEE_SRV_SIGNAL_BUS_SETUP = 0x05,
	RPMI_TEE_SRV_SIGNAL_BUS_TEARDOWN = 0x06,
	RPMI_TEE_SRV_SIGNAL_RAISE = 0x07,
	RPMI_TEE_SRV_SIGNAL_RETRIEVE = 0x08,
	RPMI_TEE_SRV_MEM_PARCEL_CREATE = 0x09,
	RPMI_TEE_SRV_MEM_PARCEL_ACCEPT = 0x0A,
	RPMI_TEE_SRV_MEM_PARCEL_RELEASE = 0x0B,
	RPMI_TEE_SRV_MEM_PARCEL_RECLAIM = 0x0C,
	RPMI_TEE_SRV_MEM_PARCEL_SEGMENT_SEND = 0x0D,
	RPMI_TEE_SRV_MEM_PARCEL_SEGMENT_RECEIVE = 0x0E,
	RPMI_TEE_SRV_TEE_CALL = 0x13,
	RPMI_TEE_SRV_MAX_COUNT,
};

/** RPMI TEE feature IDs for TEE_PROBE_FEATURES (RPMI spec section 4.16, Tables 182-184) */
enum rpmi_tee_feature_id {
	RPMI_TEE_FEAT_MEMORY_DONATE = 1,
	RPMI_TEE_FEAT_MEMORY_LEND = 2,
	RPMI_TEE_FEAT_MEMORY_SHARE = 3,
	RPMI_TEE_FEAT_SIGNAL_BUS = 4,
	RPMI_TEE_FEAT_MULTISEGMENT_OPS = 5,
	RPMI_TEE_FEAT_SYSINFO_FORMAT = 6,
};

/* TEE_PROBE_FEATURES support level reported in the response VALUE (Table 184) */
#define RPMI_TEE_FEAT_VAL_NONE		0	/* not supported */
#define RPMI_TEE_FEAT_VAL_TEE_ONLY	1	/* supported TEE-side only */
#define RPMI_TEE_FEAT_VAL_FULL_REE_TEE	2	/* supported for both REE and TEE */

/** TEE_PROBE_FEATURES request */
struct rpmi_tee_probe_features_req {
	u32 feature_id;
};

/** TEE_PROBE_FEATURES response */
struct rpmi_tee_probe_features_resp {
	s32 status;
	u32 value;
};

/*
 * SIGNAL_BUS feature (RPMI_TEE_FEAT_SIGNAL_BUS) value encoding reported by
 * TEE_PROBE_FEATURES (RPMI spec section 4.16, signal bus):
 *   [1:0]   delivery mode: 0 = not supported, 1 = System MSI, 2 = System IRQ
 *   [11:2]  max bus width (max concurrent signals per endpoint pair)
 *   [31:12] System MSI index or System IRQ index used to notify availability
 */
#define RPMI_TEE_SIGNAL_DELIVERY_NONE	0
#define RPMI_TEE_SIGNAL_DELIVERY_MSI	1
#define RPMI_TEE_SIGNAL_DELIVERY_IRQ	2
#define RPMI_TEE_SIGNAL_BUS_VALUE(idx, width, mode) \
	((((u32)(idx)) << 12) | ((((u32)(width)) & 0x3ff) << 2) | \
	 (((u32)(mode)) & 0x3))

/** TEE_SIGNAL_BUS_SETUP request (RPMI spec section 4.16.7, Table 190) */
struct rpmi_tee_signal_bus_setup_req {
	u32 target_id;
	u32 bus_width;		/* M: total signals on the bus */
	u32 sender_signals;	/* N: signals reserved for sender to receive */
};

/** TEE_SIGNAL_BUS_SETUP response (Table 191) */
struct rpmi_tee_signal_bus_setup_resp {
	s32 status;
};

/** TEE_SIGNAL_BUS_TEARDOWN request (section 4.16.8, Table 192) */
struct rpmi_tee_signal_bus_teardown_req {
	u32 target_id;
};

/** TEE_SIGNAL_BUS_TEARDOWN response (Table 193) */
struct rpmi_tee_signal_bus_teardown_resp {
	s32 status;
};

/** TEE_SIGNAL_RAISE request (section 4.16.9, Table 194) */
struct rpmi_tee_signal_raise_req {
	u32 target_id;
	u32 signal_len;		/* N: length of signal[]; cannot be 0 */
	u32 signal[];		/* signals to set pending */
};

/** TEE_SIGNAL_RAISE response (Table 195) */
struct rpmi_tee_signal_raise_resp {
	s32 status;
};

/*
 * TEE_SIGNAL_RETRIEVE (section 4.16.10): request has no parameters.
 * Response reports, per bus, the active signals readable by the caller.
 */
#define RPMI_TEE_SIGNAL_RETRIEVE_MORE_AVAILABLE	(1U << 31)
struct rpmi_tee_signal_retrieve_resp {
	s32 status;
	u32 flags;		/* bit31 MORE_AVAILABLE; bits30:0 reserved 0 */
	u32 target_id;
	u32 signal_len;		/* N: length of signal[]; nonzero on success */
	u32 signal[];		/* active signals on this bus */
};

/*
 * Signal index used by the async-notif doorbell on the width-1 REE<->TEE bus:
 * OP-TEE raises signal 0, the REE reads it and drains via GET_ASYNC_NOTIF_VALUE.
 */
#define RPMI_TEE_SIGNAL_ASYNC_NOTIF	0

/*
 * SYSINFO_FORMAT values reported by TEE_PROBE_FEATURES for the SYSINFO_FORMAT
 * feature: the encoding used by the PROBE_SYSTEM system-info blob. 0 = none.
 */
#define RPMI_TEE_SYSINFO_FORMAT_NONE	0
#define RPMI_TEE_SYSINFO_FORMAT_CBOR	1

/** TEE_PROBE_SYSTEM request (no parameters) */
struct rpmi_tee_probe_system_req {
	u32 reserved;
};

/*
 * TEE_PROBE_SYSTEM response: fixed header + a variable-length system-info blob
 * encoded per the SYSINFO_FORMAT feature value (CBOR here). info_len is the
 * number of valid bytes in data[]; format echoes the encoding.
 */
struct rpmi_tee_probe_system_resp {
	s32 status;
	u32 format;
	u32 info_len;
	u8 data[];
};

/** TEE Implementation IDs */
enum rpmi_tee_impl_id {
	RPMI_TEE_IMPL_ID_OPTEE = 0x00000000,
	/* 0x00000001 - 0x7FFFFFFF: Reserved for future use */
	/* 0x80000000 - 0xFFFFFFFF: Implementation specific */
};

/** OP-TEE specific communication parameters */
#define RPMI_TEE_OPTEE_COMM_REQ_REGS	8	/* a0-a7 */
#define RPMI_TEE_OPTEE_COMM_RESP_REGS	4	/* a0-a3 */

/**
 * Fixed TEE endpoint identities for the Track-1 prototype (RPMI spec section 4.16).
 * A single static REE endpoint invokes a single static OP-TEE endpoint.
 */
#define RPMI_TEE_ENDPOINT_REE		0
#define RPMI_TEE_ENDPOINT_OPTEE		1

/**
 * Well-known SERVICE UUID identifying the "OP-TEE communicate" service whose
 * SERVICE_DATA carries the SMC-style a0-a7 register block. This is a fixed,
 * prototype-local UUID (not an assigned GP/OP-TEE UUID); OpenSBI and the Linux
 * conduit must agree on these 16 bytes verbatim.
 *
 * UUID: 5be1b1a0-7e11-4e7a-9b10-0010c0ffee00
 */
#define RPMI_TEE_OPTEE_SERVICE_UUID { \
	0x5b, 0xe1, 0xb1, 0xa0, 0x7e, 0x11, 0x4e, 0x7a, \
	0x9b, 0x10, 0x00, 0x10, 0xc0, 0xff, 0xee, 0x00 }

/** TEE_CALL request (RPMI spec section 4.16, Table 218) */
struct rpmi_tee_call_req {
	u32 sender_id;
	u32 target_id;
	u8 service[16];
	u32 service_data_len;
	u8 service_data[];
};

/** TEE_CALL response (RPMI spec section 4.16, Table 219) */
struct rpmi_tee_call_resp {
	s32 status;
	u32 service_rsp_len;
	u8 service_rsp[];
};

/** TEE_GET_ATTRIBUTES response */
struct rpmi_tee_get_attributes_resp {
	s32 status;
	u32 tee_impl_id;
	u32 comm_req_regs;
	u32 comm_resp_regs;
};

/*
 * Memory parcel wire encodings (RPMI spec section 4.16, Tables 198-207).
 *
 * A memory parcel describes memory as a scatter-gather block list plus
 * per-receiver access rights. All fields are little-endian uint32 words on the
 * wire. Addresses in the block list are expressed in units of 4kB pages, not
 * bytes.
 */

/* Memory access encoding (Table 199) */
#define RPMI_TEE_PARCEL_ACCESS_R	(1U << 29)
#define RPMI_TEE_PARCEL_ACCESS_W	(1U << 30)
#define RPMI_TEE_PARCEL_ACCESS_X	(1U << 31)
#define RPMI_TEE_PARCEL_ACCESS_MASK	(RPMI_TEE_PARCEL_ACCESS_R | \
					 RPMI_TEE_PARCEL_ACCESS_W | \
					 RPMI_TEE_PARCEL_ACCESS_X)

/* MEM_PARCEL_CREATE FLAGS (Table 200) */
#define RPMI_TEE_PARCEL_CREATE_FLAG_MULTI_SEGMENT	(1U << 31)
#define RPMI_TEE_PARCEL_CREATE_FLAG_OWNER_XFER		(1U << 30)

/* MEM_PARCEL_ACCEPT response FLAGS (Table 203) */
#define RPMI_TEE_PARCEL_ACCEPT_RESP_FLAG_MULTI_SEGMENT	(1U << 31)

/* MEM_PARCEL_RECLAIM response FLAGS (Table 207) */
#define RPMI_TEE_PARCEL_RECLAIM_RESP_FLAG_ZEROED	(1U << 31)

/*
 * Block list encoding (Table 198): addresses are in 4kB page units.
 *   page number = (BLOCK_HIGH << 20) | (BLOCK_LOW >> 12)
 *   page count  = (BLOCK_LOW & 0xFFF) + 1   (range 1..4096)
 */
#define RPMI_TEE_PARCEL_BLOCK_PAGES(low)	(((low) & 0xFFFU) + 1)
#define RPMI_TEE_PARCEL_BLOCK_PAGE_NUM(high, low) \
	(((u64)(u32)(high) << 20) | ((u32)(low) >> 12))
#define RPMI_TEE_PARCEL_BLOCK_LABEL_LEN		16

/*
 * MEM_PARCEL_CREATE request (Table 200). Fixed header, then four back-to-back
 * variable-length uint32 arrays accessed via computed offsets into data[]:
 *   receiver_id[receiver_cnt], access[receiver_cnt],
 *   block_high[block_cnt], block_low[block_cnt]
 */
struct rpmi_tee_mem_parcel_create_req {
	u32 creator_id;
	u32 creator_access;
	u32 receiver_cnt;
	u32 flags;
	u32 nonce;
	u32 block_cnt;
	u8 label[RPMI_TEE_PARCEL_BLOCK_LABEL_LEN];
	u32 data[];
};

struct rpmi_tee_mem_parcel_create_resp {
	s32 status;
	u32 mem_parcel_id;
};

/*
 * MEM_PARCEL_ACCEPT request (Table 202). Fixed header, then two back-to-back
 * variable-length uint32 arrays in data[]: other_id[other_cnt],
 * other_access[other_cnt].
 */
struct rpmi_tee_mem_parcel_accept_req {
	u32 acceptor_id;
	u32 access;
	u32 mem_parcel_id;
	u32 nonce;
	u32 creator_id;
	u32 creator_access;
	u32 flags;
	u32 address_high;
	u32 address_low;
	u32 max_pages;
	u32 other_cnt;
	u32 data[];
};

/*
 * MEM_PARCEL_ACCEPT response (Table 203). Fixed header, then the returned
 * block list in data[]: block_high[block_cnt], block_low[block_cnt].
 */
struct rpmi_tee_mem_parcel_accept_resp {
	s32 status;
	u32 flags;
	u32 page_cnt;
	u32 block_cnt;
	u32 data[];
};

/* MEM_PARCEL_RELEASE request (Table 204) */
struct rpmi_tee_mem_parcel_release_req {
	u32 mem_parcel_id;
	u32 flags;
	u32 endpoint_cnt;
	u32 endpoint_id[];
};

struct rpmi_tee_mem_parcel_release_resp {
	s32 status;
};

/* MEM_PARCEL_RECLAIM request (Table 206) */
struct rpmi_tee_mem_parcel_reclaim_req {
	u32 mem_parcel_id;
};

struct rpmi_tee_mem_parcel_reclaim_resp {
	s32 status;
	u32 flags;
};

/*
 * SEGMENT_SEND (0x0D) / SEGMENT_RECEIVE (0x0E): stream a parcel block list that
 * does not fit a single RPMI message. The transport slot bounds each message to
 * RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) bytes, so a block list wider than one
 * message is split into fixed-cap segments carried by these two services.
 *
 * SEGMENT_SEND appends block-list segments to a parcel created with the
 * MULTI_SEGMENT flag (state "constructing"); the segment carrying the LAST flag
 * finalizes the parcel to the created state. SEGMENT_RECEIVE lets an acceptor
 * pull the block list back in segments after an ACCEPT whose response set the
 * MULTI_SEGMENT flag (i.e. could not return every block in one response).
 */
#define RPMI_TEE_PARCEL_SEGMENT_FLAG_LAST	(1U << 31)
#define RPMI_TEE_PARCEL_SEGMENT_MAX_BLOCKS	4

/* SEGMENT_SEND request: header + block_high[block_cnt] block_low[block_cnt]. */
struct rpmi_tee_mem_parcel_segment_send_req {
	u32 mem_parcel_id;
	u32 flags;
	u32 block_cnt;
	u32 data[];
};

struct rpmi_tee_mem_parcel_segment_send_resp {
	s32 status;
};

/* SEGMENT_RECEIVE request: pull the next segment for an in-progress accept. */
struct rpmi_tee_mem_parcel_segment_receive_req {
	u32 acceptor_id;
	u32 mem_parcel_id;
};

/* SEGMENT_RECEIVE response: header + block_high[block_cnt] block_low[block_cnt]. */
struct rpmi_tee_mem_parcel_segment_receive_resp {
	s32 status;
	u32 flags;
	u32 block_cnt;
	u32 data[];
};

/** RPMI Request Forward ServiceGroup Service IDs */
enum rpmi_reqfwd_service_id {
	RPMI_REQFWD_SRV_ENABLE_NOTIFICATION = 0x01,
	RPMI_REQFWD_SRV_RETRIEVE_CURRENT_MESSAGE = 0x02,
	RPMI_REQFWD_SRV_COMPLETE_CURRENT_MESSAGE = 0x03,
	RPMI_REQFWD_SRV_MAX_COUNT,
};

struct rpmi_reqfwd_enable_notification_req {
	u32 event_id;
	u32 req_state;
};

struct rpmi_reqfwd_enable_notification_resp {
	s32 status;
	u32 current_state;
};

struct rpmi_reqfwd_retrieve_current_message_req {
	u32 start_index;
};

struct rpmi_reqfwd_retrieve_current_message_resp {
	s32 status;
	u32 remaining;
	u32 returned;
	/* remaining space need to be adjusted for the above 3 u32's */
	u8 request_message[RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN) - (sizeof(u32) * 3)];
};

struct rpmi_reqfwd_complete_current_message_req {
	u8 response_data[RPMI_MSG_DATA_SIZE(RPMI_SLOT_SIZE_MIN)];
};

struct rpmi_reqfwd_complete_current_message_resp {
	s32 status;
	u32 num_messages;
};

#endif /* !__RPMI_MSGPROT_H__ */
