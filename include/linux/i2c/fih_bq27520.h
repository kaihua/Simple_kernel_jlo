/* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved. */

#ifndef __LINUX_BQ27520_H
#define __LINUX_BQ27520_H
struct bq27520_platform_data {
	const char *name;
	unsigned int soc_int;
	unsigned int bi_tout;
	unsigned int chip_en; /* CE */
	const char *vreg_name; /* regulater used by bq27520 */
	int vreg_value; /* its value */
	int enable_dlog; /* if enable on-chip coulomb counter data logger */
    unsigned int bat_low;
    unsigned int ovp_flag;
};

enum _battery_info_type_ {
    BATT_CAPACITY_INFO,         // 0x06 RARC - Remaining Active Relative Capacity
    BATT_VOLTAGE_INFO,          // 0x0c Voltage MSB
    BATT_TEMPERATURE_INFO,      // 0x0a Temperature MSB
    BATT_CURRENT_INFO,          // 0x0e Current MSB
    BATT_AVCURRENT_INFO,        // 0x08 Average Current MSB
    BATT_STATUS_INFO,           // 0x01 Status
    BATT_ACCCURRENT_INFO,       // 0x10 Accumulated Current Register MSB
    BATT_AGE_SCALAR_INFO,       // 0x14 Age Scalar
    BATT_FULL_INFO,             // 0x16 Full Capacity MSB
    BATT_FULL40_INFO,           // 0x6A Full40 MSB
    BATT_FAMILY_CODE,           // 0x33
    BATT_GET_RSNS,              // 0x69
    BATT_ACTIVE_EMPTY_INFO,
    BATT_STANDBY_EMPTY_INFO,
    BATT_ACTIVE_EMPTY_VOLTAGE_INFO,
    BATT_GET_MANUFACTURER_INFO,
    BATT_GET_DEVICE_TYPE,
    BATT_GET_VCOIN,
};

#endif /* __LINUX_BQ27520_H */
