/* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.  */

struct msm_battery_gauge {
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*is_battery_present) (void);
	int (*is_battery_temp_within_range) (void);
	int (*is_battery_id_valid) (void);
	int (*get_battery_status)(void);
	int (*get_batt_remaining_capacity) (void);
	int (*get_battery_flags)(void);
};

enum {
    BATTERY_TEST_CP,
    BATTERY_TEST_TEMPERATURE,
    BATTERY_TEST_NUMS,  
};

