/*
 * Copyright (C) 2008 HTC, Inc.
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/switch.h>

struct h2w_platform_data {
	char *power_name;
	int cable_in1;
	int cable_in2;
	int h2w_clk;
	int h2w_data;
	int debug_uart;
	void (*config_cpld)(int);
	void (*init_cpld)(void);
	void (*set_dat)(int);
	void (*set_clk)(int);
	void (*set_dat_dir)(int);
	void (*set_clk_dir)(int);
	int (*get_dat)(void);
	int (*get_clk)(void);
};

struct h2w_info {
	struct switch_dev sdev;
	struct input_dev *input;    /* Headset */
	struct input_dev *hs_input; /* Headset HOOK Key */

	atomic_t hs_state;
	atomic_t btn_state;
  
	int ignore_btn;

	int irq;
	int irq_btn;

	int cable_in1;
	int cable_in2;

	struct hrtimer timer;
	ktime_t debounce_time;

	struct hrtimer btn_timer;
	ktime_t btn_debounce_time;
	
};

