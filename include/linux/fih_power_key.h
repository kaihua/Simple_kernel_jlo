/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef FIH_POWER_KEY_H
#define FIH_POWER_KEY_H

#define	SET_INTERVAL	0

struct msm_handset_platform_data {
	const char *hs_name;
	uint32_t pwr_key_delay_ms; /* default 500ms */
	unsigned int	interval;
};

struct fih_power_key_data
{
	unsigned long	tick_time;
	unsigned int	interval;
	unsigned int	set_interval : 1;
};

void report_headset_status(bool connected);

#endif
