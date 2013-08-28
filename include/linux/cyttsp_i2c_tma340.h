/* Header file for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * include/linux/cyttsp.h
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 * Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */
#include <linux/input.h>

#ifndef _CYTTSP_H_
#define _CYTTSP_H_

#include <linux/input.h>

#define CY_I2C_NAME "cyttsp_i2c_tma340"
/* Scan Type selection for finger and/or stylus activation */
#define CY_SCN_TYP_DFLT 0x01 /* finger only mutual scan */
/* Active Power state scanning/processing refresh interval */
#define CY_ACT_INTRVL_DFLT 0x00 /* ms */
/* touch timeout for the Active power */
#define CY_TCH_TMOUT_DFLT 0x64 /* ms */
/* Low Power state scanning/processing refresh interval */
#define CY_LP_INTRVL_DFLT 0x32 /* ms */
/*
 *defines for Gen2 (Txx2xx); Gen3 (Txx3xx)
 * use these defines to set cyttsp_platform_data.gen in board config file
 */
enum cyttsp_gen {
	CY_GEN2,
	CY_GEN3,
};
/*
 * Active distance in pixels for a gesture to be reported
 * if set to 0, then all gesture movements are reported
 * Valid range is 0 - 15
 */
#define CY_ACT_DIST_DFLT 8
#define CY_ACT_DIST CY_ACT_DIST_DFLT
/* max retries for read/write ops */
#define CY_NUM_RETRY 10

#define CY_NUM_RETERY_SUSPEND 3 /*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/

/*FIH-MTD-PERIPHERAL-CH-Add_command-00++[*/
#define SECOND_MODULE
#define reset_cmd
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++]*/
/*#define FIH_VIRTUAL_BUTTON*/ /*FIH-MTD-PERIPHERAL-CH-KEY_AREA-00++*/
#define CYTTSP_CM_bit /*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++*/

/*FIH-MTD-PERIPHERAL-CH-MES-02++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_MES
#define CY_I2C_IRQ_GPIO ((fih_get_product_phase() < Phase_DP) ? 40:86)
#else
#define CY_I2C_IRQ_GPIO 86
#endif
/*FIH-MTD-PERIPHERAL-CH-MES-02++]*/
enum cyttsp_gest {
	CY_GEST_GRP_NONE = 0,
	CY_GEST_GRP1 =	0x10,
	CY_GEST_GRP2 = 0x20,
	CY_GEST_GRP3 = 0x40,
	CY_GEST_GRP4 = 0x80,
};

/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++[*/
enum cyttsp_powerstate {
	CY_IDLE_STATE,
	CY_ACTIVE_STATE,
	CY_LOW_PWR_STATE,
	CY_SLEEP_STATE,
	CY_RESET_STATE,
};
/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++]*/
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
enum cyttsp_esd_workaround {
	CY_FACE_DOWN=0,
	CY_FACE_UP,
	CY_PLUG_USB,
	CY_UNPLUG_USB,
	CY_PLUG_HS,
	CY_UNPLUG_HS,
};
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

struct cyttsp_platform_data {
	u32 maxx;
	u32 maxy;
	u32 flags;
	enum cyttsp_gen gen;
	unsigned use_st:1;
	unsigned use_mt:1;
	unsigned use_trk_id:1;
	unsigned use_hndshk:1;
	unsigned use_timer:1;
	unsigned use_sleep:1;
	unsigned use_gestures:1;
	unsigned use_load_file:1;
	unsigned use_force_fw_update:1;
	unsigned use_virtual_keys:1;
	enum cyttsp_powerstate power_state;
	u8 gest_set;
	u8 scn_typ;     /* finger and/or stylus scanning */
	u8 act_intrvl;  /* Active refresh interval; ms */
	u8 tch_tmout;   /* Active touch timeout; ms */
	u8 lp_intrvl;   /* Low power refresh interval; ms */
	int (*wakeup)(void);
	int (*init)(int on_off);
	void (*mt_sync)(struct input_dev *);
#ifdef reset_cmd /*FIH-MTD-PERIPHERAL-CH-Add_command-00++*/
	int (*reset)(void);
#endif
#ifdef SECOND_MODULE
	char *module_1; /*FIH-MTD-PERIPHERAL-CH-Add_command-00++*/
	char *module_2; /*FIH-MTD-PERIPHERAL-CH-Add_command-00++*/
#endif
	char *name;
	s16 irq_gpio;
	/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
	unsigned row_pins_number:14;
	unsigned col_pins_number:10;
	/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	
};
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
int TOUCH_ESD_WORKAROUND(int enable);
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

#endif /* _CYTTSP_H_ */
