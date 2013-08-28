/* Header file for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp_core.h
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


#ifndef __CYTTSP_CORE_H__
#define __CYTTSP_CORE_H__

#include <linux/kernel.h>

struct cyttsp_bus_ops {
	s32 (*write)(void *handle, u8 addr, u8 length, const void *values);
	s32 (*read)(void *handle, u8 addr, u8 length, void *values);
	s32 (*ext)(void *handle, void *values);
};

void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops, struct device *pdev);
void cyttsp_core_release(void *handle);

/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00+{ */
#define DEBUG 0
#if DEBUG
#define DBG_MSG(...)	printk(KERN_DEBUG "[CYPRESS_T340_DBG]" __VA_ARGS__)
#else
/*FIH-SW3-PERIPHERAL-CH-TAP-02++[*/
#define DBG_MSG(...)	do { \
        if (0 < TMA340_debug_level) \
            printk("[CYPRESS_T340_DBG]" __VA_ARGS__); \
    } while (0)
/*FIH-SW3-PERIPHERAL-CH-TAP-02++]*/
#endif
#define DBG_INFO(...)	printk(KERN_INFO "[CYPRESS_T340_INFO]" __VA_ARGS__)
#define DBG_WARN(...)	printk(KERN_WARN "[CYPRESS_T340_WARN]" __VA_ARGS__) 
#define DBG_ERR(...)	printk(KERN_ERR "[CYPRESS_T340_ERR]" __VA_ARGS__) 
/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00+} */

static int TMA340_debug_level=0;
module_param(
    TMA340_debug_level, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#ifndef AUTO_CALIB
#define AUTO_CALIB /*owenhuang+*/
#endif

#ifdef AUTO_CALIB
#define TEST_REG_BIT(reg, x) (reg & (1<<x))
#define HST_MODE	0x00
#define MFG_STAT	0x01
#define MFG_CMD 	0x02
#define MFG_REG0	0x03
#define MFG_REG1	0x04
#endif

/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#define TEST_MODE	/*FIH-SW3-PERIPHERAL-CH-TAP-03++*/
#ifdef TEST_MODE
#define TEST_MODE_DELAY_TIME	200 /* 200ms default*/

/*Test Mode Register*/
#define HST_MODE_FOR_TEST_MODE_IDAC	0x60 /*(0110_0000b)*/
#define HST_MODE_FRO_TEST_MODE_RAWS	0x70 /*(0111_0000b)*/
#define HST_MODE_FOR_TEST_MODE_DIFF	0x50 /*(0101_0000b) FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++*/

#define CHECK_TEST_ENABLE(x)	((x>>4) & 0xF)
#define GET_NEW_DATA_COUNTER(x) (x>>6)
/*Test mode type*/
#define EXIT_TEST_MODE 			0x00
#define TMODE_IDAC_SETTING		0x10
#define TMODE_IDAC_RAW_BASELINE	0x20
#define TMODE_DIFF_COUNTS		0x30 /*FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++*/
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	

#endif /* __CYTTSP_CORE_H__ */
