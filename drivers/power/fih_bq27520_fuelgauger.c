/* 
 * Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved. 
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/i2c/fih_bq27520.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/fih-msm-charger.h>
#include <mach/proc_comm.h>
#include "../../arch/arm/mach-msm/smd_private.h"
#include <linux/wakelock.h>
#include "fih_chg_ipc.h"
#include "fih_adc_msg.h"

#define DRIVER_VERSION			"1.1.0"
/* Bq27520 standard data commands */
#define BQ27520_REG_CNTL		0x00
#define BQ27520_REG_AR			0x02
#define BQ27520_REG_ARTTE		0x04
#define BQ27520_REG_TEMP		0x06
#define BQ27520_REG_VOLT		0x08
#define BQ27520_REG_FLAGS		0x0A
#define BQ27520_REG_NAC			0x0C
#define BQ27520_REG_FAC			0x0e
#define BQ27520_REG_RM			0x10
#define BQ27520_REG_FCC			0x12
#define BQ27520_REG_AI			0x14
#define BQ27520_REG_TTE			0x16
#define BQ27520_REG_TTF			0x18
#define BQ27520_REG_SI			0x1a
#define BQ27520_REG_STTE		0x1c
#define BQ27520_REG_MLI			0x1e
#define BQ27520_REG_MLTTE		0x20
#define BQ27520_REG_AE			0x22
#define BQ27520_REG_AP			0x24
#define BQ27520_REG_TTECP		0x26
#define BQ27520_REG_SOH			0x28
#define BQ27520_REG_SOH_STATUS		0x29
#define BQ27520_REG_SOC			0x2c
#define BQ27520_REG_NIC			0x2e
#define BQ27520_REG_ICR			0x30
#define BQ27520_REG_LOGIDX		0x32
#define BQ27520_REG_LOGBUF		0x34
#define BQ27520_FLAG_DSC		BIT(0)
#define BQ27520_FLAG_FC			BIT(9)
#define BQ27520_FLAG_BAT_DET		BIT(3)
#define BQ27520_CS_DLOGEN		BIT(15)
#define BQ27520_CS_SS		    BIT(13)
#define BQ27520_CS_SNOOZE	    BIT(5)
#define BQ27520_CS_OCVCMDCOMP 	BIT(9)
#define BQ27520_CS_INITCOMP	    BIT(7)
#define BQ27520_CS_QEN		    BIT(0)


/* Control subcommands */
#define BQ27520_SUBCMD_CTNL_STATUS  0x0000
#define BQ27520_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27520_SUBCMD_FW_VER  0x0002
#define BQ27520_SUBCMD_HW_VER  0x0003
#define BQ27520_SUBCMD_DF_CSUM  0x0004
#define BQ27520_SUBCMD_PREV_MACW   0x0007
#define BQ27520_SUBCMD_CHEM_ID   0x0008
#define BQ27520_SUBCMD_BD_OFFSET   0x0009
#define BQ27520_SUBCMD_INT_OFFSET  0x000a
#define BQ27520_SUBCMD_CC_VER   0x000b
#define BQ27520_SUBCMD_OCV  0x000c
#define BQ27520_SUBCMD_BAT_INS   0x000d
#define BQ27520_SUBCMD_BAT_REM   0x000e
#define BQ27520_SUBCMD_SET_HIB   0x0011
#define BQ27520_SUBCMD_CLR_HIB   0x0012
#define BQ27520_SUBCMD_SET_SLP   0x0013
#define BQ27520_SUBCMD_CLR_SLP   0x0014
#define BQ27520_SUBCMD_FCT_RES   0x0015
#define BQ27520_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27520_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27520_SUBCMD_SEALED   0x0020
#define BQ27520_SUBCMD_ENABLE_IT    0x0021
#define BQ27520_SUBCMD_DISABLE_IT   0x0023
#define BQ27520_SUBCMD_CAL_MODE  0x0040
#define BQ27520_SUBCMD_RESET   0x0041

#ifdef CONFIG_FIH_SW3_BQ275X0_ROMMODE
#define BQ275X0_EXT_CMD_DCAP        0x3C
#define BQ275X0_EXT_CMD_DFCLS       0x3E
#define BQ275X0_EXT_CMD_DFBLK       0x3F
#define BQ275X0_EXT_CMD_DFD         0x40
#define BQ275X0_EXT_CMD_DFDCKS      0x60
#define BQ275X0_EXT_CMD_DFDCNTL     0x61
#define BQ275X0_EXT_CMD_DNAMELEN    0x62
#define BQ275X0_EXT_CMD_DNAME       0x63
#define BQ275X0_EXT_CMD_APPSTAT     0x6A

#define BQ27520_SUBCMD_ROM_MODE 0x0F00
#define FAILED_MARK             0xFFFFFFFF
enum {
    BQ275X0_CNTLSTATUS_QEN,
    BQ275X0_CNTLSTATUS_VOK,
    BQ275X0_CNTLSTATUS_RUP_DIS,
    BQ275X0_CNTLSTATUS_LDMD,
    BQ275X0_CNTLSTATUS_SLEEP,
    BQ275X0_CNTLSTATUS_SNOOZE,
    BQ275X0_CNTLSTATUS_HIBERNATE,
    BQ275X0_CNTLSTATUS_BIT7,
    BQ275X0_CNTLSTATUS_BIT8,
    BQ275X0_CNTLSTATUS_BIT9,
    BQ275X0_CNTLSTATUS_BCA,
    BQ275X0_CNTLSTATUS_CCA,
    BQ275X0_CNTLSTATUS_CSV,
    BQ275X0_CNTLSTATUS_SS,
    BQ275X0_CNTLSTATUS_FAS,
    BQ275X0_CNTLSTATUS_BIT15,
};
#endif

#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27520_INIT_DELAY ((HZ)*1)
#define BQ27520_POLLING_STATUS ((HZ)*30)
#define BQ27520_COULOMB_POLL ((HZ)*30)
#define BQ27520_POLLING_STATUS_DISCHARGING ((HZ)*60)

#ifndef ABS
  #define ABS(x)            (((x) < 0) ? -(x) : (x))
#endif

enum {
	BQ275X0_MODEM_CHG_LOG = 1U << 0,
	BQ275X0_MODEM_ADC_LOG = 1U << 1,		
	BQ275X0_EXTRA_DEBUG_LOG = 1U << 2,
};

#ifdef GAUGE_ENABLE_EXTRA_LOG
  static int msm_batt_debug_mask = (BQ275X0_MODEM_CHG_LOG|BQ275X0_EXTRA_DEBUG_LOG);
#else
  static int msm_batt_debug_mask = (BQ275X0_MODEM_CHG_LOG);
#endif
module_param_named(
	debug_mask, msm_batt_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27520_device_info;
struct bq27520_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27520_device_info *di);
};

struct bq27520_device_info {
	struct device				*dev;
	int					id;
	struct bq27520_access_methods		*bus;
	struct i2c_client			*client;
	const struct bq27520_platform_data	*pdata;
	struct work_struct			counter;
	/* 300ms delay is needed after bq27520 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work			hw_config;
	uint32_t				soc_irq;
	uint32_t				bat_low_irq;
	uint32_t				ovp_irq;  
	bool				rommode;
	struct				wake_lock bq27520_wakelock;
	chg_ipc_smem *pChgIpcInfo;
	adc_ipc_smem *pAdcIpcInfo;
	struct delayed_work bq275x0_BATLOW;
	struct delayed_work ovp_work;
};

enum {
	GET_BATTERY_STATUS,
	GET_BATTERY_TEMPERATURE,
	GET_BATTERY_VOLTAGE,
	GET_BATTERY_CAPACITY,
	GET_BATTERY_CURRENT,
	GET_BATTERY_FLAGS,	
	GET_BATTERY_SOH,
	GET_BATTERY_SOH_STATUS,	
	NUM_OF_STATUS,
};

struct bq27520_status {
	/* Informations owned and maintained by Bq27520 driver, updated
	 * by poller or SOC_INT interrupt, decoupling from I/Oing
	 * hardware directly
	 */
	int			status[NUM_OF_STATUS];
	spinlock_t		lock;
	struct delayed_work	poller;
};

static struct bq27520_status current_battery_status;
static struct bq27520_device_info *bq27520_di;
static int coulomb_counter;
static spinlock_t lock; /* protect access to coulomb_counter */
static struct timer_list timer; /* charge counter timer every 30 secs */

/* FIH-SW3-KERNEL-VH-CHARGING-13*[ */
int bq27520_isEnableBatteryCheck = 0;
static int bq27520_polling_timeout = BQ27520_POLLING_STATUS;
extern void msm_batt_update_psy_status(void);
/* FIH-SW3-KERNEL-VH-CHARGING-13*] */

extern void bq275x0_flash_recovery(bool reflash);

extern void msmrtc_set_wakeup_cycle_time(int cycle_time);
static int bq27520_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27520_device_info *di);

static int bq27520_read(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
    int retVal = 0;
    retVal = di->bus->read(reg, rt_value, b_single, di);
    
    if (retVal != 0)
        dev_err(di->dev, "bq27520_read(%d) failed, retVal = %d\n", reg, retVal);
    
    mdelay(1);

    return retVal;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27520_battery_temperature(struct bq27520_device_info *di)
{
	int ret, temp = 0;

	ret = bq27520_read(BQ27520_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error %d reading temperature\n", ret);
		return ret;
	}

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

#if (!defined(CONFIG_FIH_FTM) && defined(CONFIG_FIH_SW1_FEATURE_BATTERY_THERMAL_VIA_PMIC))
/*
 * Write the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27520_battery_set_temperature(struct bq27520_device_info *di, int tempC)
{
	int retVal;
	int temp = 0;

	temp = tempC - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;

	retVal = bq27520_i2c_txsubcmd(BQ27520_REG_TEMP, temp, di);
	if (retVal != 0) {
		dev_err(di->dev,
			"set temperature %d failed, retVal = %d\n", temp, retVal);
	}

	return retVal;
}
#endif

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27520_battery_voltage(struct bq27520_device_info *di)
{
	int ret, volt = 0;

	ret = bq27520_read(BQ27520_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error %d reading voltage\n", ret);
		return ret;
	}

	return volt;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27520_battery_rsoc(struct bq27520_device_info *di)
{
	int ret, rsoc = 0;

	ret = bq27520_read(BQ27520_REG_SOC, &rsoc, 0, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading relative State-of-Charge\n", ret);
		return ret;
	}

	return rsoc;
}

/*
 * Return the battery Relative SOH
 * Or < 0 if something fails.
 */
static int bq27520_battery_flags(struct bq27520_device_info *di)
{
	int ret, flags = 0;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 1, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading flag\n", ret);
		return ret;
	}

	return flags;
}

/*
 * Return the battery SOH
 * Or < 0 if something fails.
 */
static int bq27520_battery_SOH(struct bq27520_device_info *di)
{
	int ret, SOH = 0;

	ret = bq27520_read(BQ27520_REG_SOH, &SOH, 1, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading SOH\n", ret);
		return ret;
	}

	return SOH;
}

/*
 * Return the battery SOH status
 * Or < 0 if something fails.
 */
static int bq27520_battery_SOH_status(struct bq27520_device_info *di)
{
	int ret, SOH_status = 0;

	ret = bq27520_read(BQ27520_REG_SOH_STATUS, &SOH_status, 1, di);

	if (ret) {
		dev_err(di->dev,
			"error %d reading SOH status\n", ret);
		return ret;
	}

	return SOH_status;
}

/*
 * Return the extra battery debug message.
 */
static int bq27520_battery_extra_debug_msg(struct bq27520_device_info *di)
{
	int ret, NAC, FAC, RM, FCC, NIC, CTNL_STATUS;

	ret = bq27520_read(BQ27520_REG_NAC, &NAC, 0, di);
	ret = bq27520_read(BQ27520_REG_FAC, &FAC, 0, di);
	ret = bq27520_read(BQ27520_REG_RM, &RM, 0, di);
	ret = bq27520_read(BQ27520_REG_FCC, &FCC, 0, di);
	ret = bq27520_read(BQ27520_REG_NIC, &NIC, 0, di);

	ret = bq27520_i2c_txsubcmd(BQ27520_REG_CNTL, BQ27520_SUBCMD_CTNL_STATUS, di);
	ret = bq27520_read(BQ27520_REG_CNTL, &CTNL_STATUS, 0, di);	
	
	if (ret) {
		dev_err(di->dev,
			"error %d reading gauge.\n", ret);
		return ret;
	} else {
		dev_info(bq27520_di->dev, "CP, NAC, FAC, RM, FCC, NIC, CTNL_STATUS=[%4d, %4d, %4d, %4d, %3d, 0x%x]\n",
			NAC, FAC, RM, FCC, NIC, CTNL_STATUS);	
	}

	return 0;
}

void chg_ipc_print_msg(void)
{
    static int i = 0;

    if (bq27520_di->pChgIpcInfo != NULL) {
        while((bq27520_di->pChgIpcInfo->msg_index != i) && (bq27520_di->pChgIpcInfo->msg_index != 0))
        {
            i++;
            if (i > CHG_LOG_BUFFER_RECORDS)
                i = 1;
            dev_info(bq27520_di->dev, "MC%2d.%s\n", i, &bq27520_di->pChgIpcInfo->msg[i-1][0]);
        }
    } else {
         dev_info(bq27520_di->dev, "bq27520_di->pChgIpcInfo = NULL\r\n");
    }
}

void adc_ipc_print_msg(void)
{
    static int i = 0;

    if (bq27520_di->pAdcIpcInfo != NULL) {
        while((bq27520_di->pAdcIpcInfo->msg_index != i) && (bq27520_di->pAdcIpcInfo->msg_index != 0))
        {
            i++;
            if (i > ADC_LOG_BUFFER_RECORDS)
                i = 1;
            dev_info(bq27520_di->dev, "MA%2d.%s\n", i, &bq27520_di->pAdcIpcInfo->msg[i-1][0]);
        }
    } else {
         dev_info(bq27520_di->dev, "bq27520_di->pAdcIpcInfo = NULL\r\n");
    }
}

static int bq27520_battery_AvgCurrent(struct bq27520_device_info *di)
{
	int ret, ai;
    
	ret = bq27520_read(BQ27520_REG_AI, &ai, 0, di);
    
	if (ret) {
		dev_err(di->dev,
			"error %d reading relative average current\n", ret);
		return ret;
	}

	return (int)ai;
}

static int bq27520_cntl_cmd(struct bq27520_device_info *di,
				int subcmd)
{
    int retVal;
    retVal = bq27520_i2c_txsubcmd(BQ27520_REG_CNTL, subcmd, di);
    if (retVal != 0) {
        dev_err(di->dev,
			"cntl_cmd %d failed, retVal = %d\n", subcmd, retVal);
    }
    return retVal;
}

/*
 * i2c specific code
 */
static int bq27520_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27520_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	if (i2c_transfer(di->client->adapter, &msg, 1) < 0)
		return -EIO;

	mdelay(1);
    
	return 0;
}

/*
 * Fucntion for enable/disable snooze mode.
 */
void bq27520_battery_snooze_mode(bool SetSLP)
{
	if (SetSLP == true) {
		bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_SET_SLP);
		dev_info(bq27520_di->dev, "Enable sleep+ mode.\r\n");
	} else {
		bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CLR_SLP);
		dev_info(bq27520_di->dev, "Disable sleep+ mode.\r\n");
	}
}
EXPORT_SYMBOL(bq27520_battery_snooze_mode);

static int ctrl_flag = 0;
static int bq27520_chip_config(struct bq27520_device_info *di)
{
	int flags = 0, ret = 0;
	int i = 0;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, bq27520_di);
	if (ret < 0) {
		dev_err(di->dev, "error %d reading register %02x\n",
			 ret, BQ27520_REG_FLAGS);
		return ret;
	}
	
	/* Send battery insert command. */
	if (!(flags & BQ27520_FLAG_BAT_DET)) {
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_BAT_INS);
		dev_info(bq27520_di->dev, "Sent battery insert cmd.\r\n");		
	}
	
	for (i = 0; i < 3; i++) {
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
		ret = bq27520_read(BQ27520_REG_CNTL, &ctrl_flag, 0, di);
		if (ret < 0) {
		dev_err(di->dev, "error %d reading register %02x\n",
			 ret, BQ27520_REG_CNTL);
			return ret;
		}

		if (ctrl_flag & BQ27520_CS_INITCOMP)
			break;
		else
			mdelay(100);
	}

	/* Send enable sleep+ mode and enable IT command if needed. */
	if (!(ctrl_flag & BQ27520_CS_SNOOZE)) {
		bq27520_battery_snooze_mode(true);
	}
	
	if (flags & BQ27520_FLAG_BAT_DET) {		
		if (!(ctrl_flag & BQ27520_CS_QEN)) {
			bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_IT);
			dev_info(bq27520_di->dev, "Enable IT.\r\n");
		}
	}

	if (di->pdata->enable_dlog && !(ctrl_flag & BQ27520_CS_DLOGEN)) {
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_DLOG);
	}

	dev_info(bq27520_di->dev, "ctrl_flag(0x%X), flags(0x%X)\r\n", ctrl_flag, flags);
	return 0;
}

static void bq27520_every_30secs(unsigned long data)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;

	schedule_work(&di->counter);
	mod_timer(&timer, jiffies + BQ27520_COULOMB_POLL);
}

static void bq27520_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0, count = 0;
	struct bq27520_device_info *di;
	unsigned long flags;

	di = container_of(work, struct bq27520_device_info, counter);

	/* retrieve 30 values from FIFO of coulomb data logging buffer
	 * and average over time
	 */
	do {
		ret = bq27520_read(BQ27520_REG_LOGBUF, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}
		ret = bq27520_read(BQ27520_REG_LOGIDX, &index, 0, di);
		if (ret < 0)
			break;
	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		dev_err(di->dev, "Error %d reading datalog register\n", ret);
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}

static int bq27520_is_battery_present(void)
{
	return 1;
}

static int bq27520_is_battery_temp_within_range(void)
{
	return 1;
}

static int bq27520_is_battery_id_valid(void)
{
	return 1;
}

static int bq27520_status_getter(int function)
{
	int status = 0;
	unsigned long flags;

	spin_lock_irqsave(&current_battery_status.lock, flags);
	status = current_battery_status.status[function];
	spin_unlock_irqrestore(&current_battery_status.lock, flags);

	return status;
}

int bq27520_get_battery_mvolts(void)
{
	return bq27520_status_getter(GET_BATTERY_VOLTAGE);
}

int bq27520_get_battery_temperature(void)
{
	return bq27520_status_getter(GET_BATTERY_TEMPERATURE);
}

static int bq27520_get_battery_status(void)
{
	return bq27520_status_getter(GET_BATTERY_STATUS);
}

int bq27520_get_remaining_capacity(void)
{
	return bq27520_status_getter(GET_BATTERY_CAPACITY);
}

int bq27520_get_battery_flags(void)
{
	return bq27520_status_getter(GET_BATTERY_FLAGS);
}

static struct msm_battery_gauge bq27520_batt_gauge = {
	.get_battery_mvolts		= bq27520_get_battery_mvolts,
	.get_battery_temperature	= bq27520_get_battery_temperature,
	.is_battery_present		= bq27520_is_battery_present,
	.is_battery_temp_within_range	= bq27520_is_battery_temp_within_range,
	.is_battery_id_valid		= bq27520_is_battery_id_valid,
	.get_battery_status		= bq27520_get_battery_status,
	.get_batt_remaining_capacity	= bq27520_get_remaining_capacity,
	.get_battery_flags		= bq27520_get_battery_flags,
};

/* FIH-SW3-KERNEL-VH-CHARGING-13*[ */
static void bq27520_power_down(void)
{
    if (bq27520_isEnableBatteryCheck == 1) {
        printk("Gauge failed, force shutdown\r\n");

    #ifndef CONFIG_FIH_FTM
    /* Shutdown device. */
    if (pm_power_off)
        pm_power_off();
    #endif		
    }
}

static void update_current_battery_status(int data)
{
	int status[NUM_OF_STATUS], ret = 0;
	unsigned long flag;
	int err_count = 0;

	#ifdef CONFIG_FIH_SW1_FEATURE_BATTERY_THERMAL_VIA_PMIC
	int temp_modem = 0;
	#endif

	memset(status, 0, sizeof status);

    err_count = 0;
    if ((bq27520_isEnableBatteryCheck == 0) || 
        		(bq27520_di->rommode == true)) {
        status[GET_BATTERY_CAPACITY] = 60;
        status[GET_BATTERY_VOLTAGE] = 3800;
        status[GET_BATTERY_TEMPERATURE] = 250;
        status[GET_BATTERY_CURRENT] = 100;
    } else {
        while(1)
        {
            ret = bq27520_battery_rsoc(bq27520_di);
            if (ret < 0) {
                err_count++;
            
                if (err_count > 10) {
                    bq27520_power_down();
                    break;
                }
                msleep(1000);

                if (bq27520_di->rommode == true) {
                    err_count = 0;
                    break;
                }
            } else {
                err_count = 0;
                break;
            }
        }

        status[GET_BATTERY_CAPACITY] = (ret < 0) ? 0 : ret;
        status[GET_BATTERY_VOLTAGE] = bq27520_battery_voltage(bq27520_di);
        status[GET_BATTERY_TEMPERATURE] = bq27520_battery_temperature(bq27520_di);
        status[GET_BATTERY_CURRENT] = bq27520_battery_AvgCurrent(bq27520_di);
        status[GET_BATTERY_FLAGS] = bq27520_battery_flags(bq27520_di);
        status[GET_BATTERY_SOH] = bq27520_battery_SOH(bq27520_di);
        status[GET_BATTERY_SOH_STATUS] = bq27520_battery_SOH_status(bq27520_di);		
    }
    
    if (msm_batt_debug_mask & BQ275X0_MODEM_CHG_LOG)
    	chg_ipc_print_msg();

    if (msm_batt_debug_mask & BQ275X0_MODEM_ADC_LOG)
    	adc_ipc_print_msg();

    #ifndef CONFIG_FIH_FTM
    if ((bq27520_di->pChgIpcInfo != NULL) && (err_count == 0)) {
        bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_CAPACITY] = status[GET_BATTERY_CAPACITY];
        bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_VOLTAGE] = status[GET_BATTERY_VOLTAGE];
		/* FIH-SW3-KERNEL-EL-fix_temp_read_from_PMIC-00*[ */
		#ifdef CONFIG_FIH_SW1_FEATURE_BATTERY_THERMAL_VIA_PMIC
		temp_modem = bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_TEMPERATURE];
		/* If the temperature changes more than one degree, update the gauge's temperature. */
		if (ABS(temp_modem - status[GET_BATTERY_TEMPERATURE]) >= 10) {
			dev_info(bq27520_di->dev, "Update gauge temperature = [%d]\r\n", temp_modem);
			bq27520_battery_set_temperature(bq27520_di, temp_modem);
		}
		
		status[GET_BATTERY_TEMPERATURE] = temp_modem;
		#else
		bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_TEMPERATURE] = status[GET_BATTERY_TEMPERATURE];
		#endif
		/* FIH-SW3-KERNEL-EL-fix_temp_read_from_PMIC-00*] */
        bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_I] = status[GET_BATTERY_CURRENT];
		
        dev_info(bq27520_di->dev, "CP, V, T, I=[%3d, %4d, %3d, %4d, 0x%x, %3d, %d]\r\n", status[GET_BATTERY_CAPACITY],
		status[GET_BATTERY_VOLTAGE], status[GET_BATTERY_TEMPERATURE], status[GET_BATTERY_CURRENT],
		status[GET_BATTERY_FLAGS], status[GET_BATTERY_SOH], status[GET_BATTERY_SOH_STATUS]);
		
	/* Show extra debug message. */
	if (msm_batt_debug_mask & BQ275X0_EXTRA_DEBUG_LOG)
		bq27520_battery_extra_debug_msg(bq27520_di);		
		
    }
    #else
    if (bq27520_di->pChgIpcInfo != NULL) {
            /* FIH-SW3-KERNEL-EL-fix_temp_read_from_PMIC-02*[ */
            #ifdef CONFIG_FIH_SW1_FEATURE_BATTERY_THERMAL_VIA_PMIC
            temp_modem = bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_TEMPERATURE];
            status[GET_BATTERY_TEMPERATURE] = temp_modem;
            #endif
            /* FIH-SW3-KERNEL-EL-fix_temp_read_from_PMIC-02*] */

            bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_CAPACITY] = 50;
            bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_VOLTAGE] = 3800;
            bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_TEMPERATURE] = 250;
            bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_I] = 250;

	    dev_info(bq27520_di->dev, "FTM, CP, V, T, I=[%d, %d, %d, %d]\r\n", 
		bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_CAPACITY],
		bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_VOLTAGE],
		bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_TEMPERATURE],
		bq27520_di->pChgIpcInfo->parameters[CHG_IPC_PARAMETER_GAUGE_I]);
    }
    #endif
    
    
	spin_lock_irqsave(&current_battery_status.lock, flag);
	current_battery_status.status[GET_BATTERY_STATUS] = data;
	current_battery_status.status[GET_BATTERY_VOLTAGE] =
						status[GET_BATTERY_VOLTAGE];
	current_battery_status.status[GET_BATTERY_TEMPERATURE] =
						status[GET_BATTERY_TEMPERATURE];
	current_battery_status.status[GET_BATTERY_CAPACITY] =
						status[GET_BATTERY_CAPACITY];
	current_battery_status.status[GET_BATTERY_CURRENT] =
						status[GET_BATTERY_CURRENT];
	current_battery_status.status[GET_BATTERY_FLAGS] =
						status[GET_BATTERY_FLAGS];
	current_battery_status.status[GET_BATTERY_SOH] =
						status[GET_BATTERY_SOH];
	current_battery_status.status[GET_BATTERY_SOH_STATUS] =
						status[GET_BATTERY_SOH_STATUS];	
	spin_unlock_irqrestore(&current_battery_status.lock, flag);

    if (current_battery_status.status[GET_BATTERY_CAPACITY] <= 5) {
        dev_err(bq27520_di->dev, "CP = %d, wake-up timer = 600\n", current_battery_status.status[GET_BATTERY_CAPACITY]);
        msmrtc_set_wakeup_cycle_time(600);
    } else if (current_battery_status.status[GET_BATTERY_CAPACITY] <= 10) {
        dev_err(bq27520_di->dev, "CP = %d, wake-up timer = 1800\n", current_battery_status.status[GET_BATTERY_CAPACITY]);
        msmrtc_set_wakeup_cycle_time(1800);
    } else {
        dev_err(bq27520_di->dev, "CP = %d, disable wake-up timer\n", current_battery_status.status[GET_BATTERY_CAPACITY]);
        msmrtc_set_wakeup_cycle_time(0);
    }

    /* Force trigger power_supply_changed on battery driver to update UI */
    msm_batt_update_psy_status();        
}
/* FIH-SW3-KERNEL-VH-CHARGING-13*] */

/* only if battery charging satus changes then notify msm_charger. otherwise
 * only refresh current_batter_status
 */
static int if_notify_msm_charger(int *data)
{
	int ret = 0, flags = 0, status = 0;
	unsigned long flag;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, bq27520_di);
	if (ret < 0) {
		dev_err(bq27520_di->dev, "error %d reading register %02x\n",
			ret, BQ27520_REG_FLAGS);
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	} else {
		if (flags & BQ27520_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (flags & BQ27520_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}

	*data = status;
	spin_lock_irqsave(&current_battery_status.lock, flag);
	ret = (status != current_battery_status.status[GET_BATTERY_STATUS]);
	spin_unlock_irqrestore(&current_battery_status.lock, flag);
	return ret;
}

static void battery_status_poller(struct work_struct *work)
{
	int status = 0, temp = 0;

	temp = if_notify_msm_charger(&status);
	update_current_battery_status(status);
	if (temp)
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);

	schedule_delayed_work(&current_battery_status.poller,
				bq27520_polling_timeout);

}

void bq27520_resch_change_polling_timeout(int second, int modify_timeout)
{
	int polling_timeout;

	dev_info(bq27520_di->dev, "%s(%d %d)", __func__, second, modify_timeout);
	
	polling_timeout = second * HZ;

	cancel_delayed_work(&current_battery_status.poller);
	schedule_delayed_work(&current_battery_status.poller, polling_timeout);
	
	if (modify_timeout)
		bq27520_polling_timeout = polling_timeout;
}

int bq27520_get_polling_timeout(void)
{
	return (bq27520_polling_timeout/HZ);
}

void bq27520_change_polling_timeout(int second)
{
	#if (defined CONFIG_FIH_FTM) && (defined CONFIG_FIH_SW1_FEATURE_BATTERY_THERMAL_VIA_PMIC)
	/*    if we are at FTM mode and battery temperature got from PMIC
	 *    snoop the timeout value to 3 seconds.
	 */
	second = 3;
	#endif
	
	if (bq27520_di != NULL)
		dev_info(bq27520_di->dev, "%s(%d)", __func__, second);

	bq27520_polling_timeout = second * HZ;
}

static void bq27520_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0, status = 0;
	struct bq27520_device_info *di;

	di  = container_of(work, struct bq27520_device_info, hw_config.work);

	pr_debug(KERN_INFO "Enter bq27520_hw_config\n");
	ret = bq27520_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27520 ret = %d\n", ret);
		goto exit;
	}
	/* bq27520 is ready for access, update current_battery_status by reading
	 * from hardware
	 */
	if_notify_msm_charger(&status);
	update_current_battery_status(status);
	msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);

	/* FIH-SW3-KERNEL-VH-TODO: Disable temporarily enable_irq(di->soc_irq); */
	enable_irq(di->bat_low_irq);
	enable_irq(di->ovp_irq);

	/*
		If we enter rom mode, bq27520_hw_config will return when calling q27520_chip_config
		and thus we can't have battery status updated, and no fake battery capacity reported to system
		schedule_delayed_work(&current_battery_status.poller, (HZ)*0.5);
	*/
    
	if (di->pdata->enable_dlog) {
		schedule_work(&di->counter);
		init_timer(&timer);
		timer.function = &bq27520_every_30secs;
		timer.data = (unsigned long)di;
		timer.expires = jiffies + BQ27520_COULOMB_POLL;
		add_timer(&timer);
	}

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
	bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);
	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DEVCIE_TYPE);
	bq27520_read(BQ27520_REG_CNTL, &type, 0, di);
	bq27520_cntl_cmd(di, BQ27520_SUBCMD_FW_VER);
	bq27520_read(BQ27520_REG_CNTL, &fw_ver, 0, di);

	if  ((ctrl_flag & BQ27520_CS_INITCOMP) && !(ctrl_flag & BQ27520_CS_OCVCMDCOMP)) {
		/* If initcomp set but ocvcmdcomp unset, let me flash again */
		bq275x0_flash_recovery(true);
	} else {
		/* The gauge status is fine, just set flag into MFG info */
		bq275x0_flash_recovery(false);
	}

	dev_info(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION \
		is 0x%02X\n", type, fw_ver);
	dev_info(di->dev, "Complete bq27520 configuration 0x%02X\n", flags);

exit:
	schedule_delayed_work(&current_battery_status.poller, (HZ)*0.5);
}

void schedule_bq27520_hw_config(void)
{
	if (bq27520_di) {
		dev_info(bq27520_di->dev, "%s schedule hw config", __func__);
		schedule_delayed_work(&bq27520_di->hw_config, BQ27520_INIT_DELAY);
	}
}
EXPORT_SYMBOL(schedule_bq27520_hw_config);

static int bq27520_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	mdelay(1);
    
	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
			{         
				*rt_value = get_unaligned_le16(data);
				if (*rt_value > 0x8000)
				    *rt_value |= 0xFFFF0000;
			} 
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static ssize_t bq27520_isBatteryVaild(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int temp = 0;
    temp = bq27520_get_battery_temperature();

    if ((temp >= 0) && (temp <= 800))
        return snprintf(buf, PAGE_SIZE, "1\n");
    else
        return snprintf(buf, PAGE_SIZE, "0\n");
}
static DEVICE_ATTR(isBatteryVaild, 0644, bq27520_isBatteryVaild, NULL);

static ssize_t bq27520_isGaugeReady(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    if ((bq27520_di->rommode == true))
        return snprintf(buf, PAGE_SIZE, "2\n");		
    else if (bq27520_isEnableBatteryCheck)
        return snprintf(buf, PAGE_SIZE, "1\n");
    else
        return snprintf(buf, PAGE_SIZE, "0\n");
}
static DEVICE_ATTR(isGaugeReady, 0644, bq27520_isGaugeReady, NULL);

static ssize_t bq27520_getRSOC(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;
    ret = bq27520_battery_rsoc(bq27520_di);

    return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}
static DEVICE_ATTR(rsoc, 0644, bq27520_getRSOC, NULL);


/*
 * Fucntions for gauge testing feature.
 */
#ifdef CONFIG_BQ27520_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27520_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27520_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27520_REG_ICR && reg > 0x00) {
		ret = bq27520_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else {
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");
	}

	return ret;
}

static ssize_t bq27520_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	dev_info(dev, "recv'd cmd is 0x%02X\n", reg);
	return ret;
}

static ssize_t bq27520_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret, temp = 0;
	struct platform_device *client;
	struct bq27520_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27520_SUBCMD_DEVCIE_TYPE ||
		 subcmd == BQ27520_SUBCMD_FW_VER ||
		 subcmd == BQ27520_SUBCMD_HW_VER ||
		 subcmd == BQ27520_SUBCMD_CHEM_ID) {

		bq27520_cntl_cmd(di, subcmd);/* Retrieve Chip status */
		ret = bq27520_read(BQ27520_REG_CNTL, &temp, 0, di);

		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else {
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");
	}

	return ret;
}

static ssize_t bq27520_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27520_read_stdcmd,
	bq27520_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27520_read_subcmd,
	bq27520_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name = "bq27520-test",
	.id = -1,
	.dev.platform_data = NULL,
};
#endif

/* FIH-SW3-KERNEL-VH-TODO */
#if 0
static irqreturn_t soc_irqhandler(int irq, void *dev_id)
{
	int status = 0, temp = 0;

	temp = if_notify_msm_charger(&status);
	update_current_battery_status(status);
	if (temp)
		msm_charger_notify_event(NULL, CHG_BATT_STATUS_CHANGE);
	return IRQ_HANDLED;
}
#endif

static void bq275x0_battery_bat_low_ist(struct work_struct *work)
{
    int status = 0;
	
    wake_lock(&bq27520_di->bq27520_wakelock);
    dev_err(bq27520_di->dev, "%s : BAT_LOW \r\n", __func__);
    update_current_battery_status(status);
    wake_unlock(&bq27520_di->bq27520_wakelock);
    wake_lock_timeout(&bq27520_di->bq27520_wakelock, 1 * HZ);
}

static irqreturn_t bat_low_irqhandler(int irq, void *dev_id)
{
    dev_err(bq27520_di->dev, "BAT_LOW_IRQ OCCUR!\n");
    if (bq27520_isEnableBatteryCheck == 1) {
        schedule_delayed_work(&bq27520_di->bq275x0_BATLOW,
                               msecs_to_jiffies(100));
    }
	return IRQ_HANDLED;
}

static void msm_battery_ovp_ist(struct work_struct *work)
{
    int status = 0;
	
	dev_err(bq27520_di->dev, "%s : OVP occur \r\n", __func__);
    update_current_battery_status(status);
    wake_unlock(&bq27520_di->bq27520_wakelock);
    wake_lock_timeout(&bq27520_di->bq27520_wakelock, 1 * HZ);
}

static irqreturn_t ovp_irqhandler(int irq, void *dev_id)
{
    if (bq27520_isEnableBatteryCheck == 1) {
        if (gpio_get_value(bq27520_di->pdata->ovp_flag) == 0) {
            dev_err(bq27520_di->dev, "OVP_IRQ OCCUR!\n");
            wake_lock(&bq27520_di->bq27520_wakelock);
            schedule_delayed_work(&bq27520_di->ovp_work,
                                  msecs_to_jiffies(100));
        }
    }
	return IRQ_HANDLED;
}


/*
 * Fucntions for gauge firmware update.
 */
#ifdef CONFIG_FIH_SW3_BQ275X0_ROMMODE
int bq275x0_battery_access_data_flash(unsigned char subclass,
                                        unsigned char offset,
                                        unsigned char* buf,
                                        int len,
                                        int write
                                        )
{
#define WRITE_BLOCK_SIZE 32

	int i        = 0;
	unsigned sum = 0;
	unsigned char cmd[WRITE_BLOCK_SIZE + 1];
	unsigned char block_num= offset / BLOCK_SIZE;
	unsigned char checksum = 0;
	struct i2c_msg msg;
	int len_write;
	
	if (!bq27520_di->client)
		return -ENODEV;
	
	msg.addr = bq27520_di->client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &cmd[0];

	
	/* Set SEALED/UNSEALED mode. */
	cmd[0] = BQ275X0_EXT_CMD_DFDCNTL;
	cmd[1] = 0x00;
	if (i2c_transfer(bq27520_di->client->adapter, &msg, 1) < 0)
		return -EIO;
	mdelay(1);
	
	/* Set subclass */
	cmd[0] = BQ275X0_EXT_CMD_DFCLS;
	cmd[1] = subclass;
	if (i2c_transfer(bq27520_di->client->adapter, &msg, 1) < 0)
		return -EIO;
	mdelay(1);
	
	/* Set block to R/W */
	cmd[0] = BQ275X0_EXT_CMD_DFBLK;
	cmd[1] = block_num;
	if (i2c_transfer(bq27520_di->client->adapter, &msg, 1) < 0)
		return -EIO;
	mdelay(1);
	
	if (write) {
		/* Calculate checksum */
		for (i = 0; i < len; i++)
			sum += buf[i];
		checksum = 0xFF - (sum & 0x00FF);
		
		/* Write data */
		cmd[0] = BQ275X0_EXT_CMD_DFD;
		len_write = (len >= WRITE_BLOCK_SIZE) ? WRITE_BLOCK_SIZE : len;
		memcpy((cmd + 1), buf, len_write);
		msg.len = 1 + len_write;
		if (i2c_transfer(bq27520_di->client->adapter, &msg, 1) < 0)
			return -EIO;
		mdelay(1);
		
		/* Set checksum */
		cmd[0] = BQ275X0_EXT_CMD_DFDCKS;
		cmd[1] = checksum;
		msg.len = 2;
		if (i2c_transfer(bq27520_di->client->adapter, &msg, 1) < 0)
			return -EIO;
		mdelay(1);
	} else {
		/* Read data */
		msg.flags = I2C_M_RD;
		msg.len = 32;
		msg.buf = buf;
	
		if (i2c_transfer(bq27520_di->client->adapter, &msg, 1) < 0)
			return -EIO;
		mdelay(1);
	}
	
	return 0;
}

int bq275x0_battery_fw_version(void)
{
    int ret;
    int fw_version = 0;

    ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_HW_VER);
    bq27520_read(BQ27520_REG_CNTL, &fw_version, 0, bq27520_di);

    if (ret) {
        dev_err(bq27520_di->dev, "error reading firmware version\n");
        return FAILED_MARK;
    }

    return fw_version;
}
EXPORT_SYMBOL(bq275x0_battery_fw_version);

int bq275x0_battery_device_type(void)
{
    int ret;
    int device_type;

    ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_DEVCIE_TYPE);
    bq27520_read(BQ27520_REG_CNTL, &device_type, 0, bq27520_di);
    
    if (ret) {
        dev_err(bq27520_di->dev, "error reading firmware version\n");
        return FAILED_MARK;
    }

    return device_type;
}
EXPORT_SYMBOL(bq275x0_battery_device_type);

int bq275x0_battery_IT_enable(void)
{
    int ret;
    int retries = 0;
    int flags = 0;

    ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_ENABLE_IT);
    if (ret) {
        dev_err(bq27520_di->dev, "error IT Enable\n");
        return ret;
    }
    
    mdelay(200);

    bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CTNL_STATUS);
	bq27520_read(BQ27520_REG_CNTL, &flags, 0, bq27520_di);
    
    while ((flags & (0x1 << BQ275X0_CNTLSTATUS_QEN)) != (0x1 << BQ275X0_CNTLSTATUS_QEN)) {
        if (retries > 10) {
            dev_info(bq27520_di->dev, "IT Enable Failed\n");
            return -1;
        } else
            retries++;

        mdelay(50);
        bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CTNL_STATUS);
        bq27520_read(BQ27520_REG_CNTL, &flags, 0, bq27520_di);
    }
    
    dev_info(bq27520_di->dev, "IT Enable Finished\n");
    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_IT_enable);

int bq275x0_battery_reset(void)
{
    int ret = 0;
    int retries = 0;
    int flags = 0;

    if (bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_RESET) != 0)
        return -1;

    mdelay(3300);
    if (bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CTNL_STATUS) != 0)
        return -1;
   
    if (bq27520_read(BQ27520_REG_CNTL, &flags, 0, bq27520_di) != 0)
        return -1;

    while ((flags & (0x1 << BQ275X0_CNTLSTATUS_RUP_DIS)) != (0x1 << BQ275X0_CNTLSTATUS_RUP_DIS)) {
        if (retries > 20) {
            dev_info(bq27520_di->dev, "Reset Failed\n");
            return -1;
        } else {
            retries++;
        }

        mdelay(50);
        bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_CTNL_STATUS);
        bq27520_read(BQ27520_REG_CNTL, &flags, 0, bq27520_di);       
    }
    
    dev_info(bq27520_di->dev, "Gauge RESET Finished\n");
    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_reset);

int bq275x0_battery_sealed(void)
{
    int ret;

    ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_SEALED);
    if (ret) {
        dev_err(bq27520_di->dev, "error sealed\n");
        return ret;
    }

    dev_info(bq27520_di->dev, "End Gauge SEALED\n");
    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_sealed);

void bq275x0_battery_exit_rommode(void)
{
    bq27520_di->rommode = false;
    dev_info(bq27520_di->dev, "Exit Rom Mode\n");
}
EXPORT_SYMBOL(bq275x0_battery_exit_rommode);

int bq275x0_battery_enter_rom_mode(void)
{
    int ret = 0;

    bq27520_di->rommode = true;
    ret = bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_ROM_MODE);
    if (ret) {
        dev_err(bq27520_di->dev, "failed to place bq275x0 into ROM MODE\n");
    }

    return ret;
}
EXPORT_SYMBOL(bq275x0_battery_enter_rom_mode);

int bq275x0_battery_get_MfgInfo(char *buf)
{
	int retVal = -1;
	int count = 0;
	
	while((retVal != 0) && (count <= 3)) {
		retVal = bq275x0_battery_access_data_flash(57, 0, buf, 32, false);
		count++;
	}
	return retVal;
}
EXPORT_SYMBOL(bq275x0_battery_get_MfgInfo);

int bq275x0_battery_write_MfgInfo(char *buf, int len)
{
	int retVal = -1;
	int count = 0;
	int writelen = (len < 32) ? len : 32;
	
	dev_info(bq27520_di->dev, "BQFS wrtie MFG INFO\n");
	
	while((retVal != 0) && (count <= 3)) {
		retVal = bq275x0_battery_access_data_flash(57, 0, buf, writelen, true);
		count++;
	}
	
	return retVal;
}
EXPORT_SYMBOL(bq275x0_battery_write_MfgInfo);

int bq275x0_query_fw_ver(void)
{
	int fw_ver = 0;
	
	bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_FW_VER);
	bq27520_read(BQ27520_REG_CNTL, &fw_ver, 0, bq27520_di);
	dev_err(bq27520_di->dev, "fw_ver: 0x%2X\n", fw_ver);

	return fw_ver;
}
EXPORT_SYMBOL(bq275x0_query_fw_ver);

extern unsigned char BQFS_MFG_INFO[];
int bq27520_isDFIUpToDate(void)
{
    int retVal = 0;
    char buf[32];
    
    if (bq275x0_battery_get_MfgInfo(&buf[0]) == 0) {
        dev_err(bq27520_di->dev, "MFG INFO-FLASH: %d, \"%s\"\n", buf[0], buf);
        dev_err(bq27520_di->dev, "MFG INFO-IMAGE: %d, \"%s\"\n",BQFS_MFG_INFO[0], BQFS_MFG_INFO);

        if (strncmp(buf, BQFS_MFG_INFO, 32))
            retVal = 0;
        else 
            retVal = 1;
    } else {
        dev_err(bq27520_di->dev, "%s: Read MfgInfo from gauge failed, FORCE REFLASH\n", __func__);
        retVal = -1;
    }
	
    return retVal;

}
#endif


/*
 * Fucntions for FTM test.
 */
#ifdef CONFIG_FIH_FTM
static enum _battery_info_type_ g_current_cmd = 0;
static ssize_t bq275x0_ftm_battery_write(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    int cmd     = 0;
    
    sscanf(buf, "%d", &cmd);
    dev_info(dev, "%s: COMMAND: %d\n", __func__, cmd);

    g_current_cmd = cmd;

    return count;
}

static ssize_t bq275x0_ftm_battery_read(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int retVal = 0;
    int para[3] = {0, 0, 0};

    switch (g_current_cmd) {
    /* BATTERY GETCAPACITY */
    case BATT_CAPACITY_INFO:
        retVal = bq27520_get_remaining_capacity();
        break;
        
    /* BATTERY GETADC*/
    case BATT_VOLTAGE_INFO:
        retVal = bq27520_battery_voltage(bq27520_di);
        break;
        
    /* BATTERY GETTEMP */
    case BATT_TEMPERATURE_INFO:   
        /* FIH-SW3-KERNEL-EL-fix_temp_read_from_PMIC-02*[ */
        #ifdef CONFIG_FIH_SW1_FEATURE_BATTERY_THERMAL_VIA_PMIC
            retVal = bq27520_status_getter(GET_BATTERY_TEMPERATURE);
        #else
            retVal = bq27520_battery_temperature(bq27520_di);
        #endif
        /* FIH-SW3-KERNEL-EL-fix_temp_read_from_PMIC-02*] */
        break;

    /* GASGAUGE PING */
    case BATT_FAMILY_CODE:
        bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_FW_VER);
        bq27520_read(BQ27520_REG_CNTL, &retVal, 0, bq27520_di);
        break;
        
    /* GASGAUGE CheckDFIVersion */
    case BATT_GET_MANUFACTURER_INFO:
#ifdef CONFIG_FIH_SW3_BQ275X0_ROMMODE
        retVal = bq27520_isDFIUpToDate();
#else
        retVal = 0;
#endif
        break;
        
    /* GASGAUGE GETVERSION */
    case BATT_GET_DEVICE_TYPE:
    	bq27520_cntl_cmd(bq27520_di, BQ27520_SUBCMD_DEVCIE_TYPE);
        bq27520_read(BQ27520_REG_CNTL, &retVal, 0, bq27520_di);
        break;
        
    /* BACKUPBATTERY GETADC */
    case BATT_GET_VCOIN:
        proc_comm_get_vcoin(2, &retVal, para[0]);
        break;
        
    default:
        retVal = 0;
    };

    dev_info(dev, "%s: COMMAND: %d, retVal = %d\n", __func__, g_current_cmd, retVal);

    return snprintf(buf, PAGE_SIZE, "%d\n", retVal);
}
static DEVICE_ATTR(ftm_battery, 0644, bq275x0_ftm_battery_read, bq275x0_ftm_battery_write);
#endif
/* FIH-SW3-KERNEL-VH-CHARGING-11*] */

static int bq27520_power(bool enable, struct bq27520_device_info *di)
{
	int rc = 0, ret;
	const struct bq27520_platform_data *platdata;

	platdata = di->pdata;
	if (enable) {
        #if 0
		rc = gpio_request(platdata->soc_int, "GAUGE_SOC_INT");
		if (rc) {
			dev_err(di->dev, "%s: fail to request gpio %d (%d)\n",
				__func__, platdata->soc_int, rc);
			goto gpio_fail;
		}
		gpio_direction_input(platdata->soc_int);
		di->soc_irq = gpio_to_irq(platdata->soc_int);
		rc = request_threaded_irq(di->soc_irq, NULL, soc_irqhandler,
				IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
				"BQ27520_IRQ", di);
		if (rc) {
			dev_err(di->dev, "%s: fail to request irq %d (%d)\n",
				__func__, platdata->soc_int, rc);
			goto irqreq_fail;
		} else {
			disable_irq_nosync(di->soc_irq);
		}
        #endif
        
        /* Register BAT_LOW IRQ*/
        if (gpio_tlmm_config(GPIO_CFG(platdata->bat_low, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
			pr_err( "gpio_tlmm_config(bat_low) failed\n" );     
		
        di->bat_low_irq = MSM_GPIO_TO_INT(platdata->bat_low);
        ret = request_irq(di->bat_low_irq, bat_low_irqhandler, IRQF_TRIGGER_RISING, "BQ27520_BAT_LOW_IRQ", di);

        if (ret) {
            dev_err(di->dev, "request_irq failed\n");
        } else {
            irq_set_irq_wake(di->bat_low_irq, 1);
            disable_irq_nosync(di->bat_low_irq);
        }
        
        /* Register OVP IRQ*/
        if (gpio_tlmm_config(GPIO_CFG(platdata->ovp_flag, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
			pr_err( "gpio_tlmm_config(ovp_flag) failed\n" );     
		
        di->ovp_irq = MSM_GPIO_TO_INT(platdata->ovp_flag);
        rc = request_irq(di->ovp_irq, ovp_irqhandler, IRQF_TRIGGER_FALLING, "OVP_IRQ", di);
        if (rc) {
            dev_err(di->dev, "request_irq failed\n");
        } else {
            irq_set_irq_wake(di->ovp_irq, 1);
            disable_irq_nosync(di->ovp_irq);
        }		
        
	} else {
		/* FIH-SW3-KERNEL-VH-TODO: Disable SOC IRQ temporarily free_irq(di->soc_irq, di); */
        free_irq(di->bat_low_irq, di);
        free_irq(di->ovp_irq, di);
	}
	return rc;
}

static int bq27520_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27520_device_info *di;
	struct bq27520_access_methods *bus;
	const struct bq27520_platform_data  *pdata;
	int num, retval = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pdata = client->dev.platform_data;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
	di->id = num;
	di->pdata = pdata;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27520_read_i2c;
	di->bus = bus;
	di->client = client;

#ifdef CONFIG_BQ27520_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
			 &fs_attr_group);
		if (retval)
			goto batt_failed_3;
	} else
		goto batt_failed_3;
#endif

    /* FIH-SW3-KERNEL-VH-CHARGING-09*[  */
    /* Init before bq27520_power() */
   	bq27520_di = di;
	INIT_DELAYED_WORK(&bq27520_di->bq275x0_BATLOW, bq275x0_battery_bat_low_ist);
	INIT_DELAYED_WORK(&bq27520_di->ovp_work, msm_battery_ovp_ist);
	wake_lock_init(&bq27520_di->bq27520_wakelock, WAKE_LOCK_SUSPEND, "bq27520");
	
	bq27520_di->pChgIpcInfo = (chg_ipc_smem *) smem_alloc(SMEM_BATT_INFO, sizeof(*bq27520_di->pChgIpcInfo));
	bq27520_di->pAdcIpcInfo = (adc_ipc_smem *) smem_alloc(SMEM_ID_VENDOR2, sizeof(*bq27520_di->pAdcIpcInfo));	
    /* FIH-SW3-KERNEL-VH-CHARGING-09*] */

	retval = bq27520_power(true, di);
	if (retval) {
		dev_err(&client->dev, "failed to powerup ret = %d\n", retval);
		goto batt_failed_3;
	}

    
    /* FIH-SW3-KERNEL-VH-CHARGING-09-  mdelay(1000); */ 
	spin_lock_init(&lock);

	if (pdata->enable_dlog)
		INIT_WORK(&di->counter, bq27520_coulomb_counter_work);

	INIT_DELAYED_WORK(&current_battery_status.poller,
			battery_status_poller);
	INIT_DELAYED_WORK(&di->hw_config, bq27520_hw_config);

	/* 
		Remove this because I will schedule this work at rommode driver
		for detecting gauge firmware error, I need to read OCVCMDCOMP bit.
		and MFG INFO; to make sure rommode driver init called before.
		schedule_delayed_work(&di->hw_config, BQ27520_INIT_DELAY); 
	*/
    
    #ifdef CONFIG_FIH_FTM
    retval = device_create_file(&client->dev, &dev_attr_ftm_battery);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_ftm_battery failed\n", __func__);
    }
    #endif
    
    retval = device_create_file(&client->dev, &dev_attr_isBatteryVaild);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_isBatteryVaild failed\n", __func__);
    }

    retval = device_create_file(&client->dev, &dev_attr_isGaugeReady);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_isGaugeReady failed\n", __func__);
    }

    retval = device_create_file(&client->dev, &dev_attr_rsoc);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_rsoc failed\n", __func__);
    }

    /* FIH-SW3-KERNEL-VH-TODO*/
    /* FIH-SW3-KERNEL-CHARGING-09- Remove reset for boot time issue */
    /* bq275x0_battery_reset(); */

	return 0;

batt_failed_3:
	kfree(bus);
batt_failed_2:
	kfree(di);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	dev_info(&client->dev, "%s finished\n", __func__);

	return retval;
}

static int bq27520_battery_remove(struct i2c_client *client)
{
	struct bq27520_device_info *di = i2c_get_clientdata(client);

	if (di->pdata->enable_dlog) {
		del_timer_sync(&timer);
		cancel_work_sync(&di->counter);
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_DISABLE_DLOG);
	}

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);
	cancel_delayed_work_sync(&current_battery_status.poller);

	bq27520_power(false, di);

	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

#ifdef CONFIG_PM
static int bq27520_suspend(struct device *dev)
{
	struct bq27520_device_info *di = dev_get_drvdata(dev);
    
	/* FIH-SW3-KERNEL-VH-TODO: Disable SOC IRQ temporarily disable_irq_nosync(di->soc_irq); */

    if (di->pdata->enable_dlog) {
		del_timer_sync(&timer);
		cancel_work_sync(&di->counter);
	}

	cancel_delayed_work_sync(&current_battery_status.poller);
	return 0;
}

static int bq27520_resume(struct device *dev)
{
    int status = 0, temp = 0;
	struct bq27520_device_info *di = dev_get_drvdata(dev);

	/* FIH-SW3-KERNEL-VH-TODO: Disable SOC IRQ temporarily enable_irq(di->soc_irq); */
	if (di->pdata->enable_dlog)
		add_timer(&timer);

	temp = if_notify_msm_charger(&status);
	update_current_battery_status(status);

	schedule_delayed_work(&current_battery_status.poller,
				bq27520_polling_timeout);

	return 0;
}

static const struct dev_pm_ops bq27520_pm_ops = {
	.suspend = bq27520_suspend,
	.resume = bq27520_resume,
};
#endif

static const struct i2c_device_id bq27520_id[] = {
	{ "bq27520", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver bq27520_battery_driver = {
	.driver = {
		.name = "bq27520",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &bq27520_pm_ops,
#endif
	},
	.probe = bq27520_battery_probe,
	.remove = bq27520_battery_remove,
	.id_table = bq27520_id,
};

static void init_battery_status(void)
{
	spin_lock_init(&current_battery_status.lock);
	current_battery_status.status[GET_BATTERY_STATUS] =
			POWER_SUPPLY_STATUS_UNKNOWN;
}

static int __init bq27520_battery_init(void)
{
	int ret;
	
	/* init the capacity to zero */
	current_battery_status.status[GET_BATTERY_CAPACITY] = 0;
	
	/* initialize current_battery_status, and register with msm-charger */
	init_battery_status();
	msm_battery_gauge_register(&bq27520_batt_gauge);

	ret = i2c_add_driver(&bq27520_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register driver ret = %d\n", ret);

	return ret;
}
module_init(bq27520_battery_init);

static void __exit bq27520_battery_exit(void)
{
	i2c_del_driver(&bq27520_battery_driver);
	msm_battery_gauge_unregister(&bq27520_batt_gauge);
}
module_exit(bq27520_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27520 battery monitor driver");
