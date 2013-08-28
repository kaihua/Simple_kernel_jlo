/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <linux/memblock.h>
#include <asm/mach-types.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_battery.h>

/* MTD-Kernel-HC-Remove_smsc911x-00+[ */
#ifdef CONFIG_SMSC911X
#include <linux/smsc911x.h>
#endif
/* MTD-Kernel-HC-Remove_smsc911x-00+] */

#include <linux/atmel_maxtouch.h>
#include <linux/fmem.h>
#include <linux/msm_adc.h>
#include <linux/ion.h>
#include "devices.h"
#include "timer.h"
#include "board-msm7x27a-regulator.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>
#include "pm-boot.h"
#include <mach/fih_headset.h> /* SW-MM-RC-JLO2535_headset_00 */
// MTD-BSP-Y.S-Enable drivers
#include <linux/fih_leds.h>
#include <linux/fih_vibrator.h>
// MTD-BSP-Y.S-Enable drivers

/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*[ */
#ifdef CONFIG_FIH_SW3_BATTERY
#include <linux/i2c/fih_bq27520.h>
#endif
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*] */

#include "board-msm7627a.h"
/* MTD-BSP-VT-SMEM-00+[ */
#include "linux/fih_hw_info.h" 
#include <linux/fih_sw_info.h> //MTD-SW3-KERNEL-DL-Fix_ioremap-00+
extern void fih_info_init(void);
/* MTD-BSP-VT-SMEM-00+] */

/* FIH-SW3-PERIPHERAL-CH-TouchDriver_Porting_2525-00++{ */
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340
#include <linux/cyttsp_i2c_tma340.h>
#endif 
/* FIH-SW3-PERIPHERAL-CH-TouchDriver_Porting_2525-00++} */
/* FIH-SW2-PERIPHERAL-FG-SENSOR-00+[ */
#ifdef CONFIG_FIH_MSENSOR_AKM8975
#include <linux/akm8975.h>
#endif
#ifdef CONFIG_FIH_ASENSOR_BMA250
#include <linux/bma250.h>
#endif
#ifdef CONFIG_FIH_PSENSOR_QPDSS702
#include <linux/qpdss702.h>
#endif
/* FIH-SW2-PERIPHERAL-FG-SENSOR-00+] */
#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x1F4000

#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_CORE,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0x02,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
};
#endif


#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};
#endif
//MTD-Conn-JC-JB2525_Porting-00+[

#if defined(CONFIG_BROADCOM_BCM4330_BTFM)
#define GPIO_BT_REG_ON     31
#define GPIO_BT_RST_N      32

#define BT_HOST_WAKE       33
#define HOST_WAKEUP_BT     114

#define GPIO_BTUART_RFR    43
#define GPIO_BTUART_CTS    44
#define GPIO_BTUART_RX     45
#define GPIO_BTUART_TX     46
#define GPIO_PCM_DIN       69
#define GPIO_PCM_DOUT      68
#define GPIO_PCM_SYNC      70
#define GPIO_PCM_BCLK      71

static struct platform_device bcm4330_bt_power_device = {
    .name = "bcm4330_bt_power",
    .id     = -1
};

static struct msm_gpio bt_config_power_on[] = {
    { GPIO_CFG(GPIO_BTUART_RFR, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BTUART_CTS, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,  GPIO_CFG_2MA), "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BTUART_RX,  2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,  GPIO_CFG_2MA), "UART1DM_RX" },
    { GPIO_CFG(GPIO_BTUART_TX,  2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,  GPIO_CFG_2MA), "UART1DM_TX" }
};

static struct msm_gpio bt_config_power_off[] = {
 
    { GPIO_CFG(GPIO_BTUART_RFR, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BTUART_CTS, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BTUART_RX,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_RX" },
    { GPIO_CFG(GPIO_BTUART_TX,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_TX" }
};
static int bluetooth_power(int on)
{
    int rc;

    printk("KERN_DEBUG %s: POWER %s\n", __FUNCTION__, on?"ON":"OFF");

    if (on)
    {
        rc = msm_gpios_enable(bt_config_power_on, ARRAY_SIZE(bt_config_power_on));
        if (rc < 0)
        {
            printk(KERN_DEBUG "%s: Power ON bluetooth failed.\n", __FUNCTION__);
            return rc;
        }

        mdelay(20);
        gpio_set_value(GPIO_BT_RST_N, 0);
        mdelay(20);
        gpio_set_value(GPIO_BT_RST_N, 1);     
        mdelay(20);

        gpio_set_value(BT_HOST_WAKE, 1);
        mdelay(20);

        gpio_set_value(GPIO_BT_REG_ON, 1);
        mdelay(100);       

        printk(KERN_DEBUG "%s: GPIO_BT_RST (%s)\n", __FUNCTION__, gpio_get_value(GPIO_BT_RST_N)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BT_REG_ON (%s)\n", __FUNCTION__, gpio_get_value(GPIO_BT_REG_ON)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: BT_HOST_WAKE !(%s)\n", __FUNCTION__, gpio_get_value(BT_HOST_WAKE)?"HIGH":"LOW");
    }
    else
    {
        rc = msm_gpios_enable(bt_config_power_off, ARRAY_SIZE(bt_config_power_off));
        if (rc < 0)
        {
            printk(KERN_DEBUG "%s: Power OFF bluetooth failed.\n", __FUNCTION__);
            return rc;
        }

        mdelay(20);
        gpio_set_value(BT_HOST_WAKE, 0);
        mdelay(20);
        gpio_set_value(GPIO_BT_RST_N, 0);
        mdelay(20);
        gpio_set_value(GPIO_BT_REG_ON, 0);
        mdelay(100);

        printk(KERN_DEBUG "%s: GPIO_BT_RST (%s)\n", __FUNCTION__, gpio_get_value(GPIO_BT_RST_N)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BT_REG_ON (%s)\n", __FUNCTION__, gpio_get_value(GPIO_BT_REG_ON)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: BT_HOST_WAKE !(%s)\n", __FUNCTION__, gpio_get_value(BT_HOST_WAKE)?"HIGH":"LOW");
    }

    return 0;
}

static void __init bcm4330_bt_power_init(void)
{
    gpio_set_value(GPIO_BT_REG_ON, 0);
    gpio_set_value(GPIO_BT_RST_N, 0);

    bcm4330_bt_power_device.dev.platform_data = &bluetooth_power;
}
#endif


#if defined(CONFIG_BROADCOM_BCM4330_BTFM) && defined(CONFIG_BROADCOM_BCM4330_BTFM_SLEEP)
static struct resource bluesleep_resources[] = {
    {
        .name = "gpio_host_wake",
        .start = HOST_WAKEUP_BT,
        .end = HOST_WAKEUP_BT,
        .flags = IORESOURCE_IO,
    },
    {
        .name = "gpio_ext_wake",
        .start = BT_HOST_WAKE,
        .end = BT_HOST_WAKE,
        .flags = IORESOURCE_IO,
    },
    {
        .name = "host_wake",
        .start = MSM_GPIO_TO_INT(HOST_WAKEUP_BT),
        .end = MSM_GPIO_TO_INT(HOST_WAKEUP_BT),
        .flags = IORESOURCE_IO,
    },
};

static struct platform_device bluesleep_device = {
    .name = "bluesleep",
    .id = -1,
    .num_resources = ARRAY_SIZE(bluesleep_resources),
    .resource = bluesleep_resources,
};
#endif //defined(CONFIG_BROADCOM_BCM4330_BTFM) && defined(CONFIG_BROADCOM_BCM4330_BTFM_SLEEP)
//MTD-Conn-JC-JB2525_Porting-00+]

/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*[ */
#ifdef CONFIG_FIH_SW3_BATTERY
static struct bq27520_platform_data bq27520_pdata = {
	.name		= "fuel-gauge",
	.vreg_name	= "8058_s3",
	.vreg_value	= 1800000,
	.soc_int	= 36,
	//.bi_tout	= GPIO_CAP_GAUGE_BI_TOUT,
	//.chip_en	= GPIO_BATT_GAUGE_EN,
	.enable_dlog	= 0, /* if enable coulomb counter logger */
	.bat_low    = 27,
	.ovp_flag   = 123,
};

static struct i2c_board_info msm_bq27520_board_info[] = {
    {
        I2C_BOARD_INFO("bq27520", 0xAA >> 1),
        .platform_data = &bq27520_pdata,
    },
#ifdef CONFIG_FIH_SW3_BQ275X0_ROMMODE
    {
        I2C_BOARD_INFO("bq275x0-RomMode", 0x16 >> 1),        
    },
#endif
};
#endif
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*] */
/*FIH-SW3-PERIPHERAL-CH-TouchDriver_Porting_2525-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340
#define TOUCH_DEVICE_VREG "emmc"
#define CY_RST_N_GPIO	  121 

#ifndef FIH_VIRTUAL_BUTTON 
static ssize_t tma340_virtual_keys_show(struct kobject *kobj,
                               struct kobj_attribute *attr, char *buf)
{
/* center: x: back: 65, menu: 415, home: 240, y: 912 SHIFT for moving down 14 points away  TOUCH AA */
				return snprintf(buf,74,
                      __stringify(EV_KEY) ":" __stringify(KEY_BACK)    ":65:912:130:102"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":240:912:130:102"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":415:912:130:102"
                   "\n");
}

static struct kobj_attribute tma340_virtual_keys_attr = {
        .attr = {
                .name = "virtualkeys.cyttsp_i2c_tma340",
                .mode = S_IRUGO,
        },
        .show = &tma340_virtual_keys_show,
};

static struct attribute *tma340_properties_attrs[] = {
        &tma340_virtual_keys_attr.attr,
        NULL
};

static struct attribute_group tma340_properties_attr_group = {
        .attrs = tma340_properties_attrs,
};
#endif 

//CONN-EC-WIFI-Porting-01+[
#define GPIO_WL_REG_ON     82

static unsigned wifi_config_gpio[] = {
        /*Regulator*/
        GPIO_CFG(GPIO_WL_REG_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

int wifi_power(int on)
{
    int ret = 0;
    printk(KERN_INFO "%s: POWER %s\n", __func__, on ? "ON" : "OFF");

    if (on) {
        ret = gpio_tlmm_config(wifi_config_gpio[0], GPIO_CFG_ENABLE);
        if (ret)
        {
            printk(KERN_ERR "%s: Failed to gpio_tlmm_config GPIO_WL_REG_ON(GPIO 82)\n", __func__);
            goto out;
        }

        gpio_set_value(GPIO_WL_REG_ON, 0);
        mdelay(20);
        gpio_set_value(GPIO_WL_REG_ON, 1);
        mdelay(20);
    } else {
        gpio_set_value(GPIO_WL_REG_ON, 0);
        mdelay(20);
    }
    printk(KERN_INFO "%s: GPIO_WL_REG_ON (%s)\n", __FUNCTION__, gpio_get_value(GPIO_WL_REG_ON)?"HIGH":"LOW");

out:
    return 0;
}
EXPORT_SYMBOL(wifi_power);

int bcm4330_wifi_resume(void)
{
    int ret = 0;
    ret = gpio_tlmm_config(wifi_config_gpio[0], GPIO_CFG_ENABLE);
    if (ret)
    {
        printk(KERN_ERR "%s: Failed to gpio_tlmm_config GPIO_WL_REG_ON(GPIO 82)\n", __func__);
        goto out;
    }
    gpio_set_value(GPIO_WL_REG_ON, 1);
    mdelay(20);
    printk(KERN_INFO "%s: PULL UP GPIO_WL_REG_ON, GPIO_WL_REG_ON(%s)\n", __func__, gpio_get_value(GPIO_WL_REG_ON)?"HIGH":"LOW");
out:
    return 0;
}
EXPORT_SYMBOL(bcm4330_wifi_resume);

int bcm4330_wifi_suspend(void)
{
    int ret = 0;

    ret = gpio_tlmm_config(wifi_config_gpio[0], GPIO_CFG_ENABLE);
    if (ret)
    {
        printk(KERN_ERR "%s: Failed to gpio_tlmm_config GPIO_WL_REG_ON(GPIO 82)\n", __func__);
        goto out;
    }
    gpio_set_value(GPIO_WL_REG_ON, 0);
    mdelay(20);
    printk(KERN_INFO "%s: PULL down GPIO_WL_REG_ON, GPIO_WL_REG_ON(%s)\n", __FUNCTION__, gpio_get_value(GPIO_WL_REG_ON)?"HIGH":"LOW");

out:
    return 0;
}
EXPORT_SYMBOL(bcm4330_wifi_suspend);
//CONN-EC-WIFI-Porting-01+]

static struct regulator_bulk_data regs_tch[] = {
	{ .supply = "emmc",   .min_uV = 3000000, .max_uV = 3000000 },
};

static unsigned touch_config_gpio[] = {
	/*IRQ*/
	GPIO_CFG(CY_I2C_IRQ_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	/*RESET*/
	GPIO_CFG(CY_RST_N_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*WAKU up chip*/
	GPIO_CFG(CY_I2C_IRQ_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

/* optional init function; set up IRQ GPIO;
 * call reference in platform data structure
 */
static int cyttsp_i2c_init(int on)
{ 
	int ret;
#ifndef FIH_VIRTUAL_BUTTON 
	struct kobject *properties_kobj;
#endif 

	/* add any special code to initialize any required system hw
	 * such as regulators or gpio pins
	 */
	//Power on vreg-emmc
	ret=regulator_bulk_get(NULL, ARRAY_SIZE(regs_tch), regs_tch);
	if (ret) 
	{
		printk(KERN_INFO"%s: vreg get failed \n", __func__); 
		return -EFAULT;
	}

	regulator_bulk_set_voltage(ARRAY_SIZE(regs_tch), regs_tch);
	ret=regulator_bulk_enable(ARRAY_SIZE(regs_tch), regs_tch);

	//request GPIO resource
	if (on) 
	{
		
		/* for MSM systems the call to gpio_direction_input can be
		 * replaced with the more explicit call:
		 */

		ret = gpio_tlmm_config(touch_config_gpio[0], GPIO_CFG_ENABLE);
		if (ret)
		{
			printk(KERN_ERR "%s: Failed to gpio_tlmm_config %d\n",
			       __func__, CY_I2C_IRQ_GPIO);
			return ret;
		}
		
		ret = gpio_request(CY_I2C_IRQ_GPIO, "CYTTSP IRQ GPIO");
		if (ret) 
		{
			printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			       __func__, CY_I2C_IRQ_GPIO);
			return ret;
		}

		gpio_direction_input(CY_I2C_IRQ_GPIO);

		/*Add to control reset pin*/
		ret = gpio_tlmm_config(touch_config_gpio[1], GPIO_CFG_ENABLE);
		if (ret)
		{
			printk(KERN_ERR "%s: Failed to gpio_tlmm_config %d\n",
			       __func__, CY_RST_N_GPIO);
			return ret;
		}
		
		ret = gpio_request(CY_RST_N_GPIO, "CYTTSP RST GPIO");
		if (ret) 
		{
			printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			       __func__, CY_RST_N_GPIO);
			return ret;
		}

		gpio_set_value_cansleep(CY_RST_N_GPIO, 1);
		
	} 
	else 
	{
		printk(KERN_INFO "gpio_free(%d)\n", CY_I2C_IRQ_GPIO);
		gpio_free(CY_I2C_IRQ_GPIO);
	}

#ifndef FIH_VIRTUAL_BUTTON 

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,&tma340_properties_attr_group);

	if (!properties_kobj || ret)
		printk(KERN_ERR "%s: failed to create board_properties\n",__func__);

#endif 

	return 0;
}

#ifdef reset_cmd
static int cyttsp_reset(void)
{
	gpio_direction_output(CY_RST_N_GPIO, 0);
	msleep(10);
	gpio_direction_output(CY_RST_N_GPIO, 1);
	return 0;
}
#endif

static int cyttsp_i2c_wakeup(void)
{
	//assert from host to chip by interrupt pin
	int ret =0;
	ret = gpio_tlmm_config(touch_config_gpio[2], GPIO_CFG_ENABLE);
	if (ret)
	{
		printk(KERN_ERR "%s: Failed to gpio_tlmm_config %d\n",
		       __func__, CY_I2C_IRQ_GPIO);
	}
	gpio_direction_output(CY_I2C_IRQ_GPIO, 1);
	msleep(10);
	gpio_direction_output(CY_I2C_IRQ_GPIO, 0);
	msleep(10);
	gpio_direction_output(CY_I2C_IRQ_GPIO, 1);
	ret = gpio_tlmm_config(touch_config_gpio[0], GPIO_CFG_ENABLE);
	if (ret)
	{
		printk(KERN_ERR "%s: Failed to gpio_tlmm_config %d\n",
		       __func__, CY_I2C_IRQ_GPIO);
	}

	gpio_direction_input(CY_I2C_IRQ_GPIO);

	return 0;
}

static struct cyttsp_platform_data cypress_i2c_ttsp_platform_data = {
	.wakeup = cyttsp_i2c_wakeup, /*wake up IC from deep sleep, assert interrupt pin by host-side*/
	.init = cyttsp_i2c_init, /*init power and gpio resource*/
	.mt_sync = input_mt_sync, /*sync mutli-touch event*/
#ifdef reset_cmd
	.reset = cyttsp_reset,
#endif
#ifdef SECOND_MODULE
	.module_1 = "    ELK",
	.module_2 = "  SMAC",
#endif
	.maxx = 320, /*Parameter don't use*/
	.maxy = 480, /*Parameter don't use*/
	.flags = 0, /*0x05, reverse x / y or not, 1:yes, 0:no*/
	.gen = CY_GEN3, /*generation */
	.use_st = 0, /*support single-touch*/
	.use_mt = 1, /*support multi-touch*/
	.use_trk_id = 1, 
	.use_hndshk = 0, /*FIH-MTD-PERIPHERAL-CH-Handshake-00++*/
	.use_timer = 0, /*use polling*/
	.use_sleep = 1, /*deep sleep mode for early suspend/late resume*/
	.use_gestures = 0, /*use gestures function*/
	.use_load_file = 1,
	.use_force_fw_update = 1, /*useless value*/
	.use_virtual_keys = 0, /*use virtual key function*/
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP_NONE | CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT, //active refresh interval, page 13.
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT, //touch timeout, page 13.
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT, //low power interval, page 13.
	.name = CY_I2C_NAME,
	.irq_gpio = 86, /*MSM_GPIO_TO_INT(CY_I2C_IRQ_GPIO), ISR Number*/
	.row_pins_number = 20, /*AREA_JUDGE*/
	.col_pins_number = 11, /*AREA_JUDGE*/
};
#endif
/*FIH-SW3-PERIPHERAL-CH-TouchDriver_Porting_2525-00++]*/

/* FIH-SW2-PERIPHERAL-FG-SENSOR-00+[ */
#ifdef CONFIG_FIH_MSENSOR_AKM8975
static int akm8975_gpio_init(void)
{
	int COMPASS_INT = GPIO_COMPASS_INT;
	if (gpio_request(COMPASS_INT, "COMPASS_INT"))
	{
		printk(KERN_ERR "[MSENSOR] %s(%d): Request GPIO(%d) failed.\n", __FUNCTION__, __LINE__, COMPASS_INT);
		return -EIO;
	}
	if (gpio_tlmm_config(GPIO_CFG(COMPASS_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
	{
		gpio_free(COMPASS_INT);
		printk(KERN_ERR "[MSENSOR] %s(%d): Config GPIO(%d) failed.\n", __FUNCTION__, __LINE__, COMPASS_INT);
		return -EIO;
	}
	gpio_direction_input(COMPASS_INT);

	printk("[MSENSOR] %s(%d): GPIO(%d) Done.\n", __FUNCTION__, __LINE__, COMPASS_INT);
	return 0;
}

static struct akm8975_platform_data akm8975_platform_data = {
	.gpio_init = akm8975_gpio_init,
	.layout = 2,
};
#endif

#ifdef CONFIG_FIH_ASENSOR_BMA250
static int bma250_gpio_init(void)
{
	int GS_INT = GPIO_GS_INT;
	if (gpio_request(GS_INT, "GS_INT"))
	{
		ASENSOR_DEBUG(LEVEL0, "Request GPIO(%d) failed.", GS_INT);
		return -EIO;
	}
	if (gpio_tlmm_config(GPIO_CFG(GS_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
	{
		gpio_free(GS_INT);
		ASENSOR_DEBUG(LEVEL0, "Config GPIO(%d) failed.", GS_INT);
		return -EIO;
	}
	gpio_direction_input(GS_INT);
	gpio_set_value_cansleep(GS_INT, 0);

	ASENSOR_DEBUG(LEVEL0, "GPIO(%d) Done.", GS_INT);
	return 0;
}

static struct bma250_platform_data bma250_platform_data = {
	.gpio_init = bma250_gpio_init,
	.layout = 2,
};
#endif

#ifdef CONFIG_FIH_PSENSOR_QPDSS702
static int qpdss702_gpio_init(void)
{
	int ALPS_OUT = GPIO_ALPS_OUT;
	if (gpio_request(ALPS_OUT, "ALPS_OUT"))
	{
		PSENSOR_DEBUG(LEVEL0, "Request GPIO(%d) failed.", ALPS_OUT);
		return -EIO;
	}
	if (gpio_tlmm_config(GPIO_CFG(ALPS_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
	{
		PSENSOR_DEBUG(LEVEL0, "Config GPIO(%d) failed.", ALPS_OUT);
		return -EIO;
	}
	gpio_direction_input(ALPS_OUT);

	PSENSOR_DEBUG(LEVEL0, "GPIO(%d) Done.", ALPS_OUT);
	return 0;
}

static struct qpdss702_platform_data qpdss702_platform_data = {
	.gpio_init = qpdss702_gpio_init,
	.sensitivity = QPDSS207_SENSITIVITY_LEVEL2,	/*FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-03*/
};
#endif
/* FIH-SW2-PERIPHERAL-FG-SENSOR-00+] */

#ifdef CONFIG_I2C
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
#ifdef CONFIG_GPIO_SX150X
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
#endif
/*FIH-SW3-PERIPHERAL-CH-TouchDriver_Porting_2525-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cypress_i2c_ttsp_platform_data,
	},
#endif
/*FIH-SW3-PERIPHERAL-CH-TouchDriver_Porting_2525-00++]*/
/* FIH-SW2-PERIPHERAL-FG-SENSOR-00+[ */
#ifdef CONFIG_FIH_MSENSOR_AKM8975
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x0C),
		//.flags = I2C_CLIENT_WAKE,
		.platform_data = &akm8975_platform_data,
		//.irq = MSM_GPIO_TO_INT(GPIO_COMPASS_INT),
	},
#endif
#ifdef CONFIG_FIH_ASENSOR_BMA250
	{
		I2C_BOARD_INFO(ASENSOR_NAME, 0x18),
		.platform_data = &bma250_platform_data,
		.irq = MSM_GPIO_TO_INT(GPIO_GS_INT),
	},
#endif
#ifdef CONFIG_FIH_PSENSOR_QPDSS702
	{
		I2C_BOARD_INFO(PSENSOR_NAME, 0x54),
		.platform_data = &qpdss702_platform_data,
		.irq = MSM_GPIO_TO_INT(GPIO_ALPS_OUT),
	},
#endif
/* FIH-SW2-PERIPHERAL-FG-SENSOR-00+] */
};

static void __init register_i2c_devices(void)
{

#ifdef CONFIG_GPIO_SX150X
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf() ||
			machine_is_msm8625_surf())
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];
#endif
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
}
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}
/*FIH-MTD-PERIPHERAL-CH-I2C_Clock-00++[*/
static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 400000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 400000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};
/*FIH-MTD-PERIPHERAL-CH-I2C_Clock-00++]*/
#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x2300000
#define MSM7x25A_MSM_PMEM_MDP_SIZE       0x1500000

/*MTD-MM-SL-CameraPorting-00*{ */
#define MSM_PMEM_ADSP_SIZE      0x1800000 //0x1200000 
#define MSM7x25A_MSM_PMEM_ADSP_SIZE      0x1800000 //0xB91000
/*MTD-MM-SL-CameraPorting-00*} */
#define CAMERA_ZSL_SIZE		(SZ_1M * 60)
#endif

#ifdef CONFIG_ION_MSM
#define MSM_ION_HEAP_NUM        4
static struct platform_device ion_dev;
static int msm_ion_camera_size;
static int msm_ion_audio_size;
static int msm_ion_sf_size;
#endif


static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s: could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};

/* 8625 PM platform data */
static struct msm_pm_platform_data msm8625_pm_data[MSM_PM_SLEEP_MODE_NR * 2] = {
	/* CORE0 entries */
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 0,
					.latency = 16000,
					.residency = 20000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 0,
					.latency = 12000,
					.residency = 20000,
	},

	/* picked latency & redisdency values from 7x30 */
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 0,
					.latency = 500,
					.residency = 6000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 10,
	},

	/* picked latency & redisdency values from 7x30 */
	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 0,
					.latency = 500,
					.residency = 6000,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 10,
	},

};

static struct msm_pm_boot_platform_data msm_pm_8625_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_REMAP_BOOT_ADDR,
	.v_addr = MSM_CFG_CTL_BASE,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
	.request_region = request_fmem_c_region,
	.release_region = release_fmem_c_region,
	.reusable = 1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*[ */
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 3200,
	.voltage_max_design     = 4200,
	.voltage_fail_safe      = 3340,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = NULL,	
};
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*] */

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

/* MTD-Kernel-HC-Remove_smsc911x-00+[ */
#ifdef CONFIG_SMSC911X

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#endif
/* MTD-Kernel-HC-Remove_smsc911x-00+] */

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata = {
	.dev_names = msm_adc_surf_device_names,
	.num_adc = ARRAY_SIZE(msm_adc_surf_device_names),
	.target_hw = MSM_8x25,
};

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

/* MTD-Kernel-HC-Remove_smsc911x-00+[ */
#ifdef CONFIG_SMSC911X

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}

#endif
/* MTD-Kernel-HC-Remove_smsc911x-00+] */

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) \
		&& defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)
static struct msm_gpio uart2dm_gpios[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

static struct fmem_platform_data fmem_pdata;

static struct platform_device fmem_device = {
	.name = "fmem",
	.id = 1,
	.dev = { .platform_data = &fmem_pdata },
};
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*[ */
#ifdef CONFIG_FIH_SW3_BATTERY
static void __init fih_bq27520_driver_init(void)
{
    i2c_register_board_info(1, &msm_bq27520_board_info[0], 1);

    #ifdef CONFIG_FIH_SW3_BQ275X0_ROMMODE
        i2c_register_board_info(1, &msm_bq27520_board_info[1], 1);        
    #endif
}
#endif
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*] */

static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_uart1,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
};

static struct platform_device *msm8625_rumi3_devices[] __initdata = {
	&msm8625_device_dmov,
	&msm8625_device_smd,
	&msm8625_device_uart1,
	&msm8625_gsbi0_qup_i2c_device,
};

static struct platform_device *msm7627a_surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,

/* MTD-Kernel-HC-Remove_smsc911x-00+[ */
#ifdef CONFIG_SMSC911X
	&smsc911x_device,
#endif
/* MTD-Kernel-HC-Remove_smsc911x-00+] */

	&msm_kgsl_3d0,
};

static struct platform_device *common_devices[] __initdata = {
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&fmem_device,
	&msm_device_nand,
	&msm_device_snd,
	&msm_device_cad,
	&msm_device_adspdec,
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
	&msm_batt_device,
	&msm_adc_device,
#ifdef CONFIG_ION_MSM
	&ion_dev,
#endif
   //MTD-Conn-JC-JB2525_Porting-00+[
  #if defined(CONFIG_BROADCOM_BCM4330_BTFM)
	&bcm4330_bt_power_device,
	#endif
	#if defined(CONFIG_BROADCOM_BCM4330_BTFM) && defined(CONFIG_BROADCOM_BCM4330_BTFM_SLEEP)
	&bluesleep_device,//MTD-Conn-JC-BTSleep-00+
	#endif
	//MTD-Conn-JC-JB2525_Porting-00+]


};

static struct platform_device *msm8625_surf_devices[] __initdata = {
	&msm8625_device_dmov,
	&msm8625_device_uart1,
	&msm8625_device_uart_dm1,
	&msm8625_device_uart_dm2,
	&msm8625_gsbi0_qup_i2c_device,
	&msm8625_gsbi1_qup_i2c_device,
	&msm8625_device_smd,
	&msm8625_device_otg,
	&msm8625_device_gadget_peripheral,
	&msm8625_kgsl_3d0,
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void fix_sizes(void)
{
	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		pmem_mdp_size = MSM7x25A_MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM7x25A_MSM_PMEM_ADSP_SIZE;
	} else {
		pmem_mdp_size = MSM_PMEM_MDP_SIZE;
		pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
	}

	if (get_ddr_size() > SZ_512M)
		pmem_adsp_size = CAMERA_ZSL_SIZE;
#ifdef CONFIG_ION_MSM
	msm_ion_camera_size = pmem_adsp_size;
	msm_ion_audio_size = (MSM_PMEM_AUDIO_SIZE + PMEM_KERNEL_EBI1_SIZE);
	msm_ion_sf_size = pmem_mdp_size;
#endif
}

#ifdef CONFIG_ION_MSM
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct ion_co_heap_pdata co_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
};
#endif

/**
 * These heaps are listed in the order they will be allocated.
 * Don't swap the order unless you know what you are doing!
 */
static struct ion_platform_data ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.has_outer_cache = 1,
	.heaps = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		/* PMEM_ADSP = CAMERA */
		{
			.id	= ION_CAMERA_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_CAMERA_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
		/* PMEM_AUDIO */
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
		/* PMEM_MDP = SF */
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *)&co_ion_pdata,
		},
#endif
	}
};

static struct platform_device ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &ion_pdata },
};
#endif

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	//MTD-SW3-KERNEL-DL-Fix_ioremap-00+[
	[MEMTYPE_EBI1_FIH] = {
		.start	=	RAM_CONSOLE_PHYS,
		.size	=	0x300000,	/* reserve 1M for each RAM_CONSOLE, MLOG, and FB for splash screen of ramdump*/
		.flags	=	MEMTYPE_FLAGS_FIXED,
	},
	//MTD-SW3-KERNEL-DL-Fix_ioremap-00+]
};

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static struct android_pmem_platform_data *pmem_pdata_array[] __initdata = {
		&android_pmem_adsp_pdata,
		&android_pmem_audio_pdata,
		&android_pmem_pdata,
};
#endif
#endif

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	unsigned int i;
	unsigned int reusable_count = 0;

	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;

	fmem_pdata.size = 0;
	fmem_pdata.align = PAGE_SIZE;

	/* Find pmem devices that should use FMEM (reusable) memory.
	 */
	for (i = 0; i < ARRAY_SIZE(pmem_pdata_array); ++i) {
		struct android_pmem_platform_data *pdata = pmem_pdata_array[i];

		if (!reusable_count && pdata->reusable)
			fmem_pdata.size += pdata->size;

		reusable_count += (pdata->reusable) ? 1 : 0;

		if (pdata->reusable && reusable_count > 1) {
			pr_err("%s: Too many PMEM devices specified as reusable. PMEM device %s was not configured as reusable.\n",
				__func__, pdata->name);
			pdata->reusable = 0;
		}
	}
#endif
#endif
}

#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}
#endif
#endif

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
#ifndef CONFIG_MSM_MULTIMEDIA_USE_ION
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(pmem_pdata_array); ++i)
		reserve_memory_for(pmem_pdata_array[i]);

	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
#endif
}

static void __init size_ion_devices(void)
{
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	ion_pdata.heaps[1].size = msm_ion_camera_size;
	ion_pdata.heaps[2].size = msm_ion_audio_size;
	ion_pdata.heaps[3].size = msm_ion_sf_size;
#endif
}

static void __init reserve_ion_memory(void)
{
#if defined(CONFIG_ION_MSM) && defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_camera_size;
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_audio_size;
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += msm_ion_sf_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	fix_sizes();
	size_pmem_devices();
	reserve_pmem_memory();
	size_ion_devices();
	reserve_ion_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

static void __init msm8625_reserve(void)
{
	msm7x27a_reserve();
	memblock_remove(MSM8625_SECONDARY_PHYS, SZ_8);
	memblock_remove(MSM8625_WARM_BOOT_PHYS, SZ_32);
}

static void __init msm7x27a_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

static void __init msm8625_device_i2c_init(void)
{
	msm8625_gsbi0_qup_i2c_device.dev.platform_data =
		&msm_gsbi0_qup_i2c_pdata;
	msm8625_gsbi1_qup_i2c_device.dev.platform_data =
		&msm_gsbi1_qup_i2c_pdata;
}

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf() ||
		machine_is_msm7625a_surf() || machine_is_msm8625_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

//MTD-BSP-Y.S-Enable drivers
// For LED
static struct led_device_data	fih_led_DP_data[] =
{
	{
		.name	= "button-backlight",
		.id	= 3,
		.use_hw	= LED_HW_PMIC_MPP,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_MPP_8,
				.lpg_out		= 3,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.current_sink	= PM_MPP__I_SINK__LEVEL_15mA,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "red",
		.id	= 0,
		.use_hw	= LED_HW_PMIC_MPP,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_MPP_7,
				.lpg_out		= 2,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.current_sink	= PM_MPP__I_SINK__LEVEL_5mA,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "green",
		.id	= 1,
		.use_hw	= LED_HW_PMIC_MPP,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_MPP_3,
				.lpg_out		= 1,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.current_sink	= PM_MPP__I_SINK__LEVEL_5mA,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "blue",
		.id	= 2,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_1,
				.lpg_out		= 4,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "sub-red",
		.id	= 4,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_5,
				.lpg_out		= 5,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 16,
				.lut_table_end	= 31,
			}
		},
	},
	{
		.name	= "sub-green",
		.id	= 5,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_6,
				.lpg_out		= 6,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 32,
				.lut_table_end	= 47,
			}
		},
	},
	{
		.name	= "sub-blue",
		.id	= 6,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_8,
				.lpg_out		= 7,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 48,
				.lut_table_end	= 63,
			}
		},
	},
};

static struct led_device_data	fih_led_SP2_data[] =
{
	{
		.name	= "button-backlight",
		.id	= 3,
		.use_hw	= LED_HW_PMIC_MPP,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_MPP_3,
				.lpg_out		= 1,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.current_sink	= PM_MPP__I_SINK__LEVEL_15mA,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "red",
		.id	= 0,
		.use_hw	= LED_HW_PMIC_MPP,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_MPP_8,
				.lpg_out		= 3,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.current_sink	= PM_MPP__I_SINK__LEVEL_5mA,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "green",
		.id	= 1,
		.use_hw	= LED_HW_MSM_GPIO,
		.detail	=
		{
			.gpio_data	=
			{
				.msm_pin		= 129,
				.led_on_level	= 1,
				.led_off_level	= 0,
			}
		},
	},
	{
		.name	= "blue",
		.id	= 2,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_1,
				.lpg_out		= 4,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.invert		= 1,
				.lut_table_start	= 0,
				.lut_table_end	= 15,
			}
		},
	},
	{
		.name	= "sub-red",
		.id	= 4,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_5,
				.lpg_out		= 5,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 16,
				.lut_table_end	= 31,
			}
		},
	},
	{
		.name	= "sub-green",
		.id	= 5,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_6,
				.lpg_out		= 6,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 32,
				.lut_table_end	= 47,
			}
		},
	},
	{
		.name	= "sub-blue",
		.id	= 6,
		.use_hw	= LED_HW_PMIC_GPIO,
		.detail	=
		{
			.pmic_data	=
			{
				.pmic_pin	= PM_GPIO_8,
				.lpg_out		= 7,
				.on_off_pwm	= 250,
				.blinking_pwm1	= 0,
				.blinking_pwm2	= 511,
				.pwm_clock	= PM_LPG_CLK_SELECT_19P2M,
				.pwm_div_value	= 6,
				.pwm_div_exp	= 3,
				.blinking_time1	= 500,
				.blinking_time2	= 500,
				.interval	= 32,
				.toggle_up_down	= 1,
				.ramp_loop	= 1,
				.ramp_up_down	= 1,
				.use_table	= 0,
				.lut_table_start	= 48,
				.lut_table_end	= 63,
			}
		},
	},
};

struct leds_device_data	fih_leds_data;

static struct platform_device fih_device_leds = {
	.name   = "fih_leds",
	.id	= -1,
	.dev	=
	{
		.platform_data = &fih_leds_data,
	},
};

// For Vibrator
static struct vib_device_data	fih_vibrator_data =
{
	.name	= "pmic-vib",
	.level	= 3000,
};

static struct platform_device fih_device_vibrator = {
	.name   = "fih_vibrator",
	.id	= -1,
	.dev	=
	{
		.platform_data = &fih_vibrator_data,
	},
};
// MTD-BSP-Y.S-Enable drivers

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

static void msm_adsp_add_pdev(void)
{
	int rc = 0;
	struct rpc_board_dev *rpc_adsp_pdev;

	rpc_adsp_pdev = kzalloc(sizeof(struct rpc_board_dev), GFP_KERNEL);
	if (rpc_adsp_pdev == NULL) {
		pr_err("%s: Memory Allocation failure\n", __func__);
		return;
	}
	rpc_adsp_pdev->prog = ADSP_RPC_PROG;

	if (cpu_is_msm8625())
		rpc_adsp_pdev->pdev = msm8625_device_adsp;
	else
		rpc_adsp_pdev->pdev = msm_adsp_device;
	rc = msm_rpc_add_board_dev(rpc_adsp_pdev, 1);
	if (rc < 0) {
		pr_err("%s: return val: %d\n",	__func__, rc);
		kfree(rpc_adsp_pdev);
	}
}
/* SW-MM-RC-JLO2535_headset_00 [ */
static struct h2w_platform_data fih_headset_data = {
	.cable_in1 = 18,
	.cable_in2 = 29,
};

static struct platform_device fih_device_headset = {
	.name   = "headset_sensor",
	.id     = -1,
	.dev    = {
		.platform_data = &fih_headset_data,
	},
};
/* SW-MM-RC-JLO2535_headset_00 ] */
static void __init msm7627a_rumi3_init(void)
{
	msm7x27a_init_ebi2();
	platform_add_devices(rumi_sim_devices,
			ARRAY_SIZE(rumi_sim_devices));
}

static void __init msm8625_rumi3_init(void)
{
	msm7x2x_misc_init();
	msm_adsp_add_pdev();
	msm8625_device_i2c_init();
	platform_add_devices(msm8625_rumi3_devices,
			ARRAY_SIZE(msm8625_rumi3_devices));

	msm_pm_set_platform_data(msm8625_pm_data,
			 ARRAY_SIZE(msm8625_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_8625_boot_pdata));
	msm8x25_spm_device_init();
	msm_pm_register_cpr_ops();
}

#define UART1DM_RX_GPIO		45
/* FIH-SW3-KERNEL-TH-add_last_alog-00+[ */
#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[1] = {
        [0] = {
                .start  = RAM_CONSOLE_PHYS,
                .end    = RAM_CONSOLE_PHYS + RAM_CONSOLE_SIZE - 1,
                .flags  = IORESOURCE_MEM,
        },
};

static struct platform_device ram_console_device = {
        .name   = "ram_console",
        .id     = 0,
        .num_resources  = ARRAY_SIZE(ram_console_resources),
        .resource       = ram_console_resources,
};

#ifdef CONFIG_FEATURE_FIH_SW3_LAST_ALOG
static struct resource alog_ram_console_resources[4] = {
        [0] = {
        .name = "alog_main_buffer",
                .start  = ALOG_RAM_CONSOLE_PHYS_MAIN,
                .end    = ALOG_RAM_CONSOLE_PHYS_MAIN + ALOG_RAM_CONSOLE_SIZE_MAIN - 1,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
            .name = "alog_radio_buffer",
                .start  = ALOG_RAM_CONSOLE_PHYS_RADIO,
                .end    = ALOG_RAM_CONSOLE_PHYS_RADIO + ALOG_RAM_CONSOLE_SIZE_RADIO - 1,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
        .name = "alog_events_buffer",
                .start  = ALOG_RAM_CONSOLE_PHYS_EVENTS,
                .end    = ALOG_RAM_CONSOLE_PHYS_EVENTS + ALOG_RAM_CONSOLE_SIZE_EVENTS - 1,
                .flags  = IORESOURCE_MEM,
        },
        [3] = {
		.name = "alog_system_buffer",
                .start  = ALOG_RAM_CONSOLE_PHYS_SYSTEM,
                .end    = ALOG_RAM_CONSOLE_PHYS_SYSTEM + ALOG_RAM_CONSOLE_SIZE_SYSTEM - 1,
                .flags  = IORESOURCE_MEM,
        },
};

static struct platform_device alog_ram_console_device = {
        .name   = "alog_ram_console",
        .id     = 0,
        .num_resources  = ARRAY_SIZE(alog_ram_console_resources),
        .resource       = alog_ram_console_resources,
};
#endif /* end of #ifdef CONFIG_FEATURE_FIH_SW3_LAST_ALOG */
#endif /* end of #ifdef CONFIG_ANDROID_RAM_CONSOLE */
/* FIH-SW3-KERNEL-TH-add_last_alog-00+] */

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static int __init msm7x27a_init_ar6000pm(void)
{
	msm_wlan_ar6000_pm_device.dev.platform_data = &ar600x_wlan_power;
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}
#else
static int __init msm7x27a_init_ar6000pm(void) { return 0; }
#endif

static void __init msm7x27a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}

static void __init msm7x27a_add_footswitch_devices(void)
{
	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);
}

static void __init msm7x27a_add_platform_devices(void)
{
	if (machine_is_msm8625_surf() || machine_is_msm8625_ffa()) {
		platform_add_devices(msm8625_surf_devices,
			ARRAY_SIZE(msm8625_surf_devices));
	} else {
		platform_add_devices(msm7627a_surf_ffa_devices,
			ARRAY_SIZE(msm7627a_surf_ffa_devices));
	}

	platform_add_devices(common_devices,
			ARRAY_SIZE(common_devices));
}

static void __init msm7x27a_uartdm_config(void)
{
	msm7x27a_cfg_uart2dm_serial();
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	if (cpu_is_msm8625())
		msm8625_device_uart_dm1.dev.platform_data =
			&msm_uart_dm1_pdata;
	else
		msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
}

static void __init msm7x27a_otg_gadget(void)
{
	if (cpu_is_msm8625()) {
		msm_otg_pdata.swfi_latency =
		msm8625_pm_data[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
		msm8625_device_otg.dev.platform_data = &msm_otg_pdata;
		msm8625_device_gadget_peripheral.dev.platform_data =
			&msm_gadget_pdata;
	} else {
		msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data[
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
		msm_device_otg.dev.platform_data = &msm_otg_pdata;
		msm_device_gadget_peripheral.dev.platform_data =
			&msm_gadget_pdata;
	}
}

static void __init msm7x27a_pm_init(void)
{
	if (machine_is_msm8625_surf() || machine_is_msm8625_ffa()) {
		msm_pm_set_platform_data(msm8625_pm_data,
				ARRAY_SIZE(msm8625_pm_data));
		BUG_ON(msm_pm_boot_init(&msm_pm_8625_boot_pdata));
		msm8x25_spm_device_init();
		msm_pm_register_cpr_ops();
	} else {
		msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));
		BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	}

	msm_pm_register_irqs();
}

static void __init msm7x2x_init(void)
{
	fih_get_oem_info(); /* MTD-BSP-VT-SMEM-00+ */
	fih_info_init(); /* MTD-BSP-VT-INFO-00+ */

	msm7x2x_misc_init();

	/* Initialize regulators first so that other devices can use them */
	msm7x27a_init_regulators();
	msm_adsp_add_pdev();
	if (cpu_is_msm8625())
		msm8625_device_i2c_init();
	else
		msm7x27a_device_i2c_init();
	msm7x27a_init_ebi2();
	msm7x27a_uartdm_config();

	msm7x27a_otg_gadget();

/* MTD-Kernel-HC-Remove_smsc911x-00+[ */
#ifdef CONFIG_SMSC911X
	msm7x27a_cfg_smsc911x();
#endif
/* MTD-Kernel-HC-Remove_smsc911x-00+] */


	msm7x27a_add_footswitch_devices();
	msm7x27a_add_platform_devices();
	/* Ensure ar6000pm device is registered before MMC/SDC */
	msm7x27a_init_ar6000pm();
	msm7627a_init_mmc();
	msm_fb_add_devices();
#ifdef CONFIG_USB_EHCI_MSM_72K/*MTD-CONN-EH_USBPORTING-01+*/	
	msm7x2x_init_host();
#endif
	msm7x27a_pm_init();
	register_i2c_devices();
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	msm7627a_bt_power_init();
#endif
	msm7627a_camera_init();
	msm7627a_add_io_devices();
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*[ */
#ifdef CONFIG_FIH_SW3_BATTERY
	fih_bq27520_driver_init();
#endif
/* FIH-SW3-KERNEL-PK-Battery_Gauge_Porting-00*] */

	// MTD-BSP-Y.S-Enable drivers
	switch( fih_get_product_phase() )
	{
		case	Phase_DP :
		case	Phase_SP :
			printk( "LED : Load DP/SP setting" );
			fih_leds_data.device_data	= fih_led_DP_data;
			fih_leds_data.count	= sizeof( fih_led_DP_data ) / sizeof( *fih_led_DP_data );
			break;

		case	Phase_EVB :
		case	Phase_SP2 :
		default :
			printk( "LED : Load EVB/SP2 setting" );
			if( gpio_tlmm_config( GPIO_CFG( 129, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ), GPIO_CFG_ENABLE ) )
				pr_err( "GKEY: gpio_tlmm_config(129) failed\n" );
			fih_leds_data.device_data	= fih_led_SP2_data;
			fih_leds_data.count	= sizeof( fih_led_SP2_data ) / sizeof( *fih_led_SP2_data );
			break;
	}

	platform_device_register( &fih_device_leds );
	platform_device_register( &fih_device_vibrator );
	// MTD-BSP-Y.S-Enable drivers
platform_device_register( &fih_device_headset); /* SW-MM-RC-JLO2535_headset_00 */

/* FIH-SW3-KERNEL-TH-add_last_alog-00+[ */
#ifdef CONFIG_ANDROID_RAM_CONSOLE
    platform_device_register(&ram_console_device);
	#ifdef CONFIG_FEATURE_FIH_SW3_LAST_ALOG
	    platform_device_register(&alog_ram_console_device);
	#endif
#endif
/* FIH-SW3-KERNEL-TH-add_last_alog-00+] */
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
	/*8x25 kgsl initializations*/
	msm8x25_kgsl_3d0_init();
	//MTD-Conn-JC-JB2525_Porting-00+[
	#if defined(CONFIG_BROADCOM_BCM4330_BTFM)
	bcm4330_bt_power_init();
	#endif
	//MTD-Conn-JC-JB2525_Porting-00+]
    wifi_power(1);  //CONN-EC-WIFI-Porting-02+
}

static void __init msm7x2x_init_early(void)
{
	msm_msm7627a_allocate_memory_regions();
}

MACHINE_START(MSM7X27A_RUMI3, "QCT MSM7x27a RUMI3")
	.atag_offset	= 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7627a_rumi3_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
	.atag_offset	= 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
	.atag_offset	= 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_SURF, "QCT MSM7625a SURF")
	.atag_offset    = 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_FFA, "QCT MSM7625a FFA")
	.atag_offset    = 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM8625_RUMI3, "QCT MSM8625 RUMI3")
	.atag_offset    = 0x100,
	.map_io         = msm8625_map_io,
	.reserve        = msm8625_reserve,
	.init_irq       = msm8625_init_irq,
	.init_machine   = msm8625_rumi3_init,
	.timer          = &msm_timer,
	.handle_irq	= gic_handle_irq,
MACHINE_END
MACHINE_START(MSM8625_SURF, "QCT MSM8625 SURF")
	.atag_offset    = 0x100,
	.map_io         = msm8625_map_io,
	.reserve        = msm8625_reserve,
	.init_irq       = msm8625_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= gic_handle_irq,
MACHINE_END
MACHINE_START(MSM8625_FFA, "QCT MSM8625 FFA")
	.atag_offset    = 0x100,
	.map_io         = msm8625_map_io,
	.reserve        = msm8625_reserve,
	.init_irq       = msm8625_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= gic_handle_irq,
MACHINE_END
