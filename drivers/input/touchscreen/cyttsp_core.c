/* Source for:
 * Cypress TrueTouch(TM) Standard Product touchscreen driver.
 * drivers/input/touchscreen/cyttsp_core.c
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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/earlysuspend.h>
#include <linux/cyttsp_i2c_tma340.h>
#include <linux/ctype.h>
#include "cyttsp_core.h"
#include <mach/vreg.h>
#include <linux/fih_hw_info.h> /*FIH-MTD-PERIPHERAL-CH-MES-02++[*/

/* rely on kernel input.h to define Multi-Touch capability */
#ifndef ABS_MT_TRACKING_ID
/* define only if not defined already by system; */
/* value based on linux kernel 2.6.30.10 */
#define ABS_MT_TRACKING_ID (ABS_MT_BLOB_ID + 1)
#endif /* ABS_MT_TRACKING_ID */

#define	TOUCHSCREEN_TIMEOUT (msecs_to_jiffies(28))
/* Bootloader File 0 offset */
#define CY_BL_FILE0       0x00
/* Bootloader command directive */
#define CY_BL_CMD         0xFF
/* Bootloader Enter Loader mode */
#define CY_BL_ENTER       0x38
/* Bootloader Write a Block */
#define CY_BL_WRITE_BLK   0x39
/* Bootloader Terminate Loader mode */
#define CY_BL_TERMINATE   0x3B
/* Bootloader Exit and Verify Checksum command */
#define CY_BL_EXIT        0xA5
/* Bootloader default keys */
#define CY_BL_KEY0 0
#define CY_BL_KEY1 1
#define CY_BL_KEY2 2
#define CY_BL_KEY3 3
#define CY_BL_KEY4 4
#define CY_BL_KEY5 5
#define CY_BL_KEY6 6
#define CY_BL_KEY7 7

#define CY_DIFF(m, n)               ((m) != (n))
#define GET_NUM_TOUCHES(x)          ((x) & 0x0F)
#define GET_TOUCH1_ID(x)            (((x) & 0xF0) >> 4)
#define GET_TOUCH2_ID(x)            ((x) & 0x0F)
#define GET_TOUCH3_ID(x)            (((x) & 0xF0) >> 4)
#define GET_TOUCH4_ID(x)            ((x) & 0x0F)
#define IS_LARGE_AREA(x)            (((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define FLIP_DATA_FLAG              0x01
#define REVERSE_X_FLAG              0x02
#define REVERSE_Y_FLAG              0x04
#define FLIP_DATA(flags)            ((flags) & FLIP_DATA_FLAG)
#define REVERSE_X(flags)            ((flags) & REVERSE_X_FLAG)
#define REVERSE_Y(flags)            ((flags) & REVERSE_Y_FLAG)
#define FLIP_XY(x, y)      {typeof(x) tmp; tmp = (x); (x) = (y); (y) = tmp; }
#define INVERT_X(x, xmax)           ((xmax) - (x))
#define INVERT_Y(y, ymax)           ((ymax) - (y))
#define SET_HSTMODE(reg, mode)      ((reg) & (mode))
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++[*/
#define GET_CHANGEMODEBIT(reg)     ((reg & 0x08) >> 3)
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++]*/
/*FIH-MTD-PERIPHERAL-CH-ESD-04++[*/
#define GET_ESDGSENSORBIT(reg)     (reg & 0x01)
/*FIH-MTD-PERIPHERAL-CH-ESD-04++]*/

/* maximum number of concurrent ST track IDs */
#define CY_NUM_ST_TCH_ID            2
/* maximum number of concurrent MT track IDs */
#define CY_NUM_MT_TCH_ID            4 /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16

#define CY_NTCH                     0 /* lift off */
#define CY_TCH                      1 /* touch down */
#define CY_ST_FNGR1_IDX             0
#define CY_ST_FNGR2_IDX             1
#define CY_MT_TCH1_IDX              0
#define CY_MT_TCH2_IDX              1
#define CY_MT_TCH3_IDX              2
#define CY_MT_TCH4_IDX              3
#define CY_XPOS                     0
#define CY_YPOS                     1
#define CY_IGNR_TCH               (-1)
#define CY_SMALL_TOOL_WIDTH         10
#define CY_LARGE_TOOL_WIDTH         255
#define CY_REG_BASE                 0x00
#define CY_REG_GEST_SET             0x1E
#define CY_REG_SCN_TYP              0x1C
#define CY_REG_ACT_INTRVL           0x1D
#define CY_REG_TCH_TMOUT            (CY_REG_ACT_INTRVL+1)
#define CY_REG_LP_INTRVL            (CY_REG_TCH_TMOUT+1)
#define CY_SOFT_RESET               (1 << 0)
#define CY_DEEP_SLEEP               (1 << 1)
#define CY_LOW_POWER                (1 << 2)
#define CY_MAXZ                     255
#define CY_OK                       0
#define CY_INIT                     1
#define CY_DELAY_DFLT               10 /* ms */
#define CY_DELAY_SYSINFO            20 /* ms */
#define CY_DELAY_BL                 300
#define CY_DELAY_DNLOAD             100 /* ms */
#define CY_HNDSHK_BIT               0x80
/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04
#define CY_NUM_KEY                  8

/* TrueTouch Standard Product Gen3 (Txx3xx) interface definition */
struct cyttsp_xydata {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	u16 x1 __attribute__ ((packed));
	u16 y1 __attribute__ ((packed));
	u8 z1;
	u8 touch12_id;
	u16 x2 __attribute__ ((packed));
	u16 y2 __attribute__ ((packed));
	u8 z2;
	u8 gest_cnt;
	u8 gest_id;
	u16 x3 __attribute__ ((packed));
	u16 y3 __attribute__ ((packed));
	u8 z3;
	u8 touch34_id;
	u16 x4 __attribute__ ((packed));
	u16 y4 __attribute__ ((packed));
	u8 z4;
#ifndef SECOND_MODULE
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	u8 ESD_byte;
	u8 tt_undef[2];
#else
	u8 tt_undef[3];
#endif
#else
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	u8 ESD_byte;
	u8 tt_undef;
	u8 tt_module;
#else
	u8 tt_undef[2];
	u8 tt_module;/*FIH-MTD-PERIPHERAL-CH-Add_command-00++*/
#endif
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/
	u8 gest_set;
	u8 tt_reserved;
};



/* TTSP System Information interface definition */
struct cyttsp_sysinfo_data {
	u8 hst_mode;
	u8 mfg_stat;
	u8 mfg_cmd;
	u8 cid[3];
	u8 tt_undef1;
	u8 uid[8];
	u8 bl_verh;
	u8 bl_verl;
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 tt_undef[5];
	u8 scn_typ;	/* Gen3 only: scan type [0:Mutual, 1:Self] */
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
};

/* TTSP Bootloader Register Map interface definition */
#define CY_BL_CHKSUM_OK 0x01
struct cyttsp_bootloader_data {
	u8 bl_file;
	u8 bl_status;
	u8 bl_error;
	u8 blver_hi;
	u8 blver_lo;
	u8 bld_blver_hi;
	u8 bld_blver_lo;
	u8 ttspver_hi;
	u8 ttspver_lo;
	u8 appid_hi;
	u8 appid_lo;
	u8 appver_hi;
	u8 appver_lo;
	u8 cid_0;
	u8 cid_1;
	u8 cid_2;
};

#define cyttsp_wake_data cyttsp_xydata

struct cyttsp {
	struct device *pdev;
	int irq;
	struct input_dev *input;
	struct input_dev *input_key; /* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00 */
	struct work_struct work;
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
	struct delayed_work test_mode_work;
	u8 current_test_mode;
	struct mutex test_mode_mutex;
	//atomic_t current_test_mode;
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	
	struct timer_list timer;
	struct mutex mutex;
	struct early_suspend early_suspend;
	char phys[32];
	struct cyttsp_platform_data *platform_data;
	struct cyttsp_bootloader_data bl_data;
	struct cyttsp_sysinfo_data sysinfo_data;
	u16 act_trk[CY_NUM_TRK_ID];
	u16 prv_mt_tch[CY_NUM_MT_TCH_ID];
	u16 prv_mt_pos[CY_NUM_TRK_ID][2];
	struct cyttsp_bus_ops *bus_ops;
	unsigned fw_loader_mode:1;
	unsigned suspended:1;
	struct timer_list to_timer;
	bool to_timeout;
	bool bl_ready;
	u8 reg_id;
	u8 tch_flow[CY_NUM_TRK_ID];/*FIH-MTD-PERIPHERAL-CH-AREA_JUDGE-00++*/
	u8 pre_id[CY_NUM_TRK_ID];/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
	bool op_mode;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
};

struct cyttsp_track_data {
	u8 prv_tch; //previous touch number
	u8 cur_tch; //current touch number
	u16 tmp_trk[CY_NUM_MT_TCH_ID];
	u16 snd_trk[CY_NUM_MT_TCH_ID];
	u16 cur_trk[CY_NUM_TRK_ID]; //current track-ID status is down/up
	u16 cur_mt_tch[CY_NUM_MT_TCH_ID]; //current multi-touch track-ID of each finger
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u16 cur_mt_pos[CY_NUM_TRK_ID][2]; //current multi-touch position (X,Y) of ID
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u8 cur_mt_z[CY_NUM_TRK_ID];
	u8 tool_width;
};

static const u8 bl_cmd[] = {
	CY_BL_FILE0, CY_BL_CMD, CY_BL_EXIT,
	CY_BL_KEY0, CY_BL_KEY1, CY_BL_KEY2,
	CY_BL_KEY3, CY_BL_KEY4, CY_BL_KEY5,
	CY_BL_KEY6, CY_BL_KEY7
};

/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
struct cyttsp_testmode_data {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_state;
	u8 touch1_xh;
	u8 touch1_xl;
	u8 touch1_yh;
	u8 touch1_yl;
	u8 data[256];
};
static struct cyttsp_testmode_data test_mode_data;

#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
struct cyttsp g_ts;
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/
/* MM-VH-DISPLAY-JB04*[ */
#ifdef CONFIG_FIH_UI_KPI_METER
struct cyttsp *pts;
#endif
/* MM-VH-DISPLAY-JB04*] */

/*FIH-MTD-PERIPHERAL-CH-2016-00+[*/
static int cyttsp_wr_reg(struct cyttsp *ts, u8 reg_id, u8 reg_data);
static int cyttsp_rd_reg(struct cyttsp *ts, u8 reg_id, u8 *reg_data);
/*FIH-MTD-PERIPHERAL-CH-2016-00+]*/

#ifdef AUTO_CALIB
static int cyttsp_calibration(struct cyttsp *ts);
static int check_mfg_command_status(struct cyttsp *ts, unsigned int delay, unsigned int retry);
//calibration command set
static const u8 calib_cmd[] = {
0x10, 0x00, 0x20, 0x00, 0x00
};
#endif
/* owenhuang+] */
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
static bool ESD=false;
static bool ESD_USB=false;
static bool ESD_HS=false;
static u8 ESD_status=0;
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/
/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+{ */
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_TAP
#define TS_HIGH_RES_ver				7
#define TS_MAX_X					320
#define TS_MAX_Y					480
#define TS_MAX_X_high				1024
#define TS_MAX_Y_high				1024
#elif defined(CONFIG_FIH_MACH_TAMSUI_MES)
#define TS_HIGH_RES_ver				3
#define TS_MAX_X					320
#define TS_MAX_Y					480
#define TS_MAX_X_high				1024
#define TS_MAX_Y_high				1024
#else
#define TS_HIGH_RES_ver				2
#define TS_MAX_X					1024
#define TS_MAX_Y					920
#define TS_MAX_X_high				1024
#define TS_MAX_Y_high				1024
#endif
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/
#define TS_KEY1_X                   (TS_MAX_X/3)
#define TS_KEY2_X                   ((TS_MAX_X/3)*2)
#define TS_KEY3_X                   TS_MAX_X
#define TS_KEY_DOWN					1
#define TS_KEY_UP					0

enum ts_state {
    NO_TOUCH = 0,
    PRESS_TOUCH_AREA,
    PRESS_KEY1_AREA,
    PRESS_KEY2_AREA,
    PRESS_KEY3_AREA,
    PRESS_KEY4_AREA
};

/*FIH-MTD-PERIPHERAL-CH-APP_VER-00++[*/
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_TAP
static int TMA340_APP_ver=8;
#elif defined(CONFIG_FIH_MACH_TAMSUI_MES)
static int TMA340_APP_ver=6;
#else
static int TMA340_APP_ver=4;
#endif
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++]*/
module_param(
    TMA340_APP_ver, int, S_IRUGO
);
/*FIH-MTD-PERIPHERAL-CH-APP_VER-00++]*/

/*FIH-MTD-PERIPHERAL-CH-Add_command-01++[*/
static int TMA340_ROW=0;
module_param(
    TMA340_ROW, int, S_IRUGO
);
static int TMA340_COL=0;
module_param(
    TMA340_COL, int, S_IRUGO
);
/*FIH-MTD-PERIPHERAL-CH-Add_command-01++]*/

static int TMA340_check=0;
module_param(
    TMA340_check, int, S_IRUGO
);

/*FIH-MTD-PERIPHERAL-CH-Add_command-00++[*/
#ifdef SECOND_MODULE
static int Module_get_already=0;
/*FIH-MTD-PERIPHERAL-CH-Add_reset-00++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_JLO
static bool ELK_module=false;
#endif
/*FIH-MTD-PERIPHERAL-CH-Add_reset-00++]*/
#endif
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++]*/

int prv_area[CY_NUM_MT_TCH_ID]; 
/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+} */

#define LOCK(m) do { \
	printk(KERN_INFO "%s: lock\n", __func__); \
	mutex_lock(&(m)); \
} while (0);

#define UNLOCK(m) do { \
	printk(KERN_INFO "%s: unlock\n", __func__); \
	mutex_unlock(&(m)); \
} while (0);

#if DEBUG
static void print_data_block(const char *func, u8 command,
			u8 length, void *data)
{
	char buf[1024]=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	unsigned buf_len = sizeof(buf);
	char *p = buf;
	int i;
	int l;

	l = snprintf(p, buf_len, "cmd 0x%x: ", command);
	buf_len -= l;
	p += l;
	for (i = 0; i < length && buf_len; i++, p += l, buf_len -= l)
		l = snprintf(p, buf_len, "%02x ", *((char *)data + i));
	DBG_MSG("%s: %s\n", func, buf);
}
#endif

/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++[*/
#ifdef CYTTSP_SOFT_RESET
static irqreturn_t cyttsp_bl_ready_irq(int irq, void *handle);
static int cyttsp_soft_reset(struct cyttsp *ts, bool *status);
#endif
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++]*/
static int cyttsp_set_operational_mode(struct cyttsp *ts);
static int cyttsp_exit_bl_mode(struct cyttsp *ts);
static void cyttsp_init_tch(struct cyttsp *ts)
{
	/* init the touch structures */
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	memset(ts->prv_mt_pos, CY_NTCH, sizeof(ts->prv_mt_pos));
	memset(ts->prv_mt_tch, CY_IGNR_TCH, sizeof(ts->prv_mt_tch));
}

static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int rc=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	int tries=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/

	DBG_MSG("%s: Enter\n", __func__);

	if (!buf || !length) {
		DBG_ERR("%s: Error, buf:%s len:%u\n",
				__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	for (tries = 0, rc = -1; tries < CY_NUM_RETRY && (rc < 0); tries++) {
		rc = ts->bus_ops->read(ts->bus_ops, command, length, buf);
		if (rc)
			msleep(CY_DELAY_DFLT);
	}

	if (rc < 0)
		DBG_ERR("%s: error %d\n", __func__, rc);

#if DEBUG
	print_data_block(__func__, command, length, buf);
#else
	//do nothing
#endif

	return rc;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
	u8 length, void *buf)
{
	int rc=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	int tries=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	DBG_MSG("%s: Enter\n", __func__);

	if (!buf || !length) {
		printk(KERN_ERR "%s: Error, buf:%s len:%u\n",
				__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	for (tries = 0, rc = -1; tries < CY_NUM_RETRY && (rc < 0); tries++) {
		rc = ts->bus_ops->write(ts->bus_ops, command, length, buf);
		if (rc)
			msleep(CY_DELAY_DFLT);
	}

	if (rc < 0)
		DBG_ERR("%s: error %d\n", __func__, rc);
	
#if DEBUG
	print_data_block(__func__, command, length, buf);
#else
	//do nothing
#endif

	return rc;
}


/* Timer function used as dummy interrupt driver */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG_MSG("%s: TTSP timer event!\n", __func__);
	/* schedule motion signal handling */
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
	mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	return;
}


/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function and disables further IRQs until this IRQ is processed in worker.
 * *************************************************************************/
static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG_MSG("%s: Got IRQ = %d!\n", __func__, irq);
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
	if (!work_pending(&ts->work) && (ts->current_test_mode == EXIT_TEST_MODE))
		schedule_work(&ts->work);
#else
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	
	return IRQ_HANDLED;
}



static void handle_multi_touch(struct cyttsp_track_data *t, struct cyttsp *ts)
{
	unsigned int loc_pos=0xFF;/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
	int sent_num=0; /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
	u8 id=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	void (*mt_sync_func)(struct input_dev *) = ts->platform_data->mt_sync;

	if (!ts->platform_data->use_trk_id)
		goto no_track_id;

	/* terminate any previous touch where the track
	 * is missing from the current event */
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		DBG_MSG("prvX=%d ,id=%d, preID=%d\n",ts->prv_mt_pos[id][CY_XPOS],id,ts->pre_id[id]);
			if ((ts->act_trk[id] == CY_NTCH)/* || (t->cur_trk[id] != CY_NTCH)*/)
				continue;
			if(t->cur_trk[id] != CY_NTCH)
			{
				loc_pos=id;
				DBG_MSG("loc_pos=%d",loc_pos);
				continue;
			}
				/*t.pre_id[id]*/
			#ifdef pen_up /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					t->tool_width);
			input_report_abs(ts->input, ABS_MT_PRESSURE,
					CY_NTCH);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
			/*input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CY_NTCH);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, t->tool_width);*/
			/* input_report_abs(ts->input, ABS_MT_POSITION_X,
						id == ts->prv_mt_pos[0][CY_ID] : ts->prv_mt_pos[0][CY_XPOS],ts->prv_mt_pos[1][CY_XPOS] );*/
			
			input_report_abs(ts->input, ABS_MT_POSITION_X,
						ts->prv_mt_pos[ts->pre_id[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
						ts->prv_mt_pos[ts->pre_id[id]][CY_YPOS]);
			DBG_MSG("IDCHECK_UP:id=%d, ts->pre_id[id]=%d\n",id,ts->pre_id[id]);
			/*input_report_abs(ts->input, ABS_MT_POSITION_X,
						id == ts->prv_mt_pos[0][CY_ID] ? ts->prv_mt_pos[0][CY_XPOS]:ts->prv_mt_pos[1][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
						id == ts->prv_mt_pos[0][CY_ID] ? ts->prv_mt_pos[0][CY_YPOS]:ts->prv_mt_pos[1][CY_YPOS]);*/
			
			/*input_report_key(ts->input, BTN_TOUCH, CY_NTCH);*//*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
			
			if (mt_sync_func)
				mt_sync_func(ts->input);
			
			DBG_INFO("%s: T_ID:%3d Z:%3d   X:%3d Y:%3d\n", __func__,
			id,
			CY_NTCH,
			ts->prv_mt_pos[ts->pre_id[id]][CY_XPOS],
			ts->prv_mt_pos[ts->pre_id[id]][CY_YPOS]);
		
			ts->act_trk[id] = CY_NTCH;
			ts->prv_mt_pos[ts->pre_id[id]][CY_XPOS] = 0;
			ts->prv_mt_pos[ts->pre_id[id]][CY_YPOS] = 0;
			#endif /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
		}
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/

	/* set Multi-Touch current event signals */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (t->cur_mt_tch[id] >= CY_NUM_TRK_ID)
			continue;

		input_report_abs(ts->input, ABS_MT_TRACKING_ID,
						t->cur_mt_tch[id]);
		/*input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
						t->cur_mt_z[id]);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
						t->tool_width);*/
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					t->cur_mt_z[id]);/*FIH-MTD-PERIPHERAL-CH-SIZE-00++*/
		input_report_abs(ts->input, ABS_MT_PRESSURE,
					t->cur_mt_z[id]);/*FIH-MTD-PERIPHERAL-CH-SIZE-01++*/
		input_report_abs(ts->input, ABS_MT_POSITION_X,
						t->cur_mt_pos[id][CY_XPOS]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
						t->cur_mt_pos[id][CY_YPOS]);
		/* input_report_key(ts->input, BTN_TOUCH, CY_TCH);*//*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
		sent_num++;
		if (mt_sync_func)
			mt_sync_func(ts->input);

		/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
		if(ts->act_trk[t->cur_mt_tch[id]]==CY_NTCH)
		{
			DBG_INFO("%s: ID:%3d Z:%3d  X:%3d Y:%3d\n", __func__,
				t->cur_mt_tch[id],
				t->cur_mt_z[id],
				t->cur_mt_pos[id][CY_XPOS],
				t->cur_mt_pos[id][CY_YPOS]);
		}
		else
		{
			DBG_MSG("%s: ID:%3d Z:%3d  X:%3d Y:%3d\n", __func__,
				t->cur_mt_tch[id],
				t->cur_mt_z[id],
				t->cur_mt_pos[id][CY_XPOS],
				t->cur_mt_pos[id][CY_YPOS]);
		}
		DBG_MSG("IDCHECK_DOWN:id=%d,t->cur_mt_tch[id]=%d, ts->pre_id[t->cur_mt_tch[id]]=%d\n",id,t->cur_mt_tch[id],ts->pre_id[t->cur_mt_tch[id]]);
		/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/

		ts->act_trk[t->cur_mt_tch[id]] = CY_TCH;/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		ts->prv_mt_pos[id][CY_XPOS] = t->cur_mt_pos[id][CY_XPOS];
		ts->prv_mt_pos[id][CY_YPOS] = t->cur_mt_pos[id][CY_YPOS];
	}
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++[*/
	if(sent_num==0 && mt_sync_func)
	{
		DBG_MSG("LIFT ALL UP\n");
		mt_sync_func(ts->input);
	}
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++]*/		
	return;
no_track_id:

	DBG_MSG("%s: goto no_track_id\n", __func__);
}

/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
static u8 pre_touch_down_up = 0;
static void testmode_work_func(struct work_struct *work)
{
	struct cyttsp *ts = container_of((struct delayed_work *)work,
			struct cyttsp, test_mode_work);
	s32 retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	s32 data_size = 7;
	//int i = 0;
	struct cyttsp_platform_data *pdata = (struct cyttsp_platform_data *)ts->pdev->platform_data;
	s32 row_pins = 0; /*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-01++*/
	s32 col_pins = 0; /*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-01++*/
	s32 total_section = 0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	u8 cmd;
	
	LOCK(ts->test_mode_mutex);
	DBG_INFO("Enter Test Mode Work Queue\n");
	
	//check user set into test mode
	if (!CHECK_TEST_ENABLE(ts->current_test_mode))
	{
		DBG_ERR("Not in test mode!\n");
		goto TESTMODE_FAILED;
	}

	if (!pdata)
	{
		DBG_ERR("pdata is null pointer!\n");
		goto TESTMODE_FAILED;
	}
/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-01++[*/
	row_pins = (s32)pdata->row_pins_number;
	col_pins = (s32)pdata->col_pins_number;
	total_section = row_pins * col_pins;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-01++]*/	
	//read data block from chip IC
	memset(&test_mode_data, '\0', sizeof(test_mode_data));
	data_size = data_size + total_section;	
	retval = ttsp_read_block_data(ts, CY_REG_BASE, data_size, &test_mode_data);
	if (retval < 0)
	{
		DBG_ERR("read block data from test mode error!\n");
		goto TESTMODE_FAILED;
	}

	cmd = test_mode_data.hst_mode | 0x80; //toggle bit set
	if (ttsp_write_block_data(ts, CY_REG_BASE, sizeof(test_mode_data.hst_mode), &cmd) < 0)
	{
		DBG_ERR("Write hst_mode toggle bit failed!\n");
		goto TESTMODE_FAILED;
	}

	if (test_mode_data.hst_mode != HST_MODE_FOR_TEST_MODE_IDAC &&
		test_mode_data.hst_mode != HST_MODE_FRO_TEST_MODE_RAWS &&
		test_mode_data.hst_mode != HST_MODE_FOR_TEST_MODE_DIFF)/*FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++*/
	{
		DBG_ERR("Chip is not in test mode! Please check it!\n");
		goto TESTMODE_FAILED;
	}

	//check bootloader mode
	if (GET_BOOTLOADERMODE(test_mode_data.tt_mode))
	{
		DBG_ERR("Bootloader mode detected!\n");
		goto TESTMODE_FAILED;
	}

	//check buffer is valid or not
	if (IS_BAD_PKT(test_mode_data.tt_mode))
	{
		DBG_ERR("Buffer data is invild!\n");
		goto TESTMODE_ERROR;
	}

	//check large objec detect
	if (test_mode_data.tt_state & 0x30)
	{
		DBG_ERR("large object is detect!\n");
		goto TESTMODE_ERROR;
	}

	//check touch finger number
	if (GET_NUM_TOUCHES(test_mode_data.tt_state) > 1)
	{
		DBG_ERR("Too many finger is detected!(bigger than 1)\n");
		goto TESTMODE_ERROR;
	}
	/*FIH-SW3-PERIPHERAL-CH-TAP-03++[*/
	//report (X,Y) data, if need, TODO: Report Keys
	if (GET_NUM_TOUCHES(test_mode_data.tt_state) == 1)
	{
		int x = (((int)test_mode_data.touch1_xh)<<8 | ((int)test_mode_data.touch1_xl));
		int y = (((int)test_mode_data.touch1_yh)<<8 | ((int)test_mode_data.touch1_yl));

		//print message at first time
		/*if (pre_touch_down_up == 0)*//*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
			DBG_INFO("DOWN-> [%d, %d]\n", x, y);
		
		pre_touch_down_up = 1;
/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++[*/
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,10);/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		input_report_abs(ts->input, ABS_MT_PRESSURE,255);/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_mt_sync(ts->input);
/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++]*/
	}
	else if(GET_NUM_TOUCHES(test_mode_data.tt_state) == 0 && pre_touch_down_up == 1)
	{
		int x = (((int)test_mode_data.touch1_xh)<<8 | ((int)test_mode_data.touch1_xl));
		int y = (((int)test_mode_data.touch1_yh)<<8 | ((int)test_mode_data.touch1_yl));
		pre_touch_down_up = 0;
		
		DBG_INFO("UP-> [%d, %d]\n", x, y);
		input_mt_sync(ts->input);/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
	}
	/*FIH-SW3-PERIPHERAL-CH-TAP-03++]*/
	//show all the test mode data
	DBG_INFO("HST_MODE[0x%X]\n", test_mode_data.hst_mode);
	DBG_INFO("TT_MODE[0x%X]\n", test_mode_data.tt_mode);
	DBG_INFO("New Data Counter[%d]\n", GET_NEW_DATA_COUNTER(test_mode_data.tt_mode));
	DBG_INFO("TT_STAT[0x%X]\n", test_mode_data.tt_state);

#if 0
	DBG_INFO("==================[Data Block Head]=====================\n");
	for (i = 0; i < total_section; i ++)
	{
		DBG_INFO("-[0x%X]-", test_mode_data.data[i]);
		if ((i + 1) % col_pins == 0)
			DBG_INFO("\n");
	}
	DBG_INFO("==================[Data Block Tail]=====================\n");
#endif

	cmd = test_mode_data.hst_mode;
	if (ttsp_write_block_data(ts, CY_REG_BASE, sizeof(test_mode_data.hst_mode), &cmd) < 0)
	{
		DBG_ERR("Write back to test mode failed!\n");
	}
	
TESTMODE_ERROR:
	UNLOCK(ts->test_mode_mutex);
	schedule_delayed_work(&ts->test_mode_work,
			msecs_to_jiffies(TEST_MODE_DELAY_TIME));
	return;
	
TESTMODE_FAILED:
	//Go back operational mode
	/*FIH-MTD-PERIPHERAL-CH-2016-00+[*/
	if (cyttsp_wr_reg(ts, HST_MODE, (CY_OPERATE_MODE | CY_LOW_POWER_MODE)) < 0)
	{
		DBG_ERR("go back to operational mode failed\n");
	}
	/*FIH-MTD-PERIPHERAL-CH-2016-00+]*/
	ts->current_test_mode = EXIT_TEST_MODE;
	UNLOCK(ts->test_mode_mutex);
}
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	

/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
static void clear_all_up(struct cyttsp *ts)
{
	int id;
	void (*mt_sync_func)(struct input_dev *) = ts->platform_data->mt_sync;
	for (id = 0; id < CY_NUM_TRK_ID; id++) {
		DBG_MSG("prvX=%d ,id=%d, preID=%d\n",ts->prv_mt_pos[id][CY_XPOS],id,ts->pre_id[id]);
			if ((ts->act_trk[id] == CY_NTCH)/* || (t->cur_trk[id] != CY_NTCH)*/)
				continue;
				/*t.pre_id[id]*/
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
			/*input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CY_NTCH);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 10);*/
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,10);
			input_report_abs(ts->input, ABS_MT_PRESSURE,CY_NTCH);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
						ts->prv_mt_pos[ts->pre_id[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
						ts->prv_mt_pos[ts->pre_id[id]][CY_YPOS]);
			/*input_report_key(ts->input, BTN_TOUCH, CY_NTCH);*//*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
			if (mt_sync_func)
				mt_sync_func(ts->input);
			
			/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/
			DBG_INFO("%s: T_ID:%3d Z:%3d   X:%3d Y:%3d\n", __func__,
				id,
				CY_NTCH,
				ts->prv_mt_pos[ts->pre_id[id]][CY_XPOS],
				ts->prv_mt_pos[ts->pre_id[id]][CY_YPOS]);
			/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/
		
			ts->act_trk[id] = CY_NTCH;
			ts->prv_mt_pos[id][CY_XPOS] = 0;
			ts->prv_mt_pos[id][CY_YPOS] = 0;
			
	}
}
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/

static void cyttsp_xy_worker(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, work);
	struct cyttsp_xydata xy_data;
	u8 id=0, tilt=0, rev_x=0, rev_y=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	struct cyttsp_track_data trc;
	s32 retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/

	DBG_MSG("%s: Enter\n", __func__);
	
	memset(&trc,0,sizeof(trc));/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-01++*/

	/* get event data from CYTTSP device */
	retval = ttsp_read_block_data(ts, CY_REG_BASE,
				      sizeof(xy_data), &xy_data);

	if (retval < 0) {
		DBG_ERR("%s: Error, fail to read device on host interface bus\n", __func__);
		goto exit_xy_worker;
	}

	/* touch extension handling */
/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-{
  we don't handle this. */
#if 0
	//retval = ttsp_tch_ext(ts, &xy_data);

	if (retval < 0) {
		DBG_ERR("%s: Error, touch extension handling\n",
			__func__);
		goto exit_xy_worker;
	} else if (retval > 0) {
		DBG_MSG("%s: Touch extension handled\n",
			__func__);
		goto exit_xy_worker;
	}
#endif
/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-} */

	/* provide flow control handshake */
	if (ts->irq) {
		if (ts->platform_data->use_hndshk) {
			u8 cmd;
			cmd = xy_data.hst_mode & CY_HNDSHK_BIT ?
				xy_data.hst_mode & ~CY_HNDSHK_BIT :
				xy_data.hst_mode | CY_HNDSHK_BIT;
			retval = ttsp_write_block_data(ts, CY_REG_BASE,
						       sizeof(cmd), (u8 *)&cmd);
		}
	}
	trc.cur_tch = GET_NUM_TOUCHES(xy_data.tt_stat);

	//[Purpose]Check if current touch ic status is vaild or not.
	if (ts->platform_data->power_state == CY_IDLE_STATE)
		goto exit_xy_worker;
	else if (GET_BOOTLOADERMODE(xy_data.tt_mode) && !ts->fw_loader_mode) {
		/* TTSP device has reset back to bootloader mode */
		/* reset driver touch history */
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
		u8 cmd=0x1B;
		int retry=0;
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++[*/
#ifdef CYTTSP_SOFT_RESET
		bool timeout;
#endif
		DBG_INFO("%s: Bootloader detected; reset driver\n", __func__);
		/*cyttsp_init_tch(ts); CH CH*/ /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		free_irq(ts->irq, ts);
#ifdef CYTTSP_SOFT_RESET
		cyttsp_soft_reset(ts, &timeout);
#endif
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++]*/
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_operational_mode(ts);
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
		mdelay(100);
		do{
			retval=cyttsp_wr_reg(ts, cmd, ESD_status);
			DBG_MSG("%s: Write ESD_status=%x  times=%d\n", __func__,ESD_status,retry);
			retry++;}
		while(retval<0 && retry<3);
#endif
		goto exit_xy_worker;
	}else if((GET_HSTMODE(xy_data.hst_mode) != 0) && ts->op_mode == true ){
		u8 cmd=0;
		retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
		if (retval < 0) {
			DBG_ERR("%s: Failed writing block data, err:%d\n",
				__func__, retval);
		}
		DBG_INFO("%s: Set to operation mode for unknow situation.\n", __func__);
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/
		goto exit_xy_worker;
	} else if (IS_LARGE_AREA(xy_data.tt_stat) == 1) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG_INFO("%s: Large area detected\n", __func__);
	} else if (trc.cur_tch > CY_NUM_MT_TCH_ID) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG_INFO("%s: Num touch error detected\n", __func__);
	} else if (IS_BAD_PKT(xy_data.tt_mode)) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		DBG_INFO("%s: Invalid buffer detected\n", __func__);
		goto exit_xy_worker;/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
	}

	/* set tool size */
	trc.tool_width = CY_SMALL_TOOL_WIDTH;

/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-{
   TMA340 are generation 3, */
#if 0
	if (ts->platform_data->gen == CY_GEN2) {
		/* translate Gen2 interface data into comparable Gen3 data */
		trc.cur_tch = ttsp_convert_gen2(trc.cur_tch, &xy_data);
	}
#endif
/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-} */

	/* clear current active track ID array and count previous touches */
	for (id = 0, trc.prv_tch = CY_NTCH; id < CY_NUM_TRK_ID; id++) {
		trc.cur_trk[id] = CY_NTCH;
		trc.prv_tch += ts->act_trk[id];
	}

	/* send no events if there were no previous touches */
	/* and no new touches */
	if ((trc.prv_tch == CY_NTCH) && ((trc.cur_tch == CY_NTCH) ||
				(trc.cur_tch > CY_NUM_MT_TCH_ID)))
		{
			/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
			DBG_INFO("EXIT WORKER for no new touch prv=%d cur=%d \n",trc.prv_tch,trc.cur_tch);
			clear_all_up(ts);
			goto exit_xy_worker;
			/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/
		}

	DBG_MSG("%s: prev=%d  curr=%d\n", __func__, trc.prv_tch, trc.cur_tch);
/*FIH-MTD-PERIPHERAL-CH-ESD-01++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	DBG_MSG("%s: ESD_BYTE=%x\n", __func__, xy_data.ESD_byte);
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-01++]*/

	/* clear current multi-touch arrays */
	memset(trc.cur_mt_tch, CY_IGNR_TCH, sizeof(trc.cur_mt_tch));
	memset(trc.cur_mt_pos, CY_NTCH, sizeof(trc.cur_mt_pos));
	memset(trc.cur_mt_z, CY_NTCH, sizeof(trc.cur_mt_z));

#if DEBUG	
	if (trc.cur_tch) 
	{
		unsigned i;
		u8 *pdata = (u8 *)&xy_data;

		DBG_MSG("%s: TTSP data_pack: ", __func__);
		for (i = 0; i < sizeof(struct cyttsp_xydata); i++)
			DBG_MSG("[%d]=0x%x ", i, pdata[i]);
		DBG_MSG("\n");
	}
#endif

	/* Determine if display is tilted */
	tilt = !!FLIP_DATA(ts->platform_data->flags);
	/* Check for switch in origin */
	rev_x = !!REVERSE_X(ts->platform_data->flags);
	rev_y = !!REVERSE_Y(ts->platform_data->flags);
	DBG_MSG("%s: flag=%d \n", __func__, ts->platform_data->flags);

	/* process the touches */
	switch (trc.cur_tch) {
/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-{
  Currently, system just only support 2-fingers. */
#if 1 /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
	case 4:
		xy_data.x4 = be16_to_cpu(xy_data.x4);
		xy_data.y4 = be16_to_cpu(xy_data.y4);
		if (tilt)
			FLIP_XY(xy_data.x4, xy_data.y4);

		if (rev_x)
			xy_data.x4 = INVERT_X(xy_data.x4,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y4 = INVERT_X(xy_data.y4,
					ts->platform_data->maxy);

		id = GET_TOUCH4_ID(xy_data.touch34_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_XPOS] = xy_data.x4;
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_YPOS] = xy_data.y4;
			trc.cur_mt_z[CY_MT_TCH4_IDX] = xy_data.z4;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x4;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y4;
			trc.cur_mt_z[id] = xy_data.z4;
		}
		trc.cur_mt_tch[CY_MT_TCH4_IDX] = id;
		trc.cur_trk[id] = CY_TCH;

		DBG_MSG(KERN_INFO"%s: 4th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n\n",
				__func__, xy_data.x4, xy_data.y4, xy_data.z4,
				(xy_data.touch34_id & 0x0F));

		/* do not break */
	case 3:
		xy_data.x3 = be16_to_cpu(xy_data.x3);
		xy_data.y3 = be16_to_cpu(xy_data.y3);
		if (tilt)
			FLIP_XY(xy_data.x3, xy_data.y3);

		if (rev_x)
			xy_data.x3 = INVERT_X(xy_data.x3,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y3 = INVERT_X(xy_data.y3,
					ts->platform_data->maxy);

		id = GET_TOUCH3_ID(xy_data.touch34_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_XPOS] = xy_data.x3;
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_YPOS] = xy_data.y3;
			trc.cur_mt_z[CY_MT_TCH3_IDX] = xy_data.z3;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x3;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y3;
			trc.cur_mt_z[id] = xy_data.z3;
		}
		trc.cur_mt_tch[CY_MT_TCH3_IDX] = id;
		trc.cur_trk[id] = CY_TCH;

		DBG_MSG(KERN_INFO"%s: 3rd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			__func__, xy_data.x3, xy_data.y3, xy_data.z3,
			((xy_data.touch34_id >> 4) & 0x0F));

		/* do not break */
#endif
/* SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-} */

	case 2:
		xy_data.x2 = be16_to_cpu(xy_data.x2);
		xy_data.y2 = be16_to_cpu(xy_data.y2);
		if (tilt)
			FLIP_XY(xy_data.x2, xy_data.y2);

		if (rev_x)
			xy_data.x2 = INVERT_X(xy_data.x2,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y2 = INVERT_X(xy_data.y2,
					ts->platform_data->maxy);
		id = GET_TOUCH2_ID(xy_data.touch12_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_XPOS] = xy_data.x2;
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_YPOS] = xy_data.y2;
			trc.cur_mt_z[CY_MT_TCH2_IDX] = xy_data.z2;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x2;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y2;
			trc.cur_mt_z[id] = xy_data.z2;
		}
		trc.cur_mt_tch[CY_MT_TCH2_IDX] = id;
		ts->pre_id[id]=CY_MT_TCH2_IDX;/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		trc.cur_trk[id] = CY_TCH;

		DBG_MSG("IDCHECK2:id=%d ts->pre[id]=%d\n",id,ts->pre_id[id]);/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		DBG_MSG("%s: 2nd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.x2, xy_data.y2, xy_data.z2,
				(xy_data.touch12_id & 0x0F));

		/* do not break */
	case 1:
		//Aries ++ Execution this condition
		xy_data.x1 = be16_to_cpu(xy_data.x1);
		xy_data.y1 = be16_to_cpu(xy_data.y1);
		if (tilt)
		{
			
			FLIP_XY(xy_data.x1, xy_data.y1);
		}			

		if (rev_x)
			xy_data.x1 = INVERT_X(xy_data.x1,
					ts->platform_data->maxx);
		if (rev_y)
			xy_data.y1 = INVERT_X(xy_data.y1,
					ts->platform_data->maxy);

		id = GET_TOUCH1_ID(xy_data.touch12_id);
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_XPOS] = xy_data.x1;
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_YPOS] = xy_data.y1;
			trc.cur_mt_z[CY_MT_TCH1_IDX] = xy_data.z1;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x1;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y1;
			trc.cur_mt_z[id] = xy_data.z1;
		}
		trc.cur_mt_tch[CY_MT_TCH1_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		ts->pre_id[id]=CY_MT_TCH1_IDX;/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/

		DBG_MSG("IDCHECK1:id=%d ts->pre[id]=%d\n",id,ts->pre_id[id]);/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/
		DBG_MSG("%s: S1st XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
				__func__, xy_data.x1, xy_data.y1, xy_data.z1,
				((xy_data.touch12_id >> 4) & 0x0F));

		break;
	case 0:
	default:
		break;
	}
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++[*/
#ifdef SECOND_MODULE
	if(Module_get_already==0)
	{
		if(xy_data.tt_module!=0)
			DBG_INFO("%s: The touch module is %s\n",
				__func__,ts->platform_data->module_2);
		else
		{
/*FIH-MTD-PERIPHERAL-CH-Add_reset-00++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_JLO
			ELK_module=true;
#endif
			DBG_INFO("%s: The touch module is %s\n",
				__func__,ts->platform_data->module_1);
		}
/*FIH-MTD-PERIPHERAL-CH-Add_reset-00++]*/
		Module_get_already=1;
	}
#endif
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++]*/


	if (ts->platform_data->use_mt)
		handle_multi_touch(&trc, ts);

	/* handle gestures */
	if (ts->platform_data->use_gestures && xy_data.gest_id) {
		input_report_key(ts->input, BTN_3, CY_TCH);
		input_report_abs(ts->input, ABS_HAT1X, xy_data.gest_id);
		input_report_abs(ts->input, ABS_HAT1Y, xy_data.gest_cnt);
	}

	/* signal the view motion event */
	input_sync(ts->input);

	/* update platform data for the current multi-touch information */
	memcpy(ts->act_trk, trc.cur_trk, CY_NUM_TRK_ID);

exit_xy_worker:
	DBG_MSG("%s: finished.\n", __func__);
	return;
}

/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static int cyttsp_check_polling(struct cyttsp *ts)
{
	return ts->platform_data->use_timer;
}

/* Timeout timer */
static void cyttsp_to_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG_MSG("%s: TTSP timeout timer event!\n", __func__);
	ts->to_timeout = true;
	return;
}

static void cyttsp_setup_to_timer(struct cyttsp *ts)
{
	DBG_MSG("%s: Enter\n", __func__);
	setup_timer(&ts->to_timer, cyttsp_to_timer, (unsigned long) ts);
}
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static void cyttsp_kill_to_timer(struct cyttsp *ts)
{
	DBG_MSG("%s: Enter\n", __func__);
	del_timer(&ts->to_timer);
}
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static void cyttsp_start_to_timer(struct cyttsp *ts, int ms)
{
	DBG_MSG("%s: Enter\n", __func__);
	ts->to_timeout = false;
	mod_timer(&ts->to_timer, jiffies + ms);
}

static bool cyttsp_timeout(struct cyttsp *ts)
{
	if (cyttsp_check_polling(ts))
		return false;
	else
		return ts->to_timeout;
}

static irqreturn_t cyttsp_bl_ready_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	DBG_INFO("%s: Got BL IRQ!\n", __func__);
	ts->bl_ready = true;
	return IRQ_HANDLED;
}

static void cyttsp_set_bl_ready(struct cyttsp *ts, bool set)
{
	DBG_INFO("%s: Enter\n", __func__);
	ts->bl_ready = set;
	printk(KERN_INFO"%s: bl_ready=%d\n", __func__, (int)ts->bl_ready);
}

static bool cyttsp_check_bl_ready(struct cyttsp *ts)
{
	if (cyttsp_check_polling(ts))
		return true;
	else
		return ts->bl_ready;
}
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/

static int cyttsp_load_bl_regs(struct cyttsp *ts)
{
	int retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/

	DBG_MSG("%s: Enter\n", __func__);

	retval =  ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(ts->bl_data), &(ts->bl_data));

	if (retval < 0) {
		DBG_ERR("%s: Failed reading block data, err:%d\n",
			__func__, retval);
		goto fail;
	}

	{
	      int i;
	      u8 *bl_data = (u8 *)&(ts->bl_data);
	      for (i = 0; i < sizeof(struct cyttsp_bootloader_data); i++)
			DBG_MSG("%s bl_data[%d]=0x%x\n",
				__func__, i, bl_data[i]);/*FIH-SW3-PERIPHERAL-CH-TAP-02++*/
	}
	DBG_MSG("%s: bl f=%02X s=%02X e=%02X\n",
		__func__,
		ts->bl_data.bl_file,
		ts->bl_data.bl_status,
		ts->bl_data.bl_error);

	return 0;
fail:
	return retval;
}

static bool cyttsp_bl_app_valid(struct cyttsp *ts)
{
	int retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/

	ts->bl_data.bl_status = 0x00;

	retval = cyttsp_load_bl_regs(ts);

	if (retval < 0)
		return false;

	if (ts->bl_data.bl_status == 0x11) {
		DBG_INFO("%s: App found; normal boot\n", __func__);
		return true;
	} else if (ts->bl_data.bl_status == 0x10) {
		DBG_INFO("%s: NO APP; load firmware!!\n", __func__);
		return false;
	} else {
		if (ts->bl_data.bl_file == CY_OPERATE_MODE) {
			int invalid_op_mode_status;
			invalid_op_mode_status = ts->bl_data.bl_status & 0x3f;
			if (invalid_op_mode_status)
				return false;
			else {
				if (ts->platform_data->power_state ==
					CY_ACTIVE_STATE)
					DBG_INFO("%s: Operational\n",
						__func__);
				else
					DBG_INFO("%s: No bootloader\n",
						__func__);
			}
		}
		return true;
	}
}
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static bool cyttsp_bl_status(struct cyttsp *ts)
{
	DBG_MSG("%s: Enter\n", __func__);
	return ((ts->bl_data.bl_status == 0x10) ||
		(ts->bl_data.bl_status == 0x11));
}
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static bool cyttsp_bl_err_status(struct cyttsp *ts)
{
	DBG_MSG("%s: Enter\n", __func__);
	return (((ts->bl_data.bl_status == 0x10) &&
		(ts->bl_data.bl_error == 0x20)) ||
		((ts->bl_data.bl_status == 0x11) &&
		(ts->bl_data.bl_error == 0x20)));
}

#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static bool cyttsp_wait_bl_ready(struct cyttsp *ts,
	int pre_delay, int loop_delay, int max_try,
	bool (*done)(struct cyttsp *ts))
{
	int tries=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	bool rdy = false, tmo = false;

	DBG_INFO("%s: Enter\n", __func__);
	DBG_INFO("%s: pre-dly=%d loop-dly=%d, max-try=%d\n",
		__func__, pre_delay, loop_delay, max_try);

	tries = 0;
	ts->bl_data.bl_file = 0;
	ts->bl_data.bl_status = 0;
	ts->bl_data.bl_error = 0;
	if (cyttsp_check_polling(ts)) {
		msleep(pre_delay);
		do {
			msleep(abs(loop_delay));
			cyttsp_load_bl_regs(ts);
		} while (!done(ts) &&
			tries++ < max_try);
		DBG_INFO("%s: polling mode tries=%d\n",
			__func__, tries);
	} else {
		cyttsp_start_to_timer(ts, abs(loop_delay) * max_try);
		while (!rdy && !tmo) {
			rdy = cyttsp_check_bl_ready(ts);
			tmo = cyttsp_timeout(ts);
			if (loop_delay < 0)
				udelay(abs(loop_delay));
			else
				msleep(abs(loop_delay));
			tries++;
		}
		DBG_INFO("%s: irq mode tries=%d rdy=%d tmo=%d\n",
			__func__, tries, (int)rdy, (int)tmo);
		cyttsp_load_bl_regs(ts);
	}

	if (tries >= max_try || tmo)
		return true;	/* timeout */
	else
		return false;
}
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/

static int cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	int tries = 0;

	DBG_MSG("%s: Enter\n", __func__);

	/* check if in bootloader mode;
	 * if in operational mode then just return without fail
	 */
	cyttsp_load_bl_regs(ts);
	if (!GET_BOOTLOADERMODE(ts->bl_data.bl_status))
		return 0;

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(bl_cmd),
				       (void *)bl_cmd);
	if (retval < 0) {
		DBG_ERR("%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto fail;
	}
	do {
		msleep(20);
		cyttsp_load_bl_regs(ts);
	} while (GET_BOOTLOADERMODE(ts->bl_data.bl_status) && tries++ < 10);

	DBG_INFO("%s: read tries=%d\n", __func__, tries);

	DBG_INFO("%s: Exit "
				"f=%02X s=%02X e=%02X\n",
				__func__,
				ts->bl_data.bl_file, ts->bl_data.bl_status,
				ts->bl_data.bl_error);

	return 0;
fail:
	return retval;
}

static int cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	int tries=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/
	u8 cmd = CY_SYSINFO_MODE | CY_LOW_POWER_MODE;
	u8 cmd_temp=0;
	/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/
	int temp_ver=0; /*FIH-MTD-PERIPHERAL-CH-APP_VER-00++*/

	DBG_MSG("%s: Enter\n", __func__);

	memset(&(ts->sysinfo_data), 0, sizeof(struct cyttsp_sysinfo_data));
	
	ts->op_mode=false;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/

	/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/
	retval = cyttsp_rd_reg(ts, CY_REG_BASE, &cmd_temp);
	if (retval < 0) {
		DBG_ERR("%s: read fail, err:%d\n",
			__func__, retval);
	}
	else cmd = cmd_temp | CY_SYSINFO_MODE;
	DBG_MSG("%s: read HST_MODE=%x\n", __func__, cmd_temp);
	/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/
	/* switch to sysinfo mode */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		DBG_ERR("%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto exit_sysinfo_mode;
	}

	/* read sysinfo registers */
	tries = 0;
	do {
		msleep(20);
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(ts->sysinfo_data), &(ts->sysinfo_data));
	} while (!(retval < 0) &&
		(ts->sysinfo_data.tts_verh == 0) &&
		(ts->sysinfo_data.tts_verl == 0) &&
		(tries++ < (500/20)));

/*FIH-MTD-PERIPHERAL-CH-APP_VER-00++[*/	
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++[*/
	if(ts->sysinfo_data.app_verl != 0)
	{
		temp_ver=ts->sysinfo_data.app_verh;
		TMA340_APP_ver=temp_ver<<8;/*FIH-MTD-PERIPHERAL-CH-Change_Mode-01++*/
		TMA340_APP_ver|=ts->sysinfo_data.app_verl;
	}
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++]*/
/*FIH-MTD-PERIPHERAL-CH-APP_VER-00++]*/
	DBG_INFO("%s: read tries=%d\n", __func__, tries);

	if (retval < 0) {
		DBG_ERR("%s: Failed reading block data, err:%d\n",
			__func__, retval);
		return retval;
	}

exit_sysinfo_mode:
	DBG_INFO("%s:SI2: hst_mode=0x%02X mfg_cmd=0x%02X "
		"mfg_stat=0x%02X\n", __func__, ts->sysinfo_data.hst_mode,
		ts->sysinfo_data.mfg_cmd,
		ts->sysinfo_data.mfg_stat);

	DBG_INFO("%s:SI2: bl_ver=0x%02X%02X\n",
		__func__, ts->sysinfo_data.bl_verh, ts->sysinfo_data.bl_verl);

	DBG_INFO("%s:SI2: sysinfo act_intrvl=0x%02X "
		"tch_tmout=0x%02X lp_intrvl=0x%02X\n",
		__func__, ts->sysinfo_data.act_intrvl,
		ts->sysinfo_data.tch_tmout,
		ts->sysinfo_data.lp_intrvl);

	DBG_INFO("%s:SI2:tts_ver=0x%02X%02X app_id=0x%02X%02X "
		"app_ver=0x%02X%02X c_id=0x%02X%02X%02X\n", __func__,
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);

	return retval;
}

static int cyttsp_set_sysinfo_regs(struct cyttsp *ts)
{
	int retval = 0;

//SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-{
//Please check TMA340 Spec, page 13. //CY_REG_SCN_TYP is useless.
#if 0
	if (ts->platform_data->scn_typ != CY_SCN_TYP_DFLT) {
		u8 scn_typ = ts->platform_data->scn_typ;

		retval = ttsp_write_block_data(ts,
				CY_REG_SCN_TYP,
				sizeof(scn_typ), &scn_typ);
	}

	if (retval < 0)
		return retval;
#endif
//SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-}

	if ((ts->platform_data->act_intrvl != CY_ACT_INTRVL_DFLT) ||
		(ts->platform_data->tch_tmout != CY_TCH_TMOUT_DFLT) ||
		(ts->platform_data->lp_intrvl != CY_LP_INTRVL_DFLT)) {

		u8 intrvl_ray[3];

		intrvl_ray[0] = ts->platform_data->act_intrvl;
		intrvl_ray[1] = ts->platform_data->tch_tmout;
		intrvl_ray[2] = ts->platform_data->lp_intrvl;

		/* set intrvl registers */
		retval = ttsp_write_block_data(ts,
				CY_REG_ACT_INTRVL,
				sizeof(intrvl_ray), intrvl_ray);

		msleep(CY_DELAY_SYSINFO);
	}

	return retval;
}

static int cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int retval;
	//int tries; //SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-
	u8 cmd = (CY_OPERATE_MODE | CY_LOW_POWER_MODE);/*FIH-MTD-PERIPHERAL-CH-2016-00*/
	u8 cmd_temp=0;/*FIH-MTD-PERIPHERAL-CH-2020-00*/
	//u8 gest_default; //SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-
	struct cyttsp_xydata xy_data;

	DBG_MSG(KERN_INFO"%s: Enter\n", __func__);

	memset(&(xy_data), 0, sizeof(xy_data));

/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/
	retval = cyttsp_rd_reg(ts, CY_REG_BASE, &cmd_temp);
	if (retval < 0) {
		DBG_ERR("%s: read fail, err:%d\n",
			__func__, retval);
	}
	else cmd = cmd_temp & 0x8f;
	DBG_MSG("%s: read HST_MODE=%x\n", __func__, cmd_temp);
/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		DBG_ERR("%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto write_block_data_fail;
	}
	ts->op_mode = true;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
	/* wait for TTSP Device to complete switch to Operational mode */
	msleep(20);

//SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-{
//We don't have gesture feature, so disable it.
#if 0
	tries = 0;
	gest_default =
		CY_GEST_GRP1 + CY_GEST_GRP2 +
		CY_GEST_GRP3 + CY_GEST_GRP4 +
		CY_ACT_DIST_DFLT;
	do {
		msleep(20);
		retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(xy_data), &(xy_data));
	} while (!(retval < 0) &&
		(xy_data.gest_set != gest_default) &&
		(tries++ < (500/20)));

	DBG_MSG("%s: read tries=%d\n", __func__, tries);
#endif
//SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-}

	/* Touch Timer or Interrupt setup */
	if (ts->platform_data->use_timer) {
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		DBG_INFO("%s: mod_timer\n", __func__);
	} else {
		DBG_INFO("%s: Setting up Interrupt. Device name=%s\n",
			__func__, ts->platform_data->name);
		retval = request_irq(ts->irq, cyttsp_irq,
			IRQF_TRIGGER_FALLING, ts->platform_data->name, ts);

		if (retval < 0) {
			DBG_ERR("%s: Error, could not request irq\n",
				__func__);
			goto write_block_data_fail;
		} else {
			DBG_INFO("%s: Interrupt=%d\n",
				__func__, ts->irq);
		}
	}
	return 0;
write_block_data_fail:
	return retval;
}
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
static int cyttsp_soft_reset(struct cyttsp *ts, bool *status)
{
	int retval=-1;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/
	u8 cmd = CY_SOFT_RESET_MODE | CY_LOW_POWER_MODE;
	u8 cmd_temp = 0;
/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/

	DBG_INFO("%s: Enter\n", __func__);
	ts->op_mode = false;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
	/* reset TTSP Device back to bootloader mode */
	if (!cyttsp_check_polling(ts)) {
		DBG_INFO("%s: Setting up BL Ready Interrupt. Device name=%s\n",
			__func__, ts->platform_data->name);
		retval = request_irq(ts->irq, cyttsp_bl_ready_irq,
			IRQF_TRIGGER_FALLING, ts->platform_data->name, ts);
		cyttsp_setup_to_timer(ts);
	}

/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/	
	retval = cyttsp_rd_reg(ts, CY_REG_BASE, &cmd_temp);
	if (retval < 0) {
		DBG_ERR("%s: read fail, err:%d\n",
			__func__, retval);
	}
	else cmd = cmd_temp | CY_SOFT_RESET_MODE;
	DBG_MSG("%s: read HST_MODE=%x\n", __func__, cmd_temp);
/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/
	
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	/* place this after write to reset;
	 * better to sacrifice one than fail
	 * on false early indication
	 */
	cyttsp_set_bl_ready(ts, false);
	/* wait for TTSP Device to complete reset back to bootloader */
	if (!retval)
		*status = cyttsp_wait_bl_ready(ts, 300, 10, 100,
			cyttsp_bl_status);

	if (!cyttsp_check_polling(ts)) {
		cyttsp_kill_to_timer(ts);
		free_irq(ts->irq, ts);
	}

	return retval;
}
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/

//SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-{
//We don't have guesture, just disable it.
#if 0
static int cyttsp_gesture_setup(struct cyttsp *ts)
{
	int retval;
	u8 gesture_setup;

	DBG_INFO("%s: Init gesture; active distance setup\n",
		__func__);

	gesture_setup = ts->platform_data->gest_set;
	retval = ttsp_write_block_data(ts, CY_REG_GEST_SET,
		sizeof(gesture_setup), &gesture_setup);

	return retval;
}
#endif
//SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00-}

#ifdef CY_INCLUDE_LOAD_RECS
#include "cyttsp_ldr.h"
#else
static int cyttsp_loader(struct cyttsp *ts)
{
	void *fptr = cyttsp_bl_err_status;	/* kill warning */

	if (ts) {
		DBG_MSG("%s: Enter\n", __func__);
		DBG_MSG("%s: Exit\n", __func__);
	}

	if (!fptr)
		return -EIO;
	else
		return 0;
}
#endif

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-03++*/
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
	bool timeout;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-03++*/
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
/*FIH-MTD-PERIPHERAL-CH-ESD-01++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_MES
	u8 cmd = 0x1B; /*reserved register used for this project ESD WORKAROUND*/
	u8 data = 0;
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-01++]*/
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++[*/
#ifdef CYTTSP_CM_bit
	int retry=0;
	u8  CM_BYTE=0;
#endif
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++]*/

	DBG_INFO("%s: Enter\n", __func__);

	ts->platform_data->power_state = CY_IDLE_STATE;

	/* check if the TTSP device has a bootloader installed */
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
	retval = cyttsp_soft_reset(ts, &timeout);
	DBG_INFO("%s: after softreset r=%d\n", __func__, retval);
	if (retval < 0 || timeout)
		goto bypass;
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/

	if (ts->platform_data->use_load_file)
		retval = cyttsp_loader(ts);

	if (!cyttsp_bl_app_valid(ts)) {
		retval = 1;
		goto bypass;
	}

	retval = cyttsp_exit_bl_mode(ts);
	DBG_MSG("%s: after exit bl r=%d\n", __func__, retval);

	if (retval < 0)
		goto bypass;

	/* switch to System Information mode to read */
	/* versions and set interval registers */
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++[*/
#ifdef CYTTSP_CM_bit
	retval=cyttsp_rd_reg(ts, CY_REG_BASE, &CM_BYTE);
	if(retval<0)
		goto bypass;
	msleep(28);
	CM_BYTE |= 0x08;
	retval=cyttsp_wr_reg(ts, CY_REG_BASE, CM_BYTE);
	if(retval<0)
		goto bypass;
	do{
	msleep(28);
	retval = cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto bypass;
	msleep(28);
	retval=cyttsp_rd_reg(ts, CY_REG_BASE, &CM_BYTE);
	DBG_INFO("%s:CM_BYTE=%x,retval=%d\r\n",__func__,CM_BYTE,retval);
	retry++;
	}while(retry < 5 && ((GET_CHANGEMODEBIT(CM_BYTE)==1)|| retval<0 ));
	DBG_INFO("%s:CM_BYTE=%x,retry=%d\r\n",__func__,CM_BYTE,retry);
	msleep(28);
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-01++[*/
#else
	retval = cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto bypass;
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-01++]*/
#endif
/*FIH-MTD-PERIPHERAL-CH-Change_Mode-00++]*/
	retval = cyttsp_set_sysinfo_regs(ts);
	if (retval < 0)
		goto bypass;

	/* switch back to Operational mode */
	DBG_INFO("%s: switch back to operational mode\n",
		__func__);
	retval = cyttsp_set_operational_mode(ts);
	if (retval < 0)
		goto bypass;

	/* init gesture setup; required for active distance */
	//We don't have guesture, just disable.
	//cyttsp_gesture_setup(ts); //SW1D3-Peripheral-OH-Cypress(TMA340)_TouchDriver_Porting-00- 

/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/	
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD	
	DBG_INFO(KERN_INFO"%s: NOTIFY ESD delay is 100..\n", __func__);
	msleep(100);
	retval = cyttsp_rd_reg(&g_ts, cmd, &data);
	if (retval < 0) {
		DBG_ERR("%s: Failed read block data, err:%d\n",
			__func__, retval);
		goto bypass;
	}
	DBG_INFO(KERN_INFO"%s: NOTIFY ESD at power_on %x...\n", __func__,data);
	if((data & 0x8)==0)
	{
		data|=0x8;
		cyttsp_wr_reg(&g_ts, cmd, data);
		DBG_INFO(KERN_INFO"%s: NOTIFY ESD at power on write data=%x...\n", __func__,data);
	}
	else
		DBG_INFO(KERN_INFO"%s: NOTIFY ESD at power on bit is already set...\n", __func__);
#else
/*FIH-MTD-PERIPHERAL-CH-ESD-01++[*/
#ifdef CONFIG_FIH_MACH_TAMSUI_MES
	msleep(100);
	data=0;
	cyttsp_wr_reg(ts, cmd, data);
	DBG_INFO(KERN_INFO"%s: DISANLE ESD in touch FW...\n", __func__);
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-01++]*/
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

bypass:
	if (!retval)
		ts->platform_data->power_state = CY_ACTIVE_STATE;

	DBG_INFO("%s: Power state is %s\n",
		__func__, (ts->platform_data->power_state == CY_ACTIVE_STATE) ?
		"ACTIVE" : "IDLE");
	
	return retval;
}

static int cyttsp_resume(struct cyttsp *ts)
{
	int retval = 0;
	struct cyttsp_xydata xydata;

	DBG_MSG("%s: Enter\n", __func__);
	input_mt_sync(ts->input);/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
	if (ts->platform_data->use_sleep && (ts->platform_data->power_state !=
		CY_ACTIVE_STATE)) {
/*FIH-MTD-PERIPHERAL-CH-Add_reset-00++[*/
#if defined(CONFIG_FIH_MACH_TAMSUI_JLO) && defined(SECOND_MODULE)
		if(!ELK_module)
		{
			printk(KERN_INFO"%s: SMAC MODULE...\n", __func__);
#endif
			if (ts->platform_data->wakeup) {
				retval = ts->platform_data->wakeup();
				if (retval < 0)
					DBG_ERR("%s: Error, wakeup failed!\n",
						__func__);
			} else {
				DBG_ERR("%s: Error, wakeup not implemented "
					"(check board file).\n", __func__);
				retval = -ENOSYS;
			}
#if defined(CONFIG_FIH_MACH_TAMSUI_JLO) && defined(reset_cmd)
		}
		else
		{
			printk(KERN_INFO"%s: ELK MODULE...\n", __func__);
			if (ts->platform_data->reset) {
				retval = ts->platform_data->reset();
				if (retval < 0)
					DBG_ERR("%s: Error, reset failed!\n",
						__func__);
			} else {
				DBG_ERR("%s: Error, reset not implemented "
					"(check board file).\n", __func__);
				retval = -ENOSYS;
			}
		}
#endif
/*FIH-MTD-PERIPHERAL-CH-Add_reset-00++]*/
		if (!(retval < 0)) {
			retval = ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(xydata), &xydata);
			if (!(retval < 0) && !GET_HSTMODE(xydata.hst_mode))
				ts->platform_data->power_state =
					CY_ACTIVE_STATE;
		}
	}
	DBG_INFO("%s: Wake Up %s\n", __func__,
		(retval < 0) ? "FAIL" : "PASS");
	
	return retval;
}

static int cyttsp_suspend(struct cyttsp *ts)
{
	u8 sleep_mode = 0;
	int retval = 0;
	u8 cmd_temp = 0;/*FIH-MTD-PERIPHERAL-CH-2020-00*/

	DBG_MSG("%s: Enter\n", __func__);
	ts->op_mode=true;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
	if (ts->platform_data->use_sleep &&
			(ts->platform_data->power_state == CY_ACTIVE_STATE)) {
		/*FIH-MTD-PERIPHERAL-CH-2020-00++[*/
		retval = cyttsp_rd_reg(ts, CY_REG_BASE, &cmd_temp);
		if (retval < 0) {
			DBG_ERR("%s: read fail, err:%d\n",
				__func__, retval);
			sleep_mode = CY_DEEP_SLEEP_MODE | CY_LOW_POWER_MODE;/*FIH-MTD-PERIPHERAL-CH-2016-00*/
		}
		else sleep_mode = (cmd_temp & 0x8f) | CY_DEEP_SLEEP_MODE;/*set operation mode for recover all status.*/
		DBG_MSG("%s: read HST_MODE=%x\n", __func__, cmd_temp);
		/*FIH-MTD-PERIPHERAL-CH-2020-00++]*/

		retval = ttsp_write_block_data(ts,
			CY_REG_BASE, sizeof(sleep_mode), &sleep_mode);
		if (!(retval < 0))
			ts->platform_data->power_state = CY_SLEEP_STATE;
	}
	DBG_INFO("%s: Sleep Power state is %s\n", __func__,
		(ts->platform_data->power_state == CY_ACTIVE_STATE) ?
		"ACTIVE" :
		((ts->platform_data->power_state == CY_SLEEP_STATE) ?
		"SLEEP" : "LOW POWER"));

	return retval;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_ts_early_suspend(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	DBG_INFO("%s: Enter\n", __func__);

	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		if (ts->platform_data->use_timer)
			del_timer(&ts->timer);
		else
			disable_irq_nosync(ts->irq);
		ts->suspended = 1;
		cancel_work_sync(&ts->work);
		cyttsp_suspend(ts);
	}
	
	UNLOCK(ts->mutex);
}

static void cyttsp_ts_late_resume(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	DBG_INFO("%s: Enter\n", __func__);

	LOCK(ts->mutex);
	if (!ts->fw_loader_mode && ts->suspended) {
		ts->suspended = 0;
		if (cyttsp_resume(ts) < 0)
			DBG_ERR("%s: Error, cyttsp_resume.\n",
				__func__);
		if (ts->platform_data->use_timer)
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		else
			enable_irq(ts->irq);
	}
	
	UNLOCK(ts->mutex);
}
#endif

static int cyttsp_wr_reg(struct cyttsp *ts, u8 reg_id, u8 reg_data)
{

	DBG_MSG("%s: Enter\n", __func__);

	return ttsp_write_block_data(ts,
		CY_REG_BASE + reg_id, sizeof(u8), &reg_data);
}

static int cyttsp_rd_reg(struct cyttsp *ts, u8 reg_id, u8 *reg_data)
{
	DBG_MSG("%s: Enter\n", __func__);
	return ttsp_read_block_data(ts,
		CY_REG_BASE + reg_id, sizeof(u8), reg_data);
}

static ssize_t firmware_write(struct file *bin_file, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t pos, size_t size)
{
	unsigned short val=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);
	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		val = *(unsigned short *)buf;
		ts->reg_id = (val & 0xff00) >> 8;
		if (!(ts->reg_id & 0x80)) {
			/* write user specified operational register */
			cyttsp_wr_reg(ts, ts->reg_id, (u8)(val & 0xff));
			DBG_INFO("%s: write(r=0x%02X d=0x%02X)\n",
				__func__, ts->reg_id, (val & 0xff));
		} else {
			/* save user specified operational read register */
			DBG_INFO("%s: read(r=0x%02X)\n",
				__func__, ts->reg_id);
		}
	} else {
		int retval = 0;
		int tries = 0;
		{
			char str[128];
			char *p = str;
			int i;
			for (i = 0; i < size; i++, p += 2)
				snprintf(p,sizeof("%02x"), "%02x", (unsigned char)buf[i]);/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-01++*/
			DBG_INFO("%s: size %d, pos %ld payload %s\n",
			       __func__, size, (long)pos, str);
		}
		do {
			retval = ttsp_write_block_data(ts,
				CY_REG_BASE, size, buf);
			if (retval < 0)
				msleep(500);
		} while ((retval < 0) && (tries++ < 10));
	}
	UNLOCK(ts->mutex);
	return size;
}

static ssize_t firmware_read(struct file *bin_file, struct kobject *kobj,
	struct bin_attribute *ba,
	char *buf, loff_t pos, size_t size)
{
	int count = 0;
	u8 reg_data=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);

	DBG_INFO("%s: Enter (mode=%d)\n",
		__func__, ts->fw_loader_mode);

	LOCK(ts->mutex);
	if (!ts->fw_loader_mode) {
		/* read user specified operational register */
		cyttsp_rd_reg(ts, ts->reg_id & ~0x80, &reg_data);
		*(unsigned short *)buf = reg_data << 8;
		count = sizeof(unsigned short);
		DBG_INFO("%s: read(d=0x%02X)\n",
			__func__, reg_data);
	} else {
		int retval = 0;
		int tries = 0;

		do {
			retval = cyttsp_load_bl_regs(ts);
			if (retval < 0)
				msleep(500);
		} while ((retval < 0) && (tries++ < 10));

		if (retval < 0) {
			DBG_ERR("%s: error reading status\n", __func__);
			count = 0;
		} else {
			*(unsigned short *)buf = ts->bl_data.bl_status << 8 |
				ts->bl_data.bl_error;
			count = sizeof(unsigned short);
		}

		DBG_INFO("%s:bl_f=0x%02X bl_s=0x%02X bl_e=0x%02X\n",
			__func__,
			ts->bl_data.bl_file,
			ts->bl_data.bl_status,
			ts->bl_data.bl_error);
	}
	UNLOCK(ts->mutex);
	return count;
}

static struct bin_attribute cyttsp_firmware = {
	.attr = {
		.name = "firmware",
		.mode = 0666, /* FIH-SW3-PERIPHERAL-AH-Authorize_FWUpgrade-00 */
	},
	.size = 128,
	.read = firmware_read,
	.write = firmware_write,
};

//owenhuang+[
#ifdef AUTO_CALIB
static int check_mfg_command_status(struct cyttsp *ts, unsigned int delay, unsigned int retry)
{
	int ret = 0;
	u8 buf[5] = {0};
	int retry_time = retry;
	
	while ((!TEST_REG_BIT(buf[0], 1) || !TEST_REG_BIT(buf[0], 7))
			&& retry_time >= 0)
	{
		ret = cyttsp_wr_reg(ts, HST_MODE, 0x90); //invert data toggle mode
		msleep(5);
		ret = cyttsp_rd_reg(ts, MFG_STAT, buf);
		DBG_INFO("Read MFG_STAT(0x%X)\n", buf[0]);
		TEST_REG_BIT(buf[0], 0) ? DBG_INFO("Test action in progress!\n") : DBG_ERR("Test action is done!!!\n");
		retry_time --;
		msleep(delay);
	} 

	if (!TEST_REG_BIT(buf[0], 1) || !TEST_REG_BIT(buf[0], 7))
		ret = -EIO;

	return ret;
}

static int cyttsp_calibration(struct cyttsp *ts)
{
	int ret = 0;
	u8 buf[5] = {0};

	//Enter system information mode
	ret = cyttsp_set_sysinfo_mode(ts);
	if (ret < 0)
	{
		DBG_ERR("Enter system information mode failed\n");
		return ret;
	}

	ret = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(calib_cmd),
				       (void *)calib_cmd);
	if (ret < 0)
	{
		DBG_ERR("execute re-calibration failed\n");
		goto EXIT_CALIB;
	}

	ret= check_mfg_command_status(ts, 1000, 20);
	if (ret < 0)
	{
		DBG_ERR("MFG command executed failed!\n");
		goto EXIT_CALIB;
	}

EXIT_CALIB:

	//Go back operational mode
	ret = cyttsp_wr_reg(ts, HST_MODE, (CY_OPERATE_MODE|CY_LOW_POWER_MODE));/*FIH-MTD-PERIPHERAL-CH-2016-00*/
	if (ret < 0)
	{
		DBG_ERR("go back to operational mode failed\n");
	}

	//wait for chip to switch mode from system information mode to operational mode
	msleep(40);

	//Show current mode
 	cyttsp_rd_reg(ts, HST_MODE, buf);
	DBG_INFO("Current TP mode: 0x%X\n", buf[0]);
 	
	return ret;
}
#endif
//owenhuang+]

static ssize_t attr_fwloader_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	return snprintf(buf,sizeof("0x%02X%02X 0x%02X%02X 0x%02X%02X 0x%02X%02X%02X\n"), "0x%02X%02X 0x%02X%02X 0x%02X%02X 0x%02X%02X%02X\n",
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
}

static ssize_t attr_fwloader_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	char *p=NULL;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	int ret=0;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
#ifdef CYTTSP_SOFT_RESET /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
	bool timeout=false;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
#endif /*FIH-MTD-PERIPHERAL-CH-soft_reset-00++*/
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned val = simple_strtoul(buf, &p, 10);
	u8 bl_status_cmd=0x1;
	u8 bl_status_data=0;

	ret = p - buf;
	if (*p && isspace(*p))
		ret++;
	
	DBG_MSG("%s: %u\n", __func__, val);

	LOCK(ts->mutex)
	if (val == 3) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
		DBG_INFO("%s: FW loader closed for reg r/w\n",
			__func__);
	} else if (val == 2) {
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			DBG_ERR("%s: unable to create file\n",
				__func__);
		DBG_INFO("%s: FW loader opened for reg r/w\n",
			__func__);
	} else if ((val == 1) && !ts->fw_loader_mode) {
		TMA340_check=0;
		ts->fw_loader_mode = 1;
		if (ts->suspended) {
			cyttsp_resume(ts);
		} else {
			if (ts->platform_data->use_timer)
				del_timer(&ts->timer);
			else
				disable_irq_nosync(ts->irq);
			cancel_work_sync(&ts->work);
		}
		ts->suspended = 0;
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			printk(KERN_ERR "%s: unable to create file\n",
				__func__);
		if (ts->platform_data->power_state == CY_ACTIVE_STATE)
			free_irq(ts->irq, ts);
		DBG_ERR("%s: FW loader opened for start load: ps=%d mode=%d\n",
			__func__,
			ts->platform_data->power_state, ts->fw_loader_mode);
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++[*/
#ifdef CYTTSP_SOFT_RESET
		cyttsp_soft_reset(ts, &timeout);
#else
		if(!ts->platform_data->reset())
		{
			DBG_INFO(KERN_INFO"%s: Reset...\n", __func__);
		}
#endif
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++]*/
		DBG_INFO("%s: FW loader started.\n", __func__);
	}
//owenhuang+[
#ifdef AUTO_CALIB
	else if (val == 21)
	{
		//check system is in suspend mode
		if (ts->suspended)
			cyttsp_resume(ts);

		//======Start to calibration=====//
		if (cyttsp_calibration(ts) < 0)
		{
			DBG_ERR("calibration error\n");
			goto FINISH_CALIB;
		}

		if (ts->suspended)
			cyttsp_suspend(ts);
	}
#endif
//owenhuang+]
	else if (!val && ts->fw_loader_mode) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++[*/
#ifdef CYTTSP_SOFT_RESET
		cyttsp_soft_reset(ts, &timeout);
#else
		if(!ts->platform_data->reset())
		{
			DBG_INFO(KERN_INFO"%s: Reset...\n", __func__);
		}
#endif
/*FIH-MTD-PERIPHERAL-CH-soft_reset-00++]*/
		cyttsp_rd_reg(ts, bl_status_cmd, &bl_status_data);
		TMA340_check|= (bl_status_data&0x01);
		DBG_INFO(KERN_INFO"%s: Checksum=%d...\n", __func__,TMA340_check);
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_sysinfo_mode(ts);	/* update sysinfo rev data */
		cyttsp_set_operational_mode(ts);
		ts->platform_data->power_state = CY_ACTIVE_STATE;
		ts->fw_loader_mode = 0;
		DBG_INFO("%s: FW loader finished.\n", __func__);
		cyttsp_bl_app_valid(ts);
	}
//owenhuang+[
#ifdef AUTO_CALIB
FINISH_CALIB:
#endif
//owenhuang+]
	UNLOCK(ts->mutex);
	return  ret == size ? ret : -EINVAL;
}

static struct device_attribute fwloader =
	__ATTR(fwloader, 0644, attr_fwloader_show, attr_fwloader_store);

/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
static ssize_t attr_testmode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	u32 count = 0;
	struct cyttsp_platform_data *pdata = (struct cyttsp_platform_data *)ts->pdev->platform_data;
	s32 row_pins = (s32)pdata->row_pins_number;
	s32 col_pins = (s32)pdata->col_pins_number;
	s32 total_section = row_pins * col_pins;
	int i = 0;
	int j = 0;

	LOCK(ts->test_mode_mutex);

	//not enable test mode return 0
	if (!CHECK_TEST_ENABLE(ts->current_test_mode))
	{
		DBG_ERR("Test Mode is not enable\n");
		count += snprintf(buf,sizeof("Test Mode is not enable!\n"), "Test Mode is not enable!\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
		goto READ_FAILED;
	}
	else
	{
		DBG_INFO("Current Mode (0x%X)\n", ts->current_test_mode);
	}
	
	if (ts->current_test_mode == TMODE_IDAC_SETTING)
	{
		count += snprintf(&buf[i],sizeof("[Mode]IDAC MODE(110b)\n"), "[Mode]IDAC MODE(110b)\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
		i += count;
		
		if (GET_NEW_DATA_COUNTER(test_mode_data.tt_mode) % 2 == 0) //even
		{
			count = snprintf(&buf[i],sizeof("[Type]Local IDac\n"), "[Type]Local IDac\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
			i += count;
			goto SHOW_MODE1;
		}
		else //odd
		{
			count = snprintf(&buf[i],sizeof("[Type]Global IDac\n"), "[Type]Global IDac\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
			i += count;
			goto SHOW_MODE2;
		}
	}
	else if (ts->current_test_mode == TMODE_IDAC_RAW_BASELINE)
	{
		count = snprintf(&buf[i],sizeof("[Mode]RAWS DATA/Baseline(111b)\n"), "[Mode]RAWS DATA/Baseline(111b)\n");
		i += count;
		
		if (GET_NEW_DATA_COUNTER(test_mode_data.tt_mode) % 2 == 0) //even
		{
			count = snprintf(&buf[i],sizeof("[Type]Baseline\n"), "[Type]Baseline\n");
			i += count;
			goto SHOW_MODE1;
		}
		else //odd
		{
			count = snprintf(&buf[i],sizeof("[Type]Raw data\n"), "[Type]Raw data\n");
			i += count;
			goto SHOW_MODE1;
		}
	}/*FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++[*/
	else if (ts->current_test_mode == TMODE_DIFF_COUNTS)
	{
		count = snprintf(&buf[i],sizeof("[Mode]Diff COUNTS(101b)\n"), "[Mode]Diff COUNTS(101b)\n");
		i += count;
		
		if (GET_NEW_DATA_COUNTER(test_mode_data.tt_mode) % 2 == 0) //even
		{
			count = snprintf(&buf[i],sizeof("[Type]Self-cap\n"), "[Type]Self-cap\n");
			i += count;
			goto SHOW_MODE1;
		}
		else //odd
		{
			count = snprintf(&buf[i],sizeof("[Type]Hibrid-cap\n"), "[Type]Hibrid-cap\n");
			i += count;
			goto SHOW_MODE1;
		}
	}
/*FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++]*/
SHOW_MODE2:
	total_section = test_mode_data.data[0];
	DBG_INFO("RX Slot = %d\n", total_section);
	col_pins = total_section / row_pins;

	count = snprintf(&buf[i],sizeof("=====[Global IDACs]=====\n"), "=====[Global IDACs]=====\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
	i += count;
	for (j = 0; j < total_section * 2; j++)
	{
		count = snprintf(&buf[i],sizeof("-[0x%02X]-"), "-[0x%02X]-", test_mode_data.data[j+1]);/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
		i += count;

		if ((j + 1) % col_pins == 0)
		{
			count = snprintf(&buf[i],sizeof("\n"), "\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
			i += count;
		}

		if ((j + 1) == total_section)
		{
			count = snprintf(&buf[i],sizeof("=====[Gains]=====\n"), "=====[Gains]=====\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
			i += count;
		}
	}
	
	goto PASS_SHOW_MODE1;
	
SHOW_MODE1:
	for (j = 0; j < total_section; j++)
	{
		count = snprintf(&buf[i],sizeof("-[0x%02X]-"), "-[0x%02X]-", test_mode_data.data[j]);/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
		i += count;

		if ((j + 1) % col_pins == 0)
		{
			count = snprintf(&buf[i],sizeof("\n"), "\n");/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-00++*/
			i += count;
		}
	}
	
PASS_SHOW_MODE1:
	count = i;

READ_FAILED:
	UNLOCK(ts->test_mode_mutex);	
	return count;
}

static ssize_t attr_testmode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	char *p=NULL;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
	int ret = 0;
	unsigned val = simple_strtoul(buf, &p, 10);

	LOCK(ts->test_mode_mutex);
	
	ret = p - buf;
	if (*p && isspace(*p))
		ret++;

	if (ts->suspended) 
	{
		/*FIH-SW3-PERIPHERAL-CH-TAP-03++[*/
		DBG_ERR("Set TestMode failed, it's in suspend!\n");
		/*cyttsp_resume(ts);
		ts->suspended = 0;*/
		goto suspendfail;
		/*FIH-SW3-PERIPHERAL-CH-TAP-03++]*/
	}

	if (val == TMODE_IDAC_SETTING)
	{
		u8 cmd = HST_MODE_FOR_TEST_MODE_IDAC;
		ts->op_mode=false;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
		ts->current_test_mode = TMODE_IDAC_SETTING;
		if (ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), (u8 *)&cmd) < 0)
		{
			DBG_ERR("Set TestMode(0x%X) failed!\n", cmd);
		}
		else
		{
			DBG_INFO("Start Test Mode: 0x%X\n", cmd);
			schedule_delayed_work(&ts->test_mode_work, msecs_to_jiffies(TEST_MODE_DELAY_TIME));
		}
	}
	else if (val == TMODE_IDAC_RAW_BASELINE)
	{
		u8 cmd = HST_MODE_FRO_TEST_MODE_RAWS;
		ts->op_mode=false;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
		ts->current_test_mode = TMODE_IDAC_RAW_BASELINE;
/*FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++[*/
		if (ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), (u8 *)&cmd) < 0)
		{
			DBG_ERR("Set TestMode(0x%X) failed!\n", cmd);
		}
		else
		{
			DBG_INFO("Start Test Mode: 0x%X\n", cmd);
			schedule_delayed_work(&ts->test_mode_work, msecs_to_jiffies(TEST_MODE_DELAY_TIME));
		}	
	}
	else if (val == TMODE_DIFF_COUNTS)
	{
		u8 cmd = HST_MODE_FOR_TEST_MODE_DIFF;
		ts->op_mode=false;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
		ts->current_test_mode = TMODE_DIFF_COUNTS;
/*FIH-MTD-PERIPHERAL-CH-DIFF_COUNT-00++]*/
		if (ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), (u8 *)&cmd) < 0)
		{
			DBG_ERR("Set TestMode(0x%X) failed!\n", cmd);
		}
		else
		{
			DBG_INFO("Start Test Mode: 0x%X\n", cmd);
			schedule_delayed_work(&ts->test_mode_work, msecs_to_jiffies(TEST_MODE_DELAY_TIME));
		}	
	}
	else
	{
		DBG_INFO("End Test Mode!\n");
		cancel_delayed_work_sync(&ts->test_mode_work);
		ts->op_mode=true;/*FIH-SW-PERIPHERAL-CH-Feature_Change-00++*/
		ts->current_test_mode = EXIT_TEST_MODE;
		//Go back operational mode
		/*FIH-MTD-PERIPHERAL-CH-2016-00+[*/
		if (cyttsp_wr_reg(ts, HST_MODE, (CY_OPERATE_MODE|CY_LOW_POWER_MODE)) < 0)
		{
			DBG_ERR("go back to operational mode failed\n");
		}
		/*FIH-MTD-PERIPHERAL-CH-2016-00+]*/
	}

suspendfail:/*FIH-SW3-PERIPHERAL-CH-TAP-03++*/
	UNLOCK(ts->test_mode_mutex);
	
	return  ret == size ? ret : -EINVAL;
}

static struct device_attribute testmode =
	__ATTR(testmode, 0644, attr_testmode_show, attr_testmode_store);

#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	

/*FIH-MTD-PERIPHERAL-CH-Add_command-00++[*/
#ifdef SECOND_MODULE
static ssize_t attr_module_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	u32 count = 0;
	int retval=0;
	u8 cmd = 0x1D; /*reserved register used for this project*/
	u8 data = 0;

	DBG_MSG(KERN_INFO"%s: Enter\n", __func__);
	retval = cyttsp_rd_reg(ts, cmd, &data);
	if (retval < 0) {
		DBG_ERR("%s: Failed resd block data, err:%d\n",
			__func__, retval);
		goto get_module_fail;
	}
	if(data)
		count += snprintf(buf,sizeof("MODULE is 1234567!\n"), "MODULE is %s!\n",ts->platform_data->module_2);
	else
		count += snprintf(buf,sizeof("MODULE is 1234567!\n"), "MODULE is %s!\n",ts->platform_data->module_1);
	/*data &= 0x0F;*/
	DBG_INFO(KERN_INFO"%s: TMA340 module ID ID= %d\n", __func__,data);
	return count;

get_module_fail:
	DBG_ERR("%s: Failed get module, err:%d\n",
			__func__, retval);
	return count;
}

static struct device_attribute module =
	__ATTR(module, 0644, attr_module_show, NULL);
#endif

#ifdef reset_cmd
static ssize_t attr_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int ret=0;
	char *p=NULL;
	unsigned val = simple_strtoul(buf, &p, 10);

	DBG_MSG(KERN_INFO"%s: Enter\n", __func__);
	
	ret = p - buf;
	if (*p && isspace(*p))
		ret++;
	
	if(val == 1)
	{
		if(!ts->platform_data->reset())
		{
			DBG_INFO(KERN_INFO"%s: Reset...\n", __func__);
		}	
	}
	else DBG_INFO(KERN_INFO"%s: Error reset parameter.\n", __func__);
	return  ret == size ? ret : -EINVAL;
}

struct device_attribute CYPRESS_RESET =
	__ATTR(reset, 0644, attr_module_show, attr_reset_store);
	
#endif
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++]*/

/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
static ssize_t attr_ESD_WORKAROUND(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	int ret=0;
	char *p=NULL;
	unsigned val = simple_strtoul(buf, &p, 10);
	int retval=0;
	u8 cmd = 0x1B; /*reserved register used for this project ESD WORKAROUND*/
	u8 data = 0;

	DBG_MSG(KERN_INFO"%s: Enter\n", __func__);
	
	ret = p - buf;
	if (*p && isspace(*p))
		ret++;
	
	if (ts->suspended){
		DBG_ERR("%s: Failed in suspend, err:%d\n",
				__func__, retval);
		goto ESD_suspend_fail;
	}
	DBG_INFO(KERN_INFO"%s: val=%d\n", __func__,val);
	mutex_lock(&ts->mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	if(val == CY_FACE_DOWN && ESD==false)
	{
		ESD=true;
		retval = cyttsp_rd_reg(ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESDa_fail;
		}
		DBG_MSG(KERN_INFO"%s: ENABLE ESD write data=%x...\n", __func__,data);
		if((data&0x1)==0)
		{
			data|=0x01;
			cyttsp_wr_reg(ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: ENABLE ESD bit is already set...\n", __func__);
	}
	else if(val == CY_FACE_UP && ESD==true)
	{
		ESD=false;
		retval = cyttsp_rd_reg(ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESDa_fail;
		}
		DBG_MSG(KERN_INFO"%s: DISABLE ESD write data=%x...\n", __func__,data);
		if((data & 0x1)!=0)
		{
			data&=0xFE;
			cyttsp_wr_reg(ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: DISABLE ESD bit is already set...\n", __func__);
	}
	else if(val == CY_PLUG_USB && ESD_USB==false)
	{
		ESD_USB=true;
		retval = cyttsp_rd_reg(ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESDa_fail;
		}
		DBG_MSG(KERN_INFO"%s: ENABLE ESD_USB write data=%x...\n", __func__,data);
		if((data & 0x2)==0)
		{
			data|=0x02;
			cyttsp_wr_reg(ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_USB write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: ENABLE ESD_USB bit is already set...\n", __func__);
	}
	else if(val == CY_UNPLUG_USB && ESD_USB==true)
	{
		ESD_USB=false;
		retval = cyttsp_rd_reg(ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESDa_fail;
		}
		DBG_MSG(KERN_INFO"%s: DISABLE ESD_USB write data=%x...\n", __func__,data);
		if((data & 0x2)!=0)
		{
			data&=0xFD;
			cyttsp_wr_reg(ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_USB write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: DISABLE ESD_USB bit is already set...\n", __func__);
	}
	else if(val == CY_PLUG_HS && ESD_HS==false)
	{
		ESD_HS=true;
		retval = cyttsp_rd_reg(ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESDa_fail;
		}
		DBG_MSG(KERN_INFO"%s: ENABLE ESD_HS write data=%x...\n", __func__,data);
		if((data & 0x4)==0)
		{
			data|=0x04;
			cyttsp_wr_reg(ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_HS write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: ENABLE ESD_HS bit is already set...\n", __func__);
	}
	else if(val == CY_UNPLUG_HS && ESD_HS==true)
	{
		ESD_HS=false;
		retval = cyttsp_rd_reg(ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESDa_fail;
		}
		DBG_MSG(KERN_INFO"%s: DISABLE ESD_HS write data=%x...\n", __func__,data);
		if((data & 0x4)!=0)
		{
			data&=0xFB;
			cyttsp_wr_reg(ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_HS write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: DISABLE ESD bit is already set...\n", __func__);
	}
	else DBG_MSG(KERN_INFO"%s: Error ESD write,status=%d.\n", __func__,val);
	mutex_unlock(&ts->mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	return size;
ESDa_fail:
	mutex_unlock(&ts->mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
ESD_suspend_fail:
	return  -EINVAL;
}

static ssize_t attr_ESD_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	u32 count = 0;
	int retval=0;
	u8 cmd = 0x1B; /*reserved register used for this project*/
	u8 data = 0;

	DBG_MSG(KERN_INFO"%s: Enter\n", __func__);
	if (ts->suspended)
		goto get_ESD_suspend;/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	
	mutex_lock(&ts->mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	retval = cyttsp_rd_reg(ts, cmd, &data);
	if (retval < 0) {
		DBG_ERR("%s: Failed resd block data, err:%d\n",
			__func__, retval);
		goto get_ESD_fail;
	}
	mutex_unlock(&ts->mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	if(data)
		count += snprintf(buf,sizeof("ESD status is 12!\n"), "ESD status is %x!\n",data);

	return count;

get_ESD_fail:
	mutex_unlock(&ts->mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
get_ESD_suspend:
	DBG_ERR("%s: Failed get ESD, err:%d\n",
			__func__, retval);
	return count;
}

struct device_attribute CYPRESS_ESD =
	__ATTR(ESD, 0644, attr_ESD_show, attr_ESD_WORKAROUND);


int TOUCH_ESD_WORKAROUND(int enable)
{
	int ret=0;
	int retval=0;
	u8 cmd = 0x1B; /*reserved register used for this project ESD WORKAROUND*/
	u8 data = 0;

	DBG_MSG(KERN_INFO"%s: Enter\n", __func__);
	if (g_ts.suspended)
		return -1;

	
	mutex_lock(&g_ts.mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	if(enable == CY_FACE_DOWN && ESD==false /*&& g_ts!=NULL*/)
	{
		retval = cyttsp_rd_reg(&g_ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESD_fail;
		}
		DBG_MSG(KERN_INFO"%s: ENABLE ESD write data=%x...\n", __func__,data);
		/*FIH-MTD-PERIPHERAL-CH-ESD-04++[*/
		if((data & 0x10) != 0)
		{
			if((data & 0x1) == 0)
			{
				int retry=0;
				do{
					data|=0x01;
					cyttsp_wr_reg(&g_ts, cmd, data);
					msleep(20);
					retval = cyttsp_rd_reg(&g_ts, cmd, &data);
					/*DBG_INFO(KERN_INFO"%s: ESD write data=%x ESD_status=%x...\n", __func__,data,ESD_status);*/
					DBG_INFO(KERN_INFO"%s: ESD read after write data=%x retry=%d...\n", __func__,data,retry);
					retry++;
				}while(((GET_ESDGSENSORBIT(data)==0)&&(retry < 3))||(retval<0));
				if(GET_ESDGSENSORBIT(data)==1)
				{
					ESD=true;
					ESD_status=data;
					DBG_INFO(KERN_INFO"%s: ESD write data=%x ...\n", __func__,data);
				}
			}
			else
				DBG_MSG(KERN_INFO"%s: ENABLE ESD bit is already set...\n", __func__);
		}
		else
			DBG_INFO(KERN_INFO"%s: The TOUCH IC is not ready to write enableESD(%x)...\n", __func__,data);
		/*FIH-MTD-PERIPHERAL-CH-ESD-03++]*/
	}
	else if(enable == CY_FACE_UP && ESD==true /*&& g_ts!=NULL*/)
	{
		retval = cyttsp_rd_reg(&g_ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESD_fail;
		}
		DBG_MSG(KERN_INFO"%s: DISABLE ESD write data=%x...\n", __func__,data);
		/*FIH-MTD-PERIPHERAL-CH-ESD-03++[*/
		if((data & 0x10) != 0)
		{
			if((data & 0x1) != 0)
			{
				int retry=0;
				do{
					data&=0xFE;
					cyttsp_wr_reg(&g_ts, cmd, data);
					msleep(20);
					retval = cyttsp_rd_reg(&g_ts, cmd, &data);
					DBG_INFO(KERN_INFO"%s: ESD read after write data=%x retry=%d...\n", __func__,data,retry);
					retry++;
				}while(((GET_ESDGSENSORBIT(data)==1)&&(retry < 3))||(retval<0));
				if(GET_ESDGSENSORBIT(data)==0)
				{
					ESD=false;
					ESD_status=data;
					DBG_INFO(KERN_INFO"%s: ESD write data=%x ESD_status=%x...\n", __func__,data,ESD_status);
				}
			}
			else
				DBG_MSG(KERN_INFO"%s: DISABLE ESD bit is already set...\n", __func__);
		}
		else
			DBG_INFO(KERN_INFO"%s: The TOUCH IC is not ready to write disableESD(%x)...\n", __func__,data);
		/*FIH-MTD-PERIPHERAL-CH-ESD-04++]*/
		ret=0;
	}
	else if(enable == CY_PLUG_USB && ESD_USB==false)
	{
		ESD_USB=true;
		retval = cyttsp_rd_reg(&g_ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESD_fail;
		}
		DBG_MSG(KERN_INFO"%s: ENABLE ESD_USB write data=%x...\n", __func__,data);
		if((data & 0x2)==0)
		{
			data|=0x02;
			cyttsp_wr_reg(&g_ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_USB write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: ENABLE ESD_USB bit is already set...\n", __func__);
	}
	else if(enable == CY_UNPLUG_USB && ESD_USB==true /*&& g_ts!=NULL*/)
	{
		ESD_USB=false;
		retval = cyttsp_rd_reg(&g_ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESD_fail;
		}
		DBG_MSG(KERN_INFO"%s: DISABLE ESD_USB write data=%x...\n", __func__,data);
		if((data&0x2)!=0)
		{
			data&=0xFD;
			cyttsp_wr_reg(&g_ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_USB write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: DISABLE ESD_USB bit is already set...\n", __func__);
		ret=0;
	}
	else if(enable == CY_PLUG_HS && ESD_HS==false)
	{
		ESD_HS=true;
		retval = cyttsp_rd_reg(&g_ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESD_fail;
		}
		DBG_MSG(KERN_INFO"%s: ENABLE ESD_HS write data=%x...\n", __func__,data);
		if((data & 0x4)==0)
		{
			data|=0x04;
			cyttsp_wr_reg(&g_ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_HS write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: ENABLE ESD_HS bit is already set...\n", __func__);
	}
	else if(enable == CY_UNPLUG_HS && ESD_HS==true /*&& g_ts!=NULL*/)
	{
		ESD_HS=false;
		retval = cyttsp_rd_reg(&g_ts, cmd, &data);
		if (retval < 0) {
			DBG_ERR("%s: Failed read block data, err:%d\n",
				__func__, retval);
			goto ESD_fail;
		}
		DBG_MSG(KERN_INFO"%s: DISABLE ESD_HS write data=%x...\n", __func__,data);
		if((data&0x4)!=0)
		{
			data&=0xFB;
			cyttsp_wr_reg(&g_ts, cmd, data);
			ESD_status=data;
			DBG_INFO(KERN_INFO"%s: ESD_HS write data=%x...\n", __func__,data);
		}
		else
			DBG_MSG(KERN_INFO"%s: DISABLE ESD bit is already set...\n", __func__);
		ret=0;
	}
	else DBG_MSG(KERN_INFO"%s: Error ESD write,status=%d.\n", __func__,enable);
ESD_fail:
	mutex_unlock(&g_ts.mutex);/*FIH-MTD-PERIPHERAL-CH-ESD-02++*/
	return ret;
}
EXPORT_SYMBOL(TOUCH_ESD_WORKAROUND);	
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
#ifdef FIH_VIRTUAL_BUTTON
static ssize_t tma340_virtual_keys_show(struct kobject *kobj,
                               struct kobj_attribute *attr, char *buf)
{
                
				if(TMA340_APP_ver < TS_HIGH_RES_ver)
				{
				/* center: x: back: 53, menu: 160, home: 267, y: 516 */
                return sprintf(buf,
                        __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":53:516:53:72"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":160:516:53:72"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":267:516:53:72"
                   "\n");
				}
				else
				{
				DBG_ERR("To DO:Not fine for 1024");
				/* center: x: back: 170, menu: 511, home: 852, y: 1102 */
					return sprintf(buf,
                        __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":170:1102:169:156"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":511:1102:169:156"
                   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":852:1102:169:156"
                   "\n");
				}
}
/*FIH-MTD-PERIPHERAL-CH-Key_area-00++]*/

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
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/

/* MM-VH-DISPLAY-JB04+[ */
#ifdef CONFIG_FIH_UI_KPI_METER
int dummy_touch(int id, int x, int y, int z, int delay_time_us)
{
	input_report_abs(pts->input, ABS_MT_TRACKING_ID,
					0);
	input_report_abs(pts->input, ABS_MT_TOUCH_MAJOR,
				z);
	input_report_abs(pts->input, ABS_MT_PRESSURE,
				z);
	input_report_abs(pts->input, ABS_MT_POSITION_X,
					x);
	input_report_abs(pts->input, ABS_MT_POSITION_Y,
					y);
	input_mt_sync(pts->input);
	input_sync(pts->input);
	
	if(delay_time_us != 0)
		msleep(delay_time_us);
		
	return 0;
}

static ssize_t dummy_slide_read(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}
enum{
	DUMMY_SLIDE_DISABLE,
	DUMMY_SLIDE_LEFT_TO_RIGHT,
	DUMMY_SLIDE_RIGHT_TO_LEFT,
	DUMMY_SLIDE_AUTO,
	NUM_DUMMY_SLIDE
};

#define DUMMY_SLIDER_INTERVAL_HZ (HZ/10)
static int autoDummySlider_interval =  DUMMY_SLIDER_INTERVAL_HZ;
struct delayed_work	autoDummySlider;
int dummySliderMode = 0;
static void dummy_slider(struct work_struct *work)
{
	/* LEFT TO RIGHT */
	dummy_touch(0, 1023, 630, 71, 40);
	dummy_touch(0, 1020, 629, 100, 0);
	dummy_touch(0, 1006, 629, 161, 20);
	dummy_touch(0, 976, 627, 220, 30);
	dummy_touch(0, 920, 621, 255, 20);
	dummy_touch(0, 876, 614, 255, 20);
	dummy_touch(0, 827, 605, 251, 20);
	dummy_touch(0, 758, 596, 226, 20);
	dummy_touch(0, 688, 591, 206, 20);
	dummy_touch(0, 615, 587, 188, 20);
	dummy_touch(0, 534, 590, 167, 20);
	dummy_touch(0, 422, 598, 130, 20);
	dummy_touch(0, 307, 614, 78, 20);

	/* RIGHT TO LEFT */
	/*
	dummy_touch(0, 1023, 593, 38, 40);
	dummy_touch(0, 1021, 594, 101, 0);
	dummy_touch(0, 980, 596, 251, 20);
	dummy_touch(0, 823, 601, 255, 30);
	dummy_touch(0, 602, 623, 255, 20);
	*/

	dummy_touch(0, 307, 614, 78, 20);
	dummy_touch(0, 422, 598, 130, 20);
	dummy_touch(0, 534, 590, 167, 20);
	dummy_touch(0, 615, 587, 188, 20);
	dummy_touch(0, 688, 591, 206, 20);
	dummy_touch(0, 758, 596, 226, 20);
	dummy_touch(0, 827, 605, 251, 20);
	dummy_touch(0, 876, 614, 255, 20);
	dummy_touch(0, 920, 621, 255, 20);
	dummy_touch(0, 976, 627, 220, 30);
	dummy_touch(0, 1006, 629, 161, 20);
	dummy_touch(0, 1020, 629, 100, 0);
	dummy_touch(0, 1023, 630, 71, 40);
	
	input_mt_sync(pts->input);
	input_sync(pts->input);

	schedule_delayed_work(&autoDummySlider, autoDummySlider_interval);
}
/* MM-VH-DISPLAY-JB07*[ */
static ssize_t dummy_slide_write(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;
	long data = 0;
	int error = strict_strtol(buf, 10, &data);

	if (error) {
		pr_err("[DISPLAY]%s: failure, buf <%s>, data <%ld>, err <%d>\n",
				__func__, buf, data, error);
	}

	cmd = (int)data;
	printk("+%s, cmd = %d\r\n", __func__, cmd);

	switch(cmd)
	{
		case DUMMY_SLIDE_DISABLE:
			cancel_delayed_work_sync(&autoDummySlider);
			break;
		case DUMMY_SLIDE_LEFT_TO_RIGHT:
			dummy_touch(0, 1023, 630, 71, 40);
			dummy_touch(0, 1020, 629, 100, 0);
			dummy_touch(0, 1006, 629, 161, 20);
			dummy_touch(0, 976, 627, 220, 30);
			dummy_touch(0, 920, 621, 255, 20);
			dummy_touch(0, 876, 614, 255, 20);
			dummy_touch(0, 827, 605, 251, 20);
			dummy_touch(0, 758, 596, 226, 20);
			dummy_touch(0, 688, 591, 206, 20);
			dummy_touch(0, 615, 587, 188, 20);
			dummy_touch(0, 534, 590, 167, 20);
			dummy_touch(0, 422, 598, 130, 20);
			dummy_touch(0, 307, 614, 78, 20);
			break;
		case DUMMY_SLIDE_RIGHT_TO_LEFT:
			dummy_touch(0, 1021, 593, 38, 40);
			dummy_touch(0, 1023, 594, 101, 0);
			dummy_touch(0, 980, 596, 251, 20);
			dummy_touch(0, 823, 601, 255, 30);
			dummy_touch(0, 602, 623, 255, 20);
			dummy_touch(0, 667, 292, 255, 20);
			break;
		case DUMMY_SLIDE_AUTO:
			schedule_delayed_work(&autoDummySlider, autoDummySlider_interval);
			break;

		default:
			break;
	};

	dummySliderMode = cmd;
	input_mt_sync(pts->input);
	input_sync(pts->input);
	
    return ret;
}
/* MM-VH-DISPLAY-JB07*] */
static DEVICE_ATTR(dummy_slide, 0644, dummy_slide_read, dummy_slide_write);
#endif
/* MM-VH-DISPLAY-JB04+] */

/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/	
void *cyttsp_core_init(struct cyttsp_bus_ops *bus_ops, struct device *pdev)
{
	struct input_dev *input_device;
    struct input_dev *input_key; /* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+ */
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	struct cyttsp *ts=NULL;/*FIH-MTD-PERIPHERAL-CH-CODE_DEFECT-02++*/
#endif
	int retval = 0;
	int cy_irq=0;/*FIH-MTD-PERIPHERAL-CH-MES-02++[*/
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
#ifdef FIH_VIRTUAL_BUTTON
	struct kobject *properties_kobj;
#endif
/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/

	DBG_INFO("Enter %s\n", __func__);

#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		DBG_ERR("%s: Error, kzalloc\n", __func__);
		goto error_alloc_data_failed;
	}
	mutex_init(&ts->mutex);
	ts->pdev = pdev;
	ts->platform_data = pdev->platform_data;
	ts->bus_ops = bus_ops;
	memset(ts->tch_flow,0,sizeof(ts->tch_flow));/*FIH-MTD-PERIPHERAL-CH-AREA_JUDGE-00++*/

	if (ts->platform_data->init)
		retval = ts->platform_data->init(1);
#else
	mutex_init(&g_ts.mutex);
	g_ts.pdev = pdev;
	g_ts.platform_data = pdev->platform_data;

	g_ts.bus_ops = bus_ops;
	memset(g_ts.tch_flow,0,sizeof(g_ts.tch_flow));/*FIH-MTD-PERIPHERAL-CH-AREA_JUDGE-00++*/

	if (g_ts.platform_data->init)
		retval = g_ts.platform_data->init(1);
#endif

/* MM-VH-DISPLAY-JB04+[ */
#ifdef CONFIG_FIH_UI_KPI_METER
	pts = ts;
	INIT_DELAYED_WORK(&autoDummySlider, dummy_slider);
#endif
/* MM-VH-DISPLAY-JB04+] */

	if (retval) {
		DBG_ERR("%s: platform init failed! \n", __func__);
		goto error_init;
	}

#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	if (ts->platform_data->use_timer)
		ts->irq = -1;
#else
	if (g_ts.platform_data->use_timer)
		g_ts.irq = -1;
#endif
	else
	{
		/*FIH-MTD-PERIPHERAL-CH-MES-02++[*/
		/*cyttsp_irq=ts->platform_data->irq_gpio;*/
		cy_irq=CY_I2C_IRQ_GPIO;
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
		ts->irq = MSM_GPIO_TO_INT(cy_irq);
		/*FIH-MTD-PERIPHERAL-CH-MES-02++]*/
		DBG_INFO("%s: ts->irq=%d \n", __func__, ts->irq);
#else
		g_ts.irq = MSM_GPIO_TO_INT(cy_irq);
		DBG_INFO("%s: ts->irq=%d \n", __func__, g_ts.irq);
#endif
	}		

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		retval = -ENOMEM;
		DBG_ERR("%s: Error, failed to allocate input device\n", __func__);
		goto error_input_allocate_device;
	}

	/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+{ */
    input_key = input_allocate_device();
    if (!input_key)
    {
		retval = -ENOMEM;
		DBG_ERR("%s: Error, failed to allocate input device\n", __func__);
		goto error_input_allocate_device;
    }	
	/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+} */

#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	ts->input = input_device;
	input_device->name = ts->platform_data->name;
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->pdev;
/*FIH-MTD-PERIPHERAL-CH-Add_command-01++[*/
	TMA340_ROW=ts->platform_data->row_pins_number;
	TMA340_COL=ts->platform_data->col_pins_number;
/*FIH-MTD-PERIPHERAL-CH-Add_command-01++]*/

	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work, cyttsp_xy_worker);
#else
	g_ts.input = input_device;
	input_device->name = g_ts.platform_data->name;
	input_device->phys = g_ts.phys;
	input_device->dev.parent = g_ts.pdev;
	TMA340_ROW=g_ts.platform_data->row_pins_number;
	TMA340_COL=g_ts.platform_data->col_pins_number;

	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&g_ts.work, cyttsp_xy_worker);
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	INIT_DELAYED_WORK(&ts->test_mode_work, testmode_work_func);
	ts->current_test_mode = EXIT_TEST_MODE;
	mutex_init(&ts->test_mode_mutex);
#else
	INIT_DELAYED_WORK(&g_ts.test_mode_work, testmode_work_func);
	g_ts.current_test_mode = EXIT_TEST_MODE;
	mutex_init(&g_ts.test_mode_mutex);
#endif
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */	

	//Currently, don't use timer
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD 
	if (ts->platform_data->use_timer) 
#else
	if (g_ts.platform_data->use_timer) 
#endif
	{
		DBG_INFO("%s: Setting up Timer\n", __func__);
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
#else
		setup_timer(&g_ts.timer, cyttsp_timer, (unsigned long) &g_ts);
#endif
	}
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	retval = cyttsp_power_on(ts);
#else
	retval = cyttsp_power_on(&g_ts);
#endif
	if (retval < 0) 
	{
		DBG_ERR("%s: Error, power on failed! \n", __func__);
		goto error_power_on;
	}
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	cyttsp_init_tch(ts);
#else
	cyttsp_init_tch(&g_ts);
#endif

	set_bit(EV_SYN, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_ABS, input_device->evbit);
	/*set_bit(BTN_TOUCH, input_device->keybit);*//*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-01++*/
	/*set_bit(BTN_2, input_device->keybit);*/ /*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++*/

#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD	
	if (ts->platform_data->use_gestures)
#else
	if (g_ts.platform_data->use_gestures)
#endif
		set_bit(BTN_3, input_device->keybit);

#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	if (ts->platform_data->use_mt) {
#else
	if (g_ts.platform_data->use_mt) {
#endif
		#if 0
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     ts->platform_data->maxy, 0, 0);
		#endif 
		/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+{ */
		DBG_INFO("%s: TS_KEY1~3_X: %d, %d, %d \n", __func__, TS_KEY1_X, TS_KEY2_X, TS_KEY3_X);
	/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++[*/
		if(TMA340_APP_ver < TS_HIGH_RES_ver)
		{
			input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     TS_MAX_X, 0, 0);
		
			input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     TS_MAX_Y, 0, 0);
		}
		else
		{
			input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     TS_MAX_X_high, 0, 0);
		
			input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     TS_MAX_Y_high, 0, 0);
		}
		/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+} */
		input_set_abs_params(input_device, ABS_MT_PRESSURE, 0,
				     255, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0,
				     CY_MAXZ, 0, 0);
		
		/*input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0,
				     CY_LARGE_TOOL_WIDTH, 0, 0);*/
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
		if (ts->platform_data->use_trk_id)
#else
		if (g_ts.platform_data->use_trk_id)
#endif
			input_set_abs_params(input_device, ABS_MT_TRACKING_ID,
					0, CY_NUM_TRK_ID, 0, 0);
	}
	
#ifdef FIH_VIRTUAL_BUTTON
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		retval = sysfs_create_group(properties_kobj,&tma340_properties_attr_group);

	if (!properties_kobj || retval)
		printk(KERN_ERR "%s: failed to create board_properties\n",__func__);
#endif
	/*FIH-MTD-PERIPHERAL-CH-TRACKING_ID-00++]*/


#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	if (ts->platform_data->use_virtual_keys)
#else
	if (g_ts.platform_data->use_virtual_keys)
#endif
		input_set_capability(input_device, EV_KEY, KEY_PROG1);

	retval = input_register_device(input_device);
	if (retval) {
		DBG_ERR("%s: Error, failed to register input device\n",
			__func__);
		goto error_input_register_device;
	}
	DBG_INFO("%s: Registered input device %s\n",
		   __func__, input_device->name);


	DBG_INFO("%s: Registered input device %s\n",
		   __func__, input_key->name);
	/* FIH-SW3-PERIPHERAL-AH-Touchkey_Porting-00+} */

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp_ts_early_suspend;
	ts->early_suspend.resume = cyttsp_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#else
	g_ts.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_ts.early_suspend.suspend = cyttsp_ts_early_suspend;
	g_ts.early_suspend.resume = cyttsp_ts_late_resume;
	register_early_suspend(&g_ts.early_suspend);
#endif
#endif
	retval = device_create_file(pdev, &fwloader);
	if (retval) {
		DBG_ERR("%s: Error, could not create attribute\n",
			__func__);
		goto device_create_error;
	}
/* MM-VH-DISPLAY-JB04+[ */
#ifdef CONFIG_FIH_UI_KPI_METER
	retval = device_create_file(pdev, &dev_attr_dummy_slide);
	if (retval) {
		pr_err("[DISPLAY] %s: create dev_attr_dummy_slide failed\n",
				__func__);
	}
#endif
/* MM-VH-DISPLAY-JB04+] */

/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+{ */	
#ifdef TEST_MODE
	retval = device_create_file(pdev, &testmode);
	if (retval)
	{
		DBG_ERR("%s: Error, could not create testmode attribute\n",
			__func__);
		goto device_create_error;
	}	
#endif
/* FIH-SW1-PERIPHERAL-OH-TAP_TOUCH_TestMode-00+} */
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++[*/	
#ifdef SECOND_MODULE
	retval = device_create_file(pdev, &module);
	if (retval)
	{
		DBG_ERR("%s: Error, could not create testmode attribute\n",
			__func__);
		goto device_create_error;
	}	
#endif

#ifdef reset_cmd
	retval = device_create_file(pdev, &CYPRESS_RESET);
	if (retval)
	{
		DBG_ERR("%s: Error, could not create testmode attribute\n",
			__func__);
		goto device_create_error;
	}	
#endif
/*FIH-MTD-PERIPHERAL-CH-Add_command-00++]*/

#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	DBG_INFO("%s: Register ESD file node.\n",
			__func__);
	/*retval = device_create_file(&(g_ts.input->dev), &CYPRESS_ESD);*/
	retval = device_create_file(pdev, &CYPRESS_ESD);
	if (retval)
	{
		DBG_ERR("%s: Error, could not create testmode attribute\n",
			__func__);
		goto device_create_error;
	}
#endif

#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	dev_set_drvdata(pdev, ts);
	return ts;
#else
	dev_set_drvdata(pdev, &g_ts);
	return &g_ts;
#endif

device_create_error:
#ifdef CONFIG_HAS_EARLYSUSPEND
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	unregister_early_suspend(&ts->early_suspend);
#else
	unregister_early_suspend(&g_ts.early_suspend);
#endif
#endif
error_input_register_device:
	input_unregister_device(input_device); 
error_power_on:
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	if (ts->irq >= 0)
		free_irq(ts->irq, ts);
	cyttsp_kill_to_timer(ts);
#else
	if (g_ts.irq >= 0)
		free_irq(g_ts.irq, &g_ts);
	cyttsp_kill_to_timer(&g_ts);
#endif
	input_free_device(input_device);
error_input_allocate_device:
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	if (ts->platform_data->init)
		ts->platform_data->init(0);
#else
	if (g_ts.platform_data->init)
		g_ts.platform_data->init(0);
#endif
error_init:
#ifndef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
	kfree(ts);
error_alloc_data_failed:
#endif
	return NULL;
}
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

/* registered in driver struct */
void cyttsp_core_release(void *handle)
{
	struct cyttsp *ts = handle;

	DBG_INFO("%s: Enter\n", __func__);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	cancel_work_sync(&ts->work);
	if (ts->platform_data->use_timer)
		del_timer_sync(&ts->timer);
	else
		free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	input_free_device(ts->input);
	if (ts->platform_data->init)
		ts->platform_data->init(0);
	kfree(ts);
}
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");

