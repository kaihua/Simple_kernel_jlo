/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"
#include <mach/vreg.h>
#include <linux/gpio.h>

#define AUO_PANEL_ID 0x42
#define CMI_PANEL_ID 0x43

static struct dsi_buf orise_tx_buf;
static struct dsi_buf orise_rx_buf;
static struct msm_panel_common_pdata *mipi_orise_pdata = NULL;
static int display_initialize = 0;
static char gPanelModel = 0;

static char BKL_PWM[11] = {
/*   LEVEL Value,   Duty,    Brightness */
	0x00,  /*  0,   0.0%,   0.00 cd/m^2 */
	0x1F,  /* 31,  12.5%,  50.00 cd/m^2 */
	0x34,  /* 52,  20.7%,  88.00 cd/m^2 */
	0x4C,  /* 76,  30.1%, 127.00 cd/m^2 */
	0x64,  /*100,  39.5%, 166.00 cd/m^2 */
	0x7B,  /*123,  48.4%, 205.00 cd/m^2 */
	0x94,  /*148,  58.2%, 242.00 cd/m^2 */
	0xAD,  /*173,  68.0%, 281.00 cd/m^2 */
	0xC7,  /*199,  78.1%, 319.00 cd/m^2 */
	0xE1,  /*225,  88.3%, 357.00 cd/m^2 */
	0xFF   /*255, 100.0%, 396.00 cd/m^2 */
};

/* orise fwvga panel */
/* ----------- [For AUO panel setting Start] ----------- */
static char extc[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char cmd2[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
/* FIH-SW2-MM-KW-AUO_Gamma-02+{ */
static char cmdD8[2] = {0xD8, 0x87}; /* DTYPE_GEN_LWRITE */
static char auo_gamma_cmdE1[17] = {0xE1, 0x09, 0x11, 0x16, 0x0D, 0x05,
								0x0E, 0x0A, 0x07, 0x07, 0x08, 0x0A,
								0x05, 0x0C, 0x0A, 0x09, 0x08};
static char auo_gamma_cmdE2[17] = {0xE2, 0x09, 0x11, 0x16, 0x0D, 0x05,
								0x0E, 0x0A, 0x07, 0x07, 0x08, 0x0A,
								0x05, 0x0C, 0x0A, 0x09, 0x08};
static char auo_gamma_cmdEC[34] = {0xEC, 0x40, 0x44, 0x43, 0x43, 0x43, 0x32,
								0x43, 0x33, 0x46, 0x22, 0x23, 0x32, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x34, 0x33, 0x54,
								0x44, 0x33, 0x34, 0x34, 0x44, 0x33, 0x34,
								0x43, 0x44, 0x44, 0x44, 0x43, 0x44};
static char auo_gamma_cmdED[34] = {0xED, 0x40, 0x44, 0x43, 0x43, 0x44, 0x43,
								0x33, 0x33, 0x54, 0x34, 0x32, 0x33, 0x44,
								0x43, 0x44, 0x44, 0x43, 0x24, 0x44, 0x54,
								0x34, 0x43, 0x44, 0x44, 0x44, 0x33, 0x34,
								0x44, 0x45, 0x44, 0x53, 0x63, 0x45};
static char auo_gamma_cmdEE[34] = {0xEE, 0x40, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
								0x44, 0x44, 0x44, 0x44, 0x44, 0x04};
/* FIH-SW2-MM-KW-AUO_Gamma-02-} */
/* ----------- [For AUO panel setting End] ----------- */

/* ----------- [For CMI panel setting Start] ----------- */
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-[+ */
static char cmi_extc[4] = {0xFF, 0x80, 0x09, 0x01}; /* DTYPE_GEN_LWRITE */
static char cmi_cmd2[3] = {0xFF, 0x80, 0x09}; /* DTYPE_GEN_LWRITE */
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-]- */
/* FIH-SW2-MM-KW-CMI_INIT_Gamma-00+{ */
/* Set GVDD/NGVDD */
static char gamma_cmd1[3] = {0xD8, 0xAF, 0xAF};
/* Gammm Correction Characteristics Setting (3.0+) */
static char gamma_cmdE7[17] = {0xE7, 0x00, 0x10, 0x17, 0x0F, 0x08,
							0x0B, 0x04, 0x02, 0x0B, 0x0D, 0x0F,
							0x03, 0x06, 0x08, 0x07, 0x06};
/* Gammm Correction Characteristics Setting (3.0-) */
static char gamma_cmdE8[17] = {0xE8, 0x00, 0x10, 0x17, 0x0C, 0x05,
							0x0D, 0x0A, 0x07, 0x08, 0x09, 0x0E,
							0x04, 0x0B, 0x04, 0x03, 0x01};
/* Gamma  Set */
static char gamma_set[2] = {0x26, 0x08};
/* FIH-SW2-MM-KW-CMI_INIT_Gamma-00-} */
/* ----------- [For CMI panel setting End] ----------- */
#if 0
static char ce_cmdD4[361] = {0xD4,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40,
	0xFE, 0x40, 0xFE, 0x40, 0xFE, 0x40, 0xFD, 0x40, 0xFD, 0x40,
	0xFD, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFB, 0x40, 0xFA, 0x40,
	0xFA, 0x40, 0xFA, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFC, 0x40,
	0xFD, 0x40, 0xFD, 0x40, 0xFD, 0x40, 0xFE, 0x40, 0xFE, 0x40,
	0xFE, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40,
	0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x03, 0x40, 0x03, 0x40,
	0x03, 0x40, 0x04, 0x40, 0x04, 0x40, 0x04, 0x40, 0x06, 0x40,
	0x06, 0x40, 0x05, 0x40, 0x04, 0x40, 0x04, 0x40, 0x04, 0x40,
	0x03, 0x40, 0x03, 0x40, 0x03, 0x40, 0x02, 0x40, 0x02, 0x40,
	0x02, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40,
	0xFE, 0x40, 0xFE, 0x40, 0xFE, 0x40, 0xFD, 0x40, 0xFD, 0x40,
	0xFD, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFB, 0x40, 0xFA, 0x40,
	0xFA, 0x40, 0xFA, 0x40, 0xFC, 0x40, 0xFC, 0x40, 0xFC, 0x40,
	0xFD, 0x40, 0xFD, 0x40, 0xFD, 0x40, 0xFE, 0x40, 0xFE, 0x40,
	0xFE, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0xFF, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,

	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40
};
static char ce_cmdD5[361] = {0xD5,
	0x00, 0x60, 0x00, 0x5E, 0x00, 0x5D, 0x00, 0x5B, 0x00, 0x59,
	0x00, 0x57, 0x00, 0x56, 0x00, 0x54, 0x00, 0x52, 0x00, 0x51,
	0x00, 0x4F, 0x00, 0x4D, 0x00, 0x4C, 0x00, 0x4A, 0x00, 0x48,
	0x00, 0x47, 0x00, 0x4A, 0x00, 0x4C, 0x00, 0x4E, 0x00, 0x50,
	0x00, 0x52, 0x00, 0x54, 0x00, 0x56, 0x00, 0x58, 0x00, 0x5B,
	0x00, 0x5D, 0x00, 0x5F, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65,

	0x00, 0x66, 0xFF, 0x65, 0xFE, 0x64, 0xFE, 0x63, 0xFE, 0x62,
	0xFD, 0x61, 0xFD, 0x60, 0xFC, 0x5F, 0xFB, 0x5E, 0xFB, 0x5D,
	0xFB, 0x5C, 0xFA, 0x5A, 0xFA, 0x59, 0xF9, 0x58, 0xF8, 0x57,
	0xF8, 0x57, 0xF8, 0x5A, 0xFA, 0x5D, 0xF9, 0x5F, 0xF9, 0x62,
	0xFB, 0x65, 0xFB, 0x68, 0xFA, 0x6A, 0xFC, 0x6D, 0xFC, 0x70,
	0xFC, 0x73, 0xFE, 0x75, 0xFE, 0x78, 0xFE, 0x7B, 0x00, 0x7E,

	0x00, 0x7F, 0x00, 0x7C, 0x02, 0x7A, 0x02, 0x77, 0x02, 0x74,
	0x04, 0x71, 0x04, 0x6F, 0x04, 0x6C, 0x06, 0x69, 0x05, 0x66,
	0x05, 0x64, 0x07, 0x61, 0x07, 0x5E, 0x06, 0x5B, 0x08, 0x58,
	0x08, 0x56, 0x07, 0x56, 0x06, 0x56, 0x06, 0x56, 0x06, 0x56,
	0x05, 0x56, 0x05, 0x56, 0x04, 0x56, 0x03, 0x56, 0x03, 0x56,
	0x03, 0x56, 0x02, 0x56, 0x02, 0x56, 0x01, 0x56, 0x00, 0x56,

	0x00, 0x56, 0xFF, 0x56, 0xFE, 0x56, 0xFE, 0x56, 0xFE, 0x56,
	0xFD, 0x56, 0xFD, 0x56, 0xFC, 0x56, 0xFB, 0x56, 0xFB, 0x56,
	0xFB, 0x56, 0xFA, 0x56, 0xFA, 0x56, 0xF9, 0x56, 0xF8, 0x56,
	0xF9, 0x56, 0xF9, 0x55, 0xFA, 0x54, 0xFA, 0x54, 0xFA, 0x53,
	0xFC, 0x52, 0xFC, 0x51, 0xFC, 0x50, 0xFD, 0x50, 0xFD, 0x4F,
	0xFD, 0x4E, 0xFF, 0x4D, 0xFF, 0x4C, 0xFF, 0x4B, 0x00, 0x4A,

	0x00, 0x4A, 0x00, 0x4B, 0x00, 0x4C, 0x00, 0x4D, 0x00, 0x4D,
	0x00, 0x4E, 0x00, 0x4F, 0x00, 0x50, 0x00, 0x51, 0x00, 0x52,
	0x00, 0x53, 0x00, 0x53, 0x00, 0x54, 0x00, 0x55, 0x00, 0x56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,

	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56, 0x00, 0X56,
	0x00, 0X57, 0x00, 0X57, 0x00, 0X58, 0x00, 0X59, 0x00, 0X59,
	0x00, 0X5A, 0x00, 0X5B, 0x00, 0X5B, 0x00, 0X5C, 0x00, 0X5C,
	0x00, 0X5D, 0x00, 0X5E, 0x00, 0X5E, 0x00, 0X5F, 0x00, 0X60,
};
static char ce_cmdD6[2] = {0xD6, 0x08};
#endif
/* FIH-SW2-MM-NC-BKL_PWM-01-[+ */
/* Program PWM frequency to about 33 KHz */
static char shift_addr00[2] = {0x00, 0x00}; /* DTYPE_GEN_WRITE1 */
static char shift_addr01[2] = {0x00, 0x01}; /* DTYPE_GEN_WRITE1 */
static char shift_addr80[2] = {0x00, 0x80}; /* DTYPE_GEN_WRITE1 */
static char shift_addrB1[2] = {0x00, 0xB1}; /* DTYPE_GEN_WRITE1 */
static char shift_addrB4[2] = {0x00, 0xB4}; /* DTYPE_GEN_WRITE1 */
static char pwm_para3[2] = {0xC6, 0x02}; /* DTYPE_GEN_WRITE1 */
static char pwm_para5[2] = {0xC6, 0x10}; /* DTYPE_GEN_WRITE1 */
/* FIH-SW2-MM-NC-BKL_PWM-01-]- */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
static char write_ctrl_display[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */
/* FIH-SW2-MM-NC-LCM_INIT-00-[+ */
static char write_display_brightness[2] = {0x51, 0x00}; /* DTYPE_DCS_WRITE1 */
/* FIH-SW2-MM-NC-LCM_INIT-00-]- */
static char orise_manufacture_idDA[2] = {0xDA, 0x00}; /* DTYPE_DCS_READ */

/* FIH-SW2-MM-KW-AUO_INIT-02+{ */
static struct dsi_cmd_desc orise_auo_video_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(extc), extc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmd2), cmd2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(cmdD8), cmdD8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr01), shift_addr01},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmdD8), cmdD8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdE1), auo_gamma_cmdE1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdE2), auo_gamma_cmdE2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdEC), auo_gamma_cmdEC},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdED), auo_gamma_cmdED},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auo_gamma_cmdEE), auo_gamma_cmdEE},
#if 0
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD4), ce_cmdD4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD5), ce_cmdD5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(ce_cmdD6), ce_cmdD6},
#endif
/* FIH-SW2-MM-NC-BKL_PWM-01-[+ */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB1), shift_addrB1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para3), pwm_para3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB4), shift_addrB4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para5), pwm_para5},
/* FIH-SW2-MM-NC-BKL_PWM-01-]- */
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_ctrl_display), write_ctrl_display},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_on), display_on}
};
/* FIH-SW2-MM-KW-AUO_INIT-02-} */

static struct dsi_cmd_desc orise_cmi_video_on_cmds[] = {
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-[+ */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmi_extc), cmi_extc},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cmi_cmd2), cmi_cmd2},
/* FIH-SW2-MM-NC-BKL_PWM-01-[+ */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB1), shift_addrB1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para3), pwm_para3},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addrB4), shift_addrB4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(pwm_para5), pwm_para5},
/* FIH-SW2-MM-NC-BKL_PWM-01-]- */
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_ctrl_display), write_ctrl_display},
/* FIH-SW2-MM-KW-CMI_Gamma-00+{ */
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_cmd1), gamma_cmd1},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_cmdE7), gamma_cmdE7},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_cmdE8), gamma_cmdE8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(gamma_set), gamma_set},
#if 0
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD4), ce_cmdD4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr00), shift_addr00},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ce_cmdD5), ce_cmdD5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(shift_addr80), shift_addr80},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 0,
		sizeof(ce_cmdD6), ce_cmdD6},
#endif
/* FIH-SW2-MM-KW-CMI_Gamma-00-} */
	{DTYPE_DCS_WRITE, 1, 0, 0, 20,
		sizeof(display_on), display_on},
/* FIH-SW2-MM-NC-CMI_INIT-20120409-00-]- */
};

static struct dsi_cmd_desc orise_video_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 20,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc orise_video_bkl_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_display_brightness), write_display_brightness}
};

static struct dsi_cmd_desc orise_ReadDA = {
	DTYPE_DCS_READ, 1, 0, 1, 20, sizeof(orise_manufacture_idDA), 
										orise_manufacture_idDA};

static int mipi_orise_manufacture_id(struct msm_fb_data_type *mfd)
{


	char retDA = 0;
	struct dsi_buf *tp = &orise_tx_buf;
	struct dsi_buf *rp = &orise_rx_buf;

	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);
	mipi_dsi_cmds_rx(mfd, tp, rp, &orise_ReadDA, 1);
	retDA = *((char *) rp->data);

	printk(KERN_ALERT "[DISPLAY] Panel ID <0x%02x>\n", retDA);
	return retDA;
	
}

/* FIH-SW-MM-VH-DISPLAY-22*[ */
static int mipi_orise_lcd_on(struct platform_device *pdev)
{
	int rc = 0;  /* FIH-SW2-MM-NC-LCM_INIT-00 */
	struct msm_fb_data_type *mfd = NULL;

	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (unlikely(display_initialize))
		return 0;

	if (unlikely(gPanelModel == 0)) {
		gPanelModel = mipi_orise_manufacture_id(mfd);
	}

    //gPanelModel = CMI_PANEL_ID;

	switch (gPanelModel) {
		case AUO_PANEL_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_auo_video_on_cmds,
					ARRAY_SIZE(orise_auo_video_on_cmds));
			break;
		case CMI_PANEL_ID:
			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_cmi_video_on_cmds,
					ARRAY_SIZE(orise_cmi_video_on_cmds));
			break;
		default:
			printk(KERN_ERR "[DISPLAY] illegal PID <0x%02x>\n", gPanelModel);
            			rc = mipi_dsi_cmds_tx(&orise_tx_buf, orise_cmi_video_on_cmds,
					ARRAY_SIZE(orise_cmi_video_on_cmds));
	}

	printk(KERN_ALERT "[DISPLAY] dsi commands done, rc <%d>, PID <0x%02x>\n",
			rc, gPanelModel);

	display_initialize = 1;

	if (rc > 0)
		rc = 0;

	return rc;
}
/* FIH-SW-MM-VH-DISPLAY-22*] */

static int mipi_orise_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = NULL;

	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (unlikely(!display_initialize))
		return 0;

	mipi_set_tx_power_mode(0);
	mipi_dsi_cmds_tx(&orise_tx_buf, orise_video_off_cmds,
			ARRAY_SIZE(orise_video_off_cmds));
	mipi_set_tx_power_mode(1);

	display_initialize = 0;

	return 0;
}

static void mipi_orise_lcd_backlight(struct msm_fb_data_type *mfd)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s, set backlight level to %d\n",
			__func__, mfd->bl_level);

/* FIH-SW2-MM-NC-LCM_INIT-01-[+ */
	if (unlikely(!display_initialize))
		return;

	write_display_brightness[1] = BKL_PWM[mfd->bl_level];  /* Duty_Cycle */

	down(&mfd->dma->mutex);
	mipi_set_tx_power_mode(0);
	mipi_dsi_cmds_tx(&orise_tx_buf, orise_video_bkl_cmds,
			ARRAY_SIZE(orise_video_bkl_cmds));
	mipi_set_tx_power_mode(1);
	up(&mfd->dma->mutex);
/* FIH-SW2-MM-NC-LCM_INIT-01-]- */
}

static int __devinit mipi_orise_lcd_probe(struct platform_device *pdev)
{
	printk(KERN_ALERT "[DISPLAY] Enter %s\n", __func__);

	if (pdev->id == 0) {
		mipi_orise_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_orise_lcd_probe,
	.driver = {
		.name   = "mipi_orise",
	},
};

static struct msm_fb_panel_data orise_panel_data = {
	.on		= mipi_orise_lcd_on,
	.off	= mipi_orise_lcd_off,
	.set_backlight = mipi_orise_lcd_backlight,
};

static int ch_used[3] = {0};

int mipi_orise_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret = 0;

printk(KERN_ALERT "[DISPLAY]%s: E\n", __func__);

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	pdev = platform_device_alloc("mipi_orise", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	ch_used[channel] = TRUE;
	orise_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &orise_panel_data,
		sizeof(orise_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

printk(KERN_ALERT "[DISPLAY]%s: X\n", __func__);

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_orise_lcd_init(void)
{
	printk(KERN_ALERT "[DISPLAY]%s: E\n", __func__);
	mipi_dsi_buf_alloc(&orise_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&orise_rx_buf, DSI_BUF_SIZE);

	printk(KERN_ALERT "[DISPLAY]%s: X\n", __func__);

	return platform_driver_register(&this_driver);
}

module_init(mipi_orise_lcd_init);
