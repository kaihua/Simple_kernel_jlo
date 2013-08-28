/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 * Copyright(C) 2012-2013 Foxconn International Holdings, Ltd. All rights reserved.
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

#include "msm_sensor.h"
#include "isx006.h"
#include <linux/delay.h>//For hr_msleep()
#include <mach/camera.h> /*MTD-MM-SL-SupportFlash-01+ */
#include "msm.h" /* MTD-MM-SL-Add5M2ndSource-00+ */

#define SENSOR_NAME "isx006"

#define ISX006_OM_RETRY_COUNT 100
#define ISX006_CM_RETRY_COUNT 80

#define ISX006_AE_RETRY_COUNT 30  /*MTD-MM-SL-SupportFlash-01+ */


#define isx006_MSB_MASK            0xFF00
#define isx006_LSB_MASK            0x00FF

/*MTD-MM-SL-ImproveMainCamera-00+{ */
#define ISX006_ENABLE_HW_STANDBY 1
/*MTD-MM-SL-ImproveMainCamera-00+} */

#ifdef CONFIG_FIH_HR_MSLEEP
#define cam_msleep hr_msleep
#else
#define cam_msleep msleep
#endif

extern struct focus_roi_info g_msm_sensor_focus_roi_info;//FIH-SW-MM-MC-ImplementCameraTouchFocusforIsx006-00+
DEFINE_MUTEX(isx006_mut);
static int csi_config; /*MTD-MM-SL-ImproveMainCamera-00+ */
static uint8_t vendor_id = 0x00; 
static int8_t AF_type = 0; /* MTD-MM-SL-SupportFlash-02+ */ 
static uint32_t OTP_0_value = 0xFFFFFFFF; //bit[0:31 ] 
static uint32_t OTP_1_value = 0xFFFFFFFF; //bit[32:63 ] 
static uint32_t OTP_2_value = 0xFFFFFFFF; //bit[64:95 ] 
static uint16_t AF_A_value = 0xFFFF; 
static uint16_t AF_B_value = 0xFFFF; 
static uint16_t AF_C_value = 0xFFFF; 
static uint16_t AF_D_value = 0xFFFF; 
static uint16_t AF_E_value = 0xFFFF; 
static uint16_t AF_F_value = 0xFFFF; 
static uint16_t AF_G_value = 0xFFFF; 
static uint16_t AF_H_value = 0xFFFF; 
static uint16_t C_NR = 0xFFFF; 
static uint16_t C_NB = 0xFFFF; 
static uint16_t C_PR = 0xFFFF; 
static uint16_t C_PB = 0xFFFF;
static uint16_t NORMR = 0xFFFF;   
static uint16_t NORMB = 0xFFFF; 
static uint16_t AWB_PRER = 0xFFFF; 
static uint16_t AWB_PREB = 0xFFFF;  
static uint16_t Shading_index = 0xFFFF;

/*MTD-MM-SL-SupportFlash-01+{ */
static bool flash_enable = false;
static uint16_t AF_full_range = 0; /* MTD-MM-SL-Add5M2ndSource-01+ */
bool torch_enable = false; /* MTD-MM-SL-AddEXIF-04* */
bool STARTUP = false; /*MTD-MM-SL-ImproveMainCamera-00+ */
extern bool F_STARTUP;/*MTD-MM-SL-ImproveMainCamera-03+ */
int8_t rc_af_check = 0; /* MTD-MM-SL-SupportAF-00+ */
static int16_t isx006_scene = 0; /*MTD-MM-SL-AddForSoMCScene-00+ */
static bool AF_skip = false; /* MTD-MM-SL-SupportFlash-02+ */ 

uint32_t AESCL_AUTO = 0xFFFF; //0x0288
uint32_t ERRSCL_AUTO = 0xFFFF; //0x0284
uint32_t AESCL_NOW = 0xFFFF; //0x028A
uint32_t ERRSCL_NOW = 0xFFFF; //0x0286
/*MTD-MM-SL-SupportFlash-01+} */
/* MTD-MM-UW-AF_tune-00+ */
static uint16_t current_af_mode = 0;
static bool f_CAF = false;
/* MTD-MM-UW-AF_tune-00- */
static struct msm_sensor_ctrl_t isx006_s_ctrl;
/*MTD-MM-SL-ImproveMainCamera-00+{ */
extern struct msm_sensor_ctrl_t *s_ctrl_f; 
static struct msm_sensor_ctrl_t *s_ctrl_m; /*MTD-MM-SL-ImproveMainCamera-02+ */

enum Exit_Standby_Action {
	ACT_MCLK_ON,
	ACT_EXIT_STANDBY
};
/*MTD-MM-SL-ImproveMainCamera-00+} */

/*MTD-MM-SL-PatchForCameraFeature-00*{ */
static struct msm_camera_i2c_reg_conf isx006_snap_settings[] = {
	{ 0x0011, 0x02},
};	

static struct msm_camera_i2c_reg_conf isx006_prev_settings[] = {
	{ 0x0011, 0x00},
};

static struct msm_camera_i2c_reg_conf isx006_video_settings[] = {
	{ 0x0011, 0x00},
};
/*MTD-MM-SL-PatchForCameraFeature-00*} */



static struct v4l2_subdev_info isx006_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
	/* more can be supported, to be added later */
};


static struct msm_camera_i2c_conf_array isx006_init_conf[] = {
	{&isx006_recommend_settings[0],
	ARRAY_SIZE(isx006_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&isx006_recommend_part2_settings[0],
	ARRAY_SIZE(isx006_recommend_part2_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&isx006_recommend_part3_settings[0],
	ARRAY_SIZE(isx006_recommend_part3_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&isx006_recommend_part4_settings[0],
	ARRAY_SIZE(isx006_recommend_part4_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&isx006_recommend_part5_settings[0],
	ARRAY_SIZE(isx006_recommend_part5_settings), 1, MSM_CAMERA_I2C_BYTE_DATA},	
};

static struct msm_camera_i2c_conf_array isx006_confs[] = {
	{&isx006_snap_settings[0],
	ARRAY_SIZE(isx006_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&isx006_prev_settings[0],
	ARRAY_SIZE(isx006_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&isx006_video_settings[0],
	ARRAY_SIZE(isx006_video_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},	
};

/*MTD-MM-SL-PatchForCameraFeature-00*{ */
static struct msm_sensor_output_info_t isx006_dimensions[] = {
	{ /* For SNAPSHOT */
		.x_output = 0xA20, /*2592 */  /*for 5Mp*/
		.y_output = 0x798, /*1944*/
		.line_length_pclk = 0xA20,
		.frame_length_lines = 0x798,
		.vt_pixel_clk = 37791360,  /* x_output*y_output*7.5(fps)*/
		.op_pixel_clk = 37791360,
		.binning_factor = 1,
	},
	{ /* For PREVIEW */
		.x_output = 0x500 ,/*1280 */
		.y_output = 0x3C0, /*960*/
		.line_length_pclk = 0x500,
		.frame_length_lines = 0x3C0,
		.vt_pixel_clk = 36864000,  /* x_output*y_output*30(fps)*/
		.op_pixel_clk = 18432000,  /*vt_pixel_clk  /2 */
		.binning_factor = 1,
	},
	{ /* For VIDEO */
		.x_output = 0x500 ,/*1280 */
		.y_output = 0x3C0, /*960*/
		.line_length_pclk = 0x500,
		.frame_length_lines = 0x3C0,
		.vt_pixel_clk = 36864000,  /* x_output*y_output*30(fps)*/
		.op_pixel_clk = 18432000,  /*vt_pixel_clk  /2 */
		.binning_factor = 1,
	},	
};
/*MTD-MM-SL-PatchForCameraFeature-00*} */


static struct msm_camera_i2c_reg_conf isx006_no_effect[] = {
	/*
	{0x81, 0x00, 0x00, 0x00, 0xDF},
	{0x28, 0x00,},
	{0xd2, 0x00,},
	{0xda, 0x80,},
	{0xdb, 0x80,},
	*/
};

static struct msm_camera_i2c_conf_array isx006_no_effect_confs[] = {
	{&isx006_no_effect[0],
	ARRAY_SIZE(isx006_no_effect), 0,
	MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},
};

/*MTD-MM-SL-AddWB-00+{ */ 
static struct msm_camera_i2c_reg_conf isx006_wb_oem[][1] = {
	{{-1, -1, -1, -1 , -1},},  /* OFF: NOT SUPPORTED */
	{{0x0102, 0x20},},         /* AUTO*/
	{{-1, -1, -1, -1 , -1},},	/* CUSTOM: NOT SUPPORTED */
	{{0x0102, 0x08},},         /*INCANDISCENT*/
	{{0x0102, 0x07},},	        /*FLOURESECT */
	{{0x0102, 0x14},},         /*DAYLIGHT*/
	{{0x0102, 0x16},},         /*CLOUDY*/
};
 
static struct msm_camera_i2c_conf_array isx006_wb_oem_confs[][1] = { 
	{{isx006_wb_oem[0], ARRAY_SIZE(isx006_wb_oem[0]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_wb_oem[1], ARRAY_SIZE(isx006_wb_oem[1]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_wb_oem[2], ARRAY_SIZE(isx006_wb_oem[2]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_wb_oem[3], ARRAY_SIZE(isx006_wb_oem[3]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_wb_oem[4], ARRAY_SIZE(isx006_wb_oem[4]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_wb_oem[5], ARRAY_SIZE(isx006_wb_oem[5]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_wb_oem[6], ARRAY_SIZE(isx006_wb_oem[6]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},}, 
};

static int isx006_wb_oem_enum_map[] = {
	MSM_V4L2_WB_OFF,
	MSM_V4L2_WB_AUTO ,
	MSM_V4L2_WB_CUSTOM,
	MSM_V4L2_WB_INCANDESCENT,
	MSM_V4L2_WB_FLUORESCENT,
	MSM_V4L2_WB_DAYLIGHT,
	MSM_V4L2_WB_CLOUDY_DAYLIGHT,
};

static struct msm_camera_i2c_enum_conf_array isx006_wb_oem_enum_confs = {
	.conf = &isx006_wb_oem_confs[0][0],
	.conf_enum = isx006_wb_oem_enum_map,
	.num_enum = ARRAY_SIZE(isx006_wb_oem_enum_map),
	.num_index = ARRAY_SIZE(isx006_wb_oem_confs),
	.num_conf = ARRAY_SIZE(isx006_wb_oem_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};
/*MTD-MM-SL-AddWB-00+} */

//FIH-SW-MM-MC-ImplementCameraSceneModeforIsx006-00+{
static struct msm_camera_i2c_conf_array isx006_scene_oem_confs[][1] = { 
	{{isx006_scene_oem[0], ARRAY_SIZE(isx006_scene_oem[0]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[1], ARRAY_SIZE(isx006_scene_oem[1]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[2], ARRAY_SIZE(isx006_scene_oem[2]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[3], ARRAY_SIZE(isx006_scene_oem[3]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[4], ARRAY_SIZE(isx006_scene_oem[4]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[5], ARRAY_SIZE(isx006_scene_oem[5]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[6], ARRAY_SIZE(isx006_scene_oem[6]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},}, 
	{{isx006_scene_oem[7], ARRAY_SIZE(isx006_scene_oem[7]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[8], ARRAY_SIZE(isx006_scene_oem[8]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[9], ARRAY_SIZE(isx006_scene_oem[9]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[10], ARRAY_SIZE(isx006_scene_oem[10]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[11], ARRAY_SIZE(isx006_scene_oem[11]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[12], ARRAY_SIZE(isx006_scene_oem[12]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[13], ARRAY_SIZE(isx006_scene_oem[13]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},}, 
	{{isx006_scene_oem[14], ARRAY_SIZE(isx006_scene_oem[14]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[15], ARRAY_SIZE(isx006_scene_oem[15]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[16], ARRAY_SIZE(isx006_scene_oem[16]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[17], ARRAY_SIZE(isx006_scene_oem[17]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[18], ARRAY_SIZE(isx006_scene_oem[18]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_scene_oem[19], ARRAY_SIZE(isx006_scene_oem[19]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
};

static int isx006_scene_oem_enum_map[] = {
    MSM_V4L2_SCENE_OFF,
    MSM_V4L2_SCENE_AUTO,
    MSM_V4L2_SCENE_LANDSCAPE,
    MSM_V4L2_SCENE_SNOW,
    MSM_V4L2_SCENE_BEACH,
    MSM_V4L2_SCENE_SUNSET,
    MSM_V4L2_SCENE_NIGHT,
    MSM_V4L2_SCENE_PORTRAIT,
    MSM_V4L2_SCENE_BACKLIGHT,
    MSM_V4L2_SCENE_SPORTS,
    MSM_V4L2_SCENE_ANTISHAKE,
    MSM_V4L2_SCENE_FLOWERS,
    MSM_V4L2_SCENE_CANDLELIGHT,
    MSM_V4L2_SCENE_FIREWORKS,
    MSM_V4L2_SCENE_PARTY,
    MSM_V4L2_SCENE_NIGHT_PORTRAIT,
    MSM_V4L2_SCENE_THEATRE,
    MSM_V4L2_SCENE_ACTION,
    MSM_V4L2_SCENE_AR,
    MSM_V4L2_SCENE_DOCUMENT,//19
};

static struct msm_camera_i2c_enum_conf_array isx006_scene_oem_enum_confs = {
	.conf = &isx006_scene_oem_confs[0][0],
	.conf_enum = isx006_scene_oem_enum_map,
	.num_enum = ARRAY_SIZE(isx006_scene_oem_enum_map),
	.num_index = ARRAY_SIZE(isx006_scene_oem_confs),
	.num_conf = ARRAY_SIZE(isx006_scene_oem_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};
//FIH-SW-MM-MC-ImplementCameraSceneModeforIsx006-00+}

/*MTD-MM-SL-AddBrightness-01*{ */
/*MTD-MM-SL-AddBrightness-00+{ */
/*Default camera only supports 5 level brightness (-2 ~ +2)
    SoMC camera would supprt 13 level brightness (-6 ~ +6)*/
static struct msm_camera_i2c_reg_conf isx006_brightness[][1] = {
	{{0x0080, 0xfa},}, /* LEVEL -6: -2 */
	{{0x0080, 0xfb},}, /* LEVEL -5: -1.66 */
	{{0x0080, 0xfc},}, /* LEVEL -4: -1.33 */
	{{0x0080, 0xfd},}, /* LEVEL -3: -1 */
	{{0x0080, 0xfe},}, /* LEVEL -2: -0.66 */	
	{{0x0080, 0xff},}, /* LEVEL -1: -0.33 */
	{{0x0080, 0x00},}, /* LEVEL 0 */
	{{0x0080, 0x01},}, /* LEVEL 1: 0.33 */
	{{0x0080, 0x02},}, /* LEVEL 2: 0.66*/
	{{0x0080, 0x03},}, /* LEVEL 3: 1 */
	{{0x0080, 0x04},}, /* LEVEL 4: 1.33 */	
	{{0x0080, 0x05},}, /* LEVEL 5: 1.66 */
	{{0x0080, 0x06},}, /* LEVEL 6: 2 */
};

static struct msm_camera_i2c_conf_array isx006_brightness_confs[][1] = { 
	{{isx006_brightness[0], ARRAY_SIZE(isx006_brightness[0]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[1], ARRAY_SIZE(isx006_brightness[1]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[2], ARRAY_SIZE(isx006_brightness[2]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[3], ARRAY_SIZE(isx006_brightness[3]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[4], ARRAY_SIZE(isx006_brightness[4]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[5], ARRAY_SIZE(isx006_brightness[5]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[6], ARRAY_SIZE(isx006_brightness[6]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},}, 
	{{isx006_brightness[7], ARRAY_SIZE(isx006_brightness[7]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[8], ARRAY_SIZE(isx006_brightness[8]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[9], ARRAY_SIZE(isx006_brightness[9]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[10], ARRAY_SIZE(isx006_brightness[10]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[11], ARRAY_SIZE(isx006_brightness[11]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[12], ARRAY_SIZE(isx006_brightness[12]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},	
};

static int isx006_brightness_enum_map[] = {
	MSM_V4L2_BRIGHTNESS_V0, //-6
	MSM_V4L2_BRIGHTNESS_V1,
	MSM_V4L2_BRIGHTNESS_V2,
	MSM_V4L2_BRIGHTNESS_V3,
	MSM_V4L2_BRIGHTNESS_V4,
	MSM_V4L2_BRIGHTNESS_V5,
	MSM_V4L2_BRIGHTNESS_V6, //0
	MSM_V4L2_BRIGHTNESS_V7,
	MSM_V4L2_BRIGHTNESS_V8,
	MSM_V4L2_BRIGHTNESS_V9,
	MSM_V4L2_BRIGHTNESS_V10,
	MSM_V4L2_BRIGHTNESS_V11,
	MSM_V4L2_BRIGHTNESS_V12, //6	
};

static struct msm_camera_i2c_enum_conf_array isx006_brightness_enum_confs = {
	.conf = &isx006_brightness_confs[0][0],
	.conf_enum = isx006_brightness_enum_map,
	.num_enum = ARRAY_SIZE(isx006_brightness_enum_map),
	.num_index = ARRAY_SIZE(isx006_brightness_confs),
	.num_conf = ARRAY_SIZE(isx006_brightness_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};
/*MTD-MM-SL-AddBrightness-00+} */
/*MTD-MM-SL-AddBrightness-01*} */

/*MTD-MM-UW-AddAF-00*{ */
static struct msm_camera_i2c_reg_conf isx006_af[][1] = {
	{{0x0080, 0xfe},}, /* LEVEL -2: -0.66 */	
	{{0x0080, 0xff},}, /* LEVEL -1: -0.33 */
	{{0x0080, 0x00},}, /* LEVEL 0 */
	{{0x0080, 0x01},}, /* LEVEL 1: 0.33 */
	{{0x0080, 0x02},}, /* LEVEL 2: 0.66*/
};


static struct msm_camera_i2c_conf_array isx006_af_confs[][1] = { 
	{{isx006_brightness[0], ARRAY_SIZE(isx006_af[0]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[1], ARRAY_SIZE(isx006_af[1]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[2], ARRAY_SIZE(isx006_af[2]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[3], ARRAY_SIZE(isx006_af[3]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},
	{{isx006_brightness[4], ARRAY_SIZE(isx006_af[4]),  0,
		MSM_CAMERA_I2C_BYTE_DATA},},		
};

static int isx006_af_enum_map[] = { 
	MSM_V4L2_AF_OFF, 
	MSM_V4L2_AF_ON,
	MSM_V4L2_AF_CAF, 
    MSM_V4L2_AF_SAF, 
	MSM_V4L2_AF_MACRO,
	MSM_V4L2_AF_INFINITY,
};

static struct msm_camera_i2c_enum_conf_array isx006_af_enum_confs = {
	.conf = &isx006_af_confs[0][0],
	.conf_enum = isx006_af_enum_map,
	.num_enum = ARRAY_SIZE(isx006_af_enum_map),
	.num_index = ARRAY_SIZE(isx006_af_confs),
	.num_conf = ARRAY_SIZE(isx006_af_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};
/*MTD-MM-UW-AddAF-00*} */

static int isx006_check_om(struct msm_camera_i2c_client *client, const char *tag)
{
    int i =0;
    int rc = 0;
    uint16_t irq_status = 0x0;
    uint16_t v_read;
    uint16_t v_temp; 

    printk("isx006_check_om(0x%x): %s \n", client->client->addr, tag);

    /* Make sure operation mode is changed */
    for (i = 0; i < ISX006_OM_RETRY_COUNT; i++)
    {
        rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_om: msm_camera_i2c_read failed, REG(0x%x) !\n", 0x00F8);
            goto error;
        }
        
        if (irq_status & 0x01)
            break;
        cam_msleep(10);
    }

    if (i >= ISX006_OM_RETRY_COUNT)
    {
        rc = -ETIME;
        pr_err("isx006_check_om: Operation mode not change !!!\n");
        goto error;
    }
    printk("isx006_check_om: Operation mode change\n");


    /* Clear interrupt status */
    for (i = 0; i < ISX006_OM_RETRY_COUNT; i++)
    {
        rc = msm_camera_i2c_read(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_om: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00FC);
            goto error;
        }
        v_temp = v_read | 0x01;    

        rc = msm_camera_i2c_write_invert(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_om: isx006_i2c_write_parallel failed, REG(0x%x) !\n", 0x00FC);
            goto error;
        }
        cam_msleep(10);

        //2.Check interrupt status is cleaned
        rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
        {
            pr_err("isx006_check_om: saddr: Read REG(0x%x) failed !\n", 0x00F8);
            goto error;
        }
        else
        {
            // Check OM_CHANGED STS (bit0)
            if (!(irq_status & 0x01))
            {
                printk("isx006_check_om: saddr: OM_CHANGED STS is clear. \n");
                break;
            }
        }
        cam_msleep(10);    
        printk("isx006_check_om: Retry for clear interrupt status ~ \n");
    }

    if (i >= ISX006_OM_RETRY_COUNT)
    {
        rc = -ETIME;
        pr_err("isx006_check_om: Interrupt status is not cleaned !\n");
        goto error;
    }

    printk("isx006_check_om: Success \n");
    return rc;

error:
    pr_err("isx006_check_om: Failed !\n");
    return rc;
}

/*MTD-MM-SL-ImproveMainCamera-00+{ */
static int isx006_check_om_nonAck(struct msm_camera_i2c_client *client, const char *tag)
{
	int i =0;
	int rc = 0;
	uint16_t irq_status = 0x0;
	uint16_t v_read;
	uint16_t v_temp; 
	
	printk("isx006_check_om_nonAck\n");
	
	/* Make sure operation mode is changed */
	for (i = 0; i < ISX006_OM_RETRY_COUNT; i++){
		rc = msm_camera_i2c_read_invert_nonAck(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("isx006_check_om_nonAck: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00F8);
			goto error;
		}
			
		if (irq_status & 0x01)
			break;
		cam_msleep(10);
	}
	
	if (i >= ISX006_OM_RETRY_COUNT){
		rc = -ETIME;
		pr_err("isx006_check_om_nonAck: Operation mode not change !!!\n");
		goto error;
	}
	printk("isx006_check_om_nonAck: Operation mode change\n");
	
	
	/* Clear interrupt status */
	for (i = 0; i < ISX006_OM_RETRY_COUNT; i++){
		rc = msm_camera_i2c_read_invert_nonAck(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("isx006_check_om_nonAck: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00FC);
			goto error;
		}
		v_temp = v_read | 0x01;    
	
		rc = msm_camera_i2c_write_invert_nonAck(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("isx006_check_om_nonAck: isx006_i2c_write_parallel failed, REG(0x%x) !\n", 0x00FC);
			goto error;
		}
		cam_msleep(10);
	
		//2.Check interrupt status is cleaned
		rc = msm_camera_i2c_read_invert_nonAck(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0){
			pr_err("isx006_check_om_nonAck: saddr: Read REG(0x%x) failed !\n", 0x00F8);
			goto error;
			
		}else{
			// Check OM_CHANGED STS (bit0)
			if (!(irq_status & 0x01)){
				printk("isx006_check_om_nonAck: saddr: OM_CHANGED STS is clear. \n");
				break;
			}
		}
		cam_msleep(10);    
		printk("isx006_check_om_nonAck: Retry for clear interrupt status ~ \n");
	}
	
	if (i >= ISX006_OM_RETRY_COUNT){
		rc = -ETIME;
		pr_err("isx006_check_om_nonAck: Interrupt status is not cleaned !\n");
		goto error;
	}
	
	printk("isx006_check_om_nonAck: Success \n");
	return rc;
	
error:
	pr_err("isx006_check_om_nonAck: Failed !\n");
	return rc;
}
/*MTD-MM-SL-ImproveMainCamera-00+} */

/*MTD-MM-UW-SnapshotFail-01*{ */
static int isx006_check_cm(struct msm_camera_i2c_client *client, enum isx006_mode new_mode)
{
    int i = 0;
    int rc = 0;
    uint16_t irq_status = 0x0;
    uint16_t byte_value = 0xFF;
    uint16_t v_temp = 0xFF;
    uint16_t v_read = 0xFF;

    printk("isx006_check_cm(0x%x):\n", client->client->addr);

    /* Make sure operation mode is changed */
    for (i = 0; i < ISX006_CM_RETRY_COUNT; i++)
    {
        rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_cm: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00F8);
            goto error;
        }
        if (irq_status & 0x02)
        {
                break;
        }
        cam_msleep(10);    
     }

    if (i >= ISX006_CM_RETRY_COUNT)
    {
        rc = -ETIME;
        pr_err("isx006_check_cm: Camera mode changed fail !!!\n");
        goto error;
    }
    printk("isx006_check_cm: Camera mode changed\n");

    
    /* Clear interrupt status */
    for (i = 0; i < ISX006_CM_RETRY_COUNT; i++)
    {
        /*1. Clear interrupt status*/
        rc = msm_camera_i2c_read(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_cm: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00FC);
            goto error;
        }
        v_temp = v_read | 0x02;
        
        rc = msm_camera_i2c_write_invert(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_cm: isx006_i2c_write_parallel failed, REG(0x%x) !\n",0x00FC);
            goto error;
        }
        cam_msleep(5);

        /*2.Check interrupt status is cleaned*/
        rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
        {
            pr_err("isx006_check_cm: saddr: Read REG(0x%x) failed !\n", 0x00F8);
            goto error;
        }
        else
        {
            // Check MO_CHANGED STS (bit1)
            if (!(irq_status & 0x02))
            {
                printk("isx006_check_cm: CM_CHANGED STS is clear. \n");
                break;
            }
        }
        cam_msleep(5);
        printk("isx006_check_cm: clear interrupt status --- retry time = %d !\n", i);
    }

    if (i >= ISX006_CM_RETRY_COUNT)
    {
        rc = -ETIME;
        pr_err("isx006_check_cm: Interrupt status is not cleaned !\n");
        goto error;
    }
       
    printk("isx006_check_cm: Success \n");
    return rc;

error:
    
#if 1 // Show current mode.
    rc = msm_camera_i2c_read(client, 0x0004, &byte_value, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_check_cm: Try to get current mode failed !\n");
        return rc;
    }
    printk("isx006_check_cm: Failed, REG_0x0004 = 0x%x !\n", byte_value);
#else
    pr_err("isx006_check_cm: Failed !\n");
#endif
    return rc;
}

static int isx006_check_cm_capture(struct msm_camera_i2c_client *client, enum isx006_mode new_mode)
{
    int i = 0;
    int rc = 0;
    uint16_t irq_status = 0x0;
    uint16_t byte_value = 0xFF;
    uint16_t v_temp = 0xFF;
    uint16_t v_read = 0xFF;

    printk("isx006_check_cm(0x%x):\n", client->client->addr);

    /* Make sure operation mode is changed */
    for (i = 0; i < ISX006_CM_RETRY_COUNT; i++)
    {
        rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_cm: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00F8);
            goto error;
        }
        if (irq_status & 0x02)
        {
            rc = msm_camera_i2c_read(client, 0x0004, &byte_value, MSM_CAMERA_I2C_BYTE_DATA);
            if(byte_value == new_mode)
                break;
        }
        cam_msleep(10);    
     }

    if (i >= ISX006_CM_RETRY_COUNT)
    {
        rc = -ETIME;
        pr_err("isx006_check_cm: Camera mode changed fail !!!\n");
        goto error;
    }
    printk("isx006_check_cm: Camera mode changed\n");

    
    /* Clear interrupt status */
    for (i = 0; i < ISX006_CM_RETRY_COUNT; i++)
    {
        /*1. Clear interrupt status*/
        rc = msm_camera_i2c_read(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_cm: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x00FC);
            goto error;
        }
        v_temp = v_read | 0x02;
        
        rc = msm_camera_i2c_write_invert(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_check_cm: isx006_i2c_write_parallel failed, REG(0x%x) !\n",0x00FC);
            goto error;
        }
        cam_msleep(5);

        /*2.Check interrupt status is cleaned*/
        rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
        {
            pr_err("isx006_check_cm: saddr: Read REG(0x%x) failed !\n", 0x00F8);
            goto error;
        }
        else
        {
            // Check MO_CHANGED STS (bit1)
            if (!(irq_status & 0x02))
            {
                printk("isx006_check_cm: CM_CHANGED STS is clear. \n");
                break;
            }
        }
        cam_msleep(5);
        printk("isx006_check_cm: clear interrupt status --- retry time = %d !\n", i);
    }

    if (i >= ISX006_CM_RETRY_COUNT)
    {
        rc = -ETIME;
        pr_err("isx006_check_cm: Interrupt status is not cleaned !\n");
        goto error;
    }
       
    printk("isx006_check_cm: Success \n");
    return rc;

error:
    
#if 1 // Show current mode.
    rc = msm_camera_i2c_read(client, 0x0004, &byte_value, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_check_cm: Try to get current mode failed !\n");
        return rc;
    }
    printk("isx006_check_cm: Failed, REG_0x0004 = 0x%x !\n", byte_value);
#else
    pr_err("isx006_check_cm: Failed !\n");
#endif
    return rc;
}
/*MTD-MM-UW-SnapshotFail-01*} */

int isx006_polling_state_change(struct msm_camera_i2c_client *client, enum isx006_device_status polling_status)
{
    int rc = 0;
    int count = 0;
    int max_polling = 50;
    uint16_t now_status = 0xFFFF;


	printk("isx006_polling_state_change: Start \n");
    do 
    {
        count++;
        cam_msleep(20);
        rc = msm_camera_i2c_read(client, 0x00FE, &now_status, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_polling_state_change: Read Back DEVICESTS_REG(0x%x) failed !\n", 0x00FE);
            goto error;
        }
        if (now_status != polling_status)
        {
            rc = -EINVAL;
        }
        else
        {
            printk("isx006_polling_state_change(%d): now_status(0x%x), polling_status(0x%x).\n", count, now_status, polling_status);
            rc = 0;
        }
    } while( (polling_status != now_status) && (count < max_polling));

    if (rc < 0 || count >= max_polling)
    {
        rc = -EIO;
        pr_err("isx006_polling_state_change: Failed, count = %d !\n", count);
        goto error;
    }

    switch (polling_status) {
    case STS_POWER_DOWN:
        printk("isx006_polling_state_change: Polling success, status = STS_POWER_DOWN.\n");
        break;
            
    case STS_PRE_SLEEP:
        printk("isx006_polling_state_change: Polling success, status = STS_PRE_SLEEP.\n");
        break;
            
    case STS_SLEEP:
        printk("isx006_polling_state_change: Polling success, status = STS_SLEEP.\n");
        break;
            
    case STS_ACTIVE:
        printk("isx006_polling_state_change: Polling success, status = STS_ACTIVE.\n");
        break;
            
    case STS_UNKNOWN:
    default:
        rc = -EINVAL;
        printk("isx006_polling_state_change: Polling failed, status = STS_UNKNOWN !\n");
        break;
    }

	printk("isx006_polling_state_change: Success \n");
    return rc;
    
error:
    pr_err("isx006_polling_state_change fail: now_status(0x%x), polling_status(0x%x).\n", now_status, polling_status);
    return rc;
}

static int32_t isx006_i2c_read_parallel_32bit(struct msm_camera_i2c_client *client,
    unsigned short raddr, uint32_t *rdata)
{
    int32_t rc = 0;
    unsigned char buf_64bits[4];
    buf_64bits[0] = 0xFF;
    buf_64bits[1] = 0xFF;
    buf_64bits[2] = 0xFF;
    buf_64bits[3] = 0xFF;

    if (!rdata)
        return -EIO;

    memset(buf_64bits, 0, sizeof(buf_64bits));

    buf_64bits[0] = (raddr & isx006_MSB_MASK) >> 8;
    buf_64bits[1] = (raddr & isx006_LSB_MASK);

    rc = msm_camera_i2c_rxdata(client, buf_64bits, 4);
    if (rc < 0) {
        printk("isx006_i2c_read_parallel_32bit: 0x%x failed!\n", raddr);
        return rc;
    }  

    *rdata = buf_64bits[3] << 24 | buf_64bits[2] << 16 |buf_64bits[1] << 8 | buf_64bits[0];

    return rc;
}

/*MTD-MM-SL-ImproveMainCamera-01*{ */
/*MTD-MM-SL-ImproveMainCamera-00+{ */
int isx006_enter_standby(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0; 
    uint16_t status = 0xFFFF;
	struct msm_camera_sensor_info *isx006_info = NULL;
	isx006_info = s_ctrl->sensordata;
    
    rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x00FE, &status, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
        pr_err("isx006_enter_standby: i2c_read (REG 0x00FE) failed !\n");
        goto error;
    }
    printk("isx006_enter_standby: camera om status = %d!\n", status);

    /* Set NSTANDBY from H to L */
	rc =  gpio_request(isx006_info->sensor_pwd, "CAM_5M_STBYN");
	if (rc < 0) {
       	pr_err("isx006_enter_standby: gpio_request (%d) failed !.\n", isx006_info->sensor_pwd);
	}
	rc = gpio_direction_output(isx006_info->sensor_pwd, 0);
	if (rc < 0) {
        pr_err("isx006_enter_standby: Pull low CAM_5M_STBYN pin failed !\n");
        goto gpio_sensor_pwd_fail;
    }
	gpio_free(isx006_info->sensor_pwd);

    rc = isx006_check_om_nonAck(s_ctrl->sensor_i2c_client, "For change to Sleep mode");
        if (rc < 0) {
        pr_err("isx006_enter_standby: isx006_check_om_nonAck(For change to Sleep mode) failed !\n");
        goto error;
    }

    /* Here to check ststus is STS_SLEEP */
    rc = isx006_polling_state_change(s_ctrl->sensor_i2c_client, STS_SLEEP);
    if (!rc) {
        printk("isx006_enter_standby: isx006_polling_state_change(STS_SLEEP) success !\n");
        rc = fih_disable_mclk(s_ctrl);
        if (rc < 0) {
            goto error;
        }
		
        cam_msleep(5);
    }

    printk("isx006_enter_standby: Success !\n");
    return rc;

gpio_sensor_pwd_fail:
	gpio_free(isx006_info->sensor_pwd);

error:
    printk("isx006_enter_standby: Failed !\n");
    return rc;
}

int isx006_exit_standby(struct msm_sensor_ctrl_t *s_ctrl, enum Exit_Standby_Action act)
{
    int rc = 0;
    uint16_t status = 0xFFFF;
	struct msm_camera_sensor_info *isx006_info = NULL;
	isx006_info = s_ctrl->sensordata;
    
    switch (act) {
    case ACT_MCLK_ON:
        {
            printk("isx006_exit_standby: case ACT_MCLK_ON \n");
			
            rc = fih_enable_mclk(s_ctrl);
            if (rc < 0) {
                goto error;
            }
            cam_msleep(50); 

            rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x00FE, &status, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                printk("isx006_exit_standby: Can not read camera om status after MCLK enable ..\n"); 
            else
                printk("isx006_exit_standby: [Success] Camera om status = %d ! (MCLK ON)\n", status);
        }
        break;
            
    case ACT_EXIT_STANDBY:
        {
            printk("isx006_exit_standby: case ACT_EXIT_STANDBY \n");
            /* Set NSTANDBY from L to H */
			rc =  gpio_request(isx006_info->sensor_pwd, "CAM_5M_STBYN");
			if (rc < 0) {
       			pr_err("isx006_exit_standby: gpio_request (%d) failed !.\n", isx006_info->sensor_pwd);
			}
			rc = gpio_direction_output(isx006_info->sensor_pwd, 1);
            if (rc < 0){
				pr_err("isx006_exit_standby: gpio_direction_output (%d) failed !.\n", isx006_info->sensor_pwd);
                goto gpio_sensor_pwd_fail;
            }
			gpio_free(isx006_info->sensor_pwd);
            

            rc = isx006_check_om_nonAck(s_ctrl->sensor_i2c_client, "For change to Active mode");
                if (rc < 0) {
                pr_err("isx006_exit_standby: isx006_check_om(For change to Active mode) failed !\n");
                //goto error;
            }

            /* Here to check ststus is STS_ACTIVE */
            rc = isx006_polling_state_change(s_ctrl->sensor_i2c_client, STS_ACTIVE);
            if (rc < 0) {
                pr_err("isx006_exit_standby: isx006_polling_state_change(STS_ACTIVE) failed !\n");
                goto error;
            }
            
            rc = isx006_check_cm(s_ctrl->sensor_i2c_client, MONITOR_MODE);
            if (rc < 0) {
                pr_err("isx006_exit_standby: isx006_check_cm(For CM change to Monitor mode) failed !\n");
                goto error;
            }

            rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x00FE, &status, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0)
                printk("isx006_exit_standby: Can not read camera om status after exit standby mode ..\n"); 
            else
                printk("isx006_exit_standby: [Success] Camera om status = %d ! (Exit standby mode)\n", status);            
        }
        break;
            
    default:
        {
            rc = -EINVAL;
            printk("isx006_exit_standby: ERR: Invalid item !\n");
            goto error;
        }
        break;
    }    
     
    return rc;

gpio_sensor_pwd_fail:
	gpio_free(isx006_info->sensor_pwd);
	
error:
    printk("isx006_exit_standby: Failed !\n");
    return rc;
}
/*MTD-MM-SL-ImproveMainCamera-01*} */

int front_cam_enter_standby(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    uint16_t read_value;

    printk("front_cam_enter_standby: Enter \n");
    rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0)
        goto error;
 
    read_value = read_value | 0x0001;
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x0018, read_value, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0)
        goto error;

    cam_msleep(35);
	
    printk("front_cam_enter_standby: Success \n");
    return 0;

error:
    printk("front_cam_enter_standby: Failed !\n");
    return rc;

}

/*MTD-MM-SL-ImproveMainCamera-00+} */

int isx006_OTP_get(struct msm_camera_i2c_client *client) 
{
    int rc = 0;
    uint32_t read_value;

    uint16_t I_NORMR;    
    uint16_t I_NORMB;
    uint16_t I_AWB_PRER;
    uint16_t I_AWB_PREB;  
    
    isx006_i2c_read_parallel_32bit(client, 0x0250, &read_value);    
    OTP_0_value = read_value;    
    isx006_i2c_read_parallel_32bit(client, 0x0254, &read_value);    
    OTP_1_value = read_value;    
    isx006_i2c_read_parallel_32bit(client, 0x0258, &read_value);    
    OTP_2_value = read_value;        

    printk("isx006_OTP_get: 0x0250 = 0x%x\n", OTP_0_value);    
    printk("isx006_OTP_get: 0x0254 = 0x%x\n", OTP_1_value);
    printk("isx006_OTP_get: 0x0258 = 0x%x\n", OTP_2_value);  
    
    /* Vendor ID -----------------------------------------*/  
    vendor_id = (OTP_2_value >>24) & 0x00000001;
    printk("isx006_OTP_setting: vendor_id = %d\n", vendor_id);    

    /* AF position -----------------------------------------------*/    
    AF_A_value = (uint16_t) ((OTP_0_value >> 5  ) & 0x000003FF);    
    AF_B_value = (uint16_t) ((OTP_0_value >> 16) & 0x000003FF);        
    printk("isx006_OTP_get:AF_A_value = %d\n", AF_A_value);
    printk("isx006_OTP_get:AF_B_value = %d\n", AF_B_value);    

    AF_F_value = 8; //coarse search
    AF_C_value = (AF_B_value - AF_A_value) / (AF_F_value - 2) + 1;
    printk("isx006_OTP_get+: AF_C_value =%d \n", AF_C_value);
    /* Pre-White Balance -----------------------------------------*/  
    if(!vendor_id) //KMOT
    {
        I_NORMR = 0x10FB;    
        I_NORMB = 0x10AB;
        I_AWB_PRER = 0x013C;
        I_AWB_PREB = 0x0241;   
    }else
    {
        I_NORMR = 0x10BD;    
        I_NORMB = 0x0EE0;
        I_AWB_PRER = 0x0146;
        I_AWB_PREB = 0x0239;   
    }
    
    C_NR = (uint16_t) (((OTP_1_value << 6 ) & 0x000000C0) | ((OTP_0_value >> 26  ) & 0x0000003F));    
    C_NB = (uint16_t) ((OTP_1_value >> 2 ) & 0x000000FF);
    C_PR = (uint16_t) ((OTP_1_value >> 10 ) & 0x000000FF);
    C_PB = (uint16_t) ((OTP_1_value >> 18 ) & 0x000000FF);
    printk("isx006_OTP_get: C_NR = 0x%x\n", C_NR);    
    printk("isx006_OTP_get: C_NB = 0x%x\n", C_NB);
    printk("isx006_OTP_get: C_PR = 0x%x\n", C_PR);        
    printk("isx006_OTP_get: C_PB = 0x%x\n", C_PB);

    NORMR = (uint16_t) (I_NORMR * (128 + C_NR) /256);    
    NORMB = (uint16_t) (I_NORMB * (128 + C_NB) /256);
    AWB_PRER = (uint16_t) (I_AWB_PRER * (128 + C_PR) /256);    
    AWB_PREB = (uint16_t) (I_AWB_PREB * (128 + C_PB) /256);    
    printk("isx006_OTP_get: NORMR = 0x%x\n", NORMR);    
    printk("isx006_OTP_get: NORMB = 0x%x\n", NORMB);
    printk("isx006_OTP_get: AWB_PRER = 0x%x\n", AWB_PRER);        
    printk("isx006_OTP_get: AWB_PREB = 0x%x\n", AWB_PREB);    

    /* Lens Shading ----------------------------------------------*/
    Shading_index = (uint16_t) ((OTP_2_value >> 20 ) & 0x0000000F);		
    printk("isx006_OTP_get:Shading_index = %d\n", Shading_index);	

    return rc;    
}

int isx006_OTP_setting(struct msm_sensor_ctrl_t *s_ctrl) 
{
    int rc = 0;

	/*MTD-MM-SL-FixCoverity-00*{ */
    /* AF position -----------------------------------------------*/      
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x495E, AF_B_value - AF_C_value, MSM_CAMERA_I2C_WORD_DATA); //reduce knocking noise
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4960, AF_A_value , MSM_CAMERA_I2C_WORD_DATA);  
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4962, AF_A_value + AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
	if(rc < 0){
        pr_err("isx006_OTP_setting: write AF position fail\n");
		return rc;
	}
    
    /* Pre-White Balance -----------------------------------------*/    
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4A04, NORMR, MSM_CAMERA_I2C_WORD_DATA);
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4A06, NORMB, MSM_CAMERA_I2C_WORD_DATA);
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4A08, AWB_PRER, MSM_CAMERA_I2C_WORD_DATA);
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4A0A, AWB_PREB, MSM_CAMERA_I2C_WORD_DATA);
	if(rc < 0){
        pr_err("isx006_OTP_setting: write Pre-White Balance fail\n");
		return rc;
	}
	/*MTD-MM-SL-FixCoverity-00*} */

    /* Lens Shading ----------------------------------------------*/
	/* MTD-MM-SL-Add5M2ndSource-00+{ */
    if(!vendor_id) //KMOT
    {
        printk("isx006_OTP_setting:write shading table \n");	
        switch (Shading_index) {
        case SHD_MIN:
			rc = msm_camera_i2c_write_tbl_invert(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->shd_min_2nd_settings,
				s_ctrl->msm_sensor_reg->shd_min_2nd_size,
				s_ctrl->msm_sensor_reg->default_data_type);
        	break;	 
     
        case SHD_TYP:
			rc = msm_camera_i2c_write_tbl_invert(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->shd_typ_2nd_settings,
				s_ctrl->msm_sensor_reg->shd_typ_2nd_size,
				s_ctrl->msm_sensor_reg->default_data_type);			
        	break;
     
        case SHD_MAX:
			rc = msm_camera_i2c_write_tbl_invert(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->shd_max_2nd_settings,
				s_ctrl->msm_sensor_reg->shd_max_2nd_size,
				s_ctrl->msm_sensor_reg->default_data_type);					
        	break;
       }
    }
	/* MTD-MM-SL-Add5M2ndSource-00+} */

    if(rc < 0)
        pr_err("isx006_OTP_setting:write shading table fail\n");

    return rc;    
}

int32_t isx006_sensor_start_streaming(struct msm_sensor_ctrl_t * s_ctrl)
{
    int rc = 0;
	struct msm_camera_sensor_info *isx006_info = NULL;
	isx006_info = s_ctrl->sensordata;	
	
	printk("isx006_sensor_start_streaming: START !\n"); 
	
    /* Set NSTANDBY from L to H */
	rc =  gpio_request(isx006_info->sensor_pwd, "CAM_5M_STBYN");
	if (rc < 0) {
    	pr_err("isx006_sensor_start_streaming: gpio_request (%d) failed !.\n", isx006_info->sensor_pwd);
	}
    rc = gpio_direction_output(isx006_info->sensor_pwd, 1);
    if (rc < 0) {
        pr_err("isx006_sensor_start_streaming: Pull high CAM_5M_STBYN pin failed !\n");
        goto gpio_sensor_pwd_fail;
    }
	gpio_free(isx006_info->sensor_pwd);
    cam_msleep(15);//T4 duration(max) //10

    rc = isx006_check_om(s_ctrl->sensor_i2c_client, "For change to Active mode");
    if (rc < 0) {
        pr_err("isx006_sensor_start_streaming: isx006_check_om(For change to Active mode) failed !\n");
        goto error;
    }

    /* Here to check ststus is STS_ACTIVE */
    rc = isx006_polling_state_change(s_ctrl->sensor_i2c_client, STS_ACTIVE);
    if (rc < 0) {
        pr_err("isx006_sensor_start_streaming: isx006_polling_state_change(STS_ACTIVE) failed !\n");
        goto error;
    }

    rc = isx006_check_cm(s_ctrl->sensor_i2c_client, MONITOR_MODE);
    if (rc < 0) {
        pr_err("isx006_sensor_start_streaming: isx006_check_cm(For CM change to Monitor mode) failed !\n");
        goto error;
    }

	/* MTD-MM-SL-SupportAF-00+{ */ 
    rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x002E, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0012, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	/* MTD-MM-SL-SupportAF-00+} */
	
    printk("isx006_sensor_start_streaming: Start streaming success.\n");   
    return rc;
    

gpio_sensor_pwd_fail:
	gpio_free(isx006_info->sensor_pwd);
	
error:
    pr_err("isx006_sensor_start_streaming: Failed !\n"); 
    return rc;
}

/*MTD-MM-SL-ImproveMainCamera-00+{ */
int32_t isx006_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	//isx006 has no reg for stop stream, so by-pass it
	rc = msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->stop_stream_conf,
		s_ctrl->msm_sensor_reg->stop_stream_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
	if (rc < 0){
		printk("isx006_sensor_stop_stream: msm_camera_i2c_write_tbl failed, can by-pass it! \n");
	}

	printk("isx006_sensor_stop_stream: Stop streaming success.\n");   
	return 0;	
}
/*MTD-MM-SL-ImproveMainCamera-00+} */

/*MTD-MM-UW-fix CTS preview fail-00*{ */
/* MTD-MM-SL-Add5M2ndSource-02*{ */
/* MTD-MM-SL-Add5M2ndSource-00*{ */
int32_t isx006_sensor_init_setting(struct msm_sensor_ctrl_t *s_ctrl, int update_type, int res)
{
	int32_t rc;
	printk("isx006_sensor_init_setting, update_type = %d.\n", update_type);

	/* OTP read/save value */
	printk("isx006_sensor_init_setting: Start OTP read/save value. \n");
	rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x3206, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_sensor_init_setting: change to Little I2C mode fail!\n");
    }

	rc = isx006_OTP_get(s_ctrl->sensor_i2c_client);
    if (rc < 0) {
        pr_err("isx006_sensor_init_setting: OTP read/save value failed !\n");
    }
	
	/* <0>. Pre-Sleep mode ---------------------------- */  
	rc = isx006_check_om(s_ctrl->sensor_i2c_client, "For change to PreSleep mode");
    if (rc < 0) {
        pr_err("isx006_sensor_init_setting: isx006_check_om(For change to PreSleep mode) failed !\n");
        goto error;
    }

	/* Here to check ststus is STS_SLEEP */
	rc = isx006_polling_state_change(s_ctrl->sensor_i2c_client, STS_PRE_SLEEP);
    if (rc < 0) {
        pr_err("isx006_sensor_init_setting: isx006_polling_state_change(STS_PRE_SLEEP) failed !\n");
        goto error;
    }

	/* <0-1>. Write init setting */   
	rc = msm_sensor_write_all_conf_array_invert(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->init_settings,
		s_ctrl->msm_sensor_reg->init_size);
	if (rc < 0) {
        pr_err("isx006_sensor_init_setting: Write Period1 table failed !\n");
        goto error;
    }

	cam_msleep(1);
	/* <1>. Sleep mode ---------------------------- */  
	rc = isx006_check_om(s_ctrl->sensor_i2c_client, "For change to Sleep mode");
    if (rc < 0) {
        pr_err("isx006_sensor_init_setting: isx006_check_om(For change to Sleep mode) failed !\n");
        goto error;
    }

	/* Here to check ststus is STS_SLEEP */
    rc = isx006_polling_state_change(s_ctrl->sensor_i2c_client, STS_SLEEP);
    if (rc < 0) {
        pr_err("isx006_sensor_init_setting: isx006_polling_state_change(STS_SLEEP) failed !\n");
        goto error;
    }

	/* <1-1>. Write preload2 setting */
	if (!vendor_id){ //KMOT
		printk("isx006_sensor_init_setting: Sensor Period2 for 2nd source.\n");
		rc = msm_camera_i2c_write_tbl_invert(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->preload2_2nd_settings,
			s_ctrl->msm_sensor_reg->preload2_2nd_size,
			s_ctrl->msm_sensor_reg->default_data_type);
	}else {
		printk("isx006_sensor_init_setting: Sensor Period2 for main source.\n");
		rc = msm_camera_i2c_write_tbl_invert(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->preload2_settings,
			s_ctrl->msm_sensor_reg->preload2_size,
			s_ctrl->msm_sensor_reg->default_data_type);
	}
	if (rc < 0) {
        pr_err("isx006_sensor_init_setting: Write Period2 table failed !\n");
        goto error;
    }
    printk("isx006_sensor_init_setting: Sensor Period2 done.\n");
    cam_msleep(5);

	/* <1-2>. Write preload3 setting */
	if (!vendor_id){ //KMOT
		printk("isx006_sensor_init_setting: Sensor Period3 for 2nd source.\n");
		rc = msm_camera_i2c_write_tbl_invert(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->preload3_2nd_settings,
			s_ctrl->msm_sensor_reg->preload3_2nd_size,
			s_ctrl->msm_sensor_reg->default_data_type);
	}else {
		printk("isx006_sensor_init_setting: Sensor Period3 for main source.\n");
		rc = msm_camera_i2c_write_tbl_invert(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->preload3_settings,
			s_ctrl->msm_sensor_reg->preload3_size,
			s_ctrl->msm_sensor_reg->default_data_type);
	}
	if (rc < 0) {
        pr_err("isx006_sensor_init_setting: Write Period3 table failed !\n");
        goto error;
    }
    printk("isx006_sensor_init_setting: Sensor Period3 done.\n");
    cam_msleep(5); 

	/* <1-3>. Write OTP setting */  
	rc = isx006_OTP_setting(s_ctrl);
	if (rc < 0) {
		pr_err("isx006_sensor_init_setting: OTP read/save value failed !\n");
    }
	
    /* <1-4>. Write preload3-reload setting for 2nd source */ 
    //Move preload3 to isx006_sensor_mode_init, for 2nd source use after enter/exit standby to let AF could be workable	
    printk("isx006_init_sensor: Success.\n"); 	

    /* <1-5>. Cancel AF when executing capture command*/
    rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x4886, 0x01, MSM_CAMERA_I2C_BYTE_DATA);//CAP_AF_CANCEL_F
    if (rc < 0) {
        pr_err("isx006_init_sensor: Write CAP_AF_CANCEL_F failed !\n");
        goto error;
    }

    return rc;
	
error:
    pr_err("isx006_sensor_init_setting: Failed !\n");
    return rc;	
}
/* MTD-MM-SL-Add5M2ndSource-00*} */
/* MTD-MM-SL-Add5M2ndSource-02*} */
/*MTD-MM-UW-fix CTS preview fail-00*} */

/*MTD-MM-UW-AddAF-00*{ */
static struct regulator *regulator;
int isx006_regulator_enable(struct device *dev, const char *id, int voltage, const char *tag)
{
    int rc = 0;
    pr_err("%s: id =  %s\n", __func__, id);
	
	// Voting for Regulator 
	regulator = regulator_get(dev, id);
	if (IS_ERR(regulator)) {
	    rc = PTR_ERR(regulator);
	    pr_err("%s: could not get regulator: %d\n", __func__, rc);
	    goto out;
	}
	     
	// Set the voltage level 
	rc = regulator_set_voltage(regulator, voltage, voltage);
	if (rc < 0) {
	    pr_err("%s: could not set voltage: %d\n", __func__, rc);
	    goto reg_free;
	}
	    
	// Enabling the regulator 
	rc = regulator_enable(regulator);
	if (rc) {
	    pr_err("%s: could not enable regulator: %d\n", __func__, rc);
	    goto reg_free;
	}

	printk("%s: regulator_is_enabled(%s) for %s success.\n", __func__, id, tag);
	return rc;

reg_free:
    regulator_put(regulator);
    regulator = NULL;
out:
    pr_err("%s: For %s fail !\n", __func__, tag); 
    return rc;
}

int isx006_regulator_disable(struct device *dev, const char *id, const char *tag)
{
    int rc = 0;
    
    regulator_put(regulator);
    regulator = NULL;
		
	/* Voting for 1.8V Regulator */
	regulator = regulator_get(dev, id);
	if (IS_ERR(regulator)) {
	    rc = PTR_ERR(regulator);
	    pr_err("%s: could not get regulator: %d\n", __func__, rc);
	    goto out;
	}

	if (regulator_is_enabled(regulator) == 0)
	{
	    printk("%s: regulator(%s) is disable previously for %s.\n", __func__, id, tag);
	    goto exit;
	}
	    
	/* Releasing the 1.8V Regulator */
	if (!IS_ERR_OR_NULL(regulator)) {
	    rc = regulator_disable(regulator);
	    if (rc)
	    {
	        pr_err("%s: could not disable regulator: %d\n", __func__, rc);
	        regulator_put(regulator);
	        regulator = NULL;
	        goto out;
	    }
	}

	printk("%s: regulator_disable(%s) for %s success.\n", __func__, id, tag);

exit:
    regulator_put(regulator);
    regulator = NULL;
    return rc;
    
out:
    pr_err("%s: For %s fail !\n", __func__, tag); 
    return rc;
}

/*MTD-MM-SL-ImproveMainCamera-03+{ */
int32_t isx006_normal_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
	struct msm_camera_sensor_info *isx006_info = NULL;
	isx006_info = s_ctrl->sensordata;
	s_ctrl_m = (struct msm_sensor_ctrl_t *)s_ctrl;

	printk("isx006_normal_power_up: STARTUP = 0 \n");	   

	/* I/O & Analog power up */
	//EN_VREG_CAM_VDD_V1P2
	printk("isx006_normal_power_up: Pull high EN_VREG_CAM_VDD_V1P2 pin (%d) \n", isx006_info->vreg_v1p2);
	rc = gpio_request(isx006_info->vreg_v1p2, "EN_VREG_CAM_VDD_V1P2");
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: gpio_request (%d) failed !.\n", isx006_info->vreg_v1p2);
	}    
	rc = gpio_direction_output(isx006_info->vreg_v1p2, 1);
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: Pull high EN_VREG_CAM_VDD_V1P2 pin failed !\n");
       	goto gpio_v1p2_fail;
    }
	gpio_free(isx006_info->vreg_v1p2);

	//EN_VREG_CAM_VDD_V1P8
	printk("isx006_normal_power_up: Pull high EN_VREG_CAM_VDD_V1P8 pin (%d) \n", isx006_info->vreg_v1p8);
	rc = gpio_request(isx006_info->vreg_v1p8, "EN_VREG_CAM_VDD_V1P8");
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: gpio_request (%d) failed !.\n", isx006_info->vreg_v1p8);
	}
	rc = gpio_direction_output(isx006_info->vreg_v1p8, 1);
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: Pull high EN_VREG_CAM_VDD_V1P8 pin failed !\n");
       	goto gpio_v1p8_fail;
    }
	gpio_free(isx006_info->vreg_v1p8);

	//EN_VREG_CAM_VDD_V2P8
	printk("isx006_normal_power_up: Pull high EN_VREG_CAM_VDD_V2P8 pin (%d) \n", isx006_info->vreg_v2p8);
	rc = gpio_request(isx006_info->vreg_v2p8, "EN_VREG_CAM_VDD_V2P8");
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: gpio_request (%d) failed !.\n", isx006_info->vreg_v2p8);
	}
	rc = gpio_direction_output(isx006_info->vreg_v2p8, 1);
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: Pull high EN_VREG_CAM_VDD_V2P8 pin failed !\n");
       	goto gpio_v2p8_fail;
    }
	gpio_free(isx006_info->vreg_v2p8);

    cam_msleep(5);

	/* Clock supply */
	rc = fih_enable_mclk(s_ctrl);
    if (rc < 0) {
		pr_err("isx006_normal_power_up: Pull high CAN_MCLK pin failed !\n");
		goto error;
	}
	cam_msleep(10);

	/*front camera enter standby*/
	if (s_ctrl_f != NULL){
		rc = front_cam_enter_standby(s_ctrl_f);
		if (rc < 0){
			printk("isx006_normal_power_up: front_cam_enter_standby() failed ! If during probe, this could be bypass\n");
			//goto error;
		}
	}
		
	/* Set NREST from L to H */
	printk("isx006_normal_power_up: Pull high CAM_5M_RSTN pin (%d) \n", isx006_info->sensor_reset);
	rc =  gpio_request(isx006_info->sensor_reset, "CAM_5M_RSTN");
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: gpio_request (%d) failed !.\n", isx006_info->sensor_reset);
	} 
	rc = gpio_direction_output(isx006_info->sensor_reset, 1);
	if (rc < 0) {
       	pr_err("isx006_normal_power_up: Pull high CAM_5M_RSTN pin failed !\n");
       	goto gpio_direction_output_fail;
    }
	gpio_free(isx006_info->sensor_reset);
	cam_msleep(10);	
	

	printk("isx006_normal_power_up: Success.\n");

	return rc;	

gpio_v1p2_fail:
	gpio_free(isx006_info->vreg_v1p2);
	goto error;

gpio_v1p8_fail:
	gpio_free(isx006_info->vreg_v1p8);
	goto error;

gpio_v2p8_fail:
	gpio_free(isx006_info->vreg_v2p8);
	goto error;
	
gpio_direction_output_fail:
	gpio_free(isx006_info->sensor_reset);
	goto error;
	
error:
	STARTUP = 0;
	F_STARTUP = 0;
	pr_err("isx006_normal_power_up: failed !, rc = %d.\n", rc);
	return rc;

}

int32_t isx006_normal_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
	struct msm_camera_sensor_info *isx006_info = NULL;
	isx006_info = s_ctrl->sensordata;
	
	printk("isx006_normal_power_down: STARTUP = 0 \n"); 
		
	/* Set NSTANDBY from H to L */
	rc =  gpio_request(isx006_info->sensor_pwd, "CAM_5M_STBYN");
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: gpio_request (%d) failed !.\n", isx006_info->sensor_pwd);
	}
	rc = gpio_direction_output(isx006_info->sensor_pwd, 0);
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: Pull low CAM_5M_STBYN pin failed !\n");
       	goto gpio_sensor_pwd_fail;
    }
	gpio_free(isx006_info->sensor_pwd);
	
	/* Set NREST from H to L */ 
	rc =  gpio_request(isx006_info->sensor_reset, "CAM_5M_RSTN");
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: gpio_request (%d) failed !.\n", isx006_info->sensor_reset);
	}
	rc = gpio_direction_output(isx006_info->sensor_reset, 0);
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: Pull ow CAM_5M_RSTN pin failed !\n");
       	goto gpio_sensor_reset_fail;
    }
	gpio_free(isx006_info->sensor_reset);

	cam_msleep(5);

	/* Clock disable */
	rc = fih_disable_mclk(s_ctrl);
       if (rc < 0)
           goto error;

    cam_msleep(5);
		
	/* I/O & Analog power down */
	//EN_VREG_CAM_VDD_V2P8
	rc =  gpio_request(isx006_info->vreg_v2p8, "EN_VREG_CAM_VDD_V2P8");
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: gpio_request (%d) failed !.\n", isx006_info->vreg_v2p8);
	}
	rc = gpio_direction_output(isx006_info->vreg_v2p8, 0);
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: Pull low EN_VREG_CAM_VDD_V1P2 pin failed !\n");
       	goto gpio_v2p8_fail;
    }
	gpio_free(isx006_info->vreg_v2p8);

	//EN_VREG_CAM_VDD_V1P8
	rc =  gpio_request(isx006_info->vreg_v1p8, "EN_VREG_CAM_VDD_V1P8");
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: gpio_request (%d) failed !.\n", isx006_info->vreg_v1p8);
	}
	rc = gpio_direction_output(isx006_info->vreg_v1p8, 0);
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: Pull low EN_VREG_CAM_VDD_V1P8 pin failed !\n");
       	goto gpio_v1p8_fail;
    }
	gpio_free(isx006_info->vreg_v1p8);

	//EN_VREG_CAM_VDD_V1P2
	rc =  gpio_request(isx006_info->vreg_v1p2, "EN_VREG_CAM_VDD_V1P2");
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: gpio_request (%d) failed !.\n", isx006_info->vreg_v1p2);
	}
	rc = gpio_direction_output(isx006_info->vreg_v1p2, 0);
	if (rc < 0) {
       	pr_err("isx006_normal_power_down: Pull low EN_VREG_CAM_VDD_V1P2 pin failed !\n");
       	goto gpio_v1p2_fail;
    }
	gpio_free(isx006_info->vreg_v1p2);
	
	printk("isx006_normal_power_down: End. \n");  
	return rc;

gpio_sensor_pwd_fail:
	gpio_free(isx006_info->sensor_pwd);
	goto error;
	
gpio_sensor_reset_fail:
	gpio_free(isx006_info->sensor_reset);
	goto error;

gpio_v1p2_fail:
	gpio_free(isx006_info->vreg_v1p2);
	goto error;

gpio_v1p8_fail:
	gpio_free(isx006_info->vreg_v1p8);
	goto error;

gpio_v2p8_fail:
	gpio_free(isx006_info->vreg_v2p8);
	goto error;

error:
    pr_err("isx006_normal_power_down: failed ! rc = %d.\n", rc);
	STARTUP = 0;
	F_STARTUP = 0;
	return rc;	
}

/*MTD-MM-SL-ImproveMainCamera-04*{ */
int32_t isx006_csi_config_for_reset(struct msm_sensor_ctrl_t *s_ctrl, int res)
{
	int rc = 0;

	printk("isx006_csi_config_for_reset.\n"); 

	//Follow sensor_csi_setting when !csi_config & STARTUP=0
	msm_sensor_write_conf_array(
	s_ctrl->sensor_i2c_client,
	s_ctrl->msm_sensor_reg->mode_settings, res);
	
	s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
	CDBG("CSI config in progress\n");
	v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
		NOTIFY_CSIC_CFG,
		s_ctrl->curr_csic_params);
	CDBG("CSI config is done\n");		
	mb();
	cam_msleep(30);
	csi_config = 1;
	
	v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
	NOTIFY_PCLK_CHANGE,
	&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);

	rc = s_ctrl->func_tbl->sensor_start_stream_1(s_ctrl);
	if (rc < 0){
		pr_err("isx006_csi_config_for_reset: sensor_start_stream failed!\n"); 
		return rc;
	}
	
	cam_msleep(30);

	return rc;
}

int32_t isx006_power_up_reset(struct msm_sensor_ctrl_t *s_ctrl, int mode)
{
	int rc = 0;	
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

	printk("isx006_power_up_reset: Start.\n");  

	switch (mode){
	case 0:
		printk("isx006_power_up_reset: case 0 \n"); 

		//01. Re-set flag variables.
		F_STARTUP = 0;
		STARTUP = 0;

		//03. Power off
		rc = isx006_normal_power_down(s_ctrl);
		if (rc < 0) {
			printk("isx006_power_up_reset: sensor_power_down fail \n"); 
		}

		cam_msleep(30);

		//03. Power on 
		rc = isx006_normal_power_up(s_ctrl);
		if (rc < 0) {
			printk("isx006_power_up_reset: sensor_power_up fail \n"); 
		}

		break;

	case 1:	
		printk("isx006_power_up_reset: case 1 \n"); 
		
		//01. Re-set flag variables.
		STARTUP = 0;
		F_STARTUP = 0;

		//02. Re-config front camera pin (CAM_VGA_STBY).
		rc =  gpio_request(data->sensor_f_pwd, "CAM_VGA_STBY");
		if (rc < 0) {
			pr_err("isx006_power_up_reset: gpio_request (%d) failed !.\n", data->sensor_f_pwd);
		}
		rc = gpio_direction_output(data->sensor_f_pwd, 0);
		if (rc < 0) {
	       	pr_err("isx006_power_up_reset: Pull low CAM_VGA_STBY pin failed !\n");
			goto gpio_sensor_f_pwd_fail;
	    }
		gpio_free(data->sensor_f_pwd);

		//03. Power off.
		rc = isx006_normal_power_down(s_ctrl);
		if (rc < 0) {
	       	pr_err("isx006_power_up_reset: sensor_power_down failed !\n");
		}
		
		cam_msleep(200);//Wait power off done.

		//04. Power on.
		rc = isx006_normal_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("isx006_power_up_reset: sensor_power_up failed !\n");
		}

		//05. init setting.
		rc = s_ctrl->func_tbl->sensor_init_setting(s_ctrl, MSM_SENSOR_REG_INIT,0);
		if (rc < 0) {
			pr_err("isx006_power_up_reset: init setting failed !\n");
		}
		csi_config = 0;

		//06. Config CSI
		rc = isx006_csi_config_for_reset(s_ctrl, RES_PREVIEW);
		if (rc < 0) {
			pr_err("isx006_power_up_reset: csi config failed !\n");
		}

		break;

	}
	printk("isx006_power_up_reset: End.\n"); 
	return rc;	

gpio_sensor_f_pwd_fail:
	gpio_free(data->sensor_f_pwd);
	
	return rc;
}
/*MTD-MM-SL-ImproveMainCamera-04*} */

int32_t isx006_power_down_reset(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;	

	printk("isx006_power_down_reset: Start.\n");  
	//01. Re-set flag variables.
	STARTUP = 0;
	F_STARTUP = 0;

	//02. Power off.
	rc = isx006_normal_power_down(s_ctrl);
	if (rc < 0) {
       	pr_err("isx006_power_down_reset: sensor_power_down failed !\n");
	}

	printk("isx006_power_down_reset: End.\n"); 
	return rc;	
}
/*MTD-MM-SL-ImproveMainCamera-03+} */

/*MTD-MM-SL-ImproveMainCamera-03*{ */
/*MTD-MM-SL-ImproveMainCamera-02*{ */
/*MTD-MM-SL-CantSleepInSuspend-00*{ */
/*MTD-MM-SL-ImproveMainCamera-00+{ */
int32_t isx006_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
	s_ctrl_m = (struct msm_sensor_ctrl_t *)s_ctrl;
	
    printk("isx006_sensor_power_up: Start.\n");  

	if (STARTUP == 1){
		
		printk("isx006_sensor_power_up: STARTUP = 1 \n");

		rc = isx006_exit_standby(s_ctrl, ACT_MCLK_ON);
        if (rc < 0){
			printk("isx006_sensor_power_up: isx006_exit_standby fail, need reset \n");
            rc = isx006_power_up_reset(s_ctrl, 0);
			if (rc <0){
				pr_err("isx006_sensor_power_up: isx006_power_up_reset fail! \n");
				goto error;
			}
		}
		
		printk("isx006_sensor_power_up: isx006_exit_standby Success! \n");
		goto done;	
	} 

	rc = isx006_normal_power_up(s_ctrl);
	if (rc < 0) {
       	pr_err("isx006_sensor_power_up: isx006_normal_power_up failed !\n");
       	goto error;
    }
	goto done;

done:
	/* AF_power on */
	rc = isx006_regulator_enable(&s_ctrl->sensor_i2c_client->client->dev ,"bt", 3000000, "isx006_AF");
    if (rc < 0) {
		pr_err("isx006_sensor_power_up: AF power enable failed !\n");
		goto error;
    }	
	printk("isx006_sensor_power_up: Success.\n");
	return rc;

error:
	STARTUP = 0;
	F_STARTUP = 0;
	pr_err("isx006_sensor_power_up: failed !, rc = %d.\n", rc);
	return rc;
}
/*MTD-MM-SL-CantSleepInSuspend-00*} */
/*MTD-MM-SL-ImproveMainCamera-02*} */

/*MTD-MM-SL-ImproveMainCamera-01*{ */
int32_t isx006_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
	
    printk("isx006_sensor_power_down: Start.\n");   

	if(STARTUP == 1){
		printk("isx006_sensor_power_down: STARTUP = 1 \n");   
		rc = isx006_enter_standby(s_ctrl);
		if (rc < 0){
			printk("isx006_sensor_power_down: isx006_enter_standby fail, need re-power off. \n");
            rc = isx006_power_down_reset(s_ctrl);
			if (rc <0){
				pr_err("isx006_sensor_power_down: isx006_power_down_reset fail! \n");
				goto error;
			}
		}
		
		printk("isx006_sensor_power_down: isx006_enter_standby Success! \n");
		goto done;
	}

	rc = isx006_normal_power_down(s_ctrl);
	if (rc < 0) {
       	pr_err("isx006_sensor_power_down: isx006_normal_power_up failed !\n");
       	goto error;
    }
	goto done;

done:
	/* AF_power off */
	rc = isx006_regulator_disable(&s_ctrl->sensor_i2c_client->client->dev ,"bt", "isx006_AF");
    if (rc < 0) {
       	pr_err("isx006_sensor_power_down: AF power enable failed !\n");
		goto error;
    }
	
	AF_skip = false;
	csi_config = 0;
	
	printk("isx006_sensor_power_down: End. \n");  
	return rc;

error:
    pr_err("isx006_sensor_power_down: failed ! rc = %d.\n", rc);
	/* AF_power off */
	rc = isx006_regulator_disable(&s_ctrl->sensor_i2c_client->client->dev ,"bt", "isx006_AF");
	STARTUP = 0;
	F_STARTUP = 0;
	return rc;
}
/*MTD-MM-SL-ImproveMainCamera-00*} */
/*MTD-MM-SL-ImproveMainCamera-03*} */

/*MTD-MM-SL-QuicklyUnlockCameraFail-00*{ */
/*MTD-MM-SL-ImproveMainCamera-02*{ */
/*MTD-MM-SL-ImproveMainCamera-00+{ */
int isx006_suspend(struct i2c_client *client, pm_message_t state)
{
    int rc = 0;
    
    printk("isx006_suspend \n");

    rc = fih_enable_mclk(s_ctrl_m);
    if (rc < 0) {
        goto error;
    }
	
    cam_msleep(5);

    STARTUP  = 0;
    AF_full_range = 0;
    
    rc = isx006_sensor_power_down(s_ctrl_m);
    if (rc < 0)
        goto error;

	return 0;
    
error:
    pr_err("isx006_suspend: failed ! rc = %d.\n", rc);
	return rc;
}

int isx006_resume(struct i2c_client *client)
{
     printk("isx006_resume \n");  
	 
     return 0;
}
/*MTD-MM-SL-ImproveMainCamera-00+} */
/*MTD-MM-SL-ImproveMainCamera-01*} */
/*MTD-MM-SL-ImproveMainCamera-02*} */
/*MTD-MM-SL-QuicklyUnlockCameraFail-00*} */

/* MTD-MM-UW-AF_tune-00+ */
/* MTD-MM-SL-SupportFlash-02*{ */
/* MTD-MM-SL-AddForSoMCScene-00*{ */
/* MTD-MM-SL-SupportAF-00*{ */
static int isx006_check_AF(struct msm_camera_i2c_client *client, const char *tag)
{
    int rc = 0;
    uint16_t irq_status = 0x0;
    uint16_t v_read = 0x0;	
    uint16_t v_temp = 0x0;		

    //printk("isx006_check_AF: enter !\n");
    
	/* Make sure camera mode is changed */
	rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("isx006_check_AF: Read REG(0x00F8) failed !\n");
		goto error;
	} 
	//printk("isx006_check_AF: Read REG(0x00F8) = 0x%x !\n", irq_status);
	// Check CM_CHANGED STS (bit1)
	if (!(irq_status & 0x10)){
		//pr_err("isx006_check_AF: Camera mode not changed!\n");

		if(isx006_scene == MSM_V4L2_SCENE_NIGHT || isx006_scene == MSM_V4L2_SCENE_NIGHT_PORTRAIT) /*MTD-MM-SL-AddForSoMCScene-01* */
                    cam_msleep(30);
		else if(!AF_type)
			cam_msleep(10); 
		//else
			//printk("isx006_check_cm: No delay time !\n");
		return -EIO;
	}
	
    printk("isx006_check_AF: Clear interrupt status !\n");
    /* Clear interrupt status */
    //1. Clear interrupt status
    rc = msm_camera_i2c_read(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
        pr_err("isx006_check_AF: Read REG(0x00FC) failed !\n");
        goto error;
    }
	
    v_temp = v_read | 0x10;	
    rc = msm_camera_i2c_write(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);	
    if (rc < 0) {
        pr_err("isx006_check_AF: Write REG(0x00FC) failed !\n");
        goto error;
    }
	
	//2.Check interrupt status is cleaned
	rc = msm_camera_i2c_read(client, 0x00F8, &irq_status, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("isx006_check_AF: Read REG(0x00F8) failed !\n");
		goto error;
	}
	printk("isx006_check_AF: Read REG(0x00F8),rc = %d !\n", rc);
	printk("isx006_check_AF: Read REG(0x00F8) = 0x%x !\n", irq_status);
	// Check MO_CHANGED STS (bit5)
	if (!(irq_status & 0x10)){
		// [bit5] == 0
		printk("isx006_check_AF: CM_CHANGED STS is clear. \n");
		printk("isx006_check_AF: Success\n");
		return 0;
	} else {
		pr_err("isx006_check_AF: clear interrupt status failed !\n");
		cam_msleep(10);
		return -EIO;
	}

error:
    printk("isx006_check_AF: Check AF error\n");
    return rc;
}
/* MTD-MM-SL-AddForSoMCScene-00*} */
/* MTD-MM-SL-SupportFlash-02*} */

static int isx006_get_AF_state(struct msm_camera_i2c_client *client, const char *tag)
{
    int i = 0;
    int rc = 0;
    int led_mode = 0;
    uint16_t v_read = 0x0;    
    uint16_t v_temp = 0x0;        
    uint16_t AF_result = 0xFF;
    uint16_t AF_state = 0xFF;
    bool recover_AE = false;

	//printk("isx006_get_AF_state: enter\n");

	led_mode = msm_soc_get_led_mode();
	//printk("isx006_get_AF_state: led_mode = %d !\n", led_mode);  

	rc = msm_camera_i2c_read(client, 0x6D76, &AF_result, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("isx006_get_AF_state: msm_camera_i2c_read(0x6D76) failed !\n");
		goto error;
	} 
	//printk("isx006_get_AF_state: msm_camera_i2c_read(0x6D76) AF satate = %d\n", AF_result);
	
	if( led_mode == LED_MODE_RED_EYE || led_mode == LED_MODE_AUTO){  
		if(torch_enable)
			recover_AE = true;
	}
	
	if( led_mode == LED_MODE_ON || recover_AE == true){
		/* 05.Turn off pre-flash------------------------------------------------------------*/
		printk("isx006_get_AF_state: flash off!\n");
		rc = msm_soc_torch_flash_off();
		if (rc < 0)
			pr_err("isx006_get_AF_state: msm_soc_torch_flash_off() failed !\n");
		
		//cam_msleep(50); 
	}

	/* 01.Check the AF result register -------------------------------------------------------*/
	rc = msm_camera_i2c_read(client, 0x6D77, &AF_result, MSM_CAMERA_I2C_BYTE_DATA);//ADD_AF_RESULT = 0x6D77
	if (rc < 0) {
		pr_err("isx006_get_AF_state: msm_camera_i2c_read(0x%x) failed !\n", 0x6D77);
		goto error;
	}
    
        if(AF_result == AF_OK){
            printk("isx006_get_AF_state: AF result is AF OK!!!!\n");
            rc =  0;
        }
        else if(AF_result == AF_NG){    
            printk("isx006_get_AF_state: AF result is AF NG\n");
            rc =  -EIO;
        }
        else if(AF_result == AF_During){    
            rc = msm_camera_i2c_read(client, 0x4885, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0) {
				pr_err("isx006_get_AF_state: msm_camera_i2c_read(0x4885) failed !\n");
				goto error;
			}
			
            v_temp = v_read | 0x01;    
            rc = msm_camera_i2c_write(client, 0x4885, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0) {
				pr_err("isx006_get_AF_state: msm_camera_i2c_write(0x4885) failed !\n");
				goto error;
			}

            for (i = 0; i < 30; i++){
                rc = msm_camera_i2c_read(client, 0x6D76, &AF_state, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0) {
                    pr_err("isx006_get_AF_state: msm_camera_i2c_read(0x6D76) failed !\n");
                    goto error;
                }
                if (AF_state == AF_SAF_Idle || AF_state == AF_Idle) //SAF_IDLE
                    break;
                cam_msleep(10);    
			} 
            printk("isx006_get_AF_state: AF result is during AF\n");
            rc = -EIO;
        }

        return rc;
    
    error:
        return rc;
}
/* MTD-MM-SL-SupportAF-00*} */
/* MTD-MM-UW-AF_tune-00- */

/*MTD-MM-UW-set AF mode-00*{ */
static int isx006_MF_position(struct msm_camera_i2c_client *client, int position)
{
        int rc = -EBADF;
        uint16_t v_read = 0x00;
        int i;

        printk ("isx006_MF_position: Start.\n");
        rc = msm_camera_i2c_write_invert(client, 0x4876, 0, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE1
        if (rc < 0)
            goto error;

        //0. Set MF position to move
        rc = msm_camera_i2c_write_invert(client, 0x4852, position, MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0)
            goto error;

        //1. Trigger MF  
        rc = msm_camera_i2c_write_invert(client, 0x4850, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0)
            goto error;

        for (i = 0; i < 120; i++)
        {
          /*MTD-MM-SL-FixCoverity-00*{ */
          rc = msm_camera_i2c_read_invert(client, 0x6D7A, &v_read, MSM_CAMERA_I2C_WORD_DATA);
		  if (rc < 0)
            goto error;
		  /*MTD-MM-SL-FixCoverity-00*} */
          if(v_read == position)
            break;
          cam_msleep(5);    
        }
        printk ("isx006_check_MF: loop = %d, v_read =%d \n", i, v_read);
        rc = msm_camera_i2c_write_invert(client, 0x4876, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE1
        if (rc < 0)
            goto error;

        printk ("isx006_MF_position: End.\n");  
        return rc;

error:
    return rc;
}

/*MTD-MM-SL-FixCoverity-01*{ */
/*MTD-MM-SL-AddForSoMCScene-00*{ */
/* MTD-MM-SL-Add5M2ndSource-01*{ */
int isx006_set_AF_Range(struct msm_camera_i2c_client *client, int af_mode) 
{
	int rc = 0;
	printk("isx006_set_AF_Range: AF_mode =%d \n", af_mode);
		
	switch (af_mode) {
	case AF_MODE_AUTO:
	case AF_MODE_NORMAL:		
	case AF_MODE_CAF:	 
		if(AF_full_range)
			return rc;		
		printk("isx006_set_AF_Range: AF_mode - Full range\n");
		AF_F_value = 8; //coarse search
		AF_G_value = 1023; //fullscale forDAC
		AF_H_value = 32/AF_F_value ; //fullscale forDAC
		AF_C_value = (AF_B_value - AF_A_value) / (AF_F_value - 2) + 1;
	  
		rc = msm_camera_i2c_write_invert(client, 0x486C, AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			goto error;
	
		rc = msm_camera_i2c_write_invert(client, 0x4870, AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			goto error;
	
		AF_D_value = (AF_C_value / AF_H_value); 
	
		rc = msm_camera_i2c_write_invert(client, 0x486E, AF_D_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			goto error;
	
		rc = msm_camera_i2c_write_invert(client, 0x4872, AF_D_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			goto error;
			
		AF_E_value = AF_A_value + (AF_C_value * 8);
	
		/* full */
		if(vendor_id) //SEMCO
	   {
			rc = msm_camera_i2c_write_invert(client, 0x4876, AF_A_value + 30, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE1
			if (rc < 0)
				goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4878, AF_B_value + (AF_C_value), MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_HIGH_TYPE1 /* MTD-MM-SL-SupportAF-01* */ 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4844, AF_D_value * 4, MSM_CAMERA_I2C_WORD_DATA); 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x486A, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4822, AF_D_value / 4, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4824, AF_D_value*3 /4 , MSM_CAMERA_I2C_WORD_DATA); 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4838, AF_D_value * 2, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
			   goto error;
		}	 
		else
		{
			rc = msm_camera_i2c_write_invert(client, 0x4876, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE1
			if (rc < 0)
				goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4878, AF_B_value + (AF_C_value * 2), MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_HIGH_TYPE1
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4844, AF_C_value * 2, MSM_CAMERA_I2C_WORD_DATA); 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x486A, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4822, AF_D_value / 2, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4824, AF_D_value*4 /3 , MSM_CAMERA_I2C_WORD_DATA); 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4838, AF_D_value * 3, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
			   goto error;
		}
			
		AF_full_range = 1;
			
		break;		  
	
	case AF_MODE_MACRO:
		printk("isx006_set_AF_Range: AF_mode - Marco range\n");
		AF_F_value = 16; //coarse search
		AF_G_value = 1023; //fullscale forDAC
		AF_H_value = 32/AF_F_value ; //fullscale forDAC
		AF_C_value = (AF_B_value - AF_A_value) / (AF_F_value - 2) + 1;
	
		rc = msm_camera_i2c_write_invert(client, 0x486C, AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		rc = msm_camera_i2c_write_invert(client, 0x4870, AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		AF_D_value = (AF_C_value / AF_H_value);
		
		rc = msm_camera_i2c_write_invert(client, 0x486E, AF_D_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		rc = msm_camera_i2c_write_invert(client, 0x4872, AF_D_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		AF_E_value = AF_A_value + (AF_C_value * 8);
	
		/* Macro */
		if(vendor_id) //SEMCO
		{
			rc = msm_camera_i2c_write_invert(client, 0x487A, AF_A_value + (AF_C_value * 5), MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE2	 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x487C, AF_B_value + (AF_C_value * 2), MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_HIGH_TYPE2	 
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x4844, AF_C_value * 2, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0)
				goto error;
				
		}else
		{
			rc = msm_camera_i2c_write_invert(client, 0x487A, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE2
			if (rc < 0)
			   goto error;
	
			rc = msm_camera_i2c_write_invert(client, 0x487C, AF_B_value + (AF_C_value * 2), MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_HIGH_TYPE2	
			if (rc < 0)
			   goto error;
	
			 rc = msm_camera_i2c_write_invert(client, 0x4844, AF_C_value * 4, MSM_CAMERA_I2C_WORD_DATA);
			 if (rc < 0)
				goto error;
		}
	
		rc = msm_camera_i2c_write_invert(client, 0x486A, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);    
		if (rc < 0)
		   goto error;
		AF_full_range = 0;
			
		break;
	
	case AF_MODE_INFINITY:
		printk("isx006_set_AF_Range: AF_mode - Infinity range\n");
		AF_F_value = 16; //coarse search
		AF_G_value = 1023; //fullscale forDAC
		AF_H_value = 32/AF_F_value ; //fullscale forDAC
		AF_C_value = (AF_B_value - AF_A_value) / (AF_F_value - 2) + 1;
	
		rc = msm_camera_i2c_write_invert(client, 0x486C, AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		rc = msm_camera_i2c_write_invert(client, 0x4870, AF_C_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		AF_D_value = (AF_C_value / AF_H_value);
		
		rc = msm_camera_i2c_write_invert(client, 0x486E, AF_D_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		rc = msm_camera_i2c_write_invert(client, 0x4872, AF_D_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
		   goto error;
	
		AF_E_value = AF_A_value + (AF_C_value * 8);
	
		/* INFINITY */
	   if(vendor_id) //SEMCO
	   {
		  rc = msm_camera_i2c_write_invert(client, 0x487E, AF_A_value, MSM_CAMERA_I2C_WORD_DATA); //AF_AREA_LOW_TYPE3
		  if (rc < 0)
			   goto error;
		   
		   rc = msm_camera_i2c_write_invert(client, 0x4880, AF_A_value + (AF_C_value * 7), MSM_CAMERA_I2C_WORD_DATA); //AF_AREA_HIGH_TYPE3
		   if (rc < 0)
			  goto error;
	
		   rc = msm_camera_i2c_write_invert(client, 0x4844, AF_C_value * 3, MSM_CAMERA_I2C_WORD_DATA);
		   if (rc < 0)
			  goto error;
	   }
	   else
	   {
		  rc = msm_camera_i2c_write_invert(client, 0x487E, AF_A_value, MSM_CAMERA_I2C_WORD_DATA); //AF_AREA_LOW_TYPE3
		  if (rc < 0)
			   goto error;
		   
		   rc = msm_camera_i2c_write_invert(client, 0x4880, AF_B_value, MSM_CAMERA_I2C_WORD_DATA); //AF_AREA_HIGH_TYPE3
		   if (rc < 0)
			  goto error;
	
		   rc = msm_camera_i2c_write_invert(client, 0x4844, AF_C_value * 4, MSM_CAMERA_I2C_WORD_DATA);
		   if (rc < 0)
			  goto error;
	   }
	
	
		rc = msm_camera_i2c_write_invert(client, 0x486A, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);    
		if (rc < 0)
		   goto error;
		AF_full_range = 0;
			
		break;
	
	default:
		printk("isx006_set_AF_Range: ERR: Mode = %d, This is invalid !\n", af_mode);
		return -EINVAL;
	}
	
	/*printk("isx006_set_AF_Range: AF_A_value =%d \n", AF_A_value);
	printk("isx006_set_AF_Range: AF_B_value =%d \n", AF_B_value);
	printk("isx006_set_AF_Range: AF_C_value =%d \n", AF_C_value);
	printk("isx006_set_AF_Range: AF_D_value =%d \n", AF_D_value);
	printk("isx006_set_AF_Range: AF_E_value =%d \n", AF_E_value);*/
	
	if(isx006_scene == MSM_V4L2_SCENE_AUTO)
	{
		printk("isx006_set_AF_Range: AF_mode - XXXXXXXXXXXX\n");
		switch (af_mode) {
		case AF_MODE_AUTO:	  
		case AF_MODE_CAF:  
		case AF_MODE_NORMAL:	
			rc = msm_camera_i2c_write_invert(client, 0x01D3, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
			   goto error;
			break;
	
		case AF_MODE_MACRO:
			rc = msm_camera_i2c_write_invert(client, 0x01D3, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
			   goto error;
			break;
				
		case AF_MODE_INFINITY:
			rc = msm_camera_i2c_write_invert(client, 0x01D3, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
			   goto error;
			break;
	
		}
	}
	printk("isx006_set_AF_Range: ----------X \n");
	return rc;
	
	error:
		return rc;

}
/*MTD-MM-UW-set AF mode-00*} */
/* MTD-MM-SL-Add5M2ndSource-01*} */
/*MTD-MM-SL-AddForSoMCScene-00*} */
/*MTD-MM-SL-FixCoverity-01*} */

/*MTD-MM-UW-fix CTS preview fail-00*{ */
int isx006_set_monitor_af_mode(struct msm_camera_i2c_client *client, enum isx006_moni_af_mode new_mode, const char *tag)
{
    int rc = 0;
    uint16_t v_read = 0x0;    
    uint16_t v_temp = 0x0;       
    pr_err("isx006_set_monitor_af_mode: enter !\n");
    //0. Clear interrupt status
    rc = msm_camera_i2c_read(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_monitor_af_mode: Read REG(0x%x) failed !\n", 0x00FC);
        goto error;
    }
    v_temp = v_read | 0x10;
    
    rc = msm_camera_i2c_write(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_monitor_af_mode: Write REG(0x%x) failed !\n", 0x00FC);
        goto error;
    }

    if(new_mode ==MONI_AF_RESTART)
    {
    	    //1. Re-start AF
	    rc = msm_camera_i2c_write(client, 0x0015, 1, MSM_CAMERA_I2C_BYTE_DATA);
	    if (rc < 0) {
	        pr_err("isx006_set_monitor_af_mode:   Re-start AF  !!!\n");
	        goto error;
	    }
    }
    else
    {
        //1. Set monitor AF mode
        printk("isx006_set_monitor_af_mode: pre set MONI AF mode to %s\n", tag);
        rc = msm_camera_i2c_write(client, 0x002E, new_mode, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_set_monitor_af_mode: isx006_i2c_write_parallel(0x002E) failed !\n");
            goto error;
        }
    }

    //2. MONI_REFRESH
    rc = msm_camera_i2c_write(client, 0x0012, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_monitor_af_mode: isx006_i2c_write_parallel(0x0012) failed for refresh !\n");
        goto error;
    }

error:
    return rc;
}
/*MTD-MM-UW-AddAF-00*} */
/*MTD-MM-UW-fix CTS preview fail-00*} */

/*MTD-MM-SL-AddForSoMCScene-00+{ */
/*MTD-MM-SL-AddWB-00+{ */
int isx006_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;

	/*MTD-MM-SL-AddForSoMCScene-01*{ */
    //FIH-SW-MM-MC-ImplementCameraSceneModeforIsx006-00+{
    if (ctrl_info->ctrl_id == V4L2_CID_BESTSHOT_MODE)
    {
        printk("isx006_msm_sensor_s_ctrl_by_enum: V4L2_CID_BESTSHOT_MODE, value = %d\n", value);
        if (value == MSM_V4L2_SCENE_OFF || value == MSM_V4L2_SCENE_SUNSET ||
            value == MSM_V4L2_SCENE_BACKLIGHT || value == MSM_V4L2_SCENE_ANTISHAKE || 
            value == MSM_V4L2_SCENE_FLOWERS || value == MSM_V4L2_SCENE_CANDLELIGHT || 
            value == MSM_V4L2_SCENE_FIREWORKS || value == MSM_V4L2_SCENE_PARTY ||
            value == MSM_V4L2_SCENE_THEATRE || value == MSM_V4L2_SCENE_ACTION ||
            value == MSM_V4L2_SCENE_AR || value == MSM_V4L2_SCENE_MAX)
        {
            printk("isx006_msm_sensor_s_ctrl_by_enum: The Scene mode is not support, change to MSM_V4L2_SCENE_AUTO ~\n");
            value = MSM_V4L2_SCENE_AUTO;
        }

		switch (value) {
			case MSM_V4L2_SCENE_AUTO:
			case MSM_V4L2_SCENE_SPORTS:
			case MSM_V4L2_SCENE_SNOW: 
    		case MSM_V4L2_SCENE_BEACH:
			case MSM_V4L2_SCENE_PORTRAIT:
			case MSM_V4L2_SCENE_NIGHT_PORTRAIT:
        	printk ("isx006_msm_sensor_s_ctrl_by_enum: SCENE_AUTO/SPORTS/SNOW/BEACH \n");
        	rc = isx006_set_AF_Range(s_ctrl->sensor_i2c_client, AF_MODE_AUTO);
        	if(rc < 0)
            	goto error;

			break;

		case MSM_V4L2_SCENE_NIGHT:
		case MSM_V4L2_SCENE_LANDSCAPE:
        	printk ("isx006_msm_sensor_s_ctrl_by_enum: SCENE_NIGHT/LANDSCAPE \n");
        	rc = isx006_set_AF_Range(s_ctrl->sensor_i2c_client, AF_MODE_INFINITY);
        	if(rc < 0)
            	goto error;

			break;

		case MSM_V4L2_SCENE_DOCUMENT:
        	printk ("isx006_msm_sensor_s_ctrl_by_enum: SCENE_DOCUMENT\n");
        	rc = isx006_set_AF_Range(s_ctrl->sensor_i2c_client, AF_MODE_MACRO);
        	if(rc < 0)
            	goto error;

			break;

		default:
        	printk ("isx006_msm_sensor_s_ctrl_by_enum: set_scene: wrong value setting\n");
        	return -EINVAL;	
		}		
    }
    //FIH-SW-MM-MC-ImplementCameraSceneModeforIsx006-00+}
    /*MTD-MM-SL-AddForSoMCScene-01*} */

	rc = msm_sensor_write_enum_conf_array(
		s_ctrl->sensor_i2c_client,
		ctrl_info->enum_cfg_settings, value);
	if (rc < 0) {
		pr_err("isx006_msm_sensor_s_ctrl_by_enum: write faield\n");
		return rc;
	}
	isx006_scene = value;
	return rc;

error:
    printk ("isx006_msm_sensor_s_ctrl_by_enum: set_scene Failed !\n");
    return rc;	
}
/*MTD-MM-SL-AddForSoMCScene-00+} */

//FIH-SW-MM-MC-ImplementCameraMeteringforIsx006-00+{
int isx006_set_meter_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
    uint16_t v_read = 0xFF;
    uint16_t v_temp = 0xFF;

    rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x0104, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s read fail\n", __func__);
        return rc;
    }
    v_temp = v_read & 0xFC;

    printk("isx006_set_meter_s_ctrl_by_enum: value = %d, v_temp = 0x%x, v_read = 0x%x\n", value, v_temp , v_read);
    
    switch (value) {
    case MSM_V4L2_EXP_CENTER_WEIGHTED:
        v_temp = v_temp |0x01;    
        rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0104, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
        printk ("isx006_set_meter_s_ctrl_by_enum: Center\n");
        break;
    case MSM_V4L2_EXP_FRAME_AVERAGE:
        v_temp = v_temp |0x00;   
        rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0104, v_temp, MSM_CAMERA_I2C_BYTE_DATA);            
        printk ("isx006_set_meter_s_ctrl_by_enum: Multi or Average \n");
        break;
    case MSM_V4L2_EXP_SPOT_METERING: 
        v_temp = v_temp |0x02;   
        rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0104, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
        printk ("isx006_set_meter_s_ctrl_by_enum: Spot\n");
        break;

    default: {
        printk ("isx006_set_meter_s_ctrl_by_enum, wrong value setting\n");

        return -EINVAL;
        }
    }

    return rc;
}
//FIH-SW-MM-MC-ImplementCameraMeteringforIsx006-00+}

//FIH-SW-MM-MC-ImplementCameraTouchFocusforIsx006-00+{
int isx006_set_focus_roi_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
    int rc = 0;
    uint16_t window_size_x = 800;    
    uint16_t window_size_y = 600;    
    uint16_t reset_window_size_x = 1000;    
    uint16_t reset_window_size_y = 800;    
    uint16_t focus_limit_size_x = 2592 - 33 - window_size_x;    
    uint16_t focus_limit_size_y = 1944 - 5 - window_size_y;    

    uint16_t x = g_msm_sensor_focus_roi_info.x;
    uint16_t y = g_msm_sensor_focus_roi_info.y;
    uint16_t dx = g_msm_sensor_focus_roi_info.dx;
    uint16_t dy = g_msm_sensor_focus_roi_info.dy;
    uint16_t preview_ratio = g_msm_sensor_focus_roi_info.preview_ratio;
    uint8_t num_roi = g_msm_sensor_focus_roi_info.num_roi;


#if 1// Enable for debug
    printk("Befoer: isx006_set_focus_roi_s_ctrl_by_enum: {%d, %d, %d, %d, %d}, on = %d" 
                                                        , g_msm_sensor_focus_roi_info.x
                                                        , g_msm_sensor_focus_roi_info.y
                                                        , g_msm_sensor_focus_roi_info.dx
                                                        , g_msm_sensor_focus_roi_info.dy
                                                        , g_msm_sensor_focus_roi_info.preview_ratio
                                                        , g_msm_sensor_focus_roi_info.num_roi);
#endif

    //-------------/re-set to center focus area for single foucs mode-----------------------------------
    if((x == 1000) & (y == 1000)){//re-set to conter focus area for single foucs mode
        printk("isx006_set_focus_roi_s_ctrl_by_enum: re-set to conter focus area for single foucs mode \n");
        rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C4C, (2592 /2) - (reset_window_size_x / 2), MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0) {
            pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x4C4C) failed !\n");
            goto error;
        }
        
        rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C4E, (1944 /2) - (reset_window_size_y / 2), MSM_CAMERA_I2C_WORD_DATA);
        if (rc < 0) {
            pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x4C4E) failed !\n");
            goto error;
        }

        rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C50, reset_window_size_x, MSM_CAMERA_I2C_WORD_DATA);//AF_OPD4_HDELAY
        if (rc < 0) {
            pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x4C50) failed !\n");
            goto error;
        }

        rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C52, reset_window_size_y, MSM_CAMERA_I2C_WORD_DATA);//AF_OPD4_VDELAY
        if (rc < 0) {
            pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x4C52) failed !\n");
            goto error;
        }

        return rc;
    }
    //-----------------------------------------------------------------------------------------

    if( preview_ratio == 4000/3){
        x = x  * 2592 /2000 + 8 + 41 - (100);
        y = y  * 1944 /2000 + 4 - (100);
    }else
    {
        x = x  * 2592 /2000 + 8 + 41 - (100);
        y = y  * 1080 /2000 + 432 + 4 - (100); //1080 = 1944 - 432 * 2
    }
    printk("isx006_set_focus_roi_s_ctrl_by_enum: af_rect{x,y,dx,dy,num_roi} = {%d,%d,%d,%d,%d} \n", x, y, dx, dy, num_roi);

    // Check {x, y} position.
    if (num_roi)// num_roi no equal 0 if support touch focus.
    {
        /* There is a restriction for setting AF window. AF window can not be set right side (33 pixel) and under (5 pixel) of 3M image (2592x1944). 
        Therefore, horizontal position has to be set to the data which adds 49 to actual position of image. Vertical position has to be set to the data 
        which adds 4 to actual position of image. Minimum horizontal size of AF window is 2 pixel and vertical size is 8 pixel. */
        if (x >= focus_limit_size_x)
        {
            printk("isx006_set_focus_roi_s_ctrl_by_enum: x position too right = {%d} \n", x);
            x = focus_limit_size_x;
        }

        if (y >= focus_limit_size_y)
        {
            printk("isx006_set_focus_roi_s_ctrl_by_enum: y position too low = {%d} \n", y);
            y = focus_limit_size_y;
        }
    }       

    printk("isx006_set_focus_roi_s_ctrl_by_enum: af_rect{x,y} = {%d,%d} \n", x, y);
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C4C, x, MSM_CAMERA_I2C_WORD_DATA);//AF_OPD4_HDELAY
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: Write reg AF_OPD4_HDELAY failed !\n");
        goto error;
    }

    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C4E, y, MSM_CAMERA_I2C_WORD_DATA);//AF_OPD4_VDELAY
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: Write reg AF_OPD4_VDELAY failed !\n");
        goto error;
    }

    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C50, window_size_x, MSM_CAMERA_I2C_WORD_DATA);//AF_OPD4_HDELAY
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: Write reg AF_OPD4_VDELAY failed !\n");
        goto error;
    }

    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4C52, window_size_y, MSM_CAMERA_I2C_WORD_DATA);//AF_OPD4_VDELAY
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: Write reg AF_OPD4_VDELAY failed !\n");
        goto error;
    }
    
    //1. Set monitor AF mode
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x002E, MONI_AF_OFF, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x002E) failed !\n");
        goto error;
    }

    //2. MONI_REFRESH
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x0012, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x0012) failed for refresh !\n");
        goto error;
    }
    cam_msleep(50);

    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x002E, MONI_AF_SAF, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x002E) failed !\n");
        goto error;
    }

    //2. MONI_REFRESH
    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x0012, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_focus_roi_s_ctrl_by_enum: msm_camera_i2c_write_invert(0x0012) failed for refresh !\n");
        goto error;
    }

    AF_skip = true;

error:
    return rc;
}
//FIH-SW-MM-MC-ImplementCameraTouchFocusforIsx006-00+}

bool isx006_get_AE_value(struct msm_camera_i2c_client *client)
{
        int32_t AE_value = 0;
        int32_t AE_target = 0x0000FD52 - 0x0000FFFF - 1;
        bool Low_Light = false;
        uint16_t read_value = 0;
        int rc = 0;

        printk("isx006_get_AE_value: enter\n");

        /*Get AESCL_AUTO--------------------------------------------------------------------*/
        rc = msm_camera_i2c_read_invert(client, 0x0288, &read_value, MSM_CAMERA_I2C_WORD_DATA);  	
		if(rc < 0) {
			printk("isx006_get_AE_value: read AE value(0x0288) failed !\n");
		}
 
        AESCL_AUTO = read_value;
		//printk("isx006_get_AE_value: AESCL_AUTO= %d\n", AESCL_AUTO);

        /*Get ERRSCL_AUTO--------------------------------------------------------------------*/ 
        rc = msm_camera_i2c_read_invert(client, 0x0284, &read_value, MSM_CAMERA_I2C_WORD_DATA);   
 
        if(rc < 0) {
            printk("isx006_get_AE_value: read AE value(0x0284) failed !\n");
        }
		//printk("isx006_get_AE_value: read_value= %d\n", read_value);
		

        if( read_value >= 0x00008000 ){
            ERRSCL_AUTO = (read_value - 0x0000FFFF -1);
			//printk("isx006_get_AE_value: ERRSCL_AUTO[1]= %d\n", ERRSCL_AUTO);
        }
        else{
            ERRSCL_AUTO = (read_value & 0x0000FFFF);
			//printk("isx006_get_AE_value: ERRSCL_AUTO[2]= %d\n", ERRSCL_AUTO);
        }

        AE_value = ERRSCL_AUTO;

		printk("isx006_get_AE_value: AE_value = %d, AE_target = %d \n", AE_value, AE_target);

        if(AE_value <= AE_target)
        {
            Low_Light = true;
            printk("isx006_get_AE_value: low light condition !\n");
        }
		//printk("isx006_get_AE_value: exit, Low_Light = %d \n", Low_Light);

        return Low_Light;
}

/*MTD-MM-SL-SupportFlash-01+{ */
int isx006_PreFlash_setting(struct msm_camera_i2c_client *client)
{
    int rc = 0;

    /* Set LED & AE/AWB Control----------------------------*/
    rc = msm_camera_i2c_write_invert(client, 0x0069, 0x0D, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x0069);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x02C4, 0x0010, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x02C4);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x027D, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x027D);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x4066, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x4066);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x402F, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x402F);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x404C, 0x2C, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x404C);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x400B, 0x3C, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_PreFlash_setting: Write REG(0x%x) failed !\n", 0x400B);
        return-EIO;
    }
    
    return rc;
}

/*MTD-MM-UW-fix CTS preview fail-00*{ */
/* MTD-MM-SL-SupportFlash-02*{ */
/*MTD-MM-UW-AddAF-00*{ */
int isx006_msm_sensor_s_af(struct msm_sensor_ctrl_t *s_ctrl, //uriwei
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
	int led_mode = 0;
	bool Low_Light = false;
	uint16_t AF_state = 0xFF;
	uint16_t v_read = 0x0; 
	
    printk("isx006_msm_sensor_s_af: enter\n");
	
	torch_enable = false;
	/*get current environment AE valuel----------------------------*/
    Low_Light = isx006_get_AE_value(s_ctrl->sensor_i2c_client);
	
	/*get current flash model-------------------------------------*/
	led_mode = msm_soc_get_led_mode();
	printk("isx006_msm_sensor_s_af: led_mode = %d !\n", led_mode);  

	/*judge pre-flash or not--------------------------------------*/
	if( led_mode == LED_MODE_RED_EYE || led_mode == LED_MODE_AUTO){
		 if(Low_Light)
		 	torch_enable = true;
	}

	/*pre-flash-------------------------------------------------*/
	if( led_mode == LED_MODE_ON) {
		rc = msm_soc_torch_trigger();
		if (rc < 0) 
			pr_err("isx006_msm_sensor_s_af: msm_soc_torch_trigger() failed !\n");
	}else if( torch_enable){
		printk("isx006_msm_sensor_s_af: flash enable!\n");
		rc = msm_soc_torch_trigger();
		if (rc < 0)
			pr_err("isx006_msm_sensor_s_af: msm_soc_torch_trigger() failed !\n");

		/* Set LED & AE/AWB Control----------------------------*/
		rc = isx006_PreFlash_setting(s_ctrl->sensor_i2c_client);
		if (rc < 0)
			pr_err("isx006_msm_sensor_s_af: isx006_PreFlash_setting failed !\n");	
	}
	if(AF_full_range){
		rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4876, AF_A_value, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE1
		if (rc < 0)
			pr_err("isx006_msm_sensor_s_af: msm_camera_i2c_write REG(0x%x) failed !\n", 0x4876);	
	}

	/*check CAF state*/
	rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x6D76, &AF_state, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("isx006_msm_sensor_s_af: msm_camera_i2c_write  REG(0x%x) failed !\n", 0x6D76);
		goto error;
	}

	if (AF_state == 15 || AF_state == 12){ //CAF_LOCK
		rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4887, 0x01, MSM_CAMERA_I2C_BYTE_DATA); 
		if (rc < 0){
			pr_err("isx006_msm_sensor_s_af: msm_camera_i2c_write REG(0x%x) failed !\n", 0x4887);
		}

		AF_type = 1; //fine search
	}else{
		rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4887, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0){
			pr_err("isx006_msm_sensor_s_af: msm_camera_i2c_write REG(0x%x) failed !\n", 0x4887);
		}

		AF_type = 0; //full search
	}

	/*MTD-MM-SL-FixCoverity-00*{ */
	rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x002E, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0){
		pr_err("isx006_msm_sensor_s_af: msm_camera_i2c_read REG(0x%x) failed !\n", 0x002E);
	}
	/*MTD-MM-SL-FixCoverity-00*} */
	if(AF_skip){
		printk("isx006_msm_sensor_s_af: did touch AF before, skip AF here\n");
	}
	else if(v_read == 0){
		printk("isx006_msm_sensor_s_af: restart AF \n");
              rc = isx006_set_monitor_af_mode(s_ctrl->sensor_i2c_client, MONI_AF_RESTART, "MONI_AF_RESTART");
		if (rc < 0)
			goto error; 
		
	} else {
            rc = isx006_set_monitor_af_mode(s_ctrl->sensor_i2c_client, MONI_AF_SAF, "MONI_AF_SAF");
            if(rc < 0){
                pr_err("isx006_msm_sensor_s_af: set af mode (MONI_AF_SAF) fail\n");
                goto error;
            }
	}
	printk("isx006_msm_sensor_s_af: exit\n");
	return rc;

error:
	return rc;
}
/*MTD-MM-UW-AddAF-00*} */
/* MTD-MM-SL-SupportFlash-02*} */
/*MTD-MM-UW-fix CTS preview fail-00*} */

/*MTD-MM-UW-SnapshotFail-02*{ */
/* MTD-MM-UW-AF_tune-00+ */
/* MTD-MM-SL-SupportFlash-02*{ */
/* MTD-MM-SL-Add5M2ndSource-02*{ */
/* MTD-MM-SL-SupportAF-00*{ */	
/*MTD-MM-UW-set AF mode-00*{ */
int isx006_msm_sensor_s_af_mode(struct msm_sensor_ctrl_t *s_ctrl, 
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;	 

	if(value != MSM_V4L2_AF_POLLING)
		printk("%s: AF_value = %d\n", __func__,value);

	switch (value) {
       case MSM_V4L2_AF_CAF:
           printk("isx006_msm_sensor_s_af_mode: set AF CAF\n"); 
            if(current_af_mode == MSM_V4L2_AF_GET_STATE)
            {
                f_CAF = true;
            }
            else
            {   
                if(vendor_id) //SEMCO
                    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4876, AF_A_value + 30, MSM_CAMERA_I2C_WORD_DATA);//AF_AREA_LOW_TYPE1
                rc = isx006_set_monitor_af_mode(s_ctrl->sensor_i2c_client, MONI_AF_CAF, "MONI_AF_CAF");
                if(rc < 0)
                    goto error;
            }
           break;
       case MSM_V4L2_AF_OFF:
           printk("isx006_msm_sensor_s_af_mode: set AF off\n"); 
           rc = isx006_set_monitor_af_mode(s_ctrl->sensor_i2c_client, MONI_AF_MF, "MONI_AF_MF");
           if(rc < 0){
                pr_err("isx006_msm_sensor_s_af: set af mode fail\n");
				goto error;
           	}

           rc = isx006_MF_position(s_ctrl->sensor_i2c_client, 0);
           if (rc < 0)
           		printk ("isx006_msm_sensor_s_af_mode: isx006_check_MF fail.\n"); 
           
           break;    
       case MSM_V4L2_AF_MACRO:
           printk("isx006_msm_sensor_s_af_mode: set AF Macro\n"); 
           rc = isx006_set_AF_Range(s_ctrl->sensor_i2c_client, AF_MODE_MACRO);
           if(rc < 0){
                pr_err("isx006_msm_sensor_s_af: set af mode fail\n");
				goto error;
           }
           break; 
       case MSM_V4L2_AF_INFINITY:
	    if((isx006_scene == MSM_V4L2_SCENE_NIGHT) || (isx006_scene == MSM_V4L2_SCENE_LANDSCAPE))
	    {
	    	printk("isx006_msm_sensor_s_af_mode: Already set AF Infinity! Set back CAF mode\n"); 
		rc = isx006_set_monitor_af_mode(s_ctrl->sensor_i2c_client, MONI_AF_CAF, "MONI_AF_CAF");
                if(rc < 0)
                    goto error;	
		return 0;
	    }
		
           printk("isx006_msm_sensor_s_af_mode: set AF Infinity\n"); 
           rc = isx006_set_AF_Range(s_ctrl->sensor_i2c_client, AF_MODE_INFINITY);
           if(rc < 0){
                pr_err("isx006_msm_sensor_s_af: set af mode fail\n");
				goto error;
           }

           break;	   
       case MSM_V4L2_AF_POLLING:
            //printk("isx006_msm_sensor_s_af_mode: set AF Polling\n"); 
            rc = isx006_check_AF(s_ctrl->sensor_i2c_client, "check AF polling status");
            /*if(rc < 0){
            		pr_err("isx006_msm_sensor_s_af: set af mode (AF_POLLING) fail\n");
            }*/
            
            rc_af_check = rc;
            rc = 0;
           break;
       case MSM_V4L2_AF_GET_STATE:
            printk("isx006_msm_sensor_s_af_mode: get AF State\n"); 
            rc = isx006_get_AF_state(s_ctrl->sensor_i2c_client, "check AF result");
            if (rc < 0){
                if(!AF_type){ //full search
                	 pr_err("isx006_msm_sensor_s_af_mode: full search \n");
                 rc_af_check = rc;    
                 goto error;

                } 
                else
                {
                    printk("isx006_msm_sensor_s_af_mode: fine search fail !!\n");
                    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4887, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
                    rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x0015, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
                     AF_type = 0;
                     rc_af_check = 2; //Re-focus with full range
                }
            }
            else
            {
                rc_af_check = rc;
                printk("isx006_msm_sensor_s_af_mode: get AF State ok!!!\n"); 
            }
            rc = 0;
            AF_skip = false;
            break;
       default:
            printk("isx006_msm_sensor_s_af_mode: invalid AF type\n"); 
       }
       current_af_mode = value;     
       //printk("isx006_msm_sensor_s_af_mode: exit\n");
           
	return rc;	

error:
    rc_af_check = rc;
    return rc;	
}
/*MTD-MM-UW-set AF mode-00*} */
/* MTD-MM-SL-SupportAF-00+} */
/* MTD-MM-SL-Add5M2ndSource-02*} */
/* MTD-MM-SL-SupportFlash-02*} */
/* MTD-MM-UW-AF_tune-00- */
/*MTD-MM-UW-SnapshotFail-02*} */

/*MTD-MM-SL-FixMMSRecord-00+{ */
static int32_t isx006_set_fps(struct msm_sensor_ctrl_t *s_ctrl, uint16_t fps)
{
    int32_t rc = 0;
    
    printk ("isx006_set_fps ------- value =%d\n", fps); 
    if(fps <=15){    
        rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0383, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_set_fps: Write REG(0x%x) failed !\n", 0x0383);
            goto error;
        }
    }
    else{
        rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0383, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("isx006_set_fps: Write REG(0x%x) failed !\n", 0x0383);
            goto error;
        }    
    }

    //Re-fresh register, that fps could be set successfully  
    rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0012, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_fps: Write REG(0x%x) failed !\n", 0x0012);
        goto error;
    }

error:
    return rc;
}
/*MTD-MM-SL-FixMMSRecord-00+} */

/* MTD-MM-SL-SnapshotTooLong-00+{ */ 
int isx006_RecoverPreFlash_setting(struct msm_camera_i2c_client *client)
{
    int rc = 0;

    /* Set LED & AE/AWB Control----------------------------*/
    rc = msm_camera_i2c_write_invert(client, 0x0069, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x0069);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x02C4, 0x0000, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x02C4);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x027D, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x027D);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x4066, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x4066);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x402F, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x402F);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x404C, 0x20, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x404C);
        return-EIO;
    }
    rc = msm_camera_i2c_write_invert(client, 0x400B, 0x1A, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_RecoverPreFlash_setting: Write REG(0x%x) failed !\n", 0x400B);
        return-EIO;
    }
    
    return rc;
}
/* MTD-MM-SL-SnapshotTooLong-00+} */ 

static int32_t isx006_snapshot_config(struct msm_sensor_ctrl_t *s_ctrl, int mode, int res)
{

    int32_t rc = 0;
    int led_mode = 0;
    uint16_t v_read = 0x0;            
    int32_t Diff = 0;
    uint16_t Diff_index = 0;
    int i = 0;
    uint16_t read_value = 0;
    
    /*get current flash mode*/
    led_mode = msm_soc_get_led_mode();

	printk("isx006_snapshot_config---------------E. led_mode = %d\n", led_mode);

	#if 1
    /*judge pre-flash or not--------------------------------------*/
    if(led_mode == LED_MODE_ON)
    {
        printk("isx006_snapshot_config--fill flash\n");
        if(isx006_get_AE_value(s_ctrl->sensor_i2c_client))
        {
            printk("isx006_snapshot_config--fill flash & low light\n");
            flash_enable = true;
            /* Set LED & AE/AWB Control-----------------*/
            rc = isx006_PreFlash_setting(s_ctrl->sensor_i2c_client);
            if (rc < 0) 
                printk("isx006_snapshot_config: isx006_PreFlash_setting failed !\n");
        }
    }
    else if( led_mode == LED_MODE_RED_EYE || led_mode == LED_MODE_AUTO )
    {
        if(torch_enable)
        {
            printk("isx006_snapshot_config--torch & flash enable\n");
            flash_enable = true;
        }
        else
        {
            printk("isx006_snapshot_config--flash enable\n");
            if(isx006_get_AE_value(s_ctrl->sensor_i2c_client))
            {
                printk("isx006_snapshot_config--flash enable & low light\n");
                flash_enable = true;
                /* Set LED & AE/AWB Control-----------------*/
                rc = isx006_PreFlash_setting(s_ctrl->sensor_i2c_client);
                if (rc < 0) 
                    printk("isx006_snapshot_config: isx006_PreFlash_setting failed !\n");
            }
        }
    }

    /*calulate exposure gain for flash------------------------------*/
    if(flash_enable)
    {
        rc = msm_soc_torch_trigger(); 
        cam_msleep(200);

        /* 02.Check HALF_MOVE_STS ----------------*/

        /* Make sure operation mode is changed */
        printk("isx006_snapshot_config: check HALF_MOVE_STS\n");
        for (i = 0; i < ISX006_AE_RETRY_COUNT; i++)
        {
            rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x6C00, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
            if (rc < 0) {
                pr_err("isx006_snapshot_config: isx006_i2c_read_parallel failed, REG(0x%x) !\n", 0x6C00);
                goto error;
            }
            if (v_read == 0)
                break;
            cam_msleep(10);    
        }
        
        if (i >= ISX006_AE_RETRY_COUNT)
        {
            pr_err("isx006_snapshot_config: check HALF_MOVE_STS fail !\n");
        }

        /*Get AESCL_NOW--------------------------*/
        rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x028A, &read_value, MSM_CAMERA_I2C_WORD_DATA);   
 
        AESCL_NOW = read_value;
            

        /*Get ERRSCL_NOW-------------------------*/
        rc = msm_camera_i2c_read_invert(s_ctrl->sensor_i2c_client, 0x0286, &read_value, MSM_CAMERA_I2C_WORD_DATA);   
  
        if(rc < 0) {
            printk("isx006_get_AE_value: read AE value failed !\n");
        }

        if( read_value >= 0x00008000 )
            ERRSCL_NOW = (read_value - 0x0000FFFF -1);
        else
            ERRSCL_NOW = (read_value & 0x0000FFFF);
        
        /*----------------------------------------*/

        printk("isx006_snapshot_config: AESCL_AUTO = %d, ERRSCL_AUTO = %d\n", AESCL_AUTO, ERRSCL_AUTO);
        printk("isx006_snapshot_config: AESCL_NOW = %d, ERRSCL_NOW = %d\n", AESCL_NOW, ERRSCL_NOW);

        Diff = (AESCL_NOW + ERRSCL_NOW) -(AESCL_AUTO + ERRSCL_AUTO);
        //printk("isx006_snapshot_config+: Diff = %d \n", Diff);
        /* ----------------------------------------*/
        if(Diff < 0)
            Diff_index = 0;
        else if((Diff - (Diff/10) * 10) < 5)
            Diff_index = (Diff/10);
        else
            Diff_index = (Diff/10) + 1;
        
        printk("isx006_snapshot_config: Diff_index = %d \n", Diff_index);

        if(Diff_index >= offset_size -1)
        {
            Diff_index = offset_size - 1;
        }
        
        if(led_mode == LED_MODE_ON)
        {
            if(isx006_get_AE_value(s_ctrl->sensor_i2c_client))
                rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x0282, (x_8[Diff_index] - ERRSCL_NOW) + (0x0000FFFF + 1), MSM_CAMERA_I2C_WORD_DATA);
        }
        else
       {
            rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x0282, (x_8[Diff_index] - ERRSCL_NOW) + (0x0000FFFF + 1), MSM_CAMERA_I2C_WORD_DATA);
        }
            
    }

	/*msm_sensor_set_sensor_mode*/		
    rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl, MSM_SENSOR_UPDATE_PERIODIC, RES_CAPTURE);  /*MTD-MM-SL-ImproveMainCamera-00* */
	if (rc < 0)
		return rc;
    	

    //Add condition type for flash trigger
    if(led_mode == LED_MODE_RED_EYE)
    {
        printk("isx006_snapshot_config: led_mode = LED_MODE_RED_EYE\n");
        if(flash_enable){
            rc = msm_soc_flash_trigger();
            if (rc < 0)
                printk("isx006_snapshot_config: msm_soc_flash_trigger 1 failed !\n");
            cam_msleep(300);

            rc = msm_soc_flash_trigger_off();
            if (rc < 0)
                printk("isx006_snapshot_config: msm_soc_flash_trigger_off failed !\n");

            cam_msleep(100);

            rc = msm_soc_flash_trigger();
            if (rc < 0)
                printk("isx006_snapshot_config: msm_soc_flash_trigger 2 failed !\n");
        }
    }
    else if(led_mode == LED_MODE_AUTO)
    {
        printk("isx006_snapshot_config: led_mode = LED_MODE_AUTO\n");
        if(flash_enable){
            rc = msm_soc_flash_trigger();
            if (rc < 0)
                printk("isx006_snapshot_config: msm_soc_flash_trigger failed !\n");
        }
    }
    else if(led_mode == LED_MODE_ON)
    {    
        printk("isx006_snapshot_config: led_mode = LED_MODE_ON\n");
        rc = msm_soc_flash_trigger();
        if (rc < 0)
            printk("isx006_snapshot_config: msm_soc_flash_trigger failed!\n");
    }
    else
    {
        printk("isx006_snapshot_config: led_mode == LED_MODE_OFF\n");
    }

    flash_enable = false;
    torch_enable = false;
    printk("isx006_snapshot_config---------------X\n");
	#endif
    return rc;
    
error:
    printk("isx006_snapshot_config: ---------------X <Failed> !\n");
    return rc;
 
}

/* MTD-MM-UW-AF_tune-00+ */
/* MTD-MM-SL-Add5M2ndSource-02+{ */
static int32_t isx006_video_config(struct msm_sensor_ctrl_t *s_ctrl, int mode, int res)
{
	int32_t rc = 0;

	printk("isx006_video_config---------------E. res = %d\n", res);

        if(f_CAF)
        {    
        	if(vendor_id){ //SEMCO to decrease knock noise
        		rc = msm_camera_i2c_write_invert(s_ctrl->sensor_i2c_client, 0x4876, AF_A_value + 30, MSM_CAMERA_I2C_WORD_DATA);
        		if(rc < 0){
        			printk("isx006_video_config: msm_camera_i2c_write (REG 0x4876) failed!\n");
        			return rc;
        		}	
        	}
              rc = isx006_set_monitor_af_mode(s_ctrl->sensor_i2c_client, MONI_AF_CAF, "MONI_AF_CAF"); 
        }
        f_CAF = false;

	rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl, MSM_SENSOR_UPDATE_PERIODIC, RES_PREVIEW); /*MTD-MM-SL-ImproveMainCamera-00* */
	if (rc < 0)
		return rc;	

	printk("isx006_video_config---------------X\n");
	
	return rc;

}
/* MTD-MM-SL-Add5M2ndSource-02+} */
/* MTD-MM-UW-AF_tune-00- */

int32_t isx006_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl, int mode, int res)
{
	int32_t rc = 0;

	printk("isx006_set_sensor_mode: mode = %d, res = %d\n", mode, res);

	if (s_ctrl->curr_res != res) {
		s_ctrl->curr_frame_length_lines =
			s_ctrl->msm_sensor_reg->
			output_settings[res].frame_length_lines;

		s_ctrl->curr_line_length_pclk =
			s_ctrl->msm_sensor_reg->
			output_settings[res].line_length_pclk;
	
		switch (res) {
    	case RES_PREVIEW:
        	rc = isx006_video_config(s_ctrl, mode, res);
        	break;
			
    	case RES_CAPTURE:
        	rc = isx006_snapshot_config(s_ctrl, mode, res);
        	break;
		
    	default:
        	rc = -EINVAL;
        	break;
    	}

	s_ctrl->curr_res = res;
	}

    return rc;			

}

/*MTD-MM-UW-SnapshotFail-01*{ */
/*static int isx006_check_data_on(struct msm_camera_i2c_client *client)
{
    int rc = 0;
    int i =0;
    uint16_t v_read = 0x0;	
    uint16_t v_temp = 0x0;		

    printk("isx006_check_data_on: enter !+++++++++++++++++++++++++++\n");
    for(i =0;i < 30;i++)
   {
    
	rc = msm_camera_i2c_read(client, 0x0004, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
       pr_err("isx006_check_data_on: Reg(0x0004) = 0x%x, Retry = %d!\n",v_read, i);

       v_temp = (v_read & 0x04) >> 2;
       //pr_err("isx006_check_data_on: v_temp = %d !\n",v_temp);
	if (v_temp)
		break;
        cam_msleep(10); 
    }

    return rc;
}


static int isx006_check_data_off(struct msm_camera_i2c_client *client)
{
    int rc = 0;
    int i =0;
    uint16_t v_read = 0x0;	
    uint16_t v_temp = 0x0;		

    printk("isx006_check_data_off: enter !--------------------------------------\n");
    for(i =0;i < 30;i++)
   {
	rc = msm_camera_i2c_read(client, 0x0004, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
       pr_err("isx006_check_data_off: Reg(0x0004) = 0x%x, Retry = %d!\n",v_read, i);

       v_temp = (v_read & 0x04) >> 2;
       //pr_err("isx006_check_data_off: v_temp = %d !\n",v_temp);
	if (!v_temp)
		break;
        cam_msleep(10); 
    }

//error:
   // printk("isx006_check_AF: Check AF error\n");
    return rc;
}
*/
int isx006_set_camera_mode(struct msm_camera_i2c_client *client, enum isx006_mode new_mode, const char *tag)
{
    int rc = 0;
    uint16_t v_read;
    uint16_t v_temp;
    uint16_t byte_value = 0xFF; 

    /* Clear interrupt status */
    rc = msm_camera_i2c_read(client, 0x00FC, &v_read, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_camera_mode: Read REG(0x%x) failed !\n", 0x00FC);
        goto error;
    }
    v_temp = v_read | 0x02;    

    rc = msm_camera_i2c_write(client, 0x00FC, v_temp, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_camera_mode: Write REG(0x%x) failed !\n", 0x00FC);
        goto error;
    }
    cam_msleep(10);   
    
    rc = msm_camera_i2c_read(client, 0x0004, &byte_value, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_camera_mode: Try to get current mode failed !\n");
        return rc;
    }
    printk("isx006_set_camera_mode: pre set camera mode to %s, REG_0x0004 = 0x%x\n", tag, byte_value);
	
    rc = msm_camera_i2c_write(client, 0x0011, new_mode, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("isx006_set_camera_mode: isx006_i2c_write_parallel(0x0011) failed !\n");
        goto error;
    }
    
    printk("isx006_set_camera_mode(%s): Success \n", tag);
    return rc;
    
error:
    printk("isx006_set_camera_mode(%s): Failed !\n", tag);
    return rc;
}


/*MTD-MM-SL-SupportFlash-01+} */

/*MTD-MM-SL-ImproveMainCamera-03*{ */
/*MTD-MM-UW-SnapshotFail-00*{ */
/*MTD-MM-SL-ImproveMainCamera-00*{ */
/*MTD-MM-SL-SnapshotFail-00*{ */
/*MTD-MM-SL-AddForSoMCScene-00+{ */
/* MTD-MM-SL-SnapshotTooLong-00*{ */ 
/* MTD-MM-SL-Add5M2ndSource-00+{ */
int32_t isx006_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	printk("%s: update_type = %d \n", __func__, update_type);

        /*rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0039, 0x01, MSM_CAMERA_I2C_BYTE_DATA);	
        if (rc < 0) {
            pr_err("isx006_sensor_setting: stop streaming failed !\n");
        } */
	
	if (update_type == MSM_SENSOR_REG_INIT) {
		printk("isx006_sensor_setting: Register INIT\n");
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);

		s_ctrl->func_tbl->sensor_init_setting(s_ctrl, update_type,res);
		csi_config = 0;
		
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		printk("isx006_sensor_setting: PERIODIC : %d\n", res);

		/*For  preview: after second time to launch camera*/
		if ((res == RES_PREVIEW) && (csi_config == true)){			
			printk("isx006_sensor_setting: case UPDATE_PERIODIC <RES_PREVIEW + CSI(1)>\n");
			
			rc = isx006_RecoverPreFlash_setting(s_ctrl->sensor_i2c_client);
			if (rc < 0) {
                        pr_err("isx006_sensor_setting: Restore normal settings fail !\n");
                        return rc;
			}
			
                     rc = isx006_set_camera_mode(s_ctrl->sensor_i2c_client, MONITOR_MODE,  "MONITOR_MODE");
			if (rc < 0){
				printk("isx006_sensor_setting: msm_camera_i2c_write (REG 0x0011) failed!\n");
				return rc;
			}

			rc = isx006_check_cm(s_ctrl->sensor_i2c_client, MONITOR_MODE);
			if (rc < 0) {
                        pr_err("isx006_sensor_setting: isx006_check_cm(For CM change to Monitor mode) failed !\n");
                        return rc;
                    }

			s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
			CDBG("CSI config in progress\n");
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIC_CFG,
				s_ctrl->curr_csic_params);
			CDBG("CSI config is done\n");
			mb();

		 	v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
		 	NOTIFY_PCLK_CHANGE,
		 	&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
		 }

		/*For preview: first time to launch camera*/
		if (!csi_config){		
			printk("isx006_sensor_setting: case UPDATE_PERIODIC <RES_PREVIEW + CSI(0)>\n");

			msm_sensor_write_conf_array(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->mode_settings, res);
			
			s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
			CDBG("CSI config in progress\n");
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIC_CFG,
				s_ctrl->curr_csic_params);
			CDBG("CSI config is done\n");		
			mb();
			cam_msleep(30);
			csi_config = 1;
			
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE,
			&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);

			if (STARTUP == 0){
				printk("isx006_sensor_setting: STARTUP = 0\n");
				s_ctrl->func_tbl->sensor_start_stream_1(s_ctrl);
				cam_msleep(30);
				STARTUP = ISX006_ENABLE_HW_STANDBY; /*MTD-MM-SL-CTSFail-00* */
			}else{
				printk("isx006_sensor_setting: STARTUP = 1, exit_standby! \n");
				rc = isx006_exit_standby(s_ctrl, ACT_EXIT_STANDBY);
				if (rc < 0){
					printk("isx006_sensor_setting: isx006_exit_standby fail, need reset \n");
					rc = isx006_power_up_reset(s_ctrl, 1);
						if (rc <0){
							pr_err("isx006_sensor_setting: isx006_power_up_reset fail! \n");
							return rc;
						}
					}
				}
		}

		/*For snapshot */
		if (res == RES_CAPTURE){			
			printk("isx006_sensor_setting: case UPDATE_PERIODIC <RES_CAPTURE>\n");

			if(isx006_scene == MSM_V4L2_SCENE_NIGHT || isx006_scene == MSM_V4L2_SCENE_NIGHT_PORTRAIT) /*MTD-MM-SL-AddForSoMCScene-01* */
                        cam_msleep(100);

			rc = isx006_set_camera_mode(s_ctrl->sensor_i2c_client, CAPTURE_MODE,  "CAPTURE_MODE");
			if (rc < 0){
				printk("isx006_sensor_setting: msm_camera_i2c_write (REG 0x0011) failed!\n");
				return rc;
			}
			
			rc = isx006_check_cm_capture(s_ctrl->sensor_i2c_client, CAPTURE_MODE);
			if (rc < 0) {
                        pr_err("isx006_sensor_setting: isx006_check_cm(For CM change to Capture mode) failed !\n");
                        return rc;
                    }
			
			s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
			CDBG("CSI config in progress\n");
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIC_CFG,
				s_ctrl->curr_csic_params);
			CDBG("CSI config is done\n");
			mb();
			
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE,
			&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
			
		} 
	}

        /*rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0039, 0x00, MSM_CAMERA_I2C_BYTE_DATA);	
        if (rc < 0) {
            pr_err("isx006_sensor_setting: start streaming failed !\n");
        } */
        
	printk("isx006_sensor_setting: Done\n");
	return rc;
}
/* MTD-MM-SL-Add5M2ndSource-00+} */
/* MTD-MM-SL-SnapshotTooLong-00*} */ 
/*MTD-MM-SL-AddForSoMCScene-00+} */
/*MTD-MM-SL-SnapshotFail-00*} */
/*MTD-MM-UW-SnapshotFail-00*} */
/*MTD-MM-SL-ImproveMainCamera-03*} */
/*MTD-MM-UW-SnapshotFail-01*} */

int32_t isx006_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	int32_t rc = 0;
	s_ctrl->fps_divider = Q10;
	s_ctrl->cam_mode = MSM_SENSOR_MODE_INVALID;

	printk("isx006_sensor_mode_init,STARTUP = %d \n", STARTUP);
	if (mode != s_ctrl->cam_mode) {
		printk("isx006_sensor_mode_init\n");
		s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->cam_mode = mode;

		if (STARTUP == 0){
			if (s_ctrl->is_csic ||
				!s_ctrl->sensordata->csi_if)
				rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
					MSM_SENSOR_REG_INIT, 0);
			else
				rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
					MSM_SENSOR_REG_INIT, 0);
		}
		/*MTD-MM-SL-Add5M2ndSource-03+{ */
		/* <1-4>. Write preload3-reload setting for 2nd source */  
		if(!vendor_id){ //KMOT
			rc = msm_camera_i2c_write_tbl_invert(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->reload_preload3_2nd_settings,
				s_ctrl->msm_sensor_reg->reload_preload3_2nd_size,
				s_ctrl->msm_sensor_reg->default_data_type);	

			if (rc < 0){
				pr_err("isx006_sensor_init_setting: Write reload Period3_2nd table failed !\n");
				return rc;
			}
		}
		/*MTD-MM-SL-Add5M2ndSource-03+} */
	}
	return rc;
}
/*MTD-MM-SL-ImproveMainCamera-00*} */

/*MTD-MM-SL-ImproveMainCamera-00*} */

struct msm_sensor_v4l2_ctrl_info_t isx006_v4l2_ctrl_info[] = {
	{
		.ctrl_id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.min = MSM_V4L2_WB_OFF,
		.max = MSM_V4L2_WB_CLOUDY_DAYLIGHT,
		.step = 1,
		.enum_cfg_settings = &isx006_wb_oem_enum_confs,
		.s_v4l2_ctrl = isx006_msm_sensor_s_ctrl_by_enum,
	},
	/*MTD-MM-SL-AddBrightness-01*{ */
	/*MTD-MM-SL-AddBrightness-00+{ */
	{
		.ctrl_id = V4L2_CID_EXPOSURE, 
		.min = MSM_V4L2_BRIGHTNESS_V0,
		.max = MSM_V4L2_BRIGHTNESS_V12, 
		.step = 1,
		.enum_cfg_settings = &isx006_brightness_enum_confs,
		.s_v4l2_ctrl = isx006_msm_sensor_s_ctrl_by_enum,
	},
	/*MTD-MM-SL-AddBrightness-00+} */
	/*MTD-MM-SL-AddBrightness-01*} */	
    //FIH-SW-MM-MC-ImplementCameraSceneModeforIsx006-00+{
    {
        .ctrl_id = V4L2_CID_BESTSHOT_MODE,
        .min = MSM_V4L2_SCENE_OFF,
        .max = MSM_V4L2_SCENE_DOCUMENT,
        .step = 1,
        .enum_cfg_settings = &isx006_scene_oem_enum_confs,
        .s_v4l2_ctrl = isx006_msm_sensor_s_ctrl_by_enum,
    },
    //FIH-SW-MM-MC-ImplementCameraSceneModeforIsx006-00+}
    //FIH-SW-MM-MC-ImplementCameraMeteringforIsx006-00+{
    {
        .ctrl_id = V4L2_CID_METER_MODE,
        .min = MSM_V4L2_EXP_FRAME_AVERAGE,
        .max = MSM_V4L2_EXP_SPOT_METERING,
        .step = 1,
        //.enum_cfg_settings = &isx006_meter_oem_enum_confs,
        .s_v4l2_ctrl = isx006_set_meter_s_ctrl_by_enum,
    },
    //FIH-SW-MM-MC-ImplementCameraMeteringforIsx006-00+}
    //FIH-SW-MM-MC-ImplementCameraTouchFocusforIsx006-00+{
    {
        .ctrl_id = V4L2_CID_FOCUS_ROI,
        .min = 0,
        .max = 1,
        .step = 1,
        //.enum_cfg_settings = &isx006_focus_roi_oem_enum_confs,
        .s_v4l2_ctrl = isx006_set_focus_roi_s_ctrl_by_enum,
    },
    //FIH-SW-MM-MC-ImplementCameraTouchFocusforIsx006-00+}
	/*MTD-MM-UW-AddAF-00*{ */
	{
		.ctrl_id = V4L2_CID_AF,
		.min = MSM_V4L2_AF_OFF,
		.max = MSM_V4L2_AF_MAX, /* MTD-MM-SL-SupportAF-00* */ 
		.step = 1,
		.enum_cfg_settings = &isx006_af_enum_confs,
		.s_v4l2_ctrl = isx006_msm_sensor_s_af,
	},
	/*MTD-MM-UW-AddAF-00*} */
       /*MTD-MM-UW-set AF mode-00*{ */
	{
		.ctrl_id = V4L2_CID_AF_MODE,
		.min = MSM_V4L2_AF_ON,
		.max = MSM_V4L2_AF_MAX, /* MTD-MM-SL-SupportAF-00* */ 
		.step = 1,
		.enum_cfg_settings = &isx006_af_enum_confs,//didn't used
		.s_v4l2_ctrl = isx006_msm_sensor_s_af_mode,
	},
	/*MTD-MM-UW-set AF mode-00*} */ 
};
/*MTD-MM-SL-AddWB-00+} */

static struct msm_camera_csi_params isx006_csi_params = {
	.data_format = CSI_8BIT,
	.lane_cnt    = 2,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14,
};

static struct msm_camera_csi_params *isx006_csi_params_array[] = {
	&isx006_csi_params, /* Snapshot */
	&isx006_csi_params,	/* Preview */
	&isx006_csi_params,	/* Video *//*MTD-MM-SL-PatchForCameraFeature-00+ */
};

static struct msm_sensor_output_reg_addr_t isx006_reg_addr = {
	.x_output = 0x0022,
	.y_output = 0x0028,
	.line_length_pclk = 0x0022, 
	.frame_length_lines = 0x0028,
};

static struct msm_sensor_id_info_t isx006_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x3009,
	.sensor_id2 = 0x3013,
};

static const struct i2c_device_id isx006_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&isx006_s_ctrl},
	{ }
};

/*MTD-MM-SL-ImproveMainCamera-00*{ */
static struct i2c_driver isx006_i2c_driver = {
	.id_table = isx006_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
	//Move to early_suspend, we register early_suspend in  msm_sensor_init_module
	.suspend = isx006_suspend, /*MTD-MM-SL-QuicklyUnlockCameraFail-00- */
    .resume = isx006_resume,
};
/*MTD-MM-SL-ImproveMainCamera-00*} */

static struct msm_camera_i2c_client isx006_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	int rc = 0;
	CDBG("isx006\n");

	rc = i2c_add_driver(&isx006_i2c_driver);

	return rc;
}

static struct v4l2_subdev_core_ops isx006_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops isx006_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops isx006_subdev_ops = {
	.core = &isx006_subdev_core_ops,
	.video  = &isx006_subdev_video_ops,
};

static struct msm_sensor_fn_t isx006_func_tbl = {
	.sensor_start_stream_1 = isx006_sensor_start_streaming, //msm_sensor_start_stream
	.sensor_stop_stream = isx006_sensor_stop_stream, //msm_sensor_stop_stream, /*MTD-MM-SL-ImproveMainCamera-00* */
	.sensor_csi_setting = isx006_sensor_setting, //msm_sensor_setting1, /* MTD-MM-SL-Add5M2ndSource-00+ */
	.sensor_init_setting = isx006_sensor_init_setting,
	.sensor_set_sensor_mode = isx006_set_sensor_mode, /*MTD-MM-SL-SupportFlash-01+ *///msm_sensor_set_sensor_mode,
	.sensor_mode_init = isx006_sensor_mode_init, //msm_sensor_mode_init, /*MTD-MM-SL-ImproveMainCamera-00* */
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = isx006_sensor_power_up, //msm_sensor_power_up
	.sensor_power_down = isx006_sensor_power_down,//msm_sensor_power_down
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_set_fps = isx006_set_fps, /*MTD-MM-SL-FixMMSRecord-00+ */
};

static struct msm_sensor_reg_t isx006_regs = {
	.default_data_type = MSM_CAMERA_I2C_WORD_DATA,
	.init_settings = &isx006_init_conf[0],
	.init_size = ARRAY_SIZE(isx006_init_conf),
	.preload2_settings = isx006_preload2_settings,
	.preload2_size = ARRAY_SIZE(isx006_preload2_settings),
	.preload3_settings = isx006_preload3_settings,
	.preload3_size = ARRAY_SIZE(isx006_preload3_settings),
	/* MTD-MM-SL-Add5M2ndSource-00+{ */
	.preload2_2nd_settings = isx006_preload2_2nd_settings,
	.preload2_2nd_size = ARRAY_SIZE(isx006_preload2_2nd_settings),
	.preload3_2nd_settings = isx006_preload3_2nd_settings,
	.preload3_2nd_size = ARRAY_SIZE(isx006_preload3_2nd_settings),
	.shd_max_settings = isx006_SHD_MAX_settings,
	.shd_max_size = ARRAY_SIZE(isx006_SHD_MAX_settings),
	.shd_min_settings = isx006_SHD_MIN_settings,
	.shd_min_size = ARRAY_SIZE(isx006_SHD_MIN_settings),	
	.shd_typ_settings = isx006_SHD_TYP_settings,
	.shd_typ_size = ARRAY_SIZE(isx006_SHD_TYP_settings),
	.shd_max_2nd_settings = isx006_SHD_MAX_2nd_settings,
	.shd_max_2nd_size = ARRAY_SIZE(isx006_SHD_MAX_2nd_settings),
	.shd_min_2nd_settings = isx006_SHD_MIN_2nd_settings,
	.shd_min_2nd_size = ARRAY_SIZE(isx006_SHD_MIN_2nd_settings),	
	.shd_typ_2nd_settings = isx006_SHD_TYP_2nd_settings,
	.shd_typ_2nd_size = ARRAY_SIZE(isx006_SHD_TYP_2nd_settings),	
	/* MTD-MM-SL-Add5M2ndSource-00+} */
	/* MTD-MM-SL-Add5M2ndSource-00+{ */
	.reload_preload3_2nd_settings = isx006_reload_preload3_2nd_settings,
	.reload_preload3_2nd_size = ARRAY_SIZE(isx006_reload_preload3_2nd_settings),	
	/* MTD-MM-SL-Add5M2ndSource-00+} */
	.mode_settings = &isx006_confs[0],
	.no_effect_settings = &isx006_no_effect_confs[0],
	.output_settings = &isx006_dimensions[0],
	.num_conf = ARRAY_SIZE(isx006_confs),
};

static struct msm_sensor_ctrl_t isx006_s_ctrl = {
	.msm_sensor_reg = &isx006_regs,
	.msm_sensor_v4l2_ctrl_info = isx006_v4l2_ctrl_info,
	.num_v4l2_ctrl = ARRAY_SIZE(isx006_v4l2_ctrl_info),
	.sensor_i2c_client = &isx006_sensor_i2c_client,
	.sensor_i2c_addr = 0x78,
	.sensor_output_reg_addr = &isx006_reg_addr,
	.sensor_id_info = &isx006_id_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &isx006_csi_params_array[0],
	.msm_sensor_mutex = &isx006_mut,
	.sensor_i2c_driver = &isx006_i2c_driver,
	.sensor_v4l2_subdev_info = isx006_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(isx006_subdev_info),
	.sensor_v4l2_subdev_ops = &isx006_subdev_ops,
	.func_tbl = &isx006_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivision 5M YUV sensor driver");
MODULE_LICENSE("GPL v2");
