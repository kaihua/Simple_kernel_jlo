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
#include "mt9v115.h" /*MTD-MM-SL-FixMMSRecord-01+ */
/*MTD-MM-SL-FrontCameraCapture-00+{ */
#include <linux/delay.h>//For hr_msleep()
#include <mach/camera.h>
#include "msm.h"
/*MTD-MM-SL-FrontCameraCapture-00+} */

#define SENSOR_NAME "mt9v115"

/*MTD-MM-SL-FrontCameraCapture-00+{ */
#ifdef CONFIG_FIH_HR_MSLEEP
#define cam_msleep hr_msleep
#else
#define cam_msleep msleep
#endif
/*MTD-MM-SL-FrontCameraCapture-00+} */

/*MTD-MM-SL-ImproveFrontCamera-00+{ */
/* HW standby: 1: Enable, 0: Disable */
#define MT9V115_ENABLE_HW_STANDBY 1

bool F_STARTUP = false; 
extern bool STARTUP;/*MTD-MM-SL-ImproveFrontCamera-03+ */
static int csi_config;

struct msm_sensor_ctrl_t *s_ctrl_f; 
/*MTD-MM-SL-ImproveFrontCamera-00+} */

/*MTD-MM-SL-ImproveFrontCamera-03+{ */
enum Exit_Standby_Action {
	ACT_MCLK_ON,
	ACT_EXIT_STANDBY
};
/*MTD-MM-SL-ImproveFrontCamera-03+} */

DEFINE_MUTEX(mt9v115_mut);
static struct msm_sensor_ctrl_t mt9v115_s_ctrl;

extern int32_t msm_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);
extern int32_t msm_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);

static struct msm_camera_i2c_reg_conf mt9v115_start_settings[] = {
	{0x8400, 0x02, MSM_CAMERA_I2C_BYTE_DATA, MSM_CAMERA_I2C_CMD_WRITE},
	{0x8401, 0x02, MSM_CAMERA_I2C_BYTE_DATA, MSM_CAMERA_I2C_CMD_POLL},
};

static struct msm_camera_i2c_reg_conf mt9v115_stop_settings[] = {
	{0x8400, 0x01, MSM_CAMERA_I2C_BYTE_DATA, MSM_CAMERA_I2C_CMD_WRITE},
	{0x8401, 0x01, MSM_CAMERA_I2C_BYTE_DATA, MSM_CAMERA_I2C_CMD_POLL},
};

static struct v4l2_subdev_info mt9v115_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
	/* more can be supported, to be added later */
};


static struct msm_camera_i2c_conf_array mt9v115_init_conf[] = {
	{&mt9v115_recommend_settings[0],
	ARRAY_SIZE(mt9v115_recommend_settings), 64, MSM_CAMERA_I2C_WORD_DATA},
	{&mt9v115_recommend_part2_settings[0],
	ARRAY_SIZE(mt9v115_recommend_part2_settings), 32, MSM_CAMERA_I2C_WORD_DATA},
	{&mt9v115_recommend_part3_settings[0],
	ARRAY_SIZE(mt9v115_recommend_part3_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
};

static struct msm_camera_i2c_conf_array mt9v115_confs[] = {
	{&mt9v115_full_settings[0],
	ARRAY_SIZE(mt9v115_full_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
};

static struct msm_sensor_output_info_t mt9v115_dimensions[] = {
	/* 30 fps */
	{
		.x_output = 0x280, //640
		.y_output = 0x1E0, //480
		.line_length_pclk = 0x02D6,   //726
		.frame_length_lines = 0x01F9, //505
		.vt_pixel_clk = 9216000,
		.op_pixel_clk = 9216000,
		.binning_factor = 1,
	},
};

static struct msm_camera_i2c_reg_conf mt9v115_no_effect[] = {
	/*
	{0x81, 0x00, 0x00, 0x00, 0xDF},
	{0x28, 0x00,},
	{0xd2, 0x00,},
	{0xda, 0x80,},
	{0xdb, 0x80,},
	*/
};

static struct msm_camera_i2c_conf_array mt9v115_no_effect_confs[] = {
	{&mt9v115_no_effect[0],
	ARRAY_SIZE(mt9v115_no_effect), 0,
	MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},
};

/*MTD-MM-SL-FixMMSRecord-01+{ */
static int32_t mt9v115_set_fps(struct msm_sensor_ctrl_t *s_ctrl, uint16_t fps)
{
    int32_t rc = 0;
    
    printk ("mt9v115_set_fps ------- value =%d\n", fps); 
    if(fps <=15){
		rc = msm_camera_i2c_write_tbl(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->fps_15_settings,
				s_ctrl->msm_sensor_reg->fps_15_size,
				s_ctrl->msm_sensor_reg->default_data_type);
        if (rc < 0) {
            pr_err("mt9v115_set_fps: mt9v115_i2c_write_table(mt9v115_fps_15_settings) failed !\n");
        }
    }
    else{
		rc = msm_camera_i2c_write_tbl(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->fps_30_settings,
				s_ctrl->msm_sensor_reg->fps_30_size,
				s_ctrl->msm_sensor_reg->default_data_type);
        if (rc < 0) {
            pr_err("mt9v115_set_fps: mt9v115_i2c_write_table(mt9v115_fps_30_settings) failed !\n");
        }    
    }

	return rc;

}
/*MTD-MM-SL-FixMMSRecord-01+} */

int32_t mt9v115_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	printk("mt9v115_sensor_start_stream: START !\n"); 
	
	rc = msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->start_stream_conf,
		s_ctrl->msm_sensor_reg->start_stream_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
	if (rc < 0){
		pr_err("mt9v115_sensor_start_stream: msm_camera_i2c_write_tbl failed !\n");
        goto error;

	}

	printk("mt9v115_sensor_start_stream: Start streaming success.\n");   
	return rc;

error:
    pr_err("mt9v115_sensor_start_stream: Failed !\n"); 
    return rc;	
}

/*MTD-MM-SL-ImproveFrontCamera-00+{ */
int32_t mt9v115_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	printk("mt9v115_sensor_stop_stream: Stop streaming Start.\n"); 
	rc = msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->stop_stream_conf,
		s_ctrl->msm_sensor_reg->stop_stream_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
	if (rc < 0){
		printk("mt9v115_sensor_stop_stream: msm_camera_i2c_write_tbl failed! \n");
		return rc;	
	}

	printk("mt9v115_sensor_stop_stream: Stop streaming success.\n");   
	return rc;	
}
/*MTD-MM-SL-ImproveFrontCamera-00+} */

int32_t mt9v115_sensor_init_setting(struct msm_sensor_ctrl_t *s_ctrl, int update_type, int res)
{
	int32_t rc;
	
	pr_err("mt9v115_sensor_init_setting, update_type = %d.\n", update_type);

	rc = msm_sensor_write_all_conf_array(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->init_settings,
		s_ctrl->msm_sensor_reg->init_size);
	if (rc < 0){
		pr_err("mt9v115_sensor_init_setting: msm_sensor_write_all_conf_array failed !\n");
        goto error;
	}	
	printk("mt9v115_sensor_init_setting: Success.\n"); 
	return rc;
	
error:
    pr_err("mt9v115_sensor_init_setting: Failed !\n");
    return rc;	
}

/*MTD-MM-SL-ImproveFrontCamera-01*{ */
/*MTD-MM-SL-ImproveFrontCamera-00+{ */
int mt9v115_enter_standby(struct msm_sensor_ctrl_t *s_ctrl_f)
{
    int rc = 0;
    int i = 0;
    uint16_t read_value;
	struct msm_camera_sensor_info *mt9v115_info = NULL;
	mt9v115_info = s_ctrl_f->sensordata;
	
    printk("mt9v115_enter_standby, Start! \n");    

    rc = msm_camera_i2c_read(s_ctrl_f->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);   
	if (rc < 0){
		printk("mt9v115_enter_standby: msm_camera_i2c_read (REG 0x0018) failed! \n");
		goto error;
    }
	
    read_value = read_value | 0x0001;  

    rc = msm_camera_i2c_write(s_ctrl_f->sensor_i2c_client, 0x0018, read_value, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        printk("mt9v115_enter_standby: msm_camera_i2c_write (REG 0x0018) failed !\n");
		goto error;
    }    

    for (i = 0; i < 80; i++)
    {
    	/*MTD-MM-SL-FixCoverity-00*{ */
        rc = msm_camera_i2c_read(s_ctrl_f->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
        	pr_err("mt9v115_enter_standby: msm_camera_i2c_read (REG 0x0018) failed !\n");
			goto error;
		}
		/*MTD-MM-SL-FixCoverity-00*} */
        read_value = read_value >> 14;
        read_value = read_value & 0x0001;
        
        printk("mt9v115_enter_standby: polling standy mode i = %d\n", i);
        if(read_value == 0x0001)
            break;
        cam_msleep(10);
     }
    cam_msleep(10);

	
    rc = fih_disable_mclk(s_ctrl_f);
    if (rc < 0) {
        printk ("mt9v115_enter_standby: disable mclk fail \n");
		goto error;
    }
    cam_msleep(5);

	
	printk("mt9v115_enter_standby, End! \n");	
    return 0;

error:
    printk("mt9v115_enter_standby: Failed !\n");
    return rc;		
}
/*MTD-MM-SL-ImproveFrontCamera-01*} */

/*MTD-MM-UW-fix CTS preview fail-00*{ */
#if 0
/*MTD-MM-SL-CantSleepInSuspend-00*{ */
int mt9v115_exit_standby(struct msm_sensor_ctrl_t *s_ctrl, enum Exit_Standby_Action act)
{
    int rc = 0;
    int i = 0;
    uint16_t read_value;
	struct msm_camera_sensor_info *mt9v115_info = NULL;
	mt9v115_info = s_ctrl->sensordata;
	
    printk("mt9v115_exit_standby \n");    

	switch (act) {
	case ACT_MCLK_ON:
        {
            printk("mt9v115_exit_standby: case ACT_MCLK_ON \n");
			
            rc = fih_enable_mclk(s_ctrl);
            if (rc < 0) {
                goto error;
            }
            cam_msleep(50); 

            rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);
            if (rc < 0)
                printk("mt9v115_exit_standby: Can not read camera om status after MCLK enable ..\n"); 
            else
                printk("mt9v115_exit_standby: [Success] - (MCLK ON)\n");
        }
        break;		

	case ACT_EXIT_STANDBY:
		{

			printk("mt9v115_exit_standby: case ACT_EXIT_STANDBY \n");
			
			rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);
		    if (rc < 0){
				printk("mt9v115_exit_standby: msm_camera_i2c_read (REG 0x0018) failed! \n");
				goto error;
		    }

		    read_value = read_value & 0xFFFE;

		    rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0018, read_value, MSM_CAMERA_I2C_WORD_DATA);
		    if (rc < 0) {
		        printk("mt9v115_exit_standby:msm_camera_i2c_write (REG 0x0018) failed !\n");
				goto error;
		    }    

		    for (i = 0; i < 80; i++)
		    {
		    	/*MTD-MM-SL-FixCoverity-00*{ */
		        rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);
				if (rc < 0) {
		        	pr_err("mt9v115_exit_standby:msm_camera_i2c_read (REG 0x0018) failed !\n");
					goto error;
				}
				/*MTD-MM-SL-FixCoverity-00*} */
		        read_value = read_value >> 14;
		        read_value = read_value & 0x0001;

		        printk("mt9v115_exit_standby: polling standy mode i = %d\n", i);
		        if(read_value == 0x0000)
		            break;
		        cam_msleep(10);
		     }
		}
		break;
	}

	printk("mt9v115_exit_standby, End \n");    
    return rc;

error:
    printk("mt9v115_exit_standby: Failed !\n");
    return rc;	
}
/*MTD-MM-SL-ImproveFrontCamera-00+} */
/*MTD-MM-SL-CantSleepInSuspend-00*} */
#endif

/*MTD-MM-SL-ImproveFrontCamera-05*{ */
int mt9v115_exit_standby(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    uint16_t read_value;
    struct msm_camera_sensor_info *mt9v115_info = NULL;
    mt9v115_info = s_ctrl->sensordata;
	
    printk("mt9v115_exit_standby \n");    

    rc = fih_enable_mclk(s_ctrl);
    if (rc < 0) {
        goto error;
    }
    printk("mt9v115_exit_standby: [Success] - (MCLK ON)\n");
    cam_msleep(10); //50

    rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x0018, &read_value, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0){
		printk("mt9v115_exit_standby: msm_camera_i2c_read (REG 0x0018) failed! \n");
		goto error;
    }

    read_value = read_value & 0xFFFE;

    rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x0018, read_value, MSM_CAMERA_I2C_WORD_DATA);
    if (rc < 0) {
        printk("mt9v115_exit_standby:msm_camera_i2c_write (REG 0x0018) failed !\n");
		goto error;
    }    
    printk("mt9v115_exit_standby, End \n");    
    return rc;

error:
    printk("mt9v115_exit_standby: Failed !\n");
    return rc;	
}
/*MTD-MM-UW-fix CTS preview fail-00*} */
/*MTD-MM-SL-ImproveFrontCamera-05*} */

/*MTD-MM-SL-ImproveFrontCamera-03+{ */
int32_t mt9v115_normal_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	s_ctrl_f = (struct msm_sensor_ctrl_t *)s_ctrl;
	

	printk("mt9v115_normal_power_up: F_STARTUP == 0. \n");

	/* I/O & Analog power up */  
	//CAM_VGA_STBY
	printk("mt9v115_normal_power_up: Pull high CAM_VGA_STBY pin (%d) \n", data->sensor_f_pwd);
	rc =  gpio_request(data->sensor_f_pwd, "CAM_VGA_STBY");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: gpio_request (%d) failed !.\n", data->sensor_f_pwd);
	}    
	rc = gpio_direction_output(data->sensor_f_pwd, 0);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: Pull low CAM_VGA_STBY pin failed !\n");
       	goto gpio_sensor_f_pwd_fail;
    }
	gpio_free(data->sensor_f_pwd);

	//CAM_VDDIO_V1P8
	printk("mt9v115_normal_power_up: Pull high CAM_VDDIO_V1P8 pin (%d) \n", data->vreg_v1p8);
	rc =  gpio_request(data->vreg_v1p8, "CAM_VDDIO_V1P8");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: gpio_request (%d) failed !.\n", data->vreg_v1p8);
	}
	rc = gpio_direction_output(data->vreg_v1p8, 1);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: Pull high CAM_VDDIO_V1P8 pin failed !\n");
       	goto gpio_v1p8_fail;
    }
	gpio_free(data->vreg_v1p8);

	//CAM_VDDIO_V2P8
	printk("mt9v115_normal_power_up: Pull high CAM_VDDIO_V2P8 pin (%d) \n", data->vreg_v2p8);
	rc =  gpio_request(data->vreg_v2p8, "CAM_VDDIO_V2P8");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: gpio_request (%d) failed !.\n", data->vreg_v2p8);
	}
	rc = gpio_direction_output(data->vreg_v2p8, 1);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: Pull high CAM_VDDIO_V2P8 pin failed !\n");
       	goto gpio_v2p8_fail;
    }
	gpio_free(data->vreg_v2p8);

	/* 5M camera enter standby */
	rc =  gpio_request(data->sensor_pwd, "CAM_5M_RSTN");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: gpio_request (%d) failed !.\n", data->sensor_pwd);
	}
	rc = gpio_direction_output(data->sensor_pwd, 0);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_up: Pull low (%d) pin failed !\n", data->sensor_pwd);
       	goto gpio_sensor_pwd_fail;
    }
    gpio_free(data->sensor_pwd);

	//CAM_MCLK	
	rc = fih_enable_mclk(s_ctrl);
    if (rc < 0) {
		pr_err("mt9v115_normal_power_up: Pull high CAN_MCLK pin failed !\n");
        goto error;
    }	
		
	printk("mt9v115_normal_power_up: End. \n");  
    return rc;

gpio_sensor_f_pwd_fail:
	gpio_free(data->sensor_f_pwd);
	goto error;

gpio_sensor_pwd_fail:
	gpio_free(data->sensor_pwd);
	goto error;
	
gpio_v1p8_fail:
	gpio_free(data->vreg_v1p8);
	goto error;
	
gpio_v2p8_fail:
	gpio_free(data->vreg_v2p8);
	goto error;

error:
	F_STARTUP = 0;
	STARTUP = 0;
    pr_err("mt9v115_normal_power_up: failed !, rc = %d.\n", rc);
    return rc;	

}

int32_t mt9v115_normal_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl_f->sensordata;
	int rc = 0;


	printk("mt9v115_normal_power_down: F_STARTUP == 0. \n");   

	//CAM_VGA_STBY
	rc =  gpio_request(data->sensor_f_pwd, "CAM_VGA_STBY");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_down: gpio_request (%d) failed !.\n", data->sensor_f_pwd);
	}
	rc = gpio_direction_output(data->sensor_f_pwd, 0);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_down: Pull low CAM_VGA_STBY pin failed !\n");
       	goto gpio_sensor_f_pwd_fail;
    }
	gpio_free(data->sensor_f_pwd);

	//CAM_VDDIO_V1P8
	rc =  gpio_request(data->vreg_v1p8, "CAM_VDDIO_V1P8");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_down: gpio_request (%d) failed !.\n", data->vreg_v1p8);
	}
	rc = gpio_direction_output(data->vreg_v1p8, 0);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_down: Pull low CAM_VDDIO_V1P8 pin failed !\n");
       	goto gpio_v1p8_fail;
    }
	gpio_free(data->vreg_v1p8);

	cam_msleep(5);

	//CAM_VDDIO_V2P8
	rc =  gpio_request(data->vreg_v2p8, "CAM_VDDIO_V2P8");
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_down: gpio_request (%d) failed !.\n", data->vreg_v2p8);
	}
	rc = gpio_direction_output(data->vreg_v2p8, 0);
	if (rc < 0) {
       	pr_err("mt9v115_normal_power_down: Pull low CAM_VDDIO_V2P8 pin failed !\n");
       	goto gpio_v2p8_fail;
    }
	gpio_free(data->vreg_v2p8);

	//CAM_MCLK
	rc = fih_disable_mclk(s_ctrl_f);
    if (rc < 0) {
		pr_err("mt9v115_normal_power_down: Pull low CAN_MCLK pin failed !\n");
        goto error;
    }
	cam_msleep(100);	

	
	printk ("mt9v115_normal_power_down: End. \n");  
    return rc;

gpio_sensor_f_pwd_fail:
	gpio_free(data->sensor_f_pwd);
	goto error;

gpio_v1p8_fail:
	gpio_free(data->vreg_v1p8);
	goto error;

gpio_v2p8_fail:
	gpio_free(data->vreg_v2p8);
	goto error;
	
error:
    pr_err("mt9v115_normal_power_down: failed ! rc = %d.\n", rc);
	F_STARTUP = 0;
	STARTUP = 0;
    return rc;	

}

int32_t mt9v115_power_up_reset(struct msm_sensor_ctrl_t *s_ctrl, int mode)
{
	int rc = 0;	
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

	printk("mt9v115_power_up_reset: Start.\n"); 

	switch (mode){
	case 0:
		printk("mt9v115_power_up_reset: case 0 \n"); 

		//01. Re-set flag variables.
		F_STARTUP = 0;
		STARTUP = 0;

		//03. Power off 
		rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		if (rc < 0) {
			printk("mt9v115_power_up_reset: sensor_power_down fail \n"); 
		}

		cam_msleep(30);

		//03. Power on 
		rc = mt9v115_normal_power_up(s_ctrl);
		if (rc < 0) {
			printk("mt9v115_power_up_reset: sensor_power_up fail \n"); 
		}
		
		break;
		
	case 1:	
		printk("mt9v115_power_up_reset: case 1 \n"); 
		
		//01. Re-set flag variables.
		F_STARTUP = 0;
		STARTUP = 0;

		//02. Re-config main camera pin (CAM_5M_STBYN & CAM_5M_RSTN ).
		rc =  gpio_request(data->sensor_pwd, "CAM_5M_STBYN");
		if (rc < 0) {
			pr_err("mt9v115_power_up_reset: gpio_request (%d) failed !.\n", data->sensor_pwd);
		}
		rc = gpio_direction_output(data->sensor_pwd, 0);
		if (rc < 0) {
       		pr_err("mt9v115_power_up_reset: Pull low CAM_5M_STBYN pin failed !\n");
			goto gpio_sensor_pwd_fail;
    	}
		gpio_free(data->sensor_pwd);
		
		rc =  gpio_request(data->sensor_reset, "CAM_5M_RSTN");
		if (rc < 0) {
			pr_err("mt9v115_power_up_reset: gpio_request (%d) failed !.\n", data->sensor_reset);
		}
		rc = gpio_direction_output(data->sensor_reset, 0);
		if (rc < 0) {
	       	pr_err("mt9v115_power_up_reset: Pull low CAM_5M_RSTN pin failed !\n");
			goto gpio_sensor_reset_fail;
	    }
		gpio_free(data->sensor_reset);

		//03. Power off ( F_STARTUP = 0).
		rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		if (rc < 0) {
	       	pr_err("mt9v115_power_up_reset: sensor_power_down failed !\n");
		}
		
		cam_msleep(200);//Wait power off done.

		//04. Power on.
		rc = mt9v115_normal_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("mt9v115_power_up_reset: sensor_power_up failed !\n");
		}

		//05. init setting.
		rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl, MSM_SENSOR_REG_INIT, 0);
		if (rc < 0) {
			pr_err("mt9v115_power_up_reset: init setting failed !\n");
		}

		//06. Config CSI
		rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl, MSM_SENSOR_UPDATE_PERIODIC, RES_PREVIEW); 
		if (rc < 0) {
			pr_err("mt9v115_power_up_reset: csi config failed !\n");
		}
		
		break;
	}

	printk("mt9v115_power_up_reset: End.\n"); 
	return rc;	

gpio_sensor_pwd_fail:
	gpio_free(data->sensor_pwd);

gpio_sensor_reset_fail:
		gpio_free(data->sensor_reset);

	return rc;
}

int32_t mt9v115_power_down_reset(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;	

	printk("mt9v115_power_down_reset: Start.\n");  
	//01. Re-set flag variables.
	F_STARTUP = 0;
	STARTUP = 0;

	//02. Power off.
	rc = mt9v115_normal_power_down(s_ctrl);
	if (rc < 0) {
       	pr_err("mt9v115_power_down_reset: sensor_power_down failed !\n");
	}

	printk("mt9v115_power_down_reset: End.\n"); 
	return rc;	
}
/*MTD-MM-SL-ImproveFrontCamera-03+} */

/*MTD-MM-SL-ImproveFrontCamera-05*{ */
/*MTD-MM-SL-ImproveFrontCamera-04*{ */
/*MTD-MM-SL-ImproveFrontCamera-03*{ */
/*MTD-MM-SL-ImproveFrontCamera-00*{ */
/*MTD-MM-SL-SwitchFrontCameraeNotSmooth-00+{ */
int32_t mt9v115_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	
	printk("%s: update_type = %d \n", __func__, update_type);
	
	if (update_type == MSM_SENSOR_REG_INIT) {
		printk("mt9v115_sensor_setting: Register INIT\n");
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);

		//main-init setting
		rc = s_ctrl->func_tbl->sensor_init_setting(s_ctrl, update_type,res);
		if (rc < 0){
			pr_err("mt9v115_sensor_setting: sensor_init_setting failed !\n");
			return rc;
		}
		printk("mt9v115_sensor_setting: main-init setting done \n");

		//sub-init setting
		rc = msm_sensor_write_conf_array(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->mode_settings, res);
		if (rc < 0){
			pr_err("mt9v115_sensor_setting: sub-init setting failed !\n");
			return rc;
		}
		printk("mt9v115_sensor_setting: full (sub-init) setting done \n");

		csi_config = 0;	
		
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		printk("mt9v115_sensor_setting: PERIODIC : %d\n", res);

		if ((res == RES_PREVIEW) && (csi_config == false)){
			printk("mt9v115_sensor_setting: case MSM_SENSOR_UPDATE_PERIODIC \n");

			/* stop streaming */
			rc = s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			if (rc < 0 ){
				printk("mt9v115_sensor_setting: sensor_stop_stream failed! \n");
				return rc;
			}
			//cam_msleep(30);
			
			if(csi_config == false){
				s_ctrl->curr_csic_params = s_ctrl->csic_params[res];
				printk("mt9v115_sensor_setting: CSI config in progress\n");
				v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
					NOTIFY_CSIC_CFG,
					s_ctrl->curr_csic_params);
				printk("mt9v115_sensor_setting: CSI config is done\n");	
				mb();
				cam_msleep(10); //20
				csi_config = 1;

				v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_PCLK_CHANGE,
				&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
				
				if (F_STARTUP == 0){
					printk("mt9v115_sensor_setting: F_STARTUP = 0\n");	
					F_STARTUP = MT9V115_ENABLE_HW_STANDBY;  /*MTD-MM-SL-CTSFail-00* */
				}
			}	 
			/* start streaming */
			rc = s_ctrl->func_tbl->sensor_start_stream_1(s_ctrl);
			if (rc < 0 ){
				printk("mt9v115_sensor_setting: sensor_start_stream failed! \n");
				return rc;
			}
		}
	}
	printk("mt9v115_sensor_setting: Done\n");
	return rc;
}
/*MTD-MM-SL-SwitchFrontCameraeNotSmooth-00+} */
/*MTD-MM-SL-ImproveFrontCamera-00*} */
/*MTD-MM-SL-ImproveFrontCamera-03*} */
/*MTD-MM-SL-ImproveFrontCamera-04*} */
/*MTD-MM-SL-ImproveFrontCamera-05*} */

/*MTD-MM-SL-ImproveFrontCamera-00+{ */
int32_t mt9v115_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	int32_t rc = 0;
	s_ctrl->fps_divider = Q10;
	s_ctrl->cam_mode = MSM_SENSOR_MODE_INVALID;

	printk("mt9v115_sensor_mode_init, F_STARTUP = %d \n", F_STARTUP);
	if (mode != s_ctrl->cam_mode) {
		printk("mt9v115_sensor_mode_init\n");
		s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->cam_mode = mode;

		if (F_STARTUP == 0){
			if (s_ctrl->is_csic ||
				!s_ctrl->sensordata->csi_if)
				rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
					MSM_SENSOR_REG_INIT, 0);
			else
				rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
					MSM_SENSOR_REG_INIT, 0);
		}
	}
	return rc;
}

int32_t mt9v115_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;


	printk("mt9v115_sensor_match_id\n");

	rc = msm_camera_i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_id_info->sensor_id_reg_addr, &chipid,
		MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: %s: read sensor id failed\n", __func__,
				s_ctrl->sensordata->sensor_name);
			return rc;
		}

	printk("mt9v115_sensor_match_id: sensor id: %x\n", chipid);
	if ((chipid != s_ctrl->sensor_id_info->sensor_id)) { 
		pr_err("mt9v115_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}
/*MTD-MM-SL-ImproveFrontCamera-00+} */

/*MTD-MM-UW-fix CTS preview fail-00*{ */
/*MTD-MM-SL-ImproveFrontCamera-03*{ */
/*MTD-MM-SL-ImproveFrontCamera-01*{ */
/*MTD-MM-SL-ImproveFrontCamera-00*{ */
int32_t mt9v115_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	s_ctrl_f = (struct msm_sensor_ctrl_t *)s_ctrl; /*MTD-MM-SL-ImproveFrontCamera-02+ */
	
	printk("mt9v115_sensor_power_up: Start.\n");  

	if(F_STARTUP == 1){
		printk("mt9v115_sensor_power_up: F_STARTUP = 1, move mt9v115_exit_standby to mt9v115_sensor_setting. \n");
		
		rc = mt9v115_exit_standby(s_ctrl);
		if (rc < 0) {
			printk ("mt9v115_sensor_power_up: mt9v115_exit_standby fail\n");    
			rc = mt9v115_power_up_reset(s_ctrl, 0);
			if (rc <0){
				pr_err("mt9v115_sensor_power_up: mt9v115_power_down_reset fail! \n");
				goto error;
			}
		}
		
		printk("mt9v115_sensor_power_up: mt9v115_exit_standby Success! \n");
		goto done;	
	}

	rc = mt9v115_normal_power_up(s_ctrl);
        if (rc < 0) {
        	pr_err("mt9v115_sensor_power_up: mt9v115_normal_power_up failed !\n");
        	goto error;
        }
	goto done;

done:	
	
	printk("mt9v115_sensor_power_up: End. \n");  
    return rc;

error:
	F_STARTUP = 0;
	STARTUP = 0;
    pr_err("mt9v115_sensor_power_up: failed !, rc = %d.\n", rc);
    return rc;
}
/*MTD-MM-SL-ImproveFrontCamera-00*} */
/*MTD-MM-SL-ImproveFrontCamera-01*} */
/*MTD-MM-UW-fix CTS preview fail-00*} */

/*MTD-MM-SL-ImproveFrontCamera-01*{ */
int32_t mt9v115_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl_f)
{
	int rc = 0;

	printk("mt9v115_sensor_power_down: Start.\n");  

	if(F_STARTUP == 1){
		printk("mt9v115_sensor_power_down: F_STARTUP == 1. \n"); 
		
        rc = mt9v115_enter_standby(s_ctrl_f);
		if (rc < 0){
			printk("mt9v115_sensor_power_down: mt9v115_enter_standby fail, need re-power off. \n");
            rc = mt9v115_power_down_reset(s_ctrl_f);
			if (rc <0){
				pr_err("mt9v115_sensor_power_down: mt9v115_power_down_reset fail! \n");
				goto error;
			}
		}
		
		printk("mt9v115_sensor_power_down: mt9v115_enter_standby Success! \n");
		goto done;
	}

	rc = mt9v115_normal_power_down(s_ctrl_f);
	if (rc < 0) {
       	pr_err("mt9v115_sensor_power_down: mt9v115_normal_power_down failed !\n");
       	goto error;
    }
	goto done;	

done:	
	csi_config = 0;
	
	printk ("mt9v115_sensor_power_down: End. \n");  
    return rc;
	
error:
    pr_err("mt9v115_sensor_power_down: failed ! rc = %d.\n", rc);
	F_STARTUP = 0;
	STARTUP = 0;
    return rc;		
}
/*MTD-MM-SL-ImproveFrontCamera-01*} */
/*MTD-MM-SL-ImproveFrontCamera-03*} */

/*MTD-MM-SL-QuicklyUnlockCameraFail-00*{ */
/*MTD-MM-SL-ImproveFrontCamera-01*{ */
/*MTD-MM-SL-ImproveFrontCamera-00+{ */
int mt9v115_suspend(struct i2c_client *client, pm_message_t state)
{
    int rc = 0;	
    
    printk("mt9v115_suspend \n");

    rc = fih_enable_mclk(s_ctrl_f);
    if (rc < 0) {
		pr_err("mt9v115_suspend: Pull high CAN_MCLK pin failed !\n");
        goto error;
    }
	
    cam_msleep(5);

    F_STARTUP  = 0;
    
    rc = mt9v115_sensor_power_down(s_ctrl_f);
    if (rc < 0)
        goto error;

	return 0;
    
error:
    pr_err("mt9v115_suspend: failed ! rc = %d.\n", rc);
	return rc;
}

int mt9v115_resume(struct i2c_client *client)
{
     printk("mt9v115_resume \n");  
	 
     return 0;
}
/*MTD-MM-SL-ImproveFrontCamera-00+} */
/*MTD-MM-SL-ImproveFrontCamera-01*} */
/*MTD-MM-SL-QuicklyUnlockCameraFail-00*} */

static struct msm_camera_csi_params mt9v115_csi_params = {
	.data_format = CSI_8BIT,
	.lane_cnt    = 1,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt  = 0x14,
};

static struct msm_camera_csi_params *mt9v115_csi_params_array[] = {
	&mt9v115_csi_params,
};

static struct msm_sensor_output_reg_addr_t mt9v115_reg_addr = {
	.x_output = 0xA000,
	.y_output = 0xA002,
	.line_length_pclk = 0x300C,
	.frame_length_lines = 0x300A,
};

static struct msm_sensor_id_info_t mt9v115_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x2284,
};

static const struct i2c_device_id mt9v115_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&mt9v115_s_ctrl},
	{ }
};

/*MTD-MM-SL-ImproveFrontCamera-00*{ */
static struct i2c_driver mt9v115_i2c_driver = {
	.id_table = mt9v115_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
	//Move to early_suspend, we register early_suspend in msm_sensor_init_module
	.suspend = mt9v115_suspend,  /*MTD-MM-SL-QuicklyUnlockCameraFail-00- */
    .resume = mt9v115_resume,
};
/*MTD-MM-SL-ImproveFrontCamera-00*} */

static struct msm_camera_i2c_client mt9v115_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	int rc = 0;
	CDBG("mt9v115\n");

	rc = i2c_add_driver(&mt9v115_i2c_driver);

	return rc;
}

static struct v4l2_subdev_core_ops mt9v115_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops mt9v115_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops mt9v115_subdev_ops = {
	.core = &mt9v115_subdev_core_ops,
	.video  = &mt9v115_subdev_video_ops,
};

static struct msm_sensor_fn_t mt9v115_func_tbl = {
	.sensor_start_stream_1 = mt9v115_sensor_start_stream, //msm_sensor_start_stream
	.sensor_stop_stream = mt9v115_sensor_stop_stream, //msm_sensor_stop_stream, /*MTD-MM-SL-ImproveFrontCamera-00* */
	.sensor_csi_setting = mt9v115_sensor_setting, //msm_sensor_setting1, /*MTD-MM-SL-SwitchFrontCameraeNotSmooth-00* */
	.sensor_init_setting = mt9v115_sensor_init_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = mt9v115_sensor_mode_init, //msm_sensor_mode_init, /*MTD-MM-SL-ImproveFrontCamera-00* */
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = mt9v115_sensor_power_up,
	.sensor_power_down = mt9v115_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_set_fps = mt9v115_set_fps, /*MTD-MM-SL-FixMMSRecord-01+ */
	.sensor_match_id = mt9v115_sensor_match_id, /*MTD-MM-SL-ImproveFrontCamera-00+ */
};

static struct msm_sensor_reg_t mt9v115_regs = {
	.default_data_type = MSM_CAMERA_I2C_WORD_DATA,
	.start_stream_conf = mt9v115_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(mt9v115_start_settings),
	.stop_stream_conf = mt9v115_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(mt9v115_stop_settings),
	.init_settings = &mt9v115_init_conf[0],
	.init_size = ARRAY_SIZE(mt9v115_init_conf),
	.mode_settings = &mt9v115_confs[0],
	.no_effect_settings = &mt9v115_no_effect_confs[0],
	.output_settings = &mt9v115_dimensions[0],
	.num_conf = ARRAY_SIZE(mt9v115_confs),
	/*MTD-MM-SL-FixMMSRecord-01+{ */
	.fps_15_settings = mt9v115_fps_15_settings,
	.fps_15_size = ARRAY_SIZE(mt9v115_fps_15_settings),	
	.fps_30_settings = mt9v115_fps_30_settings,
	.fps_30_size = ARRAY_SIZE(mt9v115_fps_30_settings),		
	/*MTD-MM-SL-FixMMSRecord-01+} */
};

static struct msm_sensor_ctrl_t mt9v115_s_ctrl = {
	.msm_sensor_reg = &mt9v115_regs,
#if 0
	.msm_sensor_v4l2_ctrl_info = mt9v115_v4l2_ctrl_info,
	.num_v4l2_ctrl = ARRAY_SIZE(mt9v115_v4l2_ctrl_info),
#endif
	.sensor_i2c_client = &mt9v115_sensor_i2c_client,
	.sensor_i2c_addr = 0x7A,
	.sensor_output_reg_addr = &mt9v115_reg_addr,
	.sensor_id_info = &mt9v115_id_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &mt9v115_csi_params_array[0],
	.msm_sensor_mutex = &mt9v115_mut,
	.sensor_i2c_driver = &mt9v115_i2c_driver,
	.sensor_v4l2_subdev_info = mt9v115_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mt9v115_subdev_info),
	.sensor_v4l2_subdev_ops = &mt9v115_subdev_ops,
	.func_tbl = &mt9v115_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivision VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");
