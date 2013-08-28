
#include <linux/fih_leds.h>

static const char			*select_func[]		= { "GPIO", "MPP" };
static const char			*current_sink_table[]	= { "5mA", "10mA", "15mA", "20mA", "25mA", "30mA", "35mA", "40mA" };
static struct control_class_device	*file_node_class;
static struct led_data		*leds_data;
static struct led_command_data	*command_data;
static struct led_command_info	command_info;
static struct workqueue_struct	*work_queue;

static void	led_on_off_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, parameter->para1 == LED_TURN_OFF ? msm_data->led_off_level : msm_data->led_on_level );
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s) %s\n", msm_data->msm_pin, data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= parameter->para1;
			smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s) %s, [Pin:LPG]=[%d:%d]\n", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_blinking_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s) does not support blinking\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_BLINKING;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= parameter->para1;
			smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s) blink %s, [Pin:LPG]=[%d:%d]\n", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_fade_in_out_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s) does not support fade in/out\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_FADE_IN_OUT;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= parameter->para1;
			smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s) fade in/out %s, [Pin:LPG]=[%d:%d]\n", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_on_off_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		printk( "LED : %s led, special mode\n", data->name );
	else
		led_on_off_set( data, parameter );
}

static void	led_blinking_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		printk( "LED : %s led, special mode\n", data->name );
	else
		led_blinking_set( data, parameter );
}

static void	led_fade_in_out_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		printk( "LED : %s led, special mode\n", data->name );
	else
		led_fade_in_out_set( data, parameter );
}

static void	led_sw_blinking_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s) does not support blinking\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			hrtimer_cancel( &data->led_time );

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= LED_TURN_OFF;
			smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );

			if( parameter->para1 != LED_TURN_OFF )
			{
				data->led_state		= LED_OFF_STATE;
				data->unit_time_out	= 0;
				hrtimer_start( &data->led_time, ktime_set( LED_SEC( pmic_data->blinking_time1 ), LED_NS( pmic_data->blinking_time1 ) ), HRTIMER_MODE_REL );
			}

			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s) blink %s, [Pin:LPG]=[%d:%d].\n", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_sw_fade_in_out_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s) does not support fade in/out\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			hrtimer_cancel( &data->led_time );

			mutex_lock( &data->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= data->use_hw;
			pmic_data->control	= LED_TURN_OFF;
			smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );

			if( parameter->para1 != LED_TURN_OFF )
			{
				data->led_state		= LED_OFF_STATE;
				data->sw_brightness	= 0;
				data->unit_brightness	= pmic_data->fade_in_out_pwm * MINI_UNIT / FADE_IN_OUT_LEVEL;
				data->unit_time_out	= pmic_data->interval;
				data->sw_led_count	= FADE_IN_OUT_LEVEL;
				hrtimer_start( &data->led_time, ktime_set( LED_SEC( pmic_data->blinking_time2 ), LED_NS( pmic_data->blinking_time2 ) ), HRTIMER_MODE_REL );
			}

			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s) fade in/out %s, [Pin:LPG]=[%d:%d].\n", *( select_func + data->use_hw ), data->name, parameter->para1 == LED_TURN_OFF ? "off" : "on" , pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_sw_blinking_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		printk( "LED : %s led, special mode\n", data->name );
	else
		led_sw_blinking_set( data, parameter );
}

static void	led_sw_fade_in_out_check_mode( struct led_data *data, struct command_parameter *parameter )
{
	if( data->special_mode )
		printk( "LED : %s led, special mode\n", data->name );
	else
		led_sw_fade_in_out_set( data, parameter );
}

static void	led_on_brightness_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set brightness\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->on_off_pwm	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), set PWM(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->on_off_pwm );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_blinking_brightness_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set brightness\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->blinking_pwm1	= parameter->para1;
			pmic_data->blinking_pwm2	= parameter->para2;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), set blinking PWM[%d:%d]\n", *( select_func + data->use_hw ), data->name, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2 );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_fade_in_out_interval_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set interval\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->interval	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), set intervale(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->interval );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_blinking_time_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set blinking time\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->blinking_time1	= parameter->para1;
			pmic_data->blinking_time2	= parameter->para2;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), set blinking time[%d:%d]\n", *( select_func + data->use_hw ), data->name, pmic_data->blinking_time1, pmic_data->blinking_time2 );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_mode_set( struct led_data *data, struct command_parameter *parameter )
{
	mutex_lock( &data->lock );
	data->special_mode	= parameter->para1;
	mutex_unlock( &data->lock );

	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s), %s mode\n", msm_data->msm_pin, data->name, data->special_mode ? "Special" : "Normal" );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			printk( "LED : PMIC %s led(%s), %s mode\n", *( select_func + data->use_hw ), data->name, data->special_mode ? "Special" : "Normal" );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_clk_div_exp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set clk, div & exp\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_clock	= parameter->para1;
			pmic_data->pwm_div_value	= parameter->para2;
			pmic_data->pwm_div_exp	= parameter->para3;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), [CLK:DIV:EXP] = [%d:%d:%d]\n", *( select_func + data->use_hw ), data->name, pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_clk( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set clk\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_clock	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), CLK(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->pwm_clock );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_div( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set div\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_div_value	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), DIV(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->pwm_div_value );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_exp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set exp\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pwm_div_exp	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), EXP(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->pwm_div_exp );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_toggle_loop_ramp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set toggle, loop & ramp\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->toggle_up_down	= parameter->para1;
			pmic_data->ramp_loop	= parameter->para2;
			pmic_data->ramp_up_down	= parameter->para3;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), [Toggle:Loop:Ramp]=[%s:%s:%s]\n", *( select_func + data->use_hw ), data->name, pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No" );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_toggle( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set toggle\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->toggle_up_down	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), Toggle(%s)\n", *( select_func + data->use_hw ), data->name, pmic_data->toggle_up_down ? "Yes" : "No" );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_loop( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set loop\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->ramp_loop	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), Loop(%s)\n", *( select_func + data->use_hw ), data->name, pmic_data->ramp_loop ? "Yes" : "No" );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_ramp( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set ramp\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->ramp_up_down	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), Ramp(%s)\n", *( select_func + data->use_hw ), data->name, pmic_data->ramp_up_down ? "Yes" : "No" );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_fade_in_out_brightness_set( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set fade in/out brightness\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->fade_in_out_pwm	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), set fade in/out PWM[%d]\n", *( select_func + data->use_hw ), data->name, pmic_data->fade_in_out_pwm );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_invert( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set invert\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->invert	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), %s invert\n", *( select_func + data->use_hw ), data->name, pmic_data->invert ? "Enable" : "Disable" );
			break;
		}

		case	LED_HW_PMIC_MPP :
		{
			printk( "LED : PMIC %s led(%s) can't set invert\n", *( select_func + data->use_hw ), data->name );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_current_sink( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set current sink\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		{
			printk( "LED : PMIC %s led(%s) can't set current sink\n", *( select_func + data->use_hw ), data->name );
			break;
		}

		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			if( parameter->para1 >= sizeof( current_sink_table ) / sizeof( *current_sink_table ) )
			{
				printk( "LED : PMIC %s led(%s) can't suppurt this value(%d)\n", *( select_func + data->use_hw ), data->name, parameter->para1 );
				break;
			}

			mutex_lock( &data->lock );
			pmic_data->current_sink	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), current sink(%s)\n", *( select_func + data->use_hw ), data->name, *( current_sink_table + pmic_data->current_sink ) );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_pin( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			msm_data->msm_pin	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s), set pin(%d)\n", msm_data->msm_pin, data->name, msm_data->msm_pin );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->pmic_pin	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), set pin(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->pmic_pin );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_on_off_level( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			mutex_lock( &data->lock );
			msm_data->led_on_level	= parameter->para1;
			msm_data->led_off_level	= parameter->para2;
			mutex_unlock( &data->lock );
			printk( "LED : MSM GPIO-%d(%s), level[on:off]=[%s:%s]\n", msm_data->msm_pin, data->name, msm_data->led_on_level ? "High" : "Low", msm_data->led_off_level ? "High" : "Low" );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			printk( "LED : PMIC %s led(%s) can't set on/off level\n", *( select_func + data->use_hw ), data->name );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_lpg_out( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set LPG out\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->lpg_out	= parameter->para1;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), LPG(%d)\n", *( select_func + data->use_hw ), data->name, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_lut_table_range( struct led_data *data, struct command_parameter *parameter )
{
	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s) can't set range of LUT table\n", msm_data->msm_pin, data->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;

			mutex_lock( &data->lock );
			pmic_data->lut_table_start	= parameter->para1;
			pmic_data->lut_table_end	= parameter->para2;
			mutex_unlock( &data->lock );
			printk( "LED : PMIC %s led(%s), LUT table[%d-%d]\n", *( select_func + data->use_hw ), data->name, pmic_data->lut_table_start, pmic_data->lut_table_end );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_set_on_off_state( struct led_data *data, struct command_parameter *parameter )
{
	mutex_lock( &data->lock );
	data->on_off_state	= parameter->para1;
	mutex_unlock( &data->lock );

	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;

			printk( "LED : MSM GPIO-%d(%s), the state is %s\n", msm_data->msm_pin, data->name, data->on_off_state ? "On" : "Off" );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			printk( "LED : PMIC %s led(%s), the state is %s\n", *( select_func + data->use_hw ), data->name, data->on_off_state ? "On" : "Off" );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void	led_auto_turn_off( struct led_data *data, struct command_parameter *parameter )
{
	hrtimer_cancel( &data->turn_off_time );
	mutex_lock( &data->lock );
	if( parameter->para1 )
		hrtimer_start( &data->turn_off_time, ktime_set( LED_SEC( parameter->para1 ), LED_NS( parameter->para1 ) ), HRTIMER_MODE_REL );
	mutex_unlock( &data->lock );

	switch( data->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &data->detail.gpio_data;
			printk( "LED : MSM GPIO-%d(%s) auto turn off(%dms)\n", msm_data->msm_pin, data->name, parameter->para1 );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &data->detail.pmic_data;
			printk( "LED : PMIC %s led(%s) auto turn off(%dms), [Pin:LPG]=[%d:%d].\n", *( select_func + data->use_hw ), data->name, parameter->para1, pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", data->use_hw );
			break;
	}
}

static void led_sched_blink( struct work_struct *work )
{
	struct led_data		*sw_led		= container_of( work, struct led_data, work_blink );
	struct led_pmic_data	*pmic_data	= &sw_led->detail.pmic_data;
	unsigned int		timeout;

	hrtimer_cancel( &sw_led->led_time );

	mutex_lock( &sw_led->lock );
	pmic_data->command	= SMEM_CMD_LED_ON_OFF;
	pmic_data->hardware	= sw_led->use_hw;
	pmic_data->control	= LED_TURN_ON;
	pmic_data->on_off_pwm	= sw_led->led_state == LED_OFF_STATE ? pmic_data->blinking_pwm2 : pmic_data->blinking_pwm1;
	smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );

	timeout	= sw_led->led_state == LED_OFF_STATE ? pmic_data->blinking_time2 : pmic_data->blinking_time1;
	sw_led->led_state	= sw_led->led_state == LED_OFF_STATE ? LED_ON_STATE : LED_OFF_STATE;
	hrtimer_start( &sw_led->led_time, ktime_set( LED_SEC( timeout ), LED_NS( timeout ) ), HRTIMER_MODE_REL );
	mutex_unlock( &sw_led->lock );
}

static void led_sched_fade_in_out( struct work_struct *work )
{
	struct led_data		*sw_led		= container_of( work, struct led_data, work_fade_in_out );
	struct led_pmic_data	*pmic_data	= &sw_led->detail.pmic_data;
	unsigned int		timeout;

	hrtimer_cancel( &sw_led->led_time );

	mutex_lock( &sw_led->lock );
	pmic_data->command	= SMEM_CMD_LED_ON_OFF;
	pmic_data->hardware	= sw_led->use_hw;
	pmic_data->control	= LED_TURN_ON;
	pmic_data->on_off_pwm	= sw_led->sw_brightness / MINI_UNIT;
	smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );

	if( sw_led->sw_led_count-- )
	{
		timeout	= sw_led->unit_time_out;

		if( sw_led->led_state == LED_OFF_STATE )
			sw_led->sw_brightness	= sw_led->sw_brightness + sw_led->unit_brightness > MAX_PWM * MINI_UNIT ? MAX_PWM * MINI_UNIT : sw_led->sw_brightness + sw_led->unit_brightness;
		else
			sw_led->sw_brightness	= sw_led->sw_brightness - sw_led->unit_brightness < MIN_PWM * MINI_UNIT ? MIN_PWM * MINI_UNIT : sw_led->sw_brightness - sw_led->unit_brightness;
	}
	else
	{
		sw_led->sw_led_count	= FADE_IN_OUT_LEVEL;
		timeout	= sw_led->led_state == LED_OFF_STATE ? pmic_data->blinking_time1 : pmic_data->blinking_time2;
		sw_led->led_state	= sw_led->led_state == LED_OFF_STATE ? LED_ON_STATE : LED_OFF_STATE;
	}

	hrtimer_start( &sw_led->led_time, ktime_set( LED_SEC( timeout ), LED_NS( timeout ) ), HRTIMER_MODE_REL );
	mutex_unlock( &sw_led->lock );
}

static void led_sched_turn_off( struct work_struct *work )
{
	struct led_data		*turn_off_led	= container_of( work, struct led_data, work_turn_off );

	hrtimer_cancel( &turn_off_led->turn_off_time );

	switch( turn_off_led->use_hw )
	{
		case	LED_HW_MSM_GPIO :
		{
			struct led_gpio_data	*msm_data	= &turn_off_led->detail.gpio_data;

			gpio_direction_output( msm_data->msm_pin, msm_data->led_off_level );
			printk( "LED : MSM GPIO-%d(%s) timeout, turn off\n", msm_data->msm_pin, turn_off_led->name );
			break;
		}

		case	LED_HW_PMIC_GPIO :
		case	LED_HW_PMIC_MPP :
		{
			struct led_pmic_data	*pmic_data	= &turn_off_led->detail.pmic_data;

			mutex_lock( &turn_off_led->lock );
			pmic_data->command	= SMEM_CMD_LED_ON_OFF;
			pmic_data->hardware	= turn_off_led->use_hw;
			pmic_data->control	= LED_TURN_OFF;
			smem_proc_oem_light_control( ( int* )pmic_data, sizeof( struct led_pmic_data ) );
			mutex_unlock( &turn_off_led->lock );
			printk( "LED : PMIC %s led(%s) timeout, turn off, [Pin:LPG]=[%d:%d].\n", *( select_func + turn_off_led->use_hw ), turn_off_led->name, pmic_data->pmic_pin, pmic_data->lpg_out );
			break;
		}

		default :
			printk( KERN_ERR "LED : Does not support this H/W type(%d)\n", turn_off_led->use_hw );
			break;
	}
}

static enum hrtimer_restart led_timeout( struct hrtimer *timer )
{
	struct  led_data	*sw_led	= container_of( timer, struct led_data, led_time );

	if( sw_led->unit_time_out )
		queue_work( work_queue, &sw_led->work_fade_in_out );
	else
		queue_work( work_queue, &sw_led->work_blink );

	return	HRTIMER_NORESTART;
}

static enum hrtimer_restart led_turn_off( struct hrtimer *timer )
{
	struct  led_data	*turn_off_led	= container_of( timer, struct led_data, turn_off_time );

	queue_work( work_queue, &turn_off_led->work_turn_off );
	return	HRTIMER_NORESTART;
}

static int	prepare_commands( void )
{

	struct led_command_source	led_command_list[] =
	{
		{ LED_COMMAND_ON_OFF, led_on_off_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_BLINKING, led_blinking_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT, led_fade_in_out_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_ON_BRIGHTNESS, led_on_brightness_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_BLINKING_BRIGHTNESS, led_blinking_brightness_set, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_INTERVAL, led_fade_in_out_interval_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_BLINKING_TIME, led_blinking_time_set, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_MODE, led_mode_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_ON_OFF, led_on_off_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_BLINKING, led_blinking_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_FADE_IN_OUT, led_fade_in_out_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_CLK_DIV_EXP, led_set_clk_div_exp, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_SET_CLK, led_set_clk, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_DIV, led_set_div, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_EXP, led_set_exp, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_TOGGLE_LOOP_RAMP, led_set_toggle_loop_ramp, COMMAND_3_PARAMENTER },
		{ LED_COMMAND_SET_TOGGLE, led_set_toggle, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_LOOP, led_set_loop, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_RAMP, led_set_ramp, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_INVERT, led_set_invert, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_CURRENT_SINK, led_set_current_sink, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_PIN, led_set_pin, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_ON_OFF_LEVEL, led_set_on_off_level, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_SET_LPG_OUT, led_set_lpg_out, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_FADE_IN_OUT_BRIGHTNESS, led_fade_in_out_brightness_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SW_BLINKING, led_sw_blinking_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SW_FADE_IN_OUT, led_sw_fade_in_out_check_mode, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_SW_BLINKING, led_sw_blinking_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SPECIAL_SW_FADE_IN_OUT, led_sw_fade_in_out_set, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_SET_LUT_TABLE_RANGE, led_set_lut_table_range, COMMAND_2_PARAMENTER },
		{ LED_COMMAND_SET_ON_OFF_STATE, led_set_on_off_state, COMMAND_1_PARAMENTER },
		{ LED_COMMAND_AUTO_TURN_OFF, led_auto_turn_off, COMMAND_1_PARAMENTER },
	};

	struct BST_data	*sort_buffer;
	struct BST_info	sort_info;
	int		loop, return_value;

	return_value	= 1;

	if( ( command_info.commmand_buffer = kzalloc(sizeof(struct BS_data) * sizeof( led_command_list ) / sizeof( *led_command_list ), GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for command buffer\n" );
		return	0;
	}

	if( ( sort_buffer = kzalloc(sizeof(struct BST_data) * sizeof( led_command_list ) / sizeof( *led_command_list ), GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for sort buffer of command\n" );
		kfree(command_info.commmand_buffer);
		return	0;
	}

	if( ( command_data = kzalloc(sizeof(struct led_command_data) * sizeof( led_command_list ) / sizeof( *led_command_list ), GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for sort buffer of command\n" );
		kfree(command_info.commmand_buffer);
		kfree(sort_buffer);
		return	0;
	}

	BST_init( &sort_info );

	for( loop = 0 ; loop < sizeof( led_command_list ) / sizeof( *led_command_list ) ; ++loop )
	{
		struct led_command_source	*command		= led_command_list + loop;
		struct led_command_data	*commands_data	= command_data + loop;
		struct BST_data		*sort_data	= sort_buffer + loop;

		commands_data->function	= command->function;
		commands_data->parameter	= command->parameter;
		sort_data->index		= command->command;
		sort_data->data		= commands_data;
		BST_add( &sort_info, sort_data );
	}

	if( !( command_info.count = BST_sort( &sort_info, command_info.commmand_buffer, sizeof( led_command_list ) / sizeof( *led_command_list ) ) ) )
	{
		printk( KERN_ERR "LED : Sort command failed\n" );
		return_value	= 0;
	}

	kfree(sort_buffer);
	return	return_value;
}

static ssize_t led_command( void *node_data, struct device_attribute *attr, char *buf)
{
	return	snprintf( buf, PAGE_SIZE, 
		         "CMD\n"
			 "01 : LED on/off, 1 parameter\n"
			 "02 : LED blinking, 1 parameter\n"
			 "03 : LED fade in/out, 1 parameter\n"
			 "04 : Brightness of LED on, 1 parameter\n"
			 "05 : Brightness of LED blinking, 2 parameter\n"
			 "06 : Interval for LED fade in/out, 1 parameter\n"
			 "07 : Blinking time, 2 parameter\n"
			 "08 : Set mode, 1 parameter\n"
			 "09 : LED on/off in special mode, 1 parameter\n"
			 "10 : LED blinking in special mode, 1 parameter\n"
			 "11 : LED fade in/out in special mode, 1 parameter\n"
			 "12 : Set PMIC GPIO/MPP clock, DIV & EXP, 3 parameter\n"
			 "13 : Set PMIC GPIO/MPP clock, 1 parameter\n"
			 "14 : Set PMIC GPIO/MPP DIV, 1 parameter\n"
			 "15 : Set PMIC GPIO/MPP EXP, 1 parameter\n"
			 "16 : Set PMIC GPIO/MPP toggle, loop & ramp, 3 parameter\n"
			 "17 : Set PMIC GPIO/MPP toggle, 1 parameter\n"
			 "18 : Set PMIC GPIO/MPP loop, 1 parameter\n"
			 "19 : Set PMIC GPIO/MPP reamp, 1 parameter\n"
			 "20 : Set PMIC GPIO invert, 1 parameter\n"
			 "21 : Set PMIC MPP current sink, 1 parameter\n"
			 "22 : Set LED pin, 1 parameter\n"
			 "23 : Set MSM GPIO on/off level, 2 parameter\n"
			 "24 : Set PMIC GPIO/MPP LPG, 1 parameter\n"
			 "25 : Brightness of LED fade in/out, 1 parameter\n"
			 "26 : LED SW blinking, 1 parameter\n"
			 "27 : LED SW fade in/out, 1 parameter\n"
			 "28 : LED SW blinking in special mode, 1 parameter\n"
			 "29 : LED SW fade in/out in special mode, 1 parameter\n"
			 "30 : Set range of LUT table, 2 parameter\n"
			 "31 : Set on/off state of LED, 1 parameter\n"
			 "32 : Auto turn off, 1 parameter\n"
			 );
}

static ssize_t led_control( void *node_data, struct device_attribute *attr, const char *buf, size_t size )
{

	struct leds_driver_data	*data		= node_data;
	struct wake_lock		*normal_wakelock	= &data->normal_wakelock;
	unsigned int		pointer		= 0;
	int			rem_size		= size;

	wake_lock(normal_wakelock);

	while( pointer < rem_size )
	{
		struct BS_data		*search_data;
		struct led_data		*detail_data;
		struct led_command_data	*command;
		unsigned int		char_count;
		int			user_command, led_id;

		if( !get_para_from_buffer( buf + pointer, "%d", &user_command ) )
		{
			printk( "LED : Can't get command from buffer\n" );
			break;
		}

		if( !( search_data = binary_search( command_info.commmand_buffer, command_info.count, user_command ) ) )
		{
			printk( "LED : Can't support command(%d)\n", user_command );
			break;
		}

		command		= ( struct led_command_data* )search_data->data;

		if( !( char_count = get_para_char_count( buf + pointer, buf + size, 1 ) ) )
		{
			printk( "LED : Can't get charactor count.\n" );
			break;
		}

		pointer += char_count;

		if( !get_para_from_buffer( buf + pointer, "%d", &led_id ) )
		{
			printk( "LED : Can't get led id\n" );
			break;
		}

		if( !( search_data = binary_search( data->index_buffer, data->count, led_id ) ) )
		{
			printk( "LED : Can't support led id(%d)\n", led_id );
			break;
		}

		detail_data	= ( struct led_data* )search_data->data;

		if( !( char_count = get_para_char_count( buf + pointer, buf + size, 1 ) ) )
		{
			printk( "LED : Can't get charactor count..\n" );
			break;
		}

		pointer += char_count;
		printk( "LED : [CMD:ID:Para]=[%d:%d:%d]\n", user_command, led_id, command->parameter );

		{
			struct command_parameter	parameter;
			int		*parameter_pointer[] = { &parameter.para1, &parameter.para2, &parameter.para3, &parameter.para4, &parameter.para5, &parameter.para6, &parameter.para7, &parameter.para8 };
			unsigned int	loop;

			if( command->parameter > sizeof( parameter_pointer ) / sizeof( *parameter_pointer ) )
			{
				printk( "LED : Out of parameter array\n" );
				break;
			}

			for( loop = 0 ; loop < command->parameter ; ++loop )
			{
				if( !get_para_from_buffer( buf + pointer, "%d", *( parameter_pointer + loop ) ) )
					break;

				if( !( char_count = get_para_char_count( buf + pointer, buf + size, ONE_PARAMETER ) ) )
				{
					printk( "LED : Can't get charactor count...\n" );
					break;
				}

				pointer += char_count;
			}

			if( loop == command->parameter )
				command->function( detail_data, &parameter );
			else
			{
				printk( "LED : Can't get parameter\n" );
				break;
			}
		}
	}

	wake_unlock(normal_wakelock);
	return size;
}

static ssize_t led_info( void *node_data, struct device_attribute *attr, char *buf)
{
	struct leds_driver_data	*data	= node_data;
	int	loop, info_size, rem_size;

	rem_size		= PAGE_SIZE;
	info_size	= 0;

	for( loop = 0 ; loop < data->count ; ++loop )
	{
		struct led_data	*led_info	= ( data->index_buffer + loop )->data;

		switch( led_info->use_hw )
		{
			case	LED_HW_MSM_GPIO :
			{
				struct led_gpio_data	*msm_data	= &led_info->detail.gpio_data;

				info_size += snprintf( buf + info_size, rem_size,
					             "\n%s LED(%d), MSM GPIO-%d\n"
						     "Level[On:Off]=[%d:%d]\n"
						     "%s mode\n"
						     "%s state\n",
						     led_info->name, ( data->index_buffer + loop )->index, msm_data->msm_pin,
						     msm_data->led_on_level, msm_data->led_off_level,
						     led_info->special_mode ? "Special" : "Normal",
						     led_info->on_off_state ? "On" : "Off"
						     );
				break;
			}

			case	LED_HW_PMIC_GPIO :
			{
				struct led_pmic_data	*pmic_data	= &led_info->detail.pmic_data;

				info_size += snprintf( buf + info_size, rem_size,
					             "\n%s LED(%d), PMIC GPIO-%d, LPG-%d\n"
						     "On/Off PWM(%d), Blinking PWM[%d:%d], Fade in/out PWM(%d)\n"
						     "Blinking Time[%dms:%dms], Interval(%dms)\n"
						     "Frequency[CLK:DIV:EXP]=[%d:%d:%d]\n"
						     "Ramp Generator[Toggle:Loop:Up]=[%s:%s:%s]\n"
						     "LUT table[%d-%d]\n"
						     "%s invert\n"
						     "%s mode\n"
						     "%s state\n",
						     led_info->name, ( data->index_buffer + loop )->index, pmic_data->pmic_pin + 1, pmic_data->lpg_out + 1,
						     pmic_data->on_off_pwm, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2, pmic_data->fade_in_out_pwm,
						     pmic_data->blinking_time1, pmic_data->blinking_time2, pmic_data->interval,
						     pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp,
						     pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No",
						     pmic_data->lut_table_start, pmic_data->lut_table_end,
						     pmic_data->invert ? "Enable" : "Disable",
						     led_info->special_mode ? "Special" : "Normal",
						     led_info->on_off_state ? "On" : "Off"
						     );
				break;
			}

			case	LED_HW_PMIC_MPP :
			{
				struct led_pmic_data	*pmic_data	= &led_info->detail.pmic_data;

				info_size += snprintf( buf + info_size, rem_size,
					             "\n%s LED(%d), PMIC MPP-%d, LPG-%d\n"
						     "On/Off PWM(%d), Blinking PWM[%d:%d], Fade in/out PWM(%d)\n"
						     "Blinking Time[%dms:%dms], Interval(%dms)\n"
						     "Frequency[CLK:DIV:EXP]=[%d:%d:%d]\n"
						     "Ramp Generator[Toggle:Loop:Up]=[%s:%s:%s]\n"
						     "LUT table[%d-%d]\n"
						     "Current sink(%s)\n"
						     "%s mode\n"
						     "%s state\n",
						     led_info->name, ( data->index_buffer + loop )->index, pmic_data->pmic_pin + 1, pmic_data->lpg_out + 1,
						     pmic_data->on_off_pwm, pmic_data->blinking_pwm1, pmic_data->blinking_pwm2, pmic_data->fade_in_out_pwm,
						     pmic_data->blinking_time1, pmic_data->blinking_time2, pmic_data->interval,
						     pmic_data->pwm_clock, pmic_data->pwm_div_value, pmic_data->pwm_div_exp,
						     pmic_data->toggle_up_down ? "Yes" : "No", pmic_data->ramp_loop ? "Yes" : "No", pmic_data->ramp_up_down ? "Yes" : "No",
						     pmic_data->lut_table_start, pmic_data->lut_table_end,
						     *( current_sink_table + pmic_data->current_sink ),
						     led_info->special_mode ? "Special" : "Normal",
						     led_info->on_off_state ? "On" : "Off"
						     );
				break;
			}
		}
		rem_size	= PAGE_SIZE - info_size;

		if( rem_size <= 0 )
		{
			printk( "LED : Out of buffer size\n" );
			break;
		}
	}

	return	info_size;
}

static void	led_early_suspend_function(struct early_suspend *handler)
{
	struct wake_lock		*timeout_wakelock	= &container_of(handler, struct leds_driver_data, led_early_suspend)->timeout_wakelock;
	struct BS_data		*led_datas	= container_of(handler, struct leds_driver_data, led_early_suspend)->index_buffer;
	unsigned int		count		= container_of(handler, struct leds_driver_data, led_early_suspend)->count;
	struct command_parameter	parameter;
	unsigned int		loop;

	wake_lock_timeout( timeout_wakelock, WAIT_LOCK_TIME * HZ / 1000 );
	printk( "LED : Wake lock(%dms)\n", WAIT_LOCK_TIME );
	parameter.para1	= LED_TURN_OFF;

	for( loop = 0 ; loop < count ; ++loop )
		if( !( ( struct led_data* )( led_datas + loop )->data )->on_off_state )
			led_on_off_check_mode( ( struct led_data* )( led_datas + loop )->data, &parameter );
}

static void	led_late_resume_function(struct early_suspend *handler)
{
}

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	struct leds_device_data	*device_data	= pdev->dev.platform_data;
	struct leds_driver_data	*driver_data;
	struct BST_data		*sort_buffer;
	struct control_node_load	file_node;
	struct BST_info		sort_info;
	int	loop;

	if( device_data == NULL )
	{
		printk( KERN_ERR "LED : need device data\n" );
		return	-EINVAL;
	}

	if( ( driver_data = kzalloc(sizeof(struct leds_driver_data), GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for driver\n" );
		return	-ENOMEM;
	}

	if( ( leds_data = kzalloc(sizeof(struct led_data) * device_data->count, GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for driver data\n" );
		kfree(driver_data);
		return	-ENOMEM;
	}

	if( ( driver_data->index_buffer = kzalloc(sizeof(struct BS_data) * device_data->count, GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for index buffer\n" );
		kfree(driver_data);
		kfree(leds_data);
		return	-ENOMEM;
	}

	if( ( sort_buffer = kzalloc(sizeof(struct BST_data) * device_data->count, GFP_KERNEL) ) == NULL )
	{
		printk( KERN_ERR "LED : no memory for binary search tree\n" );
		kfree(driver_data->index_buffer);
		kfree(driver_data);
		kfree(leds_data);
		return	-ENOMEM;
	}

	BST_init( &sort_info );

	work_queue	= create_workqueue( "led_wq" );

	for( loop = 0 ; loop < device_data->count ; ++loop )
	{
		struct led_data		*destination	= leds_data + loop;
		struct led_device_data	*source		= device_data->device_data + loop;
		struct BST_data		*sort_data	= sort_buffer + loop;

		mutex_init(&destination->lock);

		sort_data->index		= source->id;
		sort_data->data		= destination;

		destination->name		= source->name;
		destination->use_hw	= source->use_hw;
		destination->special_mode	= SPECIAL_CONTROL_OFF;
		memcpy( &destination->detail, &source->detail, sizeof( union led_detail ) );

		INIT_WORK( &destination->work_blink, led_sched_blink );
		INIT_WORK( &destination->work_fade_in_out, led_sched_fade_in_out );
		INIT_WORK( &destination->work_turn_off, led_sched_turn_off );
		hrtimer_init( &destination->led_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		hrtimer_init( &destination->turn_off_time, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		destination->led_time.function	= led_timeout;
		destination->turn_off_time.function	= led_turn_off;

		BST_add( &sort_info, sort_data );

		switch( source->use_hw )
		{
			case	LED_HW_MSM_GPIO :
				if( gpio_request(source->detail.gpio_data.msm_pin, source->name) )
					printk( KERN_ERR "LED : Failed to request GPIO-%d(%s)\n", source->detail.gpio_data.msm_pin, source->name );
				break;

			case	LED_HW_PMIC_GPIO :
			case	LED_HW_PMIC_MPP :
				break;
		}

		{
			struct command_parameter	parameter;

			parameter.para1	= LED_TURN_OFF;
			led_on_off_set( destination, &parameter );
		}
	}

	if( !( driver_data->count = BST_sort( &sort_info, driver_data->index_buffer, device_data->count ) ) )
	{
		printk( KERN_ERR "LED : Sort failed\n" );
		kfree(driver_data->index_buffer);
		kfree(driver_data);
		kfree(leds_data);
		kfree(sort_buffer);
		return	-ENOMEM;
	}

	// For wake lock
	wake_lock_init( &driver_data->timeout_wakelock, WAKE_LOCK_SUSPEND, "fih_led_timeout" );
	wake_lock_init( &driver_data->normal_wakelock, WAKE_LOCK_SUSPEND, "fih_led_normal" );

	driver_data->led_early_suspend.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB;
	driver_data->led_early_suspend.suspend	= led_early_suspend_function;
	driver_data->led_early_suspend.resume	= led_late_resume_function;
	register_early_suspend(&driver_data->led_early_suspend);

	if( prepare_commands() )
	{
		// Create file node in sys/class/led/fih_led
		file_node.class_name	= "led";
		file_node.device_name	= "fih_led";
		file_node.file_node_data	= driver_data;
		file_node.control_read	= led_command;
		file_node.control_write	= led_control;
		file_node.info_read	= led_info;
		file_node_class		= control_file_node_register( &file_node );
	}

	dev_set_drvdata(&pdev->dev, driver_data);
	kfree(sort_buffer);
	return	0;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)	
{
	struct leds_device_data	*device_data	= pdev->dev.platform_data;
	struct leds_driver_data	*driver_data	= dev_get_drvdata( &pdev->dev );
	int	loop;

	for( loop = 0 ; loop < device_data->count ; ++loop )
		mutex_destroy( &( ( struct led_data* )driver_data->index_buffer->data + loop )->lock );

	control_file_node_unregister( file_node_class );
	wake_lock_destroy(&driver_data->timeout_wakelock);
	wake_lock_destroy(&driver_data->normal_wakelock);
	kfree(driver_data->index_buffer);
	kfree(driver_data);
	kfree(leds_data);
	kfree(command_info.commmand_buffer);
	kfree(command_data);
	return 0;
}

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.driver		= {
	.name		= "fih_leds",
	.owner		= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);
MODULE_AUTHOR("Y.S Chang <yschang@fih-foxconn.com>");

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
