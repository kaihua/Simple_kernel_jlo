/************************************************************************************/
/*                                                                                  */
/* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved. */
/*  FIH headset device detection driver.                                            */
/*                                                                                  */
/*    Logically, the H2W driver is always present, and H2W state (hi->state)        */
/*    indicates what is currently plugged into the headset interface.               */
/*                                                                                  */
/*    The headset detection work involves GPIO. A headset will still                */
/*    pull this line low.                                                           */
/*                                                                                  */
/*    Headset insertion/removal causes UEvent's to be sent, and                     */  
/*    /sys/class/switch/headset_sensor/state to be updated.                         */
/*                                                                                  */
/*    Button presses are interpreted as input event (KEY_MEDIA).                    */ 
/*                                                                                  */
/************************************************************************************/
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
//#include <linux/debugfs.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/fih_headset.h> /* FIH-SW2-MM-AY-TAP_headset_00 */
//#include <linux/fih_hw_info.h>
//#include <asm/io.h>     
#include <mach/pmic.h>
//#include <linux/earlysuspend.h>  
//#include "../gpio_hw.h"
/* FIH-SW2-MM-AY-hsed_type-00 [ */ 
#include <mach/proc_comm.h>/*SW-MM-RC-JLO2535_headset_00*/
#include <linux/fih_hw_info.h> 
/* FIH-SW2-MM-AY-hsed_type-00 ] */

/********************************************************/
/*                                                      */
/*                FEATURE DEFINITIONs                   */
/*                                                      */
/********************************************************/
#define CONFIG_DEBUG_FIH_HEADSET 

/* MM-RC-HEADSET-MULTIBUTTON-DETECT[* */
//#define FEATURE_AUD_HOOK_BTN
/* MM-RC-HEADSET-MULTIBUTTON-DETECT]* */

#define TYPE_HEADSET_INSERT          1  //0 : Insert-Low/Remove-High,    1 : Insert-High/Remove-Low
#define TYPE_HOOK_BTN_PRESS          1  //0 : Pressed-Low/Released-High, 1 : Pressed-High/Released-Low

//#define KEY_RINGSWITCH             184 

/********************************************************/
/*                                                      */
/*   Define the switch state                            */
/*   /sys/class/switch/headset_sensor/state             */
/*   Need to be syncronized with HeadsetObserver.java   */
/*                                                      */
/********************************************************/
enum {
	NO_DEVICE	  = 0,
	HEADSET	      = 1,
	NOMIC_HEADSET = 2,
	NOT_SUPPORT = (1 << 3), /* FIH-SW2-MM-AY-hsed_type-00 */ //using BIT_ANCHEADSET to pretend not supported device.
	UNKNOWN_DEVICE = (1 << 4), /* FIH-SW2-MM-AY-hsed_type-02 */ //using BIT_ANCHEADSET_NO_MIC to pretend unknown device.
};

/********************************************************/
/*                                                      */
/*   Headset Insert Define                              */
/*                                                      */
/********************************************************/
#if (TYPE_HEADSET_INSERT)
int HS_PLUG_IN               = 1;
int HS_PLUG_OUT              = 0;
int IRQF_TRIGGER_HS_INSERTED = IRQF_TRIGGER_HIGH;
int IRQF_TRIGGER_HS_REMOVED  = IRQF_TRIGGER_LOW;
#else 
int HS_PLUG_IN               = 0;
int HS_PLUG_OUT              = 1;
int IRQF_TRIGGER_HS_INSERTED = IRQF_TRIGGER_LOW;
int IRQF_TRIGGER_HS_REMOVED  = IRQF_TRIGGER_HIGH;
#endif

/********************************************************/
/*                                                      */
/*   Hook Key Define                                    */
/*                                                      */
/********************************************************/
#if (TYPE_HOOK_BTN_PRESS)
int BTN_STATE_PRESSED         = 1;
int BTN_STATE_RELEASED        = 0;
int IRQF_TRIGGER_BTN_PRESSED  = IRQF_TRIGGER_HIGH;
int IRQF_TRIGGER_BTN_RELEASED = IRQF_TRIGGER_LOW;
#else 
int BTN_STATE_PRESSED         = 0;
int BTN_STATE_RELEASED        = 1;
int IRQF_TRIGGER_BTN_PRESSED  = IRQF_TRIGGER_LOW;
int IRQF_TRIGGER_BTN_RELEASED = IRQF_TRIGGER_HIGH;
#endif

#ifdef CONFIG_DEBUG_FIH_HEADSET
#define H2W_DBG(fmt, arg...) printk(KERN_INFO "[AUD_HS] %s " fmt "\n", __FUNCTION__, ## arg)
#else
#define H2W_DBG(fmt, arg...) do {} while (0)
#endif

static struct h2w_info *hi;
static struct workqueue_struct *g_detection_work_queue;
static void detection_work(struct work_struct *work);
static DECLARE_WORK(g_detection_work, detection_work);
static bool isCTIAheadset = false; /* FIH-SW2-MM-AY-hsed_type-03 */

int  bn_irq_enable = 0;
bool bn_state      = 0; //0-Released 1-Pressed
/*SW-MM-RC-Mesona-03434[+*/
bool bn_hook_state      = 0; //0-Released 1-Pressed
bool bn_volup_state      = 0; //0-Released 1-Pressed
bool bn_voldown_state      = 0; //0-Released 1-Pressed
bool bn_live_state      = 0; //0-Released 1-Pressed
/*SW-MM-RC-Mesona-03434]+*/
bool mHeadphone    = false;
bool mSuspend      = false; 


static ssize_t trout_h2w_print_name(struct switch_dev *sdev, char *buf)
{
    int state = 0;
	char name_a[] = "No Device\n";
	char name_b[] = "Headset\n";
	char name_c[] = "Not Support\n"; /* FIH-SW2-MM-AY-hsed_type-00 */
	char name_d[] = "Unknow Device\n"; /* FIH-SW2-MM-AY-hsed_type-02 */ 

    state = switch_get_state(&hi->sdev);

	switch (state) {
	   case NO_DEVICE:
		   return snprintf(buf, sizeof(name_a), name_a);
	   case HEADSET:         
		   return snprintf(buf, sizeof(name_b), name_b);	
	   case NOMIC_HEADSET:         
	   /* FIH-SW2-MM-AY-hsed_type-00 [ */ 
		   return snprintf(buf, sizeof(name_b), name_b);
	   case NOT_SUPPORT:
	   	   return snprintf(buf, sizeof(name_c), name_c);
	   /* FIH-SW2-MM-AY-hsed_type-00 ] */
	   /* FIH-SW2-MM-AY-hsed_type-02 [ */
	   case UNKNOWN_DEVICE:
		   return snprintf(buf, sizeof(name_d), name_d);
	   /* FIH-SW2-MM-AY-hsed_type-02 ] */	   
       default:		   
           return snprintf(buf, sizeof(name_a), name_a);
	}
	return -EINVAL;
}



void aud_hs_print_gpio(void)
{
	H2W_DBG("hs gpio(%d) = %d, ptt gpio(%d) = %d ", hi->cable_in1, gpio_get_value(hi->cable_in1), hi->cable_in2, gpio_get_value(hi->cable_in2)); /* FIH-SW2-MM-AY-TAP_headset_00 */
}

/************************************************************************************/
/*                                                                                  */
/*   KEY_MEDIA is defined in <Linux/include/input.h>                                */
/*   You can modify the keypad layout to map this key for Android                   */
/*   For ADQ project, it will be located on : Vendor/qcom/msm7225_adq/stmpe1601.kl  */
/*   check the Android.mk first.                                                    */
/*                                                                                  */
/************************************************************************************/

/* MM-RC-HEADSET-MULTIBUTTON-DETECT[* */
void button_pressed(int pkey)
{
    if (switch_get_state(&hi->sdev) == HEADSET) {
        H2W_DBG("key = %d", pkey);
        atomic_set(&hi->btn_state, 1); 
        input_report_key(hi->input, pkey, 1);
        input_sync(hi->input); /* FIH-SW2-MM-AY-issue_hook_key_event */
        bn_state = 1;
        /*SW-MM-RC-Mesona-03434[+*/
        switch(pkey)
        {
	    case KEY_MEDIA:
                 bn_hook_state=1;
                 break;
	    case KEY_VOLUMEUP:
                bn_volup_state=1;
                break;
	    case KEY_VOLUMEDOWN:
                bn_voldown_state=1;
                break;
           case BTN_3: 
                bn_live_state=1;
                break;
            default:
                bn_hook_state=1;
                break;
        }
        /*SW-MM-RC-Mesona-03434]+*/
    }
}

void button_released(int pkey)
{
    bool need_release = bn_hook_state|bn_volup_state|bn_voldown_state|bn_live_state;/*SW-MM-RC-Mesona-03434-01+*/
    if ((switch_get_state(&hi->sdev) == HEADSET)&& need_release) { /*SW-MM-RC-Mesona-03434-01**/
        H2W_DBG("key = %d", pkey);
        atomic_set(&hi->btn_state, 0);
        input_report_key(hi->input, pkey, 0); 
        input_sync(hi->input); /* FIH-SW2-MM-AY-issue_hook_key_event */
        bn_state = 0;
        /*SW-MM-RC-Mesona-03434[+*/
        switch(pkey)
        {
	    case KEY_MEDIA:
                 bn_hook_state=0;
                 break;
	    case KEY_VOLUMEUP:
                bn_volup_state=0;
                break;
	    case KEY_VOLUMEDOWN:
                bn_voldown_state=0;
                break;
           case BTN_3: 
                bn_live_state=0;
                break;
            default:
                bn_hook_state=0;
                break;
        }
        /*SW-MM-RC-Mesona-03434]+*/
    }
}
/* MM-RC-HEADSET-MULTIBUTTON-DETECT]* */

static void insert_headset(void)
{
#ifdef FEATURE_AUD_HOOK_BTN
        unsigned long irq_flags;
#endif
		int voltage = 0; /* FIH-SW2-MM-AY-hsed_type-00 */

        H2W_DBG("");

        input_sync(hi->hs_input);
        msleep(100); 
                
#ifdef FEATURE_AUD_HOOK_BTN
        /* On some non-standard headset adapters (usually those without a
        * button) the btn line is pulled down at the same time as the detect
        * line. We can check here by sampling the button line, if it is
        * low then it is probably a bad adapter so ignore the button.
        * If the button is released then we stop ignoring the button, so that
        * the user can recover from the situation where a headset is plugged
        * in with button held down.
        */

        hi->ignore_btn = 1; /* FIH-SW2-MM-AY-TAP_Tapioca-00746_00 */

        if (bn_irq_enable==0) {
                
                /* Enable button irq */
                local_irq_save(irq_flags);
                
                enable_irq(hi->irq_btn);
                
                local_irq_restore(irq_flags);
                
                bn_irq_enable=1;

                irq_set_irq_wake(hi->irq_btn, 1);
        }
#endif
	/* FIH-SW2-MM-AY-hsed_type-00 [ */

	if (isCTIAheadset) { /* FIH-SW2-MM-AY-hsed_type-03 */
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		msleep(130);
		proc_comm_get_hsed_voltage(2, &voltage, 0); //it will get the prior value, not the current value. Should not omit.
		msleep(70);
		proc_comm_get_hsed_voltage(2, &voltage, 0);
		H2W_DBG("aud_hs: voltage is %d\n ", voltage);
		/* FIH-SW2-MM-AY-hsed_type-01 [ */
		if (voltage > 1700) {
	        //do not change state to issue an UEVENT.
	        //it is a earphone jack plug.
	        mHeadphone = false;
               hi->ignore_btn = 1;
	        pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
               /* MM-RC-HEADSET-MULTIBUTTON-DETECT[* */
               #ifdef FEATURE_AUD_HOOK_BTN
	        if (bn_irq_enable) {
       	        local_irq_save(irq_flags);
       	        disable_irq(hi->irq_btn);
       	        local_irq_restore(irq_flags);
       	        bn_irq_enable=0;
       	        irq_set_irq_wake(hi->irq_btn, 0);
	        }
               #endif
               /* MM-RC-HEADSET-MULTIBUTTON-DETECT]* */
		} else if (voltage > 700 || voltage < 400) { /* FIH-SW2-MM-AY-hsed_type-01 ] *//* SW-MM-RC-CTIA-TTY* */
	        if (gpio_get_value(hi->cable_in1) == HS_PLUG_IN) {
       	        if (gpio_get_value(hi->cable_in2) == BTN_STATE_PRESSED) {
                    switch_set_state(&hi->sdev, NOMIC_HEADSET);
                    mHeadphone=true;
                    hi->ignore_btn = 1;
                    pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
                    H2W_DBG("aud_hs:HEADPHONE is plugging\n ");
                } else {
                    switch_set_state(&hi->sdev, HEADSET);
                    mHeadphone=false;
			#ifdef FEATURE_AUD_HOOK_BTN 
                    irq_set_irq_type(hi->irq_btn, IRQF_TRIGGER_BTN_PRESSED);
			#endif 
                    msleep(300);
                    hi->ignore_btn = 0;
                    H2W_DBG("aud_hs:HEADSET is plugging\n ");
                }
	        }
		} else {
            mHeadphone = false;
            hi->ignore_btn = 1;
            pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
            switch_set_state(&hi->sdev, NOT_SUPPORT);
		}
		H2W_DBG("switch_get_state= %d ",switch_get_state(&hi->sdev));
	} else {
        /* FIH-SW2-MM-AY-TAP-ControlHSED_BIAS1-01 [ */
        pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
        msleep(130); /* FIH-SW2-MM-AY-Tapioca_SS-00467-00 */
        /* FIH-SW2-MM-AY-TAP-ControlHSED_BIAS1-01 ] */
	    if (gpio_get_value(hi->cable_in1) == HS_PLUG_IN) { /* FIH-SW2-MM-AY-TAP_headset_00 */
	
       	    if (gpio_get_value(hi->cable_in2) == BTN_STATE_PRESSED) { /* FIH-SW2-MM-AY-TAP_headset_00 */
                        
                switch_set_state(&hi->sdev, NOMIC_HEADSET);
			
                mHeadphone=true;
                hi->ignore_btn = 1;  /* FIH-SW2-MM-AY-TAP_Tapioca-00746_00 */
                H2W_DBG("aud_hs:HEADPHONE is plugging\n ");
	
            } else {
        
                switch_set_state(&hi->sdev, HEADSET);

                mHeadphone=false;
			
		#ifdef FEATURE_AUD_HOOK_BTN 
                irq_set_irq_type(hi->irq_btn, IRQF_TRIGGER_BTN_PRESSED);
		#endif 
                msleep(300);  /* FIH-SW2-MM-AY-TAP_Tapioca-00746_00 */
                hi->ignore_btn = 0;  /* FIH-SW2-MM-AY-TAP_Tapioca-00746_00 */
                H2W_DBG("aud_hs:HEADSET is plugging\n ");
            }
		
            H2W_DBG("switch_get_state= %d ",switch_get_state(&hi->sdev));
        }
	}
	/* FIH-SW2-MM-AY-hsed_type-00 ] */
}

static void remove_headset(void)
{
#ifdef FEATURE_AUD_HOOK_BTN	
	unsigned long irq_flags;
#endif

    H2W_DBG("");
        /*SW-MM-RC-Mesona-03434[+*/
        if(bn_hook_state)
            button_released(KEY_MEDIA);
        if(bn_volup_state)
            button_released(KEY_VOLUMEUP);
        if(bn_voldown_state)
            button_released(KEY_VOLUMEDOWN);
        if(bn_live_state)
            button_released(BTN_3);
        /*SW-MM-RC-Mesona-03434]+*/
    hi->ignore_btn = 1;  /* FIH-SW2-MM-AY-TAP_Tapioca-00746_00 */
    pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF); /* FIH-SW2-MM-AY-TAP-ControlHSED_BIAS1-01 */
	switch_set_state(&hi->sdev, NO_DEVICE);
		
  	input_sync(hi->hs_input);
#ifdef FEATURE_AUD_HOOK_BTN

	mHeadphone = false;

	if (bn_irq_enable) {
		
		local_irq_save(irq_flags);
		
		disable_irq(hi->irq_btn);

		local_irq_restore(irq_flags);
		
		bn_irq_enable=0;

		irq_set_irq_wake(hi->irq_btn, 0);
	}
	
	if (atomic_read(&hi->btn_state))
		button_released();
#endif
}

static void detection_work(struct work_struct *work)
{
	int cable_in;

	H2W_DBG("");

	  H2W_DBG("%d %d", gpio_get_value(hi->cable_in1), HS_PLUG_IN); /* FIH-SW2-MM-AY-TAP_headset_00 */
	if (gpio_get_value(hi->cable_in1) != HS_PLUG_IN) { /* FIH-SW2-MM-AY-TAP_headset_00 */
		/* Headset plugged out */
		if ( (switch_get_state(&hi->sdev) == HEADSET) || (switch_get_state(&hi->sdev) == NOMIC_HEADSET) || (switch_get_state(&hi->sdev) == NOT_SUPPORT) || (switch_get_state(&hi->sdev) == UNKNOWN_DEVICE)) { /* FIH-SW2-MM-AY-hsed_type-02 */
			H2W_DBG("Headset is plugged out.\n");
			remove_headset();
		}
		return;
	}

	/* Something plugged in, lets make sure its a headset */
	cable_in = gpio_get_value(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */
       
    if (cable_in == HS_PLUG_IN ) {
       	if (switch_get_state(&hi->sdev) == NO_DEVICE) {
       		H2W_DBG("Headset is plugged in.\n");
       		insert_headset();
       	}
    } else {
       	H2W_DBG("WARN: AUD_PIN_HEADSET_DET be triggered, but not a headset ");
    }

}

#ifdef FEATURE_AUD_HOOK_BTN
static enum hrtimer_restart button_event_timer_func(struct hrtimer *data)
{
	H2W_DBG("");
	
    if (switch_get_state(&hi->sdev) == HEADSET) {
       	if (gpio_get_value(hi->cable_in2) == BTN_STATE_RELEASED) {  /* FIH-SW2-MM-AY-TAP_headset_00 */
       		if (hi->ignore_btn)
                    hi->ignore_btn = 0;
       	else {
       		if (bn_state)
                    button_released();
       		else {
       		    if (!hi->ignore_btn) { /* FIH-SW2-MM-AY-TAP_headset_01 */
       		        button_pressed();
       		        button_released();
       		    } /* FIH-SW2-MM-AY-TAP_headset_01 */
       		}
       	}
    } else {
       	if (!hi->ignore_btn)
       		button_pressed();
       	}
    }

    return HRTIMER_NORESTART;
}
#endif

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	H2W_DBG("");

	queue_work(g_detection_work_queue, &g_detection_work);
	
	return HRTIMER_NORESTART;
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	H2W_DBG("");
	
	aud_hs_print_gpio(); 

    /* debunce */
	do {
		value1 = gpio_get_value(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */
		irq_set_irq_type(hi->irq, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */
	} while (value1 != value2 && retry_limit-- > 0);

	H2W_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));	

   /*
    * If the sdev is NO_DEVICE, and we detect the headset has been plugged,
    * then we can do headset_insertion check.
	*/
	if ( (switch_get_state(&hi->sdev) == NO_DEVICE) ^ (value2^HS_PLUG_IN)) {
		
		if (switch_get_state(&hi->sdev) == HEADSET)      
			hi->ignore_btn = 1;

		/* Do the rest of the work in timer context */
		hrtimer_start(&hi->timer, hi->debounce_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

#ifdef FEATURE_AUD_HOOK_BTN 
static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	H2W_DBG("");
	
	aud_hs_print_gpio(); 
	
	do {
		value1 = gpio_get_value(hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */
		irq_set_irq_type(hi->irq_btn, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */
	} while (value1 != value2 && retry_limit-- > 0);

	H2W_DBG("Hook BTN :value2 = %d (%d retries)", value2, (10-retry_limit));


	hrtimer_start(&hi->btn_timer, hi->btn_debounce_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}
#endif 

static int trout_h2w_probe(struct platform_device *pdev)
{
	int ret;
	struct h2w_platform_data *pdata = pdev->dev.platform_data; /* FIH-SW2-MM-AY-TAP_headset_00 */

	printk(KERN_INFO "[AUD_HS]: Registering H2W (headset) driver\n");

	H2W_DBG("");
	
	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	
	if (!hi)
		return -ENOMEM;

  /*                                                        
   * 1. Headset insertion/removal causes UEvent's to be sent
   * 2. /sys/class/switch/headset_sensor/state to be updated 
   * 3. debounce time too short will affect the behavior of headset plugin/out in phone call
   */
   
    atomic_set(&hi->hs_state, 0);
	atomic_set(&hi->btn_state, 0);  
	
	hi->ignore_btn        = 0;
	/* FIH-SW2-MM-AY-TAP_headset_00 [ */
	hi->cable_in1 = pdata->cable_in1;
	hi->cable_in2 = pdata->cable_in2;
	/* FIH-SW2-MM-AY-TAP_headset_00 ] */
	/* MM-RC-TAP reduce delay time after removing headset  1206 */
/* MM-NC-Headset_Debounce_Time-00-[+ */
	hi->debounce_time     = ktime_set(0, 800000000);  /* 200 -> 800 ms */
/* MM-NC-Headset_Debounce_Time-00-]- */
	hi->btn_debounce_time = ktime_set(0,  80000000);  /*  80 ms */   
	hi->sdev.name         = "headset_sensor";
	hi->sdev.print_name   = trout_h2w_print_name;	
	hi->hs_input          = input_allocate_device();
	
	if (!hi->hs_input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}	
	
  	hi->hs_input->name = "fih_ringswitch";
	
 	set_bit(EV_KEY, hi->hs_input->evbit);  	  
  	//set_bit(KEY_RINGSWITCH, hi->hs_input->keybit);

	ret = input_register_device(hi->hs_input);
	
	if (ret < 0)
	    goto err_register_hs_input_dev;	  

	ret = switch_dev_register(&hi->sdev);
	
	if (ret < 0)
		goto err_switch_dev_register;

	g_detection_work_queue = create_workqueue("detection");
	
	if (g_detection_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}

	ret = gpio_request(hi->cable_in1, "h2w_detect"); /* FIH-SW2-MM-AY-TAP_headset_00 */
	
	if (ret < 0)
		goto err_request_detect_gpio;

	ret = gpio_direction_input(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */

	if (ret < 0)
		goto err_set_detect_gpio;
    else
        H2W_DBG(" set aid gpio(%d) as input pin : success.\r\n", hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */

    hi->irq = gpio_to_irq(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */
	
    if (hi->irq < 0) { /* FIH-SW3-MM-AY-GUA Coverity 1108 */
	    ret = hi->irq;
  	    goto err_get_h2w_detect_irq_num_failed;
    }
    else
	    H2W_DBG(" hs_det gpio_to_irq(%d): success.\r\n", hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */

	hrtimer_init(&hi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	
	hi->timer.function = detect_event_timer_func;	

#ifdef FEATURE_AUD_HOOK_BTN
	ret = gpio_request(hi->cable_in2, "h2w_button"); /* FIH-SW2-MM-AY-TAP_headset_00 */

	if (ret < 0)
		goto err_request_button_gpio;

	ret = gpio_direction_input(hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */
	
	if (ret < 0)
		goto err_set_button_gpio;
    else
        H2W_DBG(" set ptt gpio(%d) as input pin : success.\r\n", hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */

	hi->irq_btn = gpio_to_irq(hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */
	
	if (hi->irq_btn < 0) { /* FIH-SW3-MM-AY-GUA Coverity 1108 */
		ret = hi->irq_btn;
		goto err_get_button_irq_num_failed;
	}
    else
        H2W_DBG(" hook_btn gpio_to_irq(%d): success.\r\n", hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */

	hrtimer_init(&hi->btn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	
	hi->btn_timer.function = button_event_timer_func;
#endif

	aud_hs_print_gpio(); 

    //headset inserted : gpio H->L(detect LOW level)
	ret = request_irq(hi->irq, detect_irq_handler,
	         IRQF_TRIGGER_HS_INSERTED, "h2w_detect", NULL);

	if (ret < 0)
		goto err_request_detect_irq;
    else
        H2W_DBG(" request_irq (gpio %d, IRQF_TRIGGER_LOW) success\n", hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */

    // Set headset_detect pin as wake up pin
    ret = irq_set_irq_wake(hi->irq, 1);

if (ret < 0)
	goto err_request_input_dev;

#ifdef FEATURE_AUD_HOOK_BTN
	// Disable button until plugged in 
	set_irq_flags(hi->irq_btn, IRQF_VALID | IRQF_NOAUTOEN);

	ret = request_irq(hi->irq_btn, button_irq_handler,
			  IRQF_TRIGGER_BTN_PRESSED, "h2w_button", NULL);         
	
	if (ret < 0)
		goto err_request_h2w_headset_button_irq;
    else
        H2W_DBG("request_irq (gpio %d, IRQF_TRIGGER_HIGH) success\n", hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */

#endif
	
	hi->input = input_allocate_device();

	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name                        = "simple_remote_appkey";/* MM-SC-LIVEKEY-DETECT  //"fih_headsethook";*/
	/* MM-RC-HEADSET-MULTIBUTTON-DETECT[* */
	//hi->input->evbit[0]                    = BIT_MASK(EV_KEY); 
    //hi->input->keybit[BIT_WORD(KEY_MEDIA)] = BIT_MASK(KEY_MEDIA); 
	if (isCTIAheadset) { /* FIH-SW2-MM-AY-hsed_type-03 */
		input_set_capability(hi->input, EV_KEY, KEY_VOLUMEDOWN);
		input_set_capability(hi->input, EV_KEY, KEY_VOLUMEUP);
		input_set_capability(hi->input, EV_KEY, BTN_3);  /* MM-RC-HEADSET-LiveKEY-DETECT+ */
	}
	input_set_capability(hi->input, EV_KEY, KEY_MEDIA);
	/* MM-RC-HEADSET-MULTIBUTTON-DETECT]* */

	ret = input_register_device(hi->input);
	
	if (ret < 0)
		goto err_register_input_dev;


	return 0;

// Error Messages
err_register_input_dev:
    printk(KERN_ERR "aud_hs: err_register_input_dev\n");
	input_free_device(hi->input);

err_register_hs_input_dev:
    printk(KERN_ERR "aud_hs: err_register_hs_input_dev\n");
	input_free_device(hi->hs_input);
err_request_input_dev:
    #ifdef FEATURE_AUD_HOOK_BTN
    printk(KERN_ERR "aud_hs: err_request_input_dev\n");
	free_irq(hi->irq_btn, 0);
    #endif    
#ifdef FEATURE_AUD_HOOK_BTN		
err_request_h2w_headset_button_irq:
    printk(KERN_ERR "aud_hs: request_h2w_headset_button_irq\n");
	free_irq(hi->irq, 0);
#endif	
err_request_detect_irq:
#ifdef FEATURE_AUD_HOOK_BTN		
err_get_button_irq_num_failed:
#endif
err_get_h2w_detect_irq_num_failed:
#ifdef FEATURE_AUD_HOOK_BTN	
err_set_button_gpio:
#endif
err_set_detect_gpio:
    printk(KERN_ERR "aud_hs: AUD_PIN_HOOK_BTN, gpio/irq error\n");
    #ifdef FEATURE_AUD_HOOK_BTN
	gpio_free(hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */
    #endif
#ifdef FEATURE_AUD_HOOK_BTN	
err_request_button_gpio:
    printk(KERN_ERR "aud_hs: err_request_button_gpio\n");
	gpio_free(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */
#endif	
err_request_detect_gpio:
      printk(KERN_ERR "aud_hs: err_request_detect_gpio\n");
	destroy_workqueue(g_detection_work_queue);
err_create_work_queue:
      printk(KERN_ERR "aud_hs: err_create_work_queue\n");    
	switch_dev_unregister(&hi->sdev);
err_switch_dev_register:
	printk(KERN_ERR "aud_hs: Failed to register driver\n");

	return ret;
}

static int trout_h2w_remove(struct platform_device *pdev)
{
	H2W_DBG("");
	
	if (switch_get_state(&hi->sdev))
		remove_headset();

    #ifdef FEATURE_AUD_HOOK_BTN
	input_unregister_device(hi->input);
	gpio_free(hi->cable_in2); /* FIH-SW2-MM-AY-TAP_headset_00 */
	free_irq(hi->irq_btn, 0);	
    #endif   
	
	gpio_free(hi->cable_in1); /* FIH-SW2-MM-AY-TAP_headset_00 */
	free_irq(hi->irq, 0);
	 	
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hi->sdev);

	return 0;
}

static struct platform_driver trout_h2w_driver = {
	.probe		= trout_h2w_probe,
	.remove		= trout_h2w_remove,
	.driver		= {
		.name		= "headset_sensor",
		.owner		= THIS_MODULE,
	},
};

static int __init trout_h2w_init(void)
{
	/* FIH-SW2-MM-AY-hsed_type-03 [ */
    int product_id = fih_get_product_id();
	int product_phase = fih_get_product_phase();
	/* FIH-SW2-MM-AY-hsed_type-03 ] */

	H2W_DBG("trout_h2w_init:set interrupt trigger level\n ");

	/* FIH-SW2-MM-AY-hsed_type-03 [ */
	if (((product_id == Project_TAP) && ((product_phase == Phase_AP2) || (product_phase == Phase_MP))) 
		|| ((product_id == Project_MES) && (product_phase >= Phase_PreAP)) 
		|| ((product_id == Project_JLO) && (product_phase >= Phase_SP3))) 
		isCTIAheadset = true;
	else
		isCTIAheadset = false;
	/* FIH-SW2-MM-AY-hsed_type-03 ] */
	
	return platform_driver_register(&trout_h2w_driver);

}

static void __exit trout_h2w_exit(void)
{
	platform_driver_unregister(&trout_h2w_driver);
}

module_init(trout_h2w_init);
module_exit(trout_h2w_exit);

MODULE_AUTHOR("Seven Lin <sevenlin@fihspec.com>");
MODULE_DESCRIPTION("FIH headset detection driver");
MODULE_LICENSE("Proprietary");
