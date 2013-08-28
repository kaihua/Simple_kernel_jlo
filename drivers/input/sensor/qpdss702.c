/* drivers/input/sensor/qpdss702.c
 *
 * Copyright (C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/fih_hw_info.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/qpdss702.h>
#include <mach/gpio-v1.h>
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
#include "mach/rpc_nv.h"
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
#include <linux/wakelock.h>
#include <linux/err.h>

struct qpdss702_data
{
    struct delayed_work work;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    struct i2c_client *client;
    struct input_dev *input;
    struct mutex mutex;
    struct qpdss702_platform_data *platform_data;
    unsigned int delay;
    unsigned int mode;
    int enable;
    int irq;
    int sen_err;
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE

    unsigned int nvInvalid;
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
/*PERI-AC-suspen_noirq-00+{*/
    int suspend_flag;
    int wakeup_flag;
    struct wake_lock ps_wake_lock;
/*PERI-AC-suspen_noirq-00+}*/
};

static int qpdss702_set_mode(struct i2c_client *client, int mode)
{
    struct qpdss702_data *qpdss702 = i2c_get_clientdata(client);
    unsigned char command=0;
    int ret = 0;

    if (qpdss702)
    {
        mutex_lock(&qpdss702->mutex);
        command = (QPDSS702_MODE[mode]&0xC3)|(qpdss702->platform_data->sensitivity<<2);
        ret = i2c_smbus_write_byte_data(client, command, (QPDSS702_MODE[mode]>>8));
        qpdss702->mode = mode;

        if (mode > 0)
        {
            if (!qpdss702->enable)
            {
                enable_irq(qpdss702->irq);
                irq_set_irq_wake(qpdss702->irq, 1);
                qpdss702->enable = true;
                PSENSOR_DEBUG(LEVEL0, "mode=%d(0x%4x)", mode,((QPDSS702_MODE[mode]&0xff00)|command));
            }
        }
        else	/* showdown mode */
        {
            if (qpdss702->enable)
            {
                qpdss702->enable = false;
                irq_set_irq_wake(qpdss702->irq, 0);
                disable_irq(qpdss702->irq);
                PSENSOR_DEBUG(LEVEL0, "mode=%d(0x%4x)", mode,((QPDSS702_MODE[mode]&0xff00)|command));
            }
        }
        mutex_unlock(&qpdss702->mutex);
    }
    else
    {
        ret = -1;
    }

    return ret;
}

static ssize_t qpdss702_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int value = 0;
    int newState = TYPICAL_MODE;
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);

    qpdss702_set_mode(qpdss702->client, newState);
    msleep(500);	/* sleep 0.5 second */

    value = gpio_get_value(GPIO_ALPS_OUT);
    PSENSOR_DEBUG(LEVEL0, "value=%d, newState=%d(0x%4x)", value, newState, QPDSS702_MODE[newState]);

    newState = SHUTDOWN_MODE;
    qpdss702_set_mode(qpdss702->client, newState);

    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t qpdss702_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);

    if (!qpdss702)
        return snprintf(buf, PAGE_SIZE, "-1\n");

    return snprintf(buf, PAGE_SIZE, "%d\n", qpdss702->delay);
}

static ssize_t qpdss702_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);
    int delay = simple_strtoul(buf, NULL, 10);

    if ((!qpdss702) || (delay < 0))
        return -EINVAL;

    if (delay == 0)
        delay = PSENSOR_DEFAULT_DELAY;

    if (delay > PSENSOR_MAX_DELAY)
        delay = PSENSOR_MAX_DELAY;

    mutex_lock(&qpdss702->mutex);
    qpdss702->delay = delay;
    mutex_unlock(&qpdss702->mutex);

    if (qpdss702->enable)
    {
        cancel_delayed_work_sync(&qpdss702->work);
        schedule_delayed_work(&qpdss702->work, msecs_to_jiffies(qpdss702->delay));
    }
    PSENSOR_DEBUG(LEVEL0, "delay=%d", delay);

    return count;
}

static ssize_t qpdss702_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int enabled;
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);

    enabled = qpdss702->enable ? 1 : 0;

    return snprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t qpdss702_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);
    int value = simple_strtoul(buf, NULL, 10);
    int newState = value ? TYPICAL_MODE : SHUTDOWN_MODE;

    /* workaround: +[
    * Becaue of mechanical issue,
    * P-sensor is always detected in GUA-DP and TAP-DP phases.
    * Disable P-sensor in GUA-DP and TAP-DP phases.
    */

    if ((fih_get_product_id() == Project_GUA || fih_get_product_id() == Project_TAP) && fih_get_product_phase() == Phase_DP)
    //if ((fih_get_product_id() == Project_GUA) && fih_get_product_phase() == Phase_DP)
    {
        newState = SHUTDOWN_MODE;
        PSENSOR_DEBUG(LEVEL0, "DP device, disable proximity sensor!! newState=%d", newState);
    }

    /* workaround: +] */
    PSENSOR_DEBUG(LEVEL1, "start");

    if ((!qpdss702) || (value != 0 && value != 1))
    {
        PSENSOR_DEBUG(LEVEL0, "Failed. value=%d", value);
        return count;
    }

    qpdss702_set_mode(qpdss702->client, newState);
    if (newState)
    {
        schedule_delayed_work(&qpdss702->work, 0);
    }

    PSENSOR_DEBUG(LEVEL0, "value=%d, newState=%d(0x%4x)", value, newState, ((QPDSS702_MODE[newState]&0xffc3)|(qpdss702->platform_data->sensitivity<<2)));

    return count;
}

static ssize_t qpdss702_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", psensor_debug_level);
}

static ssize_t qpdss702_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int value = simple_strtoul(buf, NULL, 10);
    psensor_debug_level = value ? 1 : 0;

    return count;
}

static ssize_t qpdss702_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);

    if (!qpdss702)
        return snprintf(buf, PAGE_SIZE, "-1\n");

    return snprintf(buf, PAGE_SIZE, "%u\n", qpdss702->mode);
}

static ssize_t qpdss702_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);
    int ret, value = simple_strtoul(buf, NULL, 10);

    if ((!qpdss702) || (value < SHUTDOWN_MODE) || (value >= TOTAL_MODE_NUM))
        return -EINVAL;

    PSENSOR_DEBUG(LEVEL0, "mode=%d(0x%4x), enable=%d", value, QPDSS702_MODE[value], qpdss702->enable);

    ret = qpdss702_set_mode(qpdss702->client, value);
    if (ret < 0)
    {
        return ret;
    }
    if (qpdss702->enable)
    {
        schedule_delayed_work(&qpdss702->work, msecs_to_jiffies(qpdss702->delay));
    }
    else
    {
        cancel_delayed_work_sync(&qpdss702->work);
    }

    return count;
}

static ssize_t qpdss702_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
    unsigned int sensitivity=0;
    int err=0;
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
    if (!qpdss702)
        return snprintf(buf, PAGE_SIZE, "-1\n");

//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
    if(!qpdss702->nvInvalid)
    {
        err = rpc_nv_read_psensor_sensitivity(&sensitivity);
        /*When we can't get data from nv keep default data.*/
        if(err == 0)
        {
            qpdss702->platform_data->sensitivity = sensitivity;
        }
    }
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
    if (qpdss702->sen_err)
        return snprintf(buf, PAGE_SIZE, "-%u\n", qpdss702->sen_err);
    else
        return snprintf(buf, PAGE_SIZE, "%u\n", qpdss702->platform_data->sensitivity);
}

static ssize_t qpdss702_sensitivity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct qpdss702_data *qpdss702 = dev_get_drvdata(dev);
    int i = 0, ret, irq_value, value = simple_strtoul(buf, NULL, 10);
    u8 LSB, MSB;

    if ((!qpdss702))
        return -EINVAL;
    if ((value < 0) || (value > 1000))
    {
        qpdss702->sen_err = EINVAL;
        return -EINVAL;
    }

    if (value < 16)
    {
        LSB = QPDSS702_TYPICAL_MODE & 0xFF;
        MSB = QPDSS702_TYPICAL_MODE >> 8;

        LSB = (LSB & 0xC3) | (value << 2);
        mutex_lock(&qpdss702->mutex);
        ret = i2c_smbus_write_byte_data(qpdss702->client, LSB, MSB);
        mutex_unlock(&qpdss702->mutex);
        qpdss702->platform_data->sensitivity = value;
        qpdss702->sen_err = 0;
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
        if(!qpdss702->nvInvalid)
        {
            rpc_nv_write_psensor_sensitivity(value);
        }
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
    }
    else if (value > 300 && value < 1000)
    {	// Auto-tune the threshold of sensitivity to delay milliseconds
        LSB = QPDSS702_TYPICAL_MODE & 0xFF;
        MSB = QPDSS702_TYPICAL_MODE >> 8;
        qpdss702->sen_err = 0;
        for (i=15; i>=0; i--)
        {
            /*Setup sensitivity value to register*/
            LSB = (LSB & 0xC3) | (i << 2);
            mutex_lock(&qpdss702->mutex);
            ret = i2c_smbus_write_byte_data(qpdss702->client, LSB, MSB);
            mutex_unlock(&qpdss702->mutex);

            msleep(value);	/* sleep 0.5 second */
            irq_value = gpio_get_value(GPIO_ALPS_OUT);
            PSENSOR_DEBUG(LEVEL1, "newState(%2d)=0x%2x%2x, irq_value=%d", i, MSB, LSB, irq_value);
            if (!irq_value)
            {
                qpdss702->platform_data->sensitivity = i;
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
                if(!qpdss702->nvInvalid)
                {
                    rpc_nv_write_psensor_sensitivity(i);
                }
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
                break;
            }
        }
    }
    else
    {
        qpdss702->sen_err = EINVAL;
        return -EINVAL;
    }

    /*Devices IO can't find access*/
    if (i==-1)
    {
        qpdss702->platform_data->sensitivity = 0;
        qpdss702->sen_err = EIO;
    }
    PSENSOR_DEBUG(LEVEL1, "milliseconds=%d, return=%d", value, i);
    return count;
}

static DEVICE_ATTR(debug, 0660, qpdss702_debug_show, qpdss702_debug_store);
static DEVICE_ATTR(delay, 0660, qpdss702_delay_show, qpdss702_delay_store);
static DEVICE_ATTR(enable, 0660, qpdss702_enable_show, qpdss702_enable_store);
static DEVICE_ATTR(mode, 0660, qpdss702_mode_show, qpdss702_mode_store);
static DEVICE_ATTR(sensitivity, 0660, qpdss702_sensitivity_show, qpdss702_sensitivity_store);
static DEVICE_ATTR(value, 0640, qpdss702_value_show, NULL);

static struct attribute *qpdss702_attributes[] = {
    &dev_attr_debug.attr,
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_mode.attr,
    &dev_attr_sensitivity.attr,
    &dev_attr_value.attr,
    NULL
};

static struct attribute_group qpdss702_attribute_group = {
    .attrs = qpdss702_attributes
};

static int qpdss702_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    struct i2c_adapter *adapter = client->adapter;

    if (!adapter)
    {
        PSENSOR_DEBUG(LEVEL0, "Find I2C adapter failed.");
        return -ENOMEM;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
    {
        PSENSOR_DEBUG(LEVEL0, "Check I2C functionality failed.");
        return -ENODEV;
    }

    strlcpy(info->type, dev_name(&client->dev), I2C_NAME_SIZE);

    PSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static void qpdss702_irq_work(struct work_struct *work)
{
    struct qpdss702_data *qpdss702 = container_of((struct delayed_work *)work, struct qpdss702_data, work);
    int value = gpio_get_value(GPIO_ALPS_OUT);

    if (qpdss702->enable)
    {
        /*PERI-AC-suspen_noirq-00+{*/
        if(value && qpdss702->suspend_flag)
        {
            qpdss702->wakeup_flag = 1;
        }
        /*PERI-AC-suspen_noirq-00+}*/

        input_report_abs(qpdss702->input, EVENT_TYPE_PROXIMITY, value);
        input_sync(qpdss702->input);
        PSENSOR_DEBUG(LEVEL0, "GPIO(%d) = %d, input_report_abs(%d), delay = %d", GPIO_ALPS_OUT, value, value, qpdss702->delay);
    }
}

static irqreturn_t qpdss702_irq_handler(int irq, void *dev_id)
{
    struct qpdss702_data *qpdss702 = dev_id;

    schedule_delayed_work(&qpdss702->work, 0);

    return IRQ_HANDLED;
}

static int qpdss702_input_init(struct qpdss702_data *qpdss702)
{
    struct input_dev *dev = input_allocate_device();
    int err = 0;

    if (!dev)
    {
        PSENSOR_DEBUG(LEVEL0, "Allocate input device failed.");
        return -ENOMEM;
    }
    dev->name = "proximity";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    input_set_capability(dev, EV_ABS, EVENT_TYPE_PROXIMITY);
    input_set_drvdata(dev, qpdss702);

    err = input_register_device(dev);
    if (err < 0)
    {
        PSENSOR_DEBUG(LEVEL0, "Register input device failed.");
        input_free_device(dev);
        return err;
    }
    qpdss702->input = dev;

    PSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static void qpdss702_input_delete(struct qpdss702_data *qpdss702)
{
    struct input_dev *dev = qpdss702->input;

    if (!dev)
    {
        PSENSOR_DEBUG(LEVEL0, "Find input device failed.");
        return;
    }

    input_unregister_device(dev);
    input_free_device(dev);
    PSENSOR_DEBUG(LEVEL0, "Done.");
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qpdss702_early_suspend(struct early_suspend * h)
{
    struct qpdss702_data *data = container_of(h, struct qpdss702_data, early_suspend);
    int value = gpio_get_value(GPIO_ALPS_OUT);
    if(!value && data->enable)
    {
        data->suspend_flag =1;
    }
    PSENSOR_DEBUG(LEVEL0, "Done.");
    return;
}

static void qpdss702_late_resume(struct early_suspend *h)
{
/*PERI-AC-suspen_noirq-00+{*/
    struct qpdss702_data *data = container_of(h, struct qpdss702_data, early_suspend);
    data->suspend_flag = 0;
    data->wakeup_flag = 0;
/*PERI-AC-suspen_noirq-00+}*/
    PSENSOR_DEBUG(LEVEL0, "Done");
    return;
}
#endif

#ifdef CONFIG_PM
static int qpdss702_suspend(struct device *dev)
{
/*PERI-AC-suspen_noirq-00+{*/
    struct qpdss702_data *data = dev_get_drvdata(dev);
    int value = gpio_get_value(GPIO_ALPS_OUT);
    if(data->wakeup_flag && value && data->enable)
    {
        wake_lock_timeout(&data->ps_wake_lock, 1*HZ);
        return -EAGAIN;
    }
/*PERI-AC-suspen_noirq-00+}*/
    PSENSOR_DEBUG(LEVEL0, "suspen_flag=%x, value=%x,Done.",data->suspend_flag,value);
    return 0;
}

static int qpdss702_resume(struct device *dev)
{
/*PERI-AC-suspen_noirq-00+{*/
    struct qpdss702_data *data = dev_get_drvdata(dev);
    data->wakeup_flag = 0;
/*PERI-AC-suspen_noirq-00+}*/
    PSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

/*PERI-AC-suspen_noirq-00+{*/
static int qpdss702_suspend_noirq(struct device *dev)
{
    struct qpdss702_data *data = dev_get_drvdata(dev);
    int value = gpio_get_value(GPIO_ALPS_OUT);
    if(data->wakeup_flag && value && data->enable)
    {
        wake_lock_timeout(&data->ps_wake_lock, 1*HZ);
        return -EAGAIN;
    }
    PSENSOR_DEBUG(LEVEL0, "suspen_flag=%x, value=%x,Done.",data->suspend_flag,value);
    return 0;
}
/*PERI-AC-suspen_noirq-00+}*/

static struct dev_pm_ops qpdss702_pm_ops = {
    .suspend = qpdss702_suspend,
    .resume  = qpdss702_resume,
/*PERI-AC-suspen_noirq-00+{*/
    .suspend_noirq = qpdss702_suspend_noirq
/*PERI-AC-suspen_noirq-00+}*/
};
#endif	/* CONFIG_PM */

static int qpdss702_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    struct qpdss702_data *data = NULL;
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
    unsigned int sensitivity=0;
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
    /* Chcek I2C */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        err = -EIO;
        PSENSOR_DEBUG(LEVEL0, "Check I2C functionality failed.");
        goto err_check_functionality_failed;
    }

    /* Allocate Kernel Memory */
    data = kzalloc(sizeof(struct qpdss702_data), GFP_KERNEL);
    if (!data)
    {
        err = -ENOMEM;
        PSENSOR_DEBUG(LEVEL0, "Allocate kernel memory failed.");
        goto err_alloc_kernel_failed;
    }

    /* Check Platform Data */
    data->platform_data = client->dev.platform_data;
    if (!data->platform_data)
    {
        err = -ENODEV;
        PSENSOR_DEBUG(LEVEL0, "Check platform data failed.");
        goto err_check_platform_data_failed;
    }

//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
    if(!data->platform_data->gpio_init)
    {
        /*The platform data didn't register gpio function unregister driver*/
        err = -EIO;
        PSENSOR_DEBUG(LEVEL0, "Check platform data can't find GPIO function");
        goto err_check_platform_data_failed;
    }
    else
    {
        /* Init GPIO */
        err = data->platform_data->gpio_init();
    }
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}

    if (err < 0)
        goto err_init_gpio_failed;

    i2c_set_clientdata(client, data);
    data->client = client;
    data->irq = client->irq;
    mutex_init(&data->mutex);

    /* Init Delayed Work Queue and Register IRQ */
    INIT_DELAYED_WORK(&data->work, qpdss702_irq_work);
    err = request_irq(data->irq, qpdss702_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, dev_name(&client->dev), data);
    if (err < 0)
    {
        PSENSOR_DEBUG(LEVEL0, "Request IRQ(%d) failed.", data->irq);
        goto err_request_irq_failed;
    }

//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
    err = rpc_nv_read_psensor_sensitivity(&sensitivity);
    if((err==0) && (sensitivity<16) && (sensitivity>=2))
    {
        data->platform_data->sensitivity = sensitivity;
        data->nvInvalid = false;
    }
    else
    {
        data->nvInvalid = true;
    }
#endif
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
    //irq_set_irq_wake(g_qpdss702_irq, 1);
    disable_irq(data->irq);
    data->delay = PSENSOR_DEFAULT_DELAY;
    data->mode = SHUTDOWN_MODE;
    data->enable = false;
    data->sen_err = 0;
    qpdss702_set_mode(client, data->mode);

    /* Allocate and Register Input Device */
    err = qpdss702_input_init(data);
    if (err < 0)
        goto err_init_input_failed;

    /* Create SYSFS Group */
    err = sysfs_create_group(&data->input->dev.kobj, &qpdss702_attribute_group);
    if (err < 0)
        goto err_create_sysfs_failed;


    wake_lock_init(&(data->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

    /* Register Early Suspend */
#ifdef CONFIG_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
    data->early_suspend.suspend = qpdss702_early_suspend;
    data->early_suspend.resume = qpdss702_late_resume;
    register_early_suspend(&data->early_suspend);
#endif

    PSENSOR_DEBUG(LEVEL0, "default mode = %d", data->mode);
    PSENSOR_DEBUG(LEVEL0, "default sensitivity = %d", data->platform_data->sensitivity);
    PSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;

err_create_sysfs_failed:
    qpdss702_input_delete(data);
err_init_input_failed:
    free_irq(data->irq, data);
err_request_irq_failed:
    cancel_delayed_work_sync(&data->work);
err_init_gpio_failed:
err_check_platform_data_failed:
    kfree(data);
err_alloc_kernel_failed:
err_check_functionality_failed:
    PSENSOR_DEBUG(LEVEL0, "Failed.");
    return err;
}

static int qpdss702_remove(struct i2c_client *client)
{
    struct qpdss702_data *data = i2c_get_clientdata(client);

    /* Power down the device */
    qpdss702_set_mode(data->client, SHUTDOWN_MODE);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif
    sysfs_remove_group(&data->input->dev.kobj, &qpdss702_attribute_group);
    qpdss702_input_delete(data);
    free_irq(data->irq, data);
    cancel_delayed_work_sync(&data->work);
    kfree(data);

    PSENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static const struct i2c_device_id qpdss702_id[] = {
    { PSENSOR_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, qpdss702_id);

static struct i2c_driver qpdss702_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = PSENSOR_NAME,
#ifdef CONFIG_PM
        .pm     = &qpdss702_pm_ops,
#endif
    },
    .class      = I2C_CLASS_HWMON,
    .id_table   = qpdss702_id,
    .probe      = qpdss702_probe,
    .remove     = qpdss702_remove,
    .detect     = qpdss702_detect,
};

static int __init qpdss702_init(void)
{
    PSENSOR_DEBUG(LEVEL1, "Done.");
    return i2c_add_driver(&qpdss702_driver);
}

static void __exit qpdss702_exit(void)
{
    PSENSOR_DEBUG(LEVEL1, "Done.");
    i2c_del_driver(&qpdss702_driver);
}

module_init(qpdss702_init);
module_exit(qpdss702_exit);

MODULE_AUTHOR("FIH-SW2-PERIPHERAL FromkerGu <fromkergu@fihspec.com>");
MODULE_DESCRIPTION("AVAGO QPDS-S702 Proximity Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
