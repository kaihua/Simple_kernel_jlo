/* drivers/input/sensor/bma250.c
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
#include <linux/bma250.h>
#include <mach/gpio.h>
/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD
#include <linux/cyttsp_i2c_tma340.h>
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/

#define BMA250_CHIP_ID_REG      0x00
#define BMA250_X_AXIS_LSB_REG   0x02
#define BMA250_X_AXIS_MSB_REG   0x03
#define BMA250_Y_AXIS_LSB_REG   0x04
#define BMA250_Y_AXIS_MSB_REG   0x05
#define BMA250_Z_AXIS_LSB_REG   0x06
#define BMA250_Z_AXIS_MSB_REG   0x07
#define BMA250_RANGE_REG        0x0F
#define BMA250_BANDWIDTH_REG    0x10
#define BMA250_MODE_CTRL_REG    0x11
#define BMA250_RESET_REG        0x14
#define BMA250_EEPROM_CTRL_REG  0x33
#define BMA250_OFFSET1_REG      0x36
#define BMA250_OFFSET2_REG      0x37

#define BMA250_CHIP_ID          3
#define BMA250_NEW_DATA_POS     0
#define BMA250_NEW_DATA_LEN     1
#define BMA250_NEW_DATA_MSK     0x01
#define BMA250_NEW_DATA_REG     BMA250_X_AXIS_LSB_REG
#define BMA250_ACC_LSB_POS      6
#define BMA250_ACC_LSB_LEN      2
#define BMA250_ACC_LSB_MSK      0xC0
#define BMA250_ACC_LSB_REG      BMA250_X_AXIS_LSB_REG
#define BMA250_ACC_MSB_POS      0
#define BMA250_ACC_MSB_LEN      8
#define BMA250_ACC_MSB_MSK      0xFF
#define BMA250_ACC_MSB_REG      BMA250_X_AXIS_MSB_REG
#define BMA250_MODE_SUSPEND     0x80
#define BMA250_RESET            0xB6
#define BMA250_CAL_RDY          0x10
#define BMA250_NVM_RDY          0x04

#define BMA250_POLLING_NUM      10	/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-01+ */
#define BMA250_POLLING_PER_MS   100	/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-01+ */

struct bma250acc {
    s16 x,
        y,
        z;
};

struct bma250cfg {
    unsigned char bw;
    unsigned char mode;
    unsigned char range;
};

struct bma250reg {
    unsigned char addr;
    unsigned char data;
    unsigned char flag;
};

struct bma250_data {
    bool enable;
    bool interrupt;
    bool irq_flag;
    struct bma250_platform_data *platform_data;
    struct bma250acc value;
    struct bma250cfg old_cfg;
    struct bma250cfg cfg;
    struct bma250reg reg;
    struct delayed_work work;
    struct delayed_work irq_work;	/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-00* */
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    struct i2c_client *client;
    struct input_dev *input;
    struct mutex enable_mutex;
    struct mutex value_mutex;
    int irq;
    int polling_times;	/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-00+ */
    unsigned int delay;
};

static const int bma250_direction_mapping[][3][3] = {
    {{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}},	/* top/upper-left */
    {{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}},	/* top/upper-right */
    {{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}},	/* top/lower-right */
    {{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}},	/* top/lower-left */
    {{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}},	/* bottom/upper-right */
    {{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}},	/* bottom/upper-left */
    {{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}},	/* bottom/lower-left */
    {{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}},	/* bottom/lower-right */
};

static int bma250_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
    s32 dummy = i2c_smbus_read_byte_data(client, reg_addr);
    if (dummy < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed.");
        return -EPERM;
    }
    *data = dummy & 0x000000ff;

    return 0;
}

static int bma250_smbus_write_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char data)
{
    s32 dummy = i2c_smbus_write_byte_data(client, reg_addr, data);
    if (dummy < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed.");
        return -EPERM;
    }

    return 0;
}

static int bma250_smbus_read_byte_block(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    s32 dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
    if (dummy < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed.");
        return -EPERM;
    }

    return 0;
}

static int bma250_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }

    return bma250_smbus_read_byte(client, BMA250_BANDWIDTH_REG, BW);
}

static int bma250_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }
    if (bma250->cfg.bw == BW)
        return 0;

    bma250->old_cfg.bw = bma250->cfg.bw;
    bma250->cfg.bw = BW;
    if (bma250_smbus_write_byte(client, BMA250_BANDWIDTH_REG, BW) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    return 0;
}

static int bma250_get_mode(struct i2c_client *client, unsigned char *Mode)
{
    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }

    return bma250_smbus_read_byte(client, BMA250_MODE_CTRL_REG, Mode);
}

static int bma250_set_mode(struct i2c_client *client, unsigned char Mode)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }
    if (bma250->cfg.mode == Mode)
        return 0;

    bma250->old_cfg.mode = bma250->cfg.mode;
    bma250->cfg.mode = Mode;
    if (bma250_smbus_write_byte(client, BMA250_MODE_CTRL_REG, Mode) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    return 0;
}

static int bma250_get_range(struct i2c_client *client, unsigned char *Range)
{
    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }

    return bma250_smbus_read_byte(client, BMA250_RANGE_REG, Range);
}

static int bma250_set_range(struct i2c_client *client, unsigned char Range)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return -EINVAL;
    }
    if (bma250->cfg.range == Range)
        return 0;

    bma250->old_cfg.range = bma250->cfg.range;
    bma250->cfg.range = Range;
    if (bma250_smbus_write_byte(client, BMA250_RANGE_REG, Range) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    return 0;
}

static int bma250_calibration_xyz(struct i2c_client *client, unsigned char XYZ)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);
    unsigned char data = 0;
    int i = 0;

    /* 3. Write 00 in bits[2:1] of register 0x37 =>calibrate on x-axis */
    /* 6. Write 00 in bits[4:3] of register 0x37 =>calibrate on y-axis */
    /* 9. Write 01(1g) or 10(-1g) in bits[6:5] of register 0x37 =>calibrate on z-axis */
    if (XYZ > 2)
    {
        if (bma250->value.z > 0)
            data = 1 << 5;
        else
            data = 1 << 6;
    }
    if (bma250_smbus_write_byte(client, BMA250_OFFSET2_REG, data) < 0)
        return -EIO;

    /* 4. Write 01 in bits[6:5] of register 0x36 */
    /* 7. Write 10 in bits[6:5] of register 0x36 */
    /* 10. Write 11 in bits[6:5] of register 0x36 */
    data = XYZ << 5;
    if (bma250_smbus_write_byte(client, BMA250_OFFSET1_REG, data) < 0)
        return -EIO;

    /* 5. Keep reading bit4 of register 0x36 until the value is 1 */
    /* 8. Keep reading bit4 of register 0x36 until the value is 1 */
    /* 11. Keep reading bit4 of register 0x36 until the value is 1 */
    for (i=0; i<10; i++)
    {
        msleep(100);
        bma250_smbus_read_byte(client, BMA250_OFFSET1_REG, &data);
        ASENSOR_DEBUG(LEVEL1, "Reg[0x%2x] = 0x%2x(%d)", BMA250_OFFSET1_REG, data, i+1);
        if (data & BMA250_CAL_RDY)
            return 0;
    }

    ASENSOR_DEBUG(LEVEL0, "Failed.");
    return -EINVAL;
}

static int bma250_calibration_store(struct i2c_client *client)
{
    unsigned char data = 0;
    int i = 0;

    /* 12. Write 1 in bit0 of register 0x33 =>store calibration data into EEPROM */
    bma250_smbus_read_byte(client, BMA250_EEPROM_CTRL_REG, &data);
    data = data | (1 << 0);
    if (bma250_smbus_write_byte(client, BMA250_EEPROM_CTRL_REG, data) < 0)
        return -EIO;

    /* 13. Write 1 in bit1 of register 0x33 */
    data = data | (1 << 1);
    if (bma250_smbus_write_byte(client, BMA250_EEPROM_CTRL_REG, data) < 0)
        return -EIO;

    /* 14. Keep reading bit2 of register 0x33 until the value is 1 */
    for (i=0; i<10; i++)
    {
        msleep(10);
        bma250_smbus_read_byte(client, BMA250_EEPROM_CTRL_REG, &data);
        ASENSOR_DEBUG(LEVEL1, "Reg[0x%2x] = 0x%2x(%d)", BMA250_EEPROM_CTRL_REG, data, i+1);
        if (data & BMA250_NVM_RDY)
            break;
    }
    if (i==10)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    /* 15. Write 0 in bit0 of register 0x33 and calibration is done. */
    data = data & 0xFE;
    if (bma250_smbus_write_byte(client, BMA250_EEPROM_CTRL_REG, data) < 0)
        return -EIO;

    bma250_smbus_read_byte(client, BMA250_EEPROM_CTRL_REG, &data);
    ASENSOR_DEBUG(LEVEL1, "Reg[0x%2x] = 0x%2x", BMA250_EEPROM_CTRL_REG, data);

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static int bma250_calibration(struct i2c_client *client)
{
    struct bma250cfg cfg_value;

    /* 0. Write 0x00 in register 0x11 =>set default power mode */
    bma250_get_mode(client, &cfg_value.mode);
    bma250_set_mode(client, 0x00);

    /* 1. Write 0x03 in register 0x0F =>set 2g */
    bma250_get_range(client, &cfg_value.range);
    bma250_set_range(client, 0x03);

    /* 2. Write 0x0A in register 0x10 =>set 31Hz */
    bma250_get_bandwidth(client, &cfg_value.bw);
    bma250_set_bandwidth(client, 0x0A);

    bma250_calibration_xyz(client, 1);	/* calibrate on x-axis */
    bma250_calibration_xyz(client, 2);	/* calibrate on y-axis */
    bma250_calibration_xyz(client, 3);	/* calibrate on z-axis */
    bma250_calibration_store(client);	/* store calibration data into EEPROM */

    bma250_set_mode(client, cfg_value.mode);
    bma250_set_range(client, cfg_value.range);
    bma250_set_bandwidth(client, cfg_value.bw);

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static void bma250_set_delay(struct i2c_client *client, unsigned int Delay)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return;
    }
    if (bma250->delay == Delay)
        return;

    bma250->delay = Delay;
    if (bma250->enable && (!bma250->interrupt))
    {
        cancel_delayed_work_sync(&bma250->work);
        schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
    }
}

static void bma250_set_enable(struct i2c_client *client, bool Enable)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return;
    }
    if (bma250->enable == Enable)
        return;

    mutex_lock(&bma250->enable_mutex);
    bma250->enable = Enable;
    if (bma250->enable)
    {
        bma250_set_mode(bma250->client, bma250->old_cfg.mode);
        if (!bma250->interrupt)
        {	/* Polling Mode */
            schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
        }
        else
        {	/* Interrupt Mode */
            if (!bma250->irq_flag)
            {
                bma250->irq_flag = true;
                enable_irq(bma250->irq);
            }
        }
    }
    else
    {
        /* Interrupt Mode */
        if (bma250->irq_flag)
        {
            bma250->irq_flag = false;
            disable_irq(bma250->irq);
        }
        /* Polling Mode */
        cancel_delayed_work_sync(&bma250->work);
        bma250_set_mode(bma250->client, BMA250_MODE_SUSPEND);
    }
    mutex_unlock(&bma250->enable_mutex);

    ASENSOR_DEBUG(LEVEL0, "enable = %d", bma250->enable);
}

static void bma250_set_value(struct i2c_client *client, unsigned int Value)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);

    if (client == NULL)
    {
        ASENSOR_DEBUG(LEVEL0, "NULL.");
        return;
    }
    if (Value != 3112)	//cal
        return;

    if (bma250->enable)
        bma250_calibration(client);
}

static ssize_t bma250_bandwidth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned char data = 0;

    if (bma250_get_bandwidth(bma250->client, &data) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    ASENSOR_DEBUG(LEVEL0, "%d", data);
    return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t bma250_bandwidth_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    if (bma250_set_bandwidth(bma250->client, (unsigned char)data) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    ASENSOR_DEBUG(LEVEL1, "%ld", data);
    return count;
}

#ifdef ASENSOR_DBG
static ssize_t bma250_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    //struct bma250_data *bma250 = dev_get_drvdata(dev);

    ASENSOR_DEBUG(LEVEL0, "%d", asensor_debug_level);
    return snprintf(buf, PAGE_SIZE, "%d\n", asensor_debug_level);
}

static ssize_t bma250_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    asensor_debug_level = data;

    ASENSOR_DEBUG(LEVEL0, "%ld", data);
    return count;
}
#endif

static ssize_t bma250_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    ASENSOR_DEBUG(LEVEL0, "%dms", bma250->delay);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->delay);
}

static ssize_t bma250_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250_set_delay(bma250->client, (unsigned int)data);

    ASENSOR_DEBUG(LEVEL1, "%ldms", data);
    return count;
}

static ssize_t bma250_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    ASENSOR_DEBUG(LEVEL0, "%d", bma250->enable);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->enable);
}

static ssize_t bma250_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250_set_enable(bma250->client, (bool)data);

    ASENSOR_DEBUG(LEVEL1, "%ld", data);
    return count;
}

static ssize_t bma250_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    ASENSOR_DEBUG(LEVEL0, "%d", bma250->interrupt);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->interrupt);
}

static ssize_t bma250_interrupt_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250->interrupt = (bool)data;
    if (!bma250->interrupt)
    {	/* Polling Mode */
        if (bma250->irq_flag)
        {
            bma250->irq_flag = false;
            disable_irq(bma250->irq);
        }
        schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
    }
    else
    {	/* Interrupt Mode */
        cancel_delayed_work_sync(&bma250->work);
        if (!bma250->irq_flag)
        {
            bma250->irq_flag = true;
            enable_irq(bma250->irq);
        }
    }

    ASENSOR_DEBUG(LEVEL1, "%ld", data);
    return count;
}

static ssize_t bma250_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned char data = 0;

    if (bma250_get_mode(bma250->client, &data) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    ASENSOR_DEBUG(LEVEL0, "%d", data);
    return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t bma250_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    if (bma250_set_mode(bma250->client, (unsigned char)data) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    ASENSOR_DEBUG(LEVEL1, "%ld", data);
    return count;
}

static ssize_t bma250_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned char data = 0;

    if (bma250_get_range(bma250->client, &data) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    ASENSOR_DEBUG(LEVEL0, "%d", data);
    return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t bma250_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    if (bma250_set_range(bma250->client, (unsigned char)data) < 0)
        ASENSOR_DEBUG(LEVEL0, "Failed.");

    ASENSOR_DEBUG(LEVEL1, "%ld", data);
    return count;
}

static ssize_t bma250_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);

    if (bma250->reg.flag)
    {
        if (bma250_smbus_read_byte(bma250->client, bma250->reg.addr, &bma250->reg.data) < 0)
            ASENSOR_DEBUG(LEVEL0, "Failed.");
    }

    ASENSOR_DEBUG(LEVEL0, "addr = 0x%2x, data = 0x%2x, flag = %d", bma250->reg.addr, bma250->reg.data, bma250->reg.flag);
    return snprintf(buf, PAGE_SIZE, "%d\n", bma250->reg.data);
}

static ssize_t bma250_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long addr = 0, data = 0, flag = 0;
    int v = 0;
    char **p = argv_split(GFP_KERNEL, buf, &v);
    if (strict_strtoul(p[0], 10, &addr) || strict_strtoul(p[1], 10, &data) || strict_strtoul(p[2], 10, &flag))
    {
        argv_free(p);
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }
    argv_free(p);

    if (flag)
    {
        bma250->reg.addr = addr;
        bma250->reg.data = data;
        bma250->reg.flag = flag;
    }
    else
    {
        if (bma250_smbus_write_byte(bma250->client, (unsigned char)addr, (unsigned char)data) < 0)
            ASENSOR_DEBUG(LEVEL0, "Failed.");

        ASENSOR_DEBUG(LEVEL1, "Reg[0x%2lx] = 0x%2lx", addr, data);
    }

    ASENSOR_DEBUG(LEVEL2, "%s", buf);
    return count;
}

static ssize_t bma250_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    struct bma250acc acc_value;

    mutex_lock(&bma250->value_mutex);
    acc_value = bma250->value;
    mutex_unlock(&bma250->value_mutex);

    ASENSOR_DEBUG(LEVEL0, "x = %3d, y = %3d, z = %3d", acc_value.x, acc_value.y, acc_value.z);
    return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", acc_value.x, acc_value.y, acc_value.z);
}

static ssize_t bma250_value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bma250_data *bma250 = dev_get_drvdata(dev);
    unsigned long data = 0;
    int error = strict_strtoul(buf, 10, &data);
    if (error)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed. data = %s", buf);
        return count;
    }

    bma250_set_value(bma250->client, (unsigned int)data);

    ASENSOR_DEBUG(LEVEL1, "%ld", data);
    return count;
}

static DEVICE_ATTR(bandwidth, 0660, bma250_bandwidth_show, bma250_bandwidth_store);
#ifdef ASENSOR_DBG
static DEVICE_ATTR(debug, 0660, bma250_debug_show, bma250_debug_store);
#endif
static DEVICE_ATTR(delay, 0660, bma250_delay_show, bma250_delay_store);
static DEVICE_ATTR(enable, 0660, bma250_enable_show, bma250_enable_store);
static DEVICE_ATTR(interrupt, 0660, bma250_interrupt_show, bma250_interrupt_store);
static DEVICE_ATTR(mode, 0660, bma250_mode_show, bma250_mode_store);
static DEVICE_ATTR(range, 0660, bma250_range_show, bma250_range_store);
static DEVICE_ATTR(reg, 0660, bma250_reg_show, bma250_reg_store);
static DEVICE_ATTR(value, 0640, bma250_value_show, bma250_value_store);

static struct attribute *bma250_attributes[] = {
    &dev_attr_bandwidth.attr,
#ifdef ASENSOR_DBG
    &dev_attr_debug.attr,
#endif
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_interrupt.attr,
    &dev_attr_mode.attr,
    &dev_attr_range.attr,
    &dev_attr_reg.attr,
    &dev_attr_value.attr,
    NULL
};

static struct attribute_group bma250_attribute_group = {
    .attrs = bma250_attributes
};

static int bma250_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    struct i2c_adapter *adapter = client->adapter;

    if (!adapter)
    {
        ASENSOR_DEBUG(LEVEL0, "Find I2C adapter failed.");
        return -ENOMEM;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
    {
        ASENSOR_DEBUG(LEVEL0, "Check I2C functionality failed.");
        return -ENODEV;
    }

    strlcpy(info->type, dev_name(&client->dev), I2C_NAME_SIZE);

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static int bma250_reset(struct i2c_client *client)
{
    struct bma250_data *bma250 = i2c_get_clientdata(client);
    unsigned char tmp = 0;
    int err = 0;

    err = bma250_smbus_write_byte(client, BMA250_RESET_REG, BMA250_RESET);
    if (err < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed.");
        return err;
    }

    msleep(4);
    bma250_get_bandwidth(client, &tmp);
    bma250->old_cfg.bw = bma250->cfg.bw = tmp;
    bma250_get_mode(client, &tmp);
    bma250->old_cfg.mode = bma250->cfg.mode = tmp;
    bma250_get_range(client, &tmp);
    bma250->old_cfg.range = bma250->cfg.range = tmp;

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return err;
}

static void bma250_read_accel_xyz(struct bma250_data *bma250)
{
    unsigned char buf[6];
    s16 raw[3], data[3];
    int i, j;

    if (bma250_smbus_read_byte_block(bma250->client, BMA250_X_AXIS_LSB_REG, buf, sizeof(buf)) < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Failed.");
        return;
    }

    memset(raw, 0 , sizeof(raw));
    for(i=0; i<3; i++)
    {
        raw[i] = ((buf[i*2] >> BMA250_ACC_LSB_POS) | (buf[i*2+1] << BMA250_ACC_LSB_LEN));
        /* Handle a negative number */
        raw[i] = raw[i] << (sizeof(short)*8-(BMA250_ACC_LSB_LEN+BMA250_ACC_MSB_LEN));
        raw[i] = raw[i] >> (sizeof(short)*8-(BMA250_ACC_LSB_LEN+BMA250_ACC_MSB_LEN));
    }
    memset(data, 0 , sizeof(data));
    for (i=0; i<3; i++)
    {
        for (j=0; j<3; j++)
            data[i] += raw[j] * bma250_direction_mapping[bma250->platform_data->layout][i][j];
    }
    mutex_lock(&bma250->value_mutex);
    bma250->value.x = data[0];
    bma250->value.y = data[1];
    bma250->value.z = data[2];
    mutex_unlock(&bma250->value_mutex);

/*FIH-MTD-PERIPHERAL-CH-ESD-00++[*/
#ifdef CONFIG_FIH_TOUCHSCREEN_CYTTSP_I2C_TMA340_ESD	
	if(bma250->value.z < -200)
		TOUCH_ESD_WORKAROUND(CY_FACE_DOWN);
	else
		TOUCH_ESD_WORKAROUND(CY_FACE_UP);
#endif
/*FIH-MTD-PERIPHERAL-CH-ESD-00++]*/
		
    input_report_abs(bma250->input, ABS_X, bma250->value.x);
    input_report_abs(bma250->input, ABS_Y, bma250->value.y);
    input_report_abs(bma250->input, ABS_Z, bma250->value.z);
    input_sync(bma250->input);
}

static void bma250_delayed_work(struct work_struct *work)
{
    struct bma250_data *bma250 = container_of((struct delayed_work *)work, struct bma250_data, work);

    bma250_read_accel_xyz(bma250);
    schedule_delayed_work(&bma250->work, msecs_to_jiffies(bma250->delay));
}

/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-01*[ */
static void bma250_irq_work(struct work_struct *work)
{
    struct bma250_data *bma250 = container_of((struct delayed_work *)work, struct bma250_data, irq_work);

    if (bma250->enable)
    {
        bma250_read_accel_xyz(bma250);

        if (bma250->polling_times == 0 || bma250->polling_times == BMA250_POLLING_NUM-1)
            ASENSOR_DEBUG(LEVEL0, "%d) x = %3d, y = %3d, z = %3d", bma250->polling_times, bma250->value.x, bma250->value.y, bma250->value.z);
        if (abs(bma250->value.x) > 400 || abs(bma250->value.y) > 400 || abs(bma250->value.z) > 400)
            bma250->polling_times = 0;

        bma250->polling_times++;
        if (bma250->polling_times < BMA250_POLLING_NUM)
            schedule_delayed_work(&bma250->irq_work, msecs_to_jiffies(BMA250_POLLING_PER_MS));
    }
}

static irqreturn_t bma250_irq_handler(int irq, void *dev_id)
{
    struct bma250_data *bma250 = dev_id;

    bma250->polling_times = 0;
    schedule_delayed_work(&bma250->irq_work, msecs_to_jiffies(0));

    return IRQ_HANDLED;
}
/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-01*] */

static int bma250_input_init(struct bma250_data *bma250)
{
    struct input_dev *dev = input_allocate_device();
    int err = 0;

    if (!dev)
    {
        ASENSOR_DEBUG(LEVEL0, "Allocate input device failed.");
        return -ENOMEM;
    }
    dev->name = "accelerometer";
    dev->id.bustype = BUS_I2C;

    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_abs_params(dev, ABS_X, -4096, 4095, 0, 0);
    input_set_abs_params(dev, ABS_Y, -4096, 4095, 0, 0);
    input_set_abs_params(dev, ABS_Z, -4096, 4095, 0, 0);
    input_set_drvdata(dev, bma250);

    err = input_register_device(dev);
    if (err < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Register input device failed.");
        input_free_device(dev);
        return err;
    }
    bma250->input = dev;

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static void bma250_input_delete(struct bma250_data *bma250)
{
    struct input_dev *dev = bma250->input;

    if (!dev)
    {
        ASENSOR_DEBUG(LEVEL0, "Find input device failed.");
        return;
    }

    input_unregister_device(dev);
    input_free_device(dev);
    ASENSOR_DEBUG(LEVEL0, "Done.");
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
    //struct bma250_data *data = container_of(h, struct bma250_data, early_suspend);

    ASENSOR_DEBUG(LEVEL1, "Done.");
    return;
}

static void bma250_late_resume(struct early_suspend *h)
{
    //struct bma250_data *data = container_of(h, struct bma250_data, early_suspend);

    ASENSOR_DEBUG(LEVEL1, "Done.");
    return;
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef CONFIG_PM
static int bma250_suspend(struct device *dev)
{
    //struct bma250_data *data = dev_get_drvdata(dev);

    ASENSOR_DEBUG(LEVEL1, "Done.");
    return 0;
}

static int bma250_resume(struct device *dev)
{
    //struct bma250_data *data = dev_get_drvdata(dev);

    ASENSOR_DEBUG(LEVEL1, "Done.");
    return 0;
}

static struct dev_pm_ops bma250_pm_ops = {
    .suspend = bma250_suspend,
    .resume  = bma250_resume,
};
#endif	/* CONFIG_PM */

static int bma250_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    int tempvalue = 0;
    struct bma250_data *data;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        ASENSOR_DEBUG(LEVEL0, "Check I2C functionality failed.");
        goto exit;
    }

    data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
    if (!data)
    {
        err = -ENOMEM;
        ASENSOR_DEBUG(LEVEL0, "Allocate kernel memory failed.");
        goto exit;
    }

    i2c_set_clientdata(client, data);
    data->platform_data = client->dev.platform_data;
    if (!data->platform_data)
    {
        ASENSOR_DEBUG(LEVEL0, "Check platform data failed.");
        goto kfree_exit;
    }

    err = data->platform_data->gpio_init();
    if (err < 0)
        goto kfree_exit;

    err = bma250_reset(client);
    if (err < 0)
        goto kfree_exit;

    tempvalue = i2c_smbus_read_word_data(client, BMA250_CHIP_ID_REG);
    if ((tempvalue&0x00FF) == BMA250_CHIP_ID)
    {
        ASENSOR_DEBUG(LEVEL0, "Chip ID = 0x%x.", tempvalue);
    }
    else
    {
        ASENSOR_DEBUG(LEVEL0, "Check Chip ID failed.");
        err = -1;
        goto kfree_exit;
    }
    i2c_set_clientdata(client, data);
    data->client = client;
    data->irq = client->irq;
    mutex_init(&data->enable_mutex);
    mutex_init(&data->value_mutex);

    INIT_DELAYED_WORK(&data->work, bma250_delayed_work);
    INIT_DELAYED_WORK(&data->irq_work, bma250_irq_work);	/* FIH-SW2-PERIPHERAL-FG-ASENSOR_INTERRUPT-00* */
    err = request_irq(data->irq, bma250_irq_handler, IRQF_TRIGGER_RISING, dev_name(&client->dev), data);
    if (err < 0)
    {
        ASENSOR_DEBUG(LEVEL0, "Request IRQ(%d) failed.", data->irq);
        goto error_irq;
    }
    disable_irq(data->irq);
    data->delay = 200;
    data->enable = true;
    bma250_set_enable(data->client, false);

    err = bma250_input_init(data);
    if (err < 0)
        goto error_input;

    err = sysfs_create_group(&data->input->dev.kobj, &bma250_attribute_group);
    if (err < 0)
        goto error_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    data->early_suspend.suspend = bma250_early_suspend;
    data->early_suspend.resume = bma250_late_resume;
    register_early_suspend(&data->early_suspend);
#endif

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return 0;

error_sysfs:
    bma250_input_delete(data);
error_input:
    free_irq(data->irq, data);
error_irq:
    cancel_delayed_work_sync(&data->irq_work);
    cancel_delayed_work_sync(&data->work);
kfree_exit:
    kfree(data);
exit:
    ASENSOR_DEBUG(LEVEL0, "Failed.");
    return err;
}

static int bma250_remove(struct i2c_client *client)
{
    struct bma250_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif
    sysfs_remove_group(&data->input->dev.kobj, &bma250_attribute_group);
    bma250_input_delete(data);
    free_irq(data->irq, data);
    cancel_delayed_work_sync(&data->irq_work);
    cancel_delayed_work_sync(&data->work);
    kfree(data);

    ASENSOR_DEBUG(LEVEL0, "Done.");
    return 0;
}

static const struct i2c_device_id bma250_id[] = {
    { ASENSOR_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = ASENSOR_NAME,
#ifdef CONFIG_PM
        .pm     = &bma250_pm_ops,
#endif
    },
    .class      = I2C_CLASS_HWMON,
    .id_table   = bma250_id,
    .probe      = bma250_probe,
    .remove     = bma250_remove,
    .detect     = bma250_detect,
};

static int __init bma250_init(void)
{
    ASENSOR_DEBUG(LEVEL1, "Done.");
    return i2c_add_driver(&bma250_driver);
}

static void __exit bma250_exit(void)
{
    ASENSOR_DEBUG(LEVEL1, "Done.");
    i2c_del_driver(&bma250_driver);
}

module_init(bma250_init);
module_exit(bma250_exit);

MODULE_AUTHOR("FIH-SW2-PERIPHERAL FromkerGu <fromkergu@fihspec.com>");
MODULE_DESCRIPTION("Bosch BMA250 Acceleration Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

