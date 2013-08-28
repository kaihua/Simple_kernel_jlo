/* include/linux/qpdss702.h
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

#ifndef LINUX_QPDSS702_MODULE_H
#define LINUX_QPDSS702_MODULE_H

#define PSENSOR_NAME            "qpdss702"
#define GPIO_ALPS_OUT           122

#define PSENSOR_DEFAULT_DELAY   (200)	/* 200 ms */
#define PSENSOR_MAX_DELAY       (1000)	/* 1000 ms */

#define EVENT_TYPE_PROXIMITY    ABS_X

/*
AVAGO QPDS-S702
Register definition:

Bit     15   14   13   12   11   10   9   8   7    6    5    4    3    2    1   0
        TRG  PWR  NB3  NB2  NB1  NB0  F1  F0  DC1  DC0  TH3  TH2  TH1  TH0  R1  R0
Default 0    0    0    0    0    1    0   0   1    0    0    1    0    0    0   0
*/
#define QPDSS702_SHUTDOWN_MODE  0x0000
#define QPDSS702_DEFAULT_MODE   0xC490	/* 1 1 0001 00 10 0100 00 Active operations, poweron, 32-pulse, 12.5KHz, 1:128, 0.32V threshold */
#define QPDSS702_TYPICAL_MODE   0xC491	/* 1 1 0001 00 10 0100 01 32pulse, 12.5kHz, 1:128 duration cycle, Threshold 0.32V, Filter Resistor 100kohm */
//#define QPDSS702_TYPICAL_MODE   0xC4A1	/* 1 1 0001 00 10 1000 01 32pulse, 12.5kHz, 1:128 duration cycle, threshold 0.52V, Filter Resistor 100kohm */
//#define QPDSS702_TYPICAL_MODE   0xC4A5	/* 1 1 0001 00 10 1001 01 32pulse, 12.5kHz, 1:128 duration cycle, threshold 0.57V, Filter Resistor 100kohm */
//#define QPDSS702_TYPICAL_MODE   0XC4BD	/* 1 1 0001 00 10 1111 01 32pulse, 12.5kHz, 1:128 duration cycle, threshold 0.87V, Filter Resistor 100kohm */
//#define QPDSS702_TYPICAL_MODE   0xC4BF	/* 1 1 0001 00 10 1111 11 32pulse, 12.5kHz, 1:128 duration cycle, threshold 0.87V, Filter Resistor 500kohm */
#define QPDSS702_LONG_MODE      0xDC90	/* 11 0111 00 10 0100 00  Active operation, 128-pulse, 12.5KHz, 0.32V threshold */
#define QPDSS702_THRESHOLD_MASK 0x3C

/*FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-02+{*/
/*The smaller value, the more sensitive*/
enum {
    QPDSS207_SENSITIVITY_LEVEL0=0,
    QPDSS207_SENSITIVITY_LEVEL1,
    QPDSS207_SENSITIVITY_LEVEL2,
    QPDSS207_SENSITIVITY_LEVEL3,
    QPDSS207_SENSITIVITY_LEVEL4,
    QPDSS207_SENSITIVITY_LEVEL5,
    QPDSS207_SENSITIVITY_LEVEL6,
    QPDSS207_SENSITIVITY_LEVEL7,
    QPDSS207_SENSITIVITY_LEVEL8,
    QPDSS207_SENSITIVITY_LEVEL9,
    QPDSS207_SENSITIVITY_LEVEL10,
    QPDSS207_SENSITIVITY_LEVEL11,
    QPDSS207_SENSITIVITY_LEVEL12,
    QPDSS207_SENSITIVITY_LEVEL13,
    QPDSS207_SENSITIVITY_LEVEL14,
    QPDSS207_SENSITIVITY_LEVEL15,
    QPDSS207_SENSITIVITY_END,
};
/*FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-02+}*/

struct qpdss702_platform_data {
    int (*gpio_init)(void);
    int sensitivity;
};

enum {
    SHUTDOWN_MODE = 0,
    DEFAULT_MODE,
    TYPICAL_MODE,
    LONG_MODE,
    TOTAL_MODE_NUM,
};

static const u16 QPDSS702_MODE[] = {
    [SHUTDOWN_MODE] = QPDSS702_SHUTDOWN_MODE,
    [DEFAULT_MODE] = QPDSS702_DEFAULT_MODE,
    [TYPICAL_MODE] = QPDSS702_TYPICAL_MODE,
    [LONG_MODE] = QPDSS702_LONG_MODE,
};

#define LEVEL0 0
#define LEVEL1 1
#define PSENSOR_DBG
static int psensor_debug_level = 0;
#ifdef PSENSOR_DBG
#define PSENSOR_DEBUG(level, fmt, ...) \
    do { \
        if (level <= psensor_debug_level) \
            printk("[PSENSOR] %s(%d): " fmt "\n", __FUNCTION__, __LINE__, ## __VA_ARGS__); \
    } while(0)
#else
#define PSENSOR_DEBUG(level, fmt, ...)
#endif

#endif	/* LINUX_QPDSS702_MODULE_H */
