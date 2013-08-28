/*
* Copyright (C) 2011 FIH Co., Ltd.	All rights reserved.
* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
* GNU General Public License for more details.
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/fih_hw_info.h>
#include <linux/export.h>

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

void *sisi_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}
EXPORT_SYMBOL(sisi_wifi_mem_prealloc);

#define GUA_WL_HOST_WAKE 92
#define TAP_WL_HOST_WAKE 83
#define TAP_WL_HOST_WAKE_FIX 112
#define MES_WL_HOST_WAKE 83  //MTD-COMM-CD-OOB_FOR_MES-00+
#define MES_WL_HOST_WAKE_FIX 112
#define JLO_WL_HOST_WAKE 83
#define JLO_WL_HOST_WAKE_FIX 112

int sisi_get_host_wlan_irq(void)
{
  #ifdef CONFIG_FIH_MACH_TAMSUI_GUA
    return GUA_WL_HOST_WAKE;
  #elif defined(CONFIG_FIH_MACH_TAMSUI_TAP)
  if(fih_get_product_phase() < Phase_AP)
    return TAP_WL_HOST_WAKE;
  else
    return TAP_WL_HOST_WAKE_FIX;
  //MTD-COMM-CD-OOB_FOR_MES-00+[
  #elif defined(CONFIG_FIH_MACH_TAMSUI_MES)
  if(fih_get_product_phase() < Phase_SP)
    return MES_WL_HOST_WAKE;
  else
    return MES_WL_HOST_WAKE_FIX;
  //MTD-COMM-CD-OOB_FOR_MES-00+]
  #elif defined(CONFIG_FIH_MACH_TAMSUI_JLO)
  if(fih_get_product_phase() < Phase_SP2)
    return JLO_WL_HOST_WAKE;
  else
  	return JLO_WL_HOST_WAKE_FIX;
  #else
    return -1;
  #endif
}
EXPORT_SYMBOL(sisi_get_host_wlan_irq);

static int __init sisi_init_wifi_mem(void)
{
	int i;

    printk(KERN_INFO "%s: dev_alloc_skb 4096*2/8192*2\n", __func__);  //MTD_CONN_EC_HOTSPOT-EnlargeStaticBuffer-01+
	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096 * 2);   //MTD_CONN_EC_HOTSPOT-EnlargeStaticBuffer-01*
		else
			wlan_static_skb[i] = dev_alloc_skb(8192 * 2);   //MTD_CONN_EC_HOTSPOT-EnlargeStaticBuffer-01*
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

static int __init sisi_wifi_init(void)
{
	int ret = 1;

	printk("%s: start\n", __func__);
	sisi_init_wifi_mem();

    return ret;
}

late_initcall(sisi_wifi_init);
