/*************************************************************************************
 *
 * FIH Project
 *
 * General Description
 * 
 * Linux kernel: Definitions of SW related information for all project.
 *
 * Copyright (C) 2011, Foxconn Corporation (Eric Liu <huaruiliu@fihspec.com>)
 * Copyright (C) 2011-2012, Foxconn International Holdings, Ltd. All rights reserved.
 *
 */
/*************************************************************************************/
#ifndef _LINUX_FIH_SW_INFO_H
#define _LINUX_FIH_SW_INFO_H

#include "fih_hw_info.h"
/*
 * This file has definitions of SW related information for SEMC.
 * SMEC naming rule: $PartNumber_$SwRevision
 */

#define SEMC_PartNumber		"S_PartNumber"
#define SEMC_SwRevison		"S_SwRevision"
#define SEMC_ReleaseFlag	"S_ReleaseFlag"
#define SEMC_BlobVersion	"S_BlobVersion"
#define SEMC_BpVersion		"S_BpVersion"
#define SEMC_SvnVersion		"S_SvnVersion"
#define SEMC_ProductName	"S_ProductModelName"

/*MTD-BSP-AC-RESERVE_MEM-01+[*/
/* MTD-BSP-VT-S1_RESERVED_MEM-00*[ */
#define MTD_RAM_CONSOLE_ADDR  0x0F000000
#define MTD_RAM_CONSOLE_SIZE  0x00100000
#define MTD_MLOG_ADDR         0x0F100000
#define MTD_MLOG_SIZE         0x00100000
#define MTD_FB_SPARE_ADDR     0x0F200000
#define MTD_FB_SPARE_SIZE     0x00100000
/* MTD-BSP-VT-S1_RESERVED_MEM-00*] */

/*MTD-BSP-AC-RESERVE_MEM-01+]*/

// MTD-KERNEL-EL_upgrade_done_00+[
/*===========================================================================
                        FIH PWRON CAUSE
===========================================================================*/
#define MTD_PWR_ON_EVENT_PWR_OFF_CHG_REBOOT		0x00100000	// power off charging reset system
#define MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR		0x10000000	// modem fatal error
#define MTD_PWR_ON_EVENT_KERNEL_PANIC			0x20000000	// host panic
#define MTD_PWR_ON_EVENT_MODEM_ABNORMAL_RESET	0x40000000	// modem hardware watchdog reset
#define MTD_PWR_ON_EVENT_SOFTWARE_RESET			0x80000000  // host normal reset
#define MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET		0x00200000	// modem software watchdog reset
#define MTD_PWR_ON_EVENT_MODEM_UPGRADE_DONE		0x00400000	// upgrade done // MTD-KERNEL-EL_upgrade_done_00+
#define PM_PWR_ON_EVENT_KEYPAD                  0x00000001
#define PM_PWR_ON_EVENT_RTC                     0x00000002
#define PM_PWR_ON_EVENT_CABLE                   0x00000004
#define PM_PWR_ON_EVENT_SMPL                    0x00000008
#define PM_PWR_ON_EVENT_WDOG                    0x00000010
#define PM_PWR_ON_EVENT_USB_CHG                 0x00000020
#define PM_PWR_ON_EVENT_WALL_CHG                0x00000040
// MTD-KERNEL-EL_upgrade_done_00+]

/* FIH-SW3-KERNEL-EL-write_panic_file-00+[ */ 
/*===========================================================================
                        FIH RAM console definition
===========================================================================*/
//MTD-SW3-KERNEL-DL-Fix_ioremap-00+[ 
//[WARNING]When CONFIG_ANDROID_RAM_CONSOLE porting is ready, please remove this workaround.
#define RAM_CONSOLE_PHYS MTD_RAM_CONSOLE_ADDR
//MTD-SW3-KERNEL-DL-Fix_ioremap-00+] 
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	#define RAM_CONSOLE_PHYS MTD_RAM_CONSOLE_ADDR
	#define RAM_CONSOLE_SIZE 0x00020000

#ifdef CONFIG_FEATURE_FIH_SW3_LAST_ALOG

	#define ALOG_RAM_CONSOLE_PHYS_MAIN (RAM_CONSOLE_PHYS + RAM_CONSOLE_SIZE) 

	#define ALOG_RAM_CONSOLE_SIZE_MAIN 0x00020000 /*128KB */
	
	/* 0x20040000 */
	#define ALOG_RAM_CONSOLE_PHYS_RADIO (ALOG_RAM_CONSOLE_PHYS_MAIN +  ALOG_RAM_CONSOLE_SIZE_MAIN)
	#define ALOG_RAM_CONSOLE_SIZE_RADIO 0x00020000 /*128KB */
	
	/* 0x20060000 */
	#define ALOG_RAM_CONSOLE_PHYS_EVENTS (ALOG_RAM_CONSOLE_PHYS_RADIO + ALOG_RAM_CONSOLE_SIZE_RADIO) 
	#define ALOG_RAM_CONSOLE_SIZE_EVENTS 0x00020000 /*128KB */
	
	/* 0x20080000 */
	#define ALOG_RAM_CONSOLE_PHYS_SYSTEM (ALOG_RAM_CONSOLE_PHYS_EVENTS + ALOG_RAM_CONSOLE_SIZE_EVENTS) 
	#define ALOG_RAM_CONSOLE_SIZE_SYSTEM 0x00020000 /*128KB */
#endif /* end of #ifdef CONFIG_FEATURE_FIH_SW3_LAST_ALOG */
#endif /* end of #ifdef CONFIG_ANDROID_RAM_CONSOLE */
/*===========================================================================
                        FIH RAM panic data definition
===========================================================================*/
#define CONFIG_FEATURE_FIH_SW3_PANIC_FILE
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE

  #ifdef CONFIG_FEATURE_FIH_SW3_LAST_ALOG
  	#define PANIC_RAM_DATA_BEGIN (ALOG_RAM_CONSOLE_PHYS_SYSTEM + ALOG_RAM_CONSOLE_SIZE_SYSTEM)
  #else
  	#ifdef 	CONFIG_ANDROID_RAM_CONSOLE
  		#define PANIC_RAM_DATA_BEGIN (RAM_CONSOLE_PHYS + RAM_CONSOLE_SIZE)
  	#else
  		#define PANIC_RAM_DATA_BEGIN MTD_RAM_CONSOLE_ADDR
  	#endif	
  #endif
  
  #define PANIC_RAM_SIGNATURE  0x19761105
  #define PANIC_RAM_DATA_SIZE  0x00020000 /* 128KB */
  
  /* FIH-SW3-KERNEL-TH-TimestampOnRAMDump-01+[ */
  #define	CRASH_TIME_RAMDUMP_ADDR		(RAM_CONSOLE_PHYS + 0x100000 - 0x10)
  #define	CRASH_TIME_RAMDUMP_LEN		0x10
  /* FIH-SW3-KERNEL-TH-TimestampOnRAMDump-01-[ */
  
  struct fih_panic_ram_data{
    unsigned int              signature;
    unsigned int              length;
    char                      data[1];
  } ;  
#endif /* end of 	CONFIG_FEATURE_FIH_SW3_PANIC_FILE */
/* FIH-SW3-KERNEL-EL-write_panic_file-00+] */ 

#endif
