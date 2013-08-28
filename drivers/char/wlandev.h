/*
*     wlandev.h
*
*     Copyright (C) 2009 Clement Hsu <clementhsu@tp.cmcs.com.tw>
*     Copyright (C) 2009 Chi Mei Communication Systems Inc.
*
*     This program is free software; you can redistribute it and/or modify
*     it under the terms of the GNU General Public License as published by
*     the Free Software Foundation; version 2 of the License.
*/

#define IOCTL_HOTSPOT_SET  61

static int wlandev_open( struct inode * inode, struct file * file );

static ssize_t wlandev_read( struct file * file, char __user * buffer, size_t size, loff_t * f_pos );

static int wlandev_write( struct file * filp, const char __user * buffer, size_t length, loff_t * offset );

static long wlandev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg );

static int __init wlandev_init(void);

static void __exit wlandev_exit(void);                         

