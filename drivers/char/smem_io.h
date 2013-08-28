/*
*     smemio.h - GPIO access driver header for FTM use
*
*     Copyright (C) 2009 Clement Hsu <clementhsu@tp.cmcs.com.tw>
*     Copyright (C) 2009 Chi Mei Communication Systems Inc.
*
*     This program is free software; you can redistribute it and/or modify
*     it under the terms of the GNU General Public License as published by
*     the Free Software Foundation; version 2 of the License.
*/

static int smemio_dev_open( struct inode * inode, struct file * file );

static ssize_t smemio_dev_read( struct file * file, char __user * buffer, size_t size, loff_t * f_pos );

static long smemio_dev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg );

static int __init smemio_init(void);

static void __exit smemio_exit(void);    