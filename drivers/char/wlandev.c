/*
*     wlandev.c
*
*     Copyright (C) 2009 Clement Hsu <clementhsu@tp.cmcs.com.tw>
*     Copyright (C) 2009 Chi Mei Communication Systems Inc.
*
*     This program is free software; you can redistribute it and/or modify
*     it under the terms of the GNU General Public License as published by
*     the Free Software Foundation; version 2 of the License.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "wlandev.h"
#include <mach/proc_comm.h>


static const struct file_operations wlandev_fops = {
        .open = wlandev_open,                   
        .read = wlandev_read,                   
        .write = wlandev_write,                   
        .unlocked_ioctl = wlandev_ioctl,                 
};                              

static struct miscdevice wlandev_dev = {
        MISC_DYNAMIC_MINOR,
        "wlandev",
        &wlandev_fops
};

static int wlandev_open( struct inode * inode, struct file * file )
{
	printk( KERN_INFO "WLAN manager open\n" );

	if( ( file->f_flags & O_ACCMODE ) == O_WRONLY )
	{
		printk( KERN_INFO "wlandev driver device node is readonly\n" );
		return -1;
	}
	else
		return 0;
}

static ssize_t wlandev_read( struct file * file, char __user * buffer, size_t size, loff_t * f_pos )
{
	return 0;
}

static int wlandev_write( struct file * filp, const char __user * buffer, size_t length, loff_t * offset )
{	
	return 0;
}	


static long wlandev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg )
{

	printk( KERN_INFO "Wlan device driver ioctl, cmd = %d, arg = %ld\n", cmd , arg);
	
	switch(cmd)
	{
	
		case IOCTL_HOTSPOT_SET:
			printk("[Hotspot] set hotspot state\n");
			proc_comm_hotspot_sar_control(arg);
			break;

		default:
			printk( KERN_INFO "Undefined Wlan device driver IOCTL command\n" );
			return -1;
	}

	return 0;
}

 static int __init wlandev_init(void)
{
        int ret;

        printk( KERN_INFO "WLAN manager init\n" );

	ret = misc_register(&wlandev_dev);
	if (ret){
		printk(KERN_WARNING "Wlandev Unable to register misc device.\n");
		return ret;
	}

        return ret;
        
}

static void __exit wlandev_exit(void)                         
{                                                                              
        printk( KERN_INFO "WLAN manager exit\n" );          
        misc_deregister(&wlandev_dev);
}                                                        
                                                         
module_init(wlandev_init);                            
module_exit(wlandev_exit);                                
                                                         
MODULE_AUTHOR( "Clement Hsu <clementhsu@tp.cmcs.com.tw>" );
MODULE_DESCRIPTION( "WLAN manager driver" );                
MODULE_LICENSE( "GPL" );
