/*
*     smemio.c
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
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
#include "smem_io.h"

int ioctl_operation=0;

static const struct file_operations smemio_dev_fops = {
        .owner = THIS_MODULE,
        .open = smemio_dev_open,                  
        .read = smemio_dev_read,
        .unlocked_ioctl = smemio_dev_ioctl,   
};                              

static struct miscdevice smemio_dev = {
        MISC_DYNAMIC_MINOR,
        "smem_io",
        &smemio_dev_fops
};

static int smemio_dev_open( struct inode * inode, struct file * file )
{
	printk( KERN_INFO "SMEM IO driver open\n" );
	return 0;
}

static long smemio_dev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg )
{

	printk( KERN_INFO "SMEM IO driver ioctl, cmd = %d\n", cmd );
	switch(cmd)
	{
		case SMEM_FIRST_BOOT_NVSYNC:	
			ioctl_operation = cmd;	
			break;		
            
		default:
			printk( KERN_INFO "Undefined SMEM IO driver IOCTL command\n" );
			return -1;
	}
	return 0;
}
   
static ssize_t smemio_dev_read( struct file * file, char __user * buffer, size_t size, loff_t * f_pos )
{
    int *NVStatus;
    int status=0;	

    switch(ioctl_operation)
    {	
        case SMEM_FIRST_BOOT_NVSYNC:
        NVStatus = smem_alloc(SMEM_FIRST_BOOT_NVSYNC, sizeof(int));
        if(NVStatus==NULL)
        {
            NVStatus=kmalloc(sizeof(int),GFP_KERNEL);
            memset(NVStatus, 0, sizeof(int));
            status=1;
        }
			
        if(copy_to_user(buffer, NVStatus, sizeof(int)))
        {                
            printk( KERN_INFO "copy_to_user() fail!\n" );
            /*SP-AlwaysChen-FixCoverity-01+[*/
            if(status==1)
                kfree(NVStatus);
            /*SP-AlwaysChen-FixCoverity-01+]*/
            return -EFAULT;
        }
        if(status==1)                
            kfree(NVStatus);

        break;
            default:
        printk( KERN_INFO "Undefined SMEM IO driver IOCTL command\n" );
        return -1;
    }
    return 0;
}

 static int __init smemio_init(void)
{
    int ret;

    printk( KERN_INFO "SMEM IO Driver init\n" );
	ret = misc_register(&smemio_dev);
	if (ret){
		printk(KERN_WARNING "SMEM IO Unable to register misc device.\n");
		return ret;
	}
    return ret;
        
}

static void __exit smemio_exit(void)                         
{                                                                              
    printk( KERN_INFO "SMEM IO Driver exit\n" );          
    misc_deregister(&smemio_dev);
}                                                        
                                                         
module_init(smemio_init);                            
module_exit(smemio_exit);                                

MODULE_DESCRIPTION( "SMEM IO driver" );                
MODULE_LICENSE( "GPL" );                                 



















