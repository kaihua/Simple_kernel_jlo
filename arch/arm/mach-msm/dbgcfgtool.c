/**+===========================================================================
  File: dbgcfgtool.c

  Description:


  Note:


  Author:
        Chance Li Mar-12-2009
-------------------------------------------------------------------------------
** FIHSPEC CONFIDENTIAL
** Copyright(c) 2009 FIHSPEC Corporation. All Rights Reserved.
**^M
** The source code contained or described herein and all documents related
** to the source code (Material) are owned by FIHSPEC Technology Corporation.
** The Material is protected by worldwide copyright and trade secret laws and
** treaty provisions. No part of the Material may be used, copied, reproduced,
** modified, published, uploaded, posted, transmitted, distributed, or disclosed
** in any way without FIHSPEC prior express written permission.
============================================================================+*/

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <mach/proc_comm.h>
#include "mach/dbgcfgtool.h"
//#include <linux/smp_lock.h>
#include "mach/rpc_nv.h"	//MTD-SW3-KERNEL-MP-RPC_Read_NV-00+

/* #define DEBUG */

/* FIH-SW-KERNEL-HC-TCXO_SD_DURING_DISPLAY_ON-01+[ */
#ifdef CONFIG_FIH_SW_TCXO_SD_DURING_DISPLAY_ON
extern unsigned int fih_set_tcxo_sd_during_display_on(unsigned int flag);
#endif
/* FIH-SW-KERNEL-HC-TCXO_SD_DURING_DISPLAY_ON-01+] */

static char* write_buf;
#define DBGCFG_COMMAND_BUFFER_SIZE 128
#define TEMP_BUF_SIZE 10

#define DEVICE_NAME "dbgcfgtool"

static unsigned int dbgcfg_data_has_initialized = 0;

unsigned int debug_cpu_usage_enable = 0;
unsigned int debug_rpcmsg_enable = 0;

#include <mach/msm_rpcrouter.h>
#define NVPROG    0x3000000E
#define NVVERS    0x00090002  
#define ONCRPC_NV_CMD_REMOTE_PROC 9

typedef struct {
        struct rpc_request_hdr hdr;
        unsigned int cmd;
        unsigned int item;
        unsigned int more_data; 
        unsigned int disc;
        unsigned int data;
} error_action_req;

typedef struct {
        struct rpc_reply_hdr hdr;
        unsigned int retval;
        unsigned int more_data; 
        unsigned int disc;
        unsigned int data;
} error_action_rep;

#define NV_READ_F    0          /* Read item */
#define NV_WRITE_F    1         /* Write item */
#define NV_DONE_S    0

/*--------------------------------------------------------------------------* 
* NOTE on ERR_FATAL_OPTIONS NV item:
* For the code below, interpretation of the 16-bit NV item is as follows:
*
* Bits 15:8 -- Timeout value (in seconds)
* Bits 7:4  -- Timeout option
* Bits 3:0  -- Auto action
*
* Builds not using the time fields will be unaffected as the auto-action
* field is unchanged from previous usage.
*--------------------------------------------------------------------------*/
#define NV_ERR_FATAL_OPTIONS_I    905
#define ACTION_MASK  0x0000000F
static unsigned int error_fatal_option = 0;
static int error_fatal_option_has_initialized = 0;

/*--------------------------------------------------------------------------* 
 * The configuration array to be mirrored from the array in storage.
 * It can help to reduce the storage read and write times.
 * localcfg:(4 int)
 *
 *   |-- group0 (1 int) --|-- group1 (1 int) --|-- group2 (1 int) --|-- group3 (3 bytes) --|-- CheckSUM (1 byte) --|
 *   or
 *   |-- bit0, bit1, ........................................................... ,bit119 --|-- CheckSUM (1 byte) --|
 *
 *   configurations:
 *       Magic number : 0xFFFF FFFF FFFF FFFF FFFF FFFF FFFF FFFF is the default value.
 *       enable       : bit(n) = 1
 *       disable      : bit(n) = 0
 *       , and n = 0 ~ 119
 *--------------------------------------------------------------------------*/
static char localcfg[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

int (*pfnWriteDbgConfiguration)(unsigned char*);
int (*pfnReadDbgConfiguration) (unsigned char*);

static bool fihdbg_cached_flag = false;
int cached_read_fihdbg(unsigned char *fihdbg)
{
	static unsigned char cached_fihdbg[16];
	int retval = 0;;

	if (fihdbg_cached_flag==true) {
		pr_debug("%s(): cached !!!\n", __func__);
		memcpy(fihdbg, cached_fihdbg, sizeof(cached_fihdbg));
	} else {
		retval = rpc_nv_read_fihdbg_config(fihdbg);
		if (retval==0) {
			memcpy(cached_fihdbg, fihdbg, sizeof (cached_fihdbg));
			fihdbg_cached_flag = true;
		}
	}
	return retval;
}
EXPORT_SYMBOL(cached_read_fihdbg);

int cached_write_fihdbg(unsigned char *fihdbg)
{
	int retval = 0;;

	retval = rpc_nv_write_fihdbg_config(fihdbg);
	if (retval==0) {
		fihdbg_cached_flag = false;
	}
	return retval;
}
EXPORT_SYMBOL(cached_write_fihdbg);
/*--------------------------------------------------------------------------*
 * Function    : DbgCfgGetByGroup
 *
 * Description :
 *     Read fih debug configuration settings by group.
 * 
 * Parameters  :
 *
 *     gid    - a group id between 0 ~ 3.
 *     gvalue - the value according to the chosen group id returned.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int DbgCfgGetByGroup(unsigned int gid,unsigned int* gvalue)
{
    int   ret = 0;
    char* buf = (char*) gvalue;

    if (!dbgcfg_data_has_initialized)
    {
        pr_err("[dbgcfgtool] %s() LINE:%d, DbgCfg data isn't initialized\n", __func__, __LINE__);
        return -EACCES;
    }

    if ( (gid < (FIH_DEBUG_CMD_DATA_SIZE-1)) && (buf!=NULL) )
    {
        memcpy(buf, localcfg+(gid*sizeof(unsigned int)), sizeof(unsigned int));
    }
    else
    {
        ret = -EINVAL;
        pr_err("[dbgcfgtool] %s() LINE:%d, invalid arguments.(gid: %d, gvalue: 0x%x)\n", __func__, __LINE__, gid, (unsigned int)gvalue);
    }

    return ret;
}

/*--------------------------------------------------------------------------*
 * Function    : DbgCfgGetByBit
 *
 * Description :
 *     Read fih debug configuration settings by bit.
 *
 * Parameters  :
 *
 *     bid    - a bit id between 0 ~ 119.
 *     bvalue - the value according to the chosen bit id returned. ( 0 or 1 )
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int DbgCfgGetByBit(unsigned int bid,unsigned int *bvalue)
{
    int   ret = 0;

    if (!dbgcfg_data_has_initialized)
    {
        pr_err("[dbgcfgtool] %s() LINE:%d, DbgCfg data isn't initialized\n", __func__, __LINE__);
        return -EACCES;
    }

    if ( (bid < 8*(FIH_DEBUG_CMD_DATA_SIZE-1)*sizeof(unsigned int)) && (bvalue!=NULL) )
    {
        *bvalue = ( localcfg[bid/8] & (1<<(bid%8)) ) ? 1 : 0;
    }
    else
    {
        ret = -EINVAL;
        pr_err("[dbgcfgtool] %s() LINE:%d, invalid arguments.(bid: %d, bvalue: 0x%x)\n", __func__, __LINE__, bid, (unsigned int)bvalue);
    }

    return ret;
}

/*--------------------------------------------------------------------------*
 * Function    : GenerateCheckSumByChar
 *
 * Description :
 *     Calculate the check sume value in one byte from the input string.
 *
 * Parameters  :
 *
 *     str    - a string to be calculated.
 *     size   - the size . ( 0 or 1 )
 *
 * Return value: char
 *
 *     check sum value in byte
 *--------------------------------------------------------------------------*/
char GenerateCheckSumByChar(char* str, int size)
{
    char summary = 0;
    int i;

    if( str != NULL)
    {
        for(i=0;i<size;i++) 
        {
            summary+=str[i];
        }
        summary = ~summary;
    }
    return summary;
}


/*--------------------------------------------------------------------------*
 * Function    : DbgCfgSetByGroup
 *
 * Description :
 *     Write the fih debug configuration settings by by group.
 *     Step:
 *          a) prepare the new setting and calculate the check sum.
 *          b) write the new settings into storage.
 *          c) read the new setting from storage and check again.
 *          d) store the new setting in the local mirror, localcfg.
 *
 * Parameters  :
 *
 *     gid    - a group id between 0 ~ 3.
 *     gvalue - the value according to the chosen group id needs to be updated.
 *
 * Return value: integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int DbgCfgSetByGroup(unsigned int gid,unsigned int gvalue)
{
    int ret = 0;
    char* buf = (char*) &gvalue;
    char  tempcfg[FIH_DEBUG_CFG_LEN];
    char  chkcfg[FIH_DEBUG_CFG_LEN];
    unsigned int *pointer;
					
    if (!dbgcfg_data_has_initialized)
    {
        pr_err("[dbgcfgtool] %s() LINE:%d, DbgCfg data isn't initialized\n", __func__, __LINE__);
        return -EACCES;
    }

    if ( (gid < (FIH_DEBUG_CMD_DATA_SIZE-1)) && (buf!=NULL) )
    {
	pointer = (unsigned int *)localcfg;
 	pr_debug("[dbgcfgtool] DbgCfgSetByGroup before write > localcfg: 0x%08x 0x%08x 0x%08x 0x%08x\n", pointer[0], pointer[1], pointer[2], pointer[3]);
    
        memcpy(tempcfg,localcfg,FIH_DEBUG_CFG_LEN);
        memcpy(tempcfg+(gid*sizeof(unsigned int)), buf, sizeof(unsigned int));
        tempcfg[FIH_DEBUG_CFG_LEN-1] = GenerateCheckSumByChar(tempcfg,FIH_DEBUG_CFG_LEN-1);
        ret = pfnWriteDbgConfiguration(tempcfg);
        if( ret != 0 )
        {
            pr_err("[dbgcfgtool] %s() LINE:%d, Fail to write debug configurations.(ret: %d)\n", __func__, __LINE__, ret);
        }
        else
        {
            ret = pfnReadDbgConfiguration(chkcfg);
            if( ret != 0 )
            {
                pr_err("[dbgcfgtool] %s() LINE:%d, Fail to read debug configurations.(ret: %d)\n", __func__, __LINE__, ret);
            }
            else
            {
                if( memcmp(tempcfg,chkcfg,FIH_DEBUG_CFG_LEN) == 0 )
                {
                    memcpy(localcfg,tempcfg,FIH_DEBUG_CFG_LEN);
                    pointer = (unsigned int *)localcfg;
    		    pr_debug("[dbgcfgtool] DbgCfgSetByGroup after read > localcfg: 0x%08x 0x%08x 0x%08x 0x%08x\n", pointer[0], pointer[1], pointer[2], pointer[3]);
                }
                else
                {
                    pr_err("[dbgcfgtool] %s() LINE:%d, data crash in storage.\n", __func__, __LINE__);
                }
            }
        }
    }
    else
    {
        ret = -EINVAL;
        pr_err("[dbgcfgtool] %s() LINE:%d, invalid arguments.(gid: %d, gvalue: %d)\n", __func__, __LINE__, gid, gvalue);
    }

    return ret;
}

/*--------------------------------------------------------------------------*
 * Function    : DbgCfgGetErrorAction
 *
 * Description :
 *     Read the ARM9 fatal error auto action setting fron NV item #905.
 *
 * Parameters  :
 *
 *     p_value - the pointer that indicate the value of error action. (the last byte of NV item #905)
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int DbgCfgGetErrorAction(int* p_value)
{

	int ret = -1; 
	
	ret = rpc_nv_read_err_fatal_option(p_value); //MTD-SW3-KERNEL-MP-RPC_read_NV*

	if (ret == 0){
		error_fatal_option = *p_value;
		pr_info("[dbgcfgtool] DbgCfgGetErrorAction: error_fatal_option = 0x%x\n", error_fatal_option);
	}
	else
		pr_err("[dbgcfgtool] DbgCfgGetErrorAction: read fail 0x%x\n", ret);
	
    return ret;
    
}


/*--------------------------------------------------------------------------*
 * Function    : DbgCfgSetErrorAction
 *
 * Description :
 *     Write the ARM9 fatal error auto action setting to NV item #905.
 *     Also write modem debug NV correspondingly to make sure valid RAM dump can be generated.
 *
 * NV Item	Name				AutoDownload Action	Reset Action	No Action
 * 905		NV_ERR_FATAL_OPTIONS_I		0			1		2
 * 4399		NV_DETECT_HW_RESET_I		1			0		0
 * 6470		NV_CACHE_WT_I			1			0		0
 *
 * Parameters  :
 *
 *     value - the desired error auto action.
 *
 * Return value: integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int DbgCfgSetErrorAction(int value)
{
    unsigned int tmp_error_fatal_option = 0;
    int ret = -1;

//MTD-SW3-KERNEL-MP-RPC_Read_NV*[
    tmp_error_fatal_option = (error_fatal_option & ~ACTION_MASK) | value;

    if (!value) 
    {
    	ret = rpc_nv_write_detect_hw_reset(1);
    	ret = rpc_nv_write_cache_wt(1);
        ret = rpc_nv_write_err_fatal_option(tmp_error_fatal_option);
    }
    else
    {
    	ret = rpc_nv_write_detect_hw_reset(0);
    	ret = rpc_nv_write_cache_wt(0);
        ret = rpc_nv_write_err_fatal_option(tmp_error_fatal_option);        
    }	
//MTD-SW3-KERNEL-MP-RPC_Read_NV*]
	return ret;
}

static void getNextWord(char** buf, char* result, int max)
{
    char * cur = *buf;
    int i = 0;
    while(i < max) 
    {
        if((cur[i] == ' ') || (cur[i] == '\0') || (cur[i]== '\n')) 
        {
            break;
        }
        result[i] = cur[i];
        i++;
    }
    result[i] = '\0';
    *buf = &(cur[i]) + 1;
}

long dbgcfgtool_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
    int ret = 0;
    dbgcfg_ioctl_arg DbgArg,*pDbgArg = (dbgcfg_ioctl_arg *)arg;
	//lock_kernel();
    switch( cmd ) {

        case DBG_IOCTL_CMD_GET_DBGCFG_GROUP:
            if (copy_from_user(&DbgArg, (void*)pDbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_from_user failed in DBG_IOCTL_CMD_GET_DBGCFG_GROUP.(ret: %d)\n", __func__, __LINE__, ret);
            }

            ret = DbgCfgGetByGroup(DbgArg.id.group_id, &(DbgArg.value));

            if( ret != 0 )
            {
                pr_err("dbgcfgtool_ioctl: Fail to call DbgCfgGetByGroup(), ret=%d.\n",ret);
            }
            else 
            {
                /* Filter out checksum */
                if (DbgArg.id.group_id == FIH_DEBUG_CMD_DATA_SIZE-2)  //Last group
                {
                    DbgArg.value = DbgArg.value & ~CHECK_SUM_MASK;
                }

                if (copy_to_user(pDbgArg, &DbgArg, sizeof(dbgcfg_ioctl_arg)))
                {
                    ret = -EINVAL;
                    pr_err("[dbgcfgtool] %s() LINE:%d, copy_to_user failed in DBG_IOCTL_CMD_GET_DBGCFG_GROUP.(ret: %d)\n", __func__, __LINE__, ret);
                }
            }
            break;
        case DBG_IOCTL_CMD_SET_DBGCFG_GROUP:
            if (copy_from_user(&DbgArg, (void*)pDbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_from_user failed in DBG_IOCTL_CMD_SET_DBGCFG_GROUP.(ret: %d)\n", __func__, __LINE__, ret);
            }

            ret = DbgCfgSetByGroup(DbgArg.id.group_id, DbgArg.value);

            if( ret != 0 )
            {
                pr_err("dbgcfgtool_ioctl: Fail to call DbgCfgSetByGroup(), ret=%d.\n",ret);
            }

            break;
        case DBG_IOCTL_CMD_GET_DBGCFG_BIT:
            if (copy_from_user(&DbgArg, (void*)pDbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_to_user failed in DBG_IOCTL_CMD_GET_DBGCFG_BIT.(ret: %d)\n", __func__, __LINE__, ret);
            }

            ret = DbgCfgGetByBit(DbgArg.id.bit_id, &(DbgArg.value));

            if( ret != 0 )
            {
                pr_err("dbgcfgtool_ioctl: Fail to call DbgCfgGetByBit(), ret=%d.\n",ret);
            }

            if (copy_to_user(pDbgArg, &DbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_to_user failed in DBG_IOCTL_CMD_GET_DBGCFG_BIT.(ret: %d)\n", __func__, __LINE__, ret);
            }
            break;
        case DBG_IOCTL_CMD_GET_ERROR_ACTION:
            ret = DbgCfgGetErrorAction(&(DbgArg.value));

            if( ret != 0 )
            {
                pr_err("dbgcfgtool_ioctl: Fail to call DbgCfgGetErrorAction(), ret=%d.\n",ret);
            }
            else
            {
                error_fatal_option_has_initialized = 1;
            }

            if (copy_to_user(pDbgArg, &DbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_to_user failed in DBG_IOCTL_CMD_GET_ERROR_ACTION.(ret: %d)\n", __func__, __LINE__, ret);
            }
            break;
        case DBG_IOCTL_CMD_SET_ERROR_ACTION:
            if (copy_from_user(&DbgArg, (void*)pDbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_to_user failed in DBG_IOCTL_CMD_GET_DBGCFG_BIT.(ret: %d)\n", __func__, __LINE__, ret);
            }

            if (!error_fatal_option_has_initialized)
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, error_fatal_option isn't initialized.\n", __func__, __LINE__);
            }
            else
            {
                ret = DbgCfgSetErrorAction(DbgArg.value); //MTD-SW3-KERNEL-MP-RPC_Read_NV*
            }

            if( ret != 0 )
            {
                pr_err("dbgcfgtool_ioctl: Fail to call DbgCfgGetErrorAction(), ret=%d.\n",ret);
            }
            break;
        /* FIH-SW-KERNEL-HC-TCXO_SD_DURING_DISPLAY_ON-01+[ */
        #ifdef CONFIG_FIH_SW_TCXO_SD_DURING_DISPLAY_ON
        case DBG_IOCTL_CMD_SET_TCXOSD_DISPLAY_ON:
            if (copy_from_user(&DbgArg, (void*)pDbgArg, sizeof(dbgcfg_ioctl_arg)))
            {
                ret = -EINVAL;
                pr_err("[dbgcfgtool] %s() LINE:%d, copy_from_user failed in DBG_IOCTL_CMD_SET_DBGCFG_GROUP.(ret: %d)\n", __func__, __LINE__, ret);
            }


            ret = fih_set_tcxo_sd_during_display_on(DbgArg.value);

            if( ret != 0 )
            {
                pr_err("dbgcfgtool_ioctl: Fail to call DbgCfgSetByGroup(), ret=%d.\n",ret);
            }

            break;
        #endif
        /* FIH-SW-KERNEL-HC-TCXO_SD_DURING_DISPLAY_ON-01+] */
        default:
            pr_err("[dbgcfgtool_ioctl] Unknown IOCTL(Line: %d).\n", __LINE__);
            ret = -EINVAL;
    }

	//unlock_kernel();	
    return ret;
}

static ssize_t dbgcfgtool_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
    char* buf_idx;
    char szCommand[TEMP_BUF_SIZE * 3];
    char szMaskName[TEMP_BUF_SIZE];
    int mask;
    
    if(count > (DBGCFG_COMMAND_BUFFER_SIZE - 1))
    {
        return -EFAULT;
    }

    write_buf = kmalloc(DBGCFG_COMMAND_BUFFER_SIZE, GFP_KERNEL);
    if(write_buf)
    {
        memset(write_buf, 0, DBGCFG_COMMAND_BUFFER_SIZE);
    }
    else
    {
        return 0;
    }

    if(copy_from_user(write_buf, buf, count))
    {
        return -EFAULT;
    }
	
    write_buf[count] = '\0';
    pr_debug(KERN_INFO "dbgcfgtool_write: write_buf=%s=\n", write_buf);	
	
    buf_idx = write_buf;
    getNextWord(&buf_idx, szCommand, 30);

    if(!strncmp(szCommand, "HANG_LINUX", DBGCFG_COMMAND_BUFFER_SIZE))
    {
        pr_info("dbgcfgtool_write: hang linux\n");
        panic("dbgcfgtool");
    }
    else if(!strncmp(szCommand, "DALVIK_HEAP_DUMP", DBGCFG_COMMAND_BUFFER_SIZE))
    {
        getNextWord(&buf_idx, szMaskName, TEMP_BUF_SIZE);  //this is pid value.
        pr_info("dbgcfgtool_write: %s %s\n", szCommand, szMaskName);
        mask  = (int)simple_strtol(szMaskName, NULL, TEMP_BUF_SIZE);
        pr_info("dbgcfgtool_write: pid=%d sig=%d\n", mask, SIGUSR1);
        kill_proc_info(SIGUSR1, SEND_SIG_PRIV, mask);
    }

    *pos += count;

    if(write_buf)
    {
        kfree(write_buf);
        write_buf = NULL;
    }

    return count;
}


static int dbgcfgtool_open(struct inode *ip, struct file *fp)
{
    return 0;
}

static int dbgcfgtool_release(struct inode *ip, struct file *fp)
{
    return 0;
}

static struct file_operations dbgcfgtool_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = dbgcfgtool_ioctl,
    .write = dbgcfgtool_write,
    .open = dbgcfgtool_open,
    .release = dbgcfgtool_release,
};

static struct miscdevice dbgcfgtool_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "dbgcfgtool",
    .fops = &dbgcfgtool_fops,
};

static int __init dbgcfgtool_init(void)
{

    int ret = 0, value = 0;  
    char tempcfg[]  = {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int *pointer = (int *)localcfg;
    int *pointer_temp = (int *)tempcfg;
    int error_fatal_option = 0;
    int	i;
 
    write_buf= NULL;

    //MTD-SW3-KERNEL-MP-RPC_Read_NV-00*[
    pfnWriteDbgConfiguration = cached_write_fihdbg;
    pfnReadDbgConfiguration  = cached_read_fihdbg;	
    //MTD-SW3-KERNEL-MP-RPC_Read_NV-00*]

    ret = pfnReadDbgConfiguration(tempcfg);

    if( ret == 0 )
    {
        if( memcmp(tempcfg,localcfg,FIH_DEBUG_CFG_LEN) == 0 )
        {
//Div6-PT2-Core-BH-DbgCfgTool-01+[
// DbgCfg has been initialized in appsboot, not need any more here
#if 0
            pr_debug("[dbgcfgtool] %s() LINE:%d, Initialize dbgcfg data\n", __func__, __LINE__);
            memset(localcfg,0,FIH_DEBUG_CFG_LEN);
            localcfg[FIH_DEBUG_CFG_LEN-1] = GenerateCheckSumByChar(tempcfg,FIH_DEBUG_CFG_LEN-1);
            
            ret = pfnWriteDbgConfiguration(localcfg);
            
            if ( ret == 0 )
            {
                pr_info("[dbgcfgtool] %s() LINE:%d, Initialize dbgcfg data successful.\n", __func__, __LINE__);
            }
            else
            {
                pr_err("[dbgcfgtool] %s() LINE:%d, Initialize dbgcfg data failed.(ret: %d)\n", __func__, __LINE__, ret);
            }
#endif			
//Div6-PT2-Core-BH-DbgCfgTool-01+]
        }
        else if( GenerateCheckSumByChar(tempcfg,FIH_DEBUG_CFG_LEN-1) == tempcfg[FIH_DEBUG_CFG_LEN-1] )
        {
            memcpy(localcfg,tempcfg,FIH_DEBUG_CFG_LEN);
            pr_debug("[dbgcfgtool] %s() LINE:%d, Check sum is correct.\n", __func__, __LINE__);

			pr_debug("[dbgcfgtool] %s() LINE:%d, cfg data is:", __func__, __LINE__);
			for (i = 0; i < FIH_DEBUG_CFG_LEN; i++)
				pr_debug("0x%08x ", tempcfg[i]);
			pr_debug("\n");							
        }
        else
        {
            ret = -1;        
            pr_err("[dbgcfgtool] %s() LINE:%d, Data crash in storage.(ret: %d)\n", __func__, __LINE__, ret);
            pr_err("[dbgcfgtool] tempcfg: 0x%08x 0x%08x 0x%08x 0x%08x\n", pointer_temp[0], pointer_temp[1], pointer_temp[2], pointer_temp[3]);
        }
        
        if( ret == 0 )
        {
            dbgcfg_data_has_initialized = 1;

            //Initialize global flag for kernel debug
            ret = DbgCfgGetByBit(DEBUG_CPU_USAGE_CFG, (int*)&value);
            
            if (ret == 0)
            {
                debug_cpu_usage_enable = value;
            }
            else
            {
                pr_err("[dbgcfgtool] %s() LINE:%d, Initialize debug_cpu_usage_enable failed.(ret: %d)\n", __func__, __LINE__, ret);
            }

            ret = DbgCfgGetByBit(DEBUG_RPCMSG_CFG, (int*)&value);
            
            if (ret == 0)
            {
                debug_rpcmsg_enable = value;
            }
            else
            {
                pr_err("[dbgcfgtool] %s() LINE:%d, Initialize debug_rpcmsg_enable failed.(ret: %d)\n", __func__, __LINE__, ret);
            }

            pr_info("[dbgcfgtool] localcfg: 0x%08x 0x%08x 0x%08x 0x%08x\n", pointer[0], pointer[1], pointer[2], pointer[3]);

            ret = DbgCfgGetErrorAction(&error_fatal_option);

            if(ret != 0)
            {
                pr_err("[dbgcfgtool] %s() LINE:%d, Fail to call DbgCfgGetErrorAction.(ret: %d)\n", __func__, __LINE__, ret);
            }
          
            ret = misc_register(&dbgcfgtool_dev);
            if ( ret )
            {
                pr_err("Cannot register miscdev on minor(MISC_DYNAMIC_MINOR)=%d (err=%d)\n", MISC_DYNAMIC_MINOR, ret);
            }
        }
    }        
    else
    {
        pr_err("[dbgcfgtool] %s() LINE:%d, Fail to read debug configurations.(ret: %d)\n", __func__, __LINE__, ret);    
    }

    return ret;
}

module_init(dbgcfgtool_init);

static void __exit dbgcfgtool_deinit( void )
{
    pfnWriteDbgConfiguration = NULL; 
    pfnReadDbgConfiguration  = NULL;
    
    misc_deregister(&dbgcfgtool_dev);
}

module_exit(dbgcfgtool_deinit);

MODULE_DESCRIPTION("dbgcfgtool");
MODULE_LICENSE("GPL v2");
