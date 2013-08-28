/* arch/arm/mach-msm/proc_comm.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/proc_comm.h>

#include "smd_private.h"

static inline void notify_other_proc_comm(void)
{
	/* Make sure the write completes before interrupt */
	wmb();
#if defined(CONFIG_ARCH_MSM7X30)
	__raw_writel(1 << 6, MSM_APCS_GCC_BASE + 0x8);
#elif defined(CONFIG_ARCH_MSM8X60)
	__raw_writel(1 << 5, MSM_GCC_BASE + 0x8);
#else
	__raw_writel(1, MSM_CSR_BASE + 0x400 + (6) * 4);
#endif
}

#define APP_COMMAND 0x00
#define APP_STATUS  0x04
#define APP_DATA1   0x08
#define APP_DATA2   0x0C

#define MDM_COMMAND 0x10
#define MDM_STATUS  0x14
#define MDM_DATA1   0x18
#define MDM_DATA2   0x1C

static DEFINE_SPINLOCK(proc_comm_lock);
static int msm_proc_comm_disable;

/* Poll for a state change, checking for possible
 * modem crashes along the way (so we don't wait
 * forever while the ARM9 is blowing up.
 *
 * Return an error in the event of a modem crash and
 * restart so the msm_proc_comm() routine can restart
 * the operation from the beginning.
 */
static int proc_comm_wait_for(unsigned addr, unsigned value)
{
	while (1) {
		/* Barrier here prevents excessive spinning */
		mb();
		if (readl_relaxed(addr) == value)
			return 0;

		if (smsm_check_for_modem_crash())
			return -EAGAIN;

		udelay(5);
	}
}

void msm_proc_comm_reset_modem_now(void)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel_relaxed(PCOM_RESET_MODEM, base + APP_COMMAND);
	writel_relaxed(0, base + APP_DATA1);
	writel_relaxed(0, base + APP_DATA2);

	spin_unlock_irqrestore(&proc_comm_lock, flags);

	/* Make sure the writes complete before notifying the other side */
	wmb();
	notify_other_proc_comm();

	return;
}
EXPORT_SYMBOL(msm_proc_comm_reset_modem_now);

int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&proc_comm_lock, flags);

	if (msm_proc_comm_disable) {
		ret = -EIO;
		goto end;
	}


again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel_relaxed(cmd, base + APP_COMMAND);
	writel_relaxed(data1 ? *data1 : 0, base + APP_DATA1);
	writel_relaxed(data2 ? *data2 : 0, base + APP_DATA2);

	/* Make sure the writes complete before notifying the other side */
	wmb();
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
		goto again;

	if (readl_relaxed(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
		if (data1)
			*data1 = readl_relaxed(base + APP_DATA1);
		if (data2)
			*data2 = readl_relaxed(base + APP_DATA2);
		ret = 0;
	} else {
		ret = -EIO;
	}

	writel_relaxed(PCOM_CMD_IDLE, base + APP_COMMAND);

	switch (cmd) {
	case PCOM_RESET_CHIP:
	case PCOM_RESET_CHIP_IMM:
	case PCOM_RESET_APPS:
		msm_proc_comm_disable = 1;
		printk(KERN_ERR "msm: proc_comm: proc comm disabled\n");
		break;
	}
end:
	/* Make sure the writes complete before returning */
	wmb();
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_proc_comm);

//MTD-BSP-VT-PROC-00+[
int msm_proc_comm_oem(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter)
{
  unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
  unsigned long flags;
  int ret;
  size_t sizeA, sizeB;
  smem_oem_cmd_data *cmd_buf;
  void* test;
  int size;
  
  sizeA=40;
  sizeB=64;

  cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);
  test= (unsigned*)&cmd_buf->cmd_data.cmd_parameter[0];
  spin_lock_irqsave(&proc_comm_lock, flags);

again:
  if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
	goto again;

  writel(cmd, base + APP_COMMAND);
  writel(data1 ? *data1 : 0, base + APP_DATA1);
  writel(data2 ? *data2 : 0, base + APP_DATA2);
  cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;

  if(*data1==SMEM_PROC_COMM_OEM_PRODUCT_ID_WRITE)
  {
    memcpy(test,(const void *)cmd_parameter,sizeA);
  }
  else if(*data1==SMEM_PROC_COMM_OEM_TEST_FLAG_WRITE)
  {
    memcpy(test,(const void *)cmd_parameter,sizeB);
  }
  else if(*data1==SMEM_PROC_COMM_OEM_POWER_OFF)
  {
	memcpy(test,(const void *)cmd_parameter, SMEM_OEM_CMD_BUF_SIZE*sizeof(int));
  }
  else
  {
	cmd_buf->cmd_data.cmd_parameter[0] = *cmd_parameter;
  }
  notify_other_proc_comm();

  if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
	goto again;

#if 0
  if (readl(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
	if (data1)
		*data1 = readl(base + APP_DATA1);
	if (data2)
		*data2 = readl(base + APP_DATA2);
	ret = 0;
  } else {
		ret = -EIO;
  }
	#endif
	writel(PCOM_CMD_IDLE, base + APP_COMMAND);

	//spin_unlock_irqrestore(&proc_comm_lock, flags);

	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}
	ret = (cmd_buf->return_data.check_flag & 0x1111);
		
	if(!ret)
	{
	  if(*data1==SMEM_PROC_COMM_OEM_PRODUCT_ID_READ)
	  {	
		memcpy((void *)data2, test,sizeA);
	  }
	  else if(*data1==SMEM_PROC_COMM_OEM_TEST_FLAG_READ)
      {	
		memcpy((void *)data2,test,sizeB);
	  }
	  else if(*data1==SMEM_PROC_COMM_OEM_NV_READ)
	  {
		memcpy(data2,&cmd_buf->return_data.return_value[0],128); 
	  }
	  else
	  {
        *data2 = cmd_buf->return_data.return_value[0];
	  }
	}
	else
	{
	 /* Get the error code */
	  if(*data1==SMEM_PROC_COMM_OEM_OTP_PROCESS)
		memcpy(data2,&cmd_buf->return_data.return_value[0],128);
	}
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
	
}
EXPORT_SYMBOL(msm_proc_comm_oem);

int msm_proc_comm_oem_n(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter, int u32_para_size)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;
	size_t sizeA, sizeB;
	smem_oem_cmd_data *cmd_buf;
       void* test;

	/* get share memory command address dynamically */
	int size;
	sizeA=40;
	sizeB=64;
	cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);

	if( (cmd_buf == 0) || (size < u32_para_size) )
	{
		printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, Can't get shared memory entry.(size %d,u32_para_size %d)\n", __func__, __LINE__, size, u32_para_size); 
		return -EINVAL;
	}

	test= (unsigned*)&cmd_buf->cmd_data.cmd_parameter[0];

	if( (cmd_parameter == NULL) || (u32_para_size == 0) )
	{
		printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, ERROR: u32_para_size %d.\n", __func__, __LINE__, u32_para_size); 
		return -EINVAL;
	}

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel(cmd, base + APP_COMMAND);
	writel(data1 ? *data1 : 0, base + APP_DATA1);
	writel(data2 ? *data2 : 0, base + APP_DATA2);

	// Set the parameter of OEM_CMD1
	cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
	memcpy(cmd_buf->cmd_data.cmd_parameter,cmd_parameter,sizeof(unsigned)*u32_para_size);
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
	{
		goto again;
	}
	
	writel(PCOM_CMD_IDLE, base + APP_COMMAND);

	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}

	/* Div6-D1-SY-FIHDBG-00*{ 
	 * Due to review return value in AMSS, we decide to modify the mask 
	 * from 0x1111 to 0xFFFF to identify the correct error type.
	 * Notice: Need to review the "check_flag" usage in both mARM and aARM.
	 */	
	ret = (cmd_buf->return_data.check_flag & 0xFFFF);
	/* Div6-D1-SY-FIHDBG-00*} */

	if(!ret)
	{
		*data2 = cmd_buf->return_data.return_value[0];

		/* Copy the returned value back to user "cmd_parameter" */
		memcpy(cmd_parameter, cmd_buf->return_data.return_value, sizeof(unsigned) * u32_para_size);
	}
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_proc_comm_oem_n);

int	smem_proc_oem_light_control( int32_t *data, int number )
{
	uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
	uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_LIGHT_CONTROL;
	uint32_t smem_proc_comm_oem_data2   = 0;

	if( msm_proc_comm_oem_n( smem_proc_comm_oem_cmd1, &smem_proc_comm_oem_data1, &smem_proc_comm_oem_data2, data, number ) ) //MTD-BSP-REXER-PROC-00*
	{
		printk( KERN_ERR "LED : SMEM_PROC_COMM_OEM_LIGHT_CONTROL failed!!!\n" );
		return 0;
	}
       
	return 1;
}
EXPORT_SYMBOL( smem_proc_oem_light_control );
 
int msm_proc_comm_oem_multi(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter, int number)
{
    unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
    unsigned long flags;
    unsigned int size;
    int ret, index;

    smem_oem_cmd_data *cmd_buf;
//Div2-SW2-BSP, JOE HSU,rename smem_mem_type    
    cmd_buf = (smem_oem_cmd_data *)smem_get_entry(SMEM_ID_VENDOR1, &size);
    
    printk(KERN_INFO "%s: 0x%08x\n", __func__, (unsigned)cmd_buf);

    spin_lock_irqsave(&proc_comm_lock, flags);

again:
    if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
        goto again;

    writel(cmd, base + APP_COMMAND);
    writel(data1 ? *data1 : 0, base + APP_DATA1);
    writel(data2 ? *data2 : 0, base + APP_DATA2);
                        
    // Set the parameter of OEM_CMD1
    cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
    for( index = 0 ; index < number ; index++)
        cmd_buf->cmd_data.cmd_parameter[index] = cmd_parameter[index];
                        
    notify_other_proc_comm();
                        
    if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
        goto again;
                        
    if (readl(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
        if (data1)
            *data1 = readl(base + APP_DATA1);
        if (data2)
            *data2 = readl(base + APP_DATA2);
        
        ret = 0;
    } else {
        ret = -EIO;
    }
                        
    for (index = 0; index < number; index++) {
        cmd_parameter[index] = cmd_buf->return_data.return_value[index];
    }

    writel(PCOM_CMD_IDLE, base + APP_COMMAND);
                        
    while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag)) {
        //waiting
        mdelay(100);
        printk(KERN_INFO "%s: wait...... 0x%04x\n", __func__, cmd_buf->return_data.check_flag);
    }

    ret = (cmd_buf->return_data.check_flag & 0x1111);
    if(!ret) {
        *data2 = cmd_buf->return_data.return_value[0];
    }

    spin_unlock_irqrestore(&proc_comm_lock, flags);
    return ret;
}
EXPORT_SYMBOL(msm_proc_comm_oem_multi);

#ifdef CONFIG_FIH_FTM
int proc_comm_get_vcoin(int vset, int *voltage, int status)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_GET_VCOIN;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = vset;
    //status = 0:off / 1:on
    data[1] = status;
    
    ret = msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL FAILED!!\n",
                __func__);
    }
    
    *voltage = data[0];
    
    return ret; //mV
}
EXPORT_SYMBOL(proc_comm_get_vcoin);
#endif

#ifdef CONFIG_FIH_SW3_POWER_OFF_CHARGING
int proc_comm_is_pwr_key_pressed(int *isPressed)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_PWR_KEY_DECT;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = 0;
    data[1] = 0;
    
    ret = msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL FAILED!!\n",
                __func__);
    }
    
    *isPressed = data[0];
    return ret;
}
EXPORT_SYMBOL(proc_comm_is_pwr_key_pressed);

int proc_comm_is_charger_connected(int *isConnected)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_IS_CHARGER_CONNECTED;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = 0;
    data[1] = 0;
    
    ret = msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL FAILED!!\n",
                __func__);
    }
    
    *isConnected = data[0];
    return ret;
}
EXPORT_SYMBOL(proc_comm_is_charger_connected);
#endif

static int getwlanmac = 0;

void proc_comm_ftm_wlanaddr_write(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    int32_t data[3];

    printk(KERN_INFO "proc_comm_ftm_wlanaddr_write() '0x%x:%x:%x:%x:%x:%x'\n", buf[5], buf[4], buf[3], buf[2], buf[1], buf[0]);

    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)4678;   // NV_WLAN_MAC_ADDRESS_I
    memcpy((void *)&data[1],buf,6);

    if (0 != msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }

    getwlanmac = 0;
}
EXPORT_SYMBOL(proc_comm_ftm_wlanaddr_write);

int proc_comm_ftm_wlanaddr_read(char * buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    static int32_t data[3] = {0};
    int32_t ret = 0;

    if (getwlanmac == 0) {
        memset((void*)data, 0x00, sizeof(data));
        data[0] = (uint32_t)4678;   // NV_WLAN_MAC_ADDRESS_I

        ret = msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1,
                &smem_proc_comm_oem_data1,
                &smem_proc_comm_oem_data2,
                data,
                3);
        if (ret != 0) {
            printk(KERN_ERR
                    "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                    __func__);
            return ret;
        } else {
            getwlanmac = 1;
            printk(KERN_INFO "%s:Get MAC by OEM command\n", __func__);
        }
    } else {
        printk(KERN_INFO "%s:Get MAC by cached MAC\n", __func__);
    }

    memcpy(buf, &data[1], sizeof(char)*6);
    if ((data[1] == 0) && (data[2] == 0)) ret = -1;
    printk(KERN_INFO "proc_comm_ftm_wlanaddr_read() '0x%x:%x:%x:%x:%x:%x'\n", buf[5], buf[4],buf[3], buf[2],buf[1], buf[0]);
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_wlanaddr_read);

int proc_comm_hotspot_sar_control(int state)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_HOTSPOT_PWR;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;
	printk(KERN_ERR "%s: [Hotspot] hotspot state = %d \n", __func__, state);

    memset((void*)data, 0x00, sizeof(data));
    //state = 0:off / 1:on
    data[0] = state;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    1); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_HOTSPOT_PWR FAILED!!\n",
                __func__);
    }
    
    return ret; //mV
}
EXPORT_SYMBOL(proc_comm_hotspot_sar_control);

int proc_comm_get_hsed_voltage(int vset, int *voltage, int status)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_GET_HSED_VOLTAGE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = vset;
    //status = 0:off / 1:on
    data[1] = status;
    
    ret = msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL FAILED!!\n",
                __func__);
    }
    
    *voltage = data[0];
    
    return ret; //mV
}
EXPORT_SYMBOL(proc_comm_get_hsed_voltage);

int proc_comm_get_btn_type(int vset, int *voltage, int status)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_GET_BTN_TYPE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = vset;
    //status = 0:off / 1:on
    data[1] = status;
    
    ret = msm_proc_comm_oem_n(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL FAILED!!\n",
                __func__);
    }
    
    *voltage = data[0];
    
    return ret; //mV
}
EXPORT_SYMBOL(proc_comm_get_btn_type);
//MTD-BSP-VT-PROC-00+]
