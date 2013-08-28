/* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved.
 * BQ275x0 dfi programming driver
 */

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <mach/fih_bq275x0_battery.h>
#include "../../arch/arm/mach-msm/smd_private.h"
#include "fih_bqfs.h"
#include <mach/msm_smsm.h>

#define ERASE_RETRIES           3
#define FAILED_MARK             0xFFFFFFFF

static struct i2c_client *bq275x0_client;
static struct proc_dir_entry *dfi_upgrade_proc_entry;

static int g_error_type = 0;
static int counter = 0;
static int bq27520_fw_ver = 0;

static struct wake_lock bqfs_wakelock;
static struct delayed_work bqfs_upgrade;

extern int bq275x0_battery_enter_rom_mode(void);
extern int bq275x0_battery_IT_enable(void);
extern int bq275x0_battery_reset(void);
extern int bq275x0_battery_sealed(void);
extern void bq275x0_battery_exit_rommode(void);
extern int bq275x0_battery_get_MfgInfo(char *buf);
extern int bq275x0_query_fw_ver(void);
extern int bq27520_isEnableBatteryCheck;
extern void schedule_bq27520_hw_config(void);
extern int bq275x0_battery_write_MfgInfo(char *buf, int len);

enum {
    ERR_SUCCESSFUL,
    ERR_READ_DFI,
    ERR_ENTER_ROM_MODE,
    ERR_READ_IF,
    ERR_ERASE_IF,
    ERR_MASS_ERASE,
    ERR_PROGRAM_DFI,
    ERR_CHECKSUM,
    ERR_PROGRAM_IF,
    ERR_IT_ENABLE,
    ERR_RESET,
    ERR_SEALED,
    ERR_DEVICE_TYPE_FW_VER,
    ERR_UNKNOWN_CMD,
};

static int bq275x0_RomMode_read(u8 cmd, u8 *data, int length)
{
    int ret = 0;
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = bq275x0_client->addr,
            .flags  = 0,
            .buf    = (void *)&cmd,
            .len    = 1
        },
        [1] = {
            .addr   = bq275x0_client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)data,
            .len    = length
        }
    };

    if (i2c_transfer(bq275x0_client->adapter, &msgs[0], 1) != 1) {
        ret = -1;
        dev_err(&bq275x0_client->dev,"I2C error at line %d\n", __LINE__);
        goto exit;
    }
    mdelay(1);

    if (i2c_transfer(bq275x0_client->adapter, &msgs[1], 1) != 1) {
        ret = -1;
        dev_err(&bq275x0_client->dev,"I2C error at line %d\n", __LINE__);
        goto exit;
    }
    mdelay(1);

exit:
    return ret;
}

static int bq275x0_RomMode_write(u8 *cmd, int length)
{
    int ret = 0;
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = bq275x0_client->addr,
            .flags  = 0,
            .buf    = (void *)cmd,
            .len    = length
        },
    };

    if (i2c_transfer(bq275x0_client->adapter, msgs, 1) != 1) {
        ret = -1;
        dev_err(&bq275x0_client->dev,"I2C error at line %d\n", __LINE__);
        goto exit;
    }
    
    mdelay(1);
	
exit:
    return ret;  
}

static int bq275x0_RomMode_erase_first_two_rows(void)
{
    u8 cmd[3];
    u8 buf;
    u8 retries = 0;
    
    do {
        retries++;
        
        cmd[0] = 0x00;
        cmd[1] = 0x03;
        bq275x0_RomMode_write(cmd, 2);
        cmd[0] = 0x64;
        cmd[1] = 0x03;  /* 0x64 */
        cmd[2] = 0x00;  /* 0x65 */
        bq275x0_RomMode_write(cmd, sizeof(cmd));
        mdelay(20);
        
        bq275x0_RomMode_read(0x66, &buf, sizeof(buf));
        if (buf == 0x00)
            return 0;
            
        if (retries > ERASE_RETRIES)
            break;
    } while (1);
    
    return -1;
}

#define WRITE_BLOCK_SIZE 32
static int bq275x0_RomMode_program_bqfs(void)
{
    u8 cmd[33];
    u8 buf[96];
    u8 reg;
    int len = 0;
    int i = 0;
    unsigned long idx = 0;
    unsigned long size = BQFS_SIZE;
    union {
        u16 s;
        struct {
            u8 low;
            u8 high;
        } u;
    } delay;
    
    counter = 0;

    if (bq27520_fw_ver == BQFS_FW_VER) {
        idx = 61240;
        dev_err(&bq275x0_client->dev,"Only update DFI part, start from BQFS[61240].\n");
    } else {
        dev_err(&bq275x0_client->dev,"Update entire BQFS.\n");
    }

    while (idx < size) {
        counter++;
        if (counter == 1 || counter % 200 == 0)
            dev_info(&bq275x0_client->dev,"Line[%d].\n", counter);

        switch (BQFS[idx++]) {            
        case 'W':
            len = BQFS[idx++];
            reg = BQFS[idx++];
            memcpy(buf, &BQFS[idx], len - 1);
            idx = idx + (len - 1);
                
            for (i = 0; i < (len - 1) / WRITE_BLOCK_SIZE; i ++) {
                cmd[0] = reg + (i * WRITE_BLOCK_SIZE);
                memcpy(&cmd[1], &buf[i * WRITE_BLOCK_SIZE], WRITE_BLOCK_SIZE);
                if (bq275x0_RomMode_write(cmd, WRITE_BLOCK_SIZE + 1)) {
                    dev_err(&bq275x0_client->dev,"Line[%d] Write FAILED %d, (A)\n", counter, i);
                    return -1;
                }
            }
			
            if ((len - 1) % WRITE_BLOCK_SIZE) {
                cmd[0] = reg + (i * WRITE_BLOCK_SIZE);
                memcpy(&cmd[1], &buf[i * WRITE_BLOCK_SIZE], (len - 1) % WRITE_BLOCK_SIZE);
                if (bq275x0_RomMode_write(cmd, (len - 1) % WRITE_BLOCK_SIZE + 1)) {
                    dev_err(&bq275x0_client->dev,"Line[%d] Write FAILED %d, (B)\n", counter, i);
                    return -1;
                }
            }
            break;
            
        case 'R':
            break;
            
        case 'C':
//            mdelay(16);
            len = BQFS[idx++];

            cmd[0] = 0x3e; /* Give a init value other than 0 */
            if (bq275x0_RomMode_read(BQFS[idx++], cmd, len - 1)) {
                dev_err(&bq275x0_client->dev,"error read checksum back\n");
                return -1;
            }
    
            for (i = 0; i < (len - 1); i++, idx ++)
                if (cmd[i] != BQFS[idx]) {
                    dev_err(&bq275x0_client->dev, "Line[%d] C Failed %02x %02x\n", counter, cmd[i], BQFS[idx]);
                    return -1;
                }
            break;
			
        case 'X':
            delay.u.low = BQFS[idx++];
            delay.u.high = BQFS[idx++];
            mdelay(delay.s);
            break;
        }
    }
    
    dev_info(&bq275x0_client->dev,"Line[%d]\n", counter);
    
    return 0;
}

static int bq275x0_RomMode_read_erase_if(void)
{
    int ret = 0;
    
    if (bq275x0_battery_enter_rom_mode()) {
        dev_err(&bq275x0_client->dev, "Entering rom mode was failed\n");
        ret = -ERR_ENTER_ROM_MODE; /* Rom Mode is locked, or IC is damaged */
    }
    mdelay(2); /* Gauge need time to enter ROM mode. */

    if (bq275x0_RomMode_erase_first_two_rows()) {
        dev_err(&bq275x0_client->dev, "Erasing IF first two rows was failed\n");
        if (ret)
            return ret;
 
        return -ERR_ERASE_IF;
    }
    
    dev_info(&bq275x0_client->dev, "Start to Writing Image\n");
    return 0;
}

static int bq275x0_RomMode_writing_image(void)
{
    if (bq275x0_RomMode_program_bqfs()) {
        dev_err(&bq275x0_client->dev, "programming was failed\n");
        return -ERR_PROGRAM_DFI;
    }

    dev_info(&bq275x0_client->dev, "End of Writing Image\n");
    return 0;
}

enum {
    PROGRAM_READ_ERASE_IF,
    PROGRAM_WRITING_DFI,
    PROGRAM_IT_ENABLE,
    PROGRAM_RESET,
    PROGRAM_SEALED,
};

static ssize_t bq275x0_RomMode_programming_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{  
    int step    = 0;
    int cmd     = 0;
    int ret     = 0;
    
    /*
     * cmd:     Assign first command.
     * step:    Assign how many commands we want to execute since first
     *          command.
     * 
     * If PROGRAM_WRITING_DFI command is failed, IF rows should not be 
     * reporgrammed. If IF rows are empty, the IC will always enter ROM 
     * Mode.
     * 
     * PROGRAM_RESET command is strongly suggested to execute after
     * PROGRAM_IT_ENABLE command is executed.
     * 
     * PROGRAM_SEALED command need other command to execute it. This way
     * is to prevent from entering SEALED mode unexpectedly.
     */
    wake_lock(&bqfs_wakelock);
    sscanf(buf, "%1d %1d", &cmd, &step);
    switch(cmd) {
    case PROGRAM_READ_ERASE_IF: /* step 4 */
        if (step)
            step--;
        else
            break;
    
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
    
        if ((g_error_type = bq275x0_RomMode_read_erase_if())) {
            bq275x0_battery_exit_rommode();
            dev_err(&bq275x0_client->dev, "ERASE IF[FAILED]\n");
            break;
        }
        dev_info(&bq275x0_client->dev, "ERASE IF[SUCCESSFUL]\n");

    case PROGRAM_WRITING_DFI:   /* step 3 */
        if (step)
            step--;
        else 
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);

        if ((g_error_type = bq275x0_RomMode_writing_image())) {
            bq275x0_battery_exit_rommode();
            dev_err(&bq275x0_client->dev, "WRITING BQFS[FAILED]\n");
            break;
        }
        dev_info(&bq275x0_client->dev, "WRITING BQFS[SUCCESSFUL]\n");

    case PROGRAM_IT_ENABLE:     /* step 2 */
        if (step > 0) {
            if (step != 1)
                step--;
        } else {
            break;
        }
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
        if ((ret = bq275x0_battery_IT_enable())) {
            if (ret)
                g_error_type = -ERR_IT_ENABLE;
            bq275x0_battery_exit_rommode();
            dev_err(&bq275x0_client->dev, "IT ENABLE[FAILED]\n");
            break;
        }
        dev_info(&bq275x0_client->dev, "IT ENABLE[SUCCESSFUL]\n");

        counter = BQFS_TOTAL_LINES + 1;
    case PROGRAM_RESET:         /* step 1 */
        if (step)
            step--;
        else
            break;
            
        dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);

        if ((ret = bq275x0_battery_reset())) {
            if (ret)
                g_error_type = -ERR_RESET;
            bq275x0_battery_exit_rommode();
            
            dev_err(&bq275x0_client->dev, "RESET[FAILED]\n");
            break;
        }
        
        bq275x0_battery_exit_rommode();
        dev_info(&bq275x0_client->dev, "RESET[SUCCESSFUL]\n");

        counter = BQFS_TOTAL_LINES + 2;
        break;
    case PROGRAM_SEALED:
        /*dev_info(&bq275x0_client->dev, "%s: step %d\n", __func__, step);
        if (bq275x0_battery_sealed()) {
            g_error_type = -ERR_SEALED;
        }*/
        break;
    default:
        g_error_type = -ERR_UNKNOWN_CMD;
    };
    wake_unlock(&bqfs_wakelock);

    return count;
}

static ssize_t bq275x0_RomMode_programming_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return 0;
}
static DEVICE_ATTR(programming, 0644, bq275x0_RomMode_programming_show, bq275x0_RomMode_programming_store);

static char cmd[256];
static int
proc_calc_metrics(char *page, char **start, off_t off,
                 int count, int *eof, int len)
{
    if (len <= off+count) *eof = 1;
    *start = page + off;
    len -= off;
    if (len > count) len = count;
    if (len < 0) len = 0;
    return len;
}

/* Is Flashing on going? */
static bool flashing = false;

static void bq275x0_flash(void)
{

/* %%TODO: Flash LED but there has lots of problem need to be solved, so pending
    I) LED driver init latter than gauge
    II) LED file node is not ready before file system ready 
*/

    dev_info(&bq275x0_client->dev, "[START]Flash BQFS.\n");

    /* Mark flash is on going */
    flashing = true;
    g_error_type = 0;
    counter = BQFS_TOTAL_LINES;

    schedule_delayed_work(&bqfs_upgrade, msecs_to_jiffies(30));
}

static int
bq275x0_dfi_upgrade_proc_read(char *page, char **start, off_t off,
              int count, int *eof, void *data)
{
    int len;
    
    if (g_error_type < 0) {
        len = snprintf(page, PAGE_SIZE, "%d", g_error_type);
    } else {
        len = snprintf(page, PAGE_SIZE, "%d", counter);
        if (counter == BQFS_TOTAL_LINES)
            counter = 0;
    }
        
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int
bq275x0_dfi_upgrade_proc_write(struct file *file, const char __user *buffer,
               unsigned long count, void *data)
{
    dev_info(&bq275x0_client->dev, "procfile_write (/proc/dfi_upgrade) called\n");
    
    if (copy_from_user(cmd, buffer, count)) 
        return -EFAULT;
    else if (!strncmp(cmd, "flash22685511@FIHLX\n", count)) 
        bq275x0_flash();
    else if (!strncmp(cmd, "invalidmfg22685511@FIHLX\n", count)) {
        unsigned char invalid_mfg_info[32];
    
        invalid_mfg_info[0] = 0;
        memcpy(&invalid_mfg_info[1], cmd, sizeof(invalid_mfg_info) - 1);
        bq275x0_battery_write_MfgInfo(invalid_mfg_info, sizeof(invalid_mfg_info));
    }
    
    return count;
}

#define NV_NEED_RESTORE 0x8686 
extern void kernel_restart(char *cmd);

void reset_after_upgrade(void)
{
	/* %%TODO: for porting error, omit it now and enable it latter */
#if 0
    int *pnvstatus = NULL;

    pnvstatus = smem_alloc(SMEM_FIRST_BOOT_NVSYNC, sizeof(int));

    /* Detect if we need to restore default NV, if not reset the device */    
    if (pnvstatus) 
        if (*pnvstatus != NV_NEED_RESTORE)
            kernel_restart(NULL);
#endif            
}

static void bq275x0_RomMode_upgrade(struct work_struct *work)
{
    bq275x0_RomMode_programming_store(NULL, NULL, "0 4", 4);
    dev_info(&bq275x0_client->dev, "[END]Flash BQFS %d.\n", g_error_type);

    /* Upgrade done, reset the device */
    if (g_error_type == 0)
        reset_after_upgrade();
}

static char mfg_info[32];

static int bq275x0_RomMode_probe(struct i2c_client *client,
                 const struct i2c_device_id *id)
{
    int retval = 0;

    bq275x0_client = client;
    retval = device_create_file(&client->dev, &dev_attr_programming);
    if (retval) {
        dev_err(&client->dev,
               "%s: dev_attr_test failed\n", __func__);
    }

    wake_lock_init(&bqfs_wakelock, WAKE_LOCK_SUSPEND, "bq275x0_bqfs");
    INIT_DELAYED_WORK(&bqfs_upgrade, bq275x0_RomMode_upgrade);
    dfi_upgrade_proc_entry = create_proc_entry("dfi_upgrade", 0666, NULL);
    dfi_upgrade_proc_entry->read_proc   = bq275x0_dfi_upgrade_proc_read;
    dfi_upgrade_proc_entry->write_proc  = bq275x0_dfi_upgrade_proc_write;

    if (bq275x0_battery_get_MfgInfo(&mfg_info[0]) == 0) {
        dev_info(&client->dev, "MFG INFO-FLASH: %02X, \"%s\"\n", mfg_info[0], &mfg_info[1]);
        dev_info(&client->dev, "MFG INFO-IMAGE: %02X, \"%s\"\n",BQFS_MFG_INFO[0], &BQFS_MFG_INFO[1]);

        /* The most two significant bits is used to identify if gauge works good at first boot after flased */
        if (memcmp(&mfg_info[1], &BQFS_MFG_INFO[1], 31) || ((mfg_info[0] & 0x3F) != BQFS_MFG_INFO[0]) ){
            dev_info(&client->dev, "DFI mismatch.\n");
            #ifdef CONFIG_FIH_SW3_BQ275X0_ROMMODE_AUTO_UPGRADE
                dev_info(&client->dev, "DFI mismatch, going on flashing.\n");
                bq275x0_flash();
            #endif
        } else {
            dev_info(&client->dev, "DFI match.\n");
        }
        bq27520_isEnableBatteryCheck = 1;
        bq27520_fw_ver = bq275x0_query_fw_ver();
		
    } else {
        dev_err(&client->dev,"%s: Read MfgInfo from gauge failed, FORCE REFLASH\n", __func__);
        bq275x0_flash();
    }

    /* Here to continue bq27520 init process.
       Moved here to make sure the following works handled prior than hw init
       1. Get manufacturer ID
       2. Compare manufacturer ID, if different then flashing
    */
    schedule_bq27520_hw_config();
    
    return retval;
}

/*  [FUNCTION] 
                bq275x0_flash_recovery
    [PURPOSE] 
                The function reflashes the image or marks first boot after flashed successfully
    [IN] 
                reflash: 
                    If we need to reflash (because of checking gauge status fail)
    [OUT]
                NONE
*/
void bq275x0_flash_recovery(bool reflash)
{
    char buf[32];
        
        dev_info(&bq275x0_client->dev, "%s %d %d\n", __func__, reflash, flashing);

    /* If flah is ongoing, don't disturb it, just leave this function */
    if (flashing) {
        dev_info(&bq275x0_client->dev, "%s flashing ongoing\n", __func__);
        goto exit;
    }

    /* Do the jobs only when this is the first boot after flashed 
       %%TBTA: If this is not the first boot and we found OCVCMDCOMP not set, what should we do?
   */
    if ((mfg_info[0] & 0x80) == 0x00) {
        if (reflash) {
            bq275x0_flash();
	} else {
            memcpy(buf, mfg_info, 32);
            buf[0] |= 0x80;
            bq275x0_battery_write_MfgInfo(buf, 32);
        }
    }
	
exit:
    return;
}
EXPORT_SYMBOL(bq275x0_flash_recovery);

static int bq275x0_RomMode_remove(struct i2c_client *client)
{
    return 0;
}

/*
 * Module stuff
 */
static const struct i2c_device_id bq275x0_RomMode_id[] = {
    { "bq275x0-RomMode", 0 },
    {},
};

static struct i2c_driver bq275x0_RomMode_driver = {
    .driver = {
        .name = "bq275x0-RomMode",
    },
    .probe = bq275x0_RomMode_probe,
    .remove = bq275x0_RomMode_remove,
    .id_table = bq275x0_RomMode_id,
};

static int __init bq275x0_RomMode_init(void)
{
    int ret;

    ret = i2c_add_driver(&bq275x0_RomMode_driver);
    if (ret)
        printk(KERN_ERR "Unable to register bq275x0 RomMode driver\n");

    return ret;
}
module_init(bq275x0_RomMode_init);

static void __exit bq275x0_RomMode_exit(void)
{
    i2c_del_driver(&bq275x0_RomMode_driver);
}
module_exit(bq275x0_RomMode_exit);

MODULE_AUTHOR("Audi PC Huang <AudiPCHuang@fihtdc.com>");
MODULE_DESCRIPTION("bq275x0 dfi programming driver");
MODULE_LICENSE("GPL");
