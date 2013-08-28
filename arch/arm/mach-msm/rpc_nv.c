/* arch/arm/mach-msm/rpc_nv.c
 * Copyright (C) 2011 FIH Co., Ltd.  All rights reserved.
 * Author: Ken Chen <kenchen@fihspec.com>
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
#include <linux/slab.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>

#include "mach/rpc_nv.h"
#include "smd_private.h"
#include <linux/fs.h>
#include <mach/msm_rpcrouter.h>

#define NVPROG    0x3000000E
#define NVVERS    0x00090002
#define ONCRPC_NV_CMD_REMOTE_PROC 9

//#define NV_MAX_BYTES	40

typedef struct {
        struct rpc_request_hdr hdr;
        unsigned int cmd;
        unsigned int item;
        unsigned int more_data; 
        unsigned int disc;
	unsigned char data[0];
} bytes_req;

typedef struct {
        struct rpc_reply_hdr hdr;
        unsigned int retval;
        unsigned int more_data; 
        unsigned int disc;
	unsigned char data[0];
} bytes_rep;

#define NV_PRD_ID_I	8001
#define	NV_WLAN_MAC_ADDRESS_I	4678
#define NV_FIHDBG_I	8045 
#define	NV_ERR_FATAL_OPTIONS_I 905
#define NV_DETECT_HW_RESET_I	4399
#define NV_CACHE_WT_I	6470
#define NV_BD_ADDR_I	447
#define NV_UE_IMEI_I	550

//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
#define NV_PSENSOR_SENSITIVITY_I 8082
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}

#define NV_READ_F    0          /* Read item */
#define NV_WRITE_F    1         /* Write item */
#define NV_DONE_S    0

#define NV_ALIGN(x) (((x) + 3) & (~3))	//4 multiple

static int __rpc_nv_read_bytes__(unsigned int nv_item, unsigned int sz_bytes, void *nv_rbuf)
{
	int ret;
	struct msm_rpc_endpoint *nv_ep;
	unsigned char reqbuf[NV_ALIGN(sizeof(bytes_req)+NV_MAX_BYTES)];
	unsigned char repbuf[NV_ALIGN(sizeof(bytes_rep)+NV_MAX_BYTES)];
	bytes_req *req = (bytes_req *)reqbuf;
	bytes_rep *rep = (bytes_rep *)repbuf;
	unsigned int req_sz_bytes = NV_ALIGN((sizeof(bytes_req)+sz_bytes));	//must 4 multiple
	unsigned int rep_sz_bytes = NV_ALIGN((sizeof(bytes_rep)+sz_bytes));	//must 4 multiple

//	req = kzalloc(req_sz_bytes, GFP_KERNEL);
	req->cmd = cpu_to_be32(NV_READ_F);
	req->item = cpu_to_be32(nv_item);
	req->more_data = cpu_to_be32(1);
	req->disc = cpu_to_be32(nv_item);
	memset(req->data, 0xff, sz_bytes);

	rep->retval = cpu_to_be32(0xFFFFFFFF);	
	memset(rep->data, 0xFF, sz_bytes);

	nv_ep = msm_rpc_connect_compatible(NVPROG, NVVERS, 0);

	if (IS_ERR(nv_ep)) { 
		pr_err("%s() LINE:%d, nv_item %u init failed! error: %ld\n", __func__, __LINE__, nv_item, PTR_ERR(nv_ep));
		return -EINVAL;
	}

	ret = msm_rpc_call_reply (nv_ep, ONCRPC_NV_CMD_REMOTE_PROC, req, req_sz_bytes, rep, rep_sz_bytes, 5*HZ);

	if (ret<0) {
		pr_err("%s() LINE:%d, nv_item %u call failed! (%d)\n", __func__, __LINE__, nv_item, ret);
	}
	else if (be32_to_cpu(rep->retval) != NV_DONE_S) {
		pr_err("%s() LINE:%d, nv_item %u result failed! (%d)\n", __func__, __LINE__, nv_item, be32_to_cpu(rep->retval));
	}
	else {
		int i;

		ret = 0;
		pr_debug("nv_item read = %u\n", nv_item);
		for (i=0;i<sz_bytes;i++)
			pr_debug("[%02d,%02x]\n",i, rep->data[i]);
		memcpy(nv_rbuf, rep->data, sz_bytes);
	}
//	msm_rpc_close(nv_ep);

	return ret;
}

static int __rpc_nv_write_bytes__(unsigned int nv_item, unsigned int sz_bytes, void *nv_wbuf)
{
	int ret;
	struct msm_rpc_endpoint *nv_ep;
	unsigned char reqbuf[NV_ALIGN(sizeof(bytes_req)+NV_MAX_BYTES)];
	unsigned char repbuf[NV_ALIGN(sizeof(bytes_rep)+NV_MAX_BYTES)];
	bytes_req *req = (bytes_req *)reqbuf;
	bytes_rep *rep = (bytes_rep *)repbuf;
	unsigned int req_sz_bytes = NV_ALIGN((sizeof(bytes_req)+sz_bytes));	//must 4 multiple
	unsigned int rep_sz_bytes = NV_ALIGN((sizeof(bytes_rep)+sz_bytes));	//must 4 multiple

	req->cmd = cpu_to_be32(NV_WRITE_F);
	req->item = cpu_to_be32(nv_item);
	req->more_data = cpu_to_be32(1);
	req->disc = cpu_to_be32(nv_item);
	memcpy(req->data, nv_wbuf, sz_bytes);

	rep->retval = cpu_to_be32(0xFFFFFFFF);
	memset(rep->data, 0xFF, sz_bytes);

	nv_ep = msm_rpc_connect_compatible(NVPROG, NVVERS, 0);

	if (IS_ERR(nv_ep)) { 
		pr_err("%s() LINE:%d, nv_item %u init failed! error: %ld\n", __func__, __LINE__, nv_item, PTR_ERR(nv_ep));
		return -EINVAL;
	}

	ret = msm_rpc_call_reply (nv_ep, ONCRPC_NV_CMD_REMOTE_PROC, req, req_sz_bytes, rep, rep_sz_bytes, 5*HZ);

	if (ret<0) {
		pr_err("%s() LINE:%d, nv_item %u call failed! (%d)\n", __func__, __LINE__, nv_item, ret);
	}
	else if (be32_to_cpu(rep->retval) != NV_DONE_S) {
		pr_err("%s() LINE:%d, nv_item %u result failed! (%d)\n", __func__, __LINE__, nv_item, be32_to_cpu(rep->retval));
	}
	else {
		int i;

		ret = 0;
		pr_debug("nv_item write = %u\n", nv_item);
		for (i=0;i<sz_bytes;i++)
			pr_debug("[%02d,%02x]\n",i, rep->data[i]);
	}
//	msm_rpc_close(nv_ep);

	return ret;
}
/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_product_id
 *
 * Description :
 *     Read product id from nv item (NV_PRD_ID_I).
 *
 * Parameters  :
 *     Output: String of 40 bytes product id array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_product_id(unsigned char *data)
{
	return __rpc_nv_read_bytes__(NV_PRD_ID_I, NV_PRODUCT_ID_SIZE, data); 

}
EXPORT_SYMBOL(rpc_nv_read_product_id);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_product_id
 *
 * Description :
 *     Write product id to nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     Input: String of 40 bytes product id array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_product_id(unsigned char *data)
{
	return __rpc_nv_write_bytes__(NV_PRD_ID_I, NV_PRODUCT_ID_SIZE, data); 
}
EXPORT_SYMBOL(rpc_nv_write_product_id);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_wlan_mac_addr
 *
 * Description :
 *     Read wlan mac address from nv item (NV_WLAN_MAC_ADDRESS_I).
 *
 * Parameters  :
 *     Output: String of 6 bytes wlan mac address array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_wlan_mac_addr(unsigned char *buf)
{
	return __rpc_nv_read_bytes__(NV_WLAN_MAC_ADDRESS_I, NV_WLAN_MAC_SIZE, buf); 
}
EXPORT_SYMBOL(rpc_nv_read_wlan_mac_addr);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_wlan_mac_addr
 *
 * Description :
 *     Write wlan mac address from nv item (NV_WLAN_MAC_ADDRESS_I).
 *
 * Parameters  :
 *     Input: String of 6 bytes wlan mac address array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_wlan_mac_addr(unsigned char *buf)
{
	return __rpc_nv_write_bytes__(NV_WLAN_MAC_ADDRESS_I, NV_WLAN_MAC_SIZE, buf); 
}
EXPORT_SYMBOL(rpc_nv_write_wlan_mac_addr);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_fihdbg_config
 *
 * Description :
 *     Read fih debug configuration settings from nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     Output: String of 16 bytes fih debug configuration setting array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_fihdbg_config(unsigned char *fih_debug)
{
	return __rpc_nv_read_bytes__(NV_FIHDBG_I, NV_FIHDBG_SIZE, fih_debug); 
}
EXPORT_SYMBOL(rpc_nv_read_fihdbg_config);
 
/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_fihdbg_config
 *
 * Description :
 *     Write fih debug configuration settings to nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     Input: String of 16 bytes fih debug configuration setting array in 
 *     unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_fihdbg_config(unsigned char *fih_debug)
{
	return __rpc_nv_write_bytes__(NV_FIHDBG_I, NV_FIHDBG_SIZE, fih_debug); 
}
EXPORT_SYMBOL(rpc_nv_write_fihdbg_config);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_bdaddr
 *
 * Description :
 *     Read bdaddr from nv item (NV_BD_ADDR_I).
 *
 * Parameters  :
 *     Output: String of 6 bytes bdaddr array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_bdaddr(unsigned char *buf)
{
	return __rpc_nv_read_bytes__(NV_BD_ADDR_I, NV_BDADDR_SIZE, buf); 
}
EXPORT_SYMBOL(rpc_nv_read_bdaddr);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_bdaddr
 *
 * Description :
 *     Write bdaddr to nv item (NV_BD_ADDR_I).
 *
 * Parameters  :
 *     Input: String of 6 bytes bdaddr array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_bdaddr(unsigned char *buf)
{
	return __rpc_nv_write_bytes__(NV_BD_ADDR_I, NV_BDADDR_SIZE, buf); 
}
EXPORT_SYMBOL(rpc_nv_write_bdaddr);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_imei
 *
 * Description :
 *     Read IMEI from nv item (NV_UE_IMEI_I).
 *
 * Parameters  :
 *     Output: String of 9 bytes IMEI array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_imei(unsigned char *buf)
{
	return __rpc_nv_read_bytes__(NV_UE_IMEI_I, NV_IMEI_SIZE, buf); 
}
EXPORT_SYMBOL(rpc_nv_read_imei);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_imei
 *
 * Description :
 *     Write IMEI to nv item (NV_UE_IMEI_I).
 *
 * Parameters  :
 *     Input: String of 9 bytes IMEI array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_imei(unsigned char *buf)
{
	return __rpc_nv_write_bytes__(NV_UE_IMEI_I, NV_IMEI_SIZE, buf); 
}
EXPORT_SYMBOL(rpc_nv_write_imei);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_err_fatal_option
 *
 * Description :
 *     Read err fatal option from nv item (NV_ERR_FATAL_OPTIONS_I).
 *
 * Parameters  :
 *     Output: unsignd int *, 4 bytes spaces
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_err_fatal_option(unsigned int *pvalue)
{
	unsigned int opt;
	int ret;

	ret = __rpc_nv_read_bytes__(NV_ERR_FATAL_OPTIONS_I, sizeof(opt), &opt); 
	if (ret==0) {
		*pvalue = be32_to_cpu(opt);
	}

	return ret;
}
EXPORT_SYMBOL(rpc_nv_read_err_fatal_option);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_err_fatal_option
 *
 * Description :
 *     Write err fatal option to nv item (NV_ERR_FATAL_OPTIONS_I).
 *
 * Parameters  :
 *     Input: unsigned int, value of NV_ERR_FATAL_OPTIONS_I
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_err_fatal_option(unsigned int value)
{
	unsigned int opt;

	opt = cpu_to_be32(value);
	return __rpc_nv_write_bytes__(NV_ERR_FATAL_OPTIONS_I, sizeof(opt), &opt); 
}
EXPORT_SYMBOL(rpc_nv_write_err_fatal_option);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_detect_hw_reset
 *
 * Description :
 *     Write detect HW Resets nv item (NV_DETECT_HW_RESET_I).
 *
 * Parameters  :
 *     Input: unsigned int, value of NV_DETECT_HW_RESET_I
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_detect_hw_reset(unsigned int value)
{
	unsigned int detect_hw_reset;

	detect_hw_reset = cpu_to_be32(value);
	return __rpc_nv_write_bytes__(NV_DETECT_HW_RESET_I, sizeof(detect_hw_reset), &detect_hw_reset); 
}
EXPORT_SYMBOL(rpc_nv_write_detect_hw_reset);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_cache_wt
 *
 * Description :
 *     Write write through cache to nv item (NV_CACHE_WT_I).
 *
 * Parameters  :
 *     Input: unsigned int, value of NV_CACHE_WT_I
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_cache_wt(unsigned int value)
{
	unsigned int cache_wt;

	cache_wt = cpu_to_be32(value);
	return __rpc_nv_write_bytes__(NV_CACHE_WT_I, sizeof(cache_wt), &cache_wt); 
}
EXPORT_SYMBOL(rpc_nv_write_cache_wt);

/*--------------------------------------------------------------------------*
 * Function    : nv_to_imei
 *
 * Description :
 *     Convert IMEM NV(NV_UE_IMEI_I) to actual IMEI.
 *
 * Parameters  :
 *     imei: Output string of 15 bytes IMEI array in unsigned char.
 *     nv:   Input string of 9 bytes IMEI NV array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int nv_to_imei(unsigned char *imei, unsigned char *nv)
{
	imei[0] = (nv[1] & 0xf0) >> 4;
	imei[1] = nv[2] & 0xf;
	imei[2] = (nv[2] & 0xf0) >> 4;
	imei[3] = nv[3] & 0xf;
	imei[4] = (nv[3] & 0xf0) >> 4;
	imei[5] = nv[4] & 0xf;
	imei[6] = (nv[4] & 0xf0) >> 4;
	imei[7] = nv[5] & 0xf;
	imei[8] = (nv[5] & 0xf0) >> 4;
	imei[9] = nv[6] & 0xf;
	imei[10] = (nv[6] & 0xf0) >> 4;
	imei[11] = nv[7] & 0xf;
	imei[12] = (nv[7] & 0xf0) >> 4;
	imei[13] = nv[8] & 0xf;
	imei[14] = (nv[8] & 0xf0) >> 4;
	return 0;
}
EXPORT_SYMBOL(nv_to_imei);

/*--------------------------------------------------------------------------*
 * Function    : imei_to_nv
 *
 * Description :
 *     Convert actual IMEI to IMEM NV(NV_UE_IMEI_I)
 *
 * Parameters  :
 *     nv:   Output string of 9 bytes IMEI NV array in unsigned char.
 *     imei: Input string of 15 bytes IMEI array in unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int imei_to_nv(unsigned char *nv, unsigned char *imei)
{
	nv[0] = 0x8;
	nv[1] = ((imei[0] << 4) & 0xf0) | 0xa;
	nv[2] = ((imei[2] << 4) & 0xf0) | (imei[1] & 0xf);
	nv[3] = ((imei[4] << 4) & 0xf0) | (imei[3] & 0xf);
	nv[4] = ((imei[6] << 4) & 0xf0) | (imei[5] & 0xf);
	nv[5] = ((imei[8] << 4) & 0xf0) | (imei[7] & 0xf);
	nv[6] = ((imei[10] << 4) & 0xf0) | (imei[9] & 0xf);
	nv[7] = ((imei[12] << 4) & 0xf0) | (imei[11] & 0xf);
	nv[8] = ((imei[14] << 4) & 0xf0) | (imei[13] & 0xf);
	return 0;
}
EXPORT_SYMBOL(imei_to_nv);
/*FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{*/
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_read_psensor_sensitivity
 *
 * Description :
 *     Read sensitivity from nv item (NV_PSENSOR_SENSITIVITY_I).
 *
 * Parameters  :
 *     
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_read_psensor_sensitivity(unsigned int *buf)
{
	unsigned int sensitivity;
        int ret;
	//sensitivity = cpu_to_be32(buf);
	ret= __rpc_nv_read_bytes__(NV_PSENSOR_SENSITIVITY_I, sizeof(sensitivity), &sensitivity); 

	if (ret==0) {
		*buf = be32_to_cpu(sensitivity);
	}

	return ret;

}
EXPORT_SYMBOL(rpc_nv_read_psensor_sensitivity);

/*--------------------------------------------------------------------------*
 * Function    : rpc_nv_write_psensor_sensitivity
 *
 * Description :
 *     Write sensitivity to nv item (NV_PSENSOR_SENSITIVITY_I).
 *
 * Parameters  :
 *     
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int rpc_nv_write_psensor_sensitivity(unsigned int buf)
{
	unsigned int sensitivity;

	sensitivity = cpu_to_be32(buf);
	return __rpc_nv_write_bytes__(NV_PSENSOR_SENSITIVITY_I, sizeof(sensitivity), &sensitivity); 
}
EXPORT_SYMBOL(rpc_nv_write_psensor_sensitivity);
#endif
/*FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}*/

