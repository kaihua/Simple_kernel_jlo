/* arch/arm/mach-msm/include/mach/rpc_nv.h
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
#ifndef __RPC_NV_H__
#define __RPC_NV_H__

#define NV_FIHDBG_SIZE	16
#define NV_IMEI_SIZE	9
#define NV_WLAN_MAC_SIZE	6
#define NV_BDADDR_SIZE	6
#define NV_PRODUCT_ID_SIZE	40

#define NV_MAX_BYTES	40

int rpc_nv_read_product_id(unsigned char *data);
int rpc_nv_write_product_id(unsigned char *data);
int rpc_nv_read_wlan_mac_addr(unsigned char *buf);
int rpc_nv_write_wlan_mac_addr(unsigned char *buf);
int rpc_nv_read_fihdbg_config(unsigned char *fih_debug);
int rpc_nv_write_fihdbg_config(unsigned char *fih_debug);
int rpc_nv_read_err_fatal_option(unsigned int *pvalue);
int rpc_nv_write_err_fatal_option(unsigned int value);
int rpc_nv_write_detect_hw_reset(unsigned int value);
int rpc_nv_write_cache_wt(unsigned int value);
int rpc_nv_read_bdaddr(unsigned char *buf);
int rpc_nv_write_bdaddr(unsigned char *buf);
int rpc_nv_read_imei(unsigned char *imei);
int rpc_nv_write_imei(unsigned char *imei);
int nv_to_imei(unsigned char *imei, unsigned char *nv);
int imei_to_nv(unsigned char *nv, unsigned char *imei);
#ifdef CONFIG_PROXIMITY_SENSOR_CALIBRATON_REPLACE
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+{
int rpc_nv_write_psensor_sensitivity(unsigned int buf);
int rpc_nv_read_psensor_sensitivity(unsigned int *buf);
//FIH-SW1-PERIPHERAL-AC-PSENSOR_SENSITIVITY-01+}
#endif
#endif	//__RPC_NV_H__
