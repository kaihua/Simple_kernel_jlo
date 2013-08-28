/*
 * Copyright (C) 2011 Foxconn.  All rights reserved.
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <asm/mach-types.h>
#include <mach/dbgcfgtool.h>

#ifdef CONFIG_FEATURE_FIH_SW3_REMOVE_SERIAL_DYNAMICALLY
extern int cached_read_fihdbg( unsigned char* fih_debug );

unsigned int fih_read_uart_switch_from_nv(void)
{
    int ret = 0;
    
    char tempcfg[16] = {0};
    
    ret = cached_read_fihdbg(tempcfg);

    if( ret < 0 )/* fail here */
	{
        /* if we fail, we don't open UART*/
        ret = 0;
    }
    else
    {
    	ret = tempcfg[DEBUG_UART_GROUP] & (1 << (DEBUG_UARTMSG_CFG % GROUP_SIZE));
    }

	return ret;
}
#endif



