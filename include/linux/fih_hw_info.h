/*
*Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved
*/
#ifndef _FIH_HW_INFO_H
#define _FIH_HW_INFO_H

typedef enum 
{
  Project_GUA = 1,
  Project_BMB,
  Project_TAP, 
  Project_MES, 
  Project_JLO, /* MTD-BSP-VT-HWID-05+ */
  PROJECT_MAX = 0xFF
} fih_product_id_type;

typedef enum 
{
  Phase_EVB = 1,
  Phase_DP = 10,
  Phase_SP = 20,
  Phase_SP2 = 21, /* MTD-BSP-VT-HWID-02+ */
  Phase_SP3 = 22,
  Phase_PreAP = 26,
  Phase_AP = 30,
  Phase_AP2 = 32,
  Phase_TP = 40,
  Phase_TP2 = 42,
  Phase_TP3 = 44,
  Phase_PQ = 50,
  Phase_MP = 60,
  Phase_Any = 0xFE,
  PHASE_MAX = 0xFF
}fih_product_phase_type;

typedef enum 
{
  BAND_18 = 1,
  BAND_125 = 2,  
  BAND_MAX = 0xFF
}fih_band_id_type;

typedef enum {
    SINGLE_SIM=1,
    DUAL_SIM,
    SIM_MAX=0xFF,
}fih_sim_type;

//MTD-BSP-REXER-SMEM-00+[
void fih_get_oem_info(void);
unsigned int fih_get_product_id(void);
unsigned int fih_get_product_phase(void);
unsigned int fih_get_band_id(void);
unsigned int fih_get_sim_id(void);
char *fih_get_amss_version(void);
//MTD-BSP-REXER-SMEM-00+]

#endif
