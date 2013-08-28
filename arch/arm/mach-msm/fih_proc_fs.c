/*
* Copyright(C) 2011-2012 Foxconn International Holdings, Ltd. All rights reserved
*/
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/fih_hw_info.h>

unsigned int get_boot_info(void);  /* MTD-BSP-VT-RECOVERY-00+ */

static int proc_calc_metrics(char *page, char **start, off_t off,
				 int count, int *eof, int len)
{
	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}


static int device_model_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = fih_get_product_id();
	char ver[24]={0} ;
		
	switch (pi){
	case Project_GUA:
		strncpy(ver, "GUA",3);
		ver[3]='\0';
		break;
	case Project_BMB:
		strncpy(ver, "BMB",3);
		ver[3]='\0';
		break;

	case Project_TAP:
		strncpy(ver, "TAP",3);
		ver[3]='\0';
		break;
		
    case Project_MES:
		strncpy(ver, "MES",3);
		ver[3]='\0';
		break;
		
/* MTD-BSP-VT-HWID-01+[ */
    case Project_JLO:
		strncpy(ver, "JLO",3);
		ver[3]='\0';
		break;
/* MTD-BSP-VT-HWID-01+] */

	default:
		strncpy(ver, "Unkonwn Device Model",20);
		ver[20]='\0';
		break;
	}

	len = snprintf(page, count, "%s\n",
		ver);  /* MTD-BSP-VT-PROC-00* */
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}

static int baseband_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;

	int pp = fih_get_product_phase();
	char ver[25]={0};
	
	switch (pp){
	case Phase_EVB:
		strncpy(ver, "EVB", 3);
		ver[3]='\0';
		break; 
	case Phase_DP:
		strncpy(ver, "DP", 2);
		ver[2]='\0';
		break; 
	case Phase_SP:
		strncpy(ver, "SP", 2);
		ver[3]='\0';
		break; 
	/* MTD-BSP-VT-HWID-02+[ */
	case Phase_SP2:
		strncpy(ver, "SP2", 3);
		ver[3]='\0';
		break;
	/* MTD-BSP-VT-HWID-02+] */
	case Phase_SP3:
		strncpy(ver, "SP3", 3);
		ver[3]='\0';
		break;
	case Phase_AP:
		strncpy(ver, "AP", 2);
		ver[2]='\0';
		break;
	case Phase_AP2:
		strncpy(ver, "AP2", 3);
		ver[3]='\0';
		break;
	case Phase_PreAP:
		strncpy(ver, "PreAP", 5);
		ver[5]='\0';
		break;
	case Phase_TP:
		strncpy(ver, "TP", 2);
		ver[2]='\0';
		break;
	case Phase_TP2:
		strncpy(ver, "TP2", 3);
		ver[3]='\0';
		break;
	case Phase_TP3:
		strncpy(ver, "TP3", 3);
		ver[3]='\0';
		break;
	case Phase_PQ :
		strncpy(ver, "PQ", 2);
		ver[2]='\0';
		break; 
	case Phase_MP:
		strncpy(ver, "MP", 2);
		ver[2]='\0';
		break; 
	default:
		strncpy(ver, "Unkonwn Baseband version",24);
		ver[24]='\0';
		break;
	}

	len = snprintf(page, count, "%s\n",
		ver);  /* MTD-BSP-VT-PROC-00* */

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int band_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = fih_get_band_id();
	char ver[40]={0};

  switch (pi)
  {
    case BAND_18:
      strncpy( ver, "BAND_18", 7);
	  ver[7]='\0';
      break;
    case BAND_125:
      strncpy( ver, "BAND_125", 8);
	  ver[8]='\0';
      break;
    default:
      strncpy( ver, "Unkonwn RF band id", 18);
	  ver[18]='\0';
      break;
    }
    len = snprintf(page, count, "%s\n",ver);  /* MTD-BSP-VT-PROC-00* */
    
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int siminfo_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = fih_get_sim_id();
	char ver[24]={0} ;
		
	switch (pi){
	case SINGLE_SIM:
		strncpy( ver, "SINGLE SIM", 10);
	        ver[10]='\0';
		break;
	case DUAL_SIM:
		strncpy(ver, "DUAL SIM",8);
                ver[8]='\0';
		break;
	default:
		strncpy(ver, "Unkonwn SIM TYPE",16);
                ver[16]='\0';
		break;
	}

	len = snprintf(page, count, "%s\n", ver);  /* MTD-BSP-VT-PROC-00* */
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}

static int amss_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	char ver[30];
	snprintf(ver, sizeof(ver), fih_get_amss_version());

	len = snprintf(page, count, "%s\n",ver);  /* MTD-BSP-VT-PROC-00* */
		
	return proc_calc_metrics(page, start, off, count, eof, len);
}

/* MTD-BSP-VT-RECOVERY-00*[ */
static int reboot_info_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
    	int len;
	char ver[10]={0};
	unsigned int boot_info = get_boot_info();
    
       if (boot_info == 0x77665502)
    	{
           strncpy( ver, "recovery", 8);
		   ver[8]='\0';
    	}
       else
    	{
           strncpy( ver, "normal", 6);
		   ver[6]='\0';
    	}
	len = snprintf(page, count, "%s\n",ver); /* MTD-BSP-VT-PROC-00* */
		
	return proc_calc_metrics(page, start, off, count, eof, len);
}
/* MTD-BSP-VT-RECOVERY-00*] */

static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, fih_info[] = {
	{"devmodel",	device_model_read_proc},
	{"baseband",	baseband_read_proc},
	{"bandinfo",	band_read_proc},
	{"amssversion",	amss_version_read_proc},
	{"rebootinfo",	reboot_info_read_proc},  /* MTD-BSP-VT-RECOVERY-00* */
	{"siminfo", siminfo_read_proc},
	{NULL,},
};

void fih_info_init(void)
{	
	for (p = fih_info; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);
}
EXPORT_SYMBOL(fih_info_init);

void fih_info_remove(void)
{
	for (p = fih_info; p->name; p++)
		remove_proc_entry(p->name, NULL);
}
EXPORT_SYMBOL(fih_info_remove);
