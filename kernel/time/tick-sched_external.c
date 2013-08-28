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
 
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/percpu.h>
#include <linux/profile.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/rq_stats.h>

#include <asm/irq_regs.h>

#include "tick-internal.h"

#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/kallsyms.h>

u64 Last_checked_jiffies = 0;
u64 LastIdleTime = 0;
extern unsigned int debug_cpu_usage_enable;

#define CPU_USAGE_CHECK_INTERVAL_MS 1000  /* 1000ms */

/*Kernel-SC-add-cpuFreq-info-01+[*/
/* acpuclk_get_rate(): get REAL cpu frequency*/
extern unsigned long acpuclk_get_rate(int cpu);
/* cpufreq_quick_get(): get cpu governor's current cpu frequency. Return value will be same as what is shown in scaling_cur_freq in sysfs.*/
unsigned int cpufreq_quick_get(unsigned int cpu);
/*Kernel-SC-add-cpuFreq-info-01+]*/

long get_cpu_usage(void)
{
	u64 TotalTickCount, CurrTime, Temp;
	u64 IdleTickCount, CurrIdleIdleTime;
	long Usage;

	/* Collect the current time and idle time */
	CurrTime = jiffies_64;
	CurrIdleIdleTime = kcpustat_cpu(smp_processor_id()).cpustat[CPUTIME_IDLE]; /*Kernel-SC-add-cpu-usage-func-00**/

	/* Calculate the time interval */
	TotalTickCount = CurrTime - Last_checked_jiffies;

	/* Calculate the busy rate */
	IdleTickCount = CurrIdleIdleTime - LastIdleTime;
	if (TotalTickCount >= IdleTickCount)
	{
		Temp = 100 * (TotalTickCount - IdleTickCount);
		do_div(Temp,TotalTickCount);
		Usage = Temp;
	}
	else
	{
		Usage = 0;
	}

	Last_checked_jiffies = jiffies_64;
	LastIdleTime = CurrIdleIdleTime;

	return Usage;
}

void count_cpu_time(void)
{
	
	if (unlikely(debug_cpu_usage_enable == 1 && (1000*(jiffies_64 - Last_checked_jiffies)/HZ >= CPU_USAGE_CHECK_INTERVAL_MS)))
	{
		struct task_struct * p = current;
		struct pt_regs *regs = get_irq_regs();

		/*Kernel-SC-add-cpuFreq-info-01+[*/
		unsigned long cpu_freq = acpuclk_get_rate(smp_processor_id()); /*real cpu frequency*/
		unsigned long cpu_curr_freq = (unsigned long) cpufreq_quick_get(smp_processor_id()); /*cpu governor's current cpu frequency*/
		/*Kernel-SC-add-cpuFreq-info-01+]*/

		if (likely(p != 0))
		{
			if (regs != 0)
			{
				if (regs->ARM_pc <= TASK_SIZE)  /* User space */
				{
					struct mm_struct *mm = p->mm;
					struct vm_area_struct *vma;
					struct file *map_file = NULL;

					/* Parse vma information */
					vma = find_vma(mm, regs->ARM_pc);

					if (vma != NULL)
					{
						map_file = vma->vm_file;
					
						if (map_file)  /* memory-mapped file */
						{
							printk(KERN_INFO "[CPU] %3ld%% LR=0x%08lx PC=0x%08lx [U][%4d][%s][%s+0x%lx] CPUFreq=%lu CurrFreq=%lu\r\n",
								get_cpu_usage(),
								regs->ARM_lr, 
								regs->ARM_pc, 
								(unsigned int) p->pid,
								p->comm,
								map_file->f_path.dentry->d_iname, 
								regs->ARM_pc - vma->vm_start,
								cpu_freq,
								cpu_curr_freq);/*Kernel-SC-add-cpuFreq-info-01**/
						}
						else 
						{
							const char *name = arch_vma_name(vma);
							if (!name) 
							{
								if (mm) 
								{
									if (vma->vm_start <= mm->start_brk &&
										vma->vm_end >= mm->brk) 
									{
										name = "heap";
									} 
									else if (vma->vm_start <= mm->start_stack &&
										vma->vm_end >= mm->start_stack) 
									{
										name = "stack";
									}
								}
								else 
								{
									name = "vdso";
								}
							}
							printk(KERN_INFO "[CPU] %3ld%% LR=0x%08lx PC=0x%08lx [U][%4d][%s][%s] CPUFreq=%lu CurrFreq=%lu\r\n",
								get_cpu_usage(),
								regs->ARM_lr, 
								regs->ARM_pc, 
								(unsigned int) p->pid,
								p->comm, 
								name,
								cpu_freq,
								cpu_curr_freq);/*Kernel-SC-add-cpuFreq-info-01**/
						}
					}
				}
				else /* Kernel space */
				{
					printk(KERN_INFO "[CPU] %3ld%% LR=0x%08lx PC=0x%08lx [K][%4d][%s] CPUFreq=%lu CurrFreq=%lu", 
						get_cpu_usage(),
						regs->ARM_lr, 
						regs->ARM_pc, 
						(unsigned int) p->pid,
						p->comm,
						cpu_freq,
						cpu_curr_freq);/*Kernel-SC-add-cpuFreq-info-01**/
					
					#ifdef CONFIG_KALLSYMS
					print_symbol("[%s]\r\n", regs->ARM_pc);
					#else
					printk("\r\n");
					#endif
				}
			}
			else  /* Can't get PC & RA address */
			{
				printk(KERN_INFO "[CPU] %3ld%% [%s] CPUFreq=%lu CurrFreq=%lu\r\n", get_cpu_usage(), p->comm, cpu_freq, cpu_curr_freq);/*Kernel-SC-add-cpuFreq-info-01**/
			}
		}
		else  /* Can't get process information */
		{
			printk(KERN_INFO "[CPU] %3ld%% ERROR: p=0x%08lx, regs=0x%08lx CPUFreq=%lu CurrFreq=%lu\r\n", get_cpu_usage(), (long)p, (long)regs, cpu_freq, cpu_curr_freq);/*Kernel-SC-add-cpuFreq-info-01**/
		}
	}
}