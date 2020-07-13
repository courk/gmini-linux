/********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
 * on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/page.h>
#include <asm/memory.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <mach/galois_platform.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"


#ifndef CONFIG_BERLIN_SM

#ifdef CONFIG_BERLIN2CDP
  #define SM_APB_WDT_VER 0x3130372A
  #define RA_smSysCtl_SM_WDT_MASK 0x003C
#else
  #define SM_APB_WDT_VER 0x3130332A
  #define RA_smSysCtl_SM_WDT_MASK 0x0034
#endif

#define SM_APB_ICTL_BASE        (SOC_SM_APB_REG_BASE)
#define SM_APB_WDT0_BASE        (SOC_SM_APB_REG_BASE + 0x1000)
#define SM_APB_WDT1_BASE        (SOC_SM_APB_REG_BASE + 0x2000)
#define SM_APB_WDT2_BASE        (SOC_SM_APB_REG_BASE + 0x3000)

static void BFM_HOST_Bus_Write32(unsigned int addr, unsigned int val)
{
	writel(val, addr);
}

static void BFM_HOST_Bus_Read32(unsigned int addr, unsigned int *val)
{
	if(val) *val =readl(addr);
}

// use watchdog to reset Soc when SM no used
static int Galois_SM_WDT(unsigned int wdt_instance, unsigned int rst_type, int int_mode)
{
	unsigned int read,data,counter,stat, raw_status,wdt_base, ctrl_base,ictl_base;
	volatile int i,j,iResult;
	iResult=0;

	if (wdt_instance>5)
	{
		printk(" Invalid WDT instance.\n");
		iResult = 1;
	}
	ctrl_base = SM_SYS_CTRL_REG_BASE;
	ictl_base = SM_APB_ICTL_BASE; //SM_APB_ICTL0_BASE;

	// select WDT register base
	switch (wdt_instance)
	{
		case 0:
			wdt_base = SM_APB_WDT0_BASE;
			break;
		case 1:
			wdt_base = SM_APB_WDT1_BASE;
			break;
		case 2:
			wdt_base = SM_APB_WDT2_BASE;
			break;
		case 3:
			wdt_base = APB_WDT_INST0_BASE; //APB_WDT0_BASE;
			break;
		case 4:
			wdt_base = APB_WDT_INST1_BASE; //APB_WDT1_BASE;
			break;
		case 5:
			wdt_base = APB_WDT_INST2_BASE; //APB_WDT2_BASE;
			break;
		default:
			wdt_base = SM_APB_WDT0_BASE;
			break;
	}

	// check SPI ID
	BFM_HOST_Bus_Read32((wdt_base + 0x18), &read);
	if (read != SM_APB_WDT_VER)
	{
		printk(" WDT ID incorrect.\n");
		iResult = 1;
	}
	// setup RST MASK register
	data=0x3F;
	if (rst_type==0)
		data &=~( 1 <<(wdt_instance+3));                // soc reset
	else
		data &=~( 1 <<(wdt_instance));

	BFM_HOST_Bus_Write32( (ctrl_base+RA_smSysCtl_SM_WDT_MASK),data);       // sm reset
	BFM_HOST_Bus_Read32((ctrl_base+RA_smSysCtl_SM_WDT_MASK),&read);

	// setup WDT mode and time out value
	if (int_mode==0)
		BFM_HOST_Bus_Write32( (wdt_base+0x00),0x10);
	else
		BFM_HOST_Bus_Write32( (wdt_base+0x00),0x12);      //
	BFM_HOST_Bus_Write32( (wdt_base+0x04),0x08);          // time out around 2^(16 + 6) / 25M = 0.64 sec
	BFM_HOST_Bus_Read32((wdt_base+0x00),&read);
	read |=0x01;                                          // enable WDT
	BFM_HOST_Bus_Write32( (wdt_base+0x00),read);
	BFM_HOST_Bus_Write32( (wdt_base+0x0C),0x76);          // restart counter
	// kick dog three times
	for (i=0;i<1; i++)
	{
		for (;;)
		{
			for (j=0;j<100;j++);
				BFM_HOST_Bus_Read32((wdt_base+0x08),&counter);// current counter
			BFM_HOST_Bus_Read32((wdt_base+0x10),&stat);// read STAT
			if (stat ==1)
			{
				printk(" watchdog Stat = 1, Break!\n");
				break;
			}
			BFM_HOST_Bus_Read32((wdt_base+0x08),&counter);       // current counter
			if(int_mode==0 & counter < 0x10000) break;
		}
	// kick dog, pet dog whatever ..
		BFM_HOST_Bus_Read32((ictl_base+0x18),&raw_status);
		BFM_HOST_Bus_Read32((wdt_base+0x14),&read);           // read EOI to clear interrupt stat
		BFM_HOST_Bus_Write32( (wdt_base+0x0C),0x76);          // restart counter

		BFM_HOST_Bus_Read32((ictl_base+0x18),&raw_status);
		BFM_HOST_Bus_Read32((wdt_base+0x10),&stat);
	}
	printk(" Wait for RESET!!! \n");
	while(1)
	{
		for (j=0;j<10000;j++);
		BFM_HOST_Bus_Read32((wdt_base+0x08),&counter);
		if(counter>0x30000)
		{
		}
		else
		{
			break;
		}
	}
	for (j=0;j<100000;j++);
	return iResult;
}

static galois_soc_watchdog_reset(void)
{
#ifdef CONFIG_BERLIN2CDP
	Galois_SM_WDT(0, 1, 0);
#else
	Galois_SM_WDT(3, 1, 0);
#endif
	for (;;);
}

static int __init reset_init(void)
{
	arm_pm_restart = galois_soc_watchdog_reset;
	return 0;
}
void galois_arch_reset(char mode, const char *cmd)
{
	galois_soc_watchdog_reset();
}

late_initcall(reset_init);
#endif
