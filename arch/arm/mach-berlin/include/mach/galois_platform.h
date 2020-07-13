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

#ifndef __GALOIS_PLATFORM_H
#define __GALOIS_PLATFORM_H

#define MEMMAP_APBPERIF_REG_BASE	0xF7E80000
#define MEMMAP_CHIP_CTRL_REG_BASE	0xF7EA0000
#define MEMMAP_SM_REG_BASE		0xF7F80000

#ifdef CONFIG_BERLIN2CDP
#define MEMMAP_CA7_REG_BASE             0xF7920000
#endif

#define APB_GPIO_INST0_BASE		(MEMMAP_APBPERIF_REG_BASE + 0x0400)
#define APB_GPIO_INST1_BASE		(MEMMAP_APBPERIF_REG_BASE + 0x0800)
#define APB_GPIO_INST2_BASE		(MEMMAP_APBPERIF_REG_BASE + 0x0C00)
#define APB_GPIO_INST3_BASE		(MEMMAP_APBPERIF_REG_BASE + 0x1000)
#define APB_I2C_INST0_BASE		(MEMMAP_APBPERIF_REG_BASE + 0x1400)
#define APB_I2C_INST1_BASE		(MEMMAP_APBPERIF_REG_BASE + 0x1800)

#define APB_WDT_INST0_BASE              (MEMMAP_APBPERIF_REG_BASE + 0x2000)
#define APB_WDT_INST1_BASE              (MEMMAP_APBPERIF_REG_BASE + 0x2400)
#define APB_WDT_INST2_BASE              (MEMMAP_APBPERIF_REG_BASE + 0x2800)

#define SOC_SM_APB_REG_BASE		(MEMMAP_SM_REG_BASE + 0x40000)

#define SM_APB_GPIO1_BASE		(SOC_SM_APB_REG_BASE + 0x5000)
#define SM_APB_I2C0_BASE		(SOC_SM_APB_REG_BASE + 0x7000)
#define SM_APB_I2C1_BASE		(SOC_SM_APB_REG_BASE + 0x8000)
#define SM_APB_GPIO0_BASE		(SOC_SM_APB_REG_BASE + 0xC000)
#define SM_APB_GPIO_BASE		SM_APB_GPIO0_BASE
#define APB_I2C_INST2_BASE              SM_APB_I2C0_BASE
#define APB_I2C_INST3_BASE              SM_APB_I2C1_BASE

#define SM_SYS_CTRL_REG_BASE            (SOC_SM_APB_REG_BASE + 0xD000)
#endif
