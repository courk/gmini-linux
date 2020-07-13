/******************************************************************************* 
* Copyright (C) Marvell International Ltd. and its affiliates 
* 
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
********************************************************************************/

#ifndef __ASM_MACH_IRQS_H
#define __ASM_MACH_IRQS_H

/* Interrupt numbers definitions */
#define IRQ_LOCALTIMER				29
#define IRQ_LOCALWDT				30

#define IRQ_GIC_START				32

#ifdef CONFIG_ARCH_BERLIN2
#define GALOIS_APB_IRQ_START			(32 + IRQ_GIC_START)
#define GALOIS_SM_IRQ_START			(64 + IRQ_GIC_START)

#define IRQ_DHUBINTRAVIO0			(0 + IRQ_GIC_START)
#define IRQ_DHUBINTRAVIO1			(1 + IRQ_GIC_START)
#define IRQ_DHUBINTRVPRO			(2 + IRQ_GIC_START)
#define IRQ_ICTLINST0CPUIRQ			(3 + IRQ_GIC_START)
#define IRQ_CTI_NCTIIRQ_0			(4 + IRQ_GIC_START)
#define IRQ_INTRGFX3DM0				(5 + IRQ_GIC_START)
#define IRQ_INTRGFXM0				(6 + IRQ_GIC_START)
#define IRQ_SATAINTR				(7 + IRQ_GIC_START)
#define IRQ_ETHIRQ				(8 + IRQ_GIC_START)
#define IRQ_DRMINTR				(9 + IRQ_GIC_START)
#define IRQ_DRMFIGOINTR				(10 + IRQ_GIC_START)
#define IRQ_USB0INTERRUPT			(11 + IRQ_GIC_START)
#define IRQ_USB1INTERRUPT			(12 + IRQ_GIC_START)
#define IRQ_SM2SOCINT				(13 + IRQ_GIC_START)
#define IRQ_PTPIRQ				(14 + IRQ_GIC_START)
#define IRQ_SM2SOCHWINT0			(15 + IRQ_GIC_START)
#define IRQ_SM2SOCHWINT1			(16 + IRQ_GIC_START)
#define IRQ_SDIO_INTERRUPT			(17 + IRQ_GIC_START)
#define IRQ_INTRPB				(18 + IRQ_GIC_START)
#define IRQ_ZSPINT				(19 + IRQ_GIC_START)
#define IRQ_SDIO1_INTERRUPT			(20 + IRQ_GIC_START)
#define IRQ_SPDIF_RX_CHOVR_INTR			(21 + IRQ_GIC_START)
#define IRQ_SPDIF_RX_INTR			(22 + IRQ_GIC_START)
#define IRQ_GC360INTR				(23 + IRQ_GIC_START)
#define IRQ_ETH1IRQ				(24 + IRQ_GIC_START)
#define IRQ_DHUBINTRAVIO2			(25 + IRQ_GIC_START)
#define IRQ_INTRHWBLK				(26 + IRQ_GIC_START)
#define IRQ_INTRFIGO				(27 + IRQ_GIC_START)
#define IRQ_MMC_INTERRUPT			(28 + IRQ_GIC_START)
#define IRQ_CTI_NCTIIRQ_1			(29 + IRQ_GIC_START)
#define IRQ_PMU_CPU0				(30 + IRQ_GIC_START)
#define IRQ_PMU_CPU1				(31 + IRQ_GIC_START)


#elif defined (CONFIG_ARCH_BERLIN3)
#define GALOIS_APB_IRQ_START			(56 + IRQ_GIC_START)
#define GALOIS_SM_IRQ_START			(78 + IRQ_GIC_START)

#define IRQ_L2C					(0 + IRQ_GIC_START)
#define IRQ_PMU_CPU0				(1 + IRQ_GIC_START)
#define IRQ_PMU_CPU1				(2 + IRQ_GIC_START)
#define IRQ_PMU_CPU2				(3 + IRQ_GIC_START)
#define IRQ_PMU_CPU3				(4 + IRQ_GIC_START)
#define IRQ_COPYDMA_DONE			(5 + IRQ_GIC_START)
#define IRQ_COPYDMA_ERROR			(6 + IRQ_GIC_START)
#define IRQ_PNGDEC_STATUS			(7 + IRQ_GIC_START)
#define IRQ_PNGDEC_ERROR			(8 + IRQ_GIC_START)
#define IRQ_DAA					(9 + IRQ_GIC_START)
#define IRQ_SM2SOCHWINT0			(10 + IRQ_GIC_START)
#define IRQ_SM2SOCHWINT1			(11 + IRQ_GIC_START)
#define IRQ_G_GTOP				(12 + IRQ_GIC_START)
#define IRQ_G_RXTX				(13 + IRQ_GIC_START)
#define IRQ_G_TH_RXTX				(14 + IRQ_GIC_START)
#define IRQ_G_WAKEUP				(15 + IRQ_GIC_START)
#define IRQ_G_MISC				(16 + IRQ_GIC_START)
#define IRQ_NFC					(17 + IRQ_GIC_START)
#define IRQ_PB0					(18 + IRQ_GIC_START)
#define IRQ_PB1					(19 + IRQ_GIC_START)
#define IRQ_PB2					(20 + IRQ_GIC_START)
#define IRQ_PCIE0_MSI				(21 + IRQ_GIC_START)
#define IRQ_PCIE0				(22 + IRQ_GIC_START)
#define IRQ_PCIE0_PIO				(23 + IRQ_GIC_START)
#define IRQ_PCIE1_MSI				(24 + IRQ_GIC_START)
#define IRQ_PCIE1				(25 + IRQ_GIC_START)
#define IRQ_PCIE1_PIO				(26 + IRQ_GIC_START)
#define IRQ_PCIE2_MSI				(27 + IRQ_GIC_START)
#define IRQ_PCIE2				(28 + IRQ_GIC_START)
#define IRQ_PCIE2_PIO				(29 + IRQ_GIC_START)
#define IRQ_EMMC				(30 + IRQ_GIC_START)
#define IRQ_USIM0				(31 + IRQ_GIC_START)
#define IRQ_USIM1				(32 + IRQ_GIC_START)
#define IRQ_CIC0				(33 + IRQ_GIC_START)
#define IRQ_TSP_HWBLK				(34 + IRQ_GIC_START)
#define IRQ_TSP_FIGO				(35 + IRQ_GIC_START)
#define IRQ_DRM					(36 + IRQ_GIC_START)
#define IRQ_DRMFIGO				(37 + IRQ_GIC_START)
#define IRQ_BCM					(38 + IRQ_GIC_START)
#define IRQ_N2					(39 + IRQ_GIC_START)
#define IRQ_ICTLINST0CPUIRQ			(40 + IRQ_GIC_START)
#define IRQ_ICTLINST1CPUIRQ			(41 + IRQ_GIC_START)
#define IRQ_ICTLINST2CPUIRQ			(42 + IRQ_GIC_START)
#define IRQ_DMAVPRO				(43 + IRQ_GIC_START)
#define IRQ_DMAVPRO1				(44 + IRQ_GIC_START)
#define IRQ_ZSPINT				(45 + IRQ_GIC_START)
#define IRQ_INTRGFXM0				(46 + IRQ_GIC_START)
#define IRQ_INTRGFX3DM0				(47 + IRQ_GIC_START)
#define IRQ_INTRGFX3DM1				(48 + IRQ_GIC_START)
#define IRQ_DHUBINTRAVIO0			(49 + IRQ_GIC_START)
#define IRQ_DHUBINTRAVIO1			(50 + IRQ_GIC_START)
#define IRQ_DHUBINTRAVIO2			(51 + IRQ_GIC_START)
#define IRQ_SPDIF_RX				(52 + IRQ_GIC_START)
#define IRQ_SPDIF_RX_CHOVR			(53 + IRQ_GIC_START)
#define IRQ_TSEN0				(54 + IRQ_GIC_START)
#define IRQ_TSEN1				(55 + IRQ_GIC_START)

#endif

#define IRQ_APB_GPIOINST0			(0 + GALOIS_APB_IRQ_START)
#define IRQ_APB_GPIOINST1			(1 + GALOIS_APB_IRQ_START)
#define IRQ_APB_GPIOINST2			(2 + GALOIS_APB_IRQ_START)
#define IRQ_APB_GPIOINST3			(3 + GALOIS_APB_IRQ_START)
#define IRQ_APB_SSIINST1			(4 + GALOIS_APB_IRQ_START)
#define IRQ_APB_WDTINST0			(5 + GALOIS_APB_IRQ_START)
#define IRQ_APB_WDTINST1			(6 + GALOIS_APB_IRQ_START)
#define IRQ_APB_WDTINST2			(7 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_0			(8 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_1			(9 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_2			(10 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_3			(11 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_4			(12 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_5			(13 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_6			(14 + GALOIS_APB_IRQ_START)
#define IRQ_APB_TIMERINST1_7			(15 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST0			(16 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_GEN_CALL_INTR	(17 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_RX_UNDER_INTR	(18 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_RX_OVER_INTR	(19 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_RX_FULL_INTR	(20 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_TX_OVER_INTR	(21 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_TX_EMPTY_INTR	(22 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_RD_REQ_INTR		(23 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_TX_ABRT_INTR	(24 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_RX_DONE_INTR	(25 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_ACTIVITY_INTR	(26 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_STOP_DET_INTR	(27 + GALOIS_APB_IRQ_START)
#define IRQ_APB_I2CINST1_IC_START_DET_INTR	(28 + GALOIS_APB_IRQ_START)

#define IRQ_SM_WATCHDOG0			(0 + GALOIS_SM_IRQ_START)
#define IRQ_SM_WATCHDOG1			(1 + GALOIS_SM_IRQ_START)
#define IRQ_SM_WATCHDOG2			(2 + GALOIS_SM_IRQ_START)
#define IRQ_SM_TIMERS				(3 + GALOIS_SM_IRQ_START)
#define IRQ_SM_GPIO1				(4 + GALOIS_SM_IRQ_START)
#define IRQ_SM_SPI				(5 + GALOIS_SM_IRQ_START)
#define IRQ_SM_I2C0				(6 + GALOIS_SM_IRQ_START)
#define IRQ_SM_I2C1				(7 + GALOIS_SM_IRQ_START)
#define IRQ_SM_UART0				(8 + GALOIS_SM_IRQ_START)
#define IRQ_SM_UART1				(9 + GALOIS_SM_IRQ_START)
#define IRQ_SM_UART2				(10 + GALOIS_SM_IRQ_START)
#define IRQ_SM_GPIO0				(11 + GALOIS_SM_IRQ_START)
#define IRQ_SM_ADC				(12 + GALOIS_SM_IRQ_START)
#define IRQ_SM_SOC2SM				(13 + GALOIS_SM_IRQ_START)
#define IRQ_SM_TSEN				(14 + GALOIS_SM_IRQ_START)
#define IRQ_SM_WOL				(15 + GALOIS_SM_IRQ_START)
#define IRQ_SM_CEC				(16 + GALOIS_SM_IRQ_START)
#define IRQ_SM_FIFO				(17 + GALOIS_SM_IRQ_START)
#define IRQ_SM_ETH				(18 + GALOIS_SM_IRQ_START)
#define IRQ_SM_HPD				(19 + GALOIS_SM_IRQ_START)
#define IRQ_SM_IHPD				(20 + GALOIS_SM_IRQ_START)
#define IRQ_SM_HDMIRXPWR			(21 + GALOIS_SM_IRQ_START)
#define IRQ_SM_IHDMIRXPWR			(22 + GALOIS_SM_IRQ_START)

#define NR_IRQS					IRQ_SM_IHDMIRXPWR


/* For compatible with Galois */
#define IRQ_APB_TWSIINST0	IRQ_APB_I2CINST0
#define IRQ_APB_TWSIINST1	IRQ_APB_I2CINST1_IC_GEN_CALL_INTR
#define IRQ_APB_TWSIINST2	IRQ_SM_I2C0
#define IRQ_APB_TWSIINST3	IRQ_SM_I2C1

#endif /* __ASM_MACH_IRQS_H */
