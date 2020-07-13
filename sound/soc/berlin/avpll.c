/*************************************************************************************
*       Copyright (C) 2007-2011
*       Copyright ? 2007 Marvell International Ltd.
*
*       This program is free software; you can redistribute it and/or
*       modify it under the terms of the GNU General Public License
*       as published by the Free Software Foundation; either version 2
*       of the License, or (at your option) any later version.
*
*       This program is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY; without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*       GNU General Public License for more details.
*
*       You should have received a copy of the GNU General Public License
*       along with this program; if not, write to the Free Software
*       Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
***************************************************************************************/

#include <linux/module.h>
#include <linux/version.h>

#include "berlin_memmap.h"
#include "api_avpll.h"
#include "avioGbl.h"

#define AVPLL_A (MEMMAP_AVIO_GBL_BASE + RA_avioGbl_AVPLLA)
#define PHY_HOST_Bus_Read32(addr_var, u32) *((volatile unsigned*)&(u32)) = *((volatile unsigned*)&(addr_var))
#define PHY_HOST_Bus_Write32(addr_var, u32) *((volatile unsigned*)&(addr_var)) = (u32)
#define GA_REG_WORD32_READ(addr, holder)    (*(holder) = (*((volatile unsigned int *)(addr))))
#define GA_REG_WORD32_WRITE(addr, data)     ((*((volatile unsigned int *)(addr))) = ((unsigned int)(data)))

enum
{
	VCO_FREQ_1_512G=0,
	VCO_FREQ_1_620G,
	VCO_FREQ_1_856G,
	VCO_FREQ_2_160G,
	VCO_FREQ_2_227G,
	VCO_FREQ_2_520G,
	VCO_FREQ_2_700G,
	VCO_FREQ_2_970G,
};

typedef struct
{
	int vco_freq_index;
	float clkout_freq; //in MHz
	int freq_offset; //signed number
	unsigned int p_sync2;
	unsigned int p_sync1;
	unsigned int post_div;
	unsigned int post_div_0p5;
	unsigned int invalid; //this indicate it is a valid setting or not: 0 is valid, 1 is not
} CLK_FREQ;

//all the VCO freq reqired for video and audio in MHz
double avpll_vcoFreqs[]=
{
	1512,   //8 bit HDMI and 12 bit HMDI
	1620,   //12 bit HDMI
	1856.25,//10 bit HDMI
	2160,   //8 bit HDMI
	2227.5, //12 bit HDMI
	2520,   //10 bit HDMI
	2700,   //10 bit HDMI
	2970,   //8 bit HDMI
};

//from Section 7 table
unsigned char avpll_avpllRegFBDIV[]=
{
	60,  //VCO_FREQ_1_512G,
	65,  //VCO_FREQ_1_620G,
	74,  //VCO_FREQ_1_856G,
	86,  //VCO_FREQ_2_160G,
	89,  //VCO_FREQ_2_227G,
	101, //VCO_FREQ_2_520G,
	108, //VCO_FREQ_2_700G,
	119, //VCO_FREQ_2_970G,
};

//from Section 7 table, bit18 is sign bit
//Note that sign bit=0 for negative
//sign bit=1 for positive
unsigned int avpll_avpllRegFREQ_OFFSET_C8[]=
{
	(33554),       //VCO_FREQ_1_512G,
	((1<<18)|(12905)),//VCO_FREQ_1_620G,
	(14169),       //VCO_FREQ_1_856G,
	(19508),       //VCO_FREQ_2_160G,
	(4712),        //VCO_FREQ_2_227G
	((1<<18)|(8305)),//VCO_FREQ_2_520G,
	(00000),       //VCO_FREQ_2_700G,
	((1<<18)|(7049)),//VCO_FREQ_2_970G,
};

unsigned int avpll_avpllRegSPEED[]=
{
	0x2,    //1.5G<F<=1.7G , for 1.512G
	0x2,    //1.5G<F<=1.7G , for 1.62G
	0x3,    //1.7G<F<=1.9G , for 1.856G
//      0x4,    //1.9G<F<=2.1G
	0x5,    //2.1G<F<=2.3G , for 2.16G
	0x5,    //2.1G<F<=2.3G , for 2.227G
//      0x6,    //2.3G<F<=2.45G
	0x7,    //2.45G<F<=2.6G, for 2.52G
	0x8,    //2.6G<F<= 2.75G, for 2.7G
//      0x9,    //2.75G<F<= 2.9G
	0xa,    //2.9G<F<=3.0G, for 2.97G
};

/*Interpolator current setting for different frequency
 VCO Frequency '  INTPI
*/
unsigned int avpll_avpllRegINTPI[]=
{
	0x3,    //for 1.512G
	0x3,    //for 1.62G
	0x5,    //for 1.856G
	0x7,    //for 2.16G
	0x7,    //for 2.227G
	0x9,    //for 2.52G
	0xa,    //for 2.7G
	0xb,    //for 2.97G
};

unsigned int avpll_avpllRegINTPR[]=
{
	0x4,    //for 1.512G
	0x4,    //for 1.62G
	0x4,    //for 1.856G
	0x3,    //for 2.16G
	0x3,    //for 2.227G
	0x3,    //for 2.52G
	0x1,    //for 2.7G
	0x1,    //for 2.97G
};

volatile SIE_avPll regSIE_avPll;
int vcoFreqIndex = VCO_FREQ_2_160G;
int avpll_pll_A_VCO_Setting = VCO_FREQ_2_160G;
int avpll_pll_B_VCO_Setting = VCO_FREQ_1_620G;
static CLK_FREQ clk_freqs_computed[VCO_FREQ_2_970G+1];

/* PPM Adjustment methods */
static int avpll_inited = 0;
static double org_ppm = 0.0;
static double cur_ppm = 0.0;

#define OFFSET_SIZE    19
#define ONE_MILLION    1000000.0
#define OFFSET_MULTIPLIER_REFERENCE            ((double)(1<<22))

static double offset_2_ppm(int offset) {
	int v = offset & 0x3ffff;
	if(offset & 0x40000) v = -v;
	return -((ONE_MILLION * v)/(OFFSET_MULTIPLIER_REFERENCE + v));
}

// This is used to convert the real_ppm to nominal reference frequency to real base ref frequency.
// Fx/Fr = (1+ ppm_real) = Fx/f0*f0/Fr = (1+Offset_X/P)(1+ppm0) then Offset_X = P*(ppm_real-ppm0)/(1+ppm0)
// however the input parameter ppm = ppm0+ ppm_real(delta) so offset_X = P*(ppm -2*ppm0)/(1+ppm0)
static int HARD_LIMITER_19BITS(int x)
{
	int max_limit = (1 << (OFFSET_SIZE - 1)) - 1;
	int min_limit = ~(max_limit - 1);
	return clamp_val(x, min_limit, max_limit);
}

static int ppm_2_offset(double ppm, double ppm0)
{
	double c = OFFSET_MULTIPLIER_REFERENCE * (ppm - 2*ppm0) / (ONE_MILLION + ppm0);
	int offset = (int)c;
	offset = HARD_LIMITER_19BITS(offset);
	return (offset < 0) ? ((-offset) | 0x40000) : offset;
}

static void cpu_cycle_count_delay(SIGN32 ns)
{
	unsigned int start, diff, end;

	GA_REG_WORD32_READ(0xF7E82C04, &start);

	do {
		GA_REG_WORD32_READ(0xF7E82C04, &end);
		if(start >= end)
			diff = start - end;
		else
			diff = 1000*1000 + start - end;
	} while(diff < ns/10);
}

static int avpll_gcd(unsigned int m, unsigned int n)
{
	int rem;
	while(n!=0) {
		rem=m%n;
		m=n;
		n=rem;
	}

	return(m);
}

static int avpll_compute_freq_setting(unsigned int vco_freq_index, unsigned int target_freq)
{
	double offset, offset_percent;
	double divider;
	double ratio;
	int int_divider;

	memset(&(clk_freqs_computed[vco_freq_index]), 0, sizeof(CLK_FREQ));

	clk_freqs_computed[vco_freq_index].vco_freq_index=vco_freq_index;
	clk_freqs_computed[vco_freq_index].clkout_freq=(float)target_freq/1000000;

	//.5 divider is only used when divider is less than 24
	//This matches the settings in audio freq table in the IP doc
	ratio = avpll_vcoFreqs[vco_freq_index]*1000000/target_freq;

	if(ratio < 24) {
		//allow 0.5 divider, round to closest 0.5
		int_divider = (int)(ratio*2+0.5);
	} else {
		//round to closest int
		int_divider = ((int)(ratio+0.5))*2;
	}

	divider = ((double)int_divider)/2;

	clk_freqs_computed[vco_freq_index].post_div_0p5= (int_divider&1);
	clk_freqs_computed[vco_freq_index].post_div=((int)(divider));

	//now figure out the offset
	offset_percent = (target_freq*divider/1000000 - avpll_vcoFreqs[vco_freq_index])/avpll_vcoFreqs[vco_freq_index];

	offset = 4194304*offset_percent/(1+offset_percent);

	if(offset<=-262144 || offset>= 262144) {
		//offset cannot be achieved
		clk_freqs_computed[vco_freq_index].invalid = 1;
		return 1;
	} else {
		unsigned int vco_ratio, freq_ratio, gcd_val;

		//for rounding
		if(offset>0) offset+=0.5;
		else offset-=0.5;

		clk_freqs_computed[vco_freq_index].freq_offset = (int)(offset);
		gcd_val= avpll_gcd(avpll_vcoFreqs[vco_freq_index]*1000000, target_freq*divider);
		vco_ratio = (int)(avpll_vcoFreqs[vco_freq_index]*1000000/gcd_val);
		freq_ratio = (int)(target_freq*divider/gcd_val);

		if((gcd_val/1000)<1000) {
			clk_freqs_computed[vco_freq_index].p_sync2=freq_ratio;
			clk_freqs_computed[vco_freq_index].p_sync1=vco_ratio;
		} else {
			clk_freqs_computed[vco_freq_index].p_sync2=freq_ratio*((int)(gcd_val/1000000+0.5));
			clk_freqs_computed[vco_freq_index].p_sync1=vco_ratio*((int)(gcd_val/1000000+0.5));
		}
	}

	return 0;
}

static void avpll_powerDownChannel(SIE_avPll *avPllBase, int chId)
{
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_PU = 0;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
}

static void avpll_powerUpChannel(SIE_avPll *avPllBase, int chId)
{
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_PU = 1;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
}

static void avpll_powerDown(SIE_avPll *avPllBase)
{
	int chId;

	//avPllBase->uctrlPLL_PU = 0;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_PU = 0;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);

	for(chId=1; chId<=7; chId++) {
		avpll_powerDownChannel(avPllBase, chId);
	}

	//avPllBase->ie_C8.uctrl_PU = 0;
	PHY_HOST_Bus_Read32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	(regSIE_avPll.ie_C8).uctrl_PU = 0;
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
}

static void avpll_powerUp(SIE_avPll *avPllBase)
{
	int chId;

	for(chId=1; chId<=7; chId++) {
		avpll_powerUpChannel(avPllBase, chId);
	}

	//avPllBase->ie_C8.uctrl_PU = 1;
	PHY_HOST_Bus_Read32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	(regSIE_avPll.ie_C8).uctrl_PU = 1;
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);

	//avPllBase->uctrlPLL_PU = 1;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_PU = 1;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
}

static void avpll_assertPllReset(SIE_avPll *avPllBase)
{
	//assert reset
	//avPllBase->uctrlPLL_RESET=1;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_RESET = 1;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
}

//assert reset for channel1 to 7
static void avpll_assertCxReset(SIE_avPll *avPllBase)
{
	int chId;

	for(chId=1; chId<=7; chId++) {
		//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_RESET = 1;
		PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl3, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl3);
		((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_RESET = 1;
		PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl3, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl3);
	}
}

static void avpll_setVDDL_VCOref(SIE_avPll *avPllBase)
{
	//T32avPll_ctrlPLL ctrlPLL;
	//ctrlPLL.u32 = avPllBase->u32avPll_ctrlPLL;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);

	//ctrlPLL.uctrlPLL_VCO_REF1P45_SEL = 0x3;
	//ctrlPLL.uctrlPLL_VDDL = 0xF;// set VDDL to 1.16v
	regSIE_avPll.uctrlPLL_VCO_REF1P45_SEL = 0x3;
	regSIE_avPll.uctrlPLL_VDDL = 0xF;// set VDDL to 1.16v

	//* (volatile unsigned int *)(&(avPllBase->u32avPll_ctrlPLL)) = ctrlPLL.u32;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
}

static void avpll_deassertPllReset(SIE_avPll *avPllBase)
{
	volatile int i;

	for(i=0; i<10000*10; i++);

	//deassert reset
	//avPllBase->uctrlPLL_RESET=0;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_RESET = 0;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
}

//assert reset for channel1 to 7
static void avpll_deassertCxReset(SIE_avPll *avPllBase)
{
	int chId;

	for(chId=1; chId<=7; chId++) {
		//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_RESET = 0;
		PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl3, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl3);
		((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_RESET = 0;
		PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl3, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl3);
	}
}

static void avpll_calibrate(SIE_avPll *avPllBase, double fvco)
{
	volatile int i=100000;
	//MV_TimeSpec_t Time_Start, Time_End;

	//set the speed_thresh for current Fvco
	//avPllBase->uctrlCAL_SPEED_THRESH = (int)(fvco/100+0.5);
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlCAL, regSIE_avPll.u32avPll_ctrlCAL);
	regSIE_avPll.uctrlCAL_SPEED_THRESH = (int)(fvco/100+0.5);
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlCAL, regSIE_avPll.u32avPll_ctrlCAL);

	//assert PLL resebratet
	//avPllBase->uctrlPLL_RESET=1;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_RESET = 1;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);

	cpu_cycle_count_delay(20000);

	//MV_Time_GetSysTime(&Time_Start);
	//avPllBase->uctrlPLL_RESET=0;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_RESET = 0;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);

	//>20us and <50us must start calibration
	cpu_cycle_count_delay(35000);

	//avPllBase->uctrlCAL_PLL_CAL_START = 1;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlCAL, regSIE_avPll.u32avPll_ctrlCAL);
	regSIE_avPll.uctrlCAL_PLL_CAL_START = 1;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlCAL, regSIE_avPll.u32avPll_ctrlCAL);

	//poll PLL CAL done bit
	//while(!avPllBase->ustatus_PLL_CAL_DONE)
	while(1) {
		PHY_HOST_Bus_Read32(avPllBase->u32avPll_status, regSIE_avPll.u32avPll_status);
		if(regSIE_avPll.ustatus_PLL_CAL_DONE)
			break;

		if(--i<0) {
			break;
		}
	}

	cpu_cycle_count_delay(20000);

	//avPllBase->uctrlCAL_PLL_CAL_START = 0;
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlCAL, regSIE_avPll.u32avPll_ctrlCAL);
	regSIE_avPll.uctrlCAL_PLL_CAL_START = 0;
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlCAL, regSIE_avPll.u32avPll_ctrlCAL);
}

static void avpll_setVCO(SIE_avPll *avPllBase, int vco_freq_index)
{
	//first power done PLL and Channels
	avpll_powerDown(avPllBase);

	//assert resets
	avpll_assertPllReset(avPllBase);
	avpll_assertCxReset(avPllBase);

	//change VDDL and VCO_ref to meet duty cycle
	avpll_setVDDL_VCOref(avPllBase);

	//power up PLL and channels
	avpll_powerUp(avPllBase);

	// @yeliu: power down these channels by hardcoded, only for bg2cdp
	avpll_powerDownChannel(avPllBase, 5);
	avpll_powerDownChannel(avPllBase, 6);

	//following settings are done under reset to improve long term reliability
	//avPllBase->uctrlPLL_FBDIV = avpll_avpllRegFBDIV[vco_freq_index];
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);
	regSIE_avPll.uctrlPLL_FBDIV = avpll_avpllRegFBDIV[vco_freq_index];
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL, regSIE_avPll.u32avPll_ctrlPLL);

	//avPllBase->uctrlINTP_INTPI = avpll_avpllRegINTPI[vco_freq_index];
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlINTP, regSIE_avPll.u32avPll_ctrlINTP);
	regSIE_avPll.uctrlINTP_INTPI = avpll_avpllRegINTPI[vco_freq_index];
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlINTP, regSIE_avPll.u32avPll_ctrlINTP);

	//avPllBase->uctrlINTP_INTPR = avpll_avpllRegINTPR[vco_freq_index];
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlINTP, regSIE_avPll.u32avPll_ctrlINTP);
	regSIE_avPll.uctrlINTP_INTPR = avpll_avpllRegINTPR[vco_freq_index];
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlINTP, regSIE_avPll.u32avPll_ctrlINTP);

	//avPllBase->uctrlPLL_EXT_SPEED = avpll_avpllRegSPEED[vco_freq_index];
	PHY_HOST_Bus_Read32(avPllBase->u32avPll_ctrlPLL1, regSIE_avPll.u32avPll_ctrlPLL1);
	regSIE_avPll.uctrlPLL_EXT_SPEED = avpll_avpllRegSPEED[vco_freq_index];
	PHY_HOST_Bus_Write32(avPllBase->u32avPll_ctrlPLL1, regSIE_avPll.u32avPll_ctrlPLL1);

	//(avPllBase->ie_C8).uctrl_FREQ_OFFSET = avpll_avpllRegFREQ_OFFSET_C8[vco_freq_index];
	PHY_HOST_Bus_Read32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	(regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET = avpll_avpllRegFREQ_OFFSET_C8[vco_freq_index];
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);

	if(avPllBase == AVPLL_A)
		avpll_pll_A_VCO_Setting = vco_freq_index;
	else
		avpll_pll_B_VCO_Setting = vco_freq_index;

	//deassert resets
	avpll_deassertCxReset(avPllBase);

	avpll_deassertPllReset(avPllBase);

	//toggle the offset_rdy bit
	//(avPllBase->ie_C8).uctrl_FREQ_OFFSET_READY = 1;
	PHY_HOST_Bus_Read32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	(regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET_READY = 1;
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);

	//add some delay for Fvco=3GHz pulse>172ns, For Fvco=1.5GHz pulse>344ns
	cpu_cycle_count_delay(550);

	//(avPllBase->ie_C8).uctrl_FREQ_OFFSET_READY = 0;
	PHY_HOST_Bus_Read32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	(regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET_READY = 0;
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);

	//do calibration
	avpll_calibrate(avPllBase, avpll_vcoFreqs[vco_freq_index]);
}

static void avpll_setChanOffset(SIE_avPll *avPllBase, int offset, int chId)
{
	unsigned int reg_offset = 0;

	if(offset>0) {
		reg_offset = (1<<18) | (offset) ;
	} else {
		reg_offset = -offset;
	}

	//set offset register
	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_FREQ_OFFSET = reg_offset;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_FREQ_OFFSET = reg_offset;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);

	//toggle the offset_rdy bit
	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_FREQ_OFFSET_READY = 1;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_FREQ_OFFSET_READY = 1;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);

	//add some delay because need to be asserted by 30ns
	cpu_cycle_count_delay(200);

	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_FREQ_OFFSET_READY = 0;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_FREQ_OFFSET_READY = 0;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl1, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl1);
}

static void avpll_setDPll(SIE_avPll *avPllBase, int enable, int p_sync1, int p_sync2, int chId)
{
	//disable DPll first
	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_EN_DPLL = 0;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_EN_DPLL = 0;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);

	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_P_SYNC1 = p_sync1;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl2, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl2);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_P_SYNC1 = p_sync1;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl2, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl2);

	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_P_SYNC2 = p_sync2;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl3, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl3);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_P_SYNC2 = p_sync2;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl3, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl3);

	if(enable) {
		//enable DPLL
		//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_EN_DPLL = 1;
		PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
		((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_EN_DPLL = 1;
		PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
	}
}

static void avpll_set_Post_Div(SIE_avPll *avPllBase, int div, int chId)
{
	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_POSTDIV = div;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_POSTDIV = div;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
}

static void avpll_set_Post_0P5_Div(SIE_avPll *avPllBase, int enable, int chId)
{
	//((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].uctrl_POSTDIV_0P5 = enable;
	PHY_HOST_Bus_Read32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
	((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].uctrl_POSTDIV_0P5 = enable;
	PHY_HOST_Bus_Write32(((SIE_avpllCh *)(&(avPllBase->ie_C1)))[chId-1].u32avpllCh_ctrl, ((SIE_avpllCh *)&(regSIE_avPll.ie_C1))[chId-1].u32avpllCh_ctrl);
}

static int avpll_clockFreq_computed(SIE_avPll *avPllBase, int freqIndex, int chId)
{
	int avpll_pll_VCO_Setting;

	if(avPllBase == AVPLL_A) {
		avpll_pll_VCO_Setting=avpll_pll_A_VCO_Setting;
	} else {
		avpll_pll_VCO_Setting=avpll_pll_B_VCO_Setting;
	}

	if(freqIndex > (sizeof(clk_freqs_computed) / sizeof(CLK_FREQ))) {
		return 1;
	}

	if(clk_freqs_computed[freqIndex].invalid) {
		return 1;
	}

	if(avpll_pll_VCO_Setting != clk_freqs_computed[freqIndex].vco_freq_index) {
		avpll_setVCO(avPllBase, clk_freqs_computed[freqIndex].vco_freq_index);
	}

	//change offset
	avpll_setChanOffset(avPllBase, clk_freqs_computed[freqIndex].freq_offset, chId);

	//change p_sync
	avpll_setDPll(avPllBase, (clk_freqs_computed[freqIndex].p_sync1!=0),
			clk_freqs_computed[freqIndex].p_sync1,
			clk_freqs_computed[freqIndex].p_sync2, chId);

	//update now div
	avpll_set_Post_Div(avPllBase, clk_freqs_computed[freqIndex].post_div, chId);
	avpll_set_Post_0P5_Div(avPllBase, clk_freqs_computed[freqIndex].post_div_0p5, chId);

	return 0;
}

static int avpll_clockFreq(SIE_avPll *avPllBase, int vco_freq_index, unsigned int target_freq, int chId)
{
	if(avpll_compute_freq_setting(vco_freq_index, target_freq)) {
		return 1;
	} else {
		//frequency ok, set it
		avpll_clockFreq_computed(avPllBase, vco_freq_index, chId);
	}

	return 0;
}

int AVPLL_Set(int grpId, int chanId, unsigned int avFreq)
{
	if(grpId == 0)
		avpll_clockFreq(AVPLL_A, vcoFreqIndex, avFreq, chanId);

	return 0;
}

static void avpll_get_ppm(double *ppm_base, double *ppm_now)
{
	if(ppm_base) *ppm_base = org_ppm;
	if(ppm_now) *ppm_now = cur_ppm;
}

static double avpll_adjust_ppm(double ppm_delta)
{
	double ppm0, ppm;
	SIE_avPll *avPllBase;

	avPllBase = (SIE_avPll *)AVPLL_A;
	PHY_HOST_Bus_Read32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	if (avpll_inited == 0) {
		ppm0 = offset_2_ppm((regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET);
		cur_ppm = org_ppm = ppm0;
		avpll_inited = 1;
	}
	else {
		ppm0 = org_ppm;
	}
	cur_ppm += ppm_delta;
	ppm = cur_ppm;

	(regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET = ppm_2_offset(ppm, org_ppm);
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	//toggle the offset_rdy bit
	(regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET_READY = 1;
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	//add some delay for Fvco=3GHz pulse>172ns, For Fvco=1.5GHz pulse>344ns
	cpu_cycle_count_delay(1000);
	(regSIE_avPll.ie_C8).uctrl_FREQ_OFFSET_READY = 0;
	PHY_HOST_Bus_Write32((avPllBase->ie_C8).u32avpllCh8_ctrl1, (regSIE_avPll.ie_C8).u32avpllCh8_ctrl1);
	return ppm - ppm0;
}

double AVPLL_AdjustPPM(double ppm_delta)
{
	return avpll_adjust_ppm(ppm_delta);
}

void AVPLL_GetPPM(double *ppm_base, double *ppm_now)
{
	avpll_get_ppm(ppm_base, ppm_now);
}
