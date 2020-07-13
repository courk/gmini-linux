/*************************************************************************************
*       Copyright (C) 2007-2016
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
#include "ctypes.h"
#include "berlin_memmap.h"
#include "api_aio.h"
#include "avioDhub.h"
#include "aio.h"

#define GA_REG_WORD32_READ(addr, holder)    (*(holder) = (*((volatile unsigned int *)(addr))))
#define GA_REG_WORD32_WRITE(addr, data)     ((*((volatile unsigned int *)(addr))) = ((unsigned int)(data)))

#define PDM_ENABLE_ENABLE 1
#define PDM_ENABLE_DISABLE 0

#define I2S_MIC1_BASE (MEMMAP_I2S_REG_BASE + RA_AIO_MIC1)
#define I2S_MIC2_BASE (MEMMAP_I2S_REG_BASE + RA_AIO_MIC2)

#define I2S_MIC1_PRIAUD_CLKDIV (I2S_MIC1_BASE + RA_MIC_MICCTRL + RA_PRIAUD_CLKDIV)
#define I2S_MIC1_PRIAUD_CTRL (I2S_MIC1_BASE + RA_MIC_MICCTRL + RA_PRIAUD_CTRL)

#define I2S_MIC1_RXPORT (I2S_MIC1_BASE + RA_MIC_RXPORT)

#define I2S_MIC1_RSD0_CTRL (I2S_MIC1_BASE + RA_MIC_RSD0 + RA_AUDCH_CTRL)
#define I2S_MIC1_RSD0_DEBUGHI (I2S_MIC1_BASE + RA_MIC_RSD0 + RA_AUDCH_DEBUGHI)
#define I2S_MIC1_RSD0_DEBUGLO (I2S_MIC1_BASE + RA_MIC_RSD0 + RA_AUDCH_DEBUGLO)

#define I2S_MIC1_RSD1_CTRL (I2S_MIC1_BASE + RA_MIC_RSD1 + RA_AUDCH_CTRL)
#define I2S_MIC1_RSD1_DEBUGHI (I2S_MIC1_BASE + RA_MIC_RSD1 + RA_AUDCH_DEBUGHI)
#define I2S_MIC1_RSD1_DEBUGLO (I2S_MIC1_BASE + RA_MIC_RSD1 + RA_AUDCH_DEBUGLO)

#define I2S_PDM_MIC_SEL_CTRL (MEMMAP_I2S_REG_BASE + RA_AIO_PDM_MIC_SEL)
#define I2S_PDM_IOSEL_PDMCLK (MEMMAP_I2S_REG_BASE + RA_AIO_IOSEL + RA_IOSEL_PDMCLK)
#define I2S_PDM_IOSEL_PDM (MEMMAP_I2S_REG_BASE + RA_AIO_IOSEL + RA_IOSEL_PDM)
#define I2S_MIC1_PDM_CTRL1 (I2S_MIC1_BASE + RA_MIC_PDM_CTRL1)
#define I2S_MIC1_PDM_CTRL2 (I2S_MIC1_BASE + RA_MIC_PDM_CTRL2)

#define I2S_MIC2_PDM_CTRL1              (I2S_MIC2_BASE + RA_MIC_PDM_CTRL1)
#define I2S_MIC2_PDM_CTRL2              (I2S_MIC2_BASE + RA_MIC_PDM_CTRL2)

void Avio_SetHdclk()
{
    UNSG32 address;
    T32HDMI_HDPORT reg;
    address = MEMMAP_I2S_REG_BASE+RA_AIO_HDMI+RA_HDMI_HDPORT;
    GA_REG_WORD32_READ(address, &(reg.u32));
    reg.uHDPORT_TXSEL = 0; // HD clk is selected
    GA_REG_WORD32_WRITE(address, (reg.u32));
    return;
}
void PdmSetup(unsigned char host, unsigned char clk_src, unsigned char clk_div,
                unsigned char cycle_mode, unsigned char ch_bits, unsigned char latch_mode,  unsigned char mute)
{
    UNSG32 address;
    if (avioDhubChMap_ag_MIC0_W_A0 == host)  {
        address = I2S_MIC1_PDM_CTRL1;
    }
    else{
        address = I2S_MIC2_PDM_CTRL1;
    }

    T32MIC_PDM_CTRL1 reg;
    GA_REG_WORD32_READ(address, &(reg.u32));
    reg.uPDM_CTRL1_ENABLE=0;
    GA_REG_WORD32_WRITE(address, (reg.u32));

    GA_REG_WORD32_READ(address, &(reg.u32));
    reg.uPDM_CTRL1_MCLKSEL  = clk_src; //MCLK PRI
    reg.uPDM_CTRL1_CLKDIV   = clk_div;
    reg.uPDM_CTRL1_CLKEN    = 1;
    reg.uPDM_CTRL1_INVCLK_OUT   = 0;
    reg.uPDM_CTRL1_INVCLK_INT   = 1;
    reg.uPDM_CTRL1_RLSB     =  1; //LSB first
    reg.uPDM_CTRL1_LATCH_MODE   = latch_mode;
    reg.uPDM_CTRL1_MODE     = cycle_mode; //DDR mode
    reg.uPDM_CTRL1_RDM      = ch_bits; //32BIT
    reg.uPDM_CTRL1_MUTE     = mute;
    GA_REG_WORD32_WRITE(address, (reg.u32));

    /* set the PDM latch mode counter as 3 */
    T32MIC_PDM_CTRL2 reg2;
    reg2.uPDM_CTRL2_FDLT = 0x3;
    reg2.uPDM_CTRL2_RDLT = 0x3;

    address = I2S_MIC1_PDM_CTRL2;
    GA_REG_WORD32_WRITE(address, (reg2.u32));
}



void  PdmRxStart(int host, int chan)
{
    UNSG32 base, address;

   if (avioDhubChMap_ag_MIC0_W_A0 == host)  {
           base = I2S_MIC1_BASE;
    }
    else    {
            base = I2S_MIC2_BASE;
    }
    PdmSetEn(host, PDM_ENABLE_DISABLE);

    T32MIC_RXDATA reg0;
    address = base + RA_MIC_RXDATA;
    GA_REG_WORD32_READ(address, &(reg0.u32));
    reg0.uRXDATA_HBR = 0;
    GA_REG_WORD32_WRITE(address,(reg0.u32));

    T32MIC_HBRDMAP reg1;
    address = base + RA_MIC_HBRDMAP;
    GA_REG_WORD32_READ(address, &(reg1.u32));
    reg1.uHBRDMAP_PORT0 = chan;
    GA_REG_WORD32_WRITE(address,(reg1.u32));

    T32AIO_PDM_MIC_SEL reg2;
    address = I2S_PDM_MIC_SEL_CTRL;
    GA_REG_WORD32_READ(address, &(reg2.u32));
    reg2.uPDM_MIC_SEL_CTRL = 1; // 0-I2S Rx MIC mode, 1-PDM MIC mode
    GA_REG_WORD32_WRITE(address, (reg2.u32));

    T32IOSEL_PDMCLK reg3;
    address = I2S_PDM_IOSEL_PDMCLK;
    GA_REG_WORD32_READ(I2S_PDM_IOSEL_PDMCLK, &(reg3.u32));
    reg3.uPDMCLK_SEL = 1; // 0-no clk out, 1 - clk out
    GA_REG_WORD32_WRITE(address, (reg3.u32));

    T32IOSEL_PDM reg4;
    address = I2S_PDM_IOSEL_PDM;
    GA_REG_WORD32_READ(address, &(reg4.u32));
    reg4.uPDM_GENABLE = 1; // PDM GLOBAL Enable
    GA_REG_WORD32_WRITE(address, (reg4.u32));
    PdmSetEn(host, PDM_ENABLE_ENABLE);
    return;
}
void PdmSetEn(int host, int enable)
{
    UNSG32 address;
    if (avioDhubChMap_ag_MIC0_W_A0 == host)  {
        address = I2S_MIC1_PDM_CTRL1;
    }
    else{
        address = I2S_MIC2_PDM_CTRL1;
    }
    T32MIC_PDM_CTRL1 reg;
    GA_REG_WORD32_READ(address, &(reg.u32));
    reg.uPDM_CTRL1_ENABLE = enable;
    GA_REG_WORD32_WRITE(address, (reg.u32));
}

void PdmMuteEn(int host, int mute)
{
    UNSG32 address;
    T32MIC_PDM_CTRL1 reg;

    if (avioDhubChMap_ag_MIC0_W_A0 == host)  {
        address = I2S_MIC1_PDM_CTRL1;
    }
    else{
        address = I2S_MIC2_PDM_CTRL1;
    }
    GA_REG_WORD32_READ(address, &(reg.u32));
    reg.uPDM_CTRL1_MUTE = mute;
    GA_REG_WORD32_WRITE(address, (reg.u32));
    return;
}
