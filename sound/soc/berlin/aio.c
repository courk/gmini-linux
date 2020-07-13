/***********************************************************************************
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
*************************************************************************************/
#include "ctypes.h"
#include "berlin_memmap.h"
#include "api_aio.h"

#define GA_REG_WORD32_READ(addr, holder)    (*(holder) = (*((volatile unsigned int *)(addr))))
#define GA_REG_WORD32_WRITE(addr, data)     ((*((volatile unsigned int *)(addr))) = ((unsigned int)(data)))
#define bSETMASK(b)                         ((b) < 32 ? (1 << (b)) : 0)
#define CutTo(x, b)                         ((x) & (bSETMASK(b) - 1))

void AIO_SetAudChEn(UNSG32 id, UNSG32 tsd, UNSG32 enable)
{
	UNSG32 base, offset, address;
	T32AUDCH_CTRL reg;

	base = MEMMAP_I2S_REG_BASE;
	switch (id) {
	case AIO_SEC:
		offset = RA_AIO_SEC + RA_SEC_TSD;
		break;
	case AIO_MIC:
		offset = RA_AIO_MIC1 + RA_MIC_RSD0;
		break;
	default:
		offset = RA_AIO_SEC + RA_SEC_TSD;
		break;
        }

	address = base + offset + RA_AUDCH_CTRL;
	GA_REG_WORD32_READ(address, &(reg.u32));
	reg.uCTRL_ENABLE = CutTo(enable, bAUDCH_CTRL_ENABLE);
	GA_REG_WORD32_WRITE(address, (reg.u32));
}

void AIO_SetAudChMute(UNSG32 id, UNSG32 tsd, UNSG32 mute)
{
	UNSG32 base, offset, address;
	T32AUDCH_CTRL reg;

	base = MEMMAP_I2S_REG_BASE;
	switch (id) {
	case AIO_SEC:
		offset = RA_AIO_SEC + RA_SEC_TSD;
		break;
	case AIO_MIC:
                offset = RA_AIO_MIC1 + RA_MIC_RSD0;
                break;
	default:
		offset = RA_AIO_SEC + RA_SEC_TSD;
		break;
        }

	address = base + offset + RA_AUDCH_CTRL;
	GA_REG_WORD32_READ(address, &(reg.u32));
	reg.uCTRL_MUTE = CutTo( mute, bAUDCH_CTRL_MUTE);
	GA_REG_WORD32_WRITE(address, (reg.u32));
}

void AIO_SetAudChFlush(UNSG32 id, UNSG32 tsd, UNSG32 flush)
{
        UNSG32 base, offset, address;
        T32AUDCH_CTRL reg;

        base = MEMMAP_I2S_REG_BASE;
        switch (id) {
        case AIO_SEC:
                offset = RA_AIO_SEC + RA_SEC_TSD;
                break;
        case AIO_MIC:
                offset = RA_AIO_MIC1 + RA_MIC_RSD0;
                break;
        default:
                offset = RA_AIO_SEC + RA_SEC_TSD;
                break;
        }

        address = base + offset + RA_AUDCH_CTRL;
        GA_REG_WORD32_READ(address, &(reg.u32));
        reg.uCTRL_FLUSH = CutTo(flush, bAUDCH_CTRL_FLUSH);
        GA_REG_WORD32_WRITE(address, (reg.u32));
}

void AIO_SetRxPortEn(UNSG32 id, UNSG32 enable)
{
	UNSG32 base, offset, address;
        T32MIC_RXPORT reg;

	if (id != AIO_MIC)
		return;
	base = MEMMAP_I2S_REG_BASE;
	offset = RA_AIO_MIC1 + RA_MIC_RXPORT;
	address = base + offset;
        GA_REG_WORD32_READ(address, &(reg.u32));
        reg.uRXPORT_ENABLE = CutTo(enable, bMIC_RXPORT_ENABLE);
        GA_REG_WORD32_WRITE(address, (reg.u32));
}

void AIO_SetRxPortClkSel(UNSG32 id, UNSG32 sel)
{
        UNSG32 base, offset, address;
        T32Gbl_chipCntl reg;

        if (id != AIO_MIC)
		return;
        base = MEMMAP_CHIP_CTRL_REG_BASE;
        offset = RA_Gbl_chipCntl;
        address = base + offset;
        GA_REG_WORD32_READ(address, &(reg.u32));
        reg.uchipCntl_I2S2_CLK_SEL = CutTo(sel, bGbl_chipCntl_I2S2_CLK_SEL);
        GA_REG_WORD32_WRITE(address, (reg.u32));
}

void AIO_SetClkDiv(UNSG32 id, UNSG32 div)
{
	UNSG32 base, offset, address;
	T32PRIAUD_CLKDIV reg;

	base = MEMMAP_I2S_REG_BASE;
	switch (id) {
	case AIO_SEC:
		offset = RA_AIO_SEC + RA_SEC_SECAUD;
		break;
	case AIO_MIC:
		offset = RA_AIO_MIC1 + RA_MIC_MICCTRL;
                break;
	default:
		offset = RA_AIO_SEC + RA_SEC_SECAUD;
		break;
        }

	address = base + offset + RA_PRIAUD_CLKDIV;
	GA_REG_WORD32_READ(address, &(reg.u32));
	reg.uCLKDIV_SETTING = CutTo(div, bPRIAUD_CLKDIV_SETTING);
	GA_REG_WORD32_WRITE(address, (reg.u32));
}

void AIO_SetCtl(UNSG32 id, UNSG32 data_fmt, UNSG32 width_word, UNSG32 width_sample)
{
	UNSG32 base, offset, address;
	T32PRIAUD_CTRL reg;

	base = MEMMAP_I2S_REG_BASE;
	switch (id) {
	case AIO_SEC:
		offset = RA_AIO_SEC + RA_SEC_SECAUD;
		break;
	case AIO_MIC:
		offset = RA_AIO_MIC1 + RA_MIC_MICCTRL;
		break;
	default:
		offset = RA_AIO_SEC + RA_SEC_SECAUD;
		break;
	}

	address = base + offset + RA_PRIAUD_CTRL;
	GA_REG_WORD32_READ(address, &(reg.u32));

	// Set LEFTJFY
	reg.uCTRL_LEFTJFY = CutTo((data_fmt>>4), bPRIAUD_CTRL_LEFTJFY);

	// Set INVCLK
	if (id == AIO_MIC)
		reg.uCTRL_INVCLK = PRIAUD_CTRL_INVCLK_NORMAL;
	else
		reg.uCTRL_INVCLK = PRIAUD_CTRL_INVCLK_INVERTED;

	// set INVFS, I2S use Inverted mode(Low level means Left channel)
	if (id == AIO_MIC)
		reg.uCTRL_INVFS = PRIAUD_CTRL_INVFS_NORMAL;
	else if (data_fmt == AIO_I2S_MODE)
		reg.uCTRL_INVFS = PRIAUD_CTRL_INVFS_INVERTED;
	else
		reg.uCTRL_INVFS = PRIAUD_CTRL_INVFS_NORMAL;

	// TLSB, use default

	// set TDM,  using width_sample
	reg.uCTRL_TDM= CutTo(width_sample, bPRIAUD_CTRL_TDM);

	// set TCF using width_word;
	reg.uCTRL_TCF= CutTo(width_word, bPRIAUD_CTRL_TCF);

	// set TFM, using data_fmt
	reg.uCTRL_TFM = CutTo(data_fmt, bPRIAUD_CTRL_TFM);

	GA_REG_WORD32_WRITE(address, (reg.u32));
}

