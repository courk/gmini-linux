/************************************************************************************
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
*************************************************************************************/

#ifndef aio_h
#define aio_h (){}


#include "ctypes.h"

#pragma pack(1)
#ifdef __cplusplus
  extern "C" {
#endif

#ifndef _DOCC_H_BITOPS_
#define _DOCC_H_BITOPS_ (){}

    #define _bSETMASK_(b)                                      ((b)<32 ? (1<<((b)&31)) : 0)
    #define _NSETMASK_(msb,lsb)                                (_bSETMASK_((msb)+1)-_bSETMASK_(lsb))
    #define _bCLRMASK_(b)                                      (~_bSETMASK_(b))
    #define _NCLRMASK_(msb,lsb)                                (~_NSETMASK_(msb,lsb))
    #define _BFGET_(r,msb,lsb)                                 (_NSETMASK_((msb)-(lsb),0)&((r)>>(lsb)))
    #define _BFSET_(r,msb,lsb,v)                               do{ (r)&=_NCLRMASK_(msb,lsb); (r)|=_NSETMASK_(msb,lsb)&((v)<<(lsb)); }while(0)

#endif



//////
/// 
/// $INTERFACE PRIAUD                  biu              (4,4)
///     ###
///     * Audio Port configuration registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000 CLKDIV               (RW)
///               ###
///               * Audio Master Clock (MCLK) Divider register decides the ratio between MCLK and Audio Bit Clock (BCLK).
///               ###
///               %unsigned 3  SETTING                   0x2
///                                    ###
///                                    * MCLK Divider setting:
///                                    * 0 : Divide by 1
///                                    * 1: Divide by 2 (default)
///                                    * 2: Divide by 4
///                                    * 3 : Divide by 8
///                                    * 4: Divide by 16
///                                    * 5: Divide by 32
///                                    * 6: Divide by 64
///                                    * 7: Divide by 128
///                                    ###
///                                    : DIV1                      0x0
///                                    : DIV2                      0x1
///                                    : DIV4                      0x2
///                                    : DIV8                      0x3
///                                    : DIV16                     0x4
///                                    : DIV32                     0x5
///                                    : DIV64                     0x6
///                                    : DIV128                    0x7
///               %%        29         # Stuffing bits...
///     @ 0x00004 CTRL                 (RW)
///               ###
///               * Register to Control the Output data format for a audio Port
///               ###
///               %unsigned 1  LEFTJFY                   0x0
///                                    ###
///                                    * Decides the Audio Justify mode:
///                                    * 0 : Left Justify (default).
///                                    * 1 : Right Justify.
///                                    ###
///                                    : LEFT                      0x0
///                                    : RIGHT                     0x1
///               %unsigned 1  INVCLK                    0x0
///                                    ###
///                                    * Invert Bit clock (BCLK):
///                                    * 0 - Normal, data is send/received with Rising edge of bit-Clock (default).
///                                    * Note: For TX: Use this setting if DAC is sampling data at negative edge of bit clock.
///                                    * Note: For RX: Use this setting if ADC is sending data at negative edge of bit clock.
///                                    * 1 - Inverted, data is send/received with Falling edge of bit-Clock.
///                                    * Note: For TX: Use this setting if DAC is sampling data at positive edge of bit clock.
///                                    * Note: For RX: Use this setting if ADC is sending data at positive edge of bit clock.
///                                    ###
///                                    : NORMAL                    0x0
///                                    : INVERTED                  0x1
///               %unsigned 1  INVFS                     0x1
///                                    ###
///                                    * Invert Frame Sync (LR Clock):
///                                    * 0: Normal, FSYNC is Low during reset and goes High when first channel (Left) data is transmitted and keeps toggling after that.
///                                    * 1: Inverted, FSYNC is High during reset and goes Low when first channel (Left) data is transmitted and keeps toggling after that. (default)
///                                    * Note: Not used for RX port (MIC).
///                                    ###
///                                    : NORMAL                    0x0
///                                    : INVERTED                  0x1
///               %unsigned 1  TLSB                      0x0
///                                    ###
///                                    * Decides which bit is transmitted/received first(left):
///                                    * 0: MSB first
///                                    * 1: LSB first
///                                    ###
///                                    : MSB_FIRST                 0x0
///                                    : LSB_FIRST                 0x1
///               %unsigned 3  TDM                       0x0
///                                    ###
///                                    * Decides the channel resolution (number of valid bits in a half period of FSYNC):
///                                    * 0: 16-bits per channel
///                                    * 1: 18-bits per channel
///                                    * 2: 20-bits per channel
///                                    * 3: 24-bits per channel
///                                    * 4: 32-bits per channel
///                                    * 5-7: Reserved
///                                    * Note: HD port only support 16-bits per channel
///                                    ###
///                                    : 16DFM                     0x0
///                                    : 18DFM                     0x1
///                                    : 20DFM                     0x2
///                                    : 24DFM                     0x3
///                                    : 32DFM                     0x4
///               %unsigned 2  TCF                       0x2
///                                    ###
///                                    * Decides the half period of FSYNC (sampling rate) in terms of number of bit-clocks:
///                                    * 0: FSYNC half period equals to 16 bit-clocks
///                                    * 1: FSYNC half period equals to 24 bit-clocks
///                                    * 2: FSYNC half period equals to 32 bit-clocks
///                                    * 3: Reserved
///                                    ###
///                                    : 16CFM                     0x0
///                                    : 24CFM                     0x1
///                                    : 32CFM                     0x2
///               %unsigned 2  TFM                       0x2
///                                    ###
///                                    * Decides the TX/RX data format:
///                                    * 1: Justified
///                                    * 2: I2S (default)
///                                    * 0 and 3: Reserved
///                                    ###
///                                    : JUSTIFIED                 0x1
///                                    : I2S                       0x2
///               %%        21         # Stuffing bits...
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:       8B, bits:      14b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_PRIAUD
#define h_PRIAUD (){}

    #define     RA_PRIAUD_CLKDIV                               0x0000

    #define     BA_PRIAUD_CLKDIV_SETTING                       0x0000
    #define     B16PRIAUD_CLKDIV_SETTING                       0x0000
    #define   LSb32PRIAUD_CLKDIV_SETTING                          0
    #define   LSb16PRIAUD_CLKDIV_SETTING                          0
    #define       bPRIAUD_CLKDIV_SETTING                       3
    #define   MSK32PRIAUD_CLKDIV_SETTING                          0x00000007
    #define        PRIAUD_CLKDIV_SETTING_DIV1                               0x0
    #define        PRIAUD_CLKDIV_SETTING_DIV2                               0x1
    #define        PRIAUD_CLKDIV_SETTING_DIV4                               0x2
    #define        PRIAUD_CLKDIV_SETTING_DIV8                               0x3
    #define        PRIAUD_CLKDIV_SETTING_DIV16                              0x4
    #define        PRIAUD_CLKDIV_SETTING_DIV32                              0x5
    #define        PRIAUD_CLKDIV_SETTING_DIV64                              0x6
    #define        PRIAUD_CLKDIV_SETTING_DIV128                             0x7
    ///////////////////////////////////////////////////////////
    #define     RA_PRIAUD_CTRL                                 0x0004

    #define     BA_PRIAUD_CTRL_LEFTJFY                         0x0004
    #define     B16PRIAUD_CTRL_LEFTJFY                         0x0004
    #define   LSb32PRIAUD_CTRL_LEFTJFY                            0
    #define   LSb16PRIAUD_CTRL_LEFTJFY                            0
    #define       bPRIAUD_CTRL_LEFTJFY                         1
    #define   MSK32PRIAUD_CTRL_LEFTJFY                            0x00000001
    #define        PRIAUD_CTRL_LEFTJFY_LEFT                                 0x0
    #define        PRIAUD_CTRL_LEFTJFY_RIGHT                                0x1

    #define     BA_PRIAUD_CTRL_INVCLK                          0x0004
    #define     B16PRIAUD_CTRL_INVCLK                          0x0004
    #define   LSb32PRIAUD_CTRL_INVCLK                             1
    #define   LSb16PRIAUD_CTRL_INVCLK                             1
    #define       bPRIAUD_CTRL_INVCLK                          1
    #define   MSK32PRIAUD_CTRL_INVCLK                             0x00000002
    #define        PRIAUD_CTRL_INVCLK_NORMAL                                0x0
    #define        PRIAUD_CTRL_INVCLK_INVERTED                              0x1

    #define     BA_PRIAUD_CTRL_INVFS                           0x0004
    #define     B16PRIAUD_CTRL_INVFS                           0x0004
    #define   LSb32PRIAUD_CTRL_INVFS                              2
    #define   LSb16PRIAUD_CTRL_INVFS                              2
    #define       bPRIAUD_CTRL_INVFS                           1
    #define   MSK32PRIAUD_CTRL_INVFS                              0x00000004
    #define        PRIAUD_CTRL_INVFS_NORMAL                                 0x0
    #define        PRIAUD_CTRL_INVFS_INVERTED                               0x1

    #define     BA_PRIAUD_CTRL_TLSB                            0x0004
    #define     B16PRIAUD_CTRL_TLSB                            0x0004
    #define   LSb32PRIAUD_CTRL_TLSB                               3
    #define   LSb16PRIAUD_CTRL_TLSB                               3
    #define       bPRIAUD_CTRL_TLSB                            1
    #define   MSK32PRIAUD_CTRL_TLSB                               0x00000008
    #define        PRIAUD_CTRL_TLSB_MSB_FIRST                               0x0
    #define        PRIAUD_CTRL_TLSB_LSB_FIRST                               0x1

    #define     BA_PRIAUD_CTRL_TDM                             0x0004
    #define     B16PRIAUD_CTRL_TDM                             0x0004
    #define   LSb32PRIAUD_CTRL_TDM                                4
    #define   LSb16PRIAUD_CTRL_TDM                                4
    #define       bPRIAUD_CTRL_TDM                             3
    #define   MSK32PRIAUD_CTRL_TDM                                0x00000070
    #define        PRIAUD_CTRL_TDM_16DFM                                    0x0
    #define        PRIAUD_CTRL_TDM_18DFM                                    0x1
    #define        PRIAUD_CTRL_TDM_20DFM                                    0x2
    #define        PRIAUD_CTRL_TDM_24DFM                                    0x3
    #define        PRIAUD_CTRL_TDM_32DFM                                    0x4

    #define     BA_PRIAUD_CTRL_TCF                             0x0004
    #define     B16PRIAUD_CTRL_TCF                             0x0004
    #define   LSb32PRIAUD_CTRL_TCF                                7
    #define   LSb16PRIAUD_CTRL_TCF                                7
    #define       bPRIAUD_CTRL_TCF                             2
    #define   MSK32PRIAUD_CTRL_TCF                                0x00000180
    #define        PRIAUD_CTRL_TCF_16CFM                                    0x0
    #define        PRIAUD_CTRL_TCF_24CFM                                    0x1
    #define        PRIAUD_CTRL_TCF_32CFM                                    0x2

    #define     BA_PRIAUD_CTRL_TFM                             0x0005
    #define     B16PRIAUD_CTRL_TFM                             0x0004
    #define   LSb32PRIAUD_CTRL_TFM                                9
    #define   LSb16PRIAUD_CTRL_TFM                                9
    #define       bPRIAUD_CTRL_TFM                             2
    #define   MSK32PRIAUD_CTRL_TFM                                0x00000600
    #define        PRIAUD_CTRL_TFM_JUSTIFIED                                0x1
    #define        PRIAUD_CTRL_TFM_I2S                                      0x2
    ///////////////////////////////////////////////////////////

    typedef struct SIE_PRIAUD {
    ///////////////////////////////////////////////////////////
    #define   GET32PRIAUD_CLKDIV_SETTING(r32)                  _BFGET_(r32, 2, 0)
    #define   SET32PRIAUD_CLKDIV_SETTING(r32,v)                _BFSET_(r32, 2, 0,v)
    #define   GET16PRIAUD_CLKDIV_SETTING(r16)                  _BFGET_(r16, 2, 0)
    #define   SET16PRIAUD_CLKDIV_SETTING(r16,v)                _BFSET_(r16, 2, 0,v)

    #define     w32PRIAUD_CLKDIV                               {\
            UNSG32 uCLKDIV_SETTING                             :  3;\
            UNSG32 RSVDx0_b3                                   : 29;\
          }
    union { UNSG32 u32PRIAUD_CLKDIV;
            struct w32PRIAUD_CLKDIV;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32PRIAUD_CTRL_LEFTJFY(r32)                    _BFGET_(r32, 0, 0)
    #define   SET32PRIAUD_CTRL_LEFTJFY(r32,v)                  _BFSET_(r32, 0, 0,v)
    #define   GET16PRIAUD_CTRL_LEFTJFY(r16)                    _BFGET_(r16, 0, 0)
    #define   SET16PRIAUD_CTRL_LEFTJFY(r16,v)                  _BFSET_(r16, 0, 0,v)

    #define   GET32PRIAUD_CTRL_INVCLK(r32)                     _BFGET_(r32, 1, 1)
    #define   SET32PRIAUD_CTRL_INVCLK(r32,v)                   _BFSET_(r32, 1, 1,v)
    #define   GET16PRIAUD_CTRL_INVCLK(r16)                     _BFGET_(r16, 1, 1)
    #define   SET16PRIAUD_CTRL_INVCLK(r16,v)                   _BFSET_(r16, 1, 1,v)

    #define   GET32PRIAUD_CTRL_INVFS(r32)                      _BFGET_(r32, 2, 2)
    #define   SET32PRIAUD_CTRL_INVFS(r32,v)                    _BFSET_(r32, 2, 2,v)
    #define   GET16PRIAUD_CTRL_INVFS(r16)                      _BFGET_(r16, 2, 2)
    #define   SET16PRIAUD_CTRL_INVFS(r16,v)                    _BFSET_(r16, 2, 2,v)

    #define   GET32PRIAUD_CTRL_TLSB(r32)                       _BFGET_(r32, 3, 3)
    #define   SET32PRIAUD_CTRL_TLSB(r32,v)                     _BFSET_(r32, 3, 3,v)
    #define   GET16PRIAUD_CTRL_TLSB(r16)                       _BFGET_(r16, 3, 3)
    #define   SET16PRIAUD_CTRL_TLSB(r16,v)                     _BFSET_(r16, 3, 3,v)

    #define   GET32PRIAUD_CTRL_TDM(r32)                        _BFGET_(r32, 6, 4)
    #define   SET32PRIAUD_CTRL_TDM(r32,v)                      _BFSET_(r32, 6, 4,v)
    #define   GET16PRIAUD_CTRL_TDM(r16)                        _BFGET_(r16, 6, 4)
    #define   SET16PRIAUD_CTRL_TDM(r16,v)                      _BFSET_(r16, 6, 4,v)

    #define   GET32PRIAUD_CTRL_TCF(r32)                        _BFGET_(r32, 8, 7)
    #define   SET32PRIAUD_CTRL_TCF(r32,v)                      _BFSET_(r32, 8, 7,v)
    #define   GET16PRIAUD_CTRL_TCF(r16)                        _BFGET_(r16, 8, 7)
    #define   SET16PRIAUD_CTRL_TCF(r16,v)                      _BFSET_(r16, 8, 7,v)

    #define   GET32PRIAUD_CTRL_TFM(r32)                        _BFGET_(r32,10, 9)
    #define   SET32PRIAUD_CTRL_TFM(r32,v)                      _BFSET_(r32,10, 9,v)
    #define   GET16PRIAUD_CTRL_TFM(r16)                        _BFGET_(r16,10, 9)
    #define   SET16PRIAUD_CTRL_TFM(r16,v)                      _BFSET_(r16,10, 9,v)

    #define     w32PRIAUD_CTRL                                 {\
            UNSG32 uCTRL_LEFTJFY                               :  1;\
            UNSG32 uCTRL_INVCLK                                :  1;\
            UNSG32 uCTRL_INVFS                                 :  1;\
            UNSG32 uCTRL_TLSB                                  :  1;\
            UNSG32 uCTRL_TDM                                   :  3;\
            UNSG32 uCTRL_TCF                                   :  2;\
            UNSG32 uCTRL_TFM                                   :  2;\
            UNSG32 RSVDx4_b11                                  : 21;\
          }
    union { UNSG32 u32PRIAUD_CTRL;
            struct w32PRIAUD_CTRL;
          };
    ///////////////////////////////////////////////////////////
    } SIE_PRIAUD;

    typedef union  T32PRIAUD_CLKDIV
          { UNSG32 u32;
            struct w32PRIAUD_CLKDIV;
                 } T32PRIAUD_CLKDIV;
    typedef union  T32PRIAUD_CTRL
          { UNSG32 u32;
            struct w32PRIAUD_CTRL;
                 } T32PRIAUD_CTRL;
    ///////////////////////////////////////////////////////////

    typedef union  TPRIAUD_CLKDIV
          { UNSG32 u32[1];
            struct {
            struct w32PRIAUD_CLKDIV;
                   };
                 } TPRIAUD_CLKDIV;
    typedef union  TPRIAUD_CTRL
          { UNSG32 u32[1];
            struct {
            struct w32PRIAUD_CTRL;
                   };
                 } TPRIAUD_CTRL;

    ///////////////////////////////////////////////////////////
     SIGN32 PRIAUD_drvrd(SIE_PRIAUD *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 PRIAUD_drvwr(SIE_PRIAUD *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void PRIAUD_reset(SIE_PRIAUD *p);
     SIGN32 PRIAUD_cmp  (SIE_PRIAUD *p, SIE_PRIAUD *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define PRIAUD_check(p,pie,pfx,hLOG) PRIAUD_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define PRIAUD_print(p,    pfx,hLOG) PRIAUD_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: PRIAUD
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE AUDCH                   biu              (4,4)
///     ###
///     * Audio Port Control registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000 CTRL                 (RW)
///               ###
///               * Audio Port Control register
///               ###
///               %unsigned 1  ENABLE                    0x0
///                                    ###
///                                    * Control bit to enable/disable an audio channel:
///                                    * 0: Disable a audio Channel (default)
///                                    * 1: Enable a audio Channel
///                                    * Note: Bit-clock (BCLK) is not controlled using this bit. For switching off the Bit Clock the output enable register should be used(IOSEL).
///                                    ###
///                                    : DISABLE                   0x0
///                                    : ENABLE                    0x1
///               %unsigned 1  MUTE                      0x0
///                                    ###
///                                    * Control bit to turn mute function ON or OFF
///                                    * 0: Mute turned OFF (default)
///                                    * 1: Mute turned ON
///                                    ###
///                                    : MUTE_OFF                  0x0
///                                    : MUTE_ON                   0x1
///               %unsigned 1  LRSWITCH                  0x0
///                                    ###
///                                    * Control bit for switching the left channel data with right channel data:
///                                    * 0: Do not switch data (default)
///                                    * 1: Switch Data
///                                    * Note: Not supported for S/PDIF.
///                                    ###
///                                    : SWITCH_OFF                0x0
///                                    : SWITCH_ON                 0x1
///               %unsigned 1  DEBUGEN                   0x0
///                                    ###
///                                    * Control bit to enable/disable Debug mode:
///                                    * 0: Disable (default)
///                                    * 1: Enable
///                                    * Note: In debug mode transmit data is from debug registers.
///                                    * *INTERNAL_ONLY**
///                                    ###
///                                    : DISABLE                   0x0
///                                    : ENABLE                    0x1
///               %unsigned 1  FLUSH                     0x0
///                                    ###
///                                    * Control bit to clear the Data FIFO pointers related to a port:
///                                    * 0: Do not flush (default)
///                                    * 1: Flush
///                                    ###
///                                    : ON                        0x1
///                                    : OFF                       0x0
///               %%        27         # Stuffing bits...
///     @ 0x00004 DEBUGHI              (RW)
///               ###
///               * Upper 32-bit Debug data
///               ###
///               %unsigned 32 DATAHI                    0x4884103F
///                                    ###
///                                    * Upper 32-bit Debug data
///                                    * *INTERNAL_ONLY**
///                                    ###
///     @ 0x00008 DEBUGLO              (RW)
///               ###
///               * Lower 32-bit Debug data
///               ###
///               %unsigned 32 DATALO                    0xB77BEFC0
///                                    ###
///                                    * Lower 32-bit Debug data
///                                    * *INTERNAL_ONLY**
///                                    ###
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      12B, bits:      69b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_AUDCH
#define h_AUDCH (){}

    #define     RA_AUDCH_CTRL                                  0x0000

    #define     BA_AUDCH_CTRL_ENABLE                           0x0000
    #define     B16AUDCH_CTRL_ENABLE                           0x0000
    #define   LSb32AUDCH_CTRL_ENABLE                              0
    #define   LSb16AUDCH_CTRL_ENABLE                              0
    #define       bAUDCH_CTRL_ENABLE                           1
    #define   MSK32AUDCH_CTRL_ENABLE                              0x00000001
    #define        AUDCH_CTRL_ENABLE_DISABLE                                0x0
    #define        AUDCH_CTRL_ENABLE_ENABLE                                 0x1

    #define     BA_AUDCH_CTRL_MUTE                             0x0000
    #define     B16AUDCH_CTRL_MUTE                             0x0000
    #define   LSb32AUDCH_CTRL_MUTE                                1
    #define   LSb16AUDCH_CTRL_MUTE                                1
    #define       bAUDCH_CTRL_MUTE                             1
    #define   MSK32AUDCH_CTRL_MUTE                                0x00000002
    #define        AUDCH_CTRL_MUTE_MUTE_OFF                                 0x0
    #define        AUDCH_CTRL_MUTE_MUTE_ON                                  0x1

    #define     BA_AUDCH_CTRL_LRSWITCH                         0x0000
    #define     B16AUDCH_CTRL_LRSWITCH                         0x0000
    #define   LSb32AUDCH_CTRL_LRSWITCH                            2
    #define   LSb16AUDCH_CTRL_LRSWITCH                            2
    #define       bAUDCH_CTRL_LRSWITCH                         1
    #define   MSK32AUDCH_CTRL_LRSWITCH                            0x00000004
    #define        AUDCH_CTRL_LRSWITCH_SWITCH_OFF                           0x0
    #define        AUDCH_CTRL_LRSWITCH_SWITCH_ON                            0x1

    #define     BA_AUDCH_CTRL_DEBUGEN                          0x0000
    #define     B16AUDCH_CTRL_DEBUGEN                          0x0000
    #define   LSb32AUDCH_CTRL_DEBUGEN                             3
    #define   LSb16AUDCH_CTRL_DEBUGEN                             3
    #define       bAUDCH_CTRL_DEBUGEN                          1
    #define   MSK32AUDCH_CTRL_DEBUGEN                             0x00000008
    #define        AUDCH_CTRL_DEBUGEN_DISABLE                               0x0
    #define        AUDCH_CTRL_DEBUGEN_ENABLE                                0x1

    #define     BA_AUDCH_CTRL_FLUSH                            0x0000
    #define     B16AUDCH_CTRL_FLUSH                            0x0000
    #define   LSb32AUDCH_CTRL_FLUSH                               4
    #define   LSb16AUDCH_CTRL_FLUSH                               4
    #define       bAUDCH_CTRL_FLUSH                            1
    #define   MSK32AUDCH_CTRL_FLUSH                               0x00000010
    #define        AUDCH_CTRL_FLUSH_ON                                      0x1
    #define        AUDCH_CTRL_FLUSH_OFF                                     0x0
    ///////////////////////////////////////////////////////////
    #define     RA_AUDCH_DEBUGHI                               0x0004

    #define     BA_AUDCH_DEBUGHI_DATAHI                        0x0004
    #define     B16AUDCH_DEBUGHI_DATAHI                        0x0004
    #define   LSb32AUDCH_DEBUGHI_DATAHI                           0
    #define   LSb16AUDCH_DEBUGHI_DATAHI                           0
    #define       bAUDCH_DEBUGHI_DATAHI                        32
    #define   MSK32AUDCH_DEBUGHI_DATAHI                           0xFFFFFFFF
    ///////////////////////////////////////////////////////////
    #define     RA_AUDCH_DEBUGLO                               0x0008

    #define     BA_AUDCH_DEBUGLO_DATALO                        0x0008
    #define     B16AUDCH_DEBUGLO_DATALO                        0x0008
    #define   LSb32AUDCH_DEBUGLO_DATALO                           0
    #define   LSb16AUDCH_DEBUGLO_DATALO                           0
    #define       bAUDCH_DEBUGLO_DATALO                        32
    #define   MSK32AUDCH_DEBUGLO_DATALO                           0xFFFFFFFF
    ///////////////////////////////////////////////////////////

    typedef struct SIE_AUDCH {
    ///////////////////////////////////////////////////////////
    #define   GET32AUDCH_CTRL_ENABLE(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32AUDCH_CTRL_ENABLE(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16AUDCH_CTRL_ENABLE(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16AUDCH_CTRL_ENABLE(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define   GET32AUDCH_CTRL_MUTE(r32)                        _BFGET_(r32, 1, 1)
    #define   SET32AUDCH_CTRL_MUTE(r32,v)                      _BFSET_(r32, 1, 1,v)
    #define   GET16AUDCH_CTRL_MUTE(r16)                        _BFGET_(r16, 1, 1)
    #define   SET16AUDCH_CTRL_MUTE(r16,v)                      _BFSET_(r16, 1, 1,v)

    #define   GET32AUDCH_CTRL_LRSWITCH(r32)                    _BFGET_(r32, 2, 2)
    #define   SET32AUDCH_CTRL_LRSWITCH(r32,v)                  _BFSET_(r32, 2, 2,v)
    #define   GET16AUDCH_CTRL_LRSWITCH(r16)                    _BFGET_(r16, 2, 2)
    #define   SET16AUDCH_CTRL_LRSWITCH(r16,v)                  _BFSET_(r16, 2, 2,v)

    #define   GET32AUDCH_CTRL_DEBUGEN(r32)                     _BFGET_(r32, 3, 3)
    #define   SET32AUDCH_CTRL_DEBUGEN(r32,v)                   _BFSET_(r32, 3, 3,v)
    #define   GET16AUDCH_CTRL_DEBUGEN(r16)                     _BFGET_(r16, 3, 3)
    #define   SET16AUDCH_CTRL_DEBUGEN(r16,v)                   _BFSET_(r16, 3, 3,v)

    #define   GET32AUDCH_CTRL_FLUSH(r32)                       _BFGET_(r32, 4, 4)
    #define   SET32AUDCH_CTRL_FLUSH(r32,v)                     _BFSET_(r32, 4, 4,v)
    #define   GET16AUDCH_CTRL_FLUSH(r16)                       _BFGET_(r16, 4, 4)
    #define   SET16AUDCH_CTRL_FLUSH(r16,v)                     _BFSET_(r16, 4, 4,v)

    #define     w32AUDCH_CTRL                                  {\
            UNSG32 uCTRL_ENABLE                                :  1;\
            UNSG32 uCTRL_MUTE                                  :  1;\
            UNSG32 uCTRL_LRSWITCH                              :  1;\
            UNSG32 uCTRL_DEBUGEN                               :  1;\
            UNSG32 uCTRL_FLUSH                                 :  1;\
            UNSG32 RSVDx0_b5                                   : 27;\
          }
    union { UNSG32 u32AUDCH_CTRL;
            struct w32AUDCH_CTRL;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32AUDCH_DEBUGHI_DATAHI(r32)                   _BFGET_(r32,31, 0)
    #define   SET32AUDCH_DEBUGHI_DATAHI(r32,v)                 _BFSET_(r32,31, 0,v)

    #define     w32AUDCH_DEBUGHI                               {\
            UNSG32 uDEBUGHI_DATAHI                             : 32;\
          }
    union { UNSG32 u32AUDCH_DEBUGHI;
            struct w32AUDCH_DEBUGHI;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32AUDCH_DEBUGLO_DATALO(r32)                   _BFGET_(r32,31, 0)
    #define   SET32AUDCH_DEBUGLO_DATALO(r32,v)                 _BFSET_(r32,31, 0,v)

    #define     w32AUDCH_DEBUGLO                               {\
            UNSG32 uDEBUGLO_DATALO                             : 32;\
          }
    union { UNSG32 u32AUDCH_DEBUGLO;
            struct w32AUDCH_DEBUGLO;
          };
    ///////////////////////////////////////////////////////////
    } SIE_AUDCH;

    typedef union  T32AUDCH_CTRL
          { UNSG32 u32;
            struct w32AUDCH_CTRL;
                 } T32AUDCH_CTRL;
    typedef union  T32AUDCH_DEBUGHI
          { UNSG32 u32;
            struct w32AUDCH_DEBUGHI;
                 } T32AUDCH_DEBUGHI;
    typedef union  T32AUDCH_DEBUGLO
          { UNSG32 u32;
            struct w32AUDCH_DEBUGLO;
                 } T32AUDCH_DEBUGLO;
    ///////////////////////////////////////////////////////////

    typedef union  TAUDCH_CTRL
          { UNSG32 u32[1];
            struct {
            struct w32AUDCH_CTRL;
                   };
                 } TAUDCH_CTRL;
    typedef union  TAUDCH_DEBUGHI
          { UNSG32 u32[1];
            struct {
            struct w32AUDCH_DEBUGHI;
                   };
                 } TAUDCH_DEBUGHI;
    typedef union  TAUDCH_DEBUGLO
          { UNSG32 u32[1];
            struct {
            struct w32AUDCH_DEBUGLO;
                   };
                 } TAUDCH_DEBUGLO;

    ///////////////////////////////////////////////////////////
     SIGN32 AUDCH_drvrd(SIE_AUDCH *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 AUDCH_drvwr(SIE_AUDCH *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void AUDCH_reset(SIE_AUDCH *p);
     SIGN32 AUDCH_cmp  (SIE_AUDCH *p, SIE_AUDCH *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define AUDCH_check(p,pie,pfx,hLOG) AUDCH_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define AUDCH_print(p,    pfx,hLOG) AUDCH_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: AUDCH
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE SPDIF                   biu              (4,4)
///     ###
///     * S/PDIF Control registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000 CLKDIV               (RW)
///               ###
///               * S/PDIF transmitter bit clock selection register to decide the output sampling rate
///               ###
///               %unsigned 3  SETTING                   0x2
///                                    ###
///                                    * MCLK Divider setting:
///                                    * 0 : Divide by 1
///                                    * 1: Divide by 2
///                                    * 2: Divide by 4 ( default)
///                                    * 3 : Divide by 8
///                                    * 4: Divide by 16
///                                    * 5: Divide by 32
///                                    * 6: Divide by 64
///                                    * 7: Divide by 128
///                                    ###
///                                    : DIV1                      0x0
///                                    : DIV2                      0x1
///                                    : DIV4                      0x2
///                                    : DIV8                      0x3
///                                    : DIV16                     0x4
///                                    : DIV32                     0x5
///                                    : DIV64                     0x6
///                                    : DIV128                    0x7
///               %%        29         # Stuffing bits...
///     @ 0x00004                      (P)
///     # 0x00004 SPDIF                
///               $AUDCH               SPDIF             REG          
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      16B, bits:      72b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_SPDIF
#define h_SPDIF (){}

    #define     RA_SPDIF_CLKDIV                                0x0000

    #define     BA_SPDIF_CLKDIV_SETTING                        0x0000
    #define     B16SPDIF_CLKDIV_SETTING                        0x0000
    #define   LSb32SPDIF_CLKDIV_SETTING                           0
    #define   LSb16SPDIF_CLKDIV_SETTING                           0
    #define       bSPDIF_CLKDIV_SETTING                        3
    #define   MSK32SPDIF_CLKDIV_SETTING                           0x00000007
    #define        SPDIF_CLKDIV_SETTING_DIV1                                0x0
    #define        SPDIF_CLKDIV_SETTING_DIV2                                0x1
    #define        SPDIF_CLKDIV_SETTING_DIV4                                0x2
    #define        SPDIF_CLKDIV_SETTING_DIV8                                0x3
    #define        SPDIF_CLKDIV_SETTING_DIV16                               0x4
    #define        SPDIF_CLKDIV_SETTING_DIV32                               0x5
    #define        SPDIF_CLKDIV_SETTING_DIV64                               0x6
    #define        SPDIF_CLKDIV_SETTING_DIV128                              0x7
    ///////////////////////////////////////////////////////////
    #define     RA_SPDIF_SPDIF                                 0x0004
    ///////////////////////////////////////////////////////////

    typedef struct SIE_SPDIF {
    ///////////////////////////////////////////////////////////
    #define   GET32SPDIF_CLKDIV_SETTING(r32)                   _BFGET_(r32, 2, 0)
    #define   SET32SPDIF_CLKDIV_SETTING(r32,v)                 _BFSET_(r32, 2, 0,v)
    #define   GET16SPDIF_CLKDIV_SETTING(r16)                   _BFGET_(r16, 2, 0)
    #define   SET16SPDIF_CLKDIV_SETTING(r16,v)                 _BFSET_(r16, 2, 0,v)

    #define     w32SPDIF_CLKDIV                                {\
            UNSG32 uCLKDIV_SETTING                             :  3;\
            UNSG32 RSVDx0_b3                                   : 29;\
          }
    union { UNSG32 u32SPDIF_CLKDIV;
            struct w32SPDIF_CLKDIV;
          };
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_SPDIF;
    ///////////////////////////////////////////////////////////
    } SIE_SPDIF;

    typedef union  T32SPDIF_CLKDIV
          { UNSG32 u32;
            struct w32SPDIF_CLKDIV;
                 } T32SPDIF_CLKDIV;
    ///////////////////////////////////////////////////////////

    typedef union  TSPDIF_CLKDIV
          { UNSG32 u32[1];
            struct {
            struct w32SPDIF_CLKDIV;
                   };
                 } TSPDIF_CLKDIV;

    ///////////////////////////////////////////////////////////
     SIGN32 SPDIF_drvrd(SIE_SPDIF *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 SPDIF_drvwr(SIE_SPDIF *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void SPDIF_reset(SIE_SPDIF *p);
     SIGN32 SPDIF_cmp  (SIE_SPDIF *p, SIE_SPDIF *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define SPDIF_check(p,pie,pfx,hLOG) SPDIF_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define SPDIF_print(p,    pfx,hLOG) SPDIF_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: SPDIF
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE P71                                      (4,4)
///     ###
///     * Audio 7.1 Primary port Control registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000                      (P)
///     # 0x00000 PRIAUD               
///               $PRIAUD              PRIAUD            REG          
///                                    ###
///                                    * Primary Port Channel 0 control register
///                                    ###
///     @ 0x00008                      (P)
///     # 0x00008 TSD0                 
///               $AUDCH               TSD0              REG          
///                                    ###
///                                    * Primary Port Channel 1 control register
///                                    ###
///     @ 0x00014                      (P)
///     # 0x00014 TSD1                 
///               $AUDCH               TSD1              REG          
///                                    ###
///                                    * Primary Port Channel 2 control register
///                                    ###
///     @ 0x00020                      (P)
///     # 0x00020 TSD2                 
///               $AUDCH               TSD2              REG          
///                                    ###
///                                    * Primary Port Channel 3 control register
///                                    ###
///     @ 0x0002C                      (P)
///     # 0x0002C TSD3                 
///               $AUDCH               TSD3              REG          
///     @ 0x00038 PRIPORT              (RW)
///               %unsigned 1  ENABLE                    0x0
///                                    ###
///                                    * Register bit to enable or disable the whole Primary Audio port:
///                                    * 0: Disabled (default)
///                                    * 1: Enabled
///                                    * Note: User can selects the desired number of audio channels within the primary port by setting the corresponding enable bits for Channel 0/1/2/3
///                                    ###
///                                    : DISABLE                   0x0
///                                    : ENABLE                    0x1
///               %%        31         # Stuffing bits...
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      60B, bits:     291b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_P71
#define h_P71 (){}

    #define     RA_P71_PRIAUD                                  0x0000
    ///////////////////////////////////////////////////////////
    #define     RA_P71_TSD0                                    0x0008
    ///////////////////////////////////////////////////////////
    #define     RA_P71_TSD1                                    0x0014
    ///////////////////////////////////////////////////////////
    #define     RA_P71_TSD2                                    0x0020
    ///////////////////////////////////////////////////////////
    #define     RA_P71_TSD3                                    0x002C
    ///////////////////////////////////////////////////////////
    #define     RA_P71_PRIPORT                                 0x0038

    #define     BA_P71_PRIPORT_ENABLE                          0x0038
    #define     B16P71_PRIPORT_ENABLE                          0x0038
    #define   LSb32P71_PRIPORT_ENABLE                             0
    #define   LSb16P71_PRIPORT_ENABLE                             0
    #define       bP71_PRIPORT_ENABLE                          1
    #define   MSK32P71_PRIPORT_ENABLE                             0x00000001
    #define        P71_PRIPORT_ENABLE_DISABLE                               0x0
    #define        P71_PRIPORT_ENABLE_ENABLE                                0x1
    ///////////////////////////////////////////////////////////

    typedef struct SIE_P71 {
    ///////////////////////////////////////////////////////////
              SIE_PRIAUD                                       ie_PRIAUD;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_TSD0;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_TSD1;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_TSD2;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_TSD3;
    ///////////////////////////////////////////////////////////
    #define   GET32P71_PRIPORT_ENABLE(r32)                     _BFGET_(r32, 0, 0)
    #define   SET32P71_PRIPORT_ENABLE(r32,v)                   _BFSET_(r32, 0, 0,v)
    #define   GET16P71_PRIPORT_ENABLE(r16)                     _BFGET_(r16, 0, 0)
    #define   SET16P71_PRIPORT_ENABLE(r16,v)                   _BFSET_(r16, 0, 0,v)

    #define     w32P71_PRIPORT                                 {\
            UNSG32 uPRIPORT_ENABLE                             :  1;\
            UNSG32 RSVDx38_b1                                  : 31;\
          }
    union { UNSG32 u32P71_PRIPORT;
            struct w32P71_PRIPORT;
          };
    ///////////////////////////////////////////////////////////
    } SIE_P71;

    typedef union  T32P71_PRIPORT
          { UNSG32 u32;
            struct w32P71_PRIPORT;
                 } T32P71_PRIPORT;
    ///////////////////////////////////////////////////////////

    typedef union  TP71_PRIPORT
          { UNSG32 u32[1];
            struct {
            struct w32P71_PRIPORT;
                   };
                 } TP71_PRIPORT;

    ///////////////////////////////////////////////////////////
     SIGN32 P71_drvrd(SIE_P71 *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 P71_drvwr(SIE_P71 *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void P71_reset(SIE_P71 *p);
     SIGN32 P71_cmp  (SIE_P71 *p, SIE_P71 *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define P71_check(p,pie,pfx,hLOG) P71_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define P71_print(p,    pfx,hLOG) P71_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: P71
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE SEC                                      (4,4)
///     ###
///     * Audio Secondary port Control registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000                      (P)
///     # 0x00000 SECAUD               
///               $PRIAUD              SECAUD            REG          
///     @ 0x00008                      (P)
///     # 0x00008 TSD                  
///               $AUDCH               TSD               REG          
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      20B, bits:      83b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_SEC
#define h_SEC (){}

    #define     RA_SEC_SECAUD                                  0x0000
    ///////////////////////////////////////////////////////////
    #define     RA_SEC_TSD                                     0x0008
    ///////////////////////////////////////////////////////////

    typedef struct SIE_SEC {
    ///////////////////////////////////////////////////////////
              SIE_PRIAUD                                       ie_SECAUD;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_TSD;
    ///////////////////////////////////////////////////////////
    } SIE_SEC;

    ///////////////////////////////////////////////////////////
     SIGN32 SEC_drvrd(SIE_SEC *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 SEC_drvwr(SIE_SEC *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void SEC_reset(SIE_SEC *p);
     SIGN32 SEC_cmp  (SIE_SEC *p, SIE_SEC *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define SEC_check(p,pie,pfx,hLOG) SEC_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define SEC_print(p,    pfx,hLOG) SEC_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: SEC
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE HDMI                                     (4,4)
///     ###
///     * HDMI port control register
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000                      (P)
///     # 0x00000 HDAUD                
///               $PRIAUD              HDAUD             REG          
///     @ 0x00008                      (P)
///     # 0x00008 HDTSD                
///               $AUDCH               HDTSD             REG          
///     @ 0x00014 HDPORT               (RW)
///               ###
///               * High Bit Rate (HD) audio port Control register
///               ###
///               %unsigned 1  TXSEL                     0x0
///                                    ###
///                                    * Register used to select the transmitter for HDMI audio.
///                                    * 0 : HD channel
///                                    * 1: p71 channel (primary audio)
///                                    ###
///               %%        31         # Stuffing bits...
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      24B, bits:      84b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_HDMI
#define h_HDMI (){}

    #define     RA_HDMI_HDAUD                                  0x0000
    ///////////////////////////////////////////////////////////
    #define     RA_HDMI_HDTSD                                  0x0008
    ///////////////////////////////////////////////////////////
    #define     RA_HDMI_HDPORT                                 0x0014

    #define     BA_HDMI_HDPORT_TXSEL                           0x0014
    #define     B16HDMI_HDPORT_TXSEL                           0x0014
    #define   LSb32HDMI_HDPORT_TXSEL                              0
    #define   LSb16HDMI_HDPORT_TXSEL                              0
    #define       bHDMI_HDPORT_TXSEL                           1
    #define   MSK32HDMI_HDPORT_TXSEL                              0x00000001
    ///////////////////////////////////////////////////////////

    typedef struct SIE_HDMI {
    ///////////////////////////////////////////////////////////
              SIE_PRIAUD                                       ie_HDAUD;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_HDTSD;
    ///////////////////////////////////////////////////////////
    #define   GET32HDMI_HDPORT_TXSEL(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32HDMI_HDPORT_TXSEL(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16HDMI_HDPORT_TXSEL(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16HDMI_HDPORT_TXSEL(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32HDMI_HDPORT                                 {\
            UNSG32 uHDPORT_TXSEL                               :  1;\
            UNSG32 RSVDx14_b1                                  : 31;\
          }
    union { UNSG32 u32HDMI_HDPORT;
            struct w32HDMI_HDPORT;
          };
    ///////////////////////////////////////////////////////////
    } SIE_HDMI;

    typedef union  T32HDMI_HDPORT
          { UNSG32 u32;
            struct w32HDMI_HDPORT;
                 } T32HDMI_HDPORT;
    ///////////////////////////////////////////////////////////

    typedef union  THDMI_HDPORT
          { UNSG32 u32[1];
            struct {
            struct w32HDMI_HDPORT;
                   };
                 } THDMI_HDPORT;

    ///////////////////////////////////////////////////////////
     SIGN32 HDMI_drvrd(SIE_HDMI *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 HDMI_drvwr(SIE_HDMI *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void HDMI_reset(SIE_HDMI *p);
     SIGN32 HDMI_cmp  (SIE_HDMI *p, SIE_HDMI *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define HDMI_check(p,pie,pfx,hLOG) HDMI_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define HDMI_print(p,    pfx,hLOG) HDMI_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: HDMI
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE MIC                                      (4,4)
///     ###
///     * RX Port registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000                      (P)
///     # 0x00000 MICCTRL              
///               $PRIAUD              MICCTRL           REG          
///                                    ###
///                                    * RX Port Channel 0 control register
///                                    ###
///     @ 0x00008                      (P)
///     # 0x00008 RSD0                 
///               $AUDCH               RSD0              REG          
///                                    ###
///                                    * RX Port Channel 1 control register
///                                    ###
///     @ 0x00014                      (P)
///     # 0x00014 RSD1                 
///               $AUDCH               RSD1              REG          
///                                    ###
///                                    * RX Port Channel 2 control register
///                                    ###
///     @ 0x00020                      (P)
///     # 0x00020 RSD2                 
///               $AUDCH               RSD2              REG          
///                                    ###
///                                    * RX Port Channel 3 control register
///                                    ###
///     @ 0x0002C                      (P)
///     # 0x0002C RSD3                 
///               $AUDCH               RSD3              REG          
///     @ 0x00038 RXPORT               (RW)
///               %unsigned 1  ENABLE                    0x0
///                                    ###
///                                    * Register bit to enable or disable the whole RX Audio port:
///                                    * 0: Disabled (default)
///                                    * 1: Enabled
///                                    * Note: User can selects the desired number of audio channels within the RX port by setting the
///                                    * corresponding enable bits for channel 0/1/2/3
///                                    ###
///                                    : DISABLE                   0x0
///                                    : ENABLE                    0x1
///               %%        31         # Stuffing bits...
///     @ 0x0003C RXDATA               (RW)
///               %unsigned 1  HBR                       0x0
///                                    ###
///                                    * Register bit to indicate whether incoming data is 8-channel HBR or LPCM (or 2-channel HBR)
///                                    * 1 : HBR data (8-channel HBR)
///                                    * 0 : LPCM data
///                                    * Note: This bit should not be set to “1” in case of 2-channel HBR. As there is no difference in behavior of this module between 2-channel HBR or 2-channel LPCM data.
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00040 HBRDMAP              (RW)
///               ###
///               * HBR DATA MAP Register.
///               * In 8-channel HBR data mode. 128-bits of data is received by SoC. These registers bits allows the users to map the 32 bits received on each port to the desired place in 128-bits.
///               ###
///               %unsigned 2  PORT0                     0x0
///                                    ###
///                                    * Data select for [31:0]:
///                                    * 00 : Port 0 data
///                                    * 01 : Port 1 data
///                                    * 10 : Port 2 data
///                                    * 11 : Port 3 data
///                                    ###
///               %unsigned 2  PORT1                     0x1
///                                    ###
///                                    * Data select for [63:32]:
///                                    * 00 : Port 0 data
///                                    * 01 : Port 1 data
///                                    * 10 : Port 2 data
///                                    * 11 : Port 3 data
///                                    ###
///               %unsigned 2  PORT2                     0x2
///                                    ###
///                                    * Data select for [95:64]:
///                                    * 00 : Port 0 data
///                                    * 01 : Port 1 data
///                                    * 10 : Port 2 data
///                                    * 11 : Port 3 data
///                                    ###
///               %unsigned 2  PORT3                     0x3
///                                    ###
///                                    * Data select for [127:96]:
///                                    * 00 : Port 0 data
///                                    * 01 : Port 1 data
///                                    * 10 : Port 2 data
///                                    * 11 : Port 3 data
///                                    ###
///               %%        24         # Stuffing bits...
///     @ 0x00044 PDM_CTRL1            (RW)
///               %unsigned 4  MCLKSEL                   0x1
///                                    ###
///                                    * PDM MCLK Selection register:
///                                    * 3'b000 : Select MCLK PRIMARY
///                                    * 3'b001 : Select MCLK SECONDARY (default)
///                                    * 3'b010 : Select MCLK HD TX
///                                    * 3'b011 : Select MCLK SPDIF
///                                    * 3'b100 : Select MCLK HD RX
///                                    * 3'b101 to 3'b111 reserved
///                                    * Bit 4: is used to select MCLK of 4-channel MIC
///                                    * 1: Select MCLK of 4-channel MIC
///                                    ###
///               %unsigned 4  CLKDIV                    0x4
///                                    ###
///                                    * PDM CLKOUT Divider.
///                                    * 0: Divide by 1
///                                    * 1: Divide by 2
///                                    * 2: Divide by 4
///                                    * 3: Divide by 8
///                                    * 4: Divide by 16 (default)
///                                    * 5: Divide by 32
///                                    * 6: Divide by 64
///                                    * ........................
///                                    * ........................
///                                    * 14: Divide by 16384
///                                    * 15: Divide by 32768
///                                    ###
///               %unsigned 1  CLKEN                     0x0
///                                    ###
///                                    * PDM CLKOUT ENABLE
///                                    * 0 : PDM CLK disabled (default)
///                                    * 1 : PDM CLK enabled
///                                    ###
///               %unsigned 1  ENABLE                    0x0
///                                    ###
///                                    * PDM interface enable
///                                    * 0: Disabled (default)
///                                    * 1: Enabled
///                                    ###
///               %unsigned 1  MUTE                      0x0
///                                    ###
///                                    * Control bit to turn mute function ON or OFF
///                                    * 0: Mute turned OFF (default)
///                                    * 1: Mute turned ON
///                                    ###
///               %unsigned 1  LRSWITCH                  0x0
///                                    ###
///                                    * Control bit for switching the left channel data with right channel data:
///                                    * 0: Do not switch data (default)
///                                    * 1: Switch Data
///                                    ###
///               %unsigned 1  INVCLK_OUT                0x0
///                                    ###
///                                    * Invert PDM CLK OUT
///                                    * 0: Not inverted (default)
///                                    * 1: Inverted
///                                    ###
///               %unsigned 1  INVCLK_INT                0x0
///                                    ###
///                                    * Invert PDM CLK Internal
///                                    * 0: Not inverted (default)
///                                    * 1: Inverted
///                                    ###
///               %unsigned 1  RLSB                      0x0
///                                    ###
///                                    * Decides which bit is eceived first(left):
///                                    * 0: MSB first (default)
///                                    * 1: LSB first
///                                    ###
///               %unsigned 3  RDM                       0x4
///                                    ###
///                                    * L/R Channel resolution
///                                    * 0: 16-bits per channel
///                                    * 1: 18-bits per channel
///                                    * 2: 20-bits per channel
///                                    * 3: 24-bits per channel
///                                    * 4: 32-bits per channel (default)
///                                    * 5-7: Reserved
///                                    ###
///               %unsigned 1  MODE                      0x0
///                                    ###
///                                    * 0 : Half cycle PDM (DDR) (default)
///                                    * 1 : Classic PDM      (SDR)
///                                    ###
///               %unsigned 1  SDR_CLKSEL                0x0
///                                    ###
///                                    * SDR Data latch clock edge select
///                                    * 0: Latch data on falling clock edge of CLKOUT (default)
///                                    * 1: Latch data on rising clock edge of CLKOUT
///                                    ###
///               %unsigned 1  LATCH_MODE                0x0
///                                    ###
///                                    * PDM Data input latch mode
///                                    * 0: Counter, Use oversampling clock (400Mhz, sysClk) to lath the input data (default)
///                                    * 1: Edge, Use PDM CLKOUT Edge to latch the the input data
///                                    ###
///               %%        11         # Stuffing bits...
///     @ 0x00048 PDM_CTRL2            (RW)
///               %unsigned 16 RDLT                      0x1E
///                                    ###
///                                    * Rise Data Latch Time
///                                    * This registers decides PDM input data latch time after the Rising edge of PDMCLK in-terms of sysClk cycles + 1, when Data Latch mode is counter
///                                    ###
///               %unsigned 16 FDLT                      0x1E
///                                    ###
///                                    * Fall Data Latch Time
///                                    * This registers decides PDM input data latch time after the falling edge of PDMCLK in-terms of sysClk cycles + 1, when Data Latch mode is counter
///                                    ###
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      76B, bits:     353b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_MIC
#define h_MIC (){}

    #define     RA_MIC_MICCTRL                                 0x0000
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_RSD0                                    0x0008
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_RSD1                                    0x0014
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_RSD2                                    0x0020
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_RSD3                                    0x002C
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_RXPORT                                  0x0038

    #define     BA_MIC_RXPORT_ENABLE                           0x0038
    #define     B16MIC_RXPORT_ENABLE                           0x0038
    #define   LSb32MIC_RXPORT_ENABLE                              0
    #define   LSb16MIC_RXPORT_ENABLE                              0
    #define       bMIC_RXPORT_ENABLE                           1
    #define   MSK32MIC_RXPORT_ENABLE                              0x00000001
    #define        MIC_RXPORT_ENABLE_DISABLE                                0x0
    #define        MIC_RXPORT_ENABLE_ENABLE                                 0x1
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_RXDATA                                  0x003C

    #define     BA_MIC_RXDATA_HBR                              0x003C
    #define     B16MIC_RXDATA_HBR                              0x003C
    #define   LSb32MIC_RXDATA_HBR                                 0
    #define   LSb16MIC_RXDATA_HBR                                 0
    #define       bMIC_RXDATA_HBR                              1
    #define   MSK32MIC_RXDATA_HBR                                 0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_HBRDMAP                                 0x0040

    #define     BA_MIC_HBRDMAP_PORT0                           0x0040
    #define     B16MIC_HBRDMAP_PORT0                           0x0040
    #define   LSb32MIC_HBRDMAP_PORT0                              0
    #define   LSb16MIC_HBRDMAP_PORT0                              0
    #define       bMIC_HBRDMAP_PORT0                           2
    #define   MSK32MIC_HBRDMAP_PORT0                              0x00000003

    #define     BA_MIC_HBRDMAP_PORT1                           0x0040
    #define     B16MIC_HBRDMAP_PORT1                           0x0040
    #define   LSb32MIC_HBRDMAP_PORT1                              2
    #define   LSb16MIC_HBRDMAP_PORT1                              2
    #define       bMIC_HBRDMAP_PORT1                           2
    #define   MSK32MIC_HBRDMAP_PORT1                              0x0000000C

    #define     BA_MIC_HBRDMAP_PORT2                           0x0040
    #define     B16MIC_HBRDMAP_PORT2                           0x0040
    #define   LSb32MIC_HBRDMAP_PORT2                              4
    #define   LSb16MIC_HBRDMAP_PORT2                              4
    #define       bMIC_HBRDMAP_PORT2                           2
    #define   MSK32MIC_HBRDMAP_PORT2                              0x00000030

    #define     BA_MIC_HBRDMAP_PORT3                           0x0040
    #define     B16MIC_HBRDMAP_PORT3                           0x0040
    #define   LSb32MIC_HBRDMAP_PORT3                              6
    #define   LSb16MIC_HBRDMAP_PORT3                              6
    #define       bMIC_HBRDMAP_PORT3                           2
    #define   MSK32MIC_HBRDMAP_PORT3                              0x000000C0
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_PDM_CTRL1                               0x0044

    #define     BA_MIC_PDM_CTRL1_MCLKSEL                       0x0044
    #define     B16MIC_PDM_CTRL1_MCLKSEL                       0x0044
    #define   LSb32MIC_PDM_CTRL1_MCLKSEL                          0
    #define   LSb16MIC_PDM_CTRL1_MCLKSEL                          0
    #define       bMIC_PDM_CTRL1_MCLKSEL                       4
    #define   MSK32MIC_PDM_CTRL1_MCLKSEL                          0x0000000F

    #define     BA_MIC_PDM_CTRL1_CLKDIV                        0x0044
    #define     B16MIC_PDM_CTRL1_CLKDIV                        0x0044
    #define   LSb32MIC_PDM_CTRL1_CLKDIV                           4
    #define   LSb16MIC_PDM_CTRL1_CLKDIV                           4
    #define       bMIC_PDM_CTRL1_CLKDIV                        4
    #define   MSK32MIC_PDM_CTRL1_CLKDIV                           0x000000F0

    #define     BA_MIC_PDM_CTRL1_CLKEN                         0x0045
    #define     B16MIC_PDM_CTRL1_CLKEN                         0x0044
    #define   LSb32MIC_PDM_CTRL1_CLKEN                            8
    #define   LSb16MIC_PDM_CTRL1_CLKEN                            8
    #define       bMIC_PDM_CTRL1_CLKEN                         1
    #define   MSK32MIC_PDM_CTRL1_CLKEN                            0x00000100

    #define     BA_MIC_PDM_CTRL1_ENABLE                        0x0045
    #define     B16MIC_PDM_CTRL1_ENABLE                        0x0044
    #define   LSb32MIC_PDM_CTRL1_ENABLE                           9
    #define   LSb16MIC_PDM_CTRL1_ENABLE                           9
    #define       bMIC_PDM_CTRL1_ENABLE                        1
    #define   MSK32MIC_PDM_CTRL1_ENABLE                           0x00000200

    #define     BA_MIC_PDM_CTRL1_MUTE                          0x0045
    #define     B16MIC_PDM_CTRL1_MUTE                          0x0044
    #define   LSb32MIC_PDM_CTRL1_MUTE                             10
    #define   LSb16MIC_PDM_CTRL1_MUTE                             10
    #define       bMIC_PDM_CTRL1_MUTE                          1
    #define   MSK32MIC_PDM_CTRL1_MUTE                             0x00000400

    #define     BA_MIC_PDM_CTRL1_LRSWITCH                      0x0045
    #define     B16MIC_PDM_CTRL1_LRSWITCH                      0x0044
    #define   LSb32MIC_PDM_CTRL1_LRSWITCH                         11
    #define   LSb16MIC_PDM_CTRL1_LRSWITCH                         11
    #define       bMIC_PDM_CTRL1_LRSWITCH                      1
    #define   MSK32MIC_PDM_CTRL1_LRSWITCH                         0x00000800

    #define     BA_MIC_PDM_CTRL1_INVCLK_OUT                    0x0045
    #define     B16MIC_PDM_CTRL1_INVCLK_OUT                    0x0044
    #define   LSb32MIC_PDM_CTRL1_INVCLK_OUT                       12
    #define   LSb16MIC_PDM_CTRL1_INVCLK_OUT                       12
    #define       bMIC_PDM_CTRL1_INVCLK_OUT                    1
    #define   MSK32MIC_PDM_CTRL1_INVCLK_OUT                       0x00001000

    #define     BA_MIC_PDM_CTRL1_INVCLK_INT                    0x0045
    #define     B16MIC_PDM_CTRL1_INVCLK_INT                    0x0044
    #define   LSb32MIC_PDM_CTRL1_INVCLK_INT                       13
    #define   LSb16MIC_PDM_CTRL1_INVCLK_INT                       13
    #define       bMIC_PDM_CTRL1_INVCLK_INT                    1
    #define   MSK32MIC_PDM_CTRL1_INVCLK_INT                       0x00002000

    #define     BA_MIC_PDM_CTRL1_RLSB                          0x0045
    #define     B16MIC_PDM_CTRL1_RLSB                          0x0044
    #define   LSb32MIC_PDM_CTRL1_RLSB                             14
    #define   LSb16MIC_PDM_CTRL1_RLSB                             14
    #define       bMIC_PDM_CTRL1_RLSB                          1
    #define   MSK32MIC_PDM_CTRL1_RLSB                             0x00004000

    #define     BA_MIC_PDM_CTRL1_RDM                           0x0045
    #define     B16MIC_PDM_CTRL1_RDM                           0x0044
    #define   LSb32MIC_PDM_CTRL1_RDM                              15
    #define   LSb16MIC_PDM_CTRL1_RDM                              15
    #define       bMIC_PDM_CTRL1_RDM                           3
    #define   MSK32MIC_PDM_CTRL1_RDM                              0x00038000

    #define     BA_MIC_PDM_CTRL1_MODE                          0x0046
    #define     B16MIC_PDM_CTRL1_MODE                          0x0046
    #define   LSb32MIC_PDM_CTRL1_MODE                             18
    #define   LSb16MIC_PDM_CTRL1_MODE                             2
    #define       bMIC_PDM_CTRL1_MODE                          1
    #define   MSK32MIC_PDM_CTRL1_MODE                             0x00040000

    #define     BA_MIC_PDM_CTRL1_SDR_CLKSEL                    0x0046
    #define     B16MIC_PDM_CTRL1_SDR_CLKSEL                    0x0046
    #define   LSb32MIC_PDM_CTRL1_SDR_CLKSEL                       19
    #define   LSb16MIC_PDM_CTRL1_SDR_CLKSEL                       3
    #define       bMIC_PDM_CTRL1_SDR_CLKSEL                    1
    #define   MSK32MIC_PDM_CTRL1_SDR_CLKSEL                       0x00080000

    #define     BA_MIC_PDM_CTRL1_LATCH_MODE                    0x0046
    #define     B16MIC_PDM_CTRL1_LATCH_MODE                    0x0046
    #define   LSb32MIC_PDM_CTRL1_LATCH_MODE                       20
    #define   LSb16MIC_PDM_CTRL1_LATCH_MODE                       4
    #define       bMIC_PDM_CTRL1_LATCH_MODE                    1
    #define   MSK32MIC_PDM_CTRL1_LATCH_MODE                       0x00100000
    ///////////////////////////////////////////////////////////
    #define     RA_MIC_PDM_CTRL2                               0x0048

    #define     BA_MIC_PDM_CTRL2_RDLT                          0x0048
    #define     B16MIC_PDM_CTRL2_RDLT                          0x0048
    #define   LSb32MIC_PDM_CTRL2_RDLT                             0
    #define   LSb16MIC_PDM_CTRL2_RDLT                             0
    #define       bMIC_PDM_CTRL2_RDLT                          16
    #define   MSK32MIC_PDM_CTRL2_RDLT                             0x0000FFFF

    #define     BA_MIC_PDM_CTRL2_FDLT                          0x004A
    #define     B16MIC_PDM_CTRL2_FDLT                          0x004A
    #define   LSb32MIC_PDM_CTRL2_FDLT                             16
    #define   LSb16MIC_PDM_CTRL2_FDLT                             0
    #define       bMIC_PDM_CTRL2_FDLT                          16
    #define   MSK32MIC_PDM_CTRL2_FDLT                             0xFFFF0000
    ///////////////////////////////////////////////////////////

    typedef struct SIE_MIC {
    ///////////////////////////////////////////////////////////
              SIE_PRIAUD                                       ie_MICCTRL;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_RSD0;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_RSD1;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_RSD2;
    ///////////////////////////////////////////////////////////
              SIE_AUDCH                                        ie_RSD3;
    ///////////////////////////////////////////////////////////
    #define   GET32MIC_RXPORT_ENABLE(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32MIC_RXPORT_ENABLE(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16MIC_RXPORT_ENABLE(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16MIC_RXPORT_ENABLE(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32MIC_RXPORT                                  {\
            UNSG32 uRXPORT_ENABLE                              :  1;\
            UNSG32 RSVDx38_b1                                  : 31;\
          }
    union { UNSG32 u32MIC_RXPORT;
            struct w32MIC_RXPORT;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32MIC_RXDATA_HBR(r32)                         _BFGET_(r32, 0, 0)
    #define   SET32MIC_RXDATA_HBR(r32,v)                       _BFSET_(r32, 0, 0,v)
    #define   GET16MIC_RXDATA_HBR(r16)                         _BFGET_(r16, 0, 0)
    #define   SET16MIC_RXDATA_HBR(r16,v)                       _BFSET_(r16, 0, 0,v)

    #define     w32MIC_RXDATA                                  {\
            UNSG32 uRXDATA_HBR                                 :  1;\
            UNSG32 RSVDx3C_b1                                  : 31;\
          }
    union { UNSG32 u32MIC_RXDATA;
            struct w32MIC_RXDATA;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32MIC_HBRDMAP_PORT0(r32)                      _BFGET_(r32, 1, 0)
    #define   SET32MIC_HBRDMAP_PORT0(r32,v)                    _BFSET_(r32, 1, 0,v)
    #define   GET16MIC_HBRDMAP_PORT0(r16)                      _BFGET_(r16, 1, 0)
    #define   SET16MIC_HBRDMAP_PORT0(r16,v)                    _BFSET_(r16, 1, 0,v)

    #define   GET32MIC_HBRDMAP_PORT1(r32)                      _BFGET_(r32, 3, 2)
    #define   SET32MIC_HBRDMAP_PORT1(r32,v)                    _BFSET_(r32, 3, 2,v)
    #define   GET16MIC_HBRDMAP_PORT1(r16)                      _BFGET_(r16, 3, 2)
    #define   SET16MIC_HBRDMAP_PORT1(r16,v)                    _BFSET_(r16, 3, 2,v)

    #define   GET32MIC_HBRDMAP_PORT2(r32)                      _BFGET_(r32, 5, 4)
    #define   SET32MIC_HBRDMAP_PORT2(r32,v)                    _BFSET_(r32, 5, 4,v)
    #define   GET16MIC_HBRDMAP_PORT2(r16)                      _BFGET_(r16, 5, 4)
    #define   SET16MIC_HBRDMAP_PORT2(r16,v)                    _BFSET_(r16, 5, 4,v)

    #define   GET32MIC_HBRDMAP_PORT3(r32)                      _BFGET_(r32, 7, 6)
    #define   SET32MIC_HBRDMAP_PORT3(r32,v)                    _BFSET_(r32, 7, 6,v)
    #define   GET16MIC_HBRDMAP_PORT3(r16)                      _BFGET_(r16, 7, 6)
    #define   SET16MIC_HBRDMAP_PORT3(r16,v)                    _BFSET_(r16, 7, 6,v)

    #define     w32MIC_HBRDMAP                                 {\
            UNSG32 uHBRDMAP_PORT0                              :  2;\
            UNSG32 uHBRDMAP_PORT1                              :  2;\
            UNSG32 uHBRDMAP_PORT2                              :  2;\
            UNSG32 uHBRDMAP_PORT3                              :  2;\
            UNSG32 RSVDx40_b8                                  : 24;\
          }
    union { UNSG32 u32MIC_HBRDMAP;
            struct w32MIC_HBRDMAP;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32MIC_PDM_CTRL1_MCLKSEL(r32)                  _BFGET_(r32, 3, 0)
    #define   SET32MIC_PDM_CTRL1_MCLKSEL(r32,v)                _BFSET_(r32, 3, 0,v)
    #define   GET16MIC_PDM_CTRL1_MCLKSEL(r16)                  _BFGET_(r16, 3, 0)
    #define   SET16MIC_PDM_CTRL1_MCLKSEL(r16,v)                _BFSET_(r16, 3, 0,v)

    #define   GET32MIC_PDM_CTRL1_CLKDIV(r32)                   _BFGET_(r32, 7, 4)
    #define   SET32MIC_PDM_CTRL1_CLKDIV(r32,v)                 _BFSET_(r32, 7, 4,v)
    #define   GET16MIC_PDM_CTRL1_CLKDIV(r16)                   _BFGET_(r16, 7, 4)
    #define   SET16MIC_PDM_CTRL1_CLKDIV(r16,v)                 _BFSET_(r16, 7, 4,v)

    #define   GET32MIC_PDM_CTRL1_CLKEN(r32)                    _BFGET_(r32, 8, 8)
    #define   SET32MIC_PDM_CTRL1_CLKEN(r32,v)                  _BFSET_(r32, 8, 8,v)
    #define   GET16MIC_PDM_CTRL1_CLKEN(r16)                    _BFGET_(r16, 8, 8)
    #define   SET16MIC_PDM_CTRL1_CLKEN(r16,v)                  _BFSET_(r16, 8, 8,v)

    #define   GET32MIC_PDM_CTRL1_ENABLE(r32)                   _BFGET_(r32, 9, 9)
    #define   SET32MIC_PDM_CTRL1_ENABLE(r32,v)                 _BFSET_(r32, 9, 9,v)
    #define   GET16MIC_PDM_CTRL1_ENABLE(r16)                   _BFGET_(r16, 9, 9)
    #define   SET16MIC_PDM_CTRL1_ENABLE(r16,v)                 _BFSET_(r16, 9, 9,v)

    #define   GET32MIC_PDM_CTRL1_MUTE(r32)                     _BFGET_(r32,10,10)
    #define   SET32MIC_PDM_CTRL1_MUTE(r32,v)                   _BFSET_(r32,10,10,v)
    #define   GET16MIC_PDM_CTRL1_MUTE(r16)                     _BFGET_(r16,10,10)
    #define   SET16MIC_PDM_CTRL1_MUTE(r16,v)                   _BFSET_(r16,10,10,v)

    #define   GET32MIC_PDM_CTRL1_LRSWITCH(r32)                 _BFGET_(r32,11,11)
    #define   SET32MIC_PDM_CTRL1_LRSWITCH(r32,v)               _BFSET_(r32,11,11,v)
    #define   GET16MIC_PDM_CTRL1_LRSWITCH(r16)                 _BFGET_(r16,11,11)
    #define   SET16MIC_PDM_CTRL1_LRSWITCH(r16,v)               _BFSET_(r16,11,11,v)

    #define   GET32MIC_PDM_CTRL1_INVCLK_OUT(r32)               _BFGET_(r32,12,12)
    #define   SET32MIC_PDM_CTRL1_INVCLK_OUT(r32,v)             _BFSET_(r32,12,12,v)
    #define   GET16MIC_PDM_CTRL1_INVCLK_OUT(r16)               _BFGET_(r16,12,12)
    #define   SET16MIC_PDM_CTRL1_INVCLK_OUT(r16,v)             _BFSET_(r16,12,12,v)

    #define   GET32MIC_PDM_CTRL1_INVCLK_INT(r32)               _BFGET_(r32,13,13)
    #define   SET32MIC_PDM_CTRL1_INVCLK_INT(r32,v)             _BFSET_(r32,13,13,v)
    #define   GET16MIC_PDM_CTRL1_INVCLK_INT(r16)               _BFGET_(r16,13,13)
    #define   SET16MIC_PDM_CTRL1_INVCLK_INT(r16,v)             _BFSET_(r16,13,13,v)

    #define   GET32MIC_PDM_CTRL1_RLSB(r32)                     _BFGET_(r32,14,14)
    #define   SET32MIC_PDM_CTRL1_RLSB(r32,v)                   _BFSET_(r32,14,14,v)
    #define   GET16MIC_PDM_CTRL1_RLSB(r16)                     _BFGET_(r16,14,14)
    #define   SET16MIC_PDM_CTRL1_RLSB(r16,v)                   _BFSET_(r16,14,14,v)

    #define   GET32MIC_PDM_CTRL1_RDM(r32)                      _BFGET_(r32,17,15)
    #define   SET32MIC_PDM_CTRL1_RDM(r32,v)                    _BFSET_(r32,17,15,v)

    #define   GET32MIC_PDM_CTRL1_MODE(r32)                     _BFGET_(r32,18,18)
    #define   SET32MIC_PDM_CTRL1_MODE(r32,v)                   _BFSET_(r32,18,18,v)
    #define   GET16MIC_PDM_CTRL1_MODE(r16)                     _BFGET_(r16, 2, 2)
    #define   SET16MIC_PDM_CTRL1_MODE(r16,v)                   _BFSET_(r16, 2, 2,v)

    #define   GET32MIC_PDM_CTRL1_SDR_CLKSEL(r32)               _BFGET_(r32,19,19)
    #define   SET32MIC_PDM_CTRL1_SDR_CLKSEL(r32,v)             _BFSET_(r32,19,19,v)
    #define   GET16MIC_PDM_CTRL1_SDR_CLKSEL(r16)               _BFGET_(r16, 3, 3)
    #define   SET16MIC_PDM_CTRL1_SDR_CLKSEL(r16,v)             _BFSET_(r16, 3, 3,v)

    #define   GET32MIC_PDM_CTRL1_LATCH_MODE(r32)               _BFGET_(r32,20,20)
    #define   SET32MIC_PDM_CTRL1_LATCH_MODE(r32,v)             _BFSET_(r32,20,20,v)
    #define   GET16MIC_PDM_CTRL1_LATCH_MODE(r16)               _BFGET_(r16, 4, 4)
    #define   SET16MIC_PDM_CTRL1_LATCH_MODE(r16,v)             _BFSET_(r16, 4, 4,v)

    #define     w32MIC_PDM_CTRL1                               {\
            UNSG32 uPDM_CTRL1_MCLKSEL                          :  4;\
            UNSG32 uPDM_CTRL1_CLKDIV                           :  4;\
            UNSG32 uPDM_CTRL1_CLKEN                            :  1;\
            UNSG32 uPDM_CTRL1_ENABLE                           :  1;\
            UNSG32 uPDM_CTRL1_MUTE                             :  1;\
            UNSG32 uPDM_CTRL1_LRSWITCH                         :  1;\
            UNSG32 uPDM_CTRL1_INVCLK_OUT                       :  1;\
            UNSG32 uPDM_CTRL1_INVCLK_INT                       :  1;\
            UNSG32 uPDM_CTRL1_RLSB                             :  1;\
            UNSG32 uPDM_CTRL1_RDM                              :  3;\
            UNSG32 uPDM_CTRL1_MODE                             :  1;\
            UNSG32 uPDM_CTRL1_SDR_CLKSEL                       :  1;\
            UNSG32 uPDM_CTRL1_LATCH_MODE                       :  1;\
            UNSG32 RSVDx44_b21                                 : 11;\
          }
    union { UNSG32 u32MIC_PDM_CTRL1;
            struct w32MIC_PDM_CTRL1;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32MIC_PDM_CTRL2_RDLT(r32)                     _BFGET_(r32,15, 0)
    #define   SET32MIC_PDM_CTRL2_RDLT(r32,v)                   _BFSET_(r32,15, 0,v)
    #define   GET16MIC_PDM_CTRL2_RDLT(r16)                     _BFGET_(r16,15, 0)
    #define   SET16MIC_PDM_CTRL2_RDLT(r16,v)                   _BFSET_(r16,15, 0,v)

    #define   GET32MIC_PDM_CTRL2_FDLT(r32)                     _BFGET_(r32,31,16)
    #define   SET32MIC_PDM_CTRL2_FDLT(r32,v)                   _BFSET_(r32,31,16,v)
    #define   GET16MIC_PDM_CTRL2_FDLT(r16)                     _BFGET_(r16,15, 0)
    #define   SET16MIC_PDM_CTRL2_FDLT(r16,v)                   _BFSET_(r16,15, 0,v)

    #define     w32MIC_PDM_CTRL2                               {\
            UNSG32 uPDM_CTRL2_RDLT                             : 16;\
            UNSG32 uPDM_CTRL2_FDLT                             : 16;\
          }
    union { UNSG32 u32MIC_PDM_CTRL2;
            struct w32MIC_PDM_CTRL2;
          };
    ///////////////////////////////////////////////////////////
    } SIE_MIC;

    typedef union  T32MIC_RXPORT
          { UNSG32 u32;
            struct w32MIC_RXPORT;
                 } T32MIC_RXPORT;
    typedef union  T32MIC_RXDATA
          { UNSG32 u32;
            struct w32MIC_RXDATA;
                 } T32MIC_RXDATA;
    typedef union  T32MIC_HBRDMAP
          { UNSG32 u32;
            struct w32MIC_HBRDMAP;
                 } T32MIC_HBRDMAP;
    typedef union  T32MIC_PDM_CTRL1
          { UNSG32 u32;
            struct w32MIC_PDM_CTRL1;
                 } T32MIC_PDM_CTRL1;
    typedef union  T32MIC_PDM_CTRL2
          { UNSG32 u32;
            struct w32MIC_PDM_CTRL2;
                 } T32MIC_PDM_CTRL2;
    ///////////////////////////////////////////////////////////

    typedef union  TMIC_RXPORT
          { UNSG32 u32[1];
            struct {
            struct w32MIC_RXPORT;
                   };
                 } TMIC_RXPORT;
    typedef union  TMIC_RXDATA
          { UNSG32 u32[1];
            struct {
            struct w32MIC_RXDATA;
                   };
                 } TMIC_RXDATA;
    typedef union  TMIC_HBRDMAP
          { UNSG32 u32[1];
            struct {
            struct w32MIC_HBRDMAP;
                   };
                 } TMIC_HBRDMAP;
    typedef union  TMIC_PDM_CTRL1
          { UNSG32 u32[1];
            struct {
            struct w32MIC_PDM_CTRL1;
                   };
                 } TMIC_PDM_CTRL1;
    typedef union  TMIC_PDM_CTRL2
          { UNSG32 u32[1];
            struct {
            struct w32MIC_PDM_CTRL2;
                   };
                 } TMIC_PDM_CTRL2;

    ///////////////////////////////////////////////////////////
     SIGN32 MIC_drvrd(SIE_MIC *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 MIC_drvwr(SIE_MIC *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void MIC_reset(SIE_MIC *p);
     SIGN32 MIC_cmp  (SIE_MIC *p, SIE_MIC *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define MIC_check(p,pie,pfx,hLOG) MIC_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define MIC_print(p,    pfx,hLOG) MIC_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: MIC
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE IOSEL                   biu              (4,4)
///     ###
///     * PAD Output Enable selection register
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000 PRIMCLK              (RW)
///               ###
///               * Primary Port MCLK output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///                                    ###
///                                    * Note: Not used. MCLK source is selected using register defined in 88DE3010 Global Unit.
///                                    * *INTERNAL_ONLY**
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00004 SECMCLK              (RW)
///               ###
///               * Secondary Port MCLK output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///                                    ###
///                                    * Note: Not used. MCLK source is selected using register defined in 88DE3010 Global Unit.
///                                    * *INTERNAL_ONLY**
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00008 MIC1MCLK             (RW)
///               ###
///               * MIC Port MCLK output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///                                    ###
///                                    * Note: Not used. MCLK source is selected using register defined in 88DE3010 Global Unit.
///                                    * *INTERNAL_ONLY**
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x0000C PRIBCLK              (RW)
///               ###
///               * Primary port Bit-clock output enable register.
///               ###
///               %unsigned 1  SEL                       0x1
///                                    ###
///                                    * Bit clock output enable:
///                                    * 0: Bit clock is from external
///                                    * 1: Bit clock generated internally using Master Clock (MCLK) (default)
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00010 SECBCLK              (RW)
///               ###
///               * Secondary port Bit-clock output enable register.
///               ###
///               %unsigned 1  SEL                       0x1
///                                    ###
///                                    * Bit clock output enable:
///                                    * 0: Bit clock is from external
///                                    * 1: Bit clock generated internally using Master Clock (MCLK) (default)
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00014 MIC1BCLK             (RW)
///               ###
///               * MIC port Bit-clock output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///                                    ###
///                                    * Bit clock output enable:
///                                    * 0: Bit clock is from external (default)
///                                    * 1: Bit clock generated internally using MCLK
///                                    * Note: MIC Port is slave only so above bit should be set to 0.
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00018 MIC2BCLK             (RW)
///               ###
///               * MIC(4-Channel) port Bit-clock output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///               %%        31         # Stuffing bits...
///     @ 0x0001C PRIFS                (RW)
///               ###
///               * Primary port FSYNC output enable register.
///               ###
///               %unsigned 1  SEL                       0x1
///                                    ###
///                                    * In 88DE3010 Primary port FSYNC is generated internally so this bit should be set to 1.
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00020 SECFS                (RW)
///               ###
///               * Secondary port FSYNC output enable register.
///               ###
///               %unsigned 1  SEL                       0x1
///                                    ###
///                                    * In 88DE3010 Secondary port FSYNC is generated internally so this bit should be set to 1.
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x00024 MIC1FS               (RW)
///               ###
///               * MIC port FSYNC output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///               %%        31         # Stuffing bits...
///     @ 0x00028 MIC2FS               (RW)
///               ###
///               * MIC-4 port FSYNC output enable register.
///               ###
///               %unsigned 1  SEL                       0x0
///                                    ###
///                                    * In 88DE3010 MIC port FSYNC is from external so this bit should be set to 0.
///                                    ###
///               %%        31         # Stuffing bits...
///     @ 0x0002C PDMCLK               (RW)
///               ###
///               * PDM CLK Output Enable
///               * 0: No clock out
///               * 1: Clock out
///               ###
///               %unsigned 1  SEL                       0x0
///               %%        31         # Stuffing bits...
///     @ 0x00030 PDM                  (RW)
///               %unsigned 1  GENABLE                   0x0
///                                    ###
///                                    * PDM GLOBAL Enable
///                                    ###
///               %%        31         # Stuffing bits...
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:      52B, bits:      13b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_IOSEL
#define h_IOSEL (){}

    #define     RA_IOSEL_PRIMCLK                               0x0000

    #define     BA_IOSEL_PRIMCLK_SEL                           0x0000
    #define     B16IOSEL_PRIMCLK_SEL                           0x0000
    #define   LSb32IOSEL_PRIMCLK_SEL                              0
    #define   LSb16IOSEL_PRIMCLK_SEL                              0
    #define       bIOSEL_PRIMCLK_SEL                           1
    #define   MSK32IOSEL_PRIMCLK_SEL                              0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_SECMCLK                               0x0004

    #define     BA_IOSEL_SECMCLK_SEL                           0x0004
    #define     B16IOSEL_SECMCLK_SEL                           0x0004
    #define   LSb32IOSEL_SECMCLK_SEL                              0
    #define   LSb16IOSEL_SECMCLK_SEL                              0
    #define       bIOSEL_SECMCLK_SEL                           1
    #define   MSK32IOSEL_SECMCLK_SEL                              0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_MIC1MCLK                              0x0008

    #define     BA_IOSEL_MIC1MCLK_SEL                          0x0008
    #define     B16IOSEL_MIC1MCLK_SEL                          0x0008
    #define   LSb32IOSEL_MIC1MCLK_SEL                             0
    #define   LSb16IOSEL_MIC1MCLK_SEL                             0
    #define       bIOSEL_MIC1MCLK_SEL                          1
    #define   MSK32IOSEL_MIC1MCLK_SEL                             0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_PRIBCLK                               0x000C

    #define     BA_IOSEL_PRIBCLK_SEL                           0x000C
    #define     B16IOSEL_PRIBCLK_SEL                           0x000C
    #define   LSb32IOSEL_PRIBCLK_SEL                              0
    #define   LSb16IOSEL_PRIBCLK_SEL                              0
    #define       bIOSEL_PRIBCLK_SEL                           1
    #define   MSK32IOSEL_PRIBCLK_SEL                              0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_SECBCLK                               0x0010

    #define     BA_IOSEL_SECBCLK_SEL                           0x0010
    #define     B16IOSEL_SECBCLK_SEL                           0x0010
    #define   LSb32IOSEL_SECBCLK_SEL                              0
    #define   LSb16IOSEL_SECBCLK_SEL                              0
    #define       bIOSEL_SECBCLK_SEL                           1
    #define   MSK32IOSEL_SECBCLK_SEL                              0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_MIC1BCLK                              0x0014

    #define     BA_IOSEL_MIC1BCLK_SEL                          0x0014
    #define     B16IOSEL_MIC1BCLK_SEL                          0x0014
    #define   LSb32IOSEL_MIC1BCLK_SEL                             0
    #define   LSb16IOSEL_MIC1BCLK_SEL                             0
    #define       bIOSEL_MIC1BCLK_SEL                          1
    #define   MSK32IOSEL_MIC1BCLK_SEL                             0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_MIC2BCLK                              0x0018

    #define     BA_IOSEL_MIC2BCLK_SEL                          0x0018
    #define     B16IOSEL_MIC2BCLK_SEL                          0x0018
    #define   LSb32IOSEL_MIC2BCLK_SEL                             0
    #define   LSb16IOSEL_MIC2BCLK_SEL                             0
    #define       bIOSEL_MIC2BCLK_SEL                          1
    #define   MSK32IOSEL_MIC2BCLK_SEL                             0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_PRIFS                                 0x001C

    #define     BA_IOSEL_PRIFS_SEL                             0x001C
    #define     B16IOSEL_PRIFS_SEL                             0x001C
    #define   LSb32IOSEL_PRIFS_SEL                                0
    #define   LSb16IOSEL_PRIFS_SEL                                0
    #define       bIOSEL_PRIFS_SEL                             1
    #define   MSK32IOSEL_PRIFS_SEL                                0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_SECFS                                 0x0020

    #define     BA_IOSEL_SECFS_SEL                             0x0020
    #define     B16IOSEL_SECFS_SEL                             0x0020
    #define   LSb32IOSEL_SECFS_SEL                                0
    #define   LSb16IOSEL_SECFS_SEL                                0
    #define       bIOSEL_SECFS_SEL                             1
    #define   MSK32IOSEL_SECFS_SEL                                0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_MIC1FS                                0x0024

    #define     BA_IOSEL_MIC1FS_SEL                            0x0024
    #define     B16IOSEL_MIC1FS_SEL                            0x0024
    #define   LSb32IOSEL_MIC1FS_SEL                               0
    #define   LSb16IOSEL_MIC1FS_SEL                               0
    #define       bIOSEL_MIC1FS_SEL                            1
    #define   MSK32IOSEL_MIC1FS_SEL                               0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_MIC2FS                                0x0028

    #define     BA_IOSEL_MIC2FS_SEL                            0x0028
    #define     B16IOSEL_MIC2FS_SEL                            0x0028
    #define   LSb32IOSEL_MIC2FS_SEL                               0
    #define   LSb16IOSEL_MIC2FS_SEL                               0
    #define       bIOSEL_MIC2FS_SEL                            1
    #define   MSK32IOSEL_MIC2FS_SEL                               0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_PDMCLK                                0x002C

    #define     BA_IOSEL_PDMCLK_SEL                            0x002C
    #define     B16IOSEL_PDMCLK_SEL                            0x002C
    #define   LSb32IOSEL_PDMCLK_SEL                               0
    #define   LSb16IOSEL_PDMCLK_SEL                               0
    #define       bIOSEL_PDMCLK_SEL                            1
    #define   MSK32IOSEL_PDMCLK_SEL                               0x00000001
    ///////////////////////////////////////////////////////////
    #define     RA_IOSEL_PDM                                   0x0030

    #define     BA_IOSEL_PDM_GENABLE                           0x0030
    #define     B16IOSEL_PDM_GENABLE                           0x0030
    #define   LSb32IOSEL_PDM_GENABLE                              0
    #define   LSb16IOSEL_PDM_GENABLE                              0
    #define       bIOSEL_PDM_GENABLE                           1
    #define   MSK32IOSEL_PDM_GENABLE                              0x00000001
    ///////////////////////////////////////////////////////////

    typedef struct SIE_IOSEL {
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_PRIMCLK_SEL(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_PRIMCLK_SEL(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_PRIMCLK_SEL(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_PRIMCLK_SEL(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_PRIMCLK                               {\
            UNSG32 uPRIMCLK_SEL                                :  1;\
            UNSG32 RSVDx0_b1                                   : 31;\
          }
    union { UNSG32 u32IOSEL_PRIMCLK;
            struct w32IOSEL_PRIMCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_SECMCLK_SEL(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_SECMCLK_SEL(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_SECMCLK_SEL(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_SECMCLK_SEL(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_SECMCLK                               {\
            UNSG32 uSECMCLK_SEL                                :  1;\
            UNSG32 RSVDx4_b1                                   : 31;\
          }
    union { UNSG32 u32IOSEL_SECMCLK;
            struct w32IOSEL_SECMCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_MIC1MCLK_SEL(r32)                     _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_MIC1MCLK_SEL(r32,v)                   _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_MIC1MCLK_SEL(r16)                     _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_MIC1MCLK_SEL(r16,v)                   _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_MIC1MCLK                              {\
            UNSG32 uMIC1MCLK_SEL                               :  1;\
            UNSG32 RSVDx8_b1                                   : 31;\
          }
    union { UNSG32 u32IOSEL_MIC1MCLK;
            struct w32IOSEL_MIC1MCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_PRIBCLK_SEL(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_PRIBCLK_SEL(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_PRIBCLK_SEL(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_PRIBCLK_SEL(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_PRIBCLK                               {\
            UNSG32 uPRIBCLK_SEL                                :  1;\
            UNSG32 RSVDxC_b1                                   : 31;\
          }
    union { UNSG32 u32IOSEL_PRIBCLK;
            struct w32IOSEL_PRIBCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_SECBCLK_SEL(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_SECBCLK_SEL(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_SECBCLK_SEL(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_SECBCLK_SEL(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_SECBCLK                               {\
            UNSG32 uSECBCLK_SEL                                :  1;\
            UNSG32 RSVDx10_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_SECBCLK;
            struct w32IOSEL_SECBCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_MIC1BCLK_SEL(r32)                     _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_MIC1BCLK_SEL(r32,v)                   _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_MIC1BCLK_SEL(r16)                     _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_MIC1BCLK_SEL(r16,v)                   _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_MIC1BCLK                              {\
            UNSG32 uMIC1BCLK_SEL                               :  1;\
            UNSG32 RSVDx14_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_MIC1BCLK;
            struct w32IOSEL_MIC1BCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_MIC2BCLK_SEL(r32)                     _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_MIC2BCLK_SEL(r32,v)                   _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_MIC2BCLK_SEL(r16)                     _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_MIC2BCLK_SEL(r16,v)                   _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_MIC2BCLK                              {\
            UNSG32 uMIC2BCLK_SEL                               :  1;\
            UNSG32 RSVDx18_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_MIC2BCLK;
            struct w32IOSEL_MIC2BCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_PRIFS_SEL(r32)                        _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_PRIFS_SEL(r32,v)                      _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_PRIFS_SEL(r16)                        _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_PRIFS_SEL(r16,v)                      _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_PRIFS                                 {\
            UNSG32 uPRIFS_SEL                                  :  1;\
            UNSG32 RSVDx1C_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_PRIFS;
            struct w32IOSEL_PRIFS;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_SECFS_SEL(r32)                        _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_SECFS_SEL(r32,v)                      _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_SECFS_SEL(r16)                        _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_SECFS_SEL(r16,v)                      _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_SECFS                                 {\
            UNSG32 uSECFS_SEL                                  :  1;\
            UNSG32 RSVDx20_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_SECFS;
            struct w32IOSEL_SECFS;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_MIC1FS_SEL(r32)                       _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_MIC1FS_SEL(r32,v)                     _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_MIC1FS_SEL(r16)                       _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_MIC1FS_SEL(r16,v)                     _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_MIC1FS                                {\
            UNSG32 uMIC1FS_SEL                                 :  1;\
            UNSG32 RSVDx24_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_MIC1FS;
            struct w32IOSEL_MIC1FS;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_MIC2FS_SEL(r32)                       _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_MIC2FS_SEL(r32,v)                     _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_MIC2FS_SEL(r16)                       _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_MIC2FS_SEL(r16,v)                     _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_MIC2FS                                {\
            UNSG32 uMIC2FS_SEL                                 :  1;\
            UNSG32 RSVDx28_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_MIC2FS;
            struct w32IOSEL_MIC2FS;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_PDMCLK_SEL(r32)                       _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_PDMCLK_SEL(r32,v)                     _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_PDMCLK_SEL(r16)                       _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_PDMCLK_SEL(r16,v)                     _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_PDMCLK                                {\
            UNSG32 uPDMCLK_SEL                                 :  1;\
            UNSG32 RSVDx2C_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_PDMCLK;
            struct w32IOSEL_PDMCLK;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32IOSEL_PDM_GENABLE(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32IOSEL_PDM_GENABLE(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16IOSEL_PDM_GENABLE(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16IOSEL_PDM_GENABLE(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define     w32IOSEL_PDM                                   {\
            UNSG32 uPDM_GENABLE                                :  1;\
            UNSG32 RSVDx30_b1                                  : 31;\
          }
    union { UNSG32 u32IOSEL_PDM;
            struct w32IOSEL_PDM;
          };
    ///////////////////////////////////////////////////////////
    } SIE_IOSEL;

    typedef union  T32IOSEL_PRIMCLK
          { UNSG32 u32;
            struct w32IOSEL_PRIMCLK;
                 } T32IOSEL_PRIMCLK;
    typedef union  T32IOSEL_SECMCLK
          { UNSG32 u32;
            struct w32IOSEL_SECMCLK;
                 } T32IOSEL_SECMCLK;
    typedef union  T32IOSEL_MIC1MCLK
          { UNSG32 u32;
            struct w32IOSEL_MIC1MCLK;
                 } T32IOSEL_MIC1MCLK;
    typedef union  T32IOSEL_PRIBCLK
          { UNSG32 u32;
            struct w32IOSEL_PRIBCLK;
                 } T32IOSEL_PRIBCLK;
    typedef union  T32IOSEL_SECBCLK
          { UNSG32 u32;
            struct w32IOSEL_SECBCLK;
                 } T32IOSEL_SECBCLK;
    typedef union  T32IOSEL_MIC1BCLK
          { UNSG32 u32;
            struct w32IOSEL_MIC1BCLK;
                 } T32IOSEL_MIC1BCLK;
    typedef union  T32IOSEL_MIC2BCLK
          { UNSG32 u32;
            struct w32IOSEL_MIC2BCLK;
                 } T32IOSEL_MIC2BCLK;
    typedef union  T32IOSEL_PRIFS
          { UNSG32 u32;
            struct w32IOSEL_PRIFS;
                 } T32IOSEL_PRIFS;
    typedef union  T32IOSEL_SECFS
          { UNSG32 u32;
            struct w32IOSEL_SECFS;
                 } T32IOSEL_SECFS;
    typedef union  T32IOSEL_MIC1FS
          { UNSG32 u32;
            struct w32IOSEL_MIC1FS;
                 } T32IOSEL_MIC1FS;
    typedef union  T32IOSEL_MIC2FS
          { UNSG32 u32;
            struct w32IOSEL_MIC2FS;
                 } T32IOSEL_MIC2FS;
    typedef union  T32IOSEL_PDMCLK
          { UNSG32 u32;
            struct w32IOSEL_PDMCLK;
                 } T32IOSEL_PDMCLK;
    typedef union  T32IOSEL_PDM
          { UNSG32 u32;
            struct w32IOSEL_PDM;
                 } T32IOSEL_PDM;
    ///////////////////////////////////////////////////////////

    typedef union  TIOSEL_PRIMCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_PRIMCLK;
                   };
                 } TIOSEL_PRIMCLK;
    typedef union  TIOSEL_SECMCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_SECMCLK;
                   };
                 } TIOSEL_SECMCLK;
    typedef union  TIOSEL_MIC1MCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_MIC1MCLK;
                   };
                 } TIOSEL_MIC1MCLK;
    typedef union  TIOSEL_PRIBCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_PRIBCLK;
                   };
                 } TIOSEL_PRIBCLK;
    typedef union  TIOSEL_SECBCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_SECBCLK;
                   };
                 } TIOSEL_SECBCLK;
    typedef union  TIOSEL_MIC1BCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_MIC1BCLK;
                   };
                 } TIOSEL_MIC1BCLK;
    typedef union  TIOSEL_MIC2BCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_MIC2BCLK;
                   };
                 } TIOSEL_MIC2BCLK;
    typedef union  TIOSEL_PRIFS
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_PRIFS;
                   };
                 } TIOSEL_PRIFS;
    typedef union  TIOSEL_SECFS
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_SECFS;
                   };
                 } TIOSEL_SECFS;
    typedef union  TIOSEL_MIC1FS
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_MIC1FS;
                   };
                 } TIOSEL_MIC1FS;
    typedef union  TIOSEL_MIC2FS
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_MIC2FS;
                   };
                 } TIOSEL_MIC2FS;
    typedef union  TIOSEL_PDMCLK
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_PDMCLK;
                   };
                 } TIOSEL_PDMCLK;
    typedef union  TIOSEL_PDM
          { UNSG32 u32[1];
            struct {
            struct w32IOSEL_PDM;
                   };
                 } TIOSEL_PDM;

    ///////////////////////////////////////////////////////////
     SIGN32 IOSEL_drvrd(SIE_IOSEL *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 IOSEL_drvwr(SIE_IOSEL *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void IOSEL_reset(SIE_IOSEL *p);
     SIGN32 IOSEL_cmp  (SIE_IOSEL *p, SIE_IOSEL *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define IOSEL_check(p,pie,pfx,hLOG) IOSEL_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define IOSEL_print(p,    pfx,hLOG) IOSEL_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: IOSEL
////////////////////////////////////////////////////////////

//////
/// 
/// $INTERFACE AIO                     biu              (4,4)
///     ###
///     * Audio Input Output Unit Registers
///     ###
///     # # ----------------------------------------------------------
///     @ 0x00000                      (P)
///     # 0x00000 P71                  
///               $P71                 P71               REG          
///                                    ###
///                                    * Primary Port Registers
///                                    ###
///     @ 0x0003C                      (P)
///     # 0x0003C SEC                  
///               $SEC                 SEC               REG          
///                                    ###
///                                    * Secondary Port Registers
///                                    ###
///     @ 0x00050                      (P)
///     # 0x00050 HDMI                 
///               $HDMI                HDMI              REG          
///                                    ###
///                                    * HDMI port Registers
///                                    ###
///     @ 0x00068                      (P)
///     # 0x00068 SPDIF                
///               $SPDIF               SPDIF             REG          
///                                    ###
///                                    * S/PDIF Port Registers
///                                    ###
///     @ 0x00078                      (P)
///     # 0x00078 MIC1                 
///               $MIC                 MIC1              REG          
///     @ 0x000C4                      (P)
///     # 0x000C4 MIC2                 
///               $MIC                 MIC2              REG          
///                                    ###
///                                    * Audio input Registers
///                                    ###
///     @ 0x00110                      (P)
///     # 0x00110 IOSEL                
///               $IOSEL               IOSEL             REG          
///                                    ###
///                                    * Pad output enable control register
///                                    ###
///     @ 0x00144 IRQENABLE            (RW)
///               ###
///               * Interrupt Enable register
///               ###
///               %unsigned 1  PRIIRQ                    0x0
///                                    ###
///                                    * Primary Port Interrupt enable:
///                                    * 0: Not enabled
///                                    * 1: Enabled
///                                    ###
///               %unsigned 1  SECIRQ                    0x0
///                                    ###
///                                    * Secondary Port Interrupt enable:
///                                    * 0: Not enabled
///                                    * 1: Enabled
///                                    ###
///               %unsigned 1  MIC1IRQ                   0x0
///               %unsigned 1  MIC2IRQ                   0x0
///                                    ###
///                                    * MIC Port Interrupt enable:
///                                    * 0: Not enabled
///                                    * 1: Enabled
///                                    ###
///               %unsigned 1  SPDIFIRQ                  0x0
///                                    ###
///                                    * S/PDIF Port Interrupt enable:
///                                    * 0: Not enabled
///                                    * 1: Enabled
///                                    ###
///               %unsigned 1  HDMIIRQ                   0x0
///                                    ###
///                                    * HD port Interrupt enable:
///                                    * 0: Not enabled
///                                    * 1: Enabled
///                                    ###
///               %unsigned 1  PDMIRQ                    0x0
///                                    ###
///                                    * PDM port Interrupt enable:
///                                    * 0: Not enabled
///                                    * 1: Enabled
///                                    ###
///               %%        25         # Stuffing bits...
///     @ 0x00148 IRQSTS               (WOC-)
///               ###
///               * Interrupt status register, write 1 to clear.
///               * Interrupt is generated when corresponding data FIFO underflow. For MIC when FIFO overflow.
///               ###
///               %unsigned 1  PRISTS                    0x0
///                                    ###
///                                    * Primary port interrupt status:
///                                    * 0: No interrupt (default)
///                                    * 1: Interrupt Pending
///                                    ###
///               %unsigned 1  SECSTS                    0x0
///                                    ###
///                                    * Secondary port interrupt status:
///                                    * 0: No interrupt (default)
///                                    * 1: Interrupt Pending
///                                    ###
///               %unsigned 1  MIC1STS                   0x0
///               %unsigned 1  MIC2STS                   0x0
///                                    ###
///                                    * MIC port interrupt status:
///                                    * 0: No interrupt (default)
///                                    * 1: Interrupt Pending
///                                    ###
///               %unsigned 1  SPDIFSTS                  0x0
///                                    ###
///                                    * S/PDIF port interrupt status:
///                                    * 0: No interrupt (default)
///                                    * 1: Interrupt Pending
///                                    ###
///               %unsigned 1  HDMISTS                   0x0
///                                    ###
///                                    * HD port interrupt status:
///                                    * 0: No interrupt (default)
///                                    * 1: Interrupt Pending
///                                    ###
///               %unsigned 1  PDMSTS                    0x0
///                                    ###
///                                    * PDM port interrupt status:
///                                    * 0: No interrupt (default)
///                                    * 1: Interrupt Pending
///                                    ###
///               %%        25         # Stuffing bits...
///     @ 0x0014C HDSRC                (RW)
///               ###
///               * HBR Audio source selection register
///               ###
///               %unsigned 2  SEL                       0x0
///                                    ###
///                                    * Register to indicate number of samples per 64-bit data from DDR.
///                                    * 00 : HD data (2 samples {2pair of Left & Right} per 64-bit data from DDR) transmitted in 2 LRCK (over 2 channel)
///                                    * 01: L-PCM data (1 Sample {one pair of Left &Right} per 64-bit data from DDR) transmitted in 1 LRCK (over 2 channel)
///                                    * 10: HD data (4 samples {4pair of Left & Right } per 128-bit data from DDR) transmitter in 1 LRCK (over 8 channels)
///                                    ###
///               %%        30         # Stuffing bits...
///     @ 0x00150 ADAC_CLK_CTRL        (RW)
///               %unsigned 3  mclkDiv                   0x2
///                                    : DIV1                      0x0
///                                    : DIV2                      0x1
///                                    : DIV4                      0x2
///                                    : DIV8                      0x3
///                                    : DIV16                     0x4
///                                    : DIV32                     0x5
///                                    : DIV64                     0x6
///                                    : DIV128                    0x7
///                                                 ###
///                                                 * ADAC MClk divider setting defines the divide ratio between secondary audio MCLK and ADAC MCLK.
///                                                 ###
///               %unsigned 3  pwmclkDiv                 0x2
///                                    : DIV1                      0x0
///                                    : DIV2                      0x1
///                                    : DIV4                      0x2
///                                    : DIV8                      0x3
///                                    : DIV16                     0x4
///                                    : DIV32                     0x5
///                                    : DIV64                     0x6
///                                    : DIV128                    0x7
///                                                 ###
///                                                 * ADAC PWMClk divider setting defines the divide ratio between secondary audio MCLK and ADAC PWMCLK.
///                                                 ###
///               %unsigned 1  mclkInv                   0x0
///                                    : NORMAL                    0x0
///                                    : INVERTED                  0x1
///                                                 ###
///                                                 * Invert the ADAC MCLK polarity. By default, rising edge is aligned with data, if it is 1, falling edge is aligned with data.
///                                                 ###
///               %unsigned 1  pwmclkInv                 0x0
///                                    : NORMAL                    0x0
///                                    : INVERTED                  0x1
///                                                 ###
///                                                 * Invert the ADAC PWMCLK polarity. By default, rising edge is aligned with data, if it is 1, falling edge is aligned with data.
///                                                 ###
///               %unsigned 1  inputSel                  0x0
///                                    ###
///                                    * ADAC Input Select Bit
///                                    * 0: Select Secondary outputs
///                                    * 1: Select Primary outputs
///                                    ###
///               %unsigned 2  PWMCLK_SRC_SEL            0x0
///                                    ###
///                                    *   ADAC PWMCLK source select bit
///                                    * This bit selects the ADAC PWM Clock source it can be either Primary MCLK or Secondary MCLK based on these bits.
///                                    * Bit [1]:    Valid Bit
///                                    * 1 : Bit[0] decides the PWM Clock Source
///                                    * 0 : Bit[0] does not decide the clock source instead “inputSel” bit just above these bits decides the PWM Clock Source (default)
///                                    * Bit [0]: PWM Clock Source
///                                    * 0 : MCLK Secondary   (default)
///                                    * 1 : MCLK Primary
///                                    ###
///               %unsigned 1  PRIMCLK_OUT_SRC_SEL       0x0
///                                    ###
///                                    * This bits selects the source of the MCLK out on the DV0_FID pin
///                                    * 0: Primary MCLK (Default)
///                                    * 1: Secondary MCLK
///                                    ###
///               %%        20         # Stuffing bits...
///     @ 0x00154 PDM_MIC_SEL          (RW)
///               %unsigned 1  CTRL                      0x0
///               %%        31         # Stuffing bits...
///     # # ----------------------------------------------------------
/// $ENDOFINTERFACE  # size:     344B, bits:    1278b, padding:     0B
////////////////////////////////////////////////////////////
#ifndef h_AIO
#define h_AIO (){}

    #define     RA_AIO_P71                                     0x0000
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_SEC                                     0x003C
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_HDMI                                    0x0050
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_SPDIF                                   0x0068
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_MIC1                                    0x0078
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_MIC2                                    0x00C4
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_IOSEL                                   0x0110
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_IRQENABLE                               0x0144

    #define     BA_AIO_IRQENABLE_PRIIRQ                        0x0144
    #define     B16AIO_IRQENABLE_PRIIRQ                        0x0144
    #define   LSb32AIO_IRQENABLE_PRIIRQ                           0
    #define   LSb16AIO_IRQENABLE_PRIIRQ                           0
    #define       bAIO_IRQENABLE_PRIIRQ                        1
    #define   MSK32AIO_IRQENABLE_PRIIRQ                           0x00000001

    #define     BA_AIO_IRQENABLE_SECIRQ                        0x0144
    #define     B16AIO_IRQENABLE_SECIRQ                        0x0144
    #define   LSb32AIO_IRQENABLE_SECIRQ                           1
    #define   LSb16AIO_IRQENABLE_SECIRQ                           1
    #define       bAIO_IRQENABLE_SECIRQ                        1
    #define   MSK32AIO_IRQENABLE_SECIRQ                           0x00000002

    #define     BA_AIO_IRQENABLE_MIC1IRQ                       0x0144
    #define     B16AIO_IRQENABLE_MIC1IRQ                       0x0144
    #define   LSb32AIO_IRQENABLE_MIC1IRQ                          2
    #define   LSb16AIO_IRQENABLE_MIC1IRQ                          2
    #define       bAIO_IRQENABLE_MIC1IRQ                       1
    #define   MSK32AIO_IRQENABLE_MIC1IRQ                          0x00000004

    #define     BA_AIO_IRQENABLE_MIC2IRQ                       0x0144
    #define     B16AIO_IRQENABLE_MIC2IRQ                       0x0144
    #define   LSb32AIO_IRQENABLE_MIC2IRQ                          3
    #define   LSb16AIO_IRQENABLE_MIC2IRQ                          3
    #define       bAIO_IRQENABLE_MIC2IRQ                       1
    #define   MSK32AIO_IRQENABLE_MIC2IRQ                          0x00000008

    #define     BA_AIO_IRQENABLE_SPDIFIRQ                      0x0144
    #define     B16AIO_IRQENABLE_SPDIFIRQ                      0x0144
    #define   LSb32AIO_IRQENABLE_SPDIFIRQ                         4
    #define   LSb16AIO_IRQENABLE_SPDIFIRQ                         4
    #define       bAIO_IRQENABLE_SPDIFIRQ                      1
    #define   MSK32AIO_IRQENABLE_SPDIFIRQ                         0x00000010

    #define     BA_AIO_IRQENABLE_HDMIIRQ                       0x0144
    #define     B16AIO_IRQENABLE_HDMIIRQ                       0x0144
    #define   LSb32AIO_IRQENABLE_HDMIIRQ                          5
    #define   LSb16AIO_IRQENABLE_HDMIIRQ                          5
    #define       bAIO_IRQENABLE_HDMIIRQ                       1
    #define   MSK32AIO_IRQENABLE_HDMIIRQ                          0x00000020

    #define     BA_AIO_IRQENABLE_PDMIRQ                        0x0144
    #define     B16AIO_IRQENABLE_PDMIRQ                        0x0144
    #define   LSb32AIO_IRQENABLE_PDMIRQ                           6
    #define   LSb16AIO_IRQENABLE_PDMIRQ                           6
    #define       bAIO_IRQENABLE_PDMIRQ                        1
    #define   MSK32AIO_IRQENABLE_PDMIRQ                           0x00000040
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_IRQSTS                                  0x0148

    #define     BA_AIO_IRQSTS_PRISTS                           0x0148
    #define     B16AIO_IRQSTS_PRISTS                           0x0148
    #define   LSb32AIO_IRQSTS_PRISTS                              0
    #define   LSb16AIO_IRQSTS_PRISTS                              0
    #define       bAIO_IRQSTS_PRISTS                           1
    #define   MSK32AIO_IRQSTS_PRISTS                              0x00000001

    #define     BA_AIO_IRQSTS_SECSTS                           0x0148
    #define     B16AIO_IRQSTS_SECSTS                           0x0148
    #define   LSb32AIO_IRQSTS_SECSTS                              1
    #define   LSb16AIO_IRQSTS_SECSTS                              1
    #define       bAIO_IRQSTS_SECSTS                           1
    #define   MSK32AIO_IRQSTS_SECSTS                              0x00000002

    #define     BA_AIO_IRQSTS_MIC1STS                          0x0148
    #define     B16AIO_IRQSTS_MIC1STS                          0x0148
    #define   LSb32AIO_IRQSTS_MIC1STS                             2
    #define   LSb16AIO_IRQSTS_MIC1STS                             2
    #define       bAIO_IRQSTS_MIC1STS                          1
    #define   MSK32AIO_IRQSTS_MIC1STS                             0x00000004

    #define     BA_AIO_IRQSTS_MIC2STS                          0x0148
    #define     B16AIO_IRQSTS_MIC2STS                          0x0148
    #define   LSb32AIO_IRQSTS_MIC2STS                             3
    #define   LSb16AIO_IRQSTS_MIC2STS                             3
    #define       bAIO_IRQSTS_MIC2STS                          1
    #define   MSK32AIO_IRQSTS_MIC2STS                             0x00000008

    #define     BA_AIO_IRQSTS_SPDIFSTS                         0x0148
    #define     B16AIO_IRQSTS_SPDIFSTS                         0x0148
    #define   LSb32AIO_IRQSTS_SPDIFSTS                            4
    #define   LSb16AIO_IRQSTS_SPDIFSTS                            4
    #define       bAIO_IRQSTS_SPDIFSTS                         1
    #define   MSK32AIO_IRQSTS_SPDIFSTS                            0x00000010

    #define     BA_AIO_IRQSTS_HDMISTS                          0x0148
    #define     B16AIO_IRQSTS_HDMISTS                          0x0148
    #define   LSb32AIO_IRQSTS_HDMISTS                             5
    #define   LSb16AIO_IRQSTS_HDMISTS                             5
    #define       bAIO_IRQSTS_HDMISTS                          1
    #define   MSK32AIO_IRQSTS_HDMISTS                             0x00000020

    #define     BA_AIO_IRQSTS_PDMSTS                           0x0148
    #define     B16AIO_IRQSTS_PDMSTS                           0x0148
    #define   LSb32AIO_IRQSTS_PDMSTS                              6
    #define   LSb16AIO_IRQSTS_PDMSTS                              6
    #define       bAIO_IRQSTS_PDMSTS                           1
    #define   MSK32AIO_IRQSTS_PDMSTS                              0x00000040
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_HDSRC                                   0x014C

    #define     BA_AIO_HDSRC_SEL                               0x014C
    #define     B16AIO_HDSRC_SEL                               0x014C
    #define   LSb32AIO_HDSRC_SEL                                  0
    #define   LSb16AIO_HDSRC_SEL                                  0
    #define       bAIO_HDSRC_SEL                               2
    #define   MSK32AIO_HDSRC_SEL                                  0x00000003
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_ADAC_CLK_CTRL                           0x0150

    #define     BA_AIO_ADAC_CLK_CTRL_mclkDiv                   0x0150
    #define     B16AIO_ADAC_CLK_CTRL_mclkDiv                   0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_mclkDiv                      0
    #define   LSb16AIO_ADAC_CLK_CTRL_mclkDiv                      0
    #define       bAIO_ADAC_CLK_CTRL_mclkDiv                   3
    #define   MSK32AIO_ADAC_CLK_CTRL_mclkDiv                      0x00000007
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV1                           0x0
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV2                           0x1
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV4                           0x2
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV8                           0x3
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV16                          0x4
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV32                          0x5
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV64                          0x6
    #define        AIO_ADAC_CLK_CTRL_mclkDiv_DIV128                         0x7

    #define     BA_AIO_ADAC_CLK_CTRL_pwmclkDiv                 0x0150
    #define     B16AIO_ADAC_CLK_CTRL_pwmclkDiv                 0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_pwmclkDiv                    3
    #define   LSb16AIO_ADAC_CLK_CTRL_pwmclkDiv                    3
    #define       bAIO_ADAC_CLK_CTRL_pwmclkDiv                 3
    #define   MSK32AIO_ADAC_CLK_CTRL_pwmclkDiv                    0x00000038
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV1                         0x0
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV2                         0x1
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV4                         0x2
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV8                         0x3
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV16                        0x4
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV32                        0x5
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV64                        0x6
    #define        AIO_ADAC_CLK_CTRL_pwmclkDiv_DIV128                       0x7

    #define     BA_AIO_ADAC_CLK_CTRL_mclkInv                   0x0150
    #define     B16AIO_ADAC_CLK_CTRL_mclkInv                   0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_mclkInv                      6
    #define   LSb16AIO_ADAC_CLK_CTRL_mclkInv                      6
    #define       bAIO_ADAC_CLK_CTRL_mclkInv                   1
    #define   MSK32AIO_ADAC_CLK_CTRL_mclkInv                      0x00000040
    #define        AIO_ADAC_CLK_CTRL_mclkInv_NORMAL                         0x0
    #define        AIO_ADAC_CLK_CTRL_mclkInv_INVERTED                       0x1

    #define     BA_AIO_ADAC_CLK_CTRL_pwmclkInv                 0x0150
    #define     B16AIO_ADAC_CLK_CTRL_pwmclkInv                 0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_pwmclkInv                    7
    #define   LSb16AIO_ADAC_CLK_CTRL_pwmclkInv                    7
    #define       bAIO_ADAC_CLK_CTRL_pwmclkInv                 1
    #define   MSK32AIO_ADAC_CLK_CTRL_pwmclkInv                    0x00000080
    #define        AIO_ADAC_CLK_CTRL_pwmclkInv_NORMAL                       0x0
    #define        AIO_ADAC_CLK_CTRL_pwmclkInv_INVERTED                     0x1

    #define     BA_AIO_ADAC_CLK_CTRL_inputSel                  0x0151
    #define     B16AIO_ADAC_CLK_CTRL_inputSel                  0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_inputSel                     8
    #define   LSb16AIO_ADAC_CLK_CTRL_inputSel                     8
    #define       bAIO_ADAC_CLK_CTRL_inputSel                  1
    #define   MSK32AIO_ADAC_CLK_CTRL_inputSel                     0x00000100

    #define     BA_AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL            0x0151
    #define     B16AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL            0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL               9
    #define   LSb16AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL               9
    #define       bAIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL            2
    #define   MSK32AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL               0x00000600

    #define     BA_AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL       0x0151
    #define     B16AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL       0x0150
    #define   LSb32AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL          11
    #define   LSb16AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL          11
    #define       bAIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL       1
    #define   MSK32AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL          0x00000800
    ///////////////////////////////////////////////////////////
    #define     RA_AIO_PDM_MIC_SEL                             0x0154

    #define     BA_AIO_PDM_MIC_SEL_CTRL                        0x0154
    #define     B16AIO_PDM_MIC_SEL_CTRL                        0x0154
    #define   LSb32AIO_PDM_MIC_SEL_CTRL                           0
    #define   LSb16AIO_PDM_MIC_SEL_CTRL                           0
    #define       bAIO_PDM_MIC_SEL_CTRL                        1
    #define   MSK32AIO_PDM_MIC_SEL_CTRL                           0x00000001
    ///////////////////////////////////////////////////////////

    typedef struct SIE_AIO {
    ///////////////////////////////////////////////////////////
              SIE_P71                                          ie_P71;
    ///////////////////////////////////////////////////////////
              SIE_SEC                                          ie_SEC;
    ///////////////////////////////////////////////////////////
              SIE_HDMI                                         ie_HDMI;
    ///////////////////////////////////////////////////////////
              SIE_SPDIF                                        ie_SPDIF;
    ///////////////////////////////////////////////////////////
              SIE_MIC                                          ie_MIC1;
    ///////////////////////////////////////////////////////////
              SIE_MIC                                          ie_MIC2;
    ///////////////////////////////////////////////////////////
              SIE_IOSEL                                        ie_IOSEL;
    ///////////////////////////////////////////////////////////
    #define   GET32AIO_IRQENABLE_PRIIRQ(r32)                   _BFGET_(r32, 0, 0)
    #define   SET32AIO_IRQENABLE_PRIIRQ(r32,v)                 _BFSET_(r32, 0, 0,v)
    #define   GET16AIO_IRQENABLE_PRIIRQ(r16)                   _BFGET_(r16, 0, 0)
    #define   SET16AIO_IRQENABLE_PRIIRQ(r16,v)                 _BFSET_(r16, 0, 0,v)

    #define   GET32AIO_IRQENABLE_SECIRQ(r32)                   _BFGET_(r32, 1, 1)
    #define   SET32AIO_IRQENABLE_SECIRQ(r32,v)                 _BFSET_(r32, 1, 1,v)
    #define   GET16AIO_IRQENABLE_SECIRQ(r16)                   _BFGET_(r16, 1, 1)
    #define   SET16AIO_IRQENABLE_SECIRQ(r16,v)                 _BFSET_(r16, 1, 1,v)

    #define   GET32AIO_IRQENABLE_MIC1IRQ(r32)                  _BFGET_(r32, 2, 2)
    #define   SET32AIO_IRQENABLE_MIC1IRQ(r32,v)                _BFSET_(r32, 2, 2,v)
    #define   GET16AIO_IRQENABLE_MIC1IRQ(r16)                  _BFGET_(r16, 2, 2)
    #define   SET16AIO_IRQENABLE_MIC1IRQ(r16,v)                _BFSET_(r16, 2, 2,v)

    #define   GET32AIO_IRQENABLE_MIC2IRQ(r32)                  _BFGET_(r32, 3, 3)
    #define   SET32AIO_IRQENABLE_MIC2IRQ(r32,v)                _BFSET_(r32, 3, 3,v)
    #define   GET16AIO_IRQENABLE_MIC2IRQ(r16)                  _BFGET_(r16, 3, 3)
    #define   SET16AIO_IRQENABLE_MIC2IRQ(r16,v)                _BFSET_(r16, 3, 3,v)

    #define   GET32AIO_IRQENABLE_SPDIFIRQ(r32)                 _BFGET_(r32, 4, 4)
    #define   SET32AIO_IRQENABLE_SPDIFIRQ(r32,v)               _BFSET_(r32, 4, 4,v)
    #define   GET16AIO_IRQENABLE_SPDIFIRQ(r16)                 _BFGET_(r16, 4, 4)
    #define   SET16AIO_IRQENABLE_SPDIFIRQ(r16,v)               _BFSET_(r16, 4, 4,v)

    #define   GET32AIO_IRQENABLE_HDMIIRQ(r32)                  _BFGET_(r32, 5, 5)
    #define   SET32AIO_IRQENABLE_HDMIIRQ(r32,v)                _BFSET_(r32, 5, 5,v)
    #define   GET16AIO_IRQENABLE_HDMIIRQ(r16)                  _BFGET_(r16, 5, 5)
    #define   SET16AIO_IRQENABLE_HDMIIRQ(r16,v)                _BFSET_(r16, 5, 5,v)

    #define   GET32AIO_IRQENABLE_PDMIRQ(r32)                   _BFGET_(r32, 6, 6)
    #define   SET32AIO_IRQENABLE_PDMIRQ(r32,v)                 _BFSET_(r32, 6, 6,v)
    #define   GET16AIO_IRQENABLE_PDMIRQ(r16)                   _BFGET_(r16, 6, 6)
    #define   SET16AIO_IRQENABLE_PDMIRQ(r16,v)                 _BFSET_(r16, 6, 6,v)

    #define     w32AIO_IRQENABLE                               {\
            UNSG32 uIRQENABLE_PRIIRQ                           :  1;\
            UNSG32 uIRQENABLE_SECIRQ                           :  1;\
            UNSG32 uIRQENABLE_MIC1IRQ                          :  1;\
            UNSG32 uIRQENABLE_MIC2IRQ                          :  1;\
            UNSG32 uIRQENABLE_SPDIFIRQ                         :  1;\
            UNSG32 uIRQENABLE_HDMIIRQ                          :  1;\
            UNSG32 uIRQENABLE_PDMIRQ                           :  1;\
            UNSG32 RSVDx144_b7                                 : 25;\
          }
    union { UNSG32 u32AIO_IRQENABLE;
            struct w32AIO_IRQENABLE;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32AIO_IRQSTS_PRISTS(r32)                      _BFGET_(r32, 0, 0)
    #define   SET32AIO_IRQSTS_PRISTS(r32,v)                    _BFSET_(r32, 0, 0,v)
    #define   GET16AIO_IRQSTS_PRISTS(r16)                      _BFGET_(r16, 0, 0)
    #define   SET16AIO_IRQSTS_PRISTS(r16,v)                    _BFSET_(r16, 0, 0,v)

    #define   GET32AIO_IRQSTS_SECSTS(r32)                      _BFGET_(r32, 1, 1)
    #define   SET32AIO_IRQSTS_SECSTS(r32,v)                    _BFSET_(r32, 1, 1,v)
    #define   GET16AIO_IRQSTS_SECSTS(r16)                      _BFGET_(r16, 1, 1)
    #define   SET16AIO_IRQSTS_SECSTS(r16,v)                    _BFSET_(r16, 1, 1,v)

    #define   GET32AIO_IRQSTS_MIC1STS(r32)                     _BFGET_(r32, 2, 2)
    #define   SET32AIO_IRQSTS_MIC1STS(r32,v)                   _BFSET_(r32, 2, 2,v)
    #define   GET16AIO_IRQSTS_MIC1STS(r16)                     _BFGET_(r16, 2, 2)
    #define   SET16AIO_IRQSTS_MIC1STS(r16,v)                   _BFSET_(r16, 2, 2,v)

    #define   GET32AIO_IRQSTS_MIC2STS(r32)                     _BFGET_(r32, 3, 3)
    #define   SET32AIO_IRQSTS_MIC2STS(r32,v)                   _BFSET_(r32, 3, 3,v)
    #define   GET16AIO_IRQSTS_MIC2STS(r16)                     _BFGET_(r16, 3, 3)
    #define   SET16AIO_IRQSTS_MIC2STS(r16,v)                   _BFSET_(r16, 3, 3,v)

    #define   GET32AIO_IRQSTS_SPDIFSTS(r32)                    _BFGET_(r32, 4, 4)
    #define   SET32AIO_IRQSTS_SPDIFSTS(r32,v)                  _BFSET_(r32, 4, 4,v)
    #define   GET16AIO_IRQSTS_SPDIFSTS(r16)                    _BFGET_(r16, 4, 4)
    #define   SET16AIO_IRQSTS_SPDIFSTS(r16,v)                  _BFSET_(r16, 4, 4,v)

    #define   GET32AIO_IRQSTS_HDMISTS(r32)                     _BFGET_(r32, 5, 5)
    #define   SET32AIO_IRQSTS_HDMISTS(r32,v)                   _BFSET_(r32, 5, 5,v)
    #define   GET16AIO_IRQSTS_HDMISTS(r16)                     _BFGET_(r16, 5, 5)
    #define   SET16AIO_IRQSTS_HDMISTS(r16,v)                   _BFSET_(r16, 5, 5,v)

    #define   GET32AIO_IRQSTS_PDMSTS(r32)                      _BFGET_(r32, 6, 6)
    #define   SET32AIO_IRQSTS_PDMSTS(r32,v)                    _BFSET_(r32, 6, 6,v)
    #define   GET16AIO_IRQSTS_PDMSTS(r16)                      _BFGET_(r16, 6, 6)
    #define   SET16AIO_IRQSTS_PDMSTS(r16,v)                    _BFSET_(r16, 6, 6,v)

    #define     w32AIO_IRQSTS                                  {\
            UNSG32 uIRQSTS_PRISTS                              :  1;\
            UNSG32 uIRQSTS_SECSTS                              :  1;\
            UNSG32 uIRQSTS_MIC1STS                             :  1;\
            UNSG32 uIRQSTS_MIC2STS                             :  1;\
            UNSG32 uIRQSTS_SPDIFSTS                            :  1;\
            UNSG32 uIRQSTS_HDMISTS                             :  1;\
            UNSG32 uIRQSTS_PDMSTS                              :  1;\
            UNSG32 RSVDx148_b7                                 : 25;\
          }
    union { UNSG32 u32AIO_IRQSTS;
            struct w32AIO_IRQSTS;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32AIO_HDSRC_SEL(r32)                          _BFGET_(r32, 1, 0)
    #define   SET32AIO_HDSRC_SEL(r32,v)                        _BFSET_(r32, 1, 0,v)
    #define   GET16AIO_HDSRC_SEL(r16)                          _BFGET_(r16, 1, 0)
    #define   SET16AIO_HDSRC_SEL(r16,v)                        _BFSET_(r16, 1, 0,v)

    #define     w32AIO_HDSRC                                   {\
            UNSG32 uHDSRC_SEL                                  :  2;\
            UNSG32 RSVDx14C_b2                                 : 30;\
          }
    union { UNSG32 u32AIO_HDSRC;
            struct w32AIO_HDSRC;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32AIO_ADAC_CLK_CTRL_mclkDiv(r32)              _BFGET_(r32, 2, 0)
    #define   SET32AIO_ADAC_CLK_CTRL_mclkDiv(r32,v)            _BFSET_(r32, 2, 0,v)
    #define   GET16AIO_ADAC_CLK_CTRL_mclkDiv(r16)              _BFGET_(r16, 2, 0)
    #define   SET16AIO_ADAC_CLK_CTRL_mclkDiv(r16,v)            _BFSET_(r16, 2, 0,v)

    #define   GET32AIO_ADAC_CLK_CTRL_pwmclkDiv(r32)            _BFGET_(r32, 5, 3)
    #define   SET32AIO_ADAC_CLK_CTRL_pwmclkDiv(r32,v)          _BFSET_(r32, 5, 3,v)
    #define   GET16AIO_ADAC_CLK_CTRL_pwmclkDiv(r16)            _BFGET_(r16, 5, 3)
    #define   SET16AIO_ADAC_CLK_CTRL_pwmclkDiv(r16,v)          _BFSET_(r16, 5, 3,v)

    #define   GET32AIO_ADAC_CLK_CTRL_mclkInv(r32)              _BFGET_(r32, 6, 6)
    #define   SET32AIO_ADAC_CLK_CTRL_mclkInv(r32,v)            _BFSET_(r32, 6, 6,v)
    #define   GET16AIO_ADAC_CLK_CTRL_mclkInv(r16)              _BFGET_(r16, 6, 6)
    #define   SET16AIO_ADAC_CLK_CTRL_mclkInv(r16,v)            _BFSET_(r16, 6, 6,v)

    #define   GET32AIO_ADAC_CLK_CTRL_pwmclkInv(r32)            _BFGET_(r32, 7, 7)
    #define   SET32AIO_ADAC_CLK_CTRL_pwmclkInv(r32,v)          _BFSET_(r32, 7, 7,v)
    #define   GET16AIO_ADAC_CLK_CTRL_pwmclkInv(r16)            _BFGET_(r16, 7, 7)
    #define   SET16AIO_ADAC_CLK_CTRL_pwmclkInv(r16,v)          _BFSET_(r16, 7, 7,v)

    #define   GET32AIO_ADAC_CLK_CTRL_inputSel(r32)             _BFGET_(r32, 8, 8)
    #define   SET32AIO_ADAC_CLK_CTRL_inputSel(r32,v)           _BFSET_(r32, 8, 8,v)
    #define   GET16AIO_ADAC_CLK_CTRL_inputSel(r16)             _BFGET_(r16, 8, 8)
    #define   SET16AIO_ADAC_CLK_CTRL_inputSel(r16,v)           _BFSET_(r16, 8, 8,v)

    #define   GET32AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL(r32)       _BFGET_(r32,10, 9)
    #define   SET32AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL(r32,v)     _BFSET_(r32,10, 9,v)
    #define   GET16AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL(r16)       _BFGET_(r16,10, 9)
    #define   SET16AIO_ADAC_CLK_CTRL_PWMCLK_SRC_SEL(r16,v)     _BFSET_(r16,10, 9,v)

    #define   GET32AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL(r32)  _BFGET_(r32,11,11)
    #define   SET32AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL(r32,v) _BFSET_(r32,11,11,v)
    #define   GET16AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL(r16)  _BFGET_(r16,11,11)
    #define   SET16AIO_ADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL(r16,v) _BFSET_(r16,11,11,v)

    #define     w32AIO_ADAC_CLK_CTRL                           {\
            UNSG32 uADAC_CLK_CTRL_mclkDiv                      :  3;\
            UNSG32 uADAC_CLK_CTRL_pwmclkDiv                    :  3;\
            UNSG32 uADAC_CLK_CTRL_mclkInv                      :  1;\
            UNSG32 uADAC_CLK_CTRL_pwmclkInv                    :  1;\
            UNSG32 uADAC_CLK_CTRL_inputSel                     :  1;\
            UNSG32 uADAC_CLK_CTRL_PWMCLK_SRC_SEL               :  2;\
            UNSG32 uADAC_CLK_CTRL_PRIMCLK_OUT_SRC_SEL          :  1;\
            UNSG32 RSVDx150_b12                                : 20;\
          }
    union { UNSG32 u32AIO_ADAC_CLK_CTRL;
            struct w32AIO_ADAC_CLK_CTRL;
          };
    ///////////////////////////////////////////////////////////
    #define   GET32AIO_PDM_MIC_SEL_CTRL(r32)                   _BFGET_(r32, 0, 0)
    #define   SET32AIO_PDM_MIC_SEL_CTRL(r32,v)                 _BFSET_(r32, 0, 0,v)
    #define   GET16AIO_PDM_MIC_SEL_CTRL(r16)                   _BFGET_(r16, 0, 0)
    #define   SET16AIO_PDM_MIC_SEL_CTRL(r16,v)                 _BFSET_(r16, 0, 0,v)

    #define     w32AIO_PDM_MIC_SEL                             {\
            UNSG32 uPDM_MIC_SEL_CTRL                           :  1;\
            UNSG32 RSVDx154_b1                                 : 31;\
          }
    union { UNSG32 u32AIO_PDM_MIC_SEL;
            struct w32AIO_PDM_MIC_SEL;
          };
    ///////////////////////////////////////////////////////////
    } SIE_AIO;

    typedef union  T32AIO_IRQENABLE
          { UNSG32 u32;
            struct w32AIO_IRQENABLE;
                 } T32AIO_IRQENABLE;
    typedef union  T32AIO_IRQSTS
          { UNSG32 u32;
            struct w32AIO_IRQSTS;
                 } T32AIO_IRQSTS;
    typedef union  T32AIO_HDSRC
          { UNSG32 u32;
            struct w32AIO_HDSRC;
                 } T32AIO_HDSRC;
    typedef union  T32AIO_ADAC_CLK_CTRL
          { UNSG32 u32;
            struct w32AIO_ADAC_CLK_CTRL;
                 } T32AIO_ADAC_CLK_CTRL;
    typedef union  T32AIO_PDM_MIC_SEL
          { UNSG32 u32;
            struct w32AIO_PDM_MIC_SEL;
                 } T32AIO_PDM_MIC_SEL;
    ///////////////////////////////////////////////////////////

    typedef union  TAIO_IRQENABLE
          { UNSG32 u32[1];
            struct {
            struct w32AIO_IRQENABLE;
                   };
                 } TAIO_IRQENABLE;
    typedef union  TAIO_IRQSTS
          { UNSG32 u32[1];
            struct {
            struct w32AIO_IRQSTS;
                   };
                 } TAIO_IRQSTS;
    typedef union  TAIO_HDSRC
          { UNSG32 u32[1];
            struct {
            struct w32AIO_HDSRC;
                   };
                 } TAIO_HDSRC;
    typedef union  TAIO_ADAC_CLK_CTRL
          { UNSG32 u32[1];
            struct {
            struct w32AIO_ADAC_CLK_CTRL;
                   };
                 } TAIO_ADAC_CLK_CTRL;
    typedef union  TAIO_PDM_MIC_SEL
          { UNSG32 u32[1];
            struct {
            struct w32AIO_PDM_MIC_SEL;
                   };
                 } TAIO_PDM_MIC_SEL;

    ///////////////////////////////////////////////////////////
     SIGN32 AIO_drvrd(SIE_AIO *p, UNSG32 base, SIGN32 mem, SIGN32 tst);
     SIGN32 AIO_drvwr(SIE_AIO *p, UNSG32 base, SIGN32 mem, SIGN32 tst, UNSG32 *pcmd);
       void AIO_reset(SIE_AIO *p);
     SIGN32 AIO_cmp  (SIE_AIO *p, SIE_AIO *pie, char *pfx, void *hLOG, SIGN32 mem, SIGN32 tst);
    #define AIO_check(p,pie,pfx,hLOG) AIO_cmp(p,pie,pfx,(void*)(hLOG),0,0)
    #define AIO_print(p,    pfx,hLOG) AIO_cmp(p,0,  pfx,(void*)(hLOG),0,0)

#endif
//////
/// ENDOFINTERFACE: AIO
////////////////////////////////////////////////////////////



#ifdef __cplusplus
  }
#endif
#pragma  pack()

#endif
//////
/// ENDOFFILE: aio.h
////////////////////////////////////////////////////////////
