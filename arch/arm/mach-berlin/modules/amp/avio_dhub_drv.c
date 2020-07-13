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

#include "api_avio_dhub.h"

HDL_dhub2d AG_dhubHandle;
HDL_dhub2d VPP_dhubHandle;
#if (BERLIN_CHIP_VERSION >= BERLIN_BG2)
HDL_dhub2d VIP_dhubHandle;
#endif

#if (BERLIN_CHIP_VERSION == BERLIN_BG2CDP)

DHUB_channel_config  VPP_config[VPP_NUM_OF_CHANNELS_Z1] = {
	// BANK0
	{ avioDhubChMap_vpp_MV_R, VPP_DHUB_BANK0_START_ADDR, VPP_DHUB_BANK0_START_ADDR+32, 32, (2048*2-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	{ avioDhubChMap_vpp_MV_FRC_R, VPP_DHUB_BANK0_START_ADDR + 2048*2, VPP_DHUB_BANK0_START_ADDR + 2048*2+32, 32, (2048-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	{ avioDhubChMap_vpp_MV_FRC_W, VPP_DHUB_BANK0_START_ADDR + 2048*3, VPP_DHUB_BANK0_START_ADDR + 2048*3+32, 32, (2048-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	// BANK1
	{ avioDhubChMap_vpp_DINT0_R_Z1,VPP_DHUB_BANK1_START_ADDR, VPP_DHUB_BANK1_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	// BANK2
	{ avioDhubChMap_vpp_DINT1_R_Z1,VPP_DHUB_BANK2_START_ADDR, VPP_DHUB_BANK2_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	// BANK3
	{ avioDhubChMap_vpp_DINT_W_Z1,VPP_DHUB_BANK3_START_ADDR, VPP_DHUB_BANK3_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	// Bank4
	{ avioDhubChMap_vpp_BCM_R_Z1, VPP_DHUB_BANK4_START_ADDR, VPP_DHUB_BANK4_START_ADDR+128, 128, (1024-128), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_vpp_HDMI_R_Z1, VPP_DHUB_BANK4_START_ADDR+1024, VPP_DHUB_BANK4_START_ADDR+1024+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},

#if 0 /* not in use */
	{ avioDhubChMap_vpp_AUX_FRC_R_Z1,DHUB_BANK5_START_ADDR+1024*2, DHUB_BANK5_START_ADDR+1024*2+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	{ avioDhubChMap_vpp_AUX_FRC_W_Z1,DHUB_BANK5_START_ADDR+1024*3, DHUB_BANK5_START_ADDR+1024*3+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	{ avioDhubChMap_vpp_TT_R_Z1,DHUB_BANK5_START_ADDR+1024*4, DHUB_BANK5_START_ADDR+1024*4+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1}
#endif
};

DHUB_channel_config  AG_config[AG_NUM_OF_CHANNELS_Z1] = {
	// Bank0
	{ avioDhubChMap_ag_APPCMD_R_Z1, AG_DHUB_BANK0_START_ADDR, AG_DHUB_BANK0_START_ADDR+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_MA0_R_Z1, AG_DHUB_BANK0_START_ADDR+512, AG_DHUB_BANK0_START_ADDR+512+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_MA1_R_Z1, AG_DHUB_BANK0_START_ADDR+512+1024,AG_DHUB_BANK0_START_ADDR+512+1024+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_MA2_R_Z1, AG_DHUB_BANK0_START_ADDR+512+1024*2,AG_DHUB_BANK0_START_ADDR+512+1024*2+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_MA3_R_Z1, AG_DHUB_BANK0_START_ADDR+512+1024*3,AG_DHUB_BANK0_START_ADDR+512+1024*3+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_SA_R_Z1, AG_DHUB_BANK0_START_ADDR+512+1024*4,AG_DHUB_BANK0_START_ADDR+512+1024*4+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_SPDIF_R_Z1, AG_DHUB_BANK0_START_ADDR+512+1024*5, AG_DHUB_BANK0_START_ADDR+512+1024*5+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_APPDAT_R_Z1, AG_DHUB_BANK0_START_ADDR+512+1024*6, AG_DHUB_BANK0_START_ADDR+512+1024*6+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_APPDAT_W_Z1, AG_DHUB_BANK0_START_ADDR+1024*7, AG_DHUB_BANK0_START_ADDR+1024*7+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	// Bank1
	{ avioDhubChMap_ag_CSR_R_Z1, AG_DHUB_BANK1_START_ADDR, AG_DHUB_BANK1_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
	// Bank2
	{ avioDhubChMap_ag_GFX_R_Z1, AG_DHUB_BANK2_START_ADDR, AG_DHUB_BANK2_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
};

DHUB_channel_config  VPP_config_a0[VPP_NUM_OF_CHANNELS_A0] = {
	// BANK0
        { avioDhubChMap_vpp_MV_R, DHUB_BANK0_START_ADDR, DHUB_BANK0_START_ADDR+32, 32, (2048*2-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
        { avioDhubChMap_vpp_MV_FRC_R, DHUB_BANK0_START_ADDR + 2048*2, DHUB_BANK0_START_ADDR + 2048*2+32, 32, (2048-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
        { avioDhubChMap_vpp_MV_FRC_W, DHUB_BANK0_START_ADDR + 2048*3, DHUB_BANK0_START_ADDR + 2048*3+32, 32, (2048-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        // Bank1
        { avioDhubChMap_vpp_BCM_R_A0, DHUB_BANK1_START_ADDR, DHUB_BANK1_START_ADDR+128, 128, (2048-128), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        { avioDhubChMap_vpp_HDMI_R_A0, DHUB_BANK1_START_ADDR+2048, DHUB_BANK1_START_ADDR+2048+32, 32, (2048-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
};


DHUB_channel_config  AG_config_a0[AG_NUM_OF_CHANNELS_A0] = {
	// Bank0
        { avioDhubChMap_ag_MA0_R_A0, AG_DHUB_BANK0_START_ADDR, AG_DHUB_BANK0_START_ADDR+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        { avioDhubChMap_ag_MA1_R_A0, AG_DHUB_BANK0_START_ADDR+512,AG_DHUB_BANK0_START_ADDR+512+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        { avioDhubChMap_ag_MA2_R_A0, AG_DHUB_BANK0_START_ADDR+512*2,AG_DHUB_BANK0_START_ADDR+512*2+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        { avioDhubChMap_ag_MA3_R_A0, AG_DHUB_BANK0_START_ADDR+512*3,AG_DHUB_BANK0_START_ADDR+512*3+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        { avioDhubChMap_ag_SA0_R_A0, AG_DHUB_BANK0_START_ADDR+512*4,AG_DHUB_BANK0_START_ADDR+512*4+32, 32, (1024-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        { avioDhubChMap_ag_MIC0_W_A0, AG_DHUB_BANK0_START_ADDR+1024*3, AG_DHUB_BANK0_START_ADDR+1024*3+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
	{ avioDhubChMap_ag_MIC1_W_A0, AG_DHUB_BANK0_START_ADDR+512+1024*3, AG_DHUB_BANK0_START_ADDR+512+1024*3+32, 32, (512-32), dHubChannel_CFG_MTU_128byte, 0, 0, 1},
        // Bank1
        { avioDhubChMap_ag_CSR_R_A0, AG_DHUB_BANK1_START_ADDR, AG_DHUB_BANK1_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
        // Bank2
        { avioDhubChMap_ag_GFX_R_A0, AG_DHUB_BANK2_START_ADDR, AG_DHUB_BANK2_START_ADDR+32, 32, (8192-32), dHubChannel_CFG_MTU_128byte, 1, 0, 1},
};

#else /* only BG2CDP is supported */
#error
#endif

/******************************************************************************************************************
 *      Function: GetChannelInfo
 *      Description: Get the Dhub configuration of requested channel.
 *      Parameter : pdhubHandle ----- pointer to 2D dhubHandle 
 *             IChannel		----- Channel of the dhub 
 *             cfg		----- Configuration need to be updated here.
 *      Return:  0 	----	Success       
******************************************************************************************************************/
#if (BERLIN_CHIP_VERSION >= BERLIN_BG2_CT)
int getDhubChannelInfo(HDL_dhub2d *pdhubHandle, SIGN32 IChannel, T32dHubChannel_CFG *cfg)
{
       DHUB_channel_config *dhub_config;
		//Get the Dhub Config array from the received handle
       if(pdhubHandle == &VPP_dhubHandle)
       {
               dhub_config = VPP_config;
       }
       if(pdhubHandle == &AG_dhubHandle)
       {
               dhub_config = AG_config;
       }
		//Update the MTU, QOS and self loop paramteres.
       cfg->uCFG_MTU = dhub_config[IChannel].chanMtuSize;
       cfg->uCFG_QoS =  dhub_config[IChannel].chanQos;
       cfg->uCFG_selfLoop = dhub_config[IChannel].chanSelfLoop;

       return 0;
}
#endif

/******************************************************************************************************************
 *	Function: DhubInitialization 
 *	Description: Initialize DHUB .
 *	Parameter : cpuId ------------- cpu ID
 *             dHubBaseAddr -------------  dHub Base address.
 *             hboSramAddr ----- Sram Address for HBO.
 *             pdhubHandle ----- pointer to 2D dhubHandle 
 *             dhub_config ----- configuration of AG 
 *             numOfChans	 ----- number of channels
 *	Return:		void
******************************************************************************************************************/
void DhubInitialization(SIGN32 cpuId, UNSG32 dHubBaseAddr, UNSG32 hboSramAddr, HDL_dhub2d *pdhubHandle, DHUB_channel_config *dhub_config,SIGN32 numOfChans)
{
	HDL_semaphore *pSemHandle;
	SIGN32 i;
	SIGN32 chanId;
	
	//Initialize HDL_dhub with a $dHub BIU instance.
	dhub2d_hdl(	hboSramAddr,			/*!	Base address of dHub.HBO SRAM !*/
			 	dHubBaseAddr,			/*!	Base address of a BIU instance of $dHub !*/
			 	pdhubHandle				/*!	Handle to HDL_dhub2d !*/
			);
	//set up semaphore to trigger cmd done interrupt
	//note that this set of semaphores are different from the HBO semaphores
	//the ID must match the dhub ID because they are hardwired.
	pSemHandle = dhub_semaphore(&pdhubHandle->dhub);

	for (i = 0; i< numOfChans; i++) {
		//Configurate a dHub channel
		//note that in this function, it also configured right HBO channels(cmdQ and dataQ) and semaphores
		chanId = dhub_config[i].chanId;	
		{
			dhub_channel_cfg(
						&pdhubHandle->dhub,					/*!	Handle to HDL_dhub !*/
						chanId,						/*!	Channel ID in $dHubReg !*/
						dhub_config[chanId].chanCmdBase,		//UNSG32 baseCmd,	/*!	Channel FIFO base address (byte address) for cmdQ !*/
						dhub_config[chanId].chanDataBase,		//UNSG32 baseData,	/*!	Channel FIFO base address (byte address) for dataQ !*/
						dhub_config[chanId].chanCmdSize/8,	//SIGN32		depthCmd,			/*!	Channel FIFO depth for cmdQ, in 64b word !*/
						dhub_config[chanId].chanDataSize/8,	//SIGN32		depthData,			/*!	Channel FIFO depth for dataQ, in 64b word !*/
						dhub_config[chanId].chanMtuSize,						/*!	See 'dHubChannel.CFG.MTU', 0/1/2 for 8/32/128 bytes !*/
						dhub_config[chanId].chanQos,								/*!	See 'dHubChannel.CFG.QoS' !*/
						dhub_config[chanId].chanSelfLoop,								/*!	See 'dHubChannel.CFG.selfLoop' !*/
						dhub_config[chanId].chanEnable,								/*!	0 to disable, 1 to enable !*/
						0								/*!	Pass NULL to directly init dHub, or
															Pass non-zero to receive programming sequence
															in (adr,data) pairs
															!*/
						);
			// setup interrupt for channel chanId
			//configure the semaphore depth to be 1
			semaphore_cfg(pSemHandle, chanId, 1, 0);
#if 0
  			// enable interrupt from this semaphore
		  	semaphore_intr_enable (
    			pSemHandle, // semaphore handler
		    	chanId, 
    			0,       // empty
		    	1,       // full
    			0,       // almost_empty
		    	0,       // almost_full
    			cpuId        // 0~2, depending on which CPU the interrupt is enabled for.
		  	);
#endif
		}
	}
}

/******************************************************************************************************************
 *	Function: DhubChannelClear
 *	Description: Clear corresponding DHUB channel.
 *	Parameter:	hdl  ---------- handle to HDL_dhub
 *			id   ---------- channel ID in dHubReg
 *			cfgQ ---------- pass null to directly init dhub, or pass non-zero to receive programming
 *					sequence in (adr, data) pairs
 *	Return:		void
******************************************************************************************************************/
void DhubChannelClear(void *hdl, SIGN32 id, T64b cfgQ[])
{
	UNSG32	cmdID = dhub_id2hbo_cmdQ(id);
	UNSG32	dataID = dhub_id2hbo_data(id);
	HDL_dhub *dhub = (HDL_dhub *)hdl;
	HDL_hbo *hbo = &(dhub->hbo);

	/* 1.Software stops the command queue in HBO (please refer to HBO.sxw for details) */
	hbo_queue_enable(hbo, cmdID, 0, cfgQ);
	/* 2.Software stops the channel in dHub by writing zero to  dHubChannel.START.EN */
	dhub_channel_enable(dhub, id, 0, cfgQ);
	/* 3.Software clears the channel in dHub by writing one to  dHubChannel.CLEAR.EN */
	dhub_channel_clear(dhub, id);
	/* 4.Software waits for the register bits dHubChannel.PENDING.ST and dHubChannel.BUSY.ST to be 0 */
	dhub_channel_clear_done(dhub, id);
	/* 5.Software stops and clears the command queue */
	hbo_queue_enable(hbo, cmdID, 0, cfgQ);
	hbo_queue_clear(hbo, cmdID);
	/* 6.Software wait for the corresponding busy bit to be 0 */
	hbo_queue_clear_done(hbo, cmdID);
	/* 7.Software stops and clears the data queue */
	hbo_queue_enable(hbo, dataID, 0, cfgQ);
	hbo_queue_clear(hbo, dataID);
	/* 8.Software wait for the corresponding data Q busy bit to be 0 */
	hbo_queue_clear_done(hbo, dataID);
	/* 9.Software enable dHub and HBO */
	dhub_channel_enable(dhub, id, 1, cfgQ);
	hbo_queue_enable(hbo, cmdID, 1, cfgQ);
	hbo_queue_enable(hbo, dataID, 1, cfgQ);
}
