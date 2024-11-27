/******************************************************************************
  * @project: LT9211C
  * @file: ModLvdsRx.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/
#include    "../include.h"
#if ((LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)||(LT9211C_MODE_SEL == LVDS_IN_MIPI_OUT)||(LT9211C_MODE_SEL == LVDS_IN_TTL_OUT))

void Mod_LvdsRxDig_Set(void)
{
    LTLog(LT_LOG_INFO,"LVDS RX Config");
    Drv_LvdsRxPort_Set();
    g_stChipRx.ucRxFormat = Drv_LvdsRxVidFmt_Set();
    Drv_LvdsRxLaneNum_Set();
    Drv_LvdsRxPort_Swap();
#if ((LVDSRX_SYNC_INTER_MODE == ENABLED) || (LVDSRX_MODE == DE_MODE))
    Drv_LvdsRx_SelfTimingSet();
#endif

 
}


void Mod_LvdsRx_StateHandler(void)
{
    switch (g_stChipRx.ucRxState)
    {
        case STATE_CHIPRX_POWER_ON:
           
            Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
        break;

        case STATE_CHIPRX_WAIT_SOURCE:
            Drv_LvdsRx_PhySet();
            Drv_LvdsRx_ClkSel();
            Drv_SystemActRx_Sel(LVDSRX);
            Drv_LvdsRx_DataPathSel();
            Mod_SystemRx_SetState(STATE_CHIPRX_VIDTIMING_CONFIG);
        break;

        case STATE_CHIPRX_VIDTIMING_CONFIG:
            Mod_LvdsRxDig_Set();
            Mod_SystemRx_SetState(STATE_CHIPRX_PLL_CONFIG);
        break;

        case STATE_CHIPRX_PLL_CONFIG:
            Mod_SystemRx_SetState(STATE_CHIPRX_VIDEO_CHECK);
            if(Drv_LvdsRxPll_Cali() == FAIL)
            {
                Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
            }
            #if LVDSRX_DESSC_SEL == ENABLED
            LTLog(LT_LOG_INFO,"dessc mode");
            Drv_LvdsRx_DesscPll_Set();
            Drv_LvdsRx_DesscDig_Set();
            #endif
        break;

        case STATE_CHIPRX_VIDEO_CHECK:
            #if LVDSRX_DESSC_SEL == ENABLED
            Drv_System_VidChkClk_SrcSel(DESSCPLL_PIX_CLK);
            #else
            Drv_System_VidChkClk_SrcSel(RXPLL_PIX_CLK);
            #endif
            Drv_System_VidChk_SrcSel(LVDSRX);
            Drv_LvdsRx_VidChkDebug();
            Drv_Csc_Set();
           g_stChipRx.pHdmiRxNotify(MIPIRX_VIDEO_ON_EVENT);
            Mod_SystemRx_SetState(STATE_CHIPRX_PLAY_BACK);
        break;
        
        case STATE_CHIPRX_PLAY_BACK:
        break;
        
    }
}


void Mod_LvdsRx_Handler(void)
{
    Mod_LvdsRx_StateHandler();
}


#endif