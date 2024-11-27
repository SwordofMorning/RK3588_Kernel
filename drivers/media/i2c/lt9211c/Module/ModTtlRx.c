/******************************************************************************
  * @project: LT9211C
  * @file: ModTtlRx.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/
#include    "../include.h"
#if ((LT9211C_MODE_SEL == TTL_IN_LVDS_OUT)||(LT9211C_MODE_SEL == TTL_IN_MIPI_OUT)||(LT9211C_MODE_SEL == TTL_IN_TTL_OUT))

void Mod_TtlRx_DigSet(void)
{
    Drv_TtlRxDdrMode_Set();
    Drv_TtlRxHalfBit_Set();
    Drv_TtlRxDig_Set();
    Drv_TtlRxMapping_Set();
    
}

void Mod_TtlRx_StateHandler(void)
{
    switch (g_stChipRx.ucRxState)
    {
        case STATE_CHIPRX_POWER_ON:
            Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
        break;

        case STATE_CHIPRX_WAIT_SOURCE:
            Drv_TtlRx_PhyPowerOn();
            Drv_TtlRxClk_Sel();
            Drv_SystemActRx_Sel(TTLRX);
            Mod_SystemRx_SetState(STATE_CHIPRX_VIDTIMING_CONFIG);
        break;

        case STATE_CHIPRX_VIDTIMING_CONFIG:
            Mod_TtlRx_DigSet();
            Mod_SystemRx_SetState(STATE_CHIPRX_PLL_CONFIG);
        break;

        case STATE_CHIPRX_PLL_CONFIG:
            if(Drv_TtlRx_Pll_Set() == FAIL)
            {
                Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
            }
            else
            {
                #if TTLRX_SYNC_INTER_MODE == ENABLED
                    Drv_System_VidChkClk_SrcSel(RXPLL_PIX_CLK);
                    Drv_System_VidChk_SrcSel(TTLDEBUG);
                    Drv_TtlRx_SelfTimingSet();
                #endif
                #if TTLRX_DE_SYNC_MODE != TTL_NORMAL_MODE
                    Drv_System_VidChk_SrcSel(TTLRX);
                    Drv_TtlRx_DeSyncModeSet();
                #endif
                Mod_SystemRx_SetState(STATE_CHIPRX_VIDEO_CHECK);
            }
        break;

        case STATE_CHIPRX_VIDEO_CHECK:
            Drv_System_VidChkClk_SrcSel(DESSCPLL_PIX_CLK);
            Drv_System_VidChk_SrcSel(TTLRX);
            Drv_TtlRx_ClkChkDebug();
            Drv_TtlRx_VidChkDebug();
            Drv_Csc_Set();
            g_stChipRx.pHdmiRxNotify(MIPIRX_VIDEO_ON_EVENT);
            Mod_SystemRx_SetState(STATE_CHIPRX_PLAY_BACK);
        break;
        
        case STATE_CHIPRX_PLAY_BACK:
        break;
        
    }
}

void Mod_TtlRx_Handler(void)
{
    Mod_TtlRx_StateHandler();   
}

#endif