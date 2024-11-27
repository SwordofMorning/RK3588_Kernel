/******************************************************************************
  * @project: LT9211C
  * @file: ModLvdsTx.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  ****************************************************************************/
#include    "../include.h"

#if ((LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)||(LT9211C_MODE_SEL == MIPI_IN_LVDS_OUT)||(LT9211C_MODE_SEL == TTL_IN_LVDS_OUT))

void Mod_LvdsTxPll_RefPixClk_Get(void)
{
    /*lvds to lvds use rxpll pix clk as txpll ref clk*/
    #if (LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)
    g_stRxVidTiming.ulPclk_Khz = Drv_System_FmClkGet(AD_RXPLL_PIX_CLK);
    #endif
    /*mipi to lvds use desscpll pix clk as txpll ref clk*/
    #if (LT9211C_MODE_SEL == MIPI_IN_LVDS_OUT)
    g_stRxVidTiming.ulPclk_Khz = Drv_System_FmClkGet(AD_DESSCPLL_PIX_CLK);
    #endif
    /*ttl to lvds use desscpll pix clk as txpll ref clk*/
    #if (LT9211C_MODE_SEL == TTL_IN_LVDS_OUT)
    g_stRxVidTiming.ulPclk_Khz = Drv_System_FmClkGet(AD_RXPLL_PIX_CLK);
    #endif
}

void Mod_LvdsTxDig_Set(void)
{
    Drv_LvdsTxPort_Set();
    Drv_LvdsTxVidFmt_Set();
    Drv_LvdsTxLaneNum_Set();
    Drv_LvdsTxPort_Swap();
    Drv_LvdsTxPort_Copy();
    Drv_LvdsTxSw_Rst();
   
}

void Mod_LvdsTx_StateJudge(void)
{
    //monitor upstream video stable.
    if(g_stChipTx.ucTxState > STATE_CHIPTX_UPSTREAM_VIDEO_READY)
    {
        if(g_stChipTx.b1UpstreamVideoReady == FALSE)
        {
            Mod_SystemTx_SetState(STATE_CHIPTX_UPSTREAM_VIDEO_READY);
        }
    }
}

void Mod_LvdsTx_StateHandler(void)
{
    switch (g_stChipTx.ucTxState)
    {
        case STATE_CHIPTX_POWER_ON:
            Mod_SystemTx_SetState(STATE_CHIPTX_UPSTREAM_VIDEO_READY);
        break;

        case STATE_CHIPTX_UPSTREAM_VIDEO_READY:
            if(g_stChipTx.b1TxStateChanged == TRUE)
            {
                Drv_LVDSTxPhy_PowerOff();
                g_stChipTx.b1TxStateChanged = FALSE;
            }
        
            if(g_stChipTx.b1UpstreamVideoReady == TRUE) 
            {
                Drv_SystemTxSram_Sel(LVDSTX);
                
                Mod_SystemTx_SetState(STATE_CHIPTX_CONFIG_VIDEO);
            }
        break;

        case STATE_CHIPTX_CONFIG_VIDEO:
            Mod_LvdsTxPll_RefPixClk_Get();
            Drv_LvdsTxPll_RefPixClk_Set();
            Drv_LvdsTxPll_Config();
            if(Drv_LvdsTxPll_Cali() == SUCCESS)
            {
                Mod_SystemTx_SetState(STATE_CHIPTX_VIDEO_OUT);
            }
            else
            {
                Mod_SystemTx_SetState(STATE_CHIPTX_CONFIG_VIDEO);
            }
            
        break;

        case STATE_CHIPTX_VIDEO_OUT:
            Drv_VidChkAll_Get(&g_stVidChk);
            Drv_LvdsTxPhy_Poweron();
            Mod_LvdsTxDig_Set();
            
            Mod_SystemTx_SetState(STATE_CHIPTX_PLAY_BACK);
        break;

        case STATE_CHIPTX_PLAY_BACK:
        break;
        
    }
}

void Mod_LvdsTx_Handler(void)
{
    Mod_LvdsTx_StateJudge();
    Mod_LvdsTx_StateHandler();
}

#endif