/******************************************************************************
  * @project: LT9211C
  * @file: ModMipiLs.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#include    "../include.h"


#if (LT9211C_MODE_SEL == MIPI_LEVEL_SHIFT)

void Mod_MipiLsClkStb_Judge(void)
{
    if (g_stChipRx.ucRxState > STATE_CHIPRX_VIDEO_CHECK)
    {
        if(Drv_MipiLsClk_Change() == TRUE)
        {
            LTLog(LT_LOG_WARN,"MIPI Repeater Level Shift Rx Clk Change");
            Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
        }
    }    
}


void Mod_MipiLs_StateHandler(void)
{
    switch (g_stChipRx.ucRxState)
    {
        case STATE_CHIPRX_POWER_ON:
            Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
        break;    
        
        case STATE_CHIPRX_WAIT_SOURCE:
            
            Drv_MipiLs_ClkSel();
            Drv_MipiLsRxPhy_Set();
            Drv_MipiLsRxDig_Set();
            Mod_SystemRx_SetState(STATE_CHIPRX_PLL_CONFIG);
        break;
        
        case STATE_CHIPRX_PLL_CONFIG:
            Drv_MipiLsTx_PllSet();
            if (Drv_MipiLsTx_PllCali() == TRUE)
            {
                Mod_SystemRx_SetState(STATE_CHIPRX_VIDEO_CHECK);
            }
            else
            {
                Mod_SystemRx_SetState(STATE_CHIPRX_WAIT_SOURCE);
            }
        break;
            
        case STATE_CHIPRX_VIDEO_CHECK:
            Drv_MipiLsTxPhy_Set();
            Drv_MipiLsBta_Set();
            Mod_SystemRx_SetState(STATE_CHIPRX_PLAY_BACK);
        break;
        
        case STATE_CHIPRX_PLAY_BACK:
            Drv_MipiLsClk_Check();
        break;
    }
}

void Mod_MipiLs_Handler(void)
{
    Mod_MipiLsClkStb_Judge();
    Mod_MipiLs_StateHandler();
}

#endif

