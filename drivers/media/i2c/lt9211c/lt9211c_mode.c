/******************************************************************************
  * @project: LT9211C
  * @file: lt9211c_mode.c
  * @author: sxue
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#include "include.h"

/* lt9211c驱动主流程 */
int lt9211c_main(void *data)
{
    struct mutex *reset_lock = (struct mutex *)data;

    lt9211c_reset();
    LTLog(LT_LOG_INFO, "LT9211C %s %s",__DATE__,__TIME__);
    Mod_ChipID_Read();
    #if (LT9211C_MODE_SEL != PATTERN_OUT)
    Mod_SystemRx_PowerOnInit();
    Mod_SystemTx_PowerOnInit();
    #endif

    while(1)
    {
        /* PATTERN_OUT, MIPI_REPEATER时里面死循环，不加锁，防止死锁 */
        #if (LT9211C_MODE_SEL == PATTERN_OUT)
        Mod_ChipTx_PtnOut();
        #endif
        #if (LT9211C_MODE_SEL == MIPI_REPEATER)
        Mod_MipiRpt_Handler();
        #endif

        mutex_lock(reset_lock);

        #if (LT9211C_MODE_SEL == MIPI_LEVEL_SHIFT)
        Mod_MipiLs_Handler();
        #endif
        #if (LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)
        Mod_LvdsRx_Handler();
        Mod_LvdsTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == LVDS_IN_MIPI_OUT)
        Mod_LvdsRx_Handler();
        Mod_MipiTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == LVDS_IN_TTL_OUT)
        Mod_LvdsRx_Handler();
        Mod_TtlTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == MIPI_IN_LVDS_OUT)
        Mod_MipiRx_Handler();
        Mod_LvdsTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == MIPI_IN_MIPI_OUT)
        Mod_MipiRx_Handler();
        Mod_MipiTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == MIPI_IN_TTL_OUT)
        Mod_MipiRx_Handler();
        Mod_TtlTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == TTL_IN_LVDS_OUT)
        Mod_TtlRx_Handler();
        Mod_LvdsTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == TTL_IN_MIPI_OUT)
        Mod_TtlRx_Handler();
        Mod_MipiTx_Handler();
        #endif
        #if (LT9211C_MODE_SEL == TTL_IN_TTL_OUT)
        Mod_TtlRx_Handler();
        Mod_TtlTx_Handler();
        #endif

        mutex_unlock(reset_lock);
        msleep(1);
    }

    return 0; // 正常不应该走到这里
}