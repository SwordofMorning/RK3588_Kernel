#include	"../include.h"

#ifndef		_DRVMIPILS_H
#define		_DRVMIPILS_H

#if (LT9211C_MODE_SEL == MIPI_LEVEL_SHIFT)

typedef enum
{
    MIPILS_4LANE = 0x00,
    MIPILS_1LANE = 0x04,
    MIPILS_2LANE = 0x08,
    MIPILS_3LANE = 0x0c,
}Enum_MIPILSRX_PORTLANE_NUM;


u8 Drv_MipiLsClk_Change(void);

void Drv_MipiLs_ClkSel(void);
void Drv_MipiLsRxPhy_Set(void);
void Drv_MipiLsRxDig_Set(void);
void Drv_MipiLsTxPhy_Set(void);
void Drv_MipiLsTx_PllSet(void);
u8 Drv_MipiLsTx_PllCali(void);
void Drv_MipiLsBta_Set(void);
void Drv_MipiLsClk_Check(void);

#endif

#endif

