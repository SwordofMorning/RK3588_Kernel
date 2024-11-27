#include	"../include.h"

#ifndef		_DRVMIPIRPT_H
#define		_DRVMIPIRPT_H

#if (LT9211C_MODE_SEL == MIPI_REPEATER)
















typedef enum
{
    MIPIRPT_4LANE = 0x00,
    MIPIRPT_1LANE = 0x04,
    MIPIRPT_2LANE = 0x08,
    MIPIRPT_3LANE = 0x0c,
}Enum_MIPIRPTRX_PORTLANE_NUM;


u8 Drv_MipiRptClk_Change(void);
void Drv_MipiRpt_ClkSel(void);
void Drv_MipiRptTx_SkewCali(void);
void Drv_MipiRptRxPhy_Set(void);
void Drv_MipiRptRxDig_Set(void);
void Drv_MipiRptTxPhy_Set(void);
u8 Drv_MipiRptRx_ByteClkGet(void);
u8 Drv_MipiRptTx_PllSet(void);
u8 Drv_MipiRptTx_PllCali(void);
void Drv_MipiRptBta_Set(void);

#endif

#endif

