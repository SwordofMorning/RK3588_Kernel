#include	"../include.h"

#ifndef		_DRVLVDSTX_H
#define		_DRVLVDSTX_H

#if ((LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)||(LT9211C_MODE_SEL == MIPI_IN_LVDS_OUT)||(LT9211C_MODE_SEL == TTL_IN_LVDS_OUT))

void Drv_LvdsTxSw_Rst(void);
void Drv_LVDSTxPhy_PowerOff(void);
void Drv_LvdsTxPhy_Poweron(void);
void Drv_LvdsTxPll_RefPixClk_Set(void);
void Drv_LvdsTxPll_Config(void);
u8 Drv_LvdsTxPll_Cali(void);
void Drv_LvdsTxPort_Set(void);
void Drv_LvdsTxVidFmt_Set(void);
void Drv_LvdsTxLaneNum_Set(void);
void Drv_LvdsTxPort_Swap(void);
void Drv_LvdsTxPort_Copy(void);


#endif

#endif