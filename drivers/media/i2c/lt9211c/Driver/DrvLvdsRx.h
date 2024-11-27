#include	"../include.h"

#ifndef		_LVDSRX_H
#define		_LVDSRX_H


void Drv_LvdsRx_PhySet(void);
void Drv_LvdsRx_ClkSel(void);
void Drv_LvdsRx_DataPathSel(void);
void Drv_LvdsRxPort_Set(void);
void Drv_LvdsRxLaneNum_Set(void);
void Drv_LvdsRxPort_Swap(void);
u8 Drv_LvdsRxVidFmt_Set(void);
void Drv_LvdsRx_SelfTimingSet(void);
u8 Drv_LvdsRxPll_Cali(void);
void Drv_LvdsRx_VidChkDebug(void);
void Drv_LvdsRx_DesscPll_Set(void);
void Drv_LvdsRx_DesscDig_Set(void);

#endif