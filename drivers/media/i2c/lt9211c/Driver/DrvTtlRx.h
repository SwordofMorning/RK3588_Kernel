


#include	"../include.h"

#ifndef		_DRVTTLRX_H
#define		_DRVTTLRX_H

struct VideoTimingList_ForTtl
{
    u16 usHfp;
    u16 usHs;
    u16 usHbp;
    u16 usHact;
    u16 usHtotal;
    
    u16 usVfp;
    u16 usVs;
    u16 usVbp;
    u16 usVact;
    u16 usVtotal;
    
    u8  ucFrameRate;

};

void Drv_TtlRx_PhyPowerOn(void);
void Drv_TtlRxHalfBit_Set(void);
void Drv_TtlRxDdrMode_Set(void);
void Drv_TtlRxDig_Set(void);
void Drv_TtlRxMapping_Set(void);
void Drv_TtlRxClk_Sel(void);
u8 Drv_TtlRx_Pll_Set(void);
void Drv_TtlRx_SelfTimingSet(void);
void Drv_TtlRx_DeSyncModeSet(void);
void Drv_TtlRx_VidChkDebug(void);
void Drv_TtlRx_ClkChkDebug(void);


#endif