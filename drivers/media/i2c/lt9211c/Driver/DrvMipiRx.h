

#include	"../include.h"

#ifndef		_DRVMIPIRX_H
#define		_DRVMIPIRX_H

//MIPI RX Videotiming Debug
typedef struct VideoTiming_Get
{   
    u16 uswc;
    u16 usHact;
    u16 usVact;
    u8 ucFmt;
    u8 ucPa_Lpn;
    u8 ucFrameRate;
    u8 ucLane0SetNum;
    u8 ucLane1SetNum;
    u8 ucLane2SetNum;
    u8 ucLane3SetNum;
    u8 ucLane0SotData;
    u8 ucLane1SotData;
    u8 ucLane2SotData;
    u8 ucLane3SotData;
}SrtuctMipiRx_VidTiming_Get;

struct VideoTimingList
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


typedef enum
{
    MIPIRX_4LANE = 0x00,
    MIPIRX_1LANE = 0x01,
    MIPIRX_2LANE = 0x02,
    MIPIRX_3LANE = 0x03,
}Enum_MIPIRXPORTLANE_NUM;

extern StructPcrPara g_stPcrPara;
extern SrtuctMipiRx_VidTiming_Get g_stMipiRxVidTiming_Get;
void DRV_DesscPll_SdmCal(void);
u8 Drv_MipiRx_PcrCali(void);
void Drv_MipiRx_DesscPll_Set(void);
void Drv_MipiRx_VidTiming_Set(void);
u8 Drv_MipiRx_VidFmtUpdate(void);
u8 Drv_MipiRx_VidFmt_Get(IN u8 VidFmt);
u8 Drv_MipiRx_VidTiming_Get(void);
u8 Drv_MipiRx_VidTiming_Sel(void);
void Drv_MipiRx_InputSel(void);
void Drv_MipiRx_LaneSet(void);
void Drv_MipiRxClk_Sel(void);
void Drv_MipiRx_PhyPowerOn(void);
void Drv_MipiRx_VidChkDebug(void);
#endif