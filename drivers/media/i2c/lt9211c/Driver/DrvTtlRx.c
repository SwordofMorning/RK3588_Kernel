/******************************************************************************
  * @project: LT9211C
  * @file: DrvTtlRx.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  ****************************************************************************/

#include "../include.h"
#if ((LT9211C_MODE_SEL == TTL_IN_LVDS_OUT)||(LT9211C_MODE_SEL == TTL_IN_MIPI_OUT)||(LT9211C_MODE_SEL == TTL_IN_TTL_OUT))

#if (TTLRX_SYNC_INTER_MODE == ENABLED) || (TTLRX_DE_SYNC_MODE != TTL_NORMAL_MODE)
#if TTLRX_VIDEO_FORMAT == P_FORMAT
//hfp    hs     hbp     hact     htotal   vfp   vs   vbp   vact    vtotal  framerate
struct VideoTimingList_ForTtl resolution[]  = {
 {24,  96,  40,   640,    800,    33,  2,   10,  480,  525,  60/*25000*/},  //video_640x480_60Hz
 {160, 20, 140,1024,  1344,  12,  3,  20,  600, 635,   60},  //video_1024x600_60Hz
 {16,    62,    60,     720,     858,     9,    6,   30,   480,    525,   60},  //video_720x480_60Hz
 {12,    64,    88,     720,     864,     5,    5,   39,   576,    625,   50},  //video_720x576_50Hz
 {48,    128,   88,     800,     1056,    1,    4,   23,   600,    628,   60},  //video_800x600_60Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   60},  //video_1280x720_60Hz
 {24,    136,   160,    1024,    1344,    3,    6,   29,   768,    806,   60},  //video_1024x768_60Hz
 {26,    110,   110,    1366,    1592,    13,   6,   13,   768,    800,   60},  //video_1366x768_60Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   30},  //video_1280x720_30Hz
 {48,    32,    80,     1920,    2080,    5,    5,   20,   720,    750,   60},  //video_1920x720_60Hz
 {48,    112,   248,    1280,    1688,    1,    3,   38,   1024,   1066,  60},  //video_1028x1024_60Hz
 {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,  60},  //video_1920x1080_60Hz
 {64,    192,   304,    1600,    2160,    1,    3,   46,   1200,   1250,  60},  //video_1600x1200_60Hz
 {48,    32,    80,     1920,    2080,    3,    6,   26,   1200,   1235,  60},  //video_1920x1200_60Hz
 {176,   88,    296,    3840,    4400,    8,    10,  72,   2160,   2250,  30},  //video_3840x2160_30Hz
 };
#elif TTLRX_VIDEO_FORMAT == I_FORMAT
 //hfp    hs     hbp     hact     htotal   vfp   vs   vbp   vact    vtotal  framerate
struct VideoTimingList_ForTtl resolution[]  = {
 {88,    44,    148,    1920,    2200,    4,    10,  30,   1080,   1125,  60},
 };
#endif
#endif
 
void Drv_TtlRx_PhyPowerOn(void)
{
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x01,0x22); //TTL enable
    HDMI_WriteI2C_Byte(0x60,0x0a); //tx phy at TtlRx phy
    HDMI_WriteI2C_Byte(0x61,0x23);
    HDMI_WriteI2C_Byte(0x61,0x33);
    HDMI_WriteI2C_Byte(0x62,0x08);
    
}

void Drv_TtlRxHalfBit_Set(void)
{
    #if   ((TTLRX_INPUT_MODE == INPUT_YCBCR422_8BIT) || (TTLRX_INPUT_MODE == INPUT_YCBCR422_10BIT) || (TTLRX_INPUT_MODE == INPUT_YCBCR422_12BIT))
        #define     TTLRX_HALF_BIT_MODE         ENABLED
    #elif ((TTLRX_INPUT_MODE == INPUT_BT656_8BIT)    || (TTLRX_INPUT_MODE == INPUT_BT656_10BIT)    || (TTLRX_INPUT_MODE == INPUT_BT656_12BIT))
        #define     TTLRX_HALF_BIT_MODE         ENABLED
    #elif ((TTLRX_INPUT_MODE == INPUT_BT1120_8BIT)   || (TTLRX_INPUT_MODE == INPUT_BT1120_10BIT)   || (TTLRX_INPUT_MODE == INPUT_BT1120_12BIT))
        #define     TTLRX_HALF_BIT_MODE         ENABLED
    #else
        #define     TTLRX_HALF_BIT_MODE         DISABLED
    #endif
}

void Drv_TtlRxDdrMode_Set(void)
{
    #if TTLRX_MODE == SDR
        #define     TTLRX_DATARATE_MODE         SDR
        #define     TTLRX_HALF_CLOCK_MODE       DISABLED
        #define     TTLRX_HALF_WIDTH_MODE       DISABLED
    #elif TTLRX_MODE == DDR
        #define     TTLRX_DATARATE_MODE         DDR
        #define     TTLRX_HALF_CLOCK_MODE       ENABLED
        #define     TTLRX_HALF_WIDTH_MODE       DISABLED
    #elif TTLRX_MODE == DPI
        #define     TTLRX_DATARATE_MODE         DDR
        #define     TTLRX_HALF_CLOCK_MODE       DISABLED
        #define     TTLRX_HALF_WIDTH_MODE       ENABLED
    #endif
}

void Drv_TtlRxDig_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0xc1,0x00);
    HDMI_WriteI2C_Byte(0xc4,0x00); 
    HDMI_WriteI2C_Byte(0xc5,0x00);
    HDMI_WriteI2C_Byte(0xc6,0x00);     
    HDMI_WriteI2C_Byte(0xc7,0x07);
    HDMI_WriteI2C_Byte(0xc8,0x04);
    HDMI_WriteI2C_Byte(0xe8,0x00); 
    

    //ttlrx sync mode set
    #if TTLRX_SYNC_INTER_MODE == ENABLED
    
        HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x28); //embedded sync mode enable
        LTLog(LT_LOG_INFO,"Sync Mode:    Embedded");
        
        //embedded video format set
        #if TTLRX_VIDEO_FORMAT == I_FORMAT
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x80); //Interlace format
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x10); //rgd_ttlrx_bt_sync_gen_en
            LTLog(LT_LOG_INFO,"Video Format:    I Format");
        #else
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) & 0x7f); //Progressive format
            LTLog(LT_LOG_INFO,"Video Format:    P Format");
        #endif
        
        //embedded Half Bit Mode set
        #if TTLRX_HALF_BIT_MODE == ENABLED
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x40); //8Bit embedded enable
        #endif     
    #endif

    
    #if TTLRX_DATARATE_MODE == DDR
        #if TTLRX_HALF_WIDTH_MODE == ENABLED
            HDMI_WriteI2C_Byte(0xc1,(HDMI_ReadI2C_Byte(0xc1) | 0x80)); //ddr half width enable
            HDMI_WriteI2C_Byte(0xc0,(HDMI_ReadI2C_Byte(0xc0) | 0x08)); //fifo_w_en
            LTLog(LT_LOG_INFO,"DDR Half Width Mode:    Enable");
            //DDR Trans Mode set
            #if 0
                HDMI_WriteI2C_Byte(0xc1,HDMI_ReadI2C_Byte(0xc1) | 0x40); //high 12 Bit first
                LTLog(LT_LOG_INFO,"DDR Trans Mode:    High 12Bit First");
            #else
                HDMI_WriteI2C_Byte(0xc1,HDMI_ReadI2C_Byte(0xc1) & 0xbf); //low 12 Bit first
                LTLog(LT_LOG_INFO,"DDR Trans Mode:    Low 12Bit First");
            #endif
            //DDR Input Mode set
            #if 1
                HDMI_WriteI2C_Byte(0xc1,HDMI_ReadI2C_Byte(0xc1) | 0x0d);
                LTLog(LT_LOG_INFO,"DDR Input Mode:    12Bit Mode");
            #endif
        #endif
    #endif
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x0d,0x7e); //ttl rx reset
    HDMI_WriteI2C_Byte(0x0d,0x7f); //ttl rx release
}

void Drv_TtlRxMapping_Set(void)
{
    u8 ucRxVidFmt;
    
    HDMI_WriteI2C_Byte(0xff,0x85);
    
    switch(TTLRX_INPUT_MODE)
    {
        case INPUT_RGB888:
            //RGB swap case
            #if 0   //input RGB  R: D23->D16    G: D15->D8    B:D7->D0    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input RBG  R: D23->D16    B: D15->D8    G:D7->D0    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input GRB  G: D23->D16    R: D15->D8    B:D7->D0    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input GBR  G: D23->D16    B: D15->D8    R:D7->D0    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input BRG  B: D23->D16    R: D15->D8    G:D7->D0    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 1   //input BGR  B: D23->D16    G: D15->D8    R:D7->D0    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input BGR  B: D23->D16    G: D15->D8    R:D0->D7    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input BGR  B: D23->D16    G: D8->D15    R:D7->D0    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input BGR  B: D16->D23    G: D15->D8    R:D7->D0    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            ucRxVidFmt = RGB_8Bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    RGB888");
            break;
            
        case INPUT_RGB666:
            //RGB swap case
            #if 1   //input RGB  R: D23->D18    G: D15->D10    B:D7->D2    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input RBG  R: D23->D18    B: D15->D10    G:D7->D2    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input GRB  G: D23->D18    R: D15->D10    B:D7->D2    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input GBR  G: D23->D18    B: D15->D10    R:D7->D2    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input BRG  B: D23->D18    R: D15->D10    G:D7->D2    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input BGR  B: D23->D18    G: D15->D10    R:D7->D2    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input BGR  B: D23->D18    G: D15->D10    R:D0->D5    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input BGR  B: D23->D18    G: D8->D13    R:D7->D2    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input BGR  B: D16->D21    G: D15->D10    R:D7->D2    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            //RGB666 swap case
            #if 0   //input BGR  B: D17->D12    G: D11->D6    R:D5->D0    //8619C output  [606d]=a0 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x05);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0   //input BGR  B: D23->D18    G: D17->D12    R:D11->D6    //8619C output  [606d]=80 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x04);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            ucRxVidFmt = RGB_6Bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    RGB666");
            break;
            
        case INPUT_RGB565:
            //RGB swap case
            #if 0   //input RGB  R: D23->D19    G: D15->D10    B:D7->D3    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input RBG  R: D23->D19    B: D15->D11    G:D7->D2    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input GRB  G: D23->D18    R: D15->D11    B:D7->D3    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input GBR  G: D23->D18    B: D15->D11    R:D7->D3    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input BRG  B: D23->D19    R: D15->D11    G:D7->D2    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input BGR  B: D23->D19    G: D15->D10    R:D7->D3    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input BGR  B: D23->D19    G: D15->D10    R:D0->D4    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input BGR  B: D23->D19    G: D8->D13    R:D7->D3    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input BGR  B: D16->D20    G: D15->D10    R:D7->D3    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            //RGB565 case
            #if 1   //input RGB  B: D15->D11    G: D10->D5    R: D4->D0    //8619C output  [606d]=e0 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x07);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0   //input RGB  B: D23->D19    G: D18->D13    R: D12->D8    //8619C output  [606d]=c0 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x06);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            ucRxVidFmt = RGB_6Bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    RGB565");
            break;
        
        case INPUT_YCBCR444:
            //RGB swap case
            #if 1   //input YUV  V: D23->D16    Y: D15->D8    U:D7->D0    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input YUV  V: D23->D16    U: D15->D8    Y:D7->D0    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input YUV  Y: D23->D16    V: D15->D8    U:D7->D0    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input YUV  Y: D23->D16    U: D15->D8    V:D7->D0    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input YUV  U: D23->D16    V: D15->D8    Y:D7->D0    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input YUV  U: D23->D16    Y: D15->D8    V:D7->D0    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input YUV  U: D23->D16    Y: D15->D8    V:D0->D7    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input YUV  U: D23->D16    Y: D8->D15    V:D7->D0    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input YUV  U: D16->D23    Y: D15->D8    V:D7->D0    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            ucRxVidFmt = YUV444_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR444");
            break;
        
        case INPUT_YCBCR422_8BIT:
            ucRxVidFmt = YUV422_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR422_8BIT");
            break;
        
        case INPUT_YCBCR422_10BIT:
            ucRxVidFmt = YUV422_10bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR422_10BIT");
            break;
        
        case INPUT_YCBCR422_12BIT:
            ucRxVidFmt = YUV422_12bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR422_12BIT");
            break;
        
        case INPUT_YCBCR422_16BIT:
            //RGB swap case
            #if 1   //input YUV  C: D23->D16    Y: D15->D8    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input YUV  C: D23->D16    Y:D7->D0    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input YUV  Y: D23->D16    C: D15->D8    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input YUV  Y: D23->D16    C:D7->D0    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input YUV  C: D15->D8    Y:D7->D0    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input YUV  Y: D15->D8    C:D7->D0    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input YUV  Y: D15->D8    C:D0->D7    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input YUV  Y: D8->D15    C:D7->D0    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            ucRxVidFmt = YUV422_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR422_16BIT");
            break;
        
        case INPUT_YCBCR422_20BIT:
            //20Bit swap case
            #if 1  //input YUV  Y: D23->D14    C:D13->D4    //8619C output  [606d]=40 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x02);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0  //input YUV  Y: D19->D10    C:D9->D0    //8619C output  [606d]=60 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x03);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            //RGB swap case
            #if 0  //input YUV  Y: D15->D8  D19->D18    C:D8->D0  D23->D22    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D23->D16  D3->D2    C:D15->D6    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_10bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR422_20BIT");
            break;
        
        case INPUT_YCBCR422_24BIT:
            //24Bit swap case
            #if 1  //input YUV  Y: D23->D12    C:D11->D0    //8619C output  [606d]=20 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0  //input YUV  C: D23->D12    Y:D11->D0    //8619C output  [606d]=28 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            //RGB swap case
            #if 0  //input YUV  Y: D15->D8  D19->D16    C:D7->D0  D23->D20    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D23->D16  D3->D0    C:D15->D4    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_12bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    YCBCR422_24BIT");
            break;
        
        case INPUT_BT656_8BIT:
            
            HDMI_WriteI2C_Byte(0xc9,HDMI_ReadI2C_Byte(0xc9) | 0x06);  
            
            //RGB swap case
            #if 0  //input YUV  D7-D0    //8619C output  low 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 1  //input YUV  D15-D8    //8619C output middle 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0  //input YUV  D23-D16    //8619C output  high 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT656_8BIT");
            break;
        
        case INPUT_BT656_10BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  D9-D0    //8619C output  low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  D13-D4    //8619C output middle-low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            #if 0  //input YUV  D19-D10    //8619C output  middle-high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  D23-D14    //8619C output  high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            //other case
            #if 1  //input YUV  D15-D6    
                HDMI_WriteI2C_Byte(0xc6,0x01);
            #endif
            #if 0  //input YUV  D11-D2 
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_10bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT656_10BIT");
            break;
        
        case INPUT_BT656_12BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case
            #if 0  //input YUV  D11-D0    //8619C output  low 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0  //input YUV  D23-D12   //8619C output  high 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            //other case
            #if 1  //input YUV  D15-D4    
                HDMI_WriteI2C_Byte(0xc6,0x08);
            #endif
            ucRxVidFmt = YUV422_12bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT656_12BIT");
            break;
        
        case INPUT_BT1120_8BIT:
            //RGB swap case
            #if 0  //input YUV  D7-D0    //8619C output  low 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 1  //input YUV  D15-D8    //8619C output middle 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0  //input YUV  D23-D16    //8619C output  high 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT1120_8BIT");
            break;
        
        case INPUT_BT1120_10BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  D9-D0    //8619C output  low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  D13-D4    //9211C output  //8619C output middle-low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            #if 0  //input YUV  D19-D10    //8619C output  middle-high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  D23-D14    //8619C output  high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            //other case
            #if 1  //input YUV  D15-D6    
                HDMI_WriteI2C_Byte(0xc6,0x01);
            #endif
            #if 0  //input YUV  D11-D2 
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_10bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT1120_10BIT");
            break;
        
        case INPUT_BT1120_12BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case
            #if 0  //input YUV  D11-D0    //8619C output  low 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0  //input YUV  D23-D12   //8619C output  high 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            //other case
            #if 1  //input YUV  D15-D4    
                HDMI_WriteI2C_Byte(0xc6,0x08);
            #endif
            ucRxVidFmt = YUV422_12bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT1120_12BIT");
            break;
        
        case INPUT_BTA_T1004_16BIT:
              HDMI_WriteI2C_Byte(0xc9,HDMI_ReadI2C_Byte(0xc9) | 0x07);           

        //RGB swap case

            #if 0  //input YUV  C: D7-D0    Y: D15-D8    //8619C /YC no swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D7-D0    C: D15-D8    //8619C /YC swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0  //input YUV  Y: D23-D16    C: D15-D8    //8619C /YC no swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 1  //input YUV  C: D23-D16    Y: D15-D8    //8619C /YC swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            //other case
            #if 0  //input YUV  C: D23-D16    Y: D11-D4    
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            ucRxVidFmt = YUV422_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BTA_T1004_16BIT");
            break;
        
        case INPUT_BTA_T1004_20BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  C: D9-D0    Y: D19-D10    //8619C /YC no swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  Y: D9-D0    C: D19-D10    //8619C /YC swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  Y: D23-D14    C: D13-D4    //8619C /YC no swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            #if 0  //input YUV  C: D23-D14    Y: D13-D4    //8619C /YC swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            //other case
            #if 1 //input YUV  C: D23-D14    Y: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0 //input YUV  Y: D23-D14    C: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            ucRxVidFmt = YUV422_10bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BTA_T1004_20BIT");
            break;
        
        case INPUT_BTA_T1004_24BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case 
            #if 0 //input YUV  Y: D23-D12    C: D11-D0    //8619C /YC no swap output
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            #if 1 //input YUV  C: D23-D12    Y: D11-D0    //8619C /YC swap output
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_12bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BTA_T1004_24BIT");
            break;
        
        case INPUT_BT1120_16BIT: 
            
        HDMI_WriteI2C_Byte(0xc9,HDMI_ReadI2C_Byte(0xc9) | 0x05);  
            //RGB swap case
            #if 0  //input YUV  C: D7-D0    Y: D15-D8    //8619C /YC no swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D7-D0    C: D15-D8    //8619C /YC swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0  //input YUV  Y: D23-D16    C: D15-D8    //8619C /YC no swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 1  //input YUV  C: D23-D16    Y: D15-D8    //8619C /YC swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            //other case
            #if 0  //input YUV  C: D23-D16    Y: D11-D4    
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            ucRxVidFmt = YUV422_8bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT1120_16BIT");
            break;
        
        case INPUT_BT1120_20BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  C: D9-D0    Y: D19-D10    //8619C /YC no swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  Y: D9-D0    C: D19-D10    //8619C /YC swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  Y: D23-D14    C: D13-D4    //8619C /YC no swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            #if 0  //input YUV  C: D23-D14    Y: D13-D4    //8619C /YC swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            //other case
            #if 1 //input YUV  C: D23-D14    Y: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0 //input YUV  Y: D23-D14    C: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            ucRxVidFmt = YUV422_10bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT1120_20BIT");
            break;
        
        case INPUT_BT1120_24BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case 
            #if 0 //input YUV  Y: D23-D12    C: D11-D0    //8619C /YC no swap output
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            #if 1 //input YUV  C: D23-D12    Y: D11-D0    //8619C /YC swap output
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_12bit;
            LTLog(LT_LOG_INFO,"TTLRx Input Mode:    BT1120_24BIT");
            break;
    }
    
    g_stChipRx.ucRxFormat = ucRxVidFmt;
    
}



void Drv_TtlRxClk_Sel(void)
{
        /* CLK sel */
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0xe9,0x88); //sys clk sel from XTAL
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x80,0x00); 
    HDMI_WriteI2C_Byte(0x08,0x7f); 	//top ctrl reg
    
}

//Drv_TtlRx_Pll_Set��TtlRxģ��RxPll���� ����pll��״̬
u8 Drv_TtlRx_Pll_Set(void)
{
    
    u8 ucloopx;         //ѭ��ϵ��
    u8 ucrtn = FALSE;   //rxpll״̬����ֵ
        
    //����RXPLL
    //�������ã�TTL mode 1.8GHz/V VCO,3 order pll mode
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x1e,0x40);
    HDMI_WriteI2C_Byte(0x20,0x02);
    HDMI_WriteI2C_Byte(0x22,0x00);
    HDMI_WriteI2C_Byte(0x61,(HDMI_ReadI2C_Byte(0x61) | 0x02)); //rg_ttlrx_phdly_en = 1
    
    //DDR ģʽ�£���Ҫ��DDR mode����RxPLL�����d_clk���г�2����
    #if TTLRX_DATARATE_MODE == DDR
        HDMI_WriteI2C_Byte(0x61,(HDMI_ReadI2C_Byte(0x61) | 0x04)); //DLL DDR mode enable
        HDMI_WriteI2C_Byte(0x23,(HDMI_ReadI2C_Byte(0x23) | 0x10)); //double dclk as pix clk
        //һ��DDRģʽ
        #if TTLRX_HALF_CLOCK_MODE == ENABLED
            LTLog(LT_LOG_INFO,"DDR Half Clock Mode:    Enable");
        #endif
        //DPIģʽ����Ҫ��decddr_clk��pix_clk���г�2������
        #if TTLRX_HALF_WIDTH_MODE == ENABLED
            HDMI_WriteI2C_Byte(0x23,(HDMI_ReadI2C_Byte(0x23) | 0x04)); //rg_rxpll_decddr_divsel: div 2
        LTLog(LT_LOG_INFO,"DDR Half Width Mode:    Enable");
        #endif
    #endif
    
    //BT656 8BIT ģʽ�£���Ҫ��RXPLL�����pix_clk���г�2����
    #if TTLRX_HALF_BIT_MODE == ENABLED
        HDMI_WriteI2C_Byte(0x21,HDMI_ReadI2C_Byte(0x21) | 0x01); //RxPll half pix clock output
        LTLog(LT_LOG_INFO,"Half Bit Mode:    Enable");
    #endif
    
    
    //rxpll ��λ
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x03, (HDMI_ReadI2C_Byte(0x03) & 0x7f)); //rgd_rxpll_rst_n=0
    Ocm_Timer0_Delay1ms(1);
    HDMI_WriteI2C_Byte(0x03, (HDMI_ReadI2C_Byte(0x03) | 0x80)); //rgd_rxpll_rst_n=1
    Ocm_Timer0_Delay1ms(20);
    
    
    //���rxpll��lock��cal״̬��3�λ���
    for(ucloopx = 0; ucloopx < 5; ucloopx++)
    {
        //rxpll ��λ
        HDMI_WriteI2C_Byte(0xff,0x81);
    	HDMI_WriteI2C_Byte(0x03,0x7f);
        Ocm_Timer0_Delay1ms(1);
    	HDMI_WriteI2C_Byte(0x03,0xff);
        Ocm_Timer0_Delay1ms(10);
        
        //rxpll У׼vcocur
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x24,0x02);  //vcocurѡ��Ӳ������
        HDMI_WriteI2C_Byte(0xff,0x87);
        HDMI_WriteI2C_Byte(0x06,(HDMI_ReadI2C_Byte(0x06) | 0x01)); //rgd_rxpll_cal_en=1
        Ocm_Timer0_Delay1ms(300);
        
        //���rxpll��lock��cal״̬
        if ((HDMI_ReadI2C_Byte(0x31) & 0x05) == 0x05)
        {
            LTLog(LT_LOG_INFO,"Rx Pll Cal done and Lock");
            LTLog(LT_LOG_INFO,"vmax, vmin = 0x%02x 0x%02x",HDMI_ReadI2C_Byte(0x30) & 0x3f,HDMI_ReadI2C_Byte(0x2f) & 0x3f);
            LTLog(LT_LOG_INFO,"vcoset = 0x%02x",HDMI_ReadI2C_Byte(0x32) & 0x3f); //��ӡvco_cur�Զ�У׼ֵ
            ucrtn = TRUE;
            break;
        }
        else
               {
            LTLog(LT_LOG_INFO,"Rx Pll Cal not done");
          
        }
    }  
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x0d,0x7e); //ttl rx reset
    HDMI_WriteI2C_Byte(0x0d,0x7f); //ttl rx release
    
    return ucrtn;
}


#if TTLRX_SYNC_INTER_MODE == ENABLED

void Drv_TtlRx_TimingSet(StructChipRxVidTiming *VidTiming)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
    
    HDMI_WriteI2C_Byte(0xd0,(u8)(VidTiming->usHact>>8));       
	HDMI_WriteI2C_Byte(0xd1,(u8)(VidTiming->usHact));          
    HDMI_WriteI2C_Byte(0xd2,(u8)(VidTiming->usHbp>>8));        
	HDMI_WriteI2C_Byte(0xd3,(u8)(VidTiming->usHbp));           
    HDMI_WriteI2C_Byte(0xd4,(u8)(VidTiming->usHfp>>8));    
	HDMI_WriteI2C_Byte(0xd5,(u8)(VidTiming->usHfp)); 
    HDMI_WriteI2C_Byte(0xd6,(u8)(VidTiming->usHs>>8));    
	HDMI_WriteI2C_Byte(0xd7,(u8)(VidTiming->usHs));       
    HDMI_WriteI2C_Byte(0xd8,(u8)(VidTiming->usHtotal>>8));    
	HDMI_WriteI2C_Byte(0xd9,(u8)(VidTiming->usHtotal));   
    HDMI_WriteI2C_Byte(0xda,(u8)(VidTiming->usVact>>8));       
	HDMI_WriteI2C_Byte(0xdb,(u8)(VidTiming->usVact));          
    HDMI_WriteI2C_Byte(0xdc,(u8)(VidTiming->usVbp>>8));        
	HDMI_WriteI2C_Byte(0xdd,(u8)(VidTiming->usVbp));           
    HDMI_WriteI2C_Byte(0xde,(u8)(VidTiming->usVfp>>8));    
	HDMI_WriteI2C_Byte(0xdf,(u8)(VidTiming->usVfp)); 
    HDMI_WriteI2C_Byte(0xe0,(u8)(VidTiming->usVs>>8));    
	HDMI_WriteI2C_Byte(0xe1,(u8)(VidTiming->usVs));       

    #if TTLRX_VIDEO_FORMAT == I_FORMAT
        if(VidTiming->usVact == 1080)
        {
            HDMI_WriteI2C_Byte(0xe6,0x03);
            HDMI_WriteI2C_Byte(0xe7,0x8c);
        }
    #endif
}

void Drv_TtlRx_TimingSel(u8 uci)
{
    StructChipRxVidTiming VidTiming;
    
    VidTiming.usVtotal = resolution[uci].usVtotal;
    VidTiming.usVact   = resolution[uci].usVact;
    VidTiming.usVs     = resolution[uci].usVs;
    VidTiming.usVfp    = resolution[uci].usVfp;
    VidTiming.usVbp    = resolution[uci].usVbp;
    
    VidTiming.usHtotal = resolution[uci].usHtotal;
    VidTiming.usHact   = resolution[uci].usHact;
    VidTiming.usHs     = resolution[uci].usHs;
    VidTiming.usHfp    = resolution[uci].usHfp;
    VidTiming.usHbp    = resolution[uci].usHbp;
    
    Drv_TtlRx_TimingSet(&VidTiming);
    
    LTLog(LT_LOG_INFO,"self timing set:");
    LTLog(LT_LOG_INFO,"hfp, hs, hbp, hact, htotal = %d %d %d %d %d",VidTiming.usHfp, VidTiming.usHs, VidTiming.usHbp, VidTiming.usHact, VidTiming.usHtotal);
    LTLog(LT_LOG_INFO,"vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",VidTiming.usVfp, VidTiming.usVs, VidTiming.usVbp, VidTiming.usVact, VidTiming.usVtotal);
}

void Drv_TtlRx_SelfTimingSet(void)
{
    u16 ushtotal, usvtotal;
    u16 ushact, usvact;
    u8 uci;
    u8 ucResolutionnum = 0;
    u8 ucphase;
    
    //timing phase sel
    #if TTLRX_PHASE_SEL == 10
    LTLog(LT_LOG_INFO,"self timing phase check:");
    for(ucphase = 0; ucphase < 10; ucphase++)
    {
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x22,ucphase);
        
        HDMI_WriteI2C_Byte(0xff,0x86); 
        Ocm_Timer0_Delay1ms(100);
        usvtotal = HDMI_ReadI2C_Byte(0x62);
        usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);
        ushtotal = HDMI_ReadI2C_Byte(0x60);
        ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);
        usvact = HDMI_ReadI2C_Byte(0x5e);
        usvact = (usvact << 8) + HDMI_ReadI2C_Byte(0x5f);
        ushact = HDMI_ReadI2C_Byte(0x5c);
        ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);
        
        LTLog(LT_LOG_INFO,"phase: 0x%02x", ucphase);
        LTLog(LT_LOG_INFO,"hact, htotal, vact, vtotal = %d %d %d %d",ushact, ushtotal, usvact, usvtotal);
    }
    #else
        ucphase = TTLRX_PHASE_SEL;
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x22,ucphase);
        
        HDMI_WriteI2C_Byte(0xff,0x86); 
        Ocm_Timer0_Delay1ms(100);
        usvtotal = HDMI_ReadI2C_Byte(0x62);
        usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);
        ushtotal = HDMI_ReadI2C_Byte(0x60);
        ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);
        usvact = HDMI_ReadI2C_Byte(0x5e);
        usvact = (usvact << 8) + HDMI_ReadI2C_Byte(0x5f);
        ushact = HDMI_ReadI2C_Byte(0x5c);
        ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);
        
        LTLog(LT_LOG_INFO,"************timing set phase: 0x%02x", ucphase);
    #endif
    
    //flag check
    HDMI_WriteI2C_Byte(0xff,0x85); 
    if(HDMI_ReadI2C_Byte(0xbf) & 0x01)
    {
        ushact = (ushact - 2);
        LTLog(LT_LOG_INFO,"flag check:   BTA_T1004");
    }
    else if(HDMI_ReadI2C_Byte(0xbf) & 0x02)
    {
        ushact = (ushact - 2);
        LTLog(LT_LOG_INFO,"flag check:   BT656");
    }
    else if(HDMI_ReadI2C_Byte(0xbf) & 0x04)
    {
        ushact = (ushact - 4);
        LTLog(LT_LOG_INFO,"flag check:   8bit BT1120");
    }
    else if(HDMI_ReadI2C_Byte(0xbf) & 0x08)
    {
        ushact = (ushact - 4);
        LTLog(LT_LOG_INFO,"flag check:   16bit BT1120");
    }
    else
    {
        LTLog(LT_LOG_INFO,"No flag fit!");
    }
    LTLog(LT_LOG_INFO,"hact, htotal, vact, vtotal = %d %d %d %d",ushact, ushtotal, usvact, usvtotal);
    
    //timing fit search
    ucResolutionnum = sizeof(resolution) / sizeof(resolution[0]);
    #if TTLRX_VIDEO_FORMAT == P_FORMAT
    for (uci = 0; uci < ucResolutionnum; uci++)
    {
        if (( ushact == resolution[uci].usHact ) && 
            ( usvact == resolution[uci].usVact ))
        {            
            LTLog(LT_LOG_INFO,"uci fit = 0x%02x",uci);
            LTLog(LT_LOG_INFO,"ucihact, ucivact = %d %d", resolution[uci].usHact, resolution[uci].usVact);
            Drv_TtlRx_TimingSel(uci);
            break;
        }
    }
    #elif TTLRX_VIDEO_FORMAT == I_FORMAT
    for (uci = 0; uci < ucResolutionnum; uci++)
    {
        if ( (( ushtotal == resolution[uci].usHtotal ) && ( usvtotal == resolution[uci].usVtotal/2 )) ||
             (( ushtotal == resolution[uci].usHtotal ) && ( usvtotal == resolution[uci].usVtotal/2 + 1 )) )
        {
            Drv_TtlRx_TimingSel(uci);
            break;
        }
    }
    #endif
    
    if(uci == ucResolutionnum)
    {
        LTLog(LT_LOG_INFO,"No timing fit!");
    }
 
}
#endif

#if TTLRX_DE_SYNC_MODE != TTL_NORMAL_MODE
void Drv_TtlRx_DeSyncModeSet(void)
{
    u16 usVtotal,usVact,usVs,usVfp,usVbp;
    u16 usHtotal,usHact,usHs,usHfp,usHbp;
    u8 uci,ucResolutionnum;
    u8 sync_polarity;
    
    usHact = TTLRX_DE_SYNC_MODE_HACT;
    usVact = TTLRX_DE_SYNC_MODE_VACT;
    ucResolutionnum = sizeof(resolution) / sizeof(resolution[0]);
    
    for (uci = 0; uci < ucResolutionnum; uci++)
    {
        if (( usHact == resolution[uci].usHact ) && 
            ( usVact == resolution[uci].usVact ))
        {            
            LTLog(LT_LOG_INFO,"uci fit = 0x%02x",uci);
            LTLog(LT_LOG_INFO,"ucihact, ucivact = %d %d", resolution[uci].usHact, resolution[uci].usVact);
            break;
        }
    }
    
    usVtotal = resolution[uci].usVtotal;
    usVact   = resolution[uci].usVact;
    usVs     = resolution[uci].usVs;
    usVfp    = resolution[uci].usVfp;
    usVbp    = resolution[uci].usVbp;
    
    usHtotal = resolution[uci].usHtotal;
    usHact   = resolution[uci].usHact;
    usHs     = resolution[uci].usHs;
    usHfp    = resolution[uci].usHfp;
    usHbp    = resolution[uci].usHbp;
    
    HDMI_WriteI2C_Byte(0xff,0x85);
    
    HDMI_WriteI2C_Byte(0xd0,(u8)(usHact>>8));       
	HDMI_WriteI2C_Byte(0xd1,(u8)(usHact));          
    HDMI_WriteI2C_Byte(0xd2,(u8)(usHbp>>8));        
	HDMI_WriteI2C_Byte(0xd3,(u8)(usHbp));           
    HDMI_WriteI2C_Byte(0xd4,(u8)(usHfp>>8));    
	HDMI_WriteI2C_Byte(0xd5,(u8)(usHfp)); 
    HDMI_WriteI2C_Byte(0xd6,(u8)(usHs>>8));    
	HDMI_WriteI2C_Byte(0xd7,(u8)(usHs));       
    HDMI_WriteI2C_Byte(0xd8,(u8)(usHtotal>>8));    
	HDMI_WriteI2C_Byte(0xd9,(u8)(usHtotal));   
    HDMI_WriteI2C_Byte(0xda,(u8)(usVact>>8));       
	HDMI_WriteI2C_Byte(0xdb,(u8)(usVact));          
    HDMI_WriteI2C_Byte(0xdc,(u8)(usVbp>>8));        
	HDMI_WriteI2C_Byte(0xdd,(u8)(usVbp));           
    HDMI_WriteI2C_Byte(0xde,(u8)(usVfp>>8));    
	HDMI_WriteI2C_Byte(0xdf,(u8)(usVfp)); 
    HDMI_WriteI2C_Byte(0xe0,(u8)(usVs>>8));    
	HDMI_WriteI2C_Byte(0xe1,(u8)(usVs)); 
    
    LTLog(LT_LOG_INFO,"self timing set:");
    LTLog(LT_LOG_INFO,"hfp, hs, hbp, hact, htotal = %d %d %d %d %d",usHfp, usHs, usHbp, usHact, usHtotal);
    LTLog(LT_LOG_INFO,"vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usVfp, usVs, usVbp, usVact, usVtotal);
    
    #if TTLRX_DE_SYNC_MODE == TTL_SYNC_MODE
         
         Ocm_Timer0_Delay1ms(100);
         HDMI_WriteI2C_Byte(0xff,0x86);
         sync_polarity = HDMI_ReadI2C_Byte(0x4f);
         LTLog(LT_LOG_INFO,"TTL input sync_polarity = 0x%02x", sync_polarity);
         
       if(!(sync_polarity & 0x04)) //hsync
	{
        HDMI_WriteI2C_Byte(0xff,0x85);
        HDMI_WriteI2C_Byte(0xc7, (HDMI_ReadI2C_Byte(0xc7) & BIT0_0));
        LTLog(LT_LOG_INFO,"sync mode enable, set hsync pol");
	}
	if(!(sync_polarity & 0x08)) //vsync
	{
        HDMI_WriteI2C_Byte(0xff,0x85);
        HDMI_WriteI2C_Byte(0xc7, (HDMI_ReadI2C_Byte(0xc7) & BIT1_0));
        LTLog(LT_LOG_INFO,"sync mode enable, set vsync pol");
	}
         
        HDMI_WriteI2C_Byte(0xff,0x85);
        HDMI_WriteI2C_Byte(0xc7,(HDMI_ReadI2C_Byte(0xc7) & BIT2_0));
        HDMI_WriteI2C_Byte(0xc7,HDMI_ReadI2C_Byte(0xc7) | 0x80); //de_gen enable
        LTLog(LT_LOG_INFO,"sync mode enable, generate de");
    
    #elif TTLRX_DE_SYNC_MODE == TTL_DE_MODE
        HDMI_WriteI2C_Byte(0xc7,HDMI_ReadI2C_Byte(0xc7) | 0x40); //sync_gen enable
        LTLog(LT_LOG_INFO,"de mode enable, generate sync");
    #endif
    #if TTLRX_VIDEO_FORMAT == P_FORMAT
        HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) & 0x7f);
    #elif TTLRX_VIDEO_FORMAT == I_FORMAT
        HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x80);
    #endif

}
#endif

void Drv_TtlRx_ClkChkDebug(void)
{
    u32 fm_value;
  
     HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0X90,0x19);
    Ocm_Timer0_Delay1ms(5);
    if ((HDMI_ReadI2C_Byte(0x98) & 0x60) == 0x60)
	{
		LTLog(LT_LOG_INFO,"FM CLOCK DET Stable");
	}
    else
	{
        LTLog(LT_LOG_INFO,"Frequency meter output clock status unStable");
	}
    
    HDMI_WriteI2C_Byte(0x90,0x99);
    fm_value = (HDMI_ReadI2C_Byte(0x98) & 0x0f);
    fm_value = (fm_value << 8) + HDMI_ReadI2C_Byte(0x99);
    fm_value = (fm_value << 8) + HDMI_ReadI2C_Byte(0x9a);
    HDMI_WriteI2C_Byte(0x90,(HDMI_ReadI2C_Byte(0x90) & BIT7_0));
 
    LTLog(LT_LOG_DEBUG,"TTL input dclock: %ld",fm_value);
}
void Drv_TtlRx_VidChkDebug(void)
{
    u16 ushact, usvact;
    u16 ushs, usvs;
    u16 ushbp, usvbp;
    u16 ushtotal, usvtotal;
    u16 ushfp, usvfp;
	u8 ucsync_polarity;
    u8 ucphase;
    
    #if TTLRX_PHASE_SEL == 10
        for(ucphase = 0; ucphase < 10; ucphase++)
        {
            HDMI_WriteI2C_Byte(0xff,0x82);
            HDMI_WriteI2C_Byte(0x22,ucphase);
            
            Ocm_Timer0_Delay1ms(100);
            
            HDMI_WriteI2C_Byte(0xff,0x86);
        
            ucsync_polarity = HDMI_ReadI2C_Byte(0x4f);
            
            usvs = HDMI_ReadI2C_Byte(0x52); //[15:8]
            usvs = (usvs <<8 ) + HDMI_ReadI2C_Byte(0x53);

            ushs = HDMI_ReadI2C_Byte(0x50);
            ushs = (ushs << 8) + HDMI_ReadI2C_Byte(0x51);
            
            usvbp = HDMI_ReadI2C_Byte(0x57);
            usvfp = HDMI_ReadI2C_Byte(0x5b);

            ushbp = HDMI_ReadI2C_Byte(0x54);
            ushbp = (ushbp << 8) + HDMI_ReadI2C_Byte(0x55);

            ushfp = HDMI_ReadI2C_Byte(0x58);
            ushfp = (ushfp << 8) + HDMI_ReadI2C_Byte(0x59);

            usvtotal = HDMI_ReadI2C_Byte(0x62);
            usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);

            ushtotal = HDMI_ReadI2C_Byte(0x60);
            ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);

            usvact = HDMI_ReadI2C_Byte(0x5e);
            usvact = (usvact << 8)+ HDMI_ReadI2C_Byte(0x5f);

            ushact = HDMI_ReadI2C_Byte(0x5c);
            ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);

            LTLog(LT_LOG_INFO,"phase: 0x%02x", ucphase);
            LTLog(LT_LOG_INFO,"sync_polarity = 0x%02x", ucsync_polarity);
            LTLog(LT_LOG_INFO,"hfp, hs, hbp, hact, htotal = %d %d %d %d %d",ushfp, ushs, ushbp, ushact, ushtotal);
            LTLog(LT_LOG_INFO,"vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usvfp, usvs, usvbp, usvact, usvtotal);
        }
    #else
        ucphase = TTLRX_PHASE_SEL;
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x22,ucphase);
        
        Ocm_Timer0_Delay1ms(100);
        
        HDMI_WriteI2C_Byte(0xff,0x86);
    
        ucsync_polarity = HDMI_ReadI2C_Byte(0x4f);
        
        usvs = HDMI_ReadI2C_Byte(0x52); //[15:8]
        usvs = (usvs <<8 ) + HDMI_ReadI2C_Byte(0x53);

        ushs = HDMI_ReadI2C_Byte(0x50);
        ushs = (ushs << 8) + HDMI_ReadI2C_Byte(0x51);
        
        usvbp = HDMI_ReadI2C_Byte(0x57);
        usvfp = HDMI_ReadI2C_Byte(0x5b);

        ushbp = HDMI_ReadI2C_Byte(0x54);
        ushbp = (ushbp << 8) + HDMI_ReadI2C_Byte(0x55);

        ushfp = HDMI_ReadI2C_Byte(0x58);
        ushfp = (ushfp << 8) + HDMI_ReadI2C_Byte(0x59);

        usvtotal = HDMI_ReadI2C_Byte(0x62);
        usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);

        ushtotal = HDMI_ReadI2C_Byte(0x60);
        ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);

        usvact = HDMI_ReadI2C_Byte(0x5e);
        usvact = (usvact << 8)+ HDMI_ReadI2C_Byte(0x5f);

        ushact = HDMI_ReadI2C_Byte(0x5c);
        ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);

        LTLog(LT_LOG_INFO,"************video check phase: 0x%02x", ucphase);
        LTLog(LT_LOG_INFO,"sync_polarity = 0x%02x", ucsync_polarity);
        LTLog(LT_LOG_INFO,"hfp, hs, hbp, hact, htotal = %d %d %d %d %d",ushfp, ushs, ushbp, ushact, ushtotal);
        LTLog(LT_LOG_INFO,"vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usvfp, usvs, usvbp, usvact, usvtotal);
    #endif
    
}

#endif