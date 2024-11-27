/******************************************************************************
  * @project: LT9211C
  * @file: DrvLvdsRx.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#include "../include.h"

#if ((LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)||(LT9211C_MODE_SEL == LVDS_IN_MIPI_OUT)||(LT9211C_MODE_SEL == LVDS_IN_TTL_OUT))

//Drv_LvdsRx_PhySet��LvdsRxģ��Phy����
void Drv_LvdsRx_PhySet(void)
{
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x01,0x00); //LVDS RX Phy Power disable
    Ocm_Timer0_Delay1ms(10);

    //3lane��4lane�£���portA���������        
    #if (LVDSRX_PORT_SEL == PORTA) && (LVDSRX_LANENUM == FOUR_LANE)
        HDMI_WriteI2C_Byte(0x01,0xc4); //[7]:1      LVDS RX portB power enable
                                       //[6][2]:1   portA/B LVDS mode enable
        HDMI_WriteI2C_Byte(0x02,0x22); //[5][1]:1   lvds mode, clk lane need swap
                                       //[4][0]:1   clk select outer path, link clk will be sent to divider A/B  
                                       //[6]:0      divider A input use link clk A
                                       //[2]:0      divider B input use link clk B
        HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
        HDMI_WriteI2C_Byte(0x07,0x2a); //[3]:1      LVDS portB lane-2 PN swap��is need because mipi use this lane as clock lane
        HDMI_WriteI2C_Byte(0x09,0x23); //[3]:0      rxpll input clk select link clk A
                                       //[1]:0      rxpll use link clk enable
        HDMI_WriteI2C_Byte(0x0e,0x22); //[7][3]0:   not use rterm turn_off accelerate function
        
        LTLog(LT_LOG_INFO,"LVDS Input PortA");
 
    //3lane��4lane�£���portB���������    
    #elif (LVDSRX_PORT_SEL == PORTB) && (LVDSRX_LANENUM == FOUR_LANE)
        #if LT9211C_VERSION == U1 
            HDMI_WriteI2C_Byte(0x01,0x4c); //[3]:1      LVDS RX portB power enable
                                           //[6][2]:1   portA/B LVDS mode enable
            HDMI_WriteI2C_Byte(0x02,0x22); //[5][1]:1   lvds mode, clk lane need swap
                                           //[4][0]:1   clk select outer path, link clk will be sent to divider A/B  
                                           //[6]:0      divider A input use link clk A
                                           //[2]:0      divider B input use link clk B
            HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
            HDMI_WriteI2C_Byte(0x07,0x2a); //[3]:1      LVDS portB lane-2 PN swap��is need because mipi use this lane as clock lane
            HDMI_WriteI2C_Byte(0x09,0x2b); //[3]:1      rxpll input clk select link clk B
                                           //[1]:0      rxpll use link clk enable
            HDMI_WriteI2C_Byte(0x0e,0x22); //[7][3]0:   not use rterm turn_off accelerate function
            HDMI_WriteI2C_Byte(0x1a,0x15); //[7:5][0]   lvds portB lane-c delay software set: 1 step less than default delay
        #elif LT9211C_VERSION == U2
            HDMI_WriteI2C_Byte(0x01,0xec); //[7]:1      LVDS RX portA power enable
                                           //[3]:1      LVDS RX portB power enable
                                           //[5]:1      reduce portA power consumption
                                           //[6][2]:1   portA/B LVDS mode enable
            HDMI_WriteI2C_Byte(0x02,0x62); //[5][1]:1   lvds mode, clk lane need swap
                                           //[4][0]:1   clk select outer path, link clk will be sent to divider A/B  
                                           //[6]:1      divider A input use link clk B
                                           //[2]:0      divider B input use link clk B
            HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
            HDMI_WriteI2C_Byte(0x07,0x2a); //[3]:1      LVDS portB lane-2 PN swap��is need because mipi use this lane as clock lane
            HDMI_WriteI2C_Byte(0x09,0x2b); //[3]:1      rxpll input clk select link clk B
                                           //[1]:0      rxpll use link clk enable
            HDMI_WriteI2C_Byte(0x0e,0x22); //[7][3]0:   not use rterm turn_off accelerate function
        #endif

        LTLog(LT_LOG_INFO,"LVDS Input PortB");

    //2port��5lane��������ã���Ҫ�õ�����port
    #else
        HDMI_WriteI2C_Byte(0x01,0xcc); //[7]:1      LVDS RX portA power enable
                                       //[3]:1      LVDS RX portB power enable
                                       //[6][2]:1   portA/B LVDS mode enable
        HDMI_WriteI2C_Byte(0x02,0x22); //[5][1]:1   lvds mode, clk lane need swap
                                       //[4][0]:1   clk select outer path, link clk will be sent to divider A/B  
                                       //[6]:0      divider A input use link clk A
                                       //[2]:0      divider B input use link clk B
        HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
        HDMI_WriteI2C_Byte(0x07,0x2a); //[3]:1      LVDS portB lane-2 PN swap��is need because mipi use this lane as clock lane
        HDMI_WriteI2C_Byte(0x09,0x23); //[3]:0      rxpll input clk select link clk A
                                       //[1]:0      rxpll use link clk enable
        HDMI_WriteI2C_Byte(0x0e,0x22); //[7][3]0:   not use rterm turn_off accelerate function
        HDMI_WriteI2C_Byte(0x1a,0x15); //[7:5][0]   lvds portB lane-c delay software set: 1 step less than default delay

        LTLog(LT_LOG_INFO,"LVDS Input PortA&B");
    #endif
    
    #if LVDSRX_LANENUM == FIVE_LANE
        
            HDMI_WriteI2C_Byte(0x02,0x24); //[2]:1      5lane lvds byte clk must select link clk A 
            HDMI_WriteI2C_Byte(0x07,(HDMI_ReadI2C_Byte(0x07) & BIT3_0)); //5lane lvds not need portB lane-2 PN swap
            HDMI_WriteI2C_Byte(0x09,(HDMI_ReadI2C_Byte(0x09) | BIT2_1)); //5lane lvds portB lane-c work as data lane
            HDMI_WriteI2C_Byte(0x14,0x30); //[5:4]      Port-B lane-0 high speed mode software enable
            HDMI_WriteI2C_Byte(0x15,0x0c); //[3:2]      Port-B lane-4(5lane����) high speed mode software enable
            HDMI_WriteI2C_Byte(0x1a,0x45); //[7:5][0]   lvds portB lane-c delay software set: 1 step more than default delay
     
    #endif
    
    //��λLvdsRx Phy
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x08,0x7d); //lvds rx phy reset
    HDMI_WriteI2C_Byte(0x08,0x7f); //lvds rx phy release
 
}


void Drv_LvdsRx_ClkSel(void)
{
    /* CLK sel */
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0xe9,0x88); //sys clk sel from XTAL
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x80,0x00); //[7:6]rx sram rd clk src sel ad dessc half pix clk
                                   //[5:4]rx sram wr clk src sel rxpll half pix clk
                                   //[1:0]video check clk sel from rxpll pix clk
#if LVDSRX_PORT_SEL == PORTA
    HDMI_WriteI2C_Byte(0x81,0x00); //[5]0: mlrx byte clock select from ad_mlrxa_byte_clk
                                   //[4]0: rx output pixel clock select from ad_rxpll_pix_clk
#elif LVDSRX_PORT_SEL == PORTB
    HDMI_WriteI2C_Byte(0x81,0x20); //[5]1: mlrx byte clock select from ad_mlrxb_byte_clk
                                   //[4]0: rx output pixel clock select from ad_rxpll_pix_clk
#endif

    //[4]1'b1:rx output pix clk must sel desscpll pix clk when ssc enable
#if LVDSRX_DESSC_SEL == ENABLED
    HDMI_WriteI2C_Byte(0x81,(HDMI_ReadI2C_Byte(0x81) | BIT4_1));
#endif
    
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x32,0x03); //video check frame cnt set: 3 frame
}

//========================================================================
// Func Name   : Drv_LvdsRx_DataPathSel
// Description : LVDSRX data path sel when dessc or no ssc
// Input       : None
// Output      : None
// Return      : void
//========================================================================
void Drv_LvdsRx_DataPathSel(void)
{
    HDMI_WriteI2C_Byte(0xff,0xd8);
#if LVDSRX_DESSC_SEL == ENABLED
    HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) | BIT0_1));    //[0]1'b1: lvdsrx->dessc sram->CSC
#else
    HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & BIT0_0));    //[0]1'b0: lvdsrx->dessc bypass fifo->CSC
#endif
}


void Drv_LvdsRxPort_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0xd8);
#if LVDSRX_PORT_SEL == DOU_PORT
    HDMI_WriteI2C_Byte(0x10,(HDMI_ReadI2C_Byte(0x10) & BIT7_0));
#else
    HDMI_WriteI2C_Byte(0x10,(HDMI_ReadI2C_Byte(0x10) | BIT7_1));
#endif
}

void Drv_LvdsRxLaneNum_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x40,0x00);
    HDMI_WriteI2C_Byte(0x41,0x01);
    HDMI_WriteI2C_Byte(0x42,0x03);
    HDMI_WriteI2C_Byte(0x43,0x04);
    HDMI_WriteI2C_Byte(0x44,0x05);

#if LVDSRX_LANENUM == FOUR_LANE
    HDMI_WriteI2C_Byte(0x45,0x00);
    HDMI_WriteI2C_Byte(0x46,0x01);
    HDMI_WriteI2C_Byte(0x47,0x03);
    HDMI_WriteI2C_Byte(0x48,0x04);
    HDMI_WriteI2C_Byte(0x49,0x45);
    
    #if LVDSRX_PORT_SEL == PORTB
        HDMI_WriteI2C_Byte(0x44,(HDMI_ReadI2C_Byte(0X44) | BIT6_1));
        HDMI_WriteI2C_Byte(0x49,(HDMI_ReadI2C_Byte(0X49) & BIT6_0));
    #endif
    
#elif LVDSRX_LANENUM == FIVE_LANE
    HDMI_WriteI2C_Byte(0x45,0x01);
    HDMI_WriteI2C_Byte(0x46,0x02);
    HDMI_WriteI2C_Byte(0x47,0x03);
    HDMI_WriteI2C_Byte(0x48,0x04);
    HDMI_WriteI2C_Byte(0x49,0x45);
#endif
}

void Drv_LvdsRxPort_Swap(void)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
#if LVDSRX_PORT_SWAP == ENABLED
    HDMI_WriteI2C_Byte(0x44,(HDMI_ReadI2C_Byte(0X44) | BIT6_1));
    HDMI_WriteI2C_Byte(0x49,(HDMI_ReadI2C_Byte(0X49) & BIT6_0));
#endif
}

u8 Drv_LvdsRxVidFmt_Set(void)
{
    u8 ucRxVidFmt;
    
    HDMI_WriteI2C_Byte(0xff,0xd8);
#if LVDSRX_DATAFORMAT == VESA
    HDMI_WriteI2C_Byte(0x11,(HDMI_ReadI2C_Byte(0x11) & BIT6_0));
    LTLog(LT_LOG_INFO,"Data Format: VESA");
#elif LVDSRX_DATAFORMAT == JEIDA
    HDMI_WriteI2C_Byte(0x11,(HDMI_ReadI2C_Byte(0x11) | BIT6_1));
    LTLog(LT_LOG_INFO,"Data Format: JEIDA");
#endif
    
#if LVDSRX_MODE == SYNC_MODE
    HDMI_WriteI2C_Byte(0x1f,(HDMI_ReadI2C_Byte(0x1f) & BIT5_0));
    LTLog(LT_LOG_INFO,"Mode: sync");
#elif LVDSRX_MODE == DE_MODE
    HDMI_WriteI2C_Byte(0x1f,(HDMI_ReadI2C_Byte(0x1f) | BIT5_1));
    LTLog(LT_LOG_INFO,"Mode: de");
#endif

#if LVDSRX_COLORSPACE == RGB
    HDMI_WriteI2C_Byte(0xff,0xd8);
    LTLog(LT_LOG_INFO,"ColorSpace: RGB");
    #if (LVDSRX_COLORDEPTH == DEPTH_6BIT)
        HDMI_WriteI2C_Byte(0x11,(HDMI_ReadI2C_Byte(0x11) | BIT6_1)); //JEIDA MODE
        HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) | BIT7_1));
        ucRxVidFmt = RGB_6Bit;
        LTLog(LT_LOG_INFO,"ColorDepth: 6Bit");
    #elif (LVDSRX_COLORDEPTH == DEPTH_8BIT)
        HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) | BIT7_1));
        ucRxVidFmt = RGB_8Bit;
        LTLog(LT_LOG_INFO,"ColorDepth: 8Bit");
    #elif (LVDSRX_COLORDEPTH == DEPTH_10BIT)
        HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) | 0xc0));
        ucRxVidFmt = RGB_10Bit;
        LTLog(LT_LOG_INFO,"ColorDepth: 10Bit");
    #endif

#elif (LVDSRX_COLORSPACE == YUV422)
    HDMI_WriteI2C_Byte(0xff,0xd8);
    HDMI_WriteI2C_Byte(0x11,(HDMI_ReadI2C_Byte(0x11) | BIT6_1)); //JEIDA MODE
    HDMI_WriteI2C_Byte(0x2d,0x05); //YoCb
    HDMI_WriteI2C_Byte(0x2e,0x05); //YoCb
    LTLog(LT_LOG_INFO,"ColorSpace: YUV422");
    #if (LVDSRX_COLORDEPTH == DEPTH_8BIT)
        HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) & 0x1f));
        ucRxVidFmt = YUV422_8bit;
        LTLog(LT_LOG_INFO,"ColorDepth: 8Bit");
        #if (LVDSRX_LANENUM == FIVE_LANE)
            HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) & 0x1f));
            LTLog(LT_LOG_INFO,"LaneNum: 5Lane");
        #else
            HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) | 0xa0));
            LTLog(LT_LOG_INFO,"LaneNum: 4Lane");
        #endif
    #endif
#endif

#if LVDSRX_SYNC_INTER_MODE == ENABLED
    HDMI_WriteI2C_Byte(0xff,0xd8);
    HDMI_WriteI2C_Byte(0x11,(HDMI_ReadI2C_Byte(0x11) & BIT6_0)); //VESA MODE
    HDMI_WriteI2C_Byte(0x16,(HDMI_ReadI2C_Byte(0x16) | 0xa0));
	HDMI_WriteI2C_Byte(0x34,(HDMI_ReadI2C_Byte(0x34) & BIT2_0));
    HDMI_WriteI2C_Byte(0x1f,(HDMI_ReadI2C_Byte(0x1f) | BIT5_1)); //[5]1'b1:timing self building function enable
    LTLog(LT_LOG_INFO,"Lvds Sync Code Mode: Internal"); //internal sync code mode
    #if (LVDSRX_VIDEO_FORMAT == I_FORMAT)
        HDMI_WriteI2C_Byte(0x1f,(HDMI_ReadI2C_Byte(0x1f) | BIT6_1));
        LTLog(LT_LOG_INFO,"Lvds Video Format: Interlaced"); //internal sync code mode
    #else
        HDMI_WriteI2C_Byte(0x1f,(HDMI_ReadI2C_Byte(0x1f) & BIT6_0));
        LTLog(LT_LOG_INFO,"Lvds Video Format: Progressive"); //internal sync code mode
    #endif
    
    #if (LVDSRX_SYNC_CODE_SEND == REPECTIVE)
        HDMI_WriteI2C_Byte(0x34,(HDMI_ReadI2C_Byte(0x34) | BIT0_1));
        LTLog(LT_LOG_INFO,"Lvds Sync Code Send: respectively."); //sync code send method sel respectively
    #else
        HDMI_WriteI2C_Byte(0x34,(HDMI_ReadI2C_Byte(0x34) & BIT0_0));
        LTLog(LT_LOG_INFO,"Lvds Sync Code Send: Non-respectively.");
    #endif
#endif

    return ucRxVidFmt;
}

#if ((LVDSRX_SYNC_INTER_MODE == ENABLED) || (LVDSRX_MODE == DE_MODE))
void Drv_LvdsRx_SelfTimingSet(void)
{
                                      //hfp    hs     hbp     hact     htotal   vfp   vs   vbp   vact    vtotal  pixclk
    #if LVDSRX_VID_SEL == VID_640x480_60Hz
    struct ChipRxVidTiming VidTiming = {8,     96,    40,     640,     800,     33,   2,   10,   480,    525,    25000 };
    #endif
    #if LVDSRX_VID_SEL == VID_720x480_60Hz
    struct ChipRxVidTiming VidTiming = {16,    62,    60,     720,     858,     9,    6,   30,   480,    525,    27000 };
    #endif
    #if LVDSRX_VID_SEL == VID_1280x720_60Hz
    struct ChipRxVidTiming VidTiming = {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,    74250 };
    #endif
    #if LVDSRX_VID_SEL == VID_1366x768_60Hz
    struct ChipRxVidTiming VidTiming = {26,    110,   110,    1366,    1592,    13,   6,   13,   768,    800,    81000 };
    #endif
    #if LVDSRX_VID_SEL == VID_1280x720_30Hz
    struct ChipRxVidTiming VidTiming = {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,    74250 };
    #endif
    #if LVDSRX_VID_SEL == VID_1920x720_60Hz
    struct ChipRxVidTiming VidTiming = {48,    32,    80,     1920,    2080,    5,    5,   20,   720,    750,    93600 };
    #endif
    #if LVDSRX_VID_SEL == VID_1920x1080_30Hz
    struct ChipRxVidTiming VidTiming = {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,   74250 };
    #endif
    #if LVDSRX_VID_SEL == VID_1920x1080_60Hz
    struct ChipRxVidTiming VidTiming = {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,   148500};
    #endif
    #if LVDSRX_VID_SEL == VID_1920x1200_60Hz
    struct ChipRxVidTiming VidTiming = {48,    32,    80,     1920,    2080,    3,    6,   26,   1200,   1235,   154000};
    #endif
    #if LVDSRX_VID_SEL == VID_3840x2160_30Hz
    struct ChipRxVidTiming VidTiming = {176,   88,    296,    3840,    4400,    8,    10,  72,   2160,   2250,   297006};
    #endif
    
	HDMI_WriteI2C_Byte(0xff,0xd8);
    
    HDMI_WriteI2C_Byte(0x20,(u8)(VidTiming.usHfp>>8));        //hfp[15:8]
	HDMI_WriteI2C_Byte(0x21,(u8)(VidTiming.usHfp));           //hfp[7:0]
    HDMI_WriteI2C_Byte(0x22,(u8)(VidTiming.usHs>>8));         //hs[15:8]
	HDMI_WriteI2C_Byte(0x23,(u8)(VidTiming.usHs));            //hs[7:0]
    HDMI_WriteI2C_Byte(0x24,(u8)(VidTiming.usHtotal>>8));     //htotal[15:8]
	HDMI_WriteI2C_Byte(0x25,(u8)(VidTiming.usHtotal));        //htotal[7:0]
    
    HDMI_WriteI2C_Byte(0x26,(u8)(VidTiming.usVfp>>8));        //vfp[15:8]
	HDMI_WriteI2C_Byte(0x27,(u8)(VidTiming.usVfp));           //vfp[7:0]	
    HDMI_WriteI2C_Byte(0x28,(u8)(VidTiming.usVs>>8));         //vs[15:8]
	HDMI_WriteI2C_Byte(0x29,(u8)(VidTiming.usVs));            //vs[7:0]
    
    #if LVDSRX_VIDEO_FORMAT == I_FORMAT
        #if LVDSRX_VID_SEL == VID_1920x1080_30Hz
            HDMI_WriteI2C_Byte(0x29,0x0a);
        #endif
    #endif
}
#endif



//Drv_LvdsRxPll_Cali��LvdsRxģ��RxPll���� ����pll��״̬
u8 Drv_LvdsRxPll_Cali(void)
{
    u8 ucloopx;         //ѭ��ϵ��
    u8 ucrtn = false;   //rxpll״̬����ֵ
    u32 byteclk_khz;    //byteclk_khz: IO�����뵽lvdsrxģ���ref_clk
  
    
   
    
    //����RXPLL
    #if LVDSRX_PLL_6M_MODE == DISABLED
        //�������ã�Lvds mode 3.5GHz/V VCO,adaptive mode
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x1e,0x47); //[3]1'b0: rxpll normal work
        HDMI_WriteI2C_Byte(0x20,0x32); //[5]1'b1: rxpll vco sel 3.5g/Hz
        HDMI_WriteI2C_Byte(0x21,0x64);
    #else
        //6M��Ƶ���ã�
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x1e,0x46); //[3]rg_rxpll_pd=0   [2]rg_rxpll_ref_sel=1��select LVDS byte clk as ref    [0]rg_rxpll_cpcur_sel=0��select 3order PLL mode
        HDMI_WriteI2C_Byte(0x20,0x03); //[4]rg_rxpll_lpf_sel=0��select 2order LPF    [5]rg_rxpll_vco_sel=0��select 1.8GHz/V    [0]rg_rxpll_lpf_set[0]=1��select 140pF as pole cap
        HDMI_WriteI2C_Byte(0x21,0x64);
        HDMI_WriteI2C_Byte(0x22,0x70); //[5]rg_rxpll_lpf_multc=1��select internal 1.28nF    [7:6]rg_rxpll_lpf_opam_sel[1:0]=01��select high BW opam1 
    #endif
    
    //LvdsRxģ���У�byte_clk��pix_clk֮��Ĺ�ϵ�����֣�
        //1. pix_clk = byte_clk         ��port RGB6/8/10bpc��YUV422-8bpc ��ͬ��ģʽ 
        //2. pix_clk = byte_clk*2       ˫port RGB6/8/10bpc��YUV422-8bpc ��ͬ��ģʽ   ��   ��port YCbCr422-8bpc-5lane ģʽ
        //3. pix_clk = byte_clk*4       ˫port YCbCr422-8bpc-5lane ģʽ
    
    //1��2��3��Ҫѡ��ͨ��821D[3]��1�ӱ�byte_clk
    HDMI_WriteI2C_Byte(0x1d,0x2C);  ////�ӱ���rg_rxpll_doub_clk_en = 1

    
    //��ȡrefclk_khz
  
  	for(ucloopx = 0; ucloopx < 3; ucloopx++)
    {
    	  
        HDMI_WriteI2C_Byte(0xff,0x87);
        if ((HDMI_ReadI2C_Byte(0x40) & 0x06) == 0x06)
        {
            LTLog(LT_LOG_INFO,"RXPLL_FM_CLK_DET Stable");
            break;
        }
    }
    
  
    HDMI_WriteI2C_Byte(0xff,0x87);
    HDMI_WriteI2C_Byte(0x22,0xF1);//RGD_RXPLL_FM_FREQ_KHZ_FRZ_EN enable
    Ocm_Timer0_Delay1ms(5);
    
    byteclk_khz = (HDMI_ReadI2C_Byte(0x3D) & 0x0f);
    byteclk_khz = (byteclk_khz << 8) + HDMI_ReadI2C_Byte(0x3E);
    byteclk_khz = (byteclk_khz << 8) + HDMI_ReadI2C_Byte(0x3F);
    HDMI_WriteI2C_Byte(0x22,(HDMI_ReadI2C_Byte(0x22) & BIT7_0));//RGD_RXPLL_FM_FREQ_KHZ_FRZ_EN disable

    LTLog(LT_LOG_INFO,"RXPLL_FM_FREQ_IN_KHZ: %ld",byteclk_khz);
    
    //����byte_clk�ֵ�����rg_rxpll_freq_set��rg_rxpll_pixclk_divsel
        //rg_rxpll_freq_set��RxPll��·������Ƶ��֮һ ������2��������Ƶ������һ���ǹ̶�/7��Ƶ����
        //rg_rxpll_pixclk_divsel��vco_clk��pix_clk֮��ķ�Ƶ��֮һ ������2����Ƶ������һ����/7/3.5��Ƶ������821D[3]���ƣ�
    //����3�ķֵ�����
    HDMI_WriteI2C_Byte(0xff,0x82);
    #if ((LVDSRX_COLORSPACE == YUV422) && (LVDSRX_COLORDEPTH == DEPTH_8BIT) && (LVDSRX_LANENUM == FIVE_LANE) && (LVDSRX_PORT_SEL == DOU_PORT))
        if((byteclk_khz*7 > 31250)&&(byteclk_khz*7 <= 62500))
        {
            HDMI_WriteI2C_Byte(0x1e,0x37);      //rg_rxpll_freq_set: Div 16
            HDMI_WriteI2C_Byte(0x21,0x67);      //rg_rxpll_pixclk_divsel: Div 8
        }
        else if((byteclk_khz*7 > 62500)&&(byteclk_khz*7 <= 125000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x27);      //rg_rxpll_freq_set: Div 8
            HDMI_WriteI2C_Byte(0x21,0x66);      //rg_rxpll_pixclk_divsel: Div 4
        }
        else if((byteclk_khz*7 > 125000)&&(byteclk_khz*7 <= 250000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x17);      //rg_rxpll_freq_set: Div 4
            HDMI_WriteI2C_Byte(0x21,0x65);      //rg_rxpll_pixclk_divsel: Div 2
        }
        else if((byteclk_khz*7 > 250000)&&(byteclk_khz*7 <= 750000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x07);      //rg_rxpll_freq_set: Div 2
            HDMI_WriteI2C_Byte(0x21,0x64);      //rg_rxpll_pixclk_divsel: Div 1
        }
        //modify 7.31
    #elif (  ((LVDSRX_COLORSPACE == RGB) && (LVDSRX_PORT_SEL != DOU_PORT)) || (((LVDSRX_COLORSPACE == YUV422) && (LVDSRX_LANENUM == FOUR_LANE) && (LVDSRX_PORT_SEL != DOU_PORT))) )
    //����1������
    
        HDMI_WriteI2C_Byte(0x21,0x44);   
  
    //����2�ķֵ�����
    #else
        if((byteclk_khz*7 > 31250)&&(byteclk_khz*7 <= 125000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x27);      //rg_rxpll_freq_set: Div 8
            HDMI_WriteI2C_Byte(0x21,0x67);      //rg_rxpll_pixclk_divsel: Div 8
        }
        else if((byteclk_khz*7 > 125000)&&(byteclk_khz*7 <= 250000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x17);      //rg_rxpll_freq_set: Div 4
            HDMI_WriteI2C_Byte(0x21,0x66);      //rg_rxpll_pixclk_divsel: Div 4
        }
        else if((byteclk_khz*7 > 250000)&&(byteclk_khz*7 <= 500000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x07);      //rg_rxpll_freq_set: Div 2
            HDMI_WriteI2C_Byte(0x21,0x65);      //rg_rxpll_pixclk_divsel: Div 2
        }
        else if((byteclk_khz*7 > 500000)&&(byteclk_khz*7 <= 1500000))
        {
            HDMI_WriteI2C_Byte(0x1e,0x47);      //rg_rxpll_freq_set: Div 1
            HDMI_WriteI2C_Byte(0x21,0x64);      //rg_rxpll_pixclk_divsel: Div 1
        }
    #endif
        
    #if LVDSRX_PLL_6M_MODE == ENABLED
        HDMI_WriteI2C_Byte(0x1e,HDMI_ReadI2C_Byte(0x1e) & BIT0_0);  //6Mģʽ�£�821e��4λΪ2'b0110
    #endif

    //���rxpll��lock״̬��3�λ���
    for(ucloopx = 0; ucloopx < 3; ucloopx++)
    {
    	//rxpll ��λ
        HDMI_WriteI2C_Byte(0xff,0x81);
    	HDMI_WriteI2C_Byte(0x03,0x7f);
        Ocm_Timer0_Delay1ms(1);
    	HDMI_WriteI2C_Byte(0x03,0xff);
        Ocm_Timer0_Delay1ms(10);

        //���rxpll��lock״̬
        HDMI_WriteI2C_Byte(0xff,0x87);
        if ((HDMI_ReadI2C_Byte(0x31) & 0x04) == 0x04)
        {
            LTLog(LT_LOG_INFO,"Rx Pll Lock");
            ucrtn = true;
            break;
        }
    }
    
    return ucrtn;
    
}

void Drv_LvdsRx_VidChkDebug(void)
{
    u16 ushact, usvact;
    u16 ushs, usvs;
    u16 ushbp, usvbp;
    u16 ushtotal, usvtotal;
    u16 ushfp, usvfp;
	u8 ucsync_polarity;

    HDMI_WriteI2C_Byte(0xff,0xd8);
    #if LVDSRX_PORT_SEL == PORTB
    HDMI_WriteI2C_Byte(0x2a,HDMI_ReadI2C_Byte(0x2a) | BIT6_1);
    #else
    HDMI_WriteI2C_Byte(0x2a,HDMI_ReadI2C_Byte(0x2a) & BIT6_0);
    #endif
    
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

	
	LTLog(LT_LOG_INFO,"sync_polarity = 0x%02x", ucsync_polarity);
    LTLog(LT_LOG_INFO,"hfp, hs, hbp, hact, htotal = %d %d %d %d %d",ushfp, ushs, ushbp, ushact, ushtotal);
	LTLog(LT_LOG_INFO,"vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usvfp, usvs, usvbp, usvact, usvtotal);
}

#if LVDSRX_DESSC_SEL == ENABLED
void Drv_LvdsRx_DesscPll_Set(void)
{
    u32 pclk_khz;
    u8 desscpll_pixck_div =0;
    u32 pcr_m, pcr_k;

    pclk_khz = Drv_System_FmClkGet(AD_RXPLL_PIX_CLK);
    
    //dessc pll
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x26,0x20); //rg_desscpll_pd=0   rg_desscpll_ref_sel[1:0]=00
    
    if(pclk_khz < 44000)
    {
        HDMI_WriteI2C_Byte(0x2f,0x07);
        desscpll_pixck_div = 16;
    }

    else if(pclk_khz < 88000)
    {
        HDMI_WriteI2C_Byte(0x2f,0x06);
        desscpll_pixck_div = 8;
    }

    else if(pclk_khz < 176000)
    {
        HDMI_WriteI2C_Byte(0x2f,0x05);
        desscpll_pixck_div = 4;
    }

    else if(pclk_khz < 352000)
    {
        HDMI_WriteI2C_Byte(0x2f,0x04);
        desscpll_pixck_div = 2;
    }
    
    pcr_m = (pclk_khz * desscpll_pixck_div) /25;
    pcr_k = pcr_m%1000;
    pcr_m = pcr_m/1000;

    pcr_k <<= 14;
    
    //pixel clk
    HDMI_WriteI2C_Byte(0xff,0xd0); //pcr
    HDMI_WriteI2C_Byte(0x2d,0x7f);
    HDMI_WriteI2C_Byte(0x31,0x00);

    HDMI_WriteI2C_Byte(0x26,0x80|((u8)pcr_m));
    HDMI_WriteI2C_Byte(0x27,(u8)((pcr_k>>16)&0xff)); //K
    HDMI_WriteI2C_Byte(0x28,(u8)((pcr_k>>8)&0xff)); //K
    HDMI_WriteI2C_Byte(0x29,(u8)(pcr_k&0xff)); //K
    
    //dessc_reset
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x03, (HDMI_ReadI2C_Byte(0x03) & 0xfe));
    HDMI_WriteI2C_Byte(0x03, (HDMI_ReadI2C_Byte(0x03) | 0x01)); 
    Ocm_Timer0_Delay1ms(20);

}

void Drv_LvdsRx_DesscDig_Set(void)
{
    //dessc_setting
    HDMI_WriteI2C_Byte(0xff,0xd0);
    HDMI_WriteI2C_Byte(0x08,0x08); //sdm lvds
    HDMI_WriteI2C_Byte(0x20,0x0b); //sdm diff ord
    //HDMI_WriteI2C_Byte(0x1f,0x93); //soft div ratio
    
    HDMI_WriteI2C_Byte(0xff,0xd9);
    HDMI_WriteI2C_Byte(0xc0,0x01); //enable
    
    //rx setting
    HDMI_WriteI2C_Byte(0xff,0xd8);
    HDMI_WriteI2C_Byte(0x32,0x05); //[0]rgd_dessc_sel: 0- dessc bypass fifo path; 1- dessc sram path; 
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x81,0x10); //[4]lvdsrx_out_pix_sel: 0-ad_rxpll_pix_clk; 1-ad_desscpll_pix_clk
    HDMI_WriteI2C_Byte(0x08,0x1f);
    HDMI_WriteI2C_Byte(0x80,0x01);
}
#endif

#endif