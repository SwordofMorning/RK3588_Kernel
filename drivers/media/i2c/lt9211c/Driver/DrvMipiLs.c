/******************************************************************************
  * @project: LT9211C
  * @file: DrvMipiLs.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#include    "../include.h"


#if (LT9211C_MODE_SEL == MIPI_LEVEL_SHIFT)

u32 ulPixClk = 0;


u8 Drv_MipiLsClk_Change(void)
{
    u32 ulPixClk_New = 0;
    ulPixClk_New = Drv_System_FmClkGet(AD_RXPLL_PIX_CLK);

    if (ulPixClk_New == 0)
    {
        LTLog(LT_LOG_ERROR,"MIPI Repeater Level shift Rx Clk Lost");
        return TRUE;
    }
    
    if (ulPixClk_New < (ulPixClk - 35) || ulPixClk_New > (ulPixClk + 35))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
    
}



void Drv_MipiLs_ClkSel(void)
{
    /* CLK sel */
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0xe9,0x88); //sys clk sel from XTAL
    
    #if (MIPILS_INPUT_PORT == PORTA || MIPILS_INPUT_PORT == DOU_PORT)
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x81,0x00);
    #endif
    #if MIPILS_INPUT_PORT == PORTB
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x81,0x20);
    #endif
}


void Drv_MipiLsRxPhy_Set(void)
{
    LTLog(LT_LOG_INFO,"MIPI Repeater level shift Config");
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x01,0x11); //Mipi level shift RxPhy Power disable
    
    #if MIPILS_INPUT_PORT == PORTA
        HDMI_WriteI2C_Byte(0x01,0x91); //[7]:1      Mipi level shift Rx portA power enable
                                       //[6][2]:0   portA/B MIPI mode enable
        HDMI_WriteI2C_Byte(0x02,0x11); //[5][1]:0   mipi mode, clk lane not need swap
                                       //[4][0]:1   clk select outer path, link clk will be sent to divider A/B 
                                       //[6]:0      divider A input use link clk A
                                       //[2]:0      divider B input use link clk B
        HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
        HDMI_WriteI2C_Byte(0x05,0x62); //[6:4]      port A clk lane EQ control select 11 dB
        HDMI_WriteI2C_Byte(0x09,0x23); //[3]:0      rxpll input clk select link clk A
                                       //[1]:0      rxpll use link clk enable
        HDMI_WriteI2C_Byte(0x13,0x0c); //[3:2]      Port-A clk lane high speed mode software enable
    #endif
    
    #if MIPILS_INPUT_PORT == PORTB
        HDMI_WriteI2C_Byte(0x01,0x99); //[7][3]:1   Mipi level shift Rx portA/B power enable
                                       //[6][2]:0   portA/B MIPI mode enable
        HDMI_WriteI2C_Byte(0x02,0x51); //[5][1]:0   mipi mode, clk lane not need swap
                                       //[4][0]:1   clk select outer path, link clk will be sent to divider A/B 
                                       //[6]:1      divider A input use link clk B
                                       //[2]:0      divider B input use link clk B
        HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
        HDMI_WriteI2C_Byte(0x05,0x62); //[6:4]      port A clk lane EQ control select 11 dB
        HDMI_WriteI2C_Byte(0x07,0x26); //[2:0]      port B clk lane EQ control select 11 dB
        HDMI_WriteI2C_Byte(0x09,0x2b); //[3]:1      rxpll input clk select link clk B
                                       //[1]:0      rxpll use link clk enable
        HDMI_WriteI2C_Byte(0x13,0x0c); //[3:2]      Port-A clk lane high speed mode software enable
        HDMI_WriteI2C_Byte(0x14,0x03); //[1:0]      Port-B clk lane high speed mode software enable
    #endif

    #if MIPILS_INPUT_PORT == DOU_PORT
        HDMI_WriteI2C_Byte(0x01,0x99); //[7][3]:1   Mipi level shift Rx portA/B power enable
                                       //[6][2]:0   portA/B MIPI mode enable
        HDMI_WriteI2C_Byte(0x02,0x11); //[5][1]:0   mipi mode, clk lane not need swap
                                       //[4][0]:1   clk select outer path, link clk will be sent to divider A/B 
                                       //[6]:0      divider A input use link clk A
                                       //[2]:0      divider B input use link clk B
        HDMI_WriteI2C_Byte(0x03,0xcc); //[6:4][2:0] port A & B EQ reference current sel 50uA
        HDMI_WriteI2C_Byte(0x05,0x62); //[6:4]      port A clk lane EQ control select 11 dB
        HDMI_WriteI2C_Byte(0x07,0x26); //[2:0]      port B clk lane EQ control select 11 dB
        HDMI_WriteI2C_Byte(0x09,0x23); //[3]:0      rxpll input clk select link clk A
                                       //[1]:0      rxpll use link clk enable
        HDMI_WriteI2C_Byte(0x13,0x0c); //[3:2]      Port-A clk lane high speed mode software enable
        HDMI_WriteI2C_Byte(0x14,0x03); //[1:0]      Port-B clk lane high speed mode software enable
    #endif
    
    #if LT9211C_VERSION == U1
        HDMI_WriteI2C_Byte(0x0d,0x11); //[6:4]  divider A output clk phase sel: 1-UI delay
                                       //[2:0]  divider B output clk phase sel: 1-UI delay
    #elif LT9211C_VERSION == U2
        HDMI_WriteI2C_Byte(0x0d,0x12); //[6:4]  divider A output clk phase sel: 1-UI delay
                                       //[2:0]  divider B output clk phase sel: 2-UI delay
    #endif

    HDMI_WriteI2C_Byte(0x0c,0x44); //mlrx lprx0 vref sel
    HDMI_WriteI2C_Byte(0x0f,0x28); 

    #if MIPILS_MODE == RX_LS
        HDMI_WriteI2C_Byte(0x0e,0x22);
        #if MIPILS_SHIFT_MODE == HS_PORTA_LP_PORTB 
            HDMI_WriteI2C_Byte(0x13,0xff);
            HDMI_WriteI2C_Byte(0x14,0xea);
            HDMI_WriteI2C_Byte(0x15,0xaa);
        #elif MIPILS_SHIFT_MODE == LP_PORTA_HS_PORTB
            HDMI_WriteI2C_Byte(0x02,0x51);
            HDMI_WriteI2C_Byte(0x09,0x2b);
            HDMI_WriteI2C_Byte(0x13,0xae);
            HDMI_WriteI2C_Byte(0x14,0xbf);
            HDMI_WriteI2C_Byte(0x15,0xff);
        #endif
    #endif
    
    #if LT9211C_VERSION == U1
        //need reset divider
        HDMI_WriteI2C_Byte(0xff,0x81);
        HDMI_WriteI2C_Byte(0x03,0xef); //mipirx div reset
        HDMI_WriteI2C_Byte(0x03,0xff); //mipirx div release
    #elif LT9211C_VERSION == U2
        //not need reset divider
    #endif
}


void Drv_MipiLsRxDig_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0xc5,MIPILS_LANE_NUM);
    
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x3f,0x09); //[0]1: mipi repeater mode enable
    HDMI_WriteI2C_Byte(0x56,0x40); //[6:5]2'b10 mltx src sel mipi repeater
    HDMI_WriteI2C_Byte(0x40,0x04); //[2:0]pa_ch0_src_sel ch4 data
    HDMI_WriteI2C_Byte(0x41,0x03); //[2:0]pa_ch1_src_sel ch3 data
    HDMI_WriteI2C_Byte(0x42,0x02); //[2:0]pa_ch2_src_sel ch2 data
    HDMI_WriteI2C_Byte(0x43,0x01); //[2:0]pa_ch3_src_sel ch1 data
    HDMI_WriteI2C_Byte(0x44,0x00); //[2:0]pa_ch3_src_sel ch0 data,[6]0:porta src sel porta
    HDMI_WriteI2C_Byte(0x45,0x04); //[2:0]pb_ch0_src_sel ch9 data
    HDMI_WriteI2C_Byte(0x46,0x03); //[2:0]pb_ch1_src_sel ch8 data
    HDMI_WriteI2C_Byte(0x47,0x02); //[2:0]pb_ch2_src_sel ch7 data
    HDMI_WriteI2C_Byte(0x48,0x01); //[2:0]pb_ch3_src_sel ch6 data
    HDMI_WriteI2C_Byte(0x49,0x40); //[2:0]pa_ch3_src_sel ch5 data,[6]1:portb src sel portb

    #if (MIPILS_INPUT_PORT == PORTA || MIPILS_INPUT_PORT == DOU_PORT)
    HDMI_WriteI2C_Byte(0x44,0x00); //[2:0]pa_ch3_src_sel ch0 data,[6]0:porta src sel porta
    HDMI_WriteI2C_Byte(0x49,0x00); //[2:0]pa_ch3_src_sel ch5 data,[6]0:portb src sel porta
    #endif
    #if MIPILS_INPUT_PORT == PORTB
    HDMI_WriteI2C_Byte(0x44,0x40); //[2:0]pa_ch3_src_sel ch0 data,[6]1:porta src sel portb
    HDMI_WriteI2C_Byte(0x49,0x40); //[2:0]pa_ch3_src_sel ch5 data,[6]1:portb src sel portb
    #endif



    #if MIPILS_MODE == RX_LS
        #if MIPILS_SHIFT_MODE == HS_PORTA_LP_PORTB
            HDMI_WriteI2C_Byte(0x44, HDMI_ReadI2C_Byte(0x44) | 0x20);
        #elif MIPILS_SHIFT_MODE == LP_PORTA_HS_PORTB
            HDMI_WriteI2C_Byte(0x44, HDMI_ReadI2C_Byte(0x44) | 0x40);
            HDMI_WriteI2C_Byte(0x49, HDMI_ReadI2C_Byte(0x44) | 0x20);
        #endif
    #endif
}

void Drv_MipiLsTxPhy_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x82);
    #if MIPILS_OUTPUT_PORT == PORTA
    HDMI_WriteI2C_Byte(0x36,0xd5);
    HDMI_WriteI2C_Byte(0x37,0x33);
    #endif

    #if MIPILS_OUTPUT_PORT == PORTB
    HDMI_WriteI2C_Byte(0x36,0xd6);
    HDMI_WriteI2C_Byte(0x37,0x33);
    #endif

    #if MIPILS_OUTPUT_PORT == DOU_PORT
    HDMI_WriteI2C_Byte(0x36,0xd7);
    HDMI_WriteI2C_Byte(0x37,0x33);
    #endif
    
    #if MIPILS_MODE == TX_LS
        #if MIPILS_SHIFT_MODE == HS_PORTA_LP_PORTB 
            //portA HS enable
            HDMI_WriteI2C_Byte(0x56,0xc2); //lane-3 hs enable
            HDMI_WriteI2C_Byte(0x57,0xc2); //lane-2 hs enable
            HDMI_WriteI2C_Byte(0x58,0xc2); //lane-C hs enable
            HDMI_WriteI2C_Byte(0x59,0xc2); //lane-1 hs enable
            HDMI_WriteI2C_Byte(0x5a,0xc2); //lane-0 hs enable
            
            //portB LP enable
            HDMI_WriteI2C_Byte(0x5b,0x83); //lane-3 lp enable
            HDMI_WriteI2C_Byte(0x5c,0x83); //lane-2 lp enable
            HDMI_WriteI2C_Byte(0x5d,0x83); //lane-C lp enable
            HDMI_WriteI2C_Byte(0x5e,0x83); //lane-1 lp enable
            HDMI_WriteI2C_Byte(0x5f,0x83); //lane-0 lp enable
            
        #elif MIPILS_SHIFT_MODE == LP_PORTA_HS_PORTB 
            //portA LP enable
            HDMI_WriteI2C_Byte(0x56,0x83); //lane-3 lp enable
            HDMI_WriteI2C_Byte(0x57,0x83); //lane-2 lp enable
            HDMI_WriteI2C_Byte(0x58,0x83); //lane-C lp enable
            HDMI_WriteI2C_Byte(0x59,0x83); //lane-1 lp enable
            HDMI_WriteI2C_Byte(0x5a,0x83); //lane-0 lp enable
            
            //portB LP enable
            HDMI_WriteI2C_Byte(0x5b,0xc2); //lane-3 hs enable
            HDMI_WriteI2C_Byte(0x5c,0xc2); //lane-2 hs enable
            HDMI_WriteI2C_Byte(0x5d,0xc2); //lane-C hs enable
            HDMI_WriteI2C_Byte(0x5e,0xc2); //lane-1 hs enable
            HDMI_WriteI2C_Byte(0x5f,0xc2); //lane-0 hs enable
            
        #endif
        
        //Lvds level output
        HDMI_WriteI2C_Byte(0x37,0x44);
        HDMI_WriteI2C_Byte(0x38,0x54);
        HDMI_WriteI2C_Byte(0x39,0x31);
        HDMI_WriteI2C_Byte(0x46,0x22);
        HDMI_WriteI2C_Byte(0x47,0x22);
        HDMI_WriteI2C_Byte(0x48,0x22);
        HDMI_WriteI2C_Byte(0x49,0x22);
        HDMI_WriteI2C_Byte(0x4a,0x22);
        HDMI_WriteI2C_Byte(0x4b,0x22);
        HDMI_WriteI2C_Byte(0x4c,0x22);
        HDMI_WriteI2C_Byte(0x4d,0x22);
        HDMI_WriteI2C_Byte(0x4e,0x22);
        HDMI_WriteI2C_Byte(0x4f,0x22);
        HDMI_WriteI2C_Byte(0x50,0x22);
        HDMI_WriteI2C_Byte(0x51,0x22);
        
    #endif

    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x4d,0x50); //porta manual generate tx clk
    HDMI_WriteI2C_Byte(0x52,0x50); //portb manual generate tx clk
    HDMI_WriteI2C_Byte(0x7e,0x55); //0x55 or 0xaa
}


void Drv_MipiLsTx_PllSet(void)
{
    u32 ulMipiTXPhyClk = 0;
    u8 ucSericlkDiv, ucDivSet;
    u8 ucPreDiv = 0;
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x21,0x4a); //[3:2]2'b10:rxpll pixclk mux sel byte clk,[1:0]2'b10:pix_mux_clk/4
    HDMI_WriteI2C_Byte(0x30,0x00); //[7]0:txpll normal work;[2:1]2'b00:txpll ref clk sel pix clock
    ulPixClk = Drv_System_FmClkGet(AD_RXPLL_PIX_CLK);
    ulMipiTXPhyClk = ulPixClk * 4;
    
    LTLog(LT_LOG_DEBUG, "ulPixClk: %ld, ulMipiTXPhyClk: %ld",ulPixClk, ulMipiTXPhyClk);

    HDMI_WriteI2C_Byte(0xff,0x82);
    if (ulPixClk < 20000)
    {
        HDMI_WriteI2C_Byte(0x31,0x28); //[2:0]3'b000: pre div set div1
        ucPreDiv = 1;
    }
    else if (ulPixClk >= 20000 && ulPixClk < 40000)
    {
        HDMI_WriteI2C_Byte(0x31,0x28); //[2:0]3'b000: pre div set div1
        ucPreDiv = 1;
    }
    else if (ulPixClk >= 40000 && ulPixClk < 80000)
    {
        HDMI_WriteI2C_Byte(0x31,0x29); //[2:0]3'b001: pre div set div2
        ucPreDiv = 2;
    }
    else if (ulPixClk >= 80000 && ulPixClk < 160000)
    {
        HDMI_WriteI2C_Byte(0x31,0x2a); //[2:0]3'b010: pre div set div4
        ucPreDiv = 4;
    }
    else if (ulPixClk >= 160000 && ulPixClk < 320000)
    {
        HDMI_WriteI2C_Byte(0x31,0x2b); //[2:0]3'b011: pre div set div8
        ucPreDiv = 8;
    }
    else if (ulPixClk >= 320000)
    {
        HDMI_WriteI2C_Byte(0x31,0x2f); //[2:0]3'b111: pre div set div16
        ucPreDiv = 16;
    }

    HDMI_WriteI2C_Byte(0xff,0x82);
    if (ulMipiTXPhyClk >= 640000 )//640M~1.28G
    {
        HDMI_WriteI2C_Byte(0x32,0x42);
        ucSericlkDiv = 1; //sericlk div1 [6:4]:0x40
    }
    else if (ulMipiTXPhyClk >= 320000 && ulMipiTXPhyClk < 640000)
    {
        HDMI_WriteI2C_Byte(0x32,0x02);
        ucSericlkDiv = 2; //sericlk div2 [6:4]:0x00
    }
    else if (ulMipiTXPhyClk >= 160000 && ulMipiTXPhyClk < 320000)
    {
        HDMI_WriteI2C_Byte(0x32,0x12);
        ucSericlkDiv = 4; //sericlk div4 [6:4]:0x10
    }
    else if (ulMipiTXPhyClk >= 80000 && ulMipiTXPhyClk < 160000)
    {
        HDMI_WriteI2C_Byte(0x32,0x22);
        ucSericlkDiv = 8; //sericlk div8 [6:4]:0x20
    }
    else //40M~80M
    {
        HDMI_WriteI2C_Byte(0x32,0x32);
        ucSericlkDiv = 16; //sericlk div16 [6:4]:0x30
    }

    ucDivSet = (ulMipiTXPhyClk * ucSericlkDiv) / (ulPixClk / ucPreDiv);
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x34,0x01);
    HDMI_WriteI2C_Byte(0x35,ucDivSet);

    LTLog(LT_LOG_DEBUG,"ucPreDiv :0x%02x, ucSericlkDiv :0x%02x, ucDivSet :0x%02x",ucPreDiv, ucSericlkDiv, ucDivSet);

}


u8 Drv_MipiLsTx_PllCali(void)
{
    u8 ucPllScanCnt = 0;
    u8 ucRtn = FALSE;
    do 
    {    
        HDMI_WriteI2C_Byte(0xff,0x81);//register bank	
        HDMI_WriteI2C_Byte(0x0c,0x78);//tx pll rest cal_rst =0
        HDMI_WriteI2C_Byte(0xff,0x87);//register bank
        HDMI_WriteI2C_Byte(0x0f,0x00);//tx pll cal = 0;
        
        HDMI_WriteI2C_Byte(0xff,0x81);//register bank
        HDMI_WriteI2C_Byte(0x0c,0xf9);//tx pll rest cal_rst =1
        HDMI_WriteI2C_Byte(0xff,0x87);//register bank
        HDMI_WriteI2C_Byte(0x0f,0x01);//tx pll cal = 0;
        Ocm_Timer0_Delay1ms(10);
        ucPllScanCnt++;
    }while((ucPllScanCnt < 3) && ((HDMI_ReadI2C_Byte(0x39) & 0x07) != 0x05));//PLL calibration done status
    if((HDMI_ReadI2C_Byte(0x39) & 0x07)== 0x05)
    {
        ucRtn = SUCCESS;
        LTLog(LT_LOG_INFO,"Tx Pll Lock");
    }
    else
    {
        LTLog(LT_LOG_ERROR,"Tx Pll Unlocked");
    }
    return ucRtn;
}

void Drv_MipiLsBta_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0xc6,0xd0); //[7]1:repeater clk term software select enable
    #if (MIPILS_INPUT_PORT == PORTA || MIPILS_INPUT_PORT == DOU_PORT)
    HDMI_WriteI2C_Byte(0xca,0x21); //[1]0:repeater port select porta,[0]1:tx channel bypass enable
    #endif
    #if MIPILS_INPUT_PORT == PORTB
    HDMI_WriteI2C_Byte(0xca,0x23); //[1]1:repeater port select portB,[0]1:tx channel bypass enable
    #endif
    HDMI_WriteI2C_Byte(0xcd,0x1f); //hs_en_dly

    LTLog(LT_LOG_INFO,"MIPI Repeater Level Shift Output");
}

void Drv_MipiLsClk_Check(void)
{
    u32 ulRxByteClk = 0;
    u32 ulTxByteClk = 0;
    
    ulRxByteClk = Drv_System_FmClkGet(AD_MLRXA_BYTE_CLK);
    ulTxByteClk = Drv_System_FmClkGet(AD_MLTX_READ_CLK);
    LTLog(LT_LOG_INFO, "RX byte Clk: %ld, Tx byte Clk: %ld",ulRxByteClk, ulTxByteClk);
    
    while(1);
}

#endif

