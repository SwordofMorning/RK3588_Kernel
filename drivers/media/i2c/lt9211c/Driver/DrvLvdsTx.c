/******************************************************************************
  * @project: LT9211C
  * @file: DrvLvdsTx.c
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  ******************************************************************************/

#include "../include.h"

#if ((LT9211C_MODE_SEL == LVDS_IN_LVDS_OUT)||(LT9211C_MODE_SEL == MIPI_IN_LVDS_OUT)||(LT9211C_MODE_SEL == TTL_IN_LVDS_OUT))

void Drv_LvdsTxSw_Rst()
{
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x08,0x6f); //LVDS TX SW reset
    Ocm_Timer0_Delay1ms(2);
    HDMI_WriteI2C_Byte(0x08,0x7f);
    LTLog(LT_LOG_INFO,"LVDS Tx Video Out");
}

void Drv_LVDSTxPhy_PowerOff()
{
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x36,0x00); //lvds enable
    HDMI_WriteI2C_Byte(0x37,0x00);
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x08,0x6f); //LVDS TX SW reset
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x03,0xbf); //mltx reset
}

void Drv_LvdsTxPhy_Poweron(void)
{
#if LVDSTX_PORT_SEL  == PORTA
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x36,0x01); //lvds enable
    HDMI_WriteI2C_Byte(0x37,0x40);
    LTLog(LT_LOG_INFO,"LVDS Output PortA");
    #if LVDSTX_LANENUM == FIVE_LANE
    HDMI_WriteI2C_Byte(0x36,0x03); 
    #endif
    
#elif LVDSTX_PORT_SEL  == PORTB
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x36,0x02); //lvds enable
    HDMI_WriteI2C_Byte(0x37,0x04);
    LTLog(LT_LOG_INFO,"LVDS Output PortB");
#elif LVDSTX_PORT_SEL  == DOU_PORT
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x36,0x03); //lvds enable
    HDMI_WriteI2C_Byte(0x37,0x44); //port rterm enable
    LTLog(LT_LOG_INFO,"LVDS Output PortA&B");
#endif    
    
    HDMI_WriteI2C_Byte(0x38,0x14);
    HDMI_WriteI2C_Byte(0x39,0x31);
    HDMI_WriteI2C_Byte(0x3a,0xc8);
    HDMI_WriteI2C_Byte(0x3b,0x00);
    HDMI_WriteI2C_Byte(0x3c,0x0f);

    HDMI_WriteI2C_Byte(0x46,0x40);
    HDMI_WriteI2C_Byte(0x47,0x40);
    HDMI_WriteI2C_Byte(0x48,0x40);
    HDMI_WriteI2C_Byte(0x49,0x40);
    HDMI_WriteI2C_Byte(0x4a,0x40);
    HDMI_WriteI2C_Byte(0x4b,0x40);
    HDMI_WriteI2C_Byte(0x4c,0x40);
    HDMI_WriteI2C_Byte(0x4d,0x40);
    HDMI_WriteI2C_Byte(0x4e,0x40);
    HDMI_WriteI2C_Byte(0x4f,0x40);
    HDMI_WriteI2C_Byte(0x50,0x40);
    HDMI_WriteI2C_Byte(0x51,0x40);

    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x03,0xbf); //mltx reset
    HDMI_WriteI2C_Byte(0x03,0xff); //mltx release
}

void Drv_LvdsTxPll_RefPixClk_Set(void)
{
	HDMI_WriteI2C_Byte(0xff,0x82);

    HDMI_WriteI2C_Byte(0x30,0x00); //[7]0:txpll normal work; txpll ref clk sel pix clk

}

void Drv_LvdsTxPll_Config(void)
{
    u8 ucPreDiv = 0;
    u8 ucSericlkDiv = 0;
    u8 ucDivSet = 0;
    u8 temp = 1;
    //float ucPixClkDiv = 0;        // ARMv7l不支持浮点运算, 此浮点数用下面两个整数相除代替
    u8 ucPixClkDivNumerator;        // ucPixClkDiv的分子
    u8 ucPixClkDivIntDenominator;   // ucPixClkDiv的分母
    u32 ulLvdsTXPhyClk = 0;

    /* txphyclk = vco clk * ucSericlkDiv */
#if (LVDSTX_PORT_SEL == DOU_PORT)
    ulLvdsTXPhyClk = (u32)(g_stRxVidTiming.ulPclk_Khz * 7 / 2); //2 port: byte clk = pix clk / 2;
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | BIT0_1)); //htotal -> 2n
    temp = 2;
    #if ((LVDSTX_COLORSPACE == YUV422) && (LVDSTX_COLORDEPTH == DEPTH_8BIT) && (LVDSTX_LANENUM == FIVE_LANE))
    ulLvdsTXPhyClk = (u32)(g_stRxVidTiming.ulPclk_Khz * 7 / 4); //2 port YUV422 8bit 5lane: byte clk = pix clk / 4;
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | BIT1_1)); //htotal -> 4n
    temp = 4;
    #endif 
#else
    ulLvdsTXPhyClk = (u32)(g_stRxVidTiming.ulPclk_Khz * 7); //1 port: byte clk = pix clk;
    temp = 1;
    #if ((LVDSTX_COLORSPACE == YUV422) && (LVDSTX_COLORDEPTH == DEPTH_8BIT) && (LVDSTX_LANENUM == FIVE_LANE))
    ulLvdsTXPhyClk = (u32)(g_stRxVidTiming.ulPclk_Khz * 7 / 2); //1 port YUV422 8bit 5lane: byte clk = pix clk / 2;
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | BIT0_1)); //htotal -> 2n
    temp = 2;
    #endif
#endif
    
    
    /*txpll prediv sel*/
    HDMI_WriteI2C_Byte(0xff,0x82);
    if (g_stRxVidTiming.ulPclk_Khz < 20000)
    {
        HDMI_WriteI2C_Byte(0x31,0x28); //[2:0]3'b000: pre div set div1
        ucPreDiv = 1;
    }
    else if (g_stRxVidTiming.ulPclk_Khz >= 20000 && g_stRxVidTiming.ulPclk_Khz < 40000)
    {
        HDMI_WriteI2C_Byte(0x31,0x28); //[2:0]3'b000: pre div set div1
        ucPreDiv = 1;
    }
    else if (g_stRxVidTiming.ulPclk_Khz >= 40000 && g_stRxVidTiming.ulPclk_Khz < 80000)
    {
        HDMI_WriteI2C_Byte(0x31,0x29); //[2:0]3'b001: pre div set div2
        ucPreDiv = 2;
    }
    else if (g_stRxVidTiming.ulPclk_Khz >= 80000 && g_stRxVidTiming.ulPclk_Khz < 160000)
    {
        HDMI_WriteI2C_Byte(0x31,0x2a); //[2:0]3'b010: pre div set div4
        ucPreDiv = 4;
    }
    else if (g_stRxVidTiming.ulPclk_Khz >= 160000 && g_stRxVidTiming.ulPclk_Khz < 320000)
    {
        HDMI_WriteI2C_Byte(0x31,0x2b); //[2:0]3'b011: pre div set div8
        ucPreDiv = 8;
//        ulLvdsTXPhyClk = ulDessc_Pix_Clk * 3.5;
    }
    else if (g_stRxVidTiming.ulPclk_Khz >= 320000)
    {
        HDMI_WriteI2C_Byte(0x31,0x2f); //[2:0]3'b111: pre div set div16
        ucPreDiv = 16;
//        ulLvdsTXPhyClk = ulDessc_Pix_Clk * 3.5;
    }

    /*txpll serick_divsel*/
    HDMI_WriteI2C_Byte(0xff,0x82);
    if (ulLvdsTXPhyClk >= 640000 )//640M~1.28G
    {
        HDMI_WriteI2C_Byte(0x32,0x42);
        ucSericlkDiv = 1; //sericlk div1 [6:4]:0x40
    }
    else if (ulLvdsTXPhyClk >= 320000 && ulLvdsTXPhyClk < 640000)
    {
        HDMI_WriteI2C_Byte(0x32,0x02);
        ucSericlkDiv = 2; //sericlk div2 [6:4]:0x00
    }
    else if (ulLvdsTXPhyClk >= 160000 && ulLvdsTXPhyClk < 320000)
    {
        HDMI_WriteI2C_Byte(0x32,0x12);
        ucSericlkDiv = 4; //sericlk div4 [6:4]:0x10
    }
    else if (ulLvdsTXPhyClk >= 80000 && ulLvdsTXPhyClk < 160000)
    {
        HDMI_WriteI2C_Byte(0x32,0x22);
        ucSericlkDiv = 8; //sericlk div8 [6:4]:0x20
    }
    else //40M~80M
    {
        HDMI_WriteI2C_Byte(0x32,0x32);
        ucSericlkDiv = 16; //sericlk div16 [6:4]:0x30
    }

    /* txpll_pix_mux_sel & txpll_pixdiv_sel*/
    HDMI_WriteI2C_Byte(0xff,0x82);
    if (g_stRxVidTiming.ulPclk_Khz > 150000)
    {
        HDMI_WriteI2C_Byte(0x33,0x04); //pixclk > 150000, pixclk mux sel (vco clk / 3.5)
        //ucPixClkDiv = 3.5;
        ucPixClkDivNumerator = 7;
        ucPixClkDivIntDenominator = 2;
    }
    else
    {
        //ucPixClkDiv = (u8)((ulLvdsTXPhyClk * ucSericlkDiv) / (g_stRxVidTiming.ulPclk_Khz * 7));
        /* ARMv7l不支持浮点运算，修改此else分支中和ucPixClkDiv的代码以规避，修改前后代码功能等价 */
        //ucPixClkDiv = (u8)(ucSericlkDiv/temp);//modify 2013.11.13
        ucPixClkDivNumerator = ucSericlkDiv;
        ucPixClkDivIntDenominator = temp;
        
        if (ucSericlkDiv <= temp) //(ucPixClkDiv <= 1)
        {
            HDMI_WriteI2C_Byte(0x33,0x00); //pixclk div sel /7
        }
        else if (ucSericlkDiv <= (temp * 2)) //((ucPixClkDiv > 1) && (ucPixClkDiv <= 2))
        {
            HDMI_WriteI2C_Byte(0x33,0x01); //pixclk div sel /14
        }
        else if (ucSericlkDiv <= (temp * 4)) //((ucPixClkDiv > 2) && (ucPixClkDiv <= 4))
        {
            HDMI_WriteI2C_Byte(0x33,0x02); //pixclk div sel /28
        }
        else if (ucSericlkDiv <= (temp * 8)) //((ucPixClkDiv > 4) && (ucPixClkDiv <= 8))
        {
            HDMI_WriteI2C_Byte(0x33,0x03); //pixclk div sel /56
        }
        else
        {
            HDMI_WriteI2C_Byte(0x33,0x03); //pixclk div sel /56
        }
    }
    
    //ucDivSet = (u8)((ulLvdsTXPhyClk * ucSericlkDiv) / (g_stRxVidTiming.ulPclk_Khz / ucPreDiv));

    ucDivSet = (u8)((7 * ucSericlkDiv*ucPreDiv)/temp);//2023.11.13 modify ucSericlkDiv*ucPreDiv will be not <4
    
	HDMI_WriteI2C_Byte(0x34,0x01); //txpll div set software output enable
    HDMI_WriteI2C_Byte(0x35,ucDivSet);
    LTLog(LT_LOG_DEBUG,"ulPclk_Khz: %ld, ucPreDiv: %d, ucSericlkDiv: %d, ucPixClkDiv: %u/%u, ucDivSet: %d",
        g_stRxVidTiming.ulPclk_Khz, ucPreDiv, ucSericlkDiv, ucPixClkDivNumerator, ucPixClkDivIntDenominator, ucDivSet);
    LTLog(LT_LOG_INFO,"temp is: 0x%02x", temp);
    
    #if LVDS_SSC_SEL != NO_SSC

    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x34,0x00); //hardware mode
    
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x6e,0x10); //sram select
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x81,((HDMI_ReadI2C_Byte(0x81) & 0xe0) | 0x15));
    
    HDMI_WriteI2C_Byte(0xff,0x87);

    HDMI_WriteI2C_Byte(0x1e,0x00); //modulation disable
    
    HDMI_WriteI2C_Byte(0x17,0x02); //2 order

    
    HDMI_WriteI2C_Byte(0x18,0x04);
    HDMI_WriteI2C_Byte(0x19,0xd4); //ssc_prd

    #if LVDS_SSC_SEL == SSC_1920x1080_30k2
        HDMI_WriteI2C_Byte(0x1a,0x00);
        HDMI_WriteI2C_Byte(0x1b,0x05); //delta1
        HDMI_WriteI2C_Byte(0x1c,0x00);
        HDMI_WriteI2C_Byte(0x1d,0x0a); //delta
        
        HDMI_WriteI2C_Byte(0x1f,0x1c); //M
        HDMI_WriteI2C_Byte(0x20,0x00);
        HDMI_WriteI2C_Byte(0x21,0x00);
        LTLog(LT_LOG_INFO,"LVDS Output SSC 1920x1080 30k5%");
    #elif LVDS_SSC_SEL == SSC_3840x2160_30k2
        HDMI_WriteI2C_Byte(0x1a,0x00);
        HDMI_WriteI2C_Byte(0x1b,0x05); //delta1
        HDMI_WriteI2C_Byte(0x1c,0x00);
        HDMI_WriteI2C_Byte(0x1d,0x0a); //delta
        
        HDMI_WriteI2C_Byte(0x1f,0x1c); //M
        HDMI_WriteI2C_Byte(0x20,0x00);
        HDMI_WriteI2C_Byte(0x21,0x00);
        LTLog(LT_LOG_INFO,"LVDS Output SSC 3840x2160 30k5%");
    #endif

    HDMI_WriteI2C_Byte(0x1e,0x02); //modulation enable
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x0c,0xfe); //txpll reset
    HDMI_WriteI2C_Byte(0x0c,0xff); //txpll release
    
    #endif
}

u8 Drv_LvdsTxPll_Cali(void)
{
    u8 ucloopx=0;
    u8 ucRtn = FAIL;

    HDMI_WriteI2C_Byte(0xff,0x81);
	HDMI_WriteI2C_Byte(0x0c,0xfe); //txpll reset
    Ocm_Timer0_Delay1ms(1);
	HDMI_WriteI2C_Byte(0x0c,0xff); //txpll release

    do
    {
		HDMI_WriteI2C_Byte(0xff,0x87);
		HDMI_WriteI2C_Byte(0x0f,0x00);
		HDMI_WriteI2C_Byte(0x0f,0x01);
		Ocm_Timer0_Delay1ms(20);

		ucloopx++;
	}while((ucloopx < 3) && ((HDMI_ReadI2C_Byte(0x39) & 0x01) != 0x01));

    if(HDMI_ReadI2C_Byte(0x39) & 0x04)
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

void Drv_LvdsTx_VidTiming_Set(void)
{
    u16 vss,eav,sav;
	Ocm_Timer0_Delay1ms(100);
	HDMI_WriteI2C_Byte(0xff,0x85); 
    
    vss = g_stVidChk.usVs + g_stVidChk.usVbp;
    eav = g_stVidChk.usHs + g_stVidChk.usHbp + g_stVidChk.usHact + 4;
    sav = g_stVidChk.usHs + g_stVidChk.usHbp;
    
    
    HDMI_WriteI2C_Byte(0x5f,0x00);    
	HDMI_WriteI2C_Byte(0x60,0x00);  
    HDMI_WriteI2C_Byte(0x62,(u8)(g_stVidChk.usVact>>8));         //vact[15:8]
	HDMI_WriteI2C_Byte(0x61,(u8)(g_stVidChk.usVact));            //vact[7:0]
    HDMI_WriteI2C_Byte(0x63,(u8)(vss));                           //vss[7:0]
    HDMI_WriteI2C_Byte(0x65,(u8)(eav>>8));                        //eav[15:8]
	HDMI_WriteI2C_Byte(0x64,(u8)(eav));                           //eav[7:0]
    HDMI_WriteI2C_Byte(0x67,(u8)(sav>>8));                        //sav[15:8]
	HDMI_WriteI2C_Byte(0x66,(u8)(sav));                           //sav[7:0]	
    
}


void Drv_LvdsTxPort_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
#if ((LVDSTX_PORT_SEL == PORTA) || (LVDSTX_PORT_SEL == PORTB))
	HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0x80)); //[7]lvds function enable //[4]0:output 1port; [4]1:output 2port;
	//only portb output must use port copy from porta, so lvds digtial output port sel 2ports.
#elif (LVDSTX_PORT_SEL == DOU_PORT)
	HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0x90)); //[7]lvds function enable //[4]0:output 1port; [4]1:output 2port;
#endif 
}

void Drv_LvdsTxVidFmt_Set()
{
    HDMI_WriteI2C_Byte(0xff,0x85); 
#if (LVDSTX_MODE == SYNC_MODE)
    HDMI_WriteI2C_Byte(0X6e,(HDMI_ReadI2C_Byte(0x6e) & BIT3_0));
#elif (LVDSTX_MODE == DE_MODE)
    HDMI_WriteI2C_Byte(0X6e,(HDMI_ReadI2C_Byte(0x6e) | BIT3_1)); //[3]lvdstx de mode
#endif

#if (LVDSTX_DATAFORMAT == JEIDA)
    HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | BIT6_1)); //[6]1:JEIDA MODE
    LTLog(LT_LOG_INFO,"Data Format: JEIDA");
#elif (LVDSTX_DATAFORMAT == VESA)
    HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) & BIT6_0)); //[6]0:VESA MODE;
    LTLog(LT_LOG_INFO,"Data Format: VESA");
#endif

#if (LVDSTX_COLORSPACE == RGB)
    LTLog(LT_LOG_INFO,"ColorSpace: RGB");
    #if (LVDSTX_COLORDEPTH == DEPTH_6BIT)
        LTLog(LT_LOG_INFO,"ColorDepth: 6Bit");
        HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0x40)); //RGB666 [6]RGB666 output must select jeida mode
        HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) & 0xf3));
    #elif (LVDSTX_COLORDEPTH == DEPTH_8BIT)
        LTLog(LT_LOG_INFO,"ColorDepth: 8Bit");
        HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0X04));
    #elif (LVDSTX_COLORDEPTH == DEPTH_10BIT)
        LTLog(LT_LOG_INFO,"ColorDepth: 10Bit");
        HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0X0c));
    #endif

#elif (LVDSTX_COLORSPACE == YUV422)
    LTLog(LT_LOG_INFO,"ColorSpace: YUV422");    
    HDMI_WriteI2C_Byte(0xff,0x85);
    #if (LVDSTX_COLORDEPTH == DEPTH_8BIT)
        LTLog(LT_LOG_INFO,"ColorDepth: 8Bit");
        HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0X04));
        #if (LVDSTX_LANENUM == FIVE_LANE)

                LTLog(LT_LOG_INFO,"LvdsLaneNum: 5Lane");
                HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0X40)); //YUV422-8bpc-5lane mode output must sel jeida mode
                HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0X28));  //YUV422-8bpc-5lane mode set
        #else
                LTLog(LT_LOG_INFO,"LvdsLaneNum: 4Lane");
                HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) & 0Xbf)); //YUV422-8bpc-4lane mode output must sel vesa mode
        #endif
    #endif
#endif

#if (LVDSTX_SYNC_INTER_MODE == ENABLED)
    Drv_LvdsTx_VidTiming_Set();    
    LTLog(LT_LOG_INFO,"Lvds Sync Code Mode: Internal"); //internal sync code mode
	HDMI_WriteI2C_Byte(0x68,(HDMI_ReadI2C_Byte(0x68) | 0X01));
    #if (LVDSTX_VIDEO_FORMAT == I_FORMAT)
        LTLog(LT_LOG_INFO,"Lvds Video Format: interlaced"); //internal sync code mode
	    HDMI_WriteI2C_Byte(0x68,(HDMI_ReadI2C_Byte(0x68) | 0X02));
    #endif
    #if (LVDSTX_SYNC_CODE_SEND == REPECTIVE)
        LTLog(LT_LOG_INFO,"Lvds Sync Code Send: respectively."); //sync code send method sel respectively
	    HDMI_WriteI2C_Byte(0x68,(HDMI_ReadI2C_Byte(0x68) | 0X04));
    #endif

#else
	HDMI_WriteI2C_Byte(0x68,0x00);
#endif 

}

void Drv_LvdsTxLaneNum_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
	HDMI_WriteI2C_Byte(0x4a,0x01); //[0]hl_swap_en; [7:6]tx_pt0_src_sel: 0-pta;1-ptb
	HDMI_WriteI2C_Byte(0x4b,0x00);
	HDMI_WriteI2C_Byte(0x4c,0x10);
	HDMI_WriteI2C_Byte(0x4d,0x20);
	HDMI_WriteI2C_Byte(0x4e,0x50);
	HDMI_WriteI2C_Byte(0x4f,0x30);

#if (LVDSTX_LANENUM  == FOUR_LANE)
    	HDMI_WriteI2C_Byte(0x50,0x46); //[7:6]tx_pt1_src_sel: 0-pta;1-ptb
    	HDMI_WriteI2C_Byte(0x51,0x10);
    	HDMI_WriteI2C_Byte(0x52,0x20);
    	HDMI_WriteI2C_Byte(0x53,0x50);
    	HDMI_WriteI2C_Byte(0x54,0x30);
    	HDMI_WriteI2C_Byte(0x55,0x00); //[7:4]pt1_tx4_src_sel
    	HDMI_WriteI2C_Byte(0x56,0x20); //[3:0]pt1_tx5_src_sel
                                       //[6:5]rgd_mltx_src_sel: 0-mipitx;1-lvdstx
#elif (LVDSTX_LANENUM == FIVE_LANE)
        HDMI_WriteI2C_Byte(0x50,0x44); //[7:6]tx_pt1_src_sel: 0-pta;1-ptb
    	HDMI_WriteI2C_Byte(0x51,0x00);
    	HDMI_WriteI2C_Byte(0x52,0x10);
    	HDMI_WriteI2C_Byte(0x53,0x20);
    	HDMI_WriteI2C_Byte(0x54,0x50);
    	HDMI_WriteI2C_Byte(0x55,0x30); //[7:4]pt1_tx4_src_sel
    	HDMI_WriteI2C_Byte(0x56,0x24); //[3:0]pt1_tx5_src_sel
                                       //[6:5]rgd_mltx_src_sel: 0-mipitx;1-lvdstx

#if 0
        //swap when 9211 lvdstx_to_lvdsrx 2port 5lane link
        HDMI_WriteI2C_Byte(0x54,0x30);
        HDMI_WriteI2C_Byte(0x55,0x40);
#endif

#endif

}

void Drv_LvdsTxPort_Swap(void)
{
    #if LVDSTX_PORT_SEL == DOU_PORT
        #if LVDSTX_PORT_SWAP == ENABLED
            HDMI_WriteI2C_Byte(0xff,0x85);
            HDMI_WriteI2C_Byte(0x4a,0x41);
            HDMI_WriteI2C_Byte(0x50,(HDMI_ReadI2C_Byte(0x50) & BIT6_0));
            LTLog(LT_LOG_INFO,"2Port Lvds Port Swap");
        #endif
    
    #elif LVDSTX_PORT_SEL == PORTB
        HDMI_WriteI2C_Byte(0xff,0x85);
        HDMI_WriteI2C_Byte(0x4a,0x41);
        HDMI_WriteI2C_Byte(0x50,(HDMI_ReadI2C_Byte(0x50) & BIT6_0));

    #else 
        HDMI_WriteI2C_Byte(0xff,0x85);
        HDMI_WriteI2C_Byte(0x4a,0x01);
        HDMI_WriteI2C_Byte(0x50,(HDMI_ReadI2C_Byte(0x50) | BIT6_1));
    #endif
}

void Drv_LvdsTxPort_Copy(void)
{

    #if ((LVDSTX_PORT_COPY == ENABLED) &&  (LVDSTX_PORT_SEL == PORTA))
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) | 0x03)); //port swap enable when porta & portb enable

//        #if (LVDSTX_PORT_SEL == PORTA)
            HDMI_WriteI2C_Byte(0xff,0x85);
            HDMI_WriteI2C_Byte(0x4a,(HDMI_ReadI2C_Byte(0x4a) & 0xbf));
            HDMI_WriteI2C_Byte(0x50,(HDMI_ReadI2C_Byte(0x50) & 0xbf));
            LTLog(LT_LOG_INFO,"Port A Copy");
//        #elif (LVDSTX_PORT_SEL == PORTB)
//            HDMI_WriteI2C_Byte(0xff,0x85);
//            HDMI_WriteI2C_Byte(0x6f,(HDMI_ReadI2C_Byte(0x6f) | 0x10)); //[7]lvds function enable //[4]0:output 1port; [4]1:output 2port;
//            HDMI_WriteI2C_Byte(0x4a,(HDMI_ReadI2C_Byte(0x4a) | 0x40));
//            HDMI_WriteI2C_Byte(0x50,(HDMI_ReadI2C_Byte(0x50) | 0x40));
//            LTLog(LT_LOG_INFO,"Port B Copy");
//        #endif
    
    #endif

}



#endif
