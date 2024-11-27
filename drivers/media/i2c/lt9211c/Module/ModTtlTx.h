/******************************************************************************
  * @project: LT9211C
  * @file: ModTtlTx.h
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#ifndef _MODTTLTX_H
#define _MODTTLTX_H

#if ((LT9211C_MODE_SEL == LVDS_IN_TTL_OUT)||(LT9211C_MODE_SEL == MIPI_IN_TTL_OUT)||(LT9211C_MODE_SEL == TTL_IN_TTL_OUT))

#define     TTLTX_MODE                  SDR             //SDR/DDR/DPI
#define     TTLTX_VSYNC_POLARITY        PLUS            //PLUS/MINUS
#define     TTLTX_HSYNC_POLARITY        PLUS            //PLUS/MINUS
#define     TTLTX_VIDEO_FORMAT          P_FORMAT        //P_FORMAT/I_FORMAT
#define     CSC_TX_MODE                 YUV422             //RGB/YUV422
//TTLTX SDR MODE SETTING
#if TTLTX_MODE == SDR
/*==============TTLTX SDR MODE SETTING==============*/
    #define     TTLTX_SYNC_INTER_MODE       DISABLED            //ENABLED/DISABLED
  
    #define     TTLTX_OUTPUT_MODE           OUTPUT_RGB888       //OUTPUT_RGB888    OUTPUT_RGB666    OUTPUT_RGB565    OUTPUT_YCBCR444
                                                                //OUTPUT_YCBCR422_16BIT  OUTPUT_YCBCR422_20BIT    OUTPUT_YCBCR422_24BIT
                                                                //OUTPUT_BT656_8BIT      OUTPUT_BT656_10BIT       OUTPUT_BT656_12BIT
                                                                //OUTPUT_BT1120_8BIT     OUTPUT_BT1120_10BIT      OUTPUT_BT1120_12BIT
                                                                //OUTPUT_BTA_T1004_16BIT OUTPUT_BTA_T1004_20BIT   OUTPUT_BTA_T1004_24BIT
                                                                //OUTPUT_BT1120_16BIT    OUTPUT_BT1120_20BIT      OUTPUT_BT1120_24BIT
/*==============TTLTX SDR MODE SETTING==============*/
#endif

//TTLTX DDR MODE SETTING
#if TTLTX_MODE == DDR
/*==============TTLTX DDR MODE SETTING==============*/
    #define     TTLTX_SYNC_INTER_MODE       ENABLED            //ENABLED/DISABLED
  
    #define     TTLTX_OUTPUT_MODE           OUTPUT_BT1120_16BIT       //OUTPUT_RGB888    OUTPUT_RGB666    OUTPUT_RGB565    OUTPUT_YCBCR444
                                                                //OUTPUT_YCBCR422_16BIT  OUTPUT_YCBCR422_20BIT    OUTPUT_YCBCR422_24BIT
                                                                //OUTPUT_BT656_8BIT      OUTPUT_BT656_10BIT       OUTPUT_BT656_12BIT
                                                                //OUTPUT_BT1120_8BIT     OUTPUT_BT1120_10BIT      OUTPUT_BT1120_12BIT
                                                                //OUTPUT_BTA_T1004_16BIT OUTPUT_BTA_T1004_20BIT   OUTPUT_BTA_T1004_24BIT
                                                                //OUTPUT_BT1120_16BIT    OUTPUT_BT1120_20BIT      OUTPUT_BT1120_24BIT
/*==============TTLTX DDR MODE SETTING==============*/
#endif

//TTLTX DDR MODE SETTING
#if TTLTX_MODE == DPI
/*==============TTLTX DPI MODE SETTING==============*/
    #define     TTLTX_SYNC_INTER_MODE       DISABLED            //DISABLED
 
    #define     TTLTX_OUTPUT_MODE           OUTPUT_RGB888       //OUTPUT_RGB888    
/*==============TTLTX DPI MODE SETTING==============*/
#endif

void Mod_TtlTx_Handler(void);

#endif
#endif