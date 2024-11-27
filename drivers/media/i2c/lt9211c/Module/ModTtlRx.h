/******************************************************************************
  * @project: LT9211C
  * @file: ModTtlRx.h
  * @author: xding
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#ifndef _MODTTLRX_H
#define _MODTTLRX_H

#if ((LT9211C_MODE_SEL == TTL_IN_LVDS_OUT)||(LT9211C_MODE_SEL == TTL_IN_MIPI_OUT)||(LT9211C_MODE_SEL == TTL_IN_TTL_OUT))


#define     TTLRX_MODE                  SDR                 //SDR/DDR/DPI
#define     TTLRX_PHASE_SEL             9            //0~9:phase sel 0~9  //10: print all phases
#define     TTLRX_DE_SYNC_MODE          TTL_SYNC_MODE     //TTL_NORMAL_MODE/TTL_SYNC_MODE/TTL_DE_MODE  //use in separate sync mode
#define     TTLRX_DE_SYNC_MODE_HACT     3840
#define     TTLRX_DE_SYNC_MODE_VACT     2160
#define     TTLRX_VIDEO_FORMAT          P_FORMAT            //P_FORMAT/I_FORMAT
#define     CSC_RX_MODE                 RGB             //RGB/YUV422
#if TTLRX_MODE == SDR
/*==============TTLRX SDR MODE SETTING==============*/
    #define     TTLRX_SYNC_INTER_MODE       DISABLED        //ENABLED/DISABLED
  
    #define     TTLRX_INPUT_MODE            INPUT_RGB888    //INPUT_RGB888    INPUT_RGB666    INPUT_RGB565    INPUT_YCBCR444
                                                            //INPUT_YCBCR422_16BIT  INPUT_YCBCR422_20BIT    INPUT_YCBCR422_24BIT
                                                            //INPUT_BT656_8BIT      INPUT_BT656_10BIT       INPUT_BT656_12BIT
                                                            //INPUT_BT1120_8BIT     INPUT_BT1120_10BIT      INPUT_BT1120_12BIT
                                                            //INPUT_BTA_T1004_16BIT INPUT_BTA_T1004_20BIT   INPUT_BTA_T1004_24BIT
                                                            //INPUT_BT1120_16BIT    INPUT_BT1120_20BIT      INPUT_BT1120_24BIT
/*==============TTLRX SDR MODE SETTING==============*/
#endif

#if TTLRX_MODE == DDR
/*==============TTLRX DDR MODE SETTING==============*/
    #define     TTLRX_SYNC_INTER_MODE       ENABLED        //ENABLED/DISABLED
   
    #define     TTLRX_INPUT_MODE            INPUT_BT1120_16BIT    //INPUT_RGB888    INPUT_RGB666    INPUT_RGB565    INPUT_YCBCR444
                                                            //INPUT_YCBCR422_16BIT  INPUT_YCBCR422_20BIT    INPUT_YCBCR422_24BIT
                                                            //INPUT_BT656_8BIT      INPUT_BT656_10BIT       INPUT_BT656_12BIT
                                                            //INPUT_BT1120_8BIT     INPUT_BT1120_10BIT      INPUT_BT1120_12BIT
                                                            //INPUT_BTA_T1004_16BIT INPUT_BTA_T1004_20BIT   INPUT_BTA_T1004_24BIT
                                                            //INPUT_BT1120_16BIT    INPUT_BT1120_20BIT      INPUT_BT1120_24BIT
/*==============TTLRX DDR MODE SETTING==============*/
#endif

#if TTLRX_MODE == DPI
/*==============TTLRX DPI MODE SETTING==============*/
    #define     TTLRX_SYNC_INTER_MODE       DISABLED        //DISABLED
  
    #define     TTLRX_INPUT_MODE            INPUT_RGB888    //INPUT_RGB888    
/*==============TTLRX DPI MODE SETTING==============*/
#endif


void Mod_TtlRx_Handler(void);

#endif
#endif