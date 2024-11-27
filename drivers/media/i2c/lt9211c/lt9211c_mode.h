
#ifndef __LT9211C_MODE_H__
#define __LT9211C_MODE_H__


#define         LT9211C_VERSION              U2             //U1/U2
#define         LT9211C_MODE_SEL             PATTERN_OUT
/*
LVDS_IN_LVDS_OUT  LVDS_IN_MIPI_OUT  LVDS_IN_TTL_OUT
MIPI_IN_LVDS_OUT  MIPI_IN_MIPI_OUT  MIPI_IN_TTL_OUT
TTL_IN_LVDS_OUT   TTL_IN_MIPI_OUT   TTL_IN_TTL_OUT     //TTL_OUT时需注意ttltx 8264寄存器配置
MIPI_REPEATER     MIPI_LEVEL_SHIFT  PATTERN_OUT
*/


#define         TX_PATTERN_SRC_SEL          TTLTX_PATTERN   //NO_TX_PATTERN/MIPITX_PATTERN/LVDSTX_PATTERN/TTLTX_PATTERN
#define         TX_VID_PATTERN_SEL          PTN_1280x720_60  //NO_PATTERN/PTN_640x480_15/PTN_640x480_60/PTN_720x480_60/PTN_1280x720_60/PTN_1920x720_60/PTN_1920x1080_30
                                                              //PTN_1920x1080_60/PTN_1920x1080_90/PTN_1920x1080_100/PTN_2560x1440_60/PTN_3840x2160_30

#if LT9211C_MODE_SEL == PATTERN_OUT

/*==============MIPITX PATTERN SETTING==============*/

#define         MIPITX_PATTERN_OUT_SEL      MIPI_DSI         //MIPI_DSI/MIPI_CSI
#define         MIPITX_PATTERN_COLORSPACE   RGB_888          //RGB_888/RGB_666L/RGB_666/RGB_565/YUV422
#define         MIPITX_PATTERN_OUTPUT_LANE  MIPITX_4LANE     //MIPITX_4LANE/MIPITX_3LANE/MIPITX_2LANE/MIPITX_1LANE
#define         MIPITX_PATTERN_CLOCK_BURST  DISABLED         //ENABLED/DISABLED
#define         MIPITX_PATTERN_OUTPUT_PORT  PORTA            //PORTA/PORTB


/*==============LVDSTX PATTERN SETTING==============*/

#define     LVDSTX_PORT_SEL         DOU_PORT        //PORTA/PORTB/DOU_PORT
#define     LVDSTX_PORT_SWAP        DISABLED         //ENABLED/DISABLED
#define     LVDSTX_PORT_COPY        DISABLED         //ENABLED/DISABLED
#define     LVDSTX_LANENUM          FOUR_LANE       //FOUR_LANE/FIVE_LANE
#define     LVDSTX_MODE             SYNC_MODE       //SYNC_MODE/DE_MODE
#define     LVDSTX_DATAFORMAT       VESA            //VESA/JEIDA
#define     LVDSTX_COLORSPACE       RGB             //RGB/YUV422
#define     LVDSTX_COLORDEPTH       DEPTH_8BIT      //DEPTH_6BIT/DEPTH_8BIT/DEPTH_10BIT
#define     LVDSTX_SYNC_INTER_MODE  DISABLED         //ENABLED/DISABLED
#define     LVDSTX_VIDEO_FORMAT     P_FORMAT        //P_FORMAT/I_FORMAT
#define     LVDSTX_SYNC_CODE_SEND   NON_REPECTIVE       ///NON_REPECTIVE/REPECTIVE
#define     LVDS_SSC_SEL            NO_SSC          //NO_SSC/SSC_1920x1080_30k5/SSC_3840x2160_30k5

/*==============TTLTX PATTERN SETTING==============*/

//basic settings
#define     TTLTX_MODE                  SDR             //SDR/DDR/DPI
#define     TTLTX_OUTPUT_MODE           OUTPUT_RGB888   //OUTPUT_RGB888    OUTPUT_RGB666    OUTPUT_RGB565    OUTPUT_YCBCR444
                                                        //OUTPUT_YCBCR422_16BIT  OUTPUT_YCBCR422_20BIT    OUTPUT_YCBCR422_24BIT
                                                        //OUTPUT_BT656_8BIT      OUTPUT_BT656_10BIT       OUTPUT_BT656_12BIT
                                                        //OUTPUT_BT1120_8BIT     OUTPUT_BT1120_10BIT      OUTPUT_BT1120_12BIT
                                                        //OUTPUT_BTA_T1004_16BIT OUTPUT_BTA_T1004_20BIT   OUTPUT_BTA_T1004_24BIT
                                                        //OUTPUT_BT1120_16BIT    OUTPUT_BT1120_20BIT      OUTPUT_BT1120_24BIT



//vedio settings
#define     TTLTX_VIDEO_FORMAT          P_FORMAT        //P_FORMAT/I_FORMAT
#define     TTLTX_VSYNC_POLARITY        PLUS            //PLUS/MINUS
#define     TTLTX_HSYNC_POLARITY        PLUS            //PLUS/MINUS


#endif

int lt9211c_main(void *data);

#endif
