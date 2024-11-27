


#ifndef _INCLUDE_H
#define _INCLUDE_H

//device driver
#include <stdarg.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
//#include <stdio.h>
//#include <math.h>
#include "lt9211c_log.h"
#include "lt9211c_type.h"
#include "lt9211c_module.h"

//App						 
#include "lt9211c_mode.h"

//Driver
#include "Driver/DrvCsc.h"
#include "Driver/DrvMipiRx.h"
#include "Driver/DrvMipiTx.h"
#include "Driver/DrvLvdsRx.h"
#include "Driver/DrvLvdsTx.h"
#include "Driver/DrvTtlRx.h"
#include "Driver/DrvTtlTx.h"
#include "Driver/DrvDcsCmd.h"
#include "Driver/DrvSystem.h"
#include "Driver/DrvMipiRpt.h"
#include "Driver/DrvMipiLs.h"

//Module
#include "Module/ModTtlRx.h"
#include "Module/ModTtlTx.h"
#include "Module/ModLvdsTx.h"
#include "Module/ModLvdsRx.h"
#include "Module/ModMipiTx.h"
#include "Module/ModMipiRx.h"
#include "Module/ModSystem.h"
#include "Module/ModPattern.h"
#include "Module/ModMipiRpt.h"
#include "Module/ModMipiLs.h"

#define HDMI_WriteI2C_Byte  lt9211c_write_register
#define HDMI_ReadI2C_Byte   lt9211c_read_register_2
#define Ocm_Timer0_Delay1ms msleep

#endif
