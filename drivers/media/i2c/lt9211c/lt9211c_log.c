/******************************************************************************
  * @project: LT9211C
  * @file: lt9211c_log.c
  * @author: sxue
  * @company: LONTIUM COPYRIGHT and CONFIDENTIAL
  * @date: 2023.01.29
  *****************************************************************************/

#include <linux/printk.h>
#include <linux/kernel.h>
#include "lt9211c_log.h"

static unsigned char g_ucLogLevel = LT_LOG_DEBUG;
static const char * const messageTypeStr[] = {"DEBUG","INFO","WARN", "ERROR","CRIT"};

void Ocm_PrintLevel_Set(unsigned char ucLvl)
{
    g_ucLogLevel = ucLvl;
}

void LTLog(unsigned char ucLvl, const char *fmt, ...)
{
    char buf[MAX_NUMBER_BYTES];
    va_list args;
    int len;

    if (ucLvl > LT_LOG_NOTRACE) {
        return;
    }
    
    //打印大于等于该级别的字符串
    if(ucLvl >= g_ucLogLevel) {
        va_start(args, fmt);
        // 预留空间给前缀
        len = vsnprintf(buf, sizeof(buf) - 32, fmt, args);
        va_end(args);
        
        if (len > 0) {
            switch(ucLvl) {
            case LT_LOG_DEBUG:
                pr_debug("\n[%-5s] %s", messageTypeStr[ucLvl], buf);
                break;
            case LT_LOG_INFO:
                pr_info("\n[%-5s] %s", messageTypeStr[ucLvl], buf);
                break;
            case LT_LOG_WARN:
                pr_warn("\n[%-5s] %s", messageTypeStr[ucLvl], buf);
                break;
            case LT_LOG_ERROR:
            case LT_LOG_CRITICAL:
                pr_err("\n[%-5s] %s", messageTypeStr[ucLvl], buf);
                break;
            default:
                break;
            }
        }
    }
}