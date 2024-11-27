#ifndef __LT9211C_LOG_H__
#define __LT9211C_LOG_H__

#define MAX_NUMBER_BYTES  128

typedef enum
{
    LT_LOG_DEBUG =0x00,
    LT_LOG_INFO,
    LT_LOG_WARN,
    LT_LOG_ERROR,
    LT_LOG_CRITICAL,
    LT_LOG_NOTRACE,
} LT_LogLevel;

void LTLog(unsigned char ucLvl, const char *fmt, ...);

#endif // __LT9211C_LOG_H__
