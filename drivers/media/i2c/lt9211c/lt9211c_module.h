/*
 * Lontium LT9211C converter driver Header File
 *
 * Copyright (C) 2024 Lontium Semiconductor Co., Ltd.
 */

#ifndef	__DRIVERS_MEDIA_VIDEO_LT9211C_H__
#define	__DRIVERS_MEDIA_VIDEO_LT9211C_H__

#include <linux/cdev.h>

/* IOCTL参数结构体 */
typedef struct
{
    u8 address; // 寄存器地址
    u8 value;   // 寄存器值
} LT9211c_Control_Args;

/* IOCTL函数使用的CMD(user space need to include <sys/ioctl.h>) */
#define LT9211C_IOCTL_RESET_DEVICE  _IO('L', 0)
#define LT9211C_IOCTL_GET_DATA      _IOR('L', 1, LT9211c_Control_Args)
#define LT9211C_IOCTL_SET_DATA      _IOW('L', 2, LT9211c_Control_Args)

/* LT9211C字符设备结构体 */
typedef struct
{
    dev_t  dev_id;                  // 设备号
    struct cdev cdev;               // 字符设备
    struct class *class;            // 注册字符设备使用的类
    struct device *device;          // 设备
    int major;                      // 主设备号
    int minor;                      // 次设备号
    struct device_node *node;       // 设备节点
    struct i2c_client *client;      // i2c client
    struct gpio_desc *reset_gpio;   // reset引脚
    struct mutex reset_lock;        // 复位芯片和正常配置流程之间的互斥锁, 防止复位时另一线程还在配置
    struct mutex i2c_lock;          // 使用i2c资源的互斥锁, 防止多线程访问
    struct task_struct *task;       // 配置线程
} LT9211c_Dev;

int lt9211c_read_register(u8 address, u8 *value);
char lt9211c_read_register_2(u8 address);
int lt9211c_write_register(u8 address, u8 value);
int lt9211c_reset(void);

#endif	/* __DRIVERS_MEDIA_VIDEO_LT9211C_H__ */
