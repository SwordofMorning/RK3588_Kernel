/*
 * Lontium LT9211C converter driver
 *
 * Copyright (C) 2024 Lontium Semiconductor Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pinctrl/consumer.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include "lt9211c_module.h"
#include "lt9211c_type.h"
#include "lt9211c_mode.h"

LT9211c_Dev lt9211c;    // LT9211C设备

int lt9211c_read_register(u8 address, u8 *value);
int lt9211c_write_register(u8 address, u8 value);
int lt9211c_reset(void);

static int lt9211c_open(struct inode *inode, struct file *filp)
{
    //printk("lt9211c_open\n"); // test code
    filp->private_data = &lt9211c;

    return 0;
}

static int lt9211c_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t lt9211c_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    printk("lt9211c read not support, use ioctl to read!\n");
    return 0;
}

static ssize_t lt9211c_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
    printk("lt9211c write not support, use ioctl to write!\n");
    return 0;
}

static long lt9211c_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
    long ret;
    LT9211c_Control_Args parameter;

    //printk("lt9211c_ioctl\n"); // test code

    if (copy_from_user(&parameter, (void *)args, sizeof(LT9211c_Control_Args)))
    {
        printk("lt9211c ioctl copy from user failed!\n");
        return -EINVAL;
    }

    switch (cmd)
    {
        case LT9211C_IOCTL_RESET_DEVICE:
        {
            //printk("lt9211c ioctl reset parameter, address:0x%02x, value:0x%02x\n", parameter.address, parameter.value); // test code
            ret = lt9211c_reset();
            break;
        }
        case LT9211C_IOCTL_GET_DATA:
        {
            //printk("lt9211c ioctl get parameter, address:0x%02x, value:0x%02x\n", parameter.address, parameter.value); // test code
            ret = lt9211c_read_register(parameter.address, &parameter.value);
            break;
        }
        case LT9211C_IOCTL_SET_DATA:
        {
            //printk("lt9211c ioctl set parameter, address:0x%02x, value:0x%02x\n", parameter.address, parameter.value); // test code
            ret = lt9211c_write_register(parameter.address, parameter.value);
            break;
        }
        default:
        {
            printk("lt9211c ioctl cmd not support\n");
            ret = -EINVAL;
            break;
        }
    }

    if (ret == 0)
    {
        if (copy_to_user((void *)args, &parameter, sizeof(LT9211c_Control_Args)))
        {
            printk("lt9211c ioctl copy to user failed!\n");
            return -EIO;
        }
    }

    return ret;
}

static struct file_operations lt9211c_fops =
{
    .owner          = THIS_MODULE,
    .open           = lt9211c_open,
    .read           = lt9211c_read,
    .write          = lt9211c_write,
    .release        = lt9211c_release,
    .unlocked_ioctl = lt9211c_ioctl,
};

/* Read 1 register at a time
 * client: i2c client
 * address: register address
 * value: register value output
 * return: 0-successful, other: failed */
static int lt9211c_register_read(struct i2c_client *client, u8 address, u8 *value)
{
    struct i2c_msg msgs[2];
    u8 data = 0;
    int ret;

    /* Write register address */
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &address;

    /* Read data from register */
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = &data;

    mutex_lock(&lt9211c.i2c_lock); // or mutex_lock_interruptible(&lt9211c.i2c_lock)
    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    mutex_unlock(&lt9211c.i2c_lock); // 前面使用mutex_lock_interruptible也是用mutex_unlock释放
    if (ret != ARRAY_SIZE(msgs))
    {
        return -EIO;
    }

    *value = data;

    return 0;
}

/* Write 1 register at a time
 * client: i2c client
 * address: register address
 * value: register value to be written
 * return: 0-successful, other: failed */
static int lt9211c_register_write(struct i2c_client *client, u8 address, u8 value)
{
    struct i2c_msg msgs[1];
    u8 data[2] = { address, value};
    int ret;

    /* Write register address, and then the data */
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = data;

    mutex_lock(&lt9211c.i2c_lock); // or mutex_lock_interruptible(&lt9211c.i2c_lock)
    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    mutex_unlock(&lt9211c.i2c_lock); // 前面使用mutex_lock_interruptible也是用mutex_unlock释放
    if (ret != ARRAY_SIZE(msgs))
    {
        return -EIO;
    }

    return 0;
}

/* Read 1 register at a time
 * address: register address
 * return: return the value of the register on success, return minus number on failed */
char lt9211c_read_register_2(u8 address)
{
    int ret;
    u8 value = 0;

    ret = lt9211c_register_read(lt9211c.client, address, &value);
    if (ret == 0)
    {
        return value;
    }
    else
    {
        return (char)ret;
    }
}

/* Read 1 register at a time
 * address: register address
 * value: register value output
 * return: 0-successful, other: failed */
int lt9211c_read_register(u8 address, u8 *value)
{
    return lt9211c_register_read(lt9211c.client, address, value);
}

/* Write 1 register at a time
 * address: register address
 * value: register value to be written
 * return: 0-successful, other: failed */
int lt9211c_write_register(u8 address, u8 value)
{
    return lt9211c_register_write(lt9211c.client, address, value);
}

/* lt9211c reset */
int lt9211c_reset(void)
{
    if (IS_ERR(lt9211c.reset_gpio))
    {
        printk("lt9211c reset gpio NULL, reset failed\n");
        return -EIO;
    }

    mutex_lock(&lt9211c.reset_lock);
    gpiod_direction_output(lt9211c.reset_gpio, 1);
    msleep(100);
    gpiod_direction_output(lt9211c.reset_gpio, 0);
    msleep(100);
    //gpiod_direction_output(lt9211c.reset_gpio, 1); // maybe needed

    printk("lt9211c reset successfully!\n");

    /* 芯片复位后寄存器恢复默认值，根据产品需求确认是否需要重新配置
     * 配置流程有控制变量的，芯片复位时需要同步恢复控制变量的值，避免配置出错 */

    mutex_unlock(&lt9211c.reset_lock);

    return 0;
}

/* 创建lt9211设备节点，创建成功或失败不影响底层驱动和芯片初始化，不关心返回值 */
static void lt9211c_device_create(void)
{
    int ret;

    ret = alloc_chrdev_region(&lt9211c.dev_id, 0, 1, "lt9211c"); /* 申请设备号 */
    if (ret < 0)
    {   
        printk("lt9211c probe, alloc device id failed, ret:%d\n", ret);
        return;
    }
    else
    {
        printk("lt9211c probe, alloc device major id:%d, minor id:%d\n", MAJOR(lt9211c.dev_id), MINOR(lt9211c.dev_id));
    }

    cdev_init(&lt9211c.cdev, &lt9211c_fops);

    /* 添加一个字符设备 */
    ret = cdev_add(&lt9211c.cdev, lt9211c.dev_id, 1);
    if (ret < 0)
    {
        printk("lt9211c probe, alloc device id failed, ret:%d\n", ret);
        unregister_chrdev_region(lt9211c.dev_id, 1); // 释放设备号
        return;
    }

    /* 创建字符设备类 */
    lt9211c.class = class_create(THIS_MODULE, "lt9211c");
    if (IS_ERR(lt9211c.class))
    {
        printk("lt9211c probe, create class failed\n");
        cdev_del(&lt9211c.cdev); // 删除字符设备
        unregister_chrdev_region(lt9211c.dev_id, 1); // 释放设备号
        return;
    }

    /* 创建字符设备 */
    lt9211c.device = device_create(lt9211c.class, NULL, lt9211c.dev_id, NULL, "lt9211c");
    if (IS_ERR(lt9211c.device))
    {
        printk("lt9211c probe, create device failed\n");
        class_destroy(lt9211c.class); // 销毁类
        cdev_del(&lt9211c.cdev); // 删除字符设备
        unregister_chrdev_region(lt9211c.dev_id, 1); // 释放设备号
        return;
    }
}

static int lt9211c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    //printk("==tjf==lt9211c_probe enter.");
    lt9211c_device_create();

    lt9211c.client = client; // 保存client
    printk("lt9211c i2c address:0x%02x\n", lt9211c.client->addr); // test code

    lt9211c.reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_ASIS); // 获取复位GPIO
    if (IS_ERR(lt9211c.reset_gpio))
    {
        printk("lt9211c probe, get reset-gpios failed\n");
    }

    /* 互斥量初始化 */
    mutex_init(&lt9211c.reset_lock);
    mutex_init(&lt9211c.i2c_lock);

    /* 创建并启动配置线程 */
    lt9211c.task = kthread_run(lt9211c_main, &lt9211c.reset_lock, "lt9211c_main"); //测试lt9211d临时屏蔽

    return 0;
}

static int lt9211c_remove(struct i2c_client *client)
{
    //printk("==tjf==lt9211c_remove enter.");
    cdev_del(&lt9211c.cdev);
    unregister_chrdev_region(lt9211c.dev_id, 1);
    device_destroy(lt9211c.class, lt9211c.dev_id);
    class_destroy(lt9211c.class);
    kthread_stop(lt9211c.task);

    return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lt9211c_of_match[] = {
    { .compatible = "lontium,lt9211c" },
    {},
};
MODULE_DEVICE_TABLE(of, lt9211c_of_match);
#endif

static const struct i2c_device_id lt9211c_match_id[] = {
    { "lontium,lt9211c", 0 },
    { },
};

static struct i2c_driver lt9211c_i2c_driver = {
    .driver = {
        .name = "lt9211c",
        .of_match_table = of_match_ptr(lt9211c_of_match),
    },
    .probe      = &lt9211c_probe,
    .remove     = &lt9211c_remove,
    .id_table   = lt9211c_match_id,
};

static int __init converter_mod_init(void)
{
    return i2c_add_driver(&lt9211c_i2c_driver);
}

static void __exit converter_mod_exit(void)
{
    i2c_del_driver(&lt9211c_i2c_driver);
}

device_initcall_sync(converter_mod_init);
module_exit(converter_mod_exit);

MODULE_DESCRIPTION("Lontium LT9211C converter driver");
MODULE_LICENSE("GPL v2");
