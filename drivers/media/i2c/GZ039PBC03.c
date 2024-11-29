#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <video/of_display_timing.h>
#include <video/display_timing.h>

struct gz039pbc03 {
    struct i2c_client *client;
    struct i2c_adapter *adapter;
    struct device *dev;
};

static struct {
    struct gpio_desc *reset_gpio;
    struct gpio_desc *power_gpio;
    atomic_t refcount;
} shared_gpios = { .refcount = ATOMIC_INIT(0) };    //ATOMIC_INIT(0) 是一个宏，它在Linux内核中用来初始化原子类型变量 atomic_t 的一个值（在这里初始化为0）
/*
只初始化了 refcount 这个成员，将其设置为0。。reset_gpio 和 power_gpio 这两个指针成员在这个初始化中没有明确设置，会被自动初始化为NULL
*/

static int gz039pbc03_Write_Reg(struct i2c_client *client, u16 cmd, u8 val)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, cmd, val);
    if (ret < 0) {
        dev_err(&client->dev, "failed writing register 0x%04x\n", cmd);
        return ret;
    }

    return 0;
}

static int gz039pbc03_Write_Reg_Retry(struct i2c_client *client, u16 cmd, u8 val) {
    int ret, retries = 3;

    do {
        ret = gz039pbc03_Write_Reg(client, cmd, val);
        if (ret < 0) {
            dev_err(&client->dev, "Retrying writing register 0x%04x\n", cmd);
            msleep(5); // 重试
        } else {
            return 0; // 写入成功，退出函数
        }
    } while (--retries > 0);

    return ret; // 返回最后的错误
}

static void gz039pbc03_system_init(struct i2c_client *client)
{
    gz039pbc03_Write_Reg_Retry(client, 0x94, 0xda);
    gz039pbc03_Write_Reg_Retry(client, 0x01, 0x03);
}

/*
static void gz039pbc03_reset(struct i2c_client *client)
{
        gpiod_set_value_cansleep(shared_gpios.reset_gpio, 0);
        msleep(100); // 持续一段时间
        
        // 设置复位GPIO为高，结束复位状态
        gpiod_set_value_cansleep(shared_gpios.reset_gpio, 1);
        msleep(100); // 让设备有时间从复位中恢复
}
*/

static void gz039pbc03_power_on(struct device *dev)
{
	if (shared_gpios.power_gpio) 
	{
		gpiod_set_value(shared_gpios.power_gpio, 1);
		msleep(20);
	}
}

static void gz039pbc03_power_off(struct device *dev)
{
	if (shared_gpios.power_gpio)
    	{
        	gpiod_set_value(shared_gpios.power_gpio, 0);
        	msleep(10);
    	}
}

static ssize_t reinit_store(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    
    // 重新执行初始化序列
    gz039pbc03_system_init(client);
    
    return count;
}

static DEVICE_ATTR_WO(reinit);

static int gz039pbc03_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct gz039pbc03 *priv;

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        dev_err(dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }

    priv->client = client;
    i2c_set_clientdata(client, priv);

    // 增加引用计数并检查是否是第一次引用
    if (atomic_inc_return(&shared_gpios.refcount) == 1) {
        // 请求GPIO并进行初始化
        shared_gpios.power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_HIGH);
        shared_gpios.reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
        
        if (IS_ERR(shared_gpios.power_gpio) || IS_ERR(shared_gpios.reset_gpio)) {
            // 如果请求失败，回滚引用计数并处理错误
            atomic_dec(&shared_gpios.refcount);
            dev_err(dev, "Failed to get power or reset gpio\n");
            return PTR_ERR(shared_gpios.power_gpio);
        }

	gpiod_set_value_cansleep(shared_gpios.reset_gpio, 0);
        msleep(10);
	gpiod_set_value(shared_gpios.power_gpio, 0);
	msleep(10);
        gz039pbc03_power_on(dev); 
        msleep(10);
        // 资源获取成功后，执行复位
        //gz039pbc03_reset(client);
        gpiod_set_value_cansleep(shared_gpios.reset_gpio, 1);
        msleep(10);
    }

    // 此处执行其他必要的设备初始化
    gz039pbc03_system_init(client);

    dev_info(dev, "gz039pbc03 device at address 0x%02x has been initialized\n", client->addr);

    device_create_file(&client->dev, &dev_attr_reinit);

    return 0;
}


static int gz039pbc03_remove(struct i2c_client *client)
{
    if (atomic_dec_and_test(&shared_gpios.refcount)) {
        // 最后一个引用时释放 GPIO
        // devm_gpiod_put() 不是必需的，因为 devm_ 函数会处理释放

        gz039pbc03_power_off(&client->dev);
    }
    return 0;
}

static const struct of_device_id gz039pbc03_of_match[] = {
    { .compatible = "gz,gz039pbc03" },
    {},
};
MODULE_DEVICE_TABLE(of, gz039pbc03_of_match);

static const struct i2c_device_id gz039pbc03_id[] = {
    { "gz039pbc03", 0 },
    {},
};
MODULE_DEVICE_TABLE(i2c, gz039pbc03_id);

static struct i2c_driver gz039pbc03_driver = {
    .driver = {
        .name = "gz039pbc03",
        .of_match_table = gz039pbc03_of_match,
    },
    .id_table = gz039pbc03_id,
    .probe = gz039pbc03_probe,
    .remove = gz039pbc03_remove,
};

static int __init gz039pbc03_init(void)
{
    // 其他初始化代码
    i2c_add_driver(&gz039pbc03_driver);
    // 其他初始化代码
    return 0;
}
module_init(gz039pbc03_init);

static void __exit gz039pbc03_exit(void)
{
    // 其他初始化代码
    i2c_del_driver(&gz039pbc03_driver);
    // 其他初始化代码
}
module_exit(gz039pbc03_exit);

MODULE_DESCRIPTION("gz039pbc03 driver");
MODULE_AUTHOR("cdjp");
MODULE_LICENSE("GPL v2");
