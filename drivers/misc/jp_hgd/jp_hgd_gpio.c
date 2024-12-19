#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

#define DEVICE_NAME "jp_hgd_gpio_ctl"
#define MAX_GPIO_NUM 32

struct gpio_data {
    const char *name;
    int gpio;
    bool active_low;
};

struct hgd_gpio_dev {
    dev_t devt;
    struct cdev cdev;        // 添加字符设备结构
    struct device *dev;
    struct class *gpio_class;
    int gpio_count;
    struct gpio_data *gpios;
};

static struct hgd_gpio_dev *g_hgd_dev;

// 文件操作函数
// 文件操作函数
static ssize_t hgd_gpio_read(struct file *file, char __user *buf,
                            size_t count, loff_t *ppos)
{
    struct gpio_data *gpio_dat = file->private_data;
    char tmp[3];
    int value, len;

    if (*ppos > 0)
        return 0;

    value = gpio_get_value(gpio_dat->gpio);

    len = snprintf(tmp, sizeof(tmp), "%d\n", value);
    if (copy_to_user(buf, tmp, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static ssize_t hgd_gpio_write(struct file *file, const char __user *buf,
                             size_t count, loff_t *ppos)
{
    struct gpio_data *gpio_dat = file->private_data;
    char tmp[32];
    int value;
    size_t len;

    len = min(count, sizeof(tmp) - 1);
    if (copy_from_user(tmp, buf, len))
        return -EFAULT;

    tmp[len] = '\0';
    if (kstrtoint(tmp, 0, &value))
        return -EINVAL;

    gpio_set_value(gpio_dat->gpio, value);
    return count;
}

static int hgd_gpio_open(struct inode *inode, struct file *file)
{
    struct hgd_gpio_dev *hgd_dev = container_of(inode->i_cdev,
                                               struct hgd_gpio_dev, cdev);
    int index = MINOR(inode->i_rdev);

    if (index >= hgd_dev->gpio_count)
        return -EINVAL;

    file->private_data = &hgd_dev->gpios[index];
    return 0;
}

static const struct file_operations hgd_gpio_fops = {
    .owner = THIS_MODULE,
    .read = hgd_gpio_read,
    .write = hgd_gpio_write,
    .open = hgd_gpio_open,
};

static int hgd_gpio_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct device_node *child;
    struct hgd_gpio_dev *hgd_dev;
    int ret = 0, i = 0;

    hgd_dev = devm_kzalloc(&pdev->dev, sizeof(*hgd_dev), GFP_KERNEL);
    if (!hgd_dev)
        return -ENOMEM;

    hgd_dev->gpio_count = of_get_child_count(np);
    if (hgd_dev->gpio_count > MAX_GPIO_NUM)
        return -EINVAL;

    ret = alloc_chrdev_region(&hgd_dev->devt, 0, hgd_dev->gpio_count, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate device numbers\n");
        return ret;
    }

    // 初始化字符设备
    cdev_init(&hgd_dev->cdev, &hgd_gpio_fops);
    hgd_dev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&hgd_dev->cdev, hgd_dev->devt, hgd_dev->gpio_count);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add character device\n");
        goto err_unregister;
    }

    hgd_dev->gpios = devm_kzalloc(&pdev->dev,
                                 sizeof(struct gpio_data) * hgd_dev->gpio_count,
                                 GFP_KERNEL);
    if (!hgd_dev->gpios) {
        ret = -ENOMEM;
        goto err_cdev;
    }

    hgd_dev->gpio_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(hgd_dev->gpio_class)) {
        ret = PTR_ERR(hgd_dev->gpio_class);
        goto err_cdev;
    }

    for_each_child_of_node(np, child) {
        struct device *gpio_dev;
        enum of_gpio_flags flags;
        const char *name;
        
        hgd_dev->gpios[i].gpio = of_get_gpio_flags(child, 0, &flags);
        if (!gpio_is_valid(hgd_dev->gpios[i].gpio)) {
            ret = -EINVAL;
            goto err_class;
        }

        ret = of_property_read_string(child, "label", &name);
        if (ret) {
            dev_err(&pdev->dev, "Missing label property\n");
            goto err_class;
        }
        hgd_dev->gpios[i].name = name;
        hgd_dev->gpios[i].active_low = flags & OF_GPIO_ACTIVE_LOW;

        ret = devm_gpio_request_one(&pdev->dev, hgd_dev->gpios[i].gpio,
                                  GPIOF_OUT_INIT_LOW, hgd_dev->gpios[i].name);
        if (ret)
            goto err_class;

        gpio_dev = device_create(hgd_dev->gpio_class, NULL,
                               MKDEV(MAJOR(hgd_dev->devt), i),
                               NULL, "%s_%s", DEVICE_NAME,
                               hgd_dev->gpios[i].name);
        if (IS_ERR(gpio_dev)) {
            ret = PTR_ERR(gpio_dev);
            goto err_class;
        }

        i++;
    }

    g_hgd_dev = hgd_dev;
    platform_set_drvdata(pdev, hgd_dev);

    return 0;

err_class:
    class_destroy(hgd_dev->gpio_class);
err_cdev:
    cdev_del(&hgd_dev->cdev);
err_unregister:
    unregister_chrdev_region(hgd_dev->devt, hgd_dev->gpio_count);
    return ret;
}

static int hgd_gpio_remove(struct platform_device *pdev)
{
    struct hgd_gpio_dev *hgd_dev = platform_get_drvdata(pdev);
    int i;

    for (i = 0; i < hgd_dev->gpio_count; i++) {
        device_destroy(hgd_dev->gpio_class, MKDEV(MAJOR(hgd_dev->devt), i));
    }

    class_destroy(hgd_dev->gpio_class);
    cdev_del(&hgd_dev->cdev);
    unregister_chrdev_region(hgd_dev->devt, hgd_dev->gpio_count);
    return 0;
}

static const struct of_device_id hgd_gpio_match[] = {
    { .compatible = "jp,hgd-gpio-controller" },
    { }
};
MODULE_DEVICE_TABLE(of, hgd_gpio_match);

static struct platform_driver hgd_gpio_driver = {
    .probe = hgd_gpio_probe,
    .remove = hgd_gpio_remove,
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = hgd_gpio_match,
    },
};

module_platform_driver(hgd_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xiao jintao");
MODULE_DESCRIPTION("JP HGD GPIO Controller");