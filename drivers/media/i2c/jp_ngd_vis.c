// SPDX-License-Identifier: GPL-2.0
/*
 * jp_ngd_vis.c - JPVISION NGD VIS camera driver
 *
 * Copyright (C) 2024
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

/* 模块基本信息定义 */
#define DRIVER_VERSION          KERNEL_VERSION(0, 0x01, 0x00)
#define JP_NGD_VIS_NAME        "jp_ngd_vis"

/* 摄像头参数定义 */
#define JP_NGD_VIS_LANES       4
#define JP_NGD_VIS_LINK_FREQ   345000000
#define JP_NGD_VIS_PIXEL_RATE  148500000
#define JP_NGD_VIS_FMT         MEDIA_BUS_FMT_UYVY8_2X8

/* 分辨率和帧率定义 */
#define JP_NGD_VIS_2688x1520_25_WIDTH       2688
#define JP_NGD_VIS_2688x1520_25_HEIGHT      1520
#define JP_NGD_VIS_2688x1520_25_FPS         25
#define JP_NGD_VIS_2688x1520_25_HPW         80
#define JP_NGD_VIS_2688x1520_25_HBP         260
#define JP_NGD_VIS_2688x1520_25_HFP         272
#define JP_NGD_VIS_2688x1520_25_H_BLANK     (JP_NGD_VIS_2688x1520_25_HPW + JP_NGD_VIS_2688x1520_25_HBP + JP_NGD_VIS_2688x1520_25_HFP)
#define JP_NGD_VIS_2688x1520_25_VPW         20
#define JP_NGD_VIS_2688x1520_25_VBP         180
#define JP_NGD_VIS_2688x1520_25_VFP         80
#define JP_NGD_VIS_2688x1520_25_V_BLANK     (JP_NGD_VIS_2688x1520_25_VPW + JP_NGD_VIS_2688x1520_25_VBP + JP_NGD_VIS_2688x1520_25_VFP)

struct jp_ngd_vis_mode {
    u32 width;
    u32 height;
    struct v4l2_fract max_fps;
    u32 hts_def;  // 水平总时间
    u32 vts_def;  // 垂直总时间
};

static const struct jp_ngd_vis_mode supported_modes[] = {
    {
        .width = JP_NGD_VIS_2688x1520_25_WIDTH,
        .height = JP_NGD_VIS_2688x1520_25_HEIGHT,
        .max_fps = {
            .numerator = 10000,
            .denominator = JP_NGD_VIS_2688x1520_25_FPS * 10000,
        },
        .hts_def = JP_NGD_VIS_2688x1520_25_H_BLANK,
        .vts_def = JP_NGD_VIS_2688x1520_25_V_BLANK,
    },
};

struct jp_ngd_vis {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
    
    struct mutex mutex;
    bool streaming;
    bool power_on;
    
    struct clk *xvclk;
    struct gpio_desc *reset_gpio;
    
    /* 模块信息 */
    u32 module_index;
    const char *module_facing;
    const char *module_name;
    const char *len_name;

    const struct jp_ngd_vis_mode *cur_mode;
};

/* 支持的格式和分辨率 */
static const struct v4l2_mbus_framefmt jp_ngd_vis_fmt = {
    .code = JP_NGD_VIS_FMT,
    .width = JP_NGD_VIS_2688x1520_25_WIDTH,
    .height = JP_NGD_VIS_2688x1520_25_HEIGHT,
    .field = V4L2_FIELD_NONE,
    .colorspace = V4L2_COLORSPACE_SRGB,
};

static const s64 link_freq_menu_items[] = {
    JP_NGD_VIS_LINK_FREQ,
};

static inline struct jp_ngd_vis *to_jp_ngd_vis(struct v4l2_subdev *sd)
{
    return container_of(sd, struct jp_ngd_vis, sd);
}

static int __jp_ngd_vis_power_on(struct jp_ngd_vis *jp_ngd_vis)
{
    int ret;
    struct device *dev = jp_ngd_vis->sd.dev;

    /* 1. 设置reset引脚 */
    if (!IS_ERR(jp_ngd_vis->reset_gpio)) {
        gpiod_set_value_cansleep(jp_ngd_vis->reset_gpio, 1);
        dev_info(dev, "Set reset to low\n");
        usleep_range(100 * 1000, 200 * 1000);
        gpiod_set_value_cansleep(jp_ngd_vis->reset_gpio, 0);
        dev_info(dev, "Set reset to high\n");
        usleep_range(100 * 1000, 200 * 1000);
    }

    /* 2. 使能MCLK时钟 */
    ret = clk_prepare_enable(jp_ngd_vis->xvclk);
    if (ret < 0) {
        dev_err(dev, "Failed to enable xvclk\n");
        return ret;
    }

    jp_ngd_vis->power_on = true;
    return 0;
}

static void __jp_ngd_vis_power_off(struct jp_ngd_vis *jp_ngd_vis)
{
    struct device *dev = jp_ngd_vis->sd.dev;
    
    if (!IS_ERR(jp_ngd_vis->reset_gpio))
    {
        gpiod_set_value_cansleep(jp_ngd_vis->reset_gpio, 0);
        dev_info(dev, "Set reset to high\n");
    }
    
    clk_disable_unprepare(jp_ngd_vis->xvclk);
    jp_ngd_vis->power_on = false;
}

static int jp_ngd_vis_s_power(struct v4l2_subdev *sd, int on)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);
    int ret = 0;

    mutex_lock(&jp_ngd_vis->mutex);

    if (jp_ngd_vis->power_on == !!on)
        goto unlock_and_return;

    if (on)
        ret = __jp_ngd_vis_power_on(jp_ngd_vis);
    else
        __jp_ngd_vis_power_off(jp_ngd_vis);

unlock_and_return:
    mutex_unlock(&jp_ngd_vis->mutex);
    return ret;
}

static int __jp_ngd_vis_start_stream(struct jp_ngd_vis *jp_ngd_vis)
{
	int ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&jp_ngd_vis->mutex);
	ret = v4l2_ctrl_handler_setup(&jp_ngd_vis->ctrl_handler);
	mutex_lock(&jp_ngd_vis->mutex);
	if (ret)
		return ret;

	return 0;
}

static int __jp_ngd_vis_stop_stream(struct jp_ngd_vis *jp_ngd_vis)
{
    // do nothing
	return 0;
}


/* 视频流控制 */
static int jp_ngd_vis_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);
    struct device *dev = jp_ngd_vis->sd.dev;
    int ret = 0;

    mutex_lock(&jp_ngd_vis->mutex);

    if (jp_ngd_vis->streaming == enable) {
        mutex_unlock(&jp_ngd_vis->mutex);
        return 0;
    }

    if (enable) {
        dev_info(dev, "jp_ngd_vis_s_stream on\n");
        ret = __jp_ngd_vis_start_stream(jp_ngd_vis);
        if (ret) {
			dev_info(dev, "jp_ngd_vis_s_stream fail\n");
			goto err_unlock;
		}
    } else {
        __jp_ngd_vis_stop_stream(jp_ngd_vis);
        dev_info(dev, "jp_ngd_vis_s_stream off\n");
    } 

    jp_ngd_vis->streaming = enable;

err_unlock:
    mutex_unlock(&jp_ngd_vis->mutex);
    return ret;
}

/* MIPI-CSI2配置获取 */
static int jp_ngd_vis_g_mbus_config(struct v4l2_subdev *sd,
                unsigned int pad,
                struct v4l2_mbus_config *config)
{
    config->type = V4L2_MBUS_CSI2_DPHY;
    config->flags = V4L2_MBUS_CSI2_4_LANE |
                   V4L2_MBUS_CSI2_CHANNEL_0 |
                   V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

    return 0;
}

#ifdef CONFIG_PM
static int jp_ngd_vis_runtime_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);

    return __jp_ngd_vis_power_on(jp_ngd_vis);
}

static int jp_ngd_vis_runtime_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);

    __jp_ngd_vis_power_off(jp_ngd_vis);

    return 0;
}
#endif

static const struct dev_pm_ops jp_ngd_vis_pm_ops = {
    SET_RUNTIME_PM_OPS(jp_ngd_vis_runtime_suspend,
                       jp_ngd_vis_runtime_resume, NULL)
};

/* 格式设置与获取 */
static int jp_ngd_vis_set_fmt(struct v4l2_subdev *sd,
                struct v4l2_subdev_pad_config *cfg,
                struct v4l2_subdev_format *fmt)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);
    struct v4l2_mbus_framefmt *framefmt;

    mutex_lock(&jp_ngd_vis->mutex);

    fmt->format = jp_ngd_vis_fmt;
    
    if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
        framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
        *framefmt = fmt->format;
#else
        mutex_unlock(&jp_ngd_vis->mutex);
        return -ENOTTY;
#endif
    }

    mutex_unlock(&jp_ngd_vis->mutex);
    return 0;
}

static int jp_ngd_vis_get_fmt(struct v4l2_subdev *sd,
                struct v4l2_subdev_pad_config *cfg,
                struct v4l2_subdev_format *fmt)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);

    mutex_lock(&jp_ngd_vis->mutex);
    
    if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
        fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
        mutex_unlock(&jp_ngd_vis->mutex);
        return -ENOTTY;
#endif
    } else {
        fmt->format = jp_ngd_vis_fmt;
    }

    mutex_unlock(&jp_ngd_vis->mutex);
    return 0;
}

/* 帧间隔获取 */
static int jp_ngd_vis_g_frame_interval(struct v4l2_subdev *sd,
                    struct v4l2_subdev_frame_interval *fi)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);
    const struct jp_ngd_vis_mode *mode = jp_ngd_vis->cur_mode;

    fi->interval = mode->max_fps;
    return 0;
}

/* 获取模组信息 */
static void jp_ngd_vis_get_module_inf(struct jp_ngd_vis *jp_ngd_vis,
                struct rkmodule_inf *inf)
{
    if (!jp_ngd_vis || !inf)
        return;

    memset(inf, 0, sizeof(*inf));
    strlcpy(inf->base.sensor, JP_NGD_VIS_NAME, sizeof(inf->base.sensor));
    
    if (jp_ngd_vis->module_name)
        strlcpy(inf->base.module, jp_ngd_vis->module_name,
                sizeof(inf->base.module));
    
    if (jp_ngd_vis->len_name)
        strlcpy(inf->base.lens, jp_ngd_vis->len_name, sizeof(inf->base.lens));
}

/* IOCTL */
static long jp_ngd_vis_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);
    long ret = 0;
    u32 stream = 0;

    if (!jp_ngd_vis || !arg)
        return -EINVAL;

    switch (cmd) {
    case RKMODULE_GET_MODULE_INFO:
        jp_ngd_vis_get_module_inf(jp_ngd_vis, (struct rkmodule_inf *)arg);
        break;
    case RKMODULE_SET_QUICK_STREAM:
        stream = *((u32 *)arg);
        if (stream)
            ret = __jp_ngd_vis_start_stream(jp_ngd_vis);
        else
            ret = __jp_ngd_vis_stop_stream(jp_ngd_vis);
        break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }

    return ret;
}

/* 兼容32位系统的ioctl函数 */
#ifdef CONFIG_COMPAT
static long jp_ngd_vis_compat_ioctl32(struct v4l2_subdev *sd,
                unsigned int cmd, unsigned long arg)
{
    void __user *up = compat_ptr(arg);
    struct rkmodule_inf *inf;
    long ret = 0;
    u32 stream = 0;

    switch (cmd) {
    case RKMODULE_GET_MODULE_INFO:
        inf = kzalloc(sizeof(*inf), GFP_KERNEL);
        if (!inf) {
            ret = -ENOMEM;
            return ret;
        }
        ret = jp_ngd_vis_ioctl(sd, cmd, inf);
        if (!ret) 
            ret = copy_to_user(up, inf, sizeof(*inf));
        kfree(inf);
        break;
    case RKMODULE_SET_QUICK_STREAM:
        ret = copy_from_user(&stream, up, sizeof(u32));
        if (!ret)
            ret = jp_ngd_vis_ioctl(sd, cmd, &stream);
        break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }

    return ret;
}
#endif

/* 枚举支持的总线格式 */
static int jp_ngd_vis_enum_mbus_code(struct v4l2_subdev *sd,
                    struct v4l2_subdev_pad_config *cfg,
                    struct v4l2_subdev_mbus_code_enum *code)
{
    if (code->index > 0)
        return -EINVAL;

    code->code = jp_ngd_vis_fmt.code;
    return 0;
}

/* 枚举支持的分辨率 */
static int jp_ngd_vis_enum_frame_sizes(struct v4l2_subdev *sd,
                    struct v4l2_subdev_pad_config *cfg,
                    struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->index >= ARRAY_SIZE(supported_modes))
        return -EINVAL;

    if (fse->code != JP_NGD_VIS_FMT)
        return -EINVAL;

    fse->min_width = supported_modes[fse->index].width;
    fse->max_width = supported_modes[fse->index].width;
    fse->max_height = supported_modes[fse->index].height;
    fse->min_height = supported_modes[fse->index].height;

    return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int jp_ngd_vis_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);
    struct v4l2_mbus_framefmt *try_fmt =
                v4l2_subdev_get_try_format(sd, fh->pad, 0);
    const struct jp_ngd_vis_mode *def_mode = &supported_modes[0];

    mutex_lock(&jp_ngd_vis->mutex);
    /* Initialize try_fmt */
    try_fmt->width = def_mode->width;
    try_fmt->height = def_mode->height;
    try_fmt->code = JP_NGD_VIS_FMT;  // MEDIA_BUS_FMT_YUYV8_2X8
    try_fmt->field = V4L2_FIELD_NONE;
    try_fmt->colorspace = V4L2_COLORSPACE_SRGB;

    mutex_unlock(&jp_ngd_vis->mutex);
    /* No crop or compose */

    return 0;
}
#endif

static int jp_ngd_vis_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = JP_NGD_VIS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops jp_ngd_vis_internal_ops = {
    .open = jp_ngd_vis_open,
};
#endif

/* V4L2子设备操作集 */
static const struct v4l2_subdev_core_ops jp_ngd_vis_core_ops = {
    .s_power = jp_ngd_vis_s_power,
    .ioctl = jp_ngd_vis_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl32 = jp_ngd_vis_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops jp_ngd_vis_video_ops = {
    .s_stream = jp_ngd_vis_s_stream,
    .g_frame_interval = jp_ngd_vis_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops jp_ngd_vis_pad_ops = {
    .enum_mbus_code = jp_ngd_vis_enum_mbus_code,
    .enum_frame_size = jp_ngd_vis_enum_frame_sizes,
    .enum_frame_interval = jp_ngd_vis_enum_frame_interval,
    .get_fmt = jp_ngd_vis_get_fmt,
    .set_fmt = jp_ngd_vis_set_fmt,
    .get_mbus_config = jp_ngd_vis_g_mbus_config,
};

static const struct v4l2_subdev_ops jp_ngd_vis_subdev_ops = {
    .core = &jp_ngd_vis_core_ops,
    .video = &jp_ngd_vis_video_ops,
    .pad = &jp_ngd_vis_pad_ops,
};

static int jp_ngd_vis_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct jp_ngd_vis *jp_ngd_vis = container_of(ctrl->handler,
                         struct jp_ngd_vis, ctrl_handler);
    s64 max;
    int ret = 0;

    /* Propagate change of current control to all related controls */
    switch (ctrl->id) {
    case V4L2_CID_VBLANK:
        /* Update max exposure while meeting expected vblanking */
        max = jp_ngd_vis->cur_mode->height + ctrl->val - 4;
        __v4l2_ctrl_modify_range(jp_ngd_vis->vblank,
                     ctrl->minimum, max,
                     ctrl->step,
                     ctrl->default_value);
        break;
    }

    switch (ctrl->id) {
    case V4L2_CID_VBLANK:
        ret = 0;  // 因为不需要实际写寄存器，所以直接返回成功
        break;
    case V4L2_CID_HBLANK:
        ret = 0;  // 因为不需要实际写寄存器，所以直接返回成功
        break;
    default:
        break;
    }

    return ret;
}

static const struct v4l2_ctrl_ops jp_ngd_vis_ctrl_ops = {
    .s_ctrl = jp_ngd_vis_set_ctrl,
};

/* 控制器初始化 */
static int jp_ngd_vis_initialize_controls(struct jp_ngd_vis *jp_ngd_vis)
{
    struct v4l2_ctrl_handler *handler;
    const struct jp_ngd_vis_mode *mode;
    s64 vblank_def;
    s64 hblank_def;
    int ret;

    handler = &jp_ngd_vis->ctrl_handler;
    mode = jp_ngd_vis->cur_mode;
    ret = v4l2_ctrl_handler_init(handler, 4);
    if (ret)
        return ret;
    
    handler->lock = &jp_ngd_vis->mutex;

    /* 设置link frequency */
    jp_ngd_vis->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
                    V4L2_CID_LINK_FREQ,
                    0, 0, link_freq_menu_items);
    if (jp_ngd_vis->link_freq)
        jp_ngd_vis->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    /* 设置pixel rate */
    jp_ngd_vis->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
                    V4L2_CID_PIXEL_RATE,
                    0, JP_NGD_VIS_PIXEL_RATE, 1, JP_NGD_VIS_PIXEL_RATE);
    if (jp_ngd_vis->pixel_rate)
        jp_ngd_vis->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    /* 设置blanking */
    vblank_def = mode->vts_def - mode->height;
    jp_ngd_vis->vblank = v4l2_ctrl_new_std(handler, &jp_ngd_vis_ctrl_ops,
                    V4L2_CID_VBLANK,
                    vblank_def, vblank_def, 1, vblank_def);

    hblank_def = mode->hts_def - mode->width;
    jp_ngd_vis->hblank = v4l2_ctrl_new_std(handler, &jp_ngd_vis_ctrl_ops,
                    V4L2_CID_HBLANK,
                    hblank_def, hblank_def, 1, hblank_def);

    if (handler->error) {
        ret = handler->error;
        dev_err(jp_ngd_vis->sd.dev,
            "Failed to init controls(%d)\n", ret);
        goto err_free_handler;
    }

    jp_ngd_vis->sd.ctrl_handler = handler;
    return 0;

err_free_handler:
    v4l2_ctrl_handler_free(handler);
    return ret;
}

/* probe函数 */
static int jp_ngd_vis_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct device_node *node = dev->of_node;
    struct jp_ngd_vis *jp_ngd_vis;
    struct v4l2_subdev *sd;
    char facing[2];
    int ret;

    dev_info(dev, "driver version: %02x.%02x.%02x",
        DRIVER_VERSION >> 16,
        (DRIVER_VERSION & 0xff00) >> 8,
        DRIVER_VERSION & 0x00ff);

    /* 分配设备结构体内存 */
    jp_ngd_vis = devm_kzalloc(dev, sizeof(*jp_ngd_vis), GFP_KERNEL);
    if (!jp_ngd_vis)
        return -ENOMEM;

    /* 获取模块信息 */
    ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
                   &jp_ngd_vis->module_index);
    if (ret) {
        dev_warn(dev, "could not get module index!\n");
        jp_ngd_vis->module_index = 0;
    }

    ret = of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
                   &jp_ngd_vis->module_facing);
    if (ret) {
        dev_warn(dev, "could not get module facing!\n");
        jp_ngd_vis->module_facing = "back";
    }

    ret = of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
                   &jp_ngd_vis->module_name);
    if (ret) {
        dev_warn(dev, "could not get module name!\n");
        jp_ngd_vis->module_name = "unknown";
    }

    ret = of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
                   &jp_ngd_vis->len_name);
    if (ret) {
        dev_warn(dev, "could not get lens name!\n");
        jp_ngd_vis->len_name = "unknown";
    }

    /* 初始化当前模式 */
    jp_ngd_vis->cur_mode = &supported_modes[0];

    /* 获取时钟资源 */
    jp_ngd_vis->xvclk = devm_clk_get(dev, "xvclk");
    if (IS_ERR(jp_ngd_vis->xvclk)) {
        dev_err(dev, "Failed to get xvclk\n");
        return -EINVAL;
    }

    /* 获取复位GPIO */
    jp_ngd_vis->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(jp_ngd_vis->reset_gpio))
        dev_warn(dev, "Failed to get reset-gpios\n");

    /* 初始化互斥锁 */
    mutex_init(&jp_ngd_vis->mutex);

    /* 初始化V4L2子设备 */
    sd = &jp_ngd_vis->sd;
    v4l2_i2c_subdev_init(sd, client, &jp_ngd_vis_subdev_ops);

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
    sd->internal_ops = &jp_ngd_vis_internal_ops;
#endif

    /* 初始化控制器 */
    ret = jp_ngd_vis_initialize_controls(jp_ngd_vis);
    if (ret)
        goto err_destroy_mutex;

    /* 上电 */
    ret = __jp_ngd_vis_power_on(jp_ngd_vis);
    if (ret)
        goto err_free_handler;

    /* 设置子设备flags */
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

    /* 初始化media pad */
    jp_ngd_vis->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&sd->entity, 1, &jp_ngd_vis->pad);
    if (ret < 0)
        goto err_power_off;

    /* 设置模块名称 */
    memset(facing, 0, sizeof(facing));
    if (strcmp(jp_ngd_vis->module_facing, "back") == 0)
        facing[0] = 'b';
    else
        facing[0] = 'f';

    snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
         jp_ngd_vis->module_index, facing,
         JP_NGD_VIS_NAME, dev_name(sd->dev));

    /* 注册V4L2子设备 */
    ret = v4l2_async_register_subdev_sensor_common(sd);
    if (ret) {
        dev_err(dev, "v4l2 async register subdev failed\n");
        goto err_clean_entity;
    }

    /* 设置设备私有数据 */
    i2c_set_clientdata(client, jp_ngd_vis);

    /* 启用运行时PM */
    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);

    dev_info(dev, "probe success!\n");
    return 0;

err_clean_entity:
    media_entity_cleanup(&sd->entity);
err_power_off:
    __jp_ngd_vis_power_off(jp_ngd_vis);
err_free_handler:
    v4l2_ctrl_handler_free(&jp_ngd_vis->ctrl_handler);
err_destroy_mutex:
    mutex_destroy(&jp_ngd_vis->mutex);

    return ret;
}

/* remove函数 */
static int jp_ngd_vis_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct jp_ngd_vis *jp_ngd_vis = to_jp_ngd_vis(sd);

    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(&jp_ngd_vis->ctrl_handler);
    mutex_destroy(&jp_ngd_vis->mutex);

    __jp_ngd_vis_power_off(jp_ngd_vis);

    return 0;
}

/* 设备树匹配表 */
static const struct of_device_id jp_ngd_vis_of_match[] = {
    { .compatible = "jpvision,jp_ngd_vis" },
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, jp_ngd_vis_of_match);

/* I2C驱动ID表 */
static const struct i2c_device_id jp_ngd_vis_id[] = {
    { "jp_ngd_vis", 0 },
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, jp_ngd_vis_id);

/* I2C驱动结构体 */
static struct i2c_driver jp_ngd_vis_i2c_driver = {
    .driver = {
        .name = JP_NGD_VIS_NAME,
        .pm = &jp_ngd_vis_pm_ops,
        .of_match_table = of_match_ptr(jp_ngd_vis_of_match),
    },
    .probe = jp_ngd_vis_probe,
    .remove = jp_ngd_vis_remove,
    .id_table = jp_ngd_vis_id,
};

module_i2c_driver(jp_ngd_vis_i2c_driver);

MODULE_AUTHOR("jintao.xiao@vitalchem.com");
MODULE_DESCRIPTION("JPVISION NGD VIS camera driver");
MODULE_LICENSE("GPL v2");