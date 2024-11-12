// // SPDX-License-Identifier: GPL-2.0
// /*
//  * gc2155 driver
//  *
//  * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd.
//  *
//  * V0.0X01.0X01 add poweron function.
//  * V0.0X01.0X02 fix mclk issue when probe multiple camera.
//  * V0.0X01.0X03 add enum_frame_interval function.
//  * V0.0X01.0X04 add exposure control and fix v4l2_ctrl init issues.
//  */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>
#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <linux/rk-camera-module.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x04)
#define GC2155_PIXEL_RATE		(120 * 1000 * 1000)
#define GC2155_EXPOSURE_CONTROL

#define REG_CHIP_ID_H			0xf0
#define REG_CHIP_ID_L			0xf1
#define CHIP_ID_H			0x21
#define CHIP_ID_L			0x55

#define REG_NULL			0xFF

#define GC2155_XVCLK_FREQ		24000000

#define GC2155_NAME			"gc2155"

#define GC2155_FORMAT MEDIA_BUS_FMT_Y8_1X8
#define GC2155_WIDTH 1920
#define GC2155_HEIGHT 1080
#define GC2155_FRAME_RATE 60

static const char * const gc2155_supply_names[] = {
	"avdd",
	"dovdd",
	"dvdd",
};

#define GC2155_NUM_SUPPLIES ARRAY_SIZE(gc2155_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct gc2155_mode {
	u32 width;
	u32 height;
	unsigned int fps;
	struct v4l2_fract max_fps;
	const struct regval *reg_list;
};

struct gc2155 {
	struct i2c_client *client;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *power_gpio;
	struct regulator_bulk_data supplies[GC2155_NUM_SUPPLIES];

	bool streaming;
	bool power_on;
	struct mutex mutex; /* lock to serialize v4l2 callback */
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	unsigned int fps;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;

	const struct gc2155_mode *cur_mode;
	const struct gc2155_mode *framesize_cfg;
	unsigned int cfg_num;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

#define to_gc2155(sd) container_of(sd, struct gc2155, subdev)

static struct regval gc2155_global_regs[] = {
	{REG_NULL, 0x0},
};

static struct regval gc2155_800x600_15fps[] = {
	{REG_NULL, 0x0},
};

static struct regval gc2155_1600x1200_7fps[] = {
	{REG_NULL, 0x0},
};

static const struct gc2155_mode supported_modes[] = {
	{
		.width		= GC2155_WIDTH,
		.height		= GC2155_HEIGHT,
		.fps		= GC2155_FRAME_RATE,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.reg_list	= gc2155_800x600_15fps,
	},
	{
		.width		= GC2155_WIDTH,
		.height		= GC2155_HEIGHT,
		.fps		= GC2155_FRAME_RATE,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.reg_list	= gc2155_1600x1200_7fps,
	},
};

static int gc2155_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	return 0;
}

static int gc2155_write_array(struct i2c_client *client,
	const struct regval *regs)
{
	return 0;
}

static inline u8 gc2155_read_reg(struct i2c_client *client, u8 reg)
{
	return 0;
}

static int gc2155_get_reso_dist(const struct gc2155_mode *mode,
	struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		abs(mode->height - framefmt->height);
}

static const struct gc2155_mode *
gc2155_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = gc2155_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

#ifdef GC2155_EXPOSURE_CONTROL
/*
 * the function is called before sensor register setting in VIDIOC_S_FMT
 */
/* Row times = Hb + Sh_delay + win_width + 4*/

static int gc2155_aec_ctrl(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	return 0;
}
#endif

static int gc2155_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct gc2155 *gc2155 = to_gc2155(sd);
	const struct gc2155_mode *mode;

	mutex_lock(&gc2155->mutex);

	mode = gc2155_find_best_fit(fmt);
	fmt->format.code = GC2155_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc2155->mutex);
		return -ENOTTY;
#endif
	} else {
		gc2155->cur_mode = mode;
		gc2155->format = fmt->format;
	}

#ifdef GC2155_EXPOSURE_CONTROL
		if (gc2155->power_on)
			gc2155_aec_ctrl(sd, &fmt->format);
#endif
	mutex_unlock(&gc2155->mutex);

	return 0;
}

static int gc2155_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct gc2155 *gc2155 = to_gc2155(sd);
	const struct gc2155_mode *mode = gc2155->cur_mode;

	mutex_lock(&gc2155->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc2155->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC2155_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
		fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	}
	mutex_unlock(&gc2155->mutex);

	return 0;
}

static void gc2155_get_default_format(struct gc2155 *gc2155,
				      struct v4l2_mbus_framefmt *format)
{
	format->width = gc2155->cur_mode->width;
	format->height = gc2155->cur_mode->height;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->code = GC2155_FORMAT;
	format->field = V4L2_FIELD_NONE;
}

static int gc2155_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	code->code = GC2155_FORMAT;

	return 0;
}

static int gc2155_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	u32 index = fse->index;

	if (index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->code = GC2155_FORMAT;

	fse->min_width  = supported_modes[index].width;
	fse->max_width  = supported_modes[index].width;
	fse->max_height = supported_modes[index].height;
	fse->min_height = supported_modes[index].height;

	return 0;
}

static int __gc2155_power_on(struct gc2155 *gc2155)
{
	int ret;
	struct device *dev = &gc2155->client->dev;

	ret = clk_set_rate(gc2155->xvclk, GC2155_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc2155->xvclk) != GC2155_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc2155->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	gc2155->power_on = true;
	return 0;
}

static void __gc2155_power_off(struct gc2155 *gc2155)
{
	if (!IS_ERR(gc2155->xvclk))
		clk_disable_unprepare(gc2155->xvclk);
	gc2155->power_on = false;
}

static void gc2155_get_module_inf(struct gc2155 *gc2155,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, GC2155_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, gc2155->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, gc2155->len_name, sizeof(inf->base.lens));
}

static long gc2155_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc2155 *gc2155 = to_gc2155(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		gc2155_get_module_inf(gc2155, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gc2155_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc2155_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = gc2155_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int gc2155_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc2155 *gc2155 = to_gc2155(sd);
	struct i2c_client *client = gc2155->client;
	int ret = 0;
	u8 val;
	unsigned int fps;
	int delay_us;

	fps = DIV_ROUND_CLOSEST(gc2155->cur_mode->max_fps.denominator,
			  gc2155->cur_mode->max_fps.numerator);

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				gc2155->cur_mode->width,
				gc2155->cur_mode->height,
				fps);

	mutex_lock(&gc2155->mutex);

	on = !!on;
	if (on == gc2155->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&gc2155->client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = gc2155_write_array(gc2155->client,
					  gc2155->cur_mode->reg_list);
		if (ret) {
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}

	} else {
		pm_runtime_put(&client->dev);
	}
	val = on ? 0x0f : 0;
	ret = gc2155_write_reg(client, 0xf2, val);
	gc2155->streaming = on;

	/* delay to enable oneframe complete */
	if (!on) {
		delay_us = 1000 * 1000 / fps;
		usleep_range(delay_us, delay_us + 10);
		dev_info(&client->dev, "%s: on: %d, sleep(%dus)\n",
				__func__, on, delay_us);
	}

unlock_and_return:
	mutex_unlock(&gc2155->mutex);

	return ret;
}

static int gc2155_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc2155 *gc2155 = to_gc2155(sd);
	struct i2c_client *client = gc2155->client;
	int ret = 0;

	mutex_lock(&gc2155->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc2155->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = gc2155_write_array(gc2155->client, gc2155_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc2155->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc2155->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc2155->mutex);

	return ret;
}

static int gc2155_set_test_pattern(struct gc2155 *gc2155, int value)
{
	return 0;
}

static int gc2155_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2155 *gc2155 =
			container_of(ctrl->handler, struct gc2155, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		return gc2155_set_test_pattern(gc2155, ctrl->val);
	}

	return 0;
}

static const struct v4l2_ctrl_ops gc2155_ctrl_ops = {
	.s_ctrl = gc2155_s_ctrl,
};

static const char * const gc2155_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc2155_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc2155 *gc2155 = to_gc2155(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc2155_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc2155->mutex);

	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC2155_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;
	try_fmt->colorspace = V4L2_COLORSPACE_SRGB;

	mutex_unlock(&gc2155->mutex);

	return 0;
}
#endif

static int gc2155_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2155 *gc2155 = to_gc2155(sd);

	return __gc2155_power_on(gc2155);
}

static int gc2155_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2155 *gc2155 = to_gc2155(sd);

	__gc2155_power_off(gc2155);

	return 0;
}

static int gc2155_g_mbus_config(struct v4l2_subdev *sd,
                               unsigned int pad,
                               struct v4l2_mbus_config *config)
{
        config->type = V4L2_MBUS_PARALLEL;
        config->flags = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
                       V4L2_MBUS_VSYNC_ACTIVE_HIGH |
                       V4L2_MBUS_PCLK_SAMPLE_FALLING;

        return 0;
}

static int gc2155_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc2155 *gc2155 = to_gc2155(sd);

	mutex_lock(&gc2155->mutex);
	fi->interval = gc2155->cur_mode->max_fps;
	mutex_unlock(&gc2155->mutex);

	return 0;
}

static void __gc2155_try_frame_size_fps(struct gc2155 *gc2155,
					struct v4l2_mbus_framefmt *mf,
					const struct gc2155_mode **size,
					unsigned int fps)
{
	const struct gc2155_mode *fsize = &gc2155->framesize_cfg[0];
	const struct gc2155_mode *match = NULL;
	unsigned int i = gc2155->cfg_num;
	unsigned int min_err = UINT_MAX;

	while (i--) {
		unsigned int err = abs(fsize->width - mf->width)
				+ abs(fsize->height - mf->height);
		if (err < min_err && fsize->reg_list[0].addr) {
			min_err = err;
			match = fsize;
		}
		fsize++;
	}

	if (!match) {
		match = &gc2155->framesize_cfg[0];
	} else {
		fsize = &gc2155->framesize_cfg[0];
		for (i = 0; i < gc2155->cfg_num; i++) {
			if (fsize->width == match->width &&
			    fsize->height == match->height &&
			    fps >= fsize->fps)
				match = fsize;

			fsize++;
		}
	}

	mf->width  = match->width;
	mf->height = match->height;

	if (size)
		*size = match;
}

static int gc2155_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2155 *gc2155 = to_gc2155(sd);
	const struct gc2155_mode *mode = NULL;
	struct v4l2_mbus_framefmt mf;
	unsigned int fps;
	int ret = 0;

	dev_dbg(&client->dev, "Setting %d/%d frame interval\n",
		 fi->interval.numerator, fi->interval.denominator);

	mutex_lock(&gc2155->mutex);
	if (gc2155->cur_mode->width == 1600)
		goto unlock;
	fps = DIV_ROUND_CLOSEST(fi->interval.denominator,
				fi->interval.numerator);
	mf = gc2155->format;
	__gc2155_try_frame_size_fps(gc2155, &mf, &mode, fps);
	if (gc2155->cur_mode != mode) {
		ret = gc2155_write_array(client, mode->reg_list);
		if (ret)
			goto unlock;
		gc2155->cur_mode = mode;
		gc2155->fps = fps;
	}
unlock:
	mutex_unlock(&gc2155->mutex);

	return ret;
}

static int gc2155_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != GC2155_FORMAT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops gc2155_pm_ops = {
	SET_RUNTIME_PM_OPS(gc2155_runtime_suspend,
			   gc2155_runtime_resume, NULL)
};

static const struct v4l2_subdev_core_ops gc2155_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = gc2155_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc2155_compat_ioctl32,
#endif
	.s_power = gc2155_s_power,
};

static const struct v4l2_subdev_video_ops gc2155_video_ops = {
	.s_stream = gc2155_s_stream,
	.g_frame_interval = gc2155_g_frame_interval,
	.s_frame_interval = gc2155_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc2155_pad_ops = {
	.enum_mbus_code = gc2155_enum_mbus_code,
	.enum_frame_size = gc2155_enum_frame_sizes,
	.enum_frame_interval = gc2155_enum_frame_interval,
	.get_fmt = gc2155_get_fmt,
	.set_fmt = gc2155_set_fmt,
	.get_mbus_config = gc2155_g_mbus_config,
};

static const struct v4l2_subdev_ops gc2155_subdev_ops = {
	.core	= &gc2155_core_ops,
	.video	= &gc2155_video_ops,
	.pad	= &gc2155_pad_ops,
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc2155_internal_ops = {
	.open = gc2155_open,
};
#endif

static int gc2155_check_sensor_id(struct gc2155 *gc2155,
				  struct i2c_client *client)
{
	struct device *dev = &gc2155->client->dev;
	u8 id_h, id_l;

	// id_h = gc2155_read_reg(client, REG_CHIP_ID_H);
	// id_l = gc2155_read_reg(client, REG_CHIP_ID_L);
	id_h = CHIP_ID_H;
	id_l = CHIP_ID_L;
	if (id_h != CHIP_ID_H && id_l != CHIP_ID_L) {
		dev_err(dev, "Wrong camera sensor id(0x%02x%02x)\n",
			id_h, id_l);
		return -EINVAL;
	}

	dev_info(dev, "Detected GC2155 (0x%02x%02x) sensor\n",
		CHIP_ID_H, CHIP_ID_L);

	return 0;
}

static int gc2155_configure_regulators(struct gc2155 *gc2155)
{
	u32 i;

	for (i = 0; i < GC2155_NUM_SUPPLIES; i++)
		gc2155->supplies[i].supply = gc2155_supply_names[i];

	return devm_regulator_bulk_get(&gc2155->client->dev,
				       GC2155_NUM_SUPPLIES,
				       gc2155->supplies);
}

static int gc2155_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc2155 *gc2155;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc2155 = devm_kzalloc(dev, sizeof(*gc2155), GFP_KERNEL);
	if (!gc2155)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc2155->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &gc2155->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &gc2155->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &gc2155->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	gc2155->client = client;
	gc2155->cur_mode = &supported_modes[0];
	gc2155_get_default_format(gc2155, &gc2155->format);
	gc2155->format.width = gc2155->cur_mode->width;
	gc2155->format.height = gc2155->cur_mode->height;
	gc2155->fps =  DIV_ROUND_CLOSEST(gc2155->cur_mode->max_fps.denominator,
			gc2155->cur_mode->max_fps.numerator);
	gc2155->framesize_cfg = supported_modes;
	gc2155->cfg_num = ARRAY_SIZE(supported_modes);

	gc2155->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gc2155->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	// xjt GPIOD_OUT_LOW -> GPIOD_OUT_HIGH
	gc2155->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(gc2155->power_gpio))
		dev_info(dev, "Failed to get power-gpios, maybe no use\n");

	gc2155->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc2155->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc2155->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc2155->pwdn_gpio))
		dev_warn(dev, "Failed to get gc2155-gpios\n");

	ret = gc2155_configure_regulators(gc2155);
	if (ret) {
		dev_warn(dev, "Failed to get power regulators\n");
		return ret;
	}
	v4l2_ctrl_handler_init(&gc2155->ctrls, 2);
	gc2155->pixel_rate =
			v4l2_ctrl_new_std(&gc2155->ctrls, &gc2155_ctrl_ops,
					  V4L2_CID_PIXEL_RATE, 0,
					  GC2155_PIXEL_RATE, 1,
					  GC2155_PIXEL_RATE);

	v4l2_ctrl_new_std_menu_items(&gc2155->ctrls, &gc2155_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc2155_test_pattern_menu) - 1,
				     0, 0, gc2155_test_pattern_menu);
	gc2155->subdev.ctrl_handler = &gc2155->ctrls;

	if (gc2155->ctrls.error) {
		dev_err(&client->dev, "%s: control initialization error %d\n",
			__func__, gc2155->ctrls.error);
		return  gc2155->ctrls.error;
	}

	mutex_init(&gc2155->mutex);
	v4l2_i2c_subdev_init(&gc2155->subdev, client, &gc2155_subdev_ops);

	ret = __gc2155_power_on(gc2155);
	if (ret)
		goto err_destroy_mutex;

	ret = gc2155_check_sensor_id(gc2155, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	gc2155->subdev.internal_ops = &gc2155_internal_ops;
	gc2155->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc2155->pad.flags = MEDIA_PAD_FL_SOURCE;
	gc2155->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&gc2155->subdev.entity, 1, &gc2155->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	sd = &gc2155->subdev;
	memset(facing, 0, sizeof(facing));
	if (strcmp(gc2155->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc2155->module_index, facing,
		 GC2155_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&gc2155->subdev.entity);
#endif
err_power_off:
	__gc2155_power_off(gc2155);
err_destroy_mutex:
	mutex_destroy(&gc2155->mutex);
	v4l2_ctrl_handler_free(&gc2155->ctrls);
	return ret;
}

static int gc2155_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2155 *gc2155 = to_gc2155(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&gc2155->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc2155_power_off(gc2155);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc2155_of_match[] = {
	{ .compatible = "galaxycore,gc2155" },
	{},
};
MODULE_DEVICE_TABLE(of, gc2155_of_match);
#endif

static const struct i2c_device_id gc2155_match_id[] = {
	{"gc2155", 0},
	{},
};

static struct i2c_driver gc2155_i2c_driver = {
	.driver = {
		.name = GC2155_NAME,
		.pm = &gc2155_pm_ops,
		.of_match_table = of_match_ptr(gc2155_of_match),
	},
	.probe		= gc2155_probe,
	.remove		= gc2155_remove,
	.id_table	= gc2155_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc2155_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc2155_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("GalaxyCore gc2155 sensor driver");
MODULE_LICENSE("GPL v2");