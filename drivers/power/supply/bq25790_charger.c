// SPDX-License-Identifier: GPL-2.0
// BQ25790 driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/module.h>

#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>

#include <linux/acpi.h>

#include "bq25790_charger.h"

#define BQ25790_NUM_WD_VAL	8



static struct reg_default bq25790_reg_defs[] = {
	{BQ25790_INPUT_V_LIM, 0x24},//3600mV
	{BQ25790_INPUT_I_LIM_MSB, 0x01},
	{BQ25790_INPUT_I_LIM_LSB, 0x2c},//3000mA
	{BQ25790_PRECHRG_CTRL, 0xc3},
	{BQ25790_TERM_CTRL, 0x5},
	{BQ25790_VOTG_REG, 0xdc},
	{BQ25790_IOTG_REG, 0x4b},
	{BQ25790_TIMER_CTRL, 0x3d},
	{BQ25790_CHRG_CTRL_0, 0xa2},
	{BQ25790_CHRG_CTRL_1, 0x85},
	{BQ25790_CHRG_CTRL_2, 0x40},
	{BQ25790_CHRG_CTRL_3, 0x12},
	{BQ25790_CHRG_CTRL_5, 0x16},
	{BQ25790_MPPT_CTRL, 0xaa},
	{BQ25790_TEMP_CTRL, 0xc0},
	{BQ25790_NTC_CTRL_0, 0x7a},
	{BQ25790_NTC_CTRL_1, 0x54},
	{BQ25790_ICO_I_LIM, 0x0},
	{BQ25790_CHRG_STAT_0, 0x0},
	{BQ25790_CHRG_STAT_1, 0x0},
	{BQ25790_CHRG_STAT_2, 0x0},
	{BQ25790_CHRG_STAT_3, 0x0},
	{BQ25790_CHRG_STAT_4, 0x0},
	{BQ25790_FAULT_STAT_0, 0x0},
	{BQ25790_FAULT_STAT_1, 0x0},
	{BQ25790_CHRG_FLAG_0, 0x0},
	{BQ25790_CHRG_FLAG_1, 0x0},
	{BQ25790_CHRG_FLAG_2, 0x0},
	{BQ25790_CHRG_FLAG_3, 0x0},
	{BQ25790_FAULT_FLAG_0, 0x0},
	{BQ25790_FAULT_FLAG_1, 0x0},
	{BQ25790_CHRG_MSK_0, 0x0},
	{BQ25790_CHRG_MSK_1, 0x0},
	{BQ25790_CHRG_MSK_2, 0x0},
	{BQ25790_CHRG_MSK_3, 0x0},
	{BQ25790_FAULT_MSK_0, 0x0},
	{BQ25790_FAULT_MSK_1, 0x0},
	{BQ25790_ADC_CTRL, 0x30},
	{BQ25790_FN_DISABE_0, 0x0},
	{BQ25790_FN_DISABE_1, 0x0},
	{BQ25790_ADC_IBUS_MSB, 0x0},
	{BQ25790_ADC_IBUS_LSB, 0x0},
	{BQ25790_ADC_IBAT_MSB, 0x0},
	{BQ25790_ADC_IBAT_LSB, 0x0},
	{BQ25790_ADC_VAC1, 0x0},
	{BQ25790_ADC_VAC2, 0x0},
	{BQ25790_ADC_VBAT_MSB, 0x0},
	{BQ25790_ADC_VBAT_LSB, 0x0},
	{BQ25790_ADC_VBUS_MSB, 0x0},
	{BQ25790_ADC_VBUS_LSB, 0x0},
	{BQ25790_ADC_TS, 0x0},
	{BQ25790_ADC_TDIE, 0x0},
	{BQ25790_ADC_DP, 0x0},
	{BQ25790_ADC_DM, 0x0},
	{BQ25790_DPDM_DRV, 0x0},
	{BQ25790_PART_INFO, 0x0},
};

static int bq25790_watchdog_time[BQ25790_NUM_WD_VAL] = {0, 500, 1000, 2000,
							20000, 40000, 80000,
							160000};

static enum power_supply_usb_type bq25790_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
};

//debug ======================
struct bq25790_reg {
	const char *name;
	uint8_t reg;
	int writeable;
} bq25790_regs[] = {
	{ "MIN_SYS_V", BQ25790_MIN_SYS_V, 1 },
	{ "V_LIM_MSB", BQ25790_CHRG_V_LIM_MSB, 1 },
	{ "V_LIM_LSB", BQ25790_CHRG_V_LIM_LSB, 1 },
	{ "I_LIM_MSB", BQ25790_CHRG_I_LIM_MSB, 1 },
	{ "I_LIM_LSB", BQ25790_CHRG_I_LIM_LSB, 1 },
	{ "V_LIM_INPUT", BQ25790_INPUT_V_LIM, 1 },
	{ "I_LIM_INPUT_MSB", BQ25790_INPUT_I_LIM_MSB, 1 },
	{ "I_LIM_INPUT_LSB", BQ25790_INPUT_I_LIM_LSB, 1 },
	{ "PRECHG_CTRL", BQ25790_PRECHRG_CTRL, 1 },
	{ "TERM_CTRL", BQ25790_TERM_CTRL, 1 },
	{ "RECHRG_CTRL", BQ25790_RECHRG_CTRL, 1 },
	{ "VOTG_REG", BQ25790_VOTG_REG, 1 },
	{ "IOTG_REG", BQ25790_IOTG_REG, 1 },
	{ "TIMER_CTRL", BQ25790_TIMER_CTRL, 1 },
	{ "CHRG_CTRL_0", BQ25790_CHRG_CTRL_0, 1 },
	{ "CHRG_CTRL_1", BQ25790_CHRG_CTRL_1, 1 },
	{ "CHRG_CTRL_2", BQ25790_CHRG_CTRL_2, 1 },
	{ "CHRG_CTRL_3", BQ25790_CHRG_CTRL_3, 1 },
	{ "CHRG_CTRL_4", BQ25790_CHRG_CTRL_4, 1 },
	{ "CHRG_CTRL_5", BQ25790_CHRG_CTRL_5, 1 },
	{ "MPPT_CTRL", BQ25790_MPPT_CTRL, 1 },
	{ "TEMP_CTRL", BQ25790_TEMP_CTRL, 1 },
	{ "NTC_CTRL_0", BQ25790_NTC_CTRL_0, 1 },
	{ "NTC_CTRL_1", BQ25790_NTC_CTRL_1, 1 },
	{ "ICO_I_LIM", BQ25790_ICO_I_LIM, 1 },
	{ "CHRG_STAT_0", BQ25790_CHRG_STAT_0, 0 },
	{ "CHRG_STAT_1", BQ25790_CHRG_STAT_1, 0 },
	{ "CHRG_STAT_2", BQ25790_CHRG_STAT_2, 0 },
	{ "CHRG_STAT_3", BQ25790_CHRG_STAT_3, 0 },
	{ "CHRG_STAT_4", BQ25790_CHRG_STAT_4, 0 },
	{ "FAULT_STAT_0", BQ25790_FAULT_STAT_0, 0 },
	{ "FAULT_STAT_1", BQ25790_FAULT_STAT_1, 0 },
	{ "CHRG_FLAG_0", BQ25790_CHRG_FLAG_0, 0 },
	{ "CHRG_FLAG_1", BQ25790_CHRG_FLAG_1, 0 },
	{ "CHRG_FLAG_2", BQ25790_CHRG_FLAG_2, 0 },
	{ "CHRG_FLAG_3", BQ25790_CHRG_FLAG_3, 0 },
	{ "FAULT_FLAG_0", BQ25790_FAULT_FLAG_0, 0 },
	{ "FAULT_FLAG_1", BQ25790_FAULT_FLAG_1, 0 },
	{ "CHRG_MSK_0", BQ25790_CHRG_MSK_0, 1 },
	{ "CHRG_MSK_1", BQ25790_CHRG_MSK_1, 1 },
	{ "CHRG_MSK_2", BQ25790_CHRG_MSK_2, 1 },
	{ "CHRG_MSK_3", BQ25790_CHRG_MSK_3, 1 },
	{ "FAULT_MSK_0", BQ25790_FAULT_MSK_0, 1 },
	{ "FAULT_MSK_1", BQ25790_FAULT_MSK_1, 1 },
	{ "ADC_CTRL", BQ25790_ADC_CTRL, 1 },
	{ "FN_DISABE_0", BQ25790_FN_DISABE_0, 1 },
	{ "FN_DISABE_1", BQ25790_FN_DISABE_1, 1 },
	{ "ADC_IBUS", BQ25790_ADC_IBUS_LSB, 1 },
	{ "ADC_IBAT_MSB", BQ25790_ADC_IBAT_MSB, 1 },
	{ "ADC_IBAT_LSB", BQ25790_ADC_IBAT_LSB, 1 },
	{ "ADC_VBUS", BQ25790_ADC_VBUS_LSB, 1 },
	{ "ADC_VAC1", BQ25790_ADC_VAC1, 1 },
	{ "ADC_VAC2", BQ25790_ADC_VAC2, 1 },
	{ "ADC_VBAT_MSB", BQ25790_ADC_VBAT_MSB, 1 },
	{ "ADC_VBAT_LSB", BQ25790_ADC_VBAT_LSB, 1 },
	{ "ADC_VSYS_MSB", BQ25790_ADC_VSYS_MSB, 1 },
	{ "ADC_VSYS_LSB", BQ25790_ADC_VSYS_LSB, 1 },
	{ "ADC_TS", BQ25790_ADC_TS, 1 },
	{ "ADC_TDIE", BQ25790_ADC_TDIE, 1 },
	{ "ADC_DP", BQ25790_ADC_DP, 1 },
	{ "ADC_DM", BQ25790_ADC_DM, 1 },
	{ "DAPM_DRV", BQ25790_DPDM_DRV, 1 },
	{ "PART_INFO", BQ25790_PART_INFO, 0 }
};

static ssize_t bq25790_registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned i, n, reg_count;
	unsigned int read_buf;
	struct bq25790_device *data = dev_get_drvdata(dev);

	reg_count = sizeof(bq25790_regs) / sizeof(bq25790_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		regmap_read(data->regmap, bq25790_regs[i].reg, &read_buf);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       bq25790_regs[i].name,
			       read_buf);
	}
	return n;
}

static ssize_t bq25790_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct bq25790_device *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(bq25790_regs) / sizeof(bq25790_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, bq25790_regs[i].name)) {
			if (bq25790_regs[i].writeable == 1) {
				error = regmap_write(data->regmap, bq25790_regs[i].reg, value);
				if (error) {
					pr_err("%s:Failed to write %s\n",
						__func__, name);
					return -1;
				}
			} else {
				pr_err("%s:Register %s is not writeable\n",
						__func__, name);
					return -1;
			}
			return count;
		}
	}
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bq25790_registers_show, bq25790_registers_store);

static struct attribute *bq25790_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group bq25790_attr_group = {
	.attrs = bq25790_attrs,
};

int bq25790_init_debug(struct bq25790_device *bq25790)
{
	int ret;
	struct bq25790_device *dbg_bq25790 = bq25790;

	ret = sysfs_create_group(&dbg_bq25790->dev->kobj, &bq25790_attr_group);
	if (ret < 0)
		dev_err(dbg_bq25790->dev, "Failed to create sysfs: %d\n", ret);

	return ret;
}
//debug end ==================


/*
static int bq25790_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct bq25790_device *bq =
			container_of(nb, struct bq25790_device, usb_nb);

	bq->usb_event = val;
	queue_work(system_power_efficient_wq, &bq->usb_work);

	return NOTIFY_OK;
}

static void bq25790_usb_work(struct work_struct *data)
{
	struct bq25790_device *bq =
			container_of(data, struct bq25790_device, usb_work);

	switch (bq->usb_event) {
	case USB_EVENT_ID:
		break;

	case USB_EVENT_NONE:
		power_supply_changed(bq->charger);
		break;
	}

	return;

	dev_err(bq->dev, "Error switching to charger mode.\n");
}
*/
static int bq25790_get_vbat_adc(struct bq25790_device *bq)
{
	int ret;
	int vbat_adc_lsb, vbat_adc_msb;
	int vbat_adc;

	ret = regmap_update_bits(bq->regmap, BQ25790_ADC_CTRL,
				 BQ25790_ADC_EN, BQ25790_ADC_EN);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_VBAT_MSB, &vbat_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_VBAT_LSB, &vbat_adc_lsb);
	if (ret)
		return ret;

	vbat_adc = (vbat_adc_msb << 8) | vbat_adc_lsb;

	return vbat_adc * BQ25790_ADC_VOLT_STEP_uV;
}

static int bq25790_get_vbus_adc(struct bq25790_device *bq)
{
	int ret;
	int vbus_adc_lsb, vbus_adc_msb;
	int vbus_adc;

	ret = regmap_update_bits(bq->regmap, BQ25790_ADC_CTRL,
				 BQ25790_ADC_EN, BQ25790_ADC_EN);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_VBUS_MSB, &vbus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_VBUS_LSB, &vbus_adc_lsb);
	if (ret)
		return ret;

	vbus_adc = (vbus_adc_msb << 8) | vbus_adc_lsb;

	return vbus_adc * BQ25790_ADC_VOLT_STEP_uV;
}

static int bq25790_get_ibus_adc(struct bq25790_device *bq)
{
	int ret;
	int ibus_adc_lsb, ibus_adc_msb;
	int ibus_adc;

	ret = regmap_update_bits(bq->regmap, BQ25790_ADC_CTRL,
				 BQ25790_ADC_EN, BQ25790_ADC_EN);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_IBUS_MSB, &ibus_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_IBUS_LSB, &ibus_adc_lsb);
	if (ret)
		return ret;

	ibus_adc = (ibus_adc_msb << 8) | ibus_adc_lsb;

	return ibus_adc * BQ25790_ADC_CURR_STEP_uA;
}


static int bq25790_get_ibat_adc(struct bq25790_device *bq)
{
	int ret;
	unsigned int ibat_adc_lsb, ibat_adc_msb;
	int ibat_adc;

	ret = regmap_update_bits(bq->regmap, BQ25790_ADC_CTRL,
				 BQ25790_ADC_EN, BQ25790_ADC_EN);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_IBAT_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_ADC_IBAT_LSB, &ibat_adc_lsb);
	if (ret)
		return ret;

	ibat_adc = (ibat_adc_msb << 8) | ibat_adc_lsb;

	return ibat_adc * BQ25790_ADC_CURR_STEP_uA;
}

static int bq25790_get_term_curr(struct bq25790_device *bq)
{
	int ret;
	int reg_val;

	ret = regmap_read(bq->regmap, BQ25790_TERM_CTRL, &reg_val);
	if (ret)
		return ret;

	reg_val &= BQ25790_TERMCHRG_CUR_MASK;

	return reg_val * BQ25790_TERMCHRG_CURRENT_STEP_uA;
}

static int bq25790_get_prechrg_curr(struct bq25790_device *bq)
{
	int ret;
	int reg_val;

	ret = regmap_read(bq->regmap, BQ25790_PRECHRG_CTRL, &reg_val);
	if (ret)
		return ret;

	reg_val &= BQ25790_PRECHRG_CUR_MASK;

	return reg_val * BQ25790_PRECHRG_CURRENT_STEP_uA;
}

static int bq25790_get_ichg_curr(struct bq25790_device *bq)
{
	int ret;
	int ichg, ichg_lsb, ichg_msb;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_I_LIM_LSB, &ichg_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_I_LIM_MSB, &ichg_msb);
	if (ret)
		return ret;

	ichg = (ichg_msb << 8) | ichg_lsb;

	return ichg * BQ25790_ICHRG_CURRENT_STEP_uA;
}

static int bq25790_set_term_curr(struct bq25790_device *bq, int term_current)
{
	int reg_val;

	if (term_current < BQ25790_TERMCHRG_I_MIN_uA)
		term_current = BQ25790_TERMCHRG_I_MIN_uA;
	else if (term_current > BQ25790_TERMCHRG_I_MAX_uA)
		term_current = BQ25790_TERMCHRG_I_MAX_uA;

	reg_val = term_current / BQ25790_TERMCHRG_CURRENT_STEP_uA;

	return regmap_update_bits(bq->regmap, BQ25790_TERM_CTRL,
				  BQ25790_TERMCHRG_CUR_MASK, reg_val);
}

static int bq25790_set_prechrg_curr(struct bq25790_device *bq, int pre_current)
{
	int reg_val;

	if (pre_current < BQ25790_PRECHRG_I_MIN_uA)
		pre_current = BQ25790_PRECHRG_I_MIN_uA;
	else if (pre_current > BQ25790_PRECHRG_I_MAX_uA)
		pre_current = BQ25790_PRECHRG_I_MAX_uA;

	reg_val = pre_current / BQ25790_PRECHRG_CURRENT_STEP_uA;

	return regmap_update_bits(bq->regmap, BQ25790_PRECHRG_CTRL,
				  BQ25790_PRECHRG_CUR_MASK, reg_val);
}

static int bq25790_set_ichrg_curr(struct bq25790_device *bq, int chrg_curr)
{
	int ret;
	int ichg, ichg_msb, ichg_lsb;

	if (chrg_curr < BQ25790_ICHRG_I_MIN_uA)
		chrg_curr = BQ25790_ICHRG_I_MIN_uA;
	else if ( chrg_curr > bq->init_data.max_ichg)
		chrg_curr = bq->init_data.max_ichg;

	ichg = chrg_curr / BQ25790_ICHRG_CURRENT_STEP_uA;
	ichg_msb = (ichg >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ25790_CHRG_I_LIM_MSB, ichg_msb);
	if (ret)
		return ret;

	ichg_lsb = ichg & 0xff;

	return regmap_write(bq->regmap, BQ25790_CHRG_I_LIM_LSB, ichg_lsb);
}

static int bq25790_set_chrg_volt(struct bq25790_device *bq, int chrg_volt)
{
	int vlim_lsb, vlim_msb, vlim;
	int ret;

	if (chrg_volt < BQ25790_VREG_V_MIN_uV)
		chrg_volt = BQ25790_VREG_V_MIN_uV;
	else if (chrg_volt > bq->init_data.max_vreg)
		chrg_volt = bq->init_data.max_vreg;

	vlim = chrg_volt / BQ25790_VREG_V_STEP_uV;
	vlim_msb = (vlim >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ25790_CHRG_V_LIM_MSB, vlim_msb);
	if (ret)
		return ret;

	vlim_lsb = vlim & 0xff;

	return regmap_write(bq->regmap, BQ25790_CHRG_V_LIM_LSB, vlim_lsb);
}

static int bq25790_get_chrg_volt(struct bq25790_device *bq)
{
	int ret;
	int vlim_lsb, vlim_msb, chrg_volt;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_V_LIM_MSB, &vlim_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_V_LIM_LSB, &vlim_lsb);
	if (ret)
		return ret;

	chrg_volt = (vlim_msb << 8) | vlim_lsb;

	return chrg_volt * BQ25790_VREG_V_STEP_uV;
}

static int bq25790_set_input_volt_lim(struct bq25790_device *bq, int vindpm)
{
	int ret;
	int vlim_lsb, vlim_msb;
	int vlim;

	if (vindpm < BQ25790_VINDPM_V_MIN_uV ||
	    vindpm > BQ25790_VINDPM_V_MAX_uV)
 		return -EINVAL;
 
	vlim = vindpm / BQ25790_VINDPM_STEP_uV;

	vlim_msb = (vlim >> 8) & 0xff;

	ret = regmap_write(bq->regmap, BQ25790_INPUT_V_LIM, vlim_msb);
	if (ret)
		return ret;

	vlim_lsb = vlim & 0xff;

	return regmap_write(bq->regmap, BQ25790_INPUT_V_LIM, vlim_lsb);
}

static int bq25790_get_input_volt_lim(struct bq25790_device *bq)
{
	int ret;
	int vlim;

	ret = regmap_read(bq->regmap, BQ25790_INPUT_V_LIM, &vlim);
	if (ret)
		return ret;

	return vlim * BQ25790_VINDPM_STEP_uV;
}

static int bq25790_set_input_curr_lim(struct bq25790_device *bq, int iindpm)
{
	int ret;
	int ilim, ilim_lsb, ilim_msb;

	if (iindpm < BQ25790_IINDPM_I_MIN_uA ||
	    iindpm > BQ25790_IINDPM_I_MAX_uA)
		return -EINVAL;

	ilim = iindpm / BQ25790_IINDPM_STEP_uA;
	ilim_msb = (ilim >> 8) & 0xff;

	ret = regmap_write(bq->regmap, BQ25790_INPUT_I_LIM_MSB, ilim_msb);
	if (ret)
		return ret;

	ilim_lsb = ilim & 0xff;

	return regmap_write(bq->regmap, BQ25790_INPUT_I_LIM_LSB, ilim_lsb);
}

static int bq25790_get_input_curr_lim(struct bq25790_device *bq)
{
	int ret;
	int ilim_msb, ilim_lsb;
	u16 ilim;

	ret = regmap_read(bq->regmap, BQ25790_INPUT_I_LIM_MSB, &ilim_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_INPUT_I_LIM_LSB, &ilim_lsb);
	if (ret)
		return ret;

	ilim = (ilim_msb << 8) | ilim_lsb;

	return ilim * BQ25790_IINDPM_STEP_uA;
}

static int bq25790_get_state(struct bq25790_device *bq,
			     struct bq25790_state *state)
{
	int chrg_stat_0, chrg_stat_1, chrg_stat_3, chrg_stat_4;
	int chrg_ctrl_0, fault_0, fault_1;
	int ret;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_STAT_0, &chrg_stat_0);
	if (ret)
	{
		//printk("regmap_read %d \n",ret);
		return ret;
	}
	state->vbus_status = chrg_stat_0 & BQ25790_VBUS_PRESENT;
	state->online = chrg_stat_0 & BQ25790_PG_STAT;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_STAT_1, &chrg_stat_1);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_CTRL_0, &chrg_ctrl_0);
	if (ret)
		return ret;

	if (chrg_ctrl_0 & BQ25790_CHRG_EN)
		state->chrg_status = chrg_stat_1 & BQ25790_CHG_STAT_MSK;
	else
		state->chrg_status = BQ25790_NOT_CHRGING;

	state->chrg_type = chrg_stat_1 & BQ25790_VBUS_STAT_MSK;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_STAT_4, &chrg_stat_4);
	if (ret)
		return ret;

	state->health = chrg_stat_4 & BQ25790_TEMP_MASK;

	ret = regmap_read(bq->regmap, BQ25790_FAULT_STAT_0, &fault_0);
	if (ret)
		return ret;

	state->fault_0 = fault_0;

	ret = regmap_read(bq->regmap, BQ25790_FAULT_STAT_1, &fault_1);
	if (ret)
		return ret;

	state->fault_1 = fault_1;

	ret = regmap_read(bq->regmap, BQ25790_CHRG_STAT_3, &chrg_stat_3);
	if (ret)
		return ret;

	state->vbat_adc = bq25790_get_vbat_adc(bq);
	state->vbus_adc = bq25790_get_vbus_adc(bq);
	state->ibat_adc = bq25790_get_ibat_adc(bq);
	state->ibus_adc = bq25790_get_ibus_adc(bq);

	return 0;
}

static int bq25790_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq25790_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		printk("===fusb302-bq25700===\n");
		ret = bq25790_set_input_curr_lim(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25790_set_input_volt_lim(bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq25790_battery_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq25790_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25790_set_chrg_volt(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25790_set_ichrg_curr(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = bq25790_set_prechrg_curr(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = bq25790_set_term_curr(bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq25790_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq25790_device *bq = power_supply_get_drvdata(psy);
	struct bq25790_state state;
	int ret = 0;

	mutex_lock(&bq->lock);
	ret = bq25790_get_state(bq, &state);
	//printk("ret %d \n",ret);
	mutex_unlock(&bq->lock);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.chrg_type || (state.chrg_type == BQ25790_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!state.chrg_status)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == BQ25790_TERM_CHRG)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (state.chrg_status) {
		case BQ25790_TRICKLE_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ25790_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ25790_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case BQ25790_TAPER_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
			break;
		case BQ25790_TOP_OFF_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ25790_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25790_MANUFACTURER;
		//printk("test===\n");
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = BQ25790_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (!state.chrg_type) {
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
		switch (state.chrg_type) {
		case BQ25790_USB_SDP:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case BQ25790_USB_CDP:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		case BQ25790_USB_DCP:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case BQ25790_OTG_MODE:
			val->intval = POWER_SUPPLY_USB_TYPE_ACA;
			break;

		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.fault_1 && (BQ25790_OTG_OVP | BQ25790_VSYS_OVP))
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		switch (state.health) {
		case BQ25790_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
			break;
		case BQ25790_TEMP_WARM:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case BQ25790_TEMP_COOL:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case BQ25790_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbus_adc;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibus_adc;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq25790_get_input_volt_lim(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq25790_get_input_curr_lim(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq25790_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct bq25790_device *bq = power_supply_get_drvdata(psy);
	struct bq25790_state state;
	int ret = 0;

	mutex_lock(&bq->lock);
	ret = bq25790_get_state(bq, &state);
	mutex_unlock(&bq->lock);
	if (ret)
		return ret;

	ret = regmap_update_bits(bq->regmap, BQ25790_ADC_CTRL,
				 BQ25790_ADC_EN, BQ25790_ADC_EN);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbat_adc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibat_adc;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq25790_get_ichg_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq->init_data.max_ichg;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq25790_get_chrg_volt(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq->init_data.max_vreg;
		break;

	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = bq25790_get_prechrg_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = bq25790_get_term_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static bool bq25790_state_changed(struct bq25790_device *bq,
				  struct bq25790_state *new_state)
{
	struct bq25790_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return (old_state.chrg_status != new_state->chrg_status ||
		old_state.chrg_fault != new_state->chrg_fault	||
		old_state.online != new_state->online		||
		old_state.health != new_state->health	||
		old_state.fault_0 != new_state->fault_0 ||
		old_state.fault_1 != new_state->fault_1 ||
		old_state.chrg_type != new_state->chrg_type ||
		old_state.vbat_adc != new_state->vbat_adc ||
		old_state.vbus_adc != new_state->vbus_adc ||
		old_state.ibat_adc != new_state->ibat_adc);
}

static irqreturn_t bq25790_irq_handler_thread(int irq, void *private)
{
	struct bq25790_device *bq = private;
	struct bq25790_state state;
	int ret;

	ret = bq25790_get_state(bq, &state);
	if (ret < 0)
		goto irq_out;

	if (!bq25790_state_changed(bq, &state))
		goto irq_out;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

static enum power_supply_property bq25790_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static enum power_supply_property bq25790_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static char *bq25790_charger_supplied_to[] = {
	"main-battery",
};

static int bq25790_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return true;
	default:
		return false;
	}
}

static const struct power_supply_desc bq25790_power_supply_desc = {
	.name = "bq25790-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = bq25790_usb_type,
	.num_usb_types = ARRAY_SIZE(bq25790_usb_type),
	.properties = bq25790_power_supply_props,
	.num_properties = ARRAY_SIZE(bq25790_power_supply_props),
	.get_property = bq25790_charger_get_property,
	.set_property = bq25790_charger_set_property,
	.property_is_writeable = bq25790_property_is_writeable,
};

static const struct power_supply_desc bq25790_battery_desc = {
	.name = "bq25790-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = bq25790_battery_get_property,
	.set_property = bq25790_battery_set_property,
	.properties = bq25790_battery_props,
	.num_properties = ARRAY_SIZE(bq25790_battery_props),
	.property_is_writeable = bq25790_property_is_writeable,
};

static bool bq25790_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BQ25790_ICO_I_LIM...BQ25790_FAULT_FLAG_1:
	case BQ25790_ADC_CTRL...BQ25790_ADC_DM:
	case BQ25790_CHRG_CTRL_0:
	
		return true;
	default:
		return false;
	}
}

static const struct regmap_config bq25790_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ25790_PART_INFO,
	.reg_defaults	= bq25790_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25790_reg_defs),
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = bq25790_is_volatile_reg,
};

static int bq25790_power_supply_init(struct bq25790_device *bq,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = bq,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = bq25790_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25790_charger_supplied_to);
    //return 0;
	bq->charger = devm_power_supply_register(bq->dev,
						 &bq25790_power_supply_desc,
						 &psy_cfg);
	if (IS_ERR(bq->charger))
		return -EINVAL;

	bq->battery = devm_power_supply_register(bq->dev,
						      &bq25790_battery_desc,
						      &psy_cfg);
	if (IS_ERR(bq->battery))
		return -EINVAL;
	return 0;
}

static int bq25790_hw_init(struct bq25790_device *bq)
{
	int ret = 0;
	int wd_reg_val = BQ25790_WATCHDOG_DIS;
	int i;

	struct power_supply_battery_info bat_info = { };
    
	

	if (bq->watchdog_timer) {
		for (i = 0; i < BQ25790_NUM_WD_VAL; i++) {
			if (bq->watchdog_timer > bq25790_watchdog_time[i] &&
			    bq->watchdog_timer < bq25790_watchdog_time[i + 1])
				wd_reg_val = i;
		}
	}

	ret = regmap_update_bits(bq->regmap, BQ25790_CHRG_CTRL_1,
				 BQ25790_WATCHDOG_MASK, 0);
    //return 0;

	ret = power_supply_get_battery_info(bq->charger, &bat_info);
	if (ret) {
		printk("1 2 battery info missing, default values will be applied\n");

		bat_info.constant_charge_current_max_ua =
				BQ25790_ICHRG_I_DEF_uA;

		bat_info.constant_charge_voltage_max_uv =
				BQ25790_VREG_V_DEF_uV;

		bat_info.precharge_current_ua =
				BQ25790_PRECHRG_I_DEF_uA;

		bat_info.charge_term_current_ua =
				BQ25790_TERMCHRG_I_DEF_uA;

		bq->init_data.max_ichg =
				BQ25790_ICHRG_I_MAX_uA;

		bq->init_data.max_vreg =
				BQ25790_VREG_V_MAX_uV;
	} else {
		printk("xxxx == battery info missing, default values will be applied\n");
		bq->init_data.max_ichg =
			bat_info.constant_charge_current_max_ua;
		bq->init_data.max_vreg =
			bat_info.constant_charge_voltage_max_uv;
	}

	ret = bq25790_set_ichrg_curr(bq, 
				bat_info.constant_charge_current_max_ua);
		printk("set_ichrg_curr == %d ua\n",bat_info.constant_charge_current_max_ua);
	if (ret)
		goto err_out;

	ret = bq25790_set_prechrg_curr(bq, bat_info.precharge_current_ua);
	printk("precharge_current_ua == %d ua\n",bat_info.precharge_current_ua);
	if (ret)
		goto err_out;

	ret = bq25790_set_chrg_volt(bq,
				bat_info.constant_charge_voltage_max_uv);
	printk("constant_charge_voltage_max_uv == %d ua\n",bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;

	ret = bq25790_set_term_curr(bq, bat_info.charge_term_current_ua);
	printk("charge_term_current_ua == %d ua\n",bat_info.charge_term_current_ua);
	if (ret)
		goto err_out;

	ret = bq25790_set_input_volt_lim(bq, bq->init_data.vlim);
	printk("bq25790_set_input_volt_lim == %d ua\n",bq->init_data.vlim);
	if (ret)
		goto err_out;

	ret = bq25790_set_input_curr_lim(bq, bq->init_data.ilim);
	printk("bq25790_set_input_curr_lim == %d ua\n",bq->init_data.ilim);
	if (ret)
		goto err_out;

	return 0;

err_out:
	return ret;
}

static int bq25790_parse_dt(struct bq25790_device *bq)
{
	int ret;

	ret = device_property_read_u32(bq->dev, "watchdog-timer",
				       &bq->watchdog_timer);
	if (ret)
		bq->watchdog_timer = BQ25790_WATCHDOG_DIS;

	if (bq->watchdog_timer > BQ25790_WATCHDOG_MAX ||
	    bq->watchdog_timer < BQ25790_WATCHDOG_DIS)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev,
				       "input-voltage-limit-microvolt",
				       &bq->init_data.vlim);
	if (ret)
		bq->init_data.vlim = BQ25790_VINDPM_DEF_uV;

	if (bq->init_data.vlim > BQ25790_VINDPM_V_MAX_uV ||
	    bq->init_data.vlim < BQ25790_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev,
				       "input-current-limit-microamp",
				       &bq->init_data.ilim);
	if (ret)
		bq->init_data.ilim = BQ25790_IINDPM_DEF_uA;

	if (bq->init_data.ilim > BQ25790_IINDPM_I_MAX_uA ||
	    bq->init_data.ilim < BQ25790_IINDPM_I_MIN_uA)
		return -EINVAL;

	return 0;
}
//extern int bq25790_init_debug(struct bq25790_device *bq25790);

static int bq25790_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bq25790_device *bq;
	unsigned int bq2592_id;
	int ret;
	printk("===================j j bq25790_probe============\n");
	bq = devm_kzalloc(dev, sizeof(struct bq25790_device), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;
    
	mutex_init(&bq->lock);
	
	//strncpy(bq->model_name,"1234596",6);
    //printk("%lu %s %s %ld %ld\n",strlen(bq->model_name),id->name,bq->model_name,sizeof(bq->model_name),sizeof(id->name));

	strncpy(bq->model_name, "ti,jbq25792",strlen("ti,jbq25792")+1);

    //return 0;
	bq->regmap = devm_regmap_init_i2c(client, &bq25790_regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(bq->regmap);
	}
   
	i2c_set_clientdata(client, bq);

	ret = bq25790_parse_dt(bq);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		return ret;
	}
   
	/* OTG reporting 
	bq->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(bq->usb2_phy)) {
		INIT_WORK(&bq->usb_work, bq25790_usb_work);
		bq->usb_nb.notifier_call = bq25790_usb_notifier;
		usb_register_notifier(bq->usb2_phy, &bq->usb_nb);
	}

	bq->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	if (!IS_ERR_OR_NULL(bq->usb3_phy)) {
		INIT_WORK(&bq->usb_work, bq25790_usb_work);
		bq->usb_nb.notifier_call = bq25790_usb_notifier;
		usb_register_notifier(bq->usb3_phy, &bq->usb_nb);
	}
    */
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						bq25790_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), bq);
		if (ret)
			goto error_out;
	}

   

	ret = bq25790_power_supply_init(bq, dev);//error

	if (ret) {
		dev_err(dev, "Failed to register power supply\n");
		goto error_out;
	}
    
    ret = regmap_read(bq->regmap, BQ25790_PART_INFO, &bq2592_id);
    
	printk("read Device Part number  bq25792 %d  \n",bq2592_id);


    ret = bq25790_hw_init(bq);//error
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto error_out;
	}
   
    ret = bq25790_init_debug(bq);
    if (ret) {
		printk("bq25790_init_debug error \n");
		//goto error_out;
	}
   
	return ret;
error_out:
	//if (!IS_ERR_OR_NULL(bq->usb2_phy))
	//	usb_unregister_notifier(bq->usb2_phy, &bq->usb_nb);

	//if (!IS_ERR_OR_NULL(bq->usb3_phy))
	//	usb_unregister_notifier(bq->usb3_phy, &bq->usb_nb);
	return ret;
}

static const struct i2c_device_id bq25790_i2c_ids[] = {
	{ BQ25790_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25790_i2c_ids);

static const struct of_device_id bq25790_of_match[] = {
	{ .compatible = "ti,bq25790", },
	{ },
};
MODULE_DEVICE_TABLE(of, bq25790_of_match);

static const struct acpi_device_id bq25790_acpi_match[] = {
	{BQ25790_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, bq25790_acpi_match);

static struct i2c_driver bq25790_driver = {
	.driver = {
		.name = "bq25790-charger",
		.of_match_table = bq25790_of_match,
		.acpi_match_table = ACPI_PTR(bq25790_acpi_match),
	},
	.probe = bq25790_probe,
	.id_table = bq25790_i2c_ids,
};
module_i2c_driver(bq25790_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_AUTHOR("Ricardo Rivera-Matos <r-rivera-matos@ti.com>");
MODULE_DESCRIPTION("bq25790 charger driver");
MODULE_LICENSE("GPL v2");
