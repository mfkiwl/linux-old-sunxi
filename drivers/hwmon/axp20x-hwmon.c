/*
 * axp20x ADC hwmon driver.
 *
 * Copyright (C) 2013 Chen-Yu Tsai <wens@csie.org>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>
#include <linux/i2c.h>

#include <linux/mfd/axp20x.h>

/* valid bits for ADC enable registers */
#define AXP20X_ADC_EN1_MASK	0xff
#define AXP20X_ADC_EN2_MASK	0x8c

/* default values from the datasheet */
#define AXP20X_ADC_EN1_DEFAULT	0x83
#define AXP20X_ADC_EN2_DEFAULT	0x80

/* enable bits for basic ADCs */
#define AXP20X_ADC_EN1_BASIC	0xfe
#define AXP20X_ADC_EN2_BASIC	0x80

/* Use MSB register offset as index */
static const char * const input_names[] = {
	[AXP20X_ACIN_V_ADC_H] 	= "ACIN",
	[AXP20X_ACIN_I_ADC_H] 	= "ACIN",
	[AXP20X_VBUS_V_ADC_H] 	= "VBUS",
	[AXP20X_VBUS_I_ADC_H] 	= "VBUS",
	[AXP20X_TEMP_ADC_H]  	= "CHIP",
	[AXP20X_TS_IN_H]	= "TS",
	[AXP20X_GPIO0_V_ADC_H] 	= "GPIO0",
	[AXP20X_GPIO1_V_ADC_H] 	= "GPIO1",
	[AXP20X_PWR_BATT_H]	= "BATT",
	[AXP20X_BATT_V_H]	= "BATT",
	[AXP20X_BATT_CHRG_I_H]	= "BATT_CHRG",
	[AXP20X_BATT_DISCHRG_I_H] = "BATT_DISCHRG",
	[AXP20X_APS_V_H]	= "APS",
};

static const int input_step[] = {
	[AXP20X_ACIN_V_ADC_H]	= 1700,
	[AXP20X_ACIN_I_ADC_H]	= 625,
	[AXP20X_VBUS_V_ADC_H]	= 1700,
	[AXP20X_VBUS_I_ADC_H]	= 375,
	[AXP20X_TEMP_ADC_H]	= 100,
	[AXP20X_TS_IN_H]	= 800,
	[AXP20X_GPIO0_V_ADC_H]	= 500,
	[AXP20X_GPIO1_V_ADC_H]	= 500,
	[AXP20X_PWR_BATT_H]	= 1100,
	[AXP20X_BATT_V_H]	= 1100,
	[AXP20X_BATT_CHRG_I_H]	= 500,
	[AXP20X_BATT_DISCHRG_I_H] = 500,
	[AXP20X_APS_V_H]	= 1400,
};

static int axp20x_adc_read(struct axp20x_dev *axp20x, int channel)
{
	unsigned char val[3];
	int ret;

	if (channel < AXP20X_ACIN_V_ADC_H || channel > AXP20X_APS_V_H)
		return -EINVAL;

	/* ADC values are split across at most 3 registers */
	ret = regmap_bulk_read(axp20x->regmap, channel, val, 3);
	if (ret) {
		dev_dbg(axp20x->dev, "Read ADC 0x%02x failed: %d\n", channel,
				ret);
		return ret;
	}

	if (channel == AXP20X_PWR_BATT_H)
		ret = val[0] << 16 | val[1] << 8 | val[2];
	else
		ret = val[0] << 4 | val[1];

	return ret;
}

static ssize_t show_voltage(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(dev);
	int channel = to_sensor_dev_attr(attr)->index;
	int val;

	val = axp20x_adc_read(axp20x, channel) * input_step[channel];
	val = DIV_ROUND_CLOSEST(val, 1000);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_temp(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(dev);
	int channel = to_sensor_dev_attr(attr)->index;
	int val;

	val = axp20x_adc_read(axp20x, channel) * input_step[channel] - 144700;

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_label(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int channel = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", input_names[channel]);
}

#define AXP20X_NAMED_VOLTAGE(id, name) \
	static SENSOR_DEVICE_ATTR(in##id##_input, S_IRUGO, show_voltage,\
				  NULL, name);		\
	static SENSOR_DEVICE_ATTR(in##id##_label, S_IRUGO, show_label,\
				  NULL, name)

#define AXP20X_NAMED_CURRENT(id, name) \
	static SENSOR_DEVICE_ATTR(curr##id##_input, S_IRUGO, show_voltage,\
				  NULL, name);		\
	static SENSOR_DEVICE_ATTR(curr##id##_label, S_IRUGO, show_label,\
				  NULL, name)

#define AXP20X_NAMED_POWER(id, name) \
	static SENSOR_DEVICE_ATTR(power##id##_input, S_IRUGO, show_voltage,\
				  NULL, name);		\
	static SENSOR_DEVICE_ATTR(power##id##_label, S_IRUGO, show_label,\
				  NULL, name)

#define AXP20X_NAMED_TEMP(id, name) \
	static SENSOR_DEVICE_ATTR(temp##id##_input, S_IRUGO, show_temp,\
				  NULL, name);		\
	static SENSOR_DEVICE_ATTR(temp##id##_label, S_IRUGO, show_label,\
				  NULL, name)

AXP20X_NAMED_VOLTAGE(0, AXP20X_ACIN_V_ADC_H);
AXP20X_NAMED_VOLTAGE(1, AXP20X_VBUS_V_ADC_H);
AXP20X_NAMED_VOLTAGE(2, AXP20X_BATT_V_H);
AXP20X_NAMED_VOLTAGE(3, AXP20X_APS_V_H);
AXP20X_NAMED_CURRENT(1, AXP20X_ACIN_I_ADC_H);
AXP20X_NAMED_CURRENT(2, AXP20X_VBUS_I_ADC_H);
AXP20X_NAMED_CURRENT(3, AXP20X_BATT_CHRG_I_H);
AXP20X_NAMED_CURRENT(4, AXP20X_BATT_DISCHRG_I_H);
AXP20X_NAMED_POWER(1, AXP20X_PWR_BATT_H);
AXP20X_NAMED_TEMP(1, AXP20X_TEMP_ADC_H);

static struct attribute *axp20x_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_label.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_label.dev_attr.attr,

	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_curr1_label.dev_attr.attr,
	&sensor_dev_attr_curr2_input.dev_attr.attr,
	&sensor_dev_attr_curr2_label.dev_attr.attr,
	&sensor_dev_attr_curr3_input.dev_attr.attr,
	&sensor_dev_attr_curr3_label.dev_attr.attr,
	&sensor_dev_attr_curr4_input.dev_attr.attr,
	&sensor_dev_attr_curr4_label.dev_attr.attr,

	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_power1_label.dev_attr.attr,

	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,

	NULL,
};
ATTRIBUTE_GROUPS(axp20x);

static int axp20x_hwmon_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct device *hwmon_dev;
	int ret;

	ret = regmap_update_bits(axp20x->regmap, AXP20X_ADC_EN1,
			AXP20X_ADC_EN1_MASK, AXP20X_ADC_EN1_BASIC);
	if (!ret)
		ret = regmap_update_bits(axp20x->regmap, AXP20X_ADC_EN2,
				AXP20X_ADC_EN2_MASK, AXP20X_ADC_EN2_BASIC);

	if (ret) {
		dev_err(&pdev->dev, "Failed to enable ADCs\n");
		goto err;
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(
			&pdev->dev,
			axp20x->i2c_client->name,
			axp20x,
			axp20x_groups
			);
	if (IS_ERR(hwmon_dev))
		goto err_adc_en;

	return 0;

err_adc_en:
	regmap_update_bits(axp20x->regmap, AXP20X_ADC_EN1, AXP20X_ADC_EN1_MASK,
			AXP20X_ADC_EN1_DEFAULT);
	regmap_update_bits(axp20x->regmap, AXP20X_ADC_EN2, AXP20X_ADC_EN2_MASK,
			AXP20X_ADC_EN2_DEFAULT);
err:
	return ret;
}

static int axp20x_hwmon_remove(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = platform_get_drvdata(pdev);

	regmap_update_bits(axp20x->regmap, AXP20X_ADC_EN1, AXP20X_ADC_EN1_MASK,
			AXP20X_ADC_EN1_DEFAULT);
	regmap_update_bits(axp20x->regmap, AXP20X_ADC_EN2, AXP20X_ADC_EN2_MASK,
			AXP20X_ADC_EN2_DEFAULT);

	return 0;
}

static struct platform_driver axp20x_hwmon_driver = {
	.probe = axp20x_hwmon_probe,
	.remove = axp20x_hwmon_remove,
	.driver = {
		.name = "axp20x-hwmon",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(axp20x_hwmon_driver);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Chen-Yu Tsai <wens@csie.org>");
MODULE_DESCRIPTION("Hardware Monitoring Driver for AXP20X PMIC");
