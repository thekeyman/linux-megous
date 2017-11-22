/*
 * axp813_charger.c - X-Powers AXP813 PMIC Charger driver
 *
 * Copyright (C) 2017 Touchless Biometric Systems AG
 * Author: Tomas Novotny <tomas.novotny@tbs-biometrics.com>
 *
 * Updated version of axp288_charger.c:
 * Copyright (C) 2014 Intel Corporation
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This driver is mainly a configuration of our (TBS) charger.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/axp20x.h>

#define CHRG_CCCV_ITERM_20P		(1 << 4)	/* 20% of CC */

#define CHRG_CCCV_CHG_EN		(1 << 7)

#define CNTL2_CC_TIMEOUT_MASK		0x03	/* 2 bits */
#define CNTL2_CC_TIMEOUT_BIT_POS	0
#define CNTL2_CC_TIMEOUT_10HRS		0x02	/* 10 Hrs */
#define CNTL2_CC_TIMEOUT_12HRS		0x03	/* 12 Hrs */
#define CNTL2_CHG_OUT_TURNON		(1 << 5)
#define CNTL2_PC_TIMEOUT_MASK		0xC0
#define CNTL2_PC_TIMEOUT_BIT_POS	6
#define CNTL2_PC_TIMEOUT_60MINS		0x02
#define CNTL2_PC_TIMEOUT_70MINS		0x03
#define OFF_CNTL_CHGLED_DIRECT_CONTROL	GENMASK(5, 4)
#define OFF_CNTL_CHGLED_CONTROL		BIT(3)
#define CNTL2_CHGLED_TYPE		BIT(4)

#define CHRG_VLTFC_0C			0xA5	/* 0 DegC */
#define CHRG_VHTFC_45C			0x1F	/* 45 DegC */

#define FG_CNTL_OCV_ADJ_EN		(1 << 3)

/*
enum {
	VBUS_OV_IRQ = 0,
	CHARGE_DONE_IRQ,
	CHARGE_CHARGING_IRQ,
	BAT_SAFE_QUIT_IRQ,
	BAT_SAFE_ENTER_IRQ,
	QCBTU_IRQ,
	CBTU_IRQ,
	QCBTO_IRQ,
	CBTO_IRQ,
	CHRG_INTR_END,
};
*/

struct axp813_chrg_info {
	struct platform_device *pdev;
	struct regmap *regmap;
	/*
	struct regmap_irq_chip_data *regmap_irqc;
	int irq[CHRG_INTR_END];
	*/
	struct power_supply *psy_usb;
	struct mutex lock;

	int is_charger_enabled;
};


static int axp813_charger_enable_charger(struct axp813_chrg_info *info,
								bool enable)
{
	int ret;

	if ((int)enable == info->is_charger_enabled)
		return 0;

	if (enable)
		ret = regmap_update_bits(info->regmap, AXP20X_CHRG_CTRL1,
				CHRG_CCCV_CHG_EN, CHRG_CCCV_CHG_EN);
	else
		ret = regmap_update_bits(info->regmap, AXP20X_CHRG_CTRL1,
				CHRG_CCCV_CHG_EN, 0);
	if (ret < 0)
		dev_err(&info->pdev->dev, "axp813 enable charger %d\n", ret);
	else
		info->is_charger_enabled = enable;

	return ret;
}

/*
 * TODO: extend the USB power source with the overheat sensing
#define PS_STAT_VBUS_VALID		(1 << 4)
#define PS_STAT_VBUS_PRESENT		(1 << 5)
#define CHRG_STAT_BAT_SAFE_MODE		(1 << 3)
#define CHRG_STAT_PMIC_OTP		(1 << 7)
static int axp813_get_charger_health(struct axp813_chrg_info *info)
{
	int ret, pwr_stat, chrg_stat;
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;
	unsigned int val;

	ret = regmap_read(info->regmap, AXP20X_PWR_INPUT_STATUS, &val);
	if ((ret < 0) || !(val & PS_STAT_VBUS_PRESENT))
		goto health_read_fail;
	else
		pwr_stat = val;

	ret = regmap_read(info->regmap, AXP20X_PWR_OP_MODE, &val);
	if (ret < 0)
		goto health_read_fail;
	else
		chrg_stat = val;

	if (!(pwr_stat & PS_STAT_VBUS_VALID))
		health = POWER_SUPPLY_HEALTH_DEAD;
	else if (chrg_stat & CHRG_STAT_PMIC_OTP)
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chrg_stat & CHRG_STAT_BAT_SAFE_MODE)
		health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	else
		health = POWER_SUPPLY_HEALTH_GOOD;

health_read_fail:
	return health;
}
*/

static int axp813_charger_usb_set_property(struct power_supply *psy,
                                    enum power_supply_property psp,
                                    const union power_supply_propval *val)
{
	struct axp813_chrg_info *info = power_supply_get_drvdata(psy);
	int ret = 0;
	//int scaled_val;

	mutex_lock(&info->lock);

	switch (psp) {
/*		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			scaled_val = min(val->intval, info->max_cc);
			scaled_val = DIV_ROUND_CLOSEST(scaled_val, 1000);
			ret = axp813_charger_set_cc(info, scaled_val);
			if (ret < 0)
				dev_warn(&info->pdev->dev, "set charge current failed\n");
			break; */
		default:
			ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}



static int axp813_charger_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct axp813_chrg_info *info = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
/*
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = axp813_get_charger_health(info);
		break; */
	default:
		ret = -EINVAL;
		goto psy_get_prop_fail;
	}

psy_get_prop_fail:
	mutex_unlock(&info->lock);
	return ret;
}

static int axp813_charger_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
/*	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = 1;
		break; */
	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_property axp813_usb_props[] = {
	POWER_SUPPLY_PROP_TYPE,
/*	POWER_SUPPLY_PROP_HEALTH, */
};

static const struct power_supply_desc axp813_charger_desc = {
	.name			= "axp813_charger",
	/* It might be also mains supply */
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= axp813_usb_props,
	.num_properties		= ARRAY_SIZE(axp813_usb_props),
	.get_property		= axp813_charger_usb_get_property,
	.set_property           = axp813_charger_usb_set_property,
	.property_is_writeable	= axp813_charger_property_is_writeable,
};

/*
static irqreturn_t axp813_charger_irq_thread_handler(int irq, void *dev)
{
	struct axp813_chrg_info *info = dev;
	int i;

	for (i = 0; i < CHRG_INTR_END; i++) {
		if (info->irq[i] == irq)
			break;
	}

	if (i >= CHRG_INTR_END) {
		dev_warn(&info->pdev->dev, "spurious interrupt!!\n");
		return IRQ_NONE;
	}

	switch (i) {
	case VBUS_OV_IRQ:
		dev_dbg(&info->pdev->dev, "VBUS Over Voltage INTR\n");
		break;
	case CHARGE_DONE_IRQ:
		dev_dbg(&info->pdev->dev, "Charging Done INTR\n");
		break;
	case CHARGE_CHARGING_IRQ:
		dev_dbg(&info->pdev->dev, "Start Charging IRQ\n");
		break;
	case BAT_SAFE_QUIT_IRQ:
		dev_dbg(&info->pdev->dev,
			"Quit Safe Mode(restart timer) Charging IRQ\n");
		break;
	case BAT_SAFE_ENTER_IRQ:
		dev_dbg(&info->pdev->dev,
			"Enter Safe Mode(timer expire) Charging IRQ\n");
		break;
	case QCBTU_IRQ:
		dev_dbg(&info->pdev->dev,
			"Quit Battery Under Temperature(CHRG) INTR\n");
		break;
	case CBTU_IRQ:
		dev_dbg(&info->pdev->dev,
			"Hit Battery Under Temperature(CHRG) INTR\n");
		break;
	case QCBTO_IRQ:
		dev_dbg(&info->pdev->dev,
			"Quit Battery Over Temperature(CHRG) INTR\n");
		break;
	case CBTO_IRQ:
		dev_dbg(&info->pdev->dev,
			"Hit Battery Over Temperature(CHRG) INTR\n");
		break;
	default:
		dev_warn(&info->pdev->dev, "Spurious Interrupt!!!\n");
		goto out;
	}

	power_supply_changed(info->psy_usb);
out:
	return IRQ_HANDLED;
}
*/

static int charger_init_hw_regs(struct axp813_chrg_info *info)
{
	int ret;

	/* Program temperature thresholds */
	ret = regmap_write(info->regmap, AXP20X_V_LTF_CHRG, CHRG_VLTFC_0C);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
							AXP20X_V_LTF_CHRG, ret);
		return ret;
	}

	ret = regmap_write(info->regmap, AXP20X_V_HTF_CHRG, CHRG_VHTFC_45C);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
							AXP20X_V_HTF_CHRG, ret);
		return ret;
	}

	/* TODO work thresholds? */

	/* Do not turn-off charger o/p after charge cycle ends */
	ret = regmap_update_bits(info->regmap,
				AXP20X_CHRG_CTRL2,
				CNTL2_CHG_OUT_TURNON, CNTL2_CHG_OUT_TURNON);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_CHRG_CTRL2, ret);
		return ret;
	}

	/* Setup ending condition for charging to be 10% of I(chrg) */
	// TODO Consider 20%
	ret = regmap_update_bits(info->regmap,
				AXP20X_CHRG_CTRL1,
				CHRG_CCCV_ITERM_20P, 0);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_CHRG_CTRL1, ret);
		return ret;
	}

	/* Pre-charge timer */
	ret = regmap_update_bits(info->regmap,
				AXP20X_CHRG_CTRL2,
				CNTL2_PC_TIMEOUT_MASK,
				CNTL2_PC_TIMEOUT_60MINS << CNTL2_PC_TIMEOUT_BIT_POS);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_CHRG_CTRL2, ret);
		return ret;
	}

	/* Fast charge max time */
	ret = regmap_update_bits(info->regmap,
				AXP20X_CHRG_CTRL2,
				CNTL2_CC_TIMEOUT_MASK,
				CNTL2_CC_TIMEOUT_12HRS << CNTL2_CC_TIMEOUT_BIT_POS);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_CHRG_CTRL2, ret);
		return ret;
	}

	/* Charger led */
	ret = regmap_update_bits(info->regmap, AXP20X_OFF_CTRL,
				OFF_CNTL_CHGLED_CONTROL, OFF_CNTL_CHGLED_CONTROL);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_OFF_CTRL, ret);
		return ret;
	}
	ret = regmap_update_bits(info->regmap, AXP20X_CHRG_CTRL2,
				CNTL2_CHGLED_TYPE, CNTL2_CHGLED_TYPE);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_CHRG_CTRL2, ret);
		return ret;
	}

	/* Disable OCV-SOC curve calibration */
	ret = regmap_update_bits(info->regmap,
				AXP20X_CC_CTRL,
				FG_CNTL_OCV_ADJ_EN, 0);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "register(%x) write error(%d)\n",
						AXP20X_CC_CTRL, ret);
		return ret;
	}

	return 0;
}

static int axp813_charger_probe(struct platform_device *pdev)
{
	int ret/*, i, pirq*/;
	struct axp813_chrg_info *info;
	struct device *dev = &pdev->dev;
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config charger_cfg = {};
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdev = pdev;
	info->regmap = axp20x->regmap;
	//info->regmap_irqc = axp20x->regmap_irqc;
	info->is_charger_enabled = -1;

	platform_set_drvdata(pdev, info);
	mutex_init(&info->lock);

	ret = charger_init_hw_regs(info);
	if (ret)
		return ret;

	/* Register with power supply class */
	charger_cfg.drv_data = info;
	info->psy_usb = devm_power_supply_register(dev, &axp813_charger_desc,
						   &charger_cfg);
	if (IS_ERR(info->psy_usb)) {
		ret = PTR_ERR(info->psy_usb);
		dev_err(dev, "failed to register power supply: %d\n", ret);
		return ret;
	}

	ret = axp813_charger_enable_charger(info, true);
	if (ret)
		return ret;

	/* Register charger interrupts */
	/* TODO: Irqs: next time. Be aware that AXP288 has different mappings. */
	/*
	for (i = 0; i < CHRG_INTR_END; i++) {
		pirq = platform_get_irq(info->pdev, i);
		info->irq[i] = regmap_irq_get_virq(info->regmap_irqc, pirq);
		if (info->irq[i] < 0) {
			dev_warn(&info->pdev->dev,
				"failed to get virtual interrupt=%d\n", pirq);
			return info->irq[i];
		}
		ret = devm_request_threaded_irq(&info->pdev->dev, info->irq[i],
					NULL, axp813_charger_irq_thread_handler,
					IRQF_ONESHOT, info->pdev->name, info);
		if (ret) {
			dev_err(&pdev->dev, "failed to request interrupt=%d\n",
								info->irq[i]);
			return ret;
		}
	}
	*/

	return 0;
}

static const struct platform_device_id axp813_charger_id_table[] = {
	{ .name = "axp813_charger" },
	{},
};
MODULE_DEVICE_TABLE(platform, axp813_charger_id_table);

static struct platform_driver axp813_charger_driver = {
	.probe = axp813_charger_probe,
	.id_table = axp813_charger_id_table,
	.driver = {
		.name = "axp813_charger",
	},
};

module_platform_driver(axp813_charger_driver);

MODULE_AUTHOR("Tomas Novotny <tomas.novotny@tbs-biometrics.com>");
MODULE_DESCRIPTION("X-Powers AXP813 Charger Driver");
MODULE_LICENSE("GPL v2");
