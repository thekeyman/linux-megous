/*
 * AXP20x pinctrl and GPIO driver
 *
 * Copyright (C) 2016 Maxime Ripard <maxime.ripard@free-electrons.com>
 * Copyright (C) 201 Quentin Schulz <quentin.schulz@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/axp20x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>

#define AXP20X_GPIO_FUNCTIONS		0x7
#define AXP20X_GPIO_FUNCTION_OUT_LOW	0
#define AXP20X_GPIO_FUNCTION_OUT_HIGH	1
#define AXP20X_GPIO_FUNCTION_INPUT	2

#define AXP20X_PINCTRL_PIN(_pin_num, _pin)			\
	{							\
		.number = _pin_num,				\
		.name = _pin,					\
	}

#define AXP20X_PIN(_pin, ...)					\
	{							\
		.pin = _pin,					\
		.functions = (struct axp20x_desc_function[]) {	\
			      __VA_ARGS__, { } },		\
	}

#define AXP20X_FUNCTION(_val, _name)				\
	{							\
		.name = _name,					\
		.muxval = _val,					\
	}

struct axp20x_desc_function {
	const char	*name;
	u8		muxval;
};

struct axp20x_desc_pin {
	struct pinctrl_pin_desc		pin;
	struct axp20x_desc_function	*functions;
};

struct axp20x_pinctrl_desc {
	const struct axp20x_desc_pin	*pins;
	int				npins;
};

struct axp20x_pinctrl_group {
	const char	*name;
	unsigned long	config;
	unsigned int	pin;
};

struct axp20x_pinctrl_function {
	const char	*name;
	const char	**groups;
	unsigned int	ngroups;
};

struct axp20x_gpio {
	struct gpio_chip	chip;
	struct regmap		*regmap;
	struct pinctrl_dev			*pctl_dev;
	struct device				*dev;
	const struct axp20x_pinctrl_desc	*desc;
	struct axp20x_pinctrl_group		*groups;
	unsigned int				ngroups;
	struct axp20x_pinctrl_function		*functions;
	unsigned int				nfunctions;
};

static const struct axp20x_desc_pin axp209_pins[] = {
	AXP20X_PIN(AXP20X_PINCTRL_PIN(0, "GPIO0"),
		   AXP20X_FUNCTION(0x0, "gpio_out"),
		   AXP20X_FUNCTION(0x2, "gpio_in"),
		   AXP20X_FUNCTION(0x3, "ldo"),
		   AXP20X_FUNCTION(0x4, "adc")),
	AXP20X_PIN(AXP20X_PINCTRL_PIN(1, "GPIO1"),
		   AXP20X_FUNCTION(0x0, "gpio_out"),
		   AXP20X_FUNCTION(0x2, "gpio_in"),
		   AXP20X_FUNCTION(0x3, "ldo"),
		   AXP20X_FUNCTION(0x4, "adc")),
	AXP20X_PIN(AXP20X_PINCTRL_PIN(2, "GPIO2"),
		   AXP20X_FUNCTION(0x0, "gpio_out"),
		   AXP20X_FUNCTION(0x2, "gpio_in")),
};

static const struct axp20x_pinctrl_desc axp20x_pinctrl_data = {
	.pins	= axp209_pins,
	.npins	= ARRAY_SIZE(axp209_pins),
};

static int axp20x_gpio_get_reg(unsigned offset)
{
	switch (offset) {
	case 0:
		return AXP20X_GPIO0_CTRL;
	case 1:
		return AXP20X_GPIO1_CTRL;
	case 2:
		return AXP20X_GPIO2_CTRL;
	}

	return -EINVAL;
}

static int axp20x_gpio_input(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int axp20x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct axp20x_gpio *gpio = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->regmap, AXP20X_GPIO20_SS, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset + 4));
}

static int axp20x_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct axp20x_gpio *gpio = gpiochip_get_data(chip);
	unsigned int val;
	int reg, ret;

	reg = axp20x_gpio_get_reg(offset);
	if (reg < 0)
		return reg;

	ret = regmap_read(gpio->regmap, reg, &val);
	if (ret)
		return ret;

	/*
	 * This shouldn't really happen if the pin is in use already,
	 * or if it's not in use yet, it doesn't matter since we're
	 * going to change the value soon anyway. Default to output.
	 */
	if ((val & AXP20X_GPIO_FUNCTIONS) > 2)
		return 0;

	/*
	 * The GPIO directions are the three lowest values.
	 * 2 is input, 0 and 1 are output
	 */
	return val & 2;
}

static int axp20x_gpio_output(struct gpio_chip *chip, unsigned offset,
			      int value)
{
	chip->set(chip, offset, value);

	return 0;
}

static void axp20x_gpio_set(struct gpio_chip *chip, unsigned offset,
			    int value)
{
	struct axp20x_gpio *gpio = gpiochip_get_data(chip);
	int reg;

	reg = axp20x_gpio_get_reg(offset);
	if (reg < 0)
		return;

	regmap_update_bits(gpio->regmap, reg,
			   AXP20X_GPIO_FUNCTIONS,
			   value ? AXP20X_GPIO_FUNCTION_OUT_HIGH :
			   AXP20X_GPIO_FUNCTION_OUT_LOW);
}

static int axp20x_pmx_set(struct pinctrl_dev *pctldev, unsigned int offset,
			  u8 config)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);
	int reg;

	reg = axp20x_gpio_get_reg(offset);
	if (reg < 0)
		return reg;

	return regmap_update_bits(gpio->regmap, reg, AXP20X_GPIO_FUNCTIONS,
				  config);
}

static int axp20x_pmx_func_cnt(struct pinctrl_dev *pctldev)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);

	return gpio->nfunctions;
}

static const char *axp20x_pmx_func_name(struct pinctrl_dev *pctldev,
					unsigned int selector)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);

	return gpio->functions[selector].name;
}

static int axp20x_pmx_func_groups(struct pinctrl_dev *pctldev,
				  unsigned int selector,
				  const char * const **groups,
				  unsigned int *num_groups)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);

	*groups = gpio->functions[selector].groups;
	*num_groups = gpio->functions[selector].ngroups;

	return 0;
}

static struct axp20x_desc_function *
axp20x_pinctrl_desc_find_func_by_name(struct axp20x_gpio *gpio,
				      const char *group, const char *func)
{
	const struct axp20x_desc_pin *pin;
	struct axp20x_desc_function *desc_func;
	int i;

	for (i = 0; i < gpio->desc->npins; i++) {
		pin = &gpio->desc->pins[i];

		if (!strcmp(pin->pin.name, group)) {
			desc_func = pin->functions;

			while (desc_func->name) {
				if (!strcmp(desc_func->name, func))
					return desc_func;
				desc_func++;
			}

			/*
			 * Pins are uniquely named. Groups are named after one
			 * pin name. If one pin matches group name but its
			 * function cannot be found, no other pin will match
			 * group name.
			 */
			return NULL;
		}
	}

	return NULL;
}

static int axp20x_pmx_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int function, unsigned int group)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);
	struct axp20x_pinctrl_group *g = gpio->groups + group;
	struct axp20x_pinctrl_function *func = gpio->functions + function;
	struct axp20x_desc_function *desc_func =
		axp20x_pinctrl_desc_find_func_by_name(gpio, g->name,
						      func->name);
	if (!desc_func)
		return -EINVAL;

	return axp20x_pmx_set(pctldev, g->pin, desc_func->muxval);
}

static struct axp20x_desc_function *
axp20x_pctl_desc_find_func_by_pin(struct axp20x_gpio *gpio, unsigned int offset,
				  const char *func)
{
	const struct axp20x_desc_pin *pin;
	struct axp20x_desc_function *desc_func;
	int i;

	for (i = 0; i < gpio->desc->npins; i++) {
		pin = &gpio->desc->pins[i];

		if (pin->pin.number == offset) {
			desc_func = pin->functions;

			while (desc_func->name) {
				if (!strcmp(desc_func->name, func))
					return desc_func;

				desc_func++;
			}
		}
	}

	return NULL;
}

static int axp20x_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					 struct pinctrl_gpio_range *range,
					 unsigned int offset, bool input)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);
	struct axp20x_desc_function *desc_func;
	const char *func;

	if (input)
		func = "gpio_in";
	else
		func = "gpio_out";

	desc_func = axp20x_pctl_desc_find_func_by_pin(gpio, offset, func);
	if (!desc_func)
		return -EINVAL;

	return axp20x_pmx_set(pctldev, offset, desc_func->muxval);
}

static const struct pinmux_ops axp20x_pmx_ops = {
	.get_functions_count	= axp20x_pmx_func_cnt,
	.get_function_name	= axp20x_pmx_func_name,
	.get_function_groups	= axp20x_pmx_func_groups,
	.set_mux		= axp20x_pmx_set_mux,
	.gpio_set_direction	= axp20x_pmx_gpio_set_direction,
	.strict			= true,
};

static int axp20x_groups_cnt(struct pinctrl_dev *pctldev)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);

	return gpio->ngroups;
}

static int axp20x_group_pins(struct pinctrl_dev *pctldev, unsigned int selector,
			     const unsigned int **pins, unsigned int *num_pins)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);
	struct axp20x_pinctrl_group *g = gpio->groups + selector;

	*pins = (unsigned int *)&g->pin;
	*num_pins = 1;

	return 0;
}

static const char *axp20x_group_name(struct pinctrl_dev *pctldev,
				     unsigned int selector)
{
	struct axp20x_gpio *gpio = pinctrl_dev_get_drvdata(pctldev);

	return gpio->groups[selector].name;
}

static const struct pinctrl_ops axp20x_pctrl_ops = {
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinconf_generic_dt_free_map,
	.get_groups_count	= axp20x_groups_cnt,
	.get_group_name		= axp20x_group_name,
	.get_group_pins		= axp20x_group_pins,
};

static struct axp20x_pinctrl_function *
axp20x_pinctrl_function_by_name(struct axp20x_gpio *gpio, const char *name)
{
	struct axp20x_pinctrl_function *func = gpio->functions;

	while (func->name) {
		if (!strcmp(func->name, name))
			return func;
		func++;
	}

	return NULL;
}

static int axp20x_pinctrl_add_function(struct axp20x_gpio *gpio,
				       const char *name)
{
	struct axp20x_pinctrl_function *func = gpio->functions;

	while (func->name) {
		if (!strcmp(func->name, name)) {
			func->ngroups++;
			return -EEXIST;
		}

		func++;
	}

	func->name = name;
	func->ngroups = 1;

	gpio->nfunctions++;

	return 0;
}

static int axp20x_attach_group_function(struct platform_device *pdev,
					const struct axp20x_desc_pin *pin)
{
	struct axp20x_gpio *gpio = platform_get_drvdata(pdev);
	struct axp20x_desc_function *desc_func = pin->functions;
	struct axp20x_pinctrl_function *func;
	const char **func_grp;

	while (desc_func->name) {
		func = axp20x_pinctrl_function_by_name(gpio, desc_func->name);
		if (!func)
			return -EINVAL;

		if (!func->groups) {
			func->groups = devm_kzalloc(&pdev->dev,
						    func->ngroups * sizeof(const char *),
						    GFP_KERNEL);
			if (!func->groups)
				return -ENOMEM;
		}

		func_grp = func->groups;
		while (*func_grp)
			func_grp++;

		*func_grp = pin->pin.name;
		desc_func++;
	}

	return 0;
}

static int axp20x_build_state(struct platform_device *pdev)
{
	struct axp20x_gpio *gpio = platform_get_drvdata(pdev);
	unsigned int npins = gpio->desc->npins;
	const struct axp20x_desc_pin *pin;
	struct axp20x_desc_function *func;
	int i, ret;

	gpio->ngroups = npins;
	gpio->groups = devm_kzalloc(&pdev->dev,
				    gpio->ngroups * sizeof(*gpio->groups),
				    GFP_KERNEL);
	if (!gpio->groups)
		return -ENOMEM;

	for (i = 0; i < npins; i++) {
		gpio->groups[i].name = gpio->desc->pins[i].pin.name;
		gpio->groups[i].pin = gpio->desc->pins[i].pin.number;
	}

	/* We assume 4 functions per pin should be enough as a default max */
	gpio->functions = devm_kzalloc(&pdev->dev,
				       npins * 4 * sizeof(*gpio->functions),
				       GFP_KERNEL);
	if (!gpio->functions)
		return -ENOMEM;

	/* Create a list of uniquely named functions */
	for (i = 0; i < npins; i++) {
		pin = &gpio->desc->pins[i];
		func = pin->functions;

		while (func->name) {
			axp20x_pinctrl_add_function(gpio, func->name);
			func++;
		}
	}

	gpio->functions = krealloc(gpio->functions,
				   gpio->nfunctions * sizeof(*gpio->functions),
				   GFP_KERNEL);

	for (i = 0; i < npins; i++) {
		pin = &gpio->desc->pins[i];
		ret = axp20x_attach_group_function(pdev, pin);
		if (ret)
			return ret;
	}

	return 0;
}

static int axp20x_gpio_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x = dev_get_drvdata(pdev->dev.parent);
	struct axp20x_gpio *gpio;
	const struct axp20x_desc_pin *pin;
	struct pinctrl_desc *pctrl_desc;
	struct pinctrl_pin_desc *pins;
	int ret, i;

	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	if (!axp20x) {
		dev_err(&pdev->dev, "Parent drvdata not set\n");
		return -EINVAL;
	}

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->chip.base			= -1;
	gpio->chip.can_sleep		= true;
	gpio->chip.request		= gpiochip_generic_request;
	gpio->chip.free			= gpiochip_generic_free;
	gpio->chip.parent		= &pdev->dev;
	gpio->chip.label		= dev_name(&pdev->dev);
	gpio->chip.owner		= THIS_MODULE;
	gpio->chip.get			= axp20x_gpio_get;
	gpio->chip.get_direction	= axp20x_gpio_get_direction;
	gpio->chip.set			= axp20x_gpio_set;
	gpio->chip.direction_input	= axp20x_gpio_input;
	gpio->chip.direction_output	= axp20x_gpio_output;
	gpio->chip.ngpio		= 3;

	gpio->regmap = axp20x->regmap;

	gpio->desc = &axp20x_pinctrl_data;
	gpio->dev = &pdev->dev;

	platform_set_drvdata(pdev, gpio);

	ret = axp20x_build_state(pdev);
	if (ret)
		return ret;

	pins = devm_kzalloc(&pdev->dev, gpio->desc->npins * sizeof(*pins),
			    GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	for (i = 0; i < gpio->desc->npins; i++)
		pins[i] = gpio->desc->pins[i].pin;

	pctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*pctrl_desc), GFP_KERNEL);
	if (!pctrl_desc)
		return -ENOMEM;

	pctrl_desc->name = dev_name(&pdev->dev);
	pctrl_desc->owner = THIS_MODULE;
	pctrl_desc->pins = pins;
	pctrl_desc->npins = gpio->desc->npins;
	pctrl_desc->pctlops = &axp20x_pctrl_ops;
	pctrl_desc->pmxops = &axp20x_pmx_ops;

	gpio->pctl_dev = devm_pinctrl_register(&pdev->dev, pctrl_desc, gpio);
	if (IS_ERR(gpio->pctl_dev)) {
		dev_err(&pdev->dev, "couldn't register pinctrl driver\n");
		return PTR_ERR(gpio->pctl_dev);
	}

	ret = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register GPIO chip\n");
		return ret;
	}

	for (i = 0; i < gpio->desc->npins; i++) {
		pin = gpio->desc->pins + i;

		ret = gpiochip_add_pin_range(&gpio->chip, dev_name(&pdev->dev),
					     pin->pin.number, pin->pin.number,
					     1);
		if (ret) {
			dev_err(&pdev->dev, "failed to add pin range\n");
			return ret;
		}
	}

	dev_info(&pdev->dev, "AXP209 pinctrl and GPIO driver loaded\n");

	return 0;
}

static const struct of_device_id axp20x_gpio_match[] = {
	{ .compatible = "x-powers,axp209-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, axp20x_gpio_match);

static struct platform_driver axp20x_gpio_driver = {
	.probe		= axp20x_gpio_probe,
	.driver = {
		.name		= "axp20x-gpio",
		.of_match_table	= axp20x_gpio_match,
	},
};

module_platform_driver(axp20x_gpio_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_DESCRIPTION("AXP20x PMIC pinctrl and GPIO driver");
MODULE_LICENSE("GPL");
