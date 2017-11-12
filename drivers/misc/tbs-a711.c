#include <linux/module.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#define DRIVER_NAME "tbs_a711"

struct a711_dev {
	struct device *dev;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *wakeup_gpio;
	struct regulator *regulator;
};

static int a711_probe(struct platform_device *pdev)
{
	struct a711_dev *a711;
	struct device *dev = &pdev->dev;
	int ret;

	a711 = devm_kzalloc(dev, sizeof(*a711), GFP_KERNEL);
	if (!a711)
		return -ENOMEM;

	a711->dev = dev;
	platform_set_drvdata(pdev, a711);

	a711->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(a711->enable_gpio)) {
		dev_err(dev, "can't get enable gpio err=%ld",
			PTR_ERR(a711->enable_gpio));
		if (PTR_ERR(a711->enable_gpio) == -ENOENT)
			return -EPROBE_DEFER;
		return PTR_ERR(a711->enable_gpio);
	}

	a711->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(a711->reset_gpio)) {
		dev_err(dev, "can't get reset gpio err=%ld",
			PTR_ERR(a711->reset_gpio));
		if (PTR_ERR(a711->enable_gpio) == -ENOENT)
			return -EPROBE_DEFER;
		return PTR_ERR(a711->reset_gpio);
	}

	a711->wakeup_gpio = devm_gpiod_get(dev, "wakeup", GPIOD_IN);
	if (IS_ERR(a711->wakeup_gpio)) {
		dev_err(dev, "can't get wakeup gpio err=%ld",
			PTR_ERR(a711->wakeup_gpio));
		if (PTR_ERR(a711->enable_gpio) == -ENOENT)
			return -EPROBE_DEFER;
		return PTR_ERR(a711->wakeup_gpio);
	}
	//XXX: create interrupt from this

	a711->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(a711->regulator)) {
		dev_err(dev, "can't get power supply err=%ld",
			PTR_ERR(a711->regulator));
		if (PTR_ERR(a711->enable_gpio) == -ENOENT)
			return -EPROBE_DEFER;
		return PTR_ERR(a711->regulator);
	}

	// power up
	ret = regulator_enable(a711->regulator);
	if (ret < 0) {
		dev_err(dev, "can't enable power supply err=%d", ret);
		return ret;
	}

	gpiod_set_value(a711->enable_gpio, 1);

	usleep_range(3000, 4000);
	gpiod_set_value(a711->reset_gpio, 1);
	usleep_range(3000, 4000);
	gpiod_set_value(a711->reset_gpio, 0);

	return 0;
}

static int a711_remove(struct platform_device *pdev)
{
	struct a711_dev *a711 = platform_get_drvdata(pdev);

	regulator_disable(a711->regulator);
	gpiod_set_value(a711->enable_gpio, 0);
	gpiod_set_value(a711->reset_gpio, 0);

	return 0;
}

static const struct of_device_id a711_of_match[] = {
	{ .compatible = "zte,powerup-mg3732" },
	{},
};
MODULE_DEVICE_TABLE(of, a711_of_match);

static struct platform_driver a711_platform_driver = {
	.probe = a711_probe,
	.remove = a711_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = a711_of_match,
	},
};

module_platform_driver(a711_platform_driver);

MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("TBS A711 Tablet Platform Driver");
MODULE_AUTHOR("Ondrej Jirman <megous@megous.com>");
MODULE_LICENSE("GPL v2");
