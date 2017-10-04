/*
 * a83t-test.c - Test driver for CSI/Sensors on Allwinner A83T
 *
 * Copyright (C) 2016  Ondřej Jirman <megous@megous.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <asm/io.h>

struct a83t_test {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct regmap *regmap;
	int chrdev_major;
	struct regulator_bulk_data supplies[4];
	struct gpio_desc *nrst_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *nrst2_gpio;
	struct gpio_desc *pwdn2_gpio;
	struct clk *mclk;
	struct clk *sclk;
	struct clk *dram_clk;
	struct clk *bus_clk;
	struct reset_control *reset;
};

static const char* const hm5065_supply_name[] = {
	"IOVDD", /* Digital I/O (2.8V) suppply */
	"AFVDD",  /* Autofocus (2.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.8V) supply */
};

static const struct regmap_config a83t_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.can_multi_write = true,
	.reg_format_endian = REGMAP_ENDIAN_BIG
};

static int hm5065_write_regs(struct a83t_test *test, u16 start_index, u8 *data, int data_size)
{
	struct i2c_client *client = test->i2c_client;
	struct i2c_msg msg;
	u8 buf[data_size + 2];
	int ret;

	buf[0] = start_index >> 8;
	buf[1] = start_index & 0xff;
	memcpy(buf + 2, data, data_size);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = data_size + 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(test->dev, "%s: error %d: start_index=%x, data=%*ph\n",
			__func__, ret, (u32)start_index, data_size, data);
		return ret;
	}

	return 0;
}

static int hm5065_read_regs(struct a83t_test *test, u16 start_index, u8 *data, int data_size)
{
	struct i2c_client *client = test->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = start_index >> 8;
	buf[1] = start_index & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = data_size;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(test->dev, "%s: error %d: start_index=%x, data_size=%d\n",
			__func__, ret, (u32)start_index, data_size);
		return ret;
	}

	return 0;
}

static int hm5065_read_reg8(struct a83t_test *test, u16 reg, u8 *val)
{
	return hm5065_read_regs(test, reg, val, 1);
}

static int hm5065_write_reg8(struct a83t_test *test, u16 reg, u8 val)
{
	return hm5065_write_regs(test, reg, &val, 1);
}

static int hm5065_read_reg16(struct a83t_test *test, u16 reg, u16 *val)
{
	u8 buf[2];
	int ret;

	ret = hm5065_read_regs(test, reg, buf, sizeof(buf));
	if (ret)
		return ret;

	*val = ((u16)buf[0] << 8) | (u16)buf[1];
	return 0;
}

static int hm5065_write_reg16(struct a83t_test *test, u16 reg, u16 val)
{
	u8 buf[2];

	buf[0] = val >> 8;
	buf[1] = val;

	return hm5065_write_regs(test, reg, buf, sizeof(buf));
}

ssize_t a83t_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct a83t_test* test = dev_get_drvdata(dev);
	ssize_t ret;
	u16 chip_id = 0;

	if (!strcmp(attr->attr.name, "chipid")) {
		ret = hm5065_read_reg16(test, 0, &chip_id);
		if (ret) {
			dev_err(dev, "Failed to read chip id: %d\n", ret);
			return ret;
		}

		dev_info(dev, "Chip id: 0x%04x\n", (unsigned int)chip_id);

		return scnprintf(buf, PAGE_SIZE, "0x%04x", (unsigned int)chip_id);
	}

	return -ENOSYS;
}

ssize_t a83t_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct a83t_test* test = dev_get_drvdata(dev);
	int ret = 0;

	if (!strcmp(attr->attr.name, "cmd")) {
		if (sysfs_streq(buf, "regs-on")) {
			ret = regulator_bulk_enable(ARRAY_SIZE(hm5065_supply_name), test->supplies);
		} else if (sysfs_streq(buf, "regs-off")) {
			ret = regulator_bulk_disable(ARRAY_SIZE(hm5065_supply_name), test->supplies);
		} else if (sysfs_streq(buf, "nrst-1")) {
			gpiod_set_value(test->nrst_gpio, 1);
		} else if (sysfs_streq(buf, "nrst-0")) {
			gpiod_set_value(test->nrst_gpio, 0);
		} else if (sysfs_streq(buf, "pwdn-1")) {
			gpiod_set_value(test->pwdn_gpio, 1);
		} else if (sysfs_streq(buf, "pwdn-0")) {
			gpiod_set_value(test->pwdn_gpio, 0);
		} else if (sysfs_streq(buf, "nrst2-1")) {
			gpiod_set_value(test->nrst2_gpio, 1);
		} else if (sysfs_streq(buf, "nrst2-0")) {
			gpiod_set_value(test->nrst2_gpio, 0);
		} else if (sysfs_streq(buf, "pwdn2-1")) {
			gpiod_set_value(test->pwdn2_gpio, 1);
		} else if (sysfs_streq(buf, "pwdn2-0")) {
			gpiod_set_value(test->pwdn2_gpio, 0);
		} else if (sysfs_streq(buf, "rstbus-on")) {
			reset_control_assert(test->reset);
		} else if (sysfs_streq(buf, "rstbus-off")) {
			reset_control_deassert(test->reset);
		} else if (sysfs_streq(buf, "mclk-24mhz")) {
			ret = clk_set_rate(test->mclk, 24000000);
		} else if (sysfs_streq(buf, "mclk-12mhz")) {
			ret = clk_set_rate(test->mclk, 12000000);
		} else if (sysfs_streq(buf, "mclk-6mhz")) {
			ret = clk_set_rate(test->mclk, 6000000);
		} else if (sysfs_streq(buf, "mclk-on")) {
			ret = clk_enable(test->mclk);
		} else if (sysfs_streq(buf, "mclk-off")) {
			clk_disable(test->mclk);
		} else if (sysfs_streq(buf, "sclk-on")) {
			ret = clk_enable(test->sclk);
		} else if (sysfs_streq(buf, "sclk-off")) {
			clk_disable(test->sclk);
		} else if (sysfs_streq(buf, "busclk-on")) {
			ret = clk_enable(test->bus_clk);
		} else if (sysfs_streq(buf, "busclk-off")) {
			clk_disable(test->bus_clk);
		} else if (sysfs_streq(buf, "dramclk-on")) {
			ret = clk_enable(test->dram_clk);
		} else if (sysfs_streq(buf, "dramclk-off")) {
			clk_disable(test->dram_clk);
		} else if (sysfs_streq(buf, "csi-on")) {
			void __iomem *io = ioremap(0x01CB0000, SZ_4K);
			writel(BIT(0) | BIT(30), io + 0x0000);
			u32 ver = readl(io + 0x003C);
			iounmap(io);

			dev_info(dev, "Version: 0x%08x\n", ver);
		} else {
			ret = -ENOSYS;
		}
	}

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(cmd, S_IWUSR | S_IRUGO, a83t_show, a83t_store);
static DEVICE_ATTR(chipid, S_IWUSR | S_IRUGO, a83t_show, a83t_store);

/*
 * I2C driver interface functions
 */
static int a83t_test_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct a83t_test *test;
	struct device *dev = &i2c->dev;
	int ret;
	int i;
	struct clk_bulk_data clks[] = {
		{ "csi-bus" },
		{ "csi-mclk" },
		{ "csi-sclk" },
		{ "csi-dram" },
	};

	test = devm_kzalloc(dev, sizeof(struct a83t_test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;

	test->i2c_client = i2c;
	test->dev = dev;
	i2c_set_clientdata(i2c, test);

	for (i = 0; i < ARRAY_SIZE(hm5065_supply_name); i++)
		test->supplies[i].supply = hm5065_supply_name[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(hm5065_supply_name),
				 test->supplies);
	if (ret) {
		dev_err(dev, "failed to get supplies\n");
		return ret;
	}

	test->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(test->pwdn_gpio)) {
		dev_err(dev, "failed to get pwdn gpio");
		return PTR_ERR(test->pwdn_gpio);
	}

	test->nrst_gpio = devm_gpiod_get(dev, "nrst", GPIOD_OUT_LOW);
	if (IS_ERR(test->nrst_gpio)) {
		dev_err(dev, "failed to get pwdn gpio");
		return PTR_ERR(test->nrst_gpio);
	}

	test->pwdn2_gpio = devm_gpiod_get(dev, "pwdn2", GPIOD_OUT_LOW);
	if (IS_ERR(test->pwdn2_gpio)) {
		dev_err(dev, "failed to get pwdn gpio");
		return PTR_ERR(test->pwdn2_gpio);
	}

	test->nrst2_gpio = devm_gpiod_get(dev, "nrst2", GPIOD_OUT_LOW);
	if (IS_ERR(test->nrst2_gpio)) {
		dev_err(dev, "failed to get pwdn gpio");
		return PTR_ERR(test->nrst2_gpio);
	}

	test->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(test->reset)) {
		ret = PTR_ERR(test->reset);
		dev_err(dev, "failed to get reset\n");
		return ret;
	}
	reset_control_deassert(test->reset);
	
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(clks), clks);
	if (ret) {
		dev_err(dev, "failed to get clocks\n");
		return ret;
	}

	ret = clk_bulk_prepare(ARRAY_SIZE(clks), clks);
	if (ret) {
		dev_err(dev, "failed to prepare clocks\n");
		return ret;
	}

	test->bus_clk = clks[0].clk;
	test->mclk = clks[1].clk;
	test->sclk = clks[2].clk;
	test->dram_clk = clks[3].clk;

	test->regmap = devm_regmap_init_i2c(i2c, &a83t_regmap_config);
	if (IS_ERR(test->regmap)) {
		ret = PTR_ERR(test->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

        ret = device_create_file(dev, &dev_attr_cmd);
        if (ret) {
		dev_err(dev, "Failed to create sysfs attr %d\n", ret);
                return ret;
	}

        ret = device_create_file(dev, &dev_attr_chipid);
        if (ret) {
		dev_err(dev, "Failed to create sysfs attr %d\n", ret);
                return ret;
	}

	dev_err(dev, "Probed successfully\n");

	return 0;
}

static int a83t_test_remove(struct i2c_client *client)
{
	struct a83t_test *test = i2c_get_clientdata(client);

	reset_control_assert(test->reset);

	clk_unprepare(test->bus_clk);
	clk_unprepare(test->mclk);
	clk_unprepare(test->sclk);
	clk_unprepare(test->dram_clk);

	return 0;
}

static const struct of_device_id a83t_test_of_match[] = {
	{ .compatible = "allwinner,a83t-test" },
	{ },
};
MODULE_DEVICE_TABLE(of, a83t_test_of_match);

static struct i2c_driver a83t_test_driver = {
	.driver = {
		.name = "a83t-test",
		.of_match_table	= of_match_ptr(a83t_test_of_match),
	},
	.probe = a83t_test_probe,
	.remove = a83t_test_remove,
};

module_i2c_driver(a83t_test_driver);

MODULE_AUTHOR("Ondřej Jirman <megous@megous.com>");
MODULE_DESCRIPTION("Test driver for a83t hm5065 sensor/CSI/I2C interaction");
MODULE_LICENSE("GPL v2");
