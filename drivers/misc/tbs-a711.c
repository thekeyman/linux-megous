#if 0
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/slab.h>
#endif
//#include <linux/err.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
//#include <linux/slab.h>

#define DRIVER_NAME "tbs_a711"

struct a711_dev {
	struct device *dev;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *wakeup_gpio;
	struct regulator *regulator;
	u32 duration_us;

	int major;
};

static ssize_t a711_read(struct file *fp, char __user *buf,
			size_t len, loff_t *off)
{
	struct a711_dev *a711 = fp->private_data;

	//err = copy_to_user(buf, pkt, len);

	return -ENODEV;
}

static ssize_t a711_write(struct file *fp, const char __user *buf,
			 size_t len, loff_t *off)
{
	//err = copy_from_user(pkt, buf, len);
	//if (err)
		//len = 0;

	return -ENODEV;
}

static unsigned int a711_poll(struct file *fp, poll_table *wait)
{
#if 0
	struct ccb_data *data = fp->private_data;
	struct ccb *driver_ccb = &data->driver_ccb;

	poll_wait(fp, &data->ccb_waitq, wait);

	if (is_channel_reset(driver_ccb))
		return POLLERR;
	else if (a711_pkt_recv(data->a711_hw, driver_ccb))
		return POLLIN | POLLRDNORM;
#endif
	return 0;
}

static int a711_close(struct inode *ip, struct file *fp)
{
#if 0
	int slot;
	struct ccb_data *data;
	struct a711_hwinfo *hw;
	unsigned long flags;

	slot = iminor(ip) % max_ccb;
	hw = container_of(ip->i_cdev, struct a711_hwinfo, cdev);

	spin_lock(&hw->open_lock);

	if (hw->ccb_alloc[slot]->ccb_cnt == 1) {

		data = fp->private_data;

		spin_lock_irqsave(&hw->alloc_lock, flags);
		hw->ccb_alloc[slot] = NULL;
		spin_unlock_irqrestore(&hw->alloc_lock, flags);

		a711_ccb_close(hw->a711_dev, data);

		kfree(data);
	} else
		hw->ccb_alloc[slot]->ccb_cnt--;

	spin_unlock(&hw->open_lock);
#endif
	return 0;
}

static int a711_open(struct inode *ip, struct file *fp)
{
#if 0
	int slot, error;
	struct ccb_data *data;
	struct a711_hwinfo *hw;
	unsigned long flags;

	slot = iminor(ip) % max_ccb;
	hw = container_of(ip->i_cdev, struct a711_hwinfo, cdev);

	/* new ccb allocation */
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock(&hw->open_lock);

	spin_unlock(&hw->open_lock);

	return error;
#endif
	return -ENODEV;
}

static const struct file_operations a711_fops = {
	.owner		= THIS_MODULE,
	.read		= a711_read,
	.write		= a711_write,
	.poll		= a711_poll,
	.open		= a711_open,
	.release	= a711_close,
	.llseek		= noop_llseek,
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

#if 0
	a711->major = register_chrdev(0, "tbs_a711", &a711_fops);
	if (a711->major < 0) {
		dev_err(dev, "register_chrdev failed\n");
		regulator_disable(a711->regulator);
		return a711->major;
	}
#endif

	dev_info(dev, "initialized TBS A711 platform driver\n");

	return 0;
}

static int a711_remove(struct platform_device *pdev)
{
	struct a711_dev *a711 = platform_get_drvdata(pdev);

	regulator_disable(a711->regulator);
	gpiod_set_value(a711->enable_gpio, 0);
	gpiod_set_value(a711->reset_gpio, 0);
	//unregister_chrdev(a711->major, "tbs_a711");

	return 0;
}

static const struct of_device_id a711_of_match[] = {
	{ .compatible = "zte,powerup-mg3732" },
	//{ .compatible = "tbs,a711" },
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
