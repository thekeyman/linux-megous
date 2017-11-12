#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DRIVER_NAME "tbs_a711"
#define A711_IOCTL_RESET _IO('A', 0)

struct a711_dev {
	struct device *dev;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *wakeup_gpio;
	struct regulator *regulator;
	int wakeup_irq;
	u32 duration_us;

	wait_queue_head_t waitqueue;
	struct delayed_work work;
	struct list_head news;
	spinlock_t lock;
	int got_wakeup;

	struct cdev cdev;
	dev_t major;
};

struct a711_fp {
	struct a711_dev* a711;
};

static struct class* a711_class;

static void a711_work_handler(struct work_struct *work)
{
	struct a711_dev *a711 = container_of(work, struct a711_dev, work.work);

	// simulate wakeup events
	spin_lock(&a711->lock);
	a711->got_wakeup = 1;
	spin_unlock(&a711->lock);

	wake_up_interruptible(&a711->waitqueue);

	schedule_delayed_work(&a711->work, 1 * HZ);
}

static bool a711_has_wakeup(struct a711_dev* a711)
{
	bool got_wakeup;
	spin_lock(&a711->lock);
	got_wakeup = a711->got_wakeup;
	spin_unlock(&a711->lock);
	return got_wakeup;
}

static ssize_t a711_read(struct file *fp, char __user *buf, size_t len,
			 loff_t *off)
{
	struct a711_fp* data = fp->private_data;
	struct a711_dev* a711 = data->a711;
	int ret;
	char tmp_buf[1] = {1};
	int got_wakeup;
	int non_blocking = fp->f_flags & O_NONBLOCK;

	// first handle non-blocking path
	if (non_blocking && !a711_has_wakeup(a711))
		return -EWOULDBLOCK;

	// wait for availability of wakeup
	ret = wait_event_interruptible(a711->waitqueue,
				       a711_has_wakeup(a711));
	if (ret)
		return ret;

	spin_lock(&a711->lock);
	got_wakeup = a711->got_wakeup;
	a711->got_wakeup = 0;

	if (!got_wakeup) {
		ret = -EIO;
	} else {
		if (copy_to_user(buf, tmp_buf, 1))
			ret = -EFAULT;
		else
			ret = 1;
	}

	spin_unlock(&a711->lock);
	return ret;
}

static ssize_t a711_write(struct file *fp, const char __user *buf,
			 size_t len, loff_t *off)
{
	struct a711_fp* data = fp->private_data;
	struct a711_dev* a711 = data->a711;
	int ret;
	char tmp_buf[1];

	if (len == 0)
		return 0;

	ret = copy_from_user(tmp_buf, buf, 1);
	if (ret)
		return -EFAULT;

	if (tmp_buf[0] == '1') {
		spin_lock(&a711->lock);
		a711->got_wakeup = 0;
		spin_unlock(&a711->lock);
	}

	return 1;
}

static unsigned int a711_poll(struct file *fp, poll_table *wait)
{
	struct a711_fp* data = fp->private_data;
	struct a711_dev* a711 = data->a711;
	int ret = 0;

	poll_wait(fp, &a711->waitqueue, wait);

	spin_lock(&a711->lock);
	if (a711->got_wakeup)
		ret |= POLLIN | POLLRDNORM;
	spin_unlock(&a711->lock);

	return ret;
}

static long a711_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct a711_fp* data = fp->private_data;
	struct a711_dev* a711 = data->a711;
	unsigned long flags;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	switch (cmd) {
	case A711_IOCTL_RESET:
		spin_lock_irqsave(&a711->lock, flags);
		dev_info(a711->dev, "reset\n");
		spin_unlock_irqrestore(&a711->lock, flags);
		return 0;
	}

	return -ENOSYS;
}

static int a711_release(struct inode *ip, struct file *fp)
{
	//struct a711_dev* a711 = container_of(ip->i_cdev, struct a711_dev, cdev);
	struct a711_fp* data = fp->private_data;

	kfree(data);
	return 0;
}

static int a711_open(struct inode *ip, struct file *fp)
{
	struct a711_dev* a711 = container_of(ip->i_cdev, struct a711_dev, cdev);
	struct a711_fp* data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->a711 = a711;
	fp->private_data = data;

	nonseekable_open(ip, fp);
	return 0;
}

static const struct file_operations a711_fops = {
	.owner		= THIS_MODULE,
	.read		= a711_read,
	.write		= a711_write,
	.poll		= a711_poll,
	.unlocked_ioctl	= a711_ioctl,
	.open		= a711_open,
	.release	= a711_release,
	.llseek		= noop_llseek,
};

static irqreturn_t a711_wakeup_isr(int irq, void *dev_id)
{
	struct a711_dev *a711 = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&a711->lock, flags);
	a711->got_wakeup = 1;
	spin_unlock_irqrestore(&a711->lock, flags);

	wake_up_interruptible(&a711->waitqueue);

	return IRQ_HANDLED;
}

static int a711_probe(struct platform_device *pdev)
{
	struct a711_dev *a711;
	struct device *dev = &pdev->dev;
	struct device *sdev;
	int ret;

	a711 = devm_kzalloc(dev, sizeof(*a711), GFP_KERNEL);
	if (!a711)
		return -ENOMEM;

	a711->dev = dev;
	platform_set_drvdata(pdev, a711);
	init_waitqueue_head(&a711->waitqueue);
	spin_lock_init(&a711->lock);
	INIT_LIST_HEAD(&a711->news);
	INIT_DELAYED_WORK(&a711->work, &a711_work_handler);

	//list_del(&port->list);
	//list_add_tail(&portdev->list, &pdrvdata.portdevs);

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

	a711->wakeup_irq = gpiod_to_irq(a711->wakeup_gpio);
	if (a711->wakeup_irq > 0) {
		ret = devm_request_irq(dev, a711->wakeup_irq,
				       a711_wakeup_isr,
				       IRQF_TRIGGER_RISING |
				       IRQF_TRIGGER_FALLING,
				       "a711-wakeup", a711);
		if (ret) {
			dev_err(dev, "error requesting wakeup-irq: %d\n", ret);
			return ret;
		}
	}

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

	// create char device
	ret = alloc_chrdev_region(&a711->major, 0, 1, "a711");
	if (ret) {
		dev_err(dev, "can't allocate chrdev region");
		goto err_disable_regulator;
	}

	cdev_init(&a711->cdev, &a711_fops);
	a711->cdev.owner = THIS_MODULE;
	ret = cdev_add(&a711->cdev, a711->major, 1);
	if (ret) {
		dev_err(dev, "can't add cdev");
		goto err_unreg_chrev_region;
	}

	sdev = device_create(a711_class, dev, a711->major, a711, "a711");
	if (IS_ERR(sdev)) {
		ret = PTR_ERR(sdev);
		goto err_del_cdev;
	}

	dev_info(dev, "initialized TBS A711 platform driver\n");

	schedule_delayed_work(&a711->work, 1 * HZ);

	return 0;

err_del_cdev:
	cdev_del(&a711->cdev);
err_unreg_chrev_region:
	unregister_chrdev(a711->major, "a711");
err_disable_regulator:
	gpiod_set_value(a711->enable_gpio, 0);
	gpiod_set_value(a711->reset_gpio, 0);
	regulator_disable(a711->regulator);
	cancel_delayed_work(&a711->work);
	return ret;
}

static int a711_remove(struct platform_device *pdev)
{
	struct a711_dev *a711 = platform_get_drvdata(pdev);

	cancel_delayed_work(&a711->work);

	device_destroy(a711_class, a711->major);
	cdev_del(&a711->cdev);
	unregister_chrdev(a711->major, "a711");

	if (a711->wakeup_irq > 0)
		devm_free_irq(a711->dev, a711->wakeup_irq, a711);
	regulator_disable(a711->regulator);
	gpiod_set_value(a711->enable_gpio, 0);
	gpiod_set_value(a711->reset_gpio, 0);

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

static int __init a711_driver_init(void)
{
	int ret;

	a711_class = class_create(THIS_MODULE, "a711");

	ret = platform_driver_register(&a711_platform_driver);
	if (ret)
		class_destroy(a711_class);

	return ret;
}

static void __exit a711_driver_exit(void)
{
	platform_driver_unregister(&a711_platform_driver);
	class_destroy(a711_class);
}

module_init(a711_driver_init);
module_exit(a711_driver_exit);

MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("TBS A711 Tablet Platform Driver");
MODULE_AUTHOR("Ondrej Jirman <megous@megous.com>");
MODULE_LICENSE("GPL v2");
