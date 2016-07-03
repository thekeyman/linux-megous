#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/poll.h>
#include <linux/rtc.h>
#include <linux/gpio.h>
#include <mach/sys_config.h>
#include <mach/gpio.h>
#include <asm/io.h>

#define MIC_EFFECT_MAJOR    179
#define MIC_EFFECT_MAGIC    'd'
#define MIC_EFFECT_IOCMAX   10
#define MIC_EFFECT_NAME     "mic_sound_effect"

#define MIC_EFFECT_SET      _IOW(MIC_EFFECT_MAGIC, 1, unsigned long)
#define MIC_EFFECT_GET      _IOW(MIC_EFFECT_MAGIC, 2, unsigned long)

/* params for mode */
#define MIC_EFFECT_LYP      1
#define MIC_EFFECT_KTV      2
#define MIC_EFFECT_YCH      3

script_item_u io1_item;
script_item_u pt1_item;
script_item_u pt3_item;

struct mic_effect_data{
   int mode;
   int param;
};

static struct mic_effect_data bwr;

static long mic_effect_dev_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
    struct mic_effect_data bwr0;
     void __user *ubuf = NULL;
    if (_IOC_TYPE(cmd) != MIC_EFFECT_MAGIC
            || _IOC_NR(cmd) > MIC_EFFECT_IOCMAX)
        return -ENOTTY;

    ubuf = (void __user *)arg;
    switch (cmd){
        case MIC_EFFECT_SET:
            if (copy_from_user(&bwr, ubuf, sizeof(bwr))) {
                goto error;
            }
           pr_err("%s: mode = %d, param = %d\n", __func__, bwr.mode, bwr.param);
          if (bwr.mode != 1)
              goto error;
             
            switch (bwr.param)
            {
                case MIC_EFFECT_LYP:
                    printk(KERN_ERR "##MIC_EFFECT_LYP");
                    gpio_set_value(pt1_item.gpio.gpio, 0);
                    gpio_set_value(pt3_item.gpio.gpio, 0);
                    return 0;
                case MIC_EFFECT_KTV:
                    printk(KERN_ERR "##MIC_EFFECT_KTV");
                    gpio_set_value(pt1_item.gpio.gpio, 1);
                    gpio_set_value(pt3_item.gpio.gpio, 0);
                    return 0;
                case MIC_EFFECT_YCH:
                    printk(KERN_ERR "##MIC_EFFECT_YCH");
                    gpio_set_value(pt1_item.gpio.gpio, 1);
                    gpio_set_value(pt3_item.gpio.gpio, 1);
                    return 0;
                default:
                    goto error;
            }
        case MIC_EFFECT_GET:
            if (copy_from_user(&bwr0, ubuf, sizeof(bwr0))) {
                goto error;
            }
            if (bwr0.mode != 1)
                goto error;
            copy_to_user(ubuf, &bwr, sizeof(bwr));
            return 0;
        default: 
            goto error;
    }

error:
    return -EFAULT;
}

int mic_gpio_init(void)
{
    script_item_value_type_e  type;
    int ret;

    type = script_get_item("audio_echo", "io_1", &io1_item);
    if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
        pr_err("[audio_echo] io_1 type err!\n");
    }
    gpio_request(io1_item.gpio.gpio, "IO_1");
    gpio_direction_output(io1_item.gpio.gpio, 1);
    gpio_set_value(io1_item.gpio.gpio, 1);
    type = script_get_item("audio_echo", "pt_vco1", &pt1_item);
    if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
        pr_err("[audio_echo] pt_vco1 type err!\n");
    }
    ret = gpio_request(pt1_item.gpio.gpio, "PT_VCO1");
    if (ret < 0) {
    	pr_err("err:%s,l:%d, pt1_item.gpio.gpio:%d\n", __func__, __LINE__, pt1_item.gpio.gpio);
    }
    gpio_direction_output(pt1_item.gpio.gpio, 1);
    gpio_set_value(pt1_item.gpio.gpio, 0);
    type = script_get_item("audio_echo", "pt_vco3", &pt3_item);
    if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
        pr_err("[audio_echo] pt_vco1 type err!\n");
    }
    gpio_request(pt3_item.gpio.gpio, "PT_VCO3");
    gpio_direction_output(pt3_item.gpio.gpio, 1);
    gpio_set_value(pt3_item.gpio.gpio, 0);
    return 0;
}

int mic_effect_open(struct inode *fi, struct file *fp)
{
    return 0;
}

static int mic_effect_read(struct file *filp, 
        char __user *buf, size_t length, loff_t *offset)
{
    return 0;
}

static const struct file_operations mic_effect_fops = {
    .open           = mic_effect_open,
    .read           = mic_effect_read,
    .unlocked_ioctl = mic_effect_dev_ioctl,
};

static int __init mic_effect_init(void)
{
    struct class *cls;
    int ret;
    pr_err(KERN_INFO "####     %s\n", __func__);

    ret = mic_gpio_init();

	/* create mic effect dev node */
    ret = register_chrdev(MIC_EFFECT_MAJOR, MIC_EFFECT_NAME, &mic_effect_fops);
    if (ret < 0)
    {
        pr_err(KERN_ERR "Register char device for mic effect failed.\n");
        return -EFAULT;
    }

    cls = class_create(THIS_MODULE, MIC_EFFECT_NAME);
    if (!cls)
    {
        pr_err(KERN_ERR "Can not register class for mic effect .\n");
        return -EFAULT;
    }

	/* create device node */
    device_create(cls, NULL, MKDEV(MIC_EFFECT_MAJOR, 0), NULL, MIC_EFFECT_NAME);

    return 0;
}

static void __exit mic_effect_exit(void) {}

module_init(mic_effect_init);
module_exit(mic_effect_exit);
