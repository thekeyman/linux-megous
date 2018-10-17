#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/proc_fs.h>

#include <linux/hardware_info.h>

static char hardwareinfo_name[HARDWARE_MAX_ITEM][HARDWARE_MAX_ITEM_LONGTH];

int hardwareinfo_set_prop(int cmd, const char *name)
{
	if(cmd < 0 || cmd >= HARDWARE_MAX_ITEM)
		return -1;

	if(name != NULL && strlen(name) != 0 && strlen(name) < HARDWARE_MAX_ITEM_LONGTH)
	{
		sprintf(hardwareinfo_name[cmd], "%s", name);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(hardwareinfo_set_prop);

void appinfo_convert_hardwareinfo(const char *name, const char *value)
{
	printk("appinfo_convert_hardwareinfo, name = %s\n", name);
	if(!strcmp(name, "lcd type"))
	{
		hardwareinfo_set_prop(HARDWARE_LCD, value);
	}
	else if(!strcmp(name, "touch_panel"))
	{
		hardwareinfo_set_prop(HARDWARE_TP, value);
	}
	else if(!strcmp(name, "camera_main"))
	{
		hardwareinfo_set_prop(HARDWARE_BACK_CAM, value);
		if(strstr(value, "s5k3l8"))
		{
			hardwareinfo_set_prop(HARDWARE_BACK_CAM_MOUDULE_ID, "Ofilm");
		}
		else if(strstr(value, "ov13853"))
		{
			hardwareinfo_set_prop(HARDWARE_BACK_CAM_MOUDULE_ID, "Sunny");
		}
	}
	else if(!strcmp(name, "camera_slave"))
	{
		hardwareinfo_set_prop(HARDWARE_FRONT_CAM, value);
		if(strstr(value, "gc5005"))
		{
			hardwareinfo_set_prop(HARDWARE_FRONT_CAM_MOUDULE_ID, "Ofilm");
		}
		else if(strstr(value, "s5k5e2"))
		{
			hardwareinfo_set_prop(HARDWARE_FRONT_CAM_MOUDULE_ID, "Kingcome");
		}
	}
	else if(!strcmp(name, "G-Sensor"))
	{
		hardwareinfo_set_prop(HARDWARE_ACCELEROMETER, value);
	}
	else if(!strcmp(name, "M-Sensor"))
	{
		hardwareinfo_set_prop(HARDWARE_MAGNETOMETER, value);
	}
	else if(!strcmp(name, "LP-Sensor"))
	{
		hardwareinfo_set_prop(HARDWARE_ALSPS, value);
	}
	else if(!strcmp(name, "Gyro-Sensor"))
	{
		hardwareinfo_set_prop(HARDWARE_GYROSCOPE, value);
	}
	else if(!strcmp(name, "battery_id"))
	{
		hardwareinfo_set_prop(HARDWARE_BATTERY_ID, value);
	}
	else if(!strcmp(name, "fingerprint"))
	{
		hardwareinfo_set_prop(HARDWARE_FINGERPRINT, value);
	}
}

EXPORT_SYMBOL_GPL(appinfo_convert_hardwareinfo);

static char boardid[HARDWARE_MAX_ITEM_LONGTH];
static int __init early_parse_boardid(char *cmdline)
{
	if (cmdline) {
		strlcpy(boardid, cmdline, sizeof(boardid));
	}
	return 0;
}
early_param("androidboot.board_id", early_parse_boardid);

static char flashDev[HARDWARE_MAX_ITEM_LONGTH];
static int __init early_parse_flashDev(char *cmdline)
{
	if (cmdline) {
		strlcpy(flashDev, cmdline, sizeof(flashDev));
	}

	return 0;
}
early_param("androidboot.hardware.emmc", early_parse_flashDev);

static long hardwareinfo_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int ret = 0, hardwareinfo_num;
	void __user *data = (void __user *)arg;
	//char info[HARDWARE_MAX_ITEM_LONGTH];

	switch (cmd) {
	case HARDWARE_LCD_GET:
		hardwareinfo_num = HARDWARE_LCD;
		break;
	case HARDWARE_TP_GET:
		hardwareinfo_num = HARDWARE_TP;
		break;
	case HARDWARE_FLASH_GET:
		hardwareinfo_set_prop(HARDWARE_FLASH, flashDev);
		hardwareinfo_num = HARDWARE_FLASH;
		break;
	case HARDWARE_FRONT_CAM_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM;
		break;
	case HARDWARE_BACK_CAM_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM;
		break;
	case HARDWARE_BT_GET:
		hardwareinfo_set_prop(HARDWARE_BT, "Qualcomm");
		hardwareinfo_num = HARDWARE_BT;
		break;
	case HARDWARE_WIFI_GET:
		hardwareinfo_set_prop(HARDWARE_WIFI, "Qualcomm");
		hardwareinfo_num = HARDWARE_WIFI;
		break;
	case HARDWARE_ACCELEROMETER_GET:
		hardwareinfo_num = HARDWARE_ACCELEROMETER;
		break;
	case HARDWARE_ALSPS_GET:
		hardwareinfo_num = HARDWARE_ALSPS;
		break;
	case HARDWARE_GYROSCOPE_GET:
		hardwareinfo_num = HARDWARE_GYROSCOPE;
		break;
	case HARDWARE_MAGNETOMETER_GET:
		hardwareinfo_num = HARDWARE_MAGNETOMETER;
		break;
	case HARDWARE_GPS_GET:
		hardwareinfo_set_prop(HARDWARE_GPS, "Qualcomm");
	    hardwareinfo_num = HARDWARE_GPS;
		break;
	case HARDWARE_FM_GET:
		hardwareinfo_set_prop(HARDWARE_FM, "Qualcomm");
	    hardwareinfo_num = HARDWARE_FM;
		break;
	case HARDWARE_BATTERY_ID_GET:
		hardwareinfo_num = HARDWARE_BATTERY_ID;
		break;
	case HARDWARE_BACK_CAM_MOUDULE_ID_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM_MOUDULE_ID;
		break;
	case HARDWARE_FRONT_CAM_MODULE_ID_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM_MOUDULE_ID;
		break;
	case HARDWARE_BOARD_ID_GET:
		hardwareinfo_set_prop(HARDWARE_BOARD_ID, boardid);
		hardwareinfo_num = HARDWARE_BOARD_ID;
		break;
	case HARDWARE_BACK_CAM_MOUDULE_ID_SET:
		if(copy_from_user(hardwareinfo_name[HARDWARE_BACK_CAM_MOUDULE_ID], data, HARDWARE_MAX_ITEM_LONGTH))
		{
			pr_err("wgz copy_from_user error");
			ret =  -EINVAL;
		}
		goto set_ok;
		break;
	case HARDWARE_FRONT_CAM_MODULE_ID_SET:
		if(copy_from_user(hardwareinfo_name[HARDWARE_FRONT_CAM_MOUDULE_ID], data, HARDWARE_MAX_ITEM_LONGTH))
		{
			pr_err("wgz copy_from_user error");
			ret =  -EINVAL;
		}
		goto set_ok;
		break;
	case HARDWARE_FINGERPRINT_ID_GET:
		hardwareinfo_num = HARDWARE_FINGERPRINT;
		break;
	default:
		ret = -EINVAL;
		goto err_out;
	}
	if (copy_to_user(data, hardwareinfo_name[hardwareinfo_num], HARDWARE_MAX_ITEM_LONGTH)){
		ret =  -EINVAL;
	}
set_ok:
err_out:
	return ret;
}

static struct file_operations hardwareinfo_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.unlocked_ioctl = hardwareinfo_ioctl,
	.compat_ioctl = hardwareinfo_ioctl,
};

static struct miscdevice hardwareinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hardwareinfo",
	.fops = &hardwareinfo_fops,
};


static int __init hardwareinfo_init_module(void)
{
	int ret, i;

	for(i = 0; i < HARDWARE_MAX_ITEM; i++)
	{
		if(strlen(hardwareinfo_name[i]) == 0)
			sprintf(hardwareinfo_name[i], "NULL");
	}

	ret = misc_register(&hardwareinfo_device);
	if(ret < 0){
		pr_err("%s, misc_register error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static void __exit hardwareinfo_exit_module(void)
{
	misc_deregister(&hardwareinfo_device);
}

module_init(hardwareinfo_init_module);
module_exit(hardwareinfo_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ming He <heming@wingtech.com>");


