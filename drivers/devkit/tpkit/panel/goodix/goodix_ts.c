/*
 * goodix_ts.c - Main touch driver file of Goodix
 *
 * Copyright (C) 2015 - 2016 Goodix Technology Incorporated   
 * Copyright (C) 2015 - 2016 Yulong Cai <caiyulong@goodix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 */
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "goodix_ts.h"
#include "goodix_dts.h"
#include <linux/hardware_info.h>
#include "../../huawei_ts_kit.h"
#include <huawei_platform/log/log_jank.h>
#if defined(CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
#endif
#include <misc/app_info.h>

#ifdef   ROI
#undef  ROI
#endif

/*Gesture register(0xd0) value*/
#define DOUBLE_CLICK_WAKEUP  		(0xcc)
#define SPECIFIC_LETTER_W  			('w')
#define SPECIFIC_LETTER_M			('m')
#define SPECIFIC_LETTER_E			('e')
#define SPECIFIC_LETTER_C 			('c')
#define LETTER_LOCUS_NUM 6
#define LINEAR_LOCUS_NUM 2
#define IS_APP_ENABLE_GESTURE(x)  ((u32)(1<<x))

struct goodix_ts_data 		*goodix_ts; 
struct ts_kit_device_data 	*g_goodix_dev_data = NULL;
static struct mutex wrong_touch_lock;
static int goodix_get_ic_firmware_version(void);
static int goodix_hardwareinfo_set(void);
DECLARE_WORK(goodix_chip_sleep_mode_work,goodix_sleep_mode_out);
DECLARE_WORK(goodix_chip_put_device_work,goodix_put_device_outof_easy_wakeup);


/**
 * goodix_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 * Return: 0: success, otherwise: failed
 */
int goodix_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct ts_bus_info *bops = g_goodix_dev_data->ts_platform_data->bops;
	u8 stack_mem[32], *data;
	int ret;

	if (unlikely(!bops || !bops->bus_write))
		return -ENODEV;

	if (len + 2 > sizeof(stack_mem)) {
		data = kmalloc(len + 2, GFP_KERNEL);
		if (!data) {
			TS_LOG_ERR("No memory");
			return -ENOMEM;
		}
	} else {
		data = &stack_mem[0];
	}

	data[0] = addr >> 8 & 0xff;
	data[1] = addr & 0xff;
	memcpy(&data[2], buffer, len);

	ret = bops->bus_write(data, len + 2);
	if (ret < 0)
		TS_LOG_ERR("i2c write error,addr:%04x bytes:%d", addr, len);

	if (data != &stack_mem[0])
		kfree(data);

	return ret;
}

/**
 * goodix_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 * Return: 0: success, otherwise: failed
 */
int goodix_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	struct ts_bus_info *bops = g_goodix_dev_data->ts_platform_data->bops;
	int ret = 0;
	
	if (unlikely(!bops || !bops->bus_write))
		return -ENODEV;
	
	addr = cpu_to_be16(addr);

	ret = bops->bus_read((u8 *)&addr, 2, buffer, len);
	if (ret < 0)
		TS_LOG_ERR("i2c read error,addr:%04x bytes:%d", addr, len);

	return ret;
}

/**
 * goodix_i2c_read_dbl_check - read twice and double check
 * @addr: register address
 * @buffer: data buffer
 * @len: bytes to read
 * Return    <0: i2c error, 0: ok, 1:fail
 */
int goodix_i2c_read_dbl_check(u16 addr, u8 * buffer, s32 len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	int ret;

	if (len > 16) {
		TS_LOG_ERR("i2c_read_dbl_check length %d is too long, exceed %zu",
			len, sizeof(buf));
		return -EINVAL;
	}

	memset(buf, 0xAA, sizeof(buf));
	ret = goodix_i2c_read(addr, buf, len);
	if (ret < 0)
		return ret;

	msleep(5);
	memset(confirm_buf, 0, sizeof(confirm_buf));
	ret = goodix_i2c_read(addr, confirm_buf, len);
	if (ret < 0)
		return ret;

	if (!memcmp(buf, confirm_buf, len)) {
		memcpy(buffer, confirm_buf, len);
		return 0;
	}

	TS_LOG_ERR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
	return 1;
}

/**
 * goodix_send_cfg - Send config data to hw
 * @cfg_ptr: pointer to config data structure
 * Return 0--success,non-0--fail.
 */
int goodix_send_cfg(struct goodix_ts_config *cfg_ptr)
{
	static DEFINE_MUTEX(mutex_cfg);
	u8 *config;
	int i, cfg_len;
	s32 ret = 0, retry = 0;
	u8 checksum = 0;
	TS_LOG_INFO("Goodix Send Cfg");
	if (!cfg_ptr || !cfg_ptr->initialized) {
		TS_LOG_ERR("Invalid config data");
		return -EINVAL;
	}

	config = &cfg_ptr->data[0];
	cfg_len = cfg_ptr->size;

	mutex_lock(&mutex_cfg);
	TS_LOG_INFO("Send %s,ver:%02x size:%d", cfg_ptr->name, config[0], cfg_len);
	TS_LOG_INFO("write array:");                
	GTP_DEBUG_ARRAY(config, cfg_len);

	if (cfg_len != GTP_CONFIG_ORG_LENGTH) {
		TS_LOG_ERR("Invalid config size:%d", cfg_len);
		mutex_unlock(&mutex_cfg);
		return -1;
	}

	for (i = 0, checksum = 0; i < cfg_len - 2; i++)
		checksum += config[i];
	
	if (!checksum) {
		TS_LOG_ERR("Invalid config,all of the bytes is zero");
		mutex_unlock(&mutex_cfg);
		return -1;
	}

	checksum = 0 - checksum;

	config[cfg_len - 2] = checksum & 0xFF;
	config[cfg_len - 1] = 0x01;
	retry = 0;
	while (retry++ < 3) {
		ret = goodix_i2c_write(GTP_REG_CONFIG_DATA, config, cfg_len);
		if (!ret) {
			if (cfg_ptr->delay_ms > 0)
				msleep(cfg_ptr->delay_ms);
			mutex_unlock(&mutex_cfg);
			TS_LOG_INFO("Send config successfully");
			return 0;
		}
	}

	TS_LOG_ERR("Send config failed");
	mutex_unlock(&mutex_cfg);
	return ret;
}

/**
 * goodix_send_cmd - seng cmd
 * must write data & checksum first
 * byte    content
 * 0       cmd
 * 1       data
 * 2       checksum
 * Returns 0 - succeed,non-0 - failed
 */
static int goodix_send_cmd(u16 addr, u8 cmd)
{
	s32 ret;
	static DEFINE_MUTEX(cmd_mutex);
	u8 buffer[3] = { cmd, 0, 0 };

	TS_LOG_DEBUG("Send command:%u", cmd);
	mutex_lock(&cmd_mutex);
	buffer[0] = cmd;
	ret = goodix_i2c_write(addr, &buffer[0], 1);
	mutex_unlock(&cmd_mutex);

	return ret;
}

/**
 * goodix_init_watchdog - esd mechannism
 * 
 * Returns  0--success,non-0--fail.
 */
static int goodix_init_watchdog(void)
{
	//u8 value[] = {0xAA, 0x00, 0x56, 0xAA};

	//TS_LOG_ERR("Init watchdog");
	//return goodix_i2c_write(GTP_REG_CMD, &value[0], 4);
	u8 opr_buffer[1] = {0xAA};		
	GTP_DEBUG("Init external watchdog");		
	return goodix_i2c_write(0x8041, opr_buffer, 1);
}

/**
 * goodix_switch_wrokmode - Switch working mode.
 * @workmode: GTP_CMD_SLEEP - Sleep mode
 *			  GESTURE_MODE - gesture mode
 * Returns  0--success,non-0--fail.
 */
static int goodix_switch_wrokmode(int addr, int wrokmode)
{
	s32 retry = 0;
	u8 cmd;

	switch (wrokmode) {
	case SLEEP_MODE:
		cmd = GTP_CMD_SLEEP;
		break;
	case GESTURE_MODE:
		cmd = GTP_CMD_GESTURE_WAKEUP;
		break;
	default:
		return -EINVAL;
	}

	TS_LOG_INFO("Switch working mode[%02X]", cmd);
	while (retry++ < 3) {
		if (!goodix_send_cmd(addr, cmd))
			return 0;
		msleep(20);
	}

	TS_LOG_ERR("Failed to switch working mode");
	return -1;
}

static int goodix_feature_switch(struct goodix_ts_data *ts,
	enum goodix_ts_feature fea, int on)
{
	int ret = 0;

#if 0
	struct ts_feature_info *info = &g_ts_data.feature_info;
	struct goodix_ts_config *config = NULL;

	if (!ts || !info)
		return -EINVAL;

	if (on == SWITCH_ON) {
		switch (fea) {
		case TS_FEATURE_NONE:
			config = &ts->normal_config;
			break;
		case TS_FEATURE_GLOVE:
			config = &ts->glove_config;
			break;
		case TS_FEATURE_HOLSTER:
			config = &ts->holster_config;
			break;
		case TS_FEATURE_POCKET:
			config = &ts->pocket_config;
			break;
		default:
			return -EINVAL;
		}
	} else if (on == SWITCH_OFF) {
		if (info->holster_info.holster_switch)
			config = &ts->holster_config;
		/**else if (info->glove_info.glove_switch)
			config = &ts->glove_config;*/
		else
			config = &ts->normal_config;
	}

	ts->noise_env = false;
	ret = goodix_send_cfg(config);
#endif
	
	return ret;
}

static int goodix_feature_resume(struct goodix_ts_data *ts)
{
	int ret = 0;
	struct goodix_ts_config *config = NULL;

	if (ts->noise_env)
		config = &ts->normal_noise_config;
	else
		config = &ts->normal_config;	
	
	ret = goodix_send_cfg(config);
	TS_LOG_INFO("goodix_send_cfg: %d", ret);

#if 0
	if (info->charger_info.charger_switch) {
		ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
		TS_LOG_INFO("Charger switch on");
	}
#endif


	return ret;
}

static int goodix_noise_ctrl(struct goodix_ts_data *ts, bool on)
{
	int ret = 0;

#if 0
	struct ts_feature_info *info = &g_ts_data.feature_info;
	struct goodix_ts_config *config = NULL;


	TS_LOG_DEBUG("Noise ctrl:%d", on);
	if (info->holster_info.holster_switch) {
		/* reserve */
	/*} else if (info->glove_info.glove_switch) {
		if (on)
			config = &ts->glove_noise_config;
		else
			config = &ts->glove_config;*/
	} else {
		if (on)
			config = &ts->normal_noise_config;
		else
			config = &ts->normal_config;
	}

	if (config) {
		ret = goodix_send_cfg(config);
		if (ret < 0)
			return ret;
	}

#if 0
	if (info->charger_info.charger_switch) {
		ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
		TS_LOG_INFO("Charger switch on");
	}
#endif

#endif
	return ret;
}

#ifdef ROI
static int goodix_ts_roi_init(struct goodix_ts_roi *roi)
{
	unsigned int roi_bytes;

	if (!roi)
		return -EINVAL;

	if (!roi->roi_rows || !roi->roi_cols) {
		TS_LOG_ERR("Invalid roi config,rows:%d,cols:%d",
				roi->roi_rows, roi->roi_cols);
		return -EINVAL;
	}

	mutex_init(&roi->mutex);

	roi_bytes = (roi->roi_rows * roi->roi_cols + 1) * 2;
	roi->rawdata = kmalloc(roi_bytes + ROI_HEAD_LEN, GFP_KERNEL);
	if (!roi->rawdata) {
		TS_LOG_ERR("Failed to alloc memory for roi");
		return -ENOMEM;
	}

	TS_LOG_INFO("ROI init,rows:%d,cols:%d",
				roi->roi_rows, roi->roi_cols);

	return 0;
}

static int goodix_cache_roidata(struct goodix_ts_roi *roi)
{
	unsigned roi_bytes;
	unsigned char status[ROI_HEAD_LEN];
	u16 checksum = 0;
	int i, ret;
	
	if (unlikely(!roi || !roi->enabled))
		return -EINVAL;

	ret = goodix_i2c_read(ROI_STA_REG, status, ROI_HEAD_LEN);
	if (unlikely(ret < 0))
		return ret;

	for (i = 0; i < ROI_HEAD_LEN; i++)
		checksum += status[i];

	if (unlikely((u8)checksum != 0)) { /* cast to 8bit checksum,*/
		TS_LOG_ERR("roi status checksum error{%02x %02x %02x %02x}",
				status[0], status[1], status[2], status[3]);
		return -1;
	}

	if (likely(status[0] & ROI_READY_MASK)) /* roi data ready */
		roi->track_id = status[0] & ROI_TRACKID_MASK;
	else
		return -1; /* not ready */

	mutex_lock(&roi->mutex);
	roi->data_ready = false;
	roi_bytes = (roi->roi_rows * roi->roi_cols + 1) * 2;

	ret = goodix_i2c_read(ROI_DATA_REG,
			(u8 *)(roi->rawdata) + ROI_HEAD_LEN, roi_bytes);
	if (unlikely(ret < 0)) {
		mutex_unlock(&roi->mutex);
		return ret;
	}

	for (i = 0, checksum = 0; i < roi_bytes / 2; i++) /* 16bits */
		checksum += roi->rawdata[i + ROI_HEAD_LEN / 2];
	memcpy(&roi->rawdata[0], &status[0], ROI_HEAD_LEN);
	
	if (unlikely(checksum != 0))
		TS_LOG_ERR("roi data checksum error");
	else
		roi->data_ready = true;

	mutex_unlock(&roi->mutex);

	status[0] = 0x00;
	ret = goodix_i2c_write(ROI_STA_REG, status, 1);

	return ret;
}
#endif

/**
 * goodix_request_event_handler - firmware request 
 * Return    <0: failed, 0: succeed
 */
static int goodix_request_event_handler(struct goodix_ts_data *ts)
{
	u8 rqst_data = 0;
	int ret;

	ret = goodix_i2c_read(GTP_REG_RQST, &rqst_data, 1);
	if (ret)
		return ret;

	TS_LOG_DEBUG("Request state:0x%02x", rqst_data);
	switch (rqst_data & 0x0F) {
	case GTP_RQST_CONFIG:
		TS_LOG_INFO("Request Config.");
		ret = goodix_send_cfg(&ts->normal_config);
		if (ret) {
			TS_LOG_ERR("Send config error");
		} else {
			TS_LOG_INFO("Send config success");
			rqst_data = GTP_RQST_RESPONDED;
			goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		}
		break;
	case GTP_RQST_RESET:
		TS_LOG_INFO("Request Reset.");
		goodix_i2c_read(0x5097, &rqst_data, 1);
		TS_LOG_INFO("Reason code[0x5097]:%02x", rqst_data);
		goodix_chip_reset(20);
		msleep(40);
		goodix_feature_resume(ts);
		rqst_data = GTP_RQST_RESPONDED;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	case GTP_RQST_NOISE_CFG:
		TS_LOG_INFO("Request noise config");
		ret = goodix_noise_ctrl(ts, true);
		if (!ret)
			ts->noise_env = true;
		rqst_data = GTP_RQST_IDLE;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	case GTP_RQST_NORMA_CFG:
		TS_LOG_INFO("Request normal config");
		ret = goodix_noise_ctrl(ts, false);
		if (!ret)
			ts->noise_env = false;
		rqst_data = GTP_RQST_IDLE;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	default:
		break;
	}
	return 0;
}

static int easy_wakeup_gesture_report_coordinate(
						 unsigned int
						 reprot_gesture_point_num,
						 struct ts_fingers *info)
{
	int retval = 0;
	u16 x = 0;
	u16 y = 0;
	int i = 0;
	u8 buf[64*4];
	u16 coordinate_x[10] = {0};
	u16 coordinate_y[10] = {0};
	u8 point_num;
	struct goodix_coordinate point[5];
	u16 top_x = 0;
	u16 top_y = 0;
	u16 bottom_x = 0;
	u16 bottom_y = 0;
	u16 left_x = 0;
	u16 left_y = 0;
	u16 right_x =0;
	u16 right_y =0;

	if (reprot_gesture_point_num != 0)
      {
		retval = goodix_i2c_read(0x8140, buf, 45);
		if(retval <0){
			TS_LOG_ERR("%s read gesture coordinate failed \n",__func__);
			return retval;
		}

		/*
		 *The most points num is 6,point from 1(lower address) to 6(higher address) means:
		 *1.beginning 2.end 3.top 4.leftmost 5.bottom 6.rightmost
		 */
		point_num = buf[1];
		TS_LOG_INFO("%s: point_num = %d\n", __func__, point_num);

		if(reprot_gesture_point_num == 2){
			TS_LOG_INFO("%s: Gesture Dobule Click \n", __func__);
			/*1.beginning 2.end */
			for (i = 0; i < 2; i++) {
					x =  (buf[14 + i*4] << 8) | (buf[13 + i*4] & 0xFF);
					y = (buf[16 + i*4]<<8 )| (buf[15 + i*4] & 0xFF);
					coordinate_x[i] = x;
					coordinate_y[i] = y;
					g_goodix_dev_data->easy_wakeup_info.easywake_position[i] = x << 16 | y;
					TS_LOG_DEBUG("%s: Gesture Repot Point %d, x = %d, y = %d\n", __func__, i, x, y);
					TS_LOG_DEBUG("easywake_position[%d] = 0x%08x\n", i,
						    g_goodix_dev_data->easy_wakeup_info.easywake_position[i]);
					TS_LOG_DEBUG("%s: Gesture Repot Point %d, coordinate_x = %d, coordinate_y = %d\n",
						__func__, i, coordinate_x[i], coordinate_y[i]);
			}
			return retval;
		}

/* fill gesture coordinate */
#if 0
		else
		{
	            for (i = 0; i < 5; i++) {
				x =  (((s16) buf[26 + (4 * i)]) & 0x0F) << 8 | (((s16) buf[25 + (4 * i)])& 0xFF);
				y = (((s16) buf[28 + (4 * i)]) & 0x0F) <<8 | (((s16) buf[27 + (4 * i)]) & 0xFF);
				coordinate_x[2 + i] = x;
				coordinate_y[2 + i] = y;
				TS_LOG_DEBUG("%s: Gesture Repot Point %d, x = %d, y = %d\n", __func__, i, x, y);
				TS_LOG_DEBUG("%s: Gesture Repot Point %d, coordinate_x = %d, coordinate_y = %d\n",
					__func__, i+2, coordinate_x[2 + i], coordinate_y[2 + i]);
	            }

	            top_y = coordinate_y[0];
	            bottom_y = coordinate_y[0];
	            left_x = coordinate_x[0];
	            right_x = coordinate_x[0];

	            for (i = 0; i < 7; i++) {

				x = coordinate_x[i];
				y = coordinate_y[i];

	                    if(top_y > y){
	                            top_y = y;
	                            top_x = x;
	                    }
	                    if(bottom_y < y){
	                            bottom_y = y;
	                            bottom_x = x;
	                    }
	                    if( left_x > x){
	                            left_x = x;
	                            left_y = y;
	                    }
	                    if(right_x < x){
	                            right_x = x;
	                            right_y = y;
	                    }

	                    TS_LOG_DEBUG("%s: Gesture Repot Point %d, x = %d, y = %d\n", __func__, i, x, y);
	            }

	            /*3.top */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[2] = top_x << 16 | top_y;
	                    TS_LOG_DEBUG("top = 0x%08x,  top_x= %d , top_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[2], top_x, top_y);
	            /*4.leftmost */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[3] = left_x << 16 | left_y;
	                    TS_LOG_DEBUG("leftmost = 0x%08x,  left_x= %d , left_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[3], left_x, left_y);
	            /*5.bottom */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[4] = bottom_x << 16 | bottom_y;
	                    TS_LOG_DEBUG("bottom = 0x%08x,  x= %d , y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[4], bottom_x, bottom_y);
	            /*6.rightmost */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[5] = right_x << 16 | right_y;
	                    TS_LOG_DEBUG("rightmost = 0x%08x,  x= %d , y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[5], right_x, right_y);
		}
#endif

#if 1
		else{
			retval = goodix_i2c_read(0xC0EA, buf, 64*4);
			if(retval <0){
				TS_LOG_ERR("%s read gesture coordinate failed \n",__func__);
				return retval;
			}

	            x =  ((buf[1] << 8) | (buf[0] & 0xFF));
	            y =  ((buf[3] << 8) | (buf[2] & 0xFF));
	            top_y = y;
	            bottom_y = y;
	            left_x = x;
	            right_x =x;

	            for (i = 0; i < 64; i++) {
			     x =  ( (buf[1 + (4 * i)] << 8) | (buf[0 + (4 * i)]& 0xFF));
			     y = ( (buf[3 + (4 * i)] <<8) | (buf[2 + (4 * i)] & 0xFF));

	                    if(x ==0 && y==0) {
					break;
	                    }

	                    if(top_y > y){
	                            top_y = y;
	                            top_x = x;
	                    }
	                    if(bottom_y < y){
	                            bottom_y = y;
	                            bottom_x = x;
	                    }
	                    if(left_x > x){
	                            left_x = x;
	                            left_y = y;
	                    }
	                    if(right_x < x){
	                            right_x = x;
	                            right_y = y;
	                    }
	                    TS_LOG_DEBUG("%s: [0xC0EA] Gesture Repot Point %d, x = %d, y = %d\n", __func__, i, x, y);
	            }

	            /*1.begin */
		     x =  ( (buf[1] << 8) | (buf[0]& 0xFF));
		     y = ( (buf[3] <<8) | (buf[2] & 0xFF));
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[0] = x << 16 | y;
	                    TS_LOG_INFO("top = 0x%08x,  begin_x= %d , begin_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[0], x, y);

	            /*2.end */
		     x =  ( (buf[1 + (4 * (i-1))] << 8) | (buf[0 + (4 * (i-1))]& 0xFF));
		     y = ( (buf[3 + (4 * (i-1))] <<8) | (buf[2 + (4 * (i-1))] & 0xFF));
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[1] = x << 16 | y;
	                    TS_LOG_INFO("top = 0x%08x,  end_x= %d , end_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[1], x, y);

	            /*3.top */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[2] = top_x << 16 | top_y;
	                    TS_LOG_INFO("top = 0x%08x,  top_x= %d , top_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[2], top_x, top_y);
	            /*4.leftmost */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[3] = left_x << 16 | left_y;
	                    TS_LOG_INFO("leftmost = 0x%08x,  left_x= %d , left_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[3], left_x, left_y);
	            /*5.bottom */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[4] = bottom_x << 16 | bottom_y;
	                    TS_LOG_INFO("bottom = 0x%08x,  bottom_x= %d , bottom_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[4], bottom_x, bottom_y);
	            /*6.rightmost */
	            g_goodix_dev_data->easy_wakeup_info.easywake_position[5] = right_x << 16 | right_y;
	                    TS_LOG_INFO("rightmost = 0x%08x,  right_x= %d , right_y= %d \n",
	                                g_goodix_dev_data->easy_wakeup_info.easywake_position[5], right_x, right_y);
		}
#endif

	}
	return retval;
}

static int goodix_check_key_gesture_report(
					     struct ts_fingers *info,
					     struct ts_easy_wakeup_info
					     *gesture_report_info,
					     unsigned char
					     get_gesture_wakeup_data)
{
	int retval = 0;
	unsigned int reprot_gesture_key_value = 0;
	unsigned int reprot_gesture_point_num = 0;

	TS_LOG_DEBUG("get_gesture_wakeup_data is %d \n",
		    get_gesture_wakeup_data);

	switch (get_gesture_wakeup_data) {
		case DOUBLE_CLICK_WAKEUP:
			if (IS_APP_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG("@@@DOUBLE_CLICK_WAKEUP detected!@@@\n");
				reprot_gesture_key_value = TS_DOUBLE_CLICK;
				LOG_JANK_D(JLID_TP_GESTURE_KEY, "JL_TP_GESTURE_KEY");
				reprot_gesture_point_num = LINEAR_LOCUS_NUM;
		}
		break;
		case SPECIFIC_LETTER_C:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_c) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_c detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_c;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
			break;
		case SPECIFIC_LETTER_E:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_e) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_e detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_e;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
			break;
		case SPECIFIC_LETTER_M:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_m) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_m detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_m;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
			break;
		case SPECIFIC_LETTER_W:
			if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_w) &
			    gesture_report_info->easy_wakeup_gesture) {
				TS_LOG_DEBUG
				    ("@@@SPECIFIC_LETTER_w detected!@@@\n");
				reprot_gesture_key_value = TS_LETTER_w;
				reprot_gesture_point_num = LETTER_LOCUS_NUM;
			}
		break;
	default:
		TS_LOG_INFO("@@@unknow gesture detected!\n");
		return 0;
	}

	if (0 != reprot_gesture_key_value) {
		/*increase wake_lock time to avoid system suspend.*/
		wake_lock_timeout(&g_goodix_dev_data->ts_platform_data->ts_wake_lock, 5 * HZ);
		mutex_lock(&wrong_touch_lock);

		if (true == g_goodix_dev_data->easy_wakeup_info.off_motion_on) {
			//g_goodix_dev_data->easy_wakeup_info.off_motion_on = false;
			retval = easy_wakeup_gesture_report_coordinate(
								  reprot_gesture_point_num,
								  info);
			if (retval < 0) {
				mutex_unlock(&wrong_touch_lock);
				TS_LOG_ERR
				    ("%s: report line_coordinate error!retval = %d\n",
				     __func__, retval);
				return retval;
			}

			info->gesture_wakeup_value = reprot_gesture_key_value;
			TS_LOG_DEBUG
			    ("%s: info->gesture_wakeup_value = %d\n",
			     __func__, info->gesture_wakeup_value);
		}
		mutex_unlock(&wrong_touch_lock);
	}
	return NO_ERR;
}

static int goodix_read_gestrue_data(
					     struct ts_fingers *info,
					     struct ts_easy_wakeup_info
					     *gesture_report_info,
					     unsigned char gesture_id)
{
	int retval = NO_ERR;

#ifdef CONFIG_LOG_JANK
	LOG_JANK_D(JLID_TP_GESTURE_KEY , "%s", "JL_TP_GESTURE_KEY ");
#endif

	retval = goodix_check_key_gesture_report(info,
									gesture_report_info,
									gesture_id);
	return retval;
}

static int goodix_check_gesture(struct ts_fingers *info)
{
	int retval = NO_ERR;
	unsigned char gesture_id[2],doze_buf;
	struct ts_easy_wakeup_info *gesture_report_info = &g_goodix_dev_data->easy_wakeup_info;
	if (false == gesture_report_info->easy_wakeup_flag)
		return 1;

	retval = goodix_i2c_read(0x814B, gesture_id, 2);
	TS_LOG_INFO("gesture_id = 0x%02X, point_num : %d ", gesture_id[0], gesture_id[1]);
	retval = goodix_read_gestrue_data(info, gesture_report_info, gesture_id[0]);

	/*
	if(retval > 0){
		retval = goodix_read_gestrue_data(info, gesture_report_info, gesture_id);
		return 0;
	}
	else
	{
		TS_LOG_ERR("%s read state:%d \n",__func__);
		return 1;
	}*/

	/* Clear 0x814B */
	doze_buf = 0x00;
	goodix_i2c_write(0x814B, &doze_buf, 1);

	return NO_ERR;
}

/**
 * gt1x_touch_evt_handler - handle touch event 
 * (pen event, key event, finger touch envent)
 * Return    <0: failed, 0: succeed
 */
static int goodix_touch_evt_handler(struct goodix_ts_data  *ts,
				struct ts_fingers *info)
{
	u8 touch_data[1 + 8 * GTP_MAX_TOUCH + 1] = {0};
#ifdef ROI
	static u16 pre_index = 0;
#endif
	u16 cur_index = 0;
	u8 touch_num;
	u8 *coor_data = NULL;
	int id, x, y, w, i;
	int ret = -1;
	static u16 pre_touch = 0;
	u8 sync_val = 0;

	ret = goodix_check_gesture(info);
	if (!ret) {
		TS_LOG_DEBUG
		    ("focal_gesture_report is called and report gesture\n");
		return ret;
	}

	ret = goodix_i2c_read(GTP_READ_COOR_ADDR, touch_data, 10);
	if (unlikely(ret))
		goto exit;

#if 0
	if (unlikely(!touch_data[0])) {
		/* hw request */
		goodix_request_event_handler(ts);
		ret = 1;
		goto exit;
	}

	if(touch_data[0]==0){
		return;
	}
#endif

	if((touch_data[0]&0x80)==0){
		TS_LOG_DEBUG("Illegal state!");
		goto exit;
	}

	touch_num = touch_data[0] & 0x0f;
	TS_LOG_DEBUG("touch_num = %d ",touch_num);

	if (touch_num > GTP_MAX_TOUCH) {
		TS_LOG_DEBUG("Illegal finger number!");
		goto exit;
	}

	/* read the remaining coor data 
		* 0x814E(touch status) + 
		* 8(every coordinate consist of 8 bytes data) * touch num + 
		* keycode
		*/
	if (touch_num > 1) {
		u8 buf[8 * GTP_MAX_TOUCH];
		ret = goodix_i2c_read((GTP_READ_COOR_ADDR),buf, (1+touch_num* 8+1));
		if (ret)
			goto exit;
		memcpy(&touch_data[0], &buf[0], (1+8 * touch_num +1));
	}



#if 0
	key_value = touch_data[1 + 8 * touch_num];
	/*  start check current event */
	if ((touch_data[0] & 0x10) && (key_value & 0x0F)) {
		//for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
//			input_report_key(dev, gt1x_touch_key_array[i],
//					key_value & (0x01 << i));
		// key
	}
#endif

	if(touch_num>0){
		
		for (i = 0; i < touch_num; i++) {
			coor_data = &touch_data[i * 8+1];
			id = coor_data[0] & 0x0f;
			x = coor_data[1] | (coor_data[2] << 8);
			y = coor_data[3] | (coor_data[4] << 8);
			w = coor_data[5] | (coor_data[6] << 8);

			if (unlikely(ts->flip_x))
				info->fingers[id].x = ts->max_x - x;
			else
				info->fingers[id].x = x;

			if (unlikely(ts->flip_y))
				info->fingers[id].y = ts->max_y - y;
			else
				info->fingers[id].y = y;

			info->fingers[id].major = w;
			info->fingers[id].minor = w;
			info->fingers[id].pressure = w;
			info->fingers[id].status = TP_FINGER;
			cur_index |= 1 << id;
			TS_LOG_DEBUG("%s:x = 0x%x; y = 0x%x; w = 0x%x\n",__func__, x, y, w);
		}

		info->cur_finger_number = touch_num;
	}
	else if (pre_touch){
		info->cur_finger_number = 0;
	}
	TS_LOG_DEBUG("info->cur_finger_number  = [%d],pre_touch is [%d] \n", info->cur_finger_number ,pre_touch);
	pre_touch = touch_num;
	
#ifdef ROI
	if (pre_index != cur_index && (cur_index & pre_index) != cur_index)
		goodix_cache_roidata(&ts->roi);
	pre_index = cur_index;
#endif

exit:
	TS_LOG_DEBUG("evt_handler ret= [%d] \n", ret);

/*sync_evt:*/
	if (!ts->rawdiff_mode)
		ret = goodix_i2c_write(GTP_READ_COOR_ADDR, &sync_val, 1);
	else
		TS_LOG_DEBUG("Firmware rawdiff mode");
	return ret;
}

#if 0
static int goodix_gesture_evt_handler(struct goodix_ts_data *ts,
				struct ts_fingers *info)
{
	u8 doze_buf[4] = {0}, ges_type;
	int len;
	int ret = 0;

	/** package: -head 4B + track points + extra info- 
		* - head -
		*  doze_buf[0]: gesture type, 
		*  doze_buf[1]: number of gesture points ,
		*  doze_buf[2]: protocol type, 
		*  doze_buf[3]: gesture extra data length.
		*/
	ret = goodix_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 4);
	if (ret < 0)
		return 0;

	ges_type = doze_buf[0];
	len = doze_buf[1];
/*	need_chk = doze_buf[2] & 0x80;
	extra_len = doze_buf[3];
*/

	TS_LOG_DEBUG("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE,
		doze_buf[0], doze_buf[1], doze_buf[2], doze_buf[3]);

	/* check gesture type (if available?) */
	if (ges_type == 0) {
		TS_LOG_INFO("Invalid gesture");
		doze_buf[0] = 0x00;
		//goodix_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
		//gesture_enter_doze();
		info->gesture_wakeup_value = TS_GESTURE_INVALID;
		return 0;
	}

	switch (ges_type) {
	case 0xAA:
		break;
	default:
		break;
	}

	info->gesture_wakeup_value = ges_type;
	ret = 1;

	doze_buf[0] = 0; // clear ges flag
	goodix_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
	return ret;
}
#endif

/**
 * goodix_irq_bottom_half - Goodix touchscreen work function.
 */
static int goodix_irq_bottom_half(struct ts_cmd_node *in_cmd,
				struct ts_cmd_node *out_cmd)
{
	struct goodix_ts_data *ts = goodix_ts;
	struct ts_fingers *ts_fingers;
	int ret = 0;

	if (unlikely(!goodix_ts))
		return -ENODEV;

	ts_fingers = &out_cmd->cmd_param.pub_params.algo_param.info;
	out_cmd->command = TS_INVAILD_CMD;
	out_cmd->cmd_param.pub_params.algo_param.algo_order =
			g_goodix_dev_data->algo_id;
	// debug+
	TS_LOG_DEBUG("Algo-order: %d", g_goodix_dev_data->algo_id);

#if 0
	ret = goodix_gesture_evt_handler(ts, ts_fingers);
	if (ret == 1)
		goto sync_evt;
#endif

	/* handle touch event 
	 * return: <0 - error, 0 - touch event handled,
	 * 			1 - hw request event handledv */
	ret = goodix_touch_evt_handler(ts, ts_fingers);
	if (ret == 0)
		out_cmd->command = TS_INPUT_ALGO;

	return 0;
}

static int goodix_i2c_test(struct ts_kit_platform_data *goodix_pdata)
{
	u8 hw_info;
	int ret;

	TS_LOG_INFO("goodix slave_addr: 0x%x", g_goodix_dev_data->slave_addr);

	ret = goodix_i2c_read(GTP_REG_CONFIG_DATA, &hw_info,sizeof(hw_info));

	TS_LOG_INFO("IIC test Info:%08X", hw_info);
	return ret;
}

static int goodix_request_gpio(struct ts_kit_platform_data *goodix_pdata)
{
	//int reset_gpio = dev_data->reset_gpio;
	int irq_gpio = goodix_pdata->irq_gpio;
//	int ret = 0;

	if (!gpio_is_valid(irq_gpio)/* || !gpio_is_valid(reset_gpio) */) {
		TS_LOG_ERR("Invalid gpios");
		return -EINVAL;
	}

	gpio_direction_input(irq_gpio);

	TS_LOG_INFO("Request gpio: OK");
	return 0;
}

static void goodix_release_gpio(struct ts_kit_platform_data *goodix_dev_data)
{
	int reset_gpio = goodix_dev_data->reset_gpio;
	int irq_gpio = goodix_dev_data->irq_gpio;

	if (gpio_is_valid(reset_gpio))
		gpio_free(reset_gpio);

	if (gpio_is_valid(irq_gpio))
		gpio_free(irq_gpio);
}


#define GTP_VTG_MIN_UV           2800000
#define GTP_VTG_MAX_UV          2800000
#define GTP_I2C_VTG_MIN_UV    1800000
#define GTP_I2C_VTG_MAX_UV   1800000

static int goodix_get_regulators(struct goodix_ts_data *ts)
{
	int ret;

	ts->vdd_ana = regulator_get(&ts->pdev->dev, "goodix-vdd");
	if (IS_ERR(ts->vdd_ana)) {
		ret = PTR_ERR(ts->vdd_ana);
		TS_LOG_ERR("Regulator get of vdd_ana failed:%d", ret);
		ts->vdd_ana = NULL;
		return ret;
	}

	ts->vcc_i2c = regulator_get(&ts->pdev->dev, "goodix-io");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		TS_LOG_ERR("Regulator get of vcc_i2c failed:%d", ret);
		ts->vcc_i2c = NULL;
		goto err_get_vcc;
	}

	if (regulator_count_voltages(ts->vdd_ana) > 0) {
		ret = regulator_set_voltage(ts->vdd_ana, GTP_VTG_MIN_UV,
					   GTP_VTG_MAX_UV);
		if (ret) {
			TS_LOG_ERR("Regulator set_vtg failed vdd rc=%d\n", ret);
			goto err_set_vol;
		}
	}

	if (regulator_count_voltages(ts->vcc_i2c) > 0) {
		ret = regulator_set_voltage(ts->vcc_i2c, GTP_I2C_VTG_MIN_UV,
					   GTP_I2C_VTG_MAX_UV);
		if (ret) {
			TS_LOG_ERR("Regulator set_vtg failed vcc_i2c rc=%d\n", ret);
			goto err_set_vol;
		}
	}

	TS_LOG_INFO("Regulator get: OK");
	return 0;

err_set_vol:
	regulator_put(ts->vcc_i2c);
	ts->vcc_i2c = NULL;
err_get_vcc:
	regulator_put(ts->vdd_ana);
	ts->vdd_ana = NULL;
	return ret;
}

static void goodix_put_regulators(struct goodix_ts_data *ts)
{
	if (ts->vdd_ana) {
		regulator_put(ts->vdd_ana);
		ts->vdd_ana = NULL;
	}

	if (ts->vcc_i2c) {
		regulator_put(ts->vcc_i2c);
		ts->vcc_i2c = NULL;
	}
}

/**
 * goodix_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int goodix_power_switch(struct goodix_ts_data *ts, int on)
{
	int ret = 0;

	if (on) {
		TS_LOG_INFO("GTP power ON");
		ret |= regulator_enable(ts->vdd_ana);
		udelay(2);
		ret |= regulator_enable(ts->vcc_i2c);
		udelay(2);
	} else {
		TS_LOG_INFO("GTP power OFF");
		ret |= regulator_disable(ts->vdd_ana);
		udelay(2);
		ret |= regulator_disable(ts->vcc_i2c);
		udelay(2);
	}

	TS_LOG_INFO("%s:%d ret=%d ",__func__,__LINE__,ret);
	return ret;
}

/**
 * goodix_pinctrl_init - pinctrl init
 */
static int goodix_pinctrl_init(struct goodix_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(&g_goodix_dev_data->ts_platform_data->ts_dev->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		TS_LOG_ERR("Failed to get pinctrl");
		ret = PTR_ERR(ts->pinctrl);
		return ret;
	}

	ts->pins_default = pinctrl_lookup_state(ts->pinctrl, "default");
	if (IS_ERR_OR_NULL(ts->pins_default)) {
		TS_LOG_ERR("Pin state[default] not found");
		ret = PTR_ERR(ts->pins_default);
		goto exit_put;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "idle");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		TS_LOG_ERR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto exit_put;
	}

	return 0;
exit_put:
	devm_pinctrl_put(ts->pinctrl);
	ts->pinctrl = NULL;
		ts->pins_touch_key = NULL;

	ts->pins_gesture = NULL;
	ts->pins_suspend = NULL;
	ts->pins_default = NULL;
	return ret;
}

static void goodix_pinctrl_release(struct goodix_ts_data *ts)
{
	if (ts->pinctrl)
		devm_pinctrl_put(ts->pinctrl);
	ts->pinctrl = NULL;
		ts->pins_touch_key = NULL;

	ts->pins_gesture = NULL;
	ts->pins_suspend = NULL;
	ts->pins_default = NULL;
}

/**
 * goodix_pinctrl_select_normal - set normal pin state
 *  Irq pin *must* be set to *pull-up* state.
 */
static int goodix_pinctrl_select_normal(struct goodix_ts_data *ts)
{
	int ret = 0;


	if (ts->pinctrl && ts->pins_default) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_default);
		if (ret < 0)
			TS_LOG_ERR("Set normal pin state error:%d", ret);
	}
	return ret;
}

/**
 * goodix_pinctrl_select_suspend - set suspend pin state
 *  Irq pin *must* be set to *pull-up* state.
 */
static int goodix_pinctrl_select_suspend(struct goodix_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0)
			TS_LOG_ERR("Set suspend pin state error:%d", ret);
	}

	if (ts->pinctrl && ts->pins_touch_key) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_touch_key);
		if (ret < 0)
			TS_LOG_ERR("Set touch_key pin state error:%d", ret);
	}

	return ret;
}

/**
 * goodix_pinctrl_select_gesture - set gesture pin state
 *  Irq pin *must* be set to *pull-up* state.
 */
static int goodix_pinctrl_select_gesture(struct goodix_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_gesture) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_gesture);
		if (ret < 0)
			TS_LOG_ERR("Set gesture pin state error:%d", ret);
	}
	
	return ret;
}

/**
 * goodix_read_version - Read gt9xx version info.
 * @hw_info: address to store version info
 * Return 0-succeed.
 */
int goodix_read_version(struct goodix_hw_info * hw_info)
{
	u8 buf[12] = { 0 };
	u32 mask_id, patch_id;
	u8 product_id[5] = {0};
	u8 sensor_id, match_opt;
	int i, retry = 3;
	u8 checksum = 0;
	int ret = -1;

	while (retry--) {
		ret = goodix_i2c_read(GTP_REG_VERSION, buf, sizeof(buf));
		if (!ret) {
			for (i = 0, checksum = 0; i < sizeof(buf); i++)
				checksum += buf[i];

			if (checksum == 0 &&/* first 3 bytes must be number or char */
				IS_NUM_OR_CHAR(buf[0]) && IS_NUM_OR_CHAR(buf[1])
				&& IS_NUM_OR_CHAR(buf[2]) && buf[10] != 0xFF) {
				break;
			} else if (checksum == (u8)(buf[11] * 2) && buf[10] != 0xFF) {
				/* checksum calculated by boot code */
				break;
			} else {
				TS_LOG_ERR("Invalid version info:%c%c%c%c%c%c", buf[0], buf[1], buf[2], buf[3], buf[5], buf[4]);
			}
		}

		TS_LOG_DEBUG("Read version failed,retry: %d", retry);
		msleep(100);
	}

	mask_id = (u32) ((buf[7] << 16) | (buf[8] << 8) | buf[9]);
	patch_id = (u32) ((buf[4] << 16) | (buf[5] << 8) | buf[6]);
	memcpy(product_id, buf, 4);
	sensor_id = 0;//buf[10] & 0x0F;
	match_opt = (buf[10] >> 4) & 0x0F;

	TS_LOG_INFO("IC Version:GT%s_%06X(FW)_%04X(Boot)_%02X(SensorID)",
		product_id, patch_id, mask_id >> 8, sensor_id);

	if (hw_info != NULL) {
		hw_info->mask_id = mask_id;
		hw_info->patch_id = patch_id;
		memcpy(hw_info->product_id, product_id, 5);
		hw_info->sensor_id = sensor_id;
		hw_info->match_opt = match_opt;
	}

	goodix_ts->sensor_id_valid = true;
	return 0;
}

#if 0
static int goodix_parse_dts(struct goodix_ts_data *ts)
{
	struct device_node *device = ts->pdev->dev.of_node;
	int ret = 0;

	ret = of_property_read_u32(device, "x_max_mt", &ts->max_x);
	if (ret) {
		TS_LOG_ERR("Get x_max_mt failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(device, "y_max_mt", &ts->max_y);
	if (ret) {
		TS_LOG_ERR("Get y_max_mt failed");
		ret = -EINVAL;
		goto err;
	}

#ifdef ROI
	ret = of_property_read_u32_index(device, "roi_data_size", 0,
			&ts->roi.roi_rows);
	if (ret) {
		TS_LOG_ERR("Get ROI rows failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32_index(device, "roi_data_size", 1,
			&ts->roi.roi_cols);
	if (ret) {
		TS_LOG_ERR("Get ROI cols failed");
		ret = -EINVAL;
		goto err;
	}
#endif
	ret = of_property_read_bool(device, "tools_support");
	if (!ret) {
		ts->tools_support = true;
		TS_LOG_INFO("Tools support enabled");
	}

err:
	return ret;
}
#endif

static int goodix_parse_specific_dts(struct goodix_ts_data *ts)
{
	struct device_node *device;
	char project_id[20] = {0};
	u32 value;
	int ret = 0;

	sprintf(project_id, "gtp-%s", "DLI45210");//otp need from reading register
	TS_LOG_INFO("Parse specific dts:%s", project_id);
	device = of_find_compatible_node(ts->pdev->dev.of_node, NULL, project_id);
	if (!device) {
		TS_LOG_INFO("No chip specific dts:%s, need to prase", project_id);
		return -EINVAL;
	}

	ret = of_property_read_u32(device, "x_max_mt", &value);
	if (!ret) 
		ts->max_x = value;
	
	ret =  of_property_read_u32(device, "y_max_mt", &value);
	if (!ret)
		ts->max_y = value;

	return 0;
}

/**
 * goodix_parse_dt_cfg - parse config data from devices tree.
 * @dev: device that this driver attached.
 * @cfg: pointer of the config array.
 * @cfg_len: pointer of the config length.
 * @sid: sensor id.
 * Return: 0-succeed, -1-faileds
 */
int goodix_parse_cfg_data(struct goodix_ts_data *ts,
				char *cfg_type, u8 *cfg, int *cfg_len, u8 sid)
{
	struct device_node *self = ts->pdev->dev.of_node;
	struct property *prop;
	int correct_len;
	char comp_name[GTP_VENDOR_COMP_NAME_LEN] = {0};
	int ret = 0;

	TS_LOG_INFO("goodix_parse_cfg_data \n");
	ret = snprintf(comp_name, GTP_VENDOR_COMP_NAME_LEN, "%s-%s", GTP_CHIP_NAME, ts->project_id);
	if (ret >= GTP_VENDOR_COMP_NAME_LEN) {
		TS_LOG_ERR("%s:%s, ret=%d, size=%lu\n", __func__,
			"compatible_name out of range", ret, GTP_VENDOR_COMP_NAME_LEN);
		return -EINVAL;
	}

	self = of_find_compatible_node(ts->pdev->dev.of_node,
				NULL, comp_name);
	if (!self) {
		TS_LOG_ERR("No chip specific dts:%s, need to parse", comp_name);
		return -EINVAL;
	}

	TS_LOG_INFO("Parse [%s] data from dts[SENSORID%u]", cfg_type, sid);
	prop = of_find_property(self, cfg_type, cfg_len);
	if (!prop || !prop->value || *cfg_len == 0)
		return -EINVAL;/* fail */

	TS_LOG_ERR("get normal cfg_len = %d ",*cfg_len);

	memcpy(cfg, prop->value, *cfg_len);

	correct_len = GTP_CONFIG_ORG_LENGTH;

	if (*cfg_len != correct_len) {
		TS_LOG_ERR("Invalid config size:%d", *cfg_len);
		return -EINVAL;
	}

	return 0;
}

/**
 * goodix_init_panel - Prepare config data for touch ic,\
 * don't call this function after initialization.
 *
 * Return 0--success,<0 --fail.
 */
int goodix_init_configs(struct goodix_ts_data *ts)
{
	u8 sensor_id, *cfg_data;
	int cfg_len = 0;
	int ret = 0;

	/* max config data length */
	cfg_len = sizeof(ts->normal_config.data);
	cfg_data = kzalloc(cfg_len, GFP_KERNEL);
	if (!cfg_data)
		return -ENOMEM;

	/* parse normal config data */
	ret = goodix_parse_cfg_data(ts, "normal_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse normal_config data:%d", ret);
		goto exit_kfree;
	}

	//cfg_data[0] &= 0x7F; /* mask config version */
	TS_LOG_INFO("Normal config version:%d,size:%d", cfg_data[0], cfg_len);
	memcpy(&ts->normal_config.data[0], cfg_data, cfg_len);
	ts->normal_config.size = cfg_len;
	ts->normal_config.delay_ms = 200;
	ts->normal_config.name = "normal_config";
	ts->normal_config.initialized = true;

	/* parse normal noise config data*/
	ret = goodix_parse_cfg_data(ts, "normal_noise_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse normal_noise_config data:%d", ret);
		ts->normal_noise_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		TS_LOG_INFO("Normal noise config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->normal_noise_config.data[0], cfg_data, cfg_len);
		ts->normal_noise_config.size = cfg_len;
		ts->normal_noise_config.delay_ms = 100;
		ts->normal_noise_config.name = "normal_noise_config";
		ts->normal_noise_config.initialized = true;
	}

	/* parse glove config data */
	ret = goodix_parse_cfg_data(ts, "glove_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse glove_config data:%d", ret);
		ts->glove_config.initialized = false;
		ret = 0;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7F; /* mask config version */
		TS_LOG_INFO("Glove config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->glove_config.data[0], cfg_data, cfg_len);
		ts->glove_config.size = cfg_len;
		ts->glove_config.delay_ms = 20;
		ts->glove_config.name = "glove_config";
		ts->glove_config.initialized = true;
	} else {
		ts->glove_config.initialized = false;
	}

	/* parse glove noise config data*/
	ret = goodix_parse_cfg_data(ts, "glove__noise_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse glove__noise_config data:%d", ret);
		ts->glove_noise_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		TS_LOG_INFO("Normal noise config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->glove_noise_config.data[0], cfg_data, cfg_len);
		ts->glove_noise_config.size = cfg_len;
		ts->glove_noise_config.delay_ms = 100;
		ts->glove_noise_config.name = "normal_noise_config";
		ts->glove_noise_config.initialized = true;
	}

	/* parse holster config data */
	ret = goodix_parse_cfg_data(ts, "holster_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse holster_config data:%d", ret);
		ts->holster_config.initialized = false;
		ret = 0;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7F; /* mask config version */
		TS_LOG_INFO("Holster config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->holster_config.data[0], cfg_data, cfg_len);
		ts->holster_config.size = cfg_len;
		ts->holster_config.delay_ms = 20;
		ts->holster_config.name = "holster_config";
		ts->holster_config.initialized = true;
	} else {
		ts->holster_config.initialized = false;
	}

	/* parse charger config data*/
	ret = goodix_parse_cfg_data(ts, "charger_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse charger_config data:%d", ret);
		ts->charger_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		TS_LOG_INFO("Charger config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->charger_config.data[0], cfg_data, cfg_len);
		ts->charger_config.size = cfg_len;
		ts->charger_config.delay_ms = 100;
		ts->charger_config.name = "charger_config";
		ts->charger_config.initialized = true;
	}

	/* parse pocket config data*/
	ret = goodix_parse_cfg_data(ts, "pocket_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		TS_LOG_ERR("Failed to parse pocket_config data:%d", ret);
		ts->pocket_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		TS_LOG_INFO("Pocket config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->pocket_config.data[0], cfg_data, cfg_len);
		ts->pocket_config.size = cfg_len;
		ts->pocket_config.delay_ms = 100;
		ts->pocket_config.name = "pocket_config";
		ts->pocket_config.initialized = true;
	}
	
exit_kfree:
	kfree(cfg_data);
	return ret;
}

/**
 * Hisi Platform Touchscreen Interface
 */
static int goodix_chip_parse_config(struct device_node *device,
				struct ts_kit_device_data *chip_data)
{
	int ret = 0;//, gpio;
	//u32 value;

	TS_LOG_INFO("goodix parse config");
	if (!device || !chip_data)
		return -ENODEV;

	ret = of_property_read_u32(device, "irq_config",
						&chip_data->irq_config);
	if (ret) {
		TS_LOG_ERR("Get irq config failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(device, "algo_id",
						&chip_data->algo_id);
	if (ret) {
		TS_LOG_ERR("Get algo id failed");
		ret = -EINVAL;
		goto err;
	}

#if 0
	ret = of_property_read_u32(device, "vci_value",
			&chip_data->regulator_ctrl.vci_value);
	if (ret < 0) {
		chip_data->regulator_ctrl.vci_value = 3100000;
		TS_LOG_INFO("Use default vdd voltage: 3.1v");
	}

	ret = of_property_read_u32(device, "need_set_vddio_value",
			&chip_data->regulator_ctrl.need_set_vddio_value);
	if (ret) {
		TS_LOG_INFO("Not defined need_set_vddio_value");
		chip_data->regulator_ctrl.need_set_vddio_value = 0;
	} else {
		ret = of_property_read_u32(device, "vddio_value", 
		&chip_data->regulator_ctrl.vddio_value);
		if (ret) {
			TS_LOG_INFO("Not defined vddio value in dts, use default value");
			chip_data->regulator_ctrl.vddio_value = 1800000;
		}
	}
#endif

#if 0
	ret = of_property_read_string(device, "tp_test_type",
			&chip_data->tp_test_type); 
	if (ret) {
		TS_LOG_INFO("tp_test_type not exist, use default value");
		strncpy(chip_data->tp_test_type, "Normalize_type:judge_different_result",
			TS_CAP_TEST_TYPE_LEN);
	}
#endif

#if 0
	ret = of_property_read_u32(device, "charger_supported", &value);
	if (!ret) {
		TS_LOG_INFO("get chip specific charger_supported = %d",
			value);
		g_ts_data.feature_info.charger_info.charger_supported = (u8)value;
	}
#endif

/*
	ret = of_property_read_u32(device, "roi_supported", &value);
	if (!ret) {
		TS_LOG_INFO("get chip specific roi_supported = %d", value);
		g_ts_data.feature_info.roi_info.roi_supported = (u8)value;
	} else {
		TS_LOG_INFO("Can not get roi_supported value");
		g_ts_data.feature_info.roi_info.roi_supported = 0;
	}
*/

	ret = 0;
err:
	return ret;
}


struct ts_kit_device_data *goodix_get_device_data(void)
{
	return g_goodix_dev_data;
}

struct goodix_ts_data *goodix_get_platform_data(void)
{
	return goodix_ts;
}

static int goodix_chip_detect(struct ts_kit_platform_data* pdata)
{
	int ret = NO_ERR;

	TS_LOG_INFO("Chip detect");
	if (!pdata){
		TS_LOG_ERR("%s device, ts_kit_platform_data *pdata is NULL \n", __func__);
		return -ENOMEM;
	}
	goodix_ts = kzalloc(sizeof(struct goodix_ts_data), GFP_KERNEL);
	if (!goodix_ts)
		return -ENOMEM;

	goodix_ts->goodix_device_data = g_goodix_dev_data;
	goodix_ts->ts_platform_data = pdata;
	goodix_ts->pdev = pdata->ts_dev;
	g_goodix_dev_data->ts_platform_data = pdata;
	goodix_ts->pdev->dev.of_node = g_goodix_dev_data->cnode;

	ret = goodix_prase_ic_config_dts(g_goodix_dev_data->cnode, g_goodix_dev_data);
	if (ret) {
		TS_LOG_ERR("%s:parse ic config dts fail, ret=%d\n",
			__func__, ret);
	}

	pdata->client->addr =g_goodix_dev_data->slave_addr;

	if ((!pdata) &&(!pdata->ts_dev)){
		TS_LOG_ERR("%s device, ts_kit_platform_data *data or data->ts_dev is NULL \n", __func__);
		ret = -ENOMEM;
		//goto exit;
	}

	g_goodix_dev_data->is_i2c_one_byte = 0;
	g_goodix_dev_data->is_new_oem_structure= 0;
	g_goodix_dev_data->is_parade_solution= 0;


	goodix_ts->ops.i2c_read = goodix_i2c_read;
	goodix_ts->ops.i2c_write = goodix_i2c_write;
	goodix_ts->ops.chip_reset = goodix_chip_reset;
	goodix_ts->ops.send_cmd = goodix_send_cmd;
	goodix_ts->ops.send_cfg = goodix_send_cfg;
	goodix_ts->ops.i2c_read_dbl_check = goodix_i2c_read_dbl_check;
	goodix_ts->ops.read_version = goodix_read_version;
	goodix_ts->ops.parse_cfg_data = goodix_parse_cfg_data;
	goodix_ts->ops.feature_resume = goodix_feature_resume;
	mutex_init(&wrong_touch_lock);

	/* do NOT remove these logs */
	TS_LOG_INFO("Driver Version: %s", GTP_DRIVER_VERSION);

	ret = goodix_get_regulators(goodix_ts);
	if (ret < 0)
		goto err_get_regs;

	ret = goodix_request_gpio(pdata);
	if (ret < 0)
		goto err_req_gpio;



	ret = goodix_pinctrl_init(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

	ret = goodix_pinctrl_select_normal(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

	
///////test//////
#if 0
	ret = goodix_pinctrl_init(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

	ret = pinctrl_select_state(goodix_ts->pinctrl, goodix_ts->pins_suspend);
	if (ret < 0)
		TS_LOG_ERR("Set normal pin state error:%d", ret);

	ret = goodix_pinctrl_init(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

	ret = goodix_pinctrl_select_normal(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

#endif


	/* power on */
	ret = goodix_power_switch(goodix_ts, SWITCH_ON);
	if (ret < 0)
		goto err_power_on;
	
	/* detect chip */

	ret = goodix_chip_reset(20);
	if (ret < 0)
		goto err_power_on;

	ret = goodix_i2c_test(pdata);
	if (ret < 0)
		goto err_power_on;
	else
	{
		TS_LOG_INFO("%s:find goodix device\n", __func__);
		strncpy(g_goodix_dev_data->chip_name, GTP_CHIP_NAME, MAX_STR_LEN);
	}

	return 0;

err_power_on:
	goodix_pinctrl_release(goodix_ts);
	goodix_power_switch(goodix_ts, SWITCH_OFF);
err_pinctrl_init:
	goodix_release_gpio(pdata);
err_req_gpio:
	goodix_put_regulators(goodix_ts);
err_get_regs:
	kfree(goodix_ts);
	goodix_ts = NULL;
	return ret;
}

static int goodix_hardwareinfo_set(void)
{
       char firmware_ver[HARDWARE_MAX_ITEM_LONGTH] = {'\0'};
       char vendor_for_id[HARDWARE_MAX_ITEM_LONGTH] = {'\0'};
       int err;

       if(NULL != strstr(goodix_ts->project_id, "DLI45210"))
               snprintf(vendor_for_id,HARDWARE_MAX_ITEM_LONGTH,"BIEL");
       else
               snprintf(vendor_for_id,HARDWARE_MAX_ITEM_LONGTH,"Other vendor");

       snprintf(firmware_ver,HARDWARE_MAX_ITEM_LONGTH,"%s,%s,FW:%02x%02x_0x%02x,ID:%s",
													vendor_for_id,
													goodix_ts->ic_name,
													goodix_ts->fw_ver[0],
													goodix_ts->fw_ver[1],
													goodix_ts->cfg_ver,
													goodix_ts->project_id);

	err = app_info_set("touch_panel", firmware_ver);
       if (err < 0){
               TS_LOG_ERR("%s:hardwareinfo_set_prop error, err=%d\n", __func__, err);
               return -1;
       }

       return 0;
}

int goodix_read_project_id(
	struct goodix_ts_data *ts,
	size_t size)
{
	int ret = NO_ERR;

	memset(ts->project_id, '\0', size);
	ret = goodix_i2c_read(ts->projectid_addr, ts->project_id, GTP_PROJECT_ID_LEN -1);
	if (ret) {
	    TS_LOG_ERR("%s:read project id fail, ret=%d\n", __func__, ret);
	}

	TS_LOG_INFO("%s: project id =%s \n", __func__, ts->project_id);
	return ret;
}

static int goodix_param_init(struct goodix_ts_data *ts)
{
	int ret = 0;
	//u8 vendor_id = 0;
	char* ptr = NULL;

	/* init project id and fw_ver and chip id */
	ret = goodix_read_project_id(ts, GTP_PROJECT_ID_LEN);
	if (ret) {
		TS_LOG_ERR("%s:read project id fail, ret=%d,hope update fw to recovery!\n", __func__, ret);
		memset(ts->project_id,'\0',GTP_PROJECT_ID_LEN);
		return ret;
	}

	ptr = strstr(ts->project_id, GTP_DEFAULT_PROJECT_ID);
	if(ptr == NULL)
	{
		TS_LOG_INFO("%s: get default project_id \n");
		memset(ts->project_id, '\0' , GTP_PROJECT_ID_LEN);
		memcpy(ts->project_id, GTP_DEFAULT_PROJECT_ID, strlen(GTP_DEFAULT_PROJECT_ID));
	}

	TS_LOG_INFO("%s: ts->project_id: %s\n",__func__, ts->project_id);

	ret = goodix_get_vendor_name_from_dts(ts->project_id,
		ts->vendor_name, GTP_VENDOR_NAME_LEN);
	if (ret) {
		TS_LOG_ERR("%s:read vendor name fail, ret=%d\n", __func__, ret);
		return ret;
	}

#if 0
	ret = goodix_read_vendor_id(goodix_ts, &vendor_id);
	if (ret) {
		TS_LOG_ERR("%s:read vendor id fail, ret=%d\n", __func__, ret);
		return ret;
	} else {
		goodix_ts->vendor_id = vendor_id;
	}

#endif

	ret = goodix_get_ic_firmware_version();
	if (ret) {
		TS_LOG_ERR("%s:read firmware version fail, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = goodix_hardwareinfo_set();
	if (ret < 0){
	       TS_LOG_ERR("%s:hardwareinfo_set error, ret=%d\n", __func__, ret);
	       return ret;
	}

	return 0;
}

static int goodix_chip_init(void)
{
	struct goodix_ts_data *ts = goodix_ts;
	int ret = -1;
	u8 reg_val[1];

	/* Step 1: Check Firmware */
	ret = goodix_i2c_read_dbl_check(GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
	if (!ret && reg_val[0] != 0xBE) {
		ts->fw_error = 1;
		TS_LOG_ERR("Check main system not pass[0x%2X]", reg_val[0]);
	}

	/* check subsystem firmware */
	/*ret = goodix_i2c_read_dbl_check(GTP_REG_FW_CHK_SUBSYS, reg_val, 1);
	if (!ret && reg_val[0] == 0xAA) {
		TS_LOG_ERR("Check subsystem not pass[0x%2X]", reg_val[0]);
	}*/

	/* Step 2: Recover Firmware */
	if(ts->fw_error)
	{
		TS_LOG_ERR("%s Recover Firmware beagin \n", __func__);
		/* goto auto upgrade type */
		memset(goodix_ts->auto_fw_name, '\0', GTP_FW_NAME_LEN);
		memcpy(goodix_ts->auto_fw_name, GT9XX_FW_NAME, GTP_FW_NAME_LEN);
		goodix_ts->fw_type = GOODIX_AUTO_UPGRADE_FW;

		/* Recover Firmware for read project id */
		ret = gup_update_proc(UPDATE_TYPE_HEADER);
		if(SUCCESS != ret){
			TS_LOG_ERR("%s Recover Firmware failed \n", __func__);
		}
	}

	/* Step 3: Parse dts */
	ret = goodix_parse_dts(g_goodix_dev_data->cnode, goodix_ts);
	if (ret) {
	    TS_LOG_ERR("%s:parse dts fail, ret=%d\n", __func__, ret);
	}
	g_goodix_dev_data->rawdata_get_timeout = GTP_RAWDATA_TIMEOUT;

	/* Step 4: Read project id */
	ret = goodix_param_init(goodix_ts);
	if (ret) {
		TS_LOG_ERR("%s:init param fail, ret=%d\n", __func__, ret);
	}

	/* init config data, normal/glove/hoslter config data */
	goodix_init_configs(ts);

	ret = goodix_feature_resume(ts);
	if (ret < 0)
		return ret;

	init_wr_node();
	return 0;


#if 0
	/* read version information. pid/vid/sensor id */
	ret = goodix_read_version(&ts->hw_info);
	if (ret < 0)
		return ret;



	/* obtain goodix dt properties */
	ret = goodix_parse_dts(ts);
	if (ret < 0)
		return ret;


	ret = goodix_parse_specific_dts(ts);
	if (ret < 0)
		return ret;

	if (ts->tools_support)
		gt1x_init_tool_node();



#endif

#ifdef ROI
	ret = goodix_ts_roi_init(&goodix_ts->roi);
	if (ret < 0)
		return ret;
#endif

	return 0;
}

static int goodix_input_config(struct input_dev *input_dev)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	set_bit(TS_DOUBLE_CLICK, input_dev->keybit);
	set_bit(TS_SLIDE_L2R, input_dev->keybit);
	set_bit(TS_SLIDE_R2L, input_dev->keybit);
	set_bit(TS_SLIDE_T2B, input_dev->keybit);
	set_bit(TS_SLIDE_B2T, input_dev->keybit);
	set_bit(TS_CIRCLE_SLIDE, input_dev->keybit);
	set_bit(TS_LETTER_c, input_dev->keybit);
	set_bit(TS_LETTER_e, input_dev->keybit);
	set_bit(TS_LETTER_m, input_dev->keybit);
	set_bit(TS_LETTER_w, input_dev->keybit);
	set_bit(TS_PALM_COVERED, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#ifdef INPUT_TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH);
#endif
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, g_goodix_dev_data->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, g_goodix_dev_data->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 1023, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 1023, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, GTP_MAX_TOUCH, 0, 0);

	return 0;
}

int goodix_sleep_mode_out(void)
{
	return 0;
}

int goodix_put_device_outof_easy_wakeup(void)
{

	struct ts_easy_wakeup_info *info = &g_goodix_dev_data->easy_wakeup_info;

	TS_LOG_DEBUG
	    ("goodix_put_device_outof_easy_wakeup  info->easy_wakeup_flag =%d\n",
	     info->easy_wakeup_flag);

	if (false == info->easy_wakeup_flag) {
		return;
	}
	info->easy_wakeup_flag = false;
	g_goodix_dev_data->easy_wakeup_info.off_motion_on = false;
	return 0;
}

static int goodix_chip_resume(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	TS_LOG_INFO("Resume start");
	switch (g_goodix_dev_data->easy_wakeup_info.sleep_mode) {
	case TS_POWER_OFF_MODE:
		goodix_sleep_mode_out();	/*exit sleep mode*/
		//schedule_work(&goodix_chip_sleep_mode_work);

		break;
	case TS_GESTURE_MODE:
		goodix_put_device_outof_easy_wakeup();
		//schedule_work(&goodix_chip_put_device_work);
		break;
	default:
		goodix_sleep_mode_out();
		//schedule_work(&goodix_chip_sleep_mode_work);

		break;
	}

	//goodix_feature_resume(ts);
	TS_LOG_INFO("Resume end");
	ts->enter_suspend = false;
	return 0;
}

static int goodix_chip_after_resume(void *feature_info)
{
	/*if (goodix_ts) {
		TS_LOG_INFO("goodix_chip_after_resume ");
		msleep(40);
		goodix_ts->rawdiff_mode = false;
		goodix_feature_resume(goodix_ts);
	}*/
	struct goodix_ts_data *ts = goodix_ts;
	goodix_chip_reset(20);
	msleep(2);
	goodix_feature_resume(ts);
	TS_LOG_INFO("after_resume");
	return 0;
}


static void goodix_sleep_mode_in(void)
{
	s8 ret = -1;
	s8 retry = 0;
	struct goodix_ts_data *ts = goodix_ts;

	//goodix_pinctrl_select_suspend(ts);
	gpio_direction_output(ts->ts_platform_data->irq_gpio, 0);
	msleep(5);

	while(retry++ < 5)
	{
	    ret = goodix_send_cmd(0x8040, GTP_CMD_SLEEP);
	    if (ret == 0)
	    {
	        TS_LOG_INFO("GTP enter sleep!");
	        return;
	    }
	    msleep(10);
	}
	TS_LOG_ERR("GTP send sleep cmd failed.");
	return;
}

static void goodix_put_device_into_easy_wakeup(void)
{
	int retval;
	s8 retry = 0;
	struct ts_easy_wakeup_info *info = &g_goodix_dev_data->easy_wakeup_info;

	TS_LOG_DEBUG
	    ("goodix_put_device_into_easy_wakeup  info->easy_wakeup_flag =%x \n",
	     info->easy_wakeup_flag);
	/*if the sleep_gesture_flag is ture,it presents that  the tp is at sleep state*/

	if (true == info->easy_wakeup_flag) {
		TS_LOG_INFO
		    ("goodix_put_device_into_easy_wakeup  info->easy_wakeup_flag =%x \n",
		     info->easy_wakeup_flag);
		return;
	}

	/*Wakeup Gesture (d0) set 1 */
	TS_LOG_INFO("Entering gesture mode.");
	while(retry++ < 5)
	{
	    retval = goodix_send_cmd(0x8046, GTP_CMD_GESTURE_WAKEUP);
	    if (retval < 0)
	    {
	        TS_LOG_ERR("failed to set doze flag into 0x8046, %d", retval);
	        goto NEXT;
	    }

	    retval = goodix_send_cmd(0x8040, GTP_CMD_GESTURE_WAKEUP);
	    if (retval < 0)
	    {
	        TS_LOG_ERR("failed to set doze flag into 0x8040, %d", retval);
	        goto NEXT;
	    }

NEXT:
	   msleep(10);
	}

	info->easy_wakeup_flag = true;
}

static int goodix_chip_suspend(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	TS_LOG_INFO("Suspend start");
	switch (g_goodix_dev_data->easy_wakeup_info.sleep_mode) {
	case TS_POWER_OFF_MODE:
		goodix_sleep_mode_in();
		break;
	case TS_GESTURE_MODE:
		TS_LOG_INFO("goodix_gesture \n");
		if (true == g_goodix_dev_data->easy_wakeup_info.palm_cover_flag)
			g_goodix_dev_data->easy_wakeup_info.palm_cover_flag = false;
		goodix_put_device_into_easy_wakeup();
		mutex_lock(&wrong_touch_lock);
		g_goodix_dev_data->easy_wakeup_info.off_motion_on = true;
		mutex_unlock(&wrong_touch_lock);
		break;
	default:
		goodix_sleep_mode_in();
		break;
	}

	msleep(58);
	TS_LOG_INFO("Suspend end");
	ts->enter_suspend = true;
	return 0;
}

int goodix_strtolow(char *src_str, size_t size)
{
	char *str = NULL;

	if (NULL == src_str)
		return -EINVAL;

	str = src_str;
	while (*str != '\0' && size > 0) {
		if (*str >= 'A' && *str <= 'Z')
			*str += ('a' - 'A');

		str++;
		size--;
	}

	return 0;
}

static int goodix_get_firmware_name(
	const char *product_name,
	char *fw_name,
	size_t size)
{
	int ret = 0;
	char vendor_name[GTP_VENDOR_NAME_LEN] = {0};
	char project_id[GTP_PROJECT_ID_LEN] = {0};

	strncpy(vendor_name, goodix_ts->vendor_name, GTP_VENDOR_NAME_LEN - 1);
	memcpy(project_id, goodix_ts->project_id, GTP_PROJECT_ID_LEN);

	goodix_strtolow(project_id, GTP_PROJECT_ID_LEN);

	ret = snprintf(fw_name, size, "ts/%s%s_%s.BIN",
		product_name, project_id, vendor_name);
	if (ret >= size) {
		TS_LOG_ERR("%s:fw name buffer out of range, ret=%d\n",
			__func__, ret);
		return -ENOMEM;
	}

	TS_LOG_ERR("%s:fw name:%s\n", __func__, fw_name);
	return 0;
}

static int goodix_get_ffbm_mode(void)
{
	char *cmdline_tp=NULL;

        cmdline_tp = strstr(saved_command_line, "androidboot.mode=ffbm");
        if(cmdline_tp != NULL)
        {
		TS_LOG_INFO("%s: ffbm mode \n", __func__);
		return 1;/* factory mode*/
        }

       TS_LOG_INFO("%s: no ffbm mode \n", __func__);
       return 0;
}

static int goodix_get_ic_firmware_version(void)
{
	int ret = 0;
       u8 buf[6] = {0};

	ret = goodix_i2c_read_dbl_check(GTP_REG_CONFIG_DATA, &goodix_ts->cfg_ver, 1);
	if (ret)
	{
		TS_LOG_ERR("%s:Read IC Config Version Error\n",__func__);
		return ret;
	}
	TS_LOG_INFO("%s: ic cfg version %d \n",__func__, goodix_ts->cfg_ver);

	ret = goodix_i2c_read(GTP_REG_VERSION, buf, sizeof(buf));
	if (ret)
	{
		TS_LOG_ERR("%s:Read IC FW Version Error\n",__func__);
		return ret;
	}

	if (goodix_ts->fw_ver)
	{
		goodix_ts->fw_ver[0] = buf[5];
		goodix_ts->fw_ver[1] = buf[4];
	}

	if (buf[5] == 0x00)
	{
	    TS_LOG_INFO("%s ic fw version: %c%c%c_%02x%02x\n", __func__, buf[0], buf[1], buf[2], buf[5], buf[4]);
	    snprintf(goodix_ts->ic_name, HARDWARE_MAX_ITEM_LONGTH,"GT%c%c%c", buf[0], buf[1], buf[2]);
	}
	else
	{
	    TS_LOG_INFO("%s ic fw version: %c%c%c%c_%02x%02x\n", __func__, buf[0], buf[1], buf[2], buf[3], buf[5], buf[4]);
	    snprintf(goodix_ts->ic_name, HARDWARE_MAX_ITEM_LONGTH,"GT%c%c%c%c", buf[0], buf[1], buf[2], buf[3] );
	}

      return 0;
}

static int goodix_fw_update_boot(char *file_name)
{
	s32 ret = 0;

	if (goodix_ts == NULL || !file_name)
		return -ENODEV;
	TS_LOG_INFO("goodix_fw_update_boot");

	/* 1. clear firmware name */
	memset(goodix_ts->auto_fw_name, '\0', GTP_FW_NAME_LEN);
	goodix_ts->fw_type = GOODIX_AUTO_UPGRADE_FW;

	/* 2. get firmware name */
	ret = goodix_get_firmware_name(file_name, goodix_ts->auto_fw_name, GTP_FW_NAME_LEN);
	if (ret) {
		TS_LOG_ERR("%s:get firmware name fail, ret=%d\n",
			__func__, ret);
#if defined (CONFIG_HUAWEI_DSM)
			if (!dsm_client_ocuppy(ts_dclient)) {
					dsm_client_record(ts_dclient,
							  "goodix get firmware name fail.\n");
					dsm_client_notify(ts_dclient,DSM_TP_FWUPDATE_ERROR_NO);
			}
			strncpy(g_goodix_dev_data->ts_platform_data->dsm_info.fw_update_result,
					"failed", strlen("failed"));
#endif
		return ret;
	}

       /* 3. check ffbm/normal mode */
	 if(goodix_get_ffbm_mode())
	{
		if(!goodix_ts->fw_error)
		return SUCCESS;

	}
       /* 4. start upgrade firmware */
	ret = gup_update_proc(UPDATE_TYPE_HEADER);
	if (ret != SUCCESS) {
		TS_LOG_ERR("%s:firmware update fail, ret=%d\n",	__func__, ret);
#if defined (CONFIG_HUAWEI_DSM)
			if (!dsm_client_ocuppy(ts_dclient)) {
					dsm_client_record(ts_dclient,
							  "goodix fw update result: failed.\nupdata_status is %d.\n",
							 g_goodix_dev_data->ts_platform_data->dsm_info.constraints_UPDATE_status);
					dsm_client_notify(ts_dclient,DSM_TP_FWUPDATE_ERROR_NO);
			}
			strncpy(g_goodix_dev_data->ts_platform_data->dsm_info.fw_update_result,
					"failed", strlen("failed"));
#endif
		goto exit;
	}

       /* 5. get ic fw version */
	ret = goodix_get_ic_firmware_version();
	if (ret) {
		TS_LOG_ERR("%s:get firmware version fail, ret=%d\n",
		__func__, ret);
	}

       /* 6. set hw info */
	ret = goodix_hardwareinfo_set();
	if (ret < 0){
	       TS_LOG_ERR("%s:hardwareinfo_set error, ret=%d\n", __func__, ret);
	       return ret;
	}

exit:
	return ret;
}

static int goodix_fw_update_sd(void)
{
	int ret =0 ;

	if (goodix_ts == NULL)
		return -ENODEV;

	/* 1. get firmware name */
	memset(goodix_ts->manual_fw_name, '\0', GTP_FW_NAME_LEN);
	memcpy(goodix_ts->manual_fw_name, GTP_FW_MANUAL_UPDATE_FILE_NAME,
		strlen(GTP_FW_MANUAL_UPDATE_FILE_NAME));

	goodix_ts->fw_type = GOODIX_MANUAL_UPGRADE_FW;

       /* 2. start upgrade firmware */
	ret = gup_update_proc(UPDATE_TYPE_HEADER);
	if (ret != SUCCESS) {
		TS_LOG_ERR("%s:firmware update fail, ret=%d\n",	__func__, ret);
#if defined (CONFIG_HUAWEI_DSM)
			if (!dsm_client_ocuppy(ts_dclient)) {
					dsm_client_record(ts_dclient,
							  "goodix fw update result: failed.\nupdata_status is %d.\n",
							 g_goodix_dev_data->ts_platform_data->dsm_info.constraints_UPDATE_status);
					dsm_client_notify(ts_dclient,DSM_TP_FWUPDATE_ERROR_NO);
			}
			strncpy(g_goodix_dev_data->ts_platform_data->dsm_info.fw_update_result,
					"failed", strlen("failed"));
#endif
		goto exit;
	}

       /* 3. get ic fw version */
	ret = goodix_get_ic_firmware_version();
	if (ret) {
		TS_LOG_ERR("%s:get firmware version fail, ret=%d\n",
		__func__, ret);
	}

       /* 4. set hw info */
	ret = goodix_hardwareinfo_set();
	if (ret < 0){
	       TS_LOG_ERR("%s:hardwareinfo_set error, ret=%d\n", __func__, ret);
	       return ret;
	}

exit:
	return ret ;
}

char *goodix_strncat(char *dest, char *src, size_t dest_size)
{
	size_t dest_len = 0;
	char *start_index = NULL;

	dest_len = strnlen(dest, dest_size);
	start_index = dest + dest_len;

	return strncat(&dest[dest_len], src, dest_size - dest_len - 1);
}
static int goodix_chip_get_info(struct ts_chip_info_param *info)
{
	size_t ic_vendor_size = 0;
	size_t fw_vendor_size = 0;

	ic_vendor_size = CHIP_INFO_LENGTH * 2;
	strncpy(info->ic_vendor, GTP_CHIP_NAME, ic_vendor_size);
	goodix_strncat(info->ic_vendor, "-", ic_vendor_size);
	goodix_strncat(info->ic_vendor, goodix_ts->project_id, ic_vendor_size);

	strncpy(info->mod_vendor, goodix_ts->vendor_name, CHIP_INFO_LENGTH);

	fw_vendor_size = CHIP_INFO_LENGTH * 2;
	snprintf(info->fw_vendor, fw_vendor_size, "%02x%02x", goodix_ts->fw_ver[0],  goodix_ts->fw_ver[1]);

	return NO_ERR;
}

int goodix_reset_select_addr(s32 ms)
{	
	int reset_gpio;
	int irq_gpio;

	if (goodix_ts == NULL)
		return -ENODEV;

	reset_gpio = goodix_ts->ts_platform_data->reset_gpio;
	irq_gpio = goodix_ts->ts_platform_data->irq_gpio;

       gpio_direction_output(reset_gpio, 0);
	msleep(ms);
	gpio_direction_output(irq_gpio, 0);	//addr=0x14  --1    //addr=0x5d --0
	msleep(2);
	gpio_direction_output(reset_gpio, 1);
	msleep(6);     //must >= 6ms
	gpio_direction_input(reset_gpio);
	return 0;
}

#if GTP_INCELL_PANEL
int goodix_write_and_readback(u16 addr, u8 * buffer, s32 len)
{
    int ret;
    u8 d[len];
    
    ret = goodix_i2c_write(addr, buffer, len);
    if (ret < 0)
        return -1;

    ret = goodix_i2c_read(addr, d, len);
    if (ret < 0 || memcmp(buffer, d, len))
        return -1;

    return 0;
}

int goodix_incell_reset(void)
{
#define RST_RETRY       5
    int ret, retry = RST_RETRY;
    u8 d[2];


    do {
    	/* select i2c address */
	goodix_reset_select_addr();

    	/* test i2c */
        ret = goodix_i2c_read(0x4220, d, 1);

    } while (--retry && ret < 0);
    
    if (ret < 0) {
        return -1;
    }

    /* Stop cpu of the touch ic */
    retry = RST_RETRY;
    do {
        d[0] = 0x0C;
        ret = goodix_write_and_readback(0x4180, d, 1);

    } while (--retry && ret < 0);
    
    if (ret < 0) {
        TS_LOG_ERR("Hold error.");
        return -1;
    }

    /* skip sensor id check. [start] */
    retry = RST_RETRY;
    do {
        d[0] = 0x00;
        ret = goodix_write_and_readback(0x4305, d, 1);
        if (ret < 0)
            continue;
        
        d[0] = 0x2B;
        d[1] = 0x24;
        ret = goodix_write_and_readback(0x42c4, d, 2);
        if (ret < 0)
            continue;
        
        d[0] = 0xE1;
        d[1] = 0xD3;
        ret = goodix_write_and_readback(0x42e4, d, 2);
        if (ret < 0)
            continue;   
        
        d[0] = 0x01;
        ret = goodix_write_and_readback(0x4305, d, 1);
        if (ret < 0)
            continue;
        else
            break;
    } while (--retry ); 

    if (!retry)
        return -1;
    /* skip sensor id check. [end] */

    /* release hold of cpu */
    retry = RST_RETRY;
    do {
        d[0] = 0x00;
        ret = goodix_write_and_readback(0x4180, d, 1);
        
    } while (--retry && ret < 0);

    if (ret < 0)
        return -1;

    return 0;

}
#endif

int goodix_chip_reset(s32 ms)
{
#if GTP_INCELL_PANEL
	int ret;
#endif
	int irq_gpio;

	if (goodix_ts == NULL)
		return -ENODEV;

	TS_LOG_INFO("Chip reset");
	irq_gpio = goodix_ts->ts_platform_data->irq_gpio;

#if GTP_INCELL_PANEL
	ret = goodix_incell_reset();
	if (ret < 0)
		return ret;
#else 
	/* select i2c address */
	goodix_reset_select_addr(ms);
#endif

	 /* int synchronization */
	gpio_direction_output(irq_gpio, 0);
	msleep(60);
	gpio_direction_input(irq_gpio);

	return goodix_init_watchdog();
}

static int goodix_glove_switch(struct ts_glove_info *info)
{
	static bool glove_en = false;
	int ret = 0;
	u8 buf = 0;

	if (!info || !goodix_ts) {
		TS_LOG_ERR("info is Null");
		return -ENOMEM;
	}

	switch (info->op_action) {
	case TS_ACTION_READ:
		if (glove_en)
			info->glove_switch = 1;
		else
			info->glove_switch = 0;
		break;
	case TS_ACTION_WRITE:
		if (info->glove_switch) {
			/* enable glove feature */
			ret = goodix_feature_switch(goodix_ts,
					TS_FEATURE_GLOVE, SWITCH_ON);
			if (!ret)
				glove_en = true;
		} else {
			/* disable glove feature */
			ret = goodix_feature_switch(goodix_ts,
					TS_FEATURE_GLOVE, SWITCH_OFF);
			if (!ret)
				glove_en = false;
		}

		if (ret < 0)
			TS_LOG_ERR("set glove switch(%d), failed : %d", buf, ret);
		break;
	default:
		TS_LOG_ERR("invalid switch status: %d", info->glove_switch);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void goodix_chip_shutdown(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return;

	goodix_power_switch(ts, SWITCH_OFF);
	goodix_release_gpio(ts->ts_platform_data);
	goodix_put_regulators(ts);
}

#if 1
static int goodix_charger_switch(struct ts_charger_info *info)
{
	int ret = 0;

	if (info == NULL)
		return -ENOMEM;

	switch (info->op_action) {
	case TS_ACTION_WRITE:
		if (info->charger_switch) {
			ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
			TS_LOG_INFO("Charger cmd switch on");
		} else {
			ret = goodix_send_cmd(GTP_CMD_CHARGER_OFF, 0x00);
			TS_LOG_INFO("Charger cmd switch off");
		}
		break;
	case TS_ACTION_READ:
		if (info->charger_switch) {
			ret = goodix_send_cfg(&goodix_ts->charger_config);
			TS_LOG_INFO("Charger cfg switch on");
		} else {
			ret = goodix_send_cfg(&goodix_ts->normal_config);
			TS_LOG_INFO("Charger cfg switch off");
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static int goodix_palm_switch(struct ts_palm_info *info)
{
	if (goodix_ts == NULL)
		return -ENODEV;

	return 0;
}

static int goodix_holster_switch(struct ts_holster_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		TS_LOG_ERR("holster_switch: info is Null\n");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
		case TS_ACTION_WRITE:
			if (info->holster_switch)
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_HOLSTER, SWITCH_ON);
			else 
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_HOLSTER, SWITCH_OFF);
			if (ret < 0)
				TS_LOG_ERR("set holster switch(%d), failed: %d",
							info->holster_switch, ret);
			break;
		case TS_ACTION_READ:
			TS_LOG_INFO("invalid holster switch(%d) action: TS_ACTION_READ",
							info->holster_switch);
			break;
		default:
			TS_LOG_INFO("invalid holster switch(%d) action: %d\n",
							info->holster_switch, info->op_action);
			ret = -EINVAL;
			break;
	}

	return ret;
}

#if 0
static int goodix_pocket_switch(struct ts_pocket_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		TS_LOG_ERR("pocket_switch: info is Null\n");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
		case TS_ACTION_WRITE:
			if (info->pocket_switch)
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_POCKET, SWITCH_ON);
			else
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_POCKET, SWITCH_OFF);
			if (ret < 0)
				TS_LOG_ERR("set pocket switch(%d), failed: %d",
							info->pocket_switch, ret);
			break;
		case TS_ACTION_READ:
			TS_LOG_INFO("invalid pocket switch(%d) action: TS_ACTION_READ",
							info->pocket_switch);
			break;
		default:
			TS_LOG_INFO("invalid pocket switch(%d) action: %d\n",
							info->pocket_switch, info->op_action);
			ret = -EINVAL;
			break;
	}

	return ret;
}
#endif

static int goodix_esdcheck_tp_reset(void)
{
	u8 esd_buf[3] = {0};
	struct goodix_ts_data *ts = goodix_ts;
	int ret = 0;

	esd_buf[0] = GTP_ESD_RESET_VALUE3;
	esd_buf[1] = GTP_ESD_RESET_VALUE3;
	esd_buf[2] = GTP_ESD_RESET_VALUE3;


	if (ts == NULL){
		TS_LOG_ERR("ts is NULL\n");
		return 0;
	}

	ret = goodix_i2c_write(GTP_ESD_RESET_REG, esd_buf, 3);
	if(ret < 0){
		TS_LOG_ERR("%s: goodix_i2c_write  fail\n",__func__);
	}
	msleep(50);

	ret = goodix_chip_reset(20);
	if(ret < 0){
		TS_LOG_ERR("%s: goodix_chip_reset  fail\n",__func__);
	}
	msleep(50);

	ret = goodix_send_cfg(&ts->normal_config);
	if(ret < 0){
		TS_LOG_ERR("%s: goodix_send_cfg  fail\n",__func__);
	}

	return 0;
}
static int goodix_esdcheck_func(void)
{
	struct goodix_ts_data *ts =  goodix_ts;
	u8 esd_buf[4] = {0};
	u8 chk_buf[4] = {0};
	int ret = 0;
	int i = 0;

	if (ts == NULL){
		TS_LOG_ERR("%s: ts is NULL \n",__func__);
		return 0;
	}

	if (ts->enter_suspend || ts->enter_update || ts->enter_rawtest){
		TS_LOG_INFO("%s: Esd suspended \n",__func__);
		return ret;
	}

	for (i = 0; i < CHECK_HW_STATUS_RETRY; i++){
		ret = goodix_i2c_read(GTP_REG_CMD, esd_buf, 2);
		if (ret < 0){
			/* IIC communication problem */
			TS_LOG_ERR("%s: goodix_i2c_read  fail!\n",__func__);
			continue;
		}else{
			if ((esd_buf[0] == GTP_CMD_ESD_CHECK) || (esd_buf[1] != GTP_CMD_ESD_CHECK)){
				/* ESD check IC works abnormally */
				goodix_i2c_read(GTP_REG_CMD, chk_buf, 2);
				TS_LOG_ERR("%s,%d:[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X",__func__,__LINE__, chk_buf[0], chk_buf[1]);
				if ((chk_buf[0] == GTP_CMD_ESD_CHECK) || (chk_buf[1] != GTP_CMD_ESD_CHECK)){
				    i = CHECK_HW_STATUS_RETRY;
				    break;
				}else{
				    continue;
				}
			}else{
				TS_LOG_DEBUG("%s: IC works normally \n",__func__);
				/* IC works normally, Write 0x8040 0xAA, feed the dog */
				esd_buf[0] = GTP_CMD_ESD_CHECK;
				ret = goodix_i2c_write(GTP_REG_CMD, esd_buf, 1);
				if(ret < 0){
					TS_LOG_ERR("%s: goodix_i2c_write  fail!\n",__func__);
					continue;
				}
				break;
			}
		}
		TS_LOG_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X\n", esd_buf[0], esd_buf[1]);
	}

	if (i >= CHECK_HW_STATUS_RETRY){
		TS_LOG_ERR("%s: IC working abnormally! Process reset guitar\n", __func__);
		goodix_esdcheck_tp_reset();
	}

	return 0;
}

/*
 * goodix_check_hw_status - Hw exception checking
 */
static int goodix_check_hw_status(void)
{
	return goodix_esdcheck_func();
}

static int goodix_regs_operate(struct ts_regs_info *info)
{
#if 0
	int retval = NO_ERR;
	unsigned int regs_addr = info->fhandler;
	u8 value[TS_MAX_REG_VALUE_NUM] = {0};
	int i = 0;

	TS_LOG_INFO("register operate test!\n");
	if(regs_addr < 0) {
		TS_LOG_ERR("get regs_addr fail, regs_addr = %d\n", regs_addr);
		return regs_addr;
	}
	switch (info->op_action) {
	case TS_ACTION_WRITE:
		for (i = 0; i < info->num; i++) {
			value[i] = info->values[i];
		}
		retval = goodix_i2c_write(regs_addr, value, info->num);

		if (retval < 0) {
			TS_LOG_ERR("TS_ACTION_WRITE error, fhandler(%d) type: %d offset: %d\n", info->fhandler, info->type, info->offset);
			retval = -EINVAL;
			goto out;
		}
		break;
	case TS_ACTION_READ:
		retval = goodix_i2c_read(regs_addr, value, info->num);
		if (retval < 0) {
			TS_LOG_ERR("TS_ACTION_READ error, fhandler(%d) type: %d offset: %d\n", info->fhandler, info->type, info->offset);
			retval = -EINVAL;
			goto out;
		}
		for (i = 0; i < info->num; i++) {
			info->values[i] = value[i];
			TS_LOG_INFO("%s : %d  value[%d] : 0x%4x\n",__func__,__LINE__, i,  value[i]);
		}
		break;
	default:
		TS_LOG_ERR("reg operate default invalid action %d\n", info->op_action);
		retval = -EINVAL;
		break;
	}
out:
	return retval;
#endif


	return NO_ERR;
}

#ifdef ROI
static int goodix_roi_switch(struct ts_roi_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		TS_LOG_ERR("roi_switch: info is Null");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
	case TS_ACTION_WRITE:
		if (info->roi_switch == 1) {
			goodix_ts->roi.enabled = true;
		} else if (info->roi_switch == 0) {
			goodix_ts->roi.enabled = false;
		} else {
			TS_LOG_ERR("Invalid roi switch value:%d", info->roi_switch);
			ret = -EINVAL;
		}
		break;
	case TS_ACTION_READ:
		// read
		break;
	default:
		break;
	}
	return ret;
}

static u8* goodix_roi_rawdata(void)
{
	u8 * rawdata_ptr = NULL;

	if (goodix_ts == NULL)
		return NULL;

	mutex_lock(&goodix_ts->roi.mutex);
	if (goodix_ts->roi.enabled && goodix_ts->roi.data_ready)
		rawdata_ptr = (u8 *)goodix_ts->roi.rawdata;
	mutex_unlock(&goodix_ts->roi.mutex);

	return rawdata_ptr;
}
#endif

#if 1
static int goodix_chip_get_capacitance_test_type(
		struct ts_test_type_info *info)
{
	int ret = 0;

	if (!info) {
		TS_LOG_INFO("info is null");
		return -EINVAL;
	}

	switch (info->op_action) {
	case TS_ACTION_READ:
		memcpy(info->tp_test_type, 
			g_goodix_dev_data->tp_test_type,TS_CAP_TEST_TYPE_LEN);
		TS_LOG_INFO("test_type= %s", info->tp_test_type);
		break;
	case TS_ACTION_WRITE:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

#if defined(CONFIG_HUAWEI_DSM)
static int goodix_dsm_debug(void)
{
	return NO_ERR;
}

#endif

int goodix_irq_top_half(struct ts_cmd_node* cmd)
{
	TS_LOG_DEBUG(" goodix_irq_top_half  \n");
	cmd->command = TS_INT_PROCESS;
	return NO_ERR;
}

static int goodix_all_chip_reset(void)
{
	return goodix_chip_reset(20);
}

struct ts_device_ops ts_goodix_ops = {
	.chip_detect = goodix_chip_detect,
	.chip_init = goodix_chip_init,
	.chip_parse_config = goodix_chip_parse_config,
	.chip_input_config = goodix_input_config,
	//.chip_slide_config = goodix_slide_config,
	.chip_irq_top_half = goodix_irq_top_half,
	.chip_irq_bottom_half = goodix_irq_bottom_half,
	.chip_reset = goodix_all_chip_reset,
	.chip_fw_update_boot = goodix_fw_update_boot,
	.chip_fw_update_sd = goodix_fw_update_sd,
	.chip_get_info = goodix_chip_get_info,
//    .chip_set_info_flag = goodix_set_info_flag,
//    .chip_before_suspend = goodix_before_suspend,
	.chip_suspend = goodix_chip_suspend,
	.chip_resume = goodix_chip_resume,
	.chip_after_resume = goodix_chip_after_resume,
	.chip_get_rawdata = goodix_get_rawdata,
	.chip_glove_switch = goodix_glove_switch,
	.chip_shutdown = goodix_chip_shutdown,
#if 1
	.chip_charger_switch = goodix_charger_switch,
#endif
	.chip_palm_switch = goodix_palm_switch,
	.chip_holster_switch = goodix_holster_switch,
	//.chip_pocket_switch = goodix_pocket_switch,
#ifdef ROI
	.chip_roi_switch = goodix_roi_switch,
	.chip_roi_rawdata = goodix_roi_rawdata,
#endif
	.chip_check_status = goodix_check_hw_status,
	.chip_regs_operate = goodix_regs_operate,

#if defined(CONFIG_HUAWEI_DSM)
	.chip_dsm_debug = goodix_dsm_debug,
#endif

#if 1
	.chip_get_capacitance_test_type =
		goodix_chip_get_capacitance_test_type,
#endif

};

static int __init goodix_core_module_init(void)
{
	int ret = NO_ERR;
	bool found = false;
	struct device_node *child = NULL;
	struct device_node *root = NULL;

	TS_LOG_INFO("%s: called\n", __func__);

	g_goodix_dev_data =
		kzalloc(sizeof(struct ts_kit_device_data), GFP_KERNEL);
	if (NULL == g_goodix_dev_data) {
		TS_LOG_ERR("%s:alloc mem for device data fail\n", __func__);
		ret = -ENOMEM;
		goto error_exit;
	}

	root = of_find_compatible_node(NULL, NULL, HUAWEI_TS_KIT);
	if (!root) {
		TS_LOG_ERR("%s:find_compatible_node error\n", __func__);
		ret = -EINVAL;
		goto error_exit;
	}

	for_each_child_of_node(root, child) {
		if (of_device_is_compatible(child, GTP_CHIP_NAME)) {
			found = true;
			break;
		}
	}

	if (!found) {
		TS_LOG_ERR("%s:device tree node not found, name=%s\n",
			__func__, GTP_CHIP_NAME);
		ret = -EINVAL;
		goto error_exit;
	}

	g_goodix_dev_data->cnode = child;
	g_goodix_dev_data->ops = &ts_goodix_ops;
	ret = huawei_ts_chip_register(g_goodix_dev_data);
	if (ret) {
		TS_LOG_ERR("%s:chip register fail, ret=%d\n", __func__, ret);
		goto error_exit;
	}

	TS_LOG_INFO("%s:success\n", __func__);
	return 0;

error_exit:
	kfree(g_goodix_dev_data);
	g_goodix_dev_data = NULL;
	TS_LOG_INFO("%s:fail\n", __func__);
	return ret;
}

static void __exit goodix_ts_module_exit(void)
{
	kfree(g_goodix_dev_data);
	g_goodix_dev_data = NULL;

	return;
}

late_initcall(goodix_core_module_init);
module_exit(goodix_ts_module_exit);
MODULE_AUTHOR("Huawei Device Company");
MODULE_DESCRIPTION("Huawei TouchScreen Driver");
MODULE_LICENSE("GPL");

