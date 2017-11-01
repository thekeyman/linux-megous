/*
 * Himax HM5065 driver.
 * Copyright (C) 2017 Ondřej Jirman <megous@megous.com>.
 *
 * Derived from hm5065.c:
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define DEBUG

#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define HM5065_PCLK_FREQ_ABS_MAX 89000000
#define HM5065_FRAME_RATE_MAX 120

/* min/typical/max system clock (xclk) frequencies */
#define HM5065_XCLK_MIN  6000000
#define HM5065_XCLK_MAX 27000000

/* {{{ Register definitions */

/* registers are assumed to be u8 unless otherwise specified */

/* device parameters */
#define HM5065_REG_DEVICE_ID			0x0000 /* u16 */
#define HM5065_REG_DEVICE_ID_VALUE		0x039e
#define HM5065_REG_FIRMWARE_VSN			0x0002
#define HM5065_REG_PATCH_VSN			0x0003
#define HM5065_REG_EXCLOCKLUT			0x0009 /* standby */

#define HM5065_REG_INT_EVENT_FLAG		0x000a
#define HM5065_REG_INT_EVENT_FLAG_OP_MODE	BIT(0)
#define HM5065_REG_INT_EVENT_FLAG_CAM_MODE	BIT(1)
#define HM5065_REG_INT_EVENT_FLAG_JPEG_STATUS	BIT(2)
#define HM5065_REG_INT_EVENT_FLAG_NUM_FRAMES	BIT(3)
#define HM5065_REG_INT_EVENT_FLAG_AF_LOCKED	BIT(4)

/* mode manager */
#define HM5065_REG_USER_COMMAND			0x0010
#define HM5065_REG_USER_COMMAND_STOP		0x00
#define HM5065_REG_USER_COMMAND_RUN		0x01
#define HM5065_REG_USER_COMMAND_POWEROFF	0x02

#define HM5065_REG_STATE			0x0011
#define HM5065_REG_STATE_RAW			0x10
#define HM5065_REG_STATE_IDLE			0x20
#define HM5065_REG_STATE_RUNNING		0x30

#define HM5065_REG_ACTIVE_PIPE_SETUP_BANK	0x0012
#define HM5065_REG_ACTIVE_PIPE_SETUP_BANK_0	0x00
#define HM5065_REG_ACTIVE_PIPE_SETUP_BANK_1	0x01

#define HM5065_REG_NUMBER_OF_FRAMES_STREAMED	0x0014 /* ro */
#define HM5065_REG_REQUIRED_STREAM_LENGTH	0x0015

#define HM5065_REG_CSI_ENABLE			0x0016 /* standby */
#define HM5065_REG_CSI_ENABLE_DISABLE		0x00
#define HM5065_REG_CSI_ENABLE_CSI2_1LANE	0x01
#define HM5065_REG_CSI_ENABLE_CSI2_2LANE	0x02

/* pipe setup bank 0 */
#define HM5065_REG_P0_SENSOR_MODE		0x0040
#define HM5065_REG_SENSOR_MODE_FULLSIZE		0x00
#define HM5065_REG_SENSOR_MODE_BINNING_2X2	0x01
#define HM5065_REG_SENSOR_MODE_BINNING_4X4	0x02
#define HM5065_REG_SENSOR_MODE_SUBSAMPLING_2X2	0x03
#define HM5065_REG_SENSOR_MODE_SUBSAMPLING_4X4	0x04

#define HM5065_REG_P0_IMAGE_SIZE		0x0041
#define HM5065_REG_IMAGE_SIZE_5MP		0x00
#define HM5065_REG_IMAGE_SIZE_UXGA		0x01
#define HM5065_REG_IMAGE_SIZE_SXGA		0x02
#define HM5065_REG_IMAGE_SIZE_SVGA		0x03
#define HM5065_REG_IMAGE_SIZE_VGA		0x04
#define HM5065_REG_IMAGE_SIZE_CIF		0x05
#define HM5065_REG_IMAGE_SIZE_QVGA		0x06
#define HM5065_REG_IMAGE_SIZE_QCIF		0x07
#define HM5065_REG_IMAGE_SIZE_QQVGA		0x08
#define HM5065_REG_IMAGE_SIZE_QQCIF		0x09
#define HM5065_REG_IMAGE_SIZE_MANUAL		0x0a

#define HM5065_REG_P0_MANUAL_HSIZE		0x0042 /* u16 */
#define HM5065_REG_P0_MANUAL_VSIZE		0x0044 /* u16 */

#define HM5065_REG_P0_DATA_FORMAT		0x0046
#define HM5065_REG_DATA_FORMAT_YCBCR_JFIF       0x00
#define HM5065_REG_DATA_FORMAT_YCBCR_REC601     0x01
#define HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM     0x02
#define HM5065_REG_DATA_FORMAT_RGB_565          0x03
#define HM5065_REG_DATA_FORMAT_RGB_565_CUSTOM   0x04
#define HM5065_REG_DATA_FORMAT_RGB_444          0x05
#define HM5065_REG_DATA_FORMAT_RGB_555          0x06
#define HM5065_REG_DATA_FORMAT_RAW10ITU10       0x07
#define HM5065_REG_DATA_FORMAT_RAW10ITU8        0x08
#define HM5065_REG_DATA_FORMAT_JPEG             0x09

#define HM5065_REG_P0_GAMMA_GAIN		0x0049 /* 0-31 */
#define HM5065_REG_P0_GAMMA_INTERPOLATION	0x004a /* 0-16 */
#define HM5065_REG_P0_PEAKING_GAIN		0x004c /* 0-63 */

#define HM5065_REG_P0_JPEG_SQUEEZE_MODE		0x004d
#define HM5065_REG_JPEG_SQUEEZE_MODE_USER	0x00
#define HM5065_REG_JPEG_SQUEEZE_MODE_AUTO	0x01

#define HM5065_REG_P0_JPEG_TARGET_FILE_SIZE	0x004e /* u16, kB */
#define HM5065_REG_P0_JPEG_IMAGE_QUALITY	0x0050
#define HM5065_REG_JPEG_IMAGE_QUALITY_HIGH	0x00
#define HM5065_REG_JPEG_IMAGE_QUALITY_MEDIUM	0x01
#define HM5065_REG_JPEG_IMAGE_QUALITY_LOW	0x02

/* pipe setup bank 1 (only register indexes) */
#define HM5065_REG_P1_SENSOR_MODE		0x0060
#define HM5065_REG_P1_IMAGE_SIZE		0x0061
#define HM5065_REG_P1_MANUAL_HSIZE		0x0062 /* u16 */
#define HM5065_REG_P1_MANUAL_VSIZE		0x0064 /* u16 */
#define HM5065_REG_P1_DATA_FORMAT		0x0066
#define HM5065_REG_P1_GAMMA_GAIN		0x0069 /* 0-31 */
#define HM5065_REG_P1_GAMMA_INTERPOLATION	0x006a /* 0-16 */
#define HM5065_REG_P1_PEAKING_GAIN		0x006c /* 0-63 */
#define HM5065_REG_P1_JPEG_SQUEEZE_MODE		0x006d
#define HM5065_REG_P1_JPEG_TARGET_FILE_SIZE	0x006e /* u16, kB */
#define HM5065_REG_P1_JPEG_IMAGE_QUALITY	0x0070

/* pipe setup - common registers */
#define HM5065_REG_CONTRAST			0x0080 /* 0-200 */
#define HM5065_REG_COLOR_SATURATION		0x0081 /* 0-200 */
#define HM5065_REG_BRIGHTNESS			0x0082 /* 0-200 */
#define HM5065_REG_HORIZONTAL_MIRROR		0x0083 /* 0,1 */
#define HM5065_REG_VERTICAL_FLIP		0x0084 /* 0,1 */

#define HM5065_REG_YCRCB_ORDER			0x0085
#define HM5065_REG_YCRCB_ORDER_CB_Y_CR_Y	0x00
#define HM5065_REG_YCRCB_ORDER_CR_Y_CB_Y	0x01
#define HM5065_REG_YCRCB_ORDER_Y_CB_Y_CR	0x02
#define HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB	0x03
#define HM5065_REG_YCRCB_ORDER_NONE		0xff

/* clock chain parameter inputs (floating point) */
#define HM5065_REG_EXTERNAL_CLOCK_FREQ_MHZ	0x00b0 /* fp16, 6-27, standby */
#define HM5065_REG_TARGET_PLL_OUTPUT	0x00b2 /* fp16, 450-1000, standby */

/* static frame rate control */
#define HM5065_REG_DESIRED_FRAME_RATE_NUM	0x00c8 /* u16 */
#define HM5065_REG_DESIRED_FRAME_RATE_DEN	0x00ca

/* static frame rate status */
#define HM5065_REG_REQUESTED_FRAME_RATE_HZ	0x00d8 /* fp16 */
#define HM5065_REG_MAX_FRAME_RATE_HZ		0x00da /* fp16 */
#define HM5065_REG_MIN_FRAME_RATE_HZ		0x00dc /* fp16 */

/* exposure controls */
#define HM5065_REG_EXPOSURE_MODE			0x0128
#define HM5065_REG_EXPOSURE_MODE_AUTO			0x00
#define HM5065_REG_EXPOSURE_MODE_COMPILED_MANUAL	0x01
#define HM5065_REG_EXPOSURE_MODE_DIRECT_MANUAL		0x02

#define HM5065_REG_EXPOSURE_METERING		0x0129
#define HM5065_REG_EXPOSURE_METERING_FLAT	0x00
#define HM5065_REG_EXPOSURE_METERING_BACKLIT	0x01
#define HM5065_REG_EXPOSURE_METERING_CENTERED	0x02

#define HM5065_REG_MANUAL_EXPOSURE_TIME_NUM	0x012a
#define HM5065_REG_MANUAL_EXPOSURE_TIME_DEN	0x012b
#define HM5065_REG_MANUAL_EXPOSURE_TIME_US	0x012c /* fp16 */
#define HM5065_REG_COLD_START_DESIRED_TIME_US	0x012e /* fp16, standby */
#define HM5065_REG_EXPOSURE_COMPENSATION	0x0130 /* s8, -7 - +7 */

#define HM5065_REG_DIRECT_MODE_COARSE_INTEGRATION_LINES	0x0132 /* u16 */
#define HM5065_REG_DIRECT_MODE_FINE_INTEGRATION_PIXELS	0x0134 /* u16 */
#define HM5065_REG_DIRECT_MODE_CODED_ANALOG_GAIN	0x0136 /* u16 */
#define HM5065_REG_DIRECT_MODE_DIGITAL_GAIN		0x0138 /* fp16 */
#define HM5065_REG_FREEZE_AUTO_EXPOSURE			0x0142 /* 0,1 */
#define HM5065_REG_USER_MAXIMUM_INTEGRATION_TIME_US	0x0143 /* fp16 */
#define HM5065_REG_ANTI_FLICKER_MODE			0x0148 /* 0,1 */

/* exposure algorithm controls */
#define HM5065_REG_DIGITAL_GAIN_FLOOR			0x015c /* fp16 */
#define HM5065_REG_DIGITAL_GAIN_CEILING			0x015e /* fp16 */

/* exposure status */
#define HM5065_REG_COARSE_INTEGRATION			0x017c /* u16 */
#define HM5065_REG_FINE_INTEGRATION_PENDING_PIXELS	0x017e /* u16 */
#define HM5065_REG_ANALOG_GAIN_PENDING			0x0180 /* fp16 */
#define HM5065_REG_DIGITAL_GAIN_PENDING			0x0182 /* fp16 */
#define HM5065_REG_DESIRED_EXPOSURE_TIME_US		0x0184 /* fp16 */
#define HM5065_REG_COMPILED_EXPOSURE_TIME_US		0x0186 /* fp16 */
#define HM5065_REG_USER_MAXIMUM_INTEGRATION_LINES	0x0189 /* u16 */
#define HM5065_REG_TOTAL_INTEGRATION_TIME_PENDING_US	0x018b /* fp16 */
#define HM5065_REG_CODED_ANALOG_GAIN_PENDING		0x018d /* u16 */

/* flicker detect */
#define HM5065_REG_FD_ENABLE_DETECT			0x0190 /* 0,1 */
#define HM5065_REG_FD_DETECTION_START			0x0191 /* 0,1 */
#define HM5065_REG_FD_MAX_NUMBER_ATTEMP	0x0192 /* 0-255, 0 = continuous */
#define HM5065_REG_FD_FLICKER_IDENTIFICATION_THRESHOLD	0x0193 /* u16 */
#define HM5065_REG_FD_WIN_TIMES				0x0195
#define HM5065_REG_FD_FRAME_RATE_SHIFT_NUMBER		0x0196
#define HM5065_REG_FD_MANUAL_FREF_ENABLE		0x0197 /* 0,1 */
#define HM5065_REG_FD_MANU_FREF_100			0x0198 /* u16 */
#define HM5065_REG_FD_MANU_FREF_120			0x019a /* u16 */
#define HM5065_REG_FD_FLICKER_FREQUENCY			0x019c /* fp16 */

/* white balance control */
#define HM5065_REG_WB_MODE			0x01a0
#define HM5065_REG_WB_MODE_OFF			0x00
#define HM5065_REG_WB_MODE_AUTOMATIC		0x01
#define HM5065_REG_WB_MODE_AUTO_INSTANT		0x02
#define HM5065_REG_WB_MODE_MANUAL_RGB		0x03
#define HM5065_REG_WB_MODE_CLOUDY_PRESET	0x04
#define HM5065_REG_WB_MODE_SUNNY_PRESET		0x05
#define HM5065_REG_WB_MODE_LED_PRESET		0x06
#define HM5065_REG_WB_MODE_FLUORESCENT_PRESET	0x07
#define HM5065_REG_WB_MODE_TUNGSTEN_PRESET	0x08
#define HM5065_REG_WB_MODE_HORIZON_PRESET	0x09

#define HM5065_REG_WB_MANUAL_RED_GAIN		0x01a1
#define HM5065_REG_WB_MANUAL_GREEN_GAIN		0x01a2
#define HM5065_REG_WB_MANUAL_BLUE_GAIN		0x01a3

#define HM5065_REG_WB_MISC_SETTINGS		0x01a4
#define HM5065_REG_WB_MISC_SETTINGS_FREEZE_ALGO	BIT(2)

#define HM5065_REG_WB_HUE_R_BIAS		0x01a5 /* fp16 */
#define HM5065_REG_WB_HUE_B_BIAS		0x01a7 /* fp16 */

#define HM5065_REG_WB_STATUS			0x01c0
#define HM5065_REG_WB_STATUS_STABLE		BIT(0)

#define HM5065_REG_WB_NORM_RED_GAIN		0x01c8 /* fp16 */
#define HM5065_REG_WB_PART_RED_GAIN		0x01e0 /* fp16 */
#define HM5065_REG_WB_PART_GREEN_GAIN		0x01e2 /* fp16 */
#define HM5065_REG_WB_PART_BLUE_GAIN		0x01e4 /* fp16 */

/* image stability status */
#define HM5065_REG_WHITE_BALANCE_STABLE		0x0291 /* 0,1 */
#define HM5065_REG_EXPOSURE_STABLE		0x0292 /* 0,1 */
#define HM5065_REG_STABLE			0x0294 /* 0,1 */

/* special effects */
#define HM5065_REG_EFFECTS_NEGATIVE		0x0380 /* 0,1 */
#define HM5065_REG_EFFECTS_SOLARISING		0x0381 /* 0,1 */
#define HM5065_REG_EFFECTS_SKECTH		0x0382 /* 0,1 */

#define HM5065_REG_EFFECTS_COLOR		0x0384
#define HM5065_REG_EFFECTS_COLOR_NORMAL         0x00
#define HM5065_REG_EFFECTS_COLOR_RED_ONLY       0x01
#define HM5065_REG_EFFECTS_COLOR_YELLOW_ONLY    0x02
#define HM5065_REG_EFFECTS_COLOR_GREEN_ONLY     0x03
#define HM5065_REG_EFFECTS_COLOR_BLUE_ONLY      0x04
#define HM5065_REG_EFFECTS_COLOR_BLACK_WHITE    0x05
#define HM5065_REG_EFFECTS_COLOR_SEPIA          0x06
#define HM5065_REG_EFFECTS_COLOR_ANTIQUE        0x07
#define HM5065_REG_EFFECTS_COLOR_AQUA           0x08
#define HM5065_REG_EFFECTS_COLOR_MANUAL_MATRIX  0x09

/* anti-vignete, otp flash (skipped), page 79-89 */

/* flash control */
#define HM5065_REG_FLASH_MODE		0x02d0 /* 0,1 */
#define HM5065_REG_FLASH_RECOMMENDED	0x02d1 /* 0,1 */

/* test pattern */
#define HM5065_REG_ENABLE_TEST_PATTERN	0x05d8 /* 0,1 */

#define HM5065_REG_TEST_PATTERN				0x05d9
#define HM5065_REG_TEST_PATTERN_NONE			0x00
#define HM5065_REG_TEST_PATTERN_HORIZONTAL_GREY_SCALE	0x01
#define HM5065_REG_TEST_PATTERN_VERTICAL_GREY_SCALE	0x02
#define HM5065_REG_TEST_PATTERN_DIAGONAL_GREY_SCALE	0x03
#define HM5065_REG_TEST_PATTERN_PN28			0x04
#define HM5065_REG_TEST_PATTERN_PN9			0x05
#define HM5065_REG_TEST_PATTERN_SOLID_COLOR		0x06
#define HM5065_REG_TEST_PATTERN_COLOR_BARS		0x07
#define HM5065_REG_TEST_PATTERN_GRADUATED_COLOR_BARS	0x08

#define HM5065_REG_TESTDATA_RED		0x4304 /* u16, 0-1023 */
#define HM5065_REG_TESTDATA_GREEN_R	0x4308 /* u16, 0-1023 */
#define HM5065_REG_TESTDATA_BLUE	0x430c /* u16, 0-1023 */
#define HM5065_REG_TESTDATA_GREEN_B	0x4310 /* u16, 0-1023 */

/* contrast stretch */
#define HM5065_REG_CS_ENABLE			0x05e8 /* 0,1 */
#define HM5065_REG_CS_GAIN_CEILING		0x05e9 /* fp16 */
#define HM5065_REG_CS_BLACK_OFFSET_CEILING	0x05eb
#define HM5065_REG_CS_WHITE_PIX_TARGET		0x05ec /* fp16 */
#define HM5065_REG_CS_BLACK_PIX_TARGET		0x05ee /* fp16 */
#define HM5065_REG_CS_ENABLED			0x05f8 /* 0,1 */
#define HM5065_REG_CS_TOTAL_PIXEL		0x05f9 /* fp16 */
#define HM5065_REG_CS_W_TARGET			0x05fb /* u32 */
#define HM5065_REG_CS_B_TARGET			0x05ff /* u32 */
#define HM5065_REG_CS_GAIN			0x0603 /* fp16 */
#define HM5065_REG_CS_BLACK_OFFSET		0x0605
#define HM5065_REG_CS_WHITE_LIMIT		0x0606

/* preset controls */
#define HM5065_REG_PRESET_LOADER_ENABLE		0x0638 /* 0,1, standby */

#define HM5065_REG_INDIVIDUAL_PRESET		0x0639 /* standby */
#define HM5065_REG_INDIVIDUAL_PRESET_ANTIVIGNETTE	BIT(0)
#define HM5065_REG_INDIVIDUAL_PRESET_WHITE_BALANCE	BIT(1)
#define HM5065_REG_INDIVIDUAL_PRESET_VCM		BIT(4)

/* jpeg control parameters*/
#define HM5065_REG_JPEG_STATUS			0x0649
#define HM5065_REG_JPEG_RESTART			0x064a
#define HM5065_REG_JPEG_HI_SQUEEZE_VALUE	0x064b /* 5-255 (5 = max q.) */
#define HM5065_REG_JPEG_MED_SQUEEZE_VALUE	0x064c /* 5-255 */
#define HM5065_REG_JPEG_LOW_SQUEEZE_VALUE	0x064d /* 5-255 */
#define HM5065_REG_JPEG_LINE_LENGTH		0x064e /* u16, standby */
#define HM5065_REG_JPEG_CLOCK_RATIO		0x0650 /* 1-8, standby */
#define HM5065_REG_JPEG_THRES			0x0651 /* u16, standby */
#define HM5065_REG_JPEG_BYTE_SENT		0x0653 /* u32 */

/* autofocus */

#define HM5065_REG_AF_WINDOWS_SYSTEM		0x065a
#define HM5065_REG_AF_WINDOWS_SYSTEM_7_ZONES	0x00
#define HM5065_REG_AF_WINDOWS_SYSTEM_1_ZONE	0x01

#define HM5065_REG_AF_H_RATIO_NUM		0x065b
#define HM5065_REG_AF_H_RATIO_DEN		0x065c
#define HM5065_REG_AF_V_RATIO_NUM		0x065d
#define HM5065_REG_AF_V_RATIO_DEN		0x065e

#define HM5065_REG_AF_RANGE			0x0709
#define HM5065_REG_AF_RANGE_FULL		0x00
#define HM5065_REG_AF_RANGE_LANDSCAPE		0x01
#define HM5065_REG_AF_RANGE_MACRO		0x02

#define HM5065_REG_AF_MODE			0x070a
#define HM5065_REG_AF_MODE_MANUAL		0x00
#define HM5065_REG_AF_MODE_CONTINUOUS		0x01
#define HM5065_REG_AF_MODE_SINGLE		0x03

#define HM5065_REG_AF_MODE_STATUS		0x0720

#define HM5065_REG_AF_COMMAND			0x070b
#define HM5065_REG_AF_COMMAND_NULL		0x00
#define HM5065_REG_AF_COMMAND_RELEASED_BUTTON	0x01
#define HM5065_REG_AF_COMMAND_HALF_BUTTON	0x02
#define HM5065_REG_AF_COMMAND_TAKE_SNAPSHOT	0x03
#define HM5065_REG_AF_COMMAND_REFOCUS		0x04

#define HM5065_REG_AF_LENS_COMMAND		0x070c
#define HM5065_REG_AF_LENS_COMMAND_NULL				0x00
#define HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_INFINITY	0x01
#define HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_MACRO		0x02
#define HM5065_REG_AF_LENS_COMMAND_GOTO_INFINITY		0x03
#define HM5065_REG_AF_LENS_COMMAND_GOTO_MACRO			0x04
#define HM5065_REG_AF_LENS_COMMAND_GOTO_RECOVERY		0x05
#define HM5065_REG_AF_LENS_COMMAND_GOTO_TARGET_POSITION		0x07
#define HM5065_REG_AF_LENS_COMMAND_GOTO_HYPERFOCAL		0x0C

#define HM5065_REG_AF_MANUAL_STEP_SIZE		0x070d
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE	0x0714
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AF	BIT(0)
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AE	BIT(1)
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AWB	BIT(2)
#define HM5065_REG_AF_FACE_LOCATION_X_START	0x0715 /* u16 */
#define HM5065_REG_AF_FACE_LOCATION_X_SIZE	0x0717 /* u16 */
#define HM5065_REG_AF_FACE_LOCATION_Y_START	0x0719 /* u16 */
#define HM5065_REG_AF_FACE_LOCATION_Y_SIZE	0x071b /* u16 */

#define HM5065_REG_AF_IN_FOCUS			0x07ae /* ro 0,1 */
#define HM5065_REG_AF_IS_STABLE			0x0725 /* ro 0,1 */

/* }}} */

/*
 * Sensor has various pre-defined PLL configurations for a set of
 * external clock frequencies.
 */
struct hm5065_clk_lut {
	unsigned long clk_freq;
	u8 lut_id;
};

static const struct hm5065_clk_lut hm5065_clk_luts[] = {
	{ .clk_freq = 12000000, .lut_id = 0x10 },
	{ .clk_freq = 13000000, .lut_id = 0x11 },
	{ .clk_freq = 13500000, .lut_id = 0x12 },
	{ .clk_freq = 14400000, .lut_id = 0x13 },
	{ .clk_freq = 18000000, .lut_id = 0x14 },
	{ .clk_freq = 19200000, .lut_id = 0x15 },
	{ .clk_freq = 24000000, .lut_id = 0x16 },
	{ .clk_freq = 26000000, .lut_id = 0x17 },
	{ .clk_freq = 27000000, .lut_id = 0x18 },
};

static const struct hm5065_clk_lut *hm5065_find_clk_lut(unsigned long freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hm5065_clk_luts); i++)
		if (hm5065_clk_luts[i].clk_freq == freq)
			return &hm5065_clk_luts[i];

	return NULL;
}

struct hm5065_frame_size {
	u32 width;
	u32 height;
};

/* must be sorted by frame area */
static const struct hm5065_frame_size hm5065_frame_sizes[] = {
	{ .width = 2592, .height = 1944, },
	{ .width = 1920, .height = 1080, },
	{ .width = 1600, .height = 1200, },
	{ .width = 1280, .height = 1024, },
	{ .width = 1280, .height = 720, },
	{ .width = 1024, .height = 768, },
	{ .width = 1024, .height = 600, },
	{ .width = 800, .height = 600, },
	{ .width = 640, .height = 480, },
	{ .width = 352, .height = 288, },
	{ .width = 320, .height = 240, },
	{ .width = 176, .height = 144, },
	{ .width = 160, .height = 120, },
	{ .width = 88, .height = 72, },
};

#define HM5065_NUM_FRAME_SIZES ARRAY_SIZE(hm5065_frame_sizes)
#define HM5065_DEFAULT_FRAME_SIZE 4

struct hm5065_pixfmt {
	u32 code;
	u32 colorspace;
	u8 data_fmt;
	u8 ycbcr_order;
};

//XXX: identify colrorspace correctly, see datasheet page 40
static const struct hm5065_pixfmt hm5065_formats[] = {
	{
		.code              = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_JFIF,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_CB_Y_CR_Y,
	},
	{
		.code              = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_JFIF,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_CR_Y_CB_Y,
	},
	{
		.code              = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_JFIF,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CB_Y_CR,
	},
	{
		.code              = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_JFIF,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
	},
	{
		.code              = MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RGB_555,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_NONE,
	},
	{
		.code              = MEDIA_BUS_FMT_RGB565_2X8_BE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RGB_565,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_NONE,
	},
};

#define HM5065_NUM_FORMATS ARRAY_SIZE(hm5065_formats)

static const struct hm5065_pixfmt *hm5065_find_format(u32 code)
{
	int i;

	for (i = 0; i < HM5065_NUM_FORMATS; i++)
		if (hm5065_formats[i].code == code)
			return &hm5065_formats[i];

	return NULL;
}

/* regulator supplies */
static const char * const hm5065_supply_name[] = {
	"IOVDD", /* Digital I/O (2.8V) suppply */
	"AFVDD",  /* Autofocus (2.8V) supply */
	"DVDD",  /* Digital Core (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
};

#define HM5065_NUM_SUPPLIES ARRAY_SIZE(hm5065_supply_name)

struct hm5065_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct {
		struct v4l2_ctrl *wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *colorfx;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *test_data[4];
};

struct hm5065_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* external clock for HM5065 */
	u32 max_pixel_rate; /* how many pixels we can output per second */

	struct regulator_bulk_data supplies[HM5065_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio; // nrst pin
	struct gpio_desc *chipenable_gpio; // ce pin

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
	struct v4l2_fract frame_interval;
	struct hm5065_ctrls ctrls;

	bool pending_mode_change;
	bool powered;
	bool streaming;
};

static inline struct hm5065_dev *to_hm5065_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hm5065_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct hm5065_dev,
			     ctrls.handler)->sd;
}

/* {{{ Register access helpers */

static int hm5065_write_regs(struct hm5065_dev *sensor, u16 start_index,
			     u8 *data, int data_size)
{
	struct i2c_client *client = sensor->i2c_client;
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

	v4l2_info(&sensor->sd, "wr: %04x <= %*ph\n", (u32)start_index,
		  data_size, data);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l2_err(&sensor->sd,
			 "%s: error %d: start_index=%x, data=%*ph\n",
			 __func__, ret, (u32)start_index, data_size, data);
		return ret;
	}

	return 0;
}

static int hm5065_read_regs(struct hm5065_dev *sensor, u16 start_index,
			    u8 *data, int data_size)
{
	struct i2c_client *client = sensor->i2c_client;
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
		v4l2_err(&sensor->sd,
			 "%s: error %d: start_index=%x, data_size=%d\n",
			 __func__, ret, (u32)start_index, data_size);
		return ret;
	}

	v4l2_info(&sensor->sd, "rd: %04x => %*ph\n", (u32)start_index,
		  data_size, data);

	return 0;
}

static int hm5065_read(struct hm5065_dev *sensor, u16 reg, u8 *val)
{
	return hm5065_read_regs(sensor, reg, val, 1);
}

static int hm5065_write(struct hm5065_dev *sensor, u16 reg, u8 val)
{
	return hm5065_write_regs(sensor, reg, &val, 1);
}

static int hm5065_read16(struct hm5065_dev *sensor, u16 reg, u16 *val)
{
	int ret;

	ret = hm5065_read_regs(sensor, reg, (u8 *)val, sizeof(*val));
	if (ret)
		return ret;

	*val = be16_to_cpu(*val);
	return 0;
}

static int hm5065_write16(struct hm5065_dev *sensor, u16 reg, u16 val)
{
	u16 tmp = cpu_to_be16(val);

	return hm5065_write_regs(sensor, reg, (u8 *)&tmp, sizeof(tmp));
}

static int hm5065_read32(struct hm5065_dev *sensor, u16 reg, u32 *val)
{
	int ret;

	ret = hm5065_read_regs(sensor, reg, (u8 *)val, sizeof(*val));
	if (ret)
		return ret;

	*val = be32_to_cpu(*val);
	return 0;
}

static int hm5065_write32(struct hm5065_dev *sensor, u16 reg, u32 val)
{
	u32 tmp = cpu_to_be32(val);

	return hm5065_write_regs(sensor, reg, (u8 *)&tmp, sizeof(tmp));
}

/*
 * Sensor uses ST Float900 format to represent floating point numbers.
 * Binary floating point number: * (s ? -1 : 0) * 1.mmmmmmmmm * 2^eeeeee
 *
 * Following functions convert long value to and from the floating point format.
 *
 * Example:
 * mili variant: val = 123456 => fp_val = 123.456
 * micro variant: val = -12345678 => fp_val = -12.345678
 */
static long hm5065_mili_from_fp16(u16 fp_val)
{
	long val;
	long mantisa = fp_val & 0x1ff;
	int exp = ((int)(fp_val >> 9) & 0x3f) - 31;

	val = (1000 * (mantisa | 0x200));
	if (exp > 0)
		val <<= exp;
	else if (exp < 0)
		val >>= -exp;
	val >>= 9;

	if (fp_val & 0x8000)
		val = -val;

	return val;
}

static u16 hm5065_mili_to_fp16(s32 val)
{
	int fls;
	u16 e, m, s = 0;
	u64 v, rem;

	if (val == 0)
		return 0;

	if (val < 0) {
		val = -val;
		s = 0x8000;
	}

	v = (u64)val * 1024;
	rem = do_div(v, 1000);
	if (rem >= 500)
		 v++;

	fls = fls64(v) - 1;
	e = 31 + fls - 10;
	m = fls > 9 ? v >> (fls - 9) : v << (9 - fls);

	return s | (m & 0x1ff) | (e << 9);
}

/* }}} */
/* {{{ Controls */

static int hm5065_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	//int val;

	/* v4l2_ctrl_lock() locks our own mutex */

	if (!sensor->powered)
		return -EIO;

	switch (ctrl->id) {
#if 0
	case V4L2_CID_AUTOGAIN:
		if (!ctrl->val)
			return 0;
		val = hm5065_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val == V4L2_EXPOSURE_MANUAL)
			return 0;
		val = hm5065_get_exposure(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.exposure->val = val;
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static const u8 hm5065_wb_opts[][2] = {
	{ V4L2_WHITE_BALANCE_INCANDESCENT, HM5065_REG_WB_MODE_TUNGSTEN_PRESET },
	{ V4L2_WHITE_BALANCE_FLUORESCENT,
		HM5065_REG_WB_MODE_FLUORESCENT_PRESET },
	{ V4L2_WHITE_BALANCE_HORIZON, HM5065_REG_WB_MODE_HORIZON_PRESET },
	{ V4L2_WHITE_BALANCE_CLOUDY, HM5065_REG_WB_MODE_CLOUDY_PRESET },
	{ V4L2_WHITE_BALANCE_DAYLIGHT, HM5065_REG_WB_MODE_SUNNY_PRESET },
	{ V4L2_WHITE_BALANCE_AUTO, HM5065_REG_WB_MODE_AUTOMATIC },
};

static int hm5065_set_power_line_frequency(struct hm5065_dev *sensor, s32 val)
{
	u16 freq;
	int ret;

	switch (val) {
	case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		ret = hm5065_write(sensor, HM5065_REG_ANTI_FLICKER_MODE, 0);
		if (ret)
			return ret;

		return hm5065_write(sensor, HM5065_REG_FD_ENABLE_DETECT, 0);
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		ret = hm5065_write(sensor, HM5065_REG_ANTI_FLICKER_MODE, 1);
		if (ret)
			return ret;

		ret = hm5065_write(sensor, HM5065_REG_FD_ENABLE_DETECT, 0);
		if (ret)
			return ret;

		freq = val == V4L2_CID_POWER_LINE_FREQUENCY_50HZ ?
			0x4b20 : 0x4bc0;

		return hm5065_write16(sensor, HM5065_REG_FD_FLICKER_FREQUENCY,
				      freq);
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		ret = hm5065_write(sensor, HM5065_REG_FD_ENABLE_DETECT, 1);
		if (ret)
			return ret;

		ret = hm5065_write(sensor, HM5065_REG_ANTI_FLICKER_MODE, 1);
		if (ret)
			return ret;

		return hm5065_write(sensor, HM5065_REG_FD_DETECTION_START, 1);
	default:
		return -EINVAL;
	}
}

static int hm5065_set_colorfx(struct hm5065_dev *sensor, s32 val)
{
	int ret;

	ret = hm5065_write(sensor, HM5065_REG_EFFECTS_COLOR,
			   HM5065_REG_EFFECTS_COLOR_NORMAL);
	if (ret)
		return ret;

	ret = hm5065_write(sensor, HM5065_REG_EFFECTS_NEGATIVE, 0);
	if (ret)
		return ret;

	ret = hm5065_write(sensor, HM5065_REG_EFFECTS_SOLARISING, 0);
	if (ret)
		return ret;

	ret = hm5065_write(sensor, HM5065_REG_EFFECTS_SKECTH, 0);
	if (ret)
		return ret;

	switch (val) {
	case V4L2_COLORFX_NONE:
		return 0;
	case V4L2_COLORFX_NEGATIVE:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_NEGATIVE, 1);
	case V4L2_COLORFX_SOLARIZATION:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_SOLARISING, 1);
	case V4L2_COLORFX_SKETCH:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_SKECTH, 1);
	case V4L2_COLORFX_ANTIQUE:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_COLOR,
				    HM5065_REG_EFFECTS_COLOR_ANTIQUE);
	case V4L2_COLORFX_SEPIA:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_COLOR,
				    HM5065_REG_EFFECTS_COLOR_SEPIA);
	case V4L2_COLORFX_AQUA:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_COLOR,
				    HM5065_REG_EFFECTS_COLOR_AQUA);
	case V4L2_COLORFX_BW:
		return hm5065_write(sensor, HM5065_REG_EFFECTS_COLOR,
				    HM5065_REG_EFFECTS_COLOR_BLACK_WHITE);
	default:
		return -EINVAL;
	}
}

static int hm5065_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	unsigned int i;
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (!sensor->powered)
		return 0;

	switch (ctrl->id) {
#if 0
	case V4L2_CID_AUTOGAIN:
		return hm5065_set_ctrl_gain(sensor, ctrl->val);
	case V4L2_CID_EXPOSURE_AUTO:
		return hm5065_set_ctrl_exposure(sensor, ctrl->val);
	case V4L2_CID_HUE:
		return hm5065_set_ctrl_hue(sensor, ctrl->val);
#endif

	case V4L2_CID_CONTRAST:
		return hm5065_write(sensor, HM5065_REG_CONTRAST, ctrl->val);

	case V4L2_CID_SATURATION:
		return hm5065_write(sensor, HM5065_REG_COLOR_SATURATION,
				    ctrl->val);

	case V4L2_CID_BRIGHTNESS:
		return hm5065_write(sensor, HM5065_REG_BRIGHTNESS, ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return hm5065_set_power_line_frequency(sensor, ctrl->val);

	case V4L2_CID_GAMMA:
		return hm5065_write(sensor, HM5065_REG_P0_GAMMA_GAIN,
				    ctrl->val);

	case V4L2_CID_VFLIP:
		return hm5065_write(sensor, HM5065_REG_VERTICAL_FLIP,
				    ctrl->val ? 1 : 0);

	case V4L2_CID_HFLIP:
		return hm5065_write(sensor, HM5065_REG_HORIZONTAL_MIRROR,
				    ctrl->val ? 1 : 0);

	case V4L2_CID_COLORFX:
		return hm5065_set_colorfx(sensor, ctrl->val);

	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		for (i = 0; i < ARRAY_SIZE(hm5065_wb_opts); i++) {
			if (hm5065_wb_opts[i][0] != ctrl->val)
				continue;

			return hm5065_write(sensor, HM5065_REG_WB_MODE,
					    hm5065_wb_opts[i][1]);
		}

		return -EINVAL;

	case V4L2_CID_TEST_PATTERN_RED:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_RED,
				      ctrl->val);

	case V4L2_CID_TEST_PATTERN_GREENR:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_GREEN_R,
				      ctrl->val);

	case V4L2_CID_TEST_PATTERN_BLUE:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_BLUE,
				      ctrl->val);

	case V4L2_CID_TEST_PATTERN_GREENB:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_GREEN_B,
				      ctrl->val);

	case V4L2_CID_TEST_PATTERN:
		for (i = 0; i < ARRAY_SIZE(ctrls->test_data); i++)
			v4l2_ctrl_activate(ctrls->test_data[i],
					   ctrl->val == 6); /* solid color */

		ret = hm5065_write(sensor, HM5065_REG_ENABLE_TEST_PATTERN,
				   ctrl->val == 0 ? 0 : 1);
		if (ret)
			return ret;

		return hm5065_write(sensor, HM5065_REG_TEST_PATTERN, ctrl->val);

	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops hm5065_ctrl_ops = {
	.g_volatile_ctrl = hm5065_g_volatile_ctrl,
	.s_ctrl = hm5065_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Horizontal gray scale",
	"Vertical gray scale",
	"Diagonal gray scale",
	"PN28",
	"PN9",
	"Solid color",
	"Color bars",
	"Graduated color bars",
};

static int hm5065_init_controls(struct hm5065_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &hm5065_ctrl_ops;
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	u8 wb_max = 0;
	u64 wb_mask = 0;
	unsigned int i;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	for (i = 0; i < ARRAY_SIZE(hm5065_wb_opts); i++) {
		if (wb_max < hm5065_wb_opts[i][0])
			wb_max = hm5065_wb_opts[i][0];
		wb_mask |= BIT(hm5065_wb_opts[i][0]);
	}

	ctrls->wb = v4l2_ctrl_new_std_menu(hdl, ops,
			V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
			wb_max, ~wb_mask, V4L2_WHITE_BALANCE_AUTO);

	ctrls->colorfx =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_COLORFX,
				       15, ~(
					     BIT(V4L2_COLORFX_NONE) |
					     BIT(V4L2_COLORFX_NEGATIVE) |
					     BIT(V4L2_COLORFX_SOLARIZATION) |
					     BIT(V4L2_COLORFX_SKETCH) |
					     BIT(V4L2_COLORFX_SEPIA) |
					     BIT(V4L2_COLORFX_ANTIQUE) |
					     BIT(V4L2_COLORFX_AQUA) |
					     BIT(V4L2_COLORFX_BW)
				       ), V4L2_COLORFX_NONE);

	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_VFLIP, 0, 1, 1, 0);

#if 0
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4095, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4095, 1, 0);
	/* Auto/manual exposure */
	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
						 V4L2_CID_EXPOSURE_AUTO,
						 V4L2_EXPOSURE_MANUAL, 0,
						 V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 65535, 1, 0);
	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN,
					     0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 1023, 1, 0);

	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE,
				       //0, 359, 1, 0);
#endif

	ctrls->brightness = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS,
					      0, 200, 1, 100);
	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 200, 1, 118);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 200, 1, 115);

	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);
	for (i = 0; i < ARRAY_SIZE(ctrls->test_data); i++)
		ctrls->test_data[i] =
			v4l2_ctrl_new_std(hdl, ops,
					  V4L2_CID_TEST_PATTERN_RED + i,
					  0, 1023, 1, 0);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

#if 0
	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(3, &ctrls->wb, 0, false);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);
	v4l2_ctrl_auto_cluster(2, &ctrls->auto_exp, 1, true);
#endif

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

/* }}} */
/* {{{ Video ops */

static int hm5065_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int hm5065_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret = 0, frame_rate, max_frame_rate;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	/* user requested infinite frame rate */
	if (fi->interval.numerator == 0)
		frame_rate = HM5065_FRAME_RATE_MAX;
	else
		frame_rate = fi->interval.denominator / fi->interval.numerator;

	frame_rate = clamp(frame_rate, 1, HM5065_FRAME_RATE_MAX);

	/* check if requested frame rate is supported */
	max_frame_rate =
		sensor->max_pixel_rate / sensor->fmt.width / sensor->fmt.height;
	if (frame_rate > max_frame_rate)
		frame_rate = max_frame_rate;

	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = frame_rate;
	sensor->pending_mode_change = true;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int hm5065_setup_mode(struct hm5065_dev *sensor)
{
	int ret;
	const struct hm5065_pixfmt *pix_fmt;

	dev_info(&sensor->i2c_client->dev, "set mode\n");

	ret = hm5065_write(sensor, HM5065_REG_P0_SENSOR_MODE,
			   HM5065_REG_SENSOR_MODE_FULLSIZE);
	if (ret)
		return ret;

	dev_info(&sensor->i2c_client->dev, "set img width\n");

	ret = hm5065_write16(sensor, HM5065_REG_P0_MANUAL_HSIZE,
			     sensor->fmt.width);
	if (ret)
		return ret;

	dev_info(&sensor->i2c_client->dev, "set img height\n");

	ret = hm5065_write16(sensor, HM5065_REG_P0_MANUAL_VSIZE,
			     sensor->fmt.height);
	if (ret)
		return ret;

	ret = hm5065_write(sensor, HM5065_REG_P0_IMAGE_SIZE,
			   HM5065_REG_IMAGE_SIZE_MANUAL);
	if (ret)
		return ret;

	pix_fmt = hm5065_find_format(sensor->fmt.code);
	if (!pix_fmt) {
		dev_err(&sensor->i2c_client->dev,
			"pixel format not supported %u\n",
			sensor->fmt.code);
		return -EINVAL;
	}

	dev_info(&sensor->i2c_client->dev, "set P0 data fmt\n");

	ret = hm5065_write(sensor, HM5065_REG_P0_DATA_FORMAT,
			   pix_fmt->data_fmt);
	if (ret)
		return ret;

	if (pix_fmt->ycbcr_order != HM5065_REG_YCRCB_ORDER_NONE) {
		dev_info(&sensor->i2c_client->dev, "set P0 ycbcr order\n");

		ret = hm5065_write(sensor, HM5065_REG_YCRCB_ORDER,
				   pix_fmt->ycbcr_order);
		if (ret)
			return ret;
	}

	dev_info(&sensor->i2c_client->dev, "set frame rate\n");

	ret = hm5065_write16(sensor, HM5065_REG_DESIRED_FRAME_RATE_NUM,
			     sensor->frame_interval.denominator);
	if (ret)
		return ret;

	return 0;
}

static int hm5065_set_stream(struct hm5065_dev *sensor, int enable)
{
	dev_info(&sensor->i2c_client->dev, "stream cmd\n");

	return hm5065_write(sensor, HM5065_REG_USER_COMMAND, enable ?
			    HM5065_REG_USER_COMMAND_RUN :
			    HM5065_REG_USER_COMMAND_STOP);
}

static int hm5065_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !enable) {
		if (enable && sensor->pending_mode_change) {
			ret = hm5065_setup_mode(sensor);
			if (ret)
				goto out;
		}

		ret = hm5065_set_stream(sensor, enable);
		if (ret)
			goto out;

		sensor->streaming = !!enable;
	}

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

/* }}} */
/* {{{ Pad ops */

static int hm5065_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= HM5065_NUM_FORMATS)
		return -EINVAL;

	code->code = hm5065_formats[code->index].code;

	return 0;
}

static int hm5065_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index >= HM5065_NUM_FRAME_SIZES)
		return -EINVAL;

	fse->min_width = fse->max_width =
		hm5065_frame_sizes[fse->index].width;
	fse->min_height = fse->max_height =
		hm5065_frame_sizes[fse->index].height;

	return 0;
}

#if 0
// enumerate discrete frame intervals? do we need this?
// set_fmt/s_frame_interval can handle this
static int hm5065_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	//struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct v4l2_fract tpf;
	//int ret;

	if (fie->pad != 0)
		return -EINVAL;

	if (fie->index > 0)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = 15;

	/*
	 * fps = DIV_ROUND_CLOSEST(fi->denominator, fi->numerator);
	 * fi->numerator = 1;
	 * if (fps > maxfps)
	 *	fi->denominator = maxfps;
	 * else if (fps < minfps)
	 *	fi->denominator = minfps;
	 */

	fie->interval = tpf;
	return 0;
}
#endif

static int hm5065_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct v4l2_mbus_framefmt *mf;

	if (format->pad != 0)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, format->pad);
		format->format = *mf;
		return 0;
	}

	mutex_lock(&sensor->lock);
	format->format = sensor->fmt;
	mutex_unlock(&sensor->lock);

	return 0;
}

// https://static.lwn.net/kerneldoc/media/uapi/v4l/vidioc-subdev-g-fmt.html
static int hm5065_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct hm5065_pixfmt *pixfmt;
	int ret = 0, i;
	u32 max_frame_area;

	if (format->pad != 0)
		return -EINVAL;

	/* check if we support requested mbus fmt */
	pixfmt = hm5065_find_format(mf->code);
	if (!pixfmt)
		pixfmt = &hm5065_formats[0];

	mf->code = pixfmt->code;
	mf->colorspace = pixfmt->colorspace;
	mf->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	mf->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mf->quantization = V4L2_QUANTIZATION_DEFAULT;

	mf->field = V4L2_FIELD_NONE;

	v4l2_info(sd, "search for %ux%u\n", mf->width, mf->height);

	mutex_lock(&sensor->lock);

	/* find highest resolution possible for the currently used frame rate */
	max_frame_area = sensor->max_pixel_rate /
		sensor->frame_interval.denominator *
		sensor->frame_interval.numerator;

	for (i = 0; i < HM5065_NUM_FRAME_SIZES; i++) {
		const struct hm5065_frame_size *fs = &hm5065_frame_sizes[i];
		u32 predefined_frame_area = fs->width * fs->height;

		if (predefined_frame_area <= max_frame_area &&
			fs->width <= mf->width &&
			fs->height <= mf->height)
			break;
	}

	if (i == HM5065_NUM_FRAME_SIZES) {
		v4l2_warn(sd, "frame size not found, using the smallest one\n");
		i--;
	}

	mf->width = hm5065_frame_sizes[i].width;
	mf->height = hm5065_frame_sizes[i].height;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_mf;

		try_mf = v4l2_subdev_get_try_format(sd, cfg, format->pad);
		*try_mf = *mf;
		goto out;
	}

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	sensor->fmt = *mf;
	sensor->pending_mode_change = true;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

/* }}} */
/* {{{ Core Ops */

static int hm5065_log_status(struct v4l2_subdev *sd)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	u8 buf[256];
	int ret, i;

	if (!sensor->powered)
		return -EIO;

	ret = hm5065_read_regs(sensor, 0, buf, sizeof(buf));
	if (ret)
		return -EIO;

	v4l2_info(sd, "HM5065 registers:\n");
	for (i = 0; i < sizeof(buf); i++)
		v4l2_info(sd, "%04x: %02x\n", i, buf[i]);

#if 0
	v4l2_info(sd, "version: %u\n", state->chip_version);
	v4l2_info(sd, "power: %s\n", (reg_03 & 0x0c) ? "off" : "on");
	v4l2_info(sd, "reset: %s\n", (reg_03 & 0x01) ? "off" : "on");
	v4l2_info(sd, "test pattern: %s\n",
		  (reg_03 & 0x20) ? "enabled" : "disabled");
	//v4l2_info(sd, "format: %ux%u\n",);
#endif
	return 0;
}

static void hm5065_chip_enable(struct hm5065_dev *sensor, bool enable)
{
	dev_dbg(&sensor->i2c_client->dev, "%s: ce=%d\n", __func__, !!enable);

	gpiod_set_value(sensor->chipenable_gpio, enable ? 1 : 0);
	gpiod_set_value(sensor->reset_gpio, enable ? 0 : 1);
}

#if 0
static void hm5065_reset(struct hm5065_dev *sensor)
{
	/* if reset pin is not used, we will use CE for reset */
	if (sensor->reset_gpio) {
		gpiod_set_value(sensor->reset_gpio, 1);
		usleep_range(1000, 2000);
		gpiod_set_value(sensor->reset_gpio, 0);
	} else {
		gpiod_set_value(sensor->chipenable_gpio, 0);
		usleep_range(1000, 2000);
		gpiod_set_value(sensor->chipenable_gpio, 1);
	}

	usleep_range(30000, 40000);
}
#endif

static int hm5065_configure(struct hm5065_dev *sensor)
{
	int ret;
	u16 device_id;
	const struct hm5065_clk_lut *lut;
	unsigned long xclk_freq;

	dev_dbg(&sensor->i2c_client->dev, "%s: read device id\n", __func__);

	ret = hm5065_read16(sensor, HM5065_REG_DEVICE_ID, &device_id);
	if (ret)
		return ret;

	dev_dbg(&sensor->i2c_client->dev, "%s: got device id 0x%04x\n",
		__func__, (unsigned int)device_id);

	if (device_id != HM5065_REG_DEVICE_ID_VALUE) {
		dev_err(&sensor->i2c_client->dev,
			"unsupported device id: 0x%04x\n",
			(unsigned int)device_id);
		return -EINVAL;
	}

	xclk_freq = clk_get_rate(sensor->xclk);
#if 1
	lut = hm5065_find_clk_lut(xclk_freq);
	if (!lut) {
		dev_err(&sensor->i2c_client->dev,
			"xclk frequency out of range: %lu Hz\n", xclk_freq);
		return -EINVAL;
	}

	dev_info(&sensor->i2c_client->dev, "set exclklut\n");

	ret = hm5065_write(sensor, HM5065_REG_EXCLOCKLUT, lut->lut_id);
	if (ret)
		return ret;
#else
	ret = hm5065_write16(sensor, HM5065_REG_EXTERNAL_CLOCK_FREQ_MHZ,
			     0x4700);
	if (ret)
		return ret;

	ret = hm5065_write16(sensor, HM5065_REG_TARGET_PLL_OUTPUT, 0x4b20);
	if (ret)
		return ret;
#endif

	return 0;
}

static int hm5065_set_power(struct hm5065_dev *sensor, bool on)
{
	int ret = 0;

	dev_dbg(&sensor->i2c_client->dev, "%s: on=%u\n", __func__, on);

	if (on) {
		ret = regulator_bulk_enable(HM5065_NUM_SUPPLIES,
					    sensor->supplies);
		if (ret)
			return ret;

		ret = clk_prepare_enable(sensor->xclk);
		if (ret)
			goto power_off;

		ret = clk_set_rate(sensor->xclk, 24000000);
		if (ret)
			goto xclk_off;

		usleep_range(1000, 2000);
		hm5065_chip_enable(sensor, false);
		usleep_range(1000, 2000);
		hm5065_chip_enable(sensor, true);
		usleep_range(50000, 70000);

		ret = hm5065_configure(sensor);
		if (ret)
			goto xclk_off;

		ret = hm5065_setup_mode(sensor);
		if (ret)
			goto xclk_off;

		return 0;
	}

xclk_off:
	clk_disable_unprepare(sensor->xclk);
power_off:
	hm5065_chip_enable(sensor, false);
	regulator_bulk_disable(HM5065_NUM_SUPPLIES, sensor->supplies);
	return ret;
}

static int hm5065_s_power(struct v4l2_subdev *sd, int on)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	bool power_up, power_down;
	int ret = 0;

	mutex_lock(&sensor->lock);

	power_up = on && !sensor->powered;
	power_down = !on && sensor->powered;

	if (power_up || power_down) {
		ret = hm5065_set_power(sensor, power_up);
		if (!ret)
			sensor->powered = on;
	}

	mutex_unlock(&sensor->lock);

	if (!ret && power_up) {
		/* restore controls */
		ret = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	}

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int hm5065_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret;

	if (reg->reg > 0xffff)
		return -EINVAL;

	if (reg->size == 1) {
		u8 val = 0;

		ret = hm5065_read(sensor, (u16)reg->reg, &val);
		if (ret)
			return -EIO;
		reg->val = val;
	} else if (reg->size == 2) {
		u16 val = 0;

		ret = hm5065_read16(sensor, (u16)reg->reg, &val);
		if (ret)
			return -EIO;
		reg->val = val;
	} else if (reg->size == 4) {
		u32 val = 0;

		ret = hm5065_read32(sensor, (u16)reg->reg, &val);
		if (ret)
			return -EIO;
		reg->val = val;
	} else
		return -EINVAL;

	return 0;
}

static int hm5065_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);

	if (reg->reg > 0xffff)
		return -EINVAL;

	if (reg->size == 1 && reg->val <= 0xff)
		return hm5065_write(sensor, (u16)reg->reg, reg->val);
	else if (reg->size == 2 && reg->val <= 0xffff)
		return hm5065_write16(sensor, (u16)reg->reg, reg->val);
	else if (reg->size == 4 && reg->val <= 0xffffffffull)
		return hm5065_write32(sensor, (u16)reg->reg, reg->val);
	else
		return -EINVAL;

	return 0;
}
#endif

/* }}} */

static const struct v4l2_subdev_core_ops hm5065_core_ops = {
	.log_status = hm5065_log_status,
	.s_power = hm5065_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = hm5065_g_register,
	.s_register = hm5065_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops hm5065_pad_ops = {
	.enum_mbus_code = hm5065_enum_mbus_code,
	.enum_frame_size = hm5065_enum_frame_size,
	//.enum_frame_interval = hm5065_enum_frame_interval,
	.get_fmt = hm5065_get_fmt,
	.set_fmt = hm5065_set_fmt,
};

static const struct v4l2_subdev_video_ops hm5065_video_ops = {
	.g_frame_interval = hm5065_g_frame_interval,
	.s_frame_interval = hm5065_s_frame_interval,
	.s_stream = hm5065_s_stream,
};

static const struct v4l2_subdev_ops hm5065_subdev_ops = {
	.core = &hm5065_core_ops,
	.pad = &hm5065_pad_ops,
	.video = &hm5065_video_ops,
};

static int hm5065_get_regulators(struct hm5065_dev *sensor)
{
	int i;

	for (i = 0; i < HM5065_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = hm5065_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       HM5065_NUM_SUPPLIES,
				       sensor->supplies);
}

static int hm5065_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct hm5065_dev *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	sensor->fmt.code = hm5065_formats[0].code;
	sensor->fmt.width = hm5065_frame_sizes[HM5065_DEFAULT_FRAME_SIZE].width;
	sensor->fmt.height =
		hm5065_frame_sizes[HM5065_DEFAULT_FRAME_SIZE].height;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = 15;
	sensor->pending_mode_change = true;

	endpoint = fwnode_graph_get_next_endpoint(
		of_fwnode_handle(client->dev.of_node), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		return ret;
	}

	/* we don't know how to configure the camera for PARALLEL mode */
	if (sensor->ep.bus_type != V4L2_MBUS_BT656) {
		dev_err(dev, "invalid bus type, must be BT.656\n");
		return -EINVAL;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

	sensor->max_pixel_rate = HM5065_PCLK_FREQ_ABS_MAX * 10 / 22;

	sensor->chipenable_gpio = devm_gpiod_get_optional(dev, "chipenable",
						    GPIOD_OUT_LOW);
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	if (!sensor->chipenable_gpio && !sensor->reset_gpio) {
		dev_err(dev,
			"either chip enable or reset pin must be configured\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &hm5065_subdev_ops);

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = hm5065_get_regulators(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = hm5065_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;

	dev_err(dev, "sensor regsitered\n");

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static int hm5065_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

	return 0;
}

static const struct i2c_device_id hm5065_id[] = {
	{"hm5065", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, hm5065_id);

static const struct of_device_id hm5065_dt_ids[] = {
	{ .compatible = "himax,hm5065" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hm5065_dt_ids);

static struct i2c_driver hm5065_i2c_driver = {
	.driver = {
		.name  = "hm5065",
		.of_match_table	= hm5065_dt_ids,
	},
	.id_table = hm5065_id,
	.probe    = hm5065_probe,
	.remove   = hm5065_remove,
};

module_i2c_driver(hm5065_i2c_driver);

MODULE_AUTHOR("Ondrej Jirman <kernel@xff.cz>");
MODULE_DESCRIPTION("HM5065 Camera Subdev Driver");
MODULE_LICENSE("GPL");
