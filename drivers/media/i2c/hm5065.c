/*
 * Himax HM5065 driver.
 * Copyright (C) 2017 Ondřej Jirman <megous@megous.com>.
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
#include <linux/firmware.h>
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

#define HM5065_AF_FIRMWARE		"hm5065-af.bin"
#define HM5065_FIRMWARE_PARAMETERS	"hm5065-init.bin"

#define HM5065_PCLK_FREQ_ABS_MAX	89000000
#define HM5065_FRAME_RATE_MAX		30

/* min/typical/max system clock (xclk) frequencies */
#define HM5065_XCLK_MIN	6000000
#define HM5065_XCLK_MAX	27000000

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

struct reg_value {
	u16 addr;
	u8 value;
} __attribute__ ((packed));

/* {{{ Auto-focus registers */

static const struct reg_value af_init_regs[] = {
	{0xFFFF,0x01},   
	{0x9000,0x03},   
	{0xA000,0x90},   
	{0xA001,0x0C},   
	{0xA002,0x56},   
	{0xA003,0xE0},   
	{0xA004,0xFE},   
	{0xA005,0xA3},   
	{0xA006,0xE0},   
	{0xA007,0xFF},   
	{0xA008,0x12},   
	{0xA009,0x42},   
	{0xA00A,0x85},   
	{0xA00B,0x90},   
	{0xA00C,0x01},   
	{0xA00D,0xB7},   
	{0xA00E,0xEE},   
	{0xA00F,0xF0},   
	{0xA010,0xFC},   
	{0xA011,0xA3},   
	{0xA012,0xEF},   
	{0xA013,0xF0},   
	{0xA014,0xFD},   
	{0xA015,0x90},   
	{0xA016,0x06},   
	{0xA017,0x05},   
	{0xA018,0xE0},   
	{0xA019,0x75},   
	{0xA01A,0xF0},   
	{0xA01B,0x02},   
	{0xA01C,0xA4},   
	{0xA01D,0x2D},   
	{0xA01E,0xFF},   
	{0xA01F,0xE5},   
	{0xA020,0xF0},   
	{0xA021,0x3C},   
	{0xA022,0xFE},   
	{0xA023,0xAB},   
	{0xA024,0x07},   
	{0xA025,0xFA},   
	{0xA026,0x33},   
	{0xA027,0x95},   
	{0xA028,0xE0},   
	{0xA029,0xF9},   
	{0xA02A,0xF8},   
	{0xA02B,0x90},   
	{0xA02C,0x0B},   
	{0xA02D,0x4B},   
	{0xA02E,0xE0},   
	{0xA02F,0xFE},   
	{0xA030,0xA3},   
	{0xA031,0xE0},   
	{0xA032,0xFF},   
	{0xA033,0xEE},   
	{0xA034,0x33},   
	{0xA035,0x95},   
	{0xA036,0xE0},   
	{0xA037,0xFD},   
	{0xA038,0xFC},   
	{0xA039,0x12},   
	{0xA03A,0x0C},   
	{0xA03B,0x7B},   
	{0xA03C,0x90},   
	{0xA03D,0x01},   
	{0xA03E,0xB9},   
	{0xA03F,0x12},   
	{0xA040,0x0E},   
	{0xA041,0x05},   
	{0xA042,0x90},   
	{0xA043,0x01},   
	{0xA044,0xB9},   
	{0xA045,0xE0},   
	{0xA046,0xFC},   
	{0xA047,0xA3},   
	{0xA048,0xE0},   
	{0xA049,0xFD},   
	{0xA04A,0xA3},   
	{0xA04B,0xE0},   
	{0xA04C,0xFE},   
	{0xA04D,0xA3},   
	{0xA04E,0xE0},   
	{0xA04F,0xFF},   
	{0xA050,0x78},   
	{0xA051,0x08},   
	{0xA052,0x12},   
	{0xA053,0x0D},   
	{0xA054,0xBF},   
	{0xA055,0xA8},   
	{0xA056,0x04},   
	{0xA057,0xA9},   
	{0xA058,0x05},   
	{0xA059,0xAA},   
	{0xA05A,0x06},   
	{0xA05B,0xAB},   
	{0xA05C,0x07},   
	{0xA05D,0x90},   
	{0xA05E,0x0B},   
	{0xA05F,0x49},   
	{0xA060,0xE0},   
	{0xA061,0xFE},   
	{0xA062,0xA3},   
	{0xA063,0xE0},   
	{0xA064,0xFF},   
	{0xA065,0xEE},   
	{0xA066,0x33},   
	{0xA067,0x95},   
	{0xA068,0xE0},   
	{0xA069,0xFD},   
	{0xA06A,0xFC},   
	{0xA06B,0xC3},   
	{0xA06C,0xEF},   
	{0xA06D,0x9B},   
	{0xA06E,0xFF},   
	{0xA06F,0xEE},   
	{0xA070,0x9A},   
	{0xA071,0xFE},   
	{0xA072,0xED},   
	{0xA073,0x99},   
	{0xA074,0xFD},   
	{0xA075,0xEC},   
	{0xA076,0x98},   
	{0xA077,0xFC},   
	{0xA078,0x78},   
	{0xA079,0x01},   
	{0xA07A,0x12},   
	{0xA07B,0x0D},   
	{0xA07C,0xBF},   
	{0xA07D,0x90},   
	{0xA07E,0x0C},   
	{0xA07F,0x4A},   
	{0xA080,0xE0},   
	{0xA081,0xFC},   
	{0xA082,0xA3},   
	{0xA083,0xE0},   
	{0xA084,0xF5},   
	{0xA085,0x82},   
	{0xA086,0x8C},   
	{0xA087,0x83},   
	{0xA088,0xC0},   
	{0xA089,0x83},   
	{0xA08A,0xC0},   
	{0xA08B,0x82},   
	{0xA08C,0x90},   
	{0xA08D,0x0B},   
	{0xA08E,0x48},   
	{0xA08F,0xE0},   
	{0xA090,0xD0},   
	{0xA091,0x82},   
	{0xA092,0xD0},   
	{0xA093,0x83},   
	{0xA094,0x75},   
	{0xA095,0xF0},   
	{0xA096,0x02},   
	{0xA097,0x12},   
	{0xA098,0x0E},   
	{0xA099,0x45},   
	{0xA09A,0xEE},   
	{0xA09B,0xF0},   
	{0xA09C,0xA3},   
	{0xA09D,0xEF},   
	{0xA09E,0xF0},   
	{0xA09F,0x02},   
	{0xA0A0,0xBA},   
	{0xA0A1,0xD8},   
	{0xA0A2,0x90},   
	{0xA0A3,0x30},
	{0xA0A4,0x18},
	{0xA0A5,0xe4},
	{0xA0A6,0xf0},
	{0xA0A7,0x74},
	{0xA0A8,0x3f},
	{0xA0A9,0xf0},
	{0xA0AA,0x22},
	{0xA0BF,0x90},   
	{0xA0C0,0x00},   
	{0xA0C1,0x5E},   
	{0xA0C2,0xE0},   
	{0xA0C3,0xFF},   
	{0xA0C4,0x70},   
	{0xA0C5,0x20},   
	{0xA0C6,0x90},   
	{0xA0C7,0x47},   
	{0xA0C8,0x04},   
	{0xA0C9,0x74},   
	{0xA0CA,0x0A},   
	{0xA0CB,0xF0},   
	{0xA0CC,0xA3},   
	{0xA0CD,0x74},   
	{0xA0CE,0x30},   
	{0xA0CF,0xF0},   
	{0xA0D0,0x90},   
	{0xA0D1,0x47},   
	{0xA0D2,0x0C},   
	{0xA0D3,0x74},   
	{0xA0D4,0x07},   
	{0xA0D5,0xF0},   
	{0xA0D6,0xA3},   
	{0xA0D7,0x74},   
	{0xA0D8,0xA8},   
	{0xA0D9,0xF0},   
	{0xA0DA,0x90},   
	{0xA0DB,0x47},   
	{0xA0DC,0xA4},   
	{0xA0DD,0x74},   
	{0xA0DE,0x01},   
	{0xA0DF,0xF0},   
	{0xA0E0,0x90},   
	{0xA0E1,0x47},   
	{0xA0E2,0xA8},   
	{0xA0E3,0xF0},   
	{0xA0E4,0x80},   
	{0xA0E5,0x50},   
	{0xA0E6,0xEF},   
	{0xA0E7,0x64},   
	{0xA0E8,0x01},   
	{0xA0E9,0x60},   
	{0xA0EA,0x04},   
	{0xA0EB,0xEF},   
	{0xA0EC,0xB4},   
	{0xA0ED,0x03},   
	{0xA0EE,0x20},   
	{0xA0EF,0x90},   
	{0xA0F0,0x47},   
	{0xA0F1,0x04},   
	{0xA0F2,0x74},   
	{0xA0F3,0x05},   
	{0xA0F4,0xF0},   
	{0xA0F5,0xA3},   
	{0xA0F6,0x74},   
	{0xA0F7,0x18},   
	{0xA0F8,0xF0},   
	{0xA0F9,0x90},   
	{0xA0FA,0x47},   
	{0xA0FB,0x0C},   
	{0xA0FC,0x74},   
	{0xA0FD,0x03},   
	{0xA0FE,0xF0},   
	{0xA0FF,0xA3},   
	{0xA100,0x74},   
	{0xA101,0xD4},   
	{0xA102,0xF0},   
	{0xA103,0x90},   
	{0xA104,0x47},   
	{0xA105,0xA4},   
	{0xA106,0x74},   
	{0xA107,0x02},   
	{0xA108,0xF0},   
	{0xA109,0x90},   
	{0xA10A,0x47},   
	{0xA10B,0xA8},   
	{0xA10C,0xF0},   
	{0xA10D,0x80},   
	{0xA10E,0x27},   
	{0xA10F,0xEF},   
	{0xA110,0x64},   
	{0xA111,0x02},   
	{0xA112,0x60},   
	{0xA113,0x04},   
	{0xA114,0xEF},   
	{0xA115,0xB4},   
	{0xA116,0x04},   
	{0xA117,0x1E},   
	{0xA118,0x90},   
	{0xA119,0x47},   
	{0xA11A,0x04},   
	{0xA11B,0x74},   
	{0xA11C,0x02},   
	{0xA11D,0xF0},   
	{0xA11E,0xA3},   
	{0xA11F,0x74},   
	{0xA120,0x8C},   
	{0xA121,0xF0},   
	{0xA122,0x90},   
	{0xA123,0x47},   
	{0xA124,0x0C},   
	{0xA125,0x74},   
	{0xA126,0x01},   
	{0xA127,0xF0},   
	{0xA128,0xA3},   
	{0xA129,0x74},   
	{0xA12A,0xEA},   
	{0xA12B,0xF0},   
	{0xA12C,0x90},   
	{0xA12D,0x47},   
	{0xA12E,0xA4},   
	{0xA12F,0x74},   
	{0xA130,0x04},   
	{0xA131,0xF0},   
	{0xA132,0x90},   
	{0xA133,0x47},   
	{0xA134,0xA8},   
	{0xA135,0xF0},   
	{0xA136,0x22},   
	{0xA137,0x74},
	{0xA138,0x04},
	{0xA139,0xF0},
	{0xA13A,0xA3},
	{0xA13B,0x74},
	{0xA13C,0x20},
	{0xA13D,0xF0},
	{0xA13E,0xE4},
	{0xA13F,0xF5},
	{0xA140,0x22},
	{0xA141,0xE5},
	{0xA142,0x22},
	{0xA143,0xC3},
	{0xA144,0x94},
	{0xA145,0x40},
	{0xA146,0x40},
	{0xA147,0x03},
	{0xA148,0x02},
	{0xA149,0xF1},
	{0xA14A,0xFD},
	{0xA14B,0x90},
	{0xA14C,0x0A},
	{0xA14D,0xBA},
	{0xA14E,0xE0},
	{0xA14F,0xFE},
	{0xA150,0xA3},
	{0xA151,0xE0},
	{0xA152,0xFF},
	{0xA153,0xF5},
	{0xA154,0x82},
	{0xA155,0x8E},
	{0xA156,0x83},
	{0xA157,0xE0},
	{0xA158,0x54},
	{0xA159,0x70},
	{0xA15A,0xFD},
	{0xA15B,0xC4},
	{0xA15C,0x54},
	{0xA15D,0x0F},
	{0xA15E,0xFD},
	{0xA15F,0x90},
	{0xA160,0x0A},
	{0xA161,0xBC},
	{0xA162,0xE0},
	{0xA163,0xFA},
	{0xA164,0xA3},
	{0xA165,0xE0},
	{0xA166,0xF5},
	{0xA167,0x82},
	{0xA168,0x8A},
	{0xA169,0x83},
	{0xA16A,0xED},
	{0xA16B,0xF0},
	{0xA16C,0x90},
	{0xA16D,0x0A},
	{0xA16E,0xBD},
	{0xA16F,0xE0},
	{0xA170,0x04},
	{0xA171,0xF0},
	{0xA172,0x70},
	{0xA173,0x06},
	{0xA174,0x90},
	{0xA175,0x0A},
	{0xA176,0xBC},
	{0xA177,0xE0},
	{0xA178,0x04},
	{0xA179,0xF0},
	{0xA17A,0x8F},
	{0xA17B,0x82},
	{0xA17C,0x8E},
	{0xA17D,0x83},
	{0xA17E,0xA3},
	{0xA17F,0xE0},
	{0xA180,0xFF},
	{0xA181,0x90},
	{0xA182,0x0A},
	{0xA183,0xBC},
	{0xA184,0xE0},
	{0xA185,0xFC},
	{0xA186,0xA3},
	{0xA187,0xE0},
	{0xA188,0xF5},
	{0xA189,0x82},
	{0xA18A,0x8C},
	{0xA18B,0x83},
	{0xA18C,0xEF},
	{0xA18D,0xF0},
	{0xA18E,0x90},
	{0xA18F,0x0A},
	{0xA190,0xBD},
	{0xA191,0xE0},
	{0xA192,0x04},
	{0xA193,0xF0},
	{0xA194,0x70},
	{0xA195,0x06},
	{0xA196,0x90},
	{0xA197,0x0A},
	{0xA198,0xBC},
	{0xA199,0xE0},
	{0xA19A,0x04},
	{0xA19B,0xF0},
	{0xA19C,0x90},
	{0xA19D,0x0A},
	{0xA19E,0xBA},
	{0xA19F,0xE0},
	{0xA1A0,0xFE},
	{0xA1A1,0xA3},
	{0xA1A2,0xE0},
	{0xA1A3,0xFF},
	{0xA1A4,0xF5},
	{0xA1A5,0x82},
	{0xA1A6,0x8E},
	{0xA1A7,0x83},
	{0xA1A8,0xE0},
	{0xA1A9,0x54},
	{0xA1AA,0x07},
	{0xA1AB,0xFD},
	{0xA1AC,0x90},
	{0xA1AD,0x0A},
	{0xA1AE,0xBC},
	{0xA1AF,0xE0},
	{0xA1B0,0xFA},
	{0xA1B1,0xA3},
	{0xA1B2,0xE0},
	{0xA1B3,0xF5},
	{0xA1B4,0x82},
	{0xA1B5,0x8A},
	{0xA1B6,0x83},
	{0xA1B7,0xED},
	{0xA1B8,0xF0},
	{0xA1B9,0x90},
	{0xA1BA,0x0A},
	{0xA1BB,0xBD},
	{0xA1BC,0xE0},
	{0xA1BD,0x04},
	{0xA1BE,0xF0},
	{0xA1BF,0x70},
	{0xA1C0,0x06},
	{0xA1C1,0x90},
	{0xA1C2,0x0A},
	{0xA1C3,0xBC},
	{0xA1C4,0xE0},
	{0xA1C5,0x04},
	{0xA1C6,0xF0},
	{0xA1C7,0x8F},
	{0xA1C8,0x82},
	{0xA1C9,0x8E},
	{0xA1CA,0x83},
	{0xA1CB,0xA3},
	{0xA1CC,0xA3},
	{0xA1CD,0xE0},
	{0xA1CE,0xFF},
	{0xA1CF,0x90},
	{0xA1D0,0x0A},
	{0xA1D1,0xBC},
	{0xA1D2,0xE0},
	{0xA1D3,0xFC},
	{0xA1D4,0xA3},
	{0xA1D5,0xE0},
	{0xA1D6,0xF5},
	{0xA1D7,0x82},
	{0xA1D8,0x8C},
	{0xA1D9,0x83},
	{0xA1DA,0xEF},
	{0xA1DB,0xF0},
	{0xA1DC,0x90},
	{0xA1DD,0x0A},
	{0xA1DE,0xBD},
	{0xA1DF,0xE0},
	{0xA1E0,0x04},
	{0xA1E1,0xF0},
	{0xA1E2,0x70},
	{0xA1E3,0x06},
	{0xA1E4,0x90},
	{0xA1E5,0x0A},
	{0xA1E6,0xBC},
	{0xA1E7,0xE0},
	{0xA1E8,0x04},
	{0xA1E9,0xF0},
	{0xA1EA,0x90},
	{0xA1EB,0x0A},
	{0xA1EC,0xBB},
	{0xA1ED,0xE0},
	{0xA1EE,0x24},
	{0xA1EF,0x03},
	{0xA1F0,0xF0},
	{0xA1F1,0x90},
	{0xA1F2,0x0A},
	{0xA1F3,0xBA},
	{0xA1F4,0xE0},
	{0xA1F5,0x34},
	{0xA1F6,0x00},
	{0xA1F7,0xF0},
	{0xA1F8,0x05},
	{0xA1F9,0x22},
	{0xA1FA,0x02},
	{0xA1FB,0xF1},
	{0xA1FC,0x41},
	{0xA1FD,0x90},
	{0xA1FE,0x0A},
	{0xA1FF,0xBA},
	{0xA200,0x74},
	{0xA201,0x0E},
	{0xA202,0xF0},
	{0xA203,0xA3},
	{0xA204,0x74},
	{0xA205,0xDC},
	{0xA206,0xF0},
	{0xA207,0xA3},
	{0xA208,0x74},
	{0xA209,0x05},
	{0xA20A,0xF0},
	{0xA20B,0xA3},
	{0xA20C,0x74},
	{0xA20D,0x61},
	{0xA20E,0xF0},
	{0xA20F,0x90},
	{0xA210,0x0A},
	{0xA211,0xBA},
	{0xA212,0xE0},
	{0xA213,0xFE},
	{0xA214,0xA3},
	{0xA215,0xE0},
	{0xA216,0xAA},
	{0xA217,0x06},
	{0xA218,0xF9},
	{0xA219,0x7B},
	{0xA21A,0x01},
	{0xA21B,0xC0},
	{0xA21C,0x02},
	{0xA21D,0xA3},
	{0xA21E,0xE0},
	{0xA21F,0xFE},
	{0xA220,0xA3},
	{0xA221,0xE0},
	{0xA222,0xAA},
	{0xA223,0x06},
	{0xA224,0xF8},
	{0xA225,0xAC},
	{0xA226,0x02},
	{0xA227,0x7D},
	{0xA228,0x01},
	{0xA229,0xD0},
	{0xA22A,0x02},
	{0xA22B,0x7E},
	{0xA22C,0x00},
	{0xA22D,0x7F},
	{0xA22E,0x04},
	{0xA22F,0x12},
	{0xA230,0x0F},
	{0xA231,0x6F},
	{0xA232,0x02},
	{0xA233,0x66},
	{0xA234,0xD9},
	{0xA235,0x90},
	{0xA236,0x07},
	{0xA237,0xD0},
	{0xA238,0x02},
	{0xA239,0xA2},
	{0xA23A,0x69},
	{0xA240,0x02},
	{0xA241,0x21},
	{0xA242,0x7F},
	{0xA243,0x02},
	{0xA244,0x21},
	{0xA245,0xF4},
	{0xA246,0x02},
	{0xA247,0xA6},
	{0xA248,0x15},
	{0xA249,0x60},
	{0xA24A,0x0A},
	{0xA24B,0xEF},
	{0xA24C,0xB4},
	{0xA24D,0x01},
	{0xA24E,0x16},
	{0xA24F,0x90},
	{0xA250,0x00},
	{0xA251,0x5D},
	{0xA252,0xE0},
	{0xA253,0x70},
	{0xA254,0x10},
	{0xA255,0x12},
	{0xA256,0x26},
	{0xA257,0xC8},
	{0xA258,0x90},
	{0xA259,0x00},
	{0xA25A,0x11},
	{0xA25B,0x74},
	{0xA25C,0x30},
	{0xA25D,0xF0},
	{0xA25E,0x90},
	{0xA25F,0x00},
	{0xA260,0x10},
	{0xA261,0x74},
	{0xA262,0x01},
	{0xA263,0xF0},
	{0xA264,0x22},
	{0xA265,0x12},
	{0xA266,0x25},
	{0xA267,0xA8},
	{0xA268,0x02},
	{0xA269,0x29},
	{0xA26A,0xFC},
	{0xA26B,0x44},
	{0xA26C,0x18},
	{0xA26D,0xF0},
	{0xA26E,0x90},
	{0xA26F,0x72},
	{0xA270,0x18},
	{0xA271,0xE0},
	{0xA272,0x44},
	{0xA273,0x18},
	{0xA274,0xF0},
	{0xA275,0x00},
	{0xA276,0x00},
	{0xA277,0x00},
	{0xA278,0x00},
	{0xA279,0x00},
	{0xA27A,0x00},
	{0xA27B,0x90},
	{0xA27C,0x72},
	{0xA27D,0x08},
	{0xA27E,0xE0},
	{0xA27F,0x44},
	{0xA280,0x10},
	{0xA281,0xF0},
	{0xA282,0x90},
	{0xA283,0x72},
	{0xA284,0x14},
	{0xA285,0xE0},
	{0xA286,0x54},
	{0xA287,0xFD},
	{0xA288,0xF0},
	{0xA289,0x22},
	{0xA29B,0xF0},
	{0xA29C,0xD3},
	{0xA29D,0x90},
	{0xA29E,0x07},
	{0xA29F,0x91},
	{0xA2A0,0xE0},
	{0xA2A1,0x94},
	{0xA2A2,0x21},
	{0xA2A3,0x90},
	{0xA2A4,0x07},
	{0xA2A5,0x90},
	{0xA2A6,0xE0},
	{0xA2A7,0x64},
	{0xA2A8,0x80},
	{0xA2A9,0x94},
	{0xA2AA,0x81},
	{0xA2AB,0x40},
	{0xA2AC,0x08},
	{0xA2AD,0x90},
	{0xA2AE,0x07},
	{0xA2AF,0xCB},
	{0xA2B0,0x74},
	{0xA2B1,0xFF},
	{0xA2B2,0xF0},
	{0xA2B3,0x80},
	{0xA2B4,0x06},
	{0xA2B5,0x90},
	{0xA2B6,0x07},
	{0xA2B7,0xCB},
	{0xA2B8,0x74},
	{0xA2B9,0x01},
	{0xA2BA,0xF0},
	{0xA2BB,0x02},
	{0xA2BC,0xB5},
	{0xA2BD,0xC3},
	{0xA2BE,0x90},
	{0xA2BF,0x08},
	{0xA2C0,0x34},
	{0xA2C1,0xE0},
	{0xA2C2,0xFC},
	{0xA2C3,0xA3},
	{0xA2C4,0xE0},
	{0xA2C5,0xFD},
	{0xA2C6,0xA3},
	{0xA2C7,0xE0},
	{0xA2C8,0xFE},
	{0xA2C9,0xA3},
	{0xA2CA,0xE0},
	{0xA2CB,0xFF},
	{0xA2CC,0x90},
	{0xA2CD,0x07},
	{0xA2CE,0xD0},
	{0xA2CF,0xE0},
	{0xA2D0,0xF8},
	{0xA2D1,0xA3},
	{0xA2D2,0xE0},
	{0xA2D3,0xF9},
	{0xA2D4,0xA3},
	{0xA2D5,0xE0},
	{0xA2D6,0xFA},
	{0xA2D7,0xA3},
	{0xA2D8,0xE0},
	{0xA2D9,0xFB},
	{0xA2DA,0xD3},
	{0xA2DB,0x12},
	{0xA2DC,0x0D},
	{0xA2DD,0xAE},
	{0xA2DE,0x40},
	{0xA2DF,0x0B},
	{0xA2E0,0x12},
	{0xA2E1,0xB5},
	{0xA2E2,0x49},
	{0xA2E3,0x90},
	{0xA2E4,0x07},
	{0xA2E5,0xA4},
	{0xA2E6,0x74},
	{0xA2E7,0x02},
	{0xA2E8,0xF0},
	{0xA2E9,0x80},
	{0xA2EA,0x09},
	{0xA2EB,0x12},
	{0xA2EC,0xB7},
	{0xA2ED,0x51},
	{0xA2EE,0x90},
	{0xA2EF,0x07},
	{0xA2F0,0xA4},
	{0xA2F1,0x74},
	{0xA2F2,0x05},
	{0xA2F3,0xF0},
	{0xA2F4,0x02},
	{0xA2F5,0xA2},
	{0xA2F6,0xDA},
	{0xA2F7,0x90},
	{0xA2F8,0x0E},
	{0xA2F9,0xE0},
	{0xA2FA,0xE0},
	{0xA2FB,0xFD},
	{0xA2FC,0xA3},
	{0xA2FD,0xE0},
	{0xA2FE,0x90},
	{0xA2FF,0x02},
	{0xA300,0xA2},
	{0xA301,0xCD},
	{0xA302,0xF0},
	{0xA303,0xA3},
	{0xA304,0xED},
	{0xA305,0xF0},
	{0xA306,0x90},
	{0xA307,0x0E},
	{0xA308,0xE2},
	{0xA309,0xE0},
	{0xA30A,0xFD},
	{0xA30B,0xA3},
	{0xA30C,0xE0},
	{0xA30D,0x90},
	{0xA30E,0x02},
	{0xA30F,0xA8},
	{0xA310,0xCD},
	{0xA311,0xF0},
	{0xA312,0xA3},
	{0xA313,0xED},
	{0xA314,0xF0},
	{0xA315,0xE4},
	{0xA316,0x90},
	{0xA317,0x06},
	{0xA318,0x38},
	{0xA319,0xF0},
	{0xA31A,0x02},
	{0xA31B,0x67},
	{0xA31C,0x63},
	{0xA31D,0x90},
	{0xA31E,0x0E},
	{0xA31F,0xE8},
	{0xA320,0xE0},
	{0xA321,0x90},
	{0xA322,0x02},
	{0xA323,0x62},
	{0xA324,0xF0},
	{0xA325,0x90},
	{0xA326,0x0E},
	{0xA327,0xE9},
	{0xA328,0xE0},
	{0xA329,0x90},
	{0xA32A,0x02},
	{0xA32B,0x63},
	{0xA32C,0xF0},
	{0xA32D,0x02},
	{0xA32E,0x67},
	{0xA32F,0x1F},
	{0xA33B,0x90},
	{0xA33C,0x0E},
	{0xA33D,0x14},
	{0xA33E,0xE0},
	{0xA33F,0xFE},
	{0xA340,0xA3},
	{0xA341,0xE0},
	{0xA342,0xFF},
	{0xA343,0x90},
	{0xA344,0x06},
	{0xA345,0xD9},
	{0xA346,0xEE},
	{0xA347,0xF0},
	{0xA348,0xA3},
	{0xA349,0xEF},
	{0xA34A,0xF0},
	{0xA34B,0x90},
	{0xA34C,0x0E},
	{0xA34D,0x18},
	{0xA34E,0xE0},
	{0xA34F,0xFD},
	{0xA350,0x7C},
	{0xA351,0x00},
	{0xA352,0xC3},
	{0xA353,0xEF},
	{0xA354,0x9D},
	{0xA355,0xEE},
	{0xA356,0x9C},
	{0xA357,0x50},
	{0xA358,0x09},
	{0xA359,0xE4},
	{0xA35A,0x90},
	{0xA35B,0x06},
	{0xA35C,0xD7},
	{0xA35D,0xF0},
	{0xA35E,0xA3},
	{0xA35F,0xF0},
	{0xA360,0x80},
	{0xA361,0x13},
	{0xA362,0xC3},
	{0xA363,0x90},
	{0xA364,0x06},
	{0xA365,0xDA},
	{0xA366,0xE0},
	{0xA367,0x9D},
	{0xA368,0xFE},
	{0xA369,0x90},
	{0xA36A,0x06},
	{0xA36B,0xD9},
	{0xA36C,0xE0},
	{0xA36D,0x9C},
	{0xA36E,0x90},
	{0xA36F,0x06},
	{0xA370,0xD7},
	{0xA371,0xF0},
	{0xA372,0xA3},
	{0xA373,0xCE},
	{0xA374,0xF0},
	{0xA375,0x90},
	{0xA376,0x0E},
	{0xA377,0x18},
	{0xA378,0xE0},
	{0xA379,0xF9},
	{0xA37A,0xFF},
	{0xA37B,0x90},
	{0xA37C,0x06},
	{0xA37D,0xC2},
	{0xA37E,0xE0},
	{0xA37F,0xFC},
	{0xA380,0xA3},
	{0xA381,0xE0},
	{0xA382,0xFD},
	{0xA383,0xC3},
	{0xA384,0x9F},
	{0xA385,0xFF},
	{0xA386,0xEC},
	{0xA387,0x94},
	{0xA388,0x00},
	{0xA389,0xFE},
	{0xA38A,0x90},
	{0xA38B,0x0E},
	{0xA38C,0x16},
	{0xA38D,0xE0},
	{0xA38E,0xFA},
	{0xA38F,0xA3},
	{0xA390,0xE0},
	{0xA391,0xFB},
	{0xA392,0xD3},
	{0xA393,0x9F},
	{0xA394,0xEA},
	{0xA395,0x9E},
	{0xA396,0x40},
	{0xA397,0x0A},
	{0xA398,0x90},
	{0xA399,0x06},
	{0xA39A,0xD5},
	{0xA39B,0xEC},
	{0xA39C,0xF0},
	{0xA39D,0xA3},
	{0xA39E,0xED},
	{0xA39F,0xF0},
	{0xA3A0,0x80},
	{0xA3A1,0x0E},
	{0xA3A2,0xE9},
	{0xA3A3,0x7E},
	{0xA3A4,0x00},
	{0xA3A5,0x2B},
	{0xA3A6,0xFF},
	{0xA3A7,0xEE},
	{0xA3A8,0x3A},
	{0xA3A9,0x90},
	{0xA3AA,0x06},
	{0xA3AB,0xD5},
	{0xA3AC,0xF0},
	{0xA3AD,0xA3},
	{0xA3AE,0xEF},
	{0xA3AF,0xF0},
	{0xA3B0,0xE9},
	{0xA3B1,0xFB},
	{0xA3B2,0x7A},
	{0xA3B3,0x00},
	{0xA3B4,0x90},
	{0xA3B5,0x0E},
	{0xA3B6,0x15},
	{0xA3B7,0xE0},
	{0xA3B8,0x2B},
	{0xA3B9,0xFE},
	{0xA3BA,0x90},
	{0xA3BB,0x0E},
	{0xA3BC,0x14},
	{0xA3BD,0xE0},
	{0xA3BE,0x3A},
	{0xA3BF,0x90},
	{0xA3C0,0x06},
	{0xA3C1,0xE1},
	{0xA3C2,0xF0},
	{0xA3C3,0xA3},
	{0xA3C4,0xCE},
	{0xA3C5,0xF0},
	{0xA3C6,0xC3},
	{0xA3C7,0x90},
	{0xA3C8,0x0E},
	{0xA3C9,0x17},
	{0xA3CA,0xE0},
	{0xA3CB,0x9B},
	{0xA3CC,0xFE},
	{0xA3CD,0x90},
	{0xA3CE,0x0E},
	{0xA3CF,0x16},
	{0xA3D0,0x02},
	{0xA3D1,0x20},
	{0xA3D2,0xD5},
	{0xA3D3,0x90},
	{0xA3d4,0x0E},
	{0xA3d5,0xE4},
	{0xA3d6,0xE0},
	{0xA3d7,0x90},
	{0xA3d8,0x02},
	{0xA3d9,0x66},
	{0xA3da,0xF0},
	{0xA3DB,0x90},
	{0xA3dc,0x0E},
	{0xA3dd,0xE5},
	{0xA3de,0xE0},
	{0xA3df,0x90},
	{0xA3e0,0x02},
	{0xA3e1,0x64},
	{0xA3e2,0xF0},
	{0xA3e3,0x90},
	{0xA3e4,0x0E},
	{0xA3e5,0xE6},
	{0xA3e6,0xE0},
	{0xA3e7,0x90},
	{0xA3e8,0x02},
	{0xA3e9,0x65},
	{0xA3ea,0xF0},
	{0xA3eb,0x02},
	{0xA3ec,0x67},
	{0xA3ed,0xA5},
	{0xA3f0,0x12},
	{0xA3f1,0x47},
	{0xA3f2,0x59},
	{0xA3f3,0x90},
	{0xA3f4,0x00},
	{0xA3f5,0xB5},
	{0xA3f6,0xE0},
	{0xA3f7,0xB4},
	{0xA3f8,0x02},
	{0xA3f9,0x03},
	{0xA3fa,0x12},
	{0xA3fb,0x47},
	{0xA3fc,0x59},
	{0xA3fd,0x02},
	{0xA3fe,0xC5},
	{0xA3ff,0xC3},
	{0xA400,0x90},
	{0xA401,0x00},
	{0xA402,0x3D},
	{0xA403,0xF0},
	{0xA404,0x90},
	{0xA405,0x00},
	{0xA406,0x84},
	{0xA407,0xE0},
	{0xA408,0xFE},
	{0xA409,0x90},
	{0xA40A,0x00},
	{0xA40B,0x3E},
	{0xA40C,0xF0},
	{0xA40D,0xEF},
	{0xA40E,0x70},
	{0xA40F,0x03},
	{0xA410,0xEE},
	{0xA411,0x60},
	{0xA412,0x04},
	{0xA413,0x7F},
	{0xA414,0x01},
	{0xA415,0x80},
	{0xA416,0x02},
	{0xA417,0x7F},
	{0xA418,0x00},
	{0xA419,0x90},
	{0xA41A,0x00},
	{0xA41B,0x3F},
	{0xA41C,0xEF},
	{0xA41D,0xF0},
	{0xA41E,0x02},
	{0xA41F,0x89},
	{0xA420,0xD3},
	{0xA421,0x90},
	{0xA422,0x00},
	{0xA423,0x12},
	{0xA424,0xE0},
	{0xA425,0xFF},
	{0xA426,0x70},
	{0xA427,0x0C},
	{0xA428,0x90},
	{0xA429,0x00},
	{0xA42A,0x46},
	{0xA42B,0xE0},
	{0xA42C,0xC3},
	{0xA42D,0x94},
	{0xA42E,0x07},
	{0xA42F,0x40},
	{0xA430,0x03},
	{0xA431,0x75},
	{0xA432,0x2E},
	{0xA433,0x02},
	{0xA434,0xEF},
	{0xA435,0xB4},
	{0xA436,0x01},
	{0xA437,0x0C},
	{0xA438,0x90},
	{0xA439,0x00},
	{0xA43A,0x66},
	{0xA43B,0xE0},
	{0xA43C,0xC3},
	{0xA43D,0x94},
	{0xA43E,0x07},
	{0xA43F,0x40},
	{0xA440,0x03},
	{0xA441,0x75},
	{0xA442,0x2E},
	{0xA443,0x02},
	{0xA444,0x02},
	{0xA445,0xA7},
	{0xA446,0x9E},
	{0xA447,0xC3},
	{0xA448,0x90},
	{0xA449,0x0B},
	{0xA44A,0x8F},
	{0xA44B,0xE0},
	{0xA44C,0x94},
	{0xA44D,0x80},
	{0xA44E,0x90},
	{0xA44F,0x0B},
	{0xA450,0x8E},
	{0xA451,0xE0},
	{0xA452,0x94},
	{0xA453,0x44},
	{0xA454,0x40},
	{0xA455,0x22},
	{0xA456,0x90},
	{0xA457,0x0B},
	{0xA458,0x91},
	{0xA459,0xE0},
	{0xA45A,0x94},
	{0xA45B,0x80},
	{0xA45C,0x90},
	{0xA45D,0x0B},
	{0xA45E,0x90},
	{0xA45F,0xE0},
	{0xA460,0x94},
	{0xA461,0x44},
	{0xA462,0x40},
	{0xA463,0x14},
	{0xA464,0x90},
	{0xA465,0x0B},
	{0xA466,0x93},
	{0xA467,0xE0},
	{0xA468,0x94},
	{0xA469,0x80},
	{0xA46A,0x90},
	{0xA46B,0x0B},
	{0xA46C,0x92},
	{0xA46D,0xE0},
	{0xA46E,0x94},
	{0xA46F,0x44},
	{0xA470,0x40},
	{0xA471,0x06},
	{0xA472,0x90},
	{0xA473,0x01},
	{0xA474,0xA4},
	{0xA475,0x02},
	{0xA476,0x86},
	{0xA477,0x57},
	{0xA478,0x02},
	{0xA479,0x86},
	{0xA47A,0x5C},
	{0xA500,0xF5},
	{0xA501,0x3B},
	{0xA502,0x90},
	{0xA503,0x06},
	{0xA504,0x6C},
	{0xA505,0xE0},
	{0xA506,0xFF},
	{0xA507,0xE5},
	{0xA508,0x3B},
	{0xA509,0xC3},
	{0xA50A,0x9F},
	{0xA50B,0x40},
	{0xA50C,0x03},
	{0xA50D,0x02},
	{0xA50E,0xF6},
	{0xA50F,0x0E},
	{0xA510,0x90},
	{0xA511,0x0B},
	{0xA512,0xC6},
	{0xA513,0xE0},
	{0xA514,0x14},
	{0xA515,0x60},
	{0xA516,0x3C},
	{0xA517,0x14},
	{0xA518,0x60},
	{0xA519,0x6B},
	{0xA51A,0x24},
	{0xA51B,0x02},
	{0xA51C,0x60},
	{0xA51D,0x03},
	{0xA51E,0x02},
	{0xA51F,0xF5},
	{0xA520,0xB5},
	{0xA521,0x90},
	{0xA522,0x0A},
	{0xA523,0x9A},
	{0xA524,0xE0},
	{0xA525,0xFB},
	{0xA526,0xA3},
	{0xA527,0xE0},
	{0xA528,0xFA},
	{0xA529,0xA3},
	{0xA52A,0xE0},
	{0xA52B,0xF9},
	{0xA52C,0x85},
	{0xA52D,0x3B},
	{0xA52E,0x82},
	{0xA52F,0x75},
	{0xA530,0x83},
	{0xA531,0x00},
	{0xA532,0x12},
	{0xA533,0x0A},
	{0xA534,0xB8},
	{0xA535,0xFF},
	{0xA536,0x74},
	{0xA537,0xAB},
	{0xA538,0x25},
	{0xA539,0x3B},
	{0xA53A,0xF5},
	{0xA53B,0x82},
	{0xA53C,0xE4},
	{0xA53D,0x34},
	{0xA53E,0x0A},
	{0xA53F,0xF5},
	{0xA540,0x83},
	{0xA541,0xE0},
	{0xA542,0xFD},
	{0xA543,0xC3},
	{0xA544,0xEF},
	{0xA545,0x9D},
	{0xA546,0xFE},
	{0xA547,0xE4},
	{0xA548,0x94},
	{0xA549,0x00},
	{0xA54A,0x90},
	{0xA54B,0x0B},
	{0xA54C,0xCA},
	{0xA54D,0xF0},
	{0xA54E,0xA3},
	{0xA54F,0xCE},
	{0xA550,0xF0},
	{0xA551,0x80},
	{0xA552,0x62},
	{0xA553,0x90},
	{0xA554,0x0A},
	{0xA555,0x9A},
	{0xA556,0xE0},
	{0xA557,0xFB},
	{0xA558,0xA3},
	{0xA559,0xE0},
	{0xA55A,0xFA},
	{0xA55B,0xA3},
	{0xA55C,0xE0},
	{0xA55D,0xF9},
	{0xA55E,0x85},
	{0xA55F,0x3B},
	{0xA560,0x82},
	{0xA561,0x75},
	{0xA562,0x83},
	{0xA563,0x00},
	{0xA564,0x12},
	{0xA565,0x0A},
	{0xA566,0xB8},
	{0xA567,0xFF},
	{0xA568,0x74},
	{0xA569,0x9D},
	{0xA56A,0x25},
	{0xA56B,0x3B},
	{0xA56C,0xF5},
	{0xA56D,0x82},
	{0xA56E,0xE4},
	{0xA56F,0x34},
	{0xA570,0x0A},
	{0xA571,0xF5},
	{0xA572,0x83},
	{0xA573,0xE0},
	{0xA574,0xFD},
	{0xA575,0xC3},
	{0xA576,0xEF},
	{0xA577,0x9D},
	{0xA578,0xFE},
	{0xA579,0xE4},
	{0xA57A,0x94},
	{0xA57B,0x00},
	{0xA57C,0x90},
	{0xA57D,0x0B},
	{0xA57E,0xCA},
	{0xA57F,0xF0},
	{0xA580,0xA3},
	{0xA581,0xCE},
	{0xA582,0xF0},
	{0xA583,0x80},
	{0xA584,0x30},
	{0xA585,0x90},
	{0xA586,0x0A},
	{0xA587,0x9A},
	{0xA588,0xE0},
	{0xA589,0xFB},
	{0xA58A,0xA3},
	{0xA58B,0xE0},
	{0xA58C,0xFA},
	{0xA58D,0xA3},
	{0xA58E,0xE0},
	{0xA58F,0xF9},
	{0xA590,0x85},
	{0xA591,0x3B},
	{0xA592,0x82},
	{0xA593,0x75},
	{0xA594,0x83},
	{0xA595,0x00},
	{0xA596,0x12},
	{0xA597,0x0A},
	{0xA598,0xB8},
	{0xA599,0xFF},
	{0xA59A,0x74},
	{0xA59B,0xA4},
	{0xA59C,0x25},
	{0xA59D,0x3B},
	{0xA59E,0xF5},
	{0xA59F,0x82},
	{0xA5A0,0xE4},
	{0xA5A1,0x34},
	{0xA5A2,0x0A},
	{0xA5A3,0xF5},
	{0xA5A4,0x83},
	{0xA5A5,0xE0},
	{0xA5A6,0xFD},
	{0xA5A7,0xC3},
	{0xA5A8,0xEF},
	{0xA5A9,0x9D},
	{0xA5AA,0xFE},
	{0xA5AB,0xE4},
	{0xA5AC,0x94},
	{0xA5AD,0x00},
	{0xA5AE,0x90},
	{0xA5AF,0x0B},
	{0xA5B0,0xCA},
	{0xA5B1,0xF0},
	{0xA5B2,0xA3},
	{0xA5B3,0xCE},
	{0xA5B4,0xF0},
	{0xA5B5,0x90},
	{0xA5B6,0x07},
	{0xA5B7,0x83},
	{0xA5B8,0xE0},
	{0xA5B9,0xFF},
	{0xA5BA,0x7E},
	{0xA5BB,0x00},
	{0xA5BC,0x90},
	{0xA5BD,0x0D},
	{0xA5BE,0xF6},
	{0xA5BF,0xEE},
	{0xA5C0,0xF0},
	{0xA5C1,0xA3},
	{0xA5C2,0xEF},
	{0xA5C3,0xF0},
	{0xA5C4,0x90},
	{0xA5C5,0x0B},
	{0xA5C6,0xCA},
	{0xA5C7,0xE0},
	{0xA5C8,0xFC},
	{0xA5C9,0xA3},
	{0xA5CA,0xE0},
	{0xA5CB,0xFD},
	{0xA5CC,0xD3},
	{0xA5CD,0x9F},
	{0xA5CE,0x74},
	{0xA5CF,0x80},
	{0xA5D0,0xF8},
	{0xA5D1,0xEC},
	{0xA5D2,0x64},
	{0xA5D3,0x80},
	{0xA5D4,0x98},
	{0xA5D5,0x40},
	{0xA5D6,0x0C},
	{0xA5D7,0x90},
	{0xA5D8,0x0B},
	{0xA5D9,0xC8},
	{0xA5DA,0xE0},
	{0xA5DB,0x04},
	{0xA5DC,0xF0},
	{0xA5DD,0xA3},
	{0xA5DE,0xE0},
	{0xA5DF,0x04},
	{0xA5E0,0xF0},
	{0xA5E1,0x80},
	{0xA5E2,0x26},
	{0xA5E3,0x90},
	{0xA5E4,0x0D},
	{0xA5E5,0xF6},
	{0xA5E6,0xE0},
	{0xA5E7,0xFE},
	{0xA5E8,0xA3},
	{0xA5E9,0xE0},
	{0xA5EA,0xFF},
	{0xA5EB,0xC3},
	{0xA5EC,0xE4},
	{0xA5ED,0x9F},
	{0xA5EE,0xFF},
	{0xA5EF,0xE4},
	{0xA5F0,0x9E},
	{0xA5F1,0xFE},
	{0xA5F2,0xC3},
	{0xA5F3,0xED},
	{0xA5F4,0x9F},
	{0xA5F5,0xEE},
	{0xA5F6,0x64},
	{0xA5F7,0x80},
	{0xA5F8,0xF8},
	{0xA5F9,0xEC},
	{0xA5FA,0x64},
	{0xA5FB,0x80},
	{0xA5FC,0x98},
	{0xA5FD,0x50},
	{0xA5FE,0x0A},
	{0xA5FF,0x90},
	{0xA600,0x0B},
	{0xA601,0xC8},
	{0xA602,0xE0},
	{0xA603,0x14},
	{0xA604,0xF0},
	{0xA605,0xA3},
	{0xA606,0xE0},
	{0xA607,0x04},
	{0xA608,0xF0},
	{0xA609,0x05},
	{0xA60A,0x3B},
	{0xA60B,0x02},
	{0xA60C,0xF5},
	{0xA60D,0x02},
	{0xA60E,0x90},
	{0xA60F,0x08},
	{0xA610,0x58},
	{0xA611,0x02},
	{0xA612,0x9D},
	{0xA613,0x50},
	{0x9006,0xBA},   
	{0x9007,0x75},   
	{0x9008,0x00},   
	{0x9009,0x00},   
	{0x900A,0x02},   
	{0x900D,0x01},
	{0x900E,0xA2},
	{0x900F,0x8F},
	{0x9010,0x00},   
	{0x9011,0xCB},
	{0x9012,0x03},
	{0x9016,0xE6},   
	{0x9017,0x6B},
	{0x9018,0x02},
	{0x9019,0x6B},
	{0x901A,0x02},   
	{0x901D,0x01},
	{0x901E,0xAC},
	{0x901F,0x70},
	{0x9020,0x00},   
	{0x9021,0xC5},
	{0x9022,0x03},
	{0x9026,0x9C},   
	{0x9027,0x5B},   
	{0x9028,0x00},   
	{0x9029,0xBF},   
	{0x902A,0x02},   
	{0x902E,0x60},
	{0x902F,0x1C},
	{0x9030,0x01},
	{0x9031,0x37},
	{0x9032,0x02},
	{0x9035,0x01},
	{0x9036,0xBA},
	{0x9037,0x70},
	{0x9038,0x00},
	{0x9039,0x00},
	{0x903A,0x03},
	{0x903E,0x21},
	{0x903F,0x3F},
	{0x9040,0x02},
	{0x9041,0x40},
	{0x9042,0x02},
	{0x9046,0x21},
	{0x9047,0xEA},
	{0x9048,0x02},
	{0x9049,0x43},
	{0x904A,0x02},
	{0x904E,0xA6},
	{0x904F,0x12},
	{0x9050,0x02},
	{0x9051,0x46},
	{0x9052,0x02},
	{0x9056,0x29},
	{0x9057,0xE3},
	{0x9058,0x02},
	{0x9059,0x49},
	{0x905A,0x02},
	{0x905D,0x01},
	{0x905E,0x9C},
	{0x905F,0x6E},
	{0x9060,0x05},
	{0x9061,0x00},
	{0x9062,0x02},
	{0x9065,0x01},
	{0x9066,0xA2},
	{0x9067,0x66},
	{0x9068,0x02},
	{0x9069,0x35},
	{0x906A,0x02},
	{0x906D,0x01},
	{0x906E,0xB5},
	{0x906F,0xC2},
	{0x9070,0x02},
	{0x9071,0x9B},
	{0x9072,0x02},
	{0x9075,0x01},
	{0x9076,0xA2},
	{0x9077,0xD4},
	{0x9078,0x02},
	{0x9079,0xBE},
	{0x907A,0x02},
	{0x907D,0x01},
	{0x907E,0xB7},
	{0x907F,0xEA},
	{0x9080,0x00},
	{0x9081,0x02},
	{0x9082,0x03},
	{0x9086,0x67},
	{0x9087,0x31},
	{0x9088,0x02},
	{0x9089,0xF7},
	{0x908A,0x02},
	{0x908E,0x66},
	{0x908F,0xED},
	{0x9090,0x03},
	{0x9091,0x1D},
	{0x9092,0x02},
	{0x9096,0x67},
	{0x9097,0x73},
	{0x9098,0x03},
	{0x9099,0xD3},
	{0x909A,0x02},
	{0x909E,0x20},
	{0x909F,0x40},
	{0x90A0,0x03},
	{0x90A1,0x3B},
	{0x90A2,0x02},
	{0x90A6,0xC5},
	{0x90A7,0xC0},
	{0x90A8,0x03},
	{0x90A9,0xF0},
	{0x90AA,0x02},
	{0x90AE,0x41},
	{0x90AF,0xB3},
	{0x90B0,0x00},
	{0x90B1,0xA2},
	{0x90B2,0x02},
	{0x90B6,0x44},
	{0x90B7,0xBA},
	{0x90B8,0x00},
	{0x90B9,0xF0},
	{0x90BA,0x03},
	{0x90BE,0x89},
	{0x90BF,0x99},
	{0x90C0,0x04},
	{0x90C1,0x00},
	{0x90C2,0x02},
	{0x90C6,0xA7},
	{0x90C7,0x91},
	{0x90C8,0x04},
	{0x90C9,0x21},
	{0x90CA,0x02},
	{0x90CE,0x3A},
	{0x90CF,0x51},
	{0x90D0,0x00},
	{0x90D1,0xA2},
	{0x90D2,0x02},
	{0x90D6,0x86},
	{0x90D7,0x54},
	{0x90D8,0x04},
	{0x90D9,0x47},
	{0x90DA,0x02},
	{0x9000,0x01},   	
	{0xFFFF,0x00},
};

/* }}} */
/* {{{ Default register values */

static const struct reg_value default_regs[] = {
	{0x0009,0x16},   
	{0x0012,0x00},
	{0x0013,0x00},
	{0x0016,0x00},   
	{0x0021,0x00},
	{0x0022,0x01},
	{0x0040,0x01},
	{0x0041,0x04},
	{0x0042,0x05},
	{0x0043,0x00},
	{0x0044,0x03},
	{0x0045,0xC0},
	{0x0046,0x02},
	{0x0060,0x00},
	{0x0061,0x00},
	{0x0066,0x02},
	//{0x0083,0x00},
	//{0x0084,0x00},
	//{0x0085,0x02},
	{0x00B2,0x4f}, 
	{0x00B3,0xc0},
	{0x00B4,0x01},
	//{0x00B5,0x01},
	//{0x00E8,0x01},
	//{0x00ED,0x05},
	//{0x00EE,0x1E},
	{0x0129,0x00},
	{0x0130,0x00},
	{0x0137,0x00},
	{0x019C,0x4B},
	{0x019D,0xC0},
	{0x01A0,0x01},
	{0x01A1,0x80},
	{0x01A2,0x80},
	{0x01A3,0x80},
	{0x5200,0x01},
	{0x7000,0x0C},
	{0x7101,0x44},//c4
	{0x7102,0x01},
	{0x7103,0x00},
	{0x7104,0x00},
	{0x7105,0x80},
	{0x7158,0x00},
	{0x0143,0x5F},
	{0x0144,0x0D},
	{0x0046,0x00},   
	{0x0041,0x00},   
	//{0x00B5,0x02},   
	{0x7101,0x44},   
	//{0x00ED,0x0A},   	
	//{0x00EE,0x1E},   	
	//{0x00B3,0x80},    
	{0x019C,0x4B},   
	{0x019D,0xC0},   
	{0x0129,0x00},   
	{0x0130,0xFF},   
	{0x0083,0x00},   
	{0x0084,0x00},   
	{0x01A1,0x80},   
	{0x01A2,0x80},   
	{0x01A3,0x80},   
	{0x01A0,0x01},   
	{0x0021,0x00},   
	{0x0022,0x01},   
	{0x0040,0x01},   
	{0x0060,0x00},   
	{0x0013,0x00},   
	{0x0041,0x04},   
	{0x0061,0x00},   
	{0x0046,0x02},//02   gong
	{0x0066,0x02},   //02 gong
	{0x0012,0x00},   
	{0x7102,0x09},   
	{0x7103,0x00},   
	{0x7158,0x00},   
	//{0x00E8,0x01},   
	{0x7000,0x2C},   
	{0x5200,0x01},   
	{0x7000,0x0C},   
	{0x02C2,0x00},   
	{0x02C3,0xC0},
	{0x015E,0x40},
	{0x015F,0x00},   
	{0x0390,0x01},   	
	{0x0391,0x00},   	
	{0x0392,0x00},   	
	{0x03A0,0x14},   	
	{0x03A1,0x00},   	
	{0x03A2,0x5A},   	
	{0x03A3,0xEE},   	
	{0x03A4,0x69},   	
	{0x03A5,0x49},   	
	{0x03A6,0x3E},   	
	{0x03A7,0x00},   	
	{0x03A8,0x39},   	
	{0x03A9,0x33},   	
	{0x03B0,0x60},   	
	{0x03B1,0x00},   	
	{0x03B2,0x5A},   	
	{0x03B3,0xEE},   	
	{0x03B4,0x69},   	
	{0x03B5,0x49},   	
	{0x03B6,0x3E},   	
	{0x03B7,0x00},   	
	{0x03B8,0x3D},   	
	{0x03B9,0x20},   	
	{0x03C0,0x10},   	
	{0x03C1,0x00},   	
	{0x03C2,0x5A},   	
	{0x03C3,0xEE},   	
	{0x03C4,0x69},   	
	{0x03C5,0x49},   	
	{0x03C6,0x3A},   	
	{0x03C7,0x80},   	
	{0x03D0,0x64},   	
	{0x03D1,0x00},   	
	{0x03D2,0x5A},   	
	{0x03D3,0xEE},   	
	{0x03D4,0x69},   	
	{0x03D5,0x49},   	
	{0x03D6,0x34},   	
	{0x03D7,0xD1},   	
	{0x004C,0x0B},   	
	{0x006C,0x08},   	
	{0x0350,0x00},   	
	{0x0351,0x5A},   	
	{0x0352,0xEE},   	
	{0x0353,0x69},   	
	{0x0354,0x49},   	
	{0x0355,0x39},   	
	{0x0356,0x6D},   	
	{0x0357,0x10},   	
	{0x0358,0x00},   	
	{0x0359,0x3C},   	
	{0x035A,0x5A},   	
	{0x035B,0xEE},   	
	{0x035C,0x69},   	
	{0x035D,0x49},   	
	{0x035E,0x39},   	
	{0x035F,0x85},   	
	{0x0049,0x14},
	{0x004A,0x0D},
	{0x0069,0x14},
	{0x006A,0x0D},
	{0x0090,0x5A},
	{0x0091,0xEE},
	{0x0092,0x3E},
	{0x0093,0x00},
	{0x0094,0x69},
	{0x0095,0x49},
	{0x0096,0x39},
	{0x0097,0xCF},
	{0x0098,0x00},
	{0x00A0,0x5A},
	{0x00A1,0xEE},
	{0x00A2,0x3E},
	{0x00A3,0x00},
	{0x00A4,0x69},
	{0x00A5,0x49},
	{0x00A6,0x3B},
	{0x00A7,0x80},
	{0x00A8,0x00},
	{0x0420,0x00},
	{0x0421,0x09},
	{0x0422,0xff},
	{0x0423,0x9e},
	{0x0424,0x00},    
	{0x0425,0x89},
	{0x0426,0x00},    
	{0x0427,0xab},
	{0x0428,0xff},
	{0x0429,0xe9},
	{0x042a,0xff},
	{0x042b,0x8b},
	{0x042c,0x00},
	{0x042d,0x73},
	{0x042E,0xff},    
	{0x042f,0xb6},
	{0x0430,0x00},
	{0x0431,0x54},
	{0x0432,0xff},    
	{0x0433,0x43},
	{0x0434,0x01},
	{0x0435,0x04},
	{0x0436,0x01},
	{0x0437,0x34},
	{0x0438,0xff},    
	{0x0439,0x7c},
	{0x043a,0xfe},
	{0x043b,0xd2},
	{0x043c,0x00},
	{0x043d,0x63},
	{0x043e,0xff},
	{0x043f,0x15},
	{0x0450,0x00},
	{0x0451,0x3b},
	{0x0452,0xff},    
	{0x0453,0x98},
	{0x0454,0x00},    
	{0x0455,0x6f},
	{0x0456,0x00},    
	{0x0457,0x93},
	{0x0458,0xff},    
	{0x0459,0xad},
	{0x045a,0xff},
	{0x045b,0x87},
	{0x045c,0x00},
	{0x045d,0x52},
	{0x045E,0xff},    
	{0x045f,0xa7},
	{0x0440,0xff},    
	{0x0441,0xfd},
	{0x0442,0xff},    
	{0x0443,0x6c},
	{0x0444,0x00},    
	{0x0445,0x90},
	{0x0446,0x00},    
	{0x0447,0xa1},
	{0x0448,0x00},    
	{0x0449,0x02},
	{0x044a,0xff},
	{0x044b,0x48},
	{0x044c,0x00},
	{0x044d,0x5b},
	{0x044E,0xff},    
	{0x044f,0xb4},
	{0x0460,0xff},    
	{0x0461,0x69},
	{0x0462,0xff},
	{0x0463,0xbb},
	{0x0464,0x00},    
	{0x0465,0x84},
	{0x0466,0x00},    
	{0x0467,0xa3},
	{0x0468,0x00},    
	{0x0469,0x0e},
	{0x046A,0x00},    
	{0x046b,0x76},
	{0x046C,0xff},    
	{0x046d,0xaf},
	{0x046E,0xff},    
	{0x046f,0xf5},
	{0x0470,0xff},    
	{0x0471,0x8a},
	{0x0472,0xff},    
	{0x0473,0x5a},
	{0x0474,0x00},    
	{0x0475,0xef},
	{0x0476,0x01},
	{0x0477,0x16},
	{0x0478,0xff},    
	{0x0479,0xd4},
	{0x047A,0x00},    
	{0x047b,0x02},
	{0x047c,0x00},
	{0x047d,0x2c},
	{0x047E,0xff},    
	{0x047f,0x95},
	{0x0490,0xff},    
	{0x0491,0x9b},
	{0x0492,0xff},    
	{0x0493,0x91},
	{0x0494,0x00},    
	{0x0495,0x6f},
	{0x0496,0x00},    
	{0x0497,0x95},
	{0x0498,0xff},    
	{0x0499,0xd5},
	{0x049a,0x01},
	{0x049b,0x20},
	{0x049C,0xff},    
	{0x049d,0xfb},
	{0x049E,0xff},    
	{0x049f,0xe1},
	{0x0480,0xff},    
	{0x0481,0x5a},
	{0x0482,0xff},
	{0x0483,0x91},
	{0x0484,0x00},    
	{0x0485,0x8c},
	{0x0486,0x00},    
	{0x0487,0x9f},
	{0x0488,0x00},    
	{0x0489,0x29},
	{0x048A,0x00},    
	{0x048b,0x53},
	{0x048C,0xff},    
	{0x048d,0x80},
	{0x048E,0xff},    
	{0x048f,0xf7},
	{0x04A0,0xff},    
	{0x04a1,0x6c},
	{0x04a2,0xff},
	{0x04a3,0xb9},
	{0x04A4,0x00},    
	{0x04a5,0x81},
	{0x04A6,0x00},    
	{0x04a7,0x93},
	{0x04A8,0x00},    
	{0x04A9,0x1c},    
	{0x04AA,0x00},    
	{0x04ab,0x39},
	{0x04AC,0xff},    
	{0x04ad,0x9f},
	{0x04ae,0x00},
	{0x04af,0x0e},
	{0x04B0,0xff},    
	{0x04b1,0xe0},
	{0x04B2,0xff},    
	{0x04b3,0x7b},
	{0x04B4,0x00},    
	{0x04b5,0xaa},
	{0x04B6,0x00},    
	{0x04b7,0xc8},
	{0x04B8,0xff},    
	{0x04b9,0xe1},
	{0x04BA,0x00},    
	{0x04bb,0x0e},
	{0x04bc,0x00},
	{0x04bd,0x0b},
	{0x04be,0xff},
	{0x04bf,0xff},
	{0x04D0,0xff},    
	{0x04d1,0xac},
	{0x04D2,0xff},    
	{0x04d3,0x93},
	{0x04D4,0x00},    
	{0x04d5,0x64},
	{0x04D6,0x00},    
	{0x04d7,0x83},
	{0x04D8,0xff},    
	{0x04d9,0xdb},
	{0x04DA,0x00},    
	{0x04db,0xa8},
	{0x04DC,0xff},    
	{0x04dd,0xf5},
	{0x04de,0x00},
	{0x04df,0x15},
	{0x04C0,0xff},    
	{0x04c1,0x5d},
	{0x04c2,0xff},
	{0x04c3,0x9c},
	{0x04C4,0x00},    
	{0x04c5,0x82},
	{0x04C6,0x00},    
	{0x04c7,0x96},
	{0x04C8,0x00},    
	{0x04c9,0x33},
	{0x04CA,0x00},    
	{0x04cb,0x07},
	{0x04CC,0xff},    
	{0x04cd,0x71},
	{0x04ce,0x00},
	{0x04cf,0x11},
	{0x04E0,0xff},    
	{0x04e1,0x6d},
	{0x04e2,0xff},
	{0x04e3,0xb8},
	{0x04E4,0x00},    
	{0x04e5,0x84},
	{0x04E6,0x00},    
	{0x04e7,0x96},
	{0x04e8,0xff},
	{0x04e9,0xc0},
	{0x04EA,0x00},    
	{0x04eb,0x6d},
	{0x04EC,0xff},    
	{0x04ed,0xbb},
	{0x04ee,0x00},
	{0x04ef,0x00},
	{0x04F0,0xff},    
	{0x04f1,0xe0},
	{0x04F2,0xff},    
	{0x04f3,0x95},
	{0x04F4,0x00},    
	{0x04f5,0xa7},
	{0x04F6,0x00},    
	{0x04f7,0xc8},
	{0x04F8,0xff},    
	{0x04f9,0xde},
	{0x04FA,0x00},    
	{0x04fb,0x7e},
	{0x04fc,0x00},
	{0x04fd,0x36},
	{0x04fe,0x00},
	{0x04ff,0x10},
	{0x0510,0xff},    
	{0x0511,0xc1},
	{0x0512,0xff},    
	{0x0513,0x9f},
	{0x0514,0x00},    
	{0x0515,0x6a},
	{0x0516,0x00},    
	{0x0517,0x89},
	{0x0518,0xff},    
	{0x0519,0xdc},
	{0x051A,0x00},    
	{0x051b,0x55},
	{0x051c,0x00},
	{0x051d,0x09},
	{0x051e,0x00},
	{0x051f,0x0d},
	{0x0500,0xff},    
	{0x0501,0x60},
	{0x0502,0xff},
	{0x0503,0x9e},
	{0x0504,0x00},    
	{0x0505,0x81},
	{0x0506,0x00},    
	{0x0507,0x9c},
	{0x0508,0xff},
	{0x0509,0xc0},
	{0x050A,0x00},    
	{0x050b,0x40},
	{0x050C,0xff},    
	{0x050d,0x8e},
	{0x050e,0x00},
	{0x050f,0x00},
	{0x0561,0x0e},
	{0x0562,0x01},
	{0x0563,0x01},
	{0x0564,0x06},
	{0x0324,0x39},
	{0x0325,0xAE},
	{0x0326,0x3a},    
	{0x0327,0x29},
	{0x0328,0x3b},    
	{0x0329,0x0A},
	{0x032A,0x3b},    
	{0x032B,0x62},
	{0x0320,0x01},    
	{0x0321,0x04},    
	{0x0322,0x01},    
	{0x0323,0x01},    
	{0x0330,0x01},   	
	{0x0384,0x00},   	
	{0x0337,0x01},   	
	{0x03EC,0x39},   	
	{0x03ED,0x85},
	{0x03FC,0x3A},
	{0x03FD,0x14},
	{0x040C,0x3A},   	
	{0x040D,0xF6},
	{0x041C,0x3B},   	
	{0x041D,0x9A},
	{0x03E0,0xB6},   	
	{0x03E1,0x04},   	
	{0x03E2,0xBB},   	
	{0x03E3,0xE9},   	
	{0x03E4,0xBC},   	
	{0x03E5,0x70},   	
	{0x03E6,0x37},   	
	{0x03E7,0x02},   	
	{0x03E8,0xBC},   	
	{0x03E9,0x00},   	
	{0x03EA,0xBF},   	
	{0x03EB,0x12},   	
	{0x03F0,0xBA},   	
	{0x03F1,0x7B},   	
	{0x03F2,0xBA},   	
	{0x03F3,0x83},   	
	{0x03F4,0xBB},   	
	{0x03F5,0xBC},   	
	{0x03F6,0x38},   	
	{0x03F7,0x2D},   	
	{0x03F8,0xBB},   	
	{0x03F9,0x23},   	
	{0x03FA,0xBD},   	
	{0x03FB,0xAC},   	
	{0x0400,0xBE},   	
	{0x0401,0x96},   	
	{0x0402,0xB9},   	
	{0x0403,0xBE},   	
	{0x0404,0xBB},   	
	{0x0405,0x57},   	
	{0x0406,0x3A},   	
	{0x0407,0xBB},   	
	{0x0408,0xB3},   	
	{0x0409,0x17},   	
	{0x040A,0xBE},   	
	{0x040B,0x66},   	
	{0x0410,0xBB},
	{0x0411,0x2A},
	{0x0412,0xBA},   	
	{0x0413,0x00},
	{0x0414,0xBB},
	{0x0415,0x10},
	{0x0416,0xB8},   	
	{0x0417,0xCD},
	{0x0418,0xB7},
	{0x0419,0x5C},
	{0x041A,0xBB},   	
	{0x041B,0x6C},
	{0x01f8,0x3c},
	{0x01f9,0x00},
	{0x01FA,0x00},   	
	{0x02a2,0x3e},
	{0x02a3,0x00},
	{0x02a4,0x3e},
	{0x02a5,0x00},
	{0x02a6,0x3e},
	{0x02a7,0x00},
	{0x02a8,0x3e},
	{0x02a9,0x00},
	{0x056c,0x42},
	{0x056d,0x00},
	{0x056e,0x42},
	{0x056f,0x00},
	{0x0570,0x42},
	{0x0571,0x00},
	{0x0572,0x42},
	{0x0573,0x00},
	{0x0081,0x6E},
	{0x0588,0x00},   	
	{0x0589,0x5A},   	
	{0x058A,0xEE},   	
	{0x058B,0x69},   	
	{0x058C,0x49},   	
	{0x058D,0x3D},   	
	{0x058E,0x3D},   	
	{0x0080,0x6C},   	
	{0x0082,0x5A}, 
	//{0x0010,0x01},  
	//{REG_DLY,200},//mdelay(200)

	{0x4708,0x00},
	{0x4709,0x00},
	{0x4710,0x00},
	{0x4711,0x00},
/*	{0x065A,0x00},
	{0x06C9,0x01},
	{0x06CD,0x01},
	{0x06CE,0xBD},
	{0x06CF,0x00},
	{0x06D0,0x93},
	{0x06D1,0x02},
	{0x06D2,0x30},
	{0x06D3,0xD4},
	{0x06D4,0x01},
	{0x06D5,0x01},
	{0x06D6,0xBD},
	{0x06D7,0x00},
	{0x06D8,0x93},
	{0x06D9,0x00},
	{0x06DA,0x93},
	{0x06DB,0x59},	
	{0x06DC,0x0d},
	{0x0730,0x00},
	{0x0731,0x00},
	{0x0732,0x03},
	{0x0733,0xFF},
	{0x0734,0x03},
	{0x0735,0x70},
	{0x0755,0x01},
	{0x0756,0x00},
	{0x075B,0x01},
	{0x075E,0x00},
	{0x0764,0x01},
	{0x0766,0x01},
	{0x0768,0x01},
	{0x076A,0x00},
	{0x0758,0x01},
	{0x075C,0x01},
	{0x0770,0x98},
	{0x0771,0x19},
	{0x0772,0x1B},
	{0x0774,0x01},
	{0x0775,0x4a},
	{0x0777,0x00},
	{0x0778,0x45},
	{0x0779,0x00},
	{0x077A,0x02},
	{0x077D,0x01},
	{0x077E,0x03},
	{0x0783,0x10},
	{0x0785,0x14},
	{0x0788,0x04},
	{0x0846,0x06},
	{0x0847,0x05},
  */
	{0xC41A,0x05},
	{0xC423,0x11},
	{0xC427,0x11},
	{0x300B,0x09},  	
	{0x0085,0x02},//yuv order 
	{0x7000,0x08}, 
	{0x5200,0x09},  
	{0x00B5,0x02}, 
	{0x0030,0x14}, 
	#if 1
	// 1280*960
	{0x0040,0x01}, 
	{0x0041,0x0a},  
	
	{0x0042,0x05}, 	// 1280
	{0x0043,0x00},	
	{0x0044,0x02}, // 960
	{0x0045,0xc0},	
	{0x00B4,0x01},  
	#else
    // vga
	{0x0040,0x01}, 
	{0x0041,0x04},  
	{0x00B4,0x01},  
    #endif

#if 1
	{0x06D5,0x01},  // 2012-12-14 , 
	{0x06D6,0xbd},	// @150215 default:0x60
	{0x06D7,0x00},
	{0x06D8,0x93},	// @150215 default:0x93
	{0x06D9,0x00},
	{0x06DA,0x93},	// @150215 default:0x93 	
#endif

	{0x065A,0x00}, // AFStatsControls->bWindowsSystem = 7 zone AF system 

	{0x06C9,0x01}, // FLADriverLowLevelParameters->AutoSkipNextFrame = ENABLED
	{0x06CD,0x01}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefMacro MSB = 445
	{0x06CE,0xBD}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefMacro LSB
	{0x06CF,0x00}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefInfinity MSB = 147
	{0x06D0,0x93}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefInfinity LSB
	{0x06D1,0x02}, // FLADriverLowLevelParameters->AF_OTP_bStepsMultiStepDriver = 2 step driver
	{0x06D2,0x30}, // FLADriverLowLevelParameters->AF_OTP_uwMultiStepTimeDelay MSB = 12.5ms
	{0x06D3,0xD4}, // FLADriverLowLevelParameters->AF_OTP_uwMultiStepTimeDelay LSB
	{0x06D4,0x01}, // FLADriverLowLevelParameters->AF_OTP_fHostEnableOTPRead (1 = disabled)
	{0x06DB,0x59}, // FLADriverLowLevelParameters->fpActuatorResponseTime MSB 12.5ms (FP900) 
	{0x06DC,0x0d}, // FLADriverLowLevelParameters->fpActuatorResponseTime LSB

	{0x0730,0x00}, // FocusRangeConstants->wFullRange_LensMinPosition MSB = 0
	{0x0731,0x00}, // FocusRangeConstants->wFullRange_LensMinPosition LSB
	{0x0732,0x03}, // FocusRangeConstants->wFullRange_LensMaxPosition MSB = 1023
	{0x0733,0xFF}, // FocusRangeConstants->wFullRange_LensMaxPosition LSB
	{0x0734,0x03}, // FocusRangeConstants->wFullRange_LensRecoveryPosition MSB = 880
	{0x0735,0x70}, // FocusRangeConstants->wFullRange_LensRecoveryPosition LSB

	{0x0755,0x01}, // AutoFocusControls->fEnableSimpleCoarseThEvaluation = ENABLED
	{0x0756,0x03}, // AutoFocusControls->bSelectedMultizoneBehavior = REGIONSELECTIONMETHOD_AVERAGE //mk change to 0x3
	{0x075B,0x01}, // AutoFocusControls->fEnableTrackingThresholdEvaluation = DISABLED //MK change to 0x1
	{0x075E,0x00}, // AutoFocusControls->fFineToCoarseAutoTransitionEnable = DISABLED
	{0x0764,0x01}, // AutoFocusControls->fResetHCSPos = TRUE = Start from Recovery Position for every HCS
	{0x0766,0x01}, // AutoFocusControls->fEnablePrioritiesMacro = FALSE = Do not prioritise Macro //mk change to 0x01
	{0x0768,0x01}, // AutoFocusControls->fEnableInterpolationAfterFineSearch = TRUE
	{0x076A,0x00}, // AutoFocusControls->fReducedZoneSetup = TRUE //mk change to 0x0

	{0x0758,0x01}, // AutoFocusControls->bWeighedFunctionSelected = TRAPEZIUM
	{0x075C,0x01}, // AutoFocusControls->fEnableHeuristicMethod = FALSE

	{0x0770,0x98}, // AutoFocusConstants->bCoarseStep = 95
	{0x0771,0x19}, // AutoFocusConstants->bFineStep = 16
	{0x0772,0x1B}, // AutoFocusConstants->bFullSearchStep = 27
	{0x0774,0x01}, // AutoFocusConstants->uwFineThreshold MSB = 330 
	{0x0775,0x4a}, // AutoFocusConstants->uwFineThreshold LSB 
	{0x0777,0x00}, // AutoFocusConstants->uwBacklightThreshold MSB = 69
	{0x0778,0x45}, // AutoFocusConstants->uwBacklightThreshold LSB 
	{0x0779,0x00}, // AutoFocusConstants->uwMotionBlurInRatio MSB = 2
	{0x077A,0x02}, // AutoFocusConstants->uwMotionBlurInRatio LSB
	{0x077D,0x01}, // AutoFocusConstants->bMaxNumberContinuouslyInstableTime = 1
	{0x077E,0x02}, // AutoFocusConstants->bMaxNumberContinuouslyStableFrame = 3
	{0x0783,0x0A}, // AutoFocusConstants->bLightGap = 10 
	{0x0785,0x14}, // AutoFocusConstants->uwDeltaValue = 20
	{0x0788,0x04}, // AutoFocusConstants->bMinNumberMacroRegion = 4 //mk add

	{0x06CD,0x02},
	{0x06CE,0x00},
	{0x06CF,0x00},
	{0x06D0,0x00},
	{0x06D1,0x02},
	{0x06D2,0x30},
	{0x06D3,0xD4},
	{0x06D4,0x01},
	{0x06DB,0x59},
	{0x06DC,0x0D},
	{0x0730,0x00},
	{0x0731,0x00},
	{0x0732,0x03},
	{0x0733,0xFF},
	{0x0734,0x03},
	{0x0735,0xFF},
	{0x0770,0xA0},
	{0x0771,0x15},
	{0x077E,0x03},
	{0x0783,0x15},
	{0x0756,0x00},
  	
  {0x331e,0x03},

	{0x0846,0x06}, // AutoFocusHeuristicConstants->bHighToMaxFMShiftFactor = 6
	{0x0847,0x05}, // AutoFocusHeuristicConstants->bLowToHighFMShiftFactor = 5

	//{0x0010,0x00},
  	{0x070a,0x01},
};

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
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_CB_Y_CR_Y,
	},
	{
		.code              = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_CR_Y_CB_Y,
	},
	{
		.code              = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CB_Y_CR,
	},
	{
		.code              = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
	},
	{
		.code              = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RGB_565,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
	},
	{
		.code              = MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RGB_555,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
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
		struct v4l2_ctrl *auto_exposure;
		struct v4l2_ctrl *metering;
		struct v4l2_ctrl *exposure_bias;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *d_gain;
		struct v4l2_ctrl *a_gain;
	};
	struct {
		struct v4l2_ctrl *wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *focus_auto;
		struct v4l2_ctrl *af_start;
		struct v4l2_ctrl *af_stop;
		struct v4l2_ctrl *af_status;
		struct v4l2_ctrl *af_distance;
		struct v4l2_ctrl *focus_relative;
	};
	struct v4l2_ctrl *aaa_lock;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *pl_freq;
	struct v4l2_ctrl *colorfx;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *gamma;
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

	//v4l2_info(&sensor->sd, "wr: %04x <= %*ph\n", (u32)start_index,
		  //data_size, data);

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

	//v4l2_info(&sensor->sd, "rd: %04x => %*ph\n", (u32)start_index,
		  //data_size, data);

	return 0;
}

#define hm5065_read(s, r, v) _hm5065_read(s, #r, r, v)
#define hm5065_write(s, r, v) _hm5065_write(s, #r, r, v)
#define hm5065_read16(s, r, v) _hm5065_read16(s, #r, r, v)
#define hm5065_write16(s, r, v) _hm5065_write16(s, #r, r, v)

static int _hm5065_read(struct hm5065_dev *sensor, const char* reg_name,
			u16 reg, u8 *val)
{
	int ret = hm5065_read_regs(sensor, reg, val, 1);

	v4l2_info(&sensor->sd, "READ8: %s => 0x%02x\n", reg_name, *val);

	return ret;
}

static int _hm5065_write(struct hm5065_dev *sensor, const char* reg_name,
			 u16 reg, u8 val)
{
	v4l2_info(&sensor->sd, "WRITE8: %s <= 0x%02x\n", reg_name, (int)val);

	return hm5065_write_regs(sensor, reg, &val, 1);
}

static int _hm5065_read16(struct hm5065_dev *sensor, const char* reg_name,
			  u16 reg, u16 *val)
{
	int ret;

	ret = hm5065_read_regs(sensor, reg, (u8 *)val, sizeof(*val));
	if (ret)
		return ret;

	*val = be16_to_cpu(*val);
	v4l2_info(&sensor->sd, "READ16: %s <= 0x%04x\n", reg_name, (int)*val);
	return 0;
}

static int _hm5065_write16(struct hm5065_dev *sensor, const char* reg_name,
			   u16 reg, u16 val)
{
	u16 tmp = cpu_to_be16(val);

	v4l2_info(&sensor->sd, "WRITE16: %s <= 0x%04x\n", reg_name, (int)val);

	return hm5065_write_regs(sensor, reg, (u8 *)&tmp, sizeof(tmp));
}

static int hm5065_write_list(struct hm5065_dev *sensor, unsigned int list_size,
			     const struct reg_value *list)
{
	int ret;
	unsigned int i = 0;
	u16 start, len;
        u8 buf[128];

	/* we speed up I2C communication via auto-increment functionality */
	while (i < list_size) {
		start = list[i].addr;
		len = 0;

		while (i < list_size && list[i].addr == (start + len) &&
		       len < sizeof(buf))
			buf[len++] = list[i++].value;

		ret = hm5065_write_regs(sensor, start, buf, len);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * The firmware format:
 * <record 0>, ..., <record N - 1>
 * "record" is a 2-byte register address (big endian) followed by 1-byte data
 */
static int hm5065_load_firmware(struct hm5065_dev *sensor, const char *name)
{
	int ret = 0, i = 0, list_size;
	const struct firmware *fw;
	struct reg_value* list;
	u16 start, len;
        u8 buf[128];

	ret = request_firmware(&fw, name, sensor->sd.v4l2_dev->dev);
	if (ret) {
		v4l2_err(&sensor->sd, "Failed to read firmware %s (%d)\n", name,
			 ret);
		return ret;
	}

	if (fw->size % 3 != 0) {
		v4l2_err(&sensor->sd, "Firmware image %s has invalid size\n",
			 name);
		ret = -EINVAL;
		goto err_release;
	}

	list_size = fw->size / 3;
	list = (struct reg_value*)fw->data;

	/* we speed up I2C communication via auto-increment functionality */
	while (i < list_size) {
		start = be16_to_cpu(list[i].addr);
		len = 0;

		while (i < list_size &&
		       be16_to_cpu(list[i].addr) == (start + len) &&
		       len < sizeof(buf))
			buf[len++] = list[i++].value;

		ret = hm5065_write_regs(sensor, start, buf, len);
		if (ret)
			goto err_release;
	}

err_release:
	release_firmware(fw);
	return ret;
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
static s64 hm5065_mili_from_fp16(u16 fp_val)
{
	s64 val;
	s64 mantisa = fp_val & 0x1ff;
	int exp = (int)((fp_val >> 9) & 0x3f) - 31;

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

static int hm5065_get_af_status(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	u8 in_focus, is_stable, mode;
	int ret;

	ret = hm5065_read(sensor, HM5065_REG_AF_MODE_STATUS, &mode);
	if (ret)
		return ret;

	if (mode == HM5065_REG_AF_MODE_MANUAL) {
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_IDLE;
		return 0;
	}

	ret = hm5065_read(sensor, HM5065_REG_AF_IN_FOCUS, &in_focus);
	if (ret)
		return ret;

	ret = hm5065_read(sensor, HM5065_REG_AF_IS_STABLE, &is_stable);
	if (ret)
		return ret;

	if (in_focus && is_stable)
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_REACHED;
	else if (!in_focus && !is_stable)
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_BUSY;
	else
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_FAILED;

	return 0;
}

static int hm5065_get_exposure(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	u16 again, dgain, exp;
	int ret;

	ret = hm5065_read16(sensor, HM5065_REG_ANALOG_GAIN_PENDING, &again);
	if (ret)
		return ret;

	ret = hm5065_read16(sensor, HM5065_REG_DIGITAL_GAIN_PENDING, &dgain);
	if (ret)
		return ret;

	ret = hm5065_read16(sensor, HM5065_REG_COMPILED_EXPOSURE_TIME_US, &exp);
	if (ret)
		return ret;

	//XXX: potential for overflow
	sensor->ctrls.exposure->val = (s32)hm5065_mili_from_fp16(exp) / 100000;
	sensor->ctrls.d_gain->val = clamp(hm5065_mili_from_fp16(exp),
					  1000ll, 3000ll);
	//XXX: what unit is it in?
	//sensor->ctrls.a_gain->val = hm5065_mili_from_fp16(exp) / 1000;
	return 0;
}

static int hm5065_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	if (!sensor->powered)
		return -EIO;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_AUTO:
		ret = hm5065_get_af_status(sensor);
		if (ret)
			return ret;
		break;
#if 0
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val == V4L2_EXPOSURE_MANUAL)
			return 0;

		ret = hm5065_get_exposure(sensor);
		if (ret < 0)
			return ret;
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

		ret = hm5065_write(sensor, HM5065_REG_FD_ENABLE_DETECT, 1);
		if (ret)
			return ret;

		freq = (val == V4L2_CID_POWER_LINE_FREQUENCY_50HZ) ?
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

#define AE_BIAS_MENU_DEFAULT_VALUE_INDEX 7
static const s64 ae_bias_menu_values[] = {
	-2100, -1800, -1500, -1200, -900, -600, -300,
	0, 300, 600, 900, 1200, 1500, 1800, 2100
};

static const s8 ae_bias_menu_reg_values[] = {
	-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7
};

static int hm5065_set_exposure(struct hm5065_dev *sensor, s32 val)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	bool auto_exposure = (val == V4L2_EXPOSURE_AUTO);
	int ret = 0;

	if (ctrls->auto_exposure->is_new) {
		ret = hm5065_write(sensor, HM5065_REG_EXPOSURE_MODE,
				   auto_exposure ?
				   HM5065_REG_EXPOSURE_MODE_AUTO :
				   HM5065_REG_EXPOSURE_MODE_COMPILED_MANUAL);
		if (ret)
			return ret;
	}

	if (auto_exposure && ctrls->metering->is_new) {
		if (ctrls->metering->val == V4L2_EXPOSURE_METERING_AVERAGE)
			ret = hm5065_write(sensor, HM5065_REG_EXPOSURE_METERING,
					   HM5065_REG_EXPOSURE_METERING_FLAT);
		else if (ctrls->metering->val ==
			 V4L2_EXPOSURE_METERING_CENTER_WEIGHTED)
			ret = hm5065_write(sensor, HM5065_REG_EXPOSURE_METERING,
					 HM5065_REG_EXPOSURE_METERING_CENTERED);
		else
			return -EINVAL;

		if (ret)
			return ret;
	}

	if (auto_exposure && ctrls->exposure_bias->is_new) {
		s32 bias = ctrls->exposure_bias->val;

		if (bias < 0 || bias >= ARRAY_SIZE(ae_bias_menu_reg_values))
			return -EINVAL;

		ret = hm5065_write(sensor, HM5065_REG_EXPOSURE_COMPENSATION,
				   (u8)ae_bias_menu_reg_values[bias]);
		if (ret)
			return ret;
	}

	if (!auto_exposure && ctrls->exposure->is_new) {
                s32 val = ctrls->exposure->val;

		ret = hm5065_write16(sensor, HM5065_REG_MANUAL_EXPOSURE_TIME_US,
				     hm5065_mili_to_fp16(val * 100000));
	}

	return ret;
}

static int hm5065_3a_lock(struct hm5065_dev *sensor, struct v4l2_ctrl *ctrl)
{
	bool awb_lock = ctrl->val & V4L2_LOCK_WHITE_BALANCE;
	bool ae_lock = ctrl->val & V4L2_LOCK_EXPOSURE;
	bool af_lock = ctrl->val & V4L2_LOCK_FOCUS;
	int ret = 0;

	if ((ctrl->val ^ ctrl->cur.val) & V4L2_LOCK_EXPOSURE
	    && sensor->ctrls.auto_exposure->val == V4L2_EXPOSURE_AUTO) {
		ret = hm5065_write(sensor, HM5065_REG_FREEZE_AUTO_EXPOSURE,
				   ae_lock);
		if (ret)
			return ret;
	}

	if (((ctrl->val ^ ctrl->cur.val) & V4L2_LOCK_WHITE_BALANCE)
	    && sensor->ctrls.wb->val == V4L2_WHITE_BALANCE_AUTO) {
		ret = hm5065_write(sensor, HM5065_REG_WB_MISC_SETTINGS,
				   awb_lock ?
				   HM5065_REG_WB_MISC_SETTINGS_FREEZE_ALGO : 0);
		if (ret)
			return ret;
	}

	//XXX: potentially drop this
	/*
	if ((ctrl->val ^ ctrl->cur.val) & V4L2_LOCK_FOCUS
	    && sensor->ctrls.focus_auto->val)
		ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
				   af_lock ? HM5065_REG_AF_MODE_MANUAL :
				   HM5065_REG_AF_MODE_CONTINUOUS);
          */

	return ret;
}

static int hm5065_set_auto_focus(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	bool auto_focus = ctrls->focus_auto->val;
	int ret = 0;
	u8 range;
	s32 step = ctrls->focus_relative->val;

	ctrls->focus_relative->val = 0;

	if (auto_focus && ctrls->af_distance->is_new) {
		switch (ctrls->af_distance->val) {
		case V4L2_AUTO_FOCUS_RANGE_MACRO:
			range = HM5065_REG_AF_RANGE_MACRO;
			break;
		case V4L2_AUTO_FOCUS_RANGE_AUTO:
			range = HM5065_REG_AF_RANGE_FULL;
			break;
		case V4L2_AUTO_FOCUS_RANGE_INFINITY:
			range = HM5065_REG_AF_RANGE_LANDSCAPE;
			break;
		default:
			return -EINVAL;
		}

		ret = hm5065_write(sensor, HM5065_REG_AF_RANGE, range);
		if (ret)
			return ret;
	}

	if (ctrls->focus_auto->is_new) {
		ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
				   auto_focus ?
				   HM5065_REG_AF_MODE_CONTINUOUS :
				   HM5065_REG_AF_MODE_SINGLE);
		if (ret)
			return ret;

		if (!auto_focus) {
			ret = hm5065_write(sensor, HM5065_REG_AF_COMMAND,
					   HM5065_REG_AF_COMMAND_RELEASED_BUTTON);
			if (ret)
				return ret;
		}
	}

	if (!auto_focus && ctrls->af_start->is_new) {
		ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
				   HM5065_REG_AF_MODE_SINGLE);
		if (ret)
			return ret;

		ret = hm5065_write(sensor, HM5065_REG_AF_COMMAND,
				   HM5065_REG_AF_COMMAND_RELEASED_BUTTON);
		if (ret)
			return ret;

		usleep_range(190000, 200000);

		ret = hm5065_write(sensor, HM5065_REG_AF_COMMAND,
				   HM5065_REG_AF_COMMAND_HALF_BUTTON);
		if (ret)
			return ret;
	}

	if (!auto_focus && ctrls->af_stop->is_new) {
		// stop single focus op
		ret = hm5065_write(sensor, HM5065_REG_AF_COMMAND,
				   HM5065_REG_AF_COMMAND_RELEASED_BUTTON);
		if (ret)
			return ret;

		ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
				   HM5065_REG_AF_MODE_MANUAL);
		if (ret)
			return ret;
	}

	if (!auto_focus && ctrls->focus_relative->val) {
		u8 cmd = 0xff;

		ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
				   HM5065_REG_AF_MODE_MANUAL);
		if (ret)
			return ret;

		ret = hm5065_write(sensor, HM5065_REG_AF_MANUAL_STEP_SIZE,
				   abs(step));
		if (ret)
			return ret;

		if (step < 0)
			cmd = HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_INFINITY;
		else if (step > 0)
			cmd = HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_MACRO;

		if (cmd != 0xff)
			ret = hm5065_write(sensor, HM5065_REG_AF_LENS_COMMAND,
					   cmd);

		if (ret)
			return ret;
	}

	return ret;
}

static const u16 analog_gain_table[][2] = {
	/* code, 1/100th of dB */
	{ 0x00,    0 },
	{ 0x10,   56 },
	{ 0x20,  116 },
	{ 0x30,  180 },
	{ 0x40,  250 },
	{ 0x50,  325 },
	{ 0x60,  410 },
	{ 0x70,  500 },
	{ 0x80,  600 },
	{ 0x90,  720 },
	{ 0xA0,  850 },
	{ 0xB0, 1010 },
	{ 0xC0, 1200 },
	{ 0xD0, 1450 },
	{ 0xE0, 1810 },
	{ 0xE4, 1920 },
	{ 0xE8, 2060 },
	{ 0xEC, 2210 },
	{ 0xF0, 2410 },
};

static int hm5065_set_analog_gain(struct hm5065_dev *sensor, s32 val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(analog_gain_table); i++)
		if (val <= analog_gain_table[i][1])
			break;

	if (i == ARRAY_SIZE(analog_gain_table))
		i--;

	return hm5065_write16(sensor, HM5065_REG_DIRECT_MODE_CODED_ANALOG_GAIN,
			      analog_gain_table[i][0]);
}

static int hm5065_set_digital_gain(struct hm5065_dev *sensor, s32 val)
{
	return hm5065_write16(sensor, HM5065_REG_DIRECT_MODE_DIGITAL_GAIN,
			      hm5065_mili_to_fp16(val));
}

static int hm5065_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	s32 val = ctrl->val;
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
	case V4L2_CID_EXPOSURE_AUTO:
		return hm5065_set_exposure(sensor, val);

	case V4L2_CID_DIGITAL_GAIN:
		return hm5065_set_digital_gain(sensor, val);

	case V4L2_CID_ANALOGUE_GAIN:
		return hm5065_set_analog_gain(sensor, val);

	case V4L2_CID_FOCUS_AUTO:
		return hm5065_set_auto_focus(sensor);

	case V4L2_CID_CONTRAST:
		return hm5065_write(sensor, HM5065_REG_CONTRAST, val);

	case V4L2_CID_SATURATION:
		return hm5065_write(sensor, HM5065_REG_COLOR_SATURATION, val);

	case V4L2_CID_BRIGHTNESS:
		return hm5065_write(sensor, HM5065_REG_BRIGHTNESS, val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return hm5065_set_power_line_frequency(sensor, val);

	case V4L2_CID_GAMMA:
		return hm5065_write(sensor, HM5065_REG_P0_GAMMA_GAIN, val);

	case V4L2_CID_VFLIP:
		return hm5065_write(sensor, HM5065_REG_VERTICAL_FLIP,
				    val ? 1 : 0);

	case V4L2_CID_HFLIP:
		return hm5065_write(sensor, HM5065_REG_HORIZONTAL_MIRROR,
				    val ? 1 : 0);

	case V4L2_CID_COLORFX:
		return hm5065_set_colorfx(sensor, val);

	case V4L2_CID_3A_LOCK:
		return hm5065_3a_lock(sensor, ctrl);

	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		for (i = 0; i < ARRAY_SIZE(hm5065_wb_opts); i++) {
			if (hm5065_wb_opts[i][0] != val)
				continue;

			return hm5065_write(sensor, HM5065_REG_WB_MODE,
					    hm5065_wb_opts[i][1]);
		}

		return -EINVAL;

	case V4L2_CID_BLUE_BALANCE:
		return hm5065_write16(sensor, HM5065_REG_WB_HUE_B_BIAS,
				      hm5065_mili_to_fp16(val));

	case V4L2_CID_RED_BALANCE:
		return hm5065_write16(sensor, HM5065_REG_WB_HUE_R_BIAS,
				      hm5065_mili_to_fp16(val));

	case V4L2_CID_TEST_PATTERN_RED:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_RED, val);

	case V4L2_CID_TEST_PATTERN_GREENR:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_GREEN_R, val);

	case V4L2_CID_TEST_PATTERN_BLUE:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_BLUE, val);

	case V4L2_CID_TEST_PATTERN_GREENB:
		return hm5065_write16(sensor, HM5065_REG_TESTDATA_GREEN_B, val);

	case V4L2_CID_TEST_PATTERN:
		for (i = 0; i < ARRAY_SIZE(ctrls->test_data); i++)
			v4l2_ctrl_activate(ctrls->test_data[i],
					   val == 6); /* solid color */

		ret = hm5065_write(sensor, HM5065_REG_ENABLE_TEST_PATTERN,
				   val == 0 ? 0 : 1);
		if (ret)
			return ret;

		return hm5065_write(sensor, HM5065_REG_TEST_PATTERN, val);

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
	"PN9 (bus test)",
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

	ctrls->auto_exposure = v4l2_ctrl_new_std_menu(hdl, ops,
						      V4L2_CID_EXPOSURE_AUTO,
						      V4L2_EXPOSURE_MANUAL, 0,
						      V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops,
					    V4L2_CID_EXPOSURE_ABSOLUTE,
					    1, 10000, 1, 100);
					    /* don't raise max value! */
	ctrls->metering =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_EXPOSURE_METERING,
				       V4L2_EXPOSURE_METERING_CENTER_WEIGHTED,
				       0, V4L2_EXPOSURE_METERING_AVERAGE);
	ctrls->exposure_bias =
		v4l2_ctrl_new_int_menu(hdl, ops,
				       V4L2_CID_AUTO_EXPOSURE_BIAS,
				       ARRAY_SIZE(ae_bias_menu_values) - 1,
				       AE_BIAS_MENU_DEFAULT_VALUE_INDEX,
				       ae_bias_menu_values);

	ctrls->d_gain = v4l2_ctrl_new_std(hdl, ops,
					  V4L2_CID_DIGITAL_GAIN,
					  1000, 3000, 1, 1000);

	ctrls->a_gain = v4l2_ctrl_new_std(hdl, ops,
					  V4L2_CID_ANALOGUE_GAIN,
					  0, 2410, 1, 0);

	for (i = 0; i < ARRAY_SIZE(hm5065_wb_opts); i++) {
		if (wb_max < hm5065_wb_opts[i][0])
			wb_max = hm5065_wb_opts[i][0];
		wb_mask |= BIT(hm5065_wb_opts[i][0]);
	}

	ctrls->wb = v4l2_ctrl_new_std_menu(hdl, ops,
			V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
			wb_max, ~wb_mask, V4L2_WHITE_BALANCE_AUTO);

	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4000, 1, 1000);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4000, 1, 1000);

	ctrls->gamma = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAMMA,
					 0, 31, 1, 20);

	ctrls->colorfx =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_COLORFX, 15,
				       ~(BIT(V4L2_COLORFX_NONE) |
					 BIT(V4L2_COLORFX_NEGATIVE) |
					 BIT(V4L2_COLORFX_SOLARIZATION) |
					 BIT(V4L2_COLORFX_SKETCH) |
					 BIT(V4L2_COLORFX_SEPIA) |
					 BIT(V4L2_COLORFX_ANTIQUE) |
					 BIT(V4L2_COLORFX_AQUA) |
					 BIT(V4L2_COLORFX_BW)),
				       V4L2_COLORFX_NONE);

	ctrls->pl_freq =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_POWER_LINE_FREQUENCY,
				V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
				V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* Auto focus */
	ctrls->focus_auto = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_AUTO,
					      0, 1, 1, 1);

	ctrls->af_start = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTO_FOCUS_START,
					    0, 1, 1, 0);

	ctrls->af_stop = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTO_FOCUS_STOP,
					   0, 1, 1, 0);

	ctrls->af_status = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_AUTO_FOCUS_STATUS, 0,
					     (V4L2_AUTO_FOCUS_STATUS_BUSY |
					      V4L2_AUTO_FOCUS_STATUS_REACHED |
					      V4L2_AUTO_FOCUS_STATUS_FAILED),
					     0, V4L2_AUTO_FOCUS_STATUS_IDLE);

	ctrls->af_distance =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_AUTO_FOCUS_RANGE,
				       V4L2_AUTO_FOCUS_RANGE_MACRO,
				       ~(BIT(V4L2_AUTO_FOCUS_RANGE_AUTO) |
					 BIT(V4L2_AUTO_FOCUS_RANGE_INFINITY) |
					 BIT(V4L2_AUTO_FOCUS_RANGE_MACRO)),
				       V4L2_AUTO_FOCUS_RANGE_AUTO);

	ctrls->focus_relative = v4l2_ctrl_new_std(hdl, ops,
						  V4L2_CID_FOCUS_RELATIVE,
						  -100, 100, 1, 0);

	ctrls->brightness = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS,
					      0, 200, 1, 90);
	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 200, 1, 110);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 200, 1, 108);

	ctrls->aaa_lock = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK,
					    0, 0x7, 0, 0);

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
	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;
#endif
	ctrls->af_status->flags |= V4L2_CTRL_FLAG_VOLATILE |
		V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_auto_cluster(4, &ctrls->auto_exposure, V4L2_EXPOSURE_MANUAL,
			       false);
	v4l2_ctrl_cluster(6, &ctrls->focus_auto);

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

	if (fi->pad != 0)
		return -EINVAL;

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
	fi->interval = sensor->frame_interval;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int hm5065_setup_mode(struct hm5065_dev *sensor)
{
	int ret;
	const struct hm5065_pixfmt *pix_fmt;

	/*
	ret = hm5065_write(sensor, HM5065_REG_USER_COMMAND,
			   HM5065_REG_USER_COMMAND_POWEROFF);
	if (ret)
		return ret;

	ret = hm5065_write(sensor, HM5065_REG_P0_SENSOR_MODE,
			   HM5065_REG_SENSOR_MODE_FULLSIZE);
	if (ret)
		return ret;

	ret = hm5065_write16(sensor, HM5065_REG_P0_MANUAL_HSIZE,
			     sensor->fmt.width);
	if (ret)
		return ret;

	ret = hm5065_write16(sensor, HM5065_REG_P0_MANUAL_VSIZE,
			     sensor->fmt.height);
	if (ret)
		return ret;

	ret = hm5065_write(sensor, HM5065_REG_P0_IMAGE_SIZE,
			   HM5065_REG_IMAGE_SIZE_MANUAL);
	if (ret)
		return ret;
          */
	pix_fmt = hm5065_find_format(sensor->fmt.code);
	if (!pix_fmt) {
		dev_err(&sensor->i2c_client->dev,
			"pixel format not supported %u\n",
			sensor->fmt.code);
		return -EINVAL;
	}

	struct reg_value setup_mode[] = {
		{HM5065_REG_USER_COMMAND, HM5065_REG_USER_COMMAND_POWEROFF},
		{0x7000, 0x08},
		{0x5200, 0x09},
		//{0x00ed, sensor->frame_interval.denominator},
		//{0x00ee, sensor->frame_interval.denominator},
		{HM5065_REG_P0_SENSOR_MODE, HM5065_REG_SENSOR_MODE_FULLSIZE},
		{HM5065_REG_P0_MANUAL_HSIZE, sensor->fmt.width >> 8},
		{HM5065_REG_P0_MANUAL_HSIZE + 1, sensor->fmt.width},
		{HM5065_REG_P0_MANUAL_VSIZE, sensor->fmt.height >> 8},
		{HM5065_REG_P0_MANUAL_VSIZE + 1, sensor->fmt.height},
		{HM5065_REG_P0_IMAGE_SIZE, HM5065_REG_IMAGE_SIZE_MANUAL},
		{HM5065_REG_P0_DATA_FORMAT, pix_fmt->data_fmt},
		{HM5065_REG_YCRCB_ORDER, pix_fmt->ycbcr_order},
		{0x0030, 0x11},
	};

	ret = hm5065_write_list(sensor, ARRAY_SIZE(setup_mode), setup_mode);
	if (ret)
		return ret;

	ret = hm5065_write16(sensor, HM5065_REG_DESIRED_FRAME_RATE_NUM,
			     sensor->frame_interval.denominator);
	if (ret)
		return ret;

	return 0;
}

static int hm5065_set_stream(struct hm5065_dev *sensor, int enable)
{
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

		if (enable && sensor->ctrls.focus_auto->cur.val) {
			msleep(20);
			ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
					   HM5065_REG_AF_MODE_CONTINUOUS);
		}

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

static int hm5065_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct v4l2_fract tpf;
	int max_fps, i;

	if (fie->pad != 0)
		return -EINVAL;

	for (i = 0; i < HM5065_NUM_FRAME_SIZES; i++) {
		if (hm5065_frame_sizes[i].width == fie->width &&
		    hm5065_frame_sizes[i].height == fie->height)
			goto get_max_fps;
	}

	return -EINVAL;

get_max_fps:
	mutex_lock(&sensor->lock);
	max_fps = sensor->max_pixel_rate / fie->width / fie->height;
	mutex_unlock(&sensor->lock);
	max_fps = clamp(max_fps, 1, HM5065_FRAME_RATE_MAX);

	if (fie->index + 1 > max_fps)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = fie->index + 1;

	fie->interval = tpf;
	return 0;
}

static int hm5065_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_subdev_frame_interval fi;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	fi.pad = 0;
	ret = hm5065_g_frame_interval(sd, &fi);
	if (ret)
		return ret;

	cp->timeperframe = fi.interval;
	return 0;
}

static int hm5065_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_subdev_frame_interval fi;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fi.pad = 0;
	fi.interval = cp->timeperframe;
	cp->capability = V4L2_CAP_TIMEPERFRAME;

	ret = hm5065_s_frame_interval(sd, &fi);
	if (ret)
		return ret;

	cp->timeperframe = fi.interval;
	return 0;
}

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
	lut = hm5065_find_clk_lut(xclk_freq);
	if (!lut) {
		dev_err(&sensor->i2c_client->dev,
			"xclk frequency out of range: %lu Hz\n", xclk_freq);
		return -EINVAL;
	}

	ret = hm5065_write(sensor, HM5065_REG_EXCLOCKLUT, lut->lut_id);
	if (ret)
		return ret;

	/* PLL output = 480MHz */
	ret = hm5065_write16(sensor, HM5065_REG_TARGET_PLL_OUTPUT,
			     hm5065_mili_to_fp16(720000));
	if (ret)
		return ret;

	ret = hm5065_load_firmware(sensor, HM5065_AF_FIRMWARE);
	if (ret)
		return ret;

	mdelay(200);

	ret = hm5065_load_firmware(sensor, HM5065_FIRMWARE_PARAMETERS);
	if (ret)
		return ret;

	mdelay(100);
	return 0;
}

static int hm5065_set_power(struct hm5065_dev *sensor, bool on)
{
	int ret = 0;

	if (on) {
		dev_dbg(&sensor->i2c_client->dev, "%s: on\n", __func__);

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
	dev_dbg(&sensor->i2c_client->dev, "%s: off\n", __func__);
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
	u8 val = 0;

	if (reg->reg > 0xffff)
		return -EINVAL;

	reg->size = 1;
	ret = hm5065_read(sensor, reg->reg, &val);
	if (ret)
		return -EIO;

	reg->val = val;
	return 0;
}

static int hm5065_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);

	if (reg->reg > 0xffff || reg->val > 0xff)
		return -EINVAL;

	return hm5065_write(sensor, reg->reg, reg->val);
}
#endif

/* }}} */

static const struct v4l2_subdev_core_ops hm5065_core_ops = {
	.s_power = hm5065_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = hm5065_g_register,
	.s_register = hm5065_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops hm5065_pad_ops = {
	.enum_mbus_code = hm5065_enum_mbus_code,
	.enum_frame_size = hm5065_enum_frame_size,
	.enum_frame_interval = hm5065_enum_frame_interval,
	.get_fmt = hm5065_get_fmt,
	.set_fmt = hm5065_set_fmt,
};

static const struct v4l2_subdev_video_ops hm5065_video_ops = {
	.g_frame_interval = hm5065_g_frame_interval,
	.s_frame_interval = hm5065_s_frame_interval,
	.g_parm = hm5065_g_parm,
	.s_parm = hm5065_s_parm,
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
