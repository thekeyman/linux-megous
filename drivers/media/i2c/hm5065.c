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

struct reg_value {
	u16 addr;
	u8 value;
} __attribute__ ((packed));

/* {{{ Auto-focus registers */

static const struct reg_value af_init_regs[] = {
{0xffff, 0x01}, // ; MCU bypass;
{0x9000, 0x03}, // ; Enable Ram and enable Write;
{0xa000, 0x90}, // MOV      DPTR,#fpInputRange(0x0C56)
{0xa001, 0x0c}, //
{0xa002, 0x56}, //
{0xa003, 0xe0}, // MOVX     A,@DPTR
{0xa004, 0xfe}, // MOV      R6,A
{0xa005, 0xa3}, // INC      DPTR
{0xa006, 0xe0}, // MOVX     A,@DPTR
{0xa007, 0xff}, // MOV      R7,A
{0xa008, 0x12}, // LCALL    FPAlu_FloatToInt16(C:4285)
{0xa009, 0x42}, //
{0xa00a, 0x85}, //
{0xa00b, 0x90}, // MOV      DPTR,#0x01B7  (0x0B4D)
{0xa00c, 0x01}, //
{0xa00d, 0xb7}, //
{0xa00e, 0xee}, // MOV      A,R6
{0xa00f, 0xf0}, // MOVX     @DPTR,A
{0xa010, 0xfc}, // MOV      R4,A
{0xa011, 0xa3}, // INC      DPTR
{0xa012, 0xef}, // MOV      A,R7
{0xa013, 0xf0}, // MOVX     @DPTR,A
{0xa014, 0xfd}, // MOV      R5,A
{0xa015, 0x90}, // MOV      DPTR,#0x0605
{0xa016, 0x06}, //
{0xa017, 0x05}, //
{0xa018, 0xe0}, // MOVX     A,@DPTR
{0xa019, 0x75}, // MOV      B(0xF0),#0x02
{0xa01a, 0xf0}, //
{0xa01b, 0x02}, //
{0xa01c, 0xa4}, // MUL      AB
{0xa01d, 0x2d}, // ADD      A,R5
{0xa01e, 0xff}, // MOV      R7,A
{0xa01f, 0xe5}, // MOV      A,B(0xF0)
{0xa020, 0xf0}, //
{0xa021, 0x3c}, // ADDC     A,R4
{0xa022, 0xfe}, // MOV      R6,A
{0xa023, 0xab}, // MOV      R3 07
{0xa024, 0x07}, //
{0xa025, 0xfa}, // MOV      R2,A
{0xa026, 0x33}, // RLC      A
{0xa027, 0x95}, // SUBB     A,ACC(0xE0)
{0xa028, 0xe0}, //
{0xa029, 0xf9}, // MOV      R1,A
{0xa02a, 0xf8}, // MOV      R0,A
{0xa02b, 0x90}, // MOV      DPTR,#0x0B4B
{0xa02c, 0x0b}, //
{0xa02d, 0x4b}, //
{0xa02e, 0xe0}, // MOVX     A,@DPTR
{0xa02f, 0xfe}, // MOV      R6,A
{0xa030, 0xa3}, // INC      DPTR
{0xa031, 0xe0}, // MOVX     A,@DPTR
{0xa032, 0xff}, // MOV      R7,A
{0xa033, 0xee}, // MOV      A,R6
{0xa034, 0x33}, // RLC      A
{0xa035, 0x95}, // SUBB     A,ACC(0xE0)
{0xa036, 0xe0}, //
{0xa037, 0xfd}, // MOV      R5,A
{0xa038, 0xfc}, // MOV      R4,A
{0xa039, 0x12}, // LCALL    C?LMUL(C:0C7B)
{0xa03a, 0x0c}, //
{0xa03b, 0x7b}, //
{0xa03c, 0x90}, // MOV      DPTR,#0x01B9(0x0B4F)
{0xa03d, 0x01}, //
{0xa03e, 0xb9}, //
{0xa03f, 0x12}, // LCALL    C?LSTXDATA(C:0E05)
{0xa040, 0x0e}, //
{0xa041, 0x05}, //
{0xa042, 0x90}, // MOV      DPTR,#0x01B9(0x0B4F)
{0xa043, 0x01}, //
{0xa044, 0xb9}, //
{0xa045, 0xe0}, // MOVX     A,@DPTR
{0xa046, 0xfc}, // MOV      R4,A
{0xa047, 0xa3}, // INC      DPTR
{0xa048, 0xe0}, // MOVX     A,@DPTR
{0xa049, 0xfd}, // MOV      R5,A
{0xa04a, 0xa3}, // INC      DPTR
{0xa04b, 0xe0}, // MOVX     A,@DPTR
{0xa04c, 0xfe}, // MOV      R6,A
{0xa04d, 0xa3}, // INC      DPTR
{0xa04e, 0xe0}, // MOVX     A,@DPTR
{0xa04f, 0xff}, // MOV      R7,A
{0xa050, 0x78}, // MOV      R0,#g_fTimer0TimeOut(0x08)
{0xa051, 0x08}, //
{0xa052, 0x12}, // LCALL    C?ULSHR(C:0DBF)
{0xa053, 0x0d}, //
{0xa054, 0xbf}, //
{0xa055, 0xa8}, // MOV      R0,uwDelay1000(0x04)
{0xa056, 0x04}, //
{0xa057, 0xa9}, // MOV      R1 05
{0xa058, 0x05}, //
{0xa059, 0xaa}, // MOV      R2,uwDelay100(0x06)
{0xa05a, 0x06}, //
{0xa05b, 0xab}, // MOV      R3 07
{0xa05c, 0x07}, //
{0xa05d, 0x90}, // MOV      DPTR,#0x0B49
{0xa05e, 0x0b}, //
{0xa05f, 0x49}, //
{0xa060, 0xe0}, // MOVX     A,@DPTR
{0xa061, 0xfe}, // MOV      R6,A
{0xa062, 0xa3}, // INC      DPTR
{0xa063, 0xe0}, // MOVX     A,@DPTR
{0xa064, 0xff}, // MOV      R7,A
{0xa065, 0xee}, // MOV      A,R6
{0xa066, 0x33}, // RLC      A
{0xa067, 0x95}, // SUBB     A,ACC(0xE0)
{0xa068, 0xe0}, //
{0xa069, 0xfd}, // MOV      R5,A
{0xa06a, 0xfc}, // MOV      R4,A
{0xa06b, 0xc3}, // CLR      C
{0xa06c, 0xef}, // MOV      A,R7
{0xa06d, 0x9b}, // SUBB     A,R3
{0xa06e, 0xff}, // MOV      R7,A
{0xa06f, 0xee}, // MOV      A,R6
{0xa070, 0x9a}, // SUBB     A,R2
{0xa071, 0xfe}, // MOV      R6,A
{0xa072, 0xed}, // MOV      A,R5
{0xa073, 0x99}, // SUBB     A,R1
{0xa074, 0xfd}, // MOV      R5,A
{0xa075, 0xec}, // MOV      A,R4
{0xa076, 0x98}, // SUBB     A,R0
{0xa077, 0xfc}, // MOV      R4,A
{0xa078, 0x78}, // MOV      R0,#0x01
{0xa079, 0x01}, //
{0xa07a, 0x12}, // LCALL    C?ULSHR(C:0DBF)
{0xa07b, 0x0d}, //
{0xa07c, 0xbf}, //
{0xa07d, 0x90}, // MOV      DPTR,#m_pxwOffsetVector(0x0C4A)
{0xa07e, 0x0c}, //
{0xa07f, 0x4a}, //
{0xa080, 0xe0}, // MOVX     A,@DPTR
{0xa081, 0xfc}, // MOV      R4,A
{0xa082, 0xa3}, // INC      DPTR
{0xa083, 0xe0}, // MOVX     A,@DPTR
{0xa084, 0xf5}, // MOV      DPL(0x82),A
{0xa085, 0x82}, //
{0xa086, 0x8c}, // MOV      DPH(0x83),R4
{0xa087, 0x83}, //
{0xa088, 0xc0}, // PUSH     DPH(0x83)
{0xa089, 0x83}, //
{0xa08a, 0xc0}, // PUSH     DPL(0x82)
{0xa08b, 0x82}, //
{0xa08c, 0x90}, // MOV      DPTR,#0x0B48
{0xa08d, 0x0b}, //
{0xa08e, 0x48}, //
{0xa08f, 0xe0}, // MOVX     A,@DPTR
{0xa090, 0xd0}, // POP      DPL(0x82)
{0xa091, 0x82}, //
{0xa092, 0xd0}, // POP      DPH(0x83)
{0xa093, 0x83}, //
{0xa094, 0x75}, // MOV      B(0xF0),#0x02
{0xa095, 0xf0}, //
{0xa096, 0x02}, //
{0xa097, 0x12}, // LCALL    C?OFFXADD(C:0E45)
{0xa098, 0x0e}, //
{0xa099, 0x45}, //
{0xa09a, 0xee}, // MOV      A,R6
{0xa09b, 0xf0}, // MOVX     @DPTR,A
{0xa09c, 0xa3}, // INC      DPTR
{0xa09d, 0xef}, // MOV      A,R7
{0xa09e, 0xf0}, // MOVX     @DPTR,A
{0xa09f, 0x02}, // LJMP     C:BAD8
{0xa0a0, 0xba}, //
{0xa0a1, 0xd8}, //
{0xa0a2, 0x90}, //
{0xa0a3, 0x30}, // MOV      DPTR,#0x0036
{0xa0a4, 0x18}, //
{0xa0a5, 0xe4}, //
{0xa0a6, 0xf0}, // MOV      A,#0x00
{0xa0a7, 0x74}, //
{0xa0a8, 0x3f}, // MOVX     @DPTR,A
{0xa0a9, 0xf0}, // INC      DPTR
{0xa0aa, 0x22}, // INC      DPTR
{0xa0bf, 0x90}, // MOV      DPTR,#0x005E
{0xa0c0, 0x00}, //
{0xa0c1, 0x5e}, //
{0xa0c2, 0xe0}, // MOVX     A,@DPTR
{0xa0c3, 0xff}, // MOV      R7,A
{0xa0c4, 0x70}, // JNZ      B00:A9AF
{0xa0c5, 0x20}, //
{0xa0c6, 0x90}, // MOV      DPTR,#Av2x2_H_Size(0x4704)
{0xa0c7, 0x47}, //
{0xa0c8, 0x04}, //
{0xa0c9, 0x74}, // MOV      A,#bInt_Event_Status(0x0A)
{0xa0ca, 0x0a}, //
{0xa0cb, 0xf0}, // MOVX     @DPTR,A
{0xa0cc, 0xa3}, // INC      DPTR
{0xa0cd, 0x74}, // MOV      A,#0x30
{0xa0ce, 0x30}, //
{0xa0cf, 0xf0}, // MOVX     @DPTR,A
{0xa0d0, 0x90}, // MOV      DPTR,#Av2x2_V_Size(0x470C)
{0xa0d1, 0x47}, //
{0xa0d2, 0x0c}, //
{0xa0d3, 0x74}, // MOV      A,#0x07
{0xa0d4, 0x07}, //
{0xa0d5, 0xf0}, // MOVX     @DPTR,A
{0xa0d6, 0xa3}, // INC      DPTR
{0xa0d7, 0x74}, // MOV      A,#IE(0xA8)
{0xa0d8, 0xa8}, //
{0xa0d9, 0xf0}, // MOVX     @DPTR,A
{0xa0da, 0x90}, // MOV      DPTR,#Av2x2_Xscale(0x47A4)
{0xa0db, 0x47}, //
{0xa0dc, 0xa4}, //
{0xa0dd, 0x74}, // MOV      A,#0x01
{0xa0de, 0x01}, //
{0xa0df, 0xf0}, // MOVX     @DPTR,A
{0xa0e0, 0x90}, // MOV      DPTR,#Av2x2_Yscale(0x47A8)
{0xa0e1, 0x47}, //
{0xa0e2, 0xa8}, //
{0xa0e3, 0xf0}, // MOVX     @DPTR,A
{0xa0e4, 0x80}, // SJMP     B00:A9FF
{0xa0e5, 0x50}, //
{0xa0e6, 0xef}, // MOV      A,R7
{0xa0e7, 0x64}, // XRL      A,#0x01
{0xa0e8, 0x01}, //
{0xa0e9, 0x60}, // JZ       B00:A9B8
{0xa0ea, 0x04}, //
{0xa0eb, 0xef}, // MOV      A,R7
{0xa0ec, 0xb4}, // CJNE     A,#0x03,B00:A9D8
{0xa0ed, 0x03}, //
{0xa0ee, 0x20}, //
{0xa0ef, 0x90}, // MOV      DPTR,#Av2x2_H_Size(0x4704)
{0xa0f0, 0x47}, //
{0xa0f1, 0x04}, //
{0xa0f2, 0x74}, // MOV      A,#0x05
{0xa0f3, 0x05}, //
{0xa0f4, 0xf0}, // MOVX     @DPTR,A
{0xa0f5, 0xa3}, // INC      DPTR
{0xa0f6, 0x74}, // MOV      A,#0x18
{0xa0f7, 0x18}, //
{0xa0f8, 0xf0}, // MOVX     @DPTR,A
{0xa0f9, 0x90}, // MOV      DPTR,#Av2x2_V_Size(0x470C)
{0xa0fa, 0x47}, //
{0xa0fb, 0x0c}, //
{0xa0fc, 0x74}, // MOV      A,#0x03
{0xa0fd, 0x03}, //
{0xa0fe, 0xf0}, // MOVX     @DPTR,A
{0xa0ff, 0xa3}, // INC      DPTR
{0xa100, 0x74}, // MOV      A,#m_fDitherBitFormat(0xD4)
{0xa101, 0xd4}, //
{0xa102, 0xf0}, // MOVX     @DPTR,A
{0xa103, 0x90}, // MOV      DPTR,#Av2x2_Xscale(0x47A4)
{0xa104, 0x47}, //
{0xa105, 0xa4}, //
{0xa106, 0x74}, // MOV      A,#0x02
{0xa107, 0x02}, //
{0xa108, 0xf0}, // MOVX     @DPTR,A
{0xa109, 0x90}, // MOV      DPTR,#Av2x2_Yscale(0x47A8)
{0xa10a, 0x47}, //
{0xa10b, 0xa8}, //
{0xa10c, 0xf0}, // MOVX     @DPTR,A
{0xa10d, 0x80}, // SJMP     B00:A9FF
{0xa10e, 0x27}, //
{0xa10f, 0xef}, // MOV      A,R7
{0xa110, 0x64}, // XRL      A,#0x02
{0xa111, 0x02}, //
{0xa112, 0x60}, // JZ       B00:A9E1
{0xa113, 0x04}, //
{0xa114, 0xef}, // MOV      A,R7
{0xa115, 0xb4}, // CJNE     A,#uwDelay1000(0x04),B00:A9FF
{0xa116, 0x04}, //
{0xa117, 0x1e}, //
{0xa118, 0x90}, // MOV      DPTR,#Av2x2_H_Size(0x4704)
{0xa119, 0x47}, //
{0xa11a, 0x04}, //
{0xa11b, 0x74}, // MOV      A,#0x02
{0xa11c, 0x02}, //
{0xa11d, 0xf0}, // MOVX     @DPTR,A
{0xa11e, 0xa3}, // INC      DPTR
{0xa11f, 0x74}, // MOV      A,#TH0(0x8C)
{0xa120, 0x8c}, //
{0xa121, 0xf0}, // MOVX     @DPTR,A
{0xa122, 0x90}, // MOV      DPTR,#Av2x2_V_Size(0x470C)
{0xa123, 0x47}, //
{0xa124, 0x0c}, //
{0xa125, 0x74}, // MOV      A,#0x01
{0xa126, 0x01}, //
{0xa127, 0xf0}, // MOVX     @DPTR,A
{0xa128, 0xa3}, // INC      DPTR
{0xa129, 0x74}, // MOV      A,#0xEA
{0xa12a, 0xea}, //
{0xa12b, 0xf0}, // MOVX     @DPTR,A
{0xa12c, 0x90}, // MOV      DPTR,#Av2x2_Xscale(0x47A4)
{0xa12d, 0x47}, //
{0xa12e, 0xa4}, //
{0xa12f, 0x74}, // MOV      A,#uwDelay1000(0x04)
{0xa130, 0x04}, //
{0xa131, 0xf0}, // MOVX     @DPTR,A
{0xa132, 0x90}, // MOV      DPTR,#Av2x2_Yscale(0x47A8)
{0xa133, 0x47}, //
{0xa134, 0xa8}, //
{0xa135, 0xf0}, // MOVX     @DPTR,A
{0xa136, 0x22}, // RTN
{0xa137, 0x74}, // MOV      A,#uwDelay1000(0x04)
{0xa138, 0x04}, //
{0xa139, 0xf0}, // MOVX     @DPTR,A
{0xa13a, 0xa3}, // INC      DPTR
{0xa13b, 0x74}, // MOV      A,#ZoomPanControl(0x20)
{0xa13c, 0x20}, //
{0xa13d, 0xf0}, // MOVX     @DPTR,A
{0xa13e, 0xe4}, // CLR      A
{0xa13f, 0xf5}, // MOV      0x22,A
{0xa140, 0x22}, //
{0xa141, 0xe5}, // MOV      A 22
{0xa142, 0x22}, //
{0xa143, 0xc3}, // CLR      C
{0xa144, 0x94}, // SUBB     A,#PipeSetupBank0(0x40)
{0xa145, 0x40}, //
{0xa146, 0x40}, // JC       B00:AB81
{0xa147, 0x03}, //
{0xa148, 0x02}, // LJMP     B00:AC33
{0xa149, 0xf1}, //
{0xa14a, 0xfd}, //
{0xa14b, 0x90}, // MOV      DPTR,#0x0ABA
{0xa14c, 0x0a}, //
{0xa14d, 0xba}, //
{0xa14e, 0xe0}, // MOVX     A,@DPTR
{0xa14f, 0xfe}, // MOV      R6,A
{0xa150, 0xa3}, // INC      DPTR
{0xa151, 0xe0}, // MOVX     A,@DPTR
{0xa152, 0xff}, // MOV      R7,A
{0xa153, 0xf5}, // MOV      DPL(0x82),A
{0xa154, 0x82}, //
{0xa155, 0x8e}, // MOV      DPH(0x83),R6
{0xa156, 0x83}, //
{0xa157, 0xe0}, // MOVX     A,@DPTR
{0xa158, 0x54}, // ANL      A,#0x70
{0xa159, 0x70}, //
{0xa15a, 0xfd}, // MOV      R5,A
{0xa15b, 0xc4}, // SWAP     A
{0xa15c, 0x54}, // ANL      A,#0x0F
{0xa15d, 0x0f}, //
{0xa15e, 0xfd}, // MOV      R5,A
{0xa15f, 0x90}, // MOV      DPTR,#0x0ABC
{0xa160, 0x0a}, //
{0xa161, 0xbc}, //
{0xa162, 0xe0}, // MOVX     A,@DPTR
{0xa163, 0xfa}, // MOV      R2,A
{0xa164, 0xa3}, // INC      DPTR
{0xa165, 0xe0}, // MOVX     A,@DPTR
{0xa166, 0xf5}, // MOV      DPL(0x82),A
{0xa167, 0x82}, //
{0xa168, 0x8a}, // MOV      DPH(0x83),R2
{0xa169, 0x83}, //
{0xa16a, 0xed}, // MOV      A,R5
{0xa16b, 0xf0}, // MOVX     @DPTR,A
{0xa16c, 0x90}, // MOV      DPTR,#0x0ABD
{0xa16d, 0x0a}, //
{0xa16e, 0xbd}, //
{0xa16f, 0xe0}, // MOVX     A,@DPTR
{0xa170, 0x04}, // INC      A
{0xa171, 0xf0}, // MOVX     @DPTR,A
{0xa172, 0x70}, // JNZ      B00:ABB0
{0xa173, 0x06}, //
{0xa174, 0x90}, // MOV      DPTR,#0x0ABC
{0xa175, 0x0a}, //
{0xa176, 0xbc}, //
{0xa177, 0xe0}, // MOVX     A,@DPTR
{0xa178, 0x04}, // INC      A
{0xa179, 0xf0}, // MOVX     @DPTR,A
{0xa17a, 0x8f}, // MOV      DPL(0x82),R7
{0xa17b, 0x82}, //
{0xa17c, 0x8e}, // MOV      DPH(0x83),R6
{0xa17d, 0x83}, //
{0xa17e, 0xa3}, // INC      DPTR
{0xa17f, 0xe0}, // MOVX     A,@DPTR
{0xa180, 0xff}, // MOV      R7,A
{0xa181, 0x90}, // MOV      DPTR,#0x0ABC
{0xa182, 0x0a}, //
{0xa183, 0xbc}, //
{0xa184, 0xe0}, // MOVX     A,@DPTR
{0xa185, 0xfc}, // MOV      R4,A
{0xa186, 0xa3}, // INC      DPTR
{0xa187, 0xe0}, // MOVX     A,@DPTR
{0xa188, 0xf5}, // MOV      DPL(0x82),A
{0xa189, 0x82}, //
{0xa18a, 0x8c}, // MOV      DPH(0x83),R4
{0xa18b, 0x83}, //
{0xa18c, 0xef}, // MOV      A,R7
{0xa18d, 0xf0}, // MOVX     @DPTR,A
{0xa18e, 0x90}, // MOV      DPTR,#0x0ABD
{0xa18f, 0x0a}, //
{0xa190, 0xbd}, //
{0xa191, 0xe0}, // MOVX     A,@DPTR
{0xa192, 0x04}, // INC      A
{0xa193, 0xf0}, // MOVX     @DPTR,A
{0xa194, 0x70}, // JNZ      B00:ABD2
{0xa195, 0x06}, //
{0xa196, 0x90}, // MOV      DPTR,#0x0ABC
{0xa197, 0x0a}, //
{0xa198, 0xbc}, //
{0xa199, 0xe0}, // MOVX     A,@DPTR
{0xa19a, 0x04}, // INC      A
{0xa19b, 0xf0}, // MOVX     @DPTR,A
{0xa19c, 0x90}, // MOV      DPTR,#0x0ABA
{0xa19d, 0x0a}, //
{0xa19e, 0xba}, //
{0xa19f, 0xe0}, // MOVX     A,@DPTR
{0xa1a0, 0xfe}, // MOV      R6,A
{0xa1a1, 0xa3}, // INC      DPTR
{0xa1a2, 0xe0}, // MOVX     A,@DPTR
{0xa1a3, 0xff}, // MOV      R7,A
{0xa1a4, 0xf5}, // MOV      DPL(0x82),A
{0xa1a5, 0x82}, //
{0xa1a6, 0x8e}, // MOV      DPH(0x83),R6
{0xa1a7, 0x83}, //
{0xa1a8, 0xe0}, // MOVX     A,@DPTR
{0xa1a9, 0x54}, // ANL      A,#0x07
{0xa1aa, 0x07}, //
{0xa1ab, 0xfd}, // MOV      R5,A
{0xa1ac, 0x90}, // MOV      DPTR,#0x0ABC
{0xa1ad, 0x0a}, //
{0xa1ae, 0xbc}, //
{0xa1af, 0xe0}, // MOVX     A,@DPTR
{0xa1b0, 0xfa}, // MOV      R2,A
{0xa1b1, 0xa3}, // INC      DPTR
{0xa1b2, 0xe0}, // MOVX     A,@DPTR
{0xa1b3, 0xf5}, // MOV      DPL(0x82),A
{0xa1b4, 0x82}, //
{0xa1b5, 0x8a}, // MOV      DPH(0x83),R2
{0xa1b6, 0x83}, //
{0xa1b7, 0xed}, // MOV      A,R5
{0xa1b8, 0xf0}, // MOVX     @DPTR,A
{0xa1b9, 0x90}, // MOV      DPTR,#0x0ABD
{0xa1ba, 0x0a}, //
{0xa1bb, 0xbd}, //
{0xa1bc, 0xe0}, // MOVX     A,@DPTR
{0xa1bd, 0x04}, // INC      A
{0xa1be, 0xf0}, // MOVX     @DPTR,A
{0xa1bf, 0x70}, // JNZ      B00:ABFD
{0xa1c0, 0x06}, //
{0xa1c1, 0x90}, // MOV      DPTR,#0x0ABC
{0xa1c2, 0x0a}, //
{0xa1c3, 0xbc}, //
{0xa1c4, 0xe0}, // MOVX     A,@DPTR
{0xa1c5, 0x04}, // INC      A
{0xa1c6, 0xf0}, // MOVX     @DPTR,A
{0xa1c7, 0x8f}, // MOV      DPL(0x82),R7
{0xa1c8, 0x82}, //
{0xa1c9, 0x8e}, // MOV      DPH(0x83),R6
{0xa1ca, 0x83}, //
{0xa1cb, 0xa3}, // INC      DPTR
{0xa1cc, 0xa3}, // INC      DPTR
{0xa1cd, 0xe0}, // MOVX     A,@DPTR
{0xa1ce, 0xff}, // MOV      R7,A
{0xa1cf, 0x90}, // MOV      DPTR,#0x0ABC
{0xa1d0, 0x0a}, //
{0xa1d1, 0xbc}, //
{0xa1d2, 0xe0}, // MOVX     A,@DPTR
{0xa1d3, 0xfc}, // MOV      R4,A
{0xa1d4, 0xa3}, // INC      DPTR
{0xa1d5, 0xe0}, // MOVX     A,@DPTR
{0xa1d6, 0xf5}, // MOV      DPL(0x82),A
{0xa1d7, 0x82}, //
{0xa1d8, 0x8c}, // MOV      DPH(0x83),R4
{0xa1d9, 0x83}, //
{0xa1da, 0xef}, // MOV      A,R7
{0xa1db, 0xf0}, // MOVX     @DPTR,A
{0xa1dc, 0x90}, // MOV      DPTR,#0x0ABD
{0xa1dd, 0x0a}, //
{0xa1de, 0xbd}, //
{0xa1df, 0xe0}, // MOVX     A,@DPTR
{0xa1e0, 0x04}, // INC      A
{0xa1e1, 0xf0}, // MOVX     @DPTR,A
{0xa1e2, 0x70}, // JNZ      B00:AC20
{0xa1e3, 0x06}, //
{0xa1e4, 0x90}, // MOV      DPTR,#0x0ABC
{0xa1e5, 0x0a}, //
{0xa1e6, 0xbc}, //
{0xa1e7, 0xe0}, // MOVX     A,@DPTR
{0xa1e8, 0x04}, // INC      A
{0xa1e9, 0xf0}, // MOVX     @DPTR,A
{0xa1ea, 0x90}, // MOV      DPTR,#0x0ABB
{0xa1eb, 0x0a}, //
{0xa1ec, 0xbb}, //
{0xa1ed, 0xe0}, // MOVX     A,@DPTR
{0xa1ee, 0x24}, // ADD      A,#0x03
{0xa1ef, 0x03}, //
{0xa1f0, 0xf0}, // MOVX     @DPTR,A
{0xa1f1, 0x90}, // MOV      DPTR,#0x0ABA
{0xa1f2, 0x0a}, //
{0xa1f3, 0xba}, //
{0xa1f4, 0xe0}, // MOVX     A,@DPTR
{0xa1f5, 0x34}, // ADDC     A,#DeviceParameters(0x00)
{0xa1f6, 0x00}, //
{0xa1f7, 0xf0}, // MOVX     @DPTR,A
{0xa1f8, 0x05}, // INC      0x22
{0xa1f9, 0x22}, //
{0xa1fa, 0x02}, // LJMP     B00:A409
{0xa1fb, 0xf1}, //
{0xa1fc, 0x41}, //
{0xa1fd, 0x90}, // MOV      DPTR,#0x0ABA
{0xa1fe, 0x0a}, //
{0xa1ff, 0xba}, //
{0xa200, 0x74}, // MOV      A,#g_fpPixelCount(0x0E)
{0xa201, 0x0e}, //
{0xa202, 0xf0}, // MOVX     @DPTR,A
{0xa203, 0xa3}, // INC      DPTR
{0xa204, 0x74}, // MOV      A,#0xDC
{0xa205, 0xdc}, //
{0xa206, 0xf0}, // MOVX     @DPTR,A
{0xa207, 0xa3}, // INC      DPTR
{0xa208, 0x74}, // MOV      A,#0x05
{0xa209, 0x05}, //
{0xa20a, 0xf0}, // MOVX     @DPTR,A
{0xa20b, 0xa3}, // INC      DPTR
{0xa20c, 0x74}, // MOV      A,#0x61
{0xa20d, 0x61}, //
{0xa20e, 0xf0}, // MOVX     @DPTR,A
{0xa20f, 0x90}, // MOV      DPTR,#0x0ABA
{0xa210, 0x0a}, //
{0xa211, 0xba}, //
{0xa212, 0xe0}, // MOVX     A,@DPTR
{0xa213, 0xfe}, // MOV      R6,A
{0xa214, 0xa3}, // INC      DPTR
{0xa215, 0xe0}, // MOVX     A,@DPTR
{0xa216, 0xaa}, // MOV      R2,uwDelay100(0x06)
{0xa217, 0x06}, //
{0xa218, 0xf9}, // MOV      R1,A
{0xa219, 0x7b}, // MOV      R3,#0x01
{0xa21a, 0x01}, //
{0xa21b, 0xc0}, // PUSH     0x02
{0xa21c, 0x02}, //
{0xa21d, 0xa3}, // INC      DPTR
{0xa21e, 0xe0}, // MOVX     A,@DPTR
{0xa21f, 0xfe}, // MOV      R6,A
{0xa220, 0xa3}, // INC      DPTR
{0xa221, 0xe0}, // MOVX     A,@DPTR
{0xa222, 0xaa}, // MOV      R2,uwDelay100(0x06)
{0xa223, 0x06}, //
{0xa224, 0xf8}, // MOV      R0,A
{0xa225, 0xac}, // MOV      R4 02
{0xa226, 0x02}, //
{0xa227, 0x7d}, // MOV      R5,#0x01
{0xa228, 0x01}, //
{0xa229, 0xd0}, // POP      0x02
{0xa22a, 0x02}, //
{0xa22b, 0x7e}, // MOV      R6,#DeviceParameters(0x00)
{0xa22c, 0x00}, //
{0xa22d, 0x7f}, // MOV      R7,#uwDelay1000(0x04)
{0xa22e, 0x04}, //
{0xa22f, 0x12}, // LCALL    C?COPY517(C:0F6F)
{0xa230, 0x0f}, //
{0xa231, 0x6f}, //
{0xa232, 0x02}, // JUMP
{0xa233, 0x66}, //
{0xa234, 0xd9}, //
{0xa235, 0x90}, // MOV      R4 02
{0xa236, 0x07}, //
{0xa237, 0xd0}, // MOV      R5,#0x01
{0xa238, 0x02}, //
{0xa239, 0xa2}, // POP      0x02
{0xa23a, 0x69}, //
{0xa240, 0x02}, // ANL      A,#0xFD
{0xa241, 0x21}, //
{0xa242, 0x7f}, // MOVX     @DPTR,A
{0xa243, 0x02}, // ANL      A,#0xFD
{0xa244, 0x21}, //
{0xa245, 0xf4}, // MOVX     @DPTR,A
{0xa246, 0x02}, // ANL      A,#0xFD
{0xa247, 0xa6}, //
{0xa248, 0x15}, // MOVX     @DPTR,A
{0xa249, 0x60}, // JZ       C:29EF
{0xa24a, 0x0a}, //
{0xa24b, 0xef}, // MOV      A,R7
{0xa24c, 0xb4}, // CJNE     A,#0x01,C:29FF
{0xa24d, 0x01}, //
{0xa24e, 0x16}, //
{0xa24f, 0x90}, // MOV      DPTR,#ModeSetup(0x005D)
{0xa250, 0x00}, //
{0xa251, 0x5d}, //
{0xa252, 0xe0}, // MOVX     A,@DPTR
{0xa253, 0x70}, // JNZ      C:29FF
{0xa254, 0x10}, //
{0xa255, 0x12}, // LCALL    StreamManager_ResumeStreaming(C:26C8)
{0xa256, 0x26}, //
{0xa257, 0xc8}, //
{0xa258, 0x90}, // MOV      DPTR,#0x0011
{0xa259, 0x00}, //
{0xa25a, 0x11}, //
{0xa25b, 0x74}, // MOV      A,#0x30
{0xa25c, 0x30}, //
{0xa25d, 0xf0}, // MOVX     @DPTR,A
{0xa25e, 0x90}, // MOV      DPTR,#fpHighClipForDesiredExposure(0x0010)
{0xa25f, 0x00}, //
{0xa260, 0x10}, //
{0xa261, 0x74}, // MOV      A,#0x01
{0xa262, 0x01}, //
{0xa263, 0xf0}, // MOVX     @DPTR,A
{0xa264, 0x22}, // RET
{0xa265, 0x12}, // LCALL    C:25A8
{0xa266, 0x25}, //
{0xa267, 0xa8}, //
{0xa268, 0x02}, // RET
{0xa269, 0x29}, //
{0xa26a, 0xfc}, //
{0xa26b, 0x44}, // ORL      A,#fpHighClipForDesiredExposure(0x10)
{0xa26c, 0x18}, //
{0xa26d, 0xf0}, // MOVX     @DPTR,A
{0xa26e, 0x90}, // MOV      DPTR,#Tx_Csi2_Dphy_Datalane2_Pwr_Ctrl(0x7218)
{0xa26f, 0x72}, //
{0xa270, 0x18}, //
{0xa271, 0xe0}, // MOVX     A,@DPTR
{0xa272, 0x44}, // ORL      A,#fpHighClipForDesiredExposure(0x10)
{0xa273, 0x18}, //
{0xa274, 0xf0}, // MOVX     @DPTR,A
{0xa275, 0x00}, // MOV      DPTR,#Tx_Csi2_Dphy_Clklane_Pwr_Ctrl(0x7208)
{0xa276, 0x00}, //
{0xa277, 0x00}, //
{0xa278, 0x00}, // MOVX     A,@DPTR
{0xa279, 0x00}, // ORL      A,#fpHighClipForDesiredExposure(0x10)
{0xa27a, 0x00}, //
{0xa27b, 0x90}, // MOVX     @DPTR,A
{0xa27c, 0x72}, // MOV      DPTR,#Tx_Csi2_Pwr_Ctrl(0x7214)
{0xa27d, 0x08}, //
{0xa27e, 0xe0}, //
{0xa27f, 0x44}, // MOVX     A,@DPTR
{0xa280, 0x10}, // ANL      A,#0xFD
{0xa281, 0xf0}, //
{0xa282, 0x90}, // MOVX     @DPTR,A
{0xa283, 0x72}, // MOV      DPTR,#Tx_Csi2_Dphy_Pwr_Ctrl(0x7204)
{0xa284, 0x14}, //
{0xa285, 0xe0}, //
{0xa286, 0x54}, // MOV      A,#0x1F
{0xa287, 0xfd}, //
{0xa288, 0xf0}, // MOVX     @DPTR,A
{0xa289, 0x22}, // RET
{0xa29b, 0xf0}, // MOVX     @DPTR,A
{0xa29c, 0xd3}, // SETB     C
{0xa29d, 0x90}, // MOV      DPTR,#0x0791
{0xa29e, 0x07}, //
{0xa29f, 0x91}, //
{0xa2a0, 0xe0}, // MOVX     A,@DPTR
{0xa2a1, 0x94}, // SUBB     A,#0x21
{0xa2a2, 0x21}, //
{0xa2a3, 0x90}, // MOV      DPTR,#AutoFocusInput(0x0790)
{0xa2a4, 0x07}, //
{0xa2a5, 0x90}, //
{0xa2a6, 0xe0}, // MOVX     A,@DPTR
{0xa2a7, 0x64}, // XRL      A,#PipeSetupCommon(0x80)
{0xa2a8, 0x80}, //
{0xa2a9, 0x94}, // SUBB     A,#SP(0x81)
{0xa2aa, 0x81}, //
{0xa2ab, 0x40}, // JC       B01:B152
{0xa2ac, 0x08}, //
{0xa2ad, 0x90}, // MOV      DPTR,#0x07CB
{0xa2ae, 0x07}, //
{0xa2af, 0xcb}, //
{0xa2b0, 0x74}, // MOV      A,#0xFF
{0xa2b1, 0xff}, //
{0xa2b2, 0xf0}, // MOVX     @DPTR,A
{0xa2b3, 0x80}, // SJMP     B01:B158
{0xa2b4, 0x06}, //
{0xa2b5, 0x90}, // MOV      DPTR,#0x07CB
{0xa2b6, 0x07}, //
{0xa2b7, 0xcb}, //
{0xa2b8, 0x74}, // MOV      A,#0x01
{0xa2b9, 0x01}, //
{0xa2ba, 0xf0}, // MOVX     @DPTR,A
{0xa2bb, 0x02}, // JUMP
{0xa2bc, 0xb5}, //
{0xa2bd, 0xc3}, //
{0xa2be, 0x90}, // MOV      DPTR,#0x0834
{0xa2bf, 0x08}, //
{0xa2c0, 0x34}, //
{0xa2c1, 0xe0}, // MOVX     A,@DPTR
{0xa2c2, 0xfc}, // MOV      R4,A
{0xa2c3, 0xa3}, // INC      DPTR
{0xa2c4, 0xe0}, // MOVX     A,@DPTR
{0xa2c5, 0xfd}, // MOV      R5,A
{0xa2c6, 0xa3}, // INC      DPTR
{0xa2c7, 0xe0}, // MOVX     A,@DPTR
{0xa2c8, 0xfe}, // MOV      R6,A
{0xa2c9, 0xa3}, // INC      DPTR
{0xa2ca, 0xe0}, // MOVX     A,@DPTR
{0xa2cb, 0xff}, // MOV      R7,A
{0xa2cc, 0x90}, // MOV      DPTR,#AutoFocusMeasureData(0x07D0)
{0xa2cd, 0x07}, //
{0xa2ce, 0xd0}, //
{0xa2cf, 0xe0}, // MOVX     A,@DPTR
{0xa2d0, 0xf8}, // MOV      R0,A
{0xa2d1, 0xa3}, // INC      DPTR
{0xa2d2, 0xe0}, // MOVX     A,@DPTR
{0xa2d3, 0xf9}, // MOV      R1,A
{0xa2d4, 0xa3}, // INC      DPTR
{0xa2d5, 0xe0}, // MOVX     A,@DPTR
{0xa2d6, 0xfa}, // MOV      R2,A
{0xa2d7, 0xa3}, // INC      DPTR
{0xa2d8, 0xe0}, // MOVX     A,@DPTR
{0xa2d9, 0xfb}, // MOV      R3,A
{0xa2da, 0xd3}, // SETB     C
{0xa2db, 0x12}, // LCALL    C?ULCMP(C:0DAE)
{0xa2dc, 0x0d}, //
{0xa2dd, 0xae}, //
{0xa2de, 0x40}, // JC       B01:9FDA
{0xa2df, 0x0b}, //
{0xa2e0, 0x12}, // LCALL    HCS_Initialization(B01:B0EF)
{0xa2e1, 0xb5}, //
{0xa2e2, 0x49}, //
{0xa2e3, 0x90}, // MOV      DPTR,#0x07A4
{0xa2e4, 0x07}, //
{0xa2e5, 0xa4}, //
{0xa2e6, 0x74}, // MOV      A,#0x02
{0xa2e7, 0x02}, //
{0xa2e8, 0xf0}, // MOVX     @DPTR,A
{0xa2e9, 0x80}, // SJMP     B01:9FE3
{0xa2ea, 0x09}, //
{0xa2eb, 0x12}, // LCALL    LowFocusMeasureFullSearchInit(B01:B7AE)
{0xa2ec, 0xb7}, //
{0xa2ed, 0x51}, //
{0xa2ee, 0x90}, // MOV      DPTR,#0x07A4
{0xa2ef, 0x07}, //
{0xa2f0, 0xa4}, //
{0xa2f1, 0x74}, // MOV      A,#0x05
{0xa2f2, 0x05}, //
{0xa2f3, 0xf0}, // MOVX     @DPTR,A
{0xa2f4, 0x02}, // JUM
{0xa2f5, 0xa2}, //
{0xa2f6, 0xda}, //
{0xa2f7, 0x90}, // MOV      DPTR,#fOTPRed(0x0EE0)
{0xa2f8, 0x0e}, //
{0xa2f9, 0xe0}, //
{0xa2fa, 0xe0}, // MOVX     A,@DPTR
{0xa2fb, 0xfd}, // MOV      R5,A
{0xa2fc, 0xa3}, // INC      DPTR
{0xa2fd, 0xe0}, // MOVX     A,@DPTR
{0xa2fe, 0x90}, // MOV      DPTR,#0x02A2
{0xa2ff, 0x02}, //
{0xa300, 0xa2}, //
{0xa301, 0xcd}, // XCH      A,R5
{0xa302, 0xf0}, // MOVX     @DPTR,A
{0xa303, 0xa3}, // INC      DPTR
{0xa304, 0xed}, // MOV      A,R5
{0xa305, 0xf0}, // MOVX     @DPTR,A
{0xa306, 0x90}, // MOV      DPTR,#fOTPBlue(0x0EE2)
{0xa307, 0x0e}, //
{0xa308, 0xe2}, //
{0xa309, 0xe0}, // MOVX     A,@DPTR
{0xa30a, 0xfd}, // MOV      R5,A
{0xa30b, 0xa3}, // INC      DPTR
{0xa30c, 0xe0}, // MOVX     A,@DPTR
{0xa30d, 0x90}, // MOV      DPTR,#0x02A8
{0xa30e, 0x02}, //
{0xa30f, 0xa8}, //
{0xa310, 0xcd}, // XCH      A,R5
{0xa311, 0xf0}, // MOVX     @DPTR,A
{0xa312, 0xa3}, // INC      DPTR
{0xa313, 0xed}, // MOV      A,R5
{0xa314, 0xf0}, // MOVX     @DPTR,A
{0xa315, 0xe4}, // CLR      A
{0xa316, 0x90}, // MOV      DPTR,#PresetControl(0x0638)
{0xa317, 0x06}, //
{0xa318, 0x38}, //
{0xa319, 0xf0}, // MOVX     @DPTR,A
{0xa31a, 0x02}, // JUMP     #676
{0xa31b, 0x67}, //
{0xa31c, 0x63}, //
{0xa31d, 0x90}, // MOV      DPTR,#bDarkCalSR(0x0EE8)
{0xa31e, 0x0e}, //
{0xa31f, 0xe8}, //
{0xa320, 0xe0}, // MOVX     A,@DPTR
{0xa321, 0x90}, // MOV      DPTR,#0x0262
{0xa322, 0x02}, //
{0xa323, 0x62}, //
{0xa324, 0xf0}, // MOVX     @DPTR,A
{0xa325, 0x90}, // MOV      DPTR,#bDarkCalAB4(0x0EE9)
{0xa326, 0x0e}, //
{0xa327, 0xe9}, //
{0xa328, 0xe0}, // MOVX     A,@DPTR
{0xa329, 0x90}, // MOV      DPTR,#0x0263
{0xa32a, 0x02}, //
{0xa32b, 0x63}, //
{0xa32c, 0xf0}, // MOVX     @DPTR,A
{0xa32d, 0x02}, // JUMP     #676
{0xa32e, 0x67}, //
{0xa32f, 0x1f}, //
{0xa33b, 0x90}, // MOV      DPTR,#INFINITY_OTP(0xE014)
{0xa33c, 0x0e}, //
{0xa33d, 0x14}, //
{0xa33e, 0xe0}, // MOVX     A,@DPTR
{0xa33f, 0xfe}, // MOV      R6,A
{0xa340, 0xa3}, // INC      DPTR
{0xa341, 0xe0}, // MOVX     A,@DPTR
{0xa342, 0xff}, // MOV      R7,A
{0xa343, 0x90}, // MOV      DPTR,#0x06D9
{0xa344, 0x06}, //
{0xa345, 0xd9}, //
{0xa346, 0xee}, // MOV      A,R6
{0xa347, 0xf0}, // MOVX     @DPTR,A
{0xa348, 0xa3}, // INC      DPTR
{0xa349, 0xef}, // MOV      A,R7
{0xa34a, 0xf0}, // MOVX     @DPTR,A
{0xa34b, 0x90}, // MOV      DPTR,#DELTA_UP_OTP(0xE018)
{0xa34c, 0x0e}, //
{0xa34d, 0x18}, //
{0xa34e, 0xe0}, // MOVX     A,@DPTR
{0xa34f, 0xfd}, // MOV      R5,A
{0xa350, 0x7c}, // MOV      R4,#DeviceParameters(0x00)
{0xa351, 0x00}, //
{0xa352, 0xc3}, // CLR      C
{0xa353, 0xef}, // MOV      A,R7
{0xa354, 0x9d}, // SUBB     A,R5
{0xa355, 0xee}, // MOV      A,R6
{0xa356, 0x9c}, // SUBB     A,R4
{0xa357, 0x50}, // JNC      C:2067
{0xa358, 0x09}, //
{0xa359, 0xe4}, // CLR      A
{0xa35a, 0x90}, // MOV      DPTR,#0x06D7
{0xa35b, 0x06}, //
{0xa35c, 0xd7}, //
{0xa35d, 0xf0}, // MOVX     @DPTR,A
{0xa35e, 0xa3}, // INC      DPTR
{0xa35f, 0xf0}, // MOVX     @DPTR,A
{0xa360, 0x80}, // SJMP     C:207A
{0xa361, 0x13}, //
{0xa362, 0xc3}, // CLR      C
{0xa363, 0x90}, // MOV      DPTR,#0x06DA
{0xa364, 0x06}, //
{0xa365, 0xda}, //
{0xa366, 0xe0}, // MOVX     A,@DPTR
{0xa367, 0x9d}, // SUBB     A,R5
{0xa368, 0xfe}, // MOV      R6,A
{0xa369, 0x90}, // MOV      DPTR,#0x06D9
{0xa36a, 0x06}, //
{0xa36b, 0xd9}, //
{0xa36c, 0xe0}, // MOVX     A,@DPTR
{0xa36d, 0x9c}, // SUBB     A,R4
{0xa36e, 0x90}, // MOV      DPTR,#0x06D7
{0xa36f, 0x06}, //
{0xa370, 0xd7}, //
{0xa371, 0xf0}, // MOVX     @DPTR,A
{0xa372, 0xa3}, // INC      DPTR
{0xa373, 0xce}, // XCH      A,R6
{0xa374, 0xf0}, // MOVX     @DPTR,A
{0xa375, 0x90}, // MOV      DPTR,#DELTA_UP_OTP(0xE018)
{0xa376, 0x0e}, //
{0xa377, 0x18}, //
{0xa378, 0xe0}, // MOVX     A,@DPTR
{0xa379, 0xf9}, // MOV      R1,A
{0xa37a, 0xff}, // MOV      R7,A
{0xa37b, 0x90}, // MOV      DPTR,#0x06C2
{0xa37c, 0x06}, //
{0xa37d, 0xc2}, //
{0xa37e, 0xe0}, // MOVX     A,@DPTR
{0xa37f, 0xfc}, // MOV      R4,A
{0xa380, 0xa3}, // INC      DPTR
{0xa381, 0xe0}, // MOVX     A,@DPTR
{0xa382, 0xfd}, // MOV      R5,A
{0xa383, 0xc3}, // CLR      C
{0xa384, 0x9f}, // SUBB     A,R7
{0xa385, 0xff}, // MOV      R7,A
{0xa386, 0xec}, // MOV      A,R4
{0xa387, 0x94}, // SUBB     A,#DeviceParameters(0x00)
{0xa388, 0x00}, //
{0xa389, 0xfe}, // MOV      R6,A
{0xa38a, 0x90}, // MOV      DPTR,#MACRO_OTP(0xE016)
{0xa38b, 0x0e}, //
{0xa38c, 0x16}, //
{0xa38d, 0xe0}, // MOVX     A,@DPTR
{0xa38e, 0xfa}, // MOV      R2,A
{0xa38f, 0xa3}, // INC      DPTR
{0xa390, 0xe0}, // MOVX     A,@DPTR
{0xa391, 0xfb}, // MOV      R3,A
{0xa392, 0xd3}, // SETB     C
{0xa393, 0x9f}, // SUBB     A,R7
{0xa394, 0xea}, // MOV      A,R2
{0xa395, 0x9e}, // SUBB     A,R6
{0xa396, 0x40}, // JC       C:20A7
{0xa397, 0x0a}, //
{0xa398, 0x90}, // MOV      DPTR,#0x06D5
{0xa399, 0x06}, //
{0xa39a, 0xd5}, //
{0xa39b, 0xec}, // MOV      A,R4
{0xa39c, 0xf0}, // MOVX     @DPTR,A
{0xa39d, 0xa3}, // INC      DPTR
{0xa39e, 0xed}, // MOV      A,R5
{0xa39f, 0xf0}, // MOVX     @DPTR,A
{0xa3a0, 0x80}, // SJMP     C:20B5
{0xa3a1, 0x0e}, //
{0xa3a2, 0xe9}, // MOV      A,R1
{0xa3a3, 0x7e}, // MOV      R6,#DeviceParameters(0x00)
{0xa3a4, 0x00}, //
{0xa3a5, 0x2b}, // ADD      A,R3
{0xa3a6, 0xff}, // MOV      R7,A
{0xa3a7, 0xee}, // MOV      A,R6
{0xa3a8, 0x3a}, // ADDC     A,R2
{0xa3a9, 0x90}, // MOV      DPTR,#0x06D5
{0xa3aa, 0x06}, //
{0xa3ab, 0xd5}, //
{0xa3ac, 0xf0}, // MOVX     @DPTR,A
{0xa3ad, 0xa3}, // INC      DPTR
{0xa3ae, 0xef}, // MOV      A,R7
{0xa3af, 0xf0}, // MOVX     @DPTR,A
{0xa3b0, 0xe9}, // MOV      A,R1
{0xa3b1, 0xfb}, // MOV      R3,A
{0xa3b2, 0x7a}, // MOV      R2,#DeviceParameters(0x00)
{0xa3b3, 0x00}, //
{0xa3b4, 0x90}, // MOV      DPTR,#0xE015
{0xa3b5, 0x0e}, //
{0xa3b6, 0x15}, //
{0xa3b7, 0xe0}, // MOVX     A,@DPTR
{0xa3b8, 0x2b}, // ADD      A,R3
{0xa3b9, 0xfe}, // MOV      R6,A
{0xa3ba, 0x90}, // MOV      DPTR,#INFINITY_OTP(0xE014)
{0xa3bb, 0x0e}, //
{0xa3bc, 0x14}, //
{0xa3bd, 0xe0}, // MOVX     A,@DPTR
{0xa3be, 0x3a}, // ADDC     A,R2
{0xa3bf, 0x90}, // MOV      DPTR,#0x06E1
{0xa3c0, 0x06}, //
{0xa3c1, 0xe1}, //
{0xa3c2, 0xf0}, // MOVX     @DPTR,A
{0xa3c3, 0xa3}, // INC      DPTR
{0xa3c4, 0xce}, // XCH      A,R6
{0xa3c5, 0xf0}, // MOVX     @DPTR,A
{0xa3c6, 0xc3}, // CLR      C
{0xa3c7, 0x90}, // MOV      DPTR,#0xE017
{0xa3c8, 0x0e}, //
{0xa3c9, 0x17}, //
{0xa3ca, 0xe0}, // MOVX     A,@DPTR
{0xa3cb, 0x9b}, // SUBB     A,R3
{0xa3cc, 0xfe}, // MOV      R6,A
{0xa3cd, 0x90}, // MOV      DPTR,#MACRO_OTP(0xE016)
{0xa3ce, 0x0e}, //
{0xa3cf, 0x16}, //
{0xa3d0, 0x02}, // JUMP
{0xa3d1, 0x20}, //
{0xa3d2, 0xd5}, //
{0xa3d3, 0x90}, // MOV      DPTR,#bDarkCalHFPN(0x0EE4)
{0xa3d4, 0x0e}, //
{0xa3d5, 0xe4}, //
{0xa3d6, 0xe0}, // MOVX     A,@DPTR
{0xa3d7, 0x90}, // MOV      DPTR,#0x0266
{0xa3d8, 0x02}, //
{0xa3d9, 0x66}, //
{0xa3da, 0xf0}, // MOVX     @DPTR,A
{0xa3db, 0x90}, // MOV      DPTR,#bDarkCalHFPN(0x0EE4)
{0xa3dc, 0x0e}, //
{0xa3dd, 0xe5}, //
{0xa3de, 0xe0}, // MOVX     A,@DPTR
{0xa3df, 0x90}, // MOV      DPTR,#0x0266
{0xa3e0, 0x02}, //
{0xa3e1, 0x64}, //
{0xa3e2, 0xf0}, // MOVX     @DPTR,A
{0xa3e3, 0x90}, // MOV      DPTR,#bDarkCalHFPN(0x0EE4)
{0xa3e4, 0x0e}, //
{0xa3e5, 0xe6}, //
{0xa3e6, 0xe0}, // MOVX     A,@DPTR
{0xa3e7, 0x90}, // MOV      DPTR,#0x0266
{0xa3e8, 0x02}, //
{0xa3e9, 0x65}, //
{0xa3ea, 0xf0}, // MOVX     @DPTR,A
{0xa3eb, 0x02}, // JUMP
{0xa3ec, 0x67}, //
{0xa3ed, 0xa5}, //
{0xa3f0, 0x12}, //
{0xa3f1, 0x47}, //
{0xa3f2, 0x59}, // MOVX     @DPTR,A
{0xa3f3, 0x90}, // MOV      DPTR,#bDarkCalHFPN(0x0EE4)
{0xa3f4, 0x00}, //
{0xa3f5, 0xb5}, //
{0xa3f6, 0xe0}, // MOVX     A,@DPTR
{0xa3f7, 0xb4}, // MOVX     A,@DPTR
{0xa3f8, 0x02}, // MOV      DPTR,#0x0266
{0xa3f9, 0x03}, //
{0xa3fa, 0x12}, //
{0xa3fb, 0x47}, // MOVX     @DPTR,A
{0xa3fc, 0x59}, // JUMP
{0xa3fd, 0x02}, //
{0xa3fe, 0xc5}, //
{0xa3ff, 0xc3}, //
{0xa400, 0x90}, // MOV      DPTR,#c_HFlip(0x003D)
{0xa401, 0x00}, //
{0xa402, 0x3d}, //
{0xa403, 0xf0}, // MOVX     @DPTR,A
{0xa404, 0x90}, // MOV      DPTR,#0x0084
{0xa405, 0x00}, //
{0xa406, 0x84}, //
{0xa407, 0xe0}, // MOVX     A,@DPTR
{0xa408, 0xfe}, // MOV      R6,A
{0xa409, 0x90}, // MOV      DPTR,#c_VFlip(0x003E)
{0xa40a, 0x00}, //
{0xa40b, 0x3e}, //
{0xa40c, 0xf0}, // MOVX     @DPTR,A
{0xa40d, 0xef}, // MOV      A,R7
{0xa40e, 0x70}, // JNZ      B00:8201
{0xa40f, 0x03}, //
{0xa410, 0xee}, // MOV      A,R6
{0xa411, 0x60}, // JZ       B00:8205
{0xa412, 0x04}, //
{0xa413, 0x7f}, // MOV      R7,#0x01
{0xa414, 0x01}, //
{0xa415, 0x80}, // SJMP     B00:8207
{0xa416, 0x02}, //
{0xa417, 0x7f}, // MOV      R7,#DeviceParameters(0x00)
{0xa418, 0x00}, //
{0xa419, 0x90}, // MOV      DPTR,#c_HVFlip(0x003F)
{0xa41a, 0x00}, //
{0xa41b, 0x3f}, //
{0xa41c, 0xef}, // MOV      A,R7
{0xa41d, 0xf0}, // MOVX     @DPTR,A
{0xa41e, 0x02}, // JUMP bac
{0xa41f, 0x89}, //
{0xa420, 0xd3}, //
{0xa421, 0x90}, // MOV      DPTR,#uwI2CSIndex(0x0012)
{0xa422, 0x00}, //
{0xa423, 0x12}, //
{0xa424, 0xe0}, // MOVX     A,@DPTR
{0xa425, 0xff}, // MOV      R7,A
{0xa426, 0x70}, // JNZ      B00:9AC3
{0xa427, 0x0c}, //
{0xa428, 0x90}, // MOV      DPTR,#0x0046
{0xa429, 0x00}, //
{0xa42a, 0x46}, //
{0xa42b, 0xe0}, // MOVX     A,@DPTR
{0xa42c, 0xc3}, // CLR      C
{0xa42d, 0x94}, // SUBB     A,#0x07
{0xa42e, 0x07}, //
{0xa42f, 0x40}, // JC       B00:9AC3
{0xa430, 0x03}, //
{0xa431, 0x75}, // MOV      0x2E,#0x02
{0xa432, 0x2e}, //
{0xa433, 0x02}, //
{0xa434, 0xef}, // MOV      A,R7
{0xa435, 0xb4}, // CJNE     A,#0x01,B00:9AD3
{0xa436, 0x01}, //
{0xa437, 0x0c}, //
{0xa438, 0x90}, // MOV      DPTR,#XDroop_Reverse_Croping(0x0066)
{0xa439, 0x00}, //
{0xa43a, 0x66}, //
{0xa43b, 0xe0}, // MOVX     A,@DPTR
{0xa43c, 0xc3}, // CLR      C
{0xa43d, 0x94}, // SUBB     A,#0x07
{0xa43e, 0x07}, //
{0xa43f, 0x40}, // JC       B00:9AD3
{0xa440, 0x03}, //
{0xa441, 0x75}, // MOV      0x2E,#0x02
{0xa442, 0x2e}, //
{0xa443, 0x02}, //
{0xa444, 0x02}, // JUMP
{0xa445, 0xa7}, //
{0xa446, 0x9e}, //
{0xa447, 0xc3}, // CLR      C
{0xa448, 0x90}, // MOV      DPTR,#0x0B8F
{0xa449, 0x0b}, //
{0xa44a, 0x8f}, //
{0xa44b, 0xe0}, // MOVX     A,@DPTR
{0xa44c, 0x94}, // SUBB     A,#PipeSetupCommon(0x80)
{0xa44d, 0x80}, //
{0xa44e, 0x90}, // MOV      DPTR,#CalculateNormalisedStatistics?BYTE(0x0B8E)
{0xa44f, 0x0b}, //
{0xa450, 0x8e}, //
{0xa451, 0xe0}, // MOVX     A,@DPTR
{0xa452, 0x94}, // SUBB     A,#0x44
{0xa453, 0x44}, //
{0xa454, 0x40}, // JC       B00:827D
{0xa455, 0x22}, //
{0xa456, 0x90}, // MOV      DPTR,#0x0B91
{0xa457, 0x0b}, //
{0xa458, 0x91}, //
{0xa459, 0xe0}, // MOVX     A,@DPTR
{0xa45a, 0x94}, // SUBB     A,#PipeSetupCommon(0x80)
{0xa45b, 0x80}, //
{0xa45c, 0x90}, // MOV      DPTR,#0x0B90
{0xa45d, 0x0b}, //
{0xa45e, 0x90}, //
{0xa45f, 0xe0}, // MOVX     A,@DPTR
{0xa460, 0x94}, // SUBB     A,#0x44
{0xa461, 0x44}, //
{0xa462, 0x40}, // JC       B00:827D
{0xa463, 0x14}, //
{0xa464, 0x90}, // MOV      DPTR,#0x0B93
{0xa465, 0x0b}, //
{0xa466, 0x93}, //
{0xa467, 0xe0}, // MOVX     A,@DPTR
{0xa468, 0x94}, // SUBB     A,#PipeSetupCommon(0x80)
{0xa469, 0x80}, //
{0xa46a, 0x90}, // MOV      DPTR,#0x0B92
{0xa46b, 0x0b}, //
{0xa46c, 0x92}, //
{0xa46d, 0xe0}, // MOVX     A,@DPTR
{0xa46e, 0x94}, // SUBB     A,#0x44
{0xa46f, 0x44}, //
{0xa470, 0x40}, // JC       B00:827D
{0xa471, 0x06}, //
{0xa472, 0x90}, // MOV      DPTR,#0x01A4
{0xa473, 0x01}, //
{0xa474, 0xa4}, //
{0xa475, 0x02}, // LJMP     back
{0xa476, 0x86}, //
{0xa477, 0x57}, //
{0xa478, 0x02}, // LJMP     back
{0xa479, 0x86}, //
{0xa47a, 0x5c}, //
{0xa500, 0xf5}, // MOV      c_HeightScale(0x3B),A
{0xa501, 0x3b}, //
{0xa502, 0x90}, // MOV      DPTR,#0x066C
{0xa503, 0x06}, //
{0xa504, 0x6c}, //
{0xa505, 0xe0}, // MOVX     A,@DPTR
{0xa506, 0xff}, // MOV      R7,A
{0xa507, 0xe5}, // MOV      A,c_HeightScale(0x3B)
{0xa508, 0x3b}, //
{0xa509, 0xc3}, // CLR      C
{0xa50a, 0x9f}, // SUBB     A,R7
{0xa50b, 0x40}, // JC       B01:98E8
{0xa50c, 0x03}, //
{0xa50d, 0x02}, // LJMP     B01:99E6
{0xa50e, 0xf6}, //
{0xa50f, 0x0e}, //
{0xa510, 0x90}, // MOV      DPTR,#0x0BC6
{0xa511, 0x0b}, //
{0xa512, 0xc6}, //
{0xa513, 0xe0}, // MOVX     A,@DPTR
{0xa514, 0x14}, // DEC      A
{0xa515, 0x60}, // JZ       B01:992B
{0xa516, 0x3c}, //
{0xa517, 0x14}, // DEC      A
{0xa518, 0x60}, // JZ       B01:995D
{0xa519, 0x6b}, //
{0xa51a, 0x24}, // ADD      A,#0x02
{0xa51b, 0x02}, //
{0xa51c, 0x60}, // JZ       B01:98F9
{0xa51d, 0x03}, //
{0xa51e, 0x02}, // LJMP     B01:998D
{0xa51f, 0xf5}, //
{0xa520, 0xb5}, //
{0xa521, 0x90}, // MOV      DPTR,#AutoFocusInstableFocusMeasureValues(0x0A9A)
{0xa522, 0x0a}, //
{0xa523, 0x9a}, //
{0xa524, 0xe0}, // MOVX     A,@DPTR
{0xa525, 0xfb}, // MOV      R3,A
{0xa526, 0xa3}, // INC      DPTR
{0xa527, 0xe0}, // MOVX     A,@DPTR
{0xa528, 0xfa}, // MOV      R2,A
{0xa529, 0xa3}, // INC      DPTR
{0xa52a, 0xe0}, // MOVX     A,@DPTR
{0xa52b, 0xf9}, // MOV      R1,A
{0xa52c, 0x85}, // MOV      DPL(0x82),c_HeightScale(0x3B)
{0xa52d, 0x3b}, //
{0xa52e, 0x82}, //
{0xa52f, 0x75}, // MOV      DPH(0x83),#DeviceParameters(0x00)
{0xa530, 0x83}, //
{0xa531, 0x00}, //
{0xa532, 0x12}, // LCALL    C?CLDOPTR(C:0AB8)
{0xa533, 0x0a}, //
{0xa534, 0xb8}, //
{0xa535, 0xff}, // MOV      R7,A
{0xa536, 0x74}, // MOV      A,#0xAB
{0xa537, 0xab}, //
{0xa538, 0x25}, // ADD      A,c_HeightScale(0x3B)
{0xa539, 0x3b}, //
{0xa53a, 0xf5}, // MOV      DPL(0x82),A
{0xa53b, 0x82}, //
{0xa53c, 0xe4}, // CLR      A
{0xa53d, 0x34}, // ADDC     A,#bInt_Event_Status(0x0A)
{0xa53e, 0x0a}, //
{0xa53f, 0xf5}, // MOV      DPH(0x83),A
{0xa540, 0x83}, //
{0xa541, 0xe0}, // MOVX     A,@DPTR
{0xa542, 0xfd}, // MOV      R5,A
{0xa543, 0xc3}, // CLR      C
{0xa544, 0xef}, // MOV      A,R7
{0xa545, 0x9d}, // SUBB     A,R5
{0xa546, 0xfe}, // MOV      R6,A
{0xa547, 0xe4}, // CLR      A
{0xa548, 0x94}, // SUBB     A,#DeviceParameters(0x00)
{0xa549, 0x00}, //
{0xa54a, 0x90}, // MOV      DPTR,#0x0BCA
{0xa54b, 0x0b}, //
{0xa54c, 0xca}, //
{0xa54d, 0xf0}, // MOVX     @DPTR,A
{0xa54e, 0xa3}, // INC      DPTR
{0xa54f, 0xce}, // XCH      A,R6
{0xa550, 0xf0}, // MOVX     @DPTR,A
{0xa551, 0x80}, // SJMP     B01:998D
{0xa552, 0x62}, //
{0xa553, 0x90}, // MOV      DPTR,#AutoFocusInstableFocusMeasureValues(0x0A9A)
{0xa554, 0x0a}, //
{0xa555, 0x9a}, //
{0xa556, 0xe0}, // MOVX     A,@DPTR
{0xa557, 0xfb}, // MOV      R3,A
{0xa558, 0xa3}, // INC      DPTR
{0xa559, 0xe0}, // MOVX     A,@DPTR
{0xa55a, 0xfa}, // MOV      R2,A
{0xa55b, 0xa3}, // INC      DPTR
{0xa55c, 0xe0}, // MOVX     A,@DPTR
{0xa55d, 0xf9}, // MOV      R1,A
{0xa55e, 0x85}, // MOV      DPL(0x82),c_HeightScale(0x3B)
{0xa55f, 0x3b}, //
{0xa560, 0x82}, //
{0xa561, 0x75}, // MOV      DPH(0x83),#DeviceParameters(0x00)
{0xa562, 0x83}, //
{0xa563, 0x00}, //
{0xa564, 0x12}, // LCALL    C?CLDOPTR(C:0AB8)
{0xa565, 0x0a}, //
{0xa566, 0xb8}, //
{0xa567, 0xff}, // MOV      R7,A
{0xa568, 0x74}, // MOV      A,#0x9D
{0xa569, 0x9d}, //
{0xa56a, 0x25}, // ADD      A,c_HeightScale(0x3B)
{0xa56b, 0x3b}, //
{0xa56c, 0xf5}, // MOV      DPL(0x82),A
{0xa56d, 0x82}, //
{0xa56e, 0xe4}, // CLR      A
{0xa56f, 0x34}, // ADDC     A,#bInt_Event_Status(0x0A)
{0xa570, 0x0a}, //
{0xa571, 0xf5}, // MOV      DPH(0x83),A
{0xa572, 0x83}, //
{0xa573, 0xe0}, // MOVX     A,@DPTR
{0xa574, 0xfd}, // MOV      R5,A
{0xa575, 0xc3}, // CLR      C
{0xa576, 0xef}, // MOV      A,R7
{0xa577, 0x9d}, // SUBB     A,R5
{0xa578, 0xfe}, // MOV      R6,A
{0xa579, 0xe4}, // CLR      A
{0xa57a, 0x94}, // SUBB     A,#DeviceParameters(0x00)
{0xa57b, 0x00}, //
{0xa57c, 0x90}, // MOV      DPTR,#0x0BCA
{0xa57d, 0x0b}, //
{0xa57e, 0xca}, //
{0xa57f, 0xf0}, // MOVX     @DPTR,A
{0xa580, 0xa3}, // INC      DPTR
{0xa581, 0xce}, // XCH      A,R6
{0xa582, 0xf0}, // MOVX     @DPTR,A
{0xa583, 0x80}, // SJMP     B01:998D
{0xa584, 0x30}, //
{0xa585, 0x90}, // MOV      DPTR,#AutoFocusInstableFocusMeasureValues(0x0A9A)
{0xa586, 0x0a}, //
{0xa587, 0x9a}, //
{0xa588, 0xe0}, // MOVX     A,@DPTR
{0xa589, 0xfb}, // MOV      R3,A
{0xa58a, 0xa3}, // INC      DPTR
{0xa58b, 0xe0}, // MOVX     A,@DPTR
{0xa58c, 0xfa}, // MOV      R2,A
{0xa58d, 0xa3}, // INC      DPTR
{0xa58e, 0xe0}, // MOVX     A,@DPTR
{0xa58f, 0xf9}, // MOV      R1,A
{0xa590, 0x85}, // MOV      DPL(0x82),c_HeightScale(0x3B)
{0xa591, 0x3b}, //
{0xa592, 0x82}, //
{0xa593, 0x75}, // MOV      DPH(0x83),#DeviceParameters(0x00)
{0xa594, 0x83}, //
{0xa595, 0x00}, //
{0xa596, 0x12}, // LCALL    C?CLDOPTR(C:0AB8)
{0xa597, 0x0a}, //
{0xa598, 0xb8}, //
{0xa599, 0xff}, // MOV      R7,A
{0xa59a, 0x74}, // MOV      A,#0xA4
{0xa59b, 0xa4}, //
{0xa59c, 0x25}, // ADD      A,c_HeightScale(0x3B)
{0xa59d, 0x3b}, //
{0xa59e, 0xf5}, // MOV      DPL(0x82),A
{0xa59f, 0x82}, //
{0xa5a0, 0xe4}, // CLR      A
{0xa5a1, 0x34}, // ADDC     A,#bInt_Event_Status(0x0A)
{0xa5a2, 0x0a}, //
{0xa5a3, 0xf5}, // MOV      DPH(0x83),A
{0xa5a4, 0x83}, //
{0xa5a5, 0xe0}, // MOVX     A,@DPTR
{0xa5a6, 0xfd}, // MOV      R5,A
{0xa5a7, 0xc3}, // CLR      C
{0xa5a8, 0xef}, // MOV      A,R7
{0xa5a9, 0x9d}, // SUBB     A,R5
{0xa5aa, 0xfe}, // MOV      R6,A
{0xa5ab, 0xe4}, // CLR      A
{0xa5ac, 0x94}, // SUBB     A,#DeviceParameters(0x00)
{0xa5ad, 0x00}, //
{0xa5ae, 0x90}, // MOV      DPTR,#0x0BCA
{0xa5af, 0x0b}, //
{0xa5b0, 0xca}, //
{0xa5b1, 0xf0}, // MOVX     @DPTR,A
{0xa5b2, 0xa3}, // INC      DPTR
{0xa5b3, 0xce}, // XCH      A,R6
{0xa5b4, 0xf0}, // MOVX     @DPTR,A
{0xa5b5, 0x90}, // MOV      DPTR,#0x0783
{0xa5b6, 0x07}, //
{0xa5b7, 0x83}, //
{0xa5b8, 0xe0}, // MOVX     A,@DPTR
{0xa5b9, 0xff}, // MOV      R7,A
{0xa5ba, 0x7e}, // MOV      R6,#DeviceParameters(0x00)
{0xa5bb, 0x00}, //
{0xa5bc, 0x90}, // MOV      DPTR,#patch_wLightGap(0x0DF6)
{0xa5bd, 0x0d}, //
{0xa5be, 0xf6}, //
{0xa5bf, 0xee}, // MOV      A,R6
{0xa5c0, 0xf0}, // MOVX     @DPTR,A
{0xa5c1, 0xa3}, // INC      DPTR
{0xa5c2, 0xef}, // MOV      A,R7
{0xa5c3, 0xf0}, // MOVX     @DPTR,A
{0xa5c4, 0x90}, // MOV      DPTR,#0x0BCA
{0xa5c5, 0x0b}, //
{0xa5c6, 0xca}, //
{0xa5c7, 0xe0}, // MOVX     A,@DPTR
{0xa5c8, 0xfc}, // MOV      R4,A
{0xa5c9, 0xa3}, // INC      DPTR
{0xa5ca, 0xe0}, // MOVX     A,@DPTR
{0xa5cb, 0xfd}, // MOV      R5,A
{0xa5cc, 0xd3}, // SETB     C
{0xa5cd, 0x9f}, // SUBB     A,R7
{0xa5ce, 0x74}, // MOV      A,#PipeSetupCommon(0x80)
{0xa5cf, 0x80}, //
{0xa5d0, 0xf8}, // MOV      R0,A
{0xa5d1, 0xec}, // MOV      A,R4
{0xa5d2, 0x64}, // XRL      A,#PipeSetupCommon(0x80)
{0xa5d3, 0x80}, //
{0xa5d4, 0x98}, // SUBB     A,R0
{0xa5d5, 0x40}, // JC       B01:99BB
{0xa5d6, 0x0c}, //
{0xa5d7, 0x90}, // MOV      DPTR,#0x0BC8
{0xa5d8, 0x0b}, //
{0xa5d9, 0xc8}, //
{0xa5da, 0xe0}, // MOVX     A,@DPTR
{0xa5db, 0x04}, // INC      A
{0xa5dc, 0xf0}, // MOVX     @DPTR,A
{0xa5dd, 0xa3}, // INC      DPTR
{0xa5de, 0xe0}, // MOVX     A,@DPTR
{0xa5df, 0x04}, // INC      A
{0xa5e0, 0xf0}, // MOVX     @DPTR,A
{0xa5e1, 0x80}, // SJMP     B01:99E1
{0xa5e2, 0x26}, //
{0xa5e3, 0x90}, // MOV      DPTR,#patch_wLightGap(0x0DF6)
{0xa5e4, 0x0d}, //
{0xa5e5, 0xf6}, //
{0xa5e6, 0xe0}, // MOVX     A,@DPTR
{0xa5e7, 0xfe}, // MOV      R6,A
{0xa5e8, 0xa3}, // INC      DPTR
{0xa5e9, 0xe0}, // MOVX     A,@DPTR
{0xa5ea, 0xff}, // MOV      R7,A
{0xa5eb, 0xc3}, // CLR      C
{0xa5ec, 0xe4}, // CLR      A
{0xa5ed, 0x9f}, // SUBB     A,R7
{0xa5ee, 0xff}, // MOV      R7,A
{0xa5ef, 0xe4}, // CLR      A
{0xa5f0, 0x9e}, // SUBB     A,R6
{0xa5f1, 0xfe}, // MOV      R6,A
{0xa5f2, 0xc3}, // CLR      C
{0xa5f3, 0xed}, // MOV      A,R5
{0xa5f4, 0x9f}, // SUBB     A,R7
{0xa5f5, 0xee}, // MOV      A,R6
{0xa5f6, 0x64}, // XRL      A,#PipeSetupCommon(0x80)
{0xa5f7, 0x80}, //
{0xa5f8, 0xf8}, // MOV      R0,A
{0xa5f9, 0xec}, // MOV      A,R4
{0xa5fa, 0x64}, // XRL      A,#PipeSetupCommon(0x80)
{0xa5fb, 0x80}, //
{0xa5fc, 0x98}, // SUBB     A,R0
{0xa5fd, 0x50}, // JNC      B01:99E1
{0xa5fe, 0x0a}, //
{0xa5ff, 0x90}, // MOV      DPTR,#0x0BC8
{0xa600, 0x0b}, //
{0xa601, 0xc8}, //
{0xa602, 0xe0}, // MOVX     A,@DPTR
{0xa603, 0x14}, // DEC      A
{0xa604, 0xf0}, // MOVX     @DPTR,A
{0xa605, 0xa3}, // INC      DPTR
{0xa606, 0xe0}, // MOVX     A,@DPTR
{0xa607, 0x04}, // INC      A
{0xa608, 0xf0}, // MOVX     @DPTR,A
{0xa609, 0x05}, // INC      c_HeightScale(0x3B)
{0xa60a, 0x3b}, //
{0xa60b, 0x02}, // LJMP     B01:98DA
{0xa60c, 0xf5}, //
{0xa60d, 0x02}, //
{0xa60e, 0x90}, // MOV      DPTR,#AutoFocusInstableFocusMeasureStatus(0x0858)
{0xa60f, 0x08}, //
{0xa610, 0x58}, //
{0xa611, 0x02}, // LJMP
{0xa612, 0x9d}, //
{0xa613, 0x50}, //
{0x9006, 0xba}, // ; Patch break point address high byte;
{0x9007, 0x75}, // ; Patch break point address low byte;
{0x9008, 0x00}, // ; Offset High byte;
{0x9009, 0x00}, // ; Offset Low byte;
{0x900a, 0x02}, // ; Enable BP 0;
{0x900d, 0x01}, // ; Patch break point address bank;
{0x900e, 0xa2}, // ; Patch break point address high byte;
{0x900f, 0x8f}, // ; Patch break point address low byte;
{0x9010, 0x00}, // ; Offset High byte;
{0x9011, 0xcb}, // ; Offset Low byte;
{0x9012, 0x03}, // ; Enable BP 1;
{0x9016, 0xe6}, // ; Patch break point address high byte;
{0x9017, 0x6b}, // ; Patch break point address low byte;
{0x9018, 0x02}, // ; Offset High byte;
{0x9019, 0x6b}, // ; Offset Low byte;
{0x901a, 0x02}, // ; Enable BP 2;
{0x901d, 0x01}, //
{0x901e, 0xac}, // ; Patch break point address high byte;
{0x901f, 0x70}, // ; Patch break point address low byte;
{0x9020, 0x00}, // ; Offset High byte;
{0x9021, 0xc5}, // ; Offset Low byte;
{0x9022, 0x03}, // ; Enable BP 3;
{0x9026, 0x9c}, // ; Patch break point address high byte;
{0x9027, 0x5b}, // ; Patch break point address low byte;
{0x9028, 0x00}, // ; Offset High byte;
{0x9029, 0xbf}, // ; Offset Low byte;
{0x902a, 0x02}, // ; Enable BP 4;
{0x902e, 0x60}, // ; Patch break point address high byte;
{0x902f, 0x1c}, // ; Patch break point address low byte;
{0x9030, 0x01}, // ; Offset High byte;
{0x9031, 0x37}, // ; Offset Low byte;
{0x9032, 0x02}, // ; Enable BP 3;
{0x9035, 0x01}, // ; Patch break point address bank;
{0x9036, 0xba}, // ; Patch break point address high byte;
{0x9037, 0x70}, // ; Patch break point address low byte;
{0x9038, 0x00}, // ; Offset High byte;
{0x9039, 0x00}, // ; Offset Low byte;
{0x903a, 0x03}, // ; Enable BP 6;
{0x903e, 0x21}, // ; Patch break point address high byte;
{0x903f, 0x3f}, // ; Patch break point address low byte;
{0x9040, 0x02}, // ; Offset High byte;
{0x9041, 0x40}, // ; Offset Low byte;
{0x9042, 0x02}, // ; Enable BP 7;
{0x9046, 0x21}, // ; Patch break point address high byte;
{0x9047, 0xea}, // ; Patch break point address low byte;
{0x9048, 0x02}, // ; Offset High byte;
{0x9049, 0x43}, // ; Offset Low byte;
{0x904a, 0x02}, // ; Enable BP 8;
{0x904e, 0xa6}, // ; Patch break point address high byte;
{0x904f, 0x12}, // ; Patch break point address low byte;
{0x9050, 0x02}, // ; Offset High byte;
{0x9051, 0x46}, // ; Offset Low byte;
{0x9052, 0x02}, // ; Enable BP 9;
{0x9056, 0x29}, // ; Patch break point address high byte;
{0x9057, 0xe3}, // ; Patch break point address low byte;
{0x9058, 0x02}, // ; Offset High byte;
{0x9059, 0x49}, // ; Offset Low byte;
{0x905a, 0x02}, // ; Enable BP 10;
{0x905d, 0x01}, // ; Patch break point address bank;
{0x905e, 0x9c}, // ; Patch break point address high byte;
{0x905f, 0x6e}, // ; Patch break point address low byte;
{0x9060, 0x05}, // ; Offset High byte;
{0x9061, 0x00}, // ; Offset Low byte;
{0x9062, 0x02}, // ; Enable BP 11;
{0x9065, 0x01}, // ; Patch break point address bank;
{0x9066, 0xa2}, // ; Patch break point address high byte;
{0x9067, 0x66}, // ; Patch break point address low byte;
{0x9068, 0x02}, // ; Offset High byte;
{0x9069, 0x35}, // ; Offset Low byte;
{0x906a, 0x02}, // ; Enable BP 12;
{0x906d, 0x01}, // ; Patch break point address bank;
{0x906e, 0xb5}, // ; Patch break point address high byte;
{0x906f, 0xc2}, // ; Patch break point address low byte;
{0x9070, 0x02}, // ; Offset High byte;
{0x9071, 0x9b}, // ; Offset Low byte;
{0x9072, 0x02}, // ; Enable BP 13;
{0x9075, 0x01}, // ; Patch break point address bank;
{0x9076, 0xa2}, // ; Patch break point address high byte;
{0x9077, 0xd4}, // ; Patch break point address low byte;
{0x9078, 0x02}, // ; Offset High byte;
{0x9079, 0xbe}, // ; Offset Low byte;
{0x907a, 0x02}, // ; Enable BP 14;
{0x907d, 0x01}, // ; Patch break point address bank;
{0x907e, 0xb7}, // ; Patch break point address high byte;
{0x907f, 0xea}, // ; Patch break point address low byte;
{0x9080, 0x00}, // ; Offset High byte;
{0x9081, 0x02}, // ; Offset Low byte;
{0x9082, 0x03}, // ; Enable BP 15;
{0x9086, 0x67}, // ; Patch break point address high byte;
{0x9087, 0x31}, // ; Patch break point address low byte;
{0x9088, 0x02}, // ; Offset High byte;
{0x9089, 0xf7}, // ; Offset Low byte;
{0x908a, 0x02}, // ; Enable BP 16;
{0x908e, 0x66}, // ; Patch break point address high byte;
{0x908f, 0xed}, // ; Patch break point address low byte;
{0x9090, 0x03}, // ; Offset High byte;
{0x9091, 0x1d}, // ; Offset Low byte;
{0x9092, 0x02}, // ; Enable BP 17;
{0x9096, 0x67}, // ; Patch break point address high byte;
{0x9097, 0x73}, // ; Patch break point address low byte;
{0x9098, 0x03}, // ; Offset High byte;
{0x9099, 0xd3}, // ; Offset Low byte;
{0x909a, 0x02}, // ; Enable BP 18;
{0x909e, 0x20}, // ; Patch break point address high byte;
{0x909f, 0x40}, // ; Patch break point address low byte;
{0x90a0, 0x03}, // ; Offset High byte;
{0x90a1, 0x3b}, // ; Offset Low byte;
{0x90a2, 0x02}, // ; Enable BP 19;
{0x90a6, 0xc5}, // ; Patch break point address high byte;
{0x90a7, 0xc0}, // ; Patch break point address low byte;
{0x90a8, 0x03}, // ; Offset High byte;
{0x90a9, 0xf0}, // ; Offset Low byte;
{0x90aa, 0x02}, // ; Enable BP 20;
{0x90ae, 0x41}, // ; Patch break point address high byte;
{0x90af, 0xb3}, // ; Patch break point address low byte;
{0x90b0, 0x00}, // ; Offset High byte;
{0x90b1, 0xa2}, // ; Offset Low byte;
{0x90b2, 0x02}, // ; Enable BP 21;
{0x90b6, 0x44}, // ; Patch break point address high byte;
{0x90b7, 0xba}, // ; Patch break point address low byte;
{0x90b8, 0x00}, // ; Offset High byte;
{0x90b9, 0xf0}, // ; Offset Low byte;
{0x90ba, 0x03}, // ; Enable BP 22;
{0x90be, 0x89}, // ; Patch break point address high byte;
{0x90bf, 0x99}, // ; Patch break point address low byte;
{0x90c0, 0x04}, // ; Offset High byte;
{0x90c1, 0x00}, // ; Offset Low byte;
{0x90c2, 0x02}, // ; Enable BP 23;
{0x90c6, 0xa7}, // ; Patch break point address high byte;
{0x90c7, 0x91}, // ; Patch break point address low byte;
{0x90c8, 0x04}, // ; Offset High byte;
{0x90c9, 0x21}, // ; Offset Low byte;
{0x90ca, 0x02}, // ; Enable BP 24;
{0x90ce, 0x3a}, // ; Patch break point address high byte;
{0x90cf, 0x51}, // ; Patch break point address low byte;
{0x90d0, 0x00}, // ; Offset High byte;
{0x90d1, 0xa2}, // ; Offset Low byte;
{0x90d2, 0x02}, // ; Enable BP 25;
{0x90d6, 0x86}, // ; Patch break point address high byte;
{0x90d7, 0x54}, // ; Patch break point address low byte;
{0x90d8, 0x04}, // ; Offset High byte;
{0x90d9, 0x47}, // ; Offset Low byte;
{0x90da, 0x02}, // ; Enable BP 26;
{0x9000, 0x01}, // ; Enable patch;
{0xffff, 0x00}, // ; MCU release
};

/* }}} */
/* {{{ Default register values */

static const struct reg_value default_regs[] = {
{0x0013, 0x00}, // unknown
{0x0021, 0x00}, // uwZoomStepSize
{0x0022, 0x01}, // LSB
{0x00b4, 0x01}, // E_div
{0x00b5, 0x01}, // PLL3_div (XXX: or 0x01?)
{0x00e8, 0x01}, // Enable adaptive frame rate
{0x00ed, 0x0a}, // Min Framerate = 10
{0x00ee, 0x1e}, // Max Framerate = 30

{0x7000, 0x08},
{0x5200, 0x09},
{0x7101, 0x44}, // possibly CSI interface setup?
{0x7102, 0x09}, //
{0x7103, 0x00}, //
{0x7104, 0x00}, // OIF threshold = 128
{0x7105, 0x80}, //
{0x7158, 0x00}, //

{0x0143, 0x5f}, // fpUserMaximumIntegrationTime_us
{0x0144, 0x0d}, // LSB

{0x02c2, 0x00}, // uwSensorAnalogGainCeiling
{0x02c3, 0xc0}, // LSB

{0x015e, 0x40}, // fpDigitalGainCeiling
{0x015f, 0x00}, // LSB

{0x0390, 0x01}, // ArcticControl fArcticEnable
{0x0391, 0x00}, // ArcticControl fArcticConfig DEFAULT CONFIG
{0x0392, 0x00}, // ArcticControl fGNFConfig    DEFAULT CONFIG
{0x03a0, 0x14}, // ArcticCCSigmaControl fMaximumCCSigma
{0x03a1, 0x00}, // ArcticCCSigmaControl fDisablePromotion {CompiledExposureTime}
{0x03a2, 0x5a}, // ArcticCCSigmaControl fDamperLowThreshold {MSB}   //2400
{0x03a3, 0xee}, // ArcticCCSigmaControl fDamperLowThreshold {LSB}
{0x03a4, 0x69}, // ArcticCCSigmaControl fDamperHighThreshold {MSB}   //3444736
{0x03a5, 0x49}, // ArcticCCSigmaControl fDamperHighThreshold {LSB}
{0x03a6, 0x3e}, // ArcticCCSigmaControl fY1 {MSB}  // Low threshold
{0x03a7, 0x00}, // ArcticCCSigmaControl fY1 {LSB}
{0x03a8, 0x39}, // ArcticCCSigmaControl fY2 {MSB}  // High threshold
{0x03a9, 0x33}, // ArcticCCSigmaControl fY2 {LSB}
{0x03b0, 0x60}, // ArcticCCSigmaControl fMaximumRing
{0x03b1, 0x00}, // ArcticCCSigmaControl fDisablePromotion {CompiledExposureTime}
{0x03b2, 0x5a}, // ArcticCCSigmaControl fDamperLowThreshold {MSB}    //24000
{0x03b3, 0xee}, // ArcticCCSigmaControl fDamperLowThreshold {LSB}
{0x03b4, 0x69}, // ArcticCCSigmaControl DamperHighThreshold {MSB}    //3444736
{0x03b5, 0x49}, // ArcticCCSigmaControl DamperHighThreshold {LSB}
{0x03b6, 0x3e}, // ArcticCCSigmaControl fY1 {MSB}  //Low threshold
{0x03b7, 0x00}, // ArcticCCSigmaControl fY1 {LSB}
{0x03b8, 0x3d}, // ArcticCCSigmaControl fY2 {MSB}  //High threshold
{0x03b9, 0x20}, // ArcticCCSigmaControl fY2 {LSB}
{0x03c0, 0x10}, // ArcticCCSigmaControl fMaximumScoring
{0x03c1, 0x00}, // ArcticCCSigmaControl fDisablePromotion {CompiledExposureTime}
{0x03c2, 0x5a}, // ArcticCCSigmaControl fDamperLowThreshold {MSB}    //24000
{0x03c3, 0xee}, // ArcticCCSigmaControl fDamperLowThreshold {LSB}
{0x03c4, 0x69}, // ArcticCCSigmaControl DamperHighThreshold {MSB}    //3444736
{0x03c5, 0x49}, // ArcticCCSigmaControl DamperHighThreshold {LSB}
{0x03c6, 0x3a}, // ArcticCCSigmaControl fMinimumDamperOutput {MSB}
{0x03c7, 0x80}, // ArcticCCSigmaControl fMinimumDamperOutput {LSB}
{0x03d0, 0x64}, // ArcticCCSigmaControl fMaximumScoring
{0x03d1, 0x00}, // ArcticCCSigmaControl fDisablePromotion {CompiledExposureTime}
{0x03d2, 0x5a}, // ArcticCCSigmaControl fDamperLowThreshold {MSB}   //24000
{0x03d3, 0xee}, // ArcticCCSigmaControl fDamperLowThreshold {LSB}
{0x03d4, 0x69}, // ArcticCCSigmaControl DamperHighThreshold {MSB}   //3444736
{0x03d5, 0x49}, // ArcticCCSigmaControl DamperHighThreshold {LSB}
{0x03d6, 0x34}, // ArcticCCSigmaControl fMinimumDamperOutput {MSB}
{0x03d7, 0xd1}, // ArcticCCSigmaControl fMinimumDamperOutput {LSB}

{0x004c, 0x08}, // PipeSetupBank0 fPeakingGain
{0x006c, 0x08}, // PipeSetupBank1 fPeakingGain
{0x0350, 0x00}, // PeakingControl fDisableGainDamping  {CompiledExposureTime}
{0x0351, 0x5a}, // PeakingControl fDamperLowThreshold_Gain  {MSB}   //24000
{0x0352, 0xee}, // PeakingControl fDamperLowThreshold_Gain  {LSB}
{0x0353, 0x69}, // PeakingControl fDamperHighThreshold_Gain  {MSB}  //3444736
{0x0354, 0x49}, // PeakingControl fDamperHighThreshold_Gain  {LSB}
{0x0355, 0x39}, // PeakingControl fMinimumDamperOutput_Gain  {MSB}
{0x0356, 0x6d}, // PeakingControl fMinimumDamperOutput_Gain  {LSB}
{0x0357, 0x19}, // PeakingControl fUserPeakLoThresh
{0x0358, 0x00}, // PeakingControl fDisableCoringDamping  {CompiledExposureTime}
{0x0359, 0x3c}, // PeakingControl fUserPeakHiThresh
{0x035a, 0x5a}, // PeakingControl fDamperLowThreshold_Coring  {MSB}  //24000
{0x035b, 0xee}, // PeakingControl fDamperLowThreshold_Coring  {LSB}
{0x035c, 0x69}, // PeakingControl fDamperHighThreshold_Coring {MSB}  //3444736
{0x035d, 0x49}, // PeakingControl fDamperHighThreshold_Coring  {LSB}
{0x035e, 0x39}, // PeakingControl fMinimumDamperOutput_Coring  {MSB}
{0x035f, 0x85}, // PeakingControl fMinimumDamperOutput_Coring  {LSB}

{0x0049, 0x14}, // PipeSetupBank0 bGammaGain
{0x004a, 0x0d}, // PipeSetupBank0 bGammaInterpolationGain
{0x0069, 0x14}, // PipeSetupBank1 bGammaGain
{0x006a, 0x0d}, // PipeSetupBank1 bGammaInterpolationGain
{0x0090, 0x5a}, // GammaGainDamperControl fpX1 {MSB}   //24000
{0x0091, 0xee}, // GammaGainDamperControl fpX1 {LSB}
{0x0092, 0x3e}, // GammaGainDamperControl fpY1 {MSB}   //1
{0x0093, 0x00}, // GammaGainDamperControl fpY1 {LSB}
{0x0094, 0x69}, // GammaGainDamperControl fpX2 {MSB}   //3444736
{0x0095, 0x49}, // GammaGainDamperControl fpX2 {LSB}
{0x0096, 0x39}, // GammaGainDamperControl fpY2 {MSB}   //0.238
{0x0097, 0xcf}, // GammaGainDamperControl fpY2 {LSB}
{0x0098, 0x00}, // GammaGainDamperControl fDisable {CompiledExposureTime}
{0x00a0, 0x5a}, // GammaInterpolationDamperControl fpX1 {MSB}   //24000
{0x00a1, 0xee}, // GammaInterpolationDamperControl fpX1 {LSB}
{0x00a2, 0x3e}, // GammaInterpolationDamperControl fpY1 {MSB}   //1
{0x00a3, 0x00}, // GammaInterpolationDamperControl fpY1 {LSB}
{0x00a4, 0x69}, // GammaInterpolationDamperControl fpX2 {MSB}   //3444736
{0x00a5, 0x49}, // GammaInterpolationDamperControl fpX2 {LSB}
{0x00a6, 0x3b}, // GammaInterpolationDamperControl fpY2 {MSB}   //0.4375
{0x00a7, 0x80}, // GammaInterpolationDamperControl fpY2 {LSB}
{0x00a8, 0x00}, // GammaInterpolationDamperControl fDisable {CompiledExposureTime}

{0x0420, 0x00}, // C0_GreenRed_X 145
{0x0421, 0x91}, // C0_GreenRed_X LSB
{0x0422, 0xff}, // C0_GreenRed_Y -108
{0x0423, 0x94}, // C0_GreenRed_Y LSB
{0x0424, 0x00}, // C0_GreenRed_X2 125
{0x0425, 0x7d}, // C0_GreenRed_X2 LSB
{0x0426, 0x00}, // C0_GreenRed_Y2 170
{0x0427, 0xaa}, // C0_GreenRed_Y2 LSB
{0x0428, 0x00}, // C0_GreenRed_XY 57
{0x0429, 0x39}, // C0_GreenRed_XY LSB
{0x042a, 0xff}, // C0_GreenRed_X2Y -75
{0x042b, 0xb5}, // C0_GreenRed_X2Y LSB
{0x042c, 0x01}, // C0_GreenRed_XY2 444
{0x042d, 0xbc}, // C0_GreenRed_XY2 LSB
{0x042e, 0xff}, // C0_GreenRed_X2Y2 -45
{0x042f, 0xd3}, // C0_GreenRed_X2Y2 LSB
{0x0430, 0x00}, // C0_Red_X 175
{0x0431, 0xaf}, // C0_Red_X LSB
{0x0432, 0xff}, // C0_Red_Y -164
{0x0433, 0x5c}, // C0_Red_Y LSB
{0x0434, 0x00}, // C0_Red_X2 248
{0x0435, 0xf8}, // C0_Red_X2 LSB
{0x0436, 0x01}, // C0_Red_Y2 285
{0x0437, 0x1d}, // C0_Red_Y2 LSB
{0x0438, 0xff}, // C0_Red_XY -127
{0x0439, 0x81}, // C0_Red_XY LSB
{0x043a, 0xff}, // C0_Red_X2Y -143
{0x043b, 0x71}, // C0_Red_X2Y LSB
{0x043c, 0x01}, // C0_Red_XY2 375
{0x043d, 0x77}, // C0_Red_XY2 LSB
{0x043e, 0xff}, // C0_Red_X2Y2 -174
{0x043f, 0x52}, // C0_Red_X2Y2 LSB
{0x0450, 0x00}, // C0_Blue_X 117
{0x0451, 0x75}, // C0_Blue_X LSB
{0x0452, 0xff}, // C0_Blue_Y -114
{0x0453, 0x8e}, // C0_Blue_Y LSB
{0x0454, 0x00}, // C0_Blue_X2 106
{0x0455, 0x6a}, // C0_Blue_X2 LSB
{0x0456, 0x00}, // C0_Blue_Y2 138
{0x0457, 0x8a}, // C0_Blue_Y2 LSB
{0x0458, 0xff}, // C0_Blue_XY -90
{0x0459, 0xa6}, // C0_Blue_XY LSB
{0x045a, 0xff}, // C0_Blue_X2Y -86
{0x045b, 0xaa}, // C0_Blue_X2Y LSB
{0x045c, 0x00}, // C0_Blue_XY2 224
{0x045d, 0xe0}, // C0_Blue_XY2 LSB
{0x045e, 0xff}, // C0_Blue_X2Y2 -48
{0x045f, 0xd0}, // C0_Blue_X2Y2 LSB
{0x0440, 0x00}, // C0_GreenBlue_X 91
{0x0441, 0x5b}, // C0_GreenBlue_X LSB
{0x0442, 0xff}, // C0_GreenBlue_Y -123
{0x0443, 0x85}, // C0_GreenBlue_Y LSB
{0x0444, 0x00}, // C0_GreenBlue_X2 145
{0x0445, 0x91}, // C0_GreenBlue_X2 LSB
{0x0446, 0x00}, // C0_GreenBlue_Y2 151
{0x0447, 0x97}, // C0_GreenBlue_Y2 LSB
{0x0448, 0x00}, // C0_GreenBlue_XY 44
{0x0449, 0x2c}, // C0_GreenBlue_XY LSB
{0x044a, 0xff}, // C0_GreenBlue_X2Y -207
{0x044b, 0x31}, // C0_GreenBlue_X2Y LSB
{0x044c, 0x01}, // C0_GreenBlue_XY2 349
{0x044d, 0x5d}, // C0_GreenBlue_XY2 LSB
{0x044e, 0xff}, // C0_GreenBlue_X2Y2 -42
{0x044f, 0xd6}, // C0_GreenBlue_X2Y2 LSB
{0x0460, 0x00}, // C1_GreenRed_X 148
{0x0461, 0x94}, // C1_GreenRed_X LSB
{0x0462, 0xff}, // C1_GreenRed_Y -106
{0x0463, 0x96}, // C1_GreenRed_Y LSB
{0x0464, 0x00}, // C1_GreenRed_X2 124
{0x0465, 0x7c}, // C1_GreenRed_X2 LSB
{0x0466, 0x00}, // C1_GreenRed_Y2 165
{0x0467, 0xa5}, // C1_GreenRed_Y2 LSB
{0x0468, 0x00}, // C1_GreenRed_XY 66
{0x0469, 0x42}, // C1_GreenRed_XY LSB
{0x046a, 0xff}, // C1_GreenRed_X2Y -80
{0x046b, 0xb0}, // C1_GreenRed_X2Y LSB
{0x046c, 0x01}, // C1_GreenRed_XY2 450
{0x046d, 0xc2}, // C1_GreenRed_XY2 LSB
{0x046e, 0xff}, // C1_GreenRed_X2Y2 -35
{0x046f, 0xdd}, // C1_GreenRed_X2Y2 LSB
{0x0470, 0x00}, // C1_Red_X 170
{0x0471, 0xaa}, // C1_Red_X LSB
{0x0472, 0xff}, // C1_Red_Y -155
{0x0473, 0x65}, // C1_Red_Y LSB
{0x0474, 0x00}, // C1_Red_X2 234
{0x0475, 0xea}, // C1_Red_X2 LSB
{0x0476, 0x01}, // C1_Red_Y2 270
{0x0477, 0x0e}, // C1_Red_Y2 LSB
{0x0478, 0xff}, // C1_Red_XY -121
{0x0479, 0x87}, // C1_Red_XY LSB
{0x047a, 0xff}, // C1_Red_X2Y -145
{0x047b, 0x6f}, // C1_Red_X2Y LSB
{0x047c, 0x01}, // C1_Red_XY2 360
{0x047d, 0x68}, // C1_Red_XY2 LSB
{0x047e, 0xff}, // C1_Red_X2Y2 -152
{0x047f, 0x68}, // C1_Red_X2Y2 LSB
{0x0490, 0x00}, // C1_Blue_X 125
{0x0491, 0x7d}, // C1_Blue_X LSB
{0x0492, 0xff}, // C1_Blue_Y -110
{0x0493, 0x92}, // C1_Blue_Y LSB
{0x0494, 0x00}, // C1_Blue_X2 103
{0x0495, 0x67}, // C1_Blue_X2 LSB
{0x0496, 0x00}, // C1_Blue_Y2 132
{0x0497, 0x84}, // C1_Blue_Y2 LSB
{0x0498, 0xff}, // C1_Blue_XY -93
{0x0499, 0xa3}, // C1_Blue_XY LSB
{0x049a, 0xff}, // C1_Blue_X2Y -104
{0x049b, 0x98}, // C1_Blue_X2Y LSB
{0x049c, 0x01}, // C1_Blue_XY2 256
{0x049d, 0x00}, // C1_Blue_XY2 LSB
{0x049e, 0xff}, // C1_Blue_X2Y2 -32
{0x049f, 0xe0}, // C1_Blue_X2Y2 LSB
{0x0480, 0x00}, // C1_GreenBlue_X 90
{0x0481, 0x5a}, // C1_GreenBlue_X LSB
{0x0482, 0xff}, // C1_GreenBlue_Y -116
{0x0483, 0x8c}, // C1_GreenBlue_Y LSB
{0x0484, 0x00}, // C1_GreenBlue_X2 141
{0x0485, 0x8d}, // C1_GreenBlue_X2 LSB
{0x0486, 0x00}, // C1_GreenBlue_Y2 148
{0x0487, 0x94}, // C1_GreenBlue_Y2 LSB
{0x0488, 0x00}, // C1_GreenBlue_XY 55
{0x0489, 0x37}, // C1_GreenBlue_XY LSB
{0x048a, 0xff}, // C1_GreenBlue_X2Y -226
{0x048b, 0x1e}, // C1_GreenBlue_X2Y LSB
{0x048c, 0x01}, // C1_GreenBlue_XY2 349
{0x048d, 0x5d}, // C1_GreenBlue_XY2 LSB
{0x048e, 0xff}, // C1_GreenBlue_X2Y2 -33
{0x048f, 0xdf}, // C1_GreenBlue_X2Y2 LSB
{0x04a0, 0x00}, // C2_GreenRed_X 150
{0x04a1, 0x96}, // C2_GreenRed_X LSB
{0x04a2, 0xff}, // C2_GreenRed_Y -97
{0x04a3, 0x9f}, // C2_GreenRed_Y LSB
{0x04a4, 0x00}, // C2_GreenRed_X2 117
{0x04a5, 0x75}, // C2_GreenRed_X2 LSB
{0x04a6, 0x00}, // C2_GreenRed_Y2 150
{0x04a7, 0x96}, // C2_GreenRed_Y2 LSB
{0x04a8, 0x00}, // C2_GreenRed_XY 62
{0x04a9, 0x3e}, // C2_GreenRed_XY LSB
{0x04aa, 0xff}, // C2_GreenRed_X2Y -118
{0x04ab, 0x8a}, // C2_GreenRed_X2Y LSB
{0x04ac, 0x01}, // C2_GreenRed_XY2 456
{0x04ad, 0xc8}, // C2_GreenRed_XY2 LSB
{0x04ae, 0xff}, // C2_GreenRed_X2Y2 -12
{0x04af, 0xf4}, // C2_GreenRed_X2Y2 LSB
{0x04b0, 0x00}, // C2_Red_X 136
{0x04b1, 0x88}, // C2_Red_X LSB
{0x04b2, 0xff}, // C2_Red_Y -118
{0x04b3, 0x8a}, // C2_Red_Y LSB
{0x04b4, 0x00}, // C2_Red_X2 166
{0x04b5, 0xa6}, // C2_Red_X2 LSB
{0x04b6, 0x00}, // C2_Red_Y2 194
{0x04b7, 0xc2}, // C2_Red_Y2 LSB
{0x04b8, 0xff}, // C2_Red_XY -96
{0x04b9, 0xa0}, // C2_Red_XY LSB
{0x04ba, 0xff}, // C2_Red_X2Y -229
{0x04bb, 0x1b}, // C2_Red_X2Y LSB
{0x04bc, 0x01}, // C2_Red_XY2 339
{0x04bd, 0x53}, // C2_Red_XY2 LSB
{0x04be, 0xff}, // C2_Red_X2Y2 -50
{0x04bf, 0xce}, // C2_Red_X2Y2 LSB
{0x04d0, 0x00}, // C2_Blue_X 137
{0x04d1, 0x89}, // C2_Blue_X LSB
{0x04d2, 0xff}, // C2_Blue_Y -101
{0x04d3, 0x9b}, // C2_Blue_Y LSB
{0x04d4, 0x00}, // C2_Blue_X2 94
{0x04d5, 0x5e}, // C2_Blue_X2 LSB
{0x04d6, 0x00}, // C2_Blue_Y2 120
{0x04d7, 0x78}, // C2_Blue_Y2 LSB
{0x04d8, 0xff}, // C2_Blue_XY -107
{0x04d9, 0x95}, // C2_Blue_XY LSB
{0x04da, 0xff}, // C2_Blue_X2Y -135
{0x04db, 0x79}, // C2_Blue_X2Y LSB
{0x04dc, 0x01}, // C2_Blue_XY2 274
{0x04dd, 0x12}, // C2_Blue_XY2 LSB
{0x04de, 0xff}, // C2_Blue_X2Y2 -4
{0x04df, 0xfc}, // C2_Blue_X2Y2 LSB
{0x04c0, 0x00}, // C2_GreenBlue_X 91
{0x04c1, 0x5b}, // C2_GreenBlue_X LSB
{0x04c2, 0xff}, // C2_GreenBlue_Y -100
{0x04c3, 0x9c}, // C2_GreenBlue_Y LSB
{0x04c4, 0x00}, // C2_GreenBlue_X2 130
{0x04c5, 0x82}, // C2_GreenBlue_X2 LSB
{0x04c6, 0x00}, // C2_GreenBlue_Y2 139
{0x04c7, 0x8b}, // C2_GreenBlue_Y2 LSB
{0x04c8, 0x00}, // C2_GreenBlue_XY 54
{0x04c9, 0x36}, // C2_GreenBlue_XY LSB
{0x04ca, 0xfe}, // C2_GreenBlue_X2Y -260
{0x04cb, 0xfc}, // C2_GreenBlue_X2Y LSB
{0x04cc, 0x01}, // C2_GreenBlue_XY2 380
{0x04cd, 0x7c}, // C2_GreenBlue_XY2 LSB
{0x04ce, 0xff}, // C2_GreenBlue_X2Y2 -10
{0x04cf, 0xf6}, // C2_GreenBlue_X2Y2 LSB
{0x04e0, 0x00}, // C3_GreenRed_X 149
{0x04e1, 0xea}, // C3_GreenRed_X LSB
{0x04e2, 0xff}, // C3_GreenRed_Y -90
{0x04e3, 0xa6}, // C3_GreenRed_Y LSB
{0x04e4, 0x00}, // C3_GreenRed_X2 118
{0x04e5, 0x76}, // C3_GreenRed_X2 LSB
{0x04e6, 0x00}, // C3_GreenRed_Y2 148
{0x04e7, 0x94}, // C3_GreenRed_Y2 LSB
{0x04e8, 0xff}, // C3_GreenRed_XY 76
{0x04e9, 0x8b}, // C3_GreenRed_XY LSB
{0x04ea, 0xff}, // C3_GreenRed_X2Y -64
{0x04eb, 0xc0}, // C3_GreenRed_X2Y LSB
{0x04ec, 0x01}, // C3_GreenRed_XY2 449
{0x04ed, 0xc1}, // C3_GreenRed_XY2 LSB
{0x04ee, 0xff}, // C3_GreenRed_X2Y2 -24
{0x04ef, 0xe8}, // C3_GreenRed_X2Y2 LSB
{0x04f0, 0x00}, // C3_Red_X 146
{0x04f1, 0xfb}, // C3_Red_X LSB
{0x04f2, 0xff}, // C3_Red_Y -119
{0x04f3, 0x89}, // C3_Red_Y LSB
{0x04f4, 0x00}, // C3_Red_X2 179
{0x04f5, 0xd0}, // C3_Red_X2 LSB
{0x04f6, 0x00}, // C3_Red_Y2 215
{0x04f7, 0xd0}, // C3_Red_Y2 LSB
{0x04f8, 0x01}, // C3_Red_XY -84
{0x04f9, 0x3e}, // C3_Red_XY LSB
{0x04fa, 0xff}, // C3_Red_X2Y -122
{0x04fb, 0x86}, // C3_Red_X2Y LSB
{0x04fc, 0x01}, // C3_Red_XY2 287
{0x04fd, 0x1f}, // C3_Red_XY2 LSB
{0x04fe, 0xff}, // C3_Red_X2Y2 -102
{0x04ff, 0x20}, // C3_Red_X2Y2 LSB
{0x0510, 0x00}, // C3_Blue_X 144
{0x0511, 0xff}, // C3_Blue_X LSB
{0x0512, 0xff}, // C3_Blue_Y -94
{0x0513, 0xa2}, // C3_Blue_Y LSB
{0x0514, 0x00}, // C3_Blue_X2 96
{0x0515, 0x60}, // C3_Blue_X2 LSB
{0x0516, 0x00}, // C3_Blue_Y2 123
{0x0517, 0x7b}, // C3_Blue_Y2 LSB
{0x0518, 0xff}, // C3_Blue_XY -118
{0x0519, 0x8a}, // C3_Blue_XY LSB
{0x051a, 0xff}, // C3_Blue_X2Y -126
{0x051b, 0x82}, // C3_Blue_X2Y LSB
{0x051c, 0xff}, // C3_Blue_XY2 272
{0x051d, 0x8a}, // C3_Blue_XY2 LSB
{0x051e, 0x00}, // C3_Blue_X2Y2 -19
{0x051f, 0x10}, // C3_Blue_X2Y2 LSB
{0x0500, 0x00}, // C3_GreenBlue_X 84
{0x0501, 0xea}, // C3_GreenBlue_X LSB
{0x0502, 0xff}, // C3_GreenBlue_Y -99
{0x0503, 0x9d}, // C3_GreenBlue_Y LSB
{0x0504, 0x00}, // C3_GreenBlue_X2 125
{0x0505, 0x7d}, // C3_GreenBlue_X2 LSB
{0x0506, 0x00}, // C3_GreenBlue_Y2 142
{0x0507, 0x8e}, // C3_GreenBlue_Y2 LSB
{0x0508, 0xff}, // C3_GreenBlue_XY 75
{0x0509, 0x8b}, // C3_GreenBlue_XY LSB
{0x050a, 0xff}, // C3_GreenBlue_X2Y -200
{0x050b, 0x38}, // C3_GreenBlue_X2Y LSB
{0x050c, 0x01}, // C3_GreenBlue_XY2 348
{0x050d, 0x5c}, // C3_GreenBlue_XY2 LSB
{0x050e, 0xff}, // C3_GreenBlue_X2Y2 -21
{0x050f, 0xeb}, // C3_GreenBlue_X2Y2 LSB
{0x0561, 0x0d}, // C0 Unity
{0x0562, 0x0a}, // C1 Unity
{0x0563, 0x06}, // C2 Unity
{0x0564, 0x01}, // C3 Unity
{0x0324, 0x39}, // NormRedGain_Cast0 Hor
{0x0325, 0xae}, // NormRedGain_Cast0_LSB
{0x0326, 0x3a}, // NormRedGain_Cast1 IncA
{0x0327, 0x29}, // NormRedGain_Cast1_LSB
{0x0328, 0x3b}, // NormRedGain_Cast2 CWF
{0x0329, 0x0a}, // NormRedGain_Cast2_LSB
{0x032a, 0x3b}, // NormRedGain_Cast3 D65
{0x032b, 0x62}, // NormRedGain_Cast3_LSB
{0x0320, 0x01}, // AntiVignetteControl - Enable
{0x0321, 0x04}, // NbOfPresets
{0x0322, 0x01}, // AdaptiveAntiVignetteControlEnable - Enable
{0x0323, 0x01}, // LoLightAntiVignetteControlDisable - Damper Off
{0x0330, 0x01}, // Turn off colour matrix damper
{0x0384, 0x00}, // Turn off colour effects
{0x0337, 0x01}, // Turn on adaptive colour matrix
{0x03ec, 0x39}, // Matrix 0
{0x03ed, 0x85}, // LSB
{0x03fc, 0x3a}, // Matrix 1
{0x03fd, 0x14}, // LSB
{0x040c, 0x3a}, // Matrix 2
{0x040d, 0xf6}, // LSB
{0x041c, 0x3b}, // Matrix 3
{0x041d, 0x9a}, // LSB
{0x03e0, 0xb6}, // GInR
{0x03e1, 0x04}, //
{0x03e2, 0xbb}, // BInR
{0x03e3, 0xe9}, //
{0x03e4, 0xbc}, // RInG
{0x03e5, 0x70}, //
{0x03e6, 0x37}, // BInG
{0x03e7, 0x02}, //
{0x03e8, 0xbc}, // RInB
{0x03e9, 0x00}, //
{0x03ea, 0xbf}, // GInB
{0x03eb, 0x12}, //
{0x03f0, 0xba}, // GInR
{0x03f1, 0x7b}, //
{0x03f2, 0xba}, // BInR
{0x03f3, 0x83}, //
{0x03f4, 0xbb}, // RInG
{0x03f5, 0xbc}, //
{0x03f6, 0x38}, // BInG
{0x03f7, 0x2d}, //
{0x03f8, 0xbb}, // RInB
{0x03f9, 0x23}, //
{0x03fa, 0xbd}, // GInB
{0x03fb, 0xac}, //
{0x0400, 0xbe}, // GInR
{0x0401, 0x96}, //
{0x0402, 0xb9}, // BInR
{0x0403, 0xbe}, //
{0x0404, 0xbb}, // RInG
{0x0405, 0x57}, //
{0x0406, 0x3a}, // BInG
{0x0407, 0xbb}, //
{0x0408, 0xb3}, // RInB
{0x0409, 0x17}, //
{0x040a, 0xbe}, // GInB
{0x040b, 0x66}, //
{0x0410, 0xbb}, // GInR
{0x0411, 0x2a}, //
{0x0412, 0xba}, // BInR
{0x0413, 0x00}, //
{0x0414, 0xbb}, // RInG
{0x0415, 0x10}, //
{0x0416, 0xb8}, // BInG
{0x0417, 0xcd}, //
{0x0418, 0xb7}, // RInB
{0x0419, 0x5c}, //
{0x041a, 0xbb}, // GInB
{0x041b, 0x6c}, //
{0x01f8, 0x3c}, // fpMaximumDistanceAllowedFromLocus
{0x01f9, 0x00}, // =0.5
{0x01fa, 0x00}, // fEnableConstrainedWhiteBalance = false
{0x02a2, 0x3e}, // fpRedTilt
{0x02a3, 0x00}, // = 1.00
{0x02a4, 0x3e}, // fpGreenTilt1
{0x02a5, 0x00}, // = 1.00
{0x02a6, 0x3e}, // fpGreenTilt2
{0x02a7, 0x00}, // = 1.00
{0x02a8, 0x3e}, // fpBlueTilt
{0x02a9, 0x00}, // = 1.00
{0x056c, 0x42}, // fpRedTilt
{0x056d, 0x00}, // = 4.00
{0x056e, 0x42}, // fpGreenTilt1
{0x056f, 0x00}, // = 4.00
{0x0570, 0x42}, // fpGreenTilt2
{0x0571, 0x00}, // = 4.00
{0x0572, 0x42}, // fpBlueTilt
{0x0573, 0x00}, // = 4.00

{0x0081, 0x58}, // PipeSetupCommon bColourSaturation
{0x0588, 0x00}, // ColourSaturationDamper fDisable {CompiledExposureTime}
{0x0589, 0x5a}, // ColourSaturationDamper fpLowThreshold {MSB}
{0x058a, 0xee}, // ColourSaturationDamper fpLowThreshold {LSB}
{0x058b, 0x69}, // ColourSaturationDamper fpHighThreshold {MSB}
{0x058c, 0x49}, // ColourSaturationDamper fpHighThreshold {LSB}
{0x058d, 0x3d}, // ColourSaturationDamper fpMinimumOutput {MSB}
{0x058e, 0x3d}, // ColourSaturationDamper fpMinimumOutput {LSB}
{0x0080, 0x6c}, // PipeSetupCommon bContrast
{0x0082, 0x5a}, // PipeSetupCommon bBrightness

{0x065a, 0x00}, // AFStatsControls->bWindowsSystem = 7 zone AF system
{0x06c9, 0x01}, // FLADriverLowLevelParameters->AutoSkipNextFrame = ENABLED
{0x06cd, 0x01}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefMacro MSB = 445
{0x06ce, 0xbd}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefMacro LSB
{0x06cf, 0x00}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefInfinity MSB = 147
{0x06d0, 0x93}, // FLADriverLowLevelParameters->AF_OTP_uwHostDefInfinity LSB
{0x06d1, 0x02}, // FLADriverLowLevelParameters->AF_OTP_bStepsMultiStepDriver = 2 step driver
{0x06d2, 0x30}, // FLADriverLowLevelParameters->AF_OTP_uwMultiStepTimeDelay MSB = 12.5ms
{0x06d3, 0xd4}, // FLADriverLowLevelParameters->AF_OTP_uwMultiStepTimeDelay LSB
{0x06d4, 0x01}, // FLADriverLowLevelParameters->AF_OTP_fHostEnableOTPRead (1 = disabled)
{0x06db, 0x59}, // FLADriverLowLevelParameters->fpActuatorResponseTime MSB 12.5ms (FP900)
{0x06dc, 0x0d}, // FLADriverLowLevelParameters->fpActuatorResponseTime LSB
{0x0730, 0x00}, // FocusRangeConstants->wFullRange_LensMinPosition MSB = 0
{0x0731, 0x00}, // FocusRangeConstants->wFullRange_LensMinPosition LSB
{0x0732, 0x03}, // FocusRangeConstants->wFullRange_LensMaxPosition MSB = 1023
{0x0733, 0xff}, // FocusRangeConstants->wFullRange_LensMaxPosition LSB
{0x0734, 0x03}, // FocusRangeConstants->wFullRange_LensRecoveryPosition MSB = 880
{0x0735, 0x70}, // FocusRangeConstants->wFullRange_LensRecoveryPosition LSB
{0x0755, 0x01}, // AutoFocusControls->fEnableSimpleCoarseThEvaluation = ENABLED
{0x0756, 0x03}, // AutoFocusControls->bSelectedMultizoneBehavior = REGIONSELECTIONMETHOD_AVERAGE //mk change to 0x3
{0x075b, 0x01}, // AutoFocusControls->fEnableTrackingThresholdEvaluation = DISABLED //MK change to 0x1
{0x075e, 0x00}, // AutoFocusControls->fFineToCoarseAutoTransitionEnable = DISABLED
{0x0764, 0x01}, // AutoFocusControls->fResetHCSPos = TRUE = Start from Recovery Position for every HCS
{0x0766, 0x01}, // AutoFocusControls->fEnablePrioritiesMacro = FALSE = Do not prioritise Macro //mk change to 0x01
{0x0768, 0x01}, // AutoFocusControls->fEnableInterpolationAfterFineSearch = TRUE
{0x076a, 0x00}, // AutoFocusControls->fReducedZoneSetup = TRUE //mk change to 0x0
{0x0758, 0x01}, // AutoFocusControls->bWeighedFunctionSelected = TRAPEZIUM
{0x075c, 0x01}, // AutoFocusControls->fEnableHeuristicMethod = FALSE
{0x0770, 0x98}, // AutoFocusConstants->bCoarseStep = 95
{0x0771, 0x19}, // AutoFocusConstants->bFineStep = 16
{0x0772, 0x1b}, // AutoFocusConstants->bFullSearchStep = 27
{0x0774, 0x01}, // AutoFocusConstants->uwFineThreshold MSB = 330
{0x0775, 0x4a}, // AutoFocusConstants->uwFineThreshold LSB
{0x0777, 0x00}, // AutoFocusConstants->uwBacklightThreshold MSB = 69
{0x0778, 0x45}, // AutoFocusConstants->uwBacklightThreshold LSB
{0x0779, 0x00}, // AutoFocusConstants->uwMotionBlurInRatio MSB = 2
{0x077a, 0x02}, // AutoFocusConstants->uwMotionBlurInRatio LSB
{0x077d, 0x01}, // AutoFocusConstants->bMaxNumberContinuouslyInstableTime = 1
{0x077e, 0x03}, // AutoFocusConstants->bMaxNumberContinuouslyStableFrame = 3
{0x0783, 0x10}, // AutoFocusConstants->bLightGap = 10
{0x0785, 0x14}, // AutoFocusConstants->uwDeltaValue = 20
{0x0788, 0x04}, // AutoFocusConstants->bMinNumberMacroRegion = 4 //mk add
{0x0846, 0x06}, // AutoFocusHeuristicConstants->bHighToMaxFMShiftFactor = 6
{0x0847, 0x05}, // AutoFocusHeuristicConstants->bLowToHighFMShiftFactor = 5

{0xc41a, 0x05}, // TEST_LP_TX (clock slew rate)
{0xc423, 0x11}, // TEST_LP_TX_SLEW_RATE_DL1
{0xc427, 0x11}, // TEST_LP_TX_SLEW_RATE_DL2

{0x4708, 0x00}, // av2x2_h_offset
{0x4709, 0x00}, // LSB
{0x4710, 0x00}, // av2x2_v_offset {0x4710 & 11} are correct!
{0x4711, 0x00}, // LSB

{0x300b, 0x09}, // esc_clk_div (clk_sys div by 10)

{0x0030, 0x11}, // max derating ratios

/* allwinner mods */
{0x004c, 0x0b}, // PipeSetupBank0 fPeakingGain
{0x006c, 0x08}, // PipeSetupBank1 fPeakingGain

/* lens driver settings */
{0x06d5, 0x01},	// AF_VCM_uwLowLevelMacroPos
{0x06d6, 0xbd},	// LSB
{0x06d7, 0x00}, // AF_VCM_uwLowLevelInfinityPos
{0x06d8, 0x93}, // LSB
{0x06d9, 0x00}, // ???
{0x06da, 0x93}, // LSB
{0x06cd, 0x02},
{0x06ce, 0x00},
{0x06cf, 0x00},
{0x06d0, 0x00},
{0x0734, 0x03}, // FocusRangeConstants->wFullRange_LensRecoveryPosition MSB = 880
{0x0735, 0xff},

{0x0770, 0xa0}, // coarse step
{0x0771, 0x15}, // fine step

{0x0783, 0x15}, // light gap
{0x0756, 0x00}, // REGIONSELECTIONMETHOD_AVERAGE

{0x331e, 0x03},
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
		struct v4l2_ctrl *auto_exposure;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *metering;
		struct v4l2_ctrl *exposure_bias;
	};
	struct v4l2_ctrl *d_gain;
	struct v4l2_ctrl *a_gain;
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

#if 0
// cargo cult table
-1300 0xf6
-1000 0xf7
 -700 0xf8
 -300 0xf9
    0 0xfd
  300 0x03
  700 0x05
 1000 0x06
 1300 0x07
#endif

#define AE_BIAS_MENU_DEFAULT_VALUE_INDEX 6
static const s64 ae_bias_menu_values[] = {
	-7000, -6000, -5000, -4000, -3000, -2000, -1000,
	0, 1000, 2000, 3000, 4000, 5000, 6000, 7000
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
				   HM5065_REG_AF_MODE_MANUAL);
		if (ret)
			return ret;

#if 0
		ret = hm5065_write(sensor, HM5065_REG_AF_COMMAND,
				   HM5065_REG_AF_COMMAND_RELEASED_BUTTON);
		if (ret)
			return ret;
#endif
	}

	if (!auto_focus && ctrls->af_start->is_new) {
		ret = hm5065_write(sensor, HM5065_REG_AF_MODE,
				   HM5065_REG_AF_MODE_SINGLE);
		if (ret)
			return ret;

#if 0
		usleep_range(190000, 200000); // 200ms

		ret = hm5065_write(sensor, HM5065_REG_AF_COMMAND,
				   HM5065_REG_AF_COMMAND_RELEASED_BUTTON);
		if (ret)
			return ret;

		usleep_range(190000, 200000); // 200ms
#endif
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
					    1, 20000, 1, 200);
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
					  0, 100, 1, 0);

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
						0, 5000, 1, 1000);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 5000, 1, 1000);

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
	v4l2_ctrl_auto_cluster(6, &ctrls->focus_auto, 0, false);

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

	pix_fmt = hm5065_find_format(sensor->fmt.code);
	if (!pix_fmt) {
		dev_err(&sensor->i2c_client->dev,
			"pixel format not supported %u\n",
			sensor->fmt.code);
		return -EINVAL;
	}

	ret = hm5065_write(sensor, HM5065_REG_P0_DATA_FORMAT,
			   pix_fmt->data_fmt);
	if (ret)
		return ret;

	if (pix_fmt->ycbcr_order != HM5065_REG_YCRCB_ORDER_NONE) {
		ret = hm5065_write(sensor, HM5065_REG_YCRCB_ORDER,
				   pix_fmt->ycbcr_order);
		if (ret)
			return ret;
	}

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

static int hm5065_log_status(struct v4l2_subdev *sd)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	u8 buf[256];
	int ret, i;
	u16 v16;

	if (!sensor->powered)
		return -EIO;

	//ret = hm5065_read_regs(sensor, 0, buf, sizeof(buf));
	//if (ret)
		//return -EIO;

#define DUMP_FP16(name, reg) \
	ret = hm5065_read16(sensor, reg, &v16); \
	if (ret) \
		return -EIO; \
	v4l2_info(sd, #name ": %lld\n", hm5065_mili_from_fp16(v16));

#define DUMP_UI16(name, reg) \
	ret = hm5065_read16(sensor, reg, &v16); \
	if (ret) \
		return -EIO; \
	v4l2_info(sd, #name ": %u\n", v16);

	DUMP_FP16(fpAnalogGainPending, 0x180)
	DUMP_FP16(fpDigitalGainPending, 0x182)
	DUMP_FP16(fpDesiredExposureTime_us, 0x184)
	DUMP_FP16(fpCompiledExposureTime_us, 0x186)
	DUMP_UI16(uwUserMaximumIntegrationLines, 0x189)
	DUMP_FP16(fpTotalIntegrationTimePending_us, 0x18b)
	DUMP_FP16(fpRequestedFrameRate_Hz, 0xd8)
	DUMP_FP16(fpMaxFrameRate_Hz, 0xda)
	DUMP_FP16(fpMinFrameRate_Hz, 0xdc)

	//v4l2_info(sd, "HM5065 registers:\n");
	//for (i = 0; i < sizeof(buf); i++)
		//v4l2_info(sd, "%04x: %02x\n", i, buf[i]);

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
			     hm5065_mili_to_fp16(480000));
	if (ret)
		return ret;

	ret = hm5065_write_list(sensor, ARRAY_SIZE(af_init_regs), af_init_regs);
	if (ret)
		return ret;

	mdelay(200);

	ret = hm5065_write_list(sensor, ARRAY_SIZE(default_regs), default_regs);
	if (ret)
		return ret;

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
