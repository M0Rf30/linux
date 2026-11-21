// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Gianluca Boiano <morf3089@gmail.com>
/*
 * Samsung S5K3T1 (ISOCELL 3T1) image sensor driver.
 *
 * The register settings are derived from the vendor camera module library
 * of the Xiaomi Redmi Note 6 Pro (tulip) front camera (libmmcamera_tulip_
 * s5k3T1_sunny_i.so). The register interface follows the same conventions
 * as the other Samsung CCI sensors (s5k3m5, s5kjn1).
 */

#include <linux/array_size.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/* The PLL settings are only valid for a 24 MHz input clock */
#define S5K3T1_EXTCLK_FREQ		(24 * HZ_PER_MHZ)
#define S5K3T1_DATA_LANES		4

/*
 * Link and pixel clocks, derived from the PLL registers programmed by
 * s5k3t1_mode_regs (EXTCLK = 24 MHz):
 *
 * Video timing (VT) domain:
 *   0x0304 pre_pll_clk_div = 3		24 MHz / 3		=    8 MHz
 *   0x0306 pll_multiplier  = 175	8 MHz * 175		= 1400 MHz
 *   0x0302 vt_sys_clk_div  = 1
 *   0x0300 vt_pix_clk_div  = 7		1400 MHz / 1 / 7	=  200 MHz
 *   The sensor processes four pixels per VT pixel clock:
 *					200 MHz * 4		=  800 MHz
 *   This matches the vendor vt_pixel_clk of 800 MHz and the frame timing:
 *   LLP * FLL * fps = 13056 * 2040 * 30 = 799.0 MHz. It is the rate
 *   V4L2 needs for the PIXEL_RATE control, since LINE_LENGTH_PCK (0x0342)
 *   and FRAME_LENGTH_LINES (0x0340) are expressed in this clock domain.
 *
 * Output (OP) domain:
 *   0x030e secnd_pre_pll_clk_div = 4	24 MHz / 4		=    6 MHz
 *   0x0310 secnd_pll_multiplier = 400	6 MHz * 400		= 2400 MHz
 *   0x0312 op_sys_clk_div = 2		2400 MHz / 2^2		=  600 Mbps/lane
 *   The MIPI D-PHY clock is half of the per lane bit rate, i.e. 300 MHz.
 *   Cross-check with the vendor op_pixel_clk of 240 MHz:
 *					4 lanes * 600 Mbps / 10 bit = 240 Mpix/s
 *
 * The same formula applied to the S5K2L7 vendor tables reproduces its vendor
 * op_pixel_clk values (532.8 MHz and 280.8 MHz) as well.
 */
#define S5K3T1_LINK_FREQ		(300 * HZ_PER_MHZ)
#define S5K3T1_PIXEL_RATE		(800 * HZ_PER_MHZ)

/* Chip ID */
#define S5K3T1_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K3T1_CHIP_ID			0x3141

#define S5K3T1_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K3T1_MODE_STANDBY		0x00
#define S5K3T1_MODE_STREAMING		0x01

/* Image orientation, applied on the next frame start */
#define S5K3T1_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K3T1_HFLIP			BIT(0)
#define S5K3T1_VFLIP			BIT(1)

#define S5K3T1_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K3T1_EXPOSURE_MIN		4
#define S5K3T1_EXPOSURE_STEP		1
#define S5K3T1_EXPOSURE_DEFAULT		1000
/* Vendor vert_offset: coarse integration time <= FLL - 5 */
#define S5K3T1_EXPOSURE_MARGIN		5

/* Analogue gain, 0x20 is 1x. The vendor min_gain is 1.0, max_gain 16.0 */
#define S5K3T1_REG_AGAIN		CCI_REG16(0x0204)
#define S5K3T1_AGAIN_MIN		0x20
#define S5K3T1_AGAIN_MAX		0x200
#define S5K3T1_AGAIN_STEP		1
#define S5K3T1_AGAIN_DEFAULT		0x20

#define S5K3T1_REG_FLL			CCI_REG16(0x0340)
/* Vendor max_linecount */
#define S5K3T1_FLL_MAX			0xfff0

#define S5K3T1_REG_TEST_PATTERN		CCI_REG16(0x0600)

/* Pixel array: the sensor outputs a 2x2 binned image of the whole array */
#define S5K3T1_NATIVE_WIDTH		5184U
#define S5K3T1_NATIVE_HEIGHT		3880U

/* Vendor output info for the only mode */
#define S5K3T1_WIDTH			2592
#define S5K3T1_HEIGHT			1940
#define S5K3T1_LLP			13056
#define S5K3T1_FLL_DEFAULT		2040

/* Delays applied by the vendor driver after register tables, in us */
#define S5K3T1_TABLE_DELAY_US		(10 * USEC_PER_MSEC)
#define S5K3T1_SW_RESET_DELAY_US	(16 * USEC_PER_MSEC)

#define to_s5k3t1(_sd)			container_of_const(_sd, struct s5k3t1, sd)

static const s64 s5k3t1_link_freq_menu[] = {
	S5K3T1_LINK_FREQ,
};

/*
 * Bayer order for the possible orientations, indexed by (vflip << 1) | hflip.
 *
 * The vendor library reports CFA 1 for this sensor and starts streaming with
 * the orientation register cleared. Following the S5K3M5 sibling, whose
 * unflipped output was verified on hardware, the unflipped order is assumed
 * to be GRBG; flipping horizontally swaps the columns within a row, flipping
 * vertically swaps the rows.
 */
static const u32 s5k3t1_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

static const char * const s5k3t1_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
	"PN9",
};

/* Listed in the order of the vendor power-up sequence */
static const char * const s5k3t1_supply_names[] = {
	"vana",		/* Analogue power, 2.8 V */
	"vdig",		/* Digital core power */
	"vio",		/* I/O power, 1.8 V */
};

#define S5K3T1_NUM_SUPPLIES		ARRAY_SIZE(s5k3t1_supply_names)

struct s5k3t1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K3T1_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

/*
 * Register tables, verbatim from the vendor library (tables "init0", "init1"
 * and "res0"). The vendor applies them in this order with the delays defined
 * above.
 */

/* Soft reset, the vendor waits 16 ms after the 0x6010 write */
static const struct cci_reg_sequence s5k3t1_init0_regs[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6010), 0x0001 },
};

/* Firmware patch and sensor tuning, written through the 0x602a/0x6f12 window */
static const struct cci_reg_sequence s5k3t1_init1_regs[] = {
	{ CCI_REG16(0x6214), 0x7971 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x5128 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0449 },
	{ CCI_REG16(0x6f12), 0x0348 },
	{ CCI_REG16(0x6f12), 0x044a },
	{ CCI_REG16(0x6f12), 0x0860 },
	{ CCI_REG16(0x6f12), 0x101a },
	{ CCI_REG16(0x6f12), 0x8880 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xcfb8 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x5404 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4540 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x6a00 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x0546 },
	{ CCI_REG16(0x6f12), 0x754c },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xb021 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xfff8 },
	{ CCI_REG16(0x6f12), 0x208d },
	{ CCI_REG16(0x6f12), 0xe18c },
	{ CCI_REG16(0x6f12), 0x411a },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xfef8 },
	{ CCI_REG16(0x6f12), 0x608d },
	{ CCI_REG16(0x6f12), 0xa189 },
	{ CCI_REG16(0x6f12), 0x411a },
	{ CCI_REG16(0x6f12), 0x6f48 },
	{ CCI_REG16(0x6f12), 0xa0f8 },
	{ CCI_REG16(0x6f12), 0x1a13 },
	{ CCI_REG16(0x6f12), 0xb0f8 },
	{ CCI_REG16(0x6f12), 0x1a13 },
	{ CCI_REG16(0x6f12), 0x0029 },
	{ CCI_REG16(0x6f12), 0xfbd1 },
	{ CCI_REG16(0x6f12), 0x70bd },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf047 },
	{ CCI_REG16(0x6f12), 0x9046 },
	{ CCI_REG16(0x6f12), 0x0e46 },
	{ CCI_REG16(0x6f12), 0x0446 },
	{ CCI_REG16(0x6f12), 0x1ee0 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xeff8 },
	{ CCI_REG16(0x6f12), 0x8146 },
	{ CCI_REG16(0x6f12), 0x7d03 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x05f5 },
	{ CCI_REG16(0x6f12), 0x0055 },
	{ CCI_REG16(0x6f12), 0xfff7 },
	{ CCI_REG16(0x6f12), 0xd8ff },
	{ CCI_REG16(0x6f12), 0x6349 },
	{ CCI_REG16(0x6f12), 0x7020 },
	{ CCI_REG16(0x6f12), 0xa1f8 },
	{ CCI_REG16(0x6f12), 0x0e03 },
	{ CCI_REG16(0x6f12), 0xc4f3 },
	{ CCI_REG16(0x6f12), 0x0c01 },
	{ CCI_REG16(0x6f12), 0xae42 },
	{ CCI_REG16(0x6f12), 0x00d2 },
	{ CCI_REG16(0x6f12), 0x3546 },
	{ CCI_REG16(0x6f12), 0x2d1b },
	{ CCI_REG16(0x6f12), 0x2b46 },
	{ CCI_REG16(0x6f12), 0x4246 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xdef8 },
	{ CCI_REG16(0x6f12), 0x2c44 },
	{ CCI_REG16(0x6f12), 0xa844 },
	{ CCI_REG16(0x6f12), 0x4946 },
	{ CCI_REG16(0x6f12), 0x3846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xddf8 },
	{ CCI_REG16(0x6f12), 0x670b },
	{ CCI_REG16(0x6f12), 0x01d1 },
	{ CCI_REG16(0x6f12), 0xb442 },
	{ CCI_REG16(0x6f12), 0xdcd3 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf087 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x0446 },
	{ CCI_REG16(0x6f12), 0x5648 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x4168 },
	{ CCI_REG16(0x6f12), 0x0d0c },
	{ CCI_REG16(0x6f12), 0x8eb2 },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xd1f8 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xd3f8 },
	{ CCI_REG16(0x6f12), 0x34f8 },
	{ CCI_REG16(0x6f12), 0x660f },
	{ CCI_REG16(0x6f12), 0x3146 },
	{ CCI_REG16(0x6f12), 0xa080 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x7040 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xc4b8 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf041 },
	{ CCI_REG16(0x6f12), 0x0646 },
	{ CCI_REG16(0x6f12), 0x4948 },
	{ CCI_REG16(0x6f12), 0x0d46 },
	{ CCI_REG16(0x6f12), 0x8268 },
	{ CCI_REG16(0x6f12), 0x140c },
	{ CCI_REG16(0x6f12), 0x97b2 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x3946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xb7f8 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xbdf8 },
	{ CCI_REG16(0x6f12), 0x434d },
	{ CCI_REG16(0x6f12), 0x4010 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0xe860 },
	{ CCI_REG16(0x6f12), 0x3946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xabf8 },
	{ CCI_REG16(0x6f12), 0xe868 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xe860 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0xf081 },
	{ CCI_REG16(0x6f12), 0x2de9 },
	{ CCI_REG16(0x6f12), 0xf047 },
	{ CCI_REG16(0x6f12), 0x0446 },
	{ CCI_REG16(0x6f12), 0x3a48 },
	{ CCI_REG16(0x6f12), 0x9146 },
	{ CCI_REG16(0x6f12), 0x8846 },
	{ CCI_REG16(0x6f12), 0xc068 },
	{ CCI_REG16(0x6f12), 0x1e46 },
	{ CCI_REG16(0x6f12), 0x87b2 },
	{ CCI_REG16(0x6f12), 0x050c },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x3946 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x97f8 },
	{ CCI_REG16(0x6f12), 0x3346 },
	{ CCI_REG16(0x6f12), 0x4a46 },
	{ CCI_REG16(0x6f12), 0x4146 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0xa0f8 },
	{ CCI_REG16(0x6f12), 0x0646 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x3946 },
	{ CCI_REG16(0x6f12), 0x2846 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x8bf8 },
	{ CCI_REG16(0x6f12), 0x3048 },
	{ CCI_REG16(0x6f12), 0x90f8 },
	{ CCI_REG16(0x6f12), 0xd100 },
	{ CCI_REG16(0x6f12), 0x0128 },
	{ CCI_REG16(0x6f12), 0x04d1 },
	{ CCI_REG16(0x6f12), 0x2068 },
	{ CCI_REG16(0x6f12), 0xc107 },
	{ CCI_REG16(0x6f12), 0x01d0 },
	{ CCI_REG16(0x6f12), 0x401c },
	{ CCI_REG16(0x6f12), 0x2060 },
	{ CCI_REG16(0x6f12), 0x2c48 },
	{ CCI_REG16(0x6f12), 0x6188 },
	{ CCI_REG16(0x6f12), 0xa0f8 },
	{ CCI_REG16(0x6f12), 0x4013 },
	{ CCI_REG16(0x6f12), 0x2168 },
	{ CCI_REG16(0x6f12), 0x090c },
	{ CCI_REG16(0x6f12), 0xa0f8 },
	{ CCI_REG16(0x6f12), 0x4213 },
	{ CCI_REG16(0x6f12), 0x3046 },
	{ CCI_REG16(0x6f12), 0x98e7 },
	{ CCI_REG16(0x6f12), 0x70b5 },
	{ CCI_REG16(0x6f12), 0x2848 },
	{ CCI_REG16(0x6f12), 0x0088 },
	{ CCI_REG16(0x6f12), 0x40b1 },
	{ CCI_REG16(0x6f12), 0x2449 },
	{ CCI_REG16(0x6f12), 0x096e },
	{ CCI_REG16(0x6f12), 0x4843 },
	{ CCI_REG16(0x6f12), 0x4ff4 },
	{ CCI_REG16(0x6f12), 0x7a71 },
	{ CCI_REG16(0x6f12), 0xb0fb },
	{ CCI_REG16(0x6f12), 0xf1f0 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x7ef8 },
	{ CCI_REG16(0x6f12), 0x1e48 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0x0169 },
	{ CCI_REG16(0x6f12), 0x0c0c },
	{ CCI_REG16(0x6f12), 0x8db2 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x61f8 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x78f8 },
	{ CCI_REG16(0x6f12), 0x2946 },
	{ CCI_REG16(0x6f12), 0x2046 },
	{ CCI_REG16(0x6f12), 0xbde8 },
	{ CCI_REG16(0x6f12), 0x7040 },
	{ CCI_REG16(0x6f12), 0x0122 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x58b8 },
	{ CCI_REG16(0x6f12), 0x10b5 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x6311 },
	{ CCI_REG16(0x6f12), 0x1848 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x6ff8 },
	{ CCI_REG16(0x6f12), 0x114c },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x1d11 },
	{ CCI_REG16(0x6f12), 0x2060 },
	{ CCI_REG16(0x6f12), 0x1548 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x67f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xf901 },
	{ CCI_REG16(0x6f12), 0x6060 },
	{ CCI_REG16(0x6f12), 0x1248 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x60f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0xcd01 },
	{ CCI_REG16(0x6f12), 0xa060 },
	{ CCI_REG16(0x6f12), 0x1048 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x59f8 },
	{ CCI_REG16(0x6f12), 0x0022 },
	{ CCI_REG16(0x6f12), 0xaff2 },
	{ CCI_REG16(0x6f12), 0x7b01 },
	{ CCI_REG16(0x6f12), 0xe060 },
	{ CCI_REG16(0x6f12), 0x0d48 },
	{ CCI_REG16(0x6f12), 0x00f0 },
	{ CCI_REG16(0x6f12), 0x52f8 },
	{ CCI_REG16(0x6f12), 0x2061 },
	{ CCI_REG16(0x6f12), 0x10bd },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4f80 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x8000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x53f0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x42b0 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x4580 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0xc000 },
	{ CCI_REG16(0x6f12), 0x2000 },
	{ CCI_REG16(0x6f12), 0x6a00 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xf66f },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x8f23 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x72f5 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0xdb2d },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x6de3 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x992c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x2d3c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x1b2c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x554c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4ff2 },
	{ CCI_REG16(0x6f12), 0x8f2c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x40f6 },
	{ CCI_REG16(0x6f12), 0xe71c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x48f6 },
	{ CCI_REG16(0x6f12), 0x237c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x47f2 },
	{ CCI_REG16(0x6f12), 0xf52c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4df6 },
	{ CCI_REG16(0x6f12), 0x2d3c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x4cf6 },
	{ CCI_REG16(0x6f12), 0x114c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x46f6 },
	{ CCI_REG16(0x6f12), 0xe35c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x43f6 },
	{ CCI_REG16(0x6f12), 0x756c },
	{ CCI_REG16(0x6f12), 0xc0f2 },
	{ CCI_REG16(0x6f12), 0x010c },
	{ CCI_REG16(0x6f12), 0x6047 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x5122 },
	{ CCI_REG16(0x6f12), 0x0002 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x5112 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x6a00 },
	{ CCI_REG16(0x6f12), 0x0f00 },
	{ CCI_REG16(0x602a), 0x1d66 },
	{ CCI_REG16(0x6f12), 0xad10 },
	{ CCI_REG16(0x602a), 0x1d60 },
	{ CCI_REG16(0x6f12), 0x006f },
	{ CCI_REG16(0x602a), 0x1d2e },
	{ CCI_REG16(0x6f12), 0x1310 },
	{ CCI_REG16(0x602a), 0x1d32 },
	{ CCI_REG16(0x6f12), 0x0605 },
	{ CCI_REG16(0x602a), 0x1d36 },
	{ CCI_REG16(0x6f12), 0x0305 },
	{ CCI_REG16(0x602a), 0x1d3a },
	{ CCI_REG16(0x6f12), 0x0305 },
	{ CCI_REG16(0x602a), 0x1d52 },
	{ CCI_REG16(0x6f12), 0x6819 },
	{ CCI_REG16(0x602a), 0x1d56 },
	{ CCI_REG16(0x6f12), 0x01ff },
	{ CCI_REG16(0xf262), 0x0001 },
	{ CCI_REG16(0xf404), 0x0fff },
	{ CCI_REG16(0xf42a), 0x0080 },
	{ CCI_REG16(0xf430), 0x84a1 },
	{ CCI_REG16(0xf432), 0x7542 },
	{ CCI_REG16(0xf464), 0x000e },
	{ CCI_REG16(0xf466), 0x000e },
	{ CCI_REG16(0xf468), 0x0018 },
	{ CCI_REG16(0xf46a), 0x0010 },
	{ CCI_REG16(0xf474), 0x0014 },
	{ CCI_REG16(0xf476), 0x000a },
	{ CCI_REG16(0xf478), 0x0000 },
	{ CCI_REG16(0xf484), 0x0000 },
	{ CCI_REG16(0xf4b6), 0x003c },
	{ CCI_REG16(0xf4b8), 0x0044 },
	{ CCI_REG16(0xf4be), 0x0fac },
	{ CCI_REG16(0xf4c0), 0x0fbc },
	{ CCI_REG16(0xf4c2), 0x0fad },
	{ CCI_REG16(0xf4c4), 0x0fbd },
	{ CCI_REG16(0xf4c6), 0x0fb4 },
	{ CCI_REG16(0xf4c8), 0x0fc4 },
	{ CCI_REG16(0xf4ca), 0x0fb5 },
	{ CCI_REG16(0xf4cc), 0x0fc5 },
	{ CCI_REG16(0xf48e), 0x0200 },
	{ CCI_REG16(0xf42e), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x106e },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x1086 },
	{ CCI_REG16(0x6f12), 0x0005 },
	{ CCI_REG16(0x602a), 0x10e6 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x10ee },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x10f6 },
	{ CCI_REG16(0x6f12), 0x0006 },
	{ CCI_REG16(0x602a), 0x110e },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x113e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x126e },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x602a), 0x1276 },
	{ CCI_REG16(0x6f12), 0x0032 },
	{ CCI_REG16(0x602a), 0x13d6 },
	{ CCI_REG16(0x6f12), 0x0017 },
	{ CCI_REG16(0x602a), 0x15ee },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1068 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x1070 },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x602a), 0x1080 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x1088 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x1090 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x10e8 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x10f0 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x10f8 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x1108 },
	{ CCI_REG16(0x6f12), 0x0129 },
	{ CCI_REG16(0x602a), 0x1110 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x1140 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x1148 },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x1150 },
	{ CCI_REG16(0x6f12), 0x000f },
	{ CCI_REG16(0x602a), 0x1160 },
	{ CCI_REG16(0x6f12), 0x001e },
	{ CCI_REG16(0x602a), 0x1268 },
	{ CCI_REG16(0x6f12), 0x0007 },
	{ CCI_REG16(0x602a), 0x1270 },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x602a), 0x1278 },
	{ CCI_REG16(0x6f12), 0x0032 },
	{ CCI_REG16(0x602a), 0x13d0 },
	{ CCI_REG16(0x6f12), 0x0014 },
	{ CCI_REG16(0x602a), 0x13d8 },
	{ CCI_REG16(0x6f12), 0x008c },
	{ CCI_REG16(0x602a), 0x13e0 },
	{ CCI_REG16(0x6f12), 0x008c },
	{ CCI_REG16(0x602a), 0x1d08 },
	{ CCI_REG16(0x6f12), 0x0330 },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x0202), 0x0200 },
	{ CCI_REG16(0x0226), 0x0400 },
	{ CCI_REG16(0x0704), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0fec },
	{ CCI_REG16(0x6f12), 0x2020 },
	{ CCI_REG16(0x602a), 0x1000 },
	{ CCI_REG16(0x6f12), 0x14b0 },
	{ CCI_REG16(0x6f12), 0x0058 },
	{ CCI_REG16(0x6f12), 0x1008 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x100e },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0fe6 },
	{ CCI_REG16(0x6f12), 0x0006 },
	{ CCI_REG16(0x602a), 0x0fe4 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x0fe2 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x602a), 0x0ff8 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x0ffe },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x0fce },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x2f88 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2e6c },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x2f90 },
	{ CCI_REG16(0x6f12), 0x2001 },
	{ CCI_REG16(0x602a), 0x2f8c },
	{ CCI_REG16(0x6f12), 0x8010 },
	{ CCI_REG16(0x602a), 0x2070 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2c02 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x602a), 0x2b0c },
	{ CCI_REG16(0x6f12), 0x0030 },
	{ CCI_REG16(0x6f12), 0x0081 },
	{ CCI_REG16(0x602a), 0x2b78 },
	{ CCI_REG16(0x6f12), 0x02c0 },
	{ CCI_REG16(0x6f12), 0xc0c0 },
	{ CCI_REG16(0x602a), 0x2b4a },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x2bfc },
	{ CCI_REG16(0x6f12), 0x0d00 },
	{ CCI_REG16(0x0bc0), 0x0040 },
	{ CCI_REG16(0x0bce), 0x0040 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x00ae },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x005c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x0b06), 0x0100 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x00b4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x03ff },
	{ CCI_REG16(0x602a), 0x0060 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x05b4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x602a), 0x4226 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x00e0 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x0108 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x6f12), 0x4000 },
	{ CCI_REG16(0x6f12), 0x8000 },
	{ CCI_REG16(0x6f12), 0xc000 },
	{ CCI_REG16(0x6f12), 0xffff },
	{ CCI_REG16(0x602a), 0x036a },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0398 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x03c6 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x03f4 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0422 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x0fe8), 0x0b41 },
	{ CCI_REG16(0x3036), 0x0000 },
	{ CCI_REG16(0x0d00), 0x0001 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x3000 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x39c4 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0fda },
	{ CCI_REG16(0x6f12), 0x0006 },
	{ CCI_REG16(0x3070), 0x0300 },
	{ CCI_REG16(0x306e), 0x0004 },
	{ CCI_REG16(0xb13c), 0x0c80 },
	{ CCI_REG16(0x6592), 0x0010 },
	{ CCI_REG16(0x3026), 0x0001 },
	{ CCI_REG16(0x602a), 0x2bee },
	{ CCI_REG16(0x6f12), 0x143c },
	{ CCI_REG16(0x602a), 0x2bf2 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
};

/* 2592x1940 mode: 2x2 binned full array, PLL, 4 lanes and timing */
static const struct cci_reg_sequence s5k3t1_mode_regs[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0136), 0x1800 },
	{ CCI_REG16(0x0304), 0x0003 },
	{ CCI_REG16(0x0306), 0x00af },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0300), 0x0007 },
	{ CCI_REG16(0x030e), 0x0004 },
	{ CCI_REG16(0x0310), 0x0190 },
	{ CCI_REG16(0x030c), 0x0000 },
	{ CCI_REG16(0x0312), 0x0002 },
	{ CCI_REG16(0x3004), 0x0005 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x1d50 },
	{ CCI_REG16(0x6f12), 0x0096 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x0000 },
	{ CCI_REG16(0x0348), 0x143f },
	{ CCI_REG16(0x034a), 0x0f27 },
	{ CCI_REG16(0x034c), 0x0a20 },
	{ CCI_REG16(0x034e), 0x0794 },
	{ CCI_REG16(0x0350), 0x0000 },
	{ CCI_REG16(0x0352), 0x0000 },
	{ CCI_REG16(0x0342), 0x3300 },
	{ CCI_REG16(0x0340), 0x07f8 },
	{ CCI_REG16(0x0702), 0x0000 },
	{ CCI_REG16(0x0900), 0x0111 },
	{ CCI_REG16(0x0380), 0x0002 },
	{ CCI_REG16(0x0382), 0x0002 },
	{ CCI_REG16(0x0384), 0x0002 },
	{ CCI_REG16(0x0386), 0x0002 },
	{ CCI_REG16(0x0bc6), 0x0000 },
	{ CCI_REG16(0x0400), 0x1010 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x1056 },
	{ CCI_REG16(0x6f12), 0x0300 },
	{ CCI_REG16(0x602a), 0x0fc0 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x0a58 },
	{ CCI_REG16(0x6f12), 0x0804 },
	{ CCI_REG16(0x6f12), 0x1440 },
	{ CCI_REG16(0x6f12), 0x0f28 },
	{ CCI_REG16(0x602a), 0x2bfc },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0010 },
	{ CCI_REG16(0x6f12), 0x1000 },
	{ CCI_REG16(0x6f12), 0x1010 },
	{ CCI_REG16(0x3030), 0x0000 },
	{ CCI_REG16(0x3034), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0fa0 },
	{ CCI_REG16(0x6f12), 0xffff },
	{ CCI_REG16(0x303c), 0x0000 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x5110 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x5116 },
	{ CCI_REG16(0x6f12), 0x0048 },
	{ CCI_REG16(0x6f12), 0x0048 },
	{ CCI_REG16(0x6f12), 0x0048 },
	{ CCI_REG16(0x602a), 0x1d16 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1d10 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1d54 },
	{ CCI_REG16(0x6f12), 0x00bf },
	{ CCI_REG16(0x602a), 0x3900 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x3904 },
	{ CCI_REG16(0x6f12), 0x005f },
	{ CCI_REG16(0x602a), 0x390a },
	{ CCI_REG16(0x6f12), 0x005f },
	{ CCI_REG16(0x602a), 0x3910 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0x0020 },
	{ CCI_REG16(0x6f12), 0xf42e },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0xf482), 0x01a4 },
};

static int s5k3t1_write_table(struct s5k3t1 *s5k3t1,
			      const struct cci_reg_sequence *regs,
			      unsigned int num_regs, unsigned int delay_us)
{
	int ret;

	ret = cci_multi_reg_write(s5k3t1->regmap, regs, num_regs, NULL);
	if (ret)
		return ret;

	fsleep(delay_us);

	return 0;
}

static u32 s5k3t1_get_format_code(struct s5k3t1 *s5k3t1)
{
	unsigned int i;

	i = (s5k3t1->vflip->val ? 2 : 0) | (s5k3t1->hflip->val ? 1 : 0);

	return s5k3t1_mbus_formats[i];
}

static void s5k3t1_update_pad_format(struct s5k3t1 *s5k3t1,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k3t1_get_format_code(s5k3t1);
	fmt->width = S5K3T1_WIDTH;
	fmt->height = S5K3T1_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k3t1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k3t1 *s5k3t1 =
		container_of(ctrl->handler, struct s5k3t1, ctrl_handler);
	struct v4l2_subdev_state *state =
		v4l2_subdev_get_locked_active_state(&s5k3t1->sd);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK: {
		/* Honour the VBLANK limits when setting exposure */
		s64 exposure_max = S5K3T1_HEIGHT + ctrl->val -
				   S5K3T1_EXPOSURE_MARGIN;

		__v4l2_ctrl_modify_range(s5k3t1->exposure,
					 s5k3t1->exposure->minimum,
					 exposure_max, s5k3t1->exposure->step,
					 min_t(s64, exposure_max,
					       s5k3t1->exposure->val));
		break;
	}
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* The Bayer order depends on the flips, keep the format in sync */
		v4l2_subdev_state_get_format(state, 0)->code =
			s5k3t1_get_format_code(s5k3t1);
		break;
	}

	/* Controls are only written to the sensor while it is powered */
	if (pm_runtime_get_if_in_use(s5k3t1->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k3t1->regmap, S5K3T1_REG_AGAIN, ctrl->val,
				NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k3t1->regmap, S5K3T1_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k3t1->regmap, S5K3T1_REG_FLL,
				S5K3T1_HEIGHT + ctrl->val, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(s5k3t1->regmap, S5K3T1_REG_ORIENTATION,
				(s5k3t1->hflip->val ? S5K3T1_HFLIP : 0) |
				(s5k3t1->vflip->val ? S5K3T1_VFLIP : 0), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k3t1->regmap, S5K3T1_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_HBLANK:
		/* Read-only controls, nothing to write */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k3t1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k3t1_ctrl_ops = {
	.s_ctrl = s5k3t1_set_ctrl,
};

static int s5k3t1_init_controls(struct s5k3t1 *s5k3t1)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k3t1->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	s64 hblank, vblank_def;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 12);

	s5k3t1->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						   &s5k3t1_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(s5k3t1_link_freq_menu) - 1,
						   0, s5k3t1_link_freq_menu);
	if (s5k3t1->link_freq)
		s5k3t1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k3t1->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       S5K3T1_PIXEL_RATE,
					       S5K3T1_PIXEL_RATE, 1,
					       S5K3T1_PIXEL_RATE);

	hblank = S5K3T1_LLP - S5K3T1_WIDTH;
	s5k3t1->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (s5k3t1->hblank)
		s5k3t1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = S5K3T1_FLL_DEFAULT - S5K3T1_HEIGHT;
	s5k3t1->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   S5K3T1_FLL_MAX - S5K3T1_HEIGHT, 1,
					   vblank_def);

	s5k3t1->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K3T1_EXPOSURE_MIN,
					     S5K3T1_FLL_DEFAULT -
					     S5K3T1_EXPOSURE_MARGIN,
					     S5K3T1_EXPOSURE_STEP,
					     S5K3T1_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K3T1_AGAIN_MIN, S5K3T1_AGAIN_MAX,
			  S5K3T1_AGAIN_STEP, S5K3T1_AGAIN_DEFAULT);

	s5k3t1->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k3t1->hflip)
		s5k3t1->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k3t1->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3t1_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k3t1->vflip)
		s5k3t1->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k3t1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k3t1_test_pattern_menu) - 1,
				     0, 0, s5k3t1_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	ret = v4l2_fwnode_device_parse(s5k3t1->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k3t1_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	s5k3t1->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k3t1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);
	int ret;

	ret = pm_runtime_resume_and_get(s5k3t1->dev);
	if (ret)
		return ret;

	/* Software reset and wait for the sensor to come back */
	ret = s5k3t1_write_table(s5k3t1, s5k3t1_init0_regs,
				 ARRAY_SIZE(s5k3t1_init0_regs),
				 S5K3T1_SW_RESET_DELAY_US +
				 S5K3T1_TABLE_DELAY_US);
	if (ret)
		goto err_rpm_put;

	ret = s5k3t1_write_table(s5k3t1, s5k3t1_init1_regs,
				 ARRAY_SIZE(s5k3t1_init1_regs),
				 S5K3T1_TABLE_DELAY_US);
	if (ret)
		goto err_rpm_put;

	ret = s5k3t1_write_table(s5k3t1, s5k3t1_mode_regs,
				 ARRAY_SIZE(s5k3t1_mode_regs),
				 S5K3T1_TABLE_DELAY_US);
	if (ret)
		goto err_rpm_put;

	/* Apply the user controls, including the orientation */
	ret = __v4l2_ctrl_handler_setup(s5k3t1->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(s5k3t1->regmap, S5K3T1_REG_MODE_SELECT,
			S5K3T1_MODE_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;

	/* The Bayer order cannot change during streaming */
	__v4l2_ctrl_grab(s5k3t1->hflip, true);
	__v4l2_ctrl_grab(s5k3t1->vflip, true);

	return 0;

err_rpm_put:
	dev_err(s5k3t1->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put(s5k3t1->dev);

	return ret;
}

static int s5k3t1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);
	int ret;

	ret = cci_write(s5k3t1->regmap, S5K3T1_REG_MODE_SELECT,
			S5K3T1_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5k3t1->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(s5k3t1->hflip, false);
	__v4l2_ctrl_grab(s5k3t1->vflip, false);

	pm_runtime_put(s5k3t1->dev);

	return ret;
}

static int s5k3t1_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);

	/* The format cannot be changed while streaming */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	/* Only one mode is supported, the requested size is ignored */
	s5k3t1_update_pad_format(s5k3t1, &fmt->format);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;

	return 0;
}

static int s5k3t1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);

	/* One Bayer format, its order depends on the flip controls */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k3t1_get_format_code(s5k3t1);

	return 0;
}

static int s5k3t1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);

	if (fse->index > 0)
		return -EINVAL;

	if (fse->code != s5k3t1_get_format_code(s5k3t1))
		return -EINVAL;

	fse->min_width = S5K3T1_WIDTH;
	fse->max_width = S5K3T1_WIDTH;
	fse->min_height = S5K3T1_HEIGHT;
	fse->max_height = S5K3T1_HEIGHT;

	return 0;
}

static int s5k3t1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		/* The mode is a 2x2 binning of the whole pixel array */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K3T1_NATIVE_WIDTH;
		sel->r.height = S5K3T1_NATIVE_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int s5k3t1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	s5k3t1_update_pad_format(to_s5k3t1(sd),
				 v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops s5k3t1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k3t1_pad_ops = {
	.enum_mbus_code = s5k3t1_enum_mbus_code,
	.enum_frame_size = s5k3t1_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k3t1_set_pad_format,
	.get_selection = s5k3t1_get_selection,
	.enable_streams = s5k3t1_enable_streams,
	.disable_streams = s5k3t1_disable_streams,
};

static const struct v4l2_subdev_ops s5k3t1_subdev_ops = {
	.video = &s5k3t1_video_ops,
	.pad = &s5k3t1_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k3t1_internal_ops = {
	.init_state = s5k3t1_init_state,
};

static const struct media_entity_operations s5k3t1_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k3t1_identify_sensor(struct s5k3t1 *s5k3t1)
{
	u64 val;
	int ret;

	ret = cci_read(s5k3t1->regmap, S5K3T1_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(s5k3t1->dev, ret,
				     "failed to read chip id\n");

	if (val != S5K3T1_CHIP_ID)
		return dev_err_probe(s5k3t1->dev, -ENODEV,
				     "chip id mismatch: %x != %llx\n",
				     S5K3T1_CHIP_ID, val);

	return 0;
}

/*
 * Check that the endpoint describes the only supported configuration: four
 * data lanes and a 300 MHz link frequency.
 */
static int s5k3t1_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint node failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K3T1_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL, "invalid data lanes: %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out_free;
	}

	ret = v4l2_link_freq_to_bitmap(dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k3t1_link_freq_menu,
				       ARRAY_SIZE(s5k3t1_link_freq_menu),
				       &freq_bitmap);

out_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* Disable the first @count supplies in reverse order of the power-up */
static void s5k3t1_disable_supplies(struct s5k3t1 *s5k3t1, unsigned int count)
{
	while (count--) {
		regulator_disable(s5k3t1->supplies[count].consumer);
		usleep_range(1 * USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}
}

/*
 * Vendor power-up sequence: reset asserted, VANA, VDIG, VIO (1 ms each),
 * reset released (5 ms), then the 24 MHz clock (10 ms).
 */
static int s5k3t1_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(s5k3t1->reset_gpio, 1);

	/* The rails have to be enabled one after the other, in order */
	for (i = 0; i < S5K3T1_NUM_SUPPLIES; i++) {
		ret = regulator_enable(s5k3t1->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s supply: %d\n",
				s5k3t1_supply_names[i], ret);
			goto err_disable_supplies;
		}
		usleep_range(1 * USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}

	gpiod_set_value_cansleep(s5k3t1->reset_gpio, 0);
	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	ret = clk_prepare_enable(s5k3t1->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto err_reset;
	}
	usleep_range(10 * USEC_PER_MSEC, 11 * USEC_PER_MSEC);

	return 0;

err_reset:
	gpiod_set_value_cansleep(s5k3t1->reset_gpio, 1);
err_disable_supplies:
	s5k3t1_disable_supplies(s5k3t1, i);

	return ret;
}

/*
 * Vendor power-down sequence: reset asserted (1 ms), clock stopped (1 ms),
 * then VIO, VDIG and VANA (1 ms each).
 */
static int s5k3t1_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);

	gpiod_set_value_cansleep(s5k3t1->reset_gpio, 1);
	usleep_range(1 * USEC_PER_MSEC, 2 * USEC_PER_MSEC);

	clk_disable_unprepare(s5k3t1->clk);
	usleep_range(1 * USEC_PER_MSEC, 2 * USEC_PER_MSEC);

	s5k3t1_disable_supplies(s5k3t1, S5K3T1_NUM_SUPPLIES);

	return 0;
}

static int s5k3t1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k3t1 *s5k3t1;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5k3t1 = devm_kzalloc(dev, sizeof(*s5k3t1), GFP_KERNEL);
	if (!s5k3t1)
		return -ENOMEM;

	s5k3t1->dev = dev;
	v4l2_i2c_subdev_init(&s5k3t1->sd, client, &s5k3t1_subdev_ops);

	s5k3t1->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k3t1->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k3t1->regmap),
				     "failed to initialize CCI\n");

	for (i = 0; i < S5K3T1_NUM_SUPPLIES; i++)
		s5k3t1->supplies[i].supply = s5k3t1_supply_names[i];

	ret = devm_regulator_bulk_get(dev, S5K3T1_NUM_SUPPLIES,
				      s5k3t1->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	s5k3t1->clk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(s5k3t1->clk))
		return dev_err_probe(dev, PTR_ERR(s5k3t1->clk),
				     "failed to get clock\n");

	freq = clk_get_rate(s5k3t1->clk);
	if (freq != S5K3T1_EXTCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "clock frequency %lu Hz is not supported, must be %lu Hz\n",
				     freq, S5K3T1_EXTCLK_FREQ);

	ret = s5k3t1_check_hwcfg(dev);
	if (ret)
		return ret;

	s5k3t1->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k3t1->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k3t1->reset_gpio),
				     "failed to get reset gpio\n");

	/* The sensor must be powered on to read the chip ID */
	ret = s5k3t1_power_on(dev);
	if (ret)
		return ret;

	ret = s5k3t1_identify_sensor(s5k3t1);
	if (ret)
		goto err_power_off;

	ret = s5k3t1_init_controls(s5k3t1);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init controls\n");
		goto err_power_off;
	}

	s5k3t1->sd.state_lock = s5k3t1->ctrl_handler.lock;
	s5k3t1->sd.internal_ops = &s5k3t1_internal_ops;
	s5k3t1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k3t1->sd.entity.ops = &s5k3t1_subdev_entity_ops;
	s5k3t1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k3t1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k3t1->sd.entity, 1, &s5k3t1->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init media entity pads\n");
		goto err_free_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&s5k3t1->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto err_media_entity_cleanup;
	}

	/*
	 * The device is powered on at this point. Enable runtime PM before
	 * registering the subdev, so that it can be used as soon as it is
	 * visible.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k3t1->sd);
	if (ret) {
		dev_err_probe(dev, ret,
			      "failed to register sensor sub-device\n");
		goto err_subdev_cleanup;
	}

	pm_runtime_idle(dev);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&s5k3t1->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

err_media_entity_cleanup:
	media_entity_cleanup(&s5k3t1->sd.entity);

err_free_ctrls:
	v4l2_ctrl_handler_free(s5k3t1->sd.ctrl_handler);

err_power_off:
	s5k3t1_power_off(dev);

	return ret;
}

static void s5k3t1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3t1 *s5k3t1 = to_s5k3t1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(s5k3t1->dev);
	if (!pm_runtime_status_suspended(s5k3t1->dev)) {
		s5k3t1_power_off(s5k3t1->dev);
		pm_runtime_set_suspended(s5k3t1->dev);
	}
}

static const struct dev_pm_ops s5k3t1_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k3t1_power_off, s5k3t1_power_on, NULL)
};

static const struct of_device_id s5k3t1_of_match[] = {
	{ .compatible = "samsung,s5k3t1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k3t1_of_match);

static struct i2c_driver s5k3t1_i2c_driver = {
	.driver = {
		.name = "s5k3t1",
		.pm = &s5k3t1_pm_ops,
		.of_match_table = s5k3t1_of_match,
	},
	.probe = s5k3t1_probe,
	.remove = s5k3t1_remove,
};

module_i2c_driver(s5k3t1_i2c_driver);

MODULE_AUTHOR("Gianluca Boiano <morf3089@gmail.com>");
MODULE_DESCRIPTION("Samsung S5K3T1 image sensor driver");
MODULE_LICENSE("GPL");
