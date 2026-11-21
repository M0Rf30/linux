// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Gianluca Boiano <morf3089@gmail.com>
/*
 * Samsung S5K2L7 image sensor driver.
 *
 * The register settings are derived from the vendor camera module library
 * (libmmcamera_tulip_s5k2l7_*) shipped with the Xiaomi Redmi Note 6 Pro
 * (tulip). The structure follows the imx376 and s5k3m5 drivers.
 *
 * Clock tree (24 MHz EXTCLK), derived from the PLL registers of the mode
 * tables and cross-checked with the vendor out_info (op_pixel_clk, LLP, FLL
 * and frame rate):
 *
 * Video timing (both modes)
 *   0x0304 pre_pll_clk_div = 6, 0x0306 pll_multiplier = 480,
 *   0x0302 vt_sys_clk_div = 1, 0x0300 vt_pix_clk_div = 4
 *     24 MHz / 6 * 480 = 1920 MHz, / 1 = 1920 MHz, / 4 = 480 MHz,
 *     times two pixels per clock = 960 Mpixel/s.
 *   LLP * FLL * fps = 10160 * 3162 * 29.88 = 959.9 M (mode 0)
 *                   = 10160 * 3149 * 30.00 = 959.8 M (mode 3)
 *   so the pixel rate used by the line/frame timing is 960 MHz.
 *
 * Output (CSI-2 link)
 *   0x030e = 4 (pre divider), 0x0310 = multiplier, 0x0312 = post divider
 *   (power of two), 4 lanes, RAW10:
 *   mode 0 (0x0310 = 222, 0x0312 = 0):
 *     24 MHz / 4 * 222 = 1332 Mbit/s per lane, DDR clock 666 MHz
 *     vendor op_pixel_clk = 1332 Mbit/s * 4 lanes / 10 bpp = 532.8 MHz
 *   mode 3 (0x0310 = 234, 0x0312 = 1):
 *     24 MHz / 4 * 234 / 2 = 702 Mbit/s per lane, DDR clock 351 MHz
 *     vendor op_pixel_clk = 702 Mbit/s * 4 lanes / 10 bpp = 280.8 MHz
 *
 * The link frequency therefore depends on the mode.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
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

/* The PLL settings of the register tables are only valid for 24 MHz */
#define S5K2L7_EXTCLK_FREQ		(24 * HZ_PER_MHZ)
#define S5K2L7_DATA_LANES		4

#define S5K2L7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K2L7_CHIP_ID			0x20c7

#define S5K2L7_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K2L7_MODE_STANDBY		0x00
#define S5K2L7_MODE_STREAMING		0x01

#define S5K2L7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K2L7_ORIENTATION_HFLIP	BIT(0)
#define S5K2L7_ORIENTATION_VFLIP	BIT(1)

#define S5K2L7_REG_GROUP_HOLD		CCI_REG8(0x0104)

/* Frame length in lines, vertical blanking = FLL - output height */
#define S5K2L7_REG_FLL			CCI_REG16(0x0340)
#define S5K2L7_FLL_MAX			0x7fff

/* Line length in pixel clocks, fixed by the mode tables */
#define S5K2L7_LLP			10160

/* The coarse integration time must end S5K2L7_EXPOSURE_MARGIN lines early */
#define S5K2L7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K2L7_EXPOSURE_MIN		8
#define S5K2L7_EXPOSURE_STEP		1
#define S5K2L7_EXPOSURE_MARGIN		8
#define S5K2L7_EXPOSURE_DEFAULT		1000

/* Analogue gain, 0x20 is 1x, up to 16x */
#define S5K2L7_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define S5K2L7_ANA_GAIN_MIN		0x20
#define S5K2L7_ANA_GAIN_MAX		0x200
#define S5K2L7_ANA_GAIN_STEP		1
#define S5K2L7_ANA_GAIN_DEFAULT		0x20

/* Digital gain, 8.8 fixed point per colour channel, up to 2x */
#define S5K2L7_REG_DIG_GAIN_GR		CCI_REG16(0x020e)
#define S5K2L7_REG_DIG_GAIN_R		CCI_REG16(0x0210)
#define S5K2L7_REG_DIG_GAIN_B		CCI_REG16(0x0212)
#define S5K2L7_REG_DIG_GAIN_GB		CCI_REG16(0x0214)
#define S5K2L7_DIG_GAIN_MIN		0x0100
#define S5K2L7_DIG_GAIN_MAX		0x0200
#define S5K2L7_DIG_GAIN_STEP		1
#define S5K2L7_DIG_GAIN_DEFAULT		0x0100

#define S5K2L7_REG_TEST_PATTERN		CCI_REG16(0x0600)

/*
 * The sensor array addresses (X/Y_ADDR_END = 8127/3039 in the mode tables)
 * count the two photodiodes of each dual pixel horizontally, which gives a
 * 4064x3040 pixel array. Both modes crop 16 columns and 8 rows on each side.
 */
#define S5K2L7_NATIVE_WIDTH		4064U
#define S5K2L7_NATIVE_HEIGHT		3040U
#define S5K2L7_CROP_LEFT		16U
#define S5K2L7_CROP_TOP			8U
#define S5K2L7_CROP_WIDTH		4032U
#define S5K2L7_CROP_HEIGHT		3024U

/* Video timing pixel rate, see the clock tree description above */
#define S5K2L7_PIXEL_RATE		(960 * HZ_PER_MHZ)

struct s5k2l7_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k2l7_mode {
	u32 width;
	u32 height;

	/* Frame length in lines, the vendor maximum frame rate */
	u32 fll;

	/* Index in s5k2l7_link_freq_menu */
	unsigned int link_freq_index;

	struct s5k2l7_reg_list reg_list;
};

/* Mode independent register settings, loaded after every power up */
static const struct cci_reg_sequence s5k2l7_init0_regs[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x0000), 0x003a },
	{ CCI_REG16(0x0000), 0x20c7 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0xbbf4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6010), 0x0001 },
};

static const struct cci_reg_sequence s5k2l7_init1_regs[] = {
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x0a02), 0x009a },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf466), 0x000c },
	{ CCI_REG16(0xf468), 0x000d },
	{ CCI_REG16(0xf488), 0x0008 },
	{ CCI_REG16(0xf414), 0x0007 },
	{ CCI_REG16(0xf416), 0x0004 },
	{ CCI_REG16(0x30c6), 0x0100 },
	{ CCI_REG16(0x30ca), 0x0300 },
	{ CCI_REG16(0x30c8), 0x05dc },
	{ CCI_REG16(0x6b36), 0x5200 },
	{ CCI_REG16(0x6b38), 0x0000 },
	{ CCI_REG16(0x0b04), 0x0101 },
	{ CCI_REG16(0x3094), 0x2800 },
	{ CCI_REG16(0x3096), 0x5400 },
	{ CCI_REG16(0x303e), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x16c0 },
	{ CCI_REG16(0x6f12), 0x0046 },
	{ CCI_REG16(0x6f12), 0x0046 },
	{ CCI_REG16(0x602a), 0x09e4 },
	{ CCI_REG16(0x6f12), 0xf484 },
	{ CCI_REG16(0x602a), 0x16ba },
	{ CCI_REG16(0x6f12), 0x2608 },
	{ CCI_REG16(0x602a), 0x16be },
	{ CCI_REG16(0x6f12), 0x2608 },
	{ CCI_REG16(0x602a), 0x15f4 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1282 },
	{ CCI_REG16(0x6f12), 0x0013 },
	{ CCI_REG16(0x602a), 0x127a },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x14c8 },
	{ CCI_REG16(0x6f12), 0x00be },
	{ CCI_REG16(0x602a), 0x15ee },
	{ CCI_REG16(0x6f12), 0x0107 },
	{ CCI_REG16(0x602a), 0x4c64 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x4c6a },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x602a), 0x09a2 },
	{ CCI_REG16(0x6f12), 0x00f8 },
	{ CCI_REG16(0x6f12), 0x007f },
	{ CCI_REG16(0x6f12), 0x00ff },
	{ CCI_REG16(0x6f12), 0x0084 },
	{ CCI_REG16(0x602a), 0x09b0 },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x6f12), 0x0013 },
	{ CCI_REG16(0x6f12), 0xd0d0 },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x6f12), 0x0013 },
	{ CCI_REG16(0x6f12), 0xd8d0 },
	{ CCI_REG16(0x602a), 0x09da },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x602a), 0x09e0 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x602a), 0x09e6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xd004 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xd804 },
	{ CCI_REG16(0x602a), 0x0af4 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x4b2e },
	{ CCI_REG16(0x6f12), 0x00cc },
	{ CCI_REG16(0x602a), 0x4b60 },
	{ CCI_REG16(0x6f12), 0x0133 },
	{ CCI_REG16(0x602a), 0x4b92 },
	{ CCI_REG16(0x6f12), 0x00cc },
	{ CCI_REG16(0x602a), 0x4b56 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b88 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b4c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b7e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b42 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x357c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4040 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b04 },
	{ CCI_REG16(0x6f12), 0x0000 },
};

/* Mode 0: 4032x3024, 666 MHz link frequency */
static const struct cci_reg_sequence s5k2l7_mode_4032x3024_regs[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x0000 },
	{ CCI_REG16(0x0348), 0x1fbf },
	{ CCI_REG16(0x034a), 0x0bdf },
	{ CCI_REG16(0x034c), 0x0fc0 },
	{ CCI_REG16(0x034e), 0x0bd0 },
	{ CCI_REG16(0x0408), 0x0010 },
	{ CCI_REG16(0x040a), 0x0008 },
	{ CCI_REG16(0x0900), 0x0011 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0001 },
	{ CCI_REG16(0x0400), 0x0000 },
	{ CCI_REG16(0x0404), 0x0010 },
	{ CCI_REG16(0x3060), 0x0000 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0136), 0x1800 },
	{ CCI_REG16(0x0304), 0x0006 },
	{ CCI_REG16(0x0306), 0x01e0 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0300), 0x0004 },
	{ CCI_REG16(0x030c), 0x0001 },
	{ CCI_REG16(0x030e), 0x0004 },
	{ CCI_REG16(0x0310), 0x00de },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x0342), 0x27b0 },
	{ CCI_REG16(0x0340), 0x0c5a },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x3098), 0x0300 },
	{ CCI_REG16(0x309a), 0x0102 },
	{ CCI_REG16(0x30bc), 0x0131 },
	{ CCI_REG16(0x30a6), 0x0010 },
	{ CCI_REG16(0x30a8), 0x0002 },
	{ CCI_REG16(0x30aa), 0x0fc0 },
	{ CCI_REG16(0x30ac), 0x02f4 },
	{ CCI_REG16(0x30a0), 0x0000 },
	{ CCI_REG16(0x30a4), 0x0000 },
	{ CCI_REG16(0x6a0c), 0xffff },
	{ CCI_REG16(0xf41e), 0x2180 },
	{ CCI_REG16(0xf414), 0x0007 },
	{ CCI_REG16(0xf416), 0x0004 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0990 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x0af8 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x27b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2aa8 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x09ac },
	{ CCI_REG16(0x6f12), 0x1e7f },
	{ CCI_REG16(0x602a), 0x09aa },
	{ CCI_REG16(0x6f12), 0x1e7f },
	{ CCI_REG16(0x602a), 0x0b02 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1638 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x602a), 0x1630 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x602a), 0x160e },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x1698 },
	{ CCI_REG16(0x6f12), 0x00a2 },
	{ CCI_REG16(0x602a), 0x169c },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0x0030 },
	{ CCI_REG16(0x6f12), 0x0c18 },
	{ CCI_REG16(0x6f12), 0x0c18 },
	{ CCI_REG16(0x6f12), 0x0c28 },
	{ CCI_REG16(0x6f12), 0x0c28 },
	{ CCI_REG16(0x6f12), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0c30 },
	{ CCI_REG16(0x6f12), 0x0c30 },
	{ CCI_REG16(0x602a), 0x16c6 },
	{ CCI_REG16(0x6f12), 0x122f },
	{ CCI_REG16(0x6f12), 0x4328 },
	{ CCI_REG16(0x602a), 0x1604 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x14ca },
	{ CCI_REG16(0x6f12), 0x0096 },
	{ CCI_REG16(0x602a), 0x16ce },
	{ CCI_REG16(0x6f12), 0x06c0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x13aa },
	{ CCI_REG16(0x6f12), 0x02a0 },
	{ CCI_REG16(0x602a), 0x13a2 },
	{ CCI_REG16(0x6f12), 0x028c },
	{ CCI_REG16(0x602a), 0x139a },
	{ CCI_REG16(0x6f12), 0x0278 },
	{ CCI_REG16(0x602a), 0x1392 },
	{ CCI_REG16(0x6f12), 0x0264 },
	{ CCI_REG16(0x602a), 0x138a },
	{ CCI_REG16(0x6f12), 0x0250 },
	{ CCI_REG16(0x602a), 0x1382 },
	{ CCI_REG16(0x6f12), 0x023c },
	{ CCI_REG16(0x602a), 0x137a },
	{ CCI_REG16(0x6f12), 0x0228 },
	{ CCI_REG16(0x602a), 0x1372 },
	{ CCI_REG16(0x6f12), 0x0214 },
	{ CCI_REG16(0x602a), 0x136a },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1362 },
	{ CCI_REG16(0x6f12), 0x01ec },
	{ CCI_REG16(0x602a), 0x135a },
	{ CCI_REG16(0x6f12), 0x01d8 },
	{ CCI_REG16(0x602a), 0x1352 },
	{ CCI_REG16(0x6f12), 0x01c4 },
	{ CCI_REG16(0x602a), 0x134a },
	{ CCI_REG16(0x6f12), 0x01b0 },
	{ CCI_REG16(0x602a), 0x1342 },
	{ CCI_REG16(0x6f12), 0x029b },
	{ CCI_REG16(0x602a), 0x133a },
	{ CCI_REG16(0x6f12), 0x0291 },
	{ CCI_REG16(0x602a), 0x1332 },
	{ CCI_REG16(0x6f12), 0x0287 },
	{ CCI_REG16(0x602a), 0x132a },
	{ CCI_REG16(0x6f12), 0x027d },
	{ CCI_REG16(0x602a), 0x1322 },
	{ CCI_REG16(0x6f12), 0x0273 },
	{ CCI_REG16(0x602a), 0x131a },
	{ CCI_REG16(0x6f12), 0x0269 },
	{ CCI_REG16(0x602a), 0x1312 },
	{ CCI_REG16(0x6f12), 0x025f },
	{ CCI_REG16(0x602a), 0x130a },
	{ CCI_REG16(0x6f12), 0x0255 },
	{ CCI_REG16(0x602a), 0x1302 },
	{ CCI_REG16(0x6f12), 0x024b },
	{ CCI_REG16(0x602a), 0x12fa },
	{ CCI_REG16(0x6f12), 0x0241 },
	{ CCI_REG16(0x602a), 0x12f2 },
	{ CCI_REG16(0x6f12), 0x0237 },
	{ CCI_REG16(0x602a), 0x12ea },
	{ CCI_REG16(0x6f12), 0x022d },
	{ CCI_REG16(0x602a), 0x12e2 },
	{ CCI_REG16(0x6f12), 0x0223 },
	{ CCI_REG16(0x602a), 0x12da },
	{ CCI_REG16(0x6f12), 0x0219 },
	{ CCI_REG16(0x602a), 0x12d2 },
	{ CCI_REG16(0x6f12), 0x020f },
	{ CCI_REG16(0x602a), 0x12ca },
	{ CCI_REG16(0x6f12), 0x0205 },
	{ CCI_REG16(0x602a), 0x12c2 },
	{ CCI_REG16(0x6f12), 0x01fb },
	{ CCI_REG16(0x602a), 0x12ba },
	{ CCI_REG16(0x6f12), 0x01f1 },
	{ CCI_REG16(0x602a), 0x12b2 },
	{ CCI_REG16(0x6f12), 0x01e7 },
	{ CCI_REG16(0x602a), 0x12aa },
	{ CCI_REG16(0x6f12), 0x01dd },
	{ CCI_REG16(0x602a), 0x12a2 },
	{ CCI_REG16(0x6f12), 0x01d3 },
	{ CCI_REG16(0x602a), 0x129a },
	{ CCI_REG16(0x6f12), 0x01c9 },
	{ CCI_REG16(0x602a), 0x1292 },
	{ CCI_REG16(0x6f12), 0x01bf },
	{ CCI_REG16(0x602a), 0x128a },
	{ CCI_REG16(0x6f12), 0x01b5 },
	{ CCI_REG16(0x602a), 0x11e2 },
	{ CCI_REG16(0x6f12), 0x0258 },
	{ CCI_REG16(0x602a), 0x0c32 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x0b16 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x0daa },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0d9a },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0d7a },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0c22 },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0ce8 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x602a), 0x0cd8 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0d78 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0d98 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0da8 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x1168 },
	{ CCI_REG16(0x6f12), 0x0352 },
	{ CCI_REG16(0x602a), 0x1070 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x602a), 0x1038 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x1030 },
	{ CCI_REG16(0x6f12), 0x0352 },
	{ CCI_REG16(0x602a), 0x0ff8 },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x602a), 0x0f48 },
	{ CCI_REG16(0x6f12), 0x0354 },
	{ CCI_REG16(0x602a), 0x0f40 },
	{ CCI_REG16(0x6f12), 0x033e },
	{ CCI_REG16(0x602a), 0x0f38 },
	{ CCI_REG16(0x6f12), 0x033c },
	{ CCI_REG16(0x602a), 0x0f30 },
	{ CCI_REG16(0x6f12), 0x0326 },
	{ CCI_REG16(0x602a), 0x0f28 },
	{ CCI_REG16(0x6f12), 0x0324 },
	{ CCI_REG16(0x602a), 0x0f20 },
	{ CCI_REG16(0x6f12), 0x030e },
	{ CCI_REG16(0x602a), 0x0e88 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x602a), 0x0de8 },
	{ CCI_REG16(0x6f12), 0x0351 },
	{ CCI_REG16(0x602a), 0x0dc8 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x0db8 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x15ec },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0aec },
	{ CCI_REG16(0x6f12), 0x0207 },
	{ CCI_REG16(0x602a), 0x0bd8 },
	{ CCI_REG16(0x6f12), 0x06e0 },
	{ CCI_REG16(0x602a), 0x0ae4 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29e6 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29f0 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x2966 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29a6 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x2970 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29b0 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x0ae6 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29e8 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29f2 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x2968 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29a8 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x2972 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29b2 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x0bb0 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0b62 },
	{ CCI_REG16(0x6f12), 0x0142 },
	{ CCI_REG16(0x602a), 0x5160 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x09f2 },
	{ CCI_REG16(0x6f12), 0x0180 },
	{ CCI_REG16(0x6f12), 0x2180 },
	{ CCI_REG16(0x6f12), 0xf41e },
	{ CCI_REG16(0x602a), 0x0b06 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x5694 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x602a), 0x569c },
	{ CCI_REG16(0x6f12), 0x010a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0240 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x56d8 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x602a), 0x56e0 },
	{ CCI_REG16(0x6f12), 0x010a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0240 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x15f4 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x0420 },
	{ CCI_REG16(0x602a), 0x0bee },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x0a12 },
	{ CCI_REG16(0x6f12), 0x0520 },
	{ CCI_REG16(0x602a), 0x09da },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x602a), 0x09e0 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x79f0 },
	{ CCI_REG16(0x6218), 0x79f0 },
};

/* Mode 3: 2016x1512 (2x2 binned), 351 MHz link frequency */
static const struct cci_reg_sequence s5k2l7_mode_2016x1512_regs[] = {
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x0000 },
	{ CCI_REG16(0x0348), 0x1fbf },
	{ CCI_REG16(0x034a), 0x0bdf },
	{ CCI_REG16(0x034c), 0x07e0 },
	{ CCI_REG16(0x034e), 0x05e8 },
	{ CCI_REG16(0x0408), 0x0008 },
	{ CCI_REG16(0x040a), 0x0004 },
	{ CCI_REG16(0x0900), 0x0122 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0003 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0003 },
	{ CCI_REG16(0x0400), 0x0000 },
	{ CCI_REG16(0x0404), 0x0010 },
	{ CCI_REG16(0x3060), 0x0000 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0136), 0x1800 },
	{ CCI_REG16(0x0304), 0x0006 },
	{ CCI_REG16(0x0306), 0x01e0 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0300), 0x0004 },
	{ CCI_REG16(0x030c), 0x0001 },
	{ CCI_REG16(0x030e), 0x0004 },
	{ CCI_REG16(0x0310), 0x00ea },
	{ CCI_REG16(0x0312), 0x0001 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x0342), 0x27b0 },
	{ CCI_REG16(0x0340), 0x0c4d },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x3098), 0x0300 },
	{ CCI_REG16(0x309a), 0x0102 },
	{ CCI_REG16(0x30bc), 0x0131 },
	{ CCI_REG16(0x30a6), 0x0008 },
	{ CCI_REG16(0x30a8), 0x0000 },
	{ CCI_REG16(0x30aa), 0x07e0 },
	{ CCI_REG16(0x30ac), 0x0000 },
	{ CCI_REG16(0x30a0), 0x0001 },
	{ CCI_REG16(0x30a4), 0x017a },
	{ CCI_REG16(0x6a0c), 0xffff },
	{ CCI_REG16(0xf41e), 0x2180 },
	{ CCI_REG16(0xf414), 0x0007 },
	{ CCI_REG16(0xf416), 0x0004 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0990 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x0af8 },
	{ CCI_REG16(0x6f12), 0x0008 },
	{ CCI_REG16(0x602a), 0x27b8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x2aa8 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x09ac },
	{ CCI_REG16(0x6f12), 0x1e7f },
	{ CCI_REG16(0x602a), 0x09aa },
	{ CCI_REG16(0x6f12), 0x1e7f },
	{ CCI_REG16(0x602a), 0x0b02 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x1638 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x602a), 0x1630 },
	{ CCI_REG16(0x6f12), 0x0408 },
	{ CCI_REG16(0x602a), 0x160e },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x1698 },
	{ CCI_REG16(0x6f12), 0x00a0 },
	{ CCI_REG16(0x602a), 0x169c },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0x0030 },
	{ CCI_REG16(0x6f12), 0x0c18 },
	{ CCI_REG16(0x6f12), 0x0c18 },
	{ CCI_REG16(0x6f12), 0x0c28 },
	{ CCI_REG16(0x6f12), 0x0c28 },
	{ CCI_REG16(0x6f12), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0c30 },
	{ CCI_REG16(0x6f12), 0x0c30 },
	{ CCI_REG16(0x602a), 0x16c6 },
	{ CCI_REG16(0x6f12), 0x12af },
	{ CCI_REG16(0x6f12), 0x0328 },
	{ CCI_REG16(0x602a), 0x1604 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x14ca },
	{ CCI_REG16(0x6f12), 0x0096 },
	{ CCI_REG16(0x602a), 0x16ce },
	{ CCI_REG16(0x6f12), 0x06c0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x13aa },
	{ CCI_REG16(0x6f12), 0x02a0 },
	{ CCI_REG16(0x602a), 0x13a2 },
	{ CCI_REG16(0x6f12), 0x028c },
	{ CCI_REG16(0x602a), 0x139a },
	{ CCI_REG16(0x6f12), 0x0278 },
	{ CCI_REG16(0x602a), 0x1392 },
	{ CCI_REG16(0x6f12), 0x0264 },
	{ CCI_REG16(0x602a), 0x138a },
	{ CCI_REG16(0x6f12), 0x0250 },
	{ CCI_REG16(0x602a), 0x1382 },
	{ CCI_REG16(0x6f12), 0x023c },
	{ CCI_REG16(0x602a), 0x137a },
	{ CCI_REG16(0x6f12), 0x0228 },
	{ CCI_REG16(0x602a), 0x1372 },
	{ CCI_REG16(0x6f12), 0x0214 },
	{ CCI_REG16(0x602a), 0x136a },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1362 },
	{ CCI_REG16(0x6f12), 0x01ec },
	{ CCI_REG16(0x602a), 0x135a },
	{ CCI_REG16(0x6f12), 0x01d8 },
	{ CCI_REG16(0x602a), 0x1352 },
	{ CCI_REG16(0x6f12), 0x01c4 },
	{ CCI_REG16(0x602a), 0x134a },
	{ CCI_REG16(0x6f12), 0x01b0 },
	{ CCI_REG16(0x602a), 0x1342 },
	{ CCI_REG16(0x6f12), 0x029b },
	{ CCI_REG16(0x602a), 0x133a },
	{ CCI_REG16(0x6f12), 0x0291 },
	{ CCI_REG16(0x602a), 0x1332 },
	{ CCI_REG16(0x6f12), 0x0287 },
	{ CCI_REG16(0x602a), 0x132a },
	{ CCI_REG16(0x6f12), 0x027d },
	{ CCI_REG16(0x602a), 0x1322 },
	{ CCI_REG16(0x6f12), 0x0273 },
	{ CCI_REG16(0x602a), 0x131a },
	{ CCI_REG16(0x6f12), 0x0269 },
	{ CCI_REG16(0x602a), 0x1312 },
	{ CCI_REG16(0x6f12), 0x025f },
	{ CCI_REG16(0x602a), 0x130a },
	{ CCI_REG16(0x6f12), 0x0255 },
	{ CCI_REG16(0x602a), 0x1302 },
	{ CCI_REG16(0x6f12), 0x024b },
	{ CCI_REG16(0x602a), 0x12fa },
	{ CCI_REG16(0x6f12), 0x0241 },
	{ CCI_REG16(0x602a), 0x12f2 },
	{ CCI_REG16(0x6f12), 0x0237 },
	{ CCI_REG16(0x602a), 0x12ea },
	{ CCI_REG16(0x6f12), 0x022d },
	{ CCI_REG16(0x602a), 0x12e2 },
	{ CCI_REG16(0x6f12), 0x0223 },
	{ CCI_REG16(0x602a), 0x12da },
	{ CCI_REG16(0x6f12), 0x0219 },
	{ CCI_REG16(0x602a), 0x12d2 },
	{ CCI_REG16(0x6f12), 0x020f },
	{ CCI_REG16(0x602a), 0x12ca },
	{ CCI_REG16(0x6f12), 0x0205 },
	{ CCI_REG16(0x602a), 0x12c2 },
	{ CCI_REG16(0x6f12), 0x01fb },
	{ CCI_REG16(0x602a), 0x12ba },
	{ CCI_REG16(0x6f12), 0x01f1 },
	{ CCI_REG16(0x602a), 0x12b2 },
	{ CCI_REG16(0x6f12), 0x01e7 },
	{ CCI_REG16(0x602a), 0x12aa },
	{ CCI_REG16(0x6f12), 0x01dd },
	{ CCI_REG16(0x602a), 0x12a2 },
	{ CCI_REG16(0x6f12), 0x01d3 },
	{ CCI_REG16(0x602a), 0x129a },
	{ CCI_REG16(0x6f12), 0x01c9 },
	{ CCI_REG16(0x602a), 0x1292 },
	{ CCI_REG16(0x6f12), 0x01bf },
	{ CCI_REG16(0x602a), 0x128a },
	{ CCI_REG16(0x6f12), 0x01b5 },
	{ CCI_REG16(0x602a), 0x11e2 },
	{ CCI_REG16(0x6f12), 0x0258 },
	{ CCI_REG16(0x602a), 0x0c32 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x0b16 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0daa },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0d9a },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0d7a },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0c22 },
	{ CCI_REG16(0x6f12), 0x028a },
	{ CCI_REG16(0x602a), 0x0ce8 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x602a), 0x0cd8 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0d78 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0d98 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0da8 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x1168 },
	{ CCI_REG16(0x6f12), 0x0352 },
	{ CCI_REG16(0x602a), 0x1070 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x602a), 0x1038 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x1030 },
	{ CCI_REG16(0x6f12), 0x0352 },
	{ CCI_REG16(0x602a), 0x0ff8 },
	{ CCI_REG16(0x6f12), 0x0357 },
	{ CCI_REG16(0x602a), 0x0f48 },
	{ CCI_REG16(0x6f12), 0x0354 },
	{ CCI_REG16(0x602a), 0x0f40 },
	{ CCI_REG16(0x6f12), 0x033e },
	{ CCI_REG16(0x602a), 0x0f38 },
	{ CCI_REG16(0x6f12), 0x033c },
	{ CCI_REG16(0x602a), 0x0f30 },
	{ CCI_REG16(0x6f12), 0x0326 },
	{ CCI_REG16(0x602a), 0x0f28 },
	{ CCI_REG16(0x6f12), 0x0324 },
	{ CCI_REG16(0x602a), 0x0f20 },
	{ CCI_REG16(0x6f12), 0x030e },
	{ CCI_REG16(0x602a), 0x0e88 },
	{ CCI_REG16(0x6f12), 0x0353 },
	{ CCI_REG16(0x602a), 0x0de8 },
	{ CCI_REG16(0x6f12), 0x0351 },
	{ CCI_REG16(0x602a), 0x0dc8 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x0db8 },
	{ CCI_REG16(0x6f12), 0x0355 },
	{ CCI_REG16(0x602a), 0x15ec },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0aec },
	{ CCI_REG16(0x6f12), 0x0207 },
	{ CCI_REG16(0x602a), 0x0bd8 },
	{ CCI_REG16(0x6f12), 0x06e0 },
	{ CCI_REG16(0x602a), 0x0ae4 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29e6 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29f0 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x2966 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29a6 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x2970 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29b0 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x0ae6 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29e8 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29f2 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x2968 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29a8 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x2972 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29b2 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x0bb0 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0b62 },
	{ CCI_REG16(0x6f12), 0x0166 },
	{ CCI_REG16(0x602a), 0x5160 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x09f2 },
	{ CCI_REG16(0x6f12), 0x0180 },
	{ CCI_REG16(0x6f12), 0x2180 },
	{ CCI_REG16(0x6f12), 0xf41e },
	{ CCI_REG16(0x602a), 0x0b06 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x5694 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x569c },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x56d8 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x56e0 },
	{ CCI_REG16(0x6f12), 0x000c },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x15f4 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x0bde },
	{ CCI_REG16(0x6f12), 0x0420 },
	{ CCI_REG16(0x602a), 0x0bee },
	{ CCI_REG16(0x6f12), 0x0003 },
	{ CCI_REG16(0x602a), 0x0a12 },
	{ CCI_REG16(0x6f12), 0x0520 },
	{ CCI_REG16(0x602a), 0x09da },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x602a), 0x09e0 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x79f0 },
	{ CCI_REG16(0x6218), 0x79f0 },
};

/* Delays after the vendor register tables, in milliseconds */
#define S5K2L7_INIT0_DELAY_MS		(3 + 5)
#define S5K2L7_INIT1_DELAY_MS		3
#define S5K2L7_MODE_DELAY_MS		5
#define S5K2L7_STREAM_DELAY_MS		5

/*
 * With no flip, the vendor library describes the colour filter as GBRG
 * (CFA arrangement 2). Each flip moves the pattern by one pixel:
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 s5k2l7_mbus_codes[] = {
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
};

static const char * const s5k2l7_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const char * const s5k2l7_supply_name[] = {
	/* Enabled in this order, disabled in reverse order */
	"vana",	/* Analog (2.8 V) supply */
	"vdig",	/* Digital core supply */
	"vio",	/* I/O (1.8 V) supply */
};

#define S5K2L7_NUM_SUPPLIES		ARRAY_SIZE(s5k2l7_supply_name)

enum {
	S5K2L7_LINK_FREQ_666MHZ,
	S5K2L7_LINK_FREQ_351MHZ,
};

static const s64 s5k2l7_link_freq_menu[] = {
	[S5K2L7_LINK_FREQ_666MHZ] = 666 * HZ_PER_MHZ,
	[S5K2L7_LINK_FREQ_351MHZ] = 351 * HZ_PER_MHZ,
};

static const struct s5k2l7_mode s5k2l7_modes[] = {
	{
		.width = 4032,
		.height = 3024,
		.fll = 3162,
		.link_freq_index = S5K2L7_LINK_FREQ_666MHZ,
		.reg_list = {
			.regs = s5k2l7_mode_4032x3024_regs,
			.num_regs = ARRAY_SIZE(s5k2l7_mode_4032x3024_regs),
		},
	}, {
		.width = 2016,
		.height = 1512,
		.fll = 3149,
		.link_freq_index = S5K2L7_LINK_FREQ_351MHZ,
		.reg_list = {
			.regs = s5k2l7_mode_2016x1512_regs,
			.num_regs = ARRAY_SIZE(s5k2l7_mode_2016x1512_regs),
		},
	},
};

struct s5k2l7 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K2L7_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

static inline struct s5k2l7 *to_s5k2l7(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k2l7, sd);
}

/*
 * Get the bayer order based on the flip settings. Called with the subdev
 * state lock (shared with the control handler) held.
 */
static u32 s5k2l7_get_format_code(const struct s5k2l7 *s5k2l7)
{
	unsigned int i;

	i = (s5k2l7->vflip->val ? 2 : 0) | (s5k2l7->hflip->val ? 1 : 0);

	return s5k2l7_mbus_codes[i];
}

static const struct s5k2l7_mode *
s5k2l7_find_mode(const struct v4l2_mbus_framefmt *fmt)
{
	return v4l2_find_nearest_size(s5k2l7_modes, ARRAY_SIZE(s5k2l7_modes),
				      width, height, fmt->width, fmt->height);
}

static void s5k2l7_fill_crop(struct v4l2_rect *crop)
{
	crop->left = S5K2L7_CROP_LEFT;
	crop->top = S5K2L7_CROP_TOP;
	crop->width = S5K2L7_CROP_WIDTH;
	crop->height = S5K2L7_CROP_HEIGHT;
}

static int s5k2l7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);

	fmt->width = s5k2l7_modes[0].width;
	fmt->height = s5k2l7_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->code = s5k2l7_get_format_code(s5k2l7);
	fmt->colorspace = V4L2_COLORSPACE_RAW;

	s5k2l7_fill_crop(v4l2_subdev_state_get_crop(state, 0));

	return 0;
}

static void s5k2l7_adjust_exposure_range(struct s5k2l7 *s5k2l7,
					 const struct s5k2l7_mode *mode)
{
	s64 exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure */
	exposure_max = mode->height + s5k2l7->vblank->val -
		       S5K2L7_EXPOSURE_MARGIN;
	exposure_def = min_t(s64, exposure_max, s5k2l7->exposure->val);
	__v4l2_ctrl_modify_range(s5k2l7->exposure, S5K2L7_EXPOSURE_MIN,
				 exposure_max, S5K2L7_EXPOSURE_STEP,
				 exposure_def);
}

static int s5k2l7_update_digital_gain(struct s5k2l7 *s5k2l7, u32 val)
{
	int ret = 0;
	int release_ret = 0;

	/* Apply all channels in the same frame */
	cci_write(s5k2l7->regmap, S5K2L7_REG_GROUP_HOLD, 1, &ret);
	cci_write(s5k2l7->regmap, S5K2L7_REG_DIG_GAIN_GR, val, &ret);
	cci_write(s5k2l7->regmap, S5K2L7_REG_DIG_GAIN_R, val, &ret);
	cci_write(s5k2l7->regmap, S5K2L7_REG_DIG_GAIN_B, val, &ret);
	cci_write(s5k2l7->regmap, S5K2L7_REG_DIG_GAIN_GB, val, &ret);

	/* Always release the group hold, even if a write above failed */
	cci_write(s5k2l7->regmap, S5K2L7_REG_GROUP_HOLD, 0, &release_ret);

	return ret ?: release_ret;
}

static int s5k2l7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k2l7 *s5k2l7 =
		container_of(ctrl->handler, struct s5k2l7, ctrl_handler);
	struct v4l2_subdev_state *state =
		v4l2_subdev_get_locked_active_state(&s5k2l7->sd);
	struct device *dev = s5k2l7->sd.dev;
	const struct s5k2l7_mode *mode =
		s5k2l7_find_mode(v4l2_subdev_state_get_format(state, 0));
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so
	 * check and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		s5k2l7_adjust_exposure_range(s5k2l7, mode);

	/* The Bayer order depends on the flips, keep the format in sync */
	if (ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP)
		v4l2_subdev_state_get_format(state, 0)->code =
			s5k2l7_get_format_code(s5k2l7);

	/* Controls are only applied to the sensor when it is powered */
	if (pm_runtime_get_if_in_use(dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = s5k2l7_update_digital_gain(s5k2l7, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_FLL,
				mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_ORIENTATION,
				(s5k2l7->hflip->val ?
				 S5K2L7_ORIENTATION_HFLIP : 0) |
				(s5k2l7->vflip->val ?
				 S5K2L7_ORIENTATION_VFLIP : 0), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_LINK_FREQ:
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_HBLANK:
		/* Read-only controls, updated internally on mode change */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k2l7_ctrl_ops = {
	.s_ctrl = s5k2l7_set_ctrl,
};

static int s5k2l7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	/* Only one bayer format (10 bit) is supported at a time */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k2l7_get_format_code(s5k2l7);

	return 0;
}

static int s5k2l7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	if (fse->index >= ARRAY_SIZE(s5k2l7_modes))
		return -EINVAL;

	if (fse->code != s5k2l7_get_format_code(s5k2l7))
		return -EINVAL;

	fse->min_width = s5k2l7_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k2l7_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void s5k2l7_update_mode_ctrls(struct s5k2l7 *s5k2l7,
				     const struct s5k2l7_mode *mode)
{
	s64 vblank_def = mode->fll - mode->height;
	s64 hblank = S5K2L7_LLP - mode->width;

	__v4l2_ctrl_s_ctrl(s5k2l7->link_freq, mode->link_freq_index);
	__v4l2_ctrl_modify_range(s5k2l7->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(s5k2l7->vblank, vblank_def,
				 S5K2L7_FLL_MAX - mode->height, 1,
				 vblank_def);
	__v4l2_ctrl_s_ctrl(s5k2l7->vblank, vblank_def);
	s5k2l7_adjust_exposure_range(s5k2l7, mode);
}

static int s5k2l7_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	const struct s5k2l7_mode *mode;

	/* The mode cannot be changed while streaming */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = s5k2l7_find_mode(&fmt->format);

	fmt->format.code = s5k2l7_get_format_code(s5k2l7);
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	s5k2l7_fill_crop(v4l2_subdev_state_get_crop(state, fmt->pad));

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		s5k2l7_update_mode_ctrls(s5k2l7, mode);

	return 0;
}

static int s5k2l7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, sel->pad);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K2L7_NATIVE_WIDTH;
		sel->r.height = S5K2L7_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s5k2l7_fill_crop(&sel->r);
		return 0;
	}

	return -EINVAL;
}

static int s5k2l7_write_regs(struct s5k2l7 *s5k2l7,
			     const struct cci_reg_sequence *regs,
			     unsigned int num_regs, unsigned int delay_ms)
{
	int ret;

	ret = cci_multi_reg_write(s5k2l7->regmap, regs, num_regs, NULL);
	if (ret)
		return ret;

	if (delay_ms)
		msleep(delay_ms);

	return 0;
}

static int s5k2l7_start_streaming(struct s5k2l7 *s5k2l7,
				  const struct s5k2l7_mode *mode)
{
	struct device *dev = s5k2l7->sd.dev;
	int ret;

	/* The sensor firmware is lost at power off, load it every time */
	ret = s5k2l7_write_regs(s5k2l7, s5k2l7_init0_regs,
				ARRAY_SIZE(s5k2l7_init0_regs),
				S5K2L7_INIT0_DELAY_MS);
	if (ret)
		goto err;

	ret = s5k2l7_write_regs(s5k2l7, s5k2l7_init1_regs,
				ARRAY_SIZE(s5k2l7_init1_regs),
				S5K2L7_INIT1_DELAY_MS);
	if (ret)
		goto err;

	ret = s5k2l7_write_regs(s5k2l7, mode->reg_list.regs,
				mode->reg_list.num_regs, S5K2L7_MODE_DELAY_MS);
	if (ret)
		goto err;

	/* Apply the controls */
	ret = __v4l2_ctrl_handler_setup(s5k2l7->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = cci_write(s5k2l7->regmap, S5K2L7_REG_MODE_SELECT,
			S5K2L7_MODE_STREAMING, NULL);
	if (ret)
		goto err;

	msleep(S5K2L7_STREAM_DELAY_MS);

	return 0;

err:
	dev_err(dev, "failed to start streaming: %d\n", ret);

	return ret;
}

static void s5k2l7_stop_streaming(struct s5k2l7 *s5k2l7)
{
	struct device *dev = s5k2l7->sd.dev;
	int ret;

	ret = cci_write(s5k2l7->regmap, S5K2L7_REG_MODE_SELECT,
			S5K2L7_MODE_STANDBY, NULL);
	if (ret)
		dev_err(dev, "failed to stop streaming: %d\n", ret);

	msleep(S5K2L7_STREAM_DELAY_MS);
}

/*
 * Power sequence of the vendor library: reset asserted, rails vana, vdig and
 * vio, reset released, then the external clock.
 */
static int s5k2l7_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	int ret;

	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 1);

	ret = regulator_bulk_enable(S5K2L7_NUM_SUPPLIES, s5k2l7->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	usleep_range(2000, 3000);

	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 0);
	usleep_range(10000, 11000);

	ret = clk_prepare_enable(s5k2l7->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto err_reset;
	}

	usleep_range(10000, 12000);

	return 0;

err_reset:
	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 1);
	regulator_bulk_disable(S5K2L7_NUM_SUPPLIES, s5k2l7->supplies);

	return ret;
}

static int s5k2l7_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	clk_disable_unprepare(s5k2l7->clk);
	usleep_range(2000, 3000);

	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 1);
	usleep_range(2000, 3000);

	regulator_bulk_disable(S5K2L7_NUM_SUPPLIES, s5k2l7->supplies);

	return 0;
}

static int s5k2l7_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	const struct s5k2l7_mode *mode;
	struct device *dev = sd->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	mode = s5k2l7_find_mode(v4l2_subdev_state_get_format(state, 0));
	ret = s5k2l7_start_streaming(s5k2l7, mode);
	if (ret) {
		pm_runtime_put(dev);
		return ret;
	}

	/* The Bayer order cannot change during streaming */
	__v4l2_ctrl_grab(s5k2l7->vflip, true);
	__v4l2_ctrl_grab(s5k2l7->hflip, true);

	return 0;
}

static int s5k2l7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	s5k2l7_stop_streaming(s5k2l7);

	__v4l2_ctrl_grab(s5k2l7->vflip, false);
	__v4l2_ctrl_grab(s5k2l7->hflip, false);

	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k2l7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k2l7_pad_ops = {
	.enum_mbus_code = s5k2l7_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k2l7_set_pad_format,
	.enum_frame_size = s5k2l7_enum_frame_size,
	.get_selection = s5k2l7_get_selection,
	.enable_streams = s5k2l7_enable_streams,
	.disable_streams = s5k2l7_disable_streams,
};

static const struct v4l2_subdev_ops s5k2l7_subdev_ops = {
	.video = &s5k2l7_video_ops,
	.pad = &s5k2l7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k2l7_internal_ops = {
	.init_state = s5k2l7_init_state,
};

static int s5k2l7_init_controls(struct s5k2l7 *s5k2l7)
{
	const struct s5k2l7_mode *mode = &s5k2l7_modes[0];
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k2l7->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	s64 hblank, vblank_def, exposure_max;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 13);
	if (ret)
		return ret;

	s5k2l7->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k2l7_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(s5k2l7_link_freq_menu) - 1,
				       mode->link_freq_index,
				       s5k2l7_link_freq_menu);
	if (s5k2l7->link_freq)
		s5k2l7->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Both modes use the same video timing pixel rate */
	s5k2l7->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       S5K2L7_PIXEL_RATE,
					       S5K2L7_PIXEL_RATE, 1,
					       S5K2L7_PIXEL_RATE);

	hblank = S5K2L7_LLP - mode->width;
	s5k2l7->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (s5k2l7->hblank)
		s5k2l7->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->fll - mode->height;
	s5k2l7->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   S5K2L7_FLL_MAX - mode->height, 1,
					   vblank_def);

	exposure_max = mode->fll - S5K2L7_EXPOSURE_MARGIN;
	s5k2l7->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K2L7_EXPOSURE_MIN, exposure_max,
					     S5K2L7_EXPOSURE_STEP,
					     S5K2L7_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, S5K2L7_ANA_GAIN_MIN,
			  S5K2L7_ANA_GAIN_MAX, S5K2L7_ANA_GAIN_STEP,
			  S5K2L7_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
			  V4L2_CID_DIGITAL_GAIN, S5K2L7_DIG_GAIN_MIN,
			  S5K2L7_DIG_GAIN_MAX, S5K2L7_DIG_GAIN_STEP,
			  S5K2L7_DIG_GAIN_DEFAULT);

	s5k2l7->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k2l7->hflip)
		s5k2l7->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k2l7->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k2l7->vflip)
		s5k2l7->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k2l7_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k2l7_test_pattern_menu) - 1,
				     0, 0, s5k2l7_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(s5k2l7->sd.dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k2l7_ctrl_ops,
					      &props);
	if (ret)
		goto err_free;

	s5k2l7->sd.ctrl_handler = ctrl_hdlr;

	return 0;

err_free:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/*
 * Check that the endpoint describes the supported configuration: four data
 * lanes and both link frequencies, one per mode.
 */
static int s5k2l7_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *endpoint;
	unsigned long bitmap;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint node failed\n");

	if (ep.bus.mipi_csi2.num_data_lanes != S5K2L7_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL, "invalid data lanes: %u\n",
				    ep.bus.mipi_csi2.num_data_lanes);
		goto out_free;
	}

	ret = v4l2_link_freq_to_bitmap(dev, ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       s5k2l7_link_freq_menu,
				       ARRAY_SIZE(s5k2l7_link_freq_menu),
				       &bitmap);
	if (ret) {
		dev_err_probe(dev, ret, "link frequency not supported\n");
		goto out_free;
	}

	if (bitmap != GENMASK(ARRAY_SIZE(s5k2l7_link_freq_menu) - 1, 0))
		ret = dev_err_probe(dev, -EINVAL,
				    "all link frequencies are required\n");

out_free:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static int s5k2l7_identify_module(struct s5k2l7 *s5k2l7)
{
	struct device *dev = s5k2l7->sd.dev;
	u64 id;
	int ret;

	ret = cci_read(s5k2l7->regmap, S5K2L7_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip id\n");

	if (id != S5K2L7_CHIP_ID)
		return dev_err_probe(dev, -ENODEV,
				     "chip id mismatch: 0x%04x != 0x%04llx\n",
				     S5K2L7_CHIP_ID, id);

	return 0;
}

static int s5k2l7_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k2l7 *s5k2l7;
	unsigned int i;
	int ret;

	s5k2l7 = devm_kzalloc(dev, sizeof(*s5k2l7), GFP_KERNEL);
	if (!s5k2l7)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&s5k2l7->sd, client, &s5k2l7_subdev_ops);

	s5k2l7->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k2l7->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k2l7->regmap),
				     "failed to initialize CCI\n");

	for (i = 0; i < S5K2L7_NUM_SUPPLIES; i++)
		s5k2l7->supplies[i].supply = s5k2l7_supply_name[i];

	ret = devm_regulator_bulk_get(dev, S5K2L7_NUM_SUPPLIES,
				      s5k2l7->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	s5k2l7->clk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(s5k2l7->clk))
		return dev_err_probe(dev, PTR_ERR(s5k2l7->clk),
				     "failed to get clock\n");

	if (clk_get_rate(s5k2l7->clk) != S5K2L7_EXTCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "external clock must be %lu Hz\n",
				     S5K2L7_EXTCLK_FREQ);

	ret = s5k2l7_check_endpoint(dev);
	if (ret)
		return ret;

	/* Keep the sensor in reset until the power sequence runs */
	s5k2l7->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k2l7->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k2l7->reset_gpio),
				     "failed to get reset gpio\n");

	/* The device stays powered until pm_runtime_idle() */
	ret = s5k2l7_power_on(dev);
	if (ret)
		return ret;

	ret = s5k2l7_identify_module(s5k2l7);
	if (ret)
		goto err_power_off;

	ret = s5k2l7_init_controls(s5k2l7);
	if (ret)
		goto err_power_off;

	s5k2l7->sd.internal_ops = &s5k2l7_internal_ops;
	s5k2l7->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k2l7->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k2l7->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k2l7->sd.entity, 1, &s5k2l7->pad);
	if (ret)
		goto err_free_ctrls;

	s5k2l7->sd.state_lock = s5k2l7->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&s5k2l7->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev\n");
		goto err_media_entity;
	}

	/*
	 * The device is powered on at this point. Enable runtime PM before
	 * registering the subdev, so that it can be used as soon as it is
	 * visible.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k2l7->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register sensor subdev\n");
		goto err_subdev_cleanup;
	}

	pm_runtime_idle(dev);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&s5k2l7->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_media_entity:
	media_entity_cleanup(&s5k2l7->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&s5k2l7->ctrl_handler);
err_power_off:
	s5k2l7_power_off(dev);

	return ret;
}

static void s5k2l7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k2l7->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		s5k2l7_power_off(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
}

static const struct dev_pm_ops s5k2l7_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k2l7_power_off, s5k2l7_power_on, NULL)
};

static const struct of_device_id s5k2l7_of_match[] = {
	{ .compatible = "samsung,s5k2l7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k2l7_of_match);

static struct i2c_driver s5k2l7_i2c_driver = {
	.driver = {
		.name = "s5k2l7",
		.of_match_table = s5k2l7_of_match,
		.pm = &s5k2l7_pm_ops,
	},
	.probe = s5k2l7_probe,
	.remove = s5k2l7_remove,
};
module_i2c_driver(s5k2l7_i2c_driver);

MODULE_AUTHOR("Gianluca Boiano <morf3089@gmail.com>");
MODULE_DESCRIPTION("Samsung S5K2L7 image sensor driver");
MODULE_LICENSE("GPL");
