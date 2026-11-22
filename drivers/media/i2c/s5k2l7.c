// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2026 Kulesha Evgeniy <voovdop@gmail.com>
/*
 * Samsung S5K2L7 12MP CMOS image sensor driver
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K2L7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K2L7_CHIP_ID			0x20c7

#define S5K2L7_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K2L7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K2L7_REG_GROUP_HOLD		CCI_REG8(0x0104)
#define S5K2L7_REG_PAGE			CCI_REG16(0x6028)

#define S5K2L7_REG_VTS			CCI_REG16(0x0340)
#define S5K2L7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K2L7_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define S5K2L7_REG_DGAIN_GR		CCI_REG16(0x020e)
#define S5K2L7_REG_DGAIN_R		CCI_REG16(0x0210)
#define S5K2L7_REG_DGAIN_B		CCI_REG16(0x0212)
#define S5K2L7_REG_DGAIN_GB		CCI_REG16(0x0214)
#define S5K2L7_DGAIN_1X			0x0100

#define S5K2L7_VTS_MAX			0xffff

/* Active array of the sensor. */
#define S5K2L7_WIDTH			4032
#define S5K2L7_HEIGHT			3024

#define S5K2L7_EXP_MIN			4
#define S5K2L7_EXP_MARGIN		4

/* Samsung gain code, 0x20 == 1x. */
#define S5K2L7_GAIN_MIN			0x20
#define S5K2L7_GAIN_MAX			0x200
#define S5K2L7_GAIN_DEF			0x20

/* 4 lanes. */
#define S5K2L7_LANES			4
#define S5K2L7_XCLK_FREQ		24000000

/* Bayer order: GRBG. */
#define S5K2L7_MBUS_CODE		MEDIA_BUS_FMT_SGRBG10_1X10

struct s5k2l7_reg {
	u16 addr;
	u16 val;
};

static const struct s5k2l7_reg s5k2l7_init_regs[] = {
	{ 0x6028, 0x2000 }, { 0x602a, 0xbbf4 }, { 0x6f12, 0x0000 }, { 0x6028, 0x4000 },
	{ 0x6010, 0x0001 }, { 0x6214, 0x7970 }, { 0x6218, 0x7150 }, { 0x6028, 0x4000 },
	{ 0xf466, 0x000c }, { 0xf468, 0x000d }, { 0xf488, 0x0008 }, { 0xf414, 0x0007 },
	{ 0xf416, 0x0004 }, { 0x30c6, 0x0100 }, { 0x30ca, 0x0300 }, { 0x30c8, 0x05dc },
	{ 0x6b36, 0x5200 }, { 0x6b38, 0x0000 }, { 0x0b04, 0x0101 }, { 0x3094, 0x2800 },
	{ 0x3096, 0x5400 }, { 0x303e, 0x0000 }, { 0x6028, 0x2000 }, { 0x602a, 0x16c0 },
	{ 0x6f12, 0x0046 }, { 0x6f12, 0x0046 }, { 0x602a, 0x09e4 }, { 0x6f12, 0xf484 },
	{ 0x602a, 0x16ba }, { 0x6f12, 0x2608 }, { 0x602a, 0x16be }, { 0x6f12, 0x2608 },
	{ 0x602a, 0x15f4 }, { 0x6f12, 0x0001 }, { 0x6f12, 0x0101 }, { 0x602a, 0x1282 },
	{ 0x6f12, 0x0013 }, { 0x602a, 0x127a }, { 0x6f12, 0x0009 }, { 0x602a, 0x14c8 },
	{ 0x6f12, 0x00be }, { 0x602a, 0x15ee }, { 0x6f12, 0x0107 }, { 0x602a, 0x4c64 },
	{ 0x6f12, 0x0101 }, { 0x602a, 0x4c6a }, { 0x6f12, 0x0011 }, { 0x602a, 0x09a2 },
	{ 0x6f12, 0x00f8 }, { 0x6f12, 0x007f }, { 0x6f12, 0x00ff }, { 0x6f12, 0x0084 },
	{ 0x602a, 0x09b0 }, { 0x6f12, 0x0011 }, { 0x6f12, 0x0013 }, { 0x6f12, 0xd0d0 },
	{ 0x6f12, 0x0011 }, { 0x6f12, 0x0013 }, { 0x6f12, 0xd8d0 }, { 0x602a, 0x09da },
	{ 0x6f12, 0x0004 }, { 0x6f12, 0x000d }, { 0x602a, 0x09e0 }, { 0x6f12, 0x0004 },
	{ 0x6f12, 0x000d }, { 0x602a, 0x09e6 }, { 0x6f12, 0x0000 }, { 0x6f12, 0x0101 },
	{ 0x6f12, 0xd004 }, { 0x6f12, 0x0000 }, { 0x6f12, 0x0101 }, { 0x6f12, 0xd804 },
	{ 0x602a, 0x0af4 }, { 0x6f12, 0x0004 }, { 0x602a, 0x4b2e }, { 0x6f12, 0x00cc },
	{ 0x602a, 0x4b60 }, { 0x6f12, 0x0133 }, { 0x602a, 0x4b92 }, { 0x6f12, 0x00cc },
	{ 0x602a, 0x4b56 }, { 0x6f12, 0x0000 }, { 0x602a, 0x4b88 }, { 0x6f12, 0x0000 },
	{ 0x602a, 0x4b4c }, { 0x6f12, 0x0000 }, { 0x602a, 0x4b7e }, { 0x6f12, 0x0000 },
	{ 0x602a, 0x4b42 }, { 0x6f12, 0x0000 }, { 0x602a, 0x4b74 }, { 0x6f12, 0x0000 },
	{ 0x602a, 0x15ec }, { 0x6f12, 0x0001 }, { 0x602a, 0x0aec }, { 0x6f12, 0x0207 },
	{ 0x602a, 0x0b02 }, { 0x6f12, 0x0100 },
};

static const struct s5k2l7_reg s5k2l7_mode_4032x3024[] = {
	{ 0x602a, 0x0bd8 }, { 0x6f12, 0x06e0 }, { 0x602a, 0x357c }, { 0x6f12, 0x0000 },
	{ 0x602a, 0x4040 }, { 0x6f12, 0x0000 }, { 0x602a, 0x4b04 }, { 0x6f12, 0x0000 },
	{ 0x6028, 0x4000 }, { 0x6214, 0x7970 }, { 0x6218, 0x7150 }, { 0x0344, 0x0000 },
	{ 0x0346, 0x0000 }, { 0x0348, 0x1fbf }, { 0x034a, 0x0bdf }, { 0x034c, 0x0fc0 },
	{ 0x034e, 0x0bd0 }, { 0x0408, 0x0010 }, { 0x040a, 0x0008 }, { 0x0900, 0x0011 },
	{ 0x0380, 0x0001 }, { 0x0382, 0x0001 }, { 0x0384, 0x0001 }, { 0x0386, 0x0001 },
	{ 0x0400, 0x0000 }, { 0x0404, 0x0010 }, { 0x3060, 0x0000 }, { 0x0114, 0x0300 },
	{ 0x0110, 0x1002 }, { 0x0136, 0x1800 }, { 0x0304, 0x0006 }, { 0x0306, 0x01e0 },
	{ 0x0302, 0x0001 }, { 0x0300, 0x0004 }, { 0x030c, 0x0001 }, { 0x030e, 0x0004 },
	{ 0x0310, 0x00de }, { 0x0312, 0x0000 }, { 0x030a, 0x0001 }, { 0x0308, 0x0008 },
	{ 0x0342, 0x27b0 }, { 0x0340, 0x0c5a }, { 0x021e, 0x0000 }, { 0x3098, 0x0300 },
	{ 0x309a, 0x0102 }, { 0x30bc, 0x0131 }, { 0x30a6, 0x0010 }, { 0x30a8, 0x0002 },
	{ 0x30aa, 0x0fc0 }, { 0x30ac, 0x02f4 }, { 0x30a0, 0x0000 }, { 0x30a4, 0x0000 },
	{ 0x6a0c, 0xffff }, { 0xf41e, 0x2180 }, { 0x6028, 0x2000 }, { 0x602a, 0x0990 },
	{ 0x6f12, 0x0040 }, { 0x602a, 0x0af8 }, { 0x6f12, 0x0004 }, { 0x602a, 0x27b8 },
	{ 0x6f12, 0x0100 }, { 0x602a, 0x2aa8 }, { 0x6f12, 0x0001 }, { 0x602a, 0x1698 },
	{ 0x6f12, 0x00a2 }, { 0x602a, 0x169c }, { 0x6f12, 0x0028 }, { 0x6f12, 0x0030 },
	{ 0x6f12, 0x0c18 }, { 0x6f12, 0x0c18 }, { 0x6f12, 0x0c28 }, { 0x6f12, 0x0c28 },
	{ 0x6f12, 0x0c20 }, { 0x6f12, 0x0c20 }, { 0x6f12, 0x0c30 }, { 0x6f12, 0x0c30 },
	{ 0x602a, 0x16c6 }, { 0x6f12, 0x122f }, { 0x6f12, 0x4328 }, { 0x602a, 0x1604 },
	{ 0x6f12, 0x0004 }, { 0x602a, 0x14ca }, { 0x6f12, 0x0096 }, { 0x602a, 0x16ce },
	{ 0x6f12, 0x06c0 }, { 0x6f12, 0x0101 }, { 0x602a, 0x13aa }, { 0x6f12, 0x02a0 },
	{ 0x602a, 0x13a2 }, { 0x6f12, 0x028c }, { 0x602a, 0x139a }, { 0x6f12, 0x0278 },
	{ 0x602a, 0x1392 }, { 0x6f12, 0x0264 }, { 0x602a, 0x138a }, { 0x6f12, 0x0250 },
	{ 0x602a, 0x1382 }, { 0x6f12, 0x023c }, { 0x602a, 0x137a }, { 0x6f12, 0x0228 },
	{ 0x602a, 0x1372 }, { 0x6f12, 0x0214 }, { 0x602a, 0x136a }, { 0x6f12, 0x0200 },
	{ 0x602a, 0x1362 }, { 0x6f12, 0x01ec }, { 0x602a, 0x135a }, { 0x6f12, 0x01d8 },
	{ 0x602a, 0x1352 }, { 0x6f12, 0x01c4 }, { 0x602a, 0x134a }, { 0x6f12, 0x01b0 },
	{ 0x602a, 0x1342 }, { 0x6f12, 0x029b }, { 0x602a, 0x133a }, { 0x6f12, 0x0291 },
	{ 0x602a, 0x1332 }, { 0x6f12, 0x0287 }, { 0x602a, 0x132a }, { 0x6f12, 0x027d },
	{ 0x602a, 0x1322 }, { 0x6f12, 0x0273 }, { 0x602a, 0x131a }, { 0x6f12, 0x0269 },
	{ 0x602a, 0x1312 }, { 0x6f12, 0x025f }, { 0x602a, 0x130a }, { 0x6f12, 0x0255 },
	{ 0x602a, 0x1302 }, { 0x6f12, 0x024b }, { 0x602a, 0x12fa }, { 0x6f12, 0x0241 },
	{ 0x602a, 0x12f2 }, { 0x6f12, 0x0237 }, { 0x602a, 0x12ea }, { 0x6f12, 0x022d },
	{ 0x602a, 0x12e2 }, { 0x6f12, 0x0223 }, { 0x602a, 0x12da }, { 0x6f12, 0x0219 },
	{ 0x602a, 0x12d2 }, { 0x6f12, 0x020f }, { 0x602a, 0x12ca }, { 0x6f12, 0x0205 },
	{ 0x602a, 0x12c2 }, { 0x6f12, 0x01fb }, { 0x602a, 0x12ba }, { 0x6f12, 0x01f1 },
	{ 0x602a, 0x12b2 }, { 0x6f12, 0x01e7 }, { 0x602a, 0x12aa }, { 0x6f12, 0x01dd },
	{ 0x602a, 0x12a2 }, { 0x6f12, 0x01d3 }, { 0x602a, 0x129a }, { 0x6f12, 0x01c9 },
	{ 0x602a, 0x1292 }, { 0x6f12, 0x01bf }, { 0x602a, 0x128a }, { 0x6f12, 0x01b5 },
	{ 0x602a, 0x11e2 }, { 0x6f12, 0x0258 }, { 0x602a, 0x0c32 }, { 0x6f12, 0x000a },
	{ 0x602a, 0x0b16 }, { 0x6f12, 0x0200 }, { 0x602a, 0x0c20 }, { 0x6f12, 0x0361 },
	{ 0x602a, 0x0d78 }, { 0x6f12, 0x0361 }, { 0x602a, 0x0d98 }, { 0x6f12, 0x0361 },
	{ 0x602a, 0x0da8 }, { 0x6f12, 0x0361 }, { 0x602a, 0x0ae4 }, { 0x6f12, 0x0fe0 },
	{ 0x602a, 0x29e6 }, { 0x6f12, 0x0fe0 }, { 0x602a, 0x29f0 }, { 0x6f12, 0x0fe0 },
	{ 0x602a, 0x2966 }, { 0x6f12, 0x0fe0 }, { 0x602a, 0x29a6 }, { 0x6f12, 0x0fe0 },
	{ 0x602a, 0x2970 }, { 0x6f12, 0x0fe0 }, { 0x602a, 0x29b0 }, { 0x6f12, 0x0fe0 },
	{ 0x602a, 0x0ae6 }, { 0x6f12, 0x0be0 }, { 0x602a, 0x29e8 }, { 0x6f12, 0x0be0 },
	{ 0x602a, 0x29f2 }, { 0x6f12, 0x0be0 }, { 0x602a, 0x2968 }, { 0x6f12, 0x0be0 },
	{ 0x602a, 0x29a8 }, { 0x6f12, 0x0be0 }, { 0x602a, 0x2972 }, { 0x6f12, 0x0be0 },
	{ 0x602a, 0x29b2 }, { 0x6f12, 0x0be0 }, { 0x602a, 0x0bb0 }, { 0x6f12, 0x0800 },
	{ 0x602a, 0x0b62 }, { 0x6f12, 0x0142 }, { 0x602a, 0x5160 }, { 0x6f12, 0x0000 },
	{ 0x602a, 0x09f2 }, { 0x6f12, 0x0180 }, { 0x6f12, 0x2180 }, { 0x6f12, 0xf41e },
	{ 0x602a, 0x0b06 }, { 0x6f12, 0x0800 }, { 0x602a, 0x5694 }, { 0x6f12, 0xffc0 },
	{ 0x602a, 0x569c }, { 0x6f12, 0x010a }, { 0x6f12, 0x0000 }, { 0x6f12, 0x0240 },
	{ 0x6f12, 0x0000 }, { 0x602a, 0x56d8 }, { 0x6f12, 0xffc0 }, { 0x602a, 0x56e0 },
	{ 0x6f12, 0x010a }, { 0x6f12, 0x0000 }, { 0x6f12, 0x0240 }, { 0x6f12, 0x0000 },
	{ 0x6028, 0x4000 }, { 0x6214, 0x79f0 }, { 0x6218, 0x79f0 },
};

/*
 * 4032x2272 (16:9): full-width readout window cropped vertically.
 * Same PLL as 4032x3024; delta against that table: window and output
 * size, frame length and the readout-area dependent burst registers.
 */
static const struct s5k2l7_reg s5k2l7_mode_4032x2272[] = {
	{ 0x0346, 0x0170 }, { 0x034a, 0x0a5f }, { 0x034e, 0x08e0 }, { 0x0340, 0x0c4d },
	{ 0x30ac, 0x0238 }, { 0x6028, 0x2000 }, { 0x602a, 0x0ae6 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x29e8 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29f2 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x2968 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29a8 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x2972 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29b2 }, { 0x6f12, 0x0bd0 },
	{ 0x6028, 0x4000 },
};

/*
 * 2752x2064 (4:3): centre crop of the full array, no binning.
 * Same PLL as 4032x3024; delta against that table: window and output
 * size and the readout-area dependent burst registers.
 */
static const struct s5k2l7_reg s5k2l7_mode_2752x2064[] = {
	{ 0x0344, 0x04e0 }, { 0x0346, 0x01d8 }, { 0x0348, 0x1a9f }, { 0x034a, 0x09f7 },
	{ 0x034c, 0x0ac0 }, { 0x034e, 0x0810 }, { 0x30aa, 0x0ac0 }, { 0x30ac, 0x0204 },
	{ 0x6028, 0x2000 }, { 0x602a, 0x0ae4 }, { 0x6f12, 0x0fc0 }, { 0x602a, 0x29e6 },
	{ 0x6f12, 0x0fc0 }, { 0x602a, 0x29f0 }, { 0x6f12, 0x0fc0 }, { 0x602a, 0x2966 },
	{ 0x6f12, 0x0fc0 }, { 0x602a, 0x29a6 }, { 0x6f12, 0x0fc0 }, { 0x602a, 0x2970 },
	{ 0x6f12, 0x0fc0 }, { 0x602a, 0x29b0 }, { 0x6f12, 0x0fc0 }, { 0x602a, 0x0ae6 },
	{ 0x6f12, 0x0bd0 }, { 0x602a, 0x29e8 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29f2 },
	{ 0x6f12, 0x0bd0 }, { 0x602a, 0x2968 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29a8 },
	{ 0x6f12, 0x0bd0 }, { 0x602a, 0x2972 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29b2 },
	{ 0x6f12, 0x0bd0 }, { 0x602a, 0x0b62 }, { 0x6f12, 0x00ee }, { 0x6028, 0x4000 },
};

/*
 * 2016x1512 (4:3): 2x2 binned full array. Output PLL changes to 702 Mbps
 * per lane (0x310 = 0xea, 0x312 = 1). Delta against the 4032x3024 table:
 * binning, increments, output size, frame length, PLL and burst registers.
 */
static const struct s5k2l7_reg s5k2l7_mode_2016x1512[] = {
	{ 0x034c, 0x07e0 }, { 0x034e, 0x05e8 }, { 0x0408, 0x0008 }, { 0x040a, 0x0004 },
	{ 0x0900, 0x0122 }, { 0x0382, 0x0003 }, { 0x0386, 0x0003 }, { 0x0310, 0x00ea },
	{ 0x0312, 0x0001 }, { 0x0340, 0x0c4d }, { 0x30a6, 0x0008 }, { 0x30a8, 0x0000 },
	{ 0x30aa, 0x07e0 }, { 0x30ac, 0x0000 }, { 0x30a0, 0x0001 }, { 0x30a4, 0x017a },
	{ 0x6028, 0x2000 }, { 0x602a, 0x0af8 }, { 0x6f12, 0x0008 }, { 0x602a, 0x27b8 },
	{ 0x6f12, 0x0000 }, { 0x602a, 0x1698 }, { 0x6f12, 0x00a0 }, { 0x602a, 0x16c6 },
	{ 0x6f12, 0x12af }, { 0x6f12, 0x0328 }, { 0x602a, 0x0b16 }, { 0x6f12, 0x0100 },
	{ 0x602a, 0x0b62 }, { 0x6f12, 0x0166 }, { 0x602a, 0x5160 }, { 0x6f12, 0x0100 },
	{ 0x602a, 0x5694 }, { 0x6f12, 0x0000 }, { 0x602a, 0x569c }, { 0x6f12, 0x000c },
	{ 0x6f12, 0x0100 }, { 0x6f12, 0x0200 }, { 0x6f12, 0x0100 }, { 0x602a, 0x56d8 },
	{ 0x6f12, 0x0000 }, { 0x602a, 0x56e0 }, { 0x6f12, 0x000c }, { 0x6f12, 0x0100 },
	{ 0x6f12, 0x0200 }, { 0x6f12, 0x0100 }, { 0x6028, 0x4000 },
};

/*
 * 2016x1136 (16:9): 2x2 binned, vertically cropped; 60 fps. Same PLL as
 * 2016x1512 (702 Mbps per lane). Delta against the 4032x3024 table:
 * binning, increments, window, output size, frame length, PLL and bursts.
 */
static const struct s5k2l7_reg s5k2l7_mode_2016x1136[] = {
	{ 0x0346, 0x0170 }, { 0x034a, 0x0a5f }, { 0x034c, 0x07e0 }, { 0x034e, 0x0470 },
	{ 0x0408, 0x0008 }, { 0x040a, 0x0004 }, { 0x0900, 0x0122 }, { 0x0382, 0x0003 },
	{ 0x0386, 0x0003 }, { 0x0310, 0x00ea }, { 0x0312, 0x0001 }, { 0x0340, 0x0626 },
	{ 0x30a6, 0x0008 }, { 0x30a8, 0x0000 }, { 0x30aa, 0x07e0 }, { 0x30ac, 0x0000 },
	{ 0x30a0, 0x0001 }, { 0x30a4, 0x011c }, { 0x6028, 0x2000 }, { 0x602a, 0x0af8 },
	{ 0x6f12, 0x0008 }, { 0x602a, 0x27b8 }, { 0x6f12, 0x0000 }, { 0x602a, 0x1698 },
	{ 0x6f12, 0x00a0 }, { 0x602a, 0x16c6 }, { 0x6f12, 0x12af }, { 0x6f12, 0x0328 },
	{ 0x602a, 0x0b16 }, { 0x6f12, 0x0100 }, { 0x602a, 0x0ae6 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x29e8 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29f2 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x2968 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29a8 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x2972 }, { 0x6f12, 0x0bd0 }, { 0x602a, 0x29b2 }, { 0x6f12, 0x0bd0 },
	{ 0x602a, 0x0b62 }, { 0x6f12, 0x0166 }, { 0x602a, 0x5160 }, { 0x6f12, 0x0100 },
	{ 0x6028, 0x4000 },
};

/*
 * Link frequency (DDR, so the lane rate is twice this) and pixel rate on the
 * CSI-2 bus: pixel rate = link * 2 * lanes / 10 bits.
 */
static const s64 s5k2l7_link_freqs[] = {
	666000000,	/* 1332 Mbps per lane */
	351000000,	/*  702 Mbps per lane */
};

struct s5k2l7_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	u8 link_freq_idx;
	struct v4l2_rect crop;
	/*
	 * Registers applied on top of the full-resolution table; NULL for the
	 * full-resolution mode itself.
	 */
	const struct s5k2l7_reg *regs;
	unsigned int num_regs;
};

static const struct s5k2l7_mode s5k2l7_modes[] = {
	{
		.width = 4032,
		.height = 3024,
		.hts = 10160,
		.vts_def = 3162,
		.link_freq_idx = 0,
		.crop = { 0, 0, 4032, 3024 },
	},
	{
		.width = 4032,
		.height = 2272,
		.hts = 10160,
		.vts_def = 3149,
		.link_freq_idx = 0,
		.crop = { 0, 368, 4032, 2272 },
		.regs = s5k2l7_mode_4032x2272,
		.num_regs = ARRAY_SIZE(s5k2l7_mode_4032x2272),
	},
	{
		.width = 2752,
		.height = 2064,
		.hts = 10160,
		.vts_def = 3162,
		.link_freq_idx = 0,
		.crop = { 624, 472, 2752, 2064 },
		.regs = s5k2l7_mode_2752x2064,
		.num_regs = ARRAY_SIZE(s5k2l7_mode_2752x2064),
	},
	{
		.width = 2016,
		.height = 1512,
		.hts = 10160,
		.vts_def = 3149,
		.link_freq_idx = 1,
		.crop = { 0, 0, 4032, 3024 },
		.regs = s5k2l7_mode_2016x1512,
		.num_regs = ARRAY_SIZE(s5k2l7_mode_2016x1512),
	},
	{
		.width = 2016,
		.height = 1136,
		.hts = 10160,
		.vts_def = 1574,
		.link_freq_idx = 1,
		.crop = { 0, 368, 4032, 2272 },
		.regs = s5k2l7_mode_2016x1136,
		.num_regs = ARRAY_SIZE(s5k2l7_mode_2016x1136),
	},
};

static const char * const s5k2l7_supply_names[] = {
	"dovdd",	/* 1.8 V */
	"dvdd",		/* ~1.0-1.2 V */
	"avdd",		/* 2.8 V */
};
#define S5K2L7_NUM_SUPPLIES ARRAY_SIZE(s5k2l7_supply_names)

struct s5k2l7 {
	struct i2c_client *client;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K2L7_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;

	const struct s5k2l7_mode *cur_mode;
	bool streaming;
};

static inline struct s5k2l7 *to_s5k2l7(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k2l7, sd);
}

static int s5k2l7_write_table(struct s5k2l7 *s, const struct s5k2l7_reg *t,
			      unsigned int n)
{
	int ret = 0;
	unsigned int i;

	for (i = 0; i < n && !ret; i++)
		cci_write(s->regmap, CCI_REG16(t[i].addr), t[i].val, &ret);
	return ret;
}

static int s5k2l7_apply_tables(struct s5k2l7 *s)
{
	int ret = 0;

	cci_write(s->regmap, S5K2L7_REG_PAGE, 0x2000, &ret);
	cci_write(s->regmap, S5K2L7_REG_ORIENTATION, 0x00, &ret);
	if (ret)
		return ret;
	msleep(46);

	ret = s5k2l7_write_table(s, s5k2l7_init_regs,
				 ARRAY_SIZE(s5k2l7_init_regs));
	if (ret)
		return ret;
	ret = s5k2l7_write_table(s, s5k2l7_mode_4032x3024,
				 ARRAY_SIZE(s5k2l7_mode_4032x3024));
	if (ret || !s->cur_mode->regs)
		return ret;
	return s5k2l7_write_table(s, s->cur_mode->regs,
				  s->cur_mode->num_regs);
}

static int s5k2l7_power_on(struct device *dev)
{
	struct s5k2l7 *s = to_s5k2l7(dev_get_drvdata(dev));
	int ret;

	gpiod_set_value_cansleep(s->reset_gpio, 1);

	/* Power-up: assert reset, enable rails, then MCLK. */
	ret = regulator_bulk_enable(S5K2L7_NUM_SUPPLIES, s->supplies);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	ret = clk_prepare_enable(s->xclk);
	if (ret) {
		regulator_bulk_disable(S5K2L7_NUM_SUPPLIES, s->supplies);
		return ret;
	}
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(s->reset_gpio, 0);
	usleep_range(11000, 12000);
	return 0;
}

static int s5k2l7_power_off(struct device *dev)
{
	struct s5k2l7 *s = to_s5k2l7(dev_get_drvdata(dev));

	clk_disable_unprepare(s->xclk);
	gpiod_set_value_cansleep(s->reset_gpio, 1);
	regulator_bulk_disable(S5K2L7_NUM_SUPPLIES, s->supplies);
	return 0;
}

static int s5k2l7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k2l7 *s = container_of(ctrl->handler, struct s5k2l7, ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = ctrl->val + s->cur_mode->height - S5K2L7_EXP_MARGIN;

		__v4l2_ctrl_modify_range(s->exposure, S5K2L7_EXP_MIN, max, 1,
					 min_t(s64, s->exposure->val, max));
	}

	if (!pm_runtime_get_if_in_use(&s->client->dev))
		return 0;

	cci_write(s->regmap, S5K2L7_REG_GROUP_HOLD, 1, &ret);
	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(s->regmap, S5K2L7_REG_ANALOG_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(s->regmap, S5K2L7_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(s->regmap, S5K2L7_REG_VTS,
			  ctrl->val + s->cur_mode->height, &ret);
		break;
	}
	cci_write(s->regmap, S5K2L7_REG_GROUP_HOLD, 0, &ret);

	pm_runtime_put(&s->client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5k2l7_ctrl_ops = {
	.s_ctrl = s5k2l7_set_ctrl,
};

static u64 s5k2l7_mode_pixel_rate(const struct s5k2l7_mode *mode)
{
	return s5k2l7_link_freqs[mode->link_freq_idx] * 2 * S5K2L7_LANES / 10;
}

static int s5k2l7_init_controls(struct s5k2l7 *s)
{
	const struct s5k2l7_mode *mode = s->cur_mode;
	struct v4l2_fwnode_device_properties props;
	u64 pixrate = s5k2l7_mode_pixel_rate(mode);
	u32 hblank = mode->hts - mode->width;
	int ret;

	v4l2_ctrl_handler_init(&s->ctrls, 9);

	s->link_freq = v4l2_ctrl_new_int_menu(&s->ctrls, NULL,
					      V4L2_CID_LINK_FREQ,
					      ARRAY_SIZE(s5k2l7_link_freqs) - 1,
					      mode->link_freq_idx,
					      s5k2l7_link_freqs);
	if (s->link_freq)
		s->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s->pixel_rate = v4l2_ctrl_new_std(&s->ctrls, NULL,
					  V4L2_CID_PIXEL_RATE, pixrate,
					  pixrate, 1, pixrate);

	s->hblank = v4l2_ctrl_new_std(&s->ctrls, NULL, V4L2_CID_HBLANK,
				      hblank, hblank, 1, hblank);
	if (s->hblank)
		s->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s->vblank = v4l2_ctrl_new_std(&s->ctrls, &s5k2l7_ctrl_ops,
				      V4L2_CID_VBLANK, 0,
				      S5K2L7_VTS_MAX - mode->height, 1,
				      mode->vts_def - mode->height);

	s->exposure = v4l2_ctrl_new_std(&s->ctrls, &s5k2l7_ctrl_ops,
					V4L2_CID_EXPOSURE, S5K2L7_EXP_MIN,
					mode->vts_def - S5K2L7_EXP_MARGIN, 1,
					(mode->vts_def - S5K2L7_EXP_MARGIN) / 2);

	v4l2_ctrl_new_std(&s->ctrls, &s5k2l7_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K2L7_GAIN_MIN, S5K2L7_GAIN_MAX, 1, S5K2L7_GAIN_DEF);

	if (s->ctrls.error) {
		ret = s->ctrls.error;
		goto err;
	}
	ret = v4l2_fwnode_device_parse(&s->client->dev, &props);
	if (ret)
		goto err;
	ret = v4l2_ctrl_new_fwnode_properties(&s->ctrls, &s5k2l7_ctrl_ops,
					      &props);
	if (ret)
		goto err;

	s->sd.ctrl_handler = &s->ctrls;
	return 0;
err:
	v4l2_ctrl_handler_free(&s->ctrls);
	return ret;
}

static const struct s5k2l7_mode *
s5k2l7_find_mode(u32 width, u32 height)
{
	return v4l2_find_nearest_size(s5k2l7_modes, ARRAY_SIZE(s5k2l7_modes),
				      width, height, width, height);
}

static void s5k2l7_fill_fmt(const struct s5k2l7_mode *mode,
			    struct v4l2_mbus_framefmt *f)
{
	f->width = mode->width;
	f->height = mode->height;
	f->code = S5K2L7_MBUS_CODE;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = V4L2_COLORSPACE_RAW;
	f->ycbcr_enc = V4L2_YCBCR_ENC_601;
	f->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	f->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k2l7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *st,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = S5K2L7_MBUS_CODE;
	return 0;
}

static int s5k2l7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *st,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k2l7_modes) ||
	    fse->code != S5K2L7_MBUS_CODE)
		return -EINVAL;
	fse->min_width = s5k2l7_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k2l7_modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int s5k2l7_set_mode_ctrls(struct s5k2l7 *s,
				 const struct s5k2l7_mode *mode)
{
	u64 pixrate = s5k2l7_mode_pixel_rate(mode);
	u32 hblank = mode->hts - mode->width;
	int ret;

	/* The VBLANK and exposure handlers read cur_mode. */
	s->cur_mode = mode;

	ret = __v4l2_ctrl_s_ctrl(s->link_freq, mode->link_freq_idx);
	if (ret)
		return ret;
	ret = __v4l2_ctrl_modify_range(s->pixel_rate, pixrate, pixrate, 1,
				       pixrate);
	if (ret)
		return ret;
	ret = __v4l2_ctrl_modify_range(s->hblank, hblank, hblank, 1, hblank);
	if (ret)
		return ret;
	ret = __v4l2_ctrl_modify_range(s->vblank, 0,
				       S5K2L7_VTS_MAX - mode->height, 1,
				       mode->vts_def - mode->height);
	if (ret)
		return ret;
	ret = __v4l2_ctrl_s_ctrl(s->vblank, mode->vts_def - mode->height);
	if (ret)
		return ret;
	return __v4l2_ctrl_modify_range(s->exposure, S5K2L7_EXP_MIN,
					mode->vts_def - S5K2L7_EXP_MARGIN, 1,
					(mode->vts_def - S5K2L7_EXP_MARGIN) / 2);
}

static int s5k2l7_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *st,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k2l7 *s = to_s5k2l7(sd);
	const struct s5k2l7_mode *mode;

	mode = s5k2l7_find_mode(fmt->format.width, fmt->format.height);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		int ret;

		if (s->streaming)
			return -EBUSY;
		ret = s5k2l7_set_mode_ctrls(s, mode);
		if (ret)
			return ret;
	}

	s5k2l7_fill_fmt(mode, &fmt->format);
	*v4l2_subdev_state_get_format(st, fmt->pad) = fmt->format;
	return 0;
}

static int s5k2l7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *st)
{
	s5k2l7_fill_fmt(&s5k2l7_modes[0], v4l2_subdev_state_get_format(st, 0));
	return 0;
}

static int s5k2l7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *st,
				struct v4l2_subdev_selection *sel)
{
	const struct v4l2_mbus_framefmt *fmt;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		fmt = v4l2_subdev_state_get_format(st, sel->pad);
		sel->r = s5k2l7_find_mode(fmt->width, fmt->height)->crop;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = (struct v4l2_rect){ 0, 0, S5K2L7_WIDTH, S5K2L7_HEIGHT };
		return 0;
	}
	return -EINVAL;
}

static int s5k2l7_start_streaming(struct s5k2l7 *s)
{
	int ret;

	ret = pm_runtime_resume_and_get(&s->client->dev);
	if (ret < 0)
		return ret;

	ret = s5k2l7_apply_tables(s);
	if (ret)
		goto err_pm;

	cci_write(s->regmap, S5K2L7_REG_DGAIN_GR, S5K2L7_DGAIN_1X, &ret);
	cci_write(s->regmap, S5K2L7_REG_DGAIN_R, S5K2L7_DGAIN_1X, &ret);
	cci_write(s->regmap, S5K2L7_REG_DGAIN_B, S5K2L7_DGAIN_1X, &ret);
	cci_write(s->regmap, S5K2L7_REG_DGAIN_GB, S5K2L7_DGAIN_1X, &ret);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(&s->ctrls);
	if (ret)
		goto err_pm;

	ret = cci_write(s->regmap, S5K2L7_REG_MODE_SELECT, 1, NULL);
	if (ret)
		goto err_pm;

	s->streaming = true;
	return 0;
err_pm:
	pm_runtime_put(&s->client->dev);
	return ret;
}

static int s5k2l7_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k2l7 *s = to_s5k2l7(sd);
	struct v4l2_subdev_state *st;
	int ret = 0;

	st = v4l2_subdev_lock_and_get_active_state(sd);
	if (enable) {
		ret = s5k2l7_start_streaming(s);
	} else {
		cci_write(s->regmap, S5K2L7_REG_MODE_SELECT, 0, NULL);
		s->streaming = false;
		pm_runtime_put(&s->client->dev);
	}
	v4l2_subdev_unlock_state(st);
	return ret;
}

static const struct v4l2_subdev_video_ops s5k2l7_video_ops = {
	.s_stream = s5k2l7_set_stream,
};

static const struct v4l2_subdev_pad_ops s5k2l7_pad_ops = {
	.enum_mbus_code = s5k2l7_enum_mbus_code,
	.enum_frame_size = s5k2l7_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k2l7_set_fmt,
	.get_selection = s5k2l7_get_selection,
};

static const struct v4l2_subdev_ops s5k2l7_subdev_ops = {
	.video = &s5k2l7_video_ops,
	.pad = &s5k2l7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k2l7_internal_ops = {
	.init_state = s5k2l7_init_state,
};

static const struct media_entity_operations s5k2l7_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k2l7_parse_dt(struct s5k2l7 *s)
{
	struct device *dev = &s->client->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!fwnode)
		return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return ret;
	if (ep.bus.mipi_csi2.num_data_lanes != S5K2L7_LANES)
		ret = dev_err_probe(dev, -EINVAL, "need %d data lanes\n",
				    S5K2L7_LANES);
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

 /* Probe :) */
static int s5k2l7_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k2l7 *s;
	u64 id;
	unsigned int i;
	int ret = 0;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->client = client;

	s->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s->regmap))
		return PTR_ERR(s->regmap);

	ret = s5k2l7_parse_dt(s);
	if (ret)
		return dev_err_probe(dev, ret, "bad endpoint\n");

	s->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->xclk))
		return dev_err_probe(dev, PTR_ERR(s->xclk), "no clock\n");
	ret = clk_set_rate(s->xclk, S5K2L7_XCLK_FREQ);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set 24 MHz\n");

	s->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s->reset_gpio))
		return PTR_ERR(s->reset_gpio);

	for (i = 0; i < S5K2L7_NUM_SUPPLIES; i++)
		s->supplies[i].supply = s5k2l7_supply_names[i];
	ret = devm_regulator_bulk_get(dev, S5K2L7_NUM_SUPPLIES, s->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	s->cur_mode = &s5k2l7_modes[0];

	v4l2_i2c_subdev_init(&s->sd, client, &s5k2l7_subdev_ops);
	s->sd.internal_ops = &s5k2l7_internal_ops;

	ret = s5k2l7_power_on(dev);
	if (ret)
		return ret;

	ret = 0;
	cci_read(s->regmap, S5K2L7_REG_CHIP_ID, &id, &ret);
	if (ret) {
		dev_err_probe(dev, ret, "chip id read failed\n");
		goto err_power;
	}
	if (id != S5K2L7_CHIP_ID) {
		ret = dev_err_probe(dev, -ENODEV, "wrong chip id 0x%04llx\n", id);
		goto err_power;
	}

	ret = s5k2l7_init_controls(s);
	if (ret)
		goto err_power;

	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->sd.entity.ops = &s5k2l7_entity_ops;
	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret)
		goto err_ctrls;

	s->sd.state_lock = s->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (ret)
		goto err_pm;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	v4l2_subdev_cleanup(&s->sd);
err_entity:
	media_entity_cleanup(&s->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s->ctrls);
err_power:
	s5k2l7_power_off(dev);
	return ret;
}

static void s5k2l7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k2l7 *s = to_s5k2l7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k2l7_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k2l7_pm_ops, s5k2l7_power_off,
				 s5k2l7_power_on, NULL);

static const struct of_device_id s5k2l7_of_match[] = {
	{ .compatible = "samsung,s5k2l7" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k2l7_of_match);

static struct i2c_driver s5k2l7_i2c_driver = {
	.driver = {
		.name = "s5k2l7",
		.pm = pm_ptr(&s5k2l7_pm_ops),
		.of_match_table = s5k2l7_of_match,
	},
	.probe = s5k2l7_probe,
	.remove = s5k2l7_remove,
};
module_i2c_driver(s5k2l7_i2c_driver);


MODULE_AUTHOR("Kulesha Evgeniy <voovdop@gmail.com>");
MODULE_DESCRIPTION("Samsung S5K2L7 sensor driver");
MODULE_LICENSE("GPL");
