// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K5E8 image sensor driver.
 *
 * Copyright (C) 2025 Gianluca Boiano <morf3089@gmail.com>
 *
 * The register settings are derived from the vendor camera module library
 * (libmmcamera_tulip_s5k5e8_*.so) of the Xiaomi Redmi Note 6 Pro (tulip).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/*
 * The PLL settings of the register tables below are only valid for a 24 MHz
 * input clock. With EXTCLK = 24 MHz (0x0136/0x0137 = 0x1800, 8.8 fixed point
 * MHz) they result in:
 *
 *  - 0x0305 pre_pll_clk_div = 6, so the PLL input is 24 / 6 = 4 MHz.
 *  - 0x0307 pll_multiplier = 0xa8 = 168, which gives a 168 MHz video timing
 *    pixel clock. 0x0306 is 0x18 in all vendor settings; the vendor
 *    settings for the other pixel clocks (0xa9 = 169 MHz, 0x8d = 141 MHz)
 *    only differ in 0x0307, and the pixel clock follows it 1:1 in MHz.
 *    The dividers behind it (0x0308/0x0309) are not documented, so the
 *    pixel clock is taken from the vendor out_info (vt_pixel_clk) and
 *    cross-checked with the frame timing:
 *      2592x1944: 2856 * 1968 * 29.89 fps = 168.0 MHz
 *      1296x972:  5200 * 1062 * 30.42 fps = 168.0 MHz
 *  - 0x0820/0x0821 requested link bit rate = 0x0344 = 836 Mbps per lane.
 *    With two lanes and 10 bits per pixel this is 2 * 836 / 10 = 167.2 MHz,
 *    which equals the vendor op_pixel_clk. The D-PHY clock runs at half the
 *    bit rate, so the link frequency is 418 MHz.
 *
 * Both modes use the same PLL settings, hence there is a single link
 * frequency and pixel rate.
 */
#define S5K5E8_EXTCLK_FREQ		(24 * HZ_PER_MHZ)
#define S5K5E8_LINK_FREQ		(418 * HZ_PER_MHZ)
#define S5K5E8_PIXEL_RATE		(168 * HZ_PER_MHZ)
#define S5K5E8_DATA_LANES		2

#define S5K5E8_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K5E8_CHIP_ID			0x5e80

#define S5K5E8_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K5E8_MODE_STANDBY		0x00
#define S5K5E8_MODE_STREAMING		0x01

/* Image orientation, bit 0 is the horizontal and bit 1 the vertical flip */
#define S5K5E8_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K5E8_HFLIP			BIT(0)
#define S5K5E8_VFLIP			BIT(1)

#define S5K5E8_REG_VTS			CCI_REG16(0x0340)
#define S5K5E8_VTS_MAX			0xffff

#define S5K5E8_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K5E8_EXPOSURE_MIN		2
#define S5K5E8_EXPOSURE_STEP		1
#define S5K5E8_EXPOSURE_DEFAULT		0x03de
/* Vendor vert_offset: the exposure can be at most VTS - 6 lines */
#define S5K5E8_EXPOSURE_MARGIN		6

/* Analogue gain, 0x20 = 1x, 1/32 steps up to 16x */
#define S5K5E8_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define S5K5E8_ANA_GAIN_MIN		0x20
#define S5K5E8_ANA_GAIN_MAX		0x200
#define S5K5E8_ANA_GAIN_STEP		1
#define S5K5E8_ANA_GAIN_DEFAULT		0x20

#define S5K5E8_REG_TEST_PATTERN		CCI_REG8(0x0601)

/* Registers used by the stream on sequence */
#define S5K5E8_REG_STREAM_ON_FIXUP	CCI_REG8(0x3c0d)

/*
 * The modes read the area from (8, 8) to (2599, 1951) of the pixel array,
 * which is 2592x1944 pixels, the pixel array size reported by the vendor
 * library. The registers only show that the array is at least 2600x1952; the
 * native size below assumes a symmetric 8 pixel border on each side.
 */
#define S5K5E8_NATIVE_WIDTH		2608U
#define S5K5E8_NATIVE_HEIGHT		1960U
#define S5K5E8_PIXEL_ARRAY_LEFT		8U
#define S5K5E8_PIXEL_ARRAY_TOP		8U
#define S5K5E8_PIXEL_ARRAY_WIDTH	2592U
#define S5K5E8_PIXEL_ARRAY_HEIGHT	1944U

/* Time to wait after the init settings have been written */
#define S5K5E8_INIT_DELAY_MS		60

/* Vendor init settings, written after every power up */
static const struct cci_reg_sequence s5k5e8_init_regs[] = {
	{ CCI_REG8(0x3906), 0x7e },
	{ CCI_REG8(0x3c01), 0x0f },
	{ CCI_REG8(0x3c14), 0x00 },
	{ CCI_REG8(0x3235), 0x08 },
	{ CCI_REG8(0x3063), 0x2e },
	{ CCI_REG8(0x307a), 0x10 },
	{ CCI_REG8(0x307b), 0x0e },
	{ CCI_REG8(0x3079), 0x20 },
	{ CCI_REG8(0x3070), 0x05 },
	{ CCI_REG8(0x3067), 0x06 },
	{ CCI_REG8(0x3071), 0x62 },
	{ CCI_REG8(0x3203), 0x43 },
	{ CCI_REG8(0x3205), 0x43 },
	{ CCI_REG8(0x320b), 0x42 },
	{ CCI_REG8(0x3007), 0x00 },
	{ CCI_REG8(0x3008), 0x14 },
	{ CCI_REG8(0x3020), 0x58 },
	{ CCI_REG8(0x300d), 0x34 },
	{ CCI_REG8(0x300e), 0x17 },
	{ CCI_REG8(0x3021), 0x02 },
	{ CCI_REG8(0x3010), 0x59 },
	{ CCI_REG8(0x3002), 0x01 },
	{ CCI_REG8(0x3005), 0x01 },
	{ CCI_REG8(0x3008), 0x04 },
	{ CCI_REG8(0x300f), 0x70 },
	{ CCI_REG8(0x3010), 0x69 },
	{ CCI_REG8(0x3017), 0x10 },
	{ CCI_REG8(0x3019), 0x19 },
	{ CCI_REG8(0x300c), 0x62 },
	{ CCI_REG8(0x3064), 0x10 },
	{ CCI_REG8(0x3c08), 0x0e },
	{ CCI_REG8(0x3c09), 0x10 },
	{ CCI_REG8(0x3c31), 0x0d },
	{ CCI_REG8(0x3c32), 0xac },
	{ CCI_REG8(0x3929), 0x07 },
	{ CCI_REG8(0x3931), 0x02 },
	{ CCI_REG8(0x392f), 0x01 },
	{ CCI_REG8(0x3930), 0x80 },
	{ CCI_REG8(0x3932), 0x80 },
};

/*
 * The first part of the vendor start sequence. The vendor table is, in order:
 * these six writes, 0x0100 = 1, a 1 ms delay and 0x3c0d = 0. The orientation
 * (0x0101) is reset to 0 here, so the controls are applied after this part.
 */
static const struct cci_reg_sequence s5k5e8_start_pre_regs[] = {
	{ CCI_REG8(0x323b), 0x00 },
	{ CCI_REG8(0x3301), 0x00 },
	{ CCI_REG8(0x3321), 0x04 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x3c16), 0x00 },
	{ CCI_REG8(0x3c0d), 0x04 },
};

/* 2592x1944, 2856x1968 total, 29.89 fps */
static const struct cci_reg_sequence s5k5e8_mode_2592x1944_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x18 },
	{ CCI_REG8(0x0307), 0xa8 },
	{ CCI_REG8(0x0308), 0x34 },
	{ CCI_REG8(0x0309), 0x42 },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c0b), 0x04 },
	{ CCI_REG8(0x3c1c), 0x47 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x3c14), 0x04 },
	{ CCI_REG8(0x3c16), 0x00 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x44 },
	{ CCI_REG8(0x0114), 0x01 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0a },
	{ CCI_REG8(0x0349), 0x27 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x034b), 0x9f },
	{ CCI_REG8(0x034c), 0x0a },
	{ CCI_REG8(0x034d), 0x20 },
	{ CCI_REG8(0x034e), 0x07 },
	{ CCI_REG8(0x034f), 0x98 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0340), 0x07 },
	{ CCI_REG8(0x0341), 0xb0 },
	{ CCI_REG8(0x0342), 0x0b },
	{ CCI_REG8(0x0343), 0x28 },
	{ CCI_REG8(0x0200), 0x00 },
	{ CCI_REG8(0x0201), 0x00 },
	{ CCI_REG8(0x0202), 0x07 },
	{ CCI_REG8(0x0203), 0xaa },
	{ CCI_REG8(0x3303), 0x02 },
	{ CCI_REG8(0x3400), 0x00 },
	{ CCI_REG8(0x323b), 0x02 },
	{ CCI_REG8(0x3301), 0x00 },
	{ CCI_REG8(0x3321), 0x04 },
	{ CCI_REG8(0x3306), 0x00 },
	{ CCI_REG8(0x3307), 0x08 },
	{ CCI_REG8(0x3308), 0x0a },
	{ CCI_REG8(0x3309), 0x27 },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x01 },
	{ CCI_REG8(0x330e), 0x00 },
	{ CCI_REG8(0x330f), 0x08 },
	{ CCI_REG8(0x3310), 0x07 },
	{ CCI_REG8(0x3311), 0x9f },
	{ CCI_REG8(0x3312), 0x01 },
	{ CCI_REG8(0x3313), 0x01 },
};

/* 1296x972 (2x2 binned), 5200x1062 total, 30.42 fps */
static const struct cci_reg_sequence s5k5e8_mode_1296x972_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x18 },
	{ CCI_REG8(0x0307), 0xa8 },
	{ CCI_REG8(0x0308), 0x34 },
	{ CCI_REG8(0x0309), 0x42 },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c0b), 0x04 },
	{ CCI_REG8(0x3c1c), 0x47 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x3c14), 0x04 },
	{ CCI_REG8(0x3c16), 0x00 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x44 },
	{ CCI_REG8(0x0114), 0x01 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0a },
	{ CCI_REG8(0x0349), 0x27 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x034b), 0x9f },
	{ CCI_REG8(0x034c), 0x05 },
	{ CCI_REG8(0x034d), 0x10 },
	{ CCI_REG8(0x034e), 0x03 },
	{ CCI_REG8(0x034f), 0xcc },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },
	{ CCI_REG8(0x0340), 0x04 },
	{ CCI_REG8(0x0341), 0x26 },
	{ CCI_REG8(0x0342), 0x14 },
	{ CCI_REG8(0x0343), 0x50 },
	{ CCI_REG8(0x0200), 0x00 },
	{ CCI_REG8(0x0201), 0x00 },
	{ CCI_REG8(0x0202), 0x03 },
	{ CCI_REG8(0x0203), 0xde },
	{ CCI_REG8(0x3303), 0x02 },
	{ CCI_REG8(0x3400), 0x00 },
	{ CCI_REG8(0x323b), 0x02 },
	{ CCI_REG8(0x3301), 0x00 },
	{ CCI_REG8(0x3321), 0x04 },
	{ CCI_REG8(0x3306), 0x00 },
	{ CCI_REG8(0x3307), 0x08 },
	{ CCI_REG8(0x3308), 0x0a },
	{ CCI_REG8(0x3309), 0x27 },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x01 },
	{ CCI_REG8(0x330e), 0x00 },
	{ CCI_REG8(0x330f), 0x08 },
	{ CCI_REG8(0x3310), 0x07 },
	{ CCI_REG8(0x3311), 0x9f },
	{ CCI_REG8(0x3312), 0x03 },
	{ CCI_REG8(0x3313), 0x01 },
};

static const char * const s5k5e8_supply_names[] = {
	/* Listed in power up order, powered down in reverse order */
	"vana",	/* Analog (2.8 V) */
	"vdig",	/* Digital core */
	"vio",	/* I/O (1.8 V) */
};

#define S5K5E8_NUM_SUPPLIES		ARRAY_SIZE(s5k5e8_supply_names)

static const s64 s5k5e8_link_freq_menu[] = {
	S5K5E8_LINK_FREQ,
};

/*
 * The supported formats. The vendor library reports the colour filter
 * arrangement as 1, and the MediaTek driver for the same sensor describes the
 * unflipped output as Gr first, so GRBG is used without flips.
 * This table MUST contain 4 entries to cover the flip combinations in the
 * order:
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 s5k5e8_mbus_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char * const s5k5e8_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"100% Colour Bars",
	"Fade to Grey Colour Bars",
};

struct s5k5e8_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k5e8_mode {
	u32 width;
	u32 height;

	/* Line length in pixels */
	u32 hts;
	/* Default frame length in lines, also the minimum */
	u32 vts_def;

	struct s5k5e8_reg_list reg_list;
};

static const struct s5k5e8_mode s5k5e8_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.hts = 2856,
		.vts_def = 1968,
		.reg_list = {
			.regs = s5k5e8_mode_2592x1944_regs,
			.num_regs = ARRAY_SIZE(s5k5e8_mode_2592x1944_regs),
		},
	}, {
		.width = 1296,
		.height = 972,
		.hts = 5200,
		.vts_def = 1062,
		.reg_list = {
			.regs = s5k5e8_mode_1296x972_regs,
			.num_regs = ARRAY_SIZE(s5k5e8_mode_1296x972_regs),
		},
	},
};

struct s5k5e8 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;

	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K5E8_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

static inline struct s5k5e8 *to_s5k5e8(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5e8, sd);
}

/*
 * Get the bayer order based on the flip settings. Called with the subdev
 * state lock (shared with the control handler) held.
 */
static u32 s5k5e8_get_format_code(const struct s5k5e8 *s5k5e8)
{
	unsigned int i;

	i = (s5k5e8->vflip->val ? 2 : 0) | (s5k5e8->hflip->val ? 1 : 0);

	return s5k5e8_mbus_codes[i];
}

static const struct s5k5e8_mode *
s5k5e8_find_mode(u32 width, u32 height)
{
	return v4l2_find_nearest_size(s5k5e8_modes, ARRAY_SIZE(s5k5e8_modes),
				      width, height, width, height);
}

static const struct s5k5e8_mode *
s5k5e8_get_mode(struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);

	return s5k5e8_find_mode(fmt->width, fmt->height);
}

static void s5k5e8_adjust_exposure_range(struct s5k5e8 *s5k5e8,
					 const struct s5k5e8_mode *mode)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure */
	exposure_max = mode->height + s5k5e8->vblank->val -
		       S5K5E8_EXPOSURE_MARGIN;
	exposure_def = min(exposure_max, s5k5e8->exposure->val);
	__v4l2_ctrl_modify_range(s5k5e8->exposure, s5k5e8->exposure->minimum,
				 exposure_max, s5k5e8->exposure->step,
				 exposure_def);
}

static int s5k5e8_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e8 *s5k5e8 =
		container_of(ctrl->handler, struct s5k5e8, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&s5k5e8->sd);
	struct v4l2_subdev_state *state =
		v4l2_subdev_get_locked_active_state(&s5k5e8->sd);
	const struct s5k5e8_mode *mode = s5k5e8_get_mode(state);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so
	 * check and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		s5k5e8_adjust_exposure_range(s5k5e8, mode);

	/* The bayer order depends on the flips, keep the active format in sync */
	if (ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP)
		v4l2_subdev_state_get_format(state, 0)->code =
			s5k5e8_get_format_code(s5k5e8);

	/* Controls are only written to the sensor when it is powered */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_VTS,
				mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_ORIENTATION,
				(s5k5e8->hflip->val ? S5K5E8_HFLIP : 0) |
				(s5k5e8->vflip->val ? S5K5E8_VFLIP : 0),
				NULL);
		break;
	case V4L2_CID_LINK_FREQ:
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_HBLANK:
		/* Read-only controls, only updated internally on mode change */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k5e8_ctrl_ops = {
	.s_ctrl = s5k5e8_set_ctrl,
};

static int s5k5e8_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(state, 0);

	fmt->width = s5k5e8_modes[0].width;
	fmt->height = s5k5e8_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->code = s5k5e8_get_format_code(s5k5e8);
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;

	crop->left = S5K5E8_PIXEL_ARRAY_LEFT;
	crop->top = S5K5E8_PIXEL_ARRAY_TOP;
	crop->width = S5K5E8_PIXEL_ARRAY_WIDTH;
	crop->height = S5K5E8_PIXEL_ARRAY_HEIGHT;

	return 0;
}

static int s5k5e8_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	/* Only one bayer format (10 bit) is supported, depending on the flips */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k5e8_get_format_code(s5k5e8);

	return 0;
}

static int s5k5e8_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	if (fse->index >= ARRAY_SIZE(s5k5e8_modes))
		return -EINVAL;

	if (fse->code != s5k5e8_get_format_code(s5k5e8))
		return -EINVAL;

	fse->min_width = s5k5e8_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k5e8_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k5e8_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	const struct s5k5e8_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	s32 vblank_def;
	s64 hblank;

	/* The mode cannot be changed while streaming */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = s5k5e8_find_mode(fmt->format.width, fmt->format.height);

	fmt->format.code = s5k5e8_get_format_code(s5k5e8);
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	framefmt = v4l2_subdev_state_get_format(state, fmt->pad);
	*framefmt = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		/* Update the limits and set the frame rate to the default */
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(s5k5e8->vblank, vblank_def,
					 S5K5E8_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(s5k5e8->vblank, vblank_def);

		hblank = mode->hts - mode->width;
		__v4l2_ctrl_modify_range(s5k5e8->hblank, hblank, hblank, 1,
					 hblank);

		s5k5e8_adjust_exposure_range(s5k5e8, mode);
	}

	return 0;
}

static int s5k5e8_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = S5K5E8_NATIVE_WIDTH;
		sel->r.height = S5K5E8_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = S5K5E8_PIXEL_ARRAY_LEFT;
		sel->r.top = S5K5E8_PIXEL_ARRAY_TOP;
		sel->r.width = S5K5E8_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K5E8_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5k5e8_start_streaming(struct s5k5e8 *s5k5e8,
				  const struct s5k5e8_mode *mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k5e8->sd);
	int ret;

	ret = cci_multi_reg_write(s5k5e8->regmap, s5k5e8_init_regs,
				  ARRAY_SIZE(s5k5e8_init_regs), NULL);
	if (ret) {
		dev_err(&client->dev, "failed to write init settings: %d\n",
			ret);
		return ret;
	}

	msleep(S5K5E8_INIT_DELAY_MS);

	ret = cci_multi_reg_write(s5k5e8->regmap, mode->reg_list.regs,
				  mode->reg_list.num_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to write mode settings: %d\n",
			ret);
		return ret;
	}

	ret = cci_multi_reg_write(s5k5e8->regmap, s5k5e8_start_pre_regs,
				  ARRAY_SIZE(s5k5e8_start_pre_regs), NULL);
	if (ret) {
		dev_err(&client->dev, "failed to prepare streaming: %d\n", ret);
		return ret;
	}

	/* Apply the user controls, including the orientation reset above */
	ret = __v4l2_ctrl_handler_setup(s5k5e8->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = cci_write(s5k5e8->regmap, S5K5E8_REG_MODE_SELECT,
			S5K5E8_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to start streaming: %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);

	return cci_write(s5k5e8->regmap, S5K5E8_REG_STREAM_ON_FIXUP, 0x00,
			 NULL);
}

static void s5k5e8_stop_streaming(struct s5k5e8 *s5k5e8)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k5e8->sd);
	int ret;

	ret = cci_write(s5k5e8->regmap, S5K5E8_REG_MODE_SELECT,
			S5K5E8_MODE_STANDBY, NULL);
	if (ret)
		dev_err(&client->dev, "failed to stop streaming: %d\n", ret);
}

static int s5k5e8_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	ret = s5k5e8_start_streaming(s5k5e8, s5k5e8_get_mode(state));
	if (ret) {
		pm_runtime_put(&client->dev);
		return ret;
	}

	/* The bayer order cannot change during streaming */
	__v4l2_ctrl_grab(s5k5e8->vflip, true);
	__v4l2_ctrl_grab(s5k5e8->hflip, true);

	return 0;
}

static int s5k5e8_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	s5k5e8_stop_streaming(s5k5e8);

	__v4l2_ctrl_grab(s5k5e8->vflip, false);
	__v4l2_ctrl_grab(s5k5e8->hflip, false);

	pm_runtime_put(&client->dev);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k5e8_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k5e8_pad_ops = {
	.enum_mbus_code = s5k5e8_enum_mbus_code,
	.enum_frame_size = s5k5e8_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k5e8_set_pad_format,
	.get_selection = s5k5e8_get_selection,
	.enable_streams = s5k5e8_enable_streams,
	.disable_streams = s5k5e8_disable_streams,
};

static const struct v4l2_subdev_ops s5k5e8_subdev_ops = {
	.video = &s5k5e8_video_ops,
	.pad = &s5k5e8_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5e8_internal_ops = {
	.init_state = s5k5e8_init_state,
};

/*
 * Power up sequence of the vendor library: reset asserted, the rails with
 * 5 ms between them, MCLK, 1 ms, reset released, 10 ms, standby released and
 * 10 ms. The standby pin is not wired on the supported boards, so only the
 * delays after it are kept.
 */
static int s5k5e8_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 1);
	usleep_range(1000, 2000);

	for (i = 0; i < S5K5E8_NUM_SUPPLIES; i++) {
		ret = regulator_enable(s5k5e8->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s: %d\n",
				s5k5e8->supplies[i].supply, ret);
			goto err_disable_supplies;
		}

		usleep_range(5000, 6000);
	}

	ret = clk_prepare_enable(s5k5e8->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		goto err_disable_supplies;
	}

	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 0);

	/* 10 ms after reset, and 10 ms after standby in the vendor sequence */
	usleep_range(20000, 25000);

	return 0;

err_disable_supplies:
	while (i--)
		regulator_disable(s5k5e8->supplies[i].consumer);

	return ret;
}

static int s5k5e8_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	unsigned int i;

	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 1);
	usleep_range(1000, 2000);

	clk_disable_unprepare(s5k5e8->clk);
	usleep_range(1000, 2000);

	for (i = S5K5E8_NUM_SUPPLIES; i--;)
		regulator_disable(s5k5e8->supplies[i].consumer);

	return 0;
}

static int s5k5e8_identify_module(struct s5k5e8 *s5k5e8)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k5e8->sd);
	u64 val;
	int ret;

	ret = cci_read(s5k5e8->regmap, S5K5E8_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read chip id\n");

	if (val != S5K5E8_CHIP_ID)
		return dev_err_probe(&client->dev, -ENODEV,
				     "chip id mismatch: 0x%04x != 0x%04llx\n",
				     S5K5E8_CHIP_ID, val);

	return 0;
}

static int s5k5e8_init_controls(struct s5k5e8 *s5k5e8)
{
	struct i2c_client *client = v4l2_get_subdevdata(&s5k5e8->sd);
	const struct s5k5e8_mode *mode = &s5k5e8_modes[0];
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k5e8->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	s64 vblank_def;
	s64 hblank;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	s5k5e8->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						   &s5k5e8_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(s5k5e8_link_freq_menu) - 1,
						   0, s5k5e8_link_freq_menu);
	if (s5k5e8->link_freq)
		s5k5e8->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k5e8->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       S5K5E8_PIXEL_RATE,
					       S5K5E8_PIXEL_RATE, 1,
					       S5K5E8_PIXEL_RATE);
	if (s5k5e8->pixel_rate)
		s5k5e8->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	s5k5e8->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   S5K5E8_VTS_MAX - mode->height, 1,
					   vblank_def);

	hblank = mode->hts - mode->width;
	s5k5e8->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (s5k5e8->hblank)
		s5k5e8->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k5e8->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K5E8_EXPOSURE_MIN,
					     mode->vts_def -
					     S5K5E8_EXPOSURE_MARGIN,
					     S5K5E8_EXPOSURE_STEP,
					     S5K5E8_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, S5K5E8_ANA_GAIN_MIN,
			  S5K5E8_ANA_GAIN_MAX, S5K5E8_ANA_GAIN_STEP,
			  S5K5E8_ANA_GAIN_DEFAULT);

	s5k5e8->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k5e8->hflip)
		s5k5e8->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k5e8->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k5e8->vflip)
		s5k5e8->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k5e8_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k5e8_test_pattern_menu) - 1,
				     0, 0, s5k5e8_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "failed to initialize controls: %d\n",
			ret);
		goto err_free_ctrls;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto err_free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k5e8_ctrl_ops,
					      &props);
	if (ret)
		goto err_free_ctrls;

	s5k5e8->sd.ctrl_handler = ctrl_hdlr;

	return 0;

err_free_ctrls:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/*
 * Check that the endpoint describes the only supported configuration: two
 * data lanes and a 418 MHz link frequency.
 */
static int s5k5e8_check_endpoint(struct device *dev)
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

	if (ep.bus.mipi_csi2.num_data_lanes != S5K5E8_DATA_LANES) {
		ret = dev_err_probe(dev, -EINVAL, "invalid data lanes: %u\n",
				    ep.bus.mipi_csi2.num_data_lanes);
		goto out_free;
	}

	ret = v4l2_link_freq_to_bitmap(dev, ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       s5k5e8_link_freq_menu,
				       ARRAY_SIZE(s5k5e8_link_freq_menu),
				       &bitmap);
	if (ret)
		dev_err_probe(dev, ret, "link frequency not supported\n");

out_free:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static int s5k5e8_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k5e8 *s5k5e8;
	unsigned int i;
	int ret;

	s5k5e8 = devm_kzalloc(dev, sizeof(*s5k5e8), GFP_KERNEL);
	if (!s5k5e8)
		return -ENOMEM;

	s5k5e8->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k5e8->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k5e8->regmap),
				     "failed to initialize CCI\n");

	for (i = 0; i < S5K5E8_NUM_SUPPLIES; i++)
		s5k5e8->supplies[i].supply = s5k5e8_supply_names[i];

	ret = devm_regulator_bulk_get(dev, S5K5E8_NUM_SUPPLIES,
				      s5k5e8->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	s5k5e8->clk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(s5k5e8->clk))
		return dev_err_probe(dev, PTR_ERR(s5k5e8->clk),
				     "failed to get clock\n");

	if (clk_get_rate(s5k5e8->clk) != S5K5E8_EXTCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "clock frequency must be %lu Hz\n",
				     S5K5E8_EXTCLK_FREQ);

	ret = s5k5e8_check_endpoint(dev);
	if (ret)
		return ret;

	/* The sensor is held in reset until it is powered on */
	s5k5e8->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k5e8->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k5e8->reset_gpio),
				     "failed to get reset gpio\n");

	v4l2_i2c_subdev_init(&s5k5e8->sd, client, &s5k5e8_subdev_ops);

	/* Will be powered off via pm_runtime_idle() */
	ret = s5k5e8_power_on(dev);
	if (ret)
		return ret;

	ret = s5k5e8_identify_module(s5k5e8);
	if (ret)
		goto err_power_off;

	ret = s5k5e8_init_controls(s5k5e8);
	if (ret)
		goto err_power_off;

	s5k5e8->sd.internal_ops = &s5k5e8_internal_ops;
	s5k5e8->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k5e8->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	s5k5e8->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s5k5e8->sd.entity, 1, &s5k5e8->pad);
	if (ret)
		goto err_free_ctrls;

	s5k5e8->sd.state_lock = s5k5e8->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&s5k5e8->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialize subdev\n");
		goto err_media_entity_cleanup;
	}

	/*
	 * The device is powered on at this point. Enable runtime PM before
	 * registering the subdev, so that it can be used as soon as it is
	 * visible.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k5e8->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register sensor subdev\n");
		goto err_subdev_cleanup;
	}

	pm_runtime_idle(dev);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&s5k5e8->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

err_media_entity_cleanup:
	media_entity_cleanup(&s5k5e8->sd.entity);

err_free_ctrls:
	v4l2_ctrl_handler_free(&s5k5e8->ctrl_handler);

err_power_off:
	s5k5e8_power_off(dev);

	return ret;
}

static void s5k5e8_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k5e8->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		s5k5e8_power_off(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
}

static const struct dev_pm_ops s5k5e8_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k5e8_power_off, s5k5e8_power_on, NULL)
};

static const struct of_device_id s5k5e8_of_match[] = {
	{ .compatible = "samsung,s5k5e8" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k5e8_of_match);

static struct i2c_driver s5k5e8_i2c_driver = {
	.driver = {
		.name = "s5k5e8",
		.pm = &s5k5e8_pm_ops,
		.of_match_table = s5k5e8_of_match,
	},
	.probe = s5k5e8_probe,
	.remove = s5k5e8_remove,
};
module_i2c_driver(s5k5e8_i2c_driver);

MODULE_AUTHOR("Gianluca Boiano <morf3089@gmail.com>");
MODULE_DESCRIPTION("Samsung S5K5E8 sensor driver");
MODULE_LICENSE("GPL");
