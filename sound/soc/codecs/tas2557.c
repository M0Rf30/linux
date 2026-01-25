// SPDX-License-Identifier: GPL-2.0
//
// ALSA SoC Texas Instruments TAS2557 Smart Amplifier
//
// Copyright (C) 2016 Texas Instruments Inc.
// Copyright (C) 2024

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "tas2557.h"

/* Forward declarations */
static void tas2557_hw_reset(struct tas2557_priv *tas2557);
static int tas2557_failsafe_recovery(struct tas2557_priv *tas2557);

/* Special delay markers for register sequences */
#define TAS2557_UDELAY	0xFFFFFFFE
#define TAS2557_MDELAY	0xFFFFFFFD

/* Default initialization data */
static const unsigned int tas2557_default_data[] = {
	TAS2557_SAR_ADC2_REG, 0x05,	/* enable SAR ADC */
	TAS2557_CLK_ERR_CTRL2, 0x21,	/* clk1: clock hysteresis, 0.34ms */
	TAS2557_CLK_ERR_CTRL3, 0x21,	/* clk2: rampDown 15dB/us */
	TAS2557_SAFE_GUARD_REG, TAS2557_SAFE_GUARD_PATTERN,
	0xFFFFFFFF, 0xFFFFFFFF
};

/* IRQ configuration data */
static const unsigned int tas2557_irq_config[] = {
	TAS2557_CLK_HALT_REG, 0x71,	/* enable clk halt detect2 interrupt */
	TAS2557_INT_GEN1_REG, 0x11,	/* enable spk OC and OV */
	TAS2557_INT_GEN2_REG, 0x11,	/* enable clk err1 and die OT */
	TAS2557_INT_GEN3_REG, 0x11,	/* enable clk err2 and brownout */
	TAS2557_INT_GEN4_REG, 0x01,	/* disable SAR, enable clk halt */
	TAS2557_GPIO4_PIN_REG, 0x07,	/* set GPIO4 as int1 */
	TAS2557_INT_MODE_REG, 0x80,	/* active high until cleared */
	0xFFFFFFFF, 0xFFFFFFFF
};

/* Startup sequence - matches Android driver for ASI2 interface */
static const unsigned int tas2557_startup_data[] = {
	TAS2557_GPI_PIN_REG, 0x15,	/* enable DIN, MCLK, CCI */
	TAS2557_GPIO1_PIN_REG, 0x01,	/* enable BCLK (ASI1) */
	TAS2557_GPIO2_PIN_REG, 0x01,	/* enable WCLK (ASI1) */
	/* ASI2 GPIO configuration - hardware uses ASI2 interface */
	TAS2557_GPIO6_PIN_REG, 0x01,	/* GPIO6 as ASI2 WCLK input */
	TAS2557_GPIO8_PIN_REG, 0x02,	/* GPIO8 as ASI2 DIN */
	/*
	 * ASI2 DAC format - MUST be set here in startup sequence!
	 * Setting in hw_params doesn't work because device isn't powered yet.
	 * Use 32-bit format (0x18) as Android driver does - Q6AFE sends 32-bit I2S frames.
	 */
	TAS2557_ASI2_DAC_FORMAT_REG, 0x18,	/* ASI2 as 32-bit I2S */
	/* ASI2 clock dividers - two-step sequence: set ratio, then power up */
	TAS2557_ASI2_BDIV_CLK_SEL_REG, 0x01,	/* BDIV clock select */
	TAS2557_ASI2_BDIV_CLK_RATIO_REG, 0x01,	/* ASI2 BDIV ratio */
	TAS2557_ASI2_BDIV_CLK_RATIO_REG, 0x81,	/* ASI2 BDIV power up */
	TAS2557_ASI2_WDIV_CLK_RATIO_REG, 0x40,	/* ASI2 WDIV ratio */
	TAS2557_ASI2_WDIV_CLK_RATIO_REG, 0xc0,	/* ASI2 WDIV power up */
	TAS2557_GPIO5_PIN_REG, 0x01,	/* GPIO5 as ASI2 BCLK input */
	TAS2557_GPIO7_PIN_REG, 0x15,	/* GPIO7 as ASI2 DOUT */
	TAS2557_POWER_CTRL2_REG, 0xA0,	/* Class-D, Boost power up */
	TAS2557_POWER_CTRL2_REG, 0xA3,	/* Class-D, Boost, IV sense power up */
	TAS2557_POWER_CTRL1_REG, 0xF8,	/* PLL, DSP, clock dividers power up */
	TAS2557_UDELAY, 2000,		/* delay 2ms */
	TAS2557_CLK_ERR_CTRL, 0x2b,	/* enable clock error detection */
	TAS2557_DBOOST_CFG_REG, 0x0b,	/* reduce full band noise */
	0xFFFFFFFF, 0xFFFFFFFF
};

/* Unmute sequence */
static const unsigned int tas2557_unmute_data[] = {
	TAS2557_MUTE_REG, 0x00,		/* unmute */
	TAS2557_SOFT_MUTE_REG, 0x00,	/* soft unmute */
	0xFFFFFFFF, 0xFFFFFFFF
};

/* Shutdown sequence */
static const unsigned int tas2557_shutdown_data[] = {
	TAS2557_CLK_ERR_CTRL, 0x00,	/* disable clock error detection */
	TAS2557_SOFT_MUTE_REG, 0x01,	/* soft mute */
	TAS2557_UDELAY, 10000,		/* delay 10ms */
	TAS2557_MUTE_REG, 0x03,		/* mute */
	TAS2557_POWER_CTRL1_REG, 0x60,	/* DSP power down */
	TAS2557_UDELAY, 2000,		/* delay 2ms */
	TAS2557_POWER_CTRL2_REG, 0x00,	/* Class-D, Boost power down */
	TAS2557_POWER_CTRL1_REG, 0x00,	/* all power down */
	TAS2557_GPIO1_PIN_REG, 0x00,	/* disable BCLK (ASI1) */
	TAS2557_GPIO2_PIN_REG, 0x00,	/* disable WCLK (ASI1) */
	/* Disable ASI2 GPIOs */
	TAS2557_GPIO5_PIN_REG, 0x00,	/* disable ASI2 BCLK */
	TAS2557_GPIO6_PIN_REG, 0x00,	/* disable ASI2 WCLK */
	TAS2557_GPIO7_PIN_REG, 0x00,	/* disable ASI2 DOUT */
	TAS2557_GPIO8_PIN_REG, 0x00,	/* disable ASI2 DIN */
	TAS2557_GPI_PIN_REG, 0x00,	/* disable DIN, MCLK, CCI */
	0xFFFFFFFF, 0xFFFFFFFF
};

static bool tas2557_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static bool tas2557_writeable(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config tas2557_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2557_writeable,
	.volatile_reg = tas2557_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

/*
 * Book/page switching functions
 */
static int tas2557_change_book_page(struct tas2557_priv *tas2557,
				    unsigned char book, unsigned char page)
{
	int ret = 0;

	if (tas2557->current_book == book && tas2557->current_page == page)
		return 0;

	if (tas2557->current_book != book) {
		/* Switch to page 0 first */
		ret = regmap_write(tas2557->regmap, TAS2557_PAGE_REG, 0);
		if (ret < 0) {
			dev_err(tas2557->dev, "failed to switch to page 0: %d\n", ret);
			goto err;
		}
		tas2557->current_page = 0;

		/* Switch book */
		ret = regmap_write(tas2557->regmap, TAS2557_BOOK_REG, book);
		if (ret < 0) {
			dev_err(tas2557->dev, "failed to switch to book %d: %d\n",
				book, ret);
			goto err;
		}
		tas2557->current_book = book;

		/* Switch to target page if not page 0 */
		if (page != 0) {
			ret = regmap_write(tas2557->regmap, TAS2557_PAGE_REG, page);
			if (ret < 0) {
				dev_err(tas2557->dev, "failed to switch to page %d: %d\n",
					page, ret);
				goto err;
			}
			tas2557->current_page = page;
		}
	} else if (tas2557->current_page != page) {
		ret = regmap_write(tas2557->regmap, TAS2557_PAGE_REG, page);
		if (ret < 0) {
			dev_err(tas2557->dev, "failed to switch to page %d: %d\n",
				page, ret);
			goto err;
		}
		tas2557->current_page = page;
	}

	return 0;

err:
	tas2557->err_code |= ERROR_DEVA_I2C_COMM;
	return ret;
}

static int tas2557_dev_read(struct tas2557_priv *tas2557,
			    unsigned int reg, unsigned int *value)
{
	int ret;

	mutex_lock(&tas2557->dev_lock);

	ret = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret < 0)
		goto out;

	ret = regmap_read(tas2557->regmap, TAS2557_PAGE_REG_ADDR(reg), value);
	if (ret < 0) {
		dev_err(tas2557->dev, "read reg 0x%x failed: %d\n", reg, ret);
		tas2557->err_code |= ERROR_DEVA_I2C_COMM;
	} else {
		tas2557->err_code &= ~ERROR_DEVA_I2C_COMM;
	}

out:
	mutex_unlock(&tas2557->dev_lock);
	return ret;
}

static int tas2557_dev_write(struct tas2557_priv *tas2557,
			     unsigned int reg, unsigned int value)
{
	int ret;

	mutex_lock(&tas2557->dev_lock);

	ret = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret < 0)
		goto out;

	ret = regmap_write(tas2557->regmap, TAS2557_PAGE_REG_ADDR(reg), value);
	if (ret < 0) {
		dev_err(tas2557->dev, "write reg 0x%x failed: %d\n", reg, ret);
		tas2557->err_code |= ERROR_DEVA_I2C_COMM;
	} else {
		tas2557->err_code &= ~ERROR_DEVA_I2C_COMM;
	}

out:
	mutex_unlock(&tas2557->dev_lock);
	return ret;
}

static int tas2557_dev_update_bits(struct tas2557_priv *tas2557,
				   unsigned int reg, unsigned int mask,
				   unsigned int value)
{
	int ret;

	mutex_lock(&tas2557->dev_lock);

	ret = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret < 0)
		goto out;

	ret = regmap_update_bits(tas2557->regmap, TAS2557_PAGE_REG_ADDR(reg),
				 mask, value);
	if (ret < 0) {
		dev_err(tas2557->dev, "update_bits reg 0x%x failed: %d\n", reg, ret);
		tas2557->err_code |= ERROR_DEVA_I2C_COMM;
	} else {
		tas2557->err_code &= ~ERROR_DEVA_I2C_COMM;
	}

out:
	mutex_unlock(&tas2557->dev_lock);
	return ret;
}

static int tas2557_dev_bulk_read(struct tas2557_priv *tas2557,
				 unsigned int reg, u8 *data, size_t len)
{
	int ret;

	mutex_lock(&tas2557->dev_lock);

	ret = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret < 0)
		goto out;

	ret = regmap_bulk_read(tas2557->regmap, TAS2557_PAGE_REG_ADDR(reg),
			       data, len);
	if (ret < 0) {
		dev_err(tas2557->dev, "bulk_read reg 0x%x failed: %d\n", reg, ret);
		tas2557->err_code |= ERROR_DEVA_I2C_COMM;
	} else {
		tas2557->err_code &= ~ERROR_DEVA_I2C_COMM;
	}

out:
	mutex_unlock(&tas2557->dev_lock);
	return ret;
}

static int tas2557_dev_bulk_write(struct tas2557_priv *tas2557,
				  unsigned int reg, const u8 *data, size_t len)
{
	int ret;

	mutex_lock(&tas2557->dev_lock);

	ret = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(reg),
				       TAS2557_PAGE_ID(reg));
	if (ret < 0)
		goto out;

	ret = regmap_bulk_write(tas2557->regmap, TAS2557_PAGE_REG_ADDR(reg),
				data, len);
	if (ret < 0) {
		dev_err(tas2557->dev, "bulk_write reg 0x%x failed: %d\n", reg, ret);
		tas2557->err_code |= ERROR_DEVA_I2C_COMM;
	} else {
		tas2557->err_code &= ~ERROR_DEVA_I2C_COMM;
	}

out:
	mutex_unlock(&tas2557->dev_lock);
	return ret;
}

/*
 * Load register sequence data with debug tracing
 */
static int tas2557_load_data(struct tas2557_priv *tas2557,
			     const unsigned int *data)
{
	unsigned int reg, val;
	int ret = 0;
	int i = 0;

	while (1) {
		reg = data[i * 2];
		val = data[i * 2 + 1];

		if (reg == 0xFFFFFFFF)
			break;

		if (reg == TAS2557_UDELAY) {
			usleep_range(val, val + 100);
		} else if (reg == TAS2557_MDELAY) {
			msleep(val);
		} else {
			/* Debug: trace ASI2_DAC_FORMAT writes */
			if (reg == TAS2557_ASI2_DAC_FORMAT_REG) {
				unsigned int readback;

				dev_info(tas2557->dev,
					 "DEBUG: writing ASI2_DAC_FORMAT = 0x%02x\n",
					 val);
				ret = tas2557_dev_write(tas2557, reg, val);
				if (ret < 0) {
					dev_err(tas2557->dev,
						"DEBUG: ASI2_DAC_FORMAT write FAILED: %d\n",
						ret);
					break;
				}
				/* Read back to verify */
				tas2557_dev_read(tas2557, reg, &readback);
				dev_info(tas2557->dev,
					 "DEBUG: ASI2_DAC_FORMAT readback = 0x%02x %s\n",
					 readback,
					 (readback == val) ? "OK" : "MISMATCH!");
			} else {
				ret = tas2557_dev_write(tas2557, reg, val);
				if (ret < 0)
					break;
			}
		}
		i++;
	}

	return ret;
}

/*
 * Firmware parsing helpers
 */
static inline u32 fw_get_be32(const u8 *data)
{
	return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

static inline u16 fw_get_be16(const u8 *data)
{
	return (data[0] << 8) | data[1];
}

/*
 * Free firmware memory
 */
static void tas2557_fw_free(struct tas2557_firmware *fw)
{
	unsigned int i, j;

	if (!fw)
		return;

	/* Free PLLs */
	if (fw->plls) {
		for (i = 0; i < fw->num_plls; i++) {
			kfree(fw->plls[i].description);
			kfree(fw->plls[i].block.data);
		}
		kfree(fw->plls);
	}

	/* Free programs */
	if (fw->programs) {
		for (i = 0; i < fw->num_programs; i++) {
			kfree(fw->programs[i].description);
			if (fw->programs[i].data.blocks) {
				for (j = 0; j < fw->programs[i].data.num_blocks; j++)
					kfree(fw->programs[i].data.blocks[j].data);
				kfree(fw->programs[i].data.blocks);
			}
			kfree(fw->programs[i].data.description);
		}
		kfree(fw->programs);
	}

	/* Free configs */
	if (fw->configs) {
		for (i = 0; i < fw->num_configs; i++) {
			kfree(fw->configs[i].description);
			if (fw->configs[i].data.blocks) {
				for (j = 0; j < fw->configs[i].data.num_blocks; j++)
					kfree(fw->configs[i].data.blocks[j].data);
				kfree(fw->configs[i].data.blocks);
			}
			kfree(fw->configs[i].data.description);
		}
		kfree(fw->configs);
	}

	kfree(fw->description);
	kfree(fw);
}

/*
 * Parse firmware header
 * Returns offset to next section, or negative error
 */
static int fw_parse_header(struct tas2557_priv *tas2557,
			   struct tas2557_firmware *fw,
			   const u8 *data, size_t size)
{
	const u8 *start = data;
	size_t desc_len;

	if (size < 104) {
		dev_err(tas2557->dev, "firmware header too short\n");
		return -EINVAL;
	}

	/* Check magic number: 0x35 0x35 0x35 0x32 ("5552") */
	if (data[0] != 0x35 || data[1] != 0x35 ||
	    data[2] != 0x35 || data[3] != 0x32) {
		dev_err(tas2557->dev, "invalid firmware magic: %02x%02x%02x%02x\n",
			data[0], data[1], data[2], data[3]);
		return -EINVAL;
	}
	data += 4;

	fw->size = fw_get_be32(data);
	data += 4;

	fw->checksum = fw_get_be32(data);
	data += 4;

	fw->ppc_version = fw_get_be32(data);
	data += 4;

	fw->fw_version = fw_get_be32(data);
	data += 4;

	fw->driver_version = fw_get_be32(data);
	data += 4;

	fw->timestamp = fw_get_be32(data);
	data += 4;

	memcpy(fw->ddc_name, data, 64);
	data += 64;

	/* Description is null-terminated string */
	desc_len = strnlen(data, size - (data - start));
	if (desc_len > 0) {
		fw->description = kmemdup(data, desc_len + 1, GFP_KERNEL);
		if (!fw->description)
			return -ENOMEM;
	}
	data += desc_len + 1;

	if ((data - start) + 8 > size) {
		dev_err(tas2557->dev, "firmware header truncated\n");
		return -EINVAL;
	}

	fw->device_family = fw_get_be32(data);
	data += 4;

	if (fw->device_family != 0) {
		dev_err(tas2557->dev, "unsupported device family: %u\n",
			fw->device_family);
		return -EINVAL;
	}

	fw->device = fw_get_be32(data);
	data += 4;

	if (fw->device != 2) {
		dev_err(tas2557->dev, "unsupported device type: %u (expected 2 for TAS2557)\n",
			fw->device);
		return -EINVAL;
	}

	dev_dbg(tas2557->dev, "firmware: %s, version 0x%x, PPC 0x%x, driver 0x%x\n",
		fw->ddc_name, fw->fw_version, fw->ppc_version, fw->driver_version);

	return data - start;
}

/*
 * Parse a single block of register commands
 */
static int fw_parse_block(struct tas2557_priv *tas2557,
			  struct tas2557_firmware *fw,
			  struct tas2557_block *block,
			  const u8 *data, size_t remaining)
{
	const u8 *start = data;
	size_t data_len;

	if (remaining < 4)
		return -EINVAL;

	block->type = fw_get_be32(data);
	data += 4;
	remaining -= 4;

	/* Checksum fields present in newer firmware */
	if (fw->driver_version >= PPC_DRIVER_CRCCHK) {
		if (remaining < 4)
			return -EINVAL;

		block->pchksum_present = data[0];
		block->pchksum = data[1];
		block->ychksum_present = data[2];
		block->ychksum = data[3];
		data += 4;
		remaining -= 4;
	}

	if (remaining < 4)
		return -EINVAL;

	block->num_commands = fw_get_be32(data);
	data += 4;
	remaining -= 4;

	dev_dbg(tas2557->dev, "block: type=0x%x, cmds=%u, crc=%d\n",
		block->type, block->num_commands,
		fw->driver_version >= PPC_DRIVER_CRCCHK);

	/* Each command is 4 bytes */
	data_len = block->num_commands * 4;
	if (data_len > remaining) {
		dev_err(tas2557->dev, "block data truncated: need %zu, have %zu (type=0x%x)\n",
			data_len, remaining, block->type);
		return -EINVAL;
	}

	if (data_len > 0) {
		block->data = kmemdup(data, data_len, GFP_KERNEL);
		if (!block->data)
			return -ENOMEM;
	}
	data += data_len;

	return data - start;
}

/*
 * Parse a data container (name + description + blocks)
 */
static int fw_parse_data(struct tas2557_priv *tas2557,
			 struct tas2557_firmware *fw,
			 struct tas2557_data *img_data,
			 const u8 *data, size_t remaining)
{
	const u8 *start = data;
	size_t desc_len;
	unsigned int i;
	int ret;

	if (remaining < 64)
		return -EINVAL;

	memcpy(img_data->name, data, 64);
	data += 64;
	remaining -= 64;

	/* Description is null-terminated */
	desc_len = strnlen(data, remaining);
	if (desc_len > 0) {
		img_data->description = kmemdup(data, desc_len + 1, GFP_KERNEL);
		if (!img_data->description)
			return -ENOMEM;
	}
	data += desc_len + 1;
	remaining -= desc_len + 1;

	if (remaining < 2)
		return -EINVAL;

	img_data->num_blocks = fw_get_be16(data);
	data += 2;
	remaining -= 2;

	if (img_data->num_blocks == 0)
		return data - start;

	img_data->blocks = kcalloc(img_data->num_blocks,
				   sizeof(struct tas2557_block), GFP_KERNEL);
	if (!img_data->blocks)
		return -ENOMEM;

	for (i = 0; i < img_data->num_blocks; i++) {
		ret = fw_parse_block(tas2557, fw, &img_data->blocks[i],
				     data, remaining);
		if (ret < 0)
			return ret;
		data += ret;
		remaining -= ret;
	}

	return data - start;
}

/*
 * Parse PLL configurations
 */
static int fw_parse_plls(struct tas2557_priv *tas2557,
			 struct tas2557_firmware *fw,
			 const u8 *data, size_t remaining)
{
	const u8 *start = data;
	size_t desc_len;
	unsigned int i;
	int ret;

	if (remaining < 2)
		return -EINVAL;

	fw->num_plls = fw_get_be16(data);
	data += 2;
	remaining -= 2;

	dev_dbg(tas2557->dev, "parsing %u PLLs, remaining=%zu\n",
		fw->num_plls, remaining);

	if (fw->num_plls == 0)
		return data - start;

	fw->plls = kcalloc(fw->num_plls, sizeof(struct tas2557_pll), GFP_KERNEL);
	if (!fw->plls)
		return -ENOMEM;

	for (i = 0; i < fw->num_plls; i++) {
		if (remaining < 64)
			return -EINVAL;

		memcpy(fw->plls[i].name, data, 64);
		data += 64;
		remaining -= 64;

		desc_len = strnlen(data, remaining);
		if (desc_len > 0) {
			fw->plls[i].description = kmemdup(data, desc_len + 1, GFP_KERNEL);
			if (!fw->plls[i].description)
				return -ENOMEM;
		}
		data += desc_len + 1;
		remaining -= desc_len + 1;

		ret = fw_parse_block(tas2557, fw, &fw->plls[i].block,
				     data, remaining);
		if (ret < 0)
			return ret;
		data += ret;
		remaining -= ret;

		dev_dbg(tas2557->dev, "PLL[%u]: %s\n", i, fw->plls[i].name);
	}

	return data - start;
}

/*
 * Parse DSP programs
 */
static int fw_parse_programs(struct tas2557_priv *tas2557,
			     struct tas2557_firmware *fw,
			     const u8 *data, size_t remaining)
{
	const u8 *start = data;
	size_t desc_len;
	unsigned int i;
	int ret;

	if (remaining < 2)
		return -EINVAL;

	fw->num_programs = fw_get_be16(data);
	data += 2;
	remaining -= 2;

	if (fw->num_programs == 0) {
		dev_err(tas2557->dev, "firmware contains no programs\n");
		return -EINVAL;
	}

	fw->programs = kcalloc(fw->num_programs, sizeof(struct tas2557_program),
			       GFP_KERNEL);
	if (!fw->programs)
		return -ENOMEM;

	for (i = 0; i < fw->num_programs; i++) {
		if (remaining < 64)
			return -EINVAL;

		memcpy(fw->programs[i].name, data, 64);
		data += 64;
		remaining -= 64;

		desc_len = strnlen(data, remaining);
		if (desc_len > 0) {
			fw->programs[i].description = kmemdup(data, desc_len + 1, GFP_KERNEL);
			if (!fw->programs[i].description)
				return -ENOMEM;
		}
		data += desc_len + 1;
		remaining -= desc_len + 1;

		if (remaining < 3)
			return -EINVAL;

		/* App mode and boost settings */
		fw->programs[i].app_mode = data[0];
		data += 1;
		remaining -= 1;

		fw->programs[i].boost = fw_get_be16(data);
		data += 2;
		remaining -= 2;

		ret = fw_parse_data(tas2557, fw, &fw->programs[i].data,
				    data, remaining);
		if (ret < 0)
			return ret;
		data += ret;
		remaining -= ret;

		dev_dbg(tas2557->dev, "program[%u]: %s, mode=%u\n",
			i, fw->programs[i].name, fw->programs[i].app_mode);
	}

	return data - start;
}

/*
 * Parse audio configurations
 */
static int fw_parse_configs(struct tas2557_priv *tas2557,
			    struct tas2557_firmware *fw,
			    const u8 *data, size_t remaining)
{
	const u8 *start = data;
	size_t desc_len;
	unsigned int i;
	int ret;

	if (remaining < 2)
		return -EINVAL;

	fw->num_configs = fw_get_be16(data);
	data += 2;
	remaining -= 2;

	if (fw->num_configs == 0) {
		dev_err(tas2557->dev, "firmware contains no configurations\n");
		return -EINVAL;
	}

	fw->configs = kcalloc(fw->num_configs, sizeof(struct tas2557_config),
			      GFP_KERNEL);
	if (!fw->configs)
		return -ENOMEM;

	for (i = 0; i < fw->num_configs; i++) {
		if (remaining < 64)
			return -EINVAL;

		memcpy(fw->configs[i].name, data, 64);
		data += 64;
		remaining -= 64;

		desc_len = strnlen(data, remaining);
		if (desc_len > 0) {
			fw->configs[i].description = kmemdup(data, desc_len + 1, GFP_KERNEL);
			if (!fw->configs[i].description)
				return -ENOMEM;
		}
		data += desc_len + 1;
		remaining -= desc_len + 1;

		/* Device count field - 2 bytes if driver >= CONFDEV */
		if (fw->driver_version >= PPC_DRIVER_CONFDEV) {
			if (remaining < 2)
				return -EINVAL;
			/* Skip devices field (2 bytes) - single device assumed */
			data += 2;
			remaining -= 2;
		}

		if (remaining < 6)
			return -EINVAL;

		fw->configs[i].program = data[0];
		data += 1;
		remaining -= 1;

		fw->configs[i].pll = data[0];
		data += 1;
		remaining -= 1;

		fw->configs[i].sample_rate = fw_get_be32(data);
		data += 4;
		remaining -= 4;

		/* PLL source fields if driver >= MTPLLSRC */
		if (fw->driver_version >= PPC_DRIVER_MTPLLSRC) {
			if (remaining < 5)
				return -EINVAL;

			fw->configs[i].pll_src = data[0];
			data += 1;
			remaining -= 1;

			fw->configs[i].pll_src_rate = fw_get_be32(data);
			data += 4;
			remaining -= 4;
		}

		ret = fw_parse_data(tas2557, fw, &fw->configs[i].data,
				    data, remaining);
		if (ret < 0)
			return ret;
		data += ret;
		remaining -= ret;

		dev_dbg(tas2557->dev, "config[%u]: %s, program=%u, rate=%u\n",
			i, fw->configs[i].name, fw->configs[i].program,
			fw->configs[i].sample_rate);
	}

	return data - start;
}

/*
 * Main firmware parser
 */
static struct tas2557_firmware *fw_parse(struct tas2557_priv *tas2557,
					 const u8 *data, size_t size)
{
	struct tas2557_firmware *fw;
	size_t remaining = size;
	int ret;

	fw = kzalloc(sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return ERR_PTR(-ENOMEM);

	ret = fw_parse_header(tas2557, fw, data, remaining);
	if (ret < 0)
		goto err;
	data += ret;
	remaining -= ret;

	ret = fw_parse_plls(tas2557, fw, data, remaining);
	if (ret < 0)
		goto err;
	data += ret;
	remaining -= ret;

	ret = fw_parse_programs(tas2557, fw, data, remaining);
	if (ret < 0)
		goto err;
	data += ret;
	remaining -= ret;

	ret = fw_parse_configs(tas2557, fw, data, remaining);
	if (ret < 0)
		goto err;

	return fw;

err:
	tas2557_fw_free(fw);
	return ERR_PTR(ret);
}

/*
 * Load a firmware block to the device
 */
static int tas2557_load_block(struct tas2557_priv *tas2557,
			      struct tas2557_block *block)
{
	const u8 *data = block->data;
	unsigned int i;
	u8 book, page, offset, value;
	u16 sleep_time, bulk_len;
	int ret;

	dev_dbg(tas2557->dev, "loading block type=0x%x, commands=%u\n",
		block->type, block->num_commands);

	for (i = 0; i < block->num_commands; ) {
		book = data[i * 4];
		page = data[i * 4 + 1];
		offset = data[i * 4 + 2];
		value = data[i * 4 + 3];
		i++;

		if (offset <= 0x7f) {
			/* Single register write */
			ret = tas2557_dev_write(tas2557,
						TAS2557_REG(book, page, offset),
						value);
			if (ret < 0)
				return ret;
		} else if (offset == 0x81) {
			/* Sleep command: time in ms encoded in book:page */
			sleep_time = (book << 8) | page;
			msleep(sleep_time);
		} else if (offset == 0x85) {
			/* Bulk write: length in book:page, then data */
			bulk_len = (book << 8) | page;

			if (i >= block->num_commands)
				return -EINVAL;

			book = data[i * 4];
			page = data[i * 4 + 1];
			offset = data[i * 4 + 2];
			/* value = data[i * 4 + 3]; - first data byte */

			if (bulk_len > 1) {
				ret = tas2557_dev_bulk_write(tas2557,
							     TAS2557_REG(book, page, offset),
							     &data[i * 4 + 3],
							     bulk_len);
				if (ret < 0)
					return ret;
			} else {
				ret = tas2557_dev_write(tas2557,
							TAS2557_REG(book, page, offset),
							data[i * 4 + 3]);
				if (ret < 0)
					return ret;
			}

			/* Skip the bulk data commands */
			i += (bulk_len + 3) / 4;
		}
	}

	return 0;
}

/*
 * Load firmware data (blocks) to device
 */
static int tas2557_load_fw_data(struct tas2557_priv *tas2557,
				struct tas2557_data *img_data,
				unsigned int block_type)
{
	unsigned int i;
	int ret;

	for (i = 0; i < img_data->num_blocks; i++) {
		if (block_type != 0 &&
		    img_data->blocks[i].type != block_type &&
		    img_data->blocks[i].type != TAS2557_BLOCK_PGM_ALL)
			continue;

		ret = tas2557_load_block(tas2557, &img_data->blocks[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*
 * Load PLL configuration
 */
static int tas2557_load_pll(struct tas2557_priv *tas2557, unsigned int pll_idx)
{
	if (!tas2557->fw || pll_idx >= tas2557->fw->num_plls)
		return -EINVAL;

	dev_dbg(tas2557->dev, "loading PLL[%u]: %s\n",
		pll_idx, tas2557->fw->plls[pll_idx].name);

	return tas2557_load_block(tas2557, &tas2557->fw->plls[pll_idx].block);
}

/*
 * Set program and configuration
 */
static int tas2557_set_program(struct tas2557_priv *tas2557,
			       unsigned int prog_idx, int config_idx)
{
	struct tas2557_firmware *fw = tas2557->fw;
	struct tas2557_program *program;
	struct tas2557_config *config;
	unsigned int cfg_idx;
	int ret;

	if (!fw || !fw->programs || !fw->configs) {
		dev_err(tas2557->dev, "firmware not loaded\n");
		return -EINVAL;
	}

	if (prog_idx >= fw->num_programs) {
		dev_err(tas2557->dev, "program %u doesn't exist\n", prog_idx);
		return -EINVAL;
	}

	/* Find matching configuration */
	if (config_idx < 0) {
		/* Auto-select based on sample rate */
		for (cfg_idx = 0; cfg_idx < fw->num_configs; cfg_idx++) {
			if (fw->configs[cfg_idx].program == prog_idx) {
				if (tas2557->sample_rate == 0 ||
				    tas2557->sample_rate == fw->configs[cfg_idx].sample_rate) {
					config_idx = cfg_idx;
					break;
				}
			}
		}
		if (config_idx < 0) {
			dev_err(tas2557->dev, "no matching config for program %u\n",
				prog_idx);
			return -EINVAL;
		}
	}

	if (config_idx >= fw->num_configs) {
		dev_err(tas2557->dev, "config %d doesn't exist\n", config_idx);
		return -EINVAL;
	}

	program = &fw->programs[prog_idx];
	config = &fw->configs[config_idx];

	if (config->program != prog_idx) {
		dev_err(tas2557->dev, "config %u doesn't match program %u\n",
			config_idx, prog_idx);
		return -EINVAL;
	}

	/* Power down if currently powered */
	if (tas2557->powered) {
		ret = tas2557_load_data(tas2557, tas2557_shutdown_data);
		if (ret < 0)
			return ret;
	}

	/* Hardware reset */
	tas2557_hw_reset(tas2557);

	/* Software reset */
	ret = tas2557_dev_write(tas2557, TAS2557_SW_RESET_REG, 0x01);
	if (ret < 0)
		return ret;
	msleep(1);

	/* Load defaults */
	ret = tas2557_load_data(tas2557, tas2557_default_data);
	if (ret < 0)
		return ret;

	/* Load program */
	dev_dbg(tas2557->dev, "loading program %u: %s\n", prog_idx, program->name);
	ret = tas2557_load_fw_data(tas2557, &program->data, TAS2557_BLOCK_PGM_DEV_A);
	if (ret < 0)
		return ret;

	tas2557->current_program = prog_idx;

	/* Load PLL */
	if (config->pll < fw->num_plls) {
		ret = tas2557_load_pll(tas2557, config->pll);
		if (ret < 0)
			return ret;
	}

	/* Load configuration */
	dev_dbg(tas2557->dev, "loading config %u: %s (rate=%u)\n",
		config_idx, config->name, config->sample_rate);
	ret = tas2557_load_fw_data(tas2557, &config->data, 0);
	if (ret < 0)
		return ret;

	tas2557->current_config = config_idx;

	/* Power up if was powered */
	if (tas2557->powered) {
		ret = tas2557_load_data(tas2557, tas2557_startup_data);
		if (ret < 0)
			return ret;
		ret = tas2557_load_data(tas2557, tas2557_unmute_data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*
 * Firmware loading callback
 */
static void tas2557_fw_ready(const struct firmware *fw_entry, void *context)
{
	struct tas2557_priv *tas2557 = context;
	struct tas2557_firmware *fw;
	unsigned int i;
	int ret;

	if (!fw_entry || !fw_entry->data || fw_entry->size == 0) {
		dev_warn(tas2557->dev, "firmware not available, using defaults\n");
		return;
	}

	dev_dbg(tas2557->dev, "firmware loaded, size=%zu\n", fw_entry->size);

	fw = fw_parse(tas2557, fw_entry->data, fw_entry->size);
	release_firmware(fw_entry);

	if (IS_ERR(fw)) {
		dev_err(tas2557->dev, "failed to parse firmware: %ld\n",
			PTR_ERR(fw));
		return;
	}

	/* Free old firmware if any */
	if (tas2557->fw)
		tas2557_fw_free(tas2557->fw);

	tas2557->fw = fw;
	tas2557->fw_loaded = true;

	/* Log firmware contents */
	dev_info(tas2557->dev, "firmware: %u programs, %u configs\n",
		 fw->num_programs, fw->num_configs);
	for (i = 0; i < fw->num_programs; i++)
		dev_info(tas2557->dev, "  program %u: %s\n",
			 i, fw->programs[i].description ?: "(unnamed)");
	for (i = 0; i < fw->num_configs; i++)
		dev_info(tas2557->dev, "  config %u: %s (prog=%u, rate=%u, pll_src=%u)\n",
			 i, fw->configs[i].description ?: "(unnamed)",
			 fw->configs[i].program, fw->configs[i].sample_rate,
			 fw->configs[i].pll_src);

	/* Load first program */
	ret = tas2557_set_program(tas2557, 0, -1);
	if (ret < 0) {
		dev_err(tas2557->dev, "failed to load program: %d\n", ret);
		return;
	}

	dev_info(tas2557->dev, "firmware initialized successfully\n");
}

/*
 * Hardware reset
 */
static void tas2557_hw_reset(struct tas2557_priv *tas2557)
{
	if (!tas2557->reset_gpio) {
		/* Still reset book/page tracking */
		mutex_lock(&tas2557->dev_lock);
		tas2557->current_book = 0xff;
		tas2557->current_page = 0xff;
		mutex_unlock(&tas2557->dev_lock);
		return;
	}

	/* Hold mutex to prevent I2C access during reset */
	mutex_lock(&tas2557->dev_lock);

	/* Assert reset (TAS2557 reset is active-low, DTS uses GPIO_ACTIVE_HIGH) */
	gpiod_set_value_cansleep(tas2557->reset_gpio, 0);
	msleep(5);
	/* Release reset */
	gpiod_set_value_cansleep(tas2557->reset_gpio, 1);
	msleep(10);

	/* Reset book/page tracking after hardware reset */
	tas2557->current_book = 0xff;
	tas2557->current_page = 0xff;

	mutex_unlock(&tas2557->dev_lock);

	if (tas2557->err_code)
		dev_info(tas2557->dev, "before reset, err_code=0x%x\n",
			 tas2557->err_code);
	tas2557->err_code = 0;
}

/*
 * IRQ handling
 */
static void tas2557_irq_work_func(struct work_struct *work)
{
	struct tas2557_priv *tas2557 = container_of(work, struct tas2557_priv,
						    irq_work.work);
	unsigned int int1_status = 0, int2_status = 0;
	unsigned int power_flag = 0;
	int ret;

	if (!tas2557->powered) {
		dev_dbg(tas2557->dev, "device not powered\n");
		return;
	}

	/* Disable IRQ output during processing */
	tas2557_dev_write(tas2557, TAS2557_GPIO4_PIN_REG, 0x00);

	/* Read interrupt status */
	ret = tas2557_dev_read(tas2557, TAS2557_FLAGS_1, &int1_status);
	if (ret >= 0)
		ret = tas2557_dev_read(tas2557, TAS2557_FLAGS_2, &int2_status);

	if (ret < 0) {
		dev_err(tas2557->dev, "failed to read interrupt status\n");
		goto reset;
	}

	/* Check for critical errors */
	if ((int1_status & 0xfc) || (int2_status & 0x0c)) {
		dev_err(tas2557->dev, "critical error: 0x%x, 0x%x\n",
			int1_status, int2_status);

		if (int1_status & 0x80) {
			tas2557->err_code |= ERROR_OVER_CURRENT;
			dev_err(tas2557->dev, "speaker over current!\n");
		}
		if (int1_status & 0x40) {
			tas2557->err_code |= ERROR_UNDER_VOLTAGE;
			dev_err(tas2557->dev, "speaker under voltage!\n");
		}
		if (int1_status & 0x20) {
			tas2557->err_code |= ERROR_CLK_HALT;
			dev_err(tas2557->dev, "clock halted!\n");
		}
		if (int1_status & 0x10) {
			tas2557->err_code |= ERROR_DIE_OVERTEMP;
			dev_err(tas2557->dev, "die over temperature!\n");
		}
		if (int1_status & 0x08) {
			tas2557->err_code |= ERROR_BROWNOUT;
			dev_err(tas2557->dev, "brownout!\n");
		}
		if (int1_status & 0x04) {
			tas2557->err_code |= ERROR_CLK_LOST;
			dev_err(tas2557->dev, "clock lost!\n");
		}

		goto reset;
	}

	/* Check power up flag */
	ret = tas2557_dev_read(tas2557, TAS2557_POWER_UP_FLAG_REG, &power_flag);
	if (ret < 0)
		goto reset;

	if ((power_flag & 0xc0) != 0xc0) {
		dev_err(tas2557->dev, "power up failed: 0x%x\n", power_flag);
		tas2557->err_code |= ERROR_CLASSD_PWR;
		goto reset;
	}

	/* Re-enable IRQ output */
	tas2557_dev_write(tas2557, TAS2557_GPIO4_PIN_REG, 0x07);
	if (tas2557->irq_enabled && tas2557->irq)
		enable_irq(tas2557->irq);
	return;

reset:
	/* Attempt failsafe recovery */
	if (tas2557_failsafe_recovery(tas2557) == 0) {
		/* Recovery successful, re-enable IRQ */
		tas2557_dev_write(tas2557, TAS2557_GPIO4_PIN_REG, 0x07);
		if (tas2557->irq_enabled && tas2557->irq)
			enable_irq(tas2557->irq);
	}
}

static irqreturn_t tas2557_irq_handler(int irq, void *data)
{
	struct tas2557_priv *tas2557 = data;

	if (tas2557->irq_enabled) {
		disable_irq_nosync(tas2557->irq);
		schedule_delayed_work(&tas2557->irq_work, msecs_to_jiffies(100));
	}

	return IRQ_HANDLED;
}

/*
 * Die temperature reading
 *
 * The TAS2557 DSP calculates die temperature from internal sensors.
 * Temperature is stored as a 32-bit fixed-point value in book 130, page 2.
 * Formula: temp_C = (value >> 23) - 273 (approximate)
 */
static int tas2557_get_die_temp(struct tas2557_priv *tas2557, int *temp)
{
	unsigned char buf[4];
	int ret;
	int raw_temp;

	ret = tas2557_dev_bulk_read(tas2557, TAS2557_DIE_TEMP_REG, buf, 4);
	if (ret < 0)
		return ret;

	raw_temp = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];

	/* Convert to Celsius: value is in Q8.23 format representing Kelvin */
	*temp = (raw_temp >> 23) - 273;

	return 0;
}

/*
 * Temperature monitoring work function
 */
static void tas2557_temp_work_func(struct work_struct *work)
{
	struct tas2557_priv *tas2557 = container_of(work, struct tas2557_priv,
						    temp_work);
	int temp;
	int ret;

	if (!tas2557->powered || !tas2557->temp_monitor_enabled)
		return;

	ret = tas2557_get_die_temp(tas2557, &temp);
	if (ret < 0) {
		dev_warn(tas2557->dev, "failed to read die temp: %d\n", ret);
		return;
	}

	tas2557->die_temp = temp;

	/* Check for over-temperature condition */
	if (temp > TAS2557_SAFE_TEMP_HIGH) {
		dev_err(tas2557->dev, "die over-temperature: %d C, muting\n", temp);
		tas2557->err_code |= ERROR_DIE_OVERTEMP;
		tas2557_dev_update_bits(tas2557, TAS2557_MUTE_REG,
					TAS2557_CLASSD_MUTE, TAS2557_CLASSD_MUTE);
	}
}

static enum hrtimer_restart tas2557_temp_timer_callback(struct hrtimer *timer)
{
	struct tas2557_priv *tas2557 = container_of(timer, struct tas2557_priv,
						    temp_timer);

	if (tas2557->powered && tas2557->temp_monitor_enabled) {
		schedule_work(&tas2557->temp_work);
		hrtimer_forward_now(timer, ms_to_ktime(TAS2557_TEMP_CHECK_PERIOD));
		return HRTIMER_RESTART;
	}

	return HRTIMER_NORESTART;
}

static void tas2557_start_temp_monitor(struct tas2557_priv *tas2557)
{
	if (!tas2557->temp_monitor_enabled)
		return;

	hrtimer_start(&tas2557->temp_timer, ms_to_ktime(TAS2557_TEMP_CHECK_PERIOD),
		      HRTIMER_MODE_REL);
}

static void tas2557_stop_temp_monitor(struct tas2557_priv *tas2557)
{
	hrtimer_cancel(&tas2557->temp_timer);
	cancel_work_sync(&tas2557->temp_work);
}

/*
 * Failsafe recovery - attempt to recover from errors
 */
static int tas2557_failsafe_recovery(struct tas2557_priv *tas2557)
{
	int ret;

	dev_info(tas2557->dev, "attempting failsafe recovery (count=%u)\n",
		 tas2557->restart_count);

	tas2557->restart_count++;

	/* Limit recovery attempts */
	if (tas2557->restart_count > 5) {
		dev_err(tas2557->dev, "too many recovery attempts, giving up\n");
		return -EIO;
	}

	/* Hardware reset */
	tas2557_hw_reset(tas2557);

	/* Reload defaults */
	ret = tas2557_load_data(tas2557, tas2557_default_data);
	if (ret < 0)
		return ret;

	/* Reload firmware program if available */
	if (tas2557->fw_loaded && tas2557->fw) {
		ret = tas2557_set_program(tas2557, tas2557->current_program,
					  tas2557->current_config);
		if (ret < 0)
			return ret;
	}

	/* Re-enable if was powered */
	if (tas2557->powered) {
		ret = tas2557_load_data(tas2557, tas2557_startup_data);
		if (ret < 0)
			return ret;

		ret = tas2557_load_data(tas2557, tas2557_unmute_data);
		if (ret < 0)
			return ret;
	}

	tas2557->err_code = ERROR_NONE;
	dev_info(tas2557->dev, "recovery successful\n");

	return 0;
}

/*
 * Power management
 */
static int tas2557_enable(struct tas2557_priv *tas2557, bool enable)
{
	int ret = 0;

	dev_info(tas2557->dev, "tas2557_enable: enable=%d, powered=%d\n",
		 enable, tas2557->powered);

	if (enable && !tas2557->powered) {
		dev_info(tas2557->dev, "powering on amplifier\n");
		/* Stop temperature monitoring during power-up */
		tas2557_stop_temp_monitor(tas2557);

		ret = tas2557_load_data(tas2557, tas2557_startup_data);
		if (ret < 0) {
			dev_err(tas2557->dev, "startup failed: %d\n", ret);
			return ret;
		}
		dev_info(tas2557->dev, "startup sequence complete\n");

		/* Apply DAC gain setting */
		ret = tas2557_dev_update_bits(tas2557, TAS2557_SPK_CTRL_REG,
					      TAS2557_DAC_GAIN_MASK,
					      tas2557->dac_gain << TAS2557_DAC_GAIN_SHIFT);
		if (ret < 0)
			dev_warn(tas2557->dev, "failed to set gain: %d\n", ret);
		else
			dev_info(tas2557->dev, "DAC gain set to %u\n", tas2557->dac_gain);

		/* Configure sense slots if enabled */
		if (tas2557->isense_enabled || tas2557->vsense_enabled) {
			unsigned int sns_val = 0;

			if (tas2557->isense_enabled)
				sns_val |= (tas2557->imon_slot << TAS2557_ISNS_SLOT_SHIFT);
			if (tas2557->vsense_enabled)
				sns_val |= (tas2557->vmon_slot << TAS2557_VSNS_SLOT_SHIFT);

			tas2557_dev_write(tas2557, TAS2557_SNS_CTRL_REG, sns_val);
		}

		ret = tas2557_load_data(tas2557, tas2557_unmute_data);
		if (ret < 0) {
			dev_err(tas2557->dev, "unmute failed: %d\n", ret);
			return ret;
		}
		/* Verify soft mute was cleared (Book 100 register) */
		{
			unsigned int soft_mute;
			tas2557_dev_read(tas2557, TAS2557_SOFT_MUTE_REG, &soft_mute);
			dev_info(tas2557->dev, "SOFT_MUTE_REG (Book 100) = 0x%02x\n", soft_mute);
		}
		dev_info(tas2557->dev, "unmute sequence complete\n");

		tas2557->powered = true;
		tas2557->muted = false;
		tas2557->restart_count = 0;

		/* Start temperature monitoring */
		tas2557_start_temp_monitor(tas2557);

		dev_info(tas2557->dev, "amplifier powered on successfully\n");

		/*
		 * Try to make DSP use ASI2 instead of ASI1.
		 * ASI_CTL1 (B0P0R42) might control ASI selection for the DSP.
		 * Try different values to see what selects ASI2.
		 */
		{
			unsigned int asi_ctl1_before, asi_ctl1_after;
			int i;

			tas2557_dev_read(tas2557, TAS2557_ASI_CTL1_REG, &asi_ctl1_before);
			dev_info(tas2557->dev, "ASI_CTL1 before: 0x%02x\n", asi_ctl1_before);

			/* Try setting bit 0 or bit 1 to select ASI2 */
			for (i = 1; i <= 3; i++) {
				tas2557_dev_write(tas2557, TAS2557_ASI_CTL1_REG, i);
				tas2557_dev_read(tas2557, TAS2557_ASI_CTL1_REG, &asi_ctl1_after);
				dev_info(tas2557->dev, "ASI_CTL1: wrote 0x%02x, read 0x%02x\n",
					 i, asi_ctl1_after);
			}

			/* Set to value 3 (might select ASI2) */
			tas2557_dev_write(tas2557, TAS2557_ASI_CTL1_REG, 0x03);
			tas2557_dev_read(tas2557, TAS2557_ASI_CTL1_REG, &asi_ctl1_after);
			dev_info(tas2557->dev, "ASI_CTL1 final: 0x%02x\n", asi_ctl1_after);
		}

		/* Debug: read back key registers to verify state */
		{
			unsigned int pwr1, pwr2, mute, pwr_flag, flags1, flags2;
			unsigned int gpi, gpio1, gpio2, gpio5, gpio6, gpio7, gpio8;
			unsigned int asi1_fmt, asi2_fmt;

			tas2557_dev_read(tas2557, TAS2557_POWER_CTRL1_REG, &pwr1);
			tas2557_dev_read(tas2557, TAS2557_POWER_CTRL2_REG, &pwr2);
			tas2557_dev_read(tas2557, TAS2557_MUTE_REG, &mute);
			tas2557_dev_read(tas2557, TAS2557_POWER_UP_FLAG_REG, &pwr_flag);
			tas2557_dev_read(tas2557, TAS2557_FLAGS_1, &flags1);
			tas2557_dev_read(tas2557, TAS2557_FLAGS_2, &flags2);
			dev_info(tas2557->dev, "regs: PWR1=0x%02x PWR2=0x%02x MUTE=0x%02x FLAG=0x%02x\n",
				 pwr1, pwr2, mute, pwr_flag);
			dev_info(tas2557->dev, "status: FLAGS1=0x%02x FLAGS2=0x%02x\n",
				 flags1, flags2);
			if (flags1 & 0x04)
				dev_warn(tas2557->dev, "WARNING: Clock error detected!\n");

			/* Read GPIO configuration after startup */
			tas2557_dev_read(tas2557, TAS2557_GPI_PIN_REG, &gpi);
			tas2557_dev_read(tas2557, TAS2557_GPIO1_PIN_REG, &gpio1);
			tas2557_dev_read(tas2557, TAS2557_GPIO2_PIN_REG, &gpio2);
			tas2557_dev_read(tas2557, TAS2557_GPIO5_PIN_REG, &gpio5);
			tas2557_dev_read(tas2557, TAS2557_GPIO6_PIN_REG, &gpio6);
			tas2557_dev_read(tas2557, TAS2557_GPIO7_PIN_REG, &gpio7);
			tas2557_dev_read(tas2557, TAS2557_GPIO8_PIN_REG, &gpio8);
			tas2557_dev_read(tas2557, TAS2557_ASI1_DAC_FORMAT_REG, &asi1_fmt);
			tas2557_dev_read(tas2557, TAS2557_ASI2_DAC_FORMAT_REG, &asi2_fmt);

			dev_info(tas2557->dev, "post-startup GPIO: gpi=0x%02x gpio1=0x%02x gpio2=0x%02x\n",
				 gpi, gpio1, gpio2);
			dev_info(tas2557->dev, "post-startup GPIO5-8: 0x%02x 0x%02x 0x%02x 0x%02x\n",
				 gpio5, gpio6, gpio7, gpio8);
			dev_info(tas2557->dev, "post-startup ASI: ASI1_FMT=0x%02x ASI2_FMT=0x%02x\n",
				 asi1_fmt, asi2_fmt);
		}

		/* Debug: Read DSP/DAC source configuration registers */
		{
			unsigned int dsp_mode, asi_ctl1, dac_interpol;
			unsigned int main_clkin, pll_clkin;
			unsigned int asi1_mux, asi2_mux;

			tas2557_dev_read(tas2557, TAS2557_DSP_MODE_SELECT_REG, &dsp_mode);
			tas2557_dev_read(tas2557, TAS2557_ASI_CTL1_REG, &asi_ctl1);
			tas2557_dev_read(tas2557, TAS2557_DAC_INTERPOL_REG, &dac_interpol);
			tas2557_dev_read(tas2557, TAS2557_MAIN_CLKIN_REG, &main_clkin);
			tas2557_dev_read(tas2557, TAS2557_PLL_CLKIN_REG, &pll_clkin);
			tas2557_dev_read(tas2557, TAS2557_ASI1_DIN_DOUT_MUX_REG, &asi1_mux);
			tas2557_dev_read(tas2557, TAS2557_ASI2_DIN_DOUT_MUX_REG, &asi2_mux);
			dev_info(tas2557->dev,
				 "DEBUG DSP: DSP_MODE=0x%02x ASI_CTL1=0x%02x DAC_INTERPOL=0x%02x\n",
				 dsp_mode, asi_ctl1, dac_interpol);
			dev_info(tas2557->dev,
				 "DEBUG CLK: MAIN_CLKIN=0x%02x PLL_CLKIN=0x%02x\n",
				 main_clkin, pll_clkin);
			dev_info(tas2557->dev,
				 "DEBUG MUX: ASI1_MUX=0x%02x ASI2_MUX=0x%02x\n",
				 asi1_mux, asi2_mux);
		}

	} else if (!enable && tas2557->powered) {
		/* Stop temperature monitoring */
		tas2557_stop_temp_monitor(tas2557);

		ret = tas2557_load_data(tas2557, tas2557_shutdown_data);
		if (ret < 0) {
			dev_err(tas2557->dev, "shutdown failed: %d\n", ret);
			return ret;
		}

		tas2557->powered = false;
		tas2557->muted = true;

		dev_dbg(tas2557->dev, "powered off\n");
	}

	return ret;
}

/*
 * Sample rate configuration
 */
static int tas2557_set_sample_rate(struct tas2557_priv *tas2557,
				   unsigned int sample_rate)
{
	/* Sample rate is typically set through firmware/PLL configuration */
	tas2557->sample_rate = sample_rate;
	dev_dbg(tas2557->dev, "sample rate set to %u\n", sample_rate);
	return 0;
}

/*
 * Bit rate configuration
 *
 * Note: ASI2 format is set to 32-bit in tas2557_startup_data to match the
 * Q6AFE MI2S frame format. This function runs during hw_params BEFORE
 * power-up, so writes may not persist. The startup sequence is authoritative.
 */
static int tas2557_set_bit_rate(struct tas2557_priv *tas2557,
				unsigned int bit_rate)
{
	int n = -1;
	int ret;
	unsigned int readback;

	dev_info(tas2557->dev,
		 "DEBUG: set_bit_rate(%u) called, powered=%d\n",
		 bit_rate, tas2557->powered);

	switch (bit_rate) {
	case 16:
		n = 0;
		break;
	case 20:
		n = 1;
		break;
	case 24:
		n = 2;
		break;
	case 32:
		n = 3;
		break;
	default:
		dev_err(tas2557->dev, "unsupported bit rate: %u\n", bit_rate);
		return -EINVAL;
	}

	/* Set ASI1 format */
	ret = tas2557_dev_update_bits(tas2557, TAS2557_ASI1_DAC_FORMAT_REG,
				      TAS2557_WORDLENGTH_MASK, n << 3);
	if (ret < 0)
		return ret;

	/*
	 * Note: ASI2 format is set to 32-bit in startup_data.
	 * This write during hw_params may not persist if device isn't powered.
	 * Log for debugging but don't worry if it shows mismatch.
	 */
	ret = tas2557_dev_update_bits(tas2557, TAS2557_ASI2_DAC_FORMAT_REG,
				      TAS2557_WORDLENGTH_MASK, n << 3);

	/* Debug: verify ASI2 write */
	tas2557_dev_read(tas2557, TAS2557_ASI2_DAC_FORMAT_REG, &readback);
	dev_info(tas2557->dev,
		 "DEBUG: ASI2_DAC_FORMAT after hw_params: wrote 0x%02x, read 0x%02x\n",
		 n << 3, readback);

	dev_info(tas2557->dev, "ASI format set to %u-bit (n=%d)\n", bit_rate, n);
	return ret;
}

/*
 * ALSA SoC DAI operations
 */
static int tas2557_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	unsigned int asi_fmt;
	int ret;

	dev_info(tas2557->dev, "hw_params: rate=%u, format=%u, width=%d\n",
		 params_rate(params), params_format(params),
		 snd_pcm_format_width(params_format(params)));

	ret = tas2557_set_sample_rate(tas2557, params_rate(params));
	if (ret < 0)
		return ret;

	/*
	 * Use the I2S slot width from DT (ti,i2s-bits), not the stream format width.
	 * The Q6AFE sends I2S frames with the configured slot width, and the actual
	 * audio data is left-aligned within each slot.
	 */
	ret = tas2557_set_bit_rate(tas2557, tas2557->i2s_bits);
	if (ret < 0)
		return ret;

	/* Configure ASI DAC offset based on channel selection */
	/* For I2S with 32-bit slots: left=offset 0, right=offset 32 bits (4 bytes) */
	{
		unsigned int offset;

		if (tas2557->channel == 0)
			offset = 0;  /* Left channel */
		else
			offset = tas2557->i2s_bits / 8;  /* Right channel */

		/* Configure ASI1 offset */
		ret = tas2557_dev_write(tas2557, TAS2557_ASI1_OFFSET1_REG, offset);
		if (ret < 0)
			dev_warn(tas2557->dev, "failed to set ASI1 offset: %d\n", ret);

		/* Configure ASI2 offset - hardware uses ASI2 interface */
		ret = tas2557_dev_write(tas2557, TAS2557_ASI2_OFFSET1_REG, offset);
		if (ret < 0)
			dev_warn(tas2557->dev, "failed to set ASI2 offset: %d\n", ret);
		else
			dev_info(tas2557->dev, "ASI2 DAC offset set to %u (%s channel, %u-bit)\n",
				 offset, tas2557->channel ? "right" : "left", tas2557->i2s_bits);
	}

	/*
	 * Set clock sources to use ASI2 interface (hardware uses ASI2 for audio).
	 * Clock source values select GPIO pins:
	 *   0 = GPIO1 (ASI1 BCLK), 1 = GPIO2 (ASI1 WCLK)
	 *   4 = GPIO5 (ASI2 BCLK), 5 = GPIO6 (ASI2 WCLK)
	 *   13 = GPI2 (MCLK), 15 = Internal OSC
	 *
	 * For ASI2, we need GPIO5 (ASI2 BCLK) = value 4
	 */
	ret = tas2557_dev_write(tas2557, TAS2557_MAIN_CLKIN_REG, 0x04);
	if (ret < 0)
		dev_warn(tas2557->dev, "failed to set MAIN_CLKIN: %d\n", ret);
	else
		dev_info(tas2557->dev, "MAIN_CLKIN set to GPIO5/ASI2 BCLK (0x04)\n");

	ret = tas2557_dev_write(tas2557, TAS2557_PLL_CLKIN_REG, 0x04);
	if (ret < 0)
		dev_warn(tas2557->dev, "failed to set PLL clock source: %d\n", ret);
	else
		dev_info(tas2557->dev, "PLL clock source set to GPIO5/ASI2 BCLK (0x04)\n");

	/*
	 * Configure PLL for BCLK input.
	 * BCLK = sample_rate * i2s_bits * 2 channels
	 * For 48kHz, 32-bit: BCLK = 48000 * 32 * 2 = 3,072,000 Hz
	 * Target PLL output: ~98.304 MHz
	 * PLL_OUT = BCLK * J.D / P = 3.072 MHz * 32 / 1 = 98.304 MHz
	 */
	ret = tas2557_dev_write(tas2557, TAS2557_PLL_P_VAL_REG, 1);
	if (ret < 0)
		dev_warn(tas2557->dev, "failed to set PLL P: %d\n", ret);
	ret = tas2557_dev_write(tas2557, TAS2557_PLL_J_VAL_REG, 32);
	if (ret < 0)
		dev_warn(tas2557->dev, "failed to set PLL J: %d\n", ret);
	ret = tas2557_dev_write(tas2557, TAS2557_PLL_D_VAL_MSB_REG, 0);
	if (ret < 0)
		dev_warn(tas2557->dev, "failed to set PLL D MSB: %d\n", ret);
	ret = tas2557_dev_write(tas2557, TAS2557_PLL_D_VAL_LSB_REG, 0);
	if (ret < 0)
		dev_warn(tas2557->dev, "failed to set PLL D LSB: %d\n", ret);
	/* Verify PLL settings were written */
	{
		unsigned int pll_p, pll_j, pll_d_msb, pll_d_lsb;
		tas2557_dev_read(tas2557, TAS2557_PLL_P_VAL_REG, &pll_p);
		tas2557_dev_read(tas2557, TAS2557_PLL_J_VAL_REG, &pll_j);
		tas2557_dev_read(tas2557, TAS2557_PLL_D_VAL_MSB_REG, &pll_d_msb);
		tas2557_dev_read(tas2557, TAS2557_PLL_D_VAL_LSB_REG, &pll_d_lsb);
		dev_info(tas2557->dev, "PLL configured: P=%u, J=%u, D=%u.%u\n",
			 pll_p, pll_j, pll_d_msb, pll_d_lsb);
	}

	/* Debug: read ASI configuration registers */
	tas2557_dev_read(tas2557, TAS2557_ASI1_DAC_FORMAT_REG, &asi_fmt);
	dev_info(tas2557->dev, "ASI1_DAC_FORMAT=0x%02x\n", asi_fmt);
	tas2557_dev_read(tas2557, TAS2557_ASI2_DAC_FORMAT_REG, &asi_fmt);
	dev_info(tas2557->dev, "ASI2_DAC_FORMAT=0x%02x\n", asi_fmt);

	{
		unsigned int offset1, offset2, gpi, gpio1, gpio2, pll_clkin;
		unsigned int gpio5, gpio6, gpio7, gpio8;
		tas2557_dev_read(tas2557, TAS2557_ASI1_OFFSET1_REG, &offset1);
		tas2557_dev_read(tas2557, TAS2557_ASI1_OFFSET2_REG, &offset2);
		tas2557_dev_read(tas2557, TAS2557_GPI_PIN_REG, &gpi);
		tas2557_dev_read(tas2557, TAS2557_GPIO1_PIN_REG, &gpio1);
		tas2557_dev_read(tas2557, TAS2557_GPIO2_PIN_REG, &gpio2);
		tas2557_dev_read(tas2557, TAS2557_GPIO5_PIN_REG, &gpio5);
		tas2557_dev_read(tas2557, TAS2557_GPIO6_PIN_REG, &gpio6);
		tas2557_dev_read(tas2557, TAS2557_GPIO7_PIN_REG, &gpio7);
		tas2557_dev_read(tas2557, TAS2557_GPIO8_PIN_REG, &gpio8);
		tas2557_dev_read(tas2557, TAS2557_PLL_CLKIN_REG, &pll_clkin);
		dev_info(tas2557->dev, "ASI: offset1=0x%02x offset2=0x%02x\n", offset1, offset2);
		dev_info(tas2557->dev, "GPIO ASI1: gpi=0x%02x gpio1=0x%02x gpio2=0x%02x\n",
			 gpi, gpio1, gpio2);
		dev_info(tas2557->dev, "GPIO ASI2: gpio5=0x%02x gpio6=0x%02x gpio7=0x%02x gpio8=0x%02x\n",
			 gpio5, gpio6, gpio7, gpio8);
	}

	return 0;
}

static int tas2557_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	unsigned int asi_fmt = 0;
	int ret;

	dev_info(tas2557->dev, "set_dai_fmt called: fmt=0x%x\n", fmt);
	dev_info(tas2557->dev, "  format_mask=0x%x, inv=0x%x, clock=0x%x\n",
		 fmt & SND_SOC_DAIFMT_FORMAT_MASK,
		 fmt & SND_SOC_DAIFMT_INV_MASK,
		 fmt & SND_SOC_DAIFMT_CLOCK_MASK);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		asi_fmt = TAS2557_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		asi_fmt = TAS2557_FORMAT_DSP;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		asi_fmt = TAS2557_FORMAT_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		asi_fmt = TAS2557_FORMAT_LEFT_J;
		break;
	default:
		dev_err(tas2557->dev, "unsupported DAI format\n");
		return -EINVAL;
	}

	ret = tas2557_dev_update_bits(tas2557, TAS2557_ASI1_DAC_FORMAT_REG,
				      TAS2557_FORMAT_MASK, asi_fmt);
	return ret;
}

static int tas2557_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
				unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	int rx_slot = -1;
	int tx_slot = -1;
	int ret;

	if (rx_mask) {
		rx_slot = ffs(rx_mask) - 1;
		if (rx_slot < 0 || rx_slot > 7) {
			dev_err(tas2557->dev, "invalid RX slot: %d\n", rx_slot);
			return -EINVAL;
		}
	}

	if (tx_mask) {
		tx_slot = ffs(tx_mask) - 1;
		if (tx_slot < 0 || tx_slot > 7) {
			dev_err(tas2557->dev, "invalid TX slot: %d\n", tx_slot);
			return -EINVAL;
		}
	}

	/* Validate slot width */
	switch (slot_width) {
	case 16:
	case 24:
	case 32:
		break;
	default:
		if (slot_width != 0) {
			dev_err(tas2557->dev, "unsupported slot width: %d\n", slot_width);
			return -EINVAL;
		}
	}

	tas2557->tdm_rx_slot = rx_slot;
	tas2557->tdm_tx_slot = tx_slot;
	tas2557->tdm_slot_width = slot_width;

	/* Configure DAC TDM offset (slot position) */
	if (rx_slot >= 0) {
		ret = tas2557_dev_write(tas2557, TAS2557_ASI1_OFFSET1_REG,
					rx_slot * slot_width / 8);
		if (ret < 0)
			return ret;
	}

	/* Configure ADC TDM offset for sense feedback */
	if (tx_slot >= 0) {
		ret = tas2557_dev_write(tas2557, TAS2557_ASI1_OFFSET2_REG,
					tx_slot * slot_width / 8);
		if (ret < 0)
			return ret;
	}

	dev_dbg(tas2557->dev, "TDM: rx_slot=%d tx_slot=%d width=%d\n",
		rx_slot, tx_slot, slot_width);

	return 0;
}

static int tas2557_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	dev_info(tas2557->dev, "mute_stream: mute=%d, direction=%d\n", mute, direction);
	return tas2557_enable(tas2557, !mute);
}

static const struct snd_soc_dai_ops tas2557_dai_ops = {
	.hw_params = tas2557_hw_params,
	.set_fmt = tas2557_set_dai_fmt,
	.set_tdm_slot = tas2557_set_tdm_slot,
	.mute_stream = tas2557_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver tas2557_dai = {
	.name = "tas2557-amplifier",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = TAS2557_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = TAS2557_FORMATS,
	},
	.ops = &tas2557_dai_ops,
};

/*
 * ALSA controls
 */

/* Volume control TLV: -28dB to 0dB in 2dB steps */
static DECLARE_TLV_DB_SCALE(tas2557_dac_tlv, -2800, 200, 0);

static int tas2557_power_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tas2557->powered;
	return 0;
}

static int tas2557_power_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	int power = ucontrol->value.integer.value[0];

	return tas2557_enable(tas2557, power != 0);
}

static int tas2557_volume_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tas2557->dac_gain;
	return 0;
}

static int tas2557_volume_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	unsigned int gain = ucontrol->value.integer.value[0];
	int ret = 0;

	if (gain > TAS2557_DAC_GAIN_MAX)
		return -EINVAL;

	tas2557->dac_gain = gain;

	/* Apply immediately if powered */
	if (tas2557->powered) {
		ret = tas2557_dev_update_bits(tas2557, TAS2557_SPK_CTRL_REG,
					      TAS2557_DAC_GAIN_MASK,
					      gain << TAS2557_DAC_GAIN_SHIFT);
	}

	return ret;
}

static int tas2557_program_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tas2557->current_program;
	return 0;
}

static int tas2557_program_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	unsigned int program = ucontrol->value.integer.value[0];

	if (!tas2557->fw_loaded || !tas2557->fw)
		return -ENODEV;

	if (program >= tas2557->fw->num_programs)
		return -EINVAL;

	return tas2557_set_program(tas2557, program, -1);
}

static int tas2557_config_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tas2557->current_config;
	return 0;
}

static int tas2557_config_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	unsigned int config = ucontrol->value.integer.value[0];

	if (!tas2557->fw_loaded || !tas2557->fw)
		return -ENODEV;

	if (config >= tas2557->fw->num_configs)
		return -EINVAL;

	return tas2557_set_program(tas2557, tas2557->current_program, config);
}

static int tas2557_temp_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	int temp;
	int ret;

	if (tas2557->powered) {
		ret = tas2557_get_die_temp(tas2557, &temp);
		if (ret == 0)
			tas2557->die_temp = temp;
	}

	ucontrol->value.integer.value[0] = tas2557->die_temp;
	return 0;
}

static int tas2557_isense_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tas2557->isense_enabled;
	return 0;
}

static int tas2557_isense_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	bool enable = ucontrol->value.integer.value[0];
	int ret = 0;

	tas2557->isense_enabled = enable;

	if (tas2557->powered) {
		ret = tas2557_dev_update_bits(tas2557, TAS2557_POWER_CTRL2_REG,
					      TAS2557_ISENSE_ENABLE,
					      enable ? TAS2557_ISENSE_ENABLE : 0);
	}

	return ret;
}

static int tas2557_vsense_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tas2557->vsense_enabled;
	return 0;
}

static int tas2557_vsense_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	bool enable = ucontrol->value.integer.value[0];
	int ret = 0;

	tas2557->vsense_enabled = enable;

	if (tas2557->powered) {
		ret = tas2557_dev_update_bits(tas2557, TAS2557_POWER_CTRL2_REG,
					      TAS2557_VSENSE_ENABLE,
					      enable ? TAS2557_VSENSE_ENABLE : 0);
	}

	return ret;
}

static const struct snd_kcontrol_new tas2557_controls[] = {
	SOC_SINGLE_EXT("Speaker Switch", SND_SOC_NOPM, 0, 1, 0,
		       tas2557_power_get, tas2557_power_put),
	SOC_SINGLE_EXT_TLV("Speaker Volume", SND_SOC_NOPM, 0, TAS2557_DAC_GAIN_MAX, 0,
			   tas2557_volume_get, tas2557_volume_put, tas2557_dac_tlv),
	SOC_SINGLE_EXT("DSP Program", SND_SOC_NOPM, 0, 255, 0,
		       tas2557_program_get, tas2557_program_put),
	SOC_SINGLE_EXT("DSP Configuration", SND_SOC_NOPM, 0, 255, 0,
		       tas2557_config_get, tas2557_config_put),
	SOC_SINGLE_EXT("Die Temperature", SND_SOC_NOPM, 0, 200, 0,
		       tas2557_temp_get, NULL),
	SOC_SINGLE_EXT("ISENSE Enable", SND_SOC_NOPM, 0, 1, 0,
		       tas2557_isense_get, tas2557_isense_put),
	SOC_SINGLE_EXT("VSENSE Enable", SND_SOC_NOPM, 0, 1, 0,
		       tas2557_vsense_get, tas2557_vsense_put),
};

/*
 * DAPM event handler for ClassD amplifier
 */
static int tas2557_classd_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	dev_info(tas2557->dev, "classd_event: event=%d\n", event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		dev_info(tas2557->dev, "DAPM: ClassD POST_PMU\n");
		return tas2557_enable(tas2557, true);
	case SND_SOC_DAPM_PRE_PMD:
		dev_info(tas2557->dev, "DAPM: ClassD PRE_PMD\n");
		return tas2557_enable(tas2557, false);
	default:
		return 0;
	}
}

/*
 * DAPM widgets
 */
static const struct snd_soc_dapm_widget tas2557_dapm_widgets[] = {
	/* Audio inputs */
	SND_SOC_DAPM_AIF_IN("ASI1", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI2", "Playback", 0, SND_SOC_NOPM, 0, 0),

	/* DAC */
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),

	/* Class-D amplifier with power management */
	SND_SOC_DAPM_OUT_DRV_E("ClassD", SND_SOC_NOPM, 0, 0, NULL, 0,
			       tas2557_classd_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* Sense feedback (current and voltage) */
	SND_SOC_DAPM_ADC("ISENSE", NULL, TAS2557_POWER_CTRL2_REG,
			 1, 0),
	SND_SOC_DAPM_ADC("VSENSE", NULL, TAS2557_POWER_CTRL2_REG,
			 0, 0),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_AIF_OUT("SENSE", "Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route tas2557_dapm_routes[] = {
	/* Playback path */
	{ "ASI1", NULL, "Playback" },
	{ "ASI2", NULL, "Playback" },
	{ "DAC", NULL, "ASI1" },
	{ "DAC", NULL, "ASI2" },
	{ "ClassD", NULL, "DAC" },
	{ "OUT", NULL, "ClassD" },

	/* Sense feedback path */
	{ "ISENSE", NULL, "ClassD" },
	{ "VSENSE", NULL, "ClassD" },
	{ "SENSE", NULL, "ISENSE" },
	{ "SENSE", NULL, "VSENSE" },
	{ "Capture", NULL, "SENSE" },
};

/*
 * Codec probe
 */
static int tas2557_codec_probe(struct snd_soc_component *component)
{
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	int ret;
	unsigned int pg_id;

	/* Hardware reset */
	tas2557_hw_reset(tas2557);

	/* Software reset */
	ret = tas2557_dev_write(tas2557, TAS2557_SW_RESET_REG, 0x01);
	if (ret < 0) {
		dev_err(tas2557->dev, "software reset failed: %d\n", ret);
		return ret;
	}
	msleep(1);

	/* Read PG ID */
	ret = tas2557_dev_read(tas2557, TAS2557_REV_PGID_REG, &pg_id);
	if (ret < 0) {
		dev_err(tas2557->dev, "failed to read PG ID: %d\n", ret);
		return ret;
	}

	tas2557->pg_id = pg_id;

	if (pg_id == TAS2557_PG_VERSION_2P1) {
		dev_info(tas2557->dev, "PG2.1 silicon detected\n");
	} else if (pg_id == TAS2557_PG_VERSION_1P0) {
		dev_info(tas2557->dev, "PG1.0 silicon detected\n");
	} else {
		dev_warn(tas2557->dev, "unknown silicon version: 0x%x\n", pg_id);
	}

	/* Load default configuration */
	ret = tas2557_load_data(tas2557, tas2557_default_data);
	if (ret < 0) {
		dev_err(tas2557->dev, "failed to load default config: %d\n", ret);
		return ret;
	}

	/* Configure IRQ */
	ret = tas2557_load_data(tas2557, tas2557_irq_config);
	if (ret < 0) {
		dev_warn(tas2557->dev, "failed to configure IRQ: %d\n", ret);
		/* Continue without IRQ */
	}

	/* Request firmware asynchronously */
	if (pg_id == TAS2557_PG_VERSION_1P0) {
		request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,
					TAS2557_PG1P0_FW_NAME, tas2557->dev,
					GFP_KERNEL, tas2557, tas2557_fw_ready);
	} else {
		request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,
					TAS2557_FW_NAME, tas2557->dev,
					GFP_KERNEL, tas2557, tas2557_fw_ready);
	}

	dev_info(tas2557->dev, "TAS2557 codec initialized\n");
	return 0;
}

#ifdef CONFIG_PM
static int tas2557_suspend(struct snd_soc_component *component)
{
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);

	if (tas2557->powered)
		tas2557_enable(tas2557, false);

	return 0;
}

static int tas2557_resume(struct snd_soc_component *component)
{
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(component);
	int ret;

	tas2557_hw_reset(tas2557);

	ret = tas2557_load_data(tas2557, tas2557_default_data);
	if (ret < 0) {
		dev_err(tas2557->dev, "failed to load defaults on resume: %d\n", ret);
		return ret;
	}

	return 0;
}
#else
#define tas2557_suspend NULL
#define tas2557_resume NULL
#endif

static const struct snd_soc_component_driver soc_component_tas2557 = {
	.probe = tas2557_codec_probe,
	.suspend = tas2557_suspend,
	.resume = tas2557_resume,
	.controls = tas2557_controls,
	.num_controls = ARRAY_SIZE(tas2557_controls),
	.dapm_widgets = tas2557_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2557_dapm_widgets),
	.dapm_routes = tas2557_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(tas2557_dapm_routes),
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
};

/*
 * I2C probe
 */
static int tas2557_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tas2557_priv *tas2557;
	int ret;

	tas2557 = devm_kzalloc(dev, sizeof(*tas2557), GFP_KERNEL);
	if (!tas2557)
		return -ENOMEM;

	tas2557->dev = dev;
	i2c_set_clientdata(client, tas2557);

	mutex_init(&tas2557->dev_lock);
	INIT_DELAYED_WORK(&tas2557->irq_work, tas2557_irq_work_func);

	/* Initialize temperature monitoring */
	hrtimer_setup(&tas2557->temp_timer, tas2557_temp_timer_callback,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	INIT_WORK(&tas2557->temp_work, tas2557_temp_work_func);
	tas2557->temp_monitor_enabled = true;

	/* Default gain setting (0dB) */
	tas2557->dac_gain = TAS2557_DAC_GAIN_MAX;

	/* Read sense slot configuration from DT */
	if (of_property_read_u32(dev->of_node, "ti,imon-slot", &tas2557->imon_slot))
		tas2557->imon_slot = 0;
	if (of_property_read_u32(dev->of_node, "ti,vmon-slot", &tas2557->vmon_slot))
		tas2557->vmon_slot = 2;

	/* Get regulator - required for device power */
	tas2557->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(tas2557->vdd)) {
		ret = PTR_ERR(tas2557->vdd);
		if (ret == -EPROBE_DEFER)
			return ret;
		tas2557->vdd = NULL;
		dev_dbg(dev, "no vdd regulator found\n");
	}

	/* Enable regulator if available */
	if (tas2557->vdd) {
		ret = regulator_enable(tas2557->vdd);
		if (ret) {
			dev_err(dev, "failed to enable vdd: %d\n", ret);
			return ret;
		}
		/* Wait for power to stabilize */
		usleep_range(5000, 10000);
	}

	/* Get reset GPIO - active high (deassert = device enabled) */
	tas2557->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(tas2557->reset_gpio)) {
		ret = PTR_ERR(tas2557->reset_gpio);
		if (ret == -EPROBE_DEFER)
			goto err_disable_reg;
		tas2557->reset_gpio = NULL;
		dev_dbg(dev, "no reset GPIO found\n");
	}

	/* Also try vendor-style GPIO name */
	if (!tas2557->reset_gpio) {
		tas2557->reset_gpio = devm_gpiod_get_optional(dev, "ti,cdc-reset",
							      GPIOD_OUT_HIGH);
		if (IS_ERR(tas2557->reset_gpio)) {
			ret = PTR_ERR(tas2557->reset_gpio);
			if (ret == -EPROBE_DEFER)
				goto err_disable_reg;
			tas2557->reset_gpio = NULL;
		}
	}

	tas2557->regmap = devm_regmap_init_i2c(client, &tas2557_regmap_config);
	if (IS_ERR(tas2557->regmap)) {
		ret = PTR_ERR(tas2557->regmap);
		dev_err(dev, "failed to allocate regmap: %d\n", ret);
		goto err_disable_reg;
	}

	/* Initialize book/page tracking */
	tas2557->current_book = 0xff;
	tas2557->current_page = 0xff;

	/* Read I2S bit width from DT if available */
	if (of_property_read_u32(dev->of_node, "ti,i2s-bits", &tas2557->i2s_bits))
		tas2557->i2s_bits = 16;

	/* Read channel selection from DT (0 = left, 1 = right) */
	if (of_property_read_u32(dev->of_node, "ti,channel", &tas2557->channel))
		tas2557->channel = 0;  /* default to left channel */
	dev_info(dev, "configured for %s channel, %u-bit I2S\n",
		 tas2557->channel ? "right" : "left", tas2557->i2s_bits);

	/* Setup IRQ if available from I2C client */
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq,
						NULL, tas2557_irq_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"tas2557", tas2557);
		if (ret) {
			dev_warn(dev, "failed to request IRQ %d: %d\n",
				 client->irq, ret);
		} else {
			tas2557->irq = client->irq;
			disable_irq_nosync(tas2557->irq);
			tas2557->irq_enabled = false;
		}
	}

	ret = devm_snd_soc_register_component(dev, &soc_component_tas2557,
					      &tas2557_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register component: %d\n", ret);
		goto err_disable_reg;
	}

	dev_info(dev, "TAS2557 driver probed successfully\n");
	return 0;

err_disable_reg:
	if (tas2557->vdd)
		regulator_disable(tas2557->vdd);
	return ret;
}

static void tas2557_i2c_remove(struct i2c_client *client)
{
	struct tas2557_priv *tas2557 = i2c_get_clientdata(client);

	/* Stop temperature monitoring */
	tas2557_stop_temp_monitor(tas2557);

	cancel_delayed_work_sync(&tas2557->irq_work);

	/* Free firmware data */
	if (tas2557->fw) {
		tas2557_fw_free(tas2557->fw);
		tas2557->fw = NULL;
	}

	/* Disable regulator */
	if (tas2557->vdd)
		regulator_disable(tas2557->vdd);
}

static const struct i2c_device_id tas2557_i2c_id[] = {
	{ "tas2557", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2557_i2c_id);

static const struct of_device_id tas2557_of_match[] = {
	{ .compatible = "ti,tas2557" },
	{ }
};
MODULE_DEVICE_TABLE(of, tas2557_of_match);

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
		.name = "tas2557",
		.of_match_table = tas2557_of_match,
	},
	.probe = tas2557_i2c_probe,
	.remove = tas2557_i2c_remove,
	.id_table = tas2557_i2c_id,
};
module_i2c_driver(tas2557_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("ASoC TAS2557 Smart Amplifier Driver");
MODULE_LICENSE("GPL");
