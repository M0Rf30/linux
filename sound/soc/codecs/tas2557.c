/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**     tas2557.c
**
** Description:
**     ALSA SoC driver for Texas Instruments TAS2557 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/crc8.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2557.h"

struct tasdevice_block {
	unsigned int mnType;
	unsigned char mbPChkSumPresent;
	unsigned char mnPChkSum;
	unsigned int mnCommands;
	unsigned char *data;
};

struct tasdevice_data {
	char name[64];
	char *description;
	unsigned int dev_blks;
	struct tasdevice_block *dev_blkcs;
};

struct tasdevice_program {
	char name[64];
	char *description;
	unsigned char app_mode;
	unsigned short boost;
	struct tasdevice_data mData;
};

struct tasdevice_pll {
	char name[64];
	char *description;
	struct tasdevice_block mBlock;
};

struct tasdevice_config {
	char name[64];
	char *description;
	unsigned int mnDevices;
	unsigned int mnProgram;
	unsigned int mnPLL;
	unsigned int sampling_rate;
	unsigned char pll_src;
	unsigned int pll_src_rate;
	struct tasdevice_data mData;
};

struct tasdevice_calibration {
	char name[64];
	char *description;
	unsigned int mnProgram;
	unsigned int mnConfiguration;
	struct tasdevice_data mData;
};

struct tasdevice_fw {
	unsigned int mnFWSize;
	unsigned int mnChecksum;
	unsigned int mnPPCVersion;
	unsigned int mnFWVersion;
	unsigned int mnDriverVersion;
	unsigned int mnTimeStamp;
	char ddc_name[64];
	char *description;
	unsigned int mnDeviceFamily;
	unsigned int mnDevice;
	unsigned int mnPLLs;
	struct tasdevice_pll *plls;
	unsigned int mnPrograms;
	struct tasdevice_program *programs;
	unsigned int mnConfigurations;
	struct tasdevice_config *configurations;
	unsigned int mnCalibrations;
	struct tasdevice_calibration *calibrations;
};

struct tas2557_register {
	int book;
	int page;
	int reg;
};

struct tas2557_priv {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_client *client;
	struct mutex dev_lock;
	struct tasdevice_fw *fmw;
	unsigned int mnCurrentProgram;
	unsigned int mnCurrentSampleRate;
	unsigned int mnCurrentConfiguration;
	unsigned int mnNewConfiguration;
	unsigned int mnCurrentCalibration;
	bool mbPowerUp;
	bool mbLoadConfigurationPrePowerUp;

	/* parameters for TAS2557 */
	struct gpio_desc *reset;
	unsigned char dev_addr;
	unsigned char mnCurrentBook;
	unsigned char mnCurrentPage;

	unsigned int error_code;
	unsigned int mnRestart;
	struct mutex codec_lock;
};

static unsigned int p_tas2557_default_data[] = {
	TAS2557_SAR_ADC2_REG,
	0x05, /* enable SAR ADC */
	TAS2557_CLK_ERR_CTRL2,
	0x21, /*clk1:clock hysteresis, 0.34ms; clock halt, 22ms*/
	TAS2557_CLK_ERR_CTRL3,
	0x21, /*clk2: rampDown 15dB/us, clock hysteresis, 10.66us; clock halt, 22ms */
	TAS2557_SAFE_GUARD_REG,
	TAS2557_SAFE_GUARD_PATTERN, /* safe guard */
	TAS2557_CLK_ERR_CTRL,
	0x00, /*enable clock error detection*/
	0xFFFFFFFF,
	0xFFFFFFFF
};

static unsigned int p_tas2557_startup_data[] = {
	TAS2557_GPI_PIN_REG,
	0x15, /* enable DIN, MCLK, CCI */
	TAS2557_GPIO1_PIN_REG,
	0x01, /* enable BCLK */
	TAS2557_GPIO2_PIN_REG,
	0x01, /* enable WCLK */
	TAS2557_POWER_CTRL2_REG,
	0xA0, /*Class-D, Boost power up*/
	TAS2557_POWER_CTRL2_REG,
	0xA3, /*Class-D, Boost, IV sense power up*/
	TAS2557_POWER_CTRL1_REG,
	0xF8, /*PLL, DSP, clock dividers power up*/
	TAS2557_UDELAY,
	2000, /* delay */
	TAS2557_UDELAY,
	2000, /* delay */
	TAS2557_CLK_ERR_CTRL,
	0x2B, /*enable clock error detection*/
	0xFFFFFFFF,
	0xFFFFFFFF
};

static unsigned int p_tas2557_mute_data[] = { TAS2557_SOFT_MUTE_REG,
					      0x01, /*soft mute*/
					      TAS2557_MDELAY,
					      10, /*delay 10ms*/
					      TAS2557_MUTE_REG,
					      0x03, /*mute*/
					      0xFFFFFFFF,
					      0xFFFFFFFF };

static unsigned int p_tas2557_unmute_data[] = { TAS2557_MUTE_REG,
						0x00, /*unmute*/
						TAS2557_SOFT_MUTE_REG,
						0x00, /*soft unmute*/
						0xFFFFFFFF,
						0xFFFFFFFF };

static unsigned int p_tas2557_shutdown_data[] = {
	TAS2557_CLK_ERR_CTRL,
	0x00, /* disable clock error detection */
	TAS2557_SOFT_MUTE_REG,
	0x01, /*soft mute*/
	TAS2557_MDELAY,
	10, /*delay 10ms*/
	TAS2557_MUTE_REG,
	0x03, /* mute */
	TAS2557_MDELAY,
	10, /*delay 10ms*/
	TAS2557_POWER_CTRL1_REG,
	0x60, /*DSP power down*/
	TAS2557_UDELAY,
	2000, /* delay 2ms */
	TAS2557_POWER_CTRL2_REG,
	0x00, /*Class-D, Boost power down*/
	TAS2557_POWER_CTRL1_REG,
	0x00, /*all power down*/
	TAS2557_GPIO1_PIN_REG,
	0x00, /* disable BCLK */
	TAS2557_GPIO2_PIN_REG,
	0x00, /* disable WCLK */
	TAS2557_GPI_PIN_REG,
	0x00, /* disable DIN, MCLK, CCI */
	0xFFFFFFFF,
	0xFFFFFFFF
};

static int tas2557_i2c_read_device(struct tas2557_priv *tas2557,
				   unsigned char addr, unsigned char reg,
				   unsigned char *p_value)
{
	int result = 0;
	unsigned int val = 0;

	tas2557->client->addr = addr;
	result = regmap_read(tas2557->regmap, reg, &val);

	if (result < 0)
		dev_err(tas2557->dev, "%s[0x%x] Error %d\n", __func__, addr,
			result);
	else
		*p_value = (unsigned char)val;

	return result;
}

static int tas2557_i2c_write_device(struct tas2557_priv *tas2557,
				    unsigned char addr, unsigned char reg,
				    unsigned char value)
{
	int result = 0;

	tas2557->client->addr = addr;
	result = regmap_write(tas2557->regmap, reg, value);

	if (result < 0)
		dev_err(tas2557->dev, "%s[0x%x] Error %d\n", __func__, addr,
			result);

	return result;
}

static int tas2557_i2c_update_bits(struct tas2557_priv *tas2557,
				   unsigned char addr, unsigned char reg,
				   unsigned char mask, unsigned char value)
{
	int result = 0;

	tas2557->client->addr = addr;
	result = regmap_update_bits(tas2557->regmap, reg, mask, value);

	if (result < 0)
		dev_err(tas2557->dev, "%s[0x%x] Error %d\n", __func__, addr,
			result);

	return result;
}

static int tas2557_i2c_bulkwrite_device(struct tas2557_priv *tas2557,
					unsigned char addr, unsigned char reg,
					unsigned char *pBuf, unsigned int len)
{
	int result = 0;

	tas2557->client->addr = addr;
	result = regmap_bulk_write(tas2557->regmap, reg, pBuf, len);

	if (result < 0)
		dev_err(tas2557->dev, "%s[0x%x] Error %d\n", __func__, addr,
			result);

	return result;
}

static int tas2557_change_book_page(struct tas2557_priv *tas2557,
				    unsigned char nBook, unsigned char nPage)
{
	int result = 0;

	if (tas2557->mnCurrentBook == nBook) {
		if (tas2557->mnCurrentPage != nPage) {
			result = tas2557_i2c_write_device(tas2557,
							   tas2557->dev_addr,
							   TAS2557_BOOKCTL_PAGE,
							   nPage);
			if (result >= 0)
				tas2557->mnCurrentPage = nPage;
		}
	} else {
		result = tas2557_i2c_write_device(tas2557, tas2557->dev_addr,
						   TAS2557_BOOKCTL_PAGE, 0);
		if (result >= 0) {
			tas2557->mnCurrentPage = 0;
			result = tas2557_i2c_write_device(tas2557,
							   tas2557->dev_addr,
							   TAS2557_BOOKCTL_REG,
							   nBook);
			tas2557->mnCurrentBook = nBook;
			if (nPage != 0) {
				result = tas2557_i2c_write_device(
					tas2557, tas2557->dev_addr,
					TAS2557_BOOKCTL_PAGE, nPage);
				tas2557->mnCurrentPage = nPage;
			}
		}
	}

	return result;
}

static int tas2557_dev_read(struct tas2557_priv *tas2557,
			    unsigned int nRegister, unsigned int *pValue)
{
	int result = 0;
	unsigned char Value = 0;

	mutex_lock(&tas2557->dev_lock);

	result = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(nRegister),
					   TAS2557_PAGE_ID(nRegister));

	if (result >= 0) {
		result = tas2557_i2c_read_device(tas2557, tas2557->dev_addr,
						  TAS2557_PAGE_REG(nRegister),
						  &Value);
		if (result >= 0)
			*pValue = Value;
	}

	mutex_unlock(&tas2557->dev_lock);
	return result;
}

static int tas2557_dev_write(struct tas2557_priv *tas2557,
			     unsigned int nRegister, unsigned int nValue)
{
	int result = 0;

	mutex_lock(&tas2557->dev_lock);

	result = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(nRegister),
					   TAS2557_PAGE_ID(nRegister));

	if (result >= 0) {
		result = tas2557_i2c_write_device(tas2557, tas2557->dev_addr,
						   TAS2557_PAGE_REG(nRegister),
						   nValue);
	}

	mutex_unlock(&tas2557->dev_lock);
	return result;
}

static int tas2557_dev_bulk_write(struct tas2557_priv *tas2557,
				  unsigned int nRegister, unsigned char *data,
				  unsigned int nLength)
{
	int result = 0;
	unsigned char reg = 0;

	mutex_lock(&tas2557->dev_lock);

	result = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(nRegister),
					   TAS2557_PAGE_ID(nRegister));

	if (result >= 0) {
		reg = TAS2557_PAGE_REG(nRegister);

		result = tas2557_i2c_bulkwrite_device(
			tas2557, tas2557->dev_addr, reg, data, nLength);
	}

	mutex_unlock(&tas2557->dev_lock);
	return result;
}

static int tas2557_dev_update_bits(struct tas2557_priv *tas2557,
				   unsigned int nRegister, unsigned int nMask,
				   unsigned int nValue)
{
	int result = 0;

	mutex_lock(&tas2557->dev_lock);

	result = tas2557_change_book_page(tas2557, TAS2557_BOOK_ID(nRegister),
					   TAS2557_PAGE_ID(nRegister));

	if (result >= 0) {
		result = tas2557_i2c_update_bits(tas2557, tas2557->dev_addr,
						  TAS2557_PAGE_REG(nRegister),
						  nMask, nValue);
	}

	mutex_unlock(&tas2557->dev_lock);
	return result;
}

static void tas2557_hw_reset(struct tas2557_priv *tas2557)
{
	dev_dbg(tas2557->dev, "%s\n", __func__);

	gpiod_set_value_cansleep(tas2557->reset, 0);
	msleep(5);
	gpiod_set_value_cansleep(tas2557->reset, 1);
	msleep(2);

	tas2557->mnCurrentBook = -1;
	tas2557->mnCurrentPage = -1;

	if (tas2557->error_code)
		dev_info(tas2557->dev, "%s, ErrCode=0x%x\n", __func__,
			 tas2557->error_code);

	tas2557->error_code = 0;
}

static int tas2557_dev_load_data(struct tas2557_priv *tas2557,
				 unsigned int *data)
{
	int result = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = data[n * 3 + 1];
		nData = data[n * 3 + 2];
		if (nRegister == TAS2557_UDELAY) {
			udelay(nData);
		} else if (nRegister == TAS2557_MDELAY) {
			mdelay(nData);
		} else if (nRegister != 0xFFFFFFFF) {
			result = tas2557_dev_write(tas2557, nRegister, nData);
			if (result < 0)
				break;
		}

		n++;
	} while (nRegister != 0xFFFFFFFF);

	return result;
}

static int tas2557_DevStartup(struct tas2557_priv *tas2557, unsigned int dev)
{
	int result = 0;

	dev_dbg(tas2557->dev, "%s\n", __func__);
	result = tas2557_dev_load_data(tas2557, p_tas2557_startup_data);

	return result;
}

static int tas2557_DevShutdown(struct tas2557_priv *tas2557)
{
	int result = 0;

	dev_dbg(tas2557->dev, "%s\n", __func__);

	result = tas2557_dev_load_data(tas2557, p_tas2557_shutdown_data);

	return result;
}

int tas2557_set_DAC_gain(struct tas2557_priv *tas2557, unsigned int nGain)
{
	int result = 0;
	int gain = (nGain & 0x0f);

	dev_dbg(tas2557->dev, "%s, nGain: %d", __func__, nGain);

	result = tas2557_dev_update_bits(tas2557, TAS2557_SPK_CTRL_REG, 0x78,
					  (gain << 3));

	if (result < 0)
		goto end;

end:

	return result;
}

int tas2557_get_DAC_gain(struct tas2557_priv *tas2557, unsigned char *pnGain)
{
	int result = 0;
	int nGain;

	result = tas2557_dev_read(tas2557, TAS2557_SPK_CTRL_REG, &nGain);

	if (result >= 0)
		*pnGain = ((nGain >> 3) & 0x0f);

	return result;
}

int tas2557_DevMute(struct tas2557_priv *tas2557, bool mute)
{
	int result = 0;

	dev_dbg(tas2557->dev, "%s, mute=%d\n", __func__, mute);

	if (mute)
		result = tas2557_dev_load_data(tas2557, p_tas2557_mute_data);
	else
		result =
			tas2557_dev_load_data(tas2557, p_tas2557_unmute_data);

	return result;
}

int tas2557_load_default(struct tas2557_priv *tas2557)
{
	int result = 0;

	dev_dbg(tas2557->dev, "%s\n", __func__);

	result = tas2557_dev_load_data(tas2557, p_tas2557_default_data);
	if (result < 0)
		goto end;

	// Set default bit rate of 16A
	result = tas2557_dev_update_bits(tas2557, TAS2557_ASI1_DAC_FORMAT_REG,
					  0x18, 0);
	if (result < 0)
		goto end;

	/* enable DOUT tri-state for extra BCLKs */
	result = tas2557_dev_update_bits(tas2557, TAS2557_ASI1_DAC_FORMAT_REG,
					  0x01, 0x01);
	if (result < 0)
		goto end;

	/* Interrupt pin, low-highZ, high active driven */
	result = tas2557_dev_update_bits(tas2557, TAS2557_GPIO_HIZ_CTRL2_REG,
					  0x30, 0x30);

end:
	return result;
}

void tas2557_clear_firmware(struct tasdevice_fw *fmw)
{
	unsigned int n, nn;

	if (!fmw)
		return;

	kfree(fmw->description);

	if (fmw->plls != NULL) {
		for (n = 0; n < fmw->mnPLLs; n++) {
			kfree(fmw->plls[n].description);
			kfree(fmw->plls[n].mBlock.data);
		}

		kfree(fmw->plls);
	}

	if (fmw->programs != NULL) {
		for (n = 0; n < fmw->mnPrograms; n++) {
			kfree(fmw->programs[n].description);
			kfree(fmw->programs[n].mData.description);

			for (nn = 0;
			     nn < fmw->programs[n].mData.dev_blks; nn++)
				kfree(fmw->programs[n]
					      .mData.dev_blkcs[nn]
					      .data);

			kfree(fmw->programs[n].mData.dev_blkcs);
		}

		kfree(fmw->programs);
	}

	if (fmw->configurations != NULL) {
		for (n = 0; n < fmw->mnConfigurations; n++) {
			kfree(fmw->configurations[n].description);
			kfree(fmw->configurations[n]
				      .mData.description);

			for (nn = 0;
			     nn < fmw->configurations[n].mData.dev_blks;
			     nn++)
				kfree(fmw->configurations[n]
					      .mData.dev_blkcs[nn]
					      .data);

			kfree(fmw->configurations[n].mData.dev_blkcs);
		}

		kfree(fmw->configurations);
	}

	if (fmw->calibrations != NULL) {
		for (n = 0; n < fmw->mnCalibrations; n++) {
			kfree(fmw->calibrations[n].description);
			kfree(fmw->calibrations[n].mData.description);

			for (nn = 0;
			     nn < fmw->calibrations[n].mData.dev_blks;
			     nn++)
				kfree(fmw->calibrations[n]
					      .mData.dev_blkcs[nn]
					      .data);

			kfree(fmw->calibrations[n].mData.dev_blkcs);
		}

		kfree(fmw->calibrations);
	}

	memset(fmw, 0x00, sizeof(struct tasdevice_fw));
}

static int tas2557_load_block(struct tas2557_priv *tas2557,
			      struct tasdevice_block *pBlock)
{
	int result = 0;
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nValue1;
	unsigned int nLength;
	unsigned int nSleep;
	unsigned char *data = pBlock->data;

	dev_dbg(tas2557->dev, "%s: Type = %d, commands = %d\n", __func__,
		pBlock->mnType, pBlock->mnCommands);

	if (pBlock->mbPChkSumPresent) {
		result = tas2557_dev_write(tas2557, TAS2557_CRC_RESET_REG, 1);
		if (result < 0)
			goto end;
	}

	nCommand = 0;
	while (nCommand < pBlock->mnCommands) {
		data = pBlock->data + nCommand * 4;
		nBook = data[0];
		nPage = data[1];
		nOffset = data[2];
		nData = data[3];
		nCommand++;

		if (nOffset <= 0x7F) {
			result = tas2557_dev_write(
				tas2557, TAS2557_REG(nBook, nPage, nOffset),
				nData);
			if (result < 0)
				goto end;

		} else if (nOffset == 0x81) {
			nSleep = (nBook << 8) + nPage;
			msleep(nSleep);
		} else if (nOffset == 0x85) {
			data += 4;
			nLength = (nBook << 8) + nPage;
			nBook = data[0];
			nPage = data[1];
			nOffset = data[2];

			if (nLength > 1) {
				result = tas2557_dev_bulk_write(
					tas2557,
					TAS2557_REG(nBook, nPage, nOffset),
					data + 3, nLength);
				if (result < 0)
					goto end;

			} else {
				result = tas2557_dev_write(
					tas2557,
					TAS2557_REG(nBook, nPage, nOffset),
					data[3]);
				if (result < 0)
					goto end;
			}

			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}

	if (pBlock->mbPChkSumPresent) {
		result = tas2557_dev_read(tas2557, TAS2557_CRC_CHECKSUM_REG,
					   &nValue1);

		if (result < 0)
			goto end;

		if (nValue1 != pBlock->mnPChkSum) {
			dev_err(tas2557->dev,
				"Block PChkSum Error: FW = 0x%x, Reg = 0x%x\n",
				pBlock->mnPChkSum, (nValue1 & 0xff));
			result = -EAGAIN;
			tas2557->error_code |= ERROR_PRAM_CRCCHK;
		}
	}

end:

	if (result < 0)
		dev_err(tas2557->dev, "Block (%d) load error\n",
			pBlock->mnType);

	return result;
}

static int tas2557_load_data(struct tas2557_priv *tas2557, struct tasdevice_data *data,
			     unsigned int nType)
{
	int result = 0;
	unsigned int nBlock;
	struct tasdevice_block *pBlock;

	dev_dbg(tas2557->dev,
		"TAS2557 load data: %s, Blocks = %d, Block Type = %d\n",
		data->name, data->dev_blks, nType);

	for (nBlock = 0; nBlock < data->dev_blks; nBlock++) {
		pBlock = &(data->dev_blkcs[nBlock]);

		if (pBlock->mnType == nType) {
			result = tas2557_load_block(tas2557, pBlock);

			if (result < 0)
				break;
		}
	}

	return result;
}

static void failsafe(struct tas2557_priv *tas2557)
{
	dev_err(tas2557->dev, "%s\n", __func__);
	tas2557->error_code |= ERROR_FAILSAFE;

	tas2557_DevShutdown(tas2557);
	tas2557->mbPowerUp = false;
	tas2557_hw_reset(tas2557);
	tas2557_dev_write(tas2557, TAS2557_SW_RESET_REG, 0x01);
	msleep(1);
	tas2557_dev_write(tas2557, TAS2557_SPK_CTRL_REG, 0x04);

	if (tas2557->fmw != NULL)
		tas2557_clear_firmware(tas2557->fmw);
}

static int tas2557_load_coefficient(struct tas2557_priv *tas2557,
				    int nPrevConfig, int nNewConfig,
				    bool bPowerOn)
{
	int result = 0;
	struct tasdevice_pll *pPLL;
	struct tasdevice_program *program = NULL;
	struct tasdevice_config *pPrevConfiguration;
	struct tasdevice_config *pNewConfiguration;
	bool bRestorePower = false;

	dev_dbg(tas2557->dev, "%s, Prev=%d, new=%d, Pow=%d\n", __func__,
		nPrevConfig, nNewConfig, bPowerOn);

	if (!tas2557->fmw->mnConfigurations) {
		dev_err(tas2557->dev, "%s, firmware not loaded\n", __func__);
		goto end;
	}

	if (nNewConfig >= tas2557->fmw->mnConfigurations) {
		dev_err(tas2557->dev,
			"%s, invalid configuration New=%d, total=%d\n",
			__func__, nNewConfig,
			tas2557->fmw->mnConfigurations);
		goto end;
	}

	if (nPrevConfig < 0) {
		pPrevConfiguration = NULL;
	} else if (nPrevConfig == nNewConfig) {
		dev_dbg(tas2557->dev, "%d configuration is already loaded\n",
			nNewConfig);
		goto end;
	} else {
		pPrevConfiguration =
			&(tas2557->fmw->configurations[nPrevConfig]);
	}

	pNewConfiguration =
		&(tas2557->fmw->configurations[nNewConfig]);
	tas2557->mnCurrentConfiguration = nNewConfig;

	if (pPrevConfiguration) {
		if ((pPrevConfiguration->mnPLL == pNewConfiguration->mnPLL) &&
		    (pPrevConfiguration->mnDevices ==
		     pNewConfiguration->mnDevices)) {
			dev_dbg(tas2557->dev, "%s, PLL and device same\n",
				__func__);
			goto prog_coefficient;
		}
	}

	program =
		&(tas2557->fmw->programs[tas2557->mnCurrentProgram]);

	if (bPowerOn) {
		result = tas2557_DevShutdown(tas2557);

		if (result < 0)
			goto end;

		bRestorePower = true;
	}

	/* load PLL */
	pPLL = &(tas2557->fmw->plls[pNewConfiguration->mnPLL]);
	dev_dbg(tas2557->dev, "load PLL: %s block for Configuration %s\n",
		pPLL->name, pNewConfiguration->name);
	result = tas2557_load_block(tas2557, &(pPLL->mBlock));

	if (result < 0)
		goto end;

	tas2557->mnCurrentSampleRate = pNewConfiguration->sampling_rate;

	dev_dbg(tas2557->dev, "load configuration %s conefficient pre block\n",
		pNewConfiguration->name);

	if (pNewConfiguration->mnDevices) {
		result = tas2557_load_data(tas2557,
					    &(pNewConfiguration->mData),
					    TAS2557_BLOCK_CFG_PRE_DEV_A);

		if (result < 0)
			goto end;
	}

prog_coefficient:
	dev_dbg(tas2557->dev, "load new configuration: %s, coeff block data\n",
		pNewConfiguration->name);

	if (pNewConfiguration->mnDevices) {
		result = tas2557_load_data(tas2557,
					    &(pNewConfiguration->mData),
					    TAS2557_BLOCK_CFG_COEFF_DEV_A);
		if (result < 0)
			goto end;
	}

	if (bRestorePower) {
		result = tas2557_DevStartup(tas2557,
					     pNewConfiguration->mnDevices);
		if (result < 0)
			goto end;

		dev_dbg(tas2557->dev, "device powered up, load unmute\n");
		result = tas2557_DevMute(tas2557, false);
		if (result < 0)
			goto end;
	}

end:

	if (result < 0)
		dev_err(tas2557->dev, "%s, load new conf %s error\n", __func__,
			pNewConfiguration->name);

	tas2557->mnNewConfiguration = tas2557->mnCurrentConfiguration;
	return result;
}

static int tas2557_load_configuration(struct tas2557_priv *tas2557,
				      unsigned int nConfiguration,
				      bool bLoadSame)
{
	int result = 0;
	struct tasdevice_config *pCurrentConfiguration = NULL;
	struct tasdevice_config *pNewConfiguration = NULL;

	dev_dbg(tas2557->dev, "%s: %d\n", __func__, nConfiguration);

	if ((!tas2557->fmw->programs) ||
	    (!tas2557->fmw->configurations)) {
		dev_err(tas2557->dev, "Firmware not loaded\n");
		result = 0;
		goto end;
	}

	if (nConfiguration >= tas2557->fmw->mnConfigurations) {
		dev_err(tas2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		result = 0;
		goto end;
	}

	if ((!tas2557->mbLoadConfigurationPrePowerUp) &&
	    (nConfiguration == tas2557->mnCurrentConfiguration) &&
	    (!bLoadSame)) {
		dev_info(tas2557->dev, "Configuration %d is already loaded\n",
			 nConfiguration);
		result = 0;
		goto end;
	}

	pCurrentConfiguration =
		&(tas2557->fmw
			  ->configurations[tas2557->mnCurrentConfiguration]);
	pNewConfiguration =
		&(tas2557->fmw->configurations[nConfiguration]);

	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(tas2557->dev,
			"Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->name,
			pCurrentConfiguration->mnProgram);
		result = 0;
		goto end;
	}

	if (pNewConfiguration->mnPLL >= tas2557->fmw->mnPLLs) {
		dev_err(tas2557->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->name,
			pNewConfiguration->mnPLL);
		result = 0;
		goto end;
	}

	if (tas2557->mbPowerUp) {
		dev_err(tas2557->dev,
			"%s, device power on, load new conf[%d] %s\n", __func__,
			nConfiguration, pNewConfiguration->name);
		result = tas2557_load_coefficient(
			tas2557, tas2557->mnCurrentConfiguration,
			nConfiguration, true);
		tas2557->mbLoadConfigurationPrePowerUp = false;
	} else {
		dev_dbg(tas2557->dev,
			"TAS2557 was powered down, will load coefficient when power up\n");
		tas2557->mbLoadConfigurationPrePowerUp = true;
		tas2557->mnNewConfiguration = nConfiguration;
	}

end:

	if (result < 0) {
		if (tas2557->error_code &
		    (ERROR_DEVA_I2C_COMM | ERROR_PRAM_CRCCHK |
		     ERROR_YRAM_CRCCHK))
			failsafe(tas2557);
	}

	return result;
}

int tas2557_set_program(struct tas2557_priv *tas2557, unsigned int nProgram,
			int nConfig)
{
	struct tasdevice_program *program;
	struct tasdevice_config *config;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	bool bFound = false;
	int result = 0;

	if ((!tas2557->fmw->programs) ||
	    (!tas2557->fmw->configurations)) {
		dev_err(tas2557->dev, "Firmware not loaded\n");
		result = 0;
		goto end;
	}

	if (nProgram >= tas2557->fmw->mnPrograms) {
		dev_err(tas2557->dev, "TAS2557: Program %d doesn't exist\n",
			nProgram);
		result = 0;
		goto end;
	}

	if (nProgram == 1)
		tas2557->mnCurrentSampleRate = 96000;
	else
		tas2557->mnCurrentSampleRate = 48000;

	if (nConfig < 0) {
		nConfiguration = 0;
		nSampleRate = tas2557->mnCurrentSampleRate;
		dev_err(tas2557->dev, "nSampleRate: %d\n", nSampleRate);

		while (!bFound && (nConfiguration <
				   tas2557->fmw->mnConfigurations)) {
			dev_err(tas2557->dev,
				"configurations SampleRate: %d\n",
				tas2557->fmw
					->configurations[nConfiguration]
					.sampling_rate);
			if (tas2557->fmw
				    ->configurations[nConfiguration]
				    .mnProgram == nProgram) {
				if (nSampleRate == 0) {
					bFound = true;
					dev_info(
						tas2557->dev,
						"find default configuration %d\n",
						nConfiguration);
				} else if (nSampleRate ==
					   tas2557->fmw
						   ->configurations
							   [nConfiguration]
						   .sampling_rate) {
					bFound = true;
					dev_info(
						tas2557->dev,
						"find matching configuration %d\n",
						nConfiguration);
				} else {
					nConfiguration++;
				}
			} else {
				nConfiguration++;
			}
		}

		if (!bFound) {
			dev_err(tas2557->dev,
				"Program %d, no valid configuration found for sample rate %d, ignore\n",
				nProgram, nSampleRate);
			result = 0;
			goto end;
		}
	} else {
		if (tas2557->fmw->configurations[nConfig].mnProgram !=
		    nProgram) {
			dev_err(tas2557->dev,
				"%s, configuration program doesn't match\n",
				__func__);
			result = 0;
			goto end;
		}

		nConfiguration = nConfig;
	}

	program = &(tas2557->fmw->programs[nProgram]);

	if (tas2557->mbPowerUp) {
		dev_info(
			tas2557->dev,
			"device powered up, power down to load program %d (%s)\n",
			nProgram, program->name);

		result = tas2557_DevShutdown(tas2557);

		if (result < 0)
			goto end;
	}

	tas2557_hw_reset(tas2557);
	result = tas2557_dev_write(tas2557, TAS2557_SW_RESET_REG, 0x01);
	if (result < 0)
		goto end;

	msleep(1);
	result = tas2557_load_default(tas2557);
	if (result < 0)
		goto end;

	dev_info(tas2557->dev, "load program %d (%s)\n", nProgram,
		 program->name);
	result = tas2557_load_data(tas2557, &(program->mData),
				    TAS2557_BLOCK_PGM_DEV_A);
	if (result < 0)
		goto end;

	result = tas2557_load_data(tas2557, &(program->mData),
				    TAS2557_BLOCK_PGM_DEV_B);
	if (result < 0)
		goto end;

	tas2557->mnCurrentProgram = nProgram;

	result = tas2557_load_coefficient(tas2557, -1, nConfiguration, false);
	if (result < 0)
		goto end;

	if (tas2557->mbPowerUp) {
		config = &(tas2557->fmw->configurations
					   [tas2557->mnCurrentConfiguration]);
		result =
			tas2557_DevStartup(tas2557, config->mnDevices);
		if (result < 0)
			goto end;

		result = tas2557_DevMute(tas2557, false);
		if (result < 0)
			goto end;
	}

end:

	if (result < 0) {
		if (tas2557->error_code &
		    (ERROR_DEVA_I2C_COMM | ERROR_PRAM_CRCCHK |
		     ERROR_YRAM_CRCCHK))
			failsafe(tas2557);
	}

	return result;
}

static void fw_print_header(struct tas2557_priv *tas2557,
			    struct tasdevice_fw *fmw)
{
	dev_info(tas2557->dev, "FW Size       = %d", fmw->mnFWSize);
	dev_info(tas2557->dev, "Checksum      = 0x%04X",
		 fmw->mnChecksum);
	dev_info(tas2557->dev, "PPC Version   = 0x%04X",
		 fmw->mnPPCVersion);
	dev_info(tas2557->dev, "FW  Version    = 0x%04X",
		 fmw->mnFWVersion);
	dev_info(tas2557->dev, "Driver Version= 0x%04X",
		 fmw->mnDriverVersion);
	dev_info(tas2557->dev, "Timestamp     = %d", fmw->mnTimeStamp);
	dev_info(tas2557->dev, "DDC Name      = %s", fmw->ddc_name);
	dev_info(tas2557->dev, "Description   = %s", fmw->description);
}

static inline unsigned int fw_convert_number(unsigned char *data)
{
	return data[3] + (data[2] << 8) + (data[1] << 16) + (data[0] << 24);
}

static int fw_parse_header(struct tas2557_priv *tas2557,
			   struct tasdevice_fw *fmw, unsigned char *data,
			   unsigned int nSize)
{
	unsigned char *data_start = data;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };

	if (nSize < 104) {
		dev_err(tas2557->dev, "Firmware: Header too short");
		return -EINVAL;
	}

	if (memcmp(data, pMagicNumber, 4)) {
		dev_err(tas2557->dev, "Firmware: Magic number doesn't match");
		return -EINVAL;
	}

	data += 4;

	fmw->mnFWSize = fw_convert_number(data);
	data += 4;

	fmw->mnChecksum = fw_convert_number(data);
	data += 4;

	fmw->mnPPCVersion = fw_convert_number(data);
	data += 4;

	fmw->mnFWVersion = fw_convert_number(data);
	data += 4;

	fmw->mnDriverVersion = fw_convert_number(data);
	dev_err(tas2557->dev, "Firmware driver: 0x%x",
		fmw->mnDriverVersion);
	data += 4;

	fmw->mnTimeStamp = fw_convert_number(data);
	data += 4;

	memcpy(fmw->ddc_name, data, 64);
	data += 64;

	n = strlen(data);
	fmw->description = kmemdup(data, n + 1, GFP_KERNEL);
	data += n + 1;

	if ((data - data_start) >= nSize) {
		dev_err(tas2557->dev,
			"Firmware: Header too short after DDC description");
		return -EINVAL;
	}

	fmw->mnDeviceFamily = fw_convert_number(data);
	data += 4;

	if (fmw->mnDeviceFamily != 0) {
		dev_err(tas2557->dev, "deviceFamily %d, not TAS device",
			fmw->mnDeviceFamily);
		return -EINVAL;
	}

	fmw->mnDevice = fw_convert_number(data);
	data += 4;

	if (fmw->mnDevice != 2) {
		dev_err(tas2557->dev, "device %d, not TAS2557",
			fmw->mnDevice);
		return -EINVAL;
	}

	fw_print_header(tas2557, fmw);

	return data - data_start;
}

static int fw_parse_block_data(struct tas2557_priv *tas2557,
			       struct tasdevice_fw *fmw,
			       struct tasdevice_block *pBlock, unsigned char *data)
{
	unsigned char *data_start = data;
	unsigned int n;

	pBlock->mnType = fw_convert_number(data);
	data += 4;

	if (fmw->mnDriverVersion >= PPC_DRIVER_CRCCHK) {
		pBlock->mbPChkSumPresent = data[0];
		data++;

		pBlock->mnPChkSum = data[0];
		data++;

		// skip YRAM checksum data for simplicity
		data += 2;
	} else {
		pBlock->mbPChkSumPresent = 0;
	}

	pBlock->mnCommands = fw_convert_number(data);
	data += 4;

	n = pBlock->mnCommands * 4;
	pBlock->data = kmemdup(data, n, GFP_KERNEL);
	data += n;

	return data - data_start;
}

static int fw_parse_data(struct tas2557_priv *tas2557,
			 struct tasdevice_fw *fmw, struct tasdevice_data *pImageData,
			 unsigned char *data)
{
	unsigned char *data_start = data;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->name, data, 64);
	data += 64;

	n = strlen(data);
	pImageData->description = kmemdup(data, n + 1, GFP_KERNEL);
	data += n + 1;

	pImageData->dev_blks = (data[0] << 8) + data[1];
	data += 2;

	pImageData->dev_blkcs = kmalloc(
		sizeof(struct tasdevice_block) * pImageData->dev_blks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->dev_blks; nBlock++) {
		n = fw_parse_block_data(tas2557, fmw,
					&(pImageData->dev_blkcs[nBlock]), data);
		data += n;
	}

	return data - data_start;
}

static int fw_parse_pll_data(struct tas2557_priv *tas2557,
			     struct tasdevice_fw *fmw, unsigned char *data)
{
	unsigned char *data_start = data;
	unsigned int n;
	unsigned int nPLL;
	struct tasdevice_pll *pPLL;

	fmw->mnPLLs = (data[0] << 8) + data[1];
	data += 2;

	if (fmw->mnPLLs == 0)
		goto end;

	fmw->plls = kmalloc_array(fmw->mnPLLs,
					  sizeof(struct tasdevice_pll), GFP_KERNEL);

	for (nPLL = 0; nPLL < fmw->mnPLLs; nPLL++) {
		pPLL = &(fmw->plls[nPLL]);

		memcpy(pPLL->name, data, 64);
		data += 64;

		n = strlen(data);
		pPLL->description = kmemdup(data, n + 1, GFP_KERNEL);
		data += n + 1;

		n = fw_parse_block_data(tas2557, fmw, &(pPLL->mBlock),
					data);
		data += n;
	}

end:
	return data - data_start;
}

static int fw_parse_program_data(struct tas2557_priv *tas2557,
				 struct tasdevice_fw *fmw,
				 unsigned char *data)
{
	unsigned char *data_start = data;
	unsigned int n;
	unsigned int nProgram;
	struct tasdevice_program *program;

	fmw->mnPrograms = (data[0] << 8) + data[1];
	data += 2;

	if (fmw->mnPrograms == 0)
		goto end;

	fmw->programs = kmalloc(
		sizeof(struct tasdevice_program) * fmw->mnPrograms, GFP_KERNEL);

	for (nProgram = 0; nProgram < fmw->mnPrograms; nProgram++) {
		program = &(fmw->programs[nProgram]);
		memcpy(program->name, data, 64);
		data += 64;

		n = strlen(data);
		program->description = kmemdup(data, n + 1, GFP_KERNEL);
		data += n + 1;

		program->app_mode = data[0];
		data++;

		program->boost = (data[0] << 8) + data[1];
		data += 2;

		n = fw_parse_data(tas2557, fmw, &(program->mData),
				  data);
		data += n;
	}

end:

	return data - data_start;
}

static int fw_parse_configuration_data(struct tas2557_priv *tas2557,
				       struct tasdevice_fw *fmw,
				       unsigned char *data)
{
	unsigned char *data_start = data;
	unsigned int n;
	unsigned int nConfiguration;
	struct tasdevice_config *config;

	fmw->mnConfigurations = (data[0] << 8) + data[1];
	data += 2;

	if (fmw->mnConfigurations == 0)
		goto end;

	fmw->configurations = kmalloc(
		sizeof(struct tasdevice_config) * fmw->mnConfigurations,
		GFP_KERNEL);

	for (nConfiguration = 0; nConfiguration < fmw->mnConfigurations;
	     nConfiguration++) {
		config = &(fmw->configurations[nConfiguration]);
		memcpy(config->name, data, 64);
		data += 64;

		n = strlen(data);
		config->description =
			kmemdup(data, n + 1, GFP_KERNEL);
		data += n + 1;

		if ((fmw->mnDriverVersion >= PPC_DRIVER_CONFDEV) ||
		    ((fmw->mnDriverVersion >= PPC_DRIVER_CFGDEV_NONCRC) &&
		     (fmw->mnDriverVersion < PPC_DRIVER_CRCCHK))) {
			config->mnDevices = (data[0] << 8) + data[1];
			data += 2;
		}

		config->mnProgram = data[0];
		data++;

		config->mnPLL = data[0];
		data++;

		config->sampling_rate = fw_convert_number(data);
		data += 4;

		if (fmw->mnDriverVersion >= PPC_DRIVER_MTPLLSRC) {
			config->pll_src = data[0];
			data++;

			config->pll_src_rate = fw_convert_number(data);
			data += 4;
			dev_err(tas2557->dev,
				"line:%d, data: 0x%x, 0x%x, 0x%x, 0x%x",
				__LINE__, data[0], data[1], data[2],
				data[3]);
		}

		n = fw_parse_data(tas2557, fmw, &(config->mData),
				  data);
		data += n;
	}

end:

	return data - data_start;
}

static int fw_parse_calibration_data(struct tas2557_priv *tas2557,
				     struct tasdevice_fw *fmw,
				     unsigned char *data)
{
	unsigned char *data_start = data;
	unsigned int n;
	unsigned int nCalibration;
	struct tasdevice_calibration *pCalibration;

	fmw->mnCalibrations = (data[0] << 8) + data[1];
	data += 2;

	if (fmw->mnCalibrations == 0)
		goto end;

	fmw->calibrations =
		kmalloc(sizeof(struct tasdevice_calibration) * fmw->mnCalibrations,
			GFP_KERNEL);

	for (nCalibration = 0; nCalibration < fmw->mnCalibrations;
	     nCalibration++) {
		pCalibration = &(fmw->calibrations[nCalibration]);
		memcpy(pCalibration->name, data, 64);
		data += 64;

		n = strlen(data);
		pCalibration->description = kmemdup(data, n + 1, GFP_KERNEL);
		data += n + 1;

		pCalibration->mnProgram = data[0];
		data++;

		pCalibration->mnConfiguration = data[0];
		data++;

		n = fw_parse_data(tas2557, fmw, &(pCalibration->mData),
				  data);
		data += n;
	}

end:

	return data - data_start;
}

static int fw_parse(struct tas2557_priv *tas2557, struct tasdevice_fw *fmw,
		    unsigned char *data, unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(tas2557, fmw, data, nSize);

	if (nPosition < 0) {
		dev_err(tas2557->dev, "Firmware: Wrong Header");
		return -EINVAL;
	}

	if (nPosition >= nSize) {
		dev_err(tas2557->dev, "Firmware: Too short");
		return -EINVAL;
	}

	data += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(tas2557, fmw, data);

	data += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(tas2557, fmw, data);

	data += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(tas2557, fmw, data);

	data += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition =
			fw_parse_calibration_data(tas2557, fmw, data);

	return 0;
}

void tas2557_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2557_priv *tas2557 = (struct tas2557_priv *)pContext;
	int result;
	unsigned int nProgram = 0;
	unsigned int nSampleRate = 0;

	mutex_lock(&tas2557->codec_lock);

	dev_info(tas2557->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(tas2557->dev, "%s firmware is not loaded.\n",
			TAS2557_FW_NAME);
		goto end;
	}

	if (tas2557->fmw->configurations) {
		nProgram = tas2557->mnCurrentProgram;
		nSampleRate = tas2557->mnCurrentSampleRate;
		dev_dbg(tas2557->dev, "clear current firmware\n");
		tas2557_clear_firmware(tas2557->fmw);
	}

	result = fw_parse(tas2557, tas2557->fmw,
			   (unsigned char *)(pFW->data), pFW->size);
	release_firmware(pFW);

	if (result < 0) {
		dev_err(tas2557->dev, "firmware is corrupt\n");
		goto end;
	}

	if (!tas2557->fmw->mnPrograms) {
		dev_err(tas2557->dev, "firmware contains no programs\n");
		result = -EINVAL;
		goto end;
	}

	if (!tas2557->fmw->mnConfigurations) {
		dev_err(tas2557->dev, "firmware contains no configurations\n");
		result = -EINVAL;
		goto end;
	}

	if (nProgram >= tas2557->fmw->mnPrograms) {
		dev_info(tas2557->dev,
			 "no previous program, set to default\n");
		nProgram = 0;
	}

	tas2557->mnCurrentSampleRate = nSampleRate;
	result = tas2557_set_program(tas2557, nProgram, -1);

end:
	mutex_unlock(&tas2557->codec_lock);
}

int tas2557_enable(struct tas2557_priv *tas2557, bool bEnable)
{
	int result = 0;
	struct tasdevice_program *program;
	struct tasdevice_config *config;
	unsigned int nValue;

	dev_dbg(tas2557->dev, "%s: %s\n", __func__, bEnable ? "On" : "Off");

	if ((tas2557->fmw->mnPrograms == 0) ||
	    (tas2557->fmw->mnConfigurations == 0)) {
		dev_err(tas2557->dev, "%s, firmware not loaded\n", __func__);
		/*Load firmware*/
		result = request_firmware_nowait(THIS_MODULE, 1,
						  TAS2557_FW_NAME,
						  tas2557->dev, GFP_KERNEL,
						  tas2557, tas2557_fw_ready);
		if (result < 0) {
			dev_err(tas2557->dev, "%s, firmware is loaded\n",
				__func__);
			goto end;
		}
	}

	/* check safe guard*/
	result = tas2557_dev_read(tas2557, TAS2557_SAFE_GUARD_REG, &nValue);
	if (result < 0)
		goto end;
	if ((nValue & 0xff) != TAS2557_SAFE_GUARD_PATTERN) {
		dev_err(tas2557->dev, "ERROR safe guard (0x%x) failure!\n",
			nValue);
		result = -EPIPE;
		tas2557->error_code = ERROR_SAFE_GUARD;
		tas2557->mbPowerUp = true;
		goto end;
	}

	program =
		&(tas2557->fmw->programs[tas2557->mnCurrentProgram]);
	if (bEnable) {
		if (!tas2557->mbPowerUp) {
			if (tas2557->mbLoadConfigurationPrePowerUp) {
				tas2557->mbLoadConfigurationPrePowerUp = false;
				result = tas2557_load_coefficient(
					tas2557,
					tas2557->mnCurrentConfiguration,
					tas2557->mnNewConfiguration, false);
				if (result < 0)
					goto end;
			}

			config =
				&(tas2557->fmw->configurations
					  [tas2557->mnCurrentConfiguration]);
			result = tas2557_DevStartup(tas2557,
						     config->mnDevices);
			if (result < 0)
				goto end;

			result = tas2557_DevMute(tas2557, false);
			if (result < 0)
				goto end;

			tas2557->mbPowerUp = true;
			tas2557->mnRestart = 0;
		}
	} else {
		if (tas2557->mbPowerUp) {
			config =
				&(tas2557->fmw->configurations
					  [tas2557->mnCurrentConfiguration]);

			result = tas2557_DevShutdown(tas2557);
			if (result < 0)
				goto end;

			tas2557->mbPowerUp = false;
			tas2557->mnRestart = 0;
		}
	}

	result = 0;

end:

	if (result < 0) {
		if (tas2557->error_code &
		    (ERROR_DEVA_I2C_COMM | ERROR_PRAM_CRCCHK |
		     ERROR_YRAM_CRCCHK | ERROR_SAFE_GUARD))
			failsafe(tas2557);
	}

	dev_dbg(tas2557->dev, "%s: exit\n", __func__);
	return result;
}

int tas2557_set_sampling_rate(struct tas2557_priv *tas2557,
			      unsigned int nSamplingRate)
{
	int result = 0;
	struct tasdevice_config *config;
	unsigned int nConfiguration;

	dev_dbg(tas2557->dev, "%s: nSamplingRate = %d [Hz]\n", __func__,
		nSamplingRate);

	if ((!tas2557->fmw->programs) ||
	    (!tas2557->fmw->configurations)) {
		dev_err(tas2557->dev, "Firmware not loaded\n");
		result = -EINVAL;
		goto end;
	}

	config =
		&(tas2557->fmw
			  ->configurations[tas2557->mnCurrentConfiguration]);

	if (config->sampling_rate == nSamplingRate) {
		dev_info(
			tas2557->dev,
			"Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		result = 0;
		goto end;
	}

	for (nConfiguration = 0;
	     nConfiguration < tas2557->fmw->mnConfigurations;
	     nConfiguration++) {
		config = &(
			tas2557->fmw->configurations[nConfiguration]);

		if ((config->sampling_rate == nSamplingRate) &&
		    (config->mnProgram == tas2557->mnCurrentProgram)) {
			dev_info(
				tas2557->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				config->name, nSamplingRate);
			result = tas2557_load_configuration(
				tas2557, nConfiguration, false);
			goto end;
		}
	}

	dev_err(tas2557->dev,
		"Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

end:

	return result;
}

int tas2557_set_config(struct tas2557_priv *tas2557, int nr_config)
{
	struct tasdevice_config *config;
	struct tasdevice_program *program;
	unsigned int nProgram = tas2557->mnCurrentProgram;
	unsigned int nConfiguration = nr_config;
	int result = 0;

	if ((!tas2557->fmw->programs) ||
	    (!tas2557->fmw->configurations)) {
		dev_err(tas2557->dev, "Firmware not loaded\n");
		result = -EINVAL;
		goto end;
	}

	if (nConfiguration >= tas2557->fmw->mnConfigurations) {
		dev_err(tas2557->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		result = -EINVAL;
		goto end;
	}

	config =
		&(tas2557->fmw->configurations[nConfiguration]);
	program = &(tas2557->fmw->programs[nProgram]);

	if (nProgram != config->mnProgram) {
		dev_err(tas2557->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, config->name,
			config->mnProgram, nProgram, program->name);
		result = -EINVAL;
		goto end;
	}

	dev_dbg(tas2557->dev, "%s, load new conf %s\n", __func__,
		config->name);
	result = tas2557_load_configuration(tas2557, nConfiguration, false);

end:

	return result;
}

static int tas2557_parse_dt(struct device *dev, struct tas2557_priv *tas2557)
{
	struct device_node *np = dev->of_node;
	int rc = 0, ret = 0;
	unsigned int value;

	tas2557->reset =
		devm_gpiod_get(dev, "ti,tas2557-reset", GPIOD_OUT_LOW);
	if (IS_ERR(tas2557->reset)) {
		ret = PTR_ERR(tas2557->reset);
		dev_err(dev, "Failed to request reset gpio, error %d\n", ret);
		return ret;
	} else
		dev_dbg(tas2557->dev, "%s, tas2557 reset gpio\n", __func__);

	rc = of_property_read_u32(np, "reg", &value);
	if (rc) {
		dev_err(tas2557->dev,
			"Looking up %s property in node %s failed %d\n",
			"reg", np->full_name, rc);
		ret = -EINVAL;
		goto end;
	} else {
		tas2557->dev_addr = value;
		dev_dbg(tas2557->dev, "reg addr=0x%x\n",
			tas2557->dev_addr);
	}
end:

	return ret;
}

// Codec related

static const struct snd_soc_dapm_widget tas2557_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI2", "ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASIM", "ASIM Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV("ClassD", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NDivider", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_dapm_route tas2557_audio_map[] = {
	{ "DAC", NULL, "ASI1" },     { "DAC", NULL, "ASI2" },
	{ "DAC", NULL, "ASIM" },     { "ClassD", NULL, "DAC" },
	{ "OUT", NULL, "ClassD" },   { "DAC", NULL, "PLL" },
	{ "DAC", NULL, "NDivider" },
};

static int tas2557_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *codec = dai->component;
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(codec);

	mutex_lock(&tas2557->codec_lock);

	dev_dbg(tas2557->dev, "%s\n", __func__);
	tas2557_enable(tas2557, !mute);

	mutex_unlock(&tas2557->codec_lock);
	return 0;
}

static int tas2557_hw_params(struct snd_pcm_substream *pSubstream,
			     struct snd_pcm_hw_params *pParams,
			     struct snd_soc_dai *pDAI)
{
	struct snd_soc_component *pCodec = pDAI->component;
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(pCodec);

	mutex_lock(&tas2557->codec_lock);

	dev_dbg(tas2557->dev, "%s\n", __func__);
	tas2557_set_sampling_rate(tas2557, params_rate(pParams));

	mutex_unlock(&tas2557->codec_lock);
	return 0;
}

static int tas2557_configuration_get(struct snd_kcontrol *pKcontrol,
				     struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(pKcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(codec);

	mutex_lock(&tas2557->codec_lock);

	pValue->value.integer.value[0] = tas2557->mnCurrentConfiguration;
	dev_dbg(tas2557->dev, "%s = %d\n", __func__,
		tas2557->mnCurrentConfiguration);

	mutex_unlock(&tas2557->codec_lock);
	return 0;
}

static int tas2557_configuration_put(struct snd_kcontrol *pKcontrol,
				     struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(pKcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(codec);
	unsigned int nConfiguration = pValue->value.integer.value[0];
	int ret = 0;

	mutex_lock(&tas2557->codec_lock);

	dev_info(tas2557->dev, "%s = %d\n", __func__, nConfiguration);
	ret = tas2557_set_config(tas2557, nConfiguration);

	mutex_unlock(&tas2557->codec_lock);
	return ret;
}

static int tas2557_ldac_gain_get(struct snd_kcontrol *pKcontrol,
				 struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(pKcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(codec);
	unsigned char nGain = 0;
	int ret = -1;

	mutex_lock(&tas2557->codec_lock);

	ret = tas2557_get_DAC_gain(tas2557, &nGain);

	if (ret >= 0)
		pValue->value.integer.value[0] = nGain;

	dev_dbg(tas2557->dev, "%s, ret = %d, %d\n", __func__, ret, nGain);

	mutex_unlock(&tas2557->codec_lock);
	return ret;
}

static int tas2557_ldac_gain_put(struct snd_kcontrol *pKcontrol,
				 struct snd_ctl_elem_value *pValue)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(pKcontrol);
	struct tas2557_priv *tas2557 = snd_soc_component_get_drvdata(codec);
	unsigned int nGain = pValue->value.integer.value[0];
	int ret = 0;

	mutex_lock(&tas2557->codec_lock);

	ret = tas2557_set_DAC_gain(tas2557, nGain);

	mutex_unlock(&tas2557->codec_lock);
	return ret;
}

static const struct snd_kcontrol_new tas2557_snd_controls[] = {
	SOC_SINGLE_EXT("TAS2557 DAC Playback Volume", SND_SOC_NOPM, 0, 0x0f, 0,
		       tas2557_ldac_gain_get, tas2557_ldac_gain_put),
	SOC_SINGLE_EXT("Configuration", SND_SOC_NOPM, 0, 0x00FF, 0,
		       tas2557_configuration_get, tas2557_configuration_put),
};

static const struct snd_soc_component_driver soc_codec_driver_tas2557 = {
	.idle_bias_on = false,
	.controls = tas2557_snd_controls,
	.num_controls = ARRAY_SIZE(tas2557_snd_controls),
	.dapm_widgets = tas2557_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2557_dapm_widgets),
	.dapm_routes = tas2557_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tas2557_audio_map),
};

static struct snd_soc_dai_ops tas2557_dai_ops = {
	.mute_stream = tas2557_mute,
	.hw_params = tas2557_hw_params,
};

#define TAS2557_FORMATS                                       \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
static struct snd_soc_dai_driver tas2557_dai_driver[] = {
	{
		.name = "tas2557 ASI1",
		.id = 0,
		.playback = {
			.stream_name = "ASI1 Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = TAS2557_FORMATS,
		},
		.ops = &tas2557_dai_ops,
		.symmetric_rate = 1,
	},
	{
		.name = "tas2557 ASI2",
		.id = 1,
		.playback = {
			.stream_name = "ASI2 Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = TAS2557_FORMATS,
		},
		.ops = &tas2557_dai_ops,
		.symmetric_rate = 1,
	},
	{
		.name = "tas2557 ASIM",
		.id = 2,
		.playback = {
			.stream_name = "ASIM Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = TAS2557_FORMATS,
		},
		.ops = &tas2557_dai_ops,
		.symmetric_rate = 1,
	},
};

int tas2557_register_codec(struct tas2557_priv *tas2557)
{
	int result = 0;

	dev_info(tas2557->dev, "%s, enter\n", __func__);
	result = devm_snd_soc_register_component(
		tas2557->dev, &soc_codec_driver_tas2557, tas2557_dai_driver,
		ARRAY_SIZE(tas2557_dai_driver));
	return result;
}

int tas2557_deregister_codec(struct tas2557_priv *tas2557)
{
	snd_soc_unregister_component(tas2557->dev);
	return 0;
}

//I2C Driver

static bool tas2557_volatile(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static bool tas2557_writeable(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static const struct regmap_config tas2557_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2557_writeable,
	.volatile_reg = tas2557_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

static int tas2557_i2c_probe(struct i2c_client *client)
{
	struct tas2557_priv *tas2557;
	int result;

	dev_info(&client->dev, "%s enter\n", __func__);
	tas2557 = devm_kzalloc(&client->dev, sizeof(struct tas2557_priv),
				GFP_KERNEL);

	if (!tas2557) {
		dev_err(&client->dev, " -ENOMEM\n");
		result = -ENOMEM;
		goto err;
	}

	tas2557->client = client;
	tas2557->dev = &client->dev;
	i2c_set_clientdata(client, tas2557);
	dev_set_drvdata(&client->dev, tas2557);

	tas2557->regmap = devm_regmap_init_i2c(client, &tas2557_i2c_regmap);

	if (IS_ERR(tas2557->regmap)) {
		result = PTR_ERR(tas2557->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			result);
		goto err;
	}

	if (client->dev.of_node)
		tas2557_parse_dt(&client->dev, tas2557);

	tas2557_hw_reset(tas2557);

	tas2557->mnRestart = 0;

	mutex_init(&tas2557->dev_lock);

	/* Reset the chip */
	result = tas2557_dev_write(tas2557, TAS2557_SW_RESET_REG, 1);
	if (result < 0) {
		dev_err(&client->dev, "I2c fail, %d\n", result);
		goto err;
	}
	msleep(1);

	tas2557->fmw = devm_kzalloc(
		&client->dev, sizeof(struct tasdevice_fw), GFP_KERNEL);

	if (!tas2557->fmw) {
		dev_err(&client->dev, "fmw ENOMEM\n");
		result = -ENOMEM;
		goto err;
	}

	mutex_init(&tas2557->codec_lock);
	tas2557_register_codec(tas2557);

	result = request_firmware_nowait(THIS_MODULE, 1, TAS2557_FW_NAME,
					  tas2557->dev, GFP_KERNEL, tas2557,
					  tas2557_fw_ready);

err:

	return result;
}

static void tas2557_i2c_remove(struct i2c_client *client)
{
	struct tas2557_priv *tas2557 = i2c_get_clientdata(client);

	dev_info(tas2557->dev, "%s\n", __func__);

	tas2557_deregister_codec(tas2557);
	mutex_destroy(&tas2557->codec_lock);

	mutex_destroy(&tas2557->dev_lock);
	return;
}

static const struct i2c_device_id tas2557_i2c_id[] = { { "tas2557", 0 }, {} };

MODULE_DEVICE_TABLE(i2c, tas2557_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2557_of_match[] = {
	{ .compatible = "ti,tas2557" },
	{},
};

MODULE_DEVICE_TABLE(of, tas2557_of_match);
#endif

static struct i2c_driver tas2557_i2c_driver = {
	.driver = {
		.name = "tas2557",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tas2557_of_match),
	},
	.probe = tas2557_i2c_probe,
	.remove = tas2557_i2c_remove,
	.id_table = tas2557_i2c_id,
};

module_i2c_driver(tas2557_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2557 ALSA SOC Smart Amplifier Mono driver");
MODULE_LICENSE("GPL v2");
