// SPDX-License-Identifier: GPL-2.0+
// Driver for Awinic AW2023 3-channel LED driver

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define AW2023_MAX_LEDS 3

/* Reset and ID register */
#define AW2023_RSTR 0x00
#define AW2023_RSTR_RESET 0x55
#define AW2023_RSTR_CHIP_ID 0x09

/* Global control register */
#define AW2023_GCR1 0x01
#define AW2023_GCR1_ENABLE BIT(0)
#define AW2023_GCR2 0x04
#define AW2023_GCR2_IMAX_MASK (BIT(0) | BIT(1))

/* LED channel enable register */
#define AW2023_LCTR 0x30
#define AW2023_LCTR_LE(x) BIT((x))

/* LED channel control registers */
#define AW2023_LCFG(x) (0x31 + (x))
#define AW2023_LCFG_LED_CURRENT_MASK (BIT(0) | BIT(1) | BIT(2) | BIT(3)) // Should be 0-3
#define AW2023_LCFG_MD BIT(4)
#define AW2023_LCFG_FI BIT(5)
#define AW2023_LCFG_FO BIT(6)

/* LED channel PWM registers */
#define AW2023_REG_PWM(x) (0x34 + (x))

/* LED channel timing registers */
#define AW2023_LEDT0(x) (0x37 + (x) * 3)
#define AW2023_LEDT0_T1(x) ((x) << 4) // Should be 0-7
#define AW2023_LEDT0_T2(x) (x) // Should be 0-5

#define AW2023_LEDT1(x) (0x38 + (x) * 3)
#define AW2023_LEDT1_T3(x) ((x) << 4) // Should be 0-7
#define AW2023_LEDT1_T4(x) (x) // Should be 0-7

#define AW2023_LEDT2(x) (0x39 + (x) * 3)
#define AW2023_LEDT2_T0(x) ((x) << 4) // Should be 0-8
#define AW2023_LEDT2_REPEAT(x) (x) // Should be 0-15

#define AW2023_REG_MAX 0x7F

#define AW2023_TIME_STEP 130 /* ms */
/* aw2023 downstream only registers */

// /* register bits */
// #define AW2023_CHIP_DISABLE_MASK 0x00
// #define AW2023_CHIP_ENABLE_MASK 0x01
// #define aw2023_BREATH_MODE_MASK 0x10
// #define aw2023_MANUAL_MODE_MASK 0x00
// #define aw2023_BREATHE_PWM_MASK 0xFF
// #define aw2023_MANUAL_PWM_MASK 0xFF
// #define aw2023_FADEIN_MODE_MASK 0x20
// #define aw2023_FADEOUT_MODE_MASK 0x40

// #define MAX_RISE_TIME_MS 15
// #define MAX_HOLD_TIME_MS 15
// #define MAX_FALL_TIME_MS 15
// #define MAX_OFF_TIME_MS 15

// /* aw2023 register read/write access*/
// #define REG_NONE_ACCESS 0
// #define REG_RD_ACCESS 1 << 0
// #define REG_WR_ACCESS 1 << 1

struct aw2023_led {
	struct aw2023 *chip;
	struct led_classdev cdev;
	u32 num;
	unsigned int led_current;
};

struct aw2023 {
	struct mutex mutex; /* held when writing to registers */
	struct regulator *vcc_regulator;
	struct i2c_client *client;
	struct aw2023_led leds[AW2023_MAX_LEDS];
	struct regmap *regmap;
	unsigned int imax;
	int num_leds;
	bool enabled;
	// struct work_struct brightness_work;
};

static int aw2023_chip_init(struct aw2023 *chip)
{
	int i, ret;

	ret = regmap_write(chip->regmap, AW2023_GCR1, AW2023_GCR1_ENABLE);
	if (ret) {
		dev_err(&chip->client->dev, "Failed to enable the chip: %d\n",
			ret);
		return ret;
	}

	ret = regmap_update_bits(chip->regmap,
				AW2023_GCR2,
				AW2023_GCR2_IMAX_MASK,
				chip->imax);
	if (ret) {
		dev_err(&chip->client->dev, "Failed to set the chip IMAX: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < chip->num_leds; i++) {
		ret = regmap_update_bits(chip->regmap,
					 AW2023_LCFG(chip->leds[i].num),
					 AW2023_LCFG_LED_CURRENT_MASK,
					 chip->leds[i].led_current);
		if (ret) {
			dev_err(&chip->client->dev,
				"Failed to set led current for led %d: %d\n",
				chip->leds[i].num, ret);
			return ret;
		}
	}

	return ret;
}

static void aw2023_chip_disable(struct aw2023 *chip)
{
	int ret;

	if (!chip->enabled)
		return;

	regmap_write(chip->regmap, AW2023_GCR1, 0);

	ret = regulator_bulk_disable(ARRAY_SIZE(chip->regulators),
				     chip->regulators);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to disable regulators: %d\n", ret);
		return;
	}

	chip->enabled = false;
}

static int aw2023_chip_enable(struct aw2023 *chip)
{
	int ret;

	if (chip->enabled)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(chip->regulators),
				    chip->regulators);
	if (ret) {
		dev_err(&chip->client->dev,
			"Failed to enable regulators: %d\n", ret);
		return ret;
	}
	chip->enabled = true;

	ret = aw2023_chip_init(chip);
	if (ret)
		aw2023_chip_disable(chip);

	return ret;
}

static bool aw2023_chip_in_use(struct aw2023 *chip)
{
	int i;

	for (i = 0; i < chip->num_leds; i++)
		if (chip->leds[i].cdev.brightness)
			return true;

	return false;
}

static int aw2023_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct aw2023_led *led = container_of(cdev, struct aw2023_led, cdev);
	int ret, num;

	mutex_lock(&led->chip->mutex);

	if (aw2023_chip_in_use(led->chip)) {
		ret = aw2023_chip_enable(led->chip);
		if (ret)
			goto error;
	}

	num = led->num;

	ret = regmap_write(led->chip->regmap, AW2023_REG_PWM(num), brightness);
	if (ret)
		goto error;

	if (brightness) {
		ret = regmap_update_bits(led->chip->regmap, AW2023_LCTR,
					 AW2023_LCTR_LE(num), 0xFF);
	} else {
		ret = regmap_update_bits(led->chip->regmap, AW2023_LCTR,
					 AW2023_LCTR_LE(num), 0);
		if (ret)
			goto error;
		ret = regmap_update_bits(led->chip->regmap, AW2023_LCFG(num),
					 AW2023_LCFG_MD, 0);
	}
	if (ret)
		goto error;

	if (!aw2023_chip_in_use(led->chip))
		aw2023_chip_disable(led->chip);

error:
	mutex_unlock(&led->chip->mutex);

	return ret;
}

// static void aw2023_brightness_work(struct work_struct* work) {
// 	struct aw2023* led = container_of(work, struct aw2023, brightness_work);
// 	u8 val;

// 	mutex_lock(&led->pdata->led->lock);

// 	/* enable aw2023 if disabled */
// 	aw2023_read(led, AW2023_REG_GCR1, &val);
// 	if (!(val & AW2023_CHIP_ENABLE_MASK)) {
// 		aw2023_write(led, AW2023_REG_GCR1, AW2023_CHIP_ENABLE_MASK);
// 		msleep(2);
// 	}

// 	if (led->cdev.brightness > 0) {
// 		if (led->cdev.brightness > led->cdev.max_brightness)
// 			led->cdev.brightness = led->cdev.max_brightness;
// 		aw2023_write(led, AW2023_REG_GCR2, led->pdata->imax);
// 		aw2023_write(led, AW2023_REG_LCFG0 + led->id,
// 					 (aw2023_MANUAL_MODE_MASK | led->pdata->led_current));
// 		aw2023_write(led, AW2023_REG_PWM0 + led->id, led->cdev.brightness);
// 		aw2023_read(led, AW2023_REG_LEDEN, &val);
// 		aw2023_write(led, AW2023_REG_LEDEN, val | (1 << led->id));
// 	} else {
// 		aw2023_read(led, AW2023_REG_LEDEN, &val);
// 		aw2023_write(led, AW2023_REG_LEDEN, val & (~(1 << led->id)));
// 	}

// 	/*
// 	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
// 	 * all off. So we need to power it off.
// 	 */
// 	aw2023_read(led, AW2023_REG_LEDEN, &val);
// 	if (val == 0) {
// 		aw2023_write(led, AW2023_REG_GCR1, AW2023_CHIP_DISABLE_MASK);
// 		mutex_unlock(&led->pdata->led->lock);
// 		return;
// 	}

// 	mutex_unlock(&led->pdata->led->lock);
// }

static int aw2023_blink_set(struct led_classdev *cdev,
			    unsigned long *delay_on, unsigned long *delay_off)
{
	struct aw2023_led *led = container_of(cdev, struct aw2023_led, cdev);
	int ret, num = led->num;
	unsigned long off = 0, on = 0;

	/* If no blink specified, default to 1 Hz. */
	if (!*delay_off && !*delay_on) {
		*delay_off = 500;
		*delay_on = 500;
	}

	if (!led->cdev.brightness) {
		led->cdev.brightness = LED_FULL;
		ret = aw2023_brightness_set(&led->cdev, led->cdev.brightness);
		if (ret)
			return ret;
	}

	/* Never on - just set to off */
	if (!*delay_on) {
		led->cdev.brightness = LED_OFF;
		return aw2023_brightness_set(&led->cdev, LED_OFF);
	}

	mutex_lock(&led->chip->mutex);

	/* Never off - brightness is already set, disable blinking */
	if (!*delay_off) {
		ret = regmap_update_bits(led->chip->regmap, AW2023_LCFG(num),
					 AW2023_LCFG_MD, 0);
		goto out;
	}

	/* Convert into values the HW will understand. */
	off = min(5, ilog2((*delay_off - 1) / AW2023_TIME_STEP) + 1);
	on = min(7, ilog2((*delay_on - 1) / AW2023_TIME_STEP) + 1);

	*delay_off = BIT(off) * AW2023_TIME_STEP;
	*delay_on = BIT(on) * AW2023_TIME_STEP;

	/* Set timings */
	ret = regmap_write(led->chip->regmap,
			   AW2023_LEDT0(num), AW2023_LEDT0_T2(on));
	if (ret)
		goto out;
	ret = regmap_write(led->chip->regmap,
			   AW2023_LEDT1(num), AW2023_LEDT1_T4(off));
	if (ret)
		goto out;

	/* Finally, enable the LED */
	ret = regmap_update_bits(led->chip->regmap, AW2023_LCFG(num),
				 AW2023_LCFG_MD, 0xFF);
	if (ret)
		goto out;

	ret = regmap_update_bits(led->chip->regmap, AW2023_LCTR,
				 AW2023_LCTR_LE(num), 0xFF);

out:
	mutex_unlock(&led->chip->mutex);

	return ret;
}

// static void aw2023_blink_set(struct aw2023* led, unsigned long blinking) {
// 	u8 val;

// 	/* enable aw2023 if disabled */
// 	aw2023_read(led, AW2023_REG_GCR1, &val);
// 	if (!(val & AW2023_CHIP_ENABLE_MASK)) {
// 		aw2023_write(led, AW2023_REG_GCR1, AW2023_CHIP_ENABLE_MASK);
// 		msleep(2);
// 	}

// 	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;
// 	if (blinking > 0) {
// 		aw2023_write(led, AW2023_REG_GCR2, led->pdata->imax);
// 		aw2023_write(led, AW2023_REG_PWM0 + led->id, led->cdev.brightness);
// 		aw2023_write(led, AW2023_REG_LED0T0 + led->id * 3,
// 					 (led->pdata->rise_time_ms << 4 | led->pdata->hold_time_ms));
// 		aw2023_write(led, AW2023_REG_LED0T1 + led->id * 3,
// 					 (led->pdata->fall_time_ms << 4 | led->pdata->off_time_ms));
// 		aw2023_write(led, AW2023_REG_LCFG0 + led->id,
// 					 (aw2023_BREATH_MODE_MASK | led->pdata->led_current));
// 		aw2023_read(led, AW2023_REG_LEDEN, &val);
// 		aw2023_write(led, AW2023_REG_LEDEN, val | (1 << led->id));
// 	} else {
// 		aw2023_read(led, AW2023_REG_LEDEN, &val);
// 		aw2023_write(led, AW2023_REG_LEDEN, val & (~(1 << led->id)));
// 	}

// 	/*
// 	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
// 	 * all off. So we need to power it off.
// 	 */
// 	aw2023_read(led, AW2023_REG_LEDEN, &val);
// 	if (val == 0) {
// 		aw2023_write(led, AW2023_REG_GCR1, AW2023_CHIP_DISABLE_MASK);
// 	}
// }

// static ssize_t aw2023_reg_store(struct device* dev, struct device_attribute* attr, const char* buf,
// 								size_t len) {
// 	struct led_classdev* led_cdev = dev_get_drvdata(dev);
// 	struct aw2023* led = container_of(led_cdev, struct aw2023, cdev);

// 	unsigned int databuf[2];

// 	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
// 		aw2023_write(led, (unsigned char)databuf[0], (unsigned char)databuf[1]);
// 	}

// 	return len;
// }
// static DEVICE_ATTR(blink, 0664, NULL, aw2023_store_blink);
// static DEVICE_ATTR(led_time, 0664, aw2023_time_show, aw2023_time_store);
// static DEVICE_ATTR(reg, 0664, aw2023_reg_show, aw2023_reg_store);

// static struct attribute* aw2023_attributes[] = {
// 	&dev_attr_blink.attr,
// 	&dev_attr_led_time.attr,
// 	&dev_attr_reg.attr,
// 	NULL,
// };

static int aw2023_probe_dt(struct aw2023 *chip)
{
	struct device_node *np = dev_of_node(&chip->client->dev), *child;
	int count, ret = 0, i = 0;
	struct aw2023_led *led;

	count = of_get_available_child_count(np);
	if (!count || count > AW2023_MAX_LEDS)
		return -EINVAL;

	regmap_write(chip->regmap, AW2023_RSTR, AW2023_RSTR_RESET);

	for_each_available_child_of_node(np, child) {
		struct led_init_data init_data = {};
		u32 source;
		u32 led_current;

		ret = of_property_read_u32(child, "reg", &source);
		if (ret != 0 || source >= AW2023_MAX_LEDS) {
			dev_err(&chip->client->dev,
				"Couldn't read LED address: %d\n", ret);
			count--;
			continue;
		}

		// ret = of_property_read_u32(child, "rise-time-ms", &rise_time_ms);
		// if (ret != 0) {
		// 	dev_err(&chip->client->dev,
		// 		 "DT property rise-time-ms is missing\n");
		// 	count--;
		// 	continue;
		// }

		led = &chip->leds[i];
		led->num = source;
		led->chip = chip;
		init_data.fwnode = of_fwnode_handle(child);

		if (!of_property_read_u32(child, "led-current", &led_current)) {
			led->led_current = min_t(u32, led_current / 5000, 3);
		} else {
			led->led_current = 2; // 5mA
			dev_info(&chip->client->dev,
				 "DT property led-current is missing\n");
		}

		led->cdev.brightness_set_blocking = aw2023_brightness_set;
		led->cdev.blink_set = aw2023_blink_set;

		ret = devm_led_classdev_register_ext(&chip->client->dev,
						     &led->cdev, &init_data);
		if (ret < 0) {
			of_node_put(child);
			return ret;
		}

		i++;
	}

	if (!count)
		return -EINVAL;

	chip->num_leds = i;

	return 0;
}

// static int aw2023_parse_child_node(struct aw2023* led_array, struct device_node* node) {
// 	struct aw2023* led;
// 	struct device_node* temp;
// 	struct aw2023_platform_data* pdata;
// 	int rc = 0, parsed_leds = 0;

// 	for_each_child_of_node(node, temp) {
// 		led = &led_array[parsed_leds];
// 		led->client = led_array->client;

// 		pdata = devm_kzalloc(&led->client->dev, sizeof(struct aw2023_platform_data), GFP_KERNEL);
// 		if (!pdata) {
// 			dev_err(&led->client->dev, "Failed to allocate memory\n");
// 			goto free_err;
// 		}
// 		pdata->led = led_array;
// 		led->pdata = pdata;

// 		rc = of_property_read_string(temp, "aw2023,name", &led->cdev.name);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading led name, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,id", &led->id);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading id, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,imax", &led->pdata->imax);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading id, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,led-current", &led->pdata->led_current);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading led-current, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,max-brightness", &led->cdev.max_brightness);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading max-brightness, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,rise-time-ms", &led->pdata->rise_time_ms);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading rise-time-ms, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,hold-time-ms", &led->pdata->hold_time_ms);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading hold-time-ms, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,fall-time-ms", &led->pdata->fall_time_ms);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading fall-time-ms, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		rc = of_property_read_u32(temp, "aw2023,off-time-ms", &led->pdata->off_time_ms);
// 		if (rc < 0) {
// 			dev_err(&led->client->dev, "Failure reading off-time-ms, rc = %d\n", rc);
// 			goto free_pdata;
// 		}

// 		INIT_WORK(&led->brightness_work, aw2023_brightness_work);

// 		led->cdev.brightness_set = aw2023_set_brightness;

// 		rc = led_classdev_register(&led->client->dev, &led->cdev);
// 		if (rc) {
// 			dev_err(&led->client->dev, "unable to register led %d,rc=%d\n", led->id, rc);
// 			goto free_pdata;
// 		}

// 		rc = sysfs_create_group(&led->cdev.dev->kobj, &aw2023_attr_group);
// 		if (rc) {
// 			dev_err(&led->client->dev, "led sysfs rc: %d\n", rc);
// 			goto free_class;
// 		}
// 		parsed_leds++;
// 	}

// 	return 0;

// free_class:
// 	led_classdev_unregister(&led_array[parsed_leds].cdev);
// 	cancel_work_sync(&led_array[parsed_leds].brightness_work);
// 	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
// 	led_array[parsed_leds].pdata = NULL;
// 	return rc;
// }

static const struct regmap_config aw2023_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW2023_REG_MAX,
};

static int aw2023_probe(struct i2c_client *client)
{
	struct aw2023 *chip;
	int ret;
	unsigned int chipid;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->mutex);
	mutex_lock(&chip->mutex);

	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &aw2023_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		goto error;
	}

	chip->regulators[0].supply = "vcc";
	chip->regulators[1].supply = "vio";
	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(chip->regulators),
				      chip->regulators);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev,
				"Failed to request regulators: %d\n", ret);
		goto error;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(chip->regulators),
				    chip->regulators);
	if (ret) {
		dev_err(&client->dev,
			"Failed to enable regulators: %d\n", ret);
		goto error;
	}

	ret = regmap_read(chip->regmap, AW2023_RSTR, &chipid);
	if (ret) {
		dev_err(&client->dev, "Failed to read chip ID: %d\n",
			ret);
		goto error_reg;
	}

	if (chipid != AW2023_RSTR_CHIP_ID) {
		dev_err(&client->dev, "Chip reported wrong ID: %x\n",
			chipid);
		ret = -ENODEV;
		goto error_reg;
	}

	ret = aw2023_probe_dt(chip);
	if (ret < 0)
		goto error_reg;

	ret = regulator_bulk_disable(ARRAY_SIZE(chip->regulators),
				     chip->regulators);
	if (ret) {
		dev_err(&client->dev,
			"Failed to disable regulators: %d\n", ret);
		goto error;
	}

	mutex_unlock(&chip->mutex);

	return 0;

error_reg:
	regulator_bulk_disable(ARRAY_SIZE(chip->regulators),
			       chip->regulators);

error:
	mutex_destroy(&chip->mutex);
	return ret;
}

static void aw2023_remove(struct i2c_client *client)
{
	struct aw2023 *chip = i2c_get_clientdata(client);

	aw2023_chip_disable(chip);

	mutex_destroy(&chip->mutex);
}

static const struct of_device_id aw2023_match_table[] = {
	{ .compatible = "awinic,aw2023", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, aw2023_match_table);

static struct i2c_driver aw2023_driver = {
	.driver = {
		.name = "leds-aw2023",
		.of_match_table = aw2023_match_table,
	},
	.probe = aw2023_probe,
	.remove = aw2023_remove,
};

module_i2c_driver(aw2023_driver);

MODULE_AUTHOR("Nikita Travkin <nikitos.tr@gmail.com>");
MODULE_DESCRIPTION("AW2023 LED driver");
MODULE_LICENSE("GPL v2");
