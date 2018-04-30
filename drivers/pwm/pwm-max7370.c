/*
 * Copyright (C) 2017 Adakta Ltd
 *
 * Author: Maxim Paymushkin <maxim.paymushkin@gmail.com>
 *
 * License Terms: GNU General Public License, version 2
 *
 * MAX7370 PWM driver
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/mfd/max7370.h>

#define MAX7370_PWM_ACTIVE_DEFAULT           0
#define MAX7370_PWM_COMMON_RATIO_DEFAULT     0
#define MAX7370_PWM_INDIVIDUAL_RATIO_DEFAULT 0
#define MAX7370_PWM_RATIO_MAX                255
#define MAX7370_PWM_ACTIVE_RATIO_DEFAULT     0
#define MAX7370_PWM_BLINK_PERIOD_DEFAULT     0
#define MAX7370_PWM_BLINK_PERIOD_MAX         4096
#define MAX7370_PWM_FADING_TIME_DEFAULT      0
#define MAX7370_PWM_FADING_TIME_MAX          4096
#define MAX7370_PWM_BLINK_ON_TIME_DEFAULT    0
#define MAX7370_PWM_BLINK_ON_TIME_MAX        3

#define MAX7370_PWM_PERIOD_DEFAULT           2000000 /* Default period_ns = 1/500 Hz */

/**
 * struct max7370_pwm_platform_data - platform specific pwm data
 * @active_leds:             mask of active LEDs
 * @common_pwm_ratio:        common PWM ratio
 * @active_common_pwm_ratio: array of active common PWM ratio
 * @blink_period:            array of blink period per PWM
 * @blink_on_time:           array of blink-on time per PWM
 * @fading_time:             fade-in/out time
 * @constant_current:        LED Constant-Current
 */
struct max7370_pwm_platform_data {
	u8   active_leds;
	u8   common_pwm_ratio;
	bool active_common_pwm_ratio[MAX7370_MAX_PWM];
	u8   blink_period[MAX7370_MAX_PWM];
	u8   blink_on_time[MAX7370_MAX_PWM];
	u8   fading_time;
	u8   constant_current;
};

/**
 * struct max7370_pwm - data structure used by PWM driver
 * @chip:           pointer to PWM chip
 * @max7370:        pointer to max7370
 * @board:          PWM platform device
 * @duty_cycle:     array of duty cycle of LEDs
 * @enabled_leds:   array of enabled LEDs
 */
struct max7370_pwm {
	struct pwm_chip  chip;
	struct max7370  *max7370;
	const struct max7370_pwm_platform_data *board;
	u8               duty_cycle[MAX7370_MAX_PWM];
	bool             enabled_leds[MAX7370_MAX_PWM];
};

static inline struct max7370_pwm *to_max7370_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct max7370_pwm, chip);
}

static inline bool max7370_pwm_is_active(
	struct pwm_chip *chip, struct pwm_device *pwm)
{
	const struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	const struct max7370_pwm_platform_data *board = max7370_pwm->board;

	return board->active_leds & (1 << pwm->hwpwm);
}

static int max7370_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	struct max7370 *max7370 = max7370_pwm->max7370;
	int ret;

	ret = max7370_set_bits(max7370,
			MAX7370_REG_LED_DRIVER_ENABLE,
			1 << pwm->hwpwm,
			1 << pwm->hwpwm);

	if (ret < 0)
		dev_err(chip->dev, "failed to enable pwm%u\n", pwm->hwpwm);
	else
		max7370_pwm->enabled_leds[pwm->hwpwm] = true;

	return ret;
}

static void max7370_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	struct max7370 *max7370 = max7370_pwm->max7370;
	int ret;

	ret = max7370_set_bits(max7370,
			MAX7370_REG_LED_DRIVER_ENABLE,
			1 << pwm->hwpwm,
			0x00);

	if (ret < 0)
		dev_err(chip->dev, "failed to disable pwm%u\n", pwm->hwpwm);

	max7370_pwm->enabled_leds[pwm->hwpwm] = false;
}

static int max7370_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	struct max7370 *max7370 = max7370_pwm->max7370;
	const struct max7370_pwm_platform_data *board = max7370_pwm->board;
	int ret;

	if (period_ns != MAX7370_PWM_PERIOD_DEFAULT) {
		dev_err(chip->dev, "invalid PWM period, accept %u ns only\n",
				MAX7370_PWM_PERIOD_DEFAULT);
		return -EINVAL;
	}

	if (duty_ns == period_ns)
		max7370_pwm->duty_cycle[pwm->hwpwm] = MAX7370_PWM_RATIO_MAX;
	else if (duty_ns)
		max7370_pwm->duty_cycle[pwm->hwpwm] =
			(u8) (MAX7370_PWM_RATIO_MAX * duty_ns / period_ns);
	else
		max7370_pwm->duty_cycle[pwm->hwpwm] = 0;

	if (max7370_pwm->duty_cycle[pwm->hwpwm] != 0 &&
		board->active_common_pwm_ratio[pwm->hwpwm])
		dev_warn(chip->dev, "pwm%d uses common PWM ratio\n", pwm->hwpwm);

	ret = max7370_reg_write(max7370,
			MAX7370_REG_INDIVIDUAL_PWM_RATIO_BASE + pwm->hwpwm,
			max7370_pwm->duty_cycle[pwm->hwpwm]);
	if (ret < 0) {
		dev_err(chip->dev, "PWM config failed\n");
		return ret;
	}

	return 0;
}

static int max7370_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	if (!max7370_pwm_is_active(chip, pwm)) {
		dev_err(chip->dev, "pwm%u inactive\n", pwm->hwpwm);
		return -EINVAL;
	}

	return 0;
}

static void max7370_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	max7370_pwm_disable(chip, pwm);
}

static const struct pwm_ops max7370_pwm_ops = {
	.enable = max7370_pwm_enable,
	.disable = max7370_pwm_disable,
	.config = max7370_pwm_config,
	.request = max7370_pwm_request,
	.free = max7370_pwm_free,
	.owner = THIS_MODULE,
};

static int max7370_pwm_of_probe_array(struct device *dev,
	const char* propname, u32* array, u32 defvalue)
{
	struct device_node *np = dev->of_node;
	u32 val;
	int proplen, i, ret;

	if (of_get_property(np, propname, &proplen)) {

		ret = of_property_count_u32_elems(np, propname);

		if (ret < 0) {
			dev_err(dev,
				"get count of elements of property %s failed (%d)\n",
				propname, ret);
			return ret;
		}

		if (ret != MAX7370_MAX_PWM) {
			dev_err(dev,
				"count of elements of property %s must be equal to %d\n",
				propname, MAX7370_PWM_RATIO_MAX);
			return -EINVAL;
		}

		for (i = 0; i < MAX7370_MAX_PWM; ++i) {
			ret = of_property_read_u32_index(np, propname, i, &val);
			if (ret < 0) {
				dev_err(dev,
					"get element %d of property %s failed (%d)\n",
					i, propname, ret);
				return ret;
			}

			array[i] = val;
		}

	} else {
		for (i = 0; i < MAX7370_MAX_PWM; ++i)
			array[i] = defvalue;
	}

	return 0;
}

static int max7370_pwm_of_probe_leds(struct device *dev,
	struct max7370_pwm_platform_data *plat)
{
	int i, ret;
	u32 val_array[MAX7370_MAX_PWM];

	ret = max7370_pwm_of_probe_array(dev, "maxim,leds",
				val_array, MAX7370_PWM_ACTIVE_DEFAULT);
	if (ret)
		return ret;

	plat->active_leds = 0;

	for(i = 0; i < MAX7370_MAX_PWM; ++i)
		plat->active_leds |= (val_array[i] ? 1 : 0) << i;

	return 0;
}

static int max7370_pwm_of_probe_common_pwm_ratio(struct device *dev,
	struct max7370_pwm_platform_data *plat)
{
	const char* propname = "maxim,common-pwm-ratio";
	struct device_node *np = dev->of_node;
	int ret, proplen;
	u32 val;

	if (of_get_property(np, propname, &proplen)) {
		ret = of_property_read_u32(np, propname, &val);
		if (ret < 0) {
			dev_err(dev,
				"read property %s failed (%d)\n",
				propname, ret);
			return ret;
		}

		if (val > MAX7370_PWM_RATIO_MAX) {
			dev_err(dev,
				"common PWM ratio must be less or equal to %d\n",
				MAX7370_PWM_RATIO_MAX);
			return -EOVERFLOW;
		}

		plat->common_pwm_ratio = (u8) val;

	} else {
		plat->common_pwm_ratio = MAX7370_PWM_COMMON_RATIO_DEFAULT;
	}

	return 0;
}

static int max7370_pwm_of_probe_active_pwm_ratio(struct device *dev,
	struct max7370_pwm_platform_data *plat)
{
	int i, ret;
	u32 val_array[MAX7370_MAX_PWM];

	ret = max7370_pwm_of_probe_array(dev, "maxim,active-pwm-ratio",
				val_array, MAX7370_PWM_ACTIVE_RATIO_DEFAULT);
	if (ret)
		return ret;

	for(i = 0; i < MAX7370_MAX_PWM; ++i)
		plat->active_common_pwm_ratio[i] = val_array[i] ? true : false;

	return 0;
}

static int max7370_pwm_of_probe_blink_period(struct device *dev,
	struct max7370_pwm_platform_data *plat)
{
	int i, ret;
	u32 val_array[MAX7370_MAX_PWM];

	ret = max7370_pwm_of_probe_array(dev, "maxim,blink-period-ms",
				val_array, MAX7370_PWM_BLINK_PERIOD_DEFAULT);
	if (ret)
		return ret;

	for(i = 0; i < MAX7370_MAX_PWM; ++i) {

		if (val_array[i] > MAX7370_PWM_BLINK_PERIOD_MAX) {
			dev_err(dev,
				"element %d has invalid PWM blink period, "
				"must be 0, 256, 512, 1024, 2048 or 4096 ms\n", i);
			return -EOVERFLOW;
		}

		val_array[i] >>= 8;

		plat->blink_period[i] = (u8) ffs(val_array[i]);
	}

	return 0;
}

static int max7370_pwm_of_probe_fading_time(struct device *dev,
	struct max7370_pwm_platform_data *plat)
{
	const char* propname = "maxim,fading-time-ms";
	struct device_node *np = dev->of_node;
	int ret, proplen;
	u32 val;

	if (of_get_property(np, propname, &proplen)) {
		ret = of_property_read_u32(np, propname, &val);
		if (ret < 0) {
			dev_err(dev,
				"read property %s failed (%d)\n",
				propname, ret);
			return ret;
		}

		if (val > MAX7370_PWM_FADING_TIME_MAX) {
			dev_err(dev, "PWM fading time must be "
				"0, 256, 512, 1024, 2048 or 4096 ms\n");
			return -EOVERFLOW;
		}

		val >>= 8;

		plat->fading_time = (u8) ffs(val);

	} else {
		plat->fading_time = MAX7370_PWM_FADING_TIME_DEFAULT;
	}

	return 0;
}

static int max7370_pwm_of_probe_blink_on_time(struct device *dev,
	struct max7370_pwm_platform_data *plat)
{
	int i, ret;
	u32 val_array[MAX7370_MAX_PWM];

	ret = max7370_pwm_of_probe_array(dev, "maxim,blink-on-time",
				val_array, MAX7370_PWM_BLINK_ON_TIME_DEFAULT);
	if (ret)
		return ret;

	for(i = 0; i < MAX7370_MAX_PWM; ++i) {

		if (val_array[i] > MAX7370_PWM_BLINK_ON_TIME_MAX) {
			dev_err(dev,
				"element %d has invalid blink-on time, "
				"must be from 0 to 3\n", i);
			return -EOVERFLOW;
		}

		plat->blink_on_time[i] = (u8) val_array[i];
	}

	return 0;
}

static const struct max7370_pwm_platform_data *
max7370_pwm_of_probe(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct max7370_pwm_platform_data *plat;
	int ret;

	if (!np)
		return ERR_PTR(-ENODEV);

	plat = devm_kzalloc(dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

	ret = max7370_pwm_of_probe_leds(dev, plat);
	if (ret)
		return ERR_PTR(ret);

	ret = max7370_pwm_of_probe_common_pwm_ratio(dev, plat);
	if (ret)
		return ERR_PTR(ret);

	ret = max7370_pwm_of_probe_active_pwm_ratio(dev, plat);
	if (ret)
		return ERR_PTR(ret);

	ret = max7370_pwm_of_probe_blink_period(dev, plat);
	if (ret)
		return ERR_PTR(ret);

	ret = max7370_pwm_of_probe_blink_on_time(dev, plat);
	if (ret)
		return ERR_PTR(ret);

	ret = max7370_pwm_of_probe_fading_time(dev, plat);
	if (ret)
		return ERR_PTR(ret);

	plat->constant_current = of_property_read_bool(np,
			"maxim,constant-current-10mA");

	return plat;
}

static int max7370_pwm_chip_enable(struct max7370_pwm *pwm)
{
	const struct max7370_pwm_platform_data *board = pwm->board;
	struct max7370 *max7370 = pwm->max7370;
	struct device *dev = pwm->chip.dev;
	int ret, i;

	/* enable GPIO */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_ENABLE,
			0x00 | MAX7370_GPIOCFG_ENABLE);
	if (ret < 0)
		goto fail;

	/* set Fade in/out time */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_FADING,
			0x00 | board->fading_time);
	if (ret < 0)
		goto fail;

	ret = max7370_set_bits(max7370,
			MAX7370_REG_LED_CONSTANT_CURRENT,
			MAX7370_LED_CONSTANT_CURRENT_SETTINGS,
			board->constant_current);
	if (ret < 0)
		goto fail;

	ret = max7370_reg_write(max7370,
			MAX7370_REG_COMMON_PWM_RATIO,
			board->common_pwm_ratio);
	if (ret < 0)
		goto fail;

	for (i = 0; i < MAX7370_MAX_PWM; ++i) {
		ret = max7370_set_bits(max7370,
				MAX7370_REG_LED_CONFIG_BASE + i,
				MAX7370_LEDCFG_BLINK_ON_TIME_MASK |
					MAX7370_LEDCFG_BLINK_PERIOD_MASK |
					MAX7370_LEDCFG_ACTIVE_COMMON_PWM_MASK,
				(board->blink_on_time[i]
					<< MAX7370_LEDCFG_BLINK_ON_TIME_SHIFT) |
				(board->blink_period[i]
					<< MAX7370_LEDCFG_BLINK_PERIOD_SHIFT) |
				((board->active_common_pwm_ratio[i] ? 1 : 0)
					<< MAX7370_LEDCFG_ACTIVE_COMMON_PWM_SHIFT));
		if (ret < 0)
			goto fail;
	}

	for (i = 0; i < MAX7370_MAX_PWM; ++i) {
		if (!pwm->enabled_leds[i])
			continue;

		ret = max7370_reg_write(max7370,
				MAX7370_REG_INDIVIDUAL_PWM_RATIO_BASE + i,
				pwm->duty_cycle[i]);
		if (ret < 0)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "failed to enable PWM module\n");
	return ret;
}

static int max7370_pwm_chip_disable(struct max7370_pwm *pwm)
{
	struct max7370 *max7370 = pwm->max7370;
	struct device *dev = pwm->chip.dev;
	const struct max7370_pwm_platform_data *board = pwm->board;
	bool has_gpio_block;
	int ret;

	has_gpio_block = max7370->pdata->block & (MAX7370_BLOCK_GPIO);

	/* disable GPIO */
	if (!has_gpio_block) {
		ret = max7370_set_bits(max7370,
				MAX7370_REG_GPIOCFG,
				MAX7370_GPIOCFG_ENABLE,
				0x00);
		if (ret < 0)
			goto fail;
	}

	/* disable Fade in/out */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_FADING,
			0x00);
	if (ret < 0)
		goto fail;

	/* disable LED driver for all LEDs */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_LED_DRIVER_ENABLE,
			GENMASK(MAX7370_MAX_PWM - 1, 0),
			0x00);
	if (ret < 0)
		goto fail;

fail:
	dev_err(dev, "failed to disable PWM module\n");
	return ret;
}

static int max7370_pwm_probe(struct platform_device *pdev)
{
	struct max7370 *max7370 = dev_get_drvdata(pdev->dev.parent);
	struct max7370_pwm *pwm;
	const struct max7370_pwm_platform_data *plat;
	int ret,i;

	plat = max7370_pwm_of_probe(&pdev->dev);
	if (IS_ERR(plat)) {
		dev_err(&pdev->dev, "invalid PWM platform data\n");
		return PTR_ERR(plat);
	}

	pwm = devm_kzalloc(&pdev->dev, sizeof(struct max7370_pwm),
				    GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->board = plat;
	pwm->max7370 = max7370;

	pwm->chip.ops = &max7370_pwm_ops;
	/* add an extra channel for ALL_LED */
	pwm->chip.npwm = MAX7370_MAX_PWM;

	pwm->chip.dev = &pdev->dev;
	pwm->chip.base = -1;
	pwm->chip.can_sleep = true;

	ret = pwmchip_add(&pwm->chip);
	if (ret) {
		dev_err(&pdev->dev, "unable to add pwm chip: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pwm);

	for (i = 0; i < MAX7370_MAX_PWM; ++i) {
		pwm->enabled_leds[i] = false;
		pwm->duty_cycle[i] = 0;
	}

	ret = max7370_pwm_chip_disable(pwm);
	if (ret)
		return ret;

	ret = max7370_pwm_chip_enable(pwm);
	if (ret)
		return ret;

	return 0;
}

static int max7370_pwm_remove(struct platform_device *pdev)
{
	struct max7370_pwm *pwm = platform_get_drvdata(pdev);
	int ret;

	max7370_pwm_chip_disable(pwm);

	ret = pwmchip_remove(&pwm->chip);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_PM
static int max7370_pwm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_pwm *pwm = platform_get_drvdata(pdev);

	return max7370_pwm_chip_disable(pwm);
}

static int max7370_pwm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_pwm *pwm = platform_get_drvdata(pdev);

	return max7370_pwm_chip_enable(pwm);
}

static const struct dev_pm_ops max7370_pwm_dev_pm_ops = {
	.suspend = max7370_pwm_suspend,
	.resume  = max7370_pwm_resume,
};
#endif

static const struct of_device_id max7370_pwm_match[] = {
	/* Legacy compatible string */
	{ .compatible = "maxim,max7370-pwm", .data = (void *) MAX7370 },
	{ }
};
MODULE_DEVICE_TABLE(of, max7370_pwm_match);

static struct platform_driver max7370_pwm_driver = {
	.driver	= {
		.name			= "max7370-pwm",
		.owner			= THIS_MODULE,
		.of_match_table	= of_match_ptr(max7370_pwm_match),
#ifdef CONFIG_PM
		.pm				= &max7370_pwm_dev_pm_ops,
#endif
	},
	.probe			= max7370_pwm_probe,
	.remove 		= max7370_pwm_remove,
};
module_platform_driver(max7370_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Maxim Paymushkin");
MODULE_DESCRIPTION("MAX7370 PWM Driver");
MODULE_ALIAS("platform:max7370-pwm");
