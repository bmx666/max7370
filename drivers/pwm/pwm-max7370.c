/*
 * Copyright (C) 2017 Adakta Ltd
 *
 * Author: Maxim Paymushkin <maxim.paymushkin@gmail.com>
 *
 * License Terms: GNU General Public License, version 2
 *
 * MAX7370 PWM driver
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/mfd/max7370.h>

struct max7370_pwm {
	struct pwm_chip chip;
	struct max7370 *max7370;
	struct device *dev;
};

static inline struct max7370_pwm *to_max7370_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct max7370_pwm, chip);
}

static int max7370_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	int ret = 0;

	/* TODO */

	return ret;
}

static void max7370_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);

	/* TODO */
}

static int max7370_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      int duty_ns, int period_ns)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	int ret = 0;

	/* TODO */

	return ret;
}

static int max7370_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);
	int ret = 0;

	/* TODO */

	return ret;
}

static void max7370_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct max7370_pwm *max7370_pwm = to_max7370_pwm(chip);

	/* TODO */
}

static const struct pwm_ops max7370_pwm_ops = {
	.enable = max7370_pwm_enable,
	.disable = max7370_pwm_disable,
	.config = max7370_pwm_config,
	.request = max7370_pwm_request,
	.free = max7370_pwm_free,
	.owner = THIS_MODULE,
};

static int max7370_pwm_probe(struct platform_device *pdev)
{
	struct max7370 *max7370 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np = pdev->dev.of_node;
	struct max7370_pwm *max7370_pwm;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "No Device Tree node found\n");
		return -EINVAL;
	}

	max7370_pwm = devm_kzalloc(&pdev->dev, sizeof(struct max7370_pwm),
				    GFP_KERNEL);
	if (!max7370_pwm)
		return -ENOMEM;

	/* TODO: */

	max7370_pwm->chip.ops = &max7370_pwm_ops;
	/* add an extra channel for ALL_LED */
	max7370_pwm->chip.npwm = MAX7370_MAX_PWM;

	max7370_pwm->chip.dev = &pdev->dev;
	max7370_pwm->chip.base = -1;
	max7370_pwm->chip.can_sleep = true;

	ret = pwmchip_add(&max7370_pwm->chip);
	if (ret) {
		dev_err(&pdev->dev, "unable to add pwm chip: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, max7370_pwm);

	return 0;
}

static int max7370_pwm_remove(struct platform_device *pdev)
{
	struct max7370_pwm *max7370_pwm = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&max7370_pwm->chip);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_PM
static int max7370_pwm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_pwm *max7370_pwm = platform_get_drvdata(pdev);

	/* TODO: */

	return 0;
}

static int max7370_pwm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_pwm *max7370_pwm = platform_get_drvdata(pdev);

	/* TODO: */

	return 0;
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
