/*
 * Copyright (C) 2017 Adakta Ltd
 *
 * Author: Maxim Paymushkin <maxim.paymushkin@gmail.com>
 *
 * License Terms: GNU General Public License, version 2
 *
 * MAX7370 MFD core driver
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max7370.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>

/**
 * max7370_reg_read() - read a single MAX7370 register
 * @max7370:	Device to read from
 * @reg:		Register to read
 */
int max7370_reg_read(struct max7370 *max7370, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(max7370->i2c, reg);

	if (ret < 0)
		dev_err(max7370->dev, "failed to read reg %#x: %d\n",
			reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(max7370_reg_read);

/**
 * max7370_reg_write() - write a single MAX7370 register
 * @max7370:	Device to write to
 * @reg:		Register to read
 * @data:		Value to write
 */
int max7370_reg_write(struct max7370 *max7370, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(max7370->i2c, reg, data);
	if (ret < 0)
		dev_err(max7370->dev, "failed to write reg %#x: %d\n",
			reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(max7370_reg_write);

/**
 * max7370_set_bits() - set the value of a bitfield in a MAX7370 register
 * @max7370:	Device to write to
 * @reg:		Register to write
 * @mask:		Mask of bits to set
 * @val:		Value to set
 */
int max7370_set_bits(struct max7370 *max7370, u8 reg, u8 mask, u8 val)
{
	int ret;

	mutex_lock(&max7370->lock);

	ret = max7370_reg_read(max7370, reg);
	if (ret < 0)
		goto out;

	ret &= ~mask;
	ret |= val;

	ret = max7370_reg_write(max7370, reg, ret);

out:
	mutex_unlock(&max7370->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(max7370_set_bits);

static struct resource gpio_resources[] = {
	{
		.name	= MAX7370_INT_GPIIRQ_NAME,
		.start	= MAX7370_INT_GPIIRQ,
		.end	= MAX7370_INT_GPIIRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource keypad_resources[] = {
	{
		.name	= MAX7370_INT_KBDIRQ_NAME,
		.start  = MAX7370_INT_KBDIRQ,
		.end    = MAX7370_INT_KBDIRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell max7370_dev_gpio[] = {
	{
		.name			= "max7370-gpio",
		.num_resources	= ARRAY_SIZE(gpio_resources),
		.resources		= &gpio_resources[0],
		.of_compatible	= "maxim,max7370-gpio",
	},
};

static const struct mfd_cell max7370_dev_keypad[] = {
	{
		.name           = "max7370-keypad",
		.num_resources  = ARRAY_SIZE(keypad_resources),
		.resources      = &keypad_resources[0],
		.of_compatible	= "maxim,max7370-keypad",
	},
};

static const struct mfd_cell max7370_dev_pwm[] = {
	{
		.name           = "max7370-pwm",
		.of_compatible	= "maxim,max7370-pwm",
	},
};

static irqreturn_t max7370_irq(int irq, void *data)
{
	struct max7370 *max7370 = data;
	unsigned int blocks = max7370->pdata->block;
	int virq;

	dev_dbg(max7370->dev, "max7370_irq\n");

	if (blocks & MAX7370_BLOCK_GPIO) {
		virq = irq_find_mapping(max7370->domain, MAX7370_INT_GPIIRQ);
		handle_nested_irq(virq);
	}

	if (blocks & MAX7370_BLOCK_KEYPAD) {
		virq = irq_find_mapping(max7370->domain, MAX7370_INT_KBDIRQ);
		handle_nested_irq(virq);
	}

	return IRQ_HANDLED;
}

static int max7370_irq_map(struct irq_domain *d, unsigned int virq,
				irq_hw_number_t hwirq)
{
	struct max7370 *max7370 = d->host_data;

	irq_set_chip_data(virq, max7370);
	irq_set_chip_and_handler(virq, &dummy_irq_chip,
				handle_level_irq);
	irq_set_nested_thread(virq, 1);
	irq_set_noprobe(virq);

	return 0;
}

static void max7370_irq_unmap(struct irq_domain *d, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static const struct irq_domain_ops max7370_irq_ops = {
	.map    = max7370_irq_map,
	.unmap  = max7370_irq_unmap,
	.xlate  = irq_domain_xlate_onecell,
};

static int max7370_irq_init(struct max7370 *max7370, struct device_node *np)
{
	unsigned int blocks = max7370->pdata->block;
	int ret;

	max7370->domain = irq_domain_add_simple(
		np, MAX7370_NR_INTERNAL_IRQS, 0,
		&max7370_irq_ops, max7370);

	if (!max7370->domain) {
		dev_err(max7370->dev, "Failed to create irqdomain\n");
		return -ENOSYS;
	}

	if (blocks & MAX7370_BLOCK_GPIO) {
		ret = irq_create_mapping(max7370->domain, MAX7370_INT_GPIIRQ);
		if (!ret) {
			dev_err(max7370->dev, "Failed to map GPIO IRQ\n");
			return -EINVAL;
		}
	}

	if (blocks & MAX7370_BLOCK_KEYPAD) {
		ret = irq_create_mapping(max7370->domain, MAX7370_INT_KBDIRQ);
		if (!ret) {
			dev_err(max7370->dev, "Failed to map KEYPAD IRQ\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int max7370_device_prepare(struct max7370 *max7370)
{
	int ret = 0;

	/* reset GPIO */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_RESET,
			0x00 | MAX7370_GPIOCFG_RESET);
	if (ret < 0)
		return ret;

	/* wait reset */
	msleep(10);

	/* disable keypad */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_KBDSIZE,
			0x00);
	if (ret < 0)
		return ret;

	/* disable GPIO */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_ENABLE,
			0x00);
	if (ret < 0)
		return ret;

	return 0;
}

static int max7370_device_init(struct max7370 *max7370)
{
	int ret = 0;
	unsigned int blocks = max7370->pdata->block;

	ret = max7370_device_prepare(max7370);
	if (ret) {
		dev_err(max7370->dev, "failed to prepare device\n");
		return ret;
	}

	if (blocks & MAX7370_BLOCK_GPIO) {
		ret = mfd_add_devices(max7370->dev, -1,
					max7370_dev_gpio, ARRAY_SIZE(max7370_dev_gpio),
					NULL, 0, max7370->domain);
		if (ret) {
			dev_err(max7370->dev, "failed to add gpio child\n");
			return ret;
		}
		dev_info(max7370->dev, "added gpio block\n");
	}

	if (blocks & MAX7370_BLOCK_KEYPAD) {
		ret = mfd_add_devices(max7370->dev, -1,
					max7370_dev_keypad, ARRAY_SIZE(max7370_dev_keypad),
					NULL, 0, max7370->domain);
		if (ret) {
			dev_err(max7370->dev, "failed to add keypad child\n");
			return ret;
		}
		dev_info(max7370->dev, "added keypad block\n");
	}

	if (blocks & MAX7370_BLOCK_PWM) {
		ret = mfd_add_devices(max7370->dev, -1,
					max7370_dev_pwm, ARRAY_SIZE(max7370_dev_pwm),
				    NULL, 0, max7370->domain);
		if (ret) {
			dev_err(max7370->dev, "failed to add pwm child\n");
			return ret;
		}
		dev_info(max7370->dev, "added pwm block\n");
	}

	return ret;
}

static const struct of_device_id max7370_match[] = {
	/* Legacy compatible string */
	{ .compatible = "maxim,max7370", .data = (void *) MAX7370 },
	{ }
};

MODULE_DEVICE_TABLE(of, max7370_match);

static struct max7370_platform_data *
max7370_of_probe(struct device *dev, enum max7370_version *version)
{
	struct device_node *np = dev->of_node;
	struct max7370_platform_data *pdata;
	struct device_node *child;
	const struct of_device_id *of_id;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	of_id = of_match_device(max7370_match, dev);
	if (!of_id)
		return ERR_PTR(-ENODEV);
	*version = (enum max7370_version) of_id->data;

	for_each_child_of_node(np, child) {
		if (of_device_is_compatible(child, "maxim,max7370-gpio"))
			pdata->block |= MAX7370_BLOCK_GPIO;
		if (of_device_is_compatible(child, "maxim,max7370-keypad"))
			pdata->block |= MAX7370_BLOCK_KEYPAD;
		if (of_device_is_compatible(child, "maxim,max7370-pwm"))
			pdata->block |= MAX7370_BLOCK_PWM;
	}

	return pdata;
}

static int max7370_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	struct device_node *np = i2c->dev.of_node;
	struct max7370_platform_data *pdata = dev_get_platdata(&i2c->dev);
	struct max7370 *max7370;
	enum max7370_version version;
	int ret;

	if (!pdata) {
		pdata = max7370_of_probe(&i2c->dev, &version);
		if (IS_ERR(pdata)) {
			dev_err(&i2c->dev, "No platform data or DT found\n");
			return PTR_ERR(pdata);
		}
	} else {
		/* When not probing from device tree we have this ID */
		version = id->driver_data;
	}

	if (!i2c_check_functionality(i2c->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&i2c->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	max7370 = devm_kzalloc(&i2c->dev, sizeof(struct max7370),
			      GFP_KERNEL);
	if (!max7370)
		return -ENOMEM;

	mutex_init(&max7370->lock);

	max7370->dev = &i2c->dev;
	max7370->i2c = i2c;
	max7370->pdata = pdata;

	switch (version) {
		case MAX7370:
		default:
			max7370->num_gpio = MAX7370_MAX_GPIO;
			break;
	}

	i2c_set_clientdata(i2c, max7370);

	ret = max7370_irq_init(max7370, np);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(max7370->dev,
				   max7370->i2c->irq, NULL, max7370_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "max7370", max7370);
	if (ret) {
		dev_err(max7370->dev, "failed to request IRQ: %d\n", ret);
		return ret;
	}

	ret = max7370_device_init(max7370);
	if (ret) {
		dev_err(max7370->dev, "failed to add child devices\n");
		return ret;
	}

	return 0;
}

static int max7370_remove(struct i2c_client *i2c)
{
	struct max7370 *max7370 = i2c_get_clientdata(i2c);
	int virq;

	if (max7370->domain) {
		virq = irq_find_mapping(max7370->domain, MAX7370_INT_GPIIRQ);
		if (virq > 0)
			irq_dispose_mapping(virq);

		virq = irq_find_mapping(max7370->domain, MAX7370_INT_KBDIRQ);
		if (virq > 0)
			irq_dispose_mapping(virq);

		irq_domain_remove(max7370->domain);
	}

	mfd_remove_devices(max7370->dev);

	return 0;
}

#ifdef CONFIG_PM
static int max7370_suspend(struct device *dev)
{
	struct max7370 *max7370 = dev_get_drvdata(dev);
	struct i2c_client *i2c = max7370->i2c;

	disable_irq(i2c->irq);

	if (device_may_wakeup(&i2c->dev))
		enable_irq_wake(i2c->irq);

	return 0;
}

static int max7370_resume(struct device *dev)
{
	struct max7370 *max7370 = dev_get_drvdata(dev);
	struct i2c_client *i2c = max7370->i2c;

	if (device_may_wakeup(&i2c->dev))
		disable_irq_wake(i2c->irq);

	enable_irq(i2c->irq);

	return 0;
}

static const struct dev_pm_ops max7370_dev_pm_ops = {
	.suspend = max7370_suspend,
	.resume  = max7370_resume,
};
#endif

static const struct i2c_device_id max7370_id[] = {
	{ "max7370", MAX7370 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max7370_id);

static struct i2c_driver max7370_driver = {
	.driver = {
		.name	= "max7370",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max7370_match),
#ifdef CONFIG_PM
		.pm   = &max7370_dev_pm_ops,
#endif
	},
	.probe		= max7370_probe,
	.remove		= max7370_remove,
	.id_table	= max7370_id,
};
module_i2c_driver(max7370_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX7370 MFD core driver");
MODULE_AUTHOR("Maxim Paymushkin");
