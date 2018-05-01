/*
 * Copyright (C) 2017 Adakta Ltd
 *
 * Author: Maxim Paymushkin <maxim.paymushkin@gmail.com>
 *
 * License Terms: GNU General Public License, version 2
 *
 * MAX7370 GPIO driver
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/mfd/max7370.h>

#define MAX7370_MIN_DEBOUNCE 9
#define MAX7370_MAX_DEBOUNCE 40

#define MAX7370_GPIO_EDGE_NONE		0
#define MAX7370_GPIO_EDGE_RISING	1
#define MAX7370_GPIO_EDGE_BOTH		2

#define CACHE_NR_REGS	2
#define CACHE_NR_BANKS	8

/**
 * struct max7370_gpio_platform_data - platform specific gpio data
 * @debounce_period:    platform specific debounce time
 * @irqtype:            type of interrupt, falling or rising edge
 */
struct max7370_gpio_platform_data {
	u8                      debounce_period;
	u8						int_mask1;
	u8						int_mask2;
	u8						trigger_mode1;
	u8						trigger_mode2;
	u8						volt1;
	u8						volt2;
	unsigned long           irqtype;
};

struct max7370_gpio {
	struct gpio_chip    chip;
	struct max7370     *max7370;
	const struct max7370_gpio_platform_data *board;
	struct device      *dev;
	struct mutex        irq_lock;
	u8                  active_gpios;
	unsigned		    status;
	unsigned int        irq_parent;
	unsigned            irq_enabled;
};

static int max7370_gpio_enable(struct max7370_gpio *max7370_gpio);
static int max7370_gpio_disable(struct max7370_gpio *max7370_gpio);

static inline struct max7370_gpio *to_max7370_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct max7370_gpio, chip);
}

static int max7370_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);
	struct max7370 *max7370 = gpio->max7370;
	u8 reg = MAX7370_REG_GPIOVALBASE + (offset / 8);
	u8 mask = BIT(offset % 8);
	int ret;

	ret = max7370_reg_read(max7370, reg);
	if (ret < 0)
		return ret;

	return !!(ret & mask);
}

static void max7370_gpio_set(struct gpio_chip *chip, unsigned int offset, int val)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);
	struct max7370 *max7370 = gpio->max7370;
	u8 reg = MAX7370_REG_GPIOVALBASE + (offset / 8);
	unsigned int pos = offset % 8;
	u8 data = val ? BIT(pos) : 0;

	max7370_set_bits(max7370, reg, BIT(pos), data);
}

static int max7370_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int val)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);
	struct max7370 *max7370 = gpio->max7370;
	u8 reg = MAX7370_REG_GPIODIRBASE + (offset / 8);
	unsigned int pos = offset % 8;

	max7370_gpio_set(chip, offset, val);

	return max7370_set_bits(max7370, reg, BIT(pos), BIT(pos));
}

static int max7370_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);
	struct max7370 *max7370 = gpio->max7370;
	u8 reg = MAX7370_REG_GPIODIRBASE + (offset / 8);
	unsigned int pos = offset % 8;

	return max7370_set_bits(max7370, reg, BIT(pos), 0);
}

static int max7370_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);
	struct max7370 *max7370 = gpio->max7370;
	u8 reg = MAX7370_REG_GPIODIRBASE + (offset / 8);
	unsigned int pos = offset % 8;
	int ret;

	ret = max7370_reg_read(max7370, reg);
	if (ret < 0)
		return ret;

	return !(ret & BIT(pos));
}

static int max7370_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);

	dev_dbg(gpio->dev, "max7370_gpio_request, offset = %u\n", offset);

	if (gpio->active_gpios++ == 0)
		return max7370_gpio_enable(gpio);

	return 0;
}

static void max7370_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);

	dev_dbg(gpio->dev, "max7370_gpio_free, offset = %u\n", offset);

	if (--gpio->active_gpios == 0)
		max7370_gpio_disable(gpio);
}

static int max7370_gpio_set_single_ended(struct gpio_chip *chip,
	unsigned int offset, enum single_ended_mode mode)
{
	struct max7370_gpio *gpio = gpiochip_get_data(chip);

	dev_dbg(gpio->dev, "max7370_gpio_set_single_ended, "
			"offset = %u, mode = %d\n", offset, mode);

	/* TODO */

	return -ENOTSUPP;
}

static const struct gpio_chip template_chip = {
	.label				= "max7370",
	.owner				= THIS_MODULE,
	.get				= max7370_gpio_get,
	.set				= max7370_gpio_set,
	.direction_output	= max7370_gpio_direction_output,
	.direction_input	= max7370_gpio_direction_input,
	.get_direction		= max7370_gpio_get_direction,
	// .set_single_ended	= max7370_gpio_set_single_ended,
	.request			= max7370_gpio_request,
	.free				= max7370_gpio_free,
	.can_sleep			= true,
};

/*
 * NOP functions
 */
static void noop(struct irq_data *data) { }

static void max7370_gpio_irq_ack(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	dev_info(gpio->dev, "max7370_gpio_irq_ack: irq = %lu\n", data->hwirq);
}

static void max7370_gpio_irq_mask(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	dev_info(gpio->dev, "max7370_gpio_irq_mask: irq = %lu\n", data->hwirq);
}

static void max7370_gpio_irq_unmask(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	dev_info(gpio->dev, "max7370_gpio_irq_unmask: irq = %lu\n", data->hwirq);
}

static int max7370_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	dev_info(gpio->dev, "max7370_gpio_irq_set_type: irq = %lu, type = %u\n",
				data->hwirq, type);

	return 0;
}

static int max7370_gpio_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	int error = 0;

	dev_info(gpio->dev, "max7370_gpio_irq_set_wake: irq = %d\n", gpio->irq_parent);

	if (gpio->irq_parent) {
		error = irq_set_irq_wake(gpio->irq_parent, on);
		if (error) {
			dev_dbg(gpio->dev,
				"irq %u doesn't support irq_set_wake\n",
				gpio->irq_parent);
			gpio->irq_parent = 0;
		}
	}
	return error;
}

static void max7370_gpio_irq_enable(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	gpio->irq_enabled |= (1 << data->hwirq);
}

static void max7370_gpio_irq_disable(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	gpio->irq_enabled &= ~(1 << data->hwirq);
}

static void max7370_gpio_irq_bus_lock(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	mutex_lock(&gpio->irq_lock);
}

static void max7370_gpio_irq_bus_sync_unlock(struct irq_data *data)
{
	struct max7370_gpio *gpio = irq_data_get_irq_chip_data(data);

	mutex_unlock(&gpio->irq_lock);
}

static struct irq_chip max7370_gpio_irq_chip = {
	.name					= "max7370-gpio",
	.irq_enable				= max7370_gpio_irq_enable,
	.irq_disable			= max7370_gpio_irq_disable,
	// .irq_ack				= max7370_gpio_irq_ack,
	.irq_mask				= max7370_gpio_irq_mask,
	.irq_unmask				= max7370_gpio_irq_unmask,
	.irq_set_wake			= max7370_gpio_irq_set_wake,
	.irq_set_type			= max7370_gpio_irq_set_type,
	.irq_bus_lock			= max7370_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= max7370_gpio_irq_bus_sync_unlock,
};

static irqreturn_t max7370_gpio_irq(int irq, void *data)
{
	struct max7370_gpio *gpio = data;
	struct max7370 *max7370 = gpio->max7370;
	int row_status, col_status;
	unsigned long change, i, status;

	dev_info(gpio->dev, "max7370_gpio_irq\n");

	row_status = max7370_reg_read(max7370,
			MAX7370_REG_GPIOVALBASE);
	if (row_status < 0)
		return row_status;

	col_status = max7370_reg_read(max7370,
			MAX7370_REG_GPIOVALBASE + 1);
	if (row_status < 0)
		return row_status;

	status = (row_status & 0xff) |
			((col_status & 0xff) << 8);

	/*
	 * call the interrupt handler iff gpio is used as
	 * interrupt source, just to avoid bad irqs
	 */
	mutex_lock(&gpio->irq_lock);
	change = (gpio->status ^ status) & gpio->irq_enabled;
	gpio->status = status;
	mutex_unlock(&gpio->irq_lock);

	for_each_set_bit(i, &change, gpio->chip.ngpio)
		handle_nested_irq(irq_find_mapping(gpio->chip.irqdomain, i));

	return IRQ_HANDLED;
}

static int max7370_gpio_enable(struct max7370_gpio *gpio)
{
	const struct max7370_gpio_platform_data *board = gpio->board;
	struct max7370 *max7370 = gpio->max7370;
	int ret;

	/* enable GPIO */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_ENABLE,
			MAX7370_GPIOCFG_ENABLE);
	if (ret < 0)
		return ret;

	/* enable voltage 1 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_SUPPLY_VOLT1,
			board->volt1);
	if (ret < 0)
		return ret;

	/* enable voltage 2 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_SUPPLY_VOLT2,
			board->volt2);
	if (ret < 0)
		return ret;

	/* enable interrupts 1 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_INT_MASK1,
			board->int_mask1);
	if (ret < 0)
		return ret;

	/* enable interrupts 2 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_INT_MASK2,
			board->int_mask2);
	if (ret < 0)
		return ret;

	/* enable GPIO trigger mode 1 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_TRIG_MODE1,
			board->trigger_mode1);
	if (ret < 0)
		return ret;

	/* enable GPIO trigger mode 2 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_TRIG_MODE2,
			board->trigger_mode2);
	if (ret < 0)
		return ret;

	return ret;
}

static int max7370_gpio_disable(struct max7370_gpio *max7370_gpio)
{
	struct max7370 *max7370 = max7370_gpio->max7370;
	int ret;

	/* disable GPIO */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_GPIOCFG,
			MAX7370_GPIOCFG_ENABLE,
			0x00);
	if (ret < 0)
		return ret;

	/* disable interrupts 1 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_INT_MASK1,
			0x00);
	if (ret < 0)
		return ret;

	/* disable interrupts 2 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_INT_MASK2,
			0x00);
	if (ret < 0)
		return ret;

	/* disable supply voltage 1 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_SUPPLY_VOLT1,
			0x00);
	if (ret < 0)
		return ret;

	/* disable supply voltage 2 */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_SUPPLY_VOLT2,
			0x00);
	if (ret < 0)
		return ret;

	return ret;
}

static const struct max7370_gpio_platform_data *
max7370_gpio_of_probe(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct max7370_gpio_platform_data *plat;
	u32 val;
	int i;
	const char* strval;

	if (!np)
		return ERR_PTR(-ENODEV);

	plat = devm_kzalloc(dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

	/* The custom press delay format is ms */
	of_property_read_u32(np, "maxim,debounce-delay-ms", &val);
	if (val) {
		if (val < MAX7370_MIN_DEBOUNCE || val > MAX7370_MAX_DEBOUNCE) {
			dev_err(dev, "debounce must be between %d to %d\n",
				MAX7370_MIN_DEBOUNCE, MAX7370_MAX_DEBOUNCE);
			return ERR_PTR(-EINVAL);
		}
	} else {
		val = MAX7370_MIN_DEBOUNCE;
		dev_info(dev, "set default debounce %d ms\n",
				MAX7370_MIN_DEBOUNCE);
	}

	/* convert to registers values */
	val -= 9;
	plat->debounce_period = (u8) val;

	if (of_property_count_strings(np, "maxim,edges")
		!= CACHE_NR_REGS * CACHE_NR_BANKS) {
		dev_err(dev, "size of edges array must be equal to %d\n",
			CACHE_NR_REGS * CACHE_NR_BANKS);
		return ERR_PTR(-EINVAL);
	}

	plat->int_mask1 = ~0;
	plat->int_mask2 = ~0;
	plat->trigger_mode1 = 0;
	plat->trigger_mode2 = 0;
	plat->volt1 = 0;
	plat->volt2 = 0;

	for (i = 0; i < CACHE_NR_REGS * CACHE_NR_BANKS; ++i) {
		if (of_property_read_string_index(np, "maxim,edges", i, &strval)) {
			dev_err(dev, "invalid read edges element %d\n", i);
			return ERR_PTR(-EINVAL);
		}

		dev_info(dev, "edges element %d = %s\n", i, strval);

		if (memcmp(strval, "none", 4) == 0)
			val = MAX7370_GPIO_EDGE_NONE;
		else if (memcmp(strval, "rising", 6) == 0)
			val = MAX7370_GPIO_EDGE_RISING;
		else if (memcmp(strval, "both", 4) == 0) {
			val = MAX7370_GPIO_EDGE_BOTH;
			if (i < CACHE_NR_BANKS)
				plat->trigger_mode1 |= BIT(i);
			else
				plat->trigger_mode2 |= BIT(i - CACHE_NR_BANKS);
		}
		else {
			dev_err(dev, "invalid value of edges element %d, "
					"must be %s, %s, %s\n",
					i, "none", "rising", "both");
			return ERR_PTR(-EINVAL);
		}

		if (val != MAX7370_GPIO_EDGE_NONE) {
			if (i < CACHE_NR_BANKS) {
				plat->int_mask1 &= ~BIT(i);
				/* TODO */
				/* plat->volt1 |= BIT(i); */
			} else {
				plat->int_mask2 &= ~BIT(i - CACHE_NR_BANKS);
				/* TODO */
				/* plat->volt2 |= BIT(i - CACHE_NR_BANKS); */
			}
		}
	}

	/* FIXME: should be property of the IRQ resource? */
	plat->irqtype = IRQF_TRIGGER_FALLING;

	return plat;
}

static int max7370_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct max7370 *max7370 = dev_get_drvdata(pdev->dev.parent);
	struct max7370_gpio *gpio;
	const struct max7370_gpio_platform_data *plat;
	int ret;
	int irq;

	plat = max7370_gpio_of_probe(&pdev->dev);
	if (IS_ERR(plat)) {
		dev_err(&pdev->dev, "invalid gpio platform data\n");
		return PTR_ERR(plat);
	}

	irq = platform_get_irq_byname(pdev, MAX7370_INT_GPIIRQ_NAME);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return irq;
	}

	gpio = devm_kzalloc(&pdev->dev, sizeof(struct max7370_gpio),
				    GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->status = 0;
	gpio->active_gpios = 0;
	gpio->board = plat;
	gpio->dev = &pdev->dev;
	gpio->max7370 = max7370;

	ret = max7370_gpio_disable(gpio);
	if (ret) {
		dev_err(&pdev->dev, "Could not disable gpios\n");
		return ret;
	}

	mutex_init(&gpio->irq_lock);

	gpio->chip = template_chip;
	gpio->chip.ngpio = max7370->num_gpio;
	gpio->chip.parent = &pdev->dev;
	gpio->chip.base = -1;
	gpio->chip.of_node = np;

	ret = devm_gpiochip_add_data(&pdev->dev, &gpio->chip,
					gpio);
	if (ret) {
		dev_err(&pdev->dev, "unable to add gpiochip: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(&pdev->dev,
					irq, NULL, max7370_gpio_irq,
					plat->irqtype | IRQF_ONESHOT,
					"max7370-gpio", gpio);
	if (ret) {
		dev_err(&pdev->dev, "unable to get irq: %d\n", ret);
		return ret;
	}

	ret =  gpiochip_irqchip_add(&gpio->chip,
					&max7370_gpio_irq_chip,
					0,
					handle_simple_irq,
					IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&pdev->dev,
			"could not connect irqchip to gpiochip\n");
		return ret;
	}

	gpiochip_set_chained_irqchip(&gpio->chip,
				    &max7370_gpio_irq_chip,
				    irq,
				    NULL);

	gpio->irq_parent = irq;

	platform_set_drvdata(pdev, gpio);

	return 0;
}


static int max7370_gpio_remove(struct platform_device *pdev)
{
	struct max7370_gpio *gpio = platform_get_drvdata(pdev);

	return 0;
}

#ifdef CONFIG_PM
static int max7370_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_gpio *gpio = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, MAX7370_INT_GPIIRQ_NAME);

	/* if device is not a wakeup source, disable it for powersave */
	if (!device_may_wakeup(&pdev->dev))
		max7370_gpio_disable(gpio);
	else
		enable_irq_wake(irq);

	return 0;
}

static int max7370_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_gpio *gpio = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, MAX7370_INT_GPIIRQ_NAME);

	/* enable the device to resume normal operations */
	if (!device_may_wakeup(&pdev->dev))
		max7370_gpio_enable(gpio);
	else
		disable_irq_wake(irq);

	return 0;
}

static const struct dev_pm_ops max7370_gpio_dev_pm_ops = {
	.suspend = max7370_gpio_suspend,
	.resume  = max7370_gpio_resume,
};
#endif

static const struct of_device_id max7370_gpio_match[] = {
	/* Legacy compatible string */
	{ .compatible = "maxim,max7370-gpio", .data = (void *) MAX7370 },
	{ }
};
MODULE_DEVICE_TABLE(of, max7370_gpio_match);

static struct platform_driver max7370_gpio_driver = {
	.driver	= {
		.name			= "max7370-gpio",
		.owner			= THIS_MODULE,
		.of_match_table	= of_match_ptr(max7370_gpio_match),
#ifdef CONFIG_PM
		.pm				= &max7370_gpio_dev_pm_ops,
#endif
	},
	.probe			= max7370_gpio_probe,
	.remove 		= max7370_gpio_remove,
};
module_platform_driver(max7370_gpio_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Maxim Paymushkin");
MODULE_DESCRIPTION("MAX7370 GPIO Driver");
MODULE_ALIAS("platform:max7370-gpio");
