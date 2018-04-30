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

#define MAX7370_GPIOCFG_RESET_SHIFT			3
#define MAX7370_GPIOCFG_ENABLE_SHIFT		4
#define MAX7370_GPIOCFG_I2C_TIMEOUT_SHIFT	5

/*
 * These registers are modified under the irq bus lock and cached to avoid
 * unnecessary writes in bus_sync_unlock.
 */
enum { REG_IBE, REG_IEV, REG_IS, REG_IE };

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
	u8						cfg;
	unsigned long           irqtype;
};

struct max7370_gpio {
	struct gpio_chip chip;
	struct max7370 *max7370;
	const struct max7370_gpio_platform_data *board;
	struct device *dev;
	struct mutex irq_lock;
	u8     active_gpios;
	/* Caches of interrupt control registers for bus_lock */
	u8 regs[CACHE_NR_REGS][CACHE_NR_BANKS];
	u8 oldregs[CACHE_NR_REGS][CACHE_NR_BANKS];
};

static int max7370_gpio_enable(struct max7370_gpio *max7370_gpio);
static int max7370_gpio_disable(struct max7370_gpio *max7370_gpio);

static inline struct max7370_gpio *to_max7370_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct max7370_gpio, chip);
}

static int max7370_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);
	struct max7370 *max7370 = max7370_gpio->max7370;
	u8 reg = MAX7370_REG_GPIOVALBASE + (offset / 8);
	u8 mask = BIT(offset % 8);
	int ret;

	dev_dbg(max7370_gpio->dev,
				"max7370_gpio_get, reg = %#x, offset = %u\n",
				reg, offset);

	ret = max7370_reg_read(max7370, reg);
	if (ret < 0)
		return ret;

	dev_dbg(max7370_gpio->dev, "max7370_gpio_get, val = %#x\n", ret);

	return !!(ret & mask);
}

static void max7370_gpio_set(struct gpio_chip *chip, unsigned int offset, int val)
{
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);
	struct max7370 *max7370 = max7370_gpio->max7370;
	u8 reg = MAX7370_REG_GPIOVALBASE + (offset / 8);
	unsigned int pos = offset % 8;
	u8 data = val ? BIT(pos) : 0;

	dev_dbg(max7370_gpio->dev,
				"max7370_gpio_set, reg = %#x, offset = %u, val = %d\n",
				reg, offset, val);

	max7370_set_bits(max7370, reg, BIT(pos), data);
}

static int max7370_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int offset, int val)
{
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);
	struct max7370 *max7370 = max7370_gpio->max7370;
	u8 reg = MAX7370_REG_GPIODIRBASE + (offset / 8);
	unsigned int pos = offset % 8;

	dev_dbg(max7370_gpio->dev,
				"max7370_gpio_direction_output, reg = %#x, offset = %u, val = %d\n",
				reg, offset, val);

	max7370_gpio_set(chip, offset, val);

	return max7370_set_bits(max7370, reg, BIT(pos), BIT(pos));
}

static int max7370_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);
	struct max7370 *max7370 = max7370_gpio->max7370;
	u8 reg = MAX7370_REG_GPIODIRBASE + (offset / 8);
	unsigned int pos = offset % 8;

	dev_dbg(max7370_gpio->dev,
				"max7370_gpio_direction_input, reg = %#x, offset = %u\n",
				reg, offset);

	return max7370_set_bits(max7370, reg, BIT(pos), 0);
}

static int max7370_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);
	struct max7370 *max7370 = max7370_gpio->max7370;
	u8 reg = MAX7370_REG_GPIODIRBASE + (offset / 8);
	unsigned int pos = offset % 8;
	int ret;

	dev_dbg(max7370_gpio->dev,
				"max7370_gpio_get_direction, reg = %#x, offset = %u\n",
				reg, offset);

	ret = max7370_reg_read(max7370, reg);
	if (ret < 0)
		return ret;

	dev_dbg(max7370_gpio->dev, "max7370_gpio_get_direction, val = %#x\n", ret);

	return !(ret & BIT(pos));
}

static int max7370_gpio_request(struct gpio_chip *chip, unsigned offset) {
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);

	dev_dbg(max7370_gpio->dev, "max7370_gpio_request, val = %u\n", offset);

	if (max7370_gpio->active_gpios++ == 0)
		return max7370_gpio_enable(max7370_gpio);

	return 0;
}

static void max7370_gpio_free(struct gpio_chip *chip, unsigned offset) {
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);

	dev_dbg(max7370_gpio->dev, "max7370_gpio_free, val = %u\n", offset);

	if (--max7370_gpio->active_gpios == 0)
		max7370_gpio_disable(max7370_gpio);
}

#if 0
/* TODO */
static int max7370_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
				   unsigned long config)
{
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(chip);
	struct max7370 *max7370 = max7370_gpio->max7370;

	/*
	 * These registers are alterated at each second address
	 * ODM bit 0 = drive to GND or Hi-Z (open drain)
	 * ODM bit 1 = drive to VDD or Hi-Z (open source)
	 */
	u8 odmreg = TC3589x_GPIOODM0 + (offset / 8) * 2;
	u8 odereg = TC3589x_GPIOODE0 + (offset / 8) * 2;
	unsigned int pos = offset % 8;
	int ret;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		/* Set open drain mode */
		ret = max7370_set_bits(max7370, odmreg, BIT(pos), 0);
		if (ret)
			return ret;
		/* Enable open drain/source mode */
		return max7370_set_bits(max7370, odereg, BIT(pos), BIT(pos));
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		/* Set open source mode */
		ret = max7370_set_bits(max7370, odmreg, BIT(pos), BIT(pos));
		if (ret)
			return ret;
		/* Enable open drain/source mode */
		return max7370_set_bits(max7370, odereg, BIT(pos), BIT(pos));
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		/* Disable open drain/source mode */
		return max7370_set_bits(max7370, odereg, BIT(pos), 0);
	default:
		break;
	}
	return -ENOTSUPP;
}
#endif

static const struct gpio_chip template_chip = {
	.label				= "max7370",
	.owner				= THIS_MODULE,
	.get				= max7370_gpio_get,
	.set				= max7370_gpio_set,
	.direction_output	= max7370_gpio_direction_output,
	.direction_input	= max7370_gpio_direction_input,
	.get_direction		= max7370_gpio_get_direction,
	.request			= max7370_gpio_request,
	.free				= max7370_gpio_free,
#if 0
	.set_config			= max7370_gpio_set_config,
#endif
	.can_sleep			= true,
};

static int max7370_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(gc);

	dev_dbg(max7370_gpio->dev, "max7370_gpio_irq_set_type, type = %u\n", type);
	/* TODO */

#if 0
	int offset = d->hwirq;
	int regoffset = offset / 8;
	int mask = BIT(offset % 8);

	if (type == IRQ_TYPE_EDGE_BOTH) {
		max7370_gpio->regs[REG_IBE][regoffset] |= mask;
		return 0;
	}

	max7370_gpio->regs[REG_IBE][regoffset] &= ~mask;

	if (type == IRQ_TYPE_LEVEL_LOW || type == IRQ_TYPE_LEVEL_HIGH)
		max7370_gpio->regs[REG_IS][regoffset] |= mask;
	else
		max7370_gpio->regs[REG_IS][regoffset] &= ~mask;

	if (type == IRQ_TYPE_EDGE_RISING || type == IRQ_TYPE_LEVEL_HIGH)
		max7370_gpio->regs[REG_IEV][regoffset] |= mask;
	else
		max7370_gpio->regs[REG_IEV][regoffset] &= ~mask;
#endif

	return 0;
}

static void max7370_gpio_irq_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(gc);

	dev_dbg(max7370_gpio->dev, "max7370_gpio_irq_lock\n");

	mutex_lock(&max7370_gpio->irq_lock);
}

static void max7370_gpio_irq_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(gc);
	struct max7370 *max7370 = max7370_gpio->max7370;

	dev_dbg(max7370_gpio->dev, "max7370_gpio_irq_sync_unlock\n");
	/* TODO */

#if 0
	static const u8 regmap[] = {
		[REG_IBE]	= TC3589x_GPIOIBE0,
		[REG_IEV]	= TC3589x_GPIOIEV0,
		[REG_IS]	= TC3589x_GPIOIS0,
		[REG_IE]	= TC3589x_GPIOIE0,
	};
	int i, j;

	for (i = 0; i < CACHE_NR_REGS; i++) {
		for (j = 0; j < CACHE_NR_BANKS; j++) {
			u8 old = max7370_gpio->oldregs[i][j];
			u8 new = max7370_gpio->regs[i][j];

			if (new == old)
				continue;

			max7370_gpio->oldregs[i][j] = new;
			max7370_reg_write(max7370, regmap[i] + j * 8, new);
		}
	}
#endif

	mutex_unlock(&max7370_gpio->irq_lock);
}

static void max7370_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(gc);
	int offset = d->hwirq;
	int regoffset = offset / 8;
	int mask = BIT(offset % 8);

	max7370_gpio->regs[REG_IE][regoffset] &= ~mask;
}

static void max7370_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max7370_gpio *max7370_gpio = to_max7370_gpio(gc);
	int offset = d->hwirq;
	int regoffset = offset / 8;
	int mask = BIT(offset % 8);

	max7370_gpio->regs[REG_IE][regoffset] |= mask;
}

static struct irq_chip max7370_gpio_irq_chip = {
	.name					= "max7370-gpio",
	.irq_bus_lock			= max7370_gpio_irq_lock,
	.irq_bus_sync_unlock	= max7370_gpio_irq_sync_unlock,
	.irq_mask				= max7370_gpio_irq_mask,
	.irq_unmask				= max7370_gpio_irq_unmask,
	.irq_set_type			= max7370_gpio_irq_set_type,
};

static irqreturn_t max7370_gpio_irq(int irq, void *dev)
{
	struct max7370_gpio *max7370_gpio = dev;
	struct max7370 *max7370 = max7370_gpio->max7370;
	u8 status[CACHE_NR_BANKS];
	int ret;
	int i;

	dev_dbg(max7370_gpio->dev, "max7370_gpio_irq\n");
	/* TODO */

#if 0
	ret = max7370_block_read(max7370, TC3589x_GPIOMIS0,
				 ARRAY_SIZE(status), status);
	if (ret < 0)
		return IRQ_NONE;

	for (i = 0; i < ARRAY_SIZE(status); i++) {
		unsigned int stat = status[i];
		if (!stat)
			continue;

		while (stat) {
			int bit = __ffs(stat);
			int line = i * 8 + bit;
			int irq = irq_find_mapping(max7370_gpio->chip.irqdomain,
						   line);

			handle_nested_irq(irq);
			stat &= ~(1 << bit);
		}

		max7370_reg_write(max7370, TC3589x_GPIOIC0 + i, status[i]);
	}
#endif

	return IRQ_HANDLED;
}

static int max7370_gpio_enable(struct max7370_gpio *max7370_gpio)
{
	const struct max7370_gpio_platform_data *board = max7370_gpio->board;
	struct max7370 *max7370 = max7370_gpio->max7370;
	int ret;

	/* enable gpios */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_GPIOCFG,
			board->cfg);
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

	return ret;
}

static int max7370_gpio_disable(struct max7370_gpio *max7370_gpio)
{
	struct max7370 *max7370 = max7370_gpio->max7370;
	int ret;

	/* disable gpios */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_GPIOCFG,
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

	dev_info(dev, "int mask1 = %#x\n", plat->int_mask1);
	dev_info(dev, "int mask2 = %#x\n", plat->int_mask2);
	dev_info(dev, "trigger mode1 = %#x\n", plat->trigger_mode1);
	dev_info(dev, "trigger mode2 = %#x\n", plat->trigger_mode2);
	dev_info(dev, "volt1 = %#x\n", plat->volt1);
	dev_info(dev, "volt2 = %#x\n", plat->volt2);

	plat->cfg = 0;

	/* GPIO enable */
	plat->cfg |= 1 << MAX7370_GPIOCFG_ENABLE_SHIFT;

	/* I2C timeout interrupt enable */
	plat->cfg |= (of_property_read_bool(np, "maxim,i2c-timeout-interrupt") & 1)
					<< MAX7370_GPIOCFG_I2C_TIMEOUT_SHIFT;

	dev_info(dev, "reg cfg = %#x\n", plat->cfg);

	/* FIXME: should be property of the IRQ resource? */
	plat->irqtype = IRQF_TRIGGER_LOW;

	return plat;
}

static int max7370_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct max7370 *max7370 = dev_get_drvdata(pdev->dev.parent);
	struct max7370_gpio *max7370_gpio;
	const struct max7370_gpio_platform_data *plat;
	int ret;
	int irq;

	plat = max7370_gpio_of_probe(&pdev->dev);
	if (IS_ERR(plat)) {
		dev_err(&pdev->dev, "invalid keypad platform data\n");
		return PTR_ERR(plat);
	}

	irq = platform_get_irq_byname(pdev, MAX7370_INT_GPIIRQ_NAME);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return irq;
	}

	max7370_gpio = devm_kzalloc(&pdev->dev, sizeof(struct max7370_gpio),
				    GFP_KERNEL);
	if (!max7370_gpio)
		return -ENOMEM;

	max7370_gpio->active_gpios = 0;
	max7370_gpio->board = plat;
	max7370_gpio->dev = &pdev->dev;
	max7370_gpio->max7370 = max7370;

	ret = max7370_gpio_disable(max7370_gpio);
	if (ret) {
		dev_err(&pdev->dev, "Could not disable gpios\n");
		return ret;
	}

	mutex_init(&max7370_gpio->irq_lock);

	max7370_gpio->chip = template_chip;
	max7370_gpio->chip.ngpio = max7370->num_gpio;
	max7370_gpio->chip.parent = &pdev->dev;
	max7370_gpio->chip.base = -1;
	max7370_gpio->chip.of_node = np;

	ret = devm_request_threaded_irq(&pdev->dev,
					irq, NULL, max7370_gpio_irq,
					IRQF_ONESHOT, "max7370-gpio",
					max7370_gpio);
	if (ret) {
		dev_err(&pdev->dev, "unable to get irq: %d\n", ret);
		return ret;
	}

	ret = gpiochip_add(&max7370_gpio->chip);
	if (ret) {
		dev_err(&pdev->dev, "unable to add gpiochip: %d\n", ret);
		return ret;
	}

	ret =  gpiochip_irqchip_add(&max7370_gpio->chip,
					&max7370_gpio_irq_chip,
					0,
					handle_simple_irq,
					IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&pdev->dev,
			"could not connect irqchip to gpiochip\n");
		return ret;
	}

	gpiochip_set_chained_irqchip(&max7370_gpio->chip,
				    &max7370_gpio_irq_chip,
				    irq,
				    NULL);

	platform_set_drvdata(pdev, max7370_gpio);

	return 0;
}


static int max7370_gpio_remove(struct platform_device *pdev)
{
	struct max7370_gpio *max7370_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&max7370_gpio->chip);

	return 0;
}

#ifdef CONFIG_PM
static int max7370_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_gpio *max7370_gpio = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, MAX7370_INT_GPIIRQ_NAME);

	/* if device is not a wakeup source, disable it for powersave */
	if (!device_may_wakeup(&pdev->dev))
		max7370_gpio_disable(max7370_gpio);
	else
		enable_irq_wake(irq);

	return 0;
}

static int max7370_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_gpio *max7370_gpio = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, MAX7370_INT_GPIIRQ_NAME);

	/* enable the device to resume normal operations */
	if (!device_may_wakeup(&pdev->dev))
		max7370_gpio_enable(max7370_gpio);
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
