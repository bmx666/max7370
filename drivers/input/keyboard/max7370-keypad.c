/*
 * Copyright (C) 2017 Adakta Ltd
 *
 * Author: Maxim Paymushkin <maxim.paymushkin@gmail.com>
 *
 * License Terms: GNU General Public License, version 2
 *
 * MAX7370 MFD Keypad Controller driver
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mfd/max7370.h>

#define MAX7370_MAX_KEY_ROWS	8
#define MAX7370_MAX_KEY_COLS	8

/*
 * Autosleep register values (ms)
 */
#define MAX7370_AUTOSLEEP_DISABLE	0x00
#define MAX7370_AUTOSLEEP_8192		0x01
#define MAX7370_AUTOSLEEP_4096		0x02
#define MAX7370_AUTOSLEEP_2048		0x03
#define MAX7370_AUTOSLEEP_1024		0x04
#define MAX7370_AUTOSLEEP_512		0x05
#define MAX7370_AUTOSLEEP_256		0x06

/*
 * Minimum and maximum supported values
 */
#define MAX7370_MIN_KPDEBOUNCE				2
#define MAX7370_MAX_KPDEBOUNCE				32
#define MAX7370_MIN_INT_CYCLES				0
#define MAX7370_MAX_INT_CYCLES				31
#define MAX7370_MIN_INT_FIFO_EVENTS			0
#define MAX7370_MAX_INT_FIFO_EVENTS			14
#define MAX7370_MIN_AUTORPT_DELAY_CYCLES	8
#define MAX7370_MAX_AUTORPT_DELAY_CYCLES	128
#define MAX7370_MIN_AUTORPT_FREQ_CYCLES		4
#define MAX7370_MAX_AUTORPT_FREQ_CYCLES		32
#define MAX7370_MIN_AUTOSLEEP				256
#define MAX7370_MAX_AUTOSLEEP				8192

#define MAX7370_INT_FIFO_SHIFT	5

#define MAX7370_AUTOREPEAT_MASK					BIT(7)
#define MAX7370_AUTOREPEAT_DELAY_CYCLES_MASK	0x0f
#define MAX7370_AUTOREPEAT_FREQ_CYCLES_MASK		0x70
#define MAX7370_AUTOREPEAT_DELAY_CYCLES_SHIFT	0
#define MAX7370_AUTOREPEAT_FREQ_CYCLES_SHIFT	4

#define KP_RELEASE_EVT_MASK	0x40

#define KP_KEY_MASK			0x3F
#define KP_KEY_62_MASK		0xBF
#define KP_KEY_63_MASK		0xBE

#define MAX7370_FIFO_KEY_RELEASE_MASK	BIT(6)
#define MAX7370_FIFO_NOT_EMPTY_MASK		BIT(7)
#define MAX7370_FIFO_EMPTY				0x3F
#define MAX7370_FIFO_OVERFLOW			0x7F
#define MAX7370_KEYPAD_ROW_SHIFT		0x04

#define KP_DEBOUNCE_SHIFT	4

/**
 * struct max7370_keypad_platform_data - platform specific keypad data
 * @keymap_data:        matrix scan code table for keycodes
 * @krow:               mask for available rows, value is 0xFF
 * @kcol:               mask for available columns, value is 0xFF
 * @debounce_period:    platform specific debounce time
 * @autorepeat:      	platform specific autorepeat value
 * @irqtype:            type of interrupt, falling or rising edge
 * @enable_wakeup:      specifies if keypad event can wake up system from sleep
 */
struct max7370_keypad_platform_data {
	const struct matrix_keymap_data *keymap_data;
	u8                      krow;
	u8                      kcol;
	u8                      debounce_period;
	u8                      int_cycles;
	u8                      int_fifo_events;
	u8                      autorepeat;
	u8                      autosleep;
	unsigned long           irqtype;
	bool                    enable_wakeup;
};

/**
 * struct maxim_keypad - data structure used by keypad driver
 * @max7370:        pointer to max7370
 * @input:          pointer to input device object
 * @board:          keypad platform device
 * @krow:           number of rows
 * @kcol:           number of columns
 * @keymap:         matrix scan code table for keycodes
 * @keypad_stopped: holds keypad status
 */
struct max7370_keypad {
	struct max7370 *max7370;
	struct input_dev *input;
	const struct max7370_keypad_platform_data *board;
	unsigned int krow;
	unsigned int kcol;
	unsigned short *keymap;
	bool keypad_stopped;
};

static int max7370_keypad_init_key_hardware(struct max7370_keypad *keypad)
{
	const struct max7370_keypad_platform_data *board = keypad->board;
	struct max7370 *max7370 = keypad->max7370;
	int ret;

	dev_dbg(max7370->dev, "max7370_keypad_init_key_hardware\n");

	/* enable wakeup, enable key release, clear int on first read */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_CONFIG,
			MAX7370_CFG_KEY_RELEASE |
				MAX7370_CFG_INTERRUPT |
				MAX7370_CFG_WAKEUP);
	if (ret < 0)
		return ret;

	ret = max7370_reg_write(max7370,
			MAX7370_REG_DEBOUNCE,
			board->debounce_period);
	if (ret < 0)
		return ret;

	ret = max7370_reg_write(max7370,
			MAX7370_REG_INTERRUPT,
			board->int_cycles |
			(board->int_fifo_events << MAX7370_INT_FIFO_SHIFT));
	if (ret < 0)
		return ret;

	ret = max7370_reg_write(max7370,
			MAX7370_REG_AUTOREPEAT,
			board->autorepeat);
	if (ret < 0)
		return ret;

	/* disable Autosleep  */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_AUTOSLEEP,
			board->autosleep);
	if (ret < 0)
		return ret;

	return 0;
}

static irqreturn_t max7370_keypad_irq(int irq, void *dev)
{
	struct max7370_keypad *keypad = dev;
	struct max7370 *max7370 = keypad->max7370;
	u8 row_index, col_index, kbd_code, up;
	u8 code;
	int ret;

	dev_dbg(max7370->dev, "max7370_keypad_irq\n");

again:
	ret = max7370_reg_read(max7370, MAX7370_REG_KEYFIFO);
	if (ret < 0)
		return IRQ_NONE;

	kbd_code = (u8)ret;

	if (kbd_code == MAX7370_FIFO_EMPTY) {
		dev_dbg(max7370->dev, "fifo empty\n");
		return IRQ_HANDLED;
	}

	if (kbd_code == MAX7370_FIFO_OVERFLOW) {
		dev_dbg(max7370->dev, "fifo overflow\n");
		goto again;
	}

	if ((kbd_code & MAX7370_FIFO_NOT_EMPTY_MASK) != 0) {
		dev_dbg(max7370->dev, "fifo has empty mask\n");
		goto again;
	}

	up = kbd_code & KP_RELEASE_EVT_MASK;

	if ((kbd_code & KP_KEY_63_MASK) == KP_KEY_63_MASK)
		kbd_code = 63;
	else if ((kbd_code & KP_KEY_62_MASK) == KP_KEY_62_MASK)
		kbd_code = 62;
	else
		kbd_code &= KP_KEY_MASK;

	dev_dbg(max7370->dev, "kbd_code %d is %s\n", kbd_code,
			(up ? "up" : "down"));

	/* valid key is found */
	col_index = kbd_code / MAX7370_MAX_KEY_ROWS;
	row_index = kbd_code % MAX7370_MAX_KEY_ROWS;

	dev_dbg(max7370->dev, "col %d, row %d\n", col_index, row_index);

	/* MAX7370 has inverted row and col, see datasheet */
	code = row_index * MAX7370_MAX_KEY_ROWS + col_index;

	dev_dbg(max7370->dev, "code = %#x\n", code);
	dev_dbg(max7370->dev, "keymap = %#x\n", keypad->keymap[code]);

	input_event(keypad->input, EV_MSC, MSC_SCAN, code);
	input_report_key(keypad->input, keypad->keymap[code], !up);
	input_sync(keypad->input);

	return IRQ_HANDLED;
}

static int max7370_keypad_enable(struct max7370_keypad *keypad)
{
	const struct max7370_keypad_platform_data *board = keypad->board;
	struct max7370 *max7370 = keypad->max7370;
	int ret;

	dev_dbg(max7370->dev, "max7370_keypad_enable\n");

	/* validate platform configuration */
	if (board->kcol > MAX7370_MAX_KEY_COLS || board->krow > MAX7370_MAX_KEY_ROWS)
		return -EINVAL;

	/* configure keypad size */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_KBDSIZE,
			(board->krow << MAX7370_KEYPAD_ROW_SHIFT) | board->kcol);
	if (ret < 0)
		return ret;

	/* enable wakeup */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_CONFIG,
			MAX7370_CFG_WAKEUP,
			0x00 | MAX7370_CFG_WAKEUP);
	if (ret < 0)
		return ret;

	return 0;
}

static int max7370_keypad_disable(struct max7370_keypad *keypad)
{
	struct max7370 *max7370 = keypad->max7370;
	int ret = 0;

	dev_dbg(max7370->dev, "max7370_keypad_disable\n");

	/* disable keypad */
	ret = max7370_reg_write(max7370,
			MAX7370_REG_KBDSIZE,
			0x00);
	if (ret < 0)
		return ret;

	/* disable wakeup */
	ret = max7370_set_bits(max7370,
			MAX7370_REG_CONFIG,
			MAX7370_CFG_WAKEUP,
			0x00);
	if (ret < 0)
		return ret;

	return ret;
}

static int max7370_keypad_open(struct input_dev *input)
{
	struct max7370_keypad *keypad = input_get_drvdata(input);
	int error;

	/* enable the keypad module */
	error = max7370_keypad_enable(keypad);
	if (error < 0) {
		dev_err(&input->dev, "failed to enable keypad module\n");
		return error;
	}

	error = max7370_keypad_init_key_hardware(keypad);
	if (error < 0) {
		dev_err(&input->dev, "failed to configure keypad module\n");
		return error;
	}

	return 0;
}

static void max7370_keypad_close(struct input_dev *input)
{
	struct max7370_keypad *keypad = input_get_drvdata(input);
	int ret;

	/* disable the keypad module */
	ret = max7370_keypad_disable(keypad);
	if (ret)
		dev_err(&input->dev, "Could not disable keypad\n");
}

static const struct max7370_keypad_platform_data *
max7370_keypad_of_probe(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct max7370_keypad_platform_data *plat;
	u32 val;
	int proplen;

	if (!np)
		return ERR_PTR(-ENODEV);

	plat = devm_kzalloc(dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

	of_property_read_u32(np, "keypad,num-columns", &val);
	plat->kcol = (u8) val;
	of_property_read_u32(np, "keypad,num-rows", &val);
	plat->krow = (u8) val;
	if (!plat->krow || !plat->kcol ||
	     plat->krow > MAX7370_MAX_KEY_ROWS || plat->kcol > MAX7370_MAX_KEY_COLS) {
		dev_err(dev,
			"keypad columns/rows not properly specified (%ux%u)\n",
			plat->kcol, plat->krow);
		return ERR_PTR(-EINVAL);
	}

	if (!of_get_property(np, "linux,keymap", &proplen)) {
		dev_err(dev, "property linux,keymap not found\n");
		return ERR_PTR(-ENOENT);
	}

	plat->autorepeat = !of_property_read_bool(np, "linux,no-autorepeat");

	plat->enable_wakeup = of_property_read_bool(np, "wakeup-source") ||
			      /* legacy name */
			      of_property_read_bool(np, "linux,wakeup");

	/* The custom autosleep is ms */
	plat->autosleep = 0;
	if (of_get_property(np, "maxim,autosleep", &proplen)) {
		of_property_read_u32(np, "maxim,autosleep", &val);
		if (val) {
			if (val < MAX7370_MIN_AUTOSLEEP || val > MAX7370_MAX_AUTOSLEEP) {
				dev_err(dev, "autosleep must be between "
							"%d to %d and powered by 2\n",
					MAX7370_MIN_AUTOSLEEP, MAX7370_MAX_AUTOSLEEP);
				return ERR_PTR(-EINVAL);
			}

			val >>= 8;

			plat->autosleep = 7 - ffs(val);
		}
	}

	if (!plat->autosleep)
		dev_info(dev, "autosleep disabled\n");

	/* The custom press delay format is ms */
	of_property_read_u32(np, "maxim,debounce-delay-press-ms", &val);
	if (val) {
		if (val < MAX7370_MIN_KPDEBOUNCE || val > MAX7370_MAX_KPDEBOUNCE) {
			dev_err(dev, "debounce press must be between "
						"%d to %d and multiple by 2\n",
				MAX7370_MIN_KPDEBOUNCE, MAX7370_MAX_KPDEBOUNCE);
			return ERR_PTR(-EINVAL);
		}

		if (val % 2 == 1)
			dev_warn(dev, "debounce press must be multiple by 2\n");

	} else {
		val = MAX7370_MAX_KPDEBOUNCE;
		dev_info(dev, "set default debounce press %d ms\n",
				MAX7370_MAX_KPDEBOUNCE);
	}

	/* convert to registers values */
	val >>= 1;
	--val;

	plat->debounce_period = (u8) val;

	/* The custom release delay format is ms */
	of_property_read_u32(np, "maxim,debounce-delay-release-ms", &val);
	if (val) {
		if (val < MAX7370_MIN_KPDEBOUNCE || val > MAX7370_MAX_KPDEBOUNCE) {
			dev_err(dev, "debounce release must be between "
						"%d to %d and multiple by 2\n",
				MAX7370_MIN_KPDEBOUNCE, MAX7370_MAX_KPDEBOUNCE);
			return ERR_PTR(-EINVAL);
		}

		if (val % 2 == 1)
			dev_warn(dev, "debounce release must be multiple by 2\n");

	} else {
		val = MAX7370_MAX_KPDEBOUNCE;
		dev_info(dev, "set default debounce release %d ms\n",
				MAX7370_MAX_KPDEBOUNCE);
	}

	/* convert to registers values */
	val >>= 1;
	--val;

	plat->debounce_period |= (u8)(val << KP_DEBOUNCE_SHIFT);

	/* The custom assert interrupt every N cycles */
	of_property_read_u32(np, "maxim,assert-int-cycles", &val);
	if (val > MAX7370_MAX_INT_CYCLES) {
		dev_err(dev, "interrupt cycles must be between %d to %d\n",
			MAX7370_MIN_INT_CYCLES, MAX7370_MAX_INT_CYCLES);
		return ERR_PTR(-EINVAL);
	}
	plat->int_cycles = (u8) val;

	/* The custom assert interrupt fifo after N events */
	of_property_read_u32(np, "maxim,assert-int-on-fifo-events", &val);
	if (val > MAX7370_MAX_INT_FIFO_EVENTS) {
		dev_err(dev, "interrupt on fifo events must be between %d to %d\n",
			MAX7370_MIN_INT_FIFO_EVENTS, MAX7370_MAX_INT_FIFO_EVENTS);
		return ERR_PTR(-EINVAL);
	}
	plat->int_fifo_events = (u8) val;

	if (plat->autorepeat) {
		plat->autorepeat = MAX7370_AUTOREPEAT_MASK;

		of_property_read_u32(np, "maxim,autorepeat-delay-cycles", &val);
		if (val) {
			if (val < MAX7370_MIN_AUTORPT_DELAY_CYCLES ||
				val > MAX7370_MAX_AUTORPT_DELAY_CYCLES) {
				dev_err(dev, "autorepeat delay cycles must be between "
							"%d to %d and multiple by 8\n",
						MAX7370_MIN_AUTORPT_DELAY_CYCLES,
						MAX7370_MAX_AUTORPT_DELAY_CYCLES);
				return ERR_PTR(-EINVAL);
			}

			val /= 8;
			--val;

			plat->autorepeat |= MAX7370_AUTOREPEAT_DELAY_CYCLES_MASK &&
					(val << MAX7370_AUTOREPEAT_DELAY_CYCLES_SHIFT);
		} else {
			dev_info(dev, "set default autorepeat delay %d "
						"debounce cycles\n",
					MAX7370_MIN_AUTORPT_DELAY_CYCLES);
		}

		of_property_read_u32(np, "maxim,autorepeat-freq-cycles", &val);
		if (val) {
			if (val < MAX7370_MIN_AUTORPT_FREQ_CYCLES ||
				val > MAX7370_MAX_AUTORPT_FREQ_CYCLES) {
				dev_err(dev, "autorepeat freq cycles must be between "
							"%d to %d and multiple by 4\n",
						MAX7370_MIN_AUTORPT_FREQ_CYCLES,
						MAX7370_MAX_AUTORPT_FREQ_CYCLES);
				return ERR_PTR(-EINVAL);
			}

			val /= 4;
			--val;

			plat->autorepeat |= MAX7370_AUTOREPEAT_FREQ_CYCLES_MASK &&
					(val << MAX7370_AUTOREPEAT_FREQ_CYCLES_SHIFT);
		} else {
			dev_info(dev, "set default autorepeat freq %d "
						"debounce cycles\n",
					MAX7370_MIN_AUTORPT_FREQ_CYCLES);
		}
	}

	/* FIXME: should be property of the IRQ resource? */
	plat->irqtype = IRQF_TRIGGER_LOW;

	return plat;
}

static int max7370_keypad_probe(struct platform_device *pdev)
{
	struct max7370 *max7370 = dev_get_drvdata(pdev->dev.parent);
	struct max7370_keypad *keypad;
	struct input_dev *input;
	const struct max7370_keypad_platform_data *plat;
	int error, irq;

	plat = max7370_keypad_of_probe(&pdev->dev);
	if (IS_ERR(plat)) {
		dev_err(&pdev->dev, "invalid keypad platform data\n");
		return PTR_ERR(plat);
	}

	irq = platform_get_irq_byname(pdev, MAX7370_INT_KBDIRQ_NAME);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return irq;
	}

	keypad = devm_kzalloc(&pdev->dev, sizeof(struct max7370_keypad),
			      GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	keypad->board = plat;
	keypad->input = input;
	keypad->max7370 = max7370;

	input->id.bustype = BUS_I2C;
	input->name = pdev->name;
	input->dev.parent = &pdev->dev;

	input->open = max7370_keypad_open;
	input->close = max7370_keypad_close;

	error = matrix_keypad_build_keymap(plat->keymap_data, NULL,
					   MAX7370_MAX_KEY_ROWS, MAX7370_MAX_KEY_COLS,
					   NULL, input);
	if (error) {
		dev_err(&pdev->dev, "Failed to build keymap\n");
		return error;
	}

	keypad->keymap = input->keycode;

	input_set_capability(input, EV_MSC, MSC_SCAN);
	if (plat->autorepeat)
		__set_bit(EV_REP, input->evbit);

	input_set_drvdata(input, keypad);

	error = max7370_keypad_disable(keypad);
	if (error) {
		dev_err(&pdev->dev, "Could not disable keypad\n");
		return error;
	}

	error = devm_request_threaded_irq(&pdev->dev, irq,
					  NULL, max7370_keypad_irq,
					  plat->irqtype | IRQF_ONESHOT,
					  "max7370-keypad", keypad);
	if (error) {
		dev_err(&pdev->dev,
				"Could not allocate irq %d,error %d\n",
				irq, error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(&pdev->dev, "Could not register input device\n");
		return error;
	}

	/* let platform decide if keypad is a wakeup source or not */
	error = device_init_wakeup(&pdev->dev, plat->enable_wakeup);
	if (error)
		dev_err(&pdev->dev, "device_init_wakeup failed: %d\n", error);

	platform_set_drvdata(pdev, keypad);

	return 0;
}

#ifdef CONFIG_PM
static int max7370_keypad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_keypad *keypad = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, MAX7370_INT_KBDIRQ_NAME);

	/* keypad is already off; we do nothing */
	if (keypad->keypad_stopped)
		return 0;

	/* if device is not a wakeup source, disable it for powersave */
	if (!device_may_wakeup(&pdev->dev))
		max7370_keypad_disable(keypad);
	else
		enable_irq_wake(irq);

	return 0;
}

static int max7370_keypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max7370_keypad *keypad = platform_get_drvdata(pdev);
	int irq = platform_get_irq_byname(pdev, MAX7370_INT_KBDIRQ_NAME);

	if (!keypad->keypad_stopped)
		return 0;

	/* enable the device to resume normal operations */
	if (!device_may_wakeup(&pdev->dev))
		max7370_keypad_enable(keypad);
	else
		disable_irq_wake(irq);

	return 0;
}

static const struct dev_pm_ops max7370_keypad_dev_pm_ops = {
	.suspend = max7370_keypad_suspend,
	.resume  = max7370_keypad_resume,
};
#endif

static const struct of_device_id max7370_keypad_match[] = {
	/* Legacy compatible string */
	{ .compatible = "maxim,max7370-keypad", .data = (void *) MAX7370 },
	{ }
};
MODULE_DEVICE_TABLE(of, max7370_keypad_match);

static struct platform_driver max7370_keypad_driver = {
	.driver	= {
		.name			= "max7370-keypad",
		.owner			= THIS_MODULE,
		.of_match_table	= of_match_ptr(max7370_keypad_match),
#ifdef CONFIG_PM
		.pm				= &max7370_keypad_dev_pm_ops,
#endif
	},
	.probe	= max7370_keypad_probe,
};
module_platform_driver(max7370_keypad_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Maxim Paymushkin");
MODULE_DESCRIPTION("MAX7370 Keypad Driver");
MODULE_ALIAS("platform:max7370-keypad");
