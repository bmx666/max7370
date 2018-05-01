#ifndef __LINUX_MFD_MAX7370_H
#define __LINUX_MFD_MAX7370_H

struct device;

/**
 * enum max7370_version - indicates the MAX7370 version
 */
enum max7370_version {
	MAX7370,
};

enum max7370_block {
	MAX7370_BLOCK_GPIO      = 1 << 0,
	MAX7370_BLOCK_KEYPAD    = 1 << 1,
	MAX7370_BLOCK_PWM       = 1 << 2,
};

#define MAX7370_MAX_GPIO 		16
#define MAX7370_MAX_PWM 		4

/*
 * MAX7370 registers
 */
#define MAX7370_REG_KEYFIFO		0x00
#define MAX7370_REG_CONFIG		0x01
#define MAX7370_REG_DEBOUNCE	0x02
#define MAX7370_REG_INTERRUPT	0x03
#define MAX7370_REG_AUTOREPEAT	0x04
#define MAX7370_REG_KEYREP		0x05
#define MAX7370_REG_AUTOSLEEP	0x06
#define MAX7370_REG_KBDSIZE		0x30

/*
 * MAX7370 registers
 */
#define MAX7370_REG_GPIODIRBASE	0x34
#define MAX7370_REG_GPIOVALBASE	0x3a
#define MAX7370_REG_GPIOCFG		0x40
#define MAX7370_REG_GPIODEB		0x42
#define MAX7370_REG_INT_MASK1	0x58
#define MAX7370_REG_INT_MASK2	0x59
#define MAX7370_REG_TRIG_MODE1	0x5A
#define MAX7370_REG_TRIG_MODE2	0x5B
#define MAX7370_REG_SUPPLY_VOLT1	0x38
#define MAX7370_REG_SUPPLY_VOLT2	0x39

/*
 * MAX7370 registers
 */
#define MAX7370_REG_LED_DRIVER_ENABLE			0x31
#define MAX7370_REG_LED_CONSTANT_CURRENT		0x43
#define MAX7370_REG_COMMON_PWM_RATIO			0x45
#define MAX7370_REG_INDIVIDUAL_PWM_RATIO_BASE	0x50
#define MAX7370_REG_LED_CONFIG_BASE				0x54

/*
 * MAX7370 registers
 */
#define MAX7370_REG_PWMCOM		0x45
#define MAX7370_REG_PWMRATBASE	0x50
#define MAX7370_REG_LEDCFGBASE	0x54

/*
 * Configuration register bits
 */
#define MAX7370_CFG_SLEEP		BIT(7)
#define MAX7370_CFG_INTERRUPT	BIT(5)
#define MAX7370_CFG_KEY_RELEASE	BIT(3)
#define MAX7370_CFG_WAKEUP		BIT(1)
#define MAX7370_CFG_TIMEOUT		BIT(0)

/*
 * GPIO Configuration register bits
 */
#define MAX7370_GPIOCFG_ENABLE	BIT(4)
#define MAX7370_GPIOCFG_RESET	BIT(3)
#define MAX7370_GPIOCFG_FADING	GENMASK(2,0)

/*
 * LED constant current register bits
 */
#define MAX7370_LED_CONSTANT_CURRENT_SETTINGS	BIT(0)

/*
 * LED Configuration register bits
 */
#define MAX7370_LEDCFG_BLINK_ON_TIME_MASK		GENMASK(1,0)
#define MAX7370_LEDCFG_BLINK_ON_TIME_SHIFT		0
#define MAX7370_LEDCFG_BLINK_PERIOD_MASK		GENMASK(4,2)
#define MAX7370_LEDCFG_BLINK_PERIOD_SHIFT		2
#define MAX7370_LEDCFG_ACTIVE_COMMON_PWM_MASK	BIT(5)
#define MAX7370_LEDCFG_ACTIVE_COMMON_PWM_SHIFT	5

#define MAX7370_INT_GPIIRQ		0
#define MAX7370_INT_KBDIRQ		1

#define MAX7370_INT_GPIIRQ_NAME	"gpio"
#define MAX7370_INT_KBDIRQ_NAME	"keypad"

#define MAX7370_NR_INTERNAL_IRQS	2

struct max7370 {
	struct mutex lock;
	struct device *dev;
	struct i2c_client *i2c;
	struct irq_domain *domain;

	int num_gpio;
	struct max7370_platform_data *pdata;
};

extern int max7370_reg_write(struct max7370 *max7370, u8 reg, u8 data);
extern int max7370_reg_read(struct max7370 *max7370, u8 reg);
extern int max7370_set_bits(struct max7370 *max7370, u8 reg, u8 mask, u8 val);

/**
 * struct max7370_platform_data - MAX7370 platform data
 * @block: bitmask of blocks to enable (use MAX7370_BLOCK_*)
 */
struct max7370_platform_data {
	unsigned int block;
};

#endif
