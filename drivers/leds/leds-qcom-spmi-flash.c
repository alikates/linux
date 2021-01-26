// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm SPMI Flash Driver
 *
 * Copyright (c) 2020, Nícolas F. R. A. Prado <nfraprado@protonmail.com>
 *
 * Based on QPNP LEDs driver from downstream MSM kernel sources.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/led-class-flash.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <dt-bindings/leds/leds-qcom-spmi-flash.h>

#define FLASH_SAFETY_TIMER		0x40
#define FLASH_MAX_CURR			0x41
#define FLASH_LED_0_CURR		0x42
#define FLASH_LED_1_CURR		0x43
#define FLASH_CLAMP_CURR		0x44
#define FLASH_LED_TMR_CTRL		0x48
#define FLASH_HEADROOM			0x4A
#define FLASH_STARTUP_DELAY		0x4B
#define FLASH_MASK_ENABLE		0x4C
#define FLASH_VREG_OK_FORCE		0x4F
#define FLASH_ENABLE_CONTROL		0x46
#define FLASH_LED_STROBE_CTRL		0x47
#define FLASH_LED_UNLOCK_SECURE		0xD0
#define FLASH_LED_TORCH			0xE4
#define FLASH_FAULT_DETECT		0x51
#define FLASH_RAMP_RATE			0x54
#define FLASH_PERIPHERAL_SUBTYPE	0x05
#define FLASH_VPH_PWR_DROOP		0x5A

#define FLASH_MAX_LEVEL			0x4F
#define TORCH_MAX_LEVEL			0x0F
#define	FLASH_NO_MASK			0x00

#define FLASH_MASK_1			0x20
#define FLASH_MASK_REG_MASK		0xE0
#define FLASH_HEADROOM_MASK		0x03
#define FLASH_SAFETY_TIMER_MASK		0x7F
#define FLASH_CURRENT_MASK		0xFF
#define FLASH_MAX_CURRENT_MASK		0x7F
#define FLASH_TMR_MASK			0x03
#define FLASH_TMR_WATCHDOG		0x03
#define FLASH_TMR_SAFETY		0x00
#define FLASH_FAULT_DETECT_MASK		0X80
#define FLASH_HW_VREG_OK		0x40
#define FLASH_VREG_MASK			0xC0
#define FLASH_STARTUP_DLY_MASK		0x02
#define FLASH_RAMP_RATE_MASK		0xBF
#define FLASH_VPH_PWR_DROOP_MASK	0xF3

#define FLASH_ENABLE_ALL		0xE0
#define FLASH_ENABLE_MODULE		0x80
#define FLASH_ENABLE_MODULE_MASK	0x80
#define FLASH_DISABLE_ALL		0x00
#define FLASH_ENABLE_MASK		0xE0
#define FLASH_ENABLE_LED_0		0xC0
#define FLASH_ENABLE_LED_1		0xA0
#define FLASH_INIT_MASK			0xE0
#define FLASH_SELFCHECK_ENABLE		0x80
#define FLASH_SELFCHECK_DISABLE		0x00

#define FLASH_STROBE_SW			0xC0
#define FLASH_STROBE_HW			0x04
#define FLASH_STROBE_MASK		0xC7
#define FLASH_LED_0_OUTPUT		0x80
#define FLASH_LED_1_OUTPUT		0x40

#define FLASH_TORCH_MASK		0x03
#define FLASH_LED_TORCH_ENABLE		0x00
#define FLASH_LED_TORCH_DISABLE		0x03
#define FLASH_UNLOCK_SECURE		0xA5
#define FLASH_SECURE_MASK		0xFF

#define FLASH_SUBTYPE_DUAL		0x01
#define FLASH_SUBTYPE_SINGLE		0x02

#define FLASH_DURATION_STEP		10000
#define FLASH_DURATION_MIN		10000
#define FLASH_DURATION_MAX		1280000 //TODO: find real value

#define FLASH_CURRENT_STEP		12500
#define FLASH_CURRENT_MIN		12500
#define FLASH_CURRENT_MAX		1000000
#define FLASH_TORCH_CURRENT_MAX		200000

#define FLASH_DEFAULT_CLAMP		200000

enum qcom_flash_ids {
	QCOM_FLASH_ID_LED0,
	QCOM_FLASH_ID_LED1,
};

static u8 flash_debug_regs[] = {
	0x40, 0x41, 0x42, 0x43, 0x44, 0x48, 0x49, 0x4b, 0x4c,
	0x4f, 0x46, 0x47,
};

/**
 * struct qcom_flash_led - Represents each individual flash LED
 * @cdev: flash LED classdev
 * @id: led ID as given by enum qcom_flash_ids
 * @default_on: default state for the LED
 * @turn_off_delay_ms: number of msec before turning off the LED
 * @clamp_curr: Clamp current to use
 * @headroom: Headroom value to use, as given by leds-qcom-spmi-flash.h
 * @enable_module: enable address for particular flash
 * @trigger_flash: trigger flash
 * @startup_dly: startup delay for flash, as given by leds-qcom-spmi-flash.h
 * @strobe_type: select between sw and hw strobe
 * @current_addr: address to write for current
 * @second_addr: address of secondary flash to be written
 * @safety_timer: enable safety timer or watchdog timer
 * @torch_enable: whether torch mode is enabled or individual flash LED
 */
struct qcom_flash_led {
	struct led_classdev_flash cdev;
	int id;
	bool default_on;
	int turn_off_delay_ms;
	u32 clamp_curr;
	u8 headroom;
	u8 enable_module;
	u8 trigger_flash;
	u8 startup_dly;
	u8 strobe_type;
	u16 current_addr;
	u16 second_addr;
	bool safety_timer;
	bool torch_enable;
};

/**
 * struct qcom_flash_device - QCOM SPMI Flash device, contains 2 flash LEDs
 * @regmap: regmap used to access LED registers over SPMI
 * @base: base register given in device tree
 * @pdev: platform device from devicetree
 * @flash_boost_reg: voltage regulator to supply the flashes
 * @torch_boost_reg: voltage regulator to supply torch mode
 * @leds: flash LEDs
 * @num_leds: number of LEDs registered (between 0 and 2)
 * @lock: lock to protect SPMI transactions
 * @torch_enable: enable flash LED torch mode
 * @peripheral_subtype: module peripheral subtype
 * @flash_regulator_on: flash regulator status
 * @torch_regulator_on: torch regulator status
 */
struct qcom_flash_device {
	struct regmap *regmap;
	u16 base;
	struct platform_device *pdev;
	struct regulator *flash_boost_reg;
	struct regulator *torch_boost_reg;
	struct qcom_flash_led leds[2];
	u8 num_leds;
	struct mutex lock;
	u8 torch_enable;
	u8 peripheral_subtype;
	bool flash_regulator_on;
	bool torch_regulator_on;
};

static inline struct qcom_flash_led *flcdev_to_led(struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct qcom_flash_led, cdev);
}

static inline struct qcom_flash_device *led_to_leds_dev(struct qcom_flash_led *led)
{
	return container_of(led, struct qcom_flash_device, leds[led->id]);
}

static int led_read_reg(struct qcom_flash_device *leds_dev, u16 offset, u8 *data)
{
	unsigned int val;
	int ret;

	ret = regmap_read(leds_dev->regmap, leds_dev->base + offset, &val);
	if (ret < 0)
		return ret;

	*data = val;
	return 0;
}

static int led_write_reg(struct qcom_flash_device *leds_dev, u16 offset, u8 data)
{
	return regmap_write(leds_dev->regmap, leds_dev->base + offset, data);
}

static int qcom_flash_masked_write(struct qcom_flash_device *leds_dev,
				   u16 addr,
				   u8 mask,
				   u8 val)
{
	int rc;
	u8 reg;

	rc = led_read_reg(leds_dev, addr, &reg);
	if (rc)
		dev_err(&leds_dev->pdev->dev,
			"Unable to read from addr=%x, rc(%d)\n", addr, rc);

	reg &= ~mask;
	reg |= val;

	rc = led_write_reg(leds_dev, addr, reg);
	if (rc)
		dev_err(&leds_dev->pdev->dev,
			"Unable to write to addr=%x, rc(%d)\n", addr, rc);
	return rc;
}

static void qcom_flash_dump_regs(struct qcom_flash_device *leds_dev,
				 u8 regs[],
				 u8 array_size)
{
	int i;
	u8 val;

	pr_debug("===== LED register dump start =====\n");
	for (i = 0; i < array_size; i++) {
		led_read_reg(leds_dev, regs[i], &val);
		pr_debug("0x%x = 0x%x\n", leds_dev->base + regs[i], val);
	}
	pr_debug("===== LED register dump end =====\n");
}

static u8 qcom_flash_duration_to_reg(u32 us)
{
	if (us < FLASH_DURATION_MIN)
		us = FLASH_DURATION_MIN;
	return (us - FLASH_DURATION_MIN) / FLASH_DURATION_STEP;
}

static u8 qcom_flash_current_to_reg(u32 ua)
{
	if (ua < FLASH_CURRENT_MIN)
		ua = FLASH_CURRENT_MIN;
	return (ua - FLASH_CURRENT_MIN) / FLASH_CURRENT_STEP;
}

static void clamp_align(u32 *v, u32 min, u32 max, u32 step)
{
	*v = clamp_val(*v, min, max);
	if (step > 1)
		*v = (*v - min) / step * step + min;
}

static int qcom_flash_fled_regulator_operate(struct qcom_flash_device *leds_dev,
					     bool on)
{
	int rc;

	if (!on)
		goto regulator_turn_off;

	if (!leds_dev->flash_regulator_on) {
		if (leds_dev->flash_boost_reg) {
			rc = regulator_enable(leds_dev->flash_boost_reg);
			if (rc) {
				dev_err(&leds_dev->pdev->dev,
					"Regulator enable failed(%d)\n", rc);
				return rc;
			}
			leds_dev->flash_regulator_on = true;
		}
	}

	return 0;

regulator_turn_off:
	if (leds_dev->flash_regulator_on) {
		if (leds_dev->flash_boost_reg) {
			rc = qcom_flash_masked_write(leds_dev,
				FLASH_ENABLE_CONTROL,
				FLASH_ENABLE_MASK,
				FLASH_DISABLE_ALL);
			if (rc)
				dev_err(&leds_dev->pdev->dev,
					"Enable reg write failed(%d)\n", rc);

			rc = regulator_disable(leds_dev->flash_boost_reg);
			if (rc) {
				dev_err(&leds_dev->pdev->dev,
					"Regulator disable failed(%d)\n", rc);
				return rc;
			}
			leds_dev->flash_regulator_on = false;
		}
	}

	return 0;
}

static int qcom_flash_torch_regulator_operate(struct qcom_flash_device *leds_dev,
					      bool on)
{
	int rc;

	if (!on)
		goto regulator_turn_off;

	if (!leds_dev->torch_regulator_on) {
		rc = regulator_enable(leds_dev->torch_boost_reg);
		if (rc) {
			dev_err(&leds_dev->pdev->dev,
				"Regulator enable failed(%d)\n", rc);
			return rc;
		}
		leds_dev->torch_regulator_on = true;
	}
	return 0;

regulator_turn_off:
	if (leds_dev->torch_regulator_on) {
		rc = qcom_flash_masked_write(leds_dev, FLASH_ENABLE_CONTROL,
				FLASH_ENABLE_MODULE_MASK, FLASH_DISABLE_ALL);
		if (rc)
			dev_err(&leds_dev->pdev->dev,
				"Enable reg write failed(%d)\n", rc);

		rc = regulator_disable(leds_dev->torch_boost_reg);
		if (rc) {
			dev_err(&leds_dev->pdev->dev,
				"Regulator disable failed(%d)\n", rc);
			return rc;
		}
		leds_dev->torch_regulator_on = false;
	}
	return 0;
}

static int qcom_flash_fled_set(struct qcom_flash_led *led, bool on)
{
	int rc, error;
	u8 curr;
	struct qcom_flash_device *leds_dev = led_to_leds_dev(led);
	struct device *dev = &leds_dev->pdev->dev;

	/* dump flash registers */
	pr_debug("Regdump before\n");
	qcom_flash_dump_regs(leds_dev, flash_debug_regs,
			     ARRAY_SIZE(flash_debug_regs));

	/* Set led current */
	if (on) {
		if (led->torch_enable)
			curr = qcom_flash_current_to_reg(led->cdev.led_cdev.brightness);
		else
			curr = qcom_flash_current_to_reg(led->cdev.brightness.val);

		if (led->torch_enable) {
			if (leds_dev->peripheral_subtype == FLASH_SUBTYPE_DUAL) {
				rc = qcom_flash_torch_regulator_operate(leds_dev, true);
				if (rc) {
					dev_err(dev,
					"Torch regulator operate failed(%d)\n",
					rc);
					return rc;
				}
			} else if (leds_dev->peripheral_subtype == FLASH_SUBTYPE_SINGLE) {
				rc = qcom_flash_fled_regulator_operate(leds_dev, true);
				if (rc) {
					dev_err(dev,
					"Flash regulator operate failed(%d)\n",
					rc);
					goto error_flash_set;
				}

				/*
				 * Write 0x80 to MODULE_ENABLE before writing
				 * 0xE0 in order to avoid a hardware bug caused
				 * by register value going from 0x00 to 0xE0.
				 */
				rc = qcom_flash_masked_write(leds_dev,
					FLASH_ENABLE_CONTROL,
					FLASH_ENABLE_MODULE_MASK,
					FLASH_ENABLE_MODULE);
				if (rc) {
					dev_err(dev,
						"Enable reg write failed(%d)\n",
						rc);
					return rc;
				}
			}

			rc = qcom_flash_masked_write(leds_dev,
				FLASH_LED_UNLOCK_SECURE,
				FLASH_SECURE_MASK, FLASH_UNLOCK_SECURE);
			if (rc) {
				dev_err(dev, "Secure reg write failed(%d)\n", rc);
				goto error_reg_write;
			}

			rc = qcom_flash_masked_write(leds_dev,
				FLASH_LED_TORCH,
				FLASH_TORCH_MASK, FLASH_LED_TORCH_ENABLE);
			if (rc) {
				dev_err(dev, "Torch reg write failed(%d)\n", rc);
				goto error_reg_write;
			}

			rc = qcom_flash_masked_write(leds_dev,
				led->current_addr,
				FLASH_CURRENT_MASK,
				curr);
			if (rc) {
				dev_err(dev, "Current reg write failed(%d)\n", rc);
				goto error_reg_write;
			}

			rc = qcom_flash_masked_write(leds_dev,
				led->second_addr,
				FLASH_CURRENT_MASK,
				curr);
			if (rc) {
				dev_err(dev,
					"2nd Current reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			qcom_flash_masked_write(leds_dev, FLASH_MAX_CURR,
				FLASH_CURRENT_MASK,
				TORCH_MAX_LEVEL);
			if (rc) {
				dev_err(dev,
					"Max current reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			rc = qcom_flash_masked_write(leds_dev,
				FLASH_ENABLE_CONTROL,
				FLASH_ENABLE_MASK,
				leds_dev->torch_enable);
			if (rc) {
				dev_err(dev, "Enable reg write failed(%d)\n", rc);
				goto error_reg_write;
			}

			rc = qcom_flash_masked_write(leds_dev,
				FLASH_LED_STROBE_CTRL,
				FLASH_STROBE_SW,
				FLASH_STROBE_SW);
			if (rc) {
				dev_err(dev,
					"LED %d strobe reg write failed(%d)\n",
					led->id, rc);
				goto error_reg_write;
			}
		} else {
			rc = qcom_flash_fled_regulator_operate(leds_dev, true);
			if (rc) {
				dev_err(dev,
					"Flash regulator operate failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			/* Set flash safety timer */
			rc = qcom_flash_masked_write(leds_dev,
				FLASH_SAFETY_TIMER,
				FLASH_SAFETY_TIMER_MASK,
				qcom_flash_duration_to_reg(led->cdev.timeout.val));
			if (rc) {
				dev_err(dev,
					"Safety timer reg write failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			/* Set max current */
			rc = qcom_flash_masked_write(leds_dev,
				FLASH_MAX_CURR, FLASH_CURRENT_MASK,
				FLASH_MAX_LEVEL);
			if (rc) {
				dev_err(dev,
					"Max current reg write failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			/* Set clamp current */
			rc = qcom_flash_masked_write(leds_dev,
				FLASH_CLAMP_CURR,
				FLASH_CURRENT_MASK,
				qcom_flash_current_to_reg(led->clamp_curr));
			if (rc) {
				dev_err(dev,
					"Clamp current reg write failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			rc = qcom_flash_masked_write(leds_dev,
				led->current_addr,
				FLASH_CURRENT_MASK,
				curr);
			if (rc) {
				dev_err(dev, "Current reg write failed(%d)\n", rc);
				goto error_flash_set;
			}

			rc = qcom_flash_masked_write(leds_dev,
				FLASH_ENABLE_CONTROL,
				led->enable_module,
				led->enable_module);
			if (rc) {
				dev_err(dev, "Enable reg write failed(%d)\n", rc);
				goto error_flash_set;
			}

			/* TODO try to not busy wait*/
			mdelay(1);

			if (!led->strobe_type) {
				rc = qcom_flash_masked_write(leds_dev,
					FLASH_LED_STROBE_CTRL,
					led->trigger_flash,
					led->trigger_flash);
				if (rc) {
					dev_err(dev,
					"LED %d strobe reg write failed(%d)\n",
					led->id, rc);
					goto error_flash_set;
				}
			} else {
				rc = qcom_flash_masked_write(leds_dev,
					FLASH_LED_STROBE_CTRL,
					(led->trigger_flash | FLASH_STROBE_HW),
					(led->trigger_flash | FLASH_STROBE_HW));
				if (rc) {
					dev_err(dev,
					"LED %d strobe reg write failed(%d)\n",
					led->id, rc);
					goto error_flash_set;
				}
			}
		}
	} else {
		rc = qcom_flash_masked_write(leds_dev,
			FLASH_LED_STROBE_CTRL,
			led->trigger_flash,
			FLASH_DISABLE_ALL);
		if (rc) {
			dev_err(dev,
				"LED %d flash write failed(%d)\n", led->id, rc);
			if (led->torch_enable)
				goto error_torch_set;
			else
				goto error_flash_set;
		}

		/* TODO try to not busy wait*/
		mdelay(2);
		udelay(160);

		if (led->torch_enable) {
			rc = qcom_flash_masked_write(leds_dev,
				FLASH_LED_UNLOCK_SECURE,
				FLASH_SECURE_MASK, FLASH_UNLOCK_SECURE);
			if (rc) {
				dev_err(dev, "Secure reg write failed(%d)\n", rc);
				goto error_torch_set;
			}

			rc = qcom_flash_masked_write(leds_dev,
					FLASH_LED_TORCH,
					FLASH_TORCH_MASK,
					FLASH_LED_TORCH_DISABLE);
			if (rc) {
				dev_err(dev, "Torch reg write failed(%d)\n", rc);
				goto error_torch_set;
			}

			if (leds_dev->peripheral_subtype == FLASH_SUBTYPE_DUAL) {
				rc = qcom_flash_torch_regulator_operate(leds_dev,false);
				if (rc) {
					dev_err(dev,
						"Torch regulator operate failed(%d)\n",
						rc);
					return rc;
				}
			} else if (leds_dev->peripheral_subtype == FLASH_SUBTYPE_SINGLE) {
				rc = qcom_flash_fled_regulator_operate(leds_dev, false);
				if (rc) {
					dev_err(dev,
						"Flash regulator operate failed(%d)\n",
						rc);
					return rc;
				}
			}
		} else {
			rc = qcom_flash_masked_write(leds_dev,
				FLASH_ENABLE_CONTROL,
				led->enable_module &
				~FLASH_ENABLE_MODULE_MASK,
				FLASH_DISABLE_ALL);
			if (rc) {
				dev_err(dev, "Enable reg write failed(%d)\n", rc);
				if (led->torch_enable)
					goto error_torch_set;
				else
					goto error_flash_set;
			}

			rc = qcom_flash_fled_regulator_operate(leds_dev, false);
			if (rc) {
				dev_err(dev,
					"Flash regulator operate failed(%d)\n",
					rc);
				return rc;
			}
		}
	}

	pr_debug("Regdump after\n");
	qcom_flash_dump_regs(leds_dev, flash_debug_regs, ARRAY_SIZE(flash_debug_regs));

	return 0;

error_reg_write:
	if (leds_dev->peripheral_subtype == FLASH_SUBTYPE_SINGLE)
		goto error_flash_set;

error_torch_set:
	error = qcom_flash_torch_regulator_operate(leds_dev, false);
	if (error) {
		dev_err(dev, "Torch regulator operate failed(%d)\n", rc);
		return error;
	}
	return rc;

error_flash_set:
	error = qcom_flash_fled_regulator_operate(leds_dev, false);
	if (error) {
		dev_err(dev, "Flash regulator operate failed(%d)\n", rc);
		return error;
	}
	return rc;
}

static int qcom_flash_led_set(struct led_classdev *led_cdev,
			      enum led_brightness value)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct qcom_flash_led *led = flcdev_to_led(fled_cdev);
	struct qcom_flash_device *leds_dev = led_to_leds_dev(led);
	u32 val = value;
	int rc;
	bool on;

	if (val > led_cdev->max_brightness) {
		dev_err(&leds_dev->pdev->dev, "Invalid brightness value\n");
		return -EINVAL;
	}

	if (val) {
		on = true;
		clamp_align(&val, FLASH_CURRENT_MIN, led_cdev->max_brightness,
			    FLASH_CURRENT_STEP);
		led_cdev->brightness = val;
		led->torch_enable = true;
	} else {
		on = false;
	}

	mutex_lock(&leds_dev->lock);
	rc = qcom_flash_fled_set(led, on);
	if (rc < 0)
		dev_err(&leds_dev->pdev->dev, "FLASH set brightness failed (%d)\n", rc);
	mutex_unlock(&leds_dev->lock);
	return rc;
}

static int qcom_flash_fled_brightness_set(struct led_classdev_flash *fled_cdev,
					  u32 brightness)
{
	clamp_align(&brightness, FLASH_CURRENT_MIN, fled_cdev->brightness.max,
		    FLASH_CURRENT_STEP);
	fled_cdev->brightness.val = brightness;
	return 0;
}

static int qcom_flash_fled_strobe_set(struct led_classdev_flash *fled_cdev,
				      bool state)
{
	struct qcom_flash_led *led = flcdev_to_led(fled_cdev);
	struct qcom_flash_device *leds_dev = led_to_leds_dev(led);
	int rc;

	led->torch_enable = false;

	mutex_lock(&leds_dev->lock);
	rc = qcom_flash_fled_set(led, state);
	if (rc < 0)
		return rc;
	mutex_unlock(&leds_dev->lock);

	return 0;
}

static int qcom_flash_fled_strobe_get(struct led_classdev_flash *fled_cdev,
				      bool *state)
{
	//TODO
	*state = 0;
	return 0;
}

static int qcom_flash_fled_timeout_set(struct led_classdev_flash *fled_cdev,
				       u32 timeout)
{
	fled_cdev->timeout.val = timeout;
	return 0;
}

static int qcom_flash_fled_fault_get(struct led_classdev_flash *fled_cdev,
				     u32 *fault)
{
	//TODO
	*fault = 0;
	return 0;
}

static const struct led_flash_ops flash_ops = {
	.flash_brightness_set	= qcom_flash_fled_brightness_set,
	.strobe_set		= qcom_flash_fled_strobe_set,
	.strobe_get		= qcom_flash_fled_strobe_get,
	.timeout_set		= qcom_flash_fled_timeout_set,
	.fault_get		= qcom_flash_fled_fault_get,
};

static int qcom_flash_flcdev_setup(struct qcom_flash_led *led,
				   struct device_node *node)
{
	int rc;
	struct platform_device *pdev = led_to_leds_dev(led)->pdev;
	struct led_init_data init_data = {};

	init_data.fwnode = of_fwnode_handle(node);
	init_data.devicename = "qcom-spmi-flash";
	init_data.default_label = "flash";

	led->cdev.led_cdev.brightness_set_blocking    = qcom_flash_led_set;

	rc = of_property_read_u32(node, "led-max-microamp",
				  &led->cdev.led_cdev.max_brightness);
	if (rc < 0) {
		dev_err(&pdev->dev, "Failure reading max_current, rc =  %d\n", rc);
		return rc;
	}
	led->cdev.led_cdev.max_brightness = min((u32) led->cdev.led_cdev.max_brightness,
						(u32) FLASH_TORCH_CURRENT_MAX);

	rc = of_property_read_u32(node, "flash-max-microamp", &led->cdev.brightness.max);
	if (rc < 0) {
		dev_err(&pdev->dev, "Failure reading max_current, rc =  %d\n", rc);
		return rc;
	}
	led->cdev.brightness.max = min((u32) led->cdev.brightness.max,
				(u32) FLASH_CURRENT_MAX);
	led->cdev.brightness.min = FLASH_CURRENT_MIN;
	led->cdev.brightness.step = FLASH_CURRENT_STEP;
	led->cdev.brightness.val = led->cdev.brightness.max;

	rc = of_property_read_u32(node, "flash-max-timeout-us", &led->cdev.timeout.max);
	if (rc < 0) {
		dev_err(&pdev->dev, "Failure reading max_timeout, rc =  %d\n", rc);
		return rc;
	}
	led->cdev.timeout.max = min((u32) led->cdev.timeout.max,
				    (u32) FLASH_DURATION_MAX);
	led->cdev.timeout.min = FLASH_DURATION_MIN;
	led->cdev.timeout.step = FLASH_DURATION_STEP;
	led->cdev.timeout.val = led->cdev.timeout.max;

	led->cdev.ops = &flash_ops;
	led->cdev.led_cdev.flags |= LED_DEV_CAP_FLASH;

	rc = led_classdev_flash_register_ext(&pdev->dev, &led->cdev, &init_data);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led %d,rc=%d\n", led->id, rc);
		return rc;
	}

	return 0;
}

static int qcom_flash_setup_flash_regs(struct qcom_flash_led *led)
{
	int rc;
	struct qcom_flash_device *leds_dev = led_to_leds_dev(led);

	/* Set headroom */
	rc = qcom_flash_masked_write(leds_dev, FLASH_HEADROOM,
		FLASH_HEADROOM_MASK, led->headroom);
	if (rc) {
		dev_err(&leds_dev->pdev->dev, "Headroom reg write failed(%d)\n", rc);
		return rc;
	}

	/* Set startup delay */
	rc = qcom_flash_masked_write(leds_dev,
		FLASH_STARTUP_DELAY, FLASH_STARTUP_DLY_MASK,
		led->startup_dly);
	if (rc) {
		dev_err(&leds_dev->pdev->dev,
			"Startup delay reg write failed(%d)\n", rc);
		return rc;
	}

	/* Set timer control - safety or watchdog */
	if (led->safety_timer) {
		rc = qcom_flash_masked_write(leds_dev,
			FLASH_LED_TMR_CTRL,
			FLASH_TMR_MASK, FLASH_TMR_SAFETY);
		if (rc) {
			dev_err(&leds_dev->pdev->dev,
				"LED timer ctrl reg write failed(%d)\n", rc);
			return rc;
		}
	}

	/* dump flash registers */
	qcom_flash_dump_regs(leds_dev, flash_debug_regs, ARRAY_SIZE(flash_debug_regs));

	return 0;
}

static int qcom_flash_get_config_flash(struct qcom_flash_led *led,
				       struct device_node *node)
{
	int rc;
	u32 val;
	const char *temp_string;
	struct device *dev = &led_to_leds_dev(led)->pdev->dev;

	rc = of_property_read_u32(node, "led-sources", &led->id);
	if (rc < 0) {
		dev_err(dev, "Failure reading led id, rc =  %d\n", rc);
		return rc;
	}

	led->default_on = false;
	rc = of_property_read_string(node, "default-state", &temp_string);
	if (!rc) {
		if (strncmp(temp_string, "on", sizeof("on")) == 0)
			led->default_on = true;
	} else if (rc != -EINVAL)
		return rc;

	led->turn_off_delay_ms = 0;
	rc = of_property_read_u32(node, "qcom,turn-off-delay-ms", &val);
	if (!rc)
		led->turn_off_delay_ms = val;
	else if (rc != -EINVAL)
		return rc;

	if (led->id == QCOM_FLASH_ID_LED0) {
		led->enable_module = FLASH_ENABLE_LED_0;
		led->current_addr = FLASH_LED_0_CURR;
		led->trigger_flash = FLASH_LED_0_OUTPUT;

		led->second_addr = FLASH_LED_1_CURR;
	} else if (led->id == QCOM_FLASH_ID_LED1) {
		led->enable_module = FLASH_ENABLE_LED_1;
		led->current_addr = FLASH_LED_1_CURR;
		led->trigger_flash = FLASH_LED_1_OUTPUT;

		led->second_addr = FLASH_LED_0_CURR;
	} else {
		dev_err(dev, "Unknown flash LED name given\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,headroom", &val);
	if (!rc)
		led->headroom = (u8) val;
	else if (rc == -EINVAL)
		led->headroom = QCOM_SPMI_FLASH_HEADROOM_500MV;

	rc = of_property_read_u32(node, "qcom,clamp-curr", &val);
	if (!rc)
		led->clamp_curr = val;
	else if (rc == -EINVAL)
		led->clamp_curr = FLASH_DEFAULT_CLAMP;

	rc = of_property_read_u32(node, "qcom,startup-dly", &val);
	if (!rc)
		led->startup_dly = (u8) val;
	else if (rc == -EINVAL)
		led->startup_dly = QCOM_SPMI_FLASH_STARTUP_DLY_128US;

	led->safety_timer = of_property_read_bool(node, "qcom,safety-timer");

	return 0;
}

static int qcom_flash_setup_regs(struct qcom_flash_device *leds_dev)
{
	int rc;

	rc = qcom_flash_masked_write(leds_dev,
		FLASH_LED_STROBE_CTRL,
		FLASH_STROBE_MASK, FLASH_DISABLE_ALL);
	if (rc) {
		dev_err(&leds_dev->pdev->dev, "LED flash write failed(%d)\n", rc);
		return rc;
	}

	/* Disable flash LED module */
	rc = qcom_flash_masked_write(leds_dev, FLASH_ENABLE_CONTROL,
		FLASH_ENABLE_MODULE_MASK, FLASH_DISABLE_ALL);
	if (rc) {
		dev_err(&leds_dev->pdev->dev, "Enable reg write failed(%d)\n", rc);
		return rc;
	}

	/* Set Vreg force */
	rc = qcom_flash_masked_write(leds_dev, FLASH_VREG_OK_FORCE,
		FLASH_VREG_MASK, FLASH_HW_VREG_OK);
	if (rc) {
		dev_err(&leds_dev->pdev->dev, "Vreg OK reg write failed(%d)\n", rc);
		return rc;
	}

	/* Set self fault check */
	rc = qcom_flash_masked_write(leds_dev, FLASH_FAULT_DETECT,
		FLASH_FAULT_DETECT_MASK, FLASH_SELFCHECK_DISABLE);
	if (rc) {
		dev_err(&leds_dev->pdev->dev,
			"Fault detect reg write failed(%d)\n", rc);
		return rc;
	}

	/* Set mask enable */
	rc = qcom_flash_masked_write(leds_dev, FLASH_MASK_ENABLE,
		FLASH_MASK_REG_MASK, FLASH_MASK_1);
	if (rc) {
		dev_err(&leds_dev->pdev->dev, "Mask enable reg write failed(%d)\n", rc);
		return rc;
	}

	/* Set ramp rate */
	rc = qcom_flash_masked_write(leds_dev, FLASH_RAMP_RATE,
		FLASH_RAMP_RATE_MASK, 0xBF);
	if (rc) {
		dev_err(&leds_dev->pdev->dev, "Ramp rate reg write failed(%d)\n", rc);
		return rc;
	}

	/* Enable VPH_PWR_DROOP and set threshold to 2.9V */
	rc = qcom_flash_masked_write(leds_dev, FLASH_VPH_PWR_DROOP,
					FLASH_VPH_PWR_DROOP_MASK, 0xC2);
	if (rc) {
		dev_err(&leds_dev->pdev->dev,
			"FLASH_VPH_PWR_DROOP reg write failed(%d)\n", rc);
		return rc;
	}

	return 0;
}

static int qcom_flash_setup_led(struct qcom_flash_led *led,
				struct device_node *node)
{
	int rc;

	rc = qcom_flash_get_config_flash(led, node);
	if (rc < 0) {
		dev_err(&led_to_leds_dev(led)->pdev->dev,
			"Unable to read flash config data\n");
		return rc;
	}

	rc = qcom_flash_setup_flash_regs(led);
	if (rc) {
		dev_err(&led_to_leds_dev(led)->pdev->dev,
			"FLASH initialize failed(%d)\n", rc);
		return rc;
	}

	rc = qcom_flash_flcdev_setup(led, node);
	if (rc < 0)
		return rc;

	/* configure default state */
	if (led->default_on) {
		led->cdev.led_cdev.brightness = led->cdev.led_cdev.max_brightness;
		led->torch_enable = true;
		mutex_lock(&led_to_leds_dev(led)->lock);
		rc = qcom_flash_fled_set(led, true);
		if (rc < 0)
			return rc;
		mutex_unlock(&led_to_leds_dev(led)->lock);
	} else {
		led->cdev.led_cdev.brightness = LED_OFF;
	}

	return 0;
}

static int qcom_flash_setup_leds_device(struct qcom_flash_device *leds_dev,
					struct device_node *node,
					struct platform_device *pdev)
{
	u32 reg;
	int rc;

	leds_dev->pdev = pdev;

	leds_dev->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!leds_dev->regmap)
		return -ENODEV;

	rc = of_property_read_u32(node, "reg", &reg);
	if (rc < 0) {
		dev_err(&pdev->dev, "Failure reading reg, rc = %d\n", rc);
		return rc;
	}
	leds_dev->base = reg;

	qcom_flash_setup_regs(leds_dev);

	if (of_find_property(node, "flash-boost-supply", NULL)) {
		leds_dev->flash_boost_reg = regulator_get(&pdev->dev, "flash-boost");
		if (IS_ERR(leds_dev->flash_boost_reg)) {
			rc = PTR_ERR(leds_dev->flash_boost_reg);
			dev_err(&pdev->dev, "Regulator get failed(%d)\n", rc);
			regulator_put(leds_dev->flash_boost_reg);
			leds_dev->flash_boost_reg = NULL;
			return rc;
		}
	}

	if (of_find_property(node, "torch-boost-supply", NULL)) {
		leds_dev->torch_boost_reg = regulator_get(&pdev->dev, "torch-boost");
		if (IS_ERR(leds_dev->torch_boost_reg)) {
			rc = PTR_ERR(leds_dev->torch_boost_reg);
			dev_err(&pdev->dev, "Torch regulator get failed(%d)\n", rc);
			regulator_put(leds_dev->flash_boost_reg);
			regulator_put(leds_dev->torch_boost_reg);
			leds_dev->flash_boost_reg = NULL;
			leds_dev->torch_boost_reg = NULL;
			return rc;
		}
		leds_dev->torch_enable = FLASH_ENABLE_MODULE;
	} else {
		leds_dev->torch_enable = FLASH_ENABLE_ALL;
	}

	rc = led_read_reg(leds_dev, FLASH_PERIPHERAL_SUBTYPE,
			  &leds_dev->peripheral_subtype);
	if (rc) {
		dev_err(&pdev->dev,
			"Unable to read from addr=%x, rc(%d)\n",
			FLASH_PERIPHERAL_SUBTYPE, rc);
	}

	mutex_init(&leds_dev->lock);

	return 0;
}

static int qcom_flash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_flash_device *leds_dev;
	struct device_node *node = dev->of_node;
	struct qcom_flash_led *led;
	struct device_node *temp;
	int rc, i, parsed_leds = 0;

	leds_dev = devm_kzalloc(dev, sizeof(struct qcom_flash_device), GFP_KERNEL);
	if (!leds_dev)
		return -ENOMEM;

	rc = qcom_flash_setup_leds_device(leds_dev, node, pdev);
	if (rc) {
		pr_debug("Probe failed setting up base (%d)\n", rc);
		return rc;
	}

	platform_set_drvdata(pdev, leds_dev);

	for_each_child_of_node(node, temp) {
		led = &leds_dev->leds[parsed_leds];

		rc = qcom_flash_setup_led(led, temp);
		if (rc) {
			for (i = 0; i < parsed_leds; i++)
				led_classdev_flash_unregister(&leds_dev->leds[i].cdev);
			pr_debug("Probe failed setting up leds (%d)\n", rc);
			return rc;
		}

		parsed_leds++;
	}
	leds_dev->num_leds = parsed_leds;
	return 0;
}

static int qcom_flash_remove(struct platform_device *pdev)
{
	struct qcom_flash_device *leds_dev  = platform_get_drvdata(pdev);
	int i, parsed_leds = leds_dev->num_leds;

	mutex_destroy(&leds_dev->lock);
	if (leds_dev->flash_boost_reg)
		regulator_put(leds_dev->flash_boost_reg);
	if (leds_dev->torch_boost_reg)
		regulator_put(leds_dev->torch_boost_reg);
	for (i = 0; i < parsed_leds; i++)
		led_classdev_flash_unregister(&leds_dev->leds[i].cdev);

	return 0;
}

static const struct of_device_id qcom_flash_spmi_of_match[] = {
	{ .compatible = "qcom,spmi-flash" },
	{},
};
MODULE_DEVICE_TABLE(of, qcom_flash_spmi_of_match);

static struct platform_driver qcom_flash_driver = {
	.driver		= {
		.name	= "qcom,spmi-flash",
		.of_match_table = of_match_ptr(qcom_flash_spmi_of_match),
	},
	.probe		= qcom_flash_probe,
	.remove		= qcom_flash_remove,
};
module_platform_driver(qcom_flash_driver);

MODULE_DESCRIPTION("Qualcomm SPMI Flash LED driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nícolas F. R. A. Prado <nfraprado@protonmail.com>");
