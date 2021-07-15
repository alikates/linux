// SPDX-License-Identifier: GPL-2.0
/*
 * Power supply driver for Qualcomm Switch-Mode Battery Charger
 *
 * Copyright (c) 2021, Yassine Oudjana <y.oudjana@protonmail.com>
 *                     Alejandro Tafalla <atafalla@dnyon.com>
 */

#define DEBUG

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>
#include <linux/util_macros.h>
#include <asm/unaligned.h>

#include "qcom_smbcharger.h"

static int smbchg_sec_write(struct smbchg_chip *chip, u8 *val, u16 addr,
			    int len)
{
	const u8 sec_addr_val = 0xa5;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	ret = regmap_write(chip->regmap, ((chip->base + addr) & 0xff00) | 0xd0,
			sec_addr_val);
	if (ret)
		goto out;

	ret = regmap_bulk_write(chip->regmap, chip->base + addr, val, len);
out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return ret;
}

static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 addr, u8 mask, u8 val)
{
	u8 reg;
	int ret;

	ret = regmap_bulk_read(chip->regmap, chip->base + addr, &reg, 1);
	if (ret)
		return ret;

	reg &= ~mask;
	reg |= val & mask;

	return smbchg_sec_write(chip, &reg, addr, 1);
}

static bool smbchg_usb_is_present(struct smbchg_chip *chip)
{
	u32 value;
	int ret;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_USB_CHGPTH_RT_STS, &value);
	if (ret) {
		dev_err(chip->dev,
			"Failed to read USB charge path real-time status: %d\n", ret);
		return false;
	}

	if (!(value & USBIN_SRC_DET_BIT) || (value & USBIN_OV_BIT))
		return false;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_USB_CHGPTH_INPUT_STS, &value);
	if (ret) {
		dev_err(chip->dev,
			"Failed to read USB charge path input status: %d\n", ret);
		return false;
	}

	return !!(value & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static int smbchg_usb_enable(struct smbchg_chip *chip, bool enable)
{
	int ret;

	dev_dbg(chip->dev, "%sabling USB charge path\n", enable ? "En" : "Dis");

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
				USBIN_SUSPEND_BIT,
				enable ? 0 : USBIN_SUSPEND_BIT);
	if (ret)
		dev_err(chip->dev, "Failed to %sable USB charge path: %d\n",
			enable ? "en" : "dis", ret);

	return ret;
}

static enum power_supply_type smbchg_usb_get_type(struct smbchg_chip *chip)
{
	u32 reg;
	int ret;

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_MISC_IDEV_STS, &reg);
	if (ret) {
		dev_err(chip->dev, "Failed to read USB type: %d\n", ret);
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}

	if (reg & USB_TYPE_SDP_BIT)
		return POWER_SUPPLY_TYPE_USB;
	else if (reg & USB_TYPE_OTHER_BIT ||
		 reg & USB_TYPE_DCP_BIT)
		return POWER_SUPPLY_TYPE_USB_DCP;
	else if (reg & USB_TYPE_CDP_BIT)
		return POWER_SUPPLY_TYPE_USB_CDP;
	else
		return POWER_SUPPLY_TYPE_UNKNOWN;
}

static int smbchg_usb_set_sdp_mode(struct smbchg_chip *chip, int usb_type,
				       int usb_current)
{
	int ret;
	ret = smbchg_sec_masked_write(chip, SMBCHG_USB_CHGPTH_CHGPTH_CFG,
				      USB_2_3_SEL_BIT, usb_type);
	if (ret) {
		dev_err(chip->dev,
			"Failed to configure USB charge path: %d\n", ret);
		return ret;
	}
	ret = regmap_update_bits(chip->regmap,
				 chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
				 USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				 USBIN_LIMITED_MODE | usb_current);
	if (ret)
		dev_err(chip->dev, "Failed to set SDP type: %d\n", ret);

	return ret;
}

/* Set the maximum allowed current. Can be used to change current depending on
 * the temps 
 */
static int smbchg_usb_set_ilim(struct smbchg_chip *chip, int current_ma)
{
	int usb_type, usb_current, ilim_index;
	int ret = 0;

	/*
	 * TODO: Maybe disable lowering current when no battery is present
	 * to prevent the device from powering off (same as downstream)
	 */

	/*
	 * Suspend USB charge path if current is less than
	 * maximum suspend current
	 */
	if (current_ma < SUSPEND_CURRENT_MA) {
		chip->max_usb_current = 0;
		goto out;
	}

	switch(chip->usb_psy_type){
	case POWER_SUPPLY_TYPE_USB:
		/* Standard Downstream Port */
		if(current_ma > 900) {
			/* High power SDP, clamp to 500mA */
			current_ma = 500;
		}

		switch (current_ma) {
		case 150:
			/* USB 3.0, unnegotiated */
			usb_type = USB_3;
			usb_current = USB51_100MA;
			break;
		case 500:
			/* USB 2.0, negotiated */
			usb_type = USB_2;
			usb_current = USB51_500MA;
			break;
		case 900:
			/* USB 3.0, negotiated */
			usb_type = USB_3;
			usb_current = USB51_500MA;
			break;
		default:
			/* USB 2.0, unnegotiated */
			current_ma = 100;
			usb_type = USB_2;
			usb_current = USB51_100MA;
			break;
		}

		smbchg_usb_set_sdp_mode(chip, usb_type, usb_current);
		chip->max_usb_current = current_ma;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		/* Charging Downstream Port */
		if (current_ma < 1500) {
			/* Port announced less than 1500mA, use override for CDP */
			ret = regmap_update_bits(
				chip->regmap,
					chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (ret) {
				dev_err(chip->dev,
					"Failed to enable ICL override: %d\n",
					ret);
				return ret;
			}
		}
		/* fall through */
	case POWER_SUPPLY_TYPE_USB_DCP:
		/* Dedicated Charging Port */
		ilim_index = find_smaller_in_array(chip->data->ilim_usb_table,
					chip->data->ilim_usb_table_len,
					current_ma);

		ret = smbchg_sec_masked_write(chip, SMBCHG_USB_CHGPTH_IL_CFG,
					USBIN_INPUT_MASK, ilim_index);
		if (ret) {
			dev_err(chip->dev,
				"Failed to set DCP current limit: %d\n", ret);
			return ret;
		}

		ret = regmap_update_bits(chip->regmap,
					chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
					USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
		if (ret) {
			dev_err(chip->dev,
				"Failed to set high current mode: %d\n", ret);
			return ret;
		}

		chip->max_usb_current = chip->data->ilim_usb_table[ilim_index];
		break;
	default:
		dev_err(chip->dev,
			"Unknown USB type, not setting current limits");
		chip->max_usb_current = 0;
		return -ENOSYS;
	}
out:
	if(!ret)
		dev_dbg(chip->dev, "USB current set to %dmA", chip->max_usb_current);

	return ret;
}

static int smbchg_charging_enable(struct smbchg_chip *chip, bool enable)
{
	int ret;

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				CHG_EN_BIT,
				enable ? 0 : CHG_EN_BIT);
	if (ret)
		dev_err(chip->dev, "Failed to enable battery charging: %d\n", ret);

	return ret;
}

static int smbchg_charging_set_iterm(struct smbchg_chip *chip)
{
	size_t iterm_count = chip->data->iterm_table_len;
	int val, ret;

	val = chip->batt_info->charge_term_current_ua / 1000;
	val = find_closest(val,	chip->data->iterm_table, iterm_count);

	dev_dbg(chip->dev, "Setting termination current to %dmA",
		chip->data->iterm_table[val]);

	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_TCC_CFG,
				CHG_ITERM_MASK, val);
	if (ret)
		dev_err(chip->dev, "Failed to set termination current: %d", ret);

	return ret;
}

/* TODO: Use this to implement extcon */
static bool smbchg_otg_is_present(struct smbchg_chip *chip)
{
	u32 value;
	u16 usb_id;
	int ret;

	/* Check ID pin */
	ret = regmap_bulk_read(chip->regmap,
				chip->base + SMBCHG_USB_CHGPTH_USBID_MSB,
				&value, 2);
	if(ret) {
		dev_err(chip->dev, "Failed to read ID pin: %d\n", ret);
		return false;
	}

	put_unaligned_be16(value, &usb_id);
	if (usb_id > USBID_GND_THRESHOLD) {
		dev_dbg(chip->dev,
			"0x%02x read on ID pin, too high to be ground\n", usb_id);
		return false;
	}

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_RID_STS,
				&value);
	if(ret) {
		dev_err(chip->dev, "Failed to read resistance ID: %d\n", ret);
		return false;
	}

	return (value & RID_MASK) == 0;
}

static void smbchg_otg_reset_worker(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work, struct smbchg_chip,
						otg_reset_work);
	int ret;

	dev_dbg(chip->dev, "Resetting OTG VBUS regulator\n");

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, 0);
	if (ret) {
		dev_err(chip->dev,
			"Failed to disable OTG regulator for reset: %d\n", ret);
		return;
	}

	msleep(OTG_RESET_DELAY_MS);

	/*
	 * Only re-enable the OTG regulator if OTG is still present
	 * after sleeping
	 */
	if (!smbchg_otg_is_present(chip))
		return;

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, OTG_EN_BIT);
	if (ret)
		dev_err(chip->dev,
			"Failed to re-enable OTG regulator after reset: %d\n",
			ret);
}

static int smbchg_otg_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(chip->dev, "Enabling OTG VBUS regulator");

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, OTG_EN_BIT);
	if(ret)
		dev_err(chip->dev, "Failed to enable OTG regulator: %d", ret);

	return ret;
}

static int smbchg_otg_disable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(chip->dev, "Disabling OTG VBUS regulator");

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, 0);
	if (ret) {
		dev_err(chip->dev, "Failed to disable OTG regulator: %d", ret);
		return ret;
	}

	return 0;
}

static int smbchg_otg_is_enabled(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	u32 value = 0;
	int ret;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_BAT_IF_CMD_CHG, &value);
	if (ret)
		dev_err(chip->dev, "failed to read CMD_CHG\n");

	return !!(value & OTG_EN_BIT);
}

static const struct regulator_ops smbchg_otg_ops = {
	.enable = smbchg_otg_enable,
	.disable = smbchg_otg_disable,
	.is_enabled = smbchg_otg_is_enabled,
};

static irqreturn_t smbchg_handle_charger_error(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_err(chip->dev, "Charger error");

	/* TODO: Handle errors properly */

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_p2f(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_dbg(chip->dev, "Switching to fast charging");

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_rechg(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_dbg(chip->dev, "Recharging");

	/* TODO: Handle recharge properly */

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_taper(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_dbg(chip->dev, "Switching to taper charging");

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_tcc(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_dbg(chip->dev, "Termination current reached");

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_batt_temp(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_usb_source_detect(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	bool usb_present;
	int ret;

	usb_present = smbchg_usb_is_present(chip);
	dev_dbg(chip->dev, "USB %spresent\n", usb_present ? "" : "not ");

	chip->usb_psy_type = smbchg_usb_get_type(chip);

	if (usb_present) {
		switch (chip->usb_psy_type) {
		case POWER_SUPPLY_TYPE_USB:
			ret = smbchg_usb_set_ilim(chip, DEFAULT_SDP_MA);
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			ret = smbchg_usb_set_ilim(chip, DEFAULT_DCP_MA);
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			ret = smbchg_usb_set_ilim(chip, DEFAULT_CDP_MA);
			break;
		default:
			ret = smbchg_usb_set_ilim(chip, 0);
		}
	} else
		ret = smbchg_usb_set_ilim(chip, 0);
	if(ret) {
		dev_dbg(chip->dev, "Failed to set USB current limit: %d\n", ret);
		return IRQ_NONE;
	}

	ret = smbchg_usb_enable(chip, usb_present);
	if(ret) {
		dev_dbg(chip->dev, "Failed to %sable USB charge path: %d\n",
			usb_present ? "en" : "dis", ret);
		return IRQ_NONE;
	}

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

/* TODO: Use this to implement extcon */
static irqreturn_t smbchg_handle_usbid_change(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	bool otg_present;

	/* 
	 * ADC conversion for USB resistance ID in the fuel gauge can take
	 * up to 15ms to finish after the USB ID change interrupt is fired.
	 * Wait for it to finish before detecting OTG presence. Add an extra
	 * 5ms for good measure.
	 */
	msleep(20);

	otg_present = smbchg_otg_is_present(chip);
	dev_dbg(chip->dev, "OTG %spresent\n", otg_present ? "" : "not ");

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_otg_fail(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_err(chip->dev, "OTG regulator failure");

	/* Report failure */
	regulator_notifier_call_chain(chip->otg_reg,
					REGULATOR_EVENT_FAIL, NULL);

	return IRQ_HANDLED;
}

static irqreturn_t smbchg_handle_otg_oc(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	/*
	 * Inrush current of some devices can trip the over-current protection
	 * on the PMI8994 and PMI8996 smbchargers due to a hardware bug.
	 * Try resetting the OTG regulator, and only report over-current
	 * if it persists.
	 */
	if (of_device_is_compatible(chip->dev->of_node, "qcom,pmi8994-smbcharger") ||
		of_device_is_compatible(chip->dev->of_node, "qcom,pmi8996-smbcharger")) {
		if (chip->otg_resets < NUM_OTG_RESET_RETRIES) {
			schedule_work(&chip->otg_reset_work);
			chip->otg_resets++;
			return IRQ_HANDLED;
		}

		chip->otg_resets = 0;
	}

	dev_warn(chip->dev, "OTG over-current");

	/* Report over-current */
	regulator_notifier_call_chain(chip->otg_reg,
					REGULATOR_EVENT_OVER_CURRENT, NULL);

	/* Regulator is automatically disabled in hardware on over-current */
	regulator_notifier_call_chain(chip->otg_reg,
					REGULATOR_EVENT_DISABLE, NULL);

	return IRQ_HANDLED;
}

/* TODO: Handle all interrupts */
const struct smbchg_irq smbchg_irqs[] = {
	{ "chg-error", smbchg_handle_charger_error },
	{ "chg-inhibit", NULL },
	{ "chg-prechg-sft", NULL },
	{ "chg-complete-chg-sft", NULL },
	{ "chg-p2f-thr", smbchg_handle_p2f },
	{ "chg-rechg-thr", smbchg_handle_rechg },
	{ "chg-taper-thr", smbchg_handle_taper },
	{ "chg-tcc-thr", smbchg_handle_tcc },
	{ "batt-hot", smbchg_handle_batt_temp },
	{ "batt-warm", smbchg_handle_batt_temp },
	{ "batt-cold", smbchg_handle_batt_temp },
	{ "batt-cool", smbchg_handle_batt_temp },
	{ "batt-ov", NULL },
	{ "batt-low", NULL },
	{ "batt-missing", NULL },
	{ "batt-term-missing", NULL },
	{ "usbin-uv", NULL },
	{ "usbin-ov", NULL },
	{ "usbin-src-det", smbchg_handle_usb_source_detect },
	{ "usbid-change", smbchg_handle_usbid_change },
 	{ "otg-fail", smbchg_handle_otg_fail },
 	{ "otg-oc", smbchg_handle_otg_oc },
	{ "aicl-done", NULL },
	{ "dcin-uv", NULL },
	{ "dcin-ov", NULL },
	{ "power-ok", NULL },
	{ "temp-shutdown", NULL },
	{ "wdog-timeout", NULL },
	{ "flash-fail", NULL },
	{ "otst2", NULL },
	{ "otst3", NULL },
};

static enum power_supply_property smbchg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX
};

static int smbchg_get_charge_type(struct smbchg_chip *chip)
{
	int value, ret;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read charger status: %d\n", ret);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	value = (value & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	dev_vdbg(chip->dev, "Charge type: 0x%x", value);
	switch (value) {
	case BATT_NOT_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	case BATT_PRE_CHG_VAL:
		/* Low-current precharging */
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BATT_FAST_CHG_VAL:
		/* Constant current fast charging */
	case BATT_TAPER_CHG_VAL:
		/* Constant voltage fast charging */
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	default:
		dev_err(chip->dev,
			"Invalid charge type value 0x%x read\n", value);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int smbchg_get_health(struct smbchg_chip *chip)
{
	int value, ret;

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_BAT_IF_RT_STS,
				&value);
	if (ret) {
		dev_err(chip->dev,
			"Failed to read battery real-time status: %d\n", ret);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	if (value & HOT_BAT_HARD_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	else if (value & HOT_BAT_SOFT_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	else if (value & COLD_BAT_HARD_BIT)
		return POWER_SUPPLY_HEALTH_COLD;

	else if (value & COLD_BAT_SOFT_BIT)
		return POWER_SUPPLY_HEALTH_COOL;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int smbchg_get_status(struct smbchg_chip *chip)
{
	int value, ret, chg_type;

	/* Check if power input is present */
	/* TODO: Add DC charge path */
	if (!smbchg_usb_is_present(chip))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_RT_STS, &value);
	if (ret) {
		dev_err(chip->dev,
			"Failed to read charger real-time status: %d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	dev_vdbg(chip->dev, "Charger real-time status: 0x%x", value);

	/* Check if temination current is reached or if charging is inhibited */
	if (value & BAT_TCC_REACHED_BIT || value & CHG_INHIBIT_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read charger status: %d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	dev_vdbg(chip->dev, "Charger status: 0x%x", value);

	/* Check for charger hold-off */
	if (value & CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	chg_type = smbchg_get_charge_type(chip);
	switch (chg_type) {
	case POWER_SUPPLY_CHARGE_TYPE_UNKNOWN:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	case POWER_SUPPLY_CHARGE_TYPE_NONE:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_CHARGING;
	}
}

static int smbchg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct smbchg_chip *chip = power_supply_get_drvdata(psy);

	dev_vdbg(chip->dev, "Getting property: %d", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smbchg_get_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smbchg_get_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smbchg_get_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smbchg_usb_is_present(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->max_usb_current * 1000; // To uA
		break;
	default:
		dev_err(chip->dev, "Invalid property: %d\n", psp);
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc smbchg_usb_psy_desc = {
	.name = "qcom-smbcharger-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = smbchg_props,
	.num_properties = ARRAY_SIZE(smbchg_props),
	.get_property = smbchg_get_property,
};

static int smbchg_init(struct smbchg_chip *chip)
{
	int vfloat_mv, val, ret;

	/* TODO: Figure out what this really means and maybe improve comment */
	/*
	 * Overriding current set by APSD (Auto Power Source Detect) will
	 * prevent AICL from rerunning at 9V for HVDCPs.
	 * Use APSD mA ratings for the initial current values.
	 */
	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
				ICL_OVERRIDE_BIT, 0);
	if (ret) {
		dev_err(chip->dev, "Failed to disable ICL override: %d\n", ret);
		return ret;
	}

	/* TODO: Confirm initial configuration and correct comment */
	/*
	 * Initial charger configuration: Enable charger, automatic precharging
	 * to fast charging transition, auto recharging, current termination
	 * and charge inhibition
	 */
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_CFG2,
		CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
		| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
		CHG_EN_POLARITY_BIT | CHARGER_INHIBIT_BIT);
	if (ret) {
		dev_err(chip->dev, "Failed to configure charger: %d\n", ret);
		return ret;
	}

	/* Set termination current */
	smbchg_charging_set_iterm(chip);

	/* Enable charging */
	ret = smbchg_charging_enable(chip, true);
	if (ret) {
		dev_err(chip->dev, "Failed to enable charging: %d\n", ret);
		return ret;
	}

	/* TODO: Improve comment */
	/*
	 * Use the analog sensors instead of the fuel gauge
	 * ADC for choosing recharge threshold.
	 */
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_CFG1,
		TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
		TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	if (ret) {
		dev_err(chip->dev,
			"Failed to configure charge termination: %d\n", ret);
		return ret;
	}

	/*
	 * TODO: Figure out exactly what this does and replace downstream
	 * comment with a clearer one
	 */
	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	ret = smbchg_sec_masked_write(chip, SMBCHG_USB_CHGPTH_CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (ret) {
		dev_err(chip->dev,
			"Failed to configure USB charge path: %d\n", ret);
		return ret;
	}

	/*
	 * TODO: Handle mid and high float voltage ranges properly.
	 * Only handling very high range now (voltage_max_design_mv <= 4360).
	 */
	/* Set float voltage */
	vfloat_mv = (chip->batt_info->voltage_max_design_uv / 1000) - 4360;
	val = 0x2c + vfloat_mv / 20;
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_VFLOAT_CFG,
				VFLOAT_MASK, val);
	if (ret) {
		dev_err(chip->dev, "Failed to set float voltage: %d\n", ret);
	}

	/* TODO: Figure this out and improve comment */
	/* Enable recharge threshold */
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_CFG, RCHG_LVL_BIT, 0);
	if (ret) {
		dev_err(chip->dev,
			"Failed to enable recharge threshold: %d\n", ret);
		return ret;
	}

	/* Get initial USB type */
	chip->usb_psy_type = smbchg_usb_get_type(chip);

	/* Enable USB input */
	smbchg_usb_enable(chip, true);

	return ret;
}

static int smbchg_probe(struct platform_device *pdev)
{
	struct smbchg_chip *chip;
	struct regulator_config config = { };
	struct power_supply_config supply_config = {};
	int i, irq, ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Failed to get regmap\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(chip->dev->of_node, "reg", &chip->base);
	if (ret) {
		dev_err(chip->dev, "Failed to get base address: %d\n", ret);
		return ret;
	}

	spin_lock_init(&chip->sec_access_lock);
	INIT_WORK(&chip->otg_reset_work, smbchg_otg_reset_worker);

	/* Initialize OTG regulator */
	chip->otg_rdesc.id = -1;
	chip->otg_rdesc.name = "otg-vbus";
	chip->otg_rdesc.ops = &smbchg_otg_ops;
	chip->otg_rdesc.owner = THIS_MODULE;
	chip->otg_rdesc.type = REGULATOR_VOLTAGE;
	chip->otg_rdesc.of_match = "otg-vbus";

	config.dev = chip->dev;
	config.driver_data = chip;

	chip->otg_reg = devm_regulator_register(chip->dev, &chip->otg_rdesc,
					       &config);
	if (IS_ERR(chip->otg_reg)) {
		ret = PTR_ERR(chip->otg_reg);
		dev_err(chip->dev,
			"Failed to register OTG VBUS regulator: %d", ret);
		return ret;
	}

	chip->data = of_device_get_match_data(chip->dev);

	supply_config.drv_data = chip;
	supply_config.of_node = pdev->dev.of_node;
	chip->usb_psy = devm_power_supply_register(chip->dev, &smbchg_usb_psy_desc,
						&supply_config);
	if (IS_ERR(chip->usb_psy)) {
		ret = PTR_ERR(chip->usb_psy);
		dev_err(chip->dev,
			"Failed to register USB power supply: %d\n", ret);
		return ret;
	}

	ret = power_supply_get_battery_info(chip->usb_psy, &chip->batt_info);
	if (ret) {
		dev_err(chip->dev, "Failed to get battery info: %d", ret);
		return ret;
	}

	/* Initialize charger */
	ret = smbchg_init(chip);
	if (ret)
		return ret;

	/* Request interrupts */
	for (i = 0; i < ARRAY_SIZE(smbchg_irqs); ++i) {
		/* Skip unhandled interrupts for now */
		if (!smbchg_irqs[i].handler)
			continue;

		irq = of_irq_get_byname(pdev->dev.of_node, smbchg_irqs[i].name);
		if (irq < 0) {
			dev_err(chip->dev,
				"Failed to get %s IRQ: %d\n",
				smbchg_irqs[i].name, irq);
			return irq;
		}

		ret = devm_request_threaded_irq(chip->dev, irq, NULL,
						smbchg_irqs[i].handler,
						IRQF_ONESHOT , smbchg_irqs[i].name,
						chip);
		if (ret) {
			dev_err(chip->dev,
				"failed to request %s IRQ: %d\n",
				smbchg_irqs[i].name, irq);
			return ret;
		}
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static int smbchg_remove(struct platform_device *pdev)
{
	struct smbchg_chip *chip = platform_get_drvdata(pdev);

	smbchg_usb_enable(chip, false);
	smbchg_charging_enable(chip, false);

	return 0;
}

static const struct of_device_id smbchg_id_table[] = {
	{ .compatible = "qcom,pmi8950-smbcharger", .data = &smbchg_pmi8994_data },
	{ .compatible = "qcom,pmi8994-smbcharger", .data = &smbchg_pmi8994_data },
	{ .compatible = "qcom,pmi8996-smbcharger", .data = &smbchg_pmi8996_data },
	{ }
};
MODULE_DEVICE_TABLE(of, smbchg_id_table);

static struct platform_driver smbchg_driver = {
	.probe = smbchg_probe,
	.remove = smbchg_remove,
	.driver = {
		.name = "qcom-smbcharger",
		.of_match_table = smbchg_id_table,
	},
};
module_platform_driver(smbchg_driver);

MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_AUTHOR("Alejandro Tafalla <atafalla@dnyon.com>");
MODULE_DESCRIPTION("Qualcomm Switch-Mode Battery Charger");
MODULE_LICENSE("GPL");
