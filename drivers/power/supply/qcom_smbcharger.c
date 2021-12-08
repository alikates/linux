// SPDX-License-Identifier: GPL-2.0
/*
 * Power supply driver for Qualcomm Switch-Mode Battery Charger
 */

#include <asm-generic/errno-base.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>

#include <linux/power/qcom_smbcharger.h>

struct smbchg_chip {
	unsigned int base;
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *usb_psy;
	struct power_supply_battery_info batt_info;

	struct regulator_desc otg_rdesc;
	struct regulator_dev *otg_reg;

	int otg_resets;

	spinlock_t sec_access_lock;
	struct work_struct otg_reset_work;

	int max_usb_current;
	enum power_supply_type usb_psy_type;

	struct delayed_work voltage_adjust;

	const struct smbchg_ver_cfg *chg_cfg;
};

struct smbchg_ver_cfg {
	const int *term_current_table;
	unsigned int term_current_table_len;
	const int *current_limit_table;
	unsigned int current_limit_table_len;
};

struct smbchg_irq {
	const char *name;
	irq_handler_t handler;
};

static int find_closest_in_array(const int *arr, int len, int val)
{
	int i, closest = 0;

	if (len == 0)
		return closest;
	for (i = 0; i < len; i++)
		if (abs(val - arr[i]) < abs(val - arr[closest]))
			closest = i;

	return closest;
}

static int __maybe_unused find_smaller_in_array(const int *table, int len,
						int val)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}

	return i;
}

static int smbchg_sec_write(struct smbchg_chip *chip, u8 *val, u16 addr,
			    int len)
{
	u8 sec_addr_val = 0xa5;
	int ret;

	ret = regmap_bulk_write(chip->regmap,
			((chip->base + addr) & 0xff00) | 0xd0,
			&sec_addr_val, 1);
	if (ret)
		return ret;

	return regmap_bulk_write(chip->regmap, chip->base + addr, val, len);
}

/* TODO: Use and remove __maybe_unused */
static __maybe_unused int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 addr, u8 mask, u8 val)
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

static int smbchg_otg_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(chip->dev, "enabling OTG VBUS regulator");

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, OTG_EN_BIT);
	if(ret)
		dev_err(chip->dev, "failed to enable OTG regulator: %d", ret);

	return ret;
}

static int smbchg_otg_disable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(chip->dev, "disabling OTG VBUS regulator");

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, 0);
	if (ret) {
		dev_err(chip->dev, "failed to disable OTG regulator: %d", ret);
		return ret;
	}

	return 0;
}

static int smbchg_otg_is_enabled(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	unsigned int value = 0;
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

static bool smbchg_is_usb_present(struct smbchg_chip *chip)
{
	u32 value;
	int ret;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_USB_CHGPTH_RT_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read RT_STS: %d\n", ret);
		return false;
	}

	if (!(value & USBIN_SRC_DET_BIT) || (value & USBIN_OV_BIT))
		return false;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_USB_CHGPTH_INPUT_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read INPUT_STS: %d\n", ret);
		return false;
	}

	return !!(value & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static int smbchg_get_charge_type(struct smbchg_chip *chip)
{
	int value, ret;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read CHGR_STS: %d\n", ret);
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
		dev_err(chip->dev, "Failed to read battery status: %d\n", ret);
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
	if (!smbchg_is_usb_present(chip))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_RT_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read RT_STS: %d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	dev_vdbg(chip->dev, "Charger rt status: 0x%x", value);

	/* Check if charge temination is reached */
	if (value & BAT_TCC_REACHED_BIT || value & CHG_INHIBIT_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read CHGR_STS: %d\n", ret);
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

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		/* bit 0 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 1 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 4 error case, report DCP */
};

static enum power_supply_type smbchg_get_usb_type(struct smbchg_chip *chip)
{
	int rc, type2, reg;
	unsigned long type;

	rc = regmap_read(chip->regmap, chip->base + SMBCHG_MISC_IDEV_STS, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return -1;
	}

	type = reg;
	type >>= TYPE_BITS_OFFSET;
	type2 = find_first_bit(&type, N_TYPE_BITS);
	dev_dbg(chip->dev, "charger type: %d", usb_type_enum[type2]);
	return usb_type_enum[type2];
}

static bool smbchg_is_otg_present(struct smbchg_chip *chip)
{
	u32 value;
	int ret;

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_MISC_IDEV_STS,	&value);
	if(ret < 0) {
		dev_err(chip->dev, "failed to read IDEV_STS: %d\n", ret);
		return false;
	}

	if ((value & FMB_STS_MASK) != 0) {
		dev_dbg(chip->dev, "IDEV_STS = 0x%02x, not ground\n", value);
		return false;
	}

	ret = regmap_bulk_read(chip->regmap,
				chip->base + SMBCHG_USB_CHGPTH_USBID_MSB,
				&value, 2);
	if(ret < 0) {
		dev_err(chip->dev, "failed to read USBID_MSB: %d\n", ret);
		return false;
	}

	if (value > USBID_GND_THRESHOLD) {
		dev_dbg(chip->dev, "USBID = 0x%04x, too high to be ground\n",
				value);
		return false;
	}

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_RID_STS,
				&value);
	if(ret < 0) {
		dev_err(chip->dev, "failed to read RID_STS: %d\n", ret);
		return false;
	}

	dev_dbg(chip->dev, "RID_STS = 0x%02x\n", value);

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
		dev_err(chip->dev, "failed to disable OTG regulator: %d\n", ret);
		return;
	}

	msleep(20);
	/*
	 * Only re-enable the OTG regulator if OTG is still present
	 * after sleeping
	 */
	if (!smbchg_is_otg_present(chip))
		return;

	ret = regmap_update_bits(chip->regmap,
				chip->base + SMBCHG_BAT_IF_CMD_CHG,
				OTG_EN_BIT, OTG_EN_BIT);
	if (ret)
		dev_err(chip->dev, "failed to enable OTG regulator: %d\n", ret);
}


static int smbchg_set_term_current(struct smbchg_chip *chip)
{
	int ret, val;

	val = find_closest_in_array(chip->chg_cfg->term_current_table,
				    chip->chg_cfg->term_current_table_len,
				    chip->batt_info.charge_term_current_ua /
					    1000);
	dev_dbg(chip->dev, "Setting termination current to %dmA",
		chip->chg_cfg->term_current_table[val]);
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_TCC_CFG, CHG_ITERM_MASK,
				      val);
	if (ret) {
		dev_err(chip->dev, "Couldn´t set termination current");
	}
	return ret;
}

static int smbchg_usb_set_current_bits(struct smbchg_chip *chip, int usb_type,
				       int usb_current)
{
	int ret;
	ret = smbchg_sec_masked_write(chip, SMBCHG_USB_CHGPTH_CHGPTH_CFG,
				      CFG_USB_2_3_SEL_BIT, usb_type);
	if (ret) {
		dev_err(chip->dev, "Couldn't set CHGPTH_CFG, ret = %d\n", ret);
		return ret;
	}
	ret = regmap_update_bits(chip->regmap,
				 chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
				 USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				 USBIN_LIMITED_MODE | usb_current);
	if (ret) {
		dev_err(chip->dev, "Couldn't set CMD_IL ret = %d\n", ret);
	}
	return ret;
}

// Set current for high-power dedicated usb charger
static int smbchg_set_usb_chg_high_current(struct smbchg_chip *chip,
					   int current_ma)
{
	int ret, usb_cur_val, i;
	i = find_smaller_in_array(chip->chg_cfg->current_limit_table,
				  chip->chg_cfg->current_limit_table_len,
				  current_ma);
	if (i < 0) {
		//Error, set min current (100ma)
		ret = smbchg_usb_set_current_bits(chip, CFG_USB_2, USB51_100MA);
		return ret;
}
	usb_cur_val = i & USBIN_INPUT_MASK;
	ret = smbchg_sec_masked_write(chip, SMBCHG_USB_CHGPTH_IL_CFG,
				      USBIN_INPUT_MASK, usb_cur_val);
	if (ret < 0) {
		dev_err(chip->dev, "cannot write to config c ret = %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(chip->regmap,
				 chip->base + SMBCHG_USB_CHGPTH_IL_CFG,
				 USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (ret < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 ret = %d\n", ret);

	chip->max_usb_current = chip->chg_cfg->current_limit_table[i];
	dev_dbg(chip->dev, "High power USB current limit set to %dmA",
		chip->max_usb_current);
	return ret;
}

/* Set the maximum allowed current. Can be used to change current depending on
 * the temps
 */
static int smbchg_usb_set_max_current(struct smbchg_chip *chip, int current_ma)
{
	int usb_type, usb_current;
	int ret = 0;
	/* TODO: Maybe disable lowering current when no battery is present to
	 * prevent the device from powering off (same as downstream)
	*/

	if (current_ma < SUSPEND_CURRENT_MA) {
		//suspend
		chip->max_usb_current = 0;
		goto out;
	}

	switch(chip->usb_psy_type){
	case POWER_SUPPLY_TYPE_USB:
		if(current_ma > 900) {
			//High power SDP, clamp to 500ma
			// TODO: fix
			current_ma = 500;
		}

		switch (current_ma) {
		case 150:
			usb_type = CFG_USB_3;
			usb_current = USB51_100MA;
			break;
		case 500:
			usb_type = CFG_USB_2;
			usb_current = USB51_500MA;
			break;
		case 900:
			usb_type = CFG_USB_3;
			usb_current = USB51_500MA;
			break;
		default:
			current_ma = 100;
			usb_type = CFG_USB_2;
			usb_current = USB51_100MA;
			break;
		}
		smbchg_usb_set_current_bits(chip, usb_type, usb_current);
		chip->max_usb_current = current_ma;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		// (i.e High power USB from modern laptop)
		if (current_ma < 1500) {
			/* port announced less than 1500 ma
				use override for CDP */
			ret = regmap_update_bits(
				chip->regmap,
					chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (ret)
				dev_err(chip->dev,
					"Couldn't set override ret = %d\n",
					ret);
		}
		/* fall through */
	case POWER_SUPPLY_TYPE_USB_DCP: //Standard DCP
		chip->max_usb_current = current_ma;
		ret = smbchg_set_usb_chg_high_current(chip, current_ma);
		/* if (ret)
			dev_err(chip->dev, "Couldn't set %dmA ret = %d\n", current_ma, ret); */
		break;
	default:
		dev_err(chip->dev,
			"Unknow USB type, not setting current limits");
		chip->max_usb_current = 0;
		return -ENOSYS;
	}
out:
	if(!ret)
		dev_dbg(chip->dev, "Set USB current to %dmA", chip->max_usb_current);
	return ret;
}


static int smbchg_usb_set_current(struct smbchg_chip *chip)
{
	enum power_supply_type psy_type = smbchg_get_usb_type(chip);
	int ret = 0;
	switch (psy_type) {
	case POWER_SUPPLY_TYPE_USB:
		ret = smbchg_usb_set_max_current(chip, DEFAULT_SDP_MA);
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		ret = smbchg_usb_set_max_current(chip, DEFAULT_DCP_MA);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		ret = smbchg_usb_set_max_current(chip, DEFAULT_CDP_MA);
		break;
	default:
		break;
	}
	return ret;
}

static void smbchg_charging_adjust_voltage(struct work_struct *work)
{
	struct smbchg_chip *chip;
	chip = container_of(work, struct smbchg_chip, voltage_adjust.work);

	//Adjust voltage
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	return -ENOSYS;
}

irqreturn_t smbchg_handle_charger_error(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	int ret, reg;

	dev_err(chip->dev, "Charger error");

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_CHGR_RT_STS, &reg);
	if (ret < 0) {
		dev_err(chip->dev, "Unable to read RT_STS, rc = %d\n", ret);
	} else {
		dev_dbg(chip->dev, "irq triggered: 0x%02x\n", reg);
		/* if (reg & CHG_COMP_SFT_BIT) {
			power_supply_set_property(chip->fg_psy,
				POWER_SUPPLY_PROP_SAFETY_TIMER_EXPIRED, 1);
		} */
	}

	smbchg_charging_status_change(chip);
	/* TODO: Handle errors properly */
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_batt_temp(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	power_supply_changed(chip->usb_psy);

	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_otg_fail(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_err(chip->dev, "OTG failure");

	/* TODO: Handle OTG failure properly */
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_p2f(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	int reg;

	dev_dbg(chip->dev, "P2F triggered");
	regmap_read(chip->regmap, chip->base + SMBCHG_CHGR_RT_STS, &reg);
	dev_dbg(chip->dev, "triggered: 0x%02x\n", reg);
	smbchg_get_status(chip);

	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_rechg(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	int reg;

	dev_dbg(chip->dev, "Rechg triggered");
	regmap_read(chip->regmap, chip->base + SMBCHG_CHGR_RT_STS, &reg);
	dev_dbg(chip->dev, "triggered: 0x%02x\n", reg);
	smbchg_get_status(chip);

	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_taper(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	int reg;

	dev_dbg(chip->dev, "Taper triggered");
	regmap_read(chip->regmap, chip->base + SMBCHG_CHGR_RT_STS, &reg);
	dev_dbg(chip->dev, "triggered: 0x%02x\n", reg);
	smbchg_get_status(chip);
	//smbchg_parallel_usb_taper(chip);

	power_supply_changed(chip->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_tcc(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	int reg;

	dev_dbg(chip->dev, "TCC triggered");
	regmap_read(chip->regmap, chip->base + SMBCHG_CHGR_RT_STS, &reg);
	dev_dbg(chip->dev, "triggered: 0x%02x\n", reg);
	smbchg_get_status(chip);

	power_supply_changed(chip->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_otg_oc(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	/*
	 * Inrush current of some devices can trip the over-current protection
	 * on the PMI8994 smbcharger due to a hardware bug. Try resetting the
	 * OTG regulator, and only report over-current if it persists.
	 */
	if (chip->otg_resets < SMBCHG_NUM_OTG_RESET_RETRIES) {
		schedule_work(&chip->otg_reset_work);
		chip->otg_resets++;
		return IRQ_HANDLED;
	}

	dev_warn(chip->dev, "OTG over-current");

	/* Report over-current */
	regulator_notifier_call_chain(chip->otg_reg,
					REGULATOR_EVENT_OVER_CURRENT, NULL);

	chip->otg_resets = 0;

	return IRQ_HANDLED;
}

static int smbchg_usb_enable(struct smbchg_chip *chip, bool enable)
{
	int rc;

	rc = regmap_update_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_CMD_IL,
			USBIN_SUSPEND_BIT, enable ? 0 : USBIN_SUSPEND_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	dev_dbg(chip->dev, "Charger %s\n", enable ? "enabled" : "disabled");
	return rc;
}

irqreturn_t smbchg_handle_usb_source_detect(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	bool usb_present;

	usb_present = smbchg_is_usb_present(chip);
	dev_dbg(chip->dev, "USB %spresent\n", usb_present ? "" : "not ");

	power_supply_changed(chip->usb_psy);
	chip->usb_psy_type = smbchg_get_usb_type(chip);

	/* TODO: Handle USB input source changes properly */
	if (usb_present)
		smbchg_usb_set_current(chip);
	smbchg_charging_status_change(chip);
	smbchg_usb_enable(chip, usb_present);
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_usbid_change(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	bool otg_present;

	/*
	 * After the falling edge of the usbid change interrupt occurs,
	 * there may still be some time before the ADC conversion for USB RID
	 * finishes in the fuel gauge. In the worst case, this could be up to
	 * 15 ms.
	 *
	 * Wait for the conversion to finish and the USB RID status register
	 * to be updated before trying to detect OTG insertions.
	 */

	msleep(20);

	otg_present = smbchg_is_otg_present(chip);
	dev_dbg(chip->dev, "OTG %spresent\n", otg_present ? "" : "not ");

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
	{ "otg-oc", NULL },
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
		/* TODO: Add other charge paths (DC and WiPower) */
		val->intval = smbchg_is_usb_present(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->max_usb_current * 1000; //To uA
		break;
	default:
		dev_err(chip->dev, "Invalid property: %d\n", psp);
		return -EINVAL;
	}

	return 0;
}

static int smbchg_charging_enable(struct smbchg_chip *chip, bool enable)
{
	int ret = regmap_update_bits(chip->regmap, chip->base + SMBCHG_BAT_IF_CMD_CHG,
		EN_BAT_CHG_BIT, enable ? 0 : EN_BAT_CHG_BIT);
	if (ret < 0) {
		dev_err(chip->dev, "Couldn't enable battery charging=%d\n", ret);
	}
	return ret;
}

static const struct power_supply_desc smbchg_usb_psy_desc = {
	.name = "qcom-smbcharger-usb",
	/* TODO: Maybe use POWER_SUPPLY_TYPE_USB */
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = smbchg_props,
	.num_properties = ARRAY_SIZE(smbchg_props),
	.get_property = smbchg_get_property,
};

static int smbchg_init(struct smbchg_chip *chip)
{
	int ret, vfloat_mv, temp;
	/*
	 * Do not force using current from the register i.e. use auto
	 * power source detect (APSD) mA ratings for the initial current values.
	 *
	 * If this is set, AICL will not rerun at 9V for HVDCPs
	 */
	ret = regmap_update_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_CMD_IL, USE_REGISTER_FOR_CURRENT, 0);

	if (ret < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd ret=%d\n", ret);
		return ret;
	}

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable auto recharge by default.
	 * enable current termination and charge inhibition based on
	 * the device tree configuration.
	 */
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_CFG2,
		CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
		| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
		CHG_EN_POLARITY_BIT | CHARGER_INHIBIT_BIT);
	if (ret < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", ret);
		return ret;
	}

	smbchg_set_term_current(chip);

	/*
	 * enable battery charging to make sure it hasn't been changed earlier
	 * by the bootloader.
	 */
	smbchg_charging_enable(chip, true);

	/*
	 * Use the analog sensors instead of the fuelgauge
	 * adc for recharge threshold source.
	 */
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_CFG1,
		TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
		TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);

	if (ret < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", ret);
		return ret;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	ret = smbchg_sec_masked_write(chip, SMBCHG_USB_CHGPTH_CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (ret < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg ret=%d\n", ret);
		return ret;
	}

	/* Set charge voltage, TODO: fix constant values */
	vfloat_mv = (chip->batt_info.voltage_max_design_uv / 1000) - 4360;
	temp = 0x2C + vfloat_mv / 20;
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_VFLOAT_CFG, VFLOAT_MASK,
				      temp);

	if (ret) {
		dev_err(chip->dev, "Couldn't set float voltage ret = %d\n", ret);
	}

	/* Enable recharge threshold */
	ret = smbchg_sec_masked_write(chip, SMBCHG_CHGR_CFG, RCHG_LVL_BIT, 0);
	if (ret < 0) {
		dev_err(chip->dev, "Couldn't set recharge ret = %d\n", ret);
		return ret;
	}

	chip->usb_psy_type = smbchg_get_usb_type(chip);
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

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "reg", &chip->base);
	if (ret) {
		dev_err(&pdev->dev, "missing or invalid 'reg' property\n");
		return ret;
	}

	/* OTG regulator */
	chip->otg_rdesc.id = -1;
	chip->otg_rdesc.name = "otg-vbus";
	chip->otg_rdesc.ops = &smbchg_otg_ops;
	chip->otg_rdesc.owner = THIS_MODULE;
	chip->otg_rdesc.type = REGULATOR_VOLTAGE;
	chip->otg_rdesc.of_match = "otg-vbus";

	config.dev = &pdev->dev;
	config.driver_data = chip;

	chip->otg_reg = devm_regulator_register(&pdev->dev, &chip->otg_rdesc,
					       &config);
	if (IS_ERR(chip->otg_reg)) {
		ret = PTR_ERR(chip->otg_reg);
		dev_err(chip->dev, "failed to register OTG VBUS regulator: %d", ret);
		return ret;
	}

	spin_lock_init(&chip->sec_access_lock);
	INIT_WORK(&chip->otg_reset_work, smbchg_otg_reset_worker);

	supply_config.drv_data = chip;
	supply_config.of_node = pdev->dev.of_node;
	chip->usb_psy = devm_power_supply_register(chip->dev, &smbchg_usb_psy_desc,
						&supply_config);
	if (IS_ERR(chip->usb_psy)) {
		ret = PTR_ERR(chip->usb_psy);
		dev_err(&pdev->dev, "Failed to register power supply: %d\n", ret);
		return ret;
	}

	ret = power_supply_get_battery_info(chip->usb_psy, &chip->batt_info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get battery info");
		return ret;
	}

	/* TODO: maybe of_device_get_match_data??? */
	chip->chg_cfg = device_get_match_data(&pdev->dev);

	ret = smbchg_init(chip);
	if (ret) {
		return ret;
	}

	INIT_DELAYED_WORK(&chip->voltage_adjust,
			  smbchg_charging_adjust_voltage);
	schedule_delayed_work(&chip->voltage_adjust, usecs_to_jiffies(100000));

	/* Interrupts */
	for (i = 0; i < ARRAY_SIZE(smbchg_irqs); ++i) {
		/* Skip unhandled interrupts for now */
		if (!smbchg_irqs[i].handler)
			continue;

		irq = of_irq_get_byname(pdev->dev.of_node, smbchg_irqs[i].name);
		if (irq < 0) {
			dev_err(&pdev->dev,
				"Failed to get %s IRQ: %d\n",
				smbchg_irqs[i].name, irq);
			return irq;
		}

		ret = devm_request_threaded_irq(chip->dev, irq, NULL,
						smbchg_irqs[i].handler,
						IRQF_ONESHOT , smbchg_irqs[i].name,
						chip);
		if (ret < 0) {
			dev_err(&pdev->dev,
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

static const struct smbchg_ver_cfg smbchg_pmi8994_cfg = {
	.term_current_table = iterm_ma_table_8994,
	.term_current_table_len = ARRAY_SIZE(iterm_ma_table_8994),
	.current_limit_table = usb_ilim_ma_table_8994,
	.current_limit_table_len = ARRAY_SIZE(usb_ilim_ma_table_8994)
};

static const struct smbchg_ver_cfg smbchg_pmi8996_cfg = {
	.term_current_table = iterm_ma_table_8996,
	.term_current_table_len = ARRAY_SIZE(iterm_ma_table_8996),
	.current_limit_table = usb_ilim_ma_table_8996,
	.current_limit_table_len = ARRAY_SIZE(usb_ilim_ma_table_8996)
};

static const struct of_device_id smbchg_id_table[] = {
	{ .compatible = "qcom,pmi8950-smbcharger",
	  .data = &smbchg_pmi8994_cfg },
	{ .compatible = "qcom,pmi8994-smbcharger",
	  .data = &smbchg_pmi8994_cfg },
	{ .compatible = "qcom,pmi8996-smbcharger",
	  .data = &smbchg_pmi8996_cfg },
	{}
};
MODULE_DEVICE_TABLE(of, smbchg_id_table);

static struct platform_driver smbchg_driver = {
	.probe = smbchg_probe,
	.remove = smbchg_remove,
	.driver = {
		.name   = "qcom-smbcharger",
		.of_match_table = smbchg_id_table,
	},
};
module_platform_driver(smbchg_driver);

MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_DESCRIPTION("Qualcomm Switch-Mode Battery Charger");
MODULE_LICENSE("GPL");
