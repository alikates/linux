// SPDX-License-Identifier: GPL-2.0
/*
 * Power supply driver for Qualcomm Switch-Mode Battery Charger
 */

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

#define CMD_CHG_REG			0x242
#define OTG_EN_BIT			BIT(0)

#define SMBCHG_USB_CHGPTH_OFFSET	0x300
#define SMBCHG_DC_CHGPTH_OFFSET		0x400
#define SMBCHG_RID_STS			0xb
#define RID_MASK			GENMASK(3, 0)
#define SMBCHG_USBID_MSB		0xe
#define USBID_GND_THRESHOLD 		0x495

#define SMBCHG_MISC_OFFSET		0x600
#define SMBCHG_IDEV_STS			0x8
#define FMB_STS_MASK			GENMASK(3, 0)

enum rev_offsets {
	DIG_MINOR = 0x0,
	DIG_MAJOR = 0x1,
	ANA_MINOR = 0x2,
	ANA_MAJOR = 0x3,
};

struct smbchg_chip {
	unsigned int base;
	struct device *dev;
	struct regmap *regmap;
	u8 revision[4];

	// struct power_supply *usb_psy;
	struct power_supply *bat_psy;
	// struct power_supply *dc_psy;

	struct regulator_desc otg_rdesc;
	struct regulator_dev *otg_reg;
};

#define RT_STS 0x10
#define INPUT_STS 0x0D
#define USBIN_9V 	BIT(5)
#define USBIN_UNREG BIT(4)
#define USBIN_LV 	BIT(3)
#define USBIN_SRC_DET_BIT BIT(2)
#define USBIN_OV_BIT	BIT(1)
static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	int val;
	rc = regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + RT_STS, 
			&val);
	if (rc) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(val & USBIN_SRC_DET_BIT) || (val & USBIN_OV_BIT))
		return false;

	rc = regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + INPUT_STS, 
			&val);
	if (rc) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}
	return !!(val & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc, val;

	rc = regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + RT_STS, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return !!(val & USBIN_SRC_DET_BIT);
}

#define CHG_INHIBIT_BIT			BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
#define CHG_TYPE_MASK			GENMASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define CHG_HOLD_OFF_BIT		BIT(3)

#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3

#define CHGR_STS 				0x0E
static int smbchg_get_bat_status(struct smbchg_chip *chip)
{
	int rc, val, chg_type;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	rc = regmap_read(chip->regmap, chip->base + RT_STS, &val);
	if (rc) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
	if (val & (BAT_TCC_REACHED_BIT | CHG_INHIBIT_BIT)) {
		status =  POWER_SUPPLY_STATUS_FULL;
		goto out;
	}

	rc = regmap_read(chip->regmap, chip->base + CHGR_STS, &val);
	if (rc) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (val & CHG_HOLD_OFF_BIT) {
		/*
		 * when chg hold off happens the battery is
		 * not charging
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (val & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL /* && !chip->hvdcp_3_det_ignore_uv */)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
out:
	dev_info(chip->dev, "Battery status: %d", status);
	return status;
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		/* bit 0 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 1 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 4 error case, report DCP */
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_usb_type(struct smbchg_chip *chip)
{
	int rc, type2, reg;
	unsigned long type;

	rc = regmap_read(chip->regmap, chip->base + SMBCHG_MISC_OFFSET + 
		SMBCHG_IDEV_STS, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return -1;
	}

	type = reg;
	type >>= TYPE_BITS_OFFSET;
	type2 = find_first_bit(&type, N_TYPE_BITS);
	dev_info(chip->dev, "charger type: %d", usb_type_enum[type2]);
	return usb_type_enum[type2];
}

#define CMD_IL 0x40
#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc;

	rc = regmap_write_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET 
		+ CMD_IL, DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define USBIN_SUSPEND_BIT BIT(4)
#define CMD_CHG_REG_ALT	0x42
#define EN_BAT_CHG_BIT		BIT(1)
#define SMBCHG_BATT_OFFSET		0x200

static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc, val;

	rc = regmap_read(chip->regmap, chip->base + SMBCHG_BATT_OFFSET + 
		CMD_IL, &val);
	dev_info(chip->dev, "suspend reg status: %d\n", val);

	rc = regmap_write_bits(chip->regmap, chip->base + SMBCHG_BATT_OFFSET + 
		CMD_IL, USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);

	rc = regmap_read(chip->regmap, chip->base + SMBCHG_BATT_OFFSET + 
		CMD_IL, &val);
	dev_info(chip->dev, "suspend reg status: %d\n", val);
	return rc;
}

// static int smbchg_config_charger(struct smbchg_chip *chip)
// {

// }

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		GENMASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
							int current_ma)
{
	int rc;
	rc = regmap_write_bits(chip->regmap, chip->base + FCC_CFG,
		FCC_MASK, FCC_500MA_VAL);
	if (rc)
		dev_err(chip->dev, "Couldn't set %dmA rc=%d\n", 500, rc);	
	return rc;
}

static int smbchg_charging_en(struct smbchg_chip *chip, bool en)
{
	int ret;

	/* The en bit is configured active low */
	ret = regmap_write_bits(chip->regmap, chip->base + SMBCHG_BATT_OFFSET + 
		CMD_CHG_REG_ALT, EN_BAT_CHG_BIT, en ? 0 : EN_BAT_CHG_BIT);

	return ret;
}

irqreturn_t smbchg_handle_usb(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	bool usb_present = is_usb_present(chip);
	bool src_detect = is_src_detect_high(chip);
	dev_info(chip->dev, "Usb presence: %d", (int) usb_present);
	dev_info(chip->dev, "Detect status: %d", (int) src_detect);
	
	union power_supply_propval prv = {0, };
	prv.intval = smbchg_get_bat_status(chip);
	if(src_detect) {
		power_supply_set_property(chip->bat_psy, POWER_SUPPLY_PROP_STATUS, 
			&prv);
		smbchg_charging_en(chip, true);
		power_supply_changed(chip->bat_psy);
		get_usb_type(chip);
	}
	return IRQ_HANDLED;
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT BIT(7)
#define CFG_USB_2 0
#define USBIN_MODE_CHG_BIT BIT(0)
#define USB51_MODE_BIT BIT(1)
#define USBIN_LIMITED_MODE 0
#define USB51_500MA BIT(1)
#define CURRENT_500_MA 500

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	GENMASK(4, 0)
#define USBIN_HC_MODE BIT(0)

static int smbchg_usb_set_max_current(struct smbchg_chip *chip, int current_ma)
{
	int ret = 0;
	int val;
	if (current_ma == CURRENT_500_MA) {
		ret = regmap_write_bits(chip->regmap,
				chip->base + SMBCHG_USB_CHGPTH_OFFSET + CHGPTH_CFG,
				CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		if (ret) {
			dev_err(chip->dev, "Couldn't set CHGPTH_CFG ret = %d\n", ret);
			return ret;
		}

		regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + 
			CHGPTH_CFG, &val);
	
		dev_info(chip->dev, "CHGPTH_CFG reg: %d", val);

		ret = regmap_write_bits(chip->regmap,
				chip->base + SMBCHG_USB_CHGPTH_OFFSET + CMD_IL,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				USBIN_LIMITED_MODE | USB51_500MA);
		if (ret) {
			dev_err(chip->dev, "Couldn't set CMD_IL ret = %d\n", ret);
			return ret;
		}

		int val = FCC_500MA_VAL & DCIN_INPUT_MASK;
		ret = regmap_write_bits(chip->regmap, chip->base + 
			SMBCHG_USB_CHGPTH_OFFSET + IL_CFG, DCIN_INPUT_MASK, val);
		ret = regmap_write_bits(chip->regmap, chip->base + 
		SMBCHG_USB_CHGPTH_OFFSET + CMD_IL, USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
		if (ret)
			dev_err(chip->dev, "Couldn't write cfg 5 ret = %d\n", ret);
		// chip->usb_max_current_ma = 500;
	}
	return ret;
}

static int smbchg_dc_set_max_current(struct smbchg_chip *chip)
{
	int rc;
	int val = FCC_500MA_VAL & DCIN_INPUT_MASK;
	rc = regmap_write_bits(chip->regmap, chip->base + SMBCHG_DC_CHGPTH_OFFSET 
		+ IL_CFG, DCIN_INPUT_MASK, val);
	if (rc)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	return rc;
}

// irqreturn_t smbchg_handle_batt_hot()

// static irqreturn_t chg_term_handler(int irq, void *_chip);
// static irqreturn_t src_detect_handler(int irq, void *_chip);
// static irqreturn_t aicl_done_handler(int irq, void *_chip);	

static int smbchg_otg_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	int ret;

	dev_dbg(chip->dev, "enabling OTG VBUS regulator");

	ret = regmap_update_bits(chip->regmap, chip->base + CMD_CHG_REG,
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

	ret = regmap_update_bits(chip->regmap, chip->base + CMD_CHG_REG,
				OTG_EN_BIT, 0);
	if(ret)
		dev_err(chip->dev, "failed to disable OTG regulator: %d", ret);
	return ret;
}

static int smbchg_otg_is_enabled(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);
	unsigned int value = 0;
	int ret;

	ret = regmap_read(chip->regmap, chip->base + CMD_CHG_REG, &value);
	if (ret)
		dev_err(chip->dev, "failed to read CHG_REG\n");

	return !!(value & OTG_EN_BIT);
}

static const struct regulator_ops smbchg_otg_ops = {
	.enable = smbchg_otg_enable,
	.disable = smbchg_otg_disable,
	.is_enabled = smbchg_otg_is_enabled,
};

static bool smbchg_is_otg_present(struct smbchg_chip *chip)
{
	u32 value;
	int ret;

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_MISC_OFFSET +
				SMBCHG_IDEV_STS,
				&value);
	if(ret < 0) {
		dev_err(chip->dev, "failed to read IDEV_STS: %d\n", ret);
		return false;
	}

	if ((value & FMB_STS_MASK) != 0) {
		dev_dbg(chip->dev, "IDEV_STS = 0x%02x, not ground\n", value);
		return false;
	}

	ret = regmap_bulk_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET +
				SMBCHG_USBID_MSB,
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

	ret = regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET +
				SMBCHG_RID_STS,
				&value);
	if(ret < 0) {
		dev_err(chip->dev, "failed to read RID_STS: %d\n", ret);
		return false;
	}

	dev_dbg(chip->dev, "RID_STS = 0x%02x\n", value);

	return (value & RID_MASK) == 0;
}

irqreturn_t smbchg_handle_usbid_change(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	bool otg_present;

	dev_info(chip->dev, "usbid change IRQ triggered\n");

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
	dev_dbg(chip->dev, "OTG present: %d\n", otg_present);

	return IRQ_HANDLED;
}

#define CHGR_CFG2 0xFC
#define USE_REGISTER_FOR_CURRENT BIT(2)
#define CHG_EN_SRC_BIT BIT(7)
#define CHG_EN_POLARITY_BIT BIT(6)
#define P2F_CHG_TRAN BIT(5)
#define AUTO_RECHG_BIT BIT(2)
#define USBIN_CHGR_CFG 0xF1

#define TR_8OR32B 0xFE
#define BUCK_8_16_FREQ_BIT BIT(0)

static int smbchg_init(struct smbchg_chip *chip)
{
	int rc, val;

	rc = regmap_write_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET 	
		+ USBIN_CHGR_CFG, 0xFF, 0x00);
	if (rc)
		dev_err(chip->dev, "Couldn't set only 5v OVP 6.4V rc=%d\n", rc);

	/*
	 * Do not force using current from the register i.e. use auto
	 * power source detect (APSD) mA ratings for the initial current values.
	 *
	 * If this is set, AICL will not rerun at 9V for HVDCPs
	 */
	rc = regmap_write_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + CMD_IL, USE_REGISTER_FOR_CURRENT, 0);

	if (rc) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	regmap_read(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + CMD_IL, 
		&val);
	dev_info(chip->dev, "register for current: %d\n", val);


	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable auto recharge by default.
	 * enable current termination and charge inhibition based on
	 * the device tree configuration.
	 */
	rc = regmap_write_bits(chip->regmap, chip->base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
			| AUTO_RECHG_BIT,
			CHG_EN_POLARITY_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/* make the buck switch faster to prevent some vbus oscillation */
	rc = regmap_write_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + 
		TR_8OR32B, BUCK_8_16_FREQ_BIT, 0);
	if (rc) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	regmap_read(chip->regmap, chip->base + CHGR_CFG2, &val);
	dev_info(chip->dev, "register for current: %d\n", val);

	return rc;
}

#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)

static int smbchg_probe(struct platform_device *pdev)
{
	struct smbchg_chip *chip;
	struct regulator_config config = { };
	int ret, irq;

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

	ret = regmap_bulk_read(chip->regmap, chip->base + SMBCHG_MISC_OFFSET,
				&chip->revision, 4);
	if(ret < 0) {
		dev_err(&pdev->dev, "failed to read revision: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

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

	ret = smbchg_init(chip);
	if (ret) {
		dev_err(chip->dev, "failed to init charger: %d\n", ret);
		return ret;
	}
	
	smbchg_usb_set_max_current(chip, 500);
	smbchg_dc_set_max_current(chip);
	smbchg_set_fastchg_current_raw(chip, 0);

	/*
	 * enable battery charging to make sure it hasn't been changed earlier
	 * by the bootloader.
	 */
	ret = smbchg_charging_en(chip, true);
	if (ret) {
		dev_err(chip->dev, "Couldn't enable battery charging=%d\n", ret);
		return ret;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	ret = regmap_write_bits(chip->regmap, chip->base + SMBCHG_USB_CHGPTH_OFFSET + CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (ret) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", ret);
		return ret;
	}

	smbchg_dc_suspend(chip, 0);
	smbchg_usb_suspend(chip, 0);

	/* Get the battery from FG driver*/
	chip->bat_psy = devm_power_supply_get_by_phandle(chip->dev, "fuel-gauge");
	if(!chip->bat_psy) {
		dev_err(chip->dev, "failed to get battery psy: %d\n", ret);
		return ret;
	}
	
	/* Interrupts */
	// for (i = 0; i < ARRAY_SIZE(smbchg_charger_irqs); ++i) {
	// 	int irq;

	// 	irq = platform_get_irq_byname(pdev, smbb_charger_irqs[i].name);
	// 	if (irq < 0) {
	// 		dev_err(&pdev->dev, "failed to get irq '%s'\n",
	// 			smbb_charger_irqs[i].name);
	// 		return irq;
	// 	}

	// 	smbb_charger_irqs[i].handler(irq, chg);

	// 	rc = devm_request_threaded_irq(&pdev->dev, irq, NULL,
	// 			smbb_charger_irqs[i].handler, IRQF_ONESHOT,
	// 			smbb_charger_irqs[i].name, chg);
	// 	if (rc) {
	// 		dev_err(&pdev->dev, "failed to request irq '%s'\n",
	// 			smbb_charger_irqs[i].name);
	// 		return rc;
	// 	}
	// }

	irq = of_irq_get_byname(pdev->dev.of_node, "usbid-change");
	if (irq < 0) {
		dev_err(&pdev->dev, "Couldn't get usbid-change IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					smbchg_handle_usbid_change,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT, "usbid-change", chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request usbid-change IRQ: %d\n", irq);
		return ret;
	}

	irq = of_irq_get_byname(pdev->dev.of_node, "usb-change");
	if (irq < 0) {
		dev_err(&pdev->dev, "Couldn't get usb-change IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					smbchg_handle_usb,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT, "usb-change", chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request usb-change IRQ: %d\n", irq);
		return ret;
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static int smbchg_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id smbchg_id_table[] = {
	{ .compatible = "qcom,pmi8994-smbcharger" },
	{ .compatible = "qcom,pmi8950-smbcharger" },
	{ }
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
