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

/* Registers */
/* CHGR */
#define SMBCHG_CHGR_FV_STS			0x0c
#define SMBCHG_CHGR_STS				0x0e
#define SMBCHG_CHGR_RT_STS			0x10
#define SMBCHG_CHGR_FCC_CFG			0xf2
#define SMBCHG_CHGR_FCC_CMP_CFG			0xf3
#define SMBCHG_CHGR_VFLOAT_CFG			0xf4
#define SMBCHG_CHGR_FV_CMP			0xf5
#define SMBCHG_CHGR_AFVC_CFG			0xf6
#define SMBCHG_CHGR_CHG_INHIB_CFG		0xf7
#define SMBCHG_CHGR_TCC_CFG			0xf9
#define SMBCHG_CHGR_CCMP_CFG			0xfa
#define SMBCHG_CHGR_CFG1			0xfb
#define SMBCHG_CHGR_CFG2			0xfc
#define SMBCHG_CHGR_SFT_CFG			0xfd
#define SMBCHG_CHGR_CFG				0xff

/* OTG */
#define SMBCHG_OTG_RT_STS			0x110
#define SMBCHG_OTG_OTG_CFG			0x1f1
#define SMBCHG_OTG_TRIM6			0x1f6
#define SMBCHG_OTG_LOW_PWR_OPTIONS		0x1ff

/* BAT-IF */
#define SMBCHG_BAT_IF_BAT_PRES_STS		0x208
#define SMBCHG_BAT_IF_RT_STS			0x210
#define SMBCHG_BAT_IF_CMD_CHG			0x242
#define SMBCHG_BAT_IF_CMD_CHG_LED		0x243
#define SMBCHG_BAT_IF_BM_CFG			0x2f3
#define SMBCHG_BAT_IF_BAT_IF_TRIM7		0x2f7
#define SMBCHG_BAT_IF_BB_CLMP_SEL		0x2f8
#define SMBCHG_BAT_IF_ZIN_ICL_PT		0x2fc
#define SMBCHG_BAT_IF_ZIN_ICL_LV		0x2fd
#define SMBCHG_BAT_IF_ZIN_ICL_HV		0x2fe

/* USB-CHGPTH */
#define SMBCHG_USB_CHGPTH_ICL_STS_1		0x307
#define SMBCHG_USB_CHGPTH_PWR_PATH		0x308
#define SMBCHG_USB_CHGPTH_ICL_STS_2		0x309
#define SMBCHG_USB_CHGPTH_RID_STS		0x30b
#define SMBCHG_USB_CHGPTH_USBIN_HVDCP_STS	0x30c
#define SMBCHG_USB_CHGPTH_INPUT_STS		0x30d
#define SMBCHG_USB_CHGPTH_USBID_MSB		0x30e
#define SMBCHG_USB_CHGPTH_RT_STS		0x310
#define SMBCHG_USB_CHGPTH_CMD_IL		0x340
#define SMBCHG_USB_CHGPTH_CMD_APSD		0x341
#define SMBCHG_USB_CHGPTH_CMD_HVDCP_1		0x342
#define SMBCHG_USB_CHGPTH_CMD_HVDCP_2		0x343
#define SMBCHG_USB_CHGPTH_USBIN_CHGR_CFG	0x3f1
#define SMBCHG_USB_CHGPTH_IL_CFG		0x3f2
#define SMBCHG_USB_CHGPTH_AICL_CFG		0x3f3
#define SMBCHG_USB_CHGPTH_CHGPTH_CFG		0x3f4
#define SMBCHG_USB_CHGPTH_APSD_CFG		0x3f5
#define SMBCHG_USB_CHGPTH_TR_RID		0x3fa
#define SMBCHG_USB_CHGPTH_ICL_BUF_CONFIG	0x3fc
#define SMBCHG_USB_CHGPTH_TR_8OR32B		0x3fe

/* DC-CHGPTH */
#define SMBCHG_DC_CHGPTH_RT_STS			0x410
#define SMBCHG_DC_CHGPTH_IL_CFG			0x4f2
#define SMBCHG_DC_CHGPTH_AICL_CFG		0x4f3
#define SMBCHG_DC_CHGPTH_AICL_WL_SEL_CFG	0x4f5

/* MISC */
#define SMBCHG_MISC_REVISION1			0x600
#define SMBCHG_MISC_IDEV_STS			0x608
#define SMBCHG_MISC_RT_STS			0x610
#define SMBCHG_MISC_TEST			0x6e2
#define SMBCHG_MISC_NTC_VOUT_CFG		0x6f3
#define SMBCHG_MISC_TRIM_OPT_15_8		0x6f5
#define SMBCHG_MISC_TRIM_OPT_7_0		0x6f6
#define SMBCHG_MISC_TRIM_14			0x6fe

/* TODO: Sort bits and bitmasks */
#define OTG_EN_BIT			BIT(0)

#define RID_MASK			GENMASK(3, 0)

#define FMB_STS_MASK			GENMASK(3, 0)

#define BB_LOOP_DISABLE_ICL		BIT(2)

/* CHGR_STS values */
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			GENMASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3

/* CHGR_RT_STS bits */
#define CHG_INHIBIT_BIT			BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)

/* BAT_IF_RT_STS bits */
#define HOT_BAT_HARD_BIT		BIT(0)
#define HOT_BAT_SOFT_BIT		BIT(1)
#define COLD_BAT_HARD_BIT		BIT(2)
#define COLD_BAT_SOFT_BIT		BIT(3)
#define BAT_OV_BIT			BIT(4)
#define BAT_LOW_BIT			BIT(5)
#define BAT_MISSING_BIT			BIT(6)
#define BAT_TERM_MISSING_BIT		BIT(7)

/* USB_CHGPTH_RT_STS bits */
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)

#define SMBCHG_NUM_OTG_RESET_RETRIES	5
#define USBID_GND_THRESHOLD 	0x495

struct smbchg_chip {
	unsigned int base;
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *usb_psy;

	struct regulator_desc otg_rdesc;
	struct regulator_dev *otg_reg;

	int otg_resets;

	struct work_struct otg_reset_work;
};

struct smbchg_irq {
	const char *name;
	irq_handler_t handler;
};

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

irqreturn_t smbchg_handle_charger_error(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_err(chip->dev, "Charger error");

	/* TODO: Handle errors properly */
	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_batt_temp(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

irqreturn_t smbchg_handle_otg_fail(int irq, void *data)
{
	struct smbchg_chip *chip = data;

	dev_err(chip->dev, "OTG failure");

	/* TODO: Handle OTG failure properly */
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

irqreturn_t smbchg_handle_usb_source_detect(int irq, void *data)
{
	struct smbchg_chip *chip = data;
	bool usb_present;

	usb_present = smbchg_is_usb_present(chip);
	dev_dbg(chip->dev, "USB %spresent\n", usb_present ? "" : "not ");

	power_supply_changed(chip->usb_psy);

	/* TODO: Handle USB input source changes properly */
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
	{ "chg-p2f-thr", NULL },
	{ "chg-rechg-thr", NULL },
	{ "chg-taper-thr", NULL },
	{ "chg-tcc-thr", NULL },
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

	/* Check if charge temination is reached */
	if (value & BAT_TCC_REACHED_BIT || value & CHG_INHIBIT_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	ret = regmap_read(chip->regmap,
			chip->base + SMBCHG_CHGR_STS, &value);
	if (ret) {
		dev_err(chip->dev, "Failed to read CHGR_STS: %d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

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

static enum power_supply_property smbchg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE
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
	default:
		dev_err(chip->dev, "Invalid property: %d\n", psp);
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc smbchg_usb_psy_desc = {
	.name = "qcom-smbcharger-usb",
	/* TODO: Maybe use POWER_SUPPLY_TYPE_USB */
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = smbchg_props,
	.num_properties = ARRAY_SIZE(smbchg_props),
	.get_property = smbchg_get_property,
};

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

	INIT_WORK(&chip->otg_reset_work, smbchg_otg_reset_worker);

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
						IRQF_ONESHOT, smbchg_irqs[i].name,
						chip);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to request %s IRQ: %d\n",
				smbchg_irqs[i].name, irq);
			return ret;
		}
	}

	supply_config.drv_data = chip;
	supply_config.of_node = pdev->dev.of_node;
	chip->usb_psy = devm_power_supply_register(chip->dev, &smbchg_usb_psy_desc,
						&supply_config);
	if (IS_ERR(chip->usb_psy)) {
		ret = PTR_ERR(chip->usb_psy);
		dev_err(&pdev->dev, "Failed to register power supply: %d\n", ret);
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
