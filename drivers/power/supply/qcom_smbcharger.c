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

	struct regulator_desc otg_rdesc;
	struct regulator_dev *otg_reg;
};

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

	/* Interrupts */
	irq = of_irq_get_byname(pdev->dev.of_node, "usbid-change");
	if (irq < 0) {
		dev_err(&pdev->dev, "Couldn't get usbid-change IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					smbchg_handle_usbid_change,
					IRQF_ONESHOT, "usbid-change", chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request usbid-change IRQ: %d\n", irq);
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
