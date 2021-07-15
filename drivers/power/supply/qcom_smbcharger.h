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

/* Bits */
/* CHGR_STS bits */
#define CHG_TYPE_MASK			GENMASK(2, 1)
#define CHG_HOLD_OFF_BIT		BIT(3)

/* CHGR_RT_STS bits */
#define CHG_INHIBIT_BIT			BIT(1)
#define CHG_COMP_SFT_BIT		BIT(3)
#define BAT_TCC_REACHED_BIT		BIT(7)

/* CHGR_VFLOAT_CFG bits */
#define VFLOAT_MASK			GENMASK(5, 0)

/* CHGR_TCC_CFG bits */
#define CHG_ITERM_MASK			GENMASK(2, 0)

/* CHGR_CFG1 bits */
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)

/* CHGR_CFG2 bits */
#define CHARGER_INHIBIT_BIT		BIT(0)
#define AUTO_RECHG_BIT			BIT(2)
#define I_TERM_BIT			BIT(3)
#define P2F_CHG_TRAN			BIT(5)
#define CHG_EN_POLARITY_BIT		BIT(6)
#define CHG_EN_SRC_BIT			BIT(7)

/* CHGR_CFG bits */
#define RCHG_LVL_BIT			BIT(0)

/* BAT_IF_CMD_CHG bits */
#define OTG_EN_BIT			BIT(0)
#define CHG_EN_BIT			BIT(1)

/* BAT_IF_RT_STS bits */
#define HOT_BAT_HARD_BIT		BIT(0)
#define HOT_BAT_SOFT_BIT		BIT(1)
#define COLD_BAT_HARD_BIT		BIT(2)
#define COLD_BAT_SOFT_BIT		BIT(3)
#define BAT_OV_BIT			BIT(4)
#define BAT_LOW_BIT			BIT(5)
#define BAT_MISSING_BIT			BIT(6)
#define BAT_TERM_MISSING_BIT		BIT(7)

/* USB_CHGPTH_CHGPTH_CFG bits */
#define USB51AC_CTRL			BIT(1)
#define USB51_COMMAND_POL		BIT(2)
#define USB_2_3_SEL_BIT			BIT(7)

/* USB_CHGPTH_RT_STS bits */
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)

/* USB_CHGPTH_RID_STS bits */
#define RID_MASK			GENMASK(3, 0)

/* USB_CHGPTH_CMD_IL bits */
#define USBIN_MODE_CHG_BIT		BIT(0)
#define USB51_MODE_BIT			BIT(1)
#define ICL_OVERRIDE_BIT		BIT(2)
#define USBIN_SUSPEND_BIT		BIT(4)

/* SMBCHG_USB_CHGPTH_IL_CFG bits */
#define USBIN_INPUT_MASK		GENMASK(4, 0)

/* MISC_IDEV_STS bits */
#define FMB_STS_MASK			GENMASK(3, 0)

/* Values */
/* CHGR_STS values */
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3

/* CHGR_CFG1 values */
#define TERM_SRC_FG			BIT(2)

/* USB_CHGPTH_INPUT_STS values */
#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)

/* USB_CHGPTH_USBID_MSB values */
#define USBID_GND_THRESHOLD 		0x495

/* USB_CHGPTH_CHGPTH_CFG values */
#define USB_2				0
#define USB_3				BIT(7)

/* USB_CHGPTH_CMD_IL values */
#define USBIN_LIMITED_MODE		0
#define	USBIN_HC_MODE			USBIN_MODE_CHG_BIT
#define USB51_100MA			0
#define USB51_500MA			USB51_MODE_BIT

/* MISC_IDEV_STS values */
#define USB_TYPE_SDP_BIT		BIT(4)
#define USB_TYPE_OTHER_BIT		BIT(5)
#define USB_TYPE_DCP_BIT		BIT(6)
#define USB_TYPE_CDP_BIT		BIT(7)

/* Constant parameters */
#define SUSPEND_CURRENT_MA		2
#define DEFAULT_SDP_MA			500
#define DEFAULT_DCP_MA			1000
#define DEFAULT_CDP_MA			1500
#define NUM_OTG_RESET_RETRIES		5
#define OTG_RESET_DELAY_MS		20

struct smbchg_chip {
	unsigned int base;
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *usb_psy;
	struct power_supply_battery_info batt_info;

	struct regulator_desc otg_rdesc;
	struct regulator_dev *otg_reg;
	int otg_resets;

	int max_usb_current;
	enum power_supply_type usb_psy_type;

	spinlock_t sec_access_lock;
	struct work_struct otg_reset_work;

	const struct smbchg_data *data;
};

struct smbchg_irq {
	const char *name;
	irq_handler_t handler;
};

struct smbchg_data {
	const int *iterm_table;
	unsigned int iterm_table_len;
	const int *ilim_usb_table;
	unsigned int ilim_usb_table_len;
};

static const int smbchg_iterm_table_pmi8994[] = {
	50,
	100,
	150,
	200,
	250,
	300,
	500,
	600
};

static const int smbchg_ilim_usb_table_pmi8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static const int smbchg_iterm_table_pmi8996[] = {
	50,
	100,
	150,
	200,
	250,
	300,
	400,
	500
};

static const int smbchg_ilim_usb_table_pmi8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000
};

static const struct smbchg_data smbchg_pmi8994_data = {
	.iterm_table = smbchg_iterm_table_pmi8994,
	.iterm_table_len = ARRAY_SIZE(smbchg_iterm_table_pmi8994),
	.ilim_usb_table = smbchg_ilim_usb_table_pmi8994,
	.ilim_usb_table_len = ARRAY_SIZE(smbchg_ilim_usb_table_pmi8994)
};

static const struct smbchg_data smbchg_pmi8996_data = {
	.iterm_table = smbchg_iterm_table_pmi8996,
	.iterm_table_len = ARRAY_SIZE(smbchg_iterm_table_pmi8996),
	.ilim_usb_table = smbchg_ilim_usb_table_pmi8996,
	.ilim_usb_table_len = ARRAY_SIZE(smbchg_ilim_usb_table_pmi8996)
};

/* TODO: Replace with find_closest or add find_smaller to util_macros */
static inline int find_smaller_in_array(const int *table, int len,
				int val)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}

	return i;
}
