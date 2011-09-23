/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/powerkey.h>
#include <linux/modem.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/mxc_edid.h>
#include <mach/iomux-mx51.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <linux/switch.h>
#include <linux/i2c/hmc5883.h>
#include <linux/timed_gpio.h>
#include <linux/input.h>
#include <linux/wlan_plat.h>
#include <linux/kxtf9.h>
#include <linux/mc13892_power_supply.h>
#include <linux/bq24103_charger.h>
#include <linux/battery_device_info.h>
#include <linux/gps/gps.h>

#ifdef CONFIG_PM_CHECK
#include <linux/pm_check.h>
#endif

#include "devices.h"
#include "crm_regs.h"
#include "usb.h"

/*!
 * @file mach-mx51/mx51_tulip.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */

/* IOMUX to GPIO */
#define TULIP_SD_MMC_SD       (0*32 + 0)  /* GPIO_1_0 */
#define TULIP_SD1_WP          (0*32 + 1)  /* GPIO_1_1 */
#define TULIP_SYS_ON_OFF_CTL  (0*32 + 3)  /* GPIO_1_3 */
#define TULIP_WDOG_B          (0*32 + 4)  /* GPIO_1_4 */
#define TULIP_INT_FROM_PMIC   (0*32 + 8)  /* GPIO_1_8 */
#define TULIP_TOUCH_INT       (0*32 + 9)  /* GPIO_1_9 */
#define TULIP_3G_RST          (0*32 + 10) /* GPIO_1_10 */
#define TULIP_GPS_RST         (0*32 + 19) /* GPIO_1_19 */
#define TULIP_GPS_PWR_ON      (0*32 + 29) /* GPIO_1_29 */
#define TULIP_HOST_WAKEUP_WLAN (0*32 + 30) /* GPIO_1_30 */
#define TULIP_HOST_WAKEUP_BT  (0*32 + 31) /* GPIO_1_31 */

#define TULIP_AC_IN           (1*32 + 1)  /* GPIO_2_1 */
#define TULIP_CHG_OK          (1*32 + 2)  /* GPIO_2_2 */
#define TULIP_RST_USB_PHY_B   (1*32 + 5)  /* GPIO_2_5 */
#define TULIP_EIM_A16		   (1*32 + 10) /* GPIO_2_10 */
#define TULIP_EIM_A17		   (1*32 + 11) /* GPIO_2_10 */
#define TULIP_LID_CLOSE_B     (1*32 + 18) /* GPIO_2_18 */
#define TULIP_LOW_BAT_WAKEUP  (1*32 + 19) /* GPIO_2_19 */
#define TULIP_SYS_ON_OFF_REQ  (1*32 + 21) /* GPIO_2_21 */
#define TULIP_G_SENSOR_INT    (1*32 + 22) /* GPIO_2_22 */
#define TULIP_COMPASS_INT     (1*32 + 25) /* GPIO_2_25 */

#define TULIP_VCC_LCD_3V7_EN  (2*32 + 0)  /* GPIO_3_0 */
#define TULIP_LCD_LDO_EN      (2*32 + 1)  /* GPIO_3_1 */
#define TULIP_3G_WAKEUP_IN    (2*32 + 2)  /* GPIO_3_2 */
#define TULIP_TOUCH_REST      (2*32 + 3)  /* GPIO_3_3 */
#define TULIP_MIC_S           (2*32 + 5)  /* GPIO_3_5 */ 
#define TULIP_CSI1_PWDN       (2*32 + 6)  /* GPIO_3_6 */
#define TULIP_RST_USB_HUB_B   (2*32 + 7)  /* GPIO_3_7 */
#define TULIP_LVDS_SHTDN      (2*32 + 8)  /* GPIO_3_8 */
#define TULIP_VCAM_EN         (2*32 + 13) /* GPIO_3_13 */
#define TULIP_AMP_SHUT        (2*32 + 29) /* GPIO_3_29 */
#define TULIP_HPHONE_DET      (2*32 + 30) /* GPIO_3_30 */
#define TULIP_VIBRATOR_ON     (2*32 + 31) /* GPIO_3_31 */

#define TULIP_EN_WIFI_PWR      (3*32 + 2)  /* GPIO_4_2 */
#define TULIP_BT_RST           (3*32 + 3)  /* GPIO_4_3 */
#define TULIP_WL_RST           (3*32 + 4)  /* GPIO_4_4 */
#define TULIP_BT_WAKEUP_HOST   (3*32 + 5)  /* GPIO_4_5 */
#define TULIP_WLAN_WAKEUP_HOST (3*32 + 6)  /* GPIO_4_6 */
#define TULIP_TOUCH_EN         (3*32 + 8)  /* GPIO_4_8 */
#define TULIP_LVDS_RSVD        (3*32 + 9)  /* GPIO_4_9 */
#define TULIP_3G_WAKEUP_OUT    (3*32 + 10) /* GPIO_4_10 */
#define TULIP_3G_SIM_DET       (3*32 + 11) /* GPIO_4_11 */
#define TULIP_3G_PWR_EN        (3*32 + 12) /* GPIO_4_12 */
#define TULIP_3G_POWER_ON_OFF  (3*32 + 13) /* GPIO_4_13 */
#define TULIP_CSP1_SS0_GPIO    (3*32 + 24) /* GPIO_4_24 */

extern int __init mx51_tulip_init_mc13892(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
extern struct dvfs_wp *(*get_dvfs_core_wp)(int *wp);

static int num_cpu_wp;

static void psy_register_device(void);

#ifdef CONFIG_PM_CHECK
static struct gpio_check tulip_gpio_check[] = {
	{TULIP_SD_MMC_SD, 1},
	{TULIP_SD1_WP, 0},
	{TULIP_SYS_ON_OFF_CTL, 1},
	{TULIP_WDOG_B, 1},
	{TULIP_INT_FROM_PMIC, 1},
	{TULIP_TOUCH_INT, 1},
	{TULIP_3G_RST, 1},
	{TULIP_GPS_RST, 1},
	{TULIP_GPS_PWR_ON, 1},
	{TULIP_HOST_WAKEUP_WLAN, 1},
	{TULIP_HOST_WAKEUP_BT, 1},
	{TULIP_AC_IN, 1},
	{TULIP_CHG_OK, 1},
	{TULIP_RST_USB_PHY_B, 1},
	{TULIP_EIM_A16, 1},
	{TULIP_EIM_A17, 1},
	{TULIP_LID_CLOSE_B, 1},
	{TULIP_LOW_BAT_WAKEUP, 1},
	{TULIP_SYS_ON_OFF_REQ, 1},
	{TULIP_G_SENSOR_INT, 1},
	{TULIP_COMPASS_INT, 1},
	{TULIP_VCC_LCD_3V7_EN, 1},
	{TULIP_LCD_LDO_EN, 1},
	{TULIP_3G_WAKEUP_IN, 1},
	{TULIP_TOUCH_REST, 1},
	{TULIP_MIC_S, 1},
	{TULIP_CSI1_PWDN, 1},
	{TULIP_RST_USB_HUB_B, 1},
	{TULIP_LVDS_SHTDN, 1},
	{TULIP_VCAM_EN, 1},
	{TULIP_AMP_SHUT, 1},
	{TULIP_HPHONE_DET, 1},
	{TULIP_VIBRATOR_ON, 1},
	{TULIP_EN_WIFI_PWR, 1},
	{TULIP_BT_RST, 1},
	{TULIP_WL_RST, 1},
	{TULIP_BT_WAKEUP_HOST, 1},
	{TULIP_WLAN_WAKEUP_HOST, 1},
	{TULIP_TOUCH_EN, 1},
	{TULIP_LVDS_RSVD, 1},
	{TULIP_3G_WAKEUP_OUT, 1},
	{TULIP_3G_SIM_DET, 1},
	{TULIP_3G_PWR_EN, 1},
	{TULIP_3G_POWER_ON_OFF, 1},
	{TULIP_CSP1_SS0_GPIO, 1},
	{-1, 0},
};
#endif

static iomux_v3_cfg_t mx51tulip_pads[] = {
	/* charger ac_in */
	MX51_PAD_EIM_D17__GPIO2_1,
	/* charger ok */
	MX51_PAD_EIM_D18__GPIO2_2,

	/* RST_USB_PHY_B, ALT1 gpio2_5 */
	MX51_PAD_EIM_D21__GPIO2_5,

	/* HOST_WAKEUP_WLAN, to BCM4329, ALT5 gpio1_30 */
	 MX51_PAD_DISP2_DAT8__GPIO1_30,

	/* WLAN_WAKEUP_HOST, from BCM4329, ALT3 gpio4_6*/
	MX51_PAD_NANDF_D2__GPIO4_6,

	/*EN_WIFI_PWR,ALT3 gpio4_2*/
	 MX51_PAD_NANDF_D6__GPIO4_2,

	/* WL_RST, ALT3 gpio4_4 */
	 MX51_PAD_NANDF_D4__GPIO4_4,


	/* uart 3 */
	MX51_PAD_EIM_D24__UART3_CTS,
	MX51_PAD_EIM_D25__UART2_CTS,
	MX51_PAD_EIM_D26__UART2_RTS,
	MX51_PAD_EIM_D27__UART3_RTS,

	/* LID close */
	MX51_PAD_EIM_A24__GPIO2_18,

	/* low battery wake up */
	MX51_PAD_EIM_A25__GPIO2_19,

	/* CSI1_DATA_EN */
	MX51_PAD_EIM_A26__GPIO2_20,

	/* SYS_ON_OFF_REQ */
	MX51_PAD_EIM_A27__GPIO2_21,

	/* G-SENSOR_INT*/
	MX51_PAD_EIM_EB2__GPIO2_22,

	/* COMPASS_INT */
	MX51_PAD_EIM_CS0__GPIO2_25,

	/* Three gpio gpio 2_27:28 for HW version detect */
	MX51_PAD_EIM_CS2__GPIO2_27,
	MX51_PAD_EIM_CS3__GPIO2_28,
	MX51_PAD_EIM_CS4__GPIO2_29,

	/* TOUCH_EN */
	MX51_PAD_NANDF_D0__GPIO4_8,

	/* BT_WAKEUP_HOST */
	MX51_PAD_NANDF_D3__GPIO4_5,

	/* BT_RST */
	MX51_PAD_NANDF_D5__GPIO4_3,

	/* VIBRATOR_ON */
	MX51_PAD_NANDF_D9__GPIO3_31,

	/* HEADPHONE DETECT In-high, out-low */
	_MX51_PAD_NANDF_D10__GPIO3_30 | MUX_PAD_CTRL(MX51_GPIO_PAD_CTRL_PULL | PAD_CTL_PUS_100K_UP),

	/* AMP_SHUT */
	MX51_PAD_NANDF_D11__GPIO3_29,

	/* VCAM_EN */
	MX51_PAD_CSI1_D9__GPIO3_13,

	/* CAMERA CSI1_D12 - CSI1_D19 */
	MX51_PAD_CSI1_D12__CSI1_D12,
	MX51_PAD_CSI1_D13__CSI1_D13,
	MX51_PAD_CSI1_D14__CSI1_D14,
	MX51_PAD_CSI1_D15__CSI1_D15,
	MX51_PAD_CSI1_D16__CSI1_D16,
	MX51_PAD_CSI1_D17__CSI1_D17,
	MX51_PAD_CSI1_D18__CSI1_D18,
	MX51_PAD_CSI1_D19__CSI1_D19,

	MX51_PAD_CSI1_VSYNC__CSI1_VSYNC,
	MX51_PAD_CSI1_HSYNC__CSI1_HSYNC,
	MX51_PAD_CSI1_PIXCLK__CSI1_PIXCLK,
	MX51_PAD_CSI1_MCLK__CSI1_MCLK,

	/* LVDS_RSVD */
	MX51_PAD_CSI2_D12__GPIO4_9,

	/* 3G_WAKEUP_OUT */
	MX51_PAD_CSI2_D13__GPIO4_10,

	/* 3G_SIM_DET */
	MX51_PAD_CSI2_D18__GPIO4_11,

	/* 3G_PWR_EN */
	MX51_PAD_CSI2_D19__GPIO4_12,

	/* 3G_Power_ON_OFF */
	MX51_PAD_CSI2_VSYNC__GPIO4_13,

	/* VCC_LCD_3V7_EN */
	MX51_PAD_DI1_PIN11__GPIO3_0,

	/* LCD_LDO_EN */
	MX51_PAD_DI1_PIN12__GPIO3_1,

	/* 3G_WAKEUP_IN */
	MX51_PAD_DI1_PIN13__GPIO3_2,

	/* TOUCH_REST */
	MX51_PAD_DI1_D0_CS__GPIO3_3,

	/* MIC_Select */
	MX51_PAD_DISPB2_SER_DIN__GPIO3_5,

	/* CSI1_PWDN */
	MX51_PAD_DISPB2_SER_DIO__GPIO3_6,

	/* RST_USB_HUB_B */
	MX51_PAD_DISPB2_SER_CLK__GPIO3_7,

	/* LVDS_SHTDN */
	MX51_PAD_DISPB2_SER_RS__GPIO3_8,

	/* GPS_RST */
	MX51_PAD_DISP2_DAT6__GPIO1_19,

	/* GPS_ON_OFF */
	MX51_PAD_DISP2_DAT7__GPIO1_29,

	/* HOST_WAKEUP_BT */
	MX51_PAD_DISP2_DAT9__GPIO1_31,

	/* 3G_RST */
	MX51_PAD_DISP2_DAT11__GPIO1_10,

	/* USB host1 */
	/* USB host1, USB3317 STP */
	MX51_PAD_USBH1_STP__USBH1_STP,
	MX51_PAD_USBH1_CLK__USBH1_CLK,
	MX51_PAD_USBH1_DIR__USBH1_DIR,
	MX51_PAD_USBH1_NXT__USBH1_NXT,
	MX51_PAD_USBH1_DATA0__USBH1_DATA0,
	MX51_PAD_USBH1_DATA1__USBH1_DATA1,
	MX51_PAD_USBH1_DATA2__USBH1_DATA2,
	MX51_PAD_USBH1_DATA3__USBH1_DATA3,
	MX51_PAD_USBH1_DATA4__USBH1_DATA4,
	MX51_PAD_USBH1_DATA5__USBH1_DATA5,
	MX51_PAD_USBH1_DATA6__USBH1_DATA6,
	MX51_PAD_USBH1_DATA7__USBH1_DATA7,

	/* SDHC1, micro SD card slot */
	MX51_PAD_SD1_CMD__SD1_CMD,
	MX51_PAD_SD1_CLK__SD1_CLK,
	MX51_PAD_SD1_DATA0__SD1_DATA0,
	MX51_PAD_SD1_DATA1__SD1_DATA1,
	MX51_PAD_SD1_DATA2__SD1_DATA2,
	MX51_PAD_SD1_DATA3__SD1_DATA3,

	/* SDHC2, BCM4329 SDIO */
	MX51_PAD_SD2_CMD__SD2_CMD,
	MX51_PAD_SD2_CLK__SD2_CLK,
	MX51_PAD_SD2_DATA0__SD2_DATA0,
	MX51_PAD_SD2_DATA1__SD2_DATA1,
	MX51_PAD_SD2_DATA2__SD2_DATA2,
	MX51_PAD_SD2_DATA3__SD2_DATA3,

	/* SDHC3 - eMMC on board */
	MX51_PAD_NANDF_CS7__SD3_CLK,
	MX51_PAD_NANDF_RDY_INT__SD3_CMD,
	MX51_PAD_NANDF_D15__SD3_DAT7,
	MX51_PAD_NANDF_D14__SD3_DAT6,
	MX51_PAD_NANDF_D13__SD3_DAT5,
	MX51_PAD_NANDF_D12__SD3_DAT4,
	MX51_PAD_NANDF_RB0__SD3_DATA3,
	MX51_PAD_NANDF_WP_B__SD3_DATA2,
	MX51_PAD_NANDF_RE_B__SD3_DATA1,
	MX51_PAD_NANDF_WE_B__SD3_DATA0,

	/* Audio */
	MX51_PAD_AUD3_BB_TXD__AUD3_TXD,
	MX51_PAD_AUD3_BB_RXD__AUD3_RXD,
	MX51_PAD_AUD3_BB_CK__AUD3_TXC,
	MX51_PAD_AUD3_BB_FS__AUD3_TXFS,

	/* SPI_SS1_NORFLASH */
	MX51_PAD_CSPI1_SS1__ECSPI1_SS1,

	/* UART1 */
	MX51_PAD_UART1_RXD__UART1_RXD,
	MX51_PAD_UART1_TXD__UART1_TXD,
	MX51_PAD_UART1_RTS__UART1_RTS,
	MX51_PAD_UART1_CTS__UART1_CTS,

	/* UART2*/
	MX51_PAD_UART2_RXD__UART2_RXD,
	MX51_PAD_UART2_TXD__UART2_TXD,

	/* UART3 */
	MX51_PAD_UART3_RXD__UART3_RXD,
	MX51_PAD_UART3_TXD__UART3_TXD,
	
	/* OWIRE_LINE */
	MX51_PAD_OWIRE_LINE__OWIRE_LINE,

	/* KEY MATRIX 2*3 */
	MX51_PAD_KEY_ROW0__KEY_ROW0,
	MX51_PAD_KEY_ROW1__KEY_ROW1,
	MX51_PAD_KEY_COL0__KEY_COL0,
	MX51_PAD_KEY_COL1__KEY_COL1,
	MX51_PAD_KEY_COL2__KEY_COL2,

	/* i2c1 */
	MX51_PAD_EIM_D16__I2C1_SDA,
	MX51_PAD_EIM_D19__I2C1_SCL,

	/* I2C2 */
	MX51_PAD_KEY_COL4__I2C2_SCL,
	MX51_PAD_KEY_COL5__I2C2_SDA,

	/* SD_MMC_CD_B */
	MX51_PAD_GPIO1_0__GPIO1_0,

	/* SD1_WP */
	MX51_PAD_GPIO1_1__GPIO1_1,

	/* PWM1_OUT */
	MX51_PAD_GPIO1_2__PWM1_PWMO,

	/* SYS_ON_OFF_CTL */
	MX51_PAD_GPIO1_3__GPIO1_3,

	/* WDOG_B */
	MX51_PAD_GPIO1_4__GPIO1_4,

	/* INT_FROM_PMIC */
	MX51_PAD_GPIO1_8__GPIO1_8,

	/* TOUCH INT */
	MX51_PAD_GPIO1_9__GPIO1_9,

	MX51_PAD_EIM_A16__GPIO2_10,
	MX51_PAD_EIM_A17__GPIO2_11,

	/* END */
};

static struct dvfs_wp dvfs_core_setpoint[] = {
	{33, 13, 33, 10, 10, 0x08}, /* 800MHz*/
	{28, 8, 33, 10, 10, 0x08},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x08},   /* 160MHz*/
	{28, 8, 33, 20, 30, 0x08},   /*160MHz, AHB 133MHz, LPAPM mode*/
	{29, 0, 33, 20, 10, 0x08},}; /* 160MHz, AHB 24MHz */

/* CPU working point(wp) */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,
	 .enable = 0,
	 .name = "1G",},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,
	 .enable = 1,
	 .name = "800M",},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 400000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 1,
	 .cpu_voltage = 950000,
	 .enable = 0,
	 .name = "400M",},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,
	 .enable = 1,
	 .name = "166M",},
};

static struct dvfs_wp *mx51_tulip_get_dvfs_core_table(int *wp)
{
	*wp = ARRAY_SIZE(dvfs_core_setpoint);
	return dvfs_core_setpoint;
}

struct cpu_wp *mx51_tulip_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_tulip_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static u16 keymapping[6] = {
	KEY_HOME, KEY_MENU, KEY_BACK, KEY_VOLUMEUP,
	KEY_VOLUMEDOWN, KEY_0,
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 2,
	.colmax = 3,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
	//.trigger_key0 = KEY_VOLUMEUP,
	//.trigger_key1 = KEY_VOLUMEDOWN,
	//.trigger_key2 = KEY_MENU,
};

static struct mxc_pwm_platform_data mxc_pwm1_platform_data = {
	.pwmo_invert = 0,
	.enable_pwm_pad = NULL,
	.disable_pwm_pad = NULL,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.iram_enable = false,
	.iram_size = 0x14000,
	.reset = mx5_vpu_reset,
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx51_tulip_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			iomux_v3_cfg_t cspi1_ss0 = MX51_PAD_CSPI1_SS0__ECSPI1_SS0;

			mxc_iomux_v3_setup_pad(cspi1_ss0);
			break;
			}
		case 0x2:
			{
			iomux_v3_cfg_t cspi1_ss0_gpio = MX51_PAD_CSPI1_SS0__GPIO4_24;

			mxc_iomux_v3_setup_pad(cspi1_ss0_gpio);
			gpio_request(TULIP_CSP1_SS0_GPIO, "cspi1-gpio");
			gpio_direction_output(TULIP_CSP1_SS0_GPIO, 0);
			gpio_set_value(TULIP_CSP1_SS0_GPIO, 1 & (~status));
			break;
			}
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static void mx51_tulip_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			gpio_free(TULIP_CSP1_SS0_GPIO);
			break;

		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx51_tulip_gpio_spi_chipselect_active,
	.chipselect_inactive = mx51_tulip_gpio_spi_chipselect_inactive,
};

static struct imxi2c_platform_data mxci2c_data = {
	.bitrate = 100000,
};

static struct mxc_i2c_platform_data mxci2c_hs_data = {
	.i2c_clk = 400000,
};

#if 0
static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
};
#endif

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static struct mxc_dvfsper_data dvfs_per_data = {
	.reg_id = "SW2",
	.clk_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.gpc_adu = 0x0,
	.vai_mask = MXC_DVFSPMCR0_FSVAI_MASK,
	.vai_offset = MXC_DVFSPMCR0_FSVAI_OFFSET,
	.dvfs_enable_bit = MXC_DVFSPMCR0_DVFEN,
	.irq_mask = MXC_DVFSPMCR0_FSVAIM,
	.div3_offset = 0,
	.div3_mask = 0x7,
	.div3_div = 2,
	.lp_high = 1250000,
	.lp_low = 1250000,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

#if 0
/* see "Documentation/fb/framebuffer.txt" chapter "5. Video Mode Timings" */
struct fb_videomode {
	const char *name;	/* optional */
	u32 refresh;		/* optional */
	u32 xres;
	u32 yres;
	u32 pixclock;		/* pixel clock in ps (pico seconds)	*/
	u32 left_margin;	/* time from hsync to picture		*/
	u32 right_margin;	/* time from picture to next hsync	*/
	u32 upper_margin;	/* lines from vsync to picture		*/
	u32 lower_margin;	/* lines from picture to next vsync	*/
	u32 hsync_len;		/* length of horizontal sync		*/
	u32 vsync_len;		/* length of vertical sync		*/
	u32 sync;		/* see FB_SYNC_*			*/
	u32 vmode;		/* see FB_VMODE_*			*/
	u32 flag;
};
#endif
static struct fb_videomode video_modes[] = {
	{
	 /* 1024x600 @ 60 Hz , pixel clk @ 133M/3 = 44.333MHz / 22557 ps */
	 "HSD070PFW1", 60, 1024, 600, 22557, 84, 88, 8, 6, 4, 2,
	 0,	/* FB_SYNC_CLK_LAT_FALL, */
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1024x600 @ 60 Hz , pixel clk @ 133M/3 = 44.333MHz / 22557 ps */
	 "HV070WSA-100", 60, 1024, 600, 22557, 84, 88, 8, 6, 4, 2,
	 0,	/* FB_SYNC_CLK_LAT_FALL, */
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static void lcd_suspend(void)
{
	printk(KERN_DEBUG "tulip: lcd suspend\n");

	/* PULL LCD POWER ON to LOW to shut down LCD */
	gpio_set_value(TULIP_LCD_LDO_EN, 0);
	msleep(100);
	gpio_set_value(TULIP_LVDS_SHTDN, 0);
	msleep(1);
}

static void lcd_resume(void)
{
	printk(KERN_DEBUG "tulip: lcd resume\n");

	/* PULL LCD POWER ON to HIGH to Light LCD on */
	gpio_set_value(TULIP_LCD_LDO_EN, 1);
	msleep(100);
	gpio_set_value(TULIP_LVDS_SHTDN, 1);
	msleep(2);
}

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "HSD070PFW1",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 .suspend = lcd_suspend,
	 .resume = lcd_resume,
	 },
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* Enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	/* Disable fuse blown */
	if (!ccm_base)
		return;

	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX51_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX51_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx51_tulip())
		return 0;

	gpio_set_value(TULIP_LCD_LDO_EN, 1);
	msleep(100);
	gpio_set_value(TULIP_LVDS_SHTDN, 1);
	msleep(2);

	printk(KERN_INFO "DI0 is primary\n");

	/* DI0 -> DP-BG channel: */
	mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = mxcfb_resources;
	mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static void ov26xx_pwdn(int pwdn)
{
    gpio_set_value(TULIP_CSI1_PWDN, pwdn & 0x01);
}

static struct hmc5883_platform_data hmc5883_data = {
#if 0
	//point to south
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
#endif
#if 1
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
#endif
#if 0
	//point to south
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
#endif
#if 1
	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 0,
#endif
};

static struct mxc_camera_platform_data camera_data = {
    .mclk = 24000000,
    .csi = 0,
    .pwdn = ov26xx_pwdn,
};

static int tulip_chg_is_cable_in(void);
static int tulip_is_charge_ok(void);
#ifdef CONFIG_BATTERY_BQ27210
static struct battery_platform_data bq27210_plat_data = {
	.bat_voltage_of_zero_uV = 7000000,
	.is_online = tulip_chg_is_cable_in,
	.is_charging_full = tulip_is_charge_ok,
	.psy_register_device = psy_register_device,
};
#endif

#ifdef CONFIG_BATTERY_DS2778
static struct battery_platform_data ds2778_plat_data = {
	.bat_voltage_of_zero_uV = 7000000,
	.is_online = tulip_chg_is_cable_in,
	.is_charging_full = tulip_is_charge_ok,
	.psy_register_device = psy_register_device,
};
#endif

static struct bq24103_platform_data  bq24103_pdata = {
	.is_online = tulip_chg_is_cable_in,
	.irq = gpio_to_irq(TULIP_AC_IN),
};

static struct platform_device bq24103_charger = {
	.name = "bq24103-charger",
	.id   = 1,
	.dev  = {
		.platform_data = &bq24103_pdata,
	},
};

struct kxtf9_platform_data kxtf9_data = {
	.min_interval   = 1,
	.poll_interval  = 10,

	.g_range    = KXTF9_G_8G,
	.shift_adj  = SHIFT_ADJ_2G,

	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,

	.negate_x   = 1,
	.negate_y   = 0,
	.negate_z   = 0,

	.data_odr_init      = ODR12_5F,
	.ctrl_reg1_init     = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init      = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
	.tilt_timer_init    = 0x03,
	.engine_odr_init    = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init     = 0x16,
	.wuf_thresh_init    = 0x28,
	.tdt_timer_init     = 0x78,
	.tdt_h_thresh_init  = 0xFF,
	.tdt_l_thresh_init  = 0x14,
	.tdt_tap_timer_init = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = TULIP_G_SENSOR_INT,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	/* 1.8V */
	{	/* g-sensor */
	.type = "kxtf9",
	.addr = 0x0f,
	.platform_data = &kxtf9_data,
	//.irq  = gpio_to_irq(TULIP_G_SENSOR_INT),
	},
	{	/* compass */
	.type = "hmc5883",
	.addr = 0x1e,
	.platform_data = &hmc5883_data,
	.irq  = gpio_to_irq(TULIP_COMPASS_INT),
	}, 

#ifdef CONFIG_BATTERY_BQ27210
	{
	.type = "bq27210",
	.addr = 0x55,
	.platform_data = &bq27210_plat_data,
	},
#endif
#ifdef CONFIG_BATTERY_DS2778
	{
	.type = "ds2778",
	.addr = 0x59,
	.platform_data = &ds2778_plat_data,
	},
#endif
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		.type = "sgtl5000-i2c",
		.addr = 0x0a,
	},
	{   /* Camera */
		.type = "ov26xx",
		.addr = 0x30,
		.platform_data = &camera_data,
	},
};

static int sdhc_write_protect(struct device *dev)
{
	/* no write protection pin */
	return 0;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(TULIP_SD_MMC_SD);
		return ret;
	} else {		/* config the det pin for SDHC2/3 */
		/* no detection pin */
		return 0;
	}
}

static int mx51_wifi_power(int on)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, on);

	if(on) {
		//gpio_set_value(TULIP_EN_WIFI_PWR, 1);
		gpio_set_value(TULIP_WL_RST, 1);
	} else {
		gpio_set_value(TULIP_WL_RST, 0);
		//gpio_set_value(TULIP_EN_WIFI_PWR, 0);
	}
	return 0;
}

static int mx51_wifi_reset(int on)
{
	printk(KERN_DEBUG "%s(%d) do nothing\n", __func__, on);
	return 0;
}

static int mx51_wifi_set_carddetect(int on)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, on);
	mxc_mmc_force_detect(1);
	return 0;
}

static struct resource mx51_wifi_resources[] = {
	[0] = {
		.name  = "bcm4329_wlan_irq",
		.start =  gpio_to_irq(TULIP_WLAN_WAKEUP_HOST),
		.end   =  gpio_to_irq(TULIP_WLAN_WAKEUP_HOST),
		.flags = IORESOURCE_IRQ,
	},
};

static struct wifi_platform_data mx51_wifi_control = {
	.set_power      = mx51_wifi_power,
	.set_reset      = mx51_wifi_reset,
	.set_carddetect = mx51_wifi_set_carddetect,
};

static struct platform_device mx51_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(mx51_wifi_resources),
	.resource       = mx51_wifi_resources,
};

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static int mxc_sgtl5000_hp_mic_switch(int enable)
{
	gpio_set_value(TULIP_MIC_S, enable ? 0 : 1);
	return 0;
}

static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_set_value(TULIP_AMP_SHUT, enable);
	return 0;
}

static int mxc_sgtl5000_clock_enable(int enable)
{
	//gpio_set_value(BABBAGE_AUDIO_CLK_EN, !enable);
	return 0;
}

static int mxc_sgtl5000_finit(void)
{
	printk(KERN_INFO "tulip mxc_sgtl5000_finit: shutdown amp\n");
	gpio_set_value(TULIP_AMP_SHUT, 0);
	return 0;
}

static int headphone_det_status(void)
{
	return (gpio_get_value(TULIP_HPHONE_DET) == 1);
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = gpio_to_irq(TULIP_HPHONE_DET),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.clock_enable = mxc_sgtl5000_clock_enable,
	.hpmic_switch = mxc_sgtl5000_hp_mic_switch,
	.sysclk = 12288000,
	.finit = mxc_sgtl5000_finit,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_64M,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_64M,
	.cached = 1,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_all[] = {
	"rndis",
	"usb_mass_storage",
	"adb"
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0c10,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
};

static struct usb_mass_storage_platform_data mass_storage_data = {
	.nluns		= 3,
	.vendor		= "Freescale",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct usb_ether_platform_data rndis_data = {
	.vendorID	= 0x0bb4,
	.vendorDescr	= "Freescale",
};

static struct android_usb_platform_data android_usb_data = {
	.vendor_id      = 0x0bb4,
	.product_id     = 0x0c01,
	.version        = 0x0100,
	.product_name   = "Android Phone",
	.manufacturer_name = "Freescale",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

/*
 * battery charger device
 */
static int tulip_chg_is_cable_in(void)
{
	return gpio_get_value(TULIP_AC_IN) ? 0 : 1;
}

static int tulip_is_charge_ok(void)
{
	return gpio_get_value(TULIP_CHG_OK) ? 0 : 1;
}

static unsigned int tulip_get_bat_capacity(void)
{
	return 50;
}

struct container_array {
	unsigned short result[8];
	unsigned short rltbak[6];
};

#if 0
static void print_array(struct container_array *arr)
{
	int i;
	for (i = 0; i < 8; i++) {
		printk(KERN_DEBUG "%4d", arr->result[i]);
	}
}
#endif

static int unqualified_number(struct container_array *arr)
{
	int i, j, uqlf = 0;
	for (i = 0; i < 8; i++) {
		for (j = i + 1; j < 8; j++) {
			if (abs(arr->result[i] - arr->result[j]) > 3) {
				uqlf++;
			}
		}
	}
	pr_debug("unqualified number from %s is: %d\n", __func__, uqlf);
	return uqlf;
}

static int get_max_subscript(struct container_array *arr)
{
	int i, subscript = 0;
	unsigned short max = arr->result[0];
	for (i = 1; i < 8; i++) {
		if (max < arr->result[i]) {
			max = arr->result[i];
			subscript = i;
		}
	}
	pr_debug("max subscript from %s is: %d\n", __func__, subscript);
	return subscript;
}

static int get_min_subscript(struct container_array *arr)
{
    int i, subscript = 0;
	unsigned short min = arr->result[0];
    for (i = 1; i < 8; i++) {
		if (min > arr->result[i]) {
			min = arr->result[i];
			subscript = i;
		}
	}
	pr_debug("min subscript from %s is: %d\n", __func__, subscript);
	return subscript;
}

static void filter(struct container_array *arr)
{
	int xam = get_max_subscript(arr);
	int nim = get_min_subscript(arr);
	int i, j = 0;
	for (i = 0; i < 8; i++) {
		if (i != xam && i != nim) {
			arr->rltbak[j] = arr->result[i];
			j++;
		}
	}
}

static int make_average(struct container_array *arr)
{
	int i;
	unsigned short average, sum = 0;
	for (i = 0; i < 6; i++) {
		sum += arr->rltbak[i];
	}

	average = sum / 6;
	return average;
}

static int handle(int uqlf, struct container_array *arr)
{
	unsigned short result;
	if ((28 - uqlf) < 8) {
		pr_debug("in %s\ndiscrete array, discard!\n", __func__);
		return -1;
	} else {
		filter(arr);
		result = make_average(arr);
	}
	return result;
}

static unsigned int tulip_get_bat_voltage(void)
{
	struct container_array arr;
	unsigned short average;

	/*get 8 elements and put them in the array called result*/
	pmic_adc_convert(GEN_PURPOSE_AD7, arr.result);

	average = handle(unqualified_number(&arr), &arr);

	if (!gpio_get_value(TULIP_AC_IN)) {
		average += 20;
		pr_debug("when CHARGER IN, ADC value from %s is %d", __func__, average);
		pr_debug("when CHARGER IN, ADC voltage from %s is %d", __func__, average * 9510);
	} else {
		pr_debug("when CHARGER OUT, ADC value from %s is %d", __func__, average);
		pr_debug("when CHARGER OUT, ADC voltage from %s is %d", __func__, average * 9510);
	}

	return  average * 9510;
}

static unsigned int tulip_get_bat_current(void)
{
	unsigned short result[8];
	unsigned int current_raw, current_mA;

	pmic_adc_convert(BATTERY_CURRENT, result);

	current_raw = result[0];
	if (current_raw & 0x200)
		current_mA = -5870 * (0x200 - (current_raw & 0x1FF)) / 1000;
	else
		current_mA = 5870 * (current_raw & 0x1FF) / 1000;

	return current_mA;
}

static struct mc13892_power_supply_pdata mc13892_psy_data = {
	.is_cable_in = tulip_chg_is_cable_in,
	.is_charge_ok = tulip_is_charge_ok,
	.get_bat_capacity = tulip_get_bat_capacity,
	.get_bat_voltage = tulip_get_bat_voltage,
	.get_bat_current = tulip_get_bat_current,
};

static struct platform_device mc13892_psy_device = {
	.name = "mc13892_power_supply",
};

/* vibrate */
static struct timed_gpio android_gpio = {
	.name = "vibrator",
	.gpio = TULIP_VIBRATOR_ON,
	.max_timeout = 10000,
	.active_low = 0,
};

static struct timed_gpio_platform_data android_vibrator_data = {
	.num_gpios	= 1,
	.gpios		= &android_gpio,
};

static struct platform_device android_vibrator_device = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev = { .platform_data = &android_vibrator_data},                                             
};

static int gps_set_power(int enable)
{
	printk(KERN_DEBUG "gps_set_power: %d\n", enable);
	if (enable) {
		gpio_set_value(TULIP_GPS_PWR_ON, 1);
	} else {
		gpio_set_value(TULIP_GPS_PWR_ON, 0);
	}
	return 0;
}

static int gps_set_nrst(int nrst)
{
	printk(KERN_DEBUG "gps_set_nrst: %d\n", nrst);
	gpio_set_value(TULIP_GPS_RST, nrst);
	return 0;
}

static struct gps_control_data gps_ctrl_data = {
	.set_power = gps_set_power,
	.set_nrst = gps_set_nrst,
};

static struct platform_device gps_ctrl_device = {
	.name = "gps-control",
};

static int bluetooth_power(int on)
{
	printk(KERN_DEBUG "tulip: bt power %d\n", on);

	gpio_request(TULIP_BT_RST, "bt_reset");
	if (on) {
		gpio_direction_output(TULIP_BT_RST, 1);
	} else {
		gpio_direction_output(TULIP_BT_RST, 0);
	}
	gpio_free(TULIP_BT_RST);

	return 0;
}

static struct platform_device bt_power_device = {
	.name = "bt_power",
};

#ifdef CONFIG_BT_BLUESLEEP
static struct resource mx51_bluesleep_resources[] = {
	[0] = {
		.name  = "gpio_host_wake",
		.start = TULIP_BT_WAKEUP_HOST,
		.end   = TULIP_BT_WAKEUP_HOST,
		.flags = IORESOURCE_IO,
	},

	[1] = {
		.name  = "bt_wakeup",
		.start =  TULIP_HOST_WAKEUP_BT,
		.end   =  TULIP_HOST_WAKEUP_BT,
		.flags = IORESOURCE_IO,
	},

	[2] = {
		.name  = "bt_wakeup_host",
		.start =  IOMUX_TO_IRQ_V3(TULIP_BT_WAKEUP_HOST),
		.end   =  IOMUX_TO_IRQ_V3(TULIP_BT_WAKEUP_HOST),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device bluesleep_device = {
	.name           = "bluesleep",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(mx51_bluesleep_resources),
	.resource       = mx51_bluesleep_resources,
};
#endif

/*
 * psy_register_device is a callback function for register device 13892
 * if there is no battery-ic
 */
static void psy_register_device(void)
{
	mxc_register_device(&mc13892_psy_device, &mc13892_psy_data);
}

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0;
	int gpu_mem = SZ_64M;
	int fb_mem = SZ_32M;

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_tulip_get_cpu_wp;
	set_num_cpu_wp = mx51_tulip_set_num_cpu_wp;
	get_dvfs_core_wp = mx51_tulip_get_dvfs_core_table;
	num_cpu_wp = ARRAY_SIZE(cpu_wp_auto);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* pull SYS_ON_OFF_CTRL low to cut off power supply of board */

	printk(KERN_ERR "Going to shutdown board, byebye, baby!!!");

	gpio_direction_output(TULIP_SYS_ON_OFF_CTL, 0);
	gpio_set_value(TULIP_SYS_ON_OFF_CTL, 0);
}

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pwrkey_callback cb = (pwrkey_callback)dev_id;

	cb((void *)1);

	if (gpio_get_value(TULIP_SYS_ON_OFF_REQ))
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else
		set_irq_type(irq, IRQF_TRIGGER_RISING);

	return 0;
}

static void mxc_register_powerkey(pwrkey_callback pk_cb)
{
	/* Set power key as wakeup resource */
	int irq, ret;
	irq = gpio_to_irq(TULIP_SYS_ON_OFF_REQ);

	if (gpio_get_value(TULIP_SYS_ON_OFF_REQ))
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else
		set_irq_type(irq, IRQF_TRIGGER_RISING);

	ret = request_irq(irq, power_key_int, 0, "power_key", pk_cb);
	if (ret)
		pr_info("register on-off key interrupt failed\n");
	else
		enable_irq_wake(irq);
}

static int mxc_pwrkey_getstatus(int id)
{
	return gpio_get_value(TULIP_SYS_ON_OFF_REQ) ? 0 : 1;
}

static struct power_key_platform_data pwrkey_data = {
	.key_value = KEY_POWER,
	.register_pwrkey = mxc_register_powerkey,
	.get_key_status = mxc_pwrkey_getstatus,
};

static void modem_reset(void)
{
	gpio_request(TULIP_3G_RST, "3gmodem_reset");
	gpio_direction_output(TULIP_3G_RST, 0);
	msleep(100);
	gpio_direction_output(TULIP_3G_RST, 1);
	gpio_free(TULIP_3G_RST);

	return;
}

static void modem_onoff(int enable)
{
	int timeout = 10, state = 0;
	gpio_request(TULIP_3G_PWR_EN, "3gmodem_pwr_en");
	gpio_request(TULIP_3G_POWER_ON_OFF, "3gmodem_power_onoff");
	gpio_request(TULIP_3G_RST, "3gmodem_reset");
	
	if (enable) {
		gpio_direction_output(TULIP_3G_RST, 1);
		gpio_direction_output(TULIP_3G_POWER_ON_OFF, 1);
		gpio_direction_output(TULIP_3G_PWR_EN, 1);
		msleep(4000);
		gpio_direction_output(TULIP_3G_POWER_ON_OFF, 0);
		msleep(600);
		gpio_direction_output(TULIP_3G_POWER_ON_OFF, 1);
	} else {
		gpio_direction_output(TULIP_3G_PWR_EN, 0);
		gpio_direction_output(TULIP_3G_RST, 0);
	}

	gpio_free(TULIP_3G_PWR_EN);
	gpio_free(TULIP_3G_POWER_ON_OFF);
	gpio_free(TULIP_3G_RST);

	return;
}

static void modem_suspend(void)
{
	gpio_direction_output(TULIP_3G_WAKEUP_IN, 0);
}

static void modem_resume(void)
{
	gpio_direction_output(TULIP_3G_WAKEUP_IN, 1);
}

static int is_sim_in(void)
{
	return gpio_get_value(TULIP_3G_SIM_DET);
}
static struct modem_platform_data tulip_modem_platform =
{
	.sim_irq = -1,
	.wakeup_irq = -1,
	.suspend = modem_suspend,
	.resume = modem_resume,
	.reset = modem_reset,
	.onoff = modem_onoff,
	.is_sim_in = is_sim_in,
};

static struct platform_device tulip_3gmodem_device = {
	.name = "3Gmodem",
	.id = -1,
	.dev = {
		.platform_data = &tulip_modem_platform,
	},
};

/* For HUAWEI MU509 modem */
static void modem_init(void)
{
	gpio_request(TULIP_3G_POWER_ON_OFF, "modem off");
	gpio_direction_input(TULIP_3G_POWER_ON_OFF);
	gpio_free(TULIP_3G_POWER_ON_OFF);

	gpio_request(TULIP_3G_SIM_DET, "sim_cd");
	gpio_direction_input(TULIP_3G_SIM_DET);
	tulip_modem_platform.sim_irq = gpio_to_irq(TULIP_3G_SIM_DET);

	gpio_request(TULIP_3G_WAKEUP_OUT, "modem wakeup ap");
	gpio_direction_input(TULIP_3G_WAKEUP_OUT);
	tulip_modem_platform.wakeup_irq = gpio_to_irq(TULIP_3G_WAKEUP_OUT);

	gpio_request(TULIP_3G_WAKEUP_IN, "ap suspend modem");
	gpio_direction_output(TULIP_3G_WAKEUP_IN, 1);

	if (platform_device_register(&tulip_3gmodem_device)) {
		pr_err("add 3G modem device failed\n");
		return;
	}
}

static void __init mx51_tulip_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx51tulip_pads, ARRAY_SIZE(mx51tulip_pads));

	/* SYS_ON_OFF_CTRL */
	gpio_request(TULIP_SYS_ON_OFF_CTL, "power_off");
	gpio_direction_output(TULIP_SYS_ON_OFF_CTL, 1);
	gpio_set_value(TULIP_SYS_ON_OFF_CTL, 1);
	
	/* SYS_ON_OFF_REQ */
	gpio_request(TULIP_SYS_ON_OFF_REQ, "power_key");
	gpio_direction_input(TULIP_SYS_ON_OFF_REQ);

	/* PMIC INT */
	gpio_request(TULIP_INT_FROM_PMIC, "pmic_int");
	gpio_direction_input(TULIP_INT_FROM_PMIC);

	/* SD1_CD SD1_WP */ 
	gpio_request(TULIP_SD_MMC_SD, "sdhc1_detect");
	gpio_direction_input(TULIP_SD_MMC_SD);
	gpio_request(TULIP_SD1_WP, "sdhc1_wp");
	gpio_direction_input(TULIP_SD1_WP);

	/* De-assert USB PHY RESETB */
	gpio_request(TULIP_RST_USB_PHY_B, "usb_phy_reset");
	gpio_direction_output(TULIP_RST_USB_PHY_B, 1);

	/* RST_USB_HUB_B */
	gpio_request(TULIP_RST_USB_HUB_B, "rst_usb_hub");
	gpio_direction_output(TULIP_RST_USB_HUB_B, 0);
	gpio_set_value(TULIP_RST_USB_HUB_B, 0);
	msleep(1);
	gpio_set_value(TULIP_RST_USB_HUB_B, 1);

	/* cap touch */
	gpio_request(TULIP_TOUCH_INT, "touch_int");
	gpio_direction_input(TULIP_TOUCH_INT);
	gpio_request(TULIP_TOUCH_EN, "touch__en");
	gpio_direction_output(TULIP_TOUCH_EN, 1);

	/* Camera module enable */
	gpio_request(TULIP_CSI1_PWDN, "camera_enable");
	gpio_direction_output(TULIP_CSI1_PWDN, 0);
	gpio_set_value(TULIP_CSI1_PWDN, 1);

	/* TULIP_VCAM_EN */
	gpio_request(TULIP_VCAM_EN, "vcam_en");
	gpio_direction_output(TULIP_VCAM_EN, 1);

	/* gsensor int */
	gpio_request(TULIP_G_SENSOR_INT, "gsensor_int");
	gpio_direction_input(TULIP_G_SENSOR_INT);

	/* EN_WIFI_PWR */
	gpio_request(TULIP_EN_WIFI_PWR, "en_wifi_pwr");
	gpio_direction_output(TULIP_EN_WIFI_PWR, 1);
	udelay(100);

	/* WL_RST */
	gpio_request(TULIP_WL_RST, "wl_rst");
	gpio_direction_output(TULIP_WL_RST, 0);

	/* compass int */
	gpio_request(TULIP_COMPASS_INT, "compass_int");
	gpio_direction_input(TULIP_COMPASS_INT);

	/* VIBRATOR ON*/
	gpio_request(TULIP_VIBRATOR_ON, "vibrator_on"); /* driver to low or high ?*/
	gpio_direction_output(TULIP_VIBRATOR_ON, 0);

	/* Charger detect */
	gpio_request(TULIP_AC_IN, "charger_det");
	gpio_direction_input(TULIP_AC_IN);

	/* low battery wake up */
	gpio_request(TULIP_LOW_BAT_WAKEUP, "low_bat_wakeup");
	gpio_direction_input(TULIP_LOW_BAT_WAKEUP);

	/* Charger charge status */
	gpio_request(TULIP_CHG_OK, "charge_ok");
	gpio_direction_input(TULIP_CHG_OK);

	gpio_request(TULIP_EIM_A16, "eim_a16");
	gpio_direction_output(TULIP_EIM_A16, 1);
	gpio_request(TULIP_EIM_A17, "eim_a17");
	gpio_direction_output(TULIP_EIM_A17, 1);

	/* hphone_det_b */
	gpio_request(TULIP_HPHONE_DET, "hphone-det");
	gpio_direction_input(TULIP_HPHONE_DET);

	/* audio hphone mic control pin */
	gpio_request(TULIP_MIC_S, "hphone_mic");
	gpio_direction_output(TULIP_MIC_S, 1);
	gpio_set_value(TULIP_MIC_S, 0);

	/* LCD related gpio */
	gpio_request(TULIP_LCD_LDO_EN, "lcd_power");
	gpio_direction_output(TULIP_LCD_LDO_EN, 0);
	gpio_request(TULIP_VCC_LCD_3V7_EN, "lcd_power_3v7_enable");
	gpio_direction_output(TULIP_VCC_LCD_3V7_EN, 1);

	/* LVDS related gpio */
	gpio_request(TULIP_LVDS_SHTDN, "lvds_power");
	gpio_direction_output(TULIP_LVDS_SHTDN, 0);
	gpio_request(TULIP_LVDS_RSVD, "lvds_rsvd");
	gpio_direction_output(TULIP_LVDS_RSVD, 0);

	/* Blutooth Reset gpio */
	gpio_request(TULIP_BT_RST, "bt_reset");
	gpio_direction_output(TULIP_BT_RST, 0);
	gpio_free(TULIP_BT_RST);

	/* Blutooth Wakeup host gpio */
	gpio_request(TULIP_BT_WAKEUP_HOST, "bt_wakeup_host");
	gpio_direction_input(TULIP_BT_WAKEUP_HOST);
	gpio_free(TULIP_BT_WAKEUP_HOST);

	gpio_request(TULIP_HOST_WAKEUP_BT, "bt_wakeup");
	gpio_direction_output(TULIP_HOST_WAKEUP_BT, 1);
	gpio_free(TULIP_HOST_WAKEUP_BT);

	/* GPS Power on*/
	gpio_request(TULIP_GPS_PWR_ON, "gps_pwr_on");
	gpio_direction_output(TULIP_GPS_PWR_ON, 0);
	
	/* GPS Reset */
	gpio_request(TULIP_GPS_RST, "gps_reset");
	gpio_direction_output(TULIP_GPS_RST, 1);
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "csi_mclk1");
	mxc_ipu_data.csi_clk[1] = clk_get(NULL, "csi_mclk2");

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);
	/* SD card detect irqs */
	mxcsdhc1_device.resource[2].start = gpio_to_irq(TULIP_SD_MMC_SD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(TULIP_SD_MMC_SD);

	mxc_cpu_common_init();
	mx51_tulip_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_hs_device, &mxci2c_hs_data);
	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	//mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, &gpu_data);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&mx51_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_pwm1_device, &mxc_pwm1_platform_data);
	mxc_register_device(&mxc_pwm1_backlight_device,
		&mxc_pwm_backlight_data);
	mxc_register_device(&mxc_keypad_device, &keypad_plat_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device,
					&android_pmem_gpu_data);
	mxc_register_device(&usb_mass_storage_device, &mass_storage_data);
	mxc_register_device(&usb_rndis_device, &rndis_data);
	mxc_register_device(&android_usb_device, &android_usb_data);
	//mxc_register_device(&mxc_fec_device, NULL);
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	mxc_register_device(&mxc_powerkey_device, &pwrkey_data);

	/* vibrator detect */
	mxc_register_device(&android_vibrator_device, &android_vibrator_data);

	mx51_tulip_init_mc13892();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	pm_power_off = mxc_power_off;

	sgtl5000_data.sysclk = 26000000;

	/* EIM_A23: audio amp shut pin */
	gpio_request(TULIP_AMP_SHUT, "amp_shut"); /* ?????????????? is this audio standby? Driver to low? */
	gpio_direction_output(TULIP_AMP_SHUT, 0);
	gpio_set_value(TULIP_AMP_SHUT, 0);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mxc_register_device(&mx51_wifi_device, &mx51_wifi_control);

	/* Register GPS control platform device */
	mxc_register_device(&gps_ctrl_device, &gps_ctrl_data);

	/* Register bluetooth power platform device */
	mxc_register_device(&bt_power_device, &bluetooth_power);
#ifdef CONFIG_BT_BLUESLEEP
	/* Register Bluetooth Low Power Control Module */
	mxc_register_device(&bluesleep_device, NULL);
#endif

	/* Register MC13892 Power supply platform device */
#ifndef CONFIG_BATTERY_BQ27210
	mxc_register_device(&mc13892_psy_device, &mc13892_psy_data);
#endif

	/* Register BQ24103 charger platform device */
	mxc_register_device(&bq24103_charger, &bq24103_pdata);

	mx5_usb_dr_init();
	mx5_usbh1_init();

	modem_init();

#ifdef CONFIG_PM_CHECK
	register_gpio_setting(&tulip_gpio_check);
#endif
}

static void __init mx51_tulip_timer_init(void)
{
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (mx51_revision() == IMX_CHIP_REVISION_2_0) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx51_tulip_timer_init,
};

static void __init fixup_android_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0, avali_mem = 0;
	int gpu_mem = SZ_16M;
	int pmem_gpu_size = android_pmem_gpu_data.size;
	int pmem_adsp_size = android_pmem_data.size;

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_tulip_get_cpu_wp;
	set_num_cpu_wp = mx51_tulip_set_num_cpu_wp;
	get_dvfs_core_wp = mx51_tulip_get_dvfs_core_table;
	num_cpu_wp = ARRAY_SIZE(cpu_wp_auto);

	/* get mem= and gpu_memory= from cmdline */
	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				avali_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}
			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	/* get total memory from TAGS */
	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			left_mem = total_mem - gpu_mem
				- pmem_gpu_size - pmem_adsp_size;
			break;
		}
	}

	if (avali_mem > 0 && avali_mem < left_mem)
		left_mem = avali_mem;

	if (mem_tag) {
		android_pmem_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem + pmem_gpu_size;
		android_pmem_gpu_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem;
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
	}
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_TULIP data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_TULIP, "Tulip")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
#ifdef CONFIG_ANDROID_PMEM
	.fixup = fixup_android_board,
#else
	.fixup = fixup_mxc_board,
#endif
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
