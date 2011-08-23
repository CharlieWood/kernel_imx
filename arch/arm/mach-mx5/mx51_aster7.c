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
#include <linux/pmic_adc.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/powerkey.h>
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
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>
#include <linux/mc13892_power_supply.h>
#include <linux/switch.h>
#include <linux/i2c/hmc5883.h>
#include <linux/wlan_plat.h>
#include <linux/gps/gps.h>
#include <linux/timed_gpio.h>
#include <linux/input.h>
#include <linux/goodix_touch.h>
#include <linux/bq24103_charger.h>

#ifdef CONFIG_PM_CHECK
#include <linux/pm_check.h>
#endif

#include "devices.h"
#include "crm_regs.h"
#include "usb.h"

/*!
 * @file mach-mx51/mx51_aster.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */

/* IOMUX to GPIO */
#define ASTER7_SD_MMC_SD       (0*32 + 0)  /* GPIO_1_0 */
#define ASTER7_SD1_WP          (0*32 + 1)  /* GPIO_1_1 */
#define ASTER7_WDOG_B          (0*32 + 4)  /* GPIO_1_4 */
#define ASTER7_INT_FROM_PMIC   (0*32 + 8)  /* GPIO_1_8 */
#define ASTER7_TOUCH_INT       (0*32 + 9)  /* GPIO_1_9 */
#define ASTER7_GPS_UART2_TXD   (0*32 + 21) /* GPIO_1_21 */
#define ASTER7_SYS_ON_OFF_CTL  (0*32 + 23) /* GPIO_1_23 */

#define ASTER7_AC_IN           (1*32 + 1)  /* GPIO_2_1 */
#define ASTER7_CHG_OK          (1*32 + 2)  /* GPIO_2_2 */
#define ASTER7_BT_RST          (1*32 + 4)  /* GPIO_2_4 */
#define ASTER7_RST_USB_PHY_B   (1*32 + 5)  /* GPIO_2_5 */
#define ASTER7_BT_EXT_WAKE     (1*32 + 6)  /* GPIO_2_6 */
#define ASTER7_EIM_A16		   (1*32 + 10) /* GPIO_2_10 */
#define ASTER7_EIM_A17		   (1*32 + 11) /* GPIO_2_10 */
#define ASTER7_LID_CLOSE_B     (1*32 + 18) /* GPIO_2_18 */
#define ASTER7_LOW_BAT_WAKEUP  (1*32 + 19) /* GPIO_2_19 */
#define ASTER7_SYS_ON_OFF_REQ  (1*32 + 21) /* GPIO_2_21 */
#define ASTER7_G_SENSOR_INT    (1*32 + 22) /* GPIO_2_22 */
#define ASTER7_COMPASS_INT     (1*32 + 25) /* GPIO_2_25 */

#define ASTER7_TOUCH_REST      (2*32 + 3)  /* GPIO_3_3 */
#define ASTER7_3G_RST          (2*32 + 4)  /* GPIO_3_4 */
#define ASTER7_MIC_S           (2*32 + 5)  /* GPIO_3_5 */ 
#define ASTER7_CSI1_PWDN       (2*32 + 6)  /* GPIO_3_6 */
#define ASTER7_RST_USB_HUB_B   (2*32 + 7)  /* GPIO_3_7 */
#define ASTER7_WL_WOW_WAKE     (2*32 + 10) /* GPIO_3_10 */
#define ASTER7_VCAM_EN         (2*32 + 13) /* GPIO_3_13 */
#define ASTER7_WL_CHIP_PWD     (2*32 + 16) /* GPIO_3_16 */
#define ASTER7_GPS_PWR_ON      (2*32 + 17) /* GPIO_3_17 */
#define ASTER7_GPS_RST         (2*32 + 18) /* GPIO_3_18 */
#define ASTER7_HDMI_PWR_ON     (2*32 + 19) /* GPIO_3_19 */
#define ASTER7_WL_REST         (2*32 + 20) /* GPIO_3_20 */
#define ASTER7_LCD_PWR_ON      (2*32 + 21) /* GPIO_3_21 */
#define ASTER7_3G_EN           (2*32 + 22) /* GPIO_3_22 */
#define ASTER7_AMP_SHUT        (2*32 + 29) /* GPIO_3_29 */
#define ASTER7_HPHONE_DET      (2*32 + 30) /* GPIO_3_30 */
#define ASTER7_VIBRATOR_ON     (2*32 + 31) /* GPIO_3_31 */

#define ASTER7_CSP1_SS0_GPIO   (3*32 + 24) /* GPIO_4_24 */

extern int __init mx51_aster7_init_mc13892(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
extern struct dvfs_wp *(*get_dvfs_core_wp)(int *wp);
static int num_cpu_wp = 4;

#ifdef CONFIG_PM_CHECK
static struct gpio_check aster7_gpio_check[] = {
	{ASTER7_SD_MMC_SD, 1},
	{ASTER7_SD1_WP, 1},
	{ASTER7_WDOG_B, 1},
	{ASTER7_INT_FROM_PMIC, 1},
	{ASTER7_TOUCH_INT, 1},
	{ASTER7_GPS_UART2_TXD, 1},
	{ASTER7_SYS_ON_OFF_CTL, 1},
	{ASTER7_AC_IN, 1},
	{ASTER7_CHG_OK, 1},
	{ASTER7_BT_RST, 1},
	{ASTER7_RST_USB_PHY_B, 1},
	{ASTER7_BT_EXT_WAKE, 1},
	{ASTER7_EIM_A16, 1},
	{ASTER7_EIM_A17, 1},
	{ASTER7_LID_CLOSE_B, 1},
	{ASTER7_SYS_ON_OFF_REQ, 1},
	{ASTER7_G_SENSOR_INT, 1},
	{ASTER7_COMPASS_INT, 1},

	{ASTER7_TOUCH_REST, 1},
	{ASTER7_3G_RST, 1},
	{ASTER7_CSI1_PWDN, 1},
	{ASTER7_RST_USB_HUB_B, 1},
	{ASTER7_VCAM_EN, 1},
	{ASTER7_WL_CHIP_PWD, 1},
	{ASTER7_GPS_PWR_ON, 1},
	{ASTER7_GPS_RST, 1},
	{ASTER7_HDMI_PWR_ON, 1},
	{ASTER7_WL_REST, 1},
	{ASTER7_LCD_PWR_ON, 1},
	{ASTER7_3G_EN, 1},
	{ASTER7_AMP_SHUT, 1},
	{ASTER7_HPHONE_DET, 1},
	{ASTER7_MIC_S, 1},
	{ASTER7_VIBRATOR_ON, 1},
	{ASTER7_CSP1_SS0_GPIO, 1},
	{-1, 0},
};
#endif

static struct pad_desc mx51aster7_pads[] = {
	/* charger ac_in */
	MX51_PAD_EIM_D17__GPIO_2_1,
	/* charger ok */
	MX51_PAD_EIM_D18__GPIO_2_2,

	/* BT reset*/
	MX51_PAD_EIM_D20__GPIO_2_4,

	/* RST_USB_PHY_B, ALT1 gpio2_5 */
	MX51_PAD_EIM_D21__GPIO_2_5,

	/* BT_EXT_WAKE */
	MX51_PAD_EIM_D22__GPIO_2_6,

	/* SPDIF_OUT, to audio digital output, HDMI audio */
	MX51_PAD_EIM_D23__SPDIF_OUT1,

	/* uart 3 */
	MX51_PAD_EIM_D24__UART3_CTS,
	MX51_PAD_EIM_D25__UART3_RXD,
	MX51_PAD_EIM_D26__UART3_TXD,
	MX51_PAD_EIM_D27__UART3_RTS,

	/* AUD6 TXD RXD TXC TXFS */
	MX51_PAD_EIM_D28__AUD6_TXD,
	MX51_PAD_EIM_D29__AUD6_RXD,
	MX51_PAD_EIM_D30__AUD6_TXC,
	MX51_PAD_EIM_D31__AUD6_TXFS,

	/* LID close */
	MX51_PAD_EIM_A24__GPIO_2_18,

	/* low battery wake up */
	MX51_PAD_EIM_A25__GPIO_2_19,

	/* CSI1_DATA_EN */
	MX51_PAD_EIM_A26__GPIO_2_20,

	/* SYS_ON_OFF_REQ */
	MX51_PAD_EIM_A27__GPIO_2_21,

	/* G-SENSOR_INT*/
	MX51_PAD_EIM_EB2__GPIO_2_22,

	/* COMPASS_INT */
	MX51_PAD_EIM_CS0__GPIO_2_25,

	/* WL_CHIP_PWD */
	MX51_PAD_NANDF_CS0__GPIO_3_16,

	/* WLAN WOW */
	MX51_PAD_NANDF_RB2__GPIO_3_10,

	/* GPS_PWR_ON */
	MX51_PAD_NANDF_CS1__GPIO_3_17,

	/* GPS_RESET */
	MX51_PAD_NANDF_CS2__GPIO_3_18,

	/* HDMI_PWR_ON */
	MX51_PAD_NANDF_CS3__GPIO_3_19,

	/* WL_REST */
	MX51_PAD_NANDF_CS4__GPIO_3_20,

	/* LCD_PWR_ON */
	MX51_PAD_NANDF_CS5__GPIO_3_21,

	/* 3G_EN */
	MX51_PAD_NANDF_CS6__GPIO_3_22,

	/* VIBRATOR_ON */
	MX51_PAD_NANDF_D9__GPIO_3_31,

	/* HEADPHONE DETECT*/
	MX51_PAD_NANDF_D10__GPIO_3_30,

	/* AMP_SHUT */
	MX51_PAD_NANDF_D11__GPIO_3_29,

	/* MIC_Select */
	MX51_PAD_DISPB2_SER_DIN__GPIO_3_5,

	/* VCAM_EN */
	MX51_PAD_CSI1_D9__GPIO_3_13,

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

	/* TOUCH_REST */
	MX51_PAD_DI1_D0_CS__GPIO_3_3,

	/* 3G_RST */
	MX51_PAD_DI1_D1_CS__GPIO_3_4,

	/* CSI1_PWDN */
	MX51_PAD_DISPB2_SER_DIO__GPIO_3_6,

	/* RST_USB_HUB_B */
	MX51_PAD_DISPB2_SER_CLK__GPIO_3_7,

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
	MX51_PAD_NANDF_D15__SD3_DATA7,
	MX51_PAD_NANDF_D14__SD3_DATA6,
	MX51_PAD_NANDF_D13__SD3_DATA5,
	MX51_PAD_NANDF_D12__SD3_DATA4,
	MX51_PAD_NANDF_RB0__SD3_DATA3,
	MX51_PAD_NANDF_WP_B__SD3_DATA2,
	MX51_PAD_NANDF_RE_B__SD3_DATA1,
	MX51_PAD_NANDF_WE_B__SD3_DATA0,

	/* HS_I2C */
	MX51_PAD_I2C1_CLK__HSI2C_CLK,
	MX51_PAD_I2C1_DAT__HSI2C_DAT,

	/* Audio */
	MX51_PAD_AUD3_BB_TXD__AUD3_BB_TXD,
	MX51_PAD_AUD3_BB_RXD__AUD3_BB_RXD,
	MX51_PAD_AUD3_BB_CK__AUD3_BB_CK,
	MX51_PAD_AUD3_BB_FS__AUD3_BB_FS,

	/* SPI_SS1_NORFLASH */
	MX51_PAD_CSPI1_SS1__CSPI1_SS1,

	/* UART1 */
	MX51_PAD_UART1_RXD__UART1_RXD,
	MX51_PAD_UART1_TXD__UART1_TXD,
	MX51_PAD_UART1_RTS__UART1_RTS,
	MX51_PAD_UART1_CTS__UART1_CTS,

	/* UART2*/
	MX51_PAD_UART2_RXD__UART2_RXD,
	MX51_PAD_UART2_TXD__GPIO_1_21,
	
	/* SYS_ON_OFF_CTL */
	MX51_PAD_UART3_TXD__GPIO_1_23,

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
	MX51_PAD_GPIO_1_0__GPIO_1_0,

	/* SD1_WP */
	MX51_PAD_GPIO_1_1__GPIO_1_1,

	/* PWM1_OUT */
	MX51_PAD_GPIO_1_2__PWM_PWMO,

	/* WDOG_B */
	MX51_PAD_GPIO_1_4__GPIO_1_4,

	/* INT_FROM_PMIC */
	MX51_PAD_GPIO_1_8__GPIO_1_8,

	/* TOUCH INT */
	MX51_PAD_GPIO_1_9__GPIO_1_9,

	MX51_PAD_EIM_A16__GPIO_2_10,
	MX51_PAD_EIM_A17__GPIO_2_11,

	/* END */
};

static struct dvfs_wp dvfs_core_setpoint[] = {
	{33, 13, 33, 10, 10, 0x08}, /* 800MHz*/
	{28, 8, 33, 10, 10, 0x08},   /* 400MHz */
	{20, 0, 33, 20, 10, 0x08},   /* 160MHz*/
	{28, 8, 33, 20, 30, 0x08},   /*160MHz, AHB 133MHz, LPAPM mode*/
	{29, 0, 33, 20, 10, 0x08},}; /* 160MHz, AHB 24MHz */

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
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
	 "BYD8688", 60, 1024, 600, 22557, 84, 88, 8, 6, 4, 2,
	 0,	/* FB_SYNC_CLK_LAT_FALL, */
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct dvfs_wp *mx51_aster7_get_dvfs_core_table(int *wp)
{
	*wp = ARRAY_SIZE(dvfs_core_setpoint);
	return dvfs_core_setpoint;
}

struct cpu_wp *mx51_aster7_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_aster7_set_num_cpu_wp(int num)
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
	.trigger_key0 = KEY_VOLUMEUP,
	.trigger_key1 = KEY_VOLUMEDOWN,
	.trigger_key2 = KEY_MENU,
};

static struct mxc_pwm_platform_data mxc_pwm1_platform_data = {
	.pwmo_invert = 0,
	.enable_pwm_pad = NULL,
	.disable_pwm_pad = NULL,
	.duty_percent = 40,
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
	.reset = mx5_vpu_reset,
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx51_aster7_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			struct pad_desc cspi1_ss0 = MX51_PAD_CSPI1_SS0__CSPI1_SS0;

			mxc_iomux_v3_setup_pad(&cspi1_ss0);
			break;
			}
		case 0x2:
			{
			struct pad_desc cspi1_ss0_gpio = MX51_PAD_CSPI1_SS0__GPIO_4_24;

			mxc_iomux_v3_setup_pad(&cspi1_ss0_gpio);
			gpio_request(ASTER7_CSP1_SS0_GPIO, "cspi1-gpio");
			gpio_direction_output(ASTER7_CSP1_SS0_GPIO, 0);
			gpio_set_value(ASTER7_CSP1_SS0_GPIO, 1 & (~status));
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

static void mx51_aster7_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			gpio_free(ASTER7_CSP1_SS0_GPIO);
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
	.chipselect_active = mx51_aster7_gpio_spi_chipselect_active,
	.chipselect_inactive = mx51_aster7_gpio_spi_chipselect_inactive,
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
	.num_wp = 3,
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

static void lcd_suspend(void)
{
	printk("aster7: lcd suspend\n");

	/* PULL LCD POWER ON to LOW to shut down LCD */
	gpio_set_value(ASTER7_LCD_PWR_ON, 0);
}

static void lcd_resume(void)
{
	printk("aster7: lcd resume\n");

	/* PULL LCD POWER ON to HIGH to Light LCD on */
	gpio_set_value(ASTER7_LCD_PWR_ON, 1);
}

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "BYD8688",
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
	if (!machine_is_mx51_aster7())
		return 0;

	gpio_set_value(ASTER7_LCD_PWR_ON, 1);

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
    gpio_set_value(ASTER7_CSI1_PWDN, pwdn & 0x01);
}

static struct hmc5883_platform_data hmc5883_data = {
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
};

static struct mxc_camera_platform_data camera_data = {
    .mclk = 24000000,
    .csi = 0,
    .pwdn = ov26xx_pwdn,
};

static int hdmi_set_power(int enable)
{
	if (enable)
		gpio_set_value(ASTER7_HDMI_PWR_ON, 1);
	else
		gpio_set_value(ASTER7_HDMI_PWR_ON, 0);

	return 0;
}

static struct mxc_lcd_platform_data vga_data = {
	.set_power = hdmi_set_power,
};

#if 0
static struct ft5x0x_ts_platform_data ft5x0x_data = { 
	.gpio_irq = ASTER7_TOUCH_INT,
	.gpio_reset = ASTER7_TOUCH_REST,
};
#endif

static int aster7_chg_is_cable_in(void);
static int aster7_is_charge_ok(void);
static struct bq24103_platform_data  bq24103_pdata = {
	.is_online = aster7_chg_is_cable_in,
	.irq = IOMUX_TO_IRQ_V3(ASTER7_AC_IN),
};

static struct platform_device bq24103_charger = {
	.name = "bq24103-charger",
	.id   = 1,
	.dev  = {
		.platform_data = &bq24103_pdata,
	},
};

#if 1
static struct goodix_i2c_platform_data goodix_data = {
	.touch_max_height = 7680,
	.touch_max_width = 5120,
	.screen_max_height = 1024,
	.screen_max_width = 600,

	.gpio_irq = ASTER7_TOUCH_INT,
	.gpio_reset = ASTER7_TOUCH_REST,

};
#endif

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	/* 1.8V */
	{	/* g-sensor */
	.type = "bma150",
	.addr = 0x38,
	.irq  = IOMUX_TO_IRQ_V3(ASTER7_G_SENSOR_INT),
	},
	{	/* compass */
	.type = "hmc5883",
	.addr = 0x1e,
	.platform_data = &hmc5883_data,
	.irq  = IOMUX_TO_IRQ_V3(ASTER7_COMPASS_INT),
	}, 
	{
	.type = "ch7035",
	.addr = 0x76,
	.platform_data = &vga_data,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	.type = "Goodix-TS",
	.addr = 0x55,
	.platform_data = &goodix_data,
	.irq  = IOMUX_TO_IRQ_V3(ASTER7_TOUCH_INT),
	},
#if 0
	{
	.type = "ft5x0x_ts",
	.addr = 0x38,
	.platform_data = &ft5x0x_data,
	.irq  = IOMUX_TO_IRQ_V3(ASTER7_TOUCH_INT),
	}, 
#endif 
        {   /* Camera */
        .type = "ov26xx",
        .addr = 0x30,
        .platform_data = &camera_data,
        },
};

static struct i2c_board_info mxc_i2c_hs_board_info[] __initdata = {
};

#if 0
static struct mtd_partition mxc_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00040000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},

};

static struct mtd_partition mxc_dataflash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x000100000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},
};

static struct flash_platform_data mxc_spi_flash_data[] = {
	{
	 .name = "mxc_spi_nor",
	 .parts = mxc_spi_nor_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_spi_nor_partitions),
	 .type = "sst25vf016b",},
	{
	 .name = "mxc_dataflash",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "at45db321d",}
};

static struct spi_board_info mxc_spi_nor_device[] __initdata = {
	{
	 .modalias = "mxc_spi_nor",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[0],
	},
};

static struct spi_board_info mxc_dataflash_device[] __initdata = {
	{
	 .modalias = "mxc_dataflash",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[1],},
};
#endif

static int sdhc_write_protect(struct device *dev)
{
	/* no write protection pin */
	return 0;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(ASTER7_SD_MMC_SD);
		return ret;
	} else {		/* config the det pin for SDHC2/3 */
		/* no detection pin */
		return 0;
	}
}

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int mx51_wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int mx51_wifi_power(int on)
{
	printk(KERN_DEBUG "%s: %d\n", __func__, on);

	gpio_request(ASTER7_WL_CHIP_PWD, "wl_chip_pwd");
	if(on) {
		gpio_direction_output(ASTER7_WL_CHIP_PWD, 1);
		msleep(40);
	} else {
		gpio_direction_output(ASTER7_WL_CHIP_PWD, 0);
	}
	gpio_free(ASTER7_WL_CHIP_PWD);
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

	if (wifi_status_cb)
		wifi_status_cb(on, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
   
	return 0;
}

static struct wifi_platform_data mx51_wifi_data = {
	.set_power      = mx51_wifi_power,
	.set_reset      = mx51_wifi_reset,
	.set_carddetect = mx51_wifi_set_carddetect,
	.ar6k_refclock	= 19200000,
	.wow_irq        = IOMUX_TO_IRQ_V3(ASTER7_WL_WOW_WAKE),
};

static struct platform_device ar6000_pm_device = {
	.name		= "wlan_ar6000_pm_dev",
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
	.register_status_notify = mx51_wifi_status_register,
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
	gpio_set_value(ASTER7_MIC_S, enable ? 0 : 1);
	return 0;
}

static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_set_value(ASTER7_AMP_SHUT, enable ? 0 : 1);
	return 0;
}

static int mxc_sgtl5000_finit(void)
{
	printk(KERN_INFO "aster7 mxc_sgtl5000_finit: shutdown amp\n");
	gpio_set_value(ASTER7_AMP_SHUT, 1);
	return 0;
}

static int headphone_det_status(void)
{
	return (gpio_get_value(ASTER7_HPHONE_DET) == 1);
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ_V3(ASTER7_HPHONE_DET),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
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
	.size = SZ_32M,
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

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
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
		.product_id	= 0x0c03,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0c04,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
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

/* vibrate */
static struct timed_gpio android_gpio = {
	.name = "vibrator",
	.gpio = ASTER7_VIBRATOR_ON,
	.max_timeout = 10000,
	.active_low = 0,
};

struct timed_gpio_platform_data android_vibrator_data = {
	.num_gpios	= 1,
	.gpios		= &android_gpio,
};

struct platform_device android_vibrator_device = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev = { .platform_data = &android_vibrator_data},                                             
};

static int gps_set_power(int enable)
{
	if (1 == enable) {
		struct pad_desc uart2_txd_uart2_txd = MX51_PAD_UART2_TXD__UART2_TXD;
		gpio_free(ASTER7_GPS_UART2_TXD);
		mxc_iomux_v3_setup_pad(&uart2_txd_uart2_txd);

		gpio_set_value(ASTER7_GPS_PWR_ON, 1);
		mdelay(1);
		gpio_direction_output(ASTER7_GPS_RST, 0);
		gpio_set_value(ASTER7_GPS_RST, 0);
		udelay(300);
		gpio_set_value(ASTER7_GPS_RST, 1);
		
	} else if (0 == enable) {
		struct pad_desc uart2_txd_gpio1_21 = MX51_PAD_UART2_TXD__GPIO_1_21;

		gpio_set_value(ASTER7_GPS_PWR_ON, 0);
		gpio_direction_input(ASTER7_GPS_RST);

		mxc_iomux_v3_setup_pad(&uart2_txd_gpio1_21);
		gpio_request(ASTER7_GPS_UART2_TXD, "uart2_txd_gpio");
		gpio_direction_input(ASTER7_GPS_UART2_TXD);
	}

	return 0;
}

static int gps_get_status(void)
{
	return gpio_get_value(ASTER7_GPS_PWR_ON);
}

static struct gps_control_data gps_ctrl_data = {
	.set_power = gps_set_power,
	.get_status = gps_get_status,
};

static struct platform_device gps_ctrl_device = {
	.name = "gps-control",
};

static int aster7_chg_is_cable_in(void)
{
	return gpio_get_value(ASTER7_AC_IN) ? 0 : 1;
}

static int aster7_is_charge_ok(void)
{
	return gpio_get_value(ASTER7_CHG_OK);
}

static unsigned int aster7_get_bat_capacity(void)
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

static unsigned int aster7_get_bat_voltage(void)
{
	struct container_array arr;
	unsigned short average;

	/*get 8 elements and put them in the array called result*/
	pmic_adc_convert(GEN_PURPOSE_AD7, arr.result);

	average = handle(unqualified_number(&arr), &arr);

	if (!gpio_get_value(ASTER7_AC_IN)) {
		average += 20;
		pr_debug("when CHARGER IN, ADC value from %s is %d", __func__, average);
		pr_debug("when CHARGER IN, ADC voltage from %s is %d", __func__, average * 9510);
	} else {
		pr_debug("when CHARGER OUT, ADC value from %s is %d", __func__, average);
		pr_debug("when CHARGER OUT, ADC voltage from %s is %d", __func__, average * 9510);
	}

	return  average * 9510;
}

static unsigned int aster7_get_bat_current(void)
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
	.is_cable_in = aster7_chg_is_cable_in,
	.is_charge_ok = aster7_is_charge_ok,
	.get_bat_capacity = aster7_get_bat_capacity,
	.get_bat_voltage = aster7_get_bat_voltage,
	.get_bat_current = aster7_get_bat_current,
};

static struct platform_device mc13892_psy_device = {
	.name = "mc13892_power_supply",
};

static int bluetooth_power(int on)
{
	printk(KERN_DEBUG "aster7: bt power %d\n", on);

	gpio_request(ASTER7_BT_RST, "bt_reset");
	if (on) {
		gpio_direction_output(ASTER7_BT_RST, 1);
	}
	else {
		gpio_direction_output(ASTER7_BT_RST, 0);
	}
	gpio_free(ASTER7_BT_RST);

	return 0;
}

static struct platform_device bt_power_device = {
	.name = "bt_power",
};

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
	int left_mem = 0, temp_mem = 0;
	int gpu_mem = SZ_16M;
	int fb_mem = SZ_32M;
#ifdef CONFIG_ANDROID_PMEM
	int pmem_gpu_size = android_pmem_gpu_data.size;
	int pmem_adsp_size = android_pmem_data.size;
	fb_mem = 0;
#endif

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_aster7_get_cpu_wp;
	set_num_cpu_wp = mx51_aster7_set_num_cpu_wp;
	get_dvfs_core_wp = mx51_aster7_get_dvfs_core_table;


	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				temp_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}
			break;
		}
	}

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
#ifdef CONFIG_ANDROID_PMEM
			left_mem = total_mem - gpu_mem - pmem_gpu_size - pmem_adsp_size;
#else
			left_mem = total_mem - gpu_mem - fb_mem;
#endif
			break;
		}
	}

	if (temp_mem > 0 && temp_mem < left_mem)
		left_mem = temp_mem;

	if (mem_tag) {
#ifndef CONFIG_ANDROID_PMEM
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
#else
		android_pmem_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem + pmem_gpu_size;
		android_pmem_gpu_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem;
#endif
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
		gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
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

	gpio_direction_output(ASTER7_SYS_ON_OFF_CTL, 0);
	gpio_set_value(ASTER7_SYS_ON_OFF_CTL, 0);
}

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pwrkey_callback cb = (pwrkey_callback)dev_id;

	cb((void *)1);

	if (gpio_get_value(ASTER7_SYS_ON_OFF_REQ))
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else
		set_irq_type(irq, IRQF_TRIGGER_RISING);

	return 0;
}

static void mxc_register_powerkey(pwrkey_callback pk_cb)
{
	/* Set power key as wakeup resource */
	int irq, ret;
	irq = IOMUX_TO_IRQ_V3(ASTER7_SYS_ON_OFF_REQ);

	if (gpio_get_value(ASTER7_SYS_ON_OFF_REQ))
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
	return gpio_get_value(ASTER7_SYS_ON_OFF_REQ) ? 0 : 1;
}

static struct power_key_platform_data pwrkey_data = {
	.key_value = KEY_POWER,
	.register_pwrkey = mxc_register_powerkey,
	.get_key_status = mxc_pwrkey_getstatus,
};

static void __init mx51_aster7_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx51aster7_pads, ARRAY_SIZE(mx51aster7_pads));

	/* SYS_ON_OFF_CTRL */
	gpio_request(ASTER7_SYS_ON_OFF_CTL, "power_off");
	gpio_direction_output(ASTER7_SYS_ON_OFF_CTL, 1);
	gpio_set_value(ASTER7_SYS_ON_OFF_CTL, 1);
	
	/* SYS_ON_OFF_REQ */
	gpio_request(ASTER7_SYS_ON_OFF_REQ, "power_key");
	gpio_direction_input(ASTER7_SYS_ON_OFF_REQ);

	/* PMIC INT */
	gpio_request(ASTER7_INT_FROM_PMIC, "pmic_int");
	gpio_direction_input(ASTER7_INT_FROM_PMIC);

	/* SD1_CD SD1_WP */ 
	gpio_request(ASTER7_SD_MMC_SD, "sdhc1_detect");
	gpio_direction_input(ASTER7_SD_MMC_SD);
	gpio_request(ASTER7_SD1_WP, "sdhc1_wp");
	gpio_direction_input(ASTER7_SD1_WP);

	/* Drive 3G_RESET high */
	gpio_request(ASTER7_3G_RST, "3G_reset");
	gpio_direction_output(ASTER7_3G_RST, 1);
	gpio_set_value(ASTER7_3G_RST, 1);

	/* Drive 3G_ENABLE high */
	gpio_request(ASTER7_3G_EN, "3g_en");
	gpio_direction_output(ASTER7_3G_EN, 1);
	gpio_set_value(ASTER7_3G_EN, 1);

	/* GPS Power On */
	gpio_request(ASTER7_GPS_PWR_ON, "gps_pwr_on");
	gpio_direction_output(ASTER7_GPS_PWR_ON, 0);
	gpio_set_value(ASTER7_GPS_PWR_ON, 0);

	/* GPS Reset */
	gpio_request(ASTER7_GPS_RST, "gps_reset");
	gpio_direction_input(ASTER7_GPS_RST);

	gpio_request(ASTER7_GPS_UART2_TXD, "uart2_txd_gpio");
	gpio_direction_input(ASTER7_GPS_UART2_TXD);

	/* De-assert USB PHY RESETB */
	gpio_request(ASTER7_RST_USB_PHY_B, "usb_phy_reset");
	gpio_direction_output(ASTER7_RST_USB_PHY_B, 1);

	/* RST_USB_HUB_B */
	gpio_request(ASTER7_RST_USB_HUB_B, "rst_usb_hub");
	gpio_direction_output(ASTER7_RST_USB_HUB_B, 0);
	gpio_set_value(ASTER7_RST_USB_HUB_B, 0);
	msleep(1);
	gpio_set_value(ASTER7_RST_USB_HUB_B, 1);

	/* LCD related gpio */
	gpio_request(ASTER7_LCD_PWR_ON, "lcd_power_down");
	gpio_direction_output(ASTER7_LCD_PWR_ON, 1);

	/* cap touch */
	gpio_request(ASTER7_TOUCH_INT, "touch_int");
	gpio_direction_input(ASTER7_TOUCH_INT);

	/* ASTER7_WL_REST */
	gpio_request(ASTER7_WL_REST, "wlan_reset");
	gpio_direction_output(ASTER7_WL_REST, 1);
	gpio_free(ASTER7_WL_REST);

	/* WL_CHIP_PWD */
	gpio_request(ASTER7_WL_CHIP_PWD, "wl_chip_pwd");
	gpio_direction_output(ASTER7_WL_CHIP_PWD, 0);
	gpio_free(ASTER7_WL_CHIP_PWD);

	/* WL_WOW_WAKE */
	gpio_request(ASTER7_WL_WOW_WAKE, "wl_wow_wake");
	gpio_direction_input(ASTER7_WL_WOW_WAKE);

	/* ASTER7_BT_RST */
	gpio_request(ASTER7_BT_RST, "bt_reset");
	gpio_direction_output(ASTER7_BT_RST, 0);
	gpio_free(ASTER7_BT_RST);

	/* ASTER7_BT_EXT_WAKE */
	gpio_request(ASTER7_BT_EXT_WAKE, "bt_wake");
	gpio_direction_output(ASTER7_BT_EXT_WAKE, 0);
	gpio_free(ASTER7_BT_EXT_WAKE);

	/* Camera module enable */
	gpio_request(ASTER7_CSI1_PWDN, "camera_enable");
	gpio_direction_output(ASTER7_CSI1_PWDN, 0);
	gpio_set_value(ASTER7_CSI1_PWDN, 1);

	/* ASTER7_VCAM_EN */
	gpio_request(ASTER7_VCAM_EN, "vcam_en");
	gpio_direction_output(ASTER7_VCAM_EN, 1);

	/* gsensor int */
	gpio_request(ASTER7_G_SENSOR_INT, "gsensor_int");
	gpio_direction_input(ASTER7_G_SENSOR_INT);

	/* compass int */
	gpio_request(ASTER7_COMPASS_INT, "compass_int");
	gpio_direction_input(ASTER7_COMPASS_INT);

	/* VIBRATOR ON*/
	gpio_request(ASTER7_VIBRATOR_ON, "vibrator_on"); /* driver to low or high ?*/
	gpio_direction_output(ASTER7_VIBRATOR_ON, 0);

	/* Charger detect */
	gpio_request(ASTER7_AC_IN, "charger_det");
	gpio_direction_input(ASTER7_AC_IN);

	/* low battery wake up */
	gpio_request(ASTER7_LOW_BAT_WAKEUP, "low_bat_wakeup");
	gpio_direction_input(ASTER7_LOW_BAT_WAKEUP);

	/* Charger charge status */
	gpio_request(ASTER7_CHG_OK, "charge_ok");
	gpio_direction_input(ASTER7_CHG_OK);

	/* HDMI power on */
	gpio_request(ASTER7_HDMI_PWR_ON, "hdmi_en");
	gpio_direction_output(ASTER7_HDMI_PWR_ON, 1);
	gpio_set_value(ASTER7_HDMI_PWR_ON, 1);

	gpio_request(ASTER7_EIM_A16, "eim_a16");
	gpio_direction_output(ASTER7_EIM_A16, 1);
	gpio_request(ASTER7_EIM_A17, "eim_a17");
	gpio_direction_output(ASTER7_EIM_A17, 1);

	/* hphone_det_b */
	gpio_request(ASTER7_HPHONE_DET, "hphone-det");
	gpio_direction_input(ASTER7_HPHONE_DET);

	/* audio hphone mic control pin */
	gpio_request(ASTER7_MIC_S, "hphone_mic");
	gpio_direction_output(ASTER7_MIC_S, 1);
	gpio_set_value(ASTER7_MIC_S, 0);
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
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(ASTER7_SD_MMC_SD);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(ASTER7_SD_MMC_SD);

	mxc_cpu_common_init();
	mx51_aster7_io_init();

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
	mxc_register_device(&gpu_device, NULL);
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
	//mxc_register_device(&mxc_fec_device, NULL);
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	mxc_register_device(&mxc_powerkey_device, &pwrkey_data);

	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device, &android_pmem_gpu_data);
	mxc_register_device(&usb_mass_storage_device, &mass_storage_data);
	mxc_register_device(&usb_rndis_device, &rndis_data);
	mxc_register_device(&android_usb_device, &android_usb_data);

	/* vibrator detect */
	mxc_register_device(&android_vibrator_device, &android_vibrator_data);

	mx51_aster7_init_mc13892();

#if 0
	if (board_is_rev(BOARD_REV_2))
		/* BB2.5 */
		spi_register_board_info(mxc_dataflash_device,
					ARRAY_SIZE(mxc_dataflash_device));
	else
		/* BB2.0 */
		spi_register_board_info(mxc_spi_nor_device,
					ARRAY_SIZE(mxc_spi_nor_device));
#endif

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));

	pm_power_off = mxc_power_off;

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 2) {
		sgtl5000_data.sysclk = 26000000;
	}
	/* EIM_A23: audio amp shut pin */
	gpio_request(ASTER7_AMP_SHUT, "amp_shut"); /* ?????????????? is this audio standby? Driver to low? */
	gpio_direction_output(ASTER7_AMP_SHUT, 1);
	gpio_set_value(ASTER7_AMP_SHUT, 1);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);

	/* Register GPS control platform device */
	mxc_register_device(&gps_ctrl_device, &gps_ctrl_data);

	/* Register Aster Power supply platform device */
	mxc_register_device(&mc13892_psy_device, &mc13892_psy_data);

	/* Register AR6000 PM platform device */
	mxc_register_device(&ar6000_pm_device, &mx51_wifi_data);

	/* Register BQ24103 charger platform device */
	mxc_register_device(&bq24103_charger, &bq24103_pdata);

	/* Register bluetooth power platform device */
	mxc_register_device(&bt_power_device, &bluetooth_power);

	mx5_usb_dr_init();
	mx5_usbh1_init();

#ifdef CONFIG_PM_CHECK
	register_gpio_setting(&aster7_gpio_check);
#endif
}

static void __init mx51_aster7_timer_init(void)
{
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx51_aster7_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_ASTER7 data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_ASTER7, "Aster7 Freescale MX51 Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
