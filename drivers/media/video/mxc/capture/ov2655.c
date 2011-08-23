/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"

#define ov2655_VOLTAGE_ANALOG               2800000
#define ov2655_VOLTAGE_DIGITAL_CORE         1500000
#define ov2655_VOLTAGE_DIGITAL_IO           1800000
#define ov2655_VOLTAGE_DIGITAL_GPO	    2800000

/* Check these values! */
#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define ov2655_XCLK_MIN 6000000
#define ov2655_XCLK_MAX 24000000

static int not_detect = 0;

enum ov2655_mode {
	ov2655_mode_MIN = 0,
	ov2655_mode_SVGA_800_600 = 0,
	ov2655_mode_VGA_640_480 = 1,
	ov2655_mode_QVGA_320_240 = 2,
	ov2655_mode_UXGA_1600_1200 = 3,
	ov2655_mode_SXGA_1280_960= 4,
	ov2655_mode_D1_720_576 = 5,
	ov2655_mode_SXGA_1280_1024= 6,
	ov2655_mode_MAX = 6,
        ov2655_init_svga= 10
};

enum ov2655_frame_rate {
	ov2655_30_fps,
	ov2655_15_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov2655_mode_info {
	enum ov2655_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
struct sensor {
	const struct ov2655_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	int csi;
} ov2655_data;

static struct reg_value ov2655_setting_30fps_SVGA_800_600[] = {
	{0x3012, 0x80, 0, 0}, {0x308d, 0x0e, 0, 0}, {0x360b, 0x00, 0, 0},
	{0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0}, {0x30b2, 0x24, 0, 0},
	{0x300e, 0x34, 0, 0}, {0x300f, 0xa6, 0, 0}, {0x3010, 0x81, 0, 0},
	{0x3082, 0x01, 0, 0}, {0x30f4, 0x01, 0, 0}, {0x3090, 0xc0, 0, 0},
	{0x3091, 0xc0, 0, 0}, {0x30ac, 0x42, 0, 0}, {0x30d1, 0x08, 0, 0},
	{0x30a8, 0x56, 0, 0}, {0x3015, 0x03, 0, 0}, {0x3093, 0x00, 0, 0},
	{0x307e, 0xe5, 0, 0}, {0x3079, 0x00, 0, 0}, {0x30aa, 0x42, 0, 0},
	{0x3017, 0x40, 0, 0}, {0x30f3, 0x82, 0, 0}, {0x306a, 0x0c, 0, 0},
	{0x306d, 0x00, 0, 0}, {0x336a, 0x3c, 0, 0}, {0x3076, 0x6a, 0, 0},
	{0x30d9, 0x8c, 0, 0}, {0x3016, 0x82, 0, 0}, {0x3601, 0x30, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x30f1, 0x82, 0, 0}, {0x306f, 0x14, 0, 0},
	{0x302a, 0x02, 0, 0}, {0x302b, 0x6a, 0, 0}, {0x3012, 0x10, 0, 0},
	{0x3011, 0x00, 0, 0}, {0x3013, 0xf7, 0, 0}, {0x301c, 0x13, 0, 0},
	{0x301d, 0x17, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3072, 0x4d, 0, 0},
	{0x30af, 0x00, 0, 0}, {0x3048, 0x1f, 0, 0}, {0x3049, 0x4e, 0, 0},
	{0x304a, 0x20, 0, 0}, {0x304f, 0x20, 0, 0}, {0x304b, 0x02, 0, 0},
	{0x304c, 0x00, 0, 0}, {0x304d, 0x02, 0, 0}, {0x304f, 0x20, 0, 0},
	{0x30a3, 0x10, 0, 0}, {0x3013, 0xf7, 0, 0}, {0x3014, 0x44, 0, 0},
	{0x3071, 0x00, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3073, 0x00, 0, 0},
	{0x3072, 0x4d, 0, 0}, {0x301c, 0x05, 0, 0}, {0x301d, 0x06, 0, 0},
	{0x304d, 0x42, 0, 0}, {0x304a, 0x40, 0, 0}, {0x304f, 0x40, 0, 0},
	{0x3095, 0x07, 0, 0}, {0x3096, 0x16, 0, 0}, {0x3097, 0x1d, 0, 0},
	{0x300e, 0x38, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x18, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x06, 0, 0}, {0x3024, 0x06, 0, 0},
	{0x3025, 0x58, 0, 0}, {0x3026, 0x02, 0, 0}, {0x3027, 0x61, 0, 0},
	{0x3088, 0x03, 0, 0}, {0x3089, 0x20, 0, 0}, {0x308a, 0x02, 0, 0},
	{0x308b, 0x58, 0, 0}, {0x3316, 0x64, 0, 0}, {0x3317, 0x25, 0, 0},
	{0x3318, 0x80, 0, 0}, {0x3319, 0x08, 0, 0}, {0x331a, 0x64, 0, 0},
	{0x331b, 0x4b, 0, 0}, {0x331c, 0x00, 0, 0}, {0x331d, 0x38, 0, 0},
	{0x3100, 0x00, 0, 0}, {0x3320, 0xfa, 0, 0}, {0x3321, 0x11, 0, 0},
	{0x3322, 0x92, 0, 0}, {0x3323, 0x01, 0, 0}, {0x3324, 0x97, 0, 0},
	{0x3325, 0x02, 0, 0}, {0x3326, 0xff, 0, 0}, {0x3327, 0x0c, 0, 0},
	{0x3328, 0x10, 0, 0}, {0x3329, 0x10, 0, 0}, {0x332a, 0x58, 0, 0},
	{0x332b, 0x50, 0, 0}, {0x332c, 0xbe, 0, 0}, {0x332d, 0xe1, 0, 0},
	{0x332e, 0x43, 0, 0}, {0x332f, 0x36, 0, 0}, {0x3330, 0x4d, 0, 0},
	{0x3331, 0x44, 0, 0}, {0x3332, 0xf8, 0, 0}, {0x3333, 0x0a, 0, 0},
	{0x3334, 0xf0, 0, 0}, {0x3335, 0xf0, 0, 0}, {0x3336, 0xf0, 0, 0},
	{0x3337, 0x40, 0, 0}, {0x3338, 0x40, 0, 0}, {0x3339, 0x40, 0, 0},
	{0x333a, 0x00, 0, 0}, {0x333b, 0x00, 0, 0}, {0x3380, 0x28, 0, 0},
	{0x3381, 0x48, 0, 0}, {0x3382, 0x10, 0, 0}, {0x3383, 0x23, 0, 0},
	{0x3384, 0xc0, 0, 0}, {0x3385, 0xe5, 0, 0}, {0x3386, 0xc2, 0, 0},
	{0x3387, 0xb3, 0, 0}, {0x3388, 0x0e, 0, 0}, {0x3389, 0x98, 0, 0},
	{0x338a, 0x01, 0, 0}, {0x3340, 0x0e, 0, 0}, {0x3341, 0x1a, 0, 0},
	{0x3342, 0x31, 0, 0}, {0x3343, 0x45, 0, 0}, {0x3344, 0x5a, 0, 0},
	{0x3345, 0x69, 0, 0}, {0x3346, 0x75, 0, 0}, {0x3347, 0x7e, 0, 0},
	{0x3348, 0x88, 0, 0}, {0x3349, 0x96, 0, 0}, {0x334a, 0xa3, 0, 0},
	{0x334b, 0xaf, 0, 0}, {0x334c, 0xc4, 0, 0}, {0x334d, 0xd7, 0, 0},
	{0x334e, 0xe8, 0, 0}, {0x334f, 0x20, 0, 0}, {0x3350, 0x32, 0, 0},
	{0x3351, 0x25, 0, 0}, {0x3352, 0x80, 0, 0}, {0x3353, 0x1e, 0, 0},
	{0x3354, 0x00, 0, 0}, {0x3355, 0x85, 0, 0}, {0x3356, 0x32, 0, 0},
	{0x3357, 0x25, 0, 0}, {0x3358, 0x80, 0, 0}, {0x3359, 0x1b, 0, 0},
	{0x335a, 0x00, 0, 0}, {0x335b, 0x85, 0, 0}, {0x335c, 0x32, 0, 0},
	{0x335d, 0x25, 0, 0}, {0x335e, 0x80, 0, 0}, {0x335f, 0x1b, 0, 0},
	{0x3360, 0x00, 0, 0}, {0x3361, 0x85, 0, 0}, {0x3363, 0x70, 0, 0},
	{0x3364, 0x7f, 0, 0}, {0x3365, 0x00, 0, 0}, {0x3366, 0x00, 0, 0},
	{0x3301, 0xff, 0, 0}, {0x338b, 0x11, 0, 0}, {0x338c, 0x10, 0, 0},
	{0x338d, 0x40, 0, 0}, {0x3370, 0xd0, 0, 0}, {0x3371, 0x00, 0, 0},
	{0x3372, 0x00, 0, 0}, {0x3373, 0x40, 0, 0}, {0x3374, 0x10, 0, 0},
	{0x3375, 0x10, 0, 0}, {0x3376, 0x04, 0, 0}, {0x3377, 0x00, 0, 0},
	{0x3378, 0x04, 0, 0}, {0x3379, 0x80, 0, 0}, {0x3069, 0x84, 0, 0},
	{0x3090, 0x0b, 0, 0}, //mirror
	{0x307c, 0x12, 0, 0}, {0x3087, 0x02, 0, 0}, {0x3300, 0xfc, 0, 0},
	{0x3302, 0x11, 0, 0}, {0x3400, 0x02, 0, 0}, {0x3606, 0x20, 0, 0},
	{0x3601, 0x30, 0, 0}, {0x300e, 0x34, 0, 0}, {0x30f3, 0x83, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x3086, 0x0f, 0, 0}, {0x3086, 0x00, 0, 0},
};

static struct reg_value ov2655_setting_30fps_VGA_640_480[] = {
	{0x3012, 0x80, 0, 0}, {0x308d, 0x0e, 0, 0}, {0x360b, 0x00, 0, 0},
	{0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0}, {0x30b2, 0x24, 0, 0},
	{0x300e, 0x34, 0, 0}, {0x300f, 0xa6, 0, 0}, {0x3010, 0x81, 0, 0},
	{0x3082, 0x01, 0, 0}, {0x30f4, 0x01, 0, 0}, {0x3090, 0xc0, 0, 0},
	{0x3091, 0xc0, 0, 0}, {0x30ac, 0x42, 0, 0}, {0x30d1, 0x08, 0, 0},
	{0x30a8, 0x56, 0, 0}, {0x3015, 0x03, 0, 0}, {0x3093, 0x00, 0, 0},
	{0x307e, 0xe5, 0, 0}, {0x3079, 0x00, 0, 0}, {0x30aa, 0x42, 0, 0},
	{0x3017, 0x40, 0, 0}, {0x30f3, 0x82, 0, 0}, {0x306a, 0x0c, 0, 0},
	{0x306d, 0x00, 0, 0}, {0x336a, 0x3c, 0, 0}, {0x3076, 0x6a, 0, 0},
	{0x30d9, 0x8c, 0, 0}, {0x3016, 0x82, 0, 0}, {0x3601, 0x30, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x30f1, 0x82, 0, 0}, {0x306f, 0x14, 0, 0},
	{0x302a, 0x02, 0, 0}, {0x302b, 0x6a, 0, 0}, {0x3012, 0x10, 0, 0},
	{0x3011, 0x00, 0, 0}, {0x3013, 0xf7, 0, 0}, {0x301c, 0x13, 0, 0},
	{0x301d, 0x17, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3072, 0x4d, 0, 0},
	{0x30af, 0x00, 0, 0}, {0x3048, 0x1f, 0, 0}, {0x3049, 0x4e, 0, 0},
	{0x304a, 0x20, 0, 0}, {0x304f, 0x20, 0, 0}, {0x304b, 0x02, 0, 0},
	{0x304c, 0x00, 0, 0}, {0x304d, 0x02, 0, 0}, {0x304f, 0x20, 0, 0},
	{0x30a3, 0x10, 0, 0}, {0x3013, 0xf7, 0, 0}, {0x3014, 0x44, 0, 0},
	{0x3071, 0x00, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3073, 0x00, 0, 0},
	{0x3072, 0x4d, 0, 0}, {0x301c, 0x05, 0, 0}, {0x301d, 0x06, 0, 0},
	{0x304d, 0x42, 0, 0}, {0x304a, 0x40, 0, 0}, {0x304f, 0x40, 0, 0},
	{0x3095, 0x07, 0, 0}, {0x3096, 0x16, 0, 0}, {0x3097, 0x1d, 0, 0},
	{0x300e, 0x38, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x18, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x06, 0, 0}, {0x3024, 0x06, 0, 0},
	{0x3025, 0x58, 0, 0}, {0x3026, 0x02, 0, 0}, {0x3027, 0x61, 0, 0},
	{0x3088, 0x02, 0, 0}, {0x3089, 0x80, 0, 0}, {0x308a, 0x01, 0, 0},
	{0x308b, 0xe0, 0, 0}, {0x3316, 0x64, 0, 0}, {0x3317, 0x25, 0, 0},
	{0x3318, 0x80, 0, 0}, {0x3319, 0x08, 0, 0}, {0x331a, 0x28, 0, 0},
	{0x331b, 0x1e, 0, 0}, {0x331c, 0x00, 0, 0}, {0x331d, 0x38, 0, 0},
	{0x3100, 0x00, 0, 0}, {0x3320, 0xfa, 0, 0}, {0x3321, 0x11, 0, 0},
	{0x3322, 0x92, 0, 0}, {0x3323, 0x01, 0, 0}, {0x3324, 0x97, 0, 0},
	{0x3325, 0x02, 0, 0}, {0x3326, 0xff, 0, 0}, {0x3327, 0x0c, 0, 0},
	{0x3328, 0x10, 0, 0}, {0x3329, 0x10, 0, 0}, {0x332a, 0x58, 0, 0},
	{0x332b, 0x50, 0, 0}, {0x332c, 0xbe, 0, 0}, {0x332d, 0xe1, 0, 0},
	{0x332e, 0x43, 0, 0}, {0x332f, 0x36, 0, 0}, {0x3330, 0x4d, 0, 0},
	{0x3331, 0x44, 0, 0}, {0x3332, 0xf8, 0, 0}, {0x3333, 0x0a, 0, 0},
	{0x3334, 0xf0, 0, 0}, {0x3335, 0xf0, 0, 0}, {0x3336, 0xf0, 0, 0},
	{0x3337, 0x40, 0, 0}, {0x3338, 0x40, 0, 0}, {0x3339, 0x40, 0, 0},
	{0x333a, 0x00, 0, 0}, {0x333b, 0x00, 0, 0}, {0x3380, 0x28, 0, 0},
	{0x3381, 0x48, 0, 0}, {0x3382, 0x10, 0, 0}, {0x3383, 0x23, 0, 0},
	{0x3384, 0xc0, 0, 0}, {0x3385, 0xe5, 0, 0}, {0x3386, 0xc2, 0, 0},
	{0x3387, 0xb3, 0, 0}, {0x3388, 0x0e, 0, 0}, {0x3389, 0x98, 0, 0},
	{0x338a, 0x01, 0, 0}, {0x3340, 0x0e, 0, 0}, {0x3341, 0x1a, 0, 0},
	{0x3342, 0x31, 0, 0}, {0x3343, 0x45, 0, 0}, {0x3344, 0x5a, 0, 0},
	{0x3345, 0x69, 0, 0}, {0x3346, 0x75, 0, 0}, {0x3347, 0x7e, 0, 0},
	{0x3348, 0x88, 0, 0}, {0x3349, 0x96, 0, 0}, {0x334a, 0xa3, 0, 0},
	{0x334b, 0xaf, 0, 0}, {0x334c, 0xc4, 0, 0}, {0x334d, 0xd7, 0, 0},
	{0x334e, 0xe8, 0, 0}, {0x334f, 0x20, 0, 0}, {0x3350, 0x32, 0, 0},
	{0x3351, 0x25, 0, 0}, {0x3352, 0x80, 0, 0}, {0x3353, 0x1e, 0, 0},
	{0x3354, 0x00, 0, 0}, {0x3355, 0x85, 0, 0}, {0x3356, 0x32, 0, 0},
	{0x3357, 0x25, 0, 0}, {0x3358, 0x80, 0, 0}, {0x3359, 0x1b, 0, 0},
	{0x335a, 0x00, 0, 0}, {0x335b, 0x85, 0, 0}, {0x335c, 0x32, 0, 0},
	{0x335d, 0x25, 0, 0}, {0x335e, 0x80, 0, 0}, {0x335f, 0x1b, 0, 0},
	{0x3360, 0x00, 0, 0}, {0x3361, 0x85, 0, 0}, {0x3363, 0x70, 0, 0},
	{0x3364, 0x7f, 0, 0}, {0x3365, 0x00, 0, 0}, {0x3366, 0x00, 0, 0},
	{0x3301, 0xff, 0, 0}, {0x338b, 0x11, 0, 0}, {0x338c, 0x10, 0, 0},
	{0x338d, 0x40, 0, 0}, {0x3370, 0xd0, 0, 0}, {0x3371, 0x00, 0, 0},
	{0x3372, 0x00, 0, 0}, {0x3373, 0x40, 0, 0}, {0x3374, 0x10, 0, 0},
	{0x3375, 0x10, 0, 0}, {0x3376, 0x04, 0, 0}, {0x3377, 0x00, 0, 0},
	{0x3378, 0x04, 0, 0}, {0x3379, 0x80, 0, 0}, {0x3069, 0x84, 0, 0},
	{0x3090, 0x0b, 0, 0}, //mirror
	{0x307c, 0x12, 0, 0}, {0x3087, 0x02, 0, 0}, {0x3300, 0xfc, 0, 0},
	{0x3302, 0x11, 0, 0}, {0x3400, 0x02, 0, 0}, {0x3606, 0x20, 0, 0},
	{0x3601, 0x30, 0, 0}, {0x300e, 0x34, 0, 0}, {0x30f3, 0x83, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x3086, 0x0f, 0, 0}, {0x3086, 0x00, 0, 0},
};

static struct reg_value ov2655_setting_30fps_QVGA_320_240[] = {
	{0x3012, 0x80, 0, 0}, {0x308d, 0x0e, 0, 0}, {0x360b, 0x00, 0, 0},
	{0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0}, {0x30b2, 0x24, 0, 0},
	{0x300e, 0x34, 0, 0}, {0x300f, 0xa6, 0, 0}, {0x3010, 0x81, 0, 0},
	{0x3082, 0x01, 0, 0}, {0x30f4, 0x01, 0, 0}, {0x3090, 0xc0, 0, 0},
	{0x3091, 0xc0, 0, 0}, {0x30ac, 0x42, 0, 0}, {0x30d1, 0x08, 0, 0},
	{0x30a8, 0x56, 0, 0}, {0x3015, 0x03, 0, 0}, {0x3093, 0x00, 0, 0},
	{0x307e, 0xe5, 0, 0}, {0x3079, 0x00, 0, 0}, {0x30aa, 0x42, 0, 0},
	{0x3017, 0x40, 0, 0}, {0x30f3, 0x82, 0, 0}, {0x306a, 0x0c, 0, 0},
	{0x306d, 0x00, 0, 0}, {0x336a, 0x3c, 0, 0}, {0x3076, 0x6a, 0, 0},
	{0x30d9, 0x8c, 0, 0}, {0x3016, 0x82, 0, 0}, {0x3601, 0x30, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x30f1, 0x82, 0, 0}, {0x306f, 0x14, 0, 0},
	{0x302a, 0x02, 0, 0}, {0x302b, 0x6a, 0, 0}, {0x3012, 0x10, 0, 0},
	{0x3011, 0x00, 0, 0}, {0x3013, 0xf7, 0, 0}, {0x301c, 0x13, 0, 0},
	{0x301d, 0x17, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3072, 0x4d, 0, 0},
	{0x30af, 0x00, 0, 0}, {0x3048, 0x1f, 0, 0}, {0x3049, 0x4e, 0, 0},
	{0x304a, 0x20, 0, 0}, {0x304f, 0x20, 0, 0}, {0x304b, 0x02, 0, 0},
	{0x304c, 0x00, 0, 0}, {0x304d, 0x02, 0, 0}, {0x304f, 0x20, 0, 0},
	{0x30a3, 0x10, 0, 0}, {0x3013, 0xf7, 0, 0}, {0x3014, 0x44, 0, 0},
	{0x3071, 0x00, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3073, 0x00, 0, 0},
	{0x3072, 0x4d, 0, 0}, {0x301c, 0x05, 0, 0}, {0x301d, 0x06, 0, 0},
	{0x304d, 0x42, 0, 0}, {0x304a, 0x40, 0, 0}, {0x304f, 0x40, 0, 0},
	{0x3095, 0x07, 0, 0}, {0x3096, 0x16, 0, 0}, {0x3097, 0x1d, 0, 0},
	{0x300e, 0x38, 0, 0}, {0x3020, 0x01, 0, 0}, {0x3021, 0x18, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x06, 0, 0}, {0x3024, 0x06, 0, 0},
	{0x3025, 0x58, 0, 0}, {0x3026, 0x02, 0, 0}, {0x3027, 0x61, 0, 0},
	{0x3088, 0x01, 0, 0}, {0x3089, 0x40, 0, 0}, {0x308a, 0x00, 0, 0},
	{0x308b, 0xf0, 0, 0}, {0x3316, 0x64, 0, 0}, {0x3317, 0x25, 0, 0},
	{0x3318, 0x80, 0, 0}, {0x3319, 0x08, 0, 0}, {0x331a, 0x14, 0, 0},
	{0x331b, 0x0f, 0, 0}, {0x331c, 0x00, 0, 0}, {0x331d, 0x38, 0, 0},
	{0x3100, 0x00, 0, 0}, {0x3320, 0xfa, 0, 0}, {0x3321, 0x11, 0, 0},
	{0x3322, 0x92, 0, 0}, {0x3323, 0x01, 0, 0}, {0x3324, 0x97, 0, 0},
	{0x3325, 0x02, 0, 0}, {0x3326, 0xff, 0, 0}, {0x3327, 0x0c, 0, 0},
	{0x3328, 0x10, 0, 0}, {0x3329, 0x10, 0, 0}, {0x332a, 0x58, 0, 0},
	{0x332b, 0x50, 0, 0}, {0x332c, 0xbe, 0, 0}, {0x332d, 0xe1, 0, 0},
	{0x332e, 0x43, 0, 0}, {0x332f, 0x36, 0, 0}, {0x3330, 0x4d, 0, 0},
	{0x3331, 0x44, 0, 0}, {0x3332, 0xf8, 0, 0}, {0x3333, 0x0a, 0, 0},
	{0x3334, 0xf0, 0, 0}, {0x3335, 0xf0, 0, 0}, {0x3336, 0xf0, 0, 0},
	{0x3337, 0x40, 0, 0}, {0x3338, 0x40, 0, 0}, {0x3339, 0x40, 0, 0},
	{0x333a, 0x00, 0, 0}, {0x333b, 0x00, 0, 0}, {0x3380, 0x28, 0, 0},
	{0x3381, 0x48, 0, 0}, {0x3382, 0x10, 0, 0}, {0x3383, 0x23, 0, 0},
	{0x3384, 0xc0, 0, 0}, {0x3385, 0xe5, 0, 0}, {0x3386, 0xc2, 0, 0},
	{0x3387, 0xb3, 0, 0}, {0x3388, 0x0e, 0, 0}, {0x3389, 0x98, 0, 0},
	{0x338a, 0x01, 0, 0}, {0x3340, 0x0e, 0, 0}, {0x3341, 0x1a, 0, 0},
	{0x3342, 0x31, 0, 0}, {0x3343, 0x45, 0, 0}, {0x3344, 0x5a, 0, 0},
	{0x3345, 0x69, 0, 0}, {0x3346, 0x75, 0, 0}, {0x3347, 0x7e, 0, 0},
	{0x3348, 0x88, 0, 0}, {0x3349, 0x96, 0, 0}, {0x334a, 0xa3, 0, 0},
	{0x334b, 0xaf, 0, 0}, {0x334c, 0xc4, 0, 0}, {0x334d, 0xd7, 0, 0},
	{0x334e, 0xe8, 0, 0}, {0x334f, 0x20, 0, 0}, {0x3350, 0x32, 0, 0},
	{0x3351, 0x25, 0, 0}, {0x3352, 0x80, 0, 0}, {0x3353, 0x1e, 0, 0},
	{0x3354, 0x00, 0, 0}, {0x3355, 0x85, 0, 0}, {0x3356, 0x32, 0, 0},
	{0x3357, 0x25, 0, 0}, {0x3358, 0x80, 0, 0}, {0x3359, 0x1b, 0, 0},
	{0x335a, 0x00, 0, 0}, {0x335b, 0x85, 0, 0}, {0x335c, 0x32, 0, 0},
	{0x335d, 0x25, 0, 0}, {0x335e, 0x80, 0, 0}, {0x335f, 0x1b, 0, 0},
	{0x3360, 0x00, 0, 0}, {0x3361, 0x85, 0, 0}, {0x3363, 0x70, 0, 0},
	{0x3364, 0x7f, 0, 0}, {0x3365, 0x00, 0, 0}, {0x3366, 0x00, 0, 0},
	{0x3301, 0xff, 0, 0}, {0x338b, 0x11, 0, 0}, {0x338c, 0x10, 0, 0},
	{0x338d, 0x40, 0, 0}, {0x3370, 0xd0, 0, 0}, {0x3371, 0x00, 0, 0},
	{0x3372, 0x00, 0, 0}, {0x3373, 0x40, 0, 0}, {0x3374, 0x10, 0, 0},
	{0x3375, 0x10, 0, 0}, {0x3376, 0x04, 0, 0}, {0x3377, 0x00, 0, 0},
	{0x3378, 0x04, 0, 0}, {0x3379, 0x80, 0, 0}, {0x3069, 0x84, 0, 0},
	{0x3090, 0x0b, 0, 0}, //mirror
	{0x307c, 0x12, 0, 0}, {0x3087, 0x02, 0, 0}, {0x3300, 0xfc, 0, 0},
	{0x3302, 0x11, 0, 0}, {0x3400, 0x02, 0, 0}, {0x3606, 0x20, 0, 0},
	{0x3601, 0x30, 0, 0}, {0x300e, 0x34, 0, 0}, {0x30f3, 0x83, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x3086, 0x0f, 0, 0}, {0x3086, 0x00, 0, 0},
};

static struct reg_value ov2655_setting_15fps_SVGA_800_600[] = {
	{0x300e, 0x34},	{0x3011, 0x01},	{0x3012, 0x10},
	{0x302A, 0x02},	{0x302B, 0x88},	{0x306f, 0x14},

	{0x3020, 0x01},	{0x3021, 0x18},	{0x3022, 0x00},
	{0x3023, 0x06},	{0x3024, 0x06},	{0x3025, 0x58},
	{0x3026, 0x02},	{0x3027, 0x61},

	{0x3088, 0x03},	{0x3089, 0x20},	{0x308a, 0x02},
	{0x308b, 0x58},	{0x3316, 0x64},	{0x3317, 0x25},
	{0x3318, 0x80},	{0x3319, 0x08},	{0x331a, 0x64},
	{0x331b, 0x4b},	{0x331c, 0x00},	{0x331d, 0x38},
	{0x3302, 0x11}, {0x3362, 0x90}, {0x3376, 0x06, 0, 200},

};

static struct reg_value ov2655_setting_15fps_UXGA_1600_1200[] = {
	{0x300e, 0x34},	{0x3011, 0x01},	{0x3012, 0x00},	
	{0x302A, 0x04},	{0x302B, 0xd4},	{0x306f, 0x54},

	{0x3020, 0x01},	{0x3021, 0x18},	{0x3022, 0x00},
	{0x3023, 0x0a},	{0x3024, 0x06},	{0x3025, 0x58},
	{0x3026, 0x04},	{0x3027, 0xbc},


	{0x3088, 0x06},	{0x3089, 0x40},	{0x308a, 0x04},
	{0x308b, 0xb0},	{0x3316, 0x64},	{0x3317, 0x4b},
	{0x3318, 0x00},	{0x3319, 0x2c},	{0x331a, 0x64},
	{0x331b, 0x4b},	{0x331c, 0x00},	{0x331d, 0x4c},
	{0x3302, 0x01}, {0x3362, 0x80}, {0x3376, 0x08},
};

static struct reg_value ov2655_setting_15fps_SXGA_1280_1024[] = {

        {0x300e, 0x34}, {0x3011, 0x01}, {0x3012, 0x00},
        {0x302A, 0x04}, {0x302B, 0xd4}, {0x306f, 0x54},

        {0x3020, 0x01}, {0x3021, 0x18}, {0x3022, 0x00},
        {0x3023, 0x0a}, {0x3024, 0x06}, {0x3025, 0x58},
        {0x3026, 0x04}, {0x3027, 0xbc},

        {0x3088, 0x05}, {0x3089, 0x00}, {0x308a, 0x04},
        {0x308b, 0x00}, {0x3316, 0x64}, {0x3317, 0x4b},
        {0x3318, 0x00}, {0x3319, 0x2c}, {0x331a, 0x50},
        {0x331b, 0x40}, {0x331c, 0x00}, {0x331d, 0x4c},
        {0x3302, 0x11}, {0x3362, 0x80}, {0x3376, 0x08},
};

static struct reg_value ov2655_setting_15fps_SXGA_1280_960[] = {

        {0x300e, 0x34}, {0x3011, 0x01}, {0x3012, 0x00},
        {0x302A, 0x04}, {0x302B, 0xd4}, {0x306f, 0x54},

        {0x3020, 0x01}, {0x3021, 0x18}, {0x3022, 0x00},
        {0x3023, 0x0a}, {0x3024, 0x06}, {0x3025, 0x58},
        {0x3026, 0x04}, {0x3027, 0xbc},

        {0x3088, 0x05}, {0x3089, 0x00}, {0x308a, 0x03},
        {0x308b, 0xc0}, {0x3316, 0x64}, {0x3317, 0x4b},
        {0x3318, 0x00}, {0x3319, 0x2c}, {0x331a, 0x50},
        {0x331b, 0x3c}, {0x331c, 0x00}, {0x331d, 0x4c},
        {0x3302, 0x11}, {0x3362, 0x80}, {0x3376, 0x08},
};

static struct reg_value ov2655_setting_15fps_VGA_640_480[] = {

        {0x300e, 0x34}, {0x3011, 0x01}, {0x3012, 0x00},
        {0x302A, 0x04}, {0x302B, 0xd4}, {0x306f, 0x54},

        {0x3020, 0x01}, {0x3021, 0x18}, {0x3022, 0x00},
        {0x3023, 0x0a}, {0x3024, 0x06}, {0x3025, 0x58},
        {0x3026, 0x04}, {0x3027, 0xbc},

        {0x3088, 0x02}, {0x3089, 0x80}, {0x308a, 0x01},
        {0x308b, 0xe0}, {0x3316, 0x64}, {0x3317, 0x4b},
        {0x3318, 0x00}, {0x3319, 0x2c}, {0x331a, 0x28},
        {0x331b, 0x1e}, {0x331c, 0x00}, {0x331d, 0x4c},
        {0x3302, 0x11}, {0x3362, 0x80}, {0x3376, 0x08},
};

static struct reg_value ov2655_setting_15fps_D1_720_576[] = {
	{0x300e, 0x34},	{0x3011, 0x01},	{0x3012, 0x10},
	{0x302A, 0x02},	{0x302B, 0x88},	{0x306f, 0x14},
        {0x3070, 0x58}, {0x3072, 0x49},

	{0x3020, 0x01},	{0x3021, 0x18},	{0x3022, 0x00},
	{0x3023, 0x06},	{0x3024, 0x06},	{0x3025, 0x58},
	{0x3026, 0x02},	{0x3027, 0x61},

	{0x3088, 0x02},	{0x3089, 0xd0},	{0x308a, 0x02},
	{0x308b, 0x40},	{0x3316, 0x5d},	{0x3317, 0x25},
	{0x3318, 0x8c},	{0x3319, 0x08},	{0x331a, 0x64},
	{0x331b, 0x4b},	{0x331c, 0x00},	{0x331d, 0x38},
	{0x3302, 0x11, 0, 200},
};

static struct reg_value ov2655_setting_15fps_QVGA_320_240[] = {
	{0x300e, 0x34},	{0x3011, 0x01},	{0x3012, 0x10},
	{0x302A, 0x02},	{0x302B, 0x6a},	{0x306f, 0x14},
        {0x3070, 0x5d}, {0x3072, 0x4d},

	{0x3020, 0x01},	{0x3021, 0x18},	{0x3022, 0x00},
	{0x3023, 0x06},	{0x3024, 0x06},	{0x3025, 0x58},
	{0x3026, 0x02},	{0x3027, 0x61},

	{0x3088, 0x01}, {0x3089, 0x40}, {0x308a, 0x00},
	{0x308b, 0xf0}, {0x3316, 0x64}, {0x3317, 0x25},
        {0x3318, 0x80}, {0x3319, 0x08}, {0x331a, 0x14},
        {0x331b, 0x0f}, {0x331c, 0x00}, {0x331d, 0x38},
        {0x3302, 0x11, 0, 200},
};

static struct reg_value ov2655_setting_init_mode[] = {
	{0x3012, 0x80, 0, 10},
        {0x308c, 0x80, 0, 0}, {0x308d, 0x0e, 0, 0}, {0x360b, 0x00, 0, 0},
	{0x30b0, 0xff, 0, 0}, {0x30b1, 0xff, 0, 0}, {0x30b2, 0x24, 0, 0},
	{0x300e, 0x34, 0, 0}, {0x300f, 0xa6, 0, 0}, {0x3010, 0x81, 0, 0},
	{0x3082, 0x01, 0, 0}, {0x30f4, 0x01, 0, 0}, {0x3090, 0x3b, 0, 0},//mirror
	{0x3091, 0xc0, 0, 0}, {0x30ac, 0x42, 0, 0}, {0x30d1, 0x08, 0, 0},
	{0x30a8, 0x56, 0, 0}, {0x3015, 0x02, 0, 0}, {0x3093, 0x00, 0, 0},
	{0x307e, 0xe5, 0, 0}, {0x3079, 0x00, 0, 0}, {0x30aa, 0x42, 0, 0},
	{0x3017, 0x40, 0, 0}, {0x30f3, 0x82, 0, 0}, {0x306a, 0x0c, 0, 0},
	{0x306d, 0x00, 0, 0}, {0x336a, 0x3c, 0, 0}, {0x3076, 0x6a, 0, 0},
	{0x30d9, 0x8c, 0, 0}, {0x3016, 0x82, 0, 0}, {0x3601, 0x30, 0, 0},
	{0x304e, 0x88, 0, 0}, {0x30f1, 0x82, 0, 0}, {0x306f, 0x14, 0, 0},
	{0x302a, 0x02, 0, 0}, {0x302b, 0x88, 0, 0}, {0x3012, 0x10, 0, 0},
	{0x3011, 0x01, 0, 0},
        //AEC/AGC
        {0x3013, 0xff, 0, 0}, {0x301c, 0x13, 0, 0},
	{0x301d, 0x17, 0, 0}, {0x3070, 0x5d, 0, 0}, {0x3072, 0x4d, 0, 0},
	{0x3018, 0x80, 0, 0}, {0x3019, 0x70, 0, 0},
	{0x301a, 0xb4, 0, 0},
        //D5060
	{0x30af, 0x00, 0, 0}, {0x3048, 0x1f, 0, 0}, {0x3049, 0x4e, 0, 0},
	{0x304a, 0x20, 0, 0}, {0x304f, 0x20, 0, 0}, {0x304b, 0x02, 0, 0},
	{0x304c, 0x00, 0, 0}, {0x304d, 0x02, 0, 0}, {0x304f, 0x20, 0, 0},
	{0x30a3, 0x10, 0, 0}, {0x3013, 0xff, 0, 0}, {0x3014, 0x84, 0, 0},
	{0x3071, 0x00, 0, 0}, {0x3070, 0x58, 0, 0}, {0x3073, 0x00, 0, 0},
	{0x3072, 0x4d, 0, 0}, {0x301c, 0x05, 0, 0}, {0x301d, 0x06, 0, 0},
	{0x304d, 0x42, 0, 0}, {0x304a, 0x40, 0, 0}, {0x304f, 0x40, 0, 0},
	{0x3095, 0x07, 0, 0}, {0x3096, 0x16, 0, 0}, {0x3097, 0x1d, 0, 0},
        //Window Setup
	/*{0x300e, 0x38, 0, 0},*/
        {0x3020, 0x01, 0, 0}, {0x3021, 0x18, 0, 0},
	{0x3022, 0x00, 0, 0}, {0x3023, 0x06, 0, 0}, {0x3024, 0x06, 0, 0},
	{0x3025, 0x58, 0, 0}, {0x3026, 0x02, 0, 0}, {0x3027, 0x5e, 0, 0},
	{0x3088, 0x03, 0, 0}, {0x3089, 0x20, 0, 0}, {0x308a, 0x02, 0, 0},
	{0x308b, 0x58, 0, 0}, {0x3316, 0x64, 0, 0}, {0x3317, 0x25, 0, 0},
	{0x3318, 0x80, 0, 0}, {0x3319, 0x08, 0, 0}, {0x331a, 0x64, 0, 0},
	{0x331b, 0x4b, 0, 0}, {0x331c, 0x00, 0, 0}, {0x331d, 0x38, 0, 0},
	{0x3100, 0x00, 0, 0},
        //AWB
        {0x3320, 0xfa, 0, 0}, {0x3321, 0x11, 0, 0},
	{0x3322, 0x92, 0, 0}, {0x3323, 0x01, 0, 0}, {0x3324, 0x97, 0, 0},
	{0x3325, 0x02, 0, 0}, {0x3326, 0xff, 0, 0}, {0x3327, 0x0d, 0, 0},
	{0x3328, 0x10, 0, 0}, {0x3329, 0x10, 0, 0}, {0x332a, 0x59, 0, 0},
	{0x332b, 0x50, 0, 0}, {0x332c, 0xbe, 0, 0}, {0x332d, 0xb9, 0, 0},
	{0x332e, 0x3a, 0, 0}, {0x332f, 0x36, 0, 0}, {0x3330, 0x47, 0, 0},
	{0x3331, 0x44, 0, 0}, {0x3332, 0xf8, 0, 0}, {0x3333, 0x04, 0, 0},
	{0x3334, 0xf0, 0, 0}, {0x3335, 0xf0, 0, 0}, {0x3336, 0xf0, 0, 0},
	{0x3337, 0x40, 0, 0}, {0x3338, 0x40, 0, 0}, {0x3339, 0x40, 0, 0},
	{0x333a, 0x00, 0, 0}, {0x333b, 0x00, 0, 0},
        //Color Matrix
        {0x3380, 0x28, 0, 0},
	{0x3381, 0x48, 0, 0}, {0x3382, 0x10, 0, 0}, {0x3383, 0x23, 0, 0},
	{0x3384, 0xc0, 0, 0}, {0x3385, 0xe5, 0, 0}, {0x3386, 0xc2, 0, 0},
	{0x3387, 0xb3, 0, 0}, {0x3388, 0x0e, 0, 0}, {0x3389, 0x98, 0, 0},
	{0x338a, 0x01, 0, 0},
        //Gamma
        {0x3340, 0x0e, 0, 0}, {0x3341, 0x1d, 0, 0},
	{0x3342, 0x35, 0, 0}, {0x3343, 0x4a, 0, 0}, {0x3344, 0x5a, 0, 0},
	{0x3345, 0x69, 0, 0}, {0x3346, 0x75, 0, 0}, {0x3347, 0x7e, 0, 0},
	{0x3348, 0x88, 0, 0}, {0x3349, 0x96, 0, 0}, {0x334a, 0xa3, 0, 0},
	{0x334b, 0xaf, 0, 0}, {0x334c, 0xc4, 0, 0}, {0x334d, 0xd7, 0, 0},
	{0x334e, 0xe8, 0, 0}, {0x334f, 0x20, 0, 0},
        //Lens correction
        {0x3350, 0x31, 0, 0},
	{0x3351, 0x26, 0, 0}, {0x3352, 0x08, 0, 0}, {0x3353, 0x2e, 0, 0},
	{0x3354, 0x00, 0, 0}, {0x3355, 0x85, 0, 0}, {0x3356, 0x32, 0, 0},
	{0x3357, 0x26, 0, 0}, {0x3358, 0x00, 0, 0}, {0x3359, 0x2a, 0, 0},
	{0x335a, 0x00, 0, 0}, {0x335b, 0x85, 0, 0}, {0x335c, 0x32, 0, 0},
	{0x335d, 0x25, 0, 0}, {0x335e, 0x00, 0, 0}, {0x335f, 0x24, 0, 0},
	{0x3360, 0x00, 0, 0}, {0x3361, 0x85, 0, 0}, {0x3362, 0x90, 0, 0},
        {0x3363, 0x70, 0, 0},
	{0x3364, 0x7f, 0, 0}, {0x3365, 0x00, 0, 0}, {0x3366, 0x00, 0, 0},
        //UVadjust
	{0x3301, 0xff, 0, 0}, {0x338b, 0x11, 0, 0}, {0x338c, 0x10, 0, 0},
	{0x338d, 0x40, 0, 0}, {0x3370, 0xd0, 0, 0}, {0x3371, 0x00, 0, 0},
	{0x3372, 0x00, 0, 0}, {0x3373, 0x40, 0, 0}, {0x3374, 0x10, 0, 0},
	{0x3375, 0x10, 0, 0}, {0x3376, 0x06, 0, 0}, {0x3377, 0x00, 0, 0},
	{0x3378, 0x04, 0, 0}, {0x3379, 0x80, 0, 0},
        //BLC
        {0x3069, 0x84, 0, 0}, {0x307c, 0x10, 0, 0}, {0x3087, 0x02, 0, 0},
        //other function
        {0x3300, 0xfc, 0, 0}, {0x3302, 0x11, 0, 0}, {0x3400, 0x02, 0, 0},
        {0x3606, 0x20, 0, 0}, {0x3601, 0x30, 0, 0}, {0x300e, 0x34, 0, 0},
        {0x30f3, 0x83, 0, 0}, {0x304e, 0x88, 0, 0},
	{0x3391, 0x02, 0, 0}, {0x3394, 0x48, 0, 0}, {0x3395, 0x48, 0, 0},
        {0x3086, 0x0f, 0, 0}, {0x3086, 0x00, 0, 0},
};

static struct ov2655_mode_info ov2655_mode_info_data[2][ov2655_mode_MAX + 1] = {
    {
        {ov2655_mode_SVGA_800_600,    800,  600,
        ov2655_setting_30fps_SVGA_800_600,
        ARRAY_SIZE(ov2655_setting_30fps_SVGA_800_600)},
        {ov2655_mode_VGA_640_480, 640, 480, 
        ov2655_setting_30fps_VGA_640_480, 
        ARRAY_SIZE(ov2655_setting_30fps_VGA_640_480)},
        {ov2655_mode_QVGA_320_240, 320, 240, 
        ov2655_setting_30fps_QVGA_320_240, 
        ARRAY_SIZE(ov2655_setting_30fps_QVGA_320_240)},
        {ov2655_mode_UXGA_1600_1200, 0, 0, NULL, 0},
        {ov2655_mode_SXGA_1280_960, 0, 0, NULL, 0},
        {ov2655_mode_D1_720_576, 0, 0, NULL, 0},
        {ov2655_mode_SXGA_1280_1024, 0, 0, NULL, 0},
    },
	{
        {ov2655_mode_SVGA_800_600,    800,  600, 
        ov2655_setting_15fps_SVGA_800_600, 
        ARRAY_SIZE(ov2655_setting_15fps_SVGA_800_600)},
        {ov2655_mode_VGA_640_480, 640, 480, 
        ov2655_setting_15fps_VGA_640_480, 
        ARRAY_SIZE(ov2655_setting_15fps_VGA_640_480)},
        {ov2655_mode_QVGA_320_240, 320, 240, 
        ov2655_setting_15fps_QVGA_320_240, 
        ARRAY_SIZE(ov2655_setting_15fps_QVGA_320_240)},
        {ov2655_mode_UXGA_1600_1200, 1600, 1200, 
        ov2655_setting_15fps_UXGA_1600_1200, 
        ARRAY_SIZE(ov2655_setting_15fps_UXGA_1600_1200)},
        {ov2655_mode_SXGA_1280_960, 1280, 960,
        ov2655_setting_15fps_SXGA_1280_960,
        ARRAY_SIZE(ov2655_setting_15fps_SXGA_1280_960)},
        {ov2655_mode_D1_720_576, 720, 576, 
        ov2655_setting_15fps_D1_720_576, 
        ARRAY_SIZE(ov2655_setting_15fps_D1_720_576)},
        {ov2655_mode_SXGA_1280_1024, 1280, 1024,
        ov2655_setting_15fps_SXGA_1280_1024,
        ARRAY_SIZE(ov2655_setting_15fps_SXGA_1280_1024)},
	},
};

/*****************************************************\
 *************** Special Effects *********************
\*****************************************************/
enum ov2655_effect {
	OV2655_EFFECT_MIN		= 0,
	OV2655_SEPIA_EFFECT		= 0,
	OV2655_BLUISH_EFFECT	= 1,
	OV2655_GREENISH_EFFECT	= 2,
	OV2655_REDDISH_EFFECT	= 3,
	OV2655_YELLOWISH_EFFECT	= 4,
	OV2655_BW_EFFECT		= 5,
	OV2655_NEGATIVE_EFFECT	= 6,
	OV2655_NORMAL_EFFECT	= 7,
	OV2655_EFFECT_MAX		= 7,
};

struct ov2655_data_info {
//	enum ov2655_effect effect;
	struct reg_value *data_ptr;
	u32 data_size;
};
		
static struct reg_value sepia_effect[] = {
	{0x3391, 0x18, 0, 0},
	{0x3396, 0x40, 0, 0},
	{0x3397, 0xa6, 0, 0}
};

static struct reg_value bluish_effect[] = {
	{0x3391, 0x18, 0, 0},
	{0x3396, 0xa0, 0, 0},
	{0x3397, 0x40, 0, 0}
};

static struct reg_value greenish_effect[] = {
	{0x3391, 0x18, 0, 0},
	{0x3396, 0x60, 0, 0},
	{0x3397, 0x60, 0, 0}
};

static struct reg_value reddish_effect[] = {
	{0x3391, 0x18, 0, 0},
	{0x3396, 0x80, 0, 0},
	{0x3397, 0xc0, 0, 0}
};

static struct reg_value yellowish_effect[] = {
	{0x3391, 0x18, 0, 0},
	{0x3396, 0x30, 0, 0},
	{0x3397, 0x90, 0, 0}
};

static struct reg_value bw_effect[] = {
	{0x3391, 0x20, 0, 0}
};

static struct reg_value negative_effect[] = {
	{0x3391, 0x40, 0, 0}
};

static struct reg_value normal_effect[] = {
	{0x3391, 0x02, 0, 0}
};

static struct ov2655_data_info ov2655_effect_data_info[OV2655_EFFECT_MAX+1] = {
	{ sepia_effect,			ARRAY_SIZE(sepia_effect)},
	{ bluish_effect,		ARRAY_SIZE(bluish_effect)},
	{ greenish_effect,		ARRAY_SIZE(greenish_effect)},
	{ reddish_effect,		ARRAY_SIZE(reddish_effect)},
	{ yellowish_effect,		ARRAY_SIZE(yellowish_effect)},
	{ bw_effect,			ARRAY_SIZE(bw_effect)},
	{ negative_effect,		ARRAY_SIZE(negative_effect)},
	{ normal_effect,		ARRAY_SIZE(normal_effect)},
};
/*****************************************************\
 *************** Light Mode **************************
\*****************************************************/
enum ov2655_light {
	OV2655_LIGHT_MIN		= 0,
	OV2655_LIGHT_AUTO		= 0,
	OV2655_LIGHT_INCANDESCENT	= 1,
	OV2655_LIGHT_FLUORESCENT= 2,
	OV2655_LIGHT_DAYLIGHT	= 3,
	OV2655_LIGHT_SHADE	= 4,
	OV2655_LIGHT_MAX		= 4,
};

struct reg_value  ov2655_auto_light[] = {
	{0x3306, 0x00},
};

struct reg_value  ov2655_sunny_light[] = {
	{0x3306, 0x02},
	{0x3337, 0x5e},
	{0x3338, 0x40},
	{0x3339, 0x46},
};

struct reg_value  ov2655_cloudy_light[] = {
	{0x3306, 0x02},
	{0x3337, 0x68},
	{0x3338, 0x40},
	{0x3339, 0x4e},
};

struct reg_value  ov2655_office_light[] = {
	{0x3306, 0x02},
	{0x3337, 0x52},
	{0x3338, 0x40},
	{0x3339, 0x58},
};

struct reg_value  ov2655_home_light[] = {
	{0x3306, 0x02},
	{0x3337, 0x44},
	{0x3338, 0x40},
	{0x3339, 0x70},
};

static struct ov2655_data_info ov2655_light_data_info[OV2655_LIGHT_MAX+1] = {
	{ ov2655_auto_light,		ARRAY_SIZE(ov2655_auto_light)},
	{ ov2655_home_light,		ARRAY_SIZE(ov2655_home_light)},
	{ ov2655_office_light,		ARRAY_SIZE(ov2655_office_light)},
	{ ov2655_sunny_light,		ARRAY_SIZE(ov2655_sunny_light)},
	{ ov2655_cloudy_light,		ARRAY_SIZE(ov2655_cloudy_light)},
};

/*****************************************************\
 *************** Scene Mode **************************
\*****************************************************/

enum ov2655SceneMode {
	OV2655_SCENE_MODE_MIN		= 0,
	OV2655_SCENE_MODE_AUTO		= 0,
	OV2655_SCENE_MODE_NIGHT	= 1,
	OV2655_SCENE_MODE_MAX		= 1,
};

struct reg_value  ov2655_scene_auto[] = {
	{0x300e, 0x34},	{0x3011, 0x01},
	{0x302c, 0x00},	{0x3071, 0x00},
        {0x3070, 0x58}, {0x301c, 0x05},
        {0x3073, 0x00}, {0x3072, 0x49},
        {0x301d, 0x06},
};

//Night Mode with Auto Frame Rate 30fps ~ 5fps
struct reg_value  ov2655_scene_night[] = { 
	{0x300e, 0x34},	{0x3011, 0x05},
	{0x302c, 0x00},	{0x3071, 0x00},
	{0x3070, 0x31},	{0x301c, 0x13},
	{0x3073, 0x00},	{0x3072, 0x1a},
	{0x301d, 0x17},
};

static struct ov2655_data_info ov2655_scene_info[OV2655_SCENE_MODE_MAX+1] = {
	{ ov2655_scene_auto,		ARRAY_SIZE(ov2655_scene_auto)},
	{ ov2655_scene_night,		ARRAY_SIZE(ov2655_scene_night)},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct mxc_camera_platform_data *camera_plat;

static int ov2655_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov2655_remove(struct i2c_client *client);
static int ov2655_detect(void);

static s32 ov2655_read_reg(u16 reg, u8 *val);
static s32 ov2655_write_reg(u16 reg, u8 val);

static const struct i2c_device_id ov2655_id[] = {
	{"ov26xx", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov2655_id);

static struct i2c_driver ov2655_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov26xx",
		  },
	.probe  = ov2655_probe,
	.remove = ov2655_remove,
	.id_table = ov2655_id,
};

extern void gpio_sensor_active(unsigned int csi_index);
extern void gpio_sensor_inactive(unsigned int csi);

static s32 ov2655_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov2655_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov2655_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov2655_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov2655_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int ov2655_init_mode(enum ov2655_frame_rate frame_rate,
			    enum ov2655_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

        if (mode == ov2655_init_svga)
        {
                pModeSetting = ov2655_setting_init_mode;
                iModeSettingArySize = ARRAY_SIZE(ov2655_setting_init_mode);
                goto init_mode;
        }

	if (mode > ov2655_mode_MAX || mode < ov2655_mode_MIN) {
		pr_err("Wrong ov2655 mode detected!\n");
		return -1;
	}

	pModeSetting = ov2655_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov2655_mode_info_data[frame_rate][mode].init_data_size;

    pr_info("%s: frame_rate=%d, mode=%d\n", __func__, frame_rate, mode);
    ov2655_data.pix.width = ov2655_mode_info_data[frame_rate][mode].width;
    ov2655_data.pix.height = ov2655_mode_info_data[frame_rate][mode].height;

init_mode:
	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov2655_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov2655_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}

err:
	return retval;
}

static int ov2655_detect(void)
{
    u8 regval = 0;

	if (camera_plat->pwdn)
		camera_plat->pwdn(0);

    ipu_csi_enable_mclk_if(CSI_MCLK_I2C, ov2655_data.csi, true, true);

    ov2655_read_reg(0x300A, &regval);
    if (regval != 0x26) {
        ipu_csi_enable_mclk_if(CSI_MCLK_I2C, ov2655_data.csi, false, false);
		if (camera_plat->pwdn)
			camera_plat->pwdn(1);
        return 0;
    }
    ov2655_read_reg(0X300B, &regval);
    if (regval != 0x56) {
        ipu_csi_enable_mclk_if(CSI_MCLK_I2C, ov2655_data.csi, false, false);
		if (camera_plat->pwdn)
			camera_plat->pwdn(1);
        return 0;
    }

    ipu_csi_enable_mclk_if(CSI_MCLK_I2C, ov2655_data.csi, false, false);

	if (camera_plat->pwdn)
		camera_plat->pwdn(1);

    return 1;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov2655_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov2655_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = ov2655_XCLK_MIN;
	p->u.bt656.clock_max = ov2655_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	int retval;
	struct sensor *sensor = s->priv;
	static unsigned char regs[4] = {0xFF, 0xFF, 0xFF, 0xFF};

	if (on && !sensor->on) {
		gpio_sensor_active(ov2655_data.csi);

		/* Enable MCLK */
		ipu_csi_enable_mclk_if(CSI_MCLK_I2C, ov2655_data.csi, true, true);

		/* Delay from spec. */
		udelay(100);

		/* Pull PWDN pin to low */
		if (camera_plat->pwdn)
			camera_plat->pwdn(0);

		/* Restore registers */
		if (regs[0] != 0xFF && regs[1] != 0xFF && regs[2] != 0xFF && regs[3] != 0xFF) {
			retval = ov2655_write_reg(0x30AB, regs[0]);
			if (retval < 0)
				pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
			retval = ov2655_write_reg(0x30AD, regs[1]);
			if (retval < 0)
				pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
			retval = ov2655_write_reg(0x30AE, regs[2]);
			if (retval < 0)
				pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
			retval = ov2655_write_reg(0x363B, regs[3]);
			if (retval < 0)
				pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
		}

	} else if (!on && sensor->on) {
		/* Save and Set registers */
		retval = ov2655_read_reg(0x30AB, &regs[0]);
		if (retval < 0)
			pr_err("%s, %d: Camera senser read regs error\n", __func__, __LINE__);
		retval = ov2655_read_reg(0x30AD, &regs[1]);
		if (retval < 0)
			pr_err("%s, %d: Camera senser read regs error\n", __func__, __LINE__);
		retval = ov2655_read_reg(0x30AE, &regs[2]);
		if (retval < 0)
			pr_err("%s, %d: Camera senser read regs error\n", __func__, __LINE__);
		retval = ov2655_read_reg(0x363B, &regs[3]);
		if (retval < 0)
			pr_err("%s, %d: Camera senser read regs error\n", __func__, __LINE__);
		retval = ov2655_write_reg(0x30AB, 0x00);
		if (retval < 0)
			pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
		retval = ov2655_write_reg(0x30AD, 0x0A);
		if (retval < 0)
			pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
		retval = ov2655_write_reg(0x30AE, 0x27);
		if (retval < 0)
			pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
		retval = ov2655_write_reg(0x363B, 0x01);
		if (retval < 0)
			pr_err("%s, %d: Camera senser write regs error\n", __func__, __LINE__);
		
		/* Pull PWDN pin to high */
		if (camera_plat->pwdn)
			camera_plat->pwdn(1);

		/* Delay from spec. */
		udelay(100);

		/* Disable MCLK */
		ipu_csi_enable_mclk_if(CSI_MCLK_I2C, ov2655_data.csi, false, false);
		gpio_sensor_inactive(ov2655_data.csi);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov2655_frame_rate frame_rate;
	int ret = 0;

	/* Make sure power on */
	if (sensor->on != 1) {
		pr_err("%s, %d: Error: Called ioctl() but power is off, did not call open() first?!\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */

        pr_debug("%s: numerator=%d, denominator=%d\n", __func__, 
                timeperframe->numerator, timeperframe->denominator);

		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 30)
			frame_rate = ov2655_30_fps;
		else if (tgt_fps == 15)
			frame_rate = ov2655_15_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = ov2655_init_mode(frame_rate,
				       sensor->streamcap.capturemode);
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov2655_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov2655_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov2655_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov2655_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov2655_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov2655_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov2655_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	struct reg_value *pModeSetting;
	s32 iModeSettingArySize = 0;
	u8 Mask;
	u32 Delay_ms;
	u16 RegAddr;
	u8 Val;
	u8 RegVal;
	int i;
	
	pr_debug("In ov2655:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		//ov2655_write_reg(0x3320, 0x99);
		if ((vc->value < OV2655_LIGHT_MIN) || 
			(vc->value > OV2655_LIGHT_MAX)) {
			
			return -EPERM;
		}
		
		pModeSetting = 			ov2655_light_data_info[vc->value].data_ptr;
		iModeSettingArySize =	ov2655_light_data_info[vc->value].data_size;
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		if ((vc->value < OV2655_SCENE_MODE_MIN) || 
			(vc->value > OV2655_SCENE_MODE_MAX)) {
			
			return -EPERM;
		}
		
		pModeSetting = 			ov2655_scene_info[vc->value].data_ptr;
		iModeSettingArySize =	ov2655_scene_info[vc->value].data_size;
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_COLORFX:
		if ((vc->value < OV2655_EFFECT_MIN) || 
			(vc->value > OV2655_EFFECT_MAX)) {
			
			return -EPERM;
		}
		
		pModeSetting = 			ov2655_effect_data_info[vc->value].data_ptr;
		iModeSettingArySize =	ov2655_effect_data_info[vc->value].data_size;
		break;
	default:
		retval = -EPERM;
		break;
	}


	pr_err("******size: %d\n",iModeSettingArySize);
	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov2655_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				return retval;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov2655_write_reg(RegAddr, Val);
		if (retval < 0)
			return retval;

		if (Delay_ms)
			msleep(Delay_ms);
	}

	return retval;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov2655_frame_rate frame_rate;

	gpio_sensor_active(ov2655_data.csi);
	ov2655_data.on = true;

	/* mclk */
	tgt_xclk = ov2655_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)ov2655_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)ov2655_XCLK_MIN);
	ov2655_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&ov2655_data.mclk, ov2655_data.csi);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 30)
		frame_rate = ov2655_30_fps;
	else if (tgt_fps == 15)
		frame_rate = ov2655_15_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	return ov2655_init_mode(frame_rate,
				ov2655_init_svga);
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	gpio_sensor_inactive(ov2655_data.csi);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov2655_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
};

static struct v4l2_int_slave ov2655_slave = {
	.ioctls = ov2655_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov2655_ioctl_desc),
};

static struct v4l2_int_device ov2655_int_device = {
	.module = THIS_MODULE,
	.name = "ov26xx",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov2655_slave,
	},
};

/*!
 * ov2655 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov2655_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct mxc_camera_platform_data *plat_data = client->dev.platform_data;

	/* Set initial values for the sensor struct. */
	memset(&ov2655_data, 0, sizeof(ov2655_data));
	ov2655_data.mclk = 24000000; /* 6 - 54 MHz, typical 24MHz */
	ov2655_data.mclk = plat_data->mclk;
	ov2655_data.csi = plat_data->csi;

	ov2655_data.i2c_client = client;
	ov2655_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	ov2655_data.pix.width = 800;
	ov2655_data.pix.height = 600;
	ov2655_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov2655_data.streamcap.capturemode = 0;
	ov2655_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov2655_data.streamcap.timeperframe.numerator = 1;

	if (plat_data->io_regulator) {
		io_regulator = regulator_get(&client->dev,
					     plat_data->io_regulator);
		if (!IS_ERR(io_regulator)) {
			regulator_set_voltage(io_regulator,
					      ov2655_VOLTAGE_DIGITAL_IO,
					      ov2655_VOLTAGE_DIGITAL_IO);
			if (regulator_enable(io_regulator) != 0) {
				pr_err("%s:io set voltage error\n", __func__);
				goto err1;
			} else {
				dev_dbg(&client->dev,
					"%s:io set voltage ok\n", __func__);
			}
		} else
			io_regulator = NULL;
	}

	if (plat_data->core_regulator) {
		core_regulator = regulator_get(&client->dev,
					       plat_data->core_regulator);
		if (!IS_ERR(core_regulator)) {
			regulator_set_voltage(core_regulator,
					      ov2655_VOLTAGE_DIGITAL_CORE,
					      ov2655_VOLTAGE_DIGITAL_CORE);
			if (regulator_enable(core_regulator) != 0) {
				pr_err("%s:core set voltage error\n", __func__);
				goto err2;
			} else {
				dev_dbg(&client->dev,
					"%s:core set voltage ok\n", __func__);
			}
		} else
			core_regulator = NULL;
	}

	if (plat_data->analog_regulator) {
		analog_regulator = regulator_get(&client->dev,
						 plat_data->analog_regulator);
		if (!IS_ERR(analog_regulator)) {
			regulator_set_voltage(analog_regulator,
					      ov2655_VOLTAGE_ANALOG,
					      ov2655_VOLTAGE_ANALOG);
			if (regulator_enable(analog_regulator) != 0) {
				pr_err("%s:analog set voltage error\n",
					__func__);
				goto err3;
			} else {
				dev_dbg(&client->dev,
					"%s:analog set voltage ok\n", __func__);
			}
		} else
			analog_regulator = NULL;
	}

	if (plat_data->gpo_regulator) {
		gpo_regulator = regulator_get(&client->dev,
					      plat_data->gpo_regulator);
		if (!IS_ERR(gpo_regulator)) {
			regulator_set_voltage(gpo_regulator,
					      ov2655_VOLTAGE_DIGITAL_GPO,
					      ov2655_VOLTAGE_DIGITAL_GPO);
			if (regulator_enable(gpo_regulator) != 0) {
				pr_err("%s:gpo enable error\n", __func__);
				goto err4;
			} else {
				dev_dbg(&client->dev,
					"%s:gpo enable ok\n", __func__);
			}
		} else
			gpo_regulator = NULL;
	}

    camera_plat = plat_data;
	ov2655_int_device.priv = &ov2655_data;

    if (!ov2655_detect()) {
        not_detect = 1;
        pr_err("%s:ov2655 not detected!", __func__);
        goto err4;
    }

    pr_info("Camera ov2655 detected!\n");

	retval = v4l2_int_device_register(&ov2655_int_device);

	return retval;

err4:
	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}
err3:
	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}
err2:
	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}
err1:
	return -1;
}

/*!
 * ov2655 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov2655_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov2655_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}

	return 0;
}

/*!
 * ov2655 init function
 * Called by insmod ov2655_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov2655_init(void)
{
	u8 err;

    pr_debug("ov2655_init Start...\n");

	err = i2c_add_driver(&ov2655_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

    if (not_detect)
        i2c_del_driver(&ov2655_i2c_driver);

    pr_debug("ov2655_init Complete.\n");

	return err;
}

/*!
 * ov2655 cleanup function
 * Called on rmmod ov2655_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov2655_clean(void)
{
	i2c_del_driver(&ov2655_i2c_driver);
}

module_init(ov2655_init);
module_exit(ov2655_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ov2655 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
