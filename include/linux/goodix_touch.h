/*
 * include/linux/goodix_touch.h
 *
 * Copyright (C) 2008 Goodix, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef	_LINUX_GOODIX_TOUCH_H
#define	_LINUX_GOODIX_TOUCH_H

#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>

#define GOODIX_I2C_NAME "Goodix-TS"
#define GUITAR_GT80X
#define INT_PORT_ENABLE

#define GOODIX_MULTI_TOUCH
#ifndef GOODIX_MULTI_TOUCH
	#define MAX_FINGER_NUM 1
#else
	#define MAX_FINGER_NUM 5
#endif
#if defined(INT_PORT_ENABLE)
	#if MAX_FINGER_NUM <= 3
	#define READ_BYTES_NUM 1 + 2 + MAX_FINGER_NUM * 5
	#elif MAX_FINGER_NUM == 4
	#define READ_BYTES_NUM 1 + 28
	#elif MAX_FINGER_NUM == 5
	#define READ_BYTES_NUM 1 + 34
	#endif
#else
	#define READ_BYTES_NUM 1 + 34
#endif

#define goodix_swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

struct goodix_touchkey {
	int touch_key;
	int bit;

	int touch_max_valid;
	int y_pos;
};

struct goodix_i2c_platform_data {
	unsigned int touch_max_height;
	unsigned int touch_max_width;
	unsigned int screen_max_height;
	unsigned int screen_max_width;

	uint32_t gpio_irq;
	uint32_t gpio_reset;

	struct goodix_touchkey *goodix_touchkey_map;
};

#endif
