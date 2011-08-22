/*
 *  linux/include/linux/gps/gps.h
 */

#ifndef LINUX_GPS_CONTROL_H
#define LINUX_GPS_CONTROL_H

struct gps_control_data {
	int (*set_power)(int);
	int (*get_status)(void);
};

#endif
