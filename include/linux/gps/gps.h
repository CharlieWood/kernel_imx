/*
 *  linux/include/linux/gps/gps.h
 */

#ifndef LINUX_GPS_CONTROL_H
#define LINUX_GPS_CONTROL_H

struct gps_control_data {
	int (*set_power)(int);	/* power enable */
	int (*set_nrst)(int);	/* nReset control */
};

#endif
