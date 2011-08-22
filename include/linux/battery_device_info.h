#ifndef _BATTERY_DEVICE_INFO_H
#define _BATTERY_DEVICE_INFO_H

#include <linux/wakelock.h>
#include <linux/power_supply.h>

struct battery_platform_data {
	int bat_voltage_of_zero_uV;

	int (*is_online)(void);
	int (*is_charging_full)(void);
	void (*psy_register_device)(void);

};

struct battery_device_info {
	struct device           *dev;
	int                     id;
	int			bat_voltage_of_zero_uV;
	int                     voltage_uV;
	int                     current_uA;
	int                     current_avg_uA;
	int                     temp_C;
	int                     charge_rsoc_now;
	int                     charge_rsoc_pre;
	int                     charging_pre;
	int                     charging;
	int                     health;
	atomic_t                changed;
	struct power_supply     bat;
	struct i2c_client       *client;
	struct workqueue_struct *battery_wqueue;
	struct delayed_work     monitor_work;
	struct wake_lock        wake_lock;
	struct wake_lock        wake_lock_timeout;
	struct battery_platform_data *bat_plat_pdata;
};

#endif
