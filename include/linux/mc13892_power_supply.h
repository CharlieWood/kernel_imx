#include <linux/power_supply.h>
#ifndef MC13892_POWER_SUPPLY_H
#define MC13892_POWER_SUPPLY_H
struct mc13892_power_supply_info {
	struct device *dev;

	struct power_supply bat;
	struct power_supply charger;

	int batt_max;
	int batt_min;
	int charging_data_correct;
	int bat_cur_status;
	int bat_old_status;
	int bat_cur_temp;
	int bat_old_temp;
	int bat_cur_vol;
	int bat_old_vol;
	int bat_cur_cur;
	int bat_old_cur;
	int bat_old_capacity;
	int bat_cur_capacity;

	int chg_det_irq;
	int low_bat_wakeup;
	int charger_online;
	int charger_cur;
	int charge_ok;
	int charger_cur_status;
	int charger_old_status;
	
	int (*is_cable_in)(void);
	int (*is_charge_ok)(void);

	unsigned int (*get_bat_voltage)(void);
	unsigned int (*get_bat_current)(void);
	unsigned int (*get_bat_capacity)(void);

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
	struct wake_lock bat_wakelock;
};

struct mc13892_power_supply_pdata {
	int batt_max;
	int batt_min;
	int charging_data_correct;

	int (*is_cable_in)(void);
	int (*is_charge_ok)(void);

	unsigned int (*get_bat_voltage)(void);
	unsigned int (*get_bat_current)(void);
	unsigned int (*get_bat_capacity)(void);
};
#endif
