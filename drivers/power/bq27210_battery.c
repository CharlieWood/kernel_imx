/*
 * bq27210 battery driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/battery_device_info.h>

#define DRIVER_VERSION		"1.0.1"
#define BQ27210_REG_ID		0x63

#define BQ27210_REG_TEMP	0x06
#define BQ27210_REG_VOLT	0x08
#define BQ27210_REG_RSOC	0x0B
#define BQ27210_REG_AI		0x14
#define BQ27210_REG_FLAGS	0x0A
#define RSNS			20 					/* Units:millionhohms */

#define BQ27210_POLL_MSEC	5000
#define BQ27210_GPIO_POLL_MSEC	1000

#define KELVIN_TEMP_COEFFICIENT 2731	/* in fact, it's ten times of Kelvin temperature coefficient*/
#define	BATTERY_VOLTAGE_OF_EMPTY_UV	7000000

#define ALARM_TEMP_VALUE 	650

#define	GPIO_AC_IN			1
#define	GPIO_CHARGING_FULL		1

//#define BQ27210_CFG_DEBUG
#undef	BQ27210_CFG_DEBUG
#define	BQ27210_DEBUG_ON

/* If the system has several batteries we need a different name for each
 * of them
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

/*
 * Return the battery temperature in ten times of Celsius degrees
 * Or < 0 if something fails.
 */
static int battery_temperature(struct battery_device_info *di)
{
	int ret = 0;

	ret = i2c_smbus_read_word_data(di->client, BQ27210_REG_TEMP);
	if (ret < 0) {
		dev_err(di->dev, "error reading temperature %d\n", ret);
		return ret;
	}

	return ((ret * 10) >> 2) - KELVIN_TEMP_COEFFICIENT;
}

/*
 * Return the battery Voltage in uV
 * Or < 0 if something fails.
 */
static int battery_voltage(struct battery_device_info *di)
{
	int ret;

	ret = i2c_smbus_read_word_data(di->client, BQ27210_REG_VOLT);
	if (ret < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}
	ret = ret * 1000; /* almost all units are in micro, we'll get uA */
	return ret * 2;
}

static int battery_charging(struct battery_device_info *di)
{
	int ac_ok, charging_ok;
	int ret;

	ac_ok = di->bat_plat_pdata->is_online();
	charging_ok = di->bat_plat_pdata->is_charging_full();

	if (ac_ok == GPIO_AC_IN) {
		if (charging_ok == GPIO_CHARGING_FULL)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
	} else
		ret = POWER_SUPPLY_STATUS_DISCHARGING;

	return ret;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int battery_current_avg(struct battery_device_info *di)
{
	int curr = 0;
	int flags = 0;

	curr = i2c_smbus_read_word_data(di->client, BQ27210_REG_AI);
	if (curr < 0) {
		dev_err(di->dev, "error reading charging current\n");
		return curr;
	}

	flags = i2c_smbus_read_byte_data(di->client, BQ27210_REG_FLAGS);
	if (flags < 0) {
		dev_err(di->dev, "error reading flags\n");
		return flags;
	}

#ifdef BQ27210_DEBUG_ON
	curr = curr * 3570 / RSNS ;
	return curr;
#else
	if ((flags & (1 << 7)) != 0) {
		return 1000;
	} else {
		curr = curr * 3570 / RSNS ;
		return curr;
	}
#endif
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int battery_rsoc(struct battery_device_info *di)
{
	int ret;

	ret = i2c_smbus_read_byte_data(di->client, BQ27210_REG_RSOC);
	if (ret < 0) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return ret;
}

#define to_battery_device_info(x) container_of((x), struct battery_device_info, bat);
static int battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_device_info *di = to_battery_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->voltage_uV;
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->charging;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->current_avg_uA;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->charge_rsoc_now;
		if (val->intval < 0)
			val->intval = 20;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		if (val->intval < 0)
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charging;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void bq27210_external_power_changed(struct power_supply *psy)
{
	struct battery_device_info *di = container_of(psy, struct battery_device_info, bat);
	printk("%s\n", __func__);
	atomic_set(&di->changed, 1);
	cancel_delayed_work_sync(&di->monitor_work);
	queue_delayed_work(di->battery_wqueue, &di->monitor_work, msecs_to_jiffies(BQ27210_GPIO_POLL_MSEC));
}

static void power_supply_init(struct battery_device_info *di)
{

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = battery_props;
	di->bat.num_properties = ARRAY_SIZE(battery_props);
	di->bat.get_property = battery_get_property;
	di->bat.external_power_changed = bq27210_external_power_changed;

	di->charge_rsoc_now = -1;
	di->charge_rsoc_pre = -1;
	di->charging = -1;
}

static void bq27210_monitor_work(struct work_struct *work)
{
	int battery_changed = 0;
	int val;
	struct battery_device_info *di = container_of(work, struct battery_device_info, monitor_work.work);

	wake_lock(&di->wake_lock);

	val = battery_voltage(di);
	if (val >= 0 && val != di->voltage_uV)
		di->voltage_uV = val;

	val = battery_current_avg(di);
	if (val >= 0 && val != di->current_avg_uA)
		di->current_avg_uA = val;

	val = battery_charging(di);
	if (di->charging != val) {
		di->charging = val;
		battery_changed = 1;
		printk("power_supply_changed: charging(%d)\n", di->charging);
	}


	val = battery_rsoc(di);
	if (val >= 0 && val <= 100 && val != di->charge_rsoc_pre) {
		di->charge_rsoc_now = val;
	}

	if (di->charging == POWER_SUPPLY_STATUS_DISCHARGING) {
		if (di->charge_rsoc_now <= 5 && di->voltage_uV <= di->bat_plat_pdata->bat_voltage_of_zero_uV) {
			di->charge_rsoc_now = 0;

			/* the system will be shutdown, not be suspended */
			wake_lock_timeout(&di->wake_lock_timeout, 10 * HZ);
		}

		if (di->charge_rsoc_now == 100) /* when discharging, 100% will not be appeared */
			di->charge_rsoc_now = 99;
	} else if (di->charging == POWER_SUPPLY_STATUS_CHARGING) {
		if (di->charge_rsoc_now == 100)
			di->charge_rsoc_now = 99;
	} else if (di->charging == POWER_SUPPLY_STATUS_FULL) {
		di->charge_rsoc_now = 100;
	}

	if (di->charge_rsoc_now != di->charge_rsoc_pre) {
		di->charge_rsoc_pre = di->charge_rsoc_now;
		battery_changed = 1;
		printk("power_supply_changed: rsoc(%d)\n", di->charge_rsoc_now);
	}

	val = battery_temperature(di);
	if (val >= 0 && val != di->temp_C) {
		di->temp_C = val;
		if (di->temp_C >= ALARM_TEMP_VALUE) {
			battery_changed = 1;
			printk("power_supply_changed: temp(%d)\n", di->temp_C);
		}
	}

	if (battery_changed) {
		atomic_set(&di->changed, 0);
		power_supply_changed(&di->bat);
	}
	wake_unlock(&di->wake_lock);

	queue_delayed_work(di->battery_wqueue, &di->monitor_work,
			atomic_read(&di->changed) ?
				msecs_to_jiffies(500) :
				msecs_to_jiffies(BQ27210_POLL_MSEC));
}

#ifdef BQ27210_CFG_DEBUG
static void bq27210_reg_cfg_get(struct i2c_client *client)
{
	int retval = 0;
	int val;

#define BQ27210_REG_ILMD        0x76 /* value of battery capacity, mAh */
#define BQ27210_REG_SEDVF       0x77 /* the voltage of 0% */
#define BQ27210_REG_SEDV1       0x78 /* voltage of 6.25% */
#define BQ27210_REG_ISLC        0x79
#define BQ27210_REG_TAPER       0x7B /* taper current */
#define BQ27210_CFG_DEBUG_STRING "+++++++++++++++++++++++"
	retval = i2c_smbus_read_byte_data(client, BQ27210_REG_ILMD);
	val = retval * 357 * 256 / 20 / 100;
	printk(KERN_INFO "%s register ILMD is 0x%x, design capacity = %d mAh\n", BQ27210_CFG_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, BQ27210_REG_SEDVF);
	val = (retval + 256) * 8;
	printk(KERN_INFO "%s register SEDVF is 0x%x, voltage of 0%% : %d mV\n", BQ27210_CFG_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, BQ27210_REG_SEDV1);
	val = (retval + 256) * 8;
	printk(KERN_INFO "%s register SEDV1 is 0x%x, voltage of 6.25%% : %d mV\n", BQ27210_CFG_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, BQ27210_REG_TAPER);
	val = (retval & 0x7F) * 228 / 20;
	printk(KERN_INFO "%s register TAPER is 0x%x, taper current is : %d mA\n", BQ27210_CFG_DEBUG_STRING, retval, val);
}
#endif

static int bq27210_battery_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct battery_device_info *di;
	struct battery_platform_data *pdata;
	int num;
	int retval = 0;
	int wakeup = 1;

	dev_dbg(&client->dev, "%s()\n", __func__);

	pdata = client->dev.platform_data;

	retval = i2c_smbus_read_word_data(client, BQ27210_REG_ID);
	if (retval < 0) {
		printk(KERN_ERR "%s:Battery BQ27210 does not exist!\n", __func__ );

		/* register device --13892 */
		if (pdata->psy_register_device) {
			pdata->psy_register_device();
		} else
			printk(KERN_ERR "%s:psy_register_device can not be called\n", __func__ );

		return retval;
	}

#ifdef BQ27210_CFG_DEBUG
	bq27210_reg_cfg_get(client);
#endif

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;
	atomic_set(&di->changed, 0);
	di->bat_plat_pdata = client->dev.platform_data;

	INIT_DELAYED_WORK(&di->monitor_work, bq27210_monitor_work);
	di->battery_wqueue = create_singlethread_workqueue("bq27210_battery");
	if (!di->battery_wqueue) {
		dev_err(&client->dev, "%s:failed to create workqueue.\n", __func__);
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	power_supply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_3;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);
	wake_lock_init(&di->wake_lock, WAKE_LOCK_SUSPEND, "battery_wake_lock");
	wake_lock_init(&di->wake_lock_timeout, WAKE_LOCK_SUSPEND, "battery_wake_lock_timeout");
	queue_delayed_work(di->battery_wqueue, &di->monitor_work, msecs_to_jiffies(100));

	device_init_wakeup(&client->dev, wakeup);

	return 0;
batt_failed_3:
	destroy_workqueue(di->battery_wqueue);
batt_failed_2:
	kfree(di);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);
	return retval;
}

static int bq27210_battery_remove(struct i2c_client *client)
{
	struct battery_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&di->monitor_work);
	destroy_workqueue(di->battery_wqueue);
	wake_lock_destroy(&di->wake_lock);
	wake_lock_destroy(&di->wake_lock_timeout);
	power_supply_unregister(&di->bat);
	kfree(di->bat.name);
	kfree(di);
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	return 0;
}

static int bq27210_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s\n", __func__);
	return 0;
}

static int bq27210_battery_resume(struct i2c_client *client)
{
	struct battery_device_info *di = i2c_get_clientdata(client);
	printk("%s\n", __func__);
	cancel_delayed_work_sync(&di->monitor_work);
	queue_delayed_work(di->battery_wqueue, &di->monitor_work, msecs_to_jiffies(800));
	return 0;
}

/*
 * Module stuff
 */
static const struct i2c_device_id bq27210_id[] = {
	{ "bq27210", 0 },
	{},
};

static struct i2c_driver battery_driver = {
	.driver = {
		.name = "bq27210",
	},
	.probe = bq27210_battery_probe,
	.remove = bq27210_battery_remove,
	.suspend = bq27210_battery_suspend,
	.resume	= bq27210_battery_resume,
	.id_table = bq27210_id,
};

static int __init battery_init(void)
{
	int ret;

	pr_info("%s: register bq27210 battery driver\n", __func__);
	ret = i2c_add_driver(&battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register bq27210 driver\n");

	return ret;
}

static void __exit battery_exit(void)
{
	i2c_del_driver(&battery_driver);
}

module_init(battery_init);
module_exit(battery_exit);

MODULE_LICENSE("GPL");
