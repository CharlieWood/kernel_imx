/*
 * DS2778 battery driver
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
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/battery_device_info.h>
#include <linux/slab.h>

#define DRIVER_VERSION			"1.0.1"
#define DS2778_REG_RARC 		0x06	/* Remaining active relative capacity */
#define DS2778_REG_ID			0x69	/* use the RSNSP_REG */

#define DS2778_REG_VOLT_IN1_MSB 	0x0c
#define DS2778_REG_VOLT_IN2_MSB		0x1c
#define DS2778_REG_TEMP_MSB	   	0x0a
#define DS2778_REG_CURRENT_MSB	   	0x0e
#define DS2778_REG_CURRENT_AVG_MSB 	0x08
/* EEPROM Block */
#define DS2778_REG_RSNSP           	0x69	/* Sense resistor value */

#define DS2778_CURRENT_UNITS       	1563
#define DS2778_POLL_MSEC           	5000
#define DS2778_GPIO_POLL_MSEC      	1000
#define ALARM_TEMP_VALUE        	650
#define	BATTERY_VOLTAGE_OF_EMPTY_UV	7000000

#define	GPIO_AC_IN			1
#define	GPIO_CHARGING_FULL		1

//#define DS2778_CFG_DEBUG
#undef	DS2778_CFG_DEBUG
#define	DS2778_DEBUG_ON
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static inline int ds2778_read_reg(struct battery_device_info *di, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(di->client, reg);
	if (ret < 0) {
		dev_err(&di->client->dev, "register read failed\n");
		return ret;
	}

	*val = ret;

	return 0;
}

static inline int ds2778_read_reg16(struct battery_device_info *di, int reg_msb,
		s16 *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(di->client, reg_msb);
	if (ret < 0) {
		dev_err(&di->client->dev, "register read failed\n");
		return ret;
	}

	*val = swab16(ret);
	return 0;
}

/*
 * Return the battery temperature in Celsius degrees
 * Or < 0 if something fails.
 */
static int battery_temperature(struct battery_device_info *di)
{
	s16 raw;
	int ret;

	/*
	 * Temperature is measured in units of 0.125 degrees celcius, the
	 * power_supply class measures temperature in tenths of degrees
	 * celsius. The temperature value is stored as a 10 bit number, plus
	 * sign in the upper bits of a 16 bit register.
	 */
	ret = ds2778_read_reg16(di, DS2778_REG_TEMP_MSB, &raw);
	if (ret)
		return ret;

	return ((raw / 32) * 125) / 100;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int battery_voltage(struct battery_device_info *di)
{
	s16 raw_1;
	s16 raw_2;
	int raw;
	int ret;

	/*
	 * Voltage is measured in units of 4.88mV. The voltage is stored as
	 * a 10-bit number plus sign, in the upper bits of a 16-bit register
	 */
	ret = ds2778_read_reg16(di, DS2778_REG_VOLT_IN1_MSB, &raw_1);
	if (ret) {
		dev_err(di->dev, "error reading voltage in1\n");
		return ret;
	}
	ret = ds2778_read_reg16(di, DS2778_REG_VOLT_IN2_MSB, &raw_2);
	if (ret) {
		dev_err(di->dev, "error reading voltage in2\n");
		return ret;
	}

	raw = (raw_1 / 32) * 4883;
	raw += (raw_2 / 32) * 4883;
	return raw;
}

static int battery_current_now(struct battery_device_info *di)
{
	int sense_res;
	int ret;
	int current_uA;
	u8 sense_res_raw;
	s16 raw;

	/*
	 * The units of measurement for current are dependent on the value of
	 * the sense resistor.
	 */
	ret = ds2778_read_reg(di, DS2778_REG_RSNSP, &sense_res_raw);
	if (ret)
		return ret;
	if (sense_res_raw == 0) {
		dev_err(&di->client->dev, "sense resistor value is 0\n");
		return -ENXIO;
	}
	sense_res = 1000 / sense_res_raw;

	dev_dbg(&di->client->dev, "sense resistor = %d milli-ohms\n",
			sense_res);
	ret = ds2778_read_reg16(di, DS2778_REG_CURRENT_MSB, &raw);

	if (ret)
		return ret;

	current_uA = raw * (DS2778_CURRENT_UNITS / sense_res);

	if (current_uA < 0) current_uA = -(current_uA); /* battery is discharging */
#ifdef DS2778_DEBUG_ON
	/* nothing to do */
#else
	else current_uA = 1000; /* battery is charging */
#endif

	return current_uA;
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
 */
static int battery_current_avg(struct battery_device_info *di)
{
	int sense_res;
	int ret;
	u8 sense_res_raw;
	s16 raw;
	int current_avg_uA;

	/*
	 * The units of measurement for current are dependent on the value of
	 * the sense resistor.
	 */
	ret = ds2778_read_reg(di, DS2778_REG_RSNSP, &sense_res_raw);
	if (ret)
		return ret;
	if (sense_res_raw == 0) {
		dev_err(&di->client->dev, "sense resistor value is 0\n");
		return -ENXIO;
	}
	sense_res = 1000 / sense_res_raw;
	dev_dbg(&di->client->dev, "sense resistor = %d milli-ohms\n",
			sense_res);

	ret = ds2778_read_reg16(di, DS2778_REG_CURRENT_AVG_MSB, &raw);
	if (ret)
		return ret;

	current_avg_uA = raw * (DS2778_CURRENT_UNITS / sense_res);
	if (current_avg_uA < 0) current_avg_uA = -current_avg_uA;
#ifdef DS2778_DEBUG_ON
	/* nothing to do */
#else
	else current_avg_uA = 1000;
#endif
	return current_avg_uA;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int battery_rsoc(struct battery_device_info *di)
{
	int ret;
	u8 raw;

	ret = ds2778_read_reg(di, DS2778_REG_RARC, &raw);
	if (ret)
		return ret;
	return raw;
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

static void ds2778_external_power_changed(struct power_supply *psy)
{
	struct battery_device_info *di = container_of(psy, struct battery_device_info, bat);
	printk("%s\n", __func__);
	atomic_set(&di->changed, 1);
	cancel_delayed_work_sync(&di->monitor_work);
	queue_delayed_work(di->battery_wqueue, &di->monitor_work, msecs_to_jiffies(DS2778_GPIO_POLL_MSEC));
}

static void power_supply_init(struct battery_device_info *di)
{

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = battery_props;
	di->bat.num_properties = ARRAY_SIZE(battery_props);
	di->bat.get_property = battery_get_property;
	di->bat.external_power_changed = ds2778_external_power_changed;

	di->charging = -1;
	di->charge_rsoc_now = -1;
	di->charge_rsoc_pre = -1;
}

static void ds2778_monitor_work(struct work_struct *work)
{
	int val;
	int battery_changed = 0;
	struct battery_device_info *di = container_of(work, struct battery_device_info, monitor_work.work);

	wake_lock(&di->wake_lock);

	val = battery_voltage(di);
	if (val >= 0 && (val != di->voltage_uV))
		di->voltage_uV = val;

	val = battery_charging(di);
	if (val != di->charging) {
		di->charging = val;
		battery_changed = 1;
		printk("power_supply_changed: charging(%d)\n", di->charging);
	}

	val = battery_current_now(di);
	if (val >= 0 && val != di->current_uA) {
		di->current_uA = val;
	}

	val = battery_current_avg(di);
	if (val >= 0 && val != di->current_avg_uA) {
		di->current_avg_uA = val;
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
				msecs_to_jiffies(DS2778_POLL_MSEC));
}

#ifdef  DS2778_CFG_DEBUG
static void ds2778_reg_cfg_get(struct i2c_client *client)
{
	int retval = 0;
	int val;
#define DS2778_REG_VCHG         0x64 /* charge voltage */
#define DS2778_REG_IMIN         0x65 /* minimum charge current */
#define DS2778_REG_VAE          0x66 /* active-empty voltage */
#define DS2778_REG_IAE          0x67 /* active-empty current */
#define DS2778_REG_RSNSP        0x69 /* sense resistor */
#define DS2778_DEBUG_STRING	"+++++++++++++++++++++"

	retval = i2c_smbus_read_byte_data(client, DS2778_REG_VCHG);
	val = retval * 195 / 10;
	printk(KERN_INFO "%s register VCHG is 0x%x, charge voltage = %d mV\n", DS2778_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, DS2778_REG_IMIN);
	val = retval *50 / 25;
	printk(KERN_INFO "%s register IMIN is 0x%x, minimum charge current = %d mA\n", DS2778_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, DS2778_REG_VAE);
	val = retval * 195 / 10;
	printk(KERN_INFO "%s register VAE is 0x%x, active-empty voltage : %d mV\n", DS2778_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, DS2778_REG_IAE);
	val =retval * 200 / 25;
	printk(KERN_INFO "%S register IAE is 0x%x, active-empty current : %d mA\n", DS2778_DEBUG_STRING, retval, val);

	retval = i2c_smbus_read_byte_data(client, DS2778_REG_RSNSP);
	val = 1000 / retval;
	printk(KERN_INFO "%s register RSNSP is 0x%x, sense resistor : %d mohm\n", DS2778_DEBUG_STRING, retval, val);
}
#endif

static int ds2778_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct battery_device_info *di;
	struct battery_platform_data *pdata;
	int num;
	int retval = 0;
	int wakeup = 1;

	dev_dbg(&client->dev, "%s()\n", __func__);

	pdata = client->dev.platform_data;

	retval = i2c_smbus_read_byte_data(client, DS2778_REG_ID);
	if (retval < 0) {
		printk(KERN_ERR "%s: Battery DS2778 does not exist!\n", __func__);

		/* register device --13892 */
		if (pdata->psy_register_device) {
			pdata->psy_register_device();
		} else
			printk(KERN_ERR "%s:psy_register_device can not be called\n", __func__ );

		return retval;
	}

#ifdef  DS2778_CFG_DEBUG
	ds2778_reg_cfg_get(client);
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

	INIT_DELAYED_WORK(&di->monitor_work, ds2778_monitor_work);
	di->battery_wqueue = create_singlethread_workqueue("ds2778_battery");
	if (!di->battery_wqueue) {
		dev_err(&client->dev, "failed to create workqueue.\n");
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

static int ds2778_battery_remove(struct i2c_client *client)
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

static int ds2778_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s\n", __func__);
	return 0;
}

static int ds2778_battery_resume(struct i2c_client *client)
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
static const struct i2c_device_id ds2778_id[] = {
	{ "ds2778", 0 },
	{},
};

static struct i2c_driver battery_driver = {
	.driver = {
		.name = "ds2778",
	},
	.probe = ds2778_battery_probe,
	.remove = ds2778_battery_remove,
	.suspend = ds2778_battery_suspend,
	.resume	= ds2778_battery_resume,
	.id_table = ds2778_id,
};

static int __init battery_init(void)
{
	int ret;

	pr_info("%s: register ds2778 battery driver\n", __func__);
	ret = i2c_add_driver(&battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register DS2778 driver\n");

	return ret;
}

static void __exit battery_exit(void)
{
	i2c_del_driver(&battery_driver);
}

module_init(battery_init);
module_exit(battery_exit);

MODULE_LICENSE("GPL");
