#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mc13892_power_supply.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pmic_adc.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>

#include"battery_voltage.h"

int log_enable = 0;

static struct mc13892_power_supply_info *mc13892_psy_info = NULL;

static enum power_supply_property aster_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
};

static int aster_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mc13892_psy_info->bat_old_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mc13892_psy_info->bat_cur_vol * 10000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = mc13892_psy_info->bat_cur_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = mc13892_psy_info->bat_old_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = mc13892_psy_info->batt_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = mc13892_psy_info->batt_min;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = 100;
		if (mc13892_psy_info->bat_old_capacity < 0) {
			val->intval = mc13892_psy_info->bat_cur_capacity;
		} else
			val->intval = mc13892_psy_info->bat_old_capacity;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void avoid_abnormal_voltage(int i, int *value)
{
	int num;
	int *dvalue = value;

	for (num=0; (num<4)&&(i!=0); num++) {
		if (mc13892_psy_info->bat_old_vol- mc13892_psy_info->bat_cur_vol > dvalue[i]) {
			udelay(200);
			mc13892_psy_info->bat_cur_vol = ((mc13892_psy_info->get_bat_voltage() + 150000)/10000);
		} else {
			break;
		}
	}
}

void tune_battery_sweep(void)
{
	int i=0;
	int tmp = 0;
	int D_VALUE[]={20,20,20,10,5};
	int C_VALUE[]={0,9,4,20,40,80};

	if (log_enable)
		printk(KERN_ERR "Enter %s \n", __func__);

	if (mc13892_psy_info->is_cable_in) {
		tmp = mc13892_psy_info->is_cable_in();
		if (mc13892_psy_info->charger_online != tmp) {
			mc13892_psy_info->charger_online = tmp;
		}
	}


	if (mc13892_psy_info->is_charge_ok)
		mc13892_psy_info->charge_ok = mc13892_psy_info->is_charge_ok();

	if (mc13892_psy_info->charger_online && !mc13892_psy_info->charge_ok)
		mc13892_psy_info->bat_cur_status = POWER_SUPPLY_STATUS_CHARGING;
	else if (mc13892_psy_info->charger_online && mc13892_psy_info->charge_ok)
		mc13892_psy_info->bat_cur_status = POWER_SUPPLY_STATUS_FULL;
	else
		mc13892_psy_info->bat_cur_status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (mc13892_psy_info->bat_cur_status != mc13892_psy_info->bat_old_status) {
		printk(KERN_DEBUG "%s, charger status changed from: %d to %d \n", __func__,
								mc13892_psy_info->bat_old_status,
								mc13892_psy_info->bat_cur_status);
		mc13892_psy_info->bat_old_status = mc13892_psy_info->bat_cur_status;
		power_supply_changed(&mc13892_psy_info->bat);
	}

	if (mc13892_psy_info->bat_old_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		if (mc13892_psy_info->get_bat_voltage)
			mc13892_psy_info->bat_cur_vol = ((mc13892_psy_info->get_bat_voltage() + 150000)/10000);

		for (i=0; i<ARRAY_SIZE(discharg_percent)+1; i++) {

			avoid_abnormal_voltage(i, D_VALUE);
			if (discharg_percent[i].voltage > mc13892_psy_info->bat_cur_vol) {

			} else if ((i > 0)&&(i<ARRAY_SIZE(discharg_percent))) {
				mc13892_psy_info->bat_cur_capacity = ((10*(discharg_percent[i-1].percent - discharg_percent[i].percent)/
									(discharg_percent[i-1].voltage - discharg_percent[i].voltage)*
									(mc13892_psy_info->bat_cur_vol - discharg_percent[i].voltage))/10)+
									(discharg_percent[i].percent);

				break;
			} else {
				if (i==0)
					mc13892_psy_info->bat_cur_capacity = 100;
				else
					mc13892_psy_info->bat_cur_capacity = 0;
				break;
			}
		}

		if ((i != ARRAY_SIZE(discharg_percent)) && (i != 0)&& mc13892_psy_info->bat_cur_capacity%5)
			mc13892_psy_info->bat_cur_capacity = mc13892_psy_info->bat_cur_capacity-mc13892_psy_info->bat_cur_capacity%5 + 5;

		if (mc13892_psy_info->bat_cur_capacity <= 0)
			mc13892_psy_info->bat_cur_capacity = 0;
		if (mc13892_psy_info->bat_cur_capacity >= 100)
			mc13892_psy_info->bat_cur_capacity = 95;

	} else if (mc13892_psy_info->bat_old_status == POWER_SUPPLY_STATUS_CHARGING) {
		if (mc13892_psy_info->get_bat_voltage)
			mc13892_psy_info->bat_cur_vol = ((mc13892_psy_info->get_bat_voltage() + 150000)/10000);
			
		for (i=0; i<ARRAY_SIZE(charg_percent)+1; i++) {

			avoid_abnormal_voltage(i, C_VALUE);
			if (charg_percent[i].voltage > mc13892_psy_info->bat_cur_vol) {

			} else if ((i > 0)&&(i<ARRAY_SIZE(charg_percent))){
				mc13892_psy_info->bat_cur_capacity = ((10*(charg_percent[i-1].percent - charg_percent[i].percent)/
								(charg_percent[i-1].voltage - charg_percent[i].voltage)*
								(mc13892_psy_info->bat_cur_vol-charg_percent[i].voltage))/10)+
								(charg_percent[i].percent);

				break;
			} else {
				if (i==0)
					mc13892_psy_info->bat_cur_capacity = 95;
				else
					mc13892_psy_info->bat_cur_capacity = 5;
				break;
			}
		}
		mc13892_psy_info->bat_cur_capacity = mc13892_psy_info->bat_cur_capacity - mc13892_psy_info->bat_cur_capacity%5;

		if (mc13892_psy_info->bat_cur_capacity <= 0)
			mc13892_psy_info->bat_cur_capacity = 5;
		if (mc13892_psy_info->bat_cur_capacity >= 100)
			mc13892_psy_info->bat_cur_capacity = 95;
	}

	if (mc13892_psy_info->charger_online && mc13892_psy_info->charge_ok) {
		/* should be not charging status, will think about how to handle such case in future */
		mc13892_psy_info->bat_cur_capacity = 100; 
	}

}

static void update_battery_info(void)
{
	if (log_enable)
		printk(KERN_ERR "Enter %s \n", __func__);

	tune_battery_sweep();

	if (mc13892_psy_info->bat_old_status == POWER_SUPPLY_STATUS_CHARGING) {
		if (mc13892_psy_info->bat_cur_capacity > mc13892_psy_info->bat_old_capacity) {
			printk(KERN_DEBUG "%s, battery capacity changed from: %d to %d (voltage %d->%d)\n", __func__,
									mc13892_psy_info->bat_old_capacity,
									mc13892_psy_info->bat_cur_capacity,
									mc13892_psy_info->bat_old_vol,
									mc13892_psy_info->bat_cur_vol);

			mc13892_psy_info->bat_old_capacity = mc13892_psy_info->bat_cur_capacity;
			power_supply_changed(&mc13892_psy_info->bat);
		}
	} else if (mc13892_psy_info->bat_old_status == POWER_SUPPLY_STATUS_DISCHARGING) {
		if ((mc13892_psy_info->bat_cur_capacity < mc13892_psy_info->bat_old_capacity)||
				(mc13892_psy_info->bat_old_capacity == -1)) {

			printk(KERN_DEBUG "%s, battery capacity changed from: %d to %d (voltage %d->%d)\n", __func__,
								mc13892_psy_info->bat_old_capacity,
								mc13892_psy_info->bat_cur_capacity,
								mc13892_psy_info->bat_old_vol,
								mc13892_psy_info->bat_cur_vol);

			mc13892_psy_info->bat_old_capacity = mc13892_psy_info->bat_cur_capacity;
			power_supply_changed(&mc13892_psy_info->bat);
		}
	} else if (mc13892_psy_info->bat_old_status == POWER_SUPPLY_STATUS_FULL) {

		printk(KERN_DEBUG "battery capacity is full");
		mc13892_psy_info->bat_old_capacity = mc13892_psy_info->bat_cur_capacity;
		power_supply_changed(&mc13892_psy_info->bat);

	} else {
		printk(KERN_ERR "%s unknow status: %d \n", __func__, mc13892_psy_info->bat_old_status);
	}

	if (log_enable) {
		printk(KERN_ERR "Leave %s old status:%d, new status:%d, old capcity:%d, new capacity:%d, old bat vol:%d, new bat vol: %d\n",
						__func__, mc13892_psy_info->bat_cur_status, mc13892_psy_info->bat_old_status,
						mc13892_psy_info->bat_cur_capacity, mc13892_psy_info->bat_old_capacity,
						mc13892_psy_info->bat_cur_vol, mc13892_psy_info->bat_old_vol);
	}
}

static void mc13892_monitor_work(struct work_struct *work)
{
	if (!mc13892_psy_info)
		return;

	update_battery_info();

	if (mc13892_psy_info->bat_old_capacity > 10)
		queue_delayed_work(mc13892_psy_info->monitor_wqueue,  &mc13892_psy_info->monitor_work, 10*HZ);
	else
		queue_delayed_work(mc13892_psy_info->monitor_wqueue,  &mc13892_psy_info->monitor_work, 5*HZ);

	return;
}

static ssize_t log_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "log is %s", log_enable?"enabled":"disabled");
}

static ssize_t log_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL)
		log_enable = 1;
	else if (strstr(buf, "0") != NULL)
		log_enable = 0;

	return size;
}
static DEVICE_ATTR(enable, 0644, log_enable_show, log_enable_store);

static void mc13892_external_power_changed(struct power_supply *psy)
{
	printk("%s\n", __func__);
	cancel_delayed_work_sync(&mc13892_psy_info->monitor_work);
	queue_delayed_work(mc13892_psy_info->monitor_wqueue, &mc13892_psy_info->monitor_work, 1*HZ);
}

static int mc13892_power_supply_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mc13892_power_supply_pdata *pdata;

	printk(KERN_INFO "Enter %s\n", __func__);

	if (pdev->dev.platform_data)
		pdata = pdev->dev.platform_data;
	else {
		printk(KERN_ERR "%s, error: no platform data\n", __func__);
		return -1;
	}

	mc13892_psy_info = (struct mc13892_power_supply_info *)kzalloc(
							sizeof(struct mc13892_power_supply_info), GFP_KERNEL);

	if (!mc13892_psy_info) {
		printk(KERN_ERR "%s, request memory failed for mc13892_psy_info\n", __func__);
		return -ENOMEM;
	}

	mc13892_psy_info->dev = &pdev->dev;

	mc13892_psy_info->bat.name = "battery";
	mc13892_psy_info->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	mc13892_psy_info->bat.properties = aster_battery_props;
	mc13892_psy_info->bat.num_properties = ARRAY_SIZE(aster_battery_props);
	mc13892_psy_info->bat.get_property = aster_battery_get_property;
	mc13892_psy_info->bat.external_power_changed = mc13892_external_power_changed;
	mc13892_psy_info->bat.use_for_apm = 1;

	mc13892_psy_info->bat_cur_status = POWER_SUPPLY_STATUS_UNKNOWN;

	if (pdata->is_charge_ok)
		mc13892_psy_info->is_charge_ok  = pdata->is_charge_ok;
	if (pdata->is_cable_in)
		mc13892_psy_info->is_cable_in  = pdata->is_cable_in;

	/* Set battery get capacity voltage and current function */
	if (pdata->get_bat_capacity)
		mc13892_psy_info->get_bat_capacity = pdata->get_bat_capacity;
	if (pdata->get_bat_voltage)
		mc13892_psy_info->get_bat_voltage = pdata->get_bat_voltage;
	if (pdata->get_bat_current)
		mc13892_psy_info->get_bat_current = pdata->get_bat_current;

	INIT_DELAYED_WORK(&mc13892_psy_info->monitor_work, mc13892_monitor_work);

	mc13892_psy_info->monitor_wqueue = create_singlethread_workqueue("mc13892_power_supply_mon");

	if (!mc13892_psy_info->monitor_wqueue) {
		printk(KERN_ERR "%s, create power supply monitor work queue failed\n", __func__);
		goto create_mon_workqueue_failed;
	}

	ret = power_supply_register(&pdev->dev, &mc13892_psy_info->bat);
	if (ret) {
		printk(KERN_ERR "failed to register battery\n");
		goto batt_failed;
	}

	mc13892_psy_info->bat_old_capacity = -1;
	mc13892_psy_info->bat_cur_capacity = -1;

	ret = sysfs_create_file(&mc13892_psy_info->bat.dev->kobj, &dev_attr_enable.attr);
	if (ret) {
		printk(KERN_ERR
		       "DVFS: Unable to register sysdev entry for DVFS");
		goto creat_sys_failed;
	}

	queue_delayed_work(mc13892_psy_info->monitor_wqueue, &mc13892_psy_info->monitor_work, msecs_to_jiffies(100));

	device_init_wakeup(mc13892_psy_info->dev, 1);

	return 0;
creat_sys_failed:
	power_supply_unregister(&mc13892_psy_info->bat);
batt_failed:
	destroy_workqueue(mc13892_psy_info->monitor_wqueue);
create_mon_workqueue_failed:
	kfree(mc13892_psy_info);

	return ret;
}

static int mc13892_power_supply_remove(struct platform_device *pdev)
{
	device_init_wakeup(mc13892_psy_info->dev, 0);
	cancel_rearming_delayed_workqueue(mc13892_psy_info->monitor_wqueue,
					  &mc13892_psy_info->monitor_work);
	destroy_workqueue(mc13892_psy_info->monitor_wqueue);
	power_supply_unregister(&mc13892_psy_info->bat);
	kfree(mc13892_psy_info);

	return 0;
}

static int mc13892_power_supply_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mc13892_power_supply_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mc13892_power_supply_driver = {
	.probe   = mc13892_power_supply_probe,
	.remove  = mc13892_power_supply_remove,
#ifdef CONFIG_PM
	.suspend = mc13892_power_supply_suspend,
	.resume  = mc13892_power_supply_resume,
#endif
	.driver = {
		.name = "mc13892_power_supply",
	},
};

static int __init mc13892_power_supply_init(void)
{
	if (!is_pmic_adc_ready())
		return -ENODEV;

	return platform_driver_register(&mc13892_power_supply_driver);
}

static void __exit mc13892_power_supply_exit(void)
{
	platform_driver_unregister(&mc13892_power_supply_driver);
}

late_initcall(mc13892_power_supply_init);
module_exit(mc13892_power_supply_exit);

MODULE_AUTHOR("Letou Tech Co., Ltd.");
MODULE_DESCRIPTION("Letou Aster battery charger driver");
