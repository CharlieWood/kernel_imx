/*
 * bq24103 charger driver
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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/bq24103_charger.h>


struct bq24103_device_info {
	struct power_supply	ac;
	struct bq24103_platform_data *pdata;
};

static struct bq24103_device_info di;

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq24103_device_info *di = container_of(psy, struct bq24103_device_info, ac);
	struct bq24103_platform_data *pdata = di->pdata;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (pdata && pdata->is_online)
				val->intval = pdata->is_online();
			else
				val->intval = 0;
			break;

		default:
			return -EINVAL;
	}
	return 0;
}

static char *supply_list[] = {
	"battery",
};

static void power_supply_init(struct power_supply *psy)
{
	psy->name = "charger";
	psy->type = POWER_SUPPLY_TYPE_MAINS;
	psy->properties = ac_props;
	psy->num_properties = ARRAY_SIZE(ac_props);
	psy->get_property = get_property;
	psy->supplied_to = supply_list;
 	psy->num_supplicants = ARRAY_SIZE(supply_list);
	psy->external_power_changed = NULL;
}

static irqreturn_t charger_irq_handler(int irq, void *dev_id)
{
	struct bq24103_device_info *di = (struct bq24103_device_info *)dev_id;

	printk("%s \n", __func__);
	disable_irq_nosync(irq);
	power_supply_changed(&di->ac);
	enable_irq(irq);

	return IRQ_HANDLED;
}

static int __devinit bq24103_probe(struct platform_device *pdev)
{
	struct bq24103_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	int wakeup = 1;

	di.pdata = pdata;

	printk("%s \n", __func__);

	ret = request_irq(pdata->irq, charger_irq_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "AP_ACOK", &di);
	if (ret < 0) {
		pr_err("%s: fail to claim irq %d, error number: %d\n",__func__, pdata->irq, ret);
		return ret;
	}

	power_supply_init(&di.ac);

	ret = power_supply_register(&pdev->dev, &di.ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac power_supply\n");
		return ret;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;
}

static int bq24103_remove(struct platform_device *pdev)
{
	struct bq24103_platform_data *pdata = pdev->dev.platform_data;

	printk("%s \n", __func__);
	free_irq(pdata->irq, pdata);
	power_supply_unregister(&di.ac);
	return 0;
}

static int bq24103_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct bq24103_platform_data *pdata = pdev->dev.platform_data;

	printk("%s \n", __func__);
	enable_irq_wake(pdata->irq);
	return 0;
}

static int bq24103_resume(struct platform_device *pdev)
{
	struct bq24103_platform_data *pdata = pdev->dev.platform_data;

	printk("%s \n", __func__);
	disable_irq_wake(pdata->irq);
	return 0;
}

static struct platform_driver bq24103_charger_driver = {
	.driver		= {
		.name	= "bq24103-charger",
		.owner	= THIS_MODULE,
	},
	.probe		= bq24103_probe,
	.remove		= bq24103_remove,
	.suspend	= bq24103_suspend,
	.resume		= bq24103_resume,
};

static int __init charger_init(void)
{
	int ret;

	pr_info("%s: register bq24103 charger driver\n", __func__);
	ret = platform_driver_register(&bq24103_charger_driver);
	if (ret)
		pr_err("%s: unable to register bq24103 charger driver\n", __func__);

	return ret;
}

static void __exit charger_exit(void)
{
	platform_driver_unregister(&bq24103_charger_driver);
}

module_init(charger_init);
module_exit(charger_exit);

MODULE_LICENSE("GPL");
