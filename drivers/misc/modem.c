/* drivers/misc/modem.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/modem.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

//gpio 186

static int onoff_flag = 0;
static struct mutex onoff_mutex;

struct modem_info {
	struct device *dev; 
	struct delayed_work dwork;
} modem_info;

static struct wake_lock modem_wake_lock;

static ssize_t show_simcard(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int state = 0;
	struct modem_platform_data *pdata = dev->platform_data;
	if (pdata && pdata->is_sim_in)
		state = pdata->is_sim_in();
	return sprintf(buf, "%d\n", state);
}
static DEVICE_ATTR(sim, 0444, show_simcard, NULL);

static ssize_t store_reset(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct modem_platform_data *pdata = dev->platform_data;
	if (pdata && pdata->reset)
		pdata->reset();
	return size;
}
static DEVICE_ATTR(reset, 0644, NULL, store_reset);

static ssize_t show_onoff(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", onoff_flag);
}

static ssize_t store_onoff(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	int enable = 0;
	struct modem_platform_data *pdata = dev->platform_data;

	sscanf(buf, "%d", &enable);
	if (pdata && pdata->onoff && (onoff_flag != enable)) {
		mutex_lock(&onoff_mutex);
		pdata->onoff(enable);
		onoff_flag = enable;
		mutex_unlock(&onoff_mutex);
	}
	return size;
}
static DEVICE_ATTR(onoff, 0644, show_onoff, store_onoff);

static irqreturn_t modem_isr(int irq, void *dev_id)
{
	wake_lock_timeout(&modem_wake_lock, 30 * HZ);
	return IRQ_HANDLED;
}

static void simcard_worker(struct work_struct *work)
{
	struct modem_platform_data *pdata = modem_info.dev->platform_data;
	if (pdata->is_sim_in)
		kobject_uevent(&modem_info.dev->kobj, (pdata->is_sim_in() ? KOBJ_ADD : KOBJ_REMOVE));
}

#define DELAY_TIME	50
static irqreturn_t sim_cd(int irq, void *dev_id)
{
	wake_lock_timeout(&modem_wake_lock, 30 * HZ);
	schedule_delayed_work(&modem_info.dwork, msecs_to_jiffies(DELAY_TIME));
	return IRQ_HANDLED;
}

static int __devinit modem_probe(struct platform_device *pdev)
{
	int irq;
	int error;
	struct modem_platform_data *pdata = pdev->dev.platform_data;

	mutex_init(&onoff_mutex);

	wake_lock_init(&modem_wake_lock, WAKE_LOCK_SUSPEND, "modem_wakeup_ap");
	INIT_DELAYED_WORK(&modem_info.dwork, simcard_worker);
	modem_info.dev = &pdev->dev;
	irq = pdata->wakeup_irq;
	if (irq > 0) {
		error = request_irq(irq, modem_isr, IRQF_TRIGGER_RISING, "modem", NULL);
		if (error) {
			pr_err("%s: Unable to claim irq %d; error %d\n", __func__, irq, error);
			goto err;
		}
		enable_irq_wake(irq);
	}

	irq = pdata->sim_irq;
	if (irq > 0) {
		error = request_irq(irq, sim_cd, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "sim_cd", &modem_info);
		if (error) {
			pr_err("%s: Unable to claim irq %d; error %d\n", __func__, irq, error);
			return error;
		}
	}

	error = device_create_file(&pdev->dev, &dev_attr_sim);
	if (error) {
		printk(KERN_ERR "%s, register sim sysfs interface failed\n", __func__);
		goto err;
	}

	error = device_create_file(&pdev->dev, &dev_attr_reset);
	if (error) {
		printk(KERN_ERR "%s, register reset sysfs interface failed\n", __func__);
		goto err;
	}

	error = device_create_file(&pdev->dev, &dev_attr_onoff);
	if (error) {
		printk(KERN_ERR "%s, register onoff sysfs interface failed\n", __func__);
		goto err;
	}

	device_init_wakeup(&pdev->dev, 1);
	return 0;

err:
	free_irq(irq, NULL);
	return error;
}

static int prepare(struct device *dev)
{
	struct modem_platform_data *pdata = dev->platform_data;
	if (pdata && pdata->suspend)
		pdata->suspend();
	return 0;
}

static int resume(struct device *dev)
{
	struct modem_platform_data *pdata = dev->platform_data;
	if (pdata && pdata->resume)
		pdata->resume();
	schedule_delayed_work(&modem_info.dwork, msecs_to_jiffies(0));
	return 0;
}

static struct dev_pm_ops pm = {
	.prepare = prepare,
	.resume = resume,
};

static struct platform_driver modem_device_driver = {
	.probe		= modem_probe,
	.driver		= {
		.name	= "3Gmodem",
		.owner	= THIS_MODULE,
		.pm	= &pm,	
	}
};

static int __init modem_init(void)
{
	return platform_driver_register(&modem_device_driver);
}

static void __exit modem_exit(void)
{
	platform_driver_unregister(&modem_device_driver);
}

module_init(modem_init);
module_exit(modem_exit);

MODULE_LICENSE("GPL");

