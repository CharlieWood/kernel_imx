/*
 *Copyright 2010 Letou. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/fs.h>		/* Async notification */
#include <linux/uaccess.h>	/* for get_user, put_user, access_ok */
#include <linux/sched.h>	/* jiffies */
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/gps/gps.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

static struct gps_control_data *gps_pdata;

static ssize_t gps_ctrl_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	int state = gps_pdata->get_status();
	return sprintf(buf, "gps status: %s ", state ? "enabled" : "disabled");
}

static ssize_t gps_ctrl_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t size)
{
	if(!strncmp(buf, "enable", 6)) {
		gps_pdata->set_power(1);
	} else if (!strncmp(buf, "disable", 7)) {
		gps_pdata->set_power(0);
	}
	return size;
}
static DEVICE_ATTR(gps_control, 0644, gps_ctrl_show, gps_ctrl_store);

static int gps_control_probe(struct platform_device *pdev)
{
	int ret;

	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s, no platform data get\n", __func__);
		return -ENODATA;
	}

	gps_pdata = pdev->dev.platform_data;

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_gps_control.attr);
	if (ret) {
		printk(KERN_ERR "%s, register sysfs interface failed, ret: %d \n", __func__, ret);
		return ret;
	}

	return 0;
}

static int gps_control_remove(struct platform_device *pdev)
{
	return 0;
}

static int gps_control_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int gps_control_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver gps_control_driver = {
	.probe = gps_control_probe,
	.remove = gps_control_remove,
#ifdef CONFIG_PM
	.suspend = gps_control_suspend,
	.resume = gps_control_resume,
#endif
	.driver = {
		.name = "gps-control",
	},
};

static int __init gps_control_init(void)
{
	int ret;

	ret = platform_driver_register(&gps_control_driver);

	return ret;
}

static void __exit gps_control_exit(void)
{
	platform_driver_unregister(&gps_control_driver);
}

module_init(gps_control_init);
module_exit(gps_control_exit);
