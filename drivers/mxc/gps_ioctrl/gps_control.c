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
static int gps_pwr_en_value;
static int gps_nrst_value;

static ssize_t gps_pwr_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", gps_pwr_en_value);
}

static ssize_t gps_pwr_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if(!strncmp(buf, "1", 1)) {
		gps_pwr_en_value = 1;
		if(gps_pdata->set_power)
			gps_pdata->set_power(1);
	} else if (!strncmp(buf, "0", 1)) {
		gps_pwr_en_value = 0;
		if(gps_pdata->set_power)
			gps_pdata->set_power(0);
	}
	return size;
}
static DEVICE_ATTR(gps_pwr_en, 0644, gps_pwr_en_show, gps_pwr_en_store);

static ssize_t gps_nrst_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", gps_nrst_value);
}

static ssize_t gps_nrst_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if(!strncmp(buf, "1", 1)) {
		gps_nrst_value = 1;
		if(gps_pdata->set_nrst)
			gps_pdata->set_nrst(1);
	} else if (!strncmp(buf, "0", 1)) {
		gps_nrst_value = 0;
		if(gps_pdata->set_nrst)
			gps_pdata->set_nrst(0);
	}
	return size;
}
static DEVICE_ATTR(gps_nrst, 0644, gps_nrst_show, gps_nrst_store);

static int gps_control_probe(struct platform_device *pdev)
{
	int ret;

	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s, no platform data get\n", __func__);
		return -ENODATA;
	}

	gps_pdata = pdev->dev.platform_data;

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_gps_pwr_en.attr);
	if (ret) {
		printk(KERN_ERR "%s, register sysfs interface failed, ret: %d \n", __func__, ret);
		return ret;
	}
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_gps_nrst.attr);
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
