/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name			: hmc5883.c
* Original File Name	: lsm303dlh_mag_char.c
* Original Authors	: MH - C&I BU - Application Team
*				: Carmine Iascone (carmine.iascone@st.com)
*				: Matteo Dameno (matteo.dameno@st.com)
* Original Version	: V 0.3
* Original File Date	: 24/02/2010
*
********************************************************************************
*
* Original STMicroelectronics Notice:
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/******************** PORTIONS (C) COPYRIGHT 2010 Honeywell ********************
*
* Modified Author		: Ron Fang
* Modified Version	: V 0.1
* Modified Date		: 05/10/2010
*
* Honeywell Notice:
* This program contains open source software that is covered by the GNU General
* Public License version 2 as published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, HONEYWELL SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
* OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH HONEYWELL PARTS.
*
********************************************************************************
*
* Modified Items by Honeywell:
* 1. Replacing all references to "lsm303dlh_mag" by "hmc5883" including:
*    filename, variable names, function names, case names, comments, and texts.
* 2. Modified magnetometer "sensitivity" values to match HMC5883 datasheet.
*    Renamed constant names with "SENSITIVITY" to "GAIN" and combined XY and Z.
*    Added definition for "gain" 0.
* 3. Modified magnetometer "initialization" value for output rate to match
*    HMC5883 datasheet (0x20).
* 4. Swapped Y and Z sensor output data in function "hmc5883_read_mag_xyz()".
*
*******************************************************************************/

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/i2c/hmc5883.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>


#define HMC5883_MAJOR	101
#define HMC5883_MINOR	4

/* Magnetometer registers */
#define CRA_REG_M	0x00  /* Configuration register A */
#define CRB_REG_M	0x01  /* Configuration register B */
#define MR_REG_M	0x02  /* Mode register */

/* Output register start address*/
#define OUT_X_M		0x03

/* hmc5883 magnetometer identification registers */
#define IRA_REG_M	0x0A

/* Magnetometer XYZ sensitivity  */
#define GAIN_0	1280	/* XYZ sensitivity at 0.9G */
#define GAIN_1	1024	/* XYZ sensitivity at 1.2G */
#define GAIN_2	768	/* XYZ sensitivity at 1.9G */
#define GAIN_3	614	/* XYZ sensitivity at 2.5G */
#define GAIN_4	415	/* XYZ sensitivity at 4.0G */
#define GAIN_5	361	/* XYZ sensitivity at 4.6G */
#define GAIN_6	307	/* XYZ sensitivity at 5.5G */
#define GAIN_7	219	/* XYZ sensitivity at 7.9G */

#define ST_A_SET	0x11
#define ST_B_SET	0x60
#define ST_M_SET	0x01
#define ST_A_REC	0x00

#define DEBUG 1

//#define HMC5883_ENABLE_IRQ

#ifdef HMC5883_ENABLE_IRQ
static int interrupt_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(hmc5883_waitq);
#endif
/*
 * HMC5883 magnetometer data
 * brief Structure containing magnetic field values for x,y and z-axis in
 * signed short
*/
static int hal_flags = 0;

struct hmc5883_t {
	short	x, /**< x-axis magnetic field data. Range -7900 to 7900. */
		y, /**< y-axis magnetic field data. Range -7900 to 7900. */
		z; /**< z-axis magnetic filed data. Range -7900 to 7900. */
};

struct hmc5883_data{
	struct i2c_client *client;
	struct hmc5883_platform_data *pdata;
	short xy_sensitivity;
	short z_sensitivity;
};

static struct hmc5883_data *mag;

static struct class *hmc_mag_dev_class;

static char hmc5883_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char hmc5883_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

static int device_init(void);

#ifdef HMC5883_ENABLE_IRQ
static irqreturn_t hmc5883_irq_handler(int irq, void *_id)
{
	struct hmc5883_data *data;
    unsigned long flags;

	if(mag->client == NULL)
		return IRQ_HANDLED;

    data = i2c_get_clientdata(mag->client);

    if(data == NULL)
		return IRQ_HANDLED;

 	local_irq_save(flags);
	interrupt_flag = 1;
	wake_up_interruptible(&hmc5883_waitq); 
	local_irq_restore(flags);

	return IRQ_HANDLED;
}
#endif

/* set hmc5883 magnetometer bandwidth */
int hmc5883_set_bandwidth(char bw)
{
	int res = 0;
	unsigned char data;

	data = bw;
	res = hmc5883_i2c_write(CRA_REG_M, &data, 1);
	return res;
}

/* read selected bandwidth from hmc5883 */
int hmc5883_get_bandwidth(unsigned char *bw)
{
	int res = 1;
	/* TO DO */
	return res;

}


/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref hmc5883_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */
int hmc5883_read_mag_xyz(struct hmc5883_t *data)
{
	int res;
	unsigned char mag_data[6];
	int hw_d[3] = { 0 };
	int temp = 0;

	res = hmc5883_i2c_read(OUT_X_M, &mag_data[0], 6);

	hw_d[0] = (short) (((mag_data[0]) << 8) | mag_data[1]);
	hw_d[1] = (short) (((mag_data[2]) << 8) | mag_data[3]);
	hw_d[2] = (short) (((mag_data[4]) << 8) | mag_data[5]);

	temp = hw_d[1];		/* swap Y and Z */
	hw_d[1] = hw_d[2];
	hw_d[2] = temp;

	hw_d[0] = hw_d[0] * 1000 / mag->xy_sensitivity;
	hw_d[1] = hw_d[1] * 1000 / mag->xy_sensitivity;
	hw_d[2] = hw_d[2] * 1000 / mag->z_sensitivity;

	/* printk(KERN_INFO "Hx=%d, Hy=%d, Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]); */

	data->x = ((mag->pdata->negate_x) ? (-hw_d[mag->pdata->axis_map_x])
		   : (hw_d[mag->pdata->axis_map_x]));
	data->y = ((mag->pdata->negate_y) ? (-hw_d[mag->pdata->axis_map_y])
		   : (hw_d[mag->pdata->axis_map_y]));
	data->z = ((mag->pdata->negate_z) ? (-hw_d[mag->pdata->axis_map_z])
		   : (hw_d[mag->pdata->axis_map_z]));

	return res;
}

#if 1
ssize_t hmc5883_self_test(struct hmc5883_t *hdata)
{
	int res;
	ssize_t auto_test_value;
	unsigned char data;

	/* measurement mode setting */
	data = ST_A_SET;
	res = hmc5883_i2c_write(CRA_REG_M, &data, 1);
	if (res != 0) {
		goto set_error;
	}
	mdelay(20);

	/* GAIN setting */
	data = ST_B_SET;
	res = hmc5883_i2c_write(CRB_REG_M, &data, 1);
	if (res != 0) {
		goto set_error;
	}
	mdelay(20);

	/* operating mode setting */
	data = ST_M_SET;
	res = hmc5883_i2c_write(MR_REG_M, &data, 1);
	if (res != 0) {
		goto set_error;
	}
	mdelay(20);

	/* get difference from output register */
	res = hmc5883_read_mag_xyz(hdata);
	if (res < 0) {
		goto set_error;
	}
	if ((hdata->x > 0 && hdata->x < 4000) &&
			(hdata->y > 0 && hdata->y < 4000) &&
			(hdata->z > 0 && hdata->z < 4000)) {
		auto_test_value = 1;
	} else {
		auto_test_value = 0;
	}

	/* close self test */
	data = ST_A_REC;
	res = hmc5883_i2c_write(MR_REG_M, &data, 1);
	if (res != 0) {
		goto set_error;
	}
	mdelay(20);

	/* re-init chip */
	device_init();

	return auto_test_value;

set_error:
#if DEBUG
	printk(KERN_ERR "i2c write error!\n");
#endif
	auto_test_value = 0;
	return auto_test_value;
}
#endif

/* Device Initialization  */
static int device_init(void)
{
	int res;
	unsigned char buf[3];
	buf[0] = 0x10;	/* 15Hz ODR */
	buf[1] = 0x20;	/* FS= 1.2 */
	buf[2] = 0x00;	/* Continuous-Measurement Mode */
	res = hmc5883_i2c_write(CRA_REG_M, &buf[0], 3);
	mag->xy_sensitivity = GAIN_1;
	mag->z_sensitivity = GAIN_1;
	return res;
}

int hmc5883_set_range(char range)
{
	int err = 0;
	unsigned char data;

	switch (range) {
	case HMC5883_0_9G:
		mag->xy_sensitivity = GAIN_0;
		mag->z_sensitivity = GAIN_0;
		break;
	case HMC5883_1_2G:
		mag->xy_sensitivity = GAIN_1;
		mag->z_sensitivity = GAIN_1;
		break;
	case HMC5883_1_9G:
		mag->xy_sensitivity = GAIN_2;
		mag->z_sensitivity = GAIN_2;
		break;
	case HMC5883_2_5G:
		mag->xy_sensitivity = GAIN_3;
		mag->z_sensitivity = GAIN_3;
		break;
	case HMC5883_4_0G:
		mag->xy_sensitivity = GAIN_4;
		mag->z_sensitivity = GAIN_4;
		break;
	case HMC5883_4_6G:
		mag->xy_sensitivity = GAIN_5;
		mag->z_sensitivity = GAIN_5;
		break;
	case HMC5883_5_5G:
		mag->xy_sensitivity = GAIN_6;
		mag->z_sensitivity = GAIN_6;
		break;
	case HMC5883_7_9G:
		mag->xy_sensitivity = GAIN_7;
		mag->z_sensitivity = GAIN_7;
		break;
	default:
		return -EINVAL;
	}


	data = range;
	err = hmc5883_i2c_write(CRB_REG_M, &data, 1);
	return err;

}

int hmc5883_set_mode(char mode)
{
	int res = 0;
	unsigned char data;
	data = mode;
	res = hmc5883_i2c_write(MR_REG_M, &data, 1);
	return res;
}


/*  i2c write routine for hmc5883 magnetometer */
static char hmc5883_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int dummy;
	int i;

	if (mag->client == NULL)  /*  No global client pointer? */
		return -1;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(mag->client,
						  reg_addr++, data[i]);
		if (dummy) {
			printk(KERN_INFO "i2c write error\n");
			return dummy;
		}
	}
	return 0;
}

/*  i2c read routine for hmc5883 magnetometer */
static char hmc5883_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	int dummy = 0;
	int i = 0;

	if (mag->client == NULL)  /*  No global client pointer? */
		return -1;
	while (i < len) {
		dummy = i2c_smbus_read_byte_data(mag->client,
						 reg_addr++);

		if (dummy >= 0) {
			data[i] = dummy;
			i++;
		} else {
			printk(KERN_INFO" i2c read error\n ");
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}

/*  read command for HMC5883 device file  */
static ssize_t hmc5883_read(struct file *file, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct hmc5883_t data;

	if (mag->client == NULL)
		return -1;

#ifdef	HMC5883_ENABLE_IRQ
	wait_event_interruptible(hmc5883_waitq,interrupt_flag);
#endif

	hmc5883_read_mag_xyz(&data);
	/* printk(KERN_INFO "X axis: %d\n" , data.x); */
	/* printk(KERN_INFO "Y axis: %d\n" , data.y); */
	/* printk(KERN_INFO "Z axis: %d\n" , data.z); */
	if (copy_to_user(buf,&data, sizeof(data)) != 0) {
		printk(KERN_ERR "copy_to error\n");
		return -EFAULT;
	}

#ifdef	HMC5883_ENABLE_IRQ
	interrupt_flag = 0;
#endif

	return sizeof(data);

}

/*  write command for HMC5883 device file */
static ssize_t hmc5883_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *offset)
{
	if (mag->client == NULL)
		return -1;
	#if DEBUG
	printk(KERN_INFO "HMC5883 should be accessed with ioctl command\n");
	#endif
	return 0;
}

/*  open command for HMC5883 device file  */
static int hmc5883_open(struct inode *inode, struct file *file)
{
	if (mag->client == NULL) {
		#if DEBUG
		printk(KERN_ERR "I2C driver not install\n");
		#endif
		return -1;
	}
	//	device_init();

	#if DEBUG
	//printk(KERN_INFO "HMC5883 has been opened\n");
	#endif
	return 0;
}

/*  release command for HMC5883 device file */
static int hmc5883_close(struct inode *inode, struct file *file)
{
	#if DEBUG
	//printk(KERN_INFO "HMC5883 has been closed\n");
	#endif
	return 0;
}


/*  ioctl command for HMC5883 device file */
static int hmc5883_ioctl(struct inode *inode, struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check hmc5883_client */
	if (mag->client == NULL) {
		#if DEBUG
		printk(KERN_ERR "I2C driver not install\n");
		#endif
		return -EFAULT;
	}

	/* cmd mapping */

	switch (cmd) {

	case HMC5883_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = hmc5883_set_range(*data);
		return err;

	case HMC5883_SET_BANDWIDTH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = hmc5883_set_bandwidth(*data);
		return err;

	case HMC5883_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = hmc5883_set_mode(*data);
		return err;

	case HMC5883_READ_MAG_XYZ:
		err = hmc5883_read_mag_xyz(
				(struct hmc5883_t *)data);

		if (copy_to_user((struct hmc5883_t *)arg,
				 (struct hmc5883_t *)data, 6) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case ORIENTATION_GET_STATUS:

		if (copy_to_user((int *)arg, (int *)&hal_flags, sizeof(hal_flags)) != 0) {
            #if DEBUG
			printk(KERN_ERR "copy_to_user error\n");
            #endif
			return -EFAULT;
		}

		return err;

	case ORIENTATION_SET_STATUS:
		if (copy_from_user(&hal_flags, (int *)arg, sizeof(hal_flags)) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}

		return err;

	default:
		return 0;
	}
}

static unsigned int hmc5883_poll(struct file *file, poll_table *wait)
{
    unsigned int mask=0;
#ifdef HMC5883_ENABLE_IRQ
	poll_wait(file,&hmc5883_waitq,wait);

	if(interrupt_flag == 1)
    mask |= POLLIN|POLLRDNORM|POLLOUT|POLLWRNORM;
#else
	if(hal_flags == 1)
		mask |= POLLIN|POLLRDNORM|POLLOUT|POLLWRNORM;
#endif

	//	printk(KERN_INFO "%s\n",__FUNCTION__);	
    return mask;
}

static const struct file_operations hmc5883_fops = {
	.owner = THIS_MODULE,
	.read = hmc5883_read,
	.write = hmc5883_write,
	.open = hmc5883_open,
	.release = hmc5883_close,
	.ioctl = hmc5883_ioctl,
	.poll = hmc5883_poll,
};

static int hmc5883_acc_validate_pdata(struct hmc5883_data *mag)
{
	if (mag->pdata->axis_map_x > 2 ||
	    mag->pdata->axis_map_y > 2 ||
	    mag->pdata->axis_map_z > 2) {
		dev_err(&mag->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			mag->pdata->axis_map_x, mag->pdata->axis_map_y,
			mag->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (mag->pdata->negate_x > 1 ||
	    mag->pdata->negate_y > 1 ||
	    mag->pdata->negate_z > 1) {
		dev_err(&mag->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			mag->pdata->negate_x, mag->pdata->negate_y,
			mag->pdata->negate_z);
		return -EINVAL;
	}

	return 0;
}

static ssize_t hmc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t res;
	unsigned char data[6];

	res = hmc5883_self_test((struct hmc5883_t *)data);
	if (res == 1) {
		return sprintf(buf, "%d\n", 1);
	} else {
		return sprintf(buf, "%d\n", 0);
	}
}

static ssize_t hmc_store(struct device *dev,
		struct device_attribute *attr, char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(AutoTestValue, 0644, hmc_show, hmc_store);

int hmc5883_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct hmc5883_data *data;
	struct device *dev;
	int err = 0;
	int tempvalue;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto exit;

	/*
	 * OK. For now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */

	data = kzalloc(sizeof(struct hmc5883_data), GFP_KERNEL);

	if (data == NULL) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->client = client;

	data->pdata = kmalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (data->pdata == NULL)
		goto exit_kfree;

	memcpy(data->pdata, client->dev.platform_data, sizeof(*data->pdata));

	err = hmc5883_acc_validate_pdata(data);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (i2c_smbus_read_byte(client) < 0) {
		printk(KERN_ERR "HMC5883 i2c_smbus_read_byte error!!\n");
		goto exit_kfree;
	} else {
		printk(KERN_INFO "HMC5883 Device detected!\n");
	}

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, IRA_REG_M);
	if ((tempvalue & 0x00FF) == 0x0048) {
		printk(KERN_INFO "HMC5883 I2C driver registered!\n");
	} else {
		mag->client = NULL;
		goto exit_kfree;
	}

	mag = data;

	/* register a char dev */
	err = register_chrdev(HMC5883_MAJOR,
			      "hmc5883",
			      &hmc5883_fops);

	if (err)
		goto error;
	/* create lis-dev device class */
	hmc_mag_dev_class = class_create(THIS_MODULE, "HMC_MAG-dev");
	if (IS_ERR(hmc_mag_dev_class)) {
		err = PTR_ERR(hmc_mag_dev_class);
		goto error_unreg_chrdev;
	}

	/* create device node for hmc5883 magnetometer */
	dev = device_create(hmc_mag_dev_class, NULL,
		MKDEV(HMC5883_MAJOR, 0),
		NULL,
		"hmc5883");
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto error_destroy;
	}
	printk(KERN_INFO "HMC5883 device created successfully\n");

	err = sysfs_create_file(&client->dev.kobj, &dev_attr_AutoTestValue.attr);
	if (err)
		goto error_destroy;

#ifdef	HMC5883_ENABLE_IRQ
    //data->IRQ = client->irq;
	err = request_irq(client->irq, hmc5883_irq_handler, IRQ_TYPE_EDGE_FALLING, "hmc5883", data);
	if (err)
	{
		printk(KERN_ERR "could not request irq\n");
		goto error;
	}
#endif
	device_init();

	return 0;

error:
	printk(KERN_ERR "%s: Driver Initialization failed\n", __FILE__);
error_destroy:
	class_destroy(hmc_mag_dev_class);
error_unreg_chrdev:
	unregister_chrdev(HMC5883_MAJOR, "hmc5883");
exit_kfree_pdata:
	kfree(data->pdata);
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int hmc5883_remove(struct i2c_client *client)
{
	struct hmc5883_data *dev = i2c_get_clientdata(client);
	#if DEBUG
		printk(KERN_INFO "HMC5883 driver removing\n");
	#endif
	device_destroy(hmc_mag_dev_class, MKDEV(HMC5883_MAJOR, 0));
	class_destroy(hmc_mag_dev_class);
	unregister_chrdev(HMC5883_MAJOR, "hmc5883");
#ifdef HMC5883_ENABLE_IRQ
	free_irq(client->irq, dev);
#endif
	kfree(dev);
	mag->client = NULL;
	return 0;
}
#ifdef CONFIG_PM
static int hmc5883_suspend(struct i2c_client *client, pm_message_t state)
{
	#if DEBUG
	printk(KERN_INFO "%s\n", __func__);
	#endif
	/* TO DO */

	return 0;
}

static int hmc5883_resume(struct i2c_client *client)
{
	#if DEBUG
	printk(KERN_INFO "%s\n", __func__);
	#endif
	/* TO DO */

	return 0;
}
#endif

static const struct i2c_device_id hmc5883_id[] = {
	{ "hmc5883", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, hmc5883_id);

static struct i2c_driver hmc5883_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = hmc5883_probe,
	.remove = __devexit_p(hmc5883_remove),
	.id_table = hmc5883_id,
	#ifdef CONFIG_PM
	.suspend = hmc5883_suspend,
	.resume = hmc5883_resume,
	#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "hmc5883",
	},
	/*
	.detect = hmc5883_detect,
	*/
};

static int __init hmc5883_init(void)
{
	/* add i2c driver for hmc5883 magnetometer */
	return i2c_add_driver(&hmc5883_driver);
}

static void __exit hmc5883_exit(void)
{
	#if DEBUG
	printk(KERN_INFO "HMC5883 exit\n");
	#endif
	i2c_del_driver(&hmc5883_driver);
	return;
}

module_init(hmc5883_init);
module_exit(hmc5883_exit);

MODULE_DESCRIPTION("hmc5883 magnetometer driver");
MODULE_AUTHOR("Honeywell");
MODULE_LICENSE("GPL");

