#ifndef __LINUX_FT5X0X_TS_H__
#define __LINUX_FT5X0X_TS_H__

#define PRESS_MAX       255

#define FT5X0X_NAME	"ft5x0x_ts"//"synaptics_i2c_rmi"//"synaptics-rmi-ts"// 

struct ft5x0x_touchkey {
	int touch_key;
	int bit;

	int x_pos;
	int y_pos;
};

struct ft5x0x_ts_platform_data{
	unsigned int x_min;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int y_min;

	u16	gpio_irq;		/* irq number	*/
	u16	gpio_reset;		/* reset number	*/

	struct ft5x0x_touchkey *ft5x0x_touchkey_map;
};

enum ft5x0x_ts_regs {
	FT5X0X_REG_PMODE	= 0xA5,	/* Power Consume Mode		*/	
	FT5X0X_REG_THREV	= 0x80,	/* Set Threshold	*/	
	FT5X0X_REG_FIRMID						= 0xa6,
	FT5X0X_REG_ID		= 0xa3,
};

//FT5X0X_REG_PMODE
#define PMODE_ACTIVE        0x00
#define PMODE_MONITOR       0x01
#define PMODE_STANDBY       0x02
#define PMODE_HIBERNATE     0x03
#define THRESHOLD_VALUE     0x19


	#ifndef ABS_MT_TOUCH_MAJOR
	#define ABS_MT_TOUCH_MAJOR	0x30	/* touching ellipse */
	#define ABS_MT_TOUCH_MINOR	0x31	/* (omit if circular) */
	#define ABS_MT_WIDTH_MAJOR	0x32	/* approaching ellipse */
	#define ABS_MT_WIDTH_MINOR	0x33	/* (omit if circular) */
	#define ABS_MT_ORIENTATION	0x34	/* Ellipse orientation */
	#define ABS_MT_POSITION_X	0x35	/* Center X ellipse position */
	#define ABS_MT_POSITION_Y	0x36	/* Center Y ellipse position */
	#define ABS_MT_TOOL_TYPE	0x37	/* Type of touching device */
	#define ABS_MT_BLOB_ID		0x38	/* Group set of pkts as blob */
	#endif /* ABS_MT_TOUCH_MAJOR */


#endif

