/**********************Solution Choose**********************/
//#define ATMEL_168
//#define R8C_3GA_2TG
//#define R8C_AUO_I2C
#define M48

#ifdef	M48
	#define	MAXX	48
	#define	MAXY	48
#else
	#define	MAXX	32
	#define	MAXY	32
#endif

#ifdef R8C_AUO_I2C
  #ifndef R8C_3GA_2TG
  #define R8C_3GA_2TG
  #endif
#endif

struct pixcir_touchkey {
	int touch_key;
	int bit;

	int x_pos;
	int y_pos;
};

struct pixcir_touch_pdata {
	unsigned int x_min;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int y_min;

	int gpio_int;
	int gpio_reset;

	struct pixcir_touchkey *pixcir_touchkey_map;
};

struct pixcir_callback_pdata {
	struct pixcir_touch_pdata* (*callback)(int);
};
