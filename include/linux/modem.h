#ifndef _MODEM_H
#define _MODEM_H
struct modem_platform_data {
	int wakeup_irq;
	int sim_irq;
	void (*suspend)(void);
	void (*resume)(void);
	int (*is_sim_in)(void);
	void (*reset)(void);
	void (*onoff)(int);
};

#endif /* _ZLIB_H */
