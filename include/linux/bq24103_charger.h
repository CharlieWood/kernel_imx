#ifndef _BQ24103_CHARGER_H
#define _BQ24103_CHARGER_H

struct bq24103_platform_data {
	int (*is_online)(void);
	int irq;
};
#endif
