#ifndef TIMER_H_
#define TIMER_H_

#include "derivative.h"

#if defined(Landungsbruecke)
#define TIMER_MAX 8000
#elif defined(Startrampe)
#define TIMER_MAX 10000 // Frequenz von 6kHz => 166,66us pro Periode => 8000 Schritte bei 48Mhz
#endif

typedef enum {
	TIMER_CHANNEL_1,
	TIMER_CHANNEL_2,
	TIMER_CHANNEL_3
} timer_channel;

typedef struct
{
	void (*init) (void);
	void (*deInit) (void);
	void (*setDuty) (timer_channel channel, uint16_t duty);
	uint16_t (*getDuty) (timer_channel channel);
} TimerTypeDef;

TimerTypeDef Timer;

#endif /* TIMER_H_ */
