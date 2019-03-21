#ifndef TIMER_H_
#define TIMER_H_

#include "derivative.h"

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
