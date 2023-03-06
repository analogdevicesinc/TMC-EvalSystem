#ifndef TIMER_H_
#define TIMER_H_

#include "derivative.h"

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
#define TIMER_MAX 8000
#elif defined(LandungsbrueckeV3)
#define TIMER_MAX 65535
#endif

typedef enum {
	TIMER_CHANNEL_1,
	TIMER_CHANNEL_2,
	TIMER_CHANNEL_3
} timer_channel;

typedef struct
{
	bool initialized;
	void (*init) (void);
	void (*deInit) (void);
	void (*setDuty) (timer_channel channel, float duty);
	float (*getDuty) (timer_channel channel);
	void (*setPeriod) (timer_channel channel, uint16_t period);
	uint16_t (*getPeriod) (timer_channel channel);
	void (*setPeriodMin) (timer_channel channel, uint16_t period_min);
	void (*setFrequency) (timer_channel channel, float freq);
	void (*setFrequencyMin) (timer_channel channel, float freq_min);
	void (*overflow_callback) (timer_channel channel);
} TimerTypeDef;

extern TimerTypeDef Timer;

#endif /* TIMER_H_ */
