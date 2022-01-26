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
	bool initialized;
	void (*init) (void);
	void (*deInit) (void);
	void (*setDuty) (timer_channel channel, float duty);
	float (*getDuty) (timer_channel channel);
	void (*setModulo) (uint16_t modulo);
	uint16_t (*getModulo) (void);
	void (*setModuloMin) (uint16_t modulo_min);
	void (*setFrequency) (float freq);
	void (*setFrequencyMin) (float freq_min);
	void (*overflow_callback) (void);
} TimerTypeDef;

extern TimerTypeDef Timer;

#endif /* TIMER_H_ */
