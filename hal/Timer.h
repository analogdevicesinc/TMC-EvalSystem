#ifndef TIMER_H_
	#define TIMER_H_

	#include "derivative.h"

	typedef struct
	{
		void (*init) (void);
		void (*deInit) (void);
		void (*setDuty) (uint16 duty);
		uint16 (*getDuty) (void);
	} TimerTypeDef;

	TimerTypeDef Timer;

#endif /* TIMER_H_ */
