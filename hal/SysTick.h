#ifndef SysTick_H
#define SysTick_H

	#include "tmc/helpers/API_Header.h"

	void systick_init();
	u32 systick_getTick();
	void wait(uint32 delay);
	uint32 timeSince(uint32 tick);

#endif /* SysTick_H */
