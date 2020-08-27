#ifndef SysTick_H
#define SysTick_H

	#include "tmc/helpers/API_Header.h"

	void systick_init();
	uint32_t systick_getTick();
	void wait(uint32_t delay);
	uint32_t timeSince(uint32_t tick);
	uint32_t timeDiff(uint32_t newTick, uint32_t oldTick);

#if defined(Landungsbruecke)
	uint32_t systick_getMicrosecondTick();
#endif

#endif /* SysTick_H */
