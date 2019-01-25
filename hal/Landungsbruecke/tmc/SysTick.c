#include "../../HAL.h"
#include "hal/SysTick.h"

volatile u32 systick = 0;

void __attribute__ ((interrupt)) SysTick_Handler(void);

void SysTick_Handler(void)
{
	systick++;
}

void systick_init()
{
	SYST_RVR  = 48000;
	SYST_CSR  = 7;
}

u32 systick_getTick()
{
	return systick;
}

/* Systick values are in milliseconds, accessing the value is faster. As a result
 * we have a random invisible delay of less than a millisecond whenever we use
 * systicks. This can result in a situation where we access the systick just before it changes:
 *  -> Access at 0,99ms gives systick 0ms
 *  -> Access at 1.01ms gives systick 1ms
 *  -> systick difference of 1ms, even though only 0.02 ms passed
 * To prevent this, we generally apply a correction of -1 to any systick difference.
 * In wait() this is done by using '<=' instead of '<'
 * In timeSince() the subtraction is carried out on the result. That subtraction is prevented from underflowing
 * to UINT32_MAX, returning 0 in that case (Saturated subtraction).
 *
 */
void wait(uint32 delay)	// wait for [delay] ms/systicks
{
	uint32 startTick = systick;
	while((systick-startTick) <= delay) {}
}

uint32 timeSince(uint32 tick)	// time difference since the [tick] timestamp in ms/systicks
{
	uint32 tickDiff = systick - tick;

	// Prevent subtraction underflow - saturate to 0 instead
	if(tickDiff != 0)
		return tickDiff - 1;
	else
		return 0;
}
