/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "gd32f4xx.h"
#include "hal/HAL.h"
#include "hal/SysTick.h"

volatile uint32_t systick = 0;
extern volatile uint32_t I2CTimeout;

#define SYSTICK_PRE_EMPTION_PRIORITY 3

void systick_init(void)
{
  SysTick_Config(SystemCoreClock/(1000));
  NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRE_EMPTION_PRIORITY);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	// Enable the DWT CYCCNT for the µs counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


void SysTick_Handler(void)
{
	systick++;
    if(I2CTimeout>0) I2CTimeout--;
}

uint32_t systick_getTick(void)
{
	return systick;
}

uint32_t systick_getMicrosecondTick()
{
    // 240 MHz CYCCNT / 240 -> µs counter
    return DWT->CYCCNT / 240;
}

void wait(uint32_t delay)	// wait for [delay] ms/systicks
{
	uint32_t startTick = systick;
	while((systick-startTick) <= delay) {}
}

uint32_t timeSince(uint32_t tick)	// time difference since the [tick] timestamp in ms/systicks
{
	return timeDiff(systick, tick);
}

uint32_t timeDiff(uint32_t newTick, uint32_t oldTick) // Time difference between newTick and oldTick timestamps
{
	uint32_t tickDiff = newTick - oldTick;

	// Prevent subtraction underflow - saturate to 0 instead
	if(tickDiff != 0)
		return tickDiff - 1;
	else
		return 0;
}
