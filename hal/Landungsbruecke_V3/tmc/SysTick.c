#include "gd32f4xx.h"


#define SYSTICK_PRE_EMPTION_PRIORITY 3

static volatile uint32_t SysTickTimer;

void systick_init(void)
{
  SysTick_Config(SystemCoreClock/1000);
  NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRE_EMPTION_PRIORITY);
}


void SysTick_Handler(void)
{
	SysTickTimer++;
}

uint32_t systick_getTick(void)
{
	return SysTickTimer;
}
