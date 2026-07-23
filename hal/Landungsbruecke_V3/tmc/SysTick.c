/*******************************************************************************
* Copyright © 2023 Analog Devices, Inc.
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

    // Set up TIMER4 for microsecond timestamps
    rcu_periph_clock_enable(RCU_TIMER4);
    timer_deinit(TIMER4);

    // TIMER4 runs off of ABP1 clock and
    // the CK_TIMER clock is 2x the APB1 clock.
    uint32_t timerFrequency = rcu_clock_freq_get(CK_APB1)*2;

    timer_parameter_struct timer_config = { 0 };
    timer_struct_para_init(&timer_config);
    timer_config.prescaler = (timerFrequency / 1000000) - 1;
    timer_config.period = 0xFFFFFFFF;
    timer_config.counterdirection = TIMER_COUNTER_UP;
    timer_config.alignedmode = TIMER_COUNTER_EDGE;
    timer_config.clockdivision = TIMER_CKDIV_DIV1;
    timer_config.repetitioncounter = 0;
    timer_init(TIMER4, &timer_config);
    timer_enable(TIMER4);
}


void SysTick_Handler(void)
{
    systick++;
    if(I2CTimeout>0) I2CTimeout--;

}

uint32_t systick_getMicrosecondTick()
{
    return TIMER_CNT(TIMER4);
}

uint32_t systick_getTick(void)
{
    return systick;
}

void wait(uint32_t delay)  // wait for [delay] ms/systicks
{
    uint32_t startTick = systick;
    while((systick-startTick) <= delay) {}
}

uint32_t timeSince(uint32_t tick)  // time difference since the [tick] timestamp in ms/systicks
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
