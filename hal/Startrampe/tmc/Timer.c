#include "hal/HAL.h"
#include "hal/Timer.h"

static void init(void);
static void deInit(void);
static void setDuty(timer_channel, float);
static float getDuty(timer_channel);

TimerTypeDef Timer =
{
	.initialized = false,
	.init     = init,
	.deInit   = deInit,
	.setDuty  = setDuty,
	.getDuty  = getDuty,
	.setModulo = NULL,
	.getModulo = NULL,
	.setModuloMin = NULL,
	.setFrequency = NULL,
	.setFrequencyMin = NULL,
	.overflow_callback = NULL
};

static void init(void)
{
	TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM1->CNT    = 0;

	TIM1->CR1    = 0;
	TIM1->CR1    |= TIM_CR1_ARPE;

	TIM1->DIER   = 1<<3; // no interrupt

	TIM1->CCMR2  = 0;
	TIM1->CCMR2  |= TIM_CCMR2_OC3PE;  // preload enable
	TIM1->CCMR2  |= TIM_OCMode_PWM1;  // ch3 PWM1

	TIM1->CCER   = 0;
	TIM1->CCER   |= TIM_CCER_CC3E;

	TIM1->ARR    = TIMER_MAX;       // period
	TIM1->CCR3   = TIMER_MAX >> 1;  // duty
	TIM1->PSC    = 0;

	TIM1->BDTR   = 0;
	TIM1->BDTR   |= TIM_BDTR_MOE;
	TIM1->BDTR   |= TIM_BDTR_AOE;

	TIM1->EGR    = 1;
	TIM1->CR1    |= TIM_CR1_CEN;

	Timer.initialized = true;
}

static void deInit(void)
{
	TIM_DeInit(TIM1);
}

static void setDuty(timer_channel channel, float duty)
{
	UNUSED(channel);

	duty = (duty < 0.0f) ? 0.0f : duty;
	duty = (duty > 1.0f) ? 1.0f : duty;

	TIM1->CCR3 = duty * TIMER_MAX;
}

static float getDuty(timer_channel channel)
{
	UNUSED(channel);
	return (((float)TIM1->CCR3) / TIMER_MAX);
}
