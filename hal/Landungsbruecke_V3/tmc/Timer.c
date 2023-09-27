/*******************************************************************************
* Copyright © 2023 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "hal/HAL.h"
#include "hal/Timer.h"

/*
HAL Timer channels

TIMER_CHANNEL_1 = TIMER 0 Channel 2, PWM output DIO11
TIMER_CHANNEL_2 = TIMER 3 Channel 0, RAMDebug
TIMER_CHANNEL_3 = TIMER 4 Channel 0
TIMER_CHANNEL_4 = TIMER 0 Channel 1, PWM output DIO9
*/

#define TIMER_BASE_CLK 240000000

static void init(void);
static void deInit(void);
static void setDuty(timer_channel channel, float duty);
static float getDuty(timer_channel channel);
static void setPeriod(timer_channel channel, uint16_t period);
static uint16_t getPeriod(timer_channel channel);
static void setPeriodMin(timer_channel channel, uint16_t period_min);
static void setFrequency(timer_channel channel, float freq);
static void setFrequencyMin(timer_channel channel, float freq_min);

static uint16_t period_min_buf[] = { 0, 0, 0 };
static float freq_min_buf[] = { 0.0f, 0.0f, 0.0f };

TimerTypeDef Timer =
{
	.initialized = false,
	.init     = init,
	.deInit   = deInit,
	.setDuty  = setDuty,
	.getDuty  = getDuty,
	.setPeriod = setPeriod,
	.getPeriod = getPeriod,
	.setPeriodMin = setPeriodMin,
	.setFrequency = setFrequency,
	.setFrequencyMin = setFrequencyMin,
	.overflow_callback = NULL
};

static void init(void)
{
	rcu_periph_clock_enable(RCU_TIMER0);
	rcu_periph_clock_enable(RCU_TIMER3);
	rcu_periph_clock_enable(RCU_TIMER4);
	timer_deinit(TIMER0);
	timer_deinit(TIMER3);
	timer_deinit(TIMER4);

	timer_parameter_struct params;
	timer_oc_parameter_struct oc_params;

	// TIMER_CHANNEL_1

	timer_struct_para_init(&params);
	params.prescaler = 0;
	params.alignedmode = TIMER_COUNTER_EDGE;
	params.counterdirection = TIMER_COUNTER_UP;
	params.period = TIMER_MAX;
	params.clockdivision = TIMER_CKDIV_DIV1;
	params.repetitioncounter = 0;
	timer_init(TIMER0, &params);

	timer_channel_output_struct_para_init(&oc_params);
	oc_params.ocpolarity = TIMER_OC_POLARITY_HIGH;
	oc_params.outputstate = TIMER_CCX_ENABLE;
	oc_params.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
	oc_params.outputnstate = TIMER_CCXN_DISABLE;
	oc_params.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
	oc_params.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
	timer_channel_output_config(TIMER0, TIMER_CH_2, &oc_params);
	// TIMER_CHANNEL_4
	timer_channel_output_config(TIMER0, TIMER_CH_1, &oc_params);


	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, TIMER_MAX >> 1);
	timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM1);
	timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

	// TIMER_CHANNEL_4
	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, TIMER_MAX >> 1);
	timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

	timer_primary_output_config(TIMER0, ENABLE);

	timer_auto_reload_shadow_enable(TIMER0);

	timer_enable(TIMER0);

	// TIMER_CHANNEL_2

	timer_struct_para_init(&params);
	params.prescaler = 0;
	params.alignedmode = TIMER_COUNTER_EDGE;
	params.counterdirection = TIMER_COUNTER_UP;
	params.period = TIMER_MAX;
	params.clockdivision = TIMER_CKDIV_DIV1;
	params.repetitioncounter = 0;
	timer_init(TIMER3, &params);

	timer_interrupt_enable(TIMER3, TIMER_INT_UP);
	nvic_irq_enable(TIMER3_IRQn, 0, 1);
	timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
	timer_auto_reload_shadow_enable(TIMER3);

	timer_enable(TIMER3);

	// TIMER_CHANNEL_3

	timer_struct_para_init(&params);
	params.prescaler = 0;
	params.alignedmode = TIMER_COUNTER_EDGE;
	params.counterdirection = TIMER_COUNTER_UP;
	params.period = TIMER_MAX;
	params.clockdivision = TIMER_CKDIV_DIV1;
	params.repetitioncounter = 0;
	timer_init(TIMER4, &params);

	timer_auto_reload_shadow_enable(TIMER4);

	timer_enable(TIMER4);

	Timer.initialized = true;
}

static void deInit(void)
{
	timer_deinit(TIMER0);
	timer_deinit(TIMER3);
	timer_deinit(TIMER4);
}

static void setDuty(timer_channel channel, float duty)
{
	duty = (duty < 0.0f) ? 0.0f : duty;
	duty = (duty > 1.0f) ? 1.0f : duty;

	switch(channel) {
	case TIMER_CHANNEL_1:
		timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, duty * TIMER_CAR(TIMER0));
		break;
	case TIMER_CHANNEL_2:
		timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, duty * TIMER_CAR(TIMER3));
		break;
	case TIMER_CHANNEL_3:
		timer_channel_output_pulse_value_config(TIMER4, TIMER_CH_0, duty * TIMER_CAR(TIMER4));
		break;
	case TIMER_CHANNEL_4:
		timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, duty * TIMER_CAR(TIMER0));
		break;
	}
}

static float getDuty(timer_channel channel)
{
	switch(channel) {
	case TIMER_CHANNEL_1:
		return (((float) timer_channel_capture_value_register_read(TIMER0, TIMER_CH_2)) / TIMER_CAR(TIMER0));
	case TIMER_CHANNEL_2:
		return (((float) timer_channel_capture_value_register_read(TIMER3, TIMER_CH_0)) / TIMER_CAR(TIMER3));
	case TIMER_CHANNEL_3:
		return (((float) timer_channel_capture_value_register_read(TIMER4, TIMER_CH_0)) / TIMER_CAR(TIMER4));
	case TIMER_CHANNEL_4:
		return (((float) timer_channel_capture_value_register_read(TIMER0, TIMER_CH_1)) / TIMER_CAR(TIMER0));
	}
}

static void setPeriod(timer_channel channel, uint16_t period)
{
	switch(channel) {
	case TIMER_CHANNEL_1:
	case TIMER_CHANNEL_4:
		timer_autoreload_value_config(TIMER0, period);
		break;
	case TIMER_CHANNEL_2:
		timer_autoreload_value_config(TIMER3, period);
		break;
	case TIMER_CHANNEL_3:
		timer_autoreload_value_config(TIMER4, period);
		break;
	}
}

static uint16_t getPeriod(timer_channel channel)
{
	switch(channel) {
	case TIMER_CHANNEL_1:
	case TIMER_CHANNEL_4:
		return TIMER_CAR(TIMER0);
	case TIMER_CHANNEL_2:
		return TIMER_CAR(TIMER3);
	case TIMER_CHANNEL_3:
		return TIMER_CAR(TIMER4);
	}
}

static void setPeriodMin(timer_channel channel, uint16_t period_min)
{
	switch(channel) {
	case TIMER_CHANNEL_1:
	case TIMER_CHANNEL_4:
		period_min_buf[0] = period_min;
		break;
	case TIMER_CHANNEL_2:
		period_min_buf[1] = period_min;
		break;
	case TIMER_CHANNEL_3:
		period_min_buf[2] = period_min;
		break;
	}
}

static void setFrequencyMin(timer_channel channel, float freq_min)
{
	switch(channel) {
	case TIMER_CHANNEL_1:
	case TIMER_CHANNEL_4:
		freq_min_buf[0] = freq_min;
		break;
	case TIMER_CHANNEL_2:
		freq_min_buf[1] = freq_min;
		break;
	case TIMER_CHANNEL_3:
		freq_min_buf[2] = freq_min;
		break;
	}
}

static void setFrequency(timer_channel channel, float freq)
{
	float freq_min;
	uint16_t period_min;
	uint32_t timer;
	uint32_t timer_ch;
	IRQn_Type irq;

	switch(channel) {
	case TIMER_CHANNEL_1:
		freq_min = freq_min_buf[0];
		period_min = period_min_buf[0];
		timer = TIMER0;
		timer_ch = TIMER_CH_2;
		irq = TIMER0_Channel_IRQn;
		break;
	case TIMER_CHANNEL_2:
		freq_min = freq_min_buf[1];
		period_min = period_min_buf[1];
		timer = TIMER3;
		timer_ch = TIMER_CH_0;
		irq = TIMER3_IRQn;
		break;
	case TIMER_CHANNEL_3:
		freq_min = freq_min_buf[2];
		period_min = period_min_buf[2];
		timer = TIMER4;
		timer_ch = TIMER_CH_0;
		irq = TIMER4_IRQn;
		break;
	case TIMER_CHANNEL_4:
		freq_min = freq_min_buf[0];
		period_min = period_min_buf[0];
		timer = TIMER0;
		timer_ch = TIMER_CH_1;
		irq = TIMER0_Channel_IRQn;
		break;
	}

	uint16_t prescaler = 0;
	uint16_t period = 0xFFFF;

	if(freq < ((float)TIMER_BASE_CLK / (0x0000FFFF * 0x0000FFFF)))
		return;

	if(freq > (float)TIMER_BASE_CLK)
		return;

	for(; prescaler < 0xFFFF; prescaler++)
	{
		if(freq > ((float)TIMER_BASE_CLK / ((prescaler + 1) * period)))
		{
			period = (float)TIMER_BASE_CLK / ((prescaler + 1) * freq);
			if(period < period_min)
			    period = (float)TIMER_BASE_CLK / (prescaler * freq);
			break;
		}
	}

	timer_prescaler_config(timer, prescaler, TIMER_PSC_RELOAD_NOW);
	timer_autoreload_value_config(timer, period);
}

void TIMER3_IRQHandler(void)
{
	if(timer_flag_get(TIMER3, TIMER_FLAG_UP) == SET)
	{
		if(Timer.overflow_callback)
			Timer.overflow_callback(TIMER_CHANNEL_2);
		timer_flag_clear(TIMER3, TIMER_FLAG_UP);
	}
}
