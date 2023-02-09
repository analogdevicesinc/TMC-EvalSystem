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
	rcu_periph_clock_enable(RCU_TIMER0);
	timer_deinit(TIMER0);

	timer_parameter_struct params;
	timer_oc_parameter_struct oc_params;

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

	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, TIMER_MAX >> 1);
	timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM1);
	timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

	timer_primary_output_config(TIMER0, ENABLE);

	timer_auto_reload_shadow_enable(TIMER0);

	timer_enable(TIMER0);

	Timer.initialized = true;
}

static void deInit(void)
{
	timer_deinit(TIMER0);
}

static void setDuty(timer_channel channel, float duty)
{
	UNUSED(channel); // TODO

	duty = (duty < 0.0f) ? 0.0f : duty;
	duty = (duty > 1.0f) ? 1.0f : duty;

	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, duty * TIMER_MAX);
}

static float getDuty(timer_channel channel)
{
	UNUSED(channel); // TODO
	return (((float) timer_channel_capture_value_register_read(TIMER0, TIMER_CH_2)) / TIMER_MAX);
}
