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
	timer_deinit(TIMER0);
	rcu_periph_clock_enable(RCU_TIMER0);

	timer_parameter_struct params;
	timer_struct_para_init(&params);

	params.period = TIMER_MAX;

	timer_init(TIMER0, &params);

	timer_auto_reload_shadow_enable(TIMER0);

	timer_oc_parameter_struct oc_params;
	timer_channel_output_struct_para_init(&oc_params);
	timer_channel_output_config(TIMER0, TIMER_CH_2, &oc_params);

	timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM1);
	timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, TIMER_MAX >> 1);
	timer_automatic_output_enable(TIMER0);
	timer_interrupt_enable(TIMER0, TIMER_INT_UP);
	timer_interrupt_enable(TIMER0, TIMER_INT_CH2);
	timer_update_event_enable(TIMER0);

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
