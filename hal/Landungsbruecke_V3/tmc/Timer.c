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
}

static void deInit(void)
{
}

static void setDuty(timer_channel channel, float duty)
{
}

static float getDuty(timer_channel channel)
{
	return 0.0f;
}
