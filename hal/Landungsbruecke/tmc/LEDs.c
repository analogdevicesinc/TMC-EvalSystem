#include "../../HAL.h"
#include "../../LEDs.h"

static void init();
static void onStat();
static void onError();
static void offStat();
static void offError();
static void toggleStat();
static void toggleError();

LEDsTypeDef LEDs =
{
	.init  = init,
	.stat  =
	{
		.on      = onStat,
		.off     = offStat,
		.toggle  = toggleStat,
	},
	.error	=
	{
		.on      = onError,
		.off     = offError,
		.toggle  = toggleError,
	},
};

static void init()
{
	HAL.IOs->pins->LED_ERROR.configuration.GPIO_Mode   = GPIO_Mode_OUT;
	HAL.IOs->pins->LED_ERROR.configuration.GPIO_OType  = GPIO_OType_PP;
	HAL.IOs->pins->LED_STAT.configuration.GPIO_Mode    = GPIO_Mode_OUT;
	HAL.IOs->pins->LED_STAT.configuration.GPIO_OType   = GPIO_OType_PP;

	HAL.IOs->config->set(&HAL.IOs->pins->LED_ERROR);
	HAL.IOs->config->set(&HAL.IOs->pins->LED_STAT);

	LED_OFF();
	LED_ERROR_OFF();
}

static void onStat()
{
	LED_ON();
}

static void onError()
{
	LED_ERROR_ON();
}

static void offStat()
{
	LED_OFF();
}

static void offError()
{
	LED_ERROR_OFF();
}

static void toggleStat()
{
	LED_TOGGLE();
}

static void toggleError()
{
	LED_ERROR_TOGGLE();
}
