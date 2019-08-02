#include "hal/IOs.h"


static void init();
static void setPinConfiguration(IOPinTypeDef *pin);
static void copyPinConfiguration(IOPinInitTypeDef *from, IOPinTypeDef*to);
static void resetPinConfiguration(IOPinTypeDef *pin);
static void setPin2Output(IOPinTypeDef *pin);
static void setPin2Input(IOPinTypeDef *pin);
static void setPinHigh(IOPinTypeDef *pin);
static void setPinLow(IOPinTypeDef *pin);
static void setPinState(IOPinTypeDef *pin, IO_States state);
static IO_States getPinState(IOPinTypeDef *pin);
static unsigned char isPinHigh(IOPinTypeDef *pin);

IOsTypeDef IOs =
{
	.init                  = init,
	.set                   = setPinConfiguration,
	.reset                 = resetPinConfiguration,
	.copy                  = copyPinConfiguration,
	.toOutput              = setPin2Output,
	.toInput               = setPin2Input,
	.setHigh               = setPinHigh,
	.setLow                = setPinLow,
	.setToState            = setPinState,
	.getState              = getPinState,
	.isHigh                = isPinHigh,
	.HIGH_LEVEL_FUNCTIONS  =
	{
		.DEFAULT  = IO_DEFAULT,
		.DI       = IO_DI,
		.AI       = IO_AI,
		.DO       = IO_DO,
		.PWM      = IO_PWM,
		.SD       = IO_SD,
		.CLK16    = IO_CLK16,
		.SPI      = IO_SPI
	}
};

static void init()
{
	RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1); // clock out @PA8: 16MHz (crystal)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}

static void setPinConfiguration(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin    = pin->bitWeight;
	GPIO_InitStructure.GPIO_Mode   = pin->configuration.GPIO_Mode;
	GPIO_InitStructure.GPIO_OType  = pin->configuration.GPIO_OType;
	GPIO_InitStructure.GPIO_Speed  = pin->configuration.GPIO_Speed;
	GPIO_InitStructure.GPIO_PuPd   = pin->configuration.GPIO_PuPd;
	GPIO_Init(pin->port, &GPIO_InitStructure);
}

static void setPin2Output(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	pin->configuration.GPIO_Mode = GPIO_Mode_OUT;
	setPinConfiguration(pin);
}

static void setPin2Input(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	pin->configuration.GPIO_Mode = GPIO_Mode_IN;
	setPinConfiguration(pin);
}

static void setPinState(IOPinTypeDef *pin, IO_States state)
{
	if(IS_DUMMY_PIN(pin))
		return;
	switch(state)
	{
	case IOS_LOW:
		pin->configuration.GPIO_Mode = GPIO_Mode_OUT;
		*pin->resetBitRegister = pin->bitWeight;
		break;
	case IOS_HIGH:
		pin->configuration.GPIO_Mode = GPIO_Mode_OUT;
		*pin->setBitRegister = pin->bitWeight;
		break;
	case IOS_OPEN:
		pin->configuration.GPIO_Mode = GPIO_Mode_IN;
		break;
	default:
		break;
	}

	setPinConfiguration(pin);
}

static IO_States getPinState(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return IOS_OPEN;

	if(pin->configuration.GPIO_Mode == GPIO_Mode_IN)
		pin->state = IOS_OPEN;
	else if(pin->port->IDR & pin->bitWeight)
		pin->state = IOS_HIGH;
	else
		pin->state = IOS_LOW;

	return pin->state;
}

static void setPinHigh(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	*pin->setBitRegister = pin->bitWeight;
}

static void setPinLow(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	*pin->resetBitRegister = pin->bitWeight;
}

static unsigned char isPinHigh(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return -1;

	return (pin->port->IDR & pin->bitWeight)? 1 : 0;
}

static void copyPinConfiguration(IOPinInitTypeDef *from, IOPinTypeDef *to)
{
	if(IS_DUMMY_PIN(to))
		return;

	to->configuration.GPIO_Mode   = from->GPIO_Mode;
	to->configuration.GPIO_OType  = from->GPIO_OType;
	to->configuration.GPIO_PuPd   = from->GPIO_PuPd;
	to->configuration.GPIO_Speed  = from->GPIO_Speed;
	setPinConfiguration(to);
}

static void resetPinConfiguration(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	copyPinConfiguration(&(pin->resetConfiguration), pin);
	pin->highLevelFunction  = IOs.HIGH_LEVEL_FUNCTIONS.DEFAULT;
}

