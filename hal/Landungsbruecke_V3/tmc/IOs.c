/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


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

	rcu_periph_clock_enable(RCU_SYSCFG);
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);

}

static void setPinConfiguration(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	gpio_mode_set(pin->port, pin->configuration.GPIO_Mode, pin->configuration.GPIO_PuPd, pin->bitWeight);
	gpio_output_options_set(pin->port, pin->configuration.GPIO_OType, pin->configuration.GPIO_Speed, pin->bitWeight);

}

static void setPin2Output(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	pin->configuration.GPIO_Mode = GPIO_MODE_OUTPUT;
	setPinConfiguration(pin);
}

static void setPin2Input(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	pin->configuration.GPIO_Mode = GPIO_MODE_INPUT;
	setPinConfiguration(pin);
}

static void setPinState(IOPinTypeDef *pin, IO_States state)
{
	if(IS_DUMMY_PIN(pin))
		return;
	switch(state)
	{
	case IOS_LOW:
		pin->configuration.GPIO_Mode = GPIO_MODE_OUTPUT;
		*pin->resetBitRegister = pin->bitWeight;
		break;
	case IOS_HIGH:
		pin->configuration.GPIO_Mode = GPIO_MODE_OUTPUT;
		*pin->setBitRegister = pin->bitWeight;
		break;
	case IOS_OPEN:
		pin->configuration.GPIO_Mode = GPIO_MODE_ANALOG;
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

	if(pin->configuration.GPIO_Mode == GPIO_MODE_INPUT)
		pin->state = (GPIO_ISTAT(pin->port) & pin->bitWeight) ? IOS_HIGH : IOS_LOW;
	else if(pin->configuration.GPIO_Mode == GPIO_MODE_OUTPUT)
		pin->state = (GPIO_OCTL(pin->port) & pin->bitWeight) ? IOS_HIGH : IOS_LOW;
	else
		pin->state = IOS_OPEN;

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

	return (GPIO_ISTAT(pin->port) & pin->bitWeight)? 1 : 0;
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

