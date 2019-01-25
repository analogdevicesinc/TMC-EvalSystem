#include "../../HAL.h"
#include "../../IOs.h"

static void init();
static void setPinConfiguration(IOPinTypeDef *pin);
static void copyPinConfiguration(IOPinInitTypeDef *from, IOPinTypeDef*to);
static void resetPinConfiguration(IOPinTypeDef *pin);
static void setPin2Output(IOPinTypeDef *pin);
static void setPin2Input(IOPinTypeDef *pin);
static void setPinHigh(IOPinTypeDef *pin);
static void setPinLow(IOPinTypeDef *pin);
static void setPinState(IOPinTypeDef *pin, IO_States state);
static uint8 isPinHigh(IOPinTypeDef *pin);

IOsTypeDef IOs =
{
	.init        = init,
	.set         = setPinConfiguration,
	.reset       = resetPinConfiguration,
	.copy        = copyPinConfiguration,
	.toOutput    = setPin2Output,
	.toInput     = setPin2Input,
	.setHigh     = setPinHigh,
	.setLow      = setPinLow,
	.setToState  = setPinState,
	.isHigh      = isPinHigh
};

static void init()
{
	// Set the Clock divider for core/system clock
	// 96 MHz / 2 = 48 MHz
	SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(1);

	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK| SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK |SIM_SCGC5_PORTD_MASK);

	// Ausgabe des 16Mhz Taktes auf den CLK16 Pin
	SIM_SOPT2 &= ~SIM_SOPT2_CLKOUTSEL_MASK;
	SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(6);
	PORTC_PCR3 = PORT_PCR_MUX(5);
}

static void setPinConfiguration(IOPinTypeDef *pin)
{
	if(IS_DUMMY_PIN(pin))
		return;

	uint32 config = 0;
	switch(pin->configuration.GPIO_Mode)
	{
	case GPIO_Mode_IN:
		GPIO_PDD_SetPortInputDirectionMask(pin->GPIOBase, pin->bitWeight);
		config |= PORT_PCR_MUX(1);
		break;
	case GPIO_Mode_OUT:
		GPIO_PDD_SetPortOutputDirectionMask(pin->GPIOBase, pin->bitWeight);
		config |= PORT_PCR_MUX(1);
		break;
	case GPIO_Mode_AN:   config |= PORT_PCR_MUX(0);  break;
	case GPIO_Mode_AF1:  config |= PORT_PCR_MUX(1);  break;
	case GPIO_Mode_AF2:  config |= PORT_PCR_MUX(2);  break;
	case GPIO_Mode_AF3:  config |= PORT_PCR_MUX(3);  break;
	case GPIO_Mode_AF4:  config |= PORT_PCR_MUX(4);  break;
	case GPIO_Mode_AF5:  config |= PORT_PCR_MUX(5);  break;
	case GPIO_Mode_AF6:  config |= PORT_PCR_MUX(6);  break;
	case GPIO_Mode_AF7:  config |= PORT_PCR_MUX(7);  break;
	}

	switch(pin->configuration.GPIO_OType)
	{
	case GPIO_OType_PP:
		break;
	case GPIO_OType_OD:
		config |= PORT_PCR_ODE_MASK; // enable open drain
		break;
	}
	switch(pin->configuration.GPIO_Speed) // die Auswahl der Frequenz bewirkt keine Änderung
	{
	case GPIO_Speed_2MHz:    break;
	case GPIO_Speed_25MHz:   break;
	case GPIO_Speed_50MHz:   break;
	case GPIO_Speed_100MHz:  break;
	}

	switch(pin->configuration.GPIO_PuPd)
	{
	case GPIO_PuPd_NOPULL:
		config &= ~PORT_PCR_PE_MASK;
		break;
	case GPIO_PuPd_UP:
		config |= PORT_PCR_PE_MASK;
		config |= PORT_PCR_PS_MASK;
		break;
	case GPIO_PuPd_DOWN:
		config |= PORT_PCR_PE_MASK;
		config &= ~(PORT_PCR_PS_MASK);
		break;
	}

	PORT_PCR_REG(pin->portBase, pin->bit) = config;
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
		pin->configuration.GPIO_Mode   = GPIO_Mode_OUT;
		pin->configuration.GPIO_PuPd   = GPIO_PuPd_NOPULL;
		pin->configuration.GPIO_OType  = GPIO_OType_PP;
		setPinConfiguration(pin);
		*pin->resetBitRegister = pin->bitWeight;
		break;
	case IOS_HIGH:
		pin->configuration.GPIO_Mode   = GPIO_Mode_OUT;
		pin->configuration.GPIO_PuPd   = GPIO_PuPd_NOPULL;
		pin->configuration.GPIO_OType  = GPIO_OType_PP;
		setPinConfiguration(pin);
		*pin->setBitRegister = pin->bitWeight;
		break;
	case IOS_OPEN:
		pin->configuration.GPIO_Mode  = GPIO_Mode_AN;
		setPinConfiguration(pin);
		break;
	case IOS_NOCHANGE:
		break;
	}

	setPinConfiguration(pin);
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

static uint8 isPinHigh(IOPinTypeDef *pin) // Die Abfrage eines Pins funktioniert nur, wenn der Pin AF1 ist
{
	if(IS_DUMMY_PIN(pin))
		return -1;

	return (GPIO_PDIR_REG(pin->GPIOBase) & pin->bitWeight)? 1 : 0;
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

	// Extra Reset Konfiguration für CLK16
	if(pin == &IOMap.CLK16)
	{
		SIM_SOPT2 &= ~SIM_SOPT2_CLKOUTSEL_MASK;
		SIM_SOPT2 |= SIM_SOPT2_CLKOUTSEL(6);
		PORTC_PCR3 = PORT_PCR_MUX(5);
	}
}

