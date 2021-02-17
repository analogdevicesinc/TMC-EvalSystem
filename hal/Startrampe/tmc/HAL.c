#include "hal/derivative.h"
#include "hal/HAL.h"

#define AIRCR_VECTKEY_MASK ((uint32_t)0x05FA0000)

static void init(void);
static void reset(uint8_t ResetPeripherals);
static void NVIC_DeInit(void);

uint8_t hwid = 0;

static const IOsFunctionsTypeDef IOFunctions =
{
	.config  = &IOs,
	.pins    = &IOMap,
};

const HALTypeDef HAL =
{
	.init         = init,
	.reset        = reset,
	.NVIC_DeInit  = NVIC_DeInit,
	.SPI          = &SPI,
	.USB          = &USB,
	.LEDs         = &LEDs,
	.ADCs         = &ADCs,
	.IOs          = &IOFunctions,
	.RS232        = &RS232,
	.WLAN         = &WLAN,
	.Timer        = &Timer,
	.UART         = &UART
};

static void init(void)
{
	__enable_irq();

	systick_init();
	wait(100);

	IOs.init();
	IOMap.init();
	USB.init();
	SPI.init();
	RS232.init();
	LEDs.init();
	ADCs.init();
	WLAN.init();
}

static void __attribute((noreturn)) reset(uint8_t ResetPeripherals)
{
	// Disable interrupts
	__disable_irq();

	if(ResetPeripherals)
		SCB->AIRCR = AIRCR_VECTKEY_MASK | SCB_AIRCR_SYSRESETREQ_Msk;
	else
		SCB->AIRCR = AIRCR_VECTKEY_MASK | SCB_AIRCR_VECTRESET_Msk;

	// SYSRESETREQ does not happen instantly since peripheral reset timing is not specified.
	// Trap execution here so nothing else happens until the reset completes.
	while(1);
}

static void NVIC_DeInit(void)
{
	uint32_t index;

	for(index = 0; index < 8; index++)
	{
		NVIC->ICER[index] = 0xFFFFFFFF;
		NVIC->ICPR[index] = 0xFFFFFFFF;
	}

	for(index = 0; index < 240; index++)
	{
		 NVIC->IP[index] = 0x00000000;
	}
}

void _exit(int i)	// function has the attribute noreturn per default
{
	UNUSED(i);
	while(1) {};
}

void _kill(void)
{
}

void _getpid(void)
{
}
