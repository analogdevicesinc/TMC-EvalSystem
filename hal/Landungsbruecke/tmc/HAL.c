#include "../../HAL.h"

static void init(void);
static void reset(uint8 ResetPeripherals);
static void NVIC_DeInit(void);
static void get_hwid(void);

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
	Cpu.initClocks();
	Cpu.initLowLevel();
	EnableInterrupts;;

	systick_init();
	wait(100);

	IOs.init();
	IOMap.init();
	LEDs.init();
	ADCs.init();
	SPI.init();
	WLAN.init();
	RS232.init();
	USB.init();

	// Determine HW version
	get_hwid();
}

static void __attribute((noreturn)) reset(uint8 ResetPeripherals)
{
	DisableInterrupts;

	if(ResetPeripherals)
		SCB_AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_SYSRESETREQ_MASK;
	else
		SCB_AIRCR = SCB_AIRCR_VECTKEY(0x5FA) | SCB_AIRCR_VECTRESET_MASK;

	// SYSRESETREQ does not happen instantly since peripheral reset timing is not specified.
	// Trap execution here so nothing else happens until the reset completes.
	while(1);
}

static void NVIC_DeInit(void)
{
	uint8 index;

	asm volatile("CPSID I\n");	// disable interrupts

	// Clear all NVIC interrupts
	for(index = 0; index < (sizeof(NVIC_BASE_PTR->ICER)/sizeof(NVIC_BASE_PTR->ICER[0])); index++)
		NVIC_ICER_REG(NVIC_BASE_PTR,index) = 0xFFFFFFFF;	// Interrupt clear-enable Registers
	for(index = 0; index < (sizeof(NVIC_BASE_PTR->ICPR)/sizeof(NVIC_BASE_PTR->ICPR[0])); index++)
		NVIC_ICPR_REG(NVIC_BASE_PTR,index) = 0xFFFFFFFF;	// Interrupt clear-pending Registers

	// Reset interrupt priorities
	for(index = 0; index < (sizeof(NVIC_BASE_PTR->IP)/sizeof(NVIC_BASE_PTR->IP[0])); index++)
		NVIC_IP_REG(NVIC_BASE_PTR,index) = 0x00000000;

	SYST_CSR = 0; // disable systick
}

// Determines HW version of Landungsbruecke to distinct between 1.X and 2.0+
static void get_hwid(void)
{
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_HW_0);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_HW_1);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_HW_2);
	hwid =
			(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_HW_2) << 2) |
			(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_HW_1) << 1) |
			(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_HW_0) << 0);
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
