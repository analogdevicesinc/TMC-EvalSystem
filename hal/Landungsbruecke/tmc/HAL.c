#include "hal/HAL.h"

static void init(void);
static void reset(uint8_t ResetPeripherals);
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

static void __attribute((noreturn)) reset(uint8_t ResetPeripherals)
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
	uint8_t index;

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

typedef enum {
	STATE_Z    = 0,
	STATE_LOW  = 1,
	STATE_HIGH = 2
} Tristate;

// Helper for get_hwid()
static Tristate getTristate(IOPinTypeDef *pin)
{
	// Input with pulldown
	pin->configuration.GPIO_Mode = GPIO_Mode_IN;
	pin->configuration.GPIO_OType = GPIO_OType_OD;
	pin->configuration.GPIO_PuPd = GPIO_PuPd_DOWN;
	HAL.IOs->config->set(pin);

	if (HAL.IOs->config->isHigh(pin)) {
		// High despite pulldown -> High state
		return STATE_HIGH;
	}

	// Input with pullup
	pin->configuration.GPIO_Mode = GPIO_Mode_IN;
	pin->configuration.GPIO_OType = GPIO_OType_OD;
	pin->configuration.GPIO_PuPd = GPIO_PuPd_UP;
	HAL.IOs->config->set(pin);

	if (HAL.IOs->config->isHigh(pin)) {
		// High from pullup -> Z state
		return STATE_Z;
	} else {
		// Low despite pullup -> Low state
		return STATE_LOW;
	}
}

// Determines HW version of Landungsbruecke to distinct between 1.X and 2.0+
static void get_hwid(void)
{
	static uint8_t hwid_map[27] = {
		//	 Z   Z   Z   L   L   L   H   H   H <- ID_HW_1
		//	 Z   L   H   Z   L   H   Z   L   H <- ID_HW_0
			 1,  0,  0,  0,  0,  0,  0,  0,  0, // Z
			 0,  0,  0,  0,  0,  0,  0,  2,  0, // L <- ID_HW_2
			 0,  0,  0,  0,  0,  0,  0,  0,  0  // H
	};

	uint8_t tmp;
	tmp = getTristate(&HAL.IOs->pins->ID_HW_0)
	    + getTristate(&HAL.IOs->pins->ID_HW_1) * 3
	    + getTristate(&HAL.IOs->pins->ID_HW_2) * (3*3);

	hwid = hwid_map[tmp];
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
