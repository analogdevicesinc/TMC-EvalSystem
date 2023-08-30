#include "hal/HAL.h"
#include "hal/USB.h"

#include "hal/Landungsbruecke/freescale/USB_CDC/USB0.h"
#include "hal/Landungsbruecke/freescale/USB_CDC/USB1.h"
#include "hal/Landungsbruecke/freescale/USB_CDC/Tx1.h"
#include "hal/Landungsbruecke/freescale/USB_CDC/Rx1.h"
#include "hal/Landungsbruecke/freescale/USB_CDC/CDC1.h"
#include "hal/Landungsbruecke/freescale/USB_CDC/CS1.h"

extern uint8_t USB_DCI_DeInit(void);
extern uint8_t USB_Class_CDC_DeInit(uint8_t controller_ID);
extern uint8_t USB_Class_DeInit(uint8_t controller_ID);

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, uint8_t number);
static uint8_t rxN(uint8_t *ch, uint8_t number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

RXTXTypeDef USB =
{
	.init            = init,
	.deInit          = deInit,
	.rx              = rx,
	.tx              = tx,
	.rxN             = rxN,
	.txN             = txN,
	.clearBuffers    = clearBuffers,
	.baudRate        = 115200,
	.bytesAvailable  = bytesAvailable
};

void init()
{
	USB0_Init();
	Tx1_Init();
	Rx1_Init();
	USB1_Init();
	enable_irq(INT_USB0-16);
}

uint8_t rx(uint8_t *ch)
{
	return rxN(ch,1);
}

uint8_t rxN(uint8_t *str, uint8_t number)
{
	if(CDC1_GetCharsInRxBuf() >= number)
	{
		for(int32_t i = 0; i < number; i++)
		{
			if(CDC1_GetChar(&str[i])!= ERR_OK)
				return false;
		}
		return true;
	}
	return false;
}

void tx(uint8_t ch)
{
	CDC1_SendChar(ch);
}

void txN(uint8_t *str, uint8_t number)
{
	for(int32_t i = 0; i < number; i++)
	{
		tx(str[i]);
	}
}

static void clearBuffers(void)
{
	DisableInterrupts;
	Tx1_Init();
	Rx1_Init();
	EnableInterrupts;
}

static uint32_t bytesAvailable()
{
	return CDC1_GetCharsInRxBuf();
}

static void deInit(void)
{
	USB_DCI_DeInit();
	USB_Class_CDC_DeInit(0);
	USB_Class_DeInit(0);

	SIM_SCGC4 &= ~SIM_SCGC4_USBOTG_MASK;
	SIM_SCGC6 &= ~SIM_SCGC6_USBDCD_MASK;
	SIM_SOPT2 &= ~SIM_SOPT2_USBSRC_MASK;
}
