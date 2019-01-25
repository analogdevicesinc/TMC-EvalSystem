#include "../../HAL.h"
#include "../../USB.h"

#include "../freescale/USB_CDC/USB0.h"
#include "../freescale/USB_CDC/USB1.h"
#include "../freescale/USB_CDC/Tx1.h"
#include "../freescale/USB_CDC/Rx1.h"
#include "../freescale/USB_CDC/CDC1.h"
#include "../freescale/USB_CDC/CS1.h"

extern uint8 USB_DCI_DeInit(void);
extern uint8 USB_Class_CDC_DeInit(uint8 controller_ID);
extern uint8 USB_Class_DeInit(uint8 controller_ID);

static void init();
static void deInit();
static void tx(uint8 ch);
static uint8 rx(uint8 *ch);
static void txN(uint8 *str, uint8 number);
static uint8 rxN(uint8 *ch, uint8 number);
static void clearBuffers(void);
static uint32 bytesAvailable();

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

uint8 rx(uint8 *ch)
{
	return rxN(ch,1);
}

uint8 rxN(uint8 *str, uint8 number)
{
	if(CDC1_GetCharsInRxBuf() >= number)
	{
		for(int32 i = 0; i < number; i++)
		{
			if(CDC1_GetChar(&str[i])!= ERR_OK)
				return FALSE;
		}
		return TRUE;
	}
	return FALSE;
}

void tx(uint8 ch)
{
	CDC1_SendChar(ch);
}

void txN(uint8 *str, uint8 number)
{
	for(int32 i = 0; i < number; i++)
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

static uint32 bytesAvailable()
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
