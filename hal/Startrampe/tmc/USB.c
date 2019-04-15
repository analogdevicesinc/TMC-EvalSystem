/****************************************************
  Projekt: TMCM-3110 etc.

  Modul:   USB-STX2.c
           USB Virtual COM Port
           für Module mit dem STM32F2xx.

  Datum:   3.8.2012 OK
*****************************************************/

#include <stdlib.h>
#include "stm32f2xx.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_core.h"
#include "usb_dcd_int.h"
#include "hal/HAL.h"

// USB Device Descriptor
#define USBD_VID                      0x16D0
#define USBD_PID                      0x07E4
#define USBD_LANGID_STRING            0x409
#define USBD_MANUFACTURER_STRING      "Trinamic Motion Control"
#define USBD_PRODUCT_FS_STRING        "Trinamic-Eval (virtual COM)"
#define USBD_SERIALNUMBER_FS_STRING   "TMCEVAL"
#define USBD_CONFIGURATION_FS_STRING  "VCP Config"
#define USBD_INTERFACE_FS_STRING      "VCP Interface"

#define BUFFER_SIZE 2048 // KEEP THIS SIZE AS IT MATCHES BUFFERSIZE OF usbd_cdc_core.c

void __attribute__ ((interrupt)) OTG_FS_IRQHandler(void);

static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len);

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t bytesAvailable();
extern void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len);

USB_OTG_CORE_HANDLE USB_OTG_dev;  // Handle for USB Core Functions
extern uint8_t  APP_Rx_Buffer[];  // TX Buffer
extern uint32_t APP_Rx_ptr_in;    // TX ptr


static volatile uint8_t rxBuffer[BUFFER_SIZE];

static volatile uint32_t available = 0;

static RXTXBufferingTypeDef buffers =
{
	.rx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = rxBuffer
	},

	.tx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = APP_Rx_Buffer
	}
};

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

typedef struct
{
	uint32_t  bitrate;
	uint8_t   format;
	uint8_t   paritytype;
	uint8_t   datatype;
} LINE_CODING;


// User Callback Funktionen for den USB-Core
USBD_Usr_cb_TypeDef USR_cb =
{
	USBD_USR_Init,
	USBD_USR_DeviceReset,
	USBD_USR_DeviceConfigured,
	USBD_USR_DeviceSuspended,
	USBD_USR_DeviceResumed,
	USBD_USR_DeviceConnected,
	USBD_USR_DeviceDisconnected
};

CDC_IF_Prop_TypeDef VCP_fops =
{
	VCP_Init,
	VCP_DeInit,
	VCP_Ctrl,
	VCP_DataTx,
	VCP_DataRx
};

//COM-Port-Verbindungsdaten
static LINE_CODING linecoding =
{
	2048000, /* baud rate*/
	0x00,    /* stop bits-1*/
	0x00,    /* parity - none*/
	0x08     /* nb. of bits 8*/
};

//Datenstruktur mit Funktionen zur Generierung der Desktriptor-Strings
USBD_DEVICE USR_desc =
{
	USBD_USR_DeviceDescriptor,
	USBD_USR_LangIDStrDescriptor,
	USBD_USR_ManufacturerStrDescriptor,
	USBD_USR_ProductStrDescriptor,
	USBD_USR_SerialStrDescriptor,
	USBD_USR_ConfigStrDescriptor,
	USBD_USR_InterfaceStrDescriptor
};

//Deskriptor-Datenstrukturen
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
{
	0x12,                        /*bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,  /*bDescriptorType*/
	0x00,                        /*bcdUSB */
	0x02,
	0x02,                        /*bDeviceClass*/
	0x00,                        /*bDeviceSubClass*/
	0x00,                        /*bDeviceProtocol*/
	USB_OTG_MAX_EP0_SIZE,        /*bMaxPacketSize*/
	LOBYTE(USBD_VID),            /*idVendor*/
	HIBYTE(USBD_VID),            /*idVendor*/
	LOBYTE(USBD_PID),            /*idVendor*/
	HIBYTE(USBD_PID),            /*idVendor*/
	0x00,                        /*bcdDevice rel. 2.00*/
	0x02,
	USBD_IDX_MFC_STR,            /*Index of manufacturer  string*/
	USBD_IDX_PRODUCT_STR,        /*Index of product string*/
	USBD_IDX_SERIAL_STR,         /*Index of serial number string*/
	USBD_CFG_MAX_NUM             /*bNumConfigurations*/
};  /* USB_DeviceDescriptor */

__ALIGN_BEGIN uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
	USB_LEN_DEV_QUALIFIER_DESC,
	USB_DESC_TYPE_DEVICE_QUALIFIER,
	0x00,
	0x02,
	0x00,
	0x00,
	0x00,
	0x40,
	0x01,
	0x00
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
	USB_SIZ_STRING_LANGID,
	USB_DESC_TYPE_STRING,
	LOBYTE(USBD_LANGID_STRING),
	HIBYTE(USBD_LANGID_STRING)
};


//-----------------------------------------------------------------
// USB Core Callbacks
//-----------------------------------------------------------------

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
	UNUSED(pdev);

	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_DM);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_DP);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_BUS);

	GPIO_PinAFConfig(HAL.IOs->pins->USB_V_DM.port, HAL.IOs->pins->USB_V_DM.bit, GPIO_AF_OTG1_FS);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG1_FS);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_OTG1_FS);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;
}

void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
	UNUSED(pdev);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel                    = OTG_FS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USB_OTG_BSP_uDelay (const uint32_t usec)
{
	uint32_t count = 0;
	const uint32_t utime = (120 * usec / 7);

	do
	{
		if(++count > utime)
			return;
	} while(1);
}

void USB_OTG_BSP_mDelay (const uint32_t msec)
{
	USB_OTG_BSP_uDelay(msec * 1000);
}

void USBD_USR_Init(void)
{

}

void USBD_USR_DeviceReset(uint8_t speed )
{
	switch (speed)
	{
	case USB_OTG_SPEED_HIGH:
		break;
	case USB_OTG_SPEED_FULL:
		break;
	default:
		break;
	}
}

void USBD_USR_DeviceConfigured(void)
{

}

void USBD_USR_DeviceSuspended(void)
{

}

void USBD_USR_DeviceResumed(void)
{

}

void USBD_USR_DeviceConnected(void)
{

}

void USBD_USR_DeviceDisconnected(void)
{

}

//-----------------------------------------------------------------
// USB Core Descriptor Functions
//-----------------------------------------------------------------
uint8_t* USBD_USR_DeviceDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	*length = sizeof(USBD_DeviceDesc);

	return USBD_DeviceDesc;
}


uint8_t* USBD_USR_LangIDStrDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	*length = sizeof(USBD_LangIDDesc);

	return USBD_LangIDDesc;
}


uint8_t* USBD_USR_ProductStrDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	USBD_GetString((uint8_t *) USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);

	return USBD_StrDesc;
}


uint8_t* USBD_USR_ManufacturerStrDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	USBD_GetString((uint8_t *) USBD_MANUFACTURER_STRING, USBD_StrDesc, length);

	return USBD_StrDesc;
}


uint8_t* USBD_USR_SerialStrDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	USBD_GetString((uint8_t *) USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);

	return USBD_StrDesc;
}


uint8_t* USBD_USR_ConfigStrDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	USBD_GetString((uint8_t *) USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);

	return USBD_StrDesc;
}


uint8_t* USBD_USR_InterfaceStrDescriptor(uint8_t speed , uint16_t *length)
{
	UNUSED(speed);

	USBD_GetString((uint8_t *) USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);

	return USBD_StrDesc;
}


//-----------------------------------------------------------------
// Virtual COM Port Functions
//-----------------------------------------------------------------

static uint16_t VCP_Init(void)
{
	return USBD_OK;
}

static uint16_t VCP_DeInit(void)
{
	return USBD_OK;
}

static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
	UNUSED(Len);

	switch (Cmd)
	{
	case SEND_ENCAPSULATED_COMMAND:
		break;
	case GET_ENCAPSULATED_RESPONSE:
		break;
	case SET_COMM_FEATURE:
		break;
	case GET_COMM_FEATURE:
		break;
	case CLEAR_COMM_FEATURE:
		break;
	case SET_LINE_CODING:
		linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
		linecoding.format = Buf[4];
		linecoding.paritytype = Buf[5];
		linecoding.datatype = Buf[6];
		break;
	case GET_LINE_CODING:
		Buf[0] = (uint8_t) (linecoding.bitrate);
		Buf[1] = (uint8_t) (linecoding.bitrate >> 8);
		Buf[2] = (uint8_t) (linecoding.bitrate >> 16);
		Buf[3] = (uint8_t) (linecoding.bitrate >> 24);
		Buf[4] = linecoding.format;
		Buf[5] = linecoding.paritytype;
		Buf[6] = linecoding.datatype;
		break;
	case SET_CONTROL_LINE_STATE:
		break;
	case SEND_BREAK:
		break;
	default:
		break;
	}

	return USBD_OK;
}

static uint16_t VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	UNUSED(Buf);
	UNUSED(Len);

	return USBD_OK;
}

static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
	for(uint32_t i = 0; i < Len; i++)
	{
		buffers.rx.buffer[buffers.rx.wrote] = Buf[i];
		buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
		available++;
	}

	return USBD_OK;
}


// Forward Interrupt to Core
void OTG_FS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}



//-----------------------------------------------------------------
// Funtions for interface
//-----------------------------------------------------------------
static void init(void)
{
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_BUS);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_DM);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_DP);

	USBD_Init(
				&USB_OTG_dev,
				USB_OTG_FS_CORE_ID,
				&USR_desc,
				&USBD_CDC_cb,
				&USR_cb
			);
}

static void tx(uint8_t ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;
	APP_Rx_ptr_in = buffers.tx.wrote;
}

static uint8_t rx(uint8_t *ch)
{
	if(!available)
		return 0;
	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;
	available--;

	return 1;
}

static void txN(uint8_t *str, unsigned char number)
{
	for(int32_t i = 0; i < number; i++)
		tx(str[i]);
}

static uint8_t rxN(uint8_t *str, unsigned char number)
{
	if(bytesAvailable() < number)
		return 0;

	for(int32_t i = 0; i < number; i++)
		rx(&str[i]);

	return 1;
}

static void clearBuffers(void)
{
	__disable_irq();
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
	__enable_irq();
}

static uint32_t bytesAvailable()
{
	return available;
}

static void deInit(void)
{
	DCD_DevDisconnect(&USB_OTG_dev);
}
