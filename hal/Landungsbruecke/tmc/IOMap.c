#include "../../HAL.h"
#include "../../IOMap.h"
#include "../freescale/PDD/GPIO_PDD.h"

static void init();

IOPinMapTypeDef IOMap =
{
	.init    = init,
	.ID_CLK  =  // IOPinTypeDef ID_CLK
	{
		.setBitRegister      = &(GPIOB_PSOR),        // uint32 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // uint32 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // Peripheral Port Base Pointer
		.GPIOBase            = PTB_BASE_PTR,         // Peripheral PTA base pointer, GPIO;
		.bitWeight           = GPIO_PDD_PIN_20,      // uint32 pinBitWeight Pin masks;
		.bit                 = 20,                   // uint8 bit;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.ID_CH0 =  // IOPinTypeDef ID_CH0
	{
		.setBitRegister      = &(GPIOB_PSOR),        // uint32 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // uint32 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // Peripheral Port Base Pointer
		.GPIOBase            = PTB_BASE_PTR,         // Peripheral PTA base pointer, GPIO;
		.bitWeight           = GPIO_PDD_PIN_18,      // uint32 pinBitWeight;
		.bit                 = 18,                   // uint8 bit;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.ID_CH1 =  // IOPinTypeDef ID_CH1
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_19,      // uint32 pinBitWeight;
		.bit                 = 19,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO0 =  // IOPinTypeDef DIO0
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_12,      // uint32 pinBitWeight;
		.bit                 = 12,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO1 =  // IOPinTypeDef DIO1
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_13,      // uint32 pinBitWeight;
		.bit                 = 13,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO2 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_0,       // uint32 pinBitWeight;
		.bit                 = 0,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO3 =  // IOPinTypeDef DIO3
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_1,       // uint32 pinBitWeight;
		.bit                 = 1,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO4 =  // IOPinTypeDef DIO4
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_2,       // uint32 pinBitWeight;
		.bit                 = 2,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO5 =  // IOPinTypeDef DIO5
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_3,       // uint32 pinBitWeight;
		.bit                 = 3,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},


	.DIO6 =  // IOPinTypeDef DIO6
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_2,       // uint32 pinBitWeight;
		.bit                 = 2,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO7 =  // IOPinTypeDef DIO7
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_1,       // uint32 pinBitWeight;
		.bit                 = 1,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO8 =  // IOPinTypeDef DIO8
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_5,       // uint32 pinBitWeight;
		.bit                 = 5,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO9 =  // IOPinTypeDef DIO9
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_4,       // uint32 pinBitWeight;
		.bit                 = 4,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO10 =  // IOPinTypeDef DIO10
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_7,       // uint32 pinBitWeight;
		.bit                 = 7,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO11 =  // IOPinTypeDef DIO11
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_6,       // uint32 pinBitWeight;
		.bit                 = 6,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.CLK16 =  // IOPinTypeDef CLK16
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_3,       // uint32 pinBitWeight;
		.bit                 = 3,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AF5,            // GPIOMode_TypeDef GPIO_Mode; ??
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI2_CSN0 =  // IOPinTypeDef SPI2_CSN0
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_0,       // uint32 pinBitWeight;
		.bit                 = 0,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI2_CSN1 =  // IOPinTypeDef SPI2_CSN1 Konflikt mit DIO5
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_5,       // uint32 pinBitWeight;
		.bit                 = 5,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI2_CSN2 =  // IOPinTypeDef SPI2_CSN2
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_4,       // uint32 pinBitWeight;
		.bit                 = 4,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI2_SCK =  // IOPinTypeDef SPI2_SCK
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_21,      // uint32 pinBitWeight;
		.bit                 = 21,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI2_SDO =  // IOPinTypeDef SPI2_SDO
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_23,      // uint32 pinBitWeight;
		.bit                 = 23,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI2_SDI =  // IOPinTypeDef SPI2_SDI
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_22,      // uint32 pinBitWeight;
		.bit                 = 22,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI1_CSN =  // IOPinTypeDef SPI1_CSN
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_10,      // uint32 pinBitWeight;
		.bit                 = 10,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI1_SCK =  // IOPinTypeDef SPI1_SCK
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_11,      // uint32 pinBitWeight;
		.bit                 = 11,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI1_SDI =  // IOPinTypeDef SPI1_SDI
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_17,      // uint32 pinBitWeight;
		.bit                 = 17,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.SPI1_SDO =  // IOPinTypeDef SPI1_SDO
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_16,      // uint32 pinBitWeight;
		.bit                 = 16,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO12 =  // IOPinTypeDef DIO12
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_16,      // uint32 pinBitWeight;
		.bit                 = 16,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO13 =  // IOPinTypeDef DIO13
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_17,      // uint32 pinBitWeight;
		.bit                 = 17,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},


	.DIO14 =  // IOPinTypeDef DIO14
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_18,      // uint32 pinBitWeight;
		.bit                 = 18,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO15 =  // IOPinTypeDef DIO15
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_1,       // uint32 pinBitWeight;
		.bit                 = 1,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO16 =  // IOPinTypeDef DIO16
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_0,       // uint32 pinBitWeight;
		.bit                 = 0,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO17 =  // IOPinTypeDef DIO17
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_3,       // uint32 pinBitWeight;
		.bit                 = 3,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO18 =  // IOPinTypeDef DIO18
	{
		.setBitRegister      = &(GPIOD_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOD_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTD_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTD_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_2,       // uint32 pinBitWeight;
		.bit                 = 2,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DIO19 =  // IOPinTypeDef DIO19
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_15,      // uint32 pinBitWeight;
		.bit                 = 15,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},


	.WIRELESS_TX =  // IOPinTypeDef WIRELESS_TX
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_14,      // uint32 pinBitWeight;
		.bit                 = 14,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.WIRELESS_RX =  // IOPinTypeDef WIRELESS_RX
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_15,      // uint32 pinBitWeight;
		.bit                 = 15,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.WIRELESS_NRST =  // IOPinTypeDef WIRELESS_NRST
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_16,      // uint32 pinBitWeight;
		.bit                 = 16,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.RS232_TX =  // IOPinTypeDef RS232_TX
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_24,      // uint32 pinBitWeight;
		.bit                 = 24,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.RS232_RX =  // IOPinTypeDef RS232_RX
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_25,      // uint32 pinBitWeight;
		.bit                 = 25,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.USB_V_BUS =  // IOPinTypeDef USB_V_BU
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,         // GPIO_TypeDef *port;
		.bitWeight           = GPIO_PDD_PIN_4,       // uint32 pinBitWeight;
		.bit                 = 4,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_100MHz,        // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},


	.LED_STAT =  // IOPinTypeDef LED_STAT
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_2,       // uint32 pinBitWeight;
		.bit                 = 2,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.LED_ERROR =  // IOPinTypeDef LED_ERROR
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_1,       // uint32 pinBitWeight;
		.bit                 = 1,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.EXTIO_2 =  // IOPinTypeDef EXTIO_2
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_0,       // uint32 pinBitWeight;
		.bit                 = 0,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.EXTIO_3 =  // IOPinTypeDef EXTIO_3
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_1,       // uint32 pinBitWeight;
		.bit                 = 1,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},
	.EXTIO_4 =  // IOPinTypeDef EXTIO_4
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_2,       // uint32 pinBitWeight;
		.bit                 = 2,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},
	.EXTIO_5 =  // IOPinTypeDef EXTIO_5
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_3,       // uint32 pinBitWeight;
		.bit                 = 3,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.EXTIO_6 =  // IOPinTypeDef EXTIO_6
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_4,       // uint32 pinBitWeight;
		.bit                 = 4,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.EXTIO_7=  // IOPinTypeDef EXTIO_7
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_5,       // uint32 pinBitWeight;
		.bit                 = 5,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},


	.EEPROM_SCK =  // IOPinTypeDef EEPROM_SCK
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_5,       // uint32 pinBitWeight;
		.bit                 = 5,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},


	.EEPROM_SI =  // IOPinTypeDef EEPROM_SI
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_7,       // uint32 pinBitWeight;
		.bit                 = 7,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.EEPROM_SO =  // IOPinTypeDef EEPROM_SO
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_6,       // uint32 pinBitWeight;
		.bit                 = 6,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.EEPROM_NCS =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_8,       // uint32 pinBitWeight;
		.bit                 = 8,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED0 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_11,      // uint32 pinBitWeight;
		.bit                 = 11,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED1 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_12,      // uint32 pinBitWeight;
		.bit                 = 12,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED2 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_13,      // uint32 pinBitWeight;
		.bit                 = 13,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED3 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_14,      // uint32 pinBitWeight;
		.bit                 = 14,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED4 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_10,      // uint32 pinBitWeight;
		.bit                 = 10,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED5 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOC_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOC_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTC_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTC_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_9,       // uint32 pinBitWeight;
		.bit                 = 9,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.MIXED6 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_6,       // uint32 pinBitWeight;
		.bit                 = 6,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.ID_HW_0 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOE_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOE_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTE_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTE_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_26,      // uint32 pinBitWeight;
		.bit                 = 26,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.ID_HW_1 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_17,      // uint32 pinBitWeight;
		.bit                 = 17,                   // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.ID_HW_2 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIOB_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOB_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTB_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTB_BASE_PTR,
		.bitWeight           = GPIO_PDD_PIN_9,       // uint32 pinBitWeight;
		.bit                 = 9,                    // uint8 pinBitWeight;
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	},

	.DUMMY =  // Dummy Pin
	{
		.setBitRegister      = &(GPIOA_PSOR),        // __IO uint16 *setBitRegister;
		.resetBitRegister    = &(GPIOA_PCOR),        // __IO uint16 *resetBitRegister;
		.portBase            = PORTA_BASE_PTR,       // GPIO_TypeDef *port;
		.GPIOBase            = PTA_BASE_PTR,
		.bitWeight           = DUMMY_BITWEIGHT,      // invalid
		.bit                 = -1,                   // invalid
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_Mode_AN,             // GPIOMode_TypeDef GPIO_Mode;
			.GPIO_OType  = GPIO_OType_PP,            // GPIOSpeed_TypeDef GPIO_Speed;
			.GPIO_Speed  = GPIO_Speed_50MHz,         // GPIOOType_TypeDef GPIO_OType;
			.GPIO_PuPd   = GPIO_PuPd_NOPULL          // GPIOPuPd_TypeDef GPIO_PuPd;
		}
	}
};


static void init()
{
	HAL.IOs->config->reset(&HAL.IOs->pins->ID_CLK);
	HAL.IOs->config->reset(&HAL.IOs->pins->ID_CH0);
	HAL.IOs->config->reset(&HAL.IOs->pins->ID_CH1);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO0);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO1);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO2);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO3);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO4);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO5);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO6);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO7);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO8);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO9);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO10);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO11);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_CSN0);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_CSN1);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_CSN2);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SCK);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDO);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDI);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_CSN);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SCK);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDI);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDO);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO12);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO13);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO14);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO15);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO16);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO17);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO18);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO19);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIRELESS_TX);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIRELESS_RX);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIRELESS_NRST);
	HAL.IOs->config->reset(&HAL.IOs->pins->RS232_TX);
	HAL.IOs->config->reset(&HAL.IOs->pins->RS232_RX);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_BUS);
	HAL.IOs->config->reset(&HAL.IOs->pins->LED_STAT);
	HAL.IOs->config->reset(&HAL.IOs->pins->LED_ERROR);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXTIO_2);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXTIO_3);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXTIO_4);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXTIO_5);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_SCK);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_SI);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_SO);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_NCS);
	HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED0);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED1);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED2);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED3);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED4);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED5);
	HAL.IOs->config->reset(&HAL.IOs->pins->MIXED6);
	HAL.IOs->config->reset(&HAL.IOs->pins->ID_HW_0);
	HAL.IOs->config->reset(&HAL.IOs->pins->ID_HW_1);
	HAL.IOs->config->reset(&HAL.IOs->pins->ID_HW_2);
}
