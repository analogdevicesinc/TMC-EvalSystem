#include "hal/IOMap.h"
#include "hal/HAL.h"

static void init();

static IOPinTypeDef *_pins[] =
{
	&IOMap.ID_CLK,
	&IOMap.ID_CH0,
	&IOMap.ID_CH1,
	&IOMap.DIO0,
	&IOMap.DIO1,
	&IOMap.DIO2,
	&IOMap.DIO3,
	&IOMap.DIO4,
	&IOMap.DIO5,
	&IOMap.DIO6,
	&IOMap.DIO7,
	&IOMap.DIO8,
	&IOMap.DIO9,
	&IOMap.DIO10,
	&IOMap.DIO10_A,
	&IOMap.DIO10_B,
	&IOMap.DIO11,
	&IOMap.DIO11_A,
	&IOMap.DIO11_B,
	&IOMap.MUX_1,
	&IOMap.MUX_2,
	&IOMap.CLK16,
	&IOMap.SPI2_CSN0,
	&IOMap.SPI2_CSN1,
	&IOMap.SPI2_CSN2,
	&IOMap.SPI2_SCK,
	&IOMap.SPI2_SDO,
	&IOMap.SPI2_SDI,
	&IOMap.SPI1_CSN,
	&IOMap.SPI1_SCK,
	&IOMap.SPI1_SDI,
	&IOMap.SPI1_SDO,
	&IOMap.DIO12,
	&IOMap.DIO13,
	&IOMap.DIO14,
	&IOMap.DIO15,
	&IOMap.DIO16,
	&IOMap.DIO17,
	&IOMap.DIO18,
	&IOMap.DIO19,
	&IOMap.RS232_TX,
	&IOMap.RS232_RX,
	&IOMap.USB_V_BUS,
	&IOMap.USB_V_DM,
	&IOMap.USB_V_DP,
	&IOMap.LED_STAT,
	&IOMap.LED_ERROR,
	&IOMap.EXT0,
	&IOMap.EXT1,
	&IOMap.EXT2,
	&IOMap.EXT3,
	&IOMap.EXT4,
	&IOMap.EEPROM_SCK,
	&IOMap.EEPROM_SI,
	&IOMap.EEPROM_SO,
	&IOMap.EEPROM_NCS,
	&IOMap.ADC_VM,
	&IOMap.AIN0,
	&IOMap.AIN1,
	&IOMap.AIN2,
	&IOMap.WIFI_EN,
	&IOMap.WIFI_RST,
	&IOMap.WIFI_TX,
	&IOMap.WIFI_RX,
	&IOMap.DUMMY
};

IOPinMapTypeDef IOMap =
{
	.init   = init,
	.pins   = &_pins[0],
	.ID_CLK =  // IOPinTypeDef ID_CLK
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_9,       // uint32_t pinBitWeight
		.bit                 = 9,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.ID_CH0 =  // IOPinTypeDef ID_CH0
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_8,       // uint32_t pinBitWeight
		.bit                 = 8,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.ID_CH1 =  // IOPinTypeDef ID_CH1
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_7,       // uint32_t pinBitWeight
		.bit                 = 7,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO0 =  // IOPinTypeDef DIO0
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_2,       // uint32_t pinBitWeight
		.bit                 = 2,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO1 =  // IOPinTypeDef DIO1
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_3,       // uint32_t pinBitWeight
		.bit                 = 3,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO2 =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_0,       // uint32_t pinBitWeight
		.bit                 = 0,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO3 =  // IOPinTypeDef DIO3
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_1,       // uint32_t pinBitWeight
		.bit                 = 1,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO4 =  // IOPinTypeDef DIO4
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_8,       // uint32_t pinBitWeight
		.bit                 = 8,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO5 =  // IOPinTypeDef DIO5
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_9,       // uint32_t pinBitWeight
		.bit                 = 9,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO6 =  // IOPinTypeDef DIO6
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_8,       // uint32_t pinBitWeight
		.bit                 = 8,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO7 =  // IOPinTypeDef DIO7
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_9,       // uint32_t pinBitWeight
		.bit                 = 9,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO8 =  // IOPinTypeDef DIO8
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_10,      // uint32_t pinBitWeight
		.bit                 = 10,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO9 =  // IOPinTypeDef DIO9
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_11,      // uint32_t pinBitWeight
		.bit                 = 11,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO10 =  // IOPinTypeDef DIO10
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = DUMMY_BITWEIGHT,      // uint32_t pinBitWeight
		.bit                 = -1,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.DIO10_A =  // IOPinTypeDef DIO10_A
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_12,      // uint32_t pinBitWeight
		.bit                 = 12,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.DIO10_B =  // IOPinTypeDef DIO10_B
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_0,      // uint32_t pinBitWeight
		.bit                 = 0,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO11 =  // IOPinTypeDef DIO11
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = DUMMY_BITWEIGHT,      // uint32_t pinBitWeight
		.bit                 = -1,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.DIO11_A =  // IOPinTypeDef DIO11_A
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_13,      // uint32_t pinBitWeight
		.bit                 = 13,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.DIO11_B =  // IOPinTypeDef DIO11_B
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_1,      // uint32_t pinBitWeight
		.bit                 = 1,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.MUX_1 =  // IOPinTypeDef MUX_1
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_2,      // uint32_t pinBitWeight
		.bit                 = 2,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.MUX_2 =  // IOPinTypeDef MUX_2
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_1,      // uint32_t pinBitWeight
		.bit                 = 1,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	.CLK16 =  // IOPinTypeDef CLK16
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_8,       // uint32_t pinBitWeight
		.bit                 = 8,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI2_CSN0 =  // IOPinTypeDef SPI2_CSN0
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_4,      // uint32_t pinBitWeight
		.bit                 = 4,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI2_CSN1 =  // IOPinTypeDef SPI2_CSN1
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_14,      // uint32_t pinBitWeight
		.bit                 = 14,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI2_CSN2 =  // IOPinTypeDef SPI2_CSN2
	{
		.setBitRegister      = &(GPIO_BOP(GPIOE)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOE)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOE,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_15,      // uint32_t pinBitWeight
		.bit                 = 15,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI2_SCK =  // IOPinTypeDef SPI2_SCK
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_5,      // uint32_t pinBitWeight
		.bit                 = 5,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI2_SDO =  // IOPinTypeDef SPI2_SDO
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_6,      // uint32_t pinBitWeight
		.bit                 = 6,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI2_SDI =  // IOPinTypeDef SPI2_SDI
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_7,      // uint32_t pinBitWeight
		.bit                 = 7,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI1_CSN =  // IOPinTypeDef SPI1_CSN
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_12,      // uint32_t pinBitWeight
		.bit                 = 12,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI1_SCK =  // IOPinTypeDef SPI1_SCK
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_13,      // uint32_t pinBitWeight
		.bit                 = 13,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI1_SDI =  // IOPinTypeDef SPI1_SDI
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_15,      // uint32_t pinBitWeight
		.bit                 = 15,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.SPI1_SDO =  // IOPinTypeDef SPI1_SDO
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_14,      // uint32_t pinBitWeight
		.bit                 = 14,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO12 =  // IOPinTypeDef DIO12
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_8,       // uint32_t pinBitWeight
		.bit                 = 8,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO13 =  // IOPinTypeDef DIO13
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_9,       // uint32_t pinBitWeight
		.bit                 = 9,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO14 =  // IOPinTypeDef DIO14
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_10,       // uint32_t pinBitWeight
		.bit                 = 10,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO15 =  // IOPinTypeDef DIO15
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_11,       // uint32_t pinBitWeight
		.bit                 = 11,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO16 =  // IOPinTypeDef DIO016
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_12,       // uint32_t pinBitWeight
		.bit                 = 12,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO17 =  // IOPinTypeDef DIO017
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_10,       // uint32_t pinBitWeight
		.bit                 = 10,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO18 =  // IOPinTypeDef DIO18
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_11,       // uint32_t pinBitWeight
		.bit                 = 11,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DIO19 =  // IOPinTypeDef DIO19
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_15,       // uint32_t pinBitWeight
		.bit                 = 15,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.RS232_TX =  // IOPinTypeDef RS232_TX
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = DUMMY_BITWEIGHT,       // uint32_t pinBitWeight
		.bit                 = -1,                // unsigned char bit
		.resetConfiguration  =
		{
				.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
				.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
				.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
				.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.RS232_RX =  // IOPinTypeDef RS232_RX
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = DUMMY_BITWEIGHT,       // uint32_t pinBitWeight
		.bit                 = -1,                // unsigned char bit
		.resetConfiguration  =
		{
				.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
				.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
				.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
				.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.USB_V_BUS =  // IOPinTypeDef USB_V_BUS
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_9,       // uint32_t pinBitWeight
		.bit                 = 9,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.USB_V_DM =  // IOPinTypeDef USB_V_DM
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_11,      // uint32_t pinBitWeight
		.bit                 = 11,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.USB_V_DP =  // IOPinTypeDef USB_V_DP
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_12,      // uint32_t pinBitWeight
		.bit                 = 12,               // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.LED_STAT =  // IOPinTypeDef LED_STAT
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_0,       // uint32_t pinBitWeight
		.bit                 = 0,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.LED_ERROR =  // IOPinTypeDef LED_ERROR
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_1,       // uint32_t pinBitWeight
		.bit                 = 1,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EXT0 =  // IOPinTypeDef EXT0
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_7,       // uint32_t pinBitWeight
		.bit                 = 7,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EXT1 =  // IOPinTypeDef EXT1
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_6,       // uint32_t pinBitWeight
		.bit                 = 6,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EXT2 =  // IOPinTypeDef EXT2
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_5,       // uint32_t pinBitWeight
		.bit                 = 5,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EXT3 =  // IOPinTypeDef EXT3
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_4,       // uint32_t pinBitWeight
		.bit                 = 4,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EXT4 =  // IOPinTypeDef EXT4
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_3,       // uint32_t pinBitWeight
		.bit                 = 3,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_MAX,    // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EEPROM_SCK =  // IOPinTypeDef EEPROM_SCK
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_10,       // uint32_t pinBitWeight
		.bit                 = 10,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EEPROM_SI =  // IOPinTypeDef EEPROM_SI
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_12,       // uint32_t pinBitWeight
		.bit                 = 12,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EEPROM_SO =  // IOPinTypeDef EEPROM_SO
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_11,       // uint32_t pinBitWeight
		.bit                 = 11,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.EEPROM_NCS =  // IOPinTypeDef EEPROM_NCS
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_15,       // uint32_t pinBitWeight
		.bit                 = 15,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_OUTPUT,        // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.ADC_VM =  // IOPinTypeDef ADC_VM
	{
		.setBitRegister      = &(GPIO_BOP(GPIOA)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOA)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOA,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_3,       // uint32_t pinBitWeight
		.bit                 = 3,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_ANALOG,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.AIN0 =  // IOPinTypeDef AIN0
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_4,       // uint32_t pinBitWeight
		.bit                 = 4,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_ANALOG,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.AIN1 =  // IOPinTypeDef AIN1
	{
		.setBitRegister      = &(GPIO_BOP(GPIOC)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOC)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOC,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_5,       // uint32_t pinBitWeight
		.bit                 = 5,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_ANALOG,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.AIN2 =  // IOPinTypeDef AIN2
	{
		.setBitRegister      = &(GPIO_BOP(GPIOB)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOB)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOB,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_0,       // uint32_t pinBitWeight
		.bit                 = 0,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_ANALOG,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.WIFI_EN =  // IOPinTypeDef WIFI_EN
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_7,       // uint32_t pinBitWeight
		.bit                 = 7,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},
	
	.WIFI_RST =  // IOPinTypeDef WIFI_RST
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_4,       // uint32_t pinBitWeight
		.bit                 = 4,                // unsigned char bit
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.WIFI_TX =  // IOPinTypeDef WIFI_TX
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_5,       // uint32_t pinBitWeight
		.bit                 = 5,                // unsigned char bit
		.resetConfiguration  =
		{
				.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
				.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
				.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
				.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.WIFI_RX =  // IOPinTypeDef WIFI_RX
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = GPIO_PIN_6,       // uint32_t pinBitWeight
		.bit                 = 6,                // unsigned char bit
		.resetConfiguration  =
		{
				.GPIO_Mode   = GPIO_MODE_AF,         // GPIOMode_TypeDef GPIO_Mode
				.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
				.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
				.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
		}
	},

	.DUMMY =  // IOPinTypeDef
	{
		.setBitRegister      = &(GPIO_BOP(GPIOD)),  // __IO uint16_t *setBitRegister
		.resetBitRegister    = &(GPIO_BC(GPIOD)),  // __IO uint16_t *resetBitRegister
		.port                = GPIOD,            // GPIO_TypeDef *port
		.bitWeight           = DUMMY_BITWEIGHT,  // invalid
		.bit                 = -1,               // invalid
		.resetConfiguration  =
		{
			.GPIO_Mode   = GPIO_MODE_INPUT,         // GPIOMode_TypeDef GPIO_Mode
			.GPIO_OType  = GPIO_OTYPE_PP,        // GPIOSpeed_TypeDef GPIO_Speed
			.GPIO_Speed  = GPIO_OSPEED_50MHZ,     // GPIOOType_TypeDef GPIO_OType
			.GPIO_PuPd   = GPIO_PUPD_NONE      // GPIOPuPd_TypeDef GPIO_PuPd
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
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO6);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO7);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO8);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO9);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO10);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO10_A);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO10_B);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO11);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO11_A);
	HAL.IOs->config->reset(&HAL.IOs->pins->DIO11_B);
	HAL.IOs->config->reset(&HAL.IOs->pins->MUX_1);
	HAL.IOs->config->reset(&HAL.IOs->pins->MUX_2);
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
	HAL.IOs->config->reset(&HAL.IOs->pins->RS232_TX);
	HAL.IOs->config->reset(&HAL.IOs->pins->RS232_RX);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_BUS);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_DM);
	HAL.IOs->config->reset(&HAL.IOs->pins->USB_V_DP);
	HAL.IOs->config->reset(&HAL.IOs->pins->LED_STAT);
	HAL.IOs->config->reset(&HAL.IOs->pins->LED_ERROR);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXT0);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXT1);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXT2);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXT3);
	HAL.IOs->config->reset(&HAL.IOs->pins->EXT4);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_SCK);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_SI);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_SO);
	HAL.IOs->config->reset(&HAL.IOs->pins->EEPROM_NCS);
	HAL.IOs->config->reset(&HAL.IOs->pins->ADC_VM);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIFI_EN);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIFI_RST);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIFI_TX);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIFI_RX);
	HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
	gpio_af_set(HAL.IOs->pins->CLK16.port, GPIO_AF_0, HAL.IOs->pins->CLK16.bitWeight);
	// By default DIO10 and DIO11 are connected to DIO10_A and DIO11_A respectively.
	*HAL.IOs->pins->MUX_1.setBitRegister     = HAL.IOs->pins->MUX_1.bitWeight;
	*HAL.IOs->pins->MUX_2.setBitRegister     = HAL.IOs->pins->MUX_2.bitWeight;

}
