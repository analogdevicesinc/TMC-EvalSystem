#include "hal/HAL.h"
#include "hal/SPI.h"

static void init(void);
static void reset_ch1();
static void reset_ch2();

static unsigned char readWrite(SPIChannelTypeDef *SPIChannel, uint8_t data, uint8_t lastTransfer);
static unsigned char spi_ch1_readWrite(uint8_t data, uint8_t lastTransfer);
static unsigned char spi_ch2_readWrite(uint8_t data, uint8_t lastTransfer);
static void spi_ch1_readWriteArray(uint8_t *data, size_t length);
static void spi_ch2_readWriteArray(uint8_t *data, size_t length);

SPIChannelTypeDef *SPIChannel_1_default;
SPIChannelTypeDef *SPIChannel_2_default;

static IOPinTypeDef IODummy = { .bitWeight = DUMMY_BITWEIGHT };

SPITypeDef SPI=
{
	.ch1 =
	{
		.periphery       = SPI3,
		.CSN             = &IODummy,
		.readWrite       = spi_ch1_readWrite,
		.readWriteArray  = spi_ch1_readWriteArray,
		.reset           = reset_ch1
	},

	.ch2 =
	{
		.periphery       = SPI2,
		.CSN             = &IODummy,
		.readWrite       = spi_ch2_readWrite,
		.readWriteArray  = spi_ch2_readWriteArray,
		.reset           = reset_ch2
	},
	.init = init
};


static void init(void)
{
	SPI_InitTypeDef SPIInit;

	//-------- SPI2 initialisieren und mit den Pins verbinden ---------
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	//SPI initialisieren
	SPIInit.SPI_Direction          = SPI_Direction_2Lines_FullDuplex;
	SPIInit.SPI_Mode               = SPI_Mode_Master;
	SPIInit.SPI_DataSize           = SPI_DataSize_8b;
	SPIInit.SPI_CPOL               = SPI_CPOL_High;
	SPIInit.SPI_CPHA               = SPI_CPHA_2Edge;
	SPIInit.SPI_NSS                = SPI_NSS_Soft;
	SPIInit.SPI_BaudRatePrescaler  = SPI_BaudRatePrescaler_8;
	SPIInit.SPI_FirstBit           = SPI_FirstBit_MSB;
	SPIInit.SPI_CRCPolynomial      = 0;
	SPI_Init(SPI2, &SPIInit);
	SPI_Cmd(SPI2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	//-------- SPI3 initialisieren und mit den Pins verbinden ---------
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	//SPI initialisieren
	SPIInit.SPI_Direction          = SPI_Direction_2Lines_FullDuplex;
	SPIInit.SPI_Mode               = SPI_Mode_Master;
	SPIInit.SPI_DataSize           = SPI_DataSize_8b;
	SPIInit.SPI_CPOL               = SPI_CPOL_High;
	SPIInit.SPI_CPHA               = SPI_CPHA_2Edge;
	SPIInit.SPI_NSS                = SPI_NSS_Soft;
	SPIInit.SPI_BaudRatePrescaler  = SPI_BaudRatePrescaler_32;
	SPIInit.SPI_FirstBit           = SPI_FirstBit_MSB;
	SPIInit.SPI_CRCPolynomial      = 0;
	SPI_Init(SPI3, &SPIInit);
	SPI_Cmd(SPI3, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	reset_ch1();
	reset_ch2();

	// configure default SPI channel_1
	SPIChannel_1_default = &HAL.SPI->ch1;
	SPIChannel_1_default->CSN = &HAL.IOs->pins->SPI1_CSN;
	// configure default SPI channel_2
	SPIChannel_2_default = &HAL.SPI->ch2;
	SPIChannel_2_default->CSN = &HAL.IOs->pins->SPI2_CSN0;
}

static void reset_ch1()
{
	SPI.ch1.CSN        = &IODummy;
	SPI.ch1.periphery  = SPI3;
	SPI.ch1.readWrite  = spi_ch1_readWrite;
}

static void reset_ch2()
{
	SPI.ch2.CSN        = &IODummy;
	SPI.ch2.periphery  = SPI2;
	SPI.ch2.readWrite  = spi_ch2_readWrite;
}

uint32_t spi_getFrequency(SPIChannelTypeDef *SPIChannel)
{
	// Calculate the shift value of the CR1->BD bitfield since it is
	// not provided as a macro.
	uint32_t br_shift = 0;
	for (uint32_t j = SPI_CR1_BR; (j & 1) == 0; j >>=1)
	{
		br_shift++;
	}

	RCC_ClocksTypeDef RCC_ClocksStatus;

	RCC_GetClocksFreq(&RCC_ClocksStatus);

	uint8_t br = FIELD_GET(SPIChannel->periphery->CR1, SPI_CR1_BR, br_shift);

	return RCC_ClocksStatus.PCLK1_Frequency >> (br+1);
}

// Set the SPI frequency to the next-best available frequency (rounding down).
// Returns the actual frequency set or 0 if no suitable frequency was found.
uint32_t spi_setFrequency(SPIChannelTypeDef *SPIChannel, uint32_t desiredFrequency)
{
	RCC_ClocksTypeDef RCC_ClocksStatus;

	RCC_GetClocksFreq(&RCC_ClocksStatus);

	for (int i = 0; i < 8; i++)
	{
		uint32_t prescaler = 1<<(i+1);
		uint32_t frequency = RCC_ClocksStatus.PCLK1_Frequency / prescaler;

		if (frequency <= desiredFrequency)
		{
			// Calculate the shift value of the CR1->BD bitfield since it is
			// not provided as a macro.
			uint32_t shift = 0;
			for (uint32_t j = SPI_CR1_BR; (j & 1) == 0; j >>=1)
			{
				shift++;
			}

			// Update the prescaler
			uint32_t tmp = SPIChannel->periphery->CR1;

			tmp &= ~SPI_CR1_BR;

			tmp |= i << shift;

			SPIChannel->periphery->CR1 = tmp;

			return frequency;
		}
	}

	// The requested frequency was too small -> do not update the frequency
	return 0;
}

int32_t spi_readInt(SPIChannelTypeDef *SPIChannel, uint8_t address)
{
	// clear write bit
	address &= 0x7F;

	SPIChannel->readWrite(address, false);
	int value = SPIChannel->readWrite(0, false);
	value <<= 8;
	value |= SPIChannel->readWrite(0, false);
	value <<= 8;
	value |= SPIChannel->readWrite(0, false);
	value <<= 8;
	value |= SPIChannel->readWrite(0, true);

	return value;
}

int32_t spi_ch1_readInt(uint8_t address)
{
	return spi_readInt(SPIChannel_1_default, address);
}

int32_t spi_ch2_readInt(uint8_t address)
{
	return spi_readInt(SPIChannel_2_default, address);
}

void spi_writeInt(SPIChannelTypeDef *SPIChannel, uint8_t address, int value)
{
	SPIChannel->readWrite(address | 0x80, false);
	SPIChannel->readWrite(0xFF & (value>>24), false);
	SPIChannel->readWrite(0xFF & (value>>16), false);
	SPIChannel->readWrite(0xFF & (value>>8), false);
	SPIChannel->readWrite(0xFF & (value>>0), true);
}

void spi_ch1_writeInt(uint8_t address, int value)
{
	spi_writeInt(SPIChannel_1_default, address, value);
}

void spi_ch2_writeInt(uint8_t address, int value)
{
	spi_writeInt(SPIChannel_2_default, address, value);
}

static unsigned char spi_ch1_readWrite(unsigned char data, unsigned char lastTransfer)
{
	 return readWrite(&SPI.ch1, data, lastTransfer);
}

static unsigned char spi_ch2_readWrite(unsigned char data, unsigned char lastTransfer)
{
	 return readWrite(&SPI.ch2, data,lastTransfer);
}

static void spi_ch1_readWriteArray(uint8_t *data, size_t length)
{
	for(uint32_t i = 0; i < length; i++)
	{
		data[i] = readWrite(&SPI.ch1, data[i], (i == (length - 1))? true:false);
	}
}

static void spi_ch2_readWriteArray(uint8_t *data, size_t length)
{
	for(uint32_t i = 0; i < length; i++)
	{
		data[i] = readWrite(&SPI.ch2, data[i], (i == (length - 1))? true:false);
	}
}

uint8_t spi_ch1_readWriteByte(uint8_t data, uint8_t lastTransfer)
{
	return readWrite(SPIChannel_1_default, data, lastTransfer);
}

static unsigned char readWrite(SPIChannelTypeDef *SPIChannel, uint8_t data, uint8_t lastTransfer)
{
	if(IS_DUMMY_PIN(SPIChannel->CSN))
		return 0;

	HAL.IOs->config->setLow(SPIChannel->CSN);

	while(SPI_I2S_GetFlagStatus(SPIChannel->periphery, SPI_I2S_FLAG_TXE) == RESET) {};
	SPI_I2S_SendData(SPIChannel->periphery, data);
	while(SPI_I2S_GetFlagStatus(SPIChannel->periphery, SPI_I2S_FLAG_RXNE) == RESET) {};
	if(lastTransfer)
		HAL.IOs->config->setHigh(SPIChannel->CSN);

	return SPI_I2S_ReceiveData(SPIChannel->periphery);
}
