#include "../../HAL.h"
#include "../../SPI.h"

static void init(void);
static void reset_ch1();
static void reset_ch2();

static unsigned char readWrite(SPIChannelTypeDef *SPIChannel, uint8 data, uint8 lastTransfer);
static unsigned char spi_ch1_readWrite(uint8 data, uint8 lastTransfer);
static unsigned char spi_ch2_readWrite(uint8 data, uint8 lastTransfer);
static void spi_ch1_readWriteArray(uint8 *data, size_t length);
static void spi_ch2_readWriteArray(uint8 *data, size_t length);

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

int32 spi_readInt(SPIChannelTypeDef *SPIChannel, uint8 address)
{
	// clear write bit
	address &= 0x7F;

	SPIChannel->readWrite(address, FALSE);
	int value = SPIChannel->readWrite(0, FALSE);
	value <<= 8;
	value |= SPIChannel->readWrite(0, FALSE);
	value <<= 8;
	value |= SPIChannel->readWrite(0, FALSE);
	value <<= 8;
	value |= SPIChannel->readWrite(0, TRUE);

	return value;
}

int32 spi_ch1_readInt(uint8 address)
{
	return spi_readInt(SPIChannel_1_default, address);
}

int32 spi_ch2_readInt(uint8 address)
{
	return spi_readInt(SPIChannel_2_default, address);
}

void spi_writeInt(SPIChannelTypeDef *SPIChannel, uint8 address, int value)
{
	SPIChannel->readWrite(address | 0x80, FALSE);
	SPIChannel->readWrite(0xFF & (value>>24), FALSE);
	SPIChannel->readWrite(0xFF & (value>>16), FALSE);
	SPIChannel->readWrite(0xFF & (value>>8), FALSE);
	SPIChannel->readWrite(0xFF & (value>>0), TRUE);
}

void spi_ch1_writeInt(uint8 address, int value)
{
	spi_writeInt(SPIChannel_1_default, address, value);
}

void spi_ch2_writeInt(uint8 address, int value)
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

static void spi_ch1_readWriteArray(uint8 *data, size_t length)
{
	for(uint32 i = 0; i < length; i++)
	{
		data[i] = readWrite(&SPI.ch1, data[i], (i == (length - 1))? TRUE:FALSE);
	}
}

static void spi_ch2_readWriteArray(uint8 *data, size_t length)
{
	for(uint32 i = 0; i < length; i++)
	{
		data[i] = readWrite(&SPI.ch2, data[i], (i == (length - 1))? TRUE:FALSE);
	}
}

uint8 spi_ch1_readWriteByte(uint8 data, uint8 lastTransfer)
{
	return readWrite(SPIChannel_1_default, data, lastTransfer);
}

static unsigned char readWrite(SPIChannelTypeDef *SPIChannel, uint8 data, uint8 lastTransfer)
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
