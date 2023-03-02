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
		.periphery       = SPI1,
		.CSN             = &IODummy,
		.readWrite       = spi_ch1_readWrite,
		.readWriteArray  = spi_ch1_readWriteArray,
		.reset           = reset_ch1
	},

	.ch2 =
	{
		.periphery       = SPI0,
		.CSN             = &IODummy,
		.readWrite       = spi_ch2_readWrite,
		.readWriteArray  = spi_ch2_readWriteArray,
		.reset           = reset_ch2
	},
	.init = init
};


static void init(void)
{
	rcu_periph_clock_enable(RCU_SPI1);
	rcu_periph_clock_enable(RCU_SPI0);

	// Config
	spi_parameter_struct params;

	params.device_mode = SPI_MASTER;
	params.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
	params.frame_size = SPI_FRAMESIZE_8BIT;
	params.nss = SPI_NSS_SOFT;
	params.endian = SPI_ENDIAN_MSB;
	params.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
//	params.prescale = SPI_PSC_8;
	params.prescale = SPI_PSC_16;

	spi_init(SPI.ch1.periphery, &params);

	params.prescale = SPI_PSC_32;

	spi_init(SPI.ch2.periphery, &params);

	// Enable
	spi_enable(SPI.ch1.periphery);
	spi_enable(SPI.ch2.periphery);

	// Set pin AFs

	gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_15);
	gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_14);
	gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_13);

	gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_7);
	gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_6);
	gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5);

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
	SPI.ch1.periphery  = SPI1;
	SPI.ch1.readWrite  = spi_ch1_readWrite;
}

static void reset_ch2()
{
	SPI.ch2.CSN        = &IODummy;
	SPI.ch2.periphery  = SPI0;
	SPI.ch2.readWrite  = spi_ch2_readWrite;
}

uint32_t spi_getFrequency(SPIChannelTypeDef *SPIChannel)
{
	return 0;
}

// Set the SPI frequency to the next-best available frequency (rounding down).
// Returns the actual frequency set or 0 if no suitable frequency was found.
uint32_t spi_setFrequency(SPIChannelTypeDef *SPIChannel, uint32_t desiredFrequency)
{
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

	while(spi_i2s_flag_get(SPIChannel->periphery, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPIChannel->periphery, data);
	while(spi_i2s_flag_get(SPIChannel->periphery, SPI_FLAG_RBNE) == RESET);
	if(lastTransfer)
		HAL.IOs->config->setHigh(SPIChannel->CSN);

	return spi_i2s_data_receive(SPIChannel->periphery);
}
