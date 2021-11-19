#include "hal/HAL.h"
#include "hal/RS232.h"

void init();
void reset_ch1();
void reset_ch2();

static void setTMCSPIParameters(SPI_MemMapPtr basePtr);

static uint8_t readWrite(SPIChannelTypeDef *SPIChannel, uint8_t data, uint8_t lastTransfer);
static uint8_t spi_ch1_readWrite(uint8_t data, uint8_t lastTransfer);
static uint8_t spi_ch2_readWrite(uint8_t data, uint8_t lastTransfer);
static void spi_ch1_readWriteArray(uint8_t *data, size_t length);
static void spi_ch2_readWriteArray(uint8_t *data, size_t length);

SPIChannelTypeDef *SPIChannel_1_default;
SPIChannelTypeDef *SPIChannel_2_default;

static IOPinTypeDef IODummy = { .bitWeight = DUMMY_BITWEIGHT };

SPITypeDef SPI=
{
	.ch1 =
	{
		.periphery       = SPI1_BASE_PTR,
		.CSN             = &IODummy,
		.readWrite       = spi_ch1_readWrite,
		.readWriteArray  = spi_ch1_readWriteArray,
		.reset           = reset_ch1
	},
	.ch2 =
	{
		.periphery       = SPI2_BASE_PTR,
		.CSN             = &IODummy,
		.readWrite       = spi_ch2_readWrite,
		.readWriteArray  = spi_ch2_readWriteArray,
		.reset           = reset_ch2
	},
	.init = init
};


void init()
{
	// SPI0 -> EEPROM
	// -------------------------------------------------------------------------------
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;  // enable clock for PORT C
	SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK;   // enable clock for SPI0
	SIM_SCGC6 &= ~SIM_SCGC6_CRC_MASK;   // disable clock for CRC module

	// configure SPI0 pins:
	//     SCK: Port C, Pin 5
	//     SDI: Port C, Pin 6
	//     SDO: Port C, Pin 7
	//     CSN: Port C, Pin 8
	HAL.IOs->pins->EEPROM_SCK.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->EEPROM_SI.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->EEPROM_SO.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->EEPROM_NCS.configuration.GPIO_Mode = GPIO_Mode_OUT;

	HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_SCK);
	HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_SI);
	HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_SO);
	HAL.IOs->config->set(&HAL.IOs->pins->EEPROM_NCS);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->EEPROM_NCS);

	setTMCSPIParameters(SPI0_BASE_PTR);

	// SPI1 -> ch1
	// -------------------------------------------------------------------------------
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;  // enable clock for PORT B
	SIM_SCGC6 |= SIM_SCGC6_SPI1_MASK;   // enable clock for SPI1
	SIM_SCGC6 &= ~SIM_SCGC6_CRC_MASK;   // disable clock for CRC module

	// configure SPI1 pins:
	//     SCK: Port B, Pin 11
	//     SDI: Port B, Pin 16
	//     SDO: Port B, Pin 17
	//     CSN: Port B, Pin 10
	HAL.IOs->pins->SPI1_SCK.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->SPI1_SDI.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->SPI1_SDO.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->SPI1_CSN.configuration.GPIO_Mode = GPIO_Mode_OUT;

	HAL.IOs->config->set(&HAL.IOs->pins->SPI1_SCK);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI1_SDI);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI1_SDO);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI1_CSN);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI1_CSN);

	setTMCSPIParameters(SPI1_BASE_PTR);

	// SPI2 -> ch2
	// -------------------------------------------------------------------------------
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;  // enable clocks
	SIM_SCGC3 |= SIM_SCGC3_SPI2_MASK;                                                 // enable clock for SPI2
	SIM_SCGC6 &= ~SIM_SCGC6_CRC_MASK;                                                 // disable clock for CRC module

	// configure SPI2 pins:
	//     SCK:  Port B, Pin 21
	//     SDI:  Port B, Pin 22
	//     SDO:  Port B, Pin 23
	//     CSN0: Port C, Pin 0
	//     CSN1: Port A, Pin 5
	//     CSN2: Port C, Pin 4
	HAL.IOs->pins->SPI2_SCK.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->SPI2_SDI.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->SPI2_SDO.configuration.GPIO_Mode = GPIO_Mode_AF2;
	HAL.IOs->pins->SPI2_CSN0.configuration.GPIO_Mode = GPIO_Mode_OUT;
	HAL.IOs->pins->SPI2_CSN1.configuration.GPIO_Mode = GPIO_Mode_OUT;
	HAL.IOs->pins->SPI2_CSN2.configuration.GPIO_Mode = GPIO_Mode_OUT;

	HAL.IOs->config->set(&HAL.IOs->pins->SPI2_SCK);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI2_SDI);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI2_SDO);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI2_CSN0);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI2_CSN1);
	HAL.IOs->config->set(&HAL.IOs->pins->SPI2_CSN2);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI2_CSN0);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI2_CSN1);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->SPI2_CSN2);

	setTMCSPIParameters(SPI2_BASE_PTR);

	// configure default SPI channel_1
	SPIChannel_1_default = &HAL.SPI->ch1;
	SPIChannel_1_default->CSN = &HAL.IOs->pins->SPI1_CSN;
	// configure default SPI channel_2
	SPIChannel_2_default = &HAL.SPI->ch2;
	SPIChannel_2_default->CSN = &HAL.IOs->pins->SPI2_CSN0;
}

void setTMCSPIParameters(SPI_MemMapPtr basePtr)
{
	// set SPI2 to STOP mode
	SPI_MCR_REG(basePtr) = SPI_MCR_HALT_MASK;

	// set SPI2 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
	SPI_MCR_REG(basePtr) |= SPI_MCR_MSTR_MASK
	                     |  SPI_MCR_CLR_RXF_MASK
	                     |  SPI_MCR_CLR_TXF_MASK;

	// baudrate => 48MHz/18 = 2.66MHz
	SPI_CTAR_REG(basePtr, 0) = SPI_CTAR_FMSZ(7)     // duty cycle 50/50; 8bit frame(7+1); inactive SCK state low; capture->leading; MSB first;
	                         | SPI_CTAR_DT(2)       // CS a SCK = 21ns[2ns]; after SCK = 21ns [2ns]; After transfer*8= 168ns [50ns]
	                         | SPI_CTAR_BR(2)       // prescaler value: 6 (BR=2, PBR=1 => 2,66MHz) (BR=1, PBR=1 => 4MHz) (BR=0, PBR=1 => ?MHz)
	                         | SPI_CTAR_PBR(1)      // prescaler value: 3
	                         | SPI_CTAR_CPHA_MASK   // clock phase
	                         | SPI_CTAR_PCSSCK(1)   // shift NCS->SCK
	                         | SPI_CTAR_PASC(1)     // shift NCS->SCK
	                         | SPI_CTAR_CPOL_MASK;  // clk high in idle

	// set SPI2 to RUNNING mode
	SPI_SR_REG(basePtr) |= SPI_SR_EOQF_MASK;
	SPI_MCR_REG(basePtr) &= ~SPI_MCR_HALT_MASK;
	SPI_MCR_REG(basePtr) &= ~SPI_MCR_FRZ_MASK;
}

void reset_ch1()
{
	// configure SPI1 pins PORTB_PCR11(SCK), PORTB_PCR17(SDI), PORTB_PCR15(SDO), PORTB_PCR10(CSN)
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SCK);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDI);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI1_SDO);
	HAL.IOs->config->reset(SPI.ch1.CSN);

	// set SPI0 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
	SPI_MCR_REG(SPI.ch1.periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
}

void reset_ch2()
{
//	// configure SPI2 pins PORTB_PCR21(SCK), PORTB_PCR23(SDI), PORTB_PCR22(SDO), PORTC_PCR0(CSN0), PORTA_PCR0(CSN5), PORTA_PCR4(CSN2)
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SCK);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDI);
	HAL.IOs->config->reset(&HAL.IOs->pins->SPI2_SDO);
	HAL.IOs->config->reset(SPI.ch2.CSN);
	SPI.ch2.readWrite = spi_ch2_readWrite;

	// set SPI0 to master mode, set inactive state of chip select to HIGH, flush both FIFO buffer by clearing their counters (Tx FIFO, and Rx FIFO are enabled)
	SPI_MCR_REG(SPI.ch2.periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
}

// Helper lookup tables
static uint8_t PBR_values[4] = { 2, 3, 5, 7 };
static uint16_t BR_values[16] = { 2, 4, 6, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768 };

uint32_t spi_getFrequency(SPIChannelTypeDef *SPIChannel)
{
	uint32_t tmp = SPI_CTAR_REG(SPIChannel->periphery, 0);
	uint8_t pbr  = PBR_values[FIELD_GET(tmp, SPI_CTAR_PBR_MASK, SPI_CTAR_PBR_SHIFT)];
	uint8_t br   = BR_values[FIELD_GET(tmp, SPI_CTAR_BR_MASK, SPI_CTAR_BR_SHIFT)];

	return CPU_BUS_CLK_HZ / pbr / br;
}

// Set the SPI frequency to the next-best available frequency (rounding down).
// Returns the actual frequency set or 0 if no suitable frequency was found.
uint32_t spi_setFrequency(SPIChannelTypeDef *SPIChannel, uint32_t desiredFrequency)
{
	if (!SPIChannel)
		return 0;

	uint32_t bestFrequency = 0;
	uint8_t bestPBR = 0;
	uint8_t bestBR  = 0;

	// Find the highest frequency that is lower or equal to the desired frequency
	for (uint32_t i = 0; i < ARRAY_SIZE(PBR_values); i++)
	{
		// For each possible PBR prescaler value...
		uint8_t pbr = PBR_values[i];
		uint32_t pbrFreq = CPU_BUS_CLK_HZ / pbr;

		// Calculate the ideal divisor (rounding up to not exceed the desired frequency)
		uint32_t divisor = (pbrFreq + desiredFrequency - 1) / desiredFrequency;

		// Find the next-largest available BR divisor
		uint32_t j;
		for (j = 0; j < ARRAY_SIZE(BR_values); j++)
		{
			if (BR_values[j] >= divisor)
				break;
		}

		// If no BR divisor value is larger, check the next PBR value
		if (j == ARRAY_SIZE(BR_values))
			continue;

		// Calculate the resulting actual frequency
		uint32_t actualFrequency = pbrFreq / BR_values[j];

		// Sanity check: Verify that the actual frequency is smaller than the desired frequency
		if (actualFrequency > desiredFrequency)
			continue;

		// Check if the actual frequency is better than the previous ones
		// and store the parameters if it is.
		if (actualFrequency > bestFrequency)
		{
			bestFrequency = actualFrequency;
			bestPBR = i;
			bestBR  = j;
		}
	}

	// If we couldn't find a suitable frequency, abort
	if (bestFrequency == 0)
		return 0;

	// Update the hardware parameters
	uint32_t tmp = SPI_CTAR_REG(SPIChannel->periphery, 0) & ~(SPI_CTAR_DT_MASK | SPI_CTAR_BR_MASK | SPI_CTAR_PBR_MASK);
	tmp |= SPI_CTAR_DT(0);
	tmp |= SPI_CTAR_PBR(bestPBR);
	tmp |= SPI_CTAR_BR(bestBR);
	SPI_CTAR_REG(SPIChannel->periphery, 0) = tmp;

	return bestFrequency;
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
	SPIChannel->readWrite(address|0x80, false);
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

uint8_t spi_ch1_readWrite(uint8_t data, uint8_t lastTransfer)
{
	return readWrite(&SPI.ch1, data, lastTransfer);
}

uint8_t spi_ch2_readWrite(uint8_t data, uint8_t lastTransfer)
{
	return readWrite(&SPI.ch2, data, lastTransfer);
}

static void spi_ch1_readWriteArray(uint8_t *data, size_t length)
{
	for(size_t i = 0; i < length; i++)
	{
		data[i] = readWrite(&SPI.ch1, data[i], (i == (length - 1))? true:false);
	}
}

static void spi_ch2_readWriteArray(uint8_t *data, size_t length)
{
	for(size_t i = 0; i < length; i++)
	{
		data[i] = readWrite(&SPI.ch2, data[i], (i == (length - 1))? true:false);
	}
}

uint8_t spi_ch1_readWriteByte(uint8_t data, uint8_t lastTransfer)
{
	return readWrite(SPIChannel_1_default, data, lastTransfer);
}

uint8_t readWrite(SPIChannelTypeDef *SPIChannel, uint8_t writeData, uint8_t lastTransfer)
{
	uint8_t readData = 0;

	if(IS_DUMMY_PIN(SPIChannel->CSN))
		return 0;

	HAL.IOs->config->setLow(SPIChannel->CSN); // Chip Select

	if(lastTransfer)
	{
		// send last byte
		SPI_PUSHR_REG(SPIChannel->periphery) = SPI_PUSHR_EOQ_MASK | SPI_PUSHR_TXDATA(writeData) ;

		while(!(SPI_SR_REG(SPIChannel->periphery) & SPI_SR_EOQF_MASK)) {} // wait until End Of Queue flag has been set -> transfer done

		SPI_SR_REG(SPIChannel->periphery) |= SPI_SR_EOQF_MASK;   // clear EOQ Flag by writing a 1 to EOQF

		HAL.IOs->config->setHigh(SPIChannel->CSN); // reset CSN manual, falls Probleme Auftreten, dann diese Zeile unter die while Schleife

		// wait for an answer
		while(((SPI_SR_REG(SPIChannel->periphery) & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT) == 0) {}

		// read the data
		readData = SPI_POPR_REG(SPIChannel->periphery);

		// clear TXF and RXF
		SPI_MCR_REG(SPIChannel->periphery) |= SPI_MCR_CLR_RXF_MASK | SPI_MCR_CLR_TXF_MASK;
	} else {
		// continuous transfer
		SPI_PUSHR_REG(SPIChannel->periphery) = SPI_PUSHR_CONT_MASK | SPI_PUSHR_TXDATA(writeData); // | SPI_PUSHR_PCS(0x0);

		while(((SPI_SR_REG(SPIChannel->periphery) & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT) > 3) {} // wait if TX counter > 3

		// wait for an answer
		while(((SPI_SR_REG(SPIChannel->periphery) & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT) == 0) {}

		// read the data
		readData = SPI_POPR_REG(SPIChannel->periphery);
	}

	return readData;
}
