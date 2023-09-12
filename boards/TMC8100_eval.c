/*
 * TMC8462_eval.c
 *
 *  Created on: 07.09.2023
 *      Author: ASU
 */

#include "Board.h"

static SPIChannelTypeDef *TMC8100_SPIChannel;

typedef struct
{

	IOPinTypeDef  *SPI_DATA_AVAILABLE;
	IOPinTypeDef  *SDI;
	IOPinTypeDef  *SDO;
	IOPinTypeDef  *SCK;
	IOPinTypeDef  *CS;
} PinsTypeDef;

static PinsTypeDef Pins;
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);

void tmc8100_writeInt(int32_t value){

	spi_ch2_readWriteByte(TMC8100_SPIChannel, (0x07 & (value>>24)) | 0x08, false);
	spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>16), false);
	spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>8), false);
	spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>0), true);

}

int32_t tmc8100_readInt(int32_t address){

		spi_ch2_readWriteByte(TMC8100_SPIChannel, (0x07 & (address>>24)) | 0x88, false);
		spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (address>>16), false);
		spi_ch2_readWriteByte(TMC8100_SPIChannel, 0, false);
		spi_ch2_readWriteByte(TMC8100_SPIChannel, 0, true);

		while(HAL.IOs->config->getState(Pins.SPI_DATA_AVAILABLE) == 0);

		spi_ch2_readWriteByte(TMC8100_SPIChannel, (0x07 & (address>>24)) | 0x88, false);
		spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (address>>16), false);
		int32_t value = spi_ch2_readWriteByte(TMC8100_SPIChannel, 0, false);
		value <<= 8;
		value |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0, true);
	    return value;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;
	int32_t tempValue;

	switch(type)
	{
	case 0:
		if(readWrite == READ) {
			*value = tmc8100_readInt(*value);
		} else if(readWrite == WRITE) {
		tmc8100_writeInt(*value);
		}
		break;
//	case 1:
//		break;
	case 2:
		if(readWrite == READ) {
		*value = 	HAL.IOs->config->getState(Pins.SPI_DATA_AVAILABLE);
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
	return handleParameter(WRITE, motor, type, &value);
}
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
	return handleParameter(READ, motor, type, value);
}


void TMC8100_init(void)
{
	Pins.SPI_DATA_AVAILABLE    = &HAL.IOs->pins->DIO9;
	Pins.SCK             = &HAL.IOs->pins->SPI2_SCK; //Pin27
	Pins.SDI             = &HAL.IOs->pins->SPI2_SDI; //Pin29
	Pins.SDO             = &HAL.IOs->pins->SPI2_SDO; //Pin28
	Pins.CS              = &HAL.IOs->pins->SPI2_CSN2; //Pin26

	HAL.IOs->config->reset(Pins.SCK);
	HAL.IOs->config->reset(Pins.SDI);
	HAL.IOs->config->reset(Pins.SDO);
	HAL.IOs->config->reset(Pins.CS);
	HAL.IOs->config->reset(Pins.SPI_DATA_AVAILABLE);

	SPI.init();
	TMC8100_SPIChannel = &HAL.SPI->ch2;
	TMC8100_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN2;

	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.GAP                  = GAP;



}
