/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5220/TMC5220.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

static TMC5220BusType activeBus = IC_BUS_SPI;
static SPIChannelTypeDef *TMC5220_SPIChannel;

#define DEFAULT_MOTOR  0

void tmc5220_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
	UNUSED(icID);
	TMC5220_SPIChannel->readWriteArray(data, dataLength);
}

TMC5220BusType tmc5220_getBusType(uint16_t icID)
{
	UNUSED(icID);

	return activeBus;
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
	UNUSED(motor);
	tmc5220_writeRegister(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
	UNUSED(motor);

	*value = tmc5220_readRegister(DEFAULT_MOTOR, address);
}

void TMC5220_init(void)
{

    SPI.init();
    TMC5220_SPIChannel = &HAL.SPI->ch1;
    TMC5220_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	Evalboards.ch1.writeRegister        = writeRegister;
	Evalboards.ch1.readRegister         = readRegister;
    Evalboards.ch1.VMMin                = VM_MIN;
    Evalboards.ch1.VMMax                = VM_MAX;

};
