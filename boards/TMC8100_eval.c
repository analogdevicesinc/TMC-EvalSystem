/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"

static SPIChannelTypeDef *TMC8100_SPIChannel;

typedef struct
{
    IOPinTypeDef  *SPI_DATA_AVAILABLE;
    IOPinTypeDef  *SDI;
    IOPinTypeDef  *SDO;
    IOPinTypeDef  *SCK;
    IOPinTypeDef  *CS;
    IOPinTypeDef  *RESETN;
} PinsTypeDef;

static PinsTypeDef Pins;

void tmc8100_writeInt(int32_t value)
{
    spi_ch2_readWriteByte(TMC8100_SPIChannel, value>>24, false);
    spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>16), false);
    spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>8), false);
    spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>0), true);
}

int32_t tmc8100_readInt(int32_t address)
{
    spi_ch2_readWriteByte(TMC8100_SPIChannel, address>>24, false);
    spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (address>>16), false);
    spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (address>>8), false);
    spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & address, true);

    while(HAL.IOs->config->getState(Pins.SPI_DATA_AVAILABLE) == 0);

    int32_t value = spi_ch2_readWriteByte(TMC8100_SPIChannel, (address>>24), false);
    value <<= 8;
    value |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (address>>16), false);
    value <<= 8;
    value |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (address>>8), false);
    value <<= 8;
    value |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & address, true);
    return value;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    uint32_t errors = TMC_ERROR_NONE;

    switch(type)
    {
    case 0:
        //writing
        tmc8100_writeInt(*value);
        break;
    case 1:
        //reading
        *value = tmc8100_readInt(*value);
        break;
    case 2:
        // check GPIO6 state
        *value = 	HAL.IOs->config->getState(Pins.SPI_DATA_AVAILABLE);
        break;
    case 3:
        // Set to Low
        HAL.IOs->config->setToState(Pins.RESETN, IOS_LOW);
        break;
    case 4:
        // Set to High
        HAL.IOs->config->setToState(Pins.RESETN, IOS_HIGH);
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

void TMC8100_init(void)
{
    Pins.SPI_DATA_AVAILABLE    = &HAL.IOs->pins->DIO9;
    Pins.SCK                   = &HAL.IOs->pins->SPI2_SCK; //Pin27
    Pins.SDI                   = &HAL.IOs->pins->SPI2_SDI; //Pin29
    Pins.SDO                   = &HAL.IOs->pins->SPI2_SDO; //Pin28
    Pins.CS                    = &HAL.IOs->pins->SPI2_CSN2; //Pin26
    Pins.RESETN                = &HAL.IOs->pins->DIO8; //Pin19

    HAL.IOs->config->reset(Pins.SCK);
    HAL.IOs->config->reset(Pins.SDI);
    HAL.IOs->config->reset(Pins.SDO);
    HAL.IOs->config->reset(Pins.CS);
    HAL.IOs->config->reset(Pins.RESETN);
    HAL.IOs->config->reset(Pins.SPI_DATA_AVAILABLE);

    SPI.init();
    TMC8100_SPIChannel = &HAL.SPI->ch2;
    TMC8100_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN2;

    Evalboards.ch2.userFunction    = userFunction;
}
