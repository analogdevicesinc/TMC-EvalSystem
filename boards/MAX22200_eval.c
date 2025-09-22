/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"
#include <tmc/ic/MAX22200/MAX22200.h>

#define DEFAULT_ICID 0
#define MOTORS       8

#define VM_MIN 45  // VM[V/10] min
#define VM_MAX 360 // VM[V/10] max

static SPIChannelTypeDef *MAX22200_SPIChannel;
static uint8_t RREF = 60;

static void readRegister(uint8_t icID, uint16_t address, int32_t *value);
static void writeRegister(uint8_t icID, uint16_t address, int32_t value);
static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static void enableDriver(DriverState state);
static uint8_t restore(void);

typedef struct
{
    IOPinTypeDef *CMD;
    IOPinTypeDef *EN;
    IOPinTypeDef *FAULT_N;
    IOPinTypeDef *TRIGA;
    IOPinTypeDef *TRIGB;
    IOPinTypeDef *IREF_CFG1;
    IOPinTypeDef *IREF_CFG2;
    IOPinTypeDef *IREF_CFG3;
} PinsTypeDef;
static PinsTypeDef Pins;

void max22200_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength, bool cmd_mode)
{
    UNUSED(icID);
    if (cmd_mode == true)
        HAL.IOs->config->setHigh(Pins.CMD);

    MAX22200_SPIChannel->readWriteArray(data, dataLength);
    HAL.IOs->config->setLow(Pins.CMD);
}

static void writeRegister(uint8_t icID, uint16_t address, int32_t value)
{
    UNUSED(icID);
    max22200_writeRegister(DEFAULT_ICID, address, value);
}

static void readRegister(uint8_t icID, uint16_t address, int32_t *value)
{
    UNUSED(icID);
    *value = max22200_readRegister(DEFAULT_ICID, address);
}

static uint8_t reset()
{
    return 0;
}

static uint8_t restore()
{
    return 0;
}

static void enableDriver(DriverState state)
{
    UNUSED(state);
}

static void periodicJob(uint32_t tick)
{
    UNUSED(tick);
}

static void deInit(void)
{
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    uint32_t errors = 0;

    UNUSED(motor);

    switch(type)
    {
        case 0:  // IREF
            if(*value == 0x0)
            {
                HAL.IOs->config->setLow(Pins.IREF_CFG1);
                HAL.IOs->config->setLow(Pins.IREF_CFG2);
                HAL.IOs->config->setLow(Pins.IREF_CFG3);
                RREF = 60; // 60Kohm resistor (Refer to schematics)
            }else if(*value == 0x1)
            {
                HAL.IOs->config->setLow(Pins.IREF_CFG1);
                HAL.IOs->config->setLow(Pins.IREF_CFG2);
                HAL.IOs->config->setHigh(Pins.IREF_CFG3);
                RREF = 50; // 50Kohm resistor (Refer to schematics)
            }else if(*value == 0x2)
            {
                HAL.IOs->config->setLow(Pins.IREF_CFG1);
                HAL.IOs->config->setHigh(Pins.IREF_CFG2);
                HAL.IOs->config->setLow(Pins.IREF_CFG3);
                RREF = 30; // 30Kohm resistor (Refer to schematics)
            }else if(*value == 0x3)
            {
                HAL.IOs->config->setHigh(Pins.IREF_CFG1);
                HAL.IOs->config->setLow(Pins.IREF_CFG2);
                HAL.IOs->config->setLow(Pins.IREF_CFG3);
                RREF = 15; // 15Kohm resistor (Refer to schematics)
            }
            break;
        case 1: // RREF
            *value = RREF;
            break;


        default:
            errors |= TMC_ERROR_TYPE;
            break;
    }
    return errors;
}


void MAX22200_init(void)
{
    Pins.CMD       = &HAL.IOs->pins->DIO14;
    Pins.EN        = &HAL.IOs->pins->DIO0;
    Pins.FAULT_N   = &HAL.IOs->pins->DIO1;
    Pins.TRIGA     = &HAL.IOs->pins->DIO10;
    Pins.TRIGB     = &HAL.IOs->pins->DIO8;
    Pins.IREF_CFG1 = &HAL.IOs->pins->DIO13;
    Pins.IREF_CFG2 = &HAL.IOs->pins->DIO12;
    Pins.IREF_CFG3 = &HAL.IOs->pins->SPI1_SDI;

    MAX22200_SPIChannel      = &HAL.SPI->ch2;
    MAX22200_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

    spi_setFrequency(MAX22200_SPIChannel, 1000000); // 1MHz

    HAL.IOs->config->toOutput(Pins.CMD);
    HAL.IOs->config->toOutput(Pins.EN);
    HAL.IOs->config->toInput(Pins.FAULT_N);

    HAL.IOs->config->setHigh(Pins.EN);
    // Set RREF to 60k Ohm (Refer to the schematic)
    HAL.IOs->config->setLow(Pins.IREF_CFG1);
    HAL.IOs->config->setLow(Pins.IREF_CFG2);
    HAL.IOs->config->setLow(Pins.IREF_CFG3);

    Evalboards.ch2.config->reset       = reset;
    Evalboards.ch2.config->restore     = restore;
    Evalboards.ch2.config->state       = CONFIG_READY;
    Evalboards.ch2.config->configIndex = 0;

    Evalboards.ch2.writeRegister = writeRegister;
    Evalboards.ch2.readRegister  = readRegister;

    Evalboards.ch2.enableDriver = enableDriver;

    Evalboards.ch2.numberOfMotors = MOTORS;
    Evalboards.ch2.VMMin          = VM_MIN;
    Evalboards.ch2.VMMax          = VM_MAX;
    Evalboards.ch2.deInit         = deInit;
    Evalboards.ch2.periodicJob    = periodicJob;
    Evalboards.ch2.userFunction   = userFunction;

    max22200_writeRegister(DEFAULT_ICID, 0x00, 0x01); // Active = 1
    max22200_readRegister(DEFAULT_ICID, 0x00);        // Read status register to clear UVM flag

    Timer.init();
    wait(100);
}
