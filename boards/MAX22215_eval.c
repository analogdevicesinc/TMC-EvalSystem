/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"
#include "../TMC-API/tmc/ic/MAX22215/MAX22215.h"

#define DEFAULT_ICID  0

typedef struct
{

	IOPinTypeDef  *SLEEPN;
	IOPinTypeDef  *PWM_INT;
    IOPinTypeDef  *A0;
    IOPinTypeDef  *A1;
} PinsTypeDef;

static PinsTypeDef Pins;
static MAX22215BusType activeBus = IC_BUS_IIC;
static IICTypeDef *MAX22215_IIC;
static uint8_t deviceAddress = 0x20;

static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);

bool max22215_readWriteIIC(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
   if(IICMasterWriteRead(data[0],&data[1],writeLength,&data[2],readLength))
       return true;

    return false;
}

MAX22215BusType max22215_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint8_t max22215_getDeviceAddress(uint16_t icID)
{
    UNUSED(icID);

    return deviceAddress;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	uint32_t errors = TMC_ERROR_NONE;
    switch(type)
    {
    case 0:
        uint8_t data[1],regAddr=0x08;
//        if(IICMasterWriteRead(0x20,&regAddr,1,&data[0],1)) // slave address b0010000WR
//        {
//            *value = (int32_t)data[0];
//        }
//        else
//        {
//            *value = -1;
//        }
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
	return errors;

}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    max22215_writeRegister(DEFAULT_ICID, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = max22215_readRegister(DEFAULT_ICID, address);
}

void MAX22215_init(void)
{
	Pins.SLEEPN          = &HAL.IOs->pins->DIO8;
	Pins.PWM_INT         = &HAL.IOs->pins->DIO9;
    Pins.A0              = &HAL.IOs->pins->DIO6;
    Pins.A1              = &HAL.IOs->pins->DIO7;


	HAL.IOs->config->toOutput(Pins.SLEEPN);
    HAL.IOs->config->toOutput(Pins.A0);
    HAL.IOs->config->toOutput(Pins.A1);

    IIC.init();
	MAX22215_IIC = HAL.IIC;

	Evalboards.ch2.userFunction                  = userFunction;
    Evalboards.ch2.writeRegister                 = writeRegister;
    Evalboards.ch2.readRegister                  = readRegister;

	HAL.IOs->config->setLow(Pins.SLEEPN);

	//Setting the slave ID to 0x10
    HAL.IOs->config->setLow(Pins.A0);
    HAL.IOs->config->setLow(Pins.A1);



}
