/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"

typedef struct
{

	IOPinTypeDef  *SLEEPN;
	IOPinTypeDef  *PWM_INT;
    IOPinTypeDef  *A0;
    IOPinTypeDef  *A1;
} PinsTypeDef;

static PinsTypeDef Pins;
static IICTypeDef *MAX22215_IIC;
static uint8_t slaveAddress = 0x10;

//static uint8_t max22215_getSlaveAddress(uint16_t icID)
//{
//    UNUSED(icID);
//
//    return slaveAddress;
//}
//
//static int32_t readRegisterIIC(uint16_t icID, uint8_t registerAddress)
//{
//    uint8_t data[8] = { 0 };
//
//    registerAddress = registerAddress & TMC5272_ADDRESS_MASK;
//
//    data[0] = 0x;
//    data[1] = tmc5272_getNodeAddress(icID);
//    data[2] = registerAddress;
//    data[3] = CRC8(data, 3);
//
//    if (!tmc5272_readWriteUART(icID, &data[0], 4, 8))
//        return 0;
//
//    // Byte 0: Sync nibble correct?
//    if (data[0] != 0x05)
//        return 0;
//
//    // Byte 1: Master address correct?
//    if (data[1] != 0xFF)
//        return 0;
//
//    // Byte 2: Address correct?
//    if (data[2] != registerAddress)
//        return 0;
//
//    // Byte 7: CRC correct?
//    if (data[7] != CRC8(data, 7))
//        return 0;
//
//    return ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | (data[5] << 8) | data[6];
//}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	uint32_t errors = TMC_ERROR_NONE;
    switch(type)
    {
    case 0:
        uint8_t data[1],regAddr=0x08;
        if(IICMasterWriteRead(0x20,&regAddr,1,&data[0],1)) // slave address b0010000WR
        {
            *value = (int32_t)data[0];
        }
        else
        {
            *value = -1;
        }
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
	return errors;

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

	MAX22215_IIC = HAL.IIC;

	//SPI.init();
	Evalboards.ch2.userFunction                  = userFunction;

	HAL.IOs->config->setLow(Pins.SLEEPN);

	//Setting the slave ID to 0x10
    HAL.IOs->config->setLow(Pins.A0);
    HAL.IOs->config->setLow(Pins.A1);



}
