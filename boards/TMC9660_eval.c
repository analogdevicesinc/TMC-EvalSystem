/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"

static SPIChannelTypeDef *TMC9660_SPIChannel;
static UART_Config *TMC9660_UARTChannel;

typedef struct
{
    IOPinTypeDef  *SPI_EN;
    IOPinTypeDef  *SPI1_MOSI;
    IOPinTypeDef  *SPI1_MISO;
    IOPinTypeDef  *SPI1_SCK;
    IOPinTypeDef  *SPI1_CSN;
    IOPinTypeDef  *HOLD_FLASHN;
    IOPinTypeDef  *I2C_EN;
    IOPinTypeDef  *I2C_SDA;
    IOPinTypeDef  *I2C_SCL;
    IOPinTypeDef  *UART_RX;
    IOPinTypeDef  *UART_TX;
    IOPinTypeDef  *RESETN;
    IOPinTypeDef  *DRV_ENABLE;
} PinsTypeDef;

static PinsTypeDef Pins;

void TMC9660_init(void)
{
    Pins.SPI_EN                = &HAL.IOs->pins->DIO5;
    Pins.SPI1_SCK              = &HAL.IOs->pins->SPI1_SCK;
    Pins.SPI1_MOSI             = &HAL.IOs->pins->SPI1_SDI;
    Pins.SPI1_MISO             = &HAL.IOs->pins->SPI1_SDO;
    Pins.SPI1_CSN              = &HAL.IOs->pins->SPI1_CSN;

#if defined(LandungsbrueckeV3)
    Pins.UART_RX               = &HAL.IOs->pins->DIO10_UART_TX; //Pin21
    Pins.UART_TX               = &HAL.IOs->pins->DIO11_UART_RX; //Pin22

    //Set MUX_1 and MUX_2 to zero to connect DIO10 and DIO11 to UART pins DIO10_UART_TX and DIO11_UART_RX respectively.
    *HAL.IOs->pins->SW_UART_PWM.resetBitRegister     = HAL.IOs->pins->SW_UART_PWM.bitWeight;
#else
    Pins.UART_RX               = &HAL.IOs->pins->DIO10; //Pin21
    Pins.UART_TX               = &HAL.IOs->pins->DIO11; //Pin22
#endif

//    HAL.IOs->config->toOutput(Pins.SPI_EN);
//    HAL.IOs->config->toOutput(Pins.I2C_EN);
//    HAL.IOs->config->toOutput(Pins.HOLD_FLASHN);
//    HAL.IOs->config->toOutput(Pins.RESETN);

}
