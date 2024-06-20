/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"

static SPIChannelTypeDef *TMC8100_SPIChannel;

typedef struct
{
    IOPinTypeDef  *SPI_DATA_AVAILABLE;
    IOPinTypeDef  *SPI_MOSI;
    IOPinTypeDef  *SPI_MISO;
    IOPinTypeDef  *SPI_SCK;
    IOPinTypeDef  *SPI_CSN;
    IOPinTypeDef  *RESETN;
    IOPinTypeDef  *I2C_SDA;
    IOPinTypeDef  *I2C_SCL;
    IOPinTypeDef  *UART_RX;
    IOPinTypeDef  *UART_TX;

} PinsTypeDef;

static PinsTypeDef Pins;

int32_t tmc8100_writeInt(int32_t value)
{
    int32_t retval = spi_ch2_readWriteByte(TMC8100_SPIChannel, value>>24, false);
    retval <<= 8;
    retval |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>16), false);
    retval <<= 8;
    retval |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>8), false);
    retval <<= 8;
    retval |= spi_ch2_readWriteByte(TMC8100_SPIChannel, 0xFF & (value>>0), true);
    return retval;
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
    	*value = tmc8100_writeInt(*value);
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
    case 5:
        // Pull down I2C_SDA & I2C_SCL
#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
        Pins.I2C_SDA->configuration.GPIO_PuPd  = GPIO_PuPd_DOWN;
        Pins.I2C_SCL->configuration.GPIO_PuPd  = GPIO_PuPd_DOWN;
#elif defined(LandungsbrueckeV3)
        Pins.I2C_SDA->configuration.GPIO_PuPd  = GPIO_PUPD_PULLDOWN;
        Pins.I2C_SCL->configuration.GPIO_PuPd  = GPIO_PUPD_PULLDOWN;
#endif
        HAL.IOs->config->toOutput(Pins.I2C_SDA);
        HAL.IOs->config->toOutput(Pins.I2C_SCL);
        break;
    case 6:
        // Set I2C_SDA and I2C_SCL to high-Z
        HAL.IOs->config->reset(Pins.I2C_SDA);
        HAL.IOs->config->reset(Pins.I2C_SCL);
        break;
    case 7:
        // Pull down UART pins
#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
        Pins.UART_TX->configuration.GPIO_PuPd  = GPIO_PuPd_DOWN;
        Pins.UART_RX->configuration.GPIO_PuPd  = GPIO_PuPd_DOWN;
#elif defined(LandungsbrueckeV3)
        Pins.UART_TX->configuration.GPIO_PuPd  = GPIO_PUPD_PULLDOWN;
        Pins.UART_RX->configuration.GPIO_PuPd  = GPIO_PUPD_PULLDOWN;
#endif
        HAL.IOs->config->toOutput(Pins.UART_TX);
        HAL.IOs->config->toOutput(Pins.UART_RX);
        break;
    case 8:
        // Set UART pins to high-Z
        HAL.IOs->config->reset(Pins.UART_TX);
        HAL.IOs->config->reset(Pins.UART_RX);
        HAL.IOs->config->toInput(Pins.UART_TX);
        HAL.IOs->config->toInput(Pins.UART_RX);
        break;
    case 9:
        // Set SPI pins to high-Z
        HAL.IOs->config->reset(Pins.SPI_MOSI);
        HAL.IOs->config->reset(Pins.SPI_MISO);
        HAL.IOs->config->reset(Pins.SPI_SCK);
        HAL.IOs->config->reset(Pins.SPI_CSN);
        HAL.IOs->config->toInput(Pins.SPI_MOSI);
        HAL.IOs->config->toInput(Pins.SPI_MISO);
        HAL.IOs->config->toInput(Pins.SPI_SCK);
        HAL.IOs->config->toInput(Pins.SPI_CSN);
        break;
    case 10:
        // Enable SPI
        HAL.IOs->config->reset(Pins.SPI_MOSI);
        HAL.IOs->config->reset(Pins.SPI_MISO);
        HAL.IOs->config->reset(Pins.SPI_SCK);
        HAL.IOs->config->reset(Pins.SPI_CSN);
        SPI.init();
        TMC8100_SPIChannel = &HAL.SPI->ch2;
        TMC8100_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN2;
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
    Pins.SPI_SCK               = &HAL.IOs->pins->SPI2_SCK; //Pin27
    Pins.SPI_MOSI              = &HAL.IOs->pins->SPI2_SDI; //Pin29
    Pins.SPI_MISO              = &HAL.IOs->pins->SPI2_SDO; //Pin28
    Pins.SPI_CSN               = &HAL.IOs->pins->SPI2_CSN2; //Pin26
    Pins.RESETN                = &HAL.IOs->pins->DIO8; //Pin19
    Pins.I2C_SDA               = &HAL.IOs->pins->DIO3; //Pin11
    Pins.I2C_SCL               = &HAL.IOs->pins->DIO2; //Pin10

#if defined(LandungsbrueckeV3)
    Pins.UART_RX               = &HAL.IOs->pins->DIO10_UART_TX; //Pin21
    Pins.UART_TX               = &HAL.IOs->pins->DIO11_UART_RX; //Pin22

    //Set MUX_1 and MUX_2 to zero to connect DIO10 and DIO11 to UART pins DIO10_UART_TX and DIO11_UART_RX respectively.
    *HAL.IOs->pins->SW_UART_PWM.resetBitRegister     = HAL.IOs->pins->SW_UART_PWM.bitWeight;
#else
    Pins.UART_RX               = &HAL.IOs->pins->DIO10; //Pin21
    Pins.UART_TX               = &HAL.IOs->pins->DIO11; //Pin22
#endif

    HAL.IOs->config->reset(Pins.SPI_SCK);
    HAL.IOs->config->reset(Pins.SPI_MOSI);
    HAL.IOs->config->reset(Pins.SPI_MISO);
    HAL.IOs->config->reset(Pins.SPI_CSN);
    HAL.IOs->config->reset(Pins.RESETN);
    HAL.IOs->config->reset(Pins.SPI_DATA_AVAILABLE);

    SPI.init();
    TMC8100_SPIChannel = &HAL.SPI->ch2;
    TMC8100_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN2;

    Evalboards.ch2.userFunction    = userFunction;
}
