/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include <stdlib.h>
#include "gd32f4xx.h"
//#include "bits.h"

#include "hal/HAL.h"
#include "hal/I2C.h"

#define FALSE 0
#define TRUE  1
#define I2C_TIMEOUT 10

static void init();

volatile uint32_t I2CTimeout;  //is decremented in the SysTick timer (see SysTick.c)

I2CTypeDef I2C=
{
        .init = init
};

/*******************************************************************
   Function: init
   Parameters: ---

   Return value: ---

   Purpose: Initialization of the I2C interface.
 ********************************************************************/
static void init(void)
{
    rcu_periph_clock_enable(RCU_I2C0);
    /* I2C0 GPIO ports */
    /* connect PB8 to I2C0_SCL */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_8);//DIO4
    /* connect PB9 to I2C0_SDA */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_9);//DIO5

    /* configure I2C0 GPIO */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* configure I2C clock */
    i2c_clock_config(I2C0, 400000, I2C_DTCY_2);//400kHz
    /* configure I2C address */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x7E);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}


/*******************************************************************
   Function: I2CMasterWrite
   Parameters: address: I2C address of the slave
               data: Array with the data to be written
               size: Number of bytes to be written

   Return value: TRUE: I2C transfer successful
                 FALSE: Error

   Purpose: Send data to an I2C slave and wait until
            the data transfer is complete.
 ********************************************************************/
uint8_t I2CMasterWrite(uint8_t address, uint8_t *data, uint8_t size)
{
    uint8_t i;
    uint8_t retval;

    retval=TRUE;

    /* wait until I2C bus is idle */
    I2CTimeout=I2C_TIMEOUT;
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && I2CTimeout>0);
    if(I2CTimeout>0)
    {
        /* send a start condition to I2C bus */
        i2c_start_on_bus(I2C0);
        /* wait until SBSEND bit is set */
        I2CTimeout=I2C_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && I2CTimeout>0);
        if(I2CTimeout>0)
        {
            /* send slave address to I2C bus */
            i2c_master_addressing(I2C0, address, I2C_TRANSMITTER);
            /* wait until ADDSEND bit is set */
            I2CTimeout=I2C_TIMEOUT;
            while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && I2CTimeout>0);
            if(I2CTimeout>0)
            {
                /* clear ADDSEND bit */
                i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
                /* wait until the transmit data buffer is empty */
                I2CTimeout=I2C_TIMEOUT;
                while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && I2CTimeout>0);
                if(I2CTimeout>0)
                {
                    I2CTimeout=I2C_TIMEOUT;
                    for(i = 0; i < size; i++)
                    {
                        /* data transmission */
                        i2c_data_transmit(I2C0, data[i]);
                        /* wait until the TBE bit is set */
                        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && I2CTimeout>0);
                        if(I2CTimeout==0)
                        {
                            retval=FALSE;
                            break;
                        }
                    }
                } else retval=FALSE;
            } else retval=FALSE;
        } else retval=FALSE;
    } else retval=FALSE;

    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    while(I2C_CTL0(I2C0) & I2C_CTL0_STOP);

    return retval;
}


/*******************************************************************
   Function: I2CMasterRead
   Parameters: address: I2C address of the slave
               data: Array for the read data
               size: Number of bytes to be read

   Return value: TRUE: I2C transfer successful
                 FALSE: Error

   Purpose: Read data from an I2C slave and wait until
            the data transfer is complete.
 ********************************************************************/
uint8_t I2CMasterRead(uint8_t address, uint8_t *data, uint8_t size)
{
    uint8_t i;
    uint8_t retval;

    retval=TRUE;

    /* wait until I2C bus is idle */
    I2CTimeout=I2C_TIMEOUT;
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && I2CTimeout>0);
    if(I2CTimeout>0)
    {
        /* send a start condition to I2C bus */
        i2c_start_on_bus(I2C0);
        /* wait until SBSEND bit is set */
        I2CTimeout=I2C_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && I2CTimeout>0);
        if(I2CTimeout>0)
        {
            /* send slave address to I2C bus */
            i2c_master_addressing(I2C0, address, I2C_RECEIVER);

            /* wait until ADDSEND bit is set */
            I2CTimeout=I2C_TIMEOUT;
            while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && I2CTimeout>0);
            if(I2CTimeout>0)
            {
                /* clear ADDSEND bit */
                i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

                I2CTimeout=I2C_TIMEOUT;
                for(i = 0; i < size; i++)
                {
                    if(size-3 == i)
                    {
                        /* wait until the second last data byte is received into the shift register */
                        while(!i2c_flag_get(I2C0, I2C_FLAG_BTC) && I2CTimeout>0);
                        if(I2CTimeout==0)
                        {
                            retval=FALSE;
                            break;
                        }
                        /* disable acknowledge */
                        i2c_ack_config(I2C0, I2C_ACK_DISABLE);
                    }
                    /* wait until the RBNE bit is set */
                    while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE) && I2CTimeout>0);
                    if(I2CTimeout==0)
                    {
                        retval=FALSE;
                        break;
                    }
                    /* read a data from I2C_DATA */
                    data[i] = i2c_data_receive(I2C0);
                }
            } else retval=FALSE;
        } else retval=FALSE;
    } else retval=FALSE;

    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & I2C_CTL0_STOP);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);

    return retval;
}

uint8_t I2CMasterWriteRead(uint8_t address, uint8_t *writeData, uint8_t writeLength, uint8_t *readData, uint8_t readLength)
{
    uint8_t retval;

    retval=TRUE;

    /* wait until I2C bus is idle */
    I2CTimeout=I2C_TIMEOUT;
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && I2CTimeout>0);
    if(I2CTimeout>0)
    {
        /* send a start condition to I2C bus */
        i2c_start_on_bus(I2C0);
        /* wait until SBSEND bit is set */
        I2CTimeout=I2C_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && I2CTimeout>0);
        if(I2CTimeout>0)
        {
            address = address & 0xFE;  //Write bit
            /* send slave address to I2C bus */
            i2c_master_addressing(I2C0, address, I2C_TRANSMITTER);
            /* wait until ADDSEND bit is set */
            I2CTimeout=I2C_TIMEOUT;
            while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && I2CTimeout>0);
            if(I2CTimeout>0)
            {
                /* clear ADDSEND bit */
                i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
                /* wait until the transmit data buffer is empty */
                I2CTimeout=I2C_TIMEOUT;
                while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && I2CTimeout>0);
                if(I2CTimeout>0)
                {
                    I2CTimeout=I2C_TIMEOUT;
                    for(uint8_t i = 0; i < writeLength; i++)
                    {
                        /* data transmission */
                        i2c_data_transmit(I2C0, writeData[i]);
                        /* wait until the TBE bit is set */
                        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && I2CTimeout>0);
                        if(I2CTimeout==0)
                        {
                            retval=FALSE;
                            break;
                        }
                    }
                } else retval=FALSE;
            } else retval=FALSE;
        } else retval=FALSE;
    } else retval=FALSE;

    //*******************************

    /* wait until I2C bus is idle */
    I2CTimeout=I2C_TIMEOUT;
    if(I2CTimeout>0)
    {
        /* send a start condition to I2C bus */
        i2c_start_on_bus(I2C0);
        /* wait until SBSEND bit is set */
        I2CTimeout=I2C_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && I2CTimeout>0);
        if(I2CTimeout>0)
        {
            address = address|0x01;  //Adresse mit Read-Bit senden
            /* send slave address to I2C bus */
            i2c_master_addressing(I2C0, address, I2C_RECEIVER);

            /* wait until ADDSEND bit is set */
            I2CTimeout=I2C_TIMEOUT;
            while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && I2CTimeout>0);
            if(I2CTimeout>0)
            {
                if(readLength==1 || readLength==2){
                    i2c_ack_config(I2C0, I2C_ACK_DISABLE);

                }
                /* clear ADDSEND bit */
                i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
                if(readLength==1){
                    i2c_stop_on_bus(I2C0);
                }
                I2CTimeout=I2C_TIMEOUT;
                for(uint8_t i = 0; i < readLength; i++)
                {
                    if(readLength==2){
                        while(!i2c_flag_get(I2C0, I2C_FLAG_BTC) && I2CTimeout>0);
                        i2c_stop_on_bus(I2C0);
                        readData[0] = i2c_data_receive(I2C0);
                        readData[1] = i2c_data_receive(I2C0);
                        return retval;
                    }

                    if(readLength-3 == i)
                    {
                        /* wait until the second last data byte is received into the shift register */
                        while(!i2c_flag_get(I2C0, I2C_FLAG_BTC) && I2CTimeout>0);
                        if(I2CTimeout==0)
                        {
                            retval=FALSE;
                            break;
                        }
                        /* disable acknowledge */
                        i2c_ack_config(I2C0, I2C_ACK_DISABLE);
                    }
                    /* wait until the RBNE bit is set */
                    while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE) && I2CTimeout>0);
                    if(I2CTimeout==0)
                    {
                        retval=FALSE;
                        break;
                    }
                    /* read a data from I2C_DATA */
                    readData[i] = i2c_data_receive(I2C0);
                    if(readLength==1)
                        return retval;
                }
            } else retval=FALSE;
        } else retval=FALSE;
    } else retval=FALSE;

    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & I2C_CTL0_STOP);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);

    return retval;

}
