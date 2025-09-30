/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include <stdlib.h>

#include "hal/derivative.h"
#include "nvic.h"
#include "hal/HAL.h"
#include "hal/I2C.h"

typedef enum {I2C_IDLE, I2C_WRITE_DATA, I2C_READ_DATA, I2C_READ_ADDRESS} TI2CState;

static volatile uint8_t I2CWriteLength;
static volatile uint8_t I2CWriteCount;
static volatile uint8_t I2CReadLength;
static volatile uint8_t I2CReadCount;
static volatile uint8_t I2CUseRepeatedStart;
static volatile uint8_t I2CRepeatedReadAddress;
static volatile uint8_t I2CErrorFlag;
static volatile TI2CState I2CState;
static volatile uint8_t *I2CWriteData;
static volatile uint8_t *I2CReadData;
static void init();

void __attribute__ ((interrupt)) I2C0_IRQHandler(void);
I2CTypeDef I2C=
{
        .init = init
};

/*******************************************************************
 WARNING:
  If the functions I2CMasterRead, I2CMasterWrite, or
  I2CMasterWriteRead are called multiple times in succession,
  a short pause (>=1ms, depending on the slave) is necessary
  between calls.

  What is still missing: Timeout in case the I2C slave is not present.
 ********************************************************************/

/*******************************************************************
   Function: I2C0_IRQHandler
   Parameters: ---

   Return value: ---

   Purpose: Interrupt handler for the I2C interface. The actual
            data transfer takes place here.
 ********************************************************************/
void I2C0_IRQHandler(void)
{
    // Reset interrupt flag
    I2C0_S|=I2C_S_IICIF_MASK;

    // Continue according to the current state
    switch(I2CState)
    {
    case I2C_IDLE:
        break;

    case I2C_WRITE_DATA:  //Write the data
        if(I2CWriteCount<I2CWriteLength)
        {
            if(!(I2C0_S & I2C_S_RXAK_MASK))
            {
                //ACK received from the receiver => send next byte
                I2C0_D=I2CWriteData[I2CWriteCount++];
            }
            else
            {
                //NAK from the receiver or receiver not present => generate stop condition, abort operation
                I2C0_C1&= ~I2C_C1_MST_MASK;
                I2C0_C1&= ~I2C_C1_TX_MASK;
                I2CErrorFlag=TRUE;
                I2CState=I2C_IDLE;
            }
        }
        else
        {
            //All bytes sent => generate stop condition or repeated start condition
            if(!I2CUseRepeatedStart)
            {
                //Generate stop condition and end
                I2C0_C1&= ~I2C_C1_MST_MASK;
                I2C0_C1&= ~I2C_C1_TX_MASK;
                I2CState=I2C_IDLE;
            }
            else
            {
                //Generate repeated start => send read address and continue with read operation
                I2C0_C1|=I2C_C1_RSTA_MASK;
                I2C0_D=I2CRepeatedReadAddress;
                I2CUseRepeatedStart=FALSE;
                I2CState=I2C_READ_ADDRESS;
            }
        }
        break;

    case I2C_READ_ADDRESS:  //End of address cycle in read operation
        //Switch interface to read mode
        I2C0_C1&= ~I2C_C1_TX_MASK;
        I2CReadData[0]=I2C0_D;  //Dummy Read
        if(!(I2C0_S & I2C_S_RXAK_MASK))
        {
            //ACK from the slave => read operation continues
            I2CState=I2C_READ_DATA;
        }
        else
        {
            //NAK from the slave or slave not present => generate stop condition, abort operation
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            I2CErrorFlag=TRUE;
            I2CState=I2C_IDLE;
        }
        break;

    case I2C_READ_DATA:  //Read the data
        if(I2CReadCount<I2CReadLength)
        {

            //Fetch one byte
            I2CReadData[I2CReadCount++]=I2C0_D;

            //Last byte => generate stop condition
            if((I2CReadLength-I2CReadCount) == 1)
            {
                I2C0_C1&= ~I2C_C1_MST_MASK;
                I2CState=I2C_IDLE;
            }

            //Second last or last byte => prepare NAK
            if(I2CReadLength-I2CReadCount<=2)
                I2C0_C1|=I2C_C1_TXAK_MASK;
        }
        else
        {
            //I2CMasterRead or I2CMasterWriteRead was called with readLength=0 => end
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2CState=I2C_IDLE;
        }
        break;
    }
}


/*******************************************************************
   Function: init
   Parameters: ---

   Return value: ---

   Purpose: Initialization of the I2C interface.
 ********************************************************************/
void init(void)
{
    //Enable clock for I2C
    SIM_SCGC4|=SIM_SCGC4_I2C0_MASK;

    //Configure pins PTB2 and PTB3 for I2C
    PORTB_PCR2=PORT_PCR_MUX(2)|PORT_PCR_ODE_MASK|PORT_PCR_DSE_MASK;
    PORTB_PCR3=PORT_PCR_MUX(2)|PORT_PCR_ODE_MASK|PORT_PCR_DSE_MASK;

    //Configure I2C (Master, 7-bit addresses, 400kHz, interrupt)
    I2C0_F=I2C_F_MULT(0)|I2C_F_ICR(0x1A); //1MHz (ICR=0B => SCL divider=40), 399kHz (ICR=1A)

    I2C0_C1=I2C_C1_IICEN_MASK|I2C_C1_IICIE_MASK; //Enables I2C module operation and I2C interrupt requests.
    I2C0_C2=0x00;

    enable_irq(INT_I2C0-16);
}


/*******************************************************************
   Function: I2CMasterWrite
   Parameters: address: I2C address of the slave
               data: Array with the data to be written
               length: Number of bytes to be written

   Return value: TRUE: I2C transfer successful
                 FALSE: Error

   Purpose: Send data to an I2C slave and wait until
            the data transfer is complete.
 ********************************************************************/
uint8_t I2CMasterWrite(uint8_t address, uint8_t *data, uint8_t length)
{
    uint32_t Timeout;

    //Initialization
    I2CErrorFlag=FALSE;
    I2CUseRepeatedStart=FALSE;
    I2CWriteData=data;
    I2CWriteLength=length;
    I2CWriteCount=0;
    I2CState=I2C_WRITE_DATA;

    //Start operation
    I2C0_C1&= ~I2C_C1_TXAK_MASK;  //Prepare ACK
    I2C0_C1|=I2C_C1_TX_MASK; //Transmit Mode
    I2C0_C1|=I2C_C1_MST_MASK; //Master Mode


    //Send address
    I2C0_D=address & 0xFE;  //Send address without read bit

    //Wait until done (the rest happens in the interrupt)
    Timeout=systick_getTick();
    while(I2CState!=I2C_IDLE)
    {
        if((systick_getTick()-Timeout)>200)
        {
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            I2CState=I2C_IDLE;
            I2CErrorFlag=TRUE;
        }
    }

    //Return error flag
    return !I2CErrorFlag;
}


/*******************************************************************
   Function: I2CMasterRead
   Parameters: address: I2C address of the slave
               data: Array for the read data
               length: Number of bytes to be read

   Return value: TRUE: I2C transfer successful
                 FALSE: Error

   Purpose: Read data from an I2C slave and wait until
            the data transfer is complete.
 ********************************************************************/
uint8_t I2CMasterRead(uint8_t address, uint8_t *data, uint8_t length)
{
    uint32_t Timeout;

    //Initialization
    I2CErrorFlag=FALSE;
    I2CUseRepeatedStart=FALSE;
    I2CReadData=data;
    I2CReadLength=length;
    I2CReadCount=0;
    I2CState=I2C_READ_ADDRESS;

    //Start operation
    I2C0_C1&= ~I2C_C1_TXAK_MASK;  //Prepare ACK
    I2C0_C1|=I2C_C1_TX_MASK;      //Transmit mode
    I2C0_C1|=I2C_C1_MST_MASK;     //Start-Condition

    //Adresse senden
    I2C0_D=address|0x01;  //Send address with read bit

    //Wait until done (the rest happens in the interrupt)
    Timeout=systick_getTick();
    while(I2CState!=I2C_IDLE)
    {
        if((systick_getTick()-Timeout)>200)
        {
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            I2CState=I2C_IDLE;
            I2CErrorFlag=TRUE;
        }
    }

    //Return error flag
    return !I2CErrorFlag;
}


/*******************************************************************
   Function: I2CMasterWriteRead
   Parameters: address: I2C address of the slave
               writeData: Array with the data to be written
               writeLength: Number of bytes to be written
               readData: Array for the data to be read
               readLength: Number of bytes to be read

   Return value: TRUE: I2C transfer successful
                 FALSE: Error

   Purpose: Write to an I2C slave and then read from the same
            slave (with repeated start). For example, to set the
            address register on an EEPROM and then read the data.
 ********************************************************************/
uint8_t I2CMasterWriteRead(uint8_t address, uint8_t *writeData, uint8_t writeLength, uint8_t *readData, uint8_t readLength)
{
    uint32_t Timeout;

    //Initialization
    I2CErrorFlag=FALSE;

    if(!readLength)
        I2CUseRepeatedStart=FALSE;
    else
        I2CUseRepeatedStart=TRUE;

    I2CWriteData=writeData;
    I2CWriteLength=writeLength;
    I2CWriteCount=0;
    I2CReadData=readData;
    I2CReadLength=readLength;
    I2CReadCount=0;
    I2CRepeatedReadAddress=address|0x01;  //Send address with read bit
    I2CState=I2C_WRITE_DATA;

    //Start the operation
    I2C0_C1&= ~I2C_C1_TXAK_MASK;  //Prepare ACK
    I2C0_C1|=I2C_C1_TX_MASK;
    I2C0_C1|=I2C_C1_MST_MASK;

    //Send address
    I2C0_D=address & 0xFE;  //Send address without read bit

    //Wait until done (the rest happens in the interrupt)
    Timeout=systick_getTick();
    while(I2CState!=I2C_IDLE)
    {
        if((systick_getTick()-Timeout)>200)
        {
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            I2CState=I2C_IDLE;
            I2CErrorFlag=TRUE;
        }
    }

    //Return the error flag
    return !I2CErrorFlag;
}
