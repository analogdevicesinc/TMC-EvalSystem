/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include <stdlib.h>
#include "hal/derivative.h"
#include "nvic.h"
#include "hal/HAL.h"
#include "hal/IIC.h"

typedef enum {IIC_IDLE, IIC_WRITE_DATA, IIC_READ_DATA, IIC_READ_ADDRESS} TIICState;

static volatile uint8_t IICWriteLength;
static volatile uint8_t IICWriteCount;
static volatile uint8_t IICReadLength;
static volatile uint8_t IICReadCount;
static volatile uint8_t IICUseRepeatedStart;
static volatile uint8_t IICRepeatedReadAddress;
static volatile uint8_t IICErrorFlag;
static volatile TIICState IICState;
static volatile uint8_t *IICWriteData;
static volatile uint8_t *IICReadData;
static void init();

void __attribute__ ((interrupt)) I2C0_IRQHandler(void);
IICTypeDef IIC=
{
        .init = init
};

/*******************************************************************
 WARNING:
  If the functions IICMasterRead, IICMasterWrite, or
  IICMasterWriteRead are called multiple times in succession,
  a short pause (>=1ms, depending on the slave) is necessary
  between calls.

  What is still missing: Timeout in case the IIC slave is not present.
 ********************************************************************/

/*******************************************************************
   Function: I2C0_IRQHandler
   Parameters: ---

   Return value: ---

   Purpose: Interrupt handler for the IIC interface. The actual
            data transfer takes place here.
 ********************************************************************/
void I2C0_IRQHandler(void)
{
    // Reset interrupt flag
    I2C0_S|=I2C_S_IICIF_MASK;

    // Continue according to the current state
    switch(IICState)
    {
    case IIC_IDLE:
        break;

    case IIC_WRITE_DATA:  //Write the data
        if(IICWriteCount<IICWriteLength)
        {
            if(!(I2C0_S & I2C_S_RXAK_MASK))
            {
                //ACK received from the receiver => send next byte
                I2C0_D=IICWriteData[IICWriteCount++];
            }
            else
            {
                //NAK from the receiver or receiver not present => generate stop condition, abort operation
                I2C0_C1&= ~I2C_C1_MST_MASK;
                I2C0_C1&= ~I2C_C1_TX_MASK;
                IICErrorFlag=TRUE;
                IICState=IIC_IDLE;
            }
        }
        else
        {
            //All bytes sent => generate stop condition or repeated start condition
            if(!IICUseRepeatedStart)
            {
                //Generate stop condition and end
                I2C0_C1&= ~I2C_C1_MST_MASK;
                I2C0_C1&= ~I2C_C1_TX_MASK;
                IICState=IIC_IDLE;
            }
            else
            {
                //Generate repeated start => send read address and continue with read operation
                I2C0_C1|=I2C_C1_RSTA_MASK;
                I2C0_D=IICRepeatedReadAddress;
                IICUseRepeatedStart=FALSE;
                IICState=IIC_READ_ADDRESS;
            }
        }
        break;

    case IIC_READ_ADDRESS:  //End of address cycle in read operation
        //Switch interface to read mode
        I2C0_C1&= ~I2C_C1_TX_MASK;
        IICReadData[0]=I2C0_D;  //Dummy Read
        if(!(I2C0_S & I2C_S_RXAK_MASK))
        {
            //ACK from the slave => read operation continues
            IICState=IIC_READ_DATA;
        }
        else
        {
            //NAK from the slave or slave not present => generate stop condition, abort operation
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            IICErrorFlag=TRUE;
            IICState=IIC_IDLE;
        }
        break;

    case IIC_READ_DATA:  //Read the data
        if(IICReadCount<IICReadLength)
        {
            //Last byte => generate stop condition
            if(IICReadLength-IICReadCount==1)
            {
                I2C0_C1&= ~I2C_C1_MST_MASK;
                IICState=IIC_IDLE;
            }

            //Second last or last byte => prepare NAK
            if(IICReadLength-IICReadCount<=2) I2C0_C1|=I2C_C1_TXAK_MASK;

            //Fetch one byte
            IICReadData[IICReadCount++]=I2C0_D;
        }
        else
        {
            //IICMasterRead or IICMasterWriteRead was called with readLength=0 => end
            I2C0_C1&= ~I2C_C1_MST_MASK;
            IICState=IIC_IDLE;
        }
        break;
    }
}


/*******************************************************************
   Function: init
   Parameters: ---

   Return value: ---

   Purpose: Initialization of the IIC interface.
 ********************************************************************/
void init(void)
{
    //Enable clock for IIC
    SIM_SCGC4|=SIM_SCGC4_I2C0_MASK;

    //Configure pins PTB2 and PTB3 for IIC
    PORTB_PCR2=PORT_PCR_MUX(2)|PORT_PCR_ODE_MASK|PORT_PCR_DSE_MASK;
    PORTB_PCR3=PORT_PCR_MUX(2)|PORT_PCR_ODE_MASK|PORT_PCR_DSE_MASK;

    //Configure IIC (Master, 7-bit addresses, 400kHz, interrupt)
    I2C0_F=I2C_F_MULT(0)|I2C_F_ICR(0x1A); //1MHz (ICR=0B => SCL divider=40), 399kHz (ICR=1A)

    I2C0_C1=I2C_C1_IICEN_MASK|I2C_C1_IICIE_MASK; //Enables I2C module operation and I2C interrupt requests.
    I2C0_C2=0x00;

    enable_irq(INT_I2C0-16);
}


/*******************************************************************
   Function: IICMasterWrite
   Parameters: address: IIC address of the slave
               data: Array with the data to be written
               length: Number of bytes to be written

   Return value: TRUE: IIC transfer successful
                 FALSE: Error

   Purpose: Send data to an IIC slave and wait until
            the data transfer is complete.
 ********************************************************************/
uint8_t IICMasterWrite(uint8_t address, uint8_t *data, uint8_t length)
{
    uint32_t Timeout;

    //Initialization
    IICErrorFlag=FALSE;
    IICUseRepeatedStart=FALSE;
    IICWriteData=data;
    IICWriteLength=length;
    IICWriteCount=0;
    IICState=IIC_WRITE_DATA;

    //Start operation
    I2C0_C1&= ~I2C_C1_TXAK_MASK;  //Prepare ACK
    I2C0_C1|=I2C_C1_TX_MASK; //Transmit Mode
    I2C0_C1|=I2C_C1_MST_MASK; //Master Mode


    //Send address
    I2C0_D=address & 0xFE;  //Send address without read bit

    //Wait until done (the rest happens in the interrupt)
    Timeout=systick_getTick();
    while(IICState!=IIC_IDLE)
    {
        if(abs(systick_getTick()-Timeout)>200)
        {
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            IICState=IIC_IDLE;
            IICErrorFlag=TRUE;
        }
    }

    //Return error flag
    return !IICErrorFlag;
}


/*******************************************************************
   Function: IICMasterRead
   Parameters: address: IIC address of the slave
               data: Array for the read data
               length: Number of bytes to be read

   Return value: TRUE: IIC transfer successful
                 FALSE: Error

   Purpose: Read data from an IIC slave and wait until
            the data transfer is complete.
 ********************************************************************/
uint8_t IICMasterRead(uint8_t address, uint8_t *data, uint8_t length)
{
    uint32_t Timeout;

    //Initialization
    IICErrorFlag=FALSE;
    IICUseRepeatedStart=FALSE;
    IICReadData=data;
    IICReadLength=length;
    IICReadCount=0;
    IICState=IIC_READ_ADDRESS;

    //Start operation
    I2C0_C1&= ~I2C_C1_TXAK_MASK;  //Prepare ACK
    I2C0_C1|=I2C_C1_TX_MASK;      //Transmit mode
    I2C0_C1|=I2C_C1_MST_MASK;     //Start-Condition

    //Adresse senden
    I2C0_D=address|0x01;  //Send address with read bit

    //Wait until done (the rest happens in the interrupt)
    Timeout=systick_getTick();
    while(IICState!=IIC_IDLE)
    {
        if(abs(systick_getTick()-Timeout)>200)
        {
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            IICState=IIC_IDLE;
            IICErrorFlag=TRUE;
        }
    }

    //Return error flag
    return !IICErrorFlag;
}


/*******************************************************************
   Function: IICMasterWriteRead
   Parameters: address: IIC address of the slave
               writeData: Array with the data to be written
               writeLength: Number of bytes to be written
               readData: Array for the data to be read
               readLength: Number of bytes to be read

   Return value: TRUE: IIC transfer successful
                 FALSE: Error

   Purpose: Write to an IIC slave and then read from the same
            slave (with repeated start). For example, to set the
            address register on an EEPROM and then read the data.
 ********************************************************************/
uint8_t IICMasterWriteRead(uint8_t address, uint8_t *writeData, uint8_t writeLength, uint8_t *readData, uint8_t readLength)
{
    uint32_t Timeout;

    //Initialization
    IICErrorFlag=FALSE;
    IICUseRepeatedStart=TRUE;
    IICWriteData=writeData;
    IICWriteLength=writeLength;
    IICWriteCount=0;
    IICReadData=readData;
    IICReadLength=readLength;
    IICReadCount=0;
    IICRepeatedReadAddress=address|0x01;  //Send address with read bit
    IICState=IIC_WRITE_DATA;

    //Start the operation
    I2C0_C1&= ~I2C_C1_TXAK_MASK;  //Prepare ACK
    I2C0_C1|=I2C_C1_TX_MASK;
    I2C0_C1|=I2C_C1_MST_MASK;

    //Send address
    I2C0_D=address & 0xFE;  //Send address without read bit

    //Wait until done (the rest happens in the interrupt)
    Timeout=systick_getTick();
    while(IICState!=IIC_IDLE)
    {
        if(abs(systick_getTick()-Timeout)>200)
        {
            I2C0_C1&= ~I2C_C1_MST_MASK;
            I2C0_C1&= ~I2C_C1_TX_MASK;
            IICState=IIC_IDLE;
            IICErrorFlag=TRUE;
        }
    }

    //Return the error flag
    return !IICErrorFlag;
}
