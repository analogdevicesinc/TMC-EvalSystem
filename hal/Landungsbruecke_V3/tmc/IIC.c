/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include <stdlib.h>
#include "gd32f4xx.h"
//#include "bits.h"

#include "hal/HAL.h"
#include "hal/IIC.h"

#define FALSE 0
#define TRUE  1
#define IIC_TIMEOUT 100

static void init();

volatile uint32_t IICTimeout;  //wird im SysTick-Timer heruntergezählt (siehe SysTick.c)

IICTypeDef IIC=
{
    .init = init
};

/*
void I2C0_IRQHandler(void)
{
//Reset interrupt flag
I2C0->ICR |= I2C_ICR_CR_IRQ;

//Continue based on the current state
switch(IICState)
{
case IIC_IDLE:
break;

rust
Copy code
case IIC_WRITE_DATA:  //Writing data
  if(IICWriteCount < IICWriteLength)
  {
    if(!(I2C0->SR1 & I2C_SR1_RXNE))
    {
      //ACK received from receiver => send next byte
      I2C0->DR = IICWriteData[IICWriteCount++];
    }
    else
    {
      //NAK received from receiver or receiver not present => generate stop condition, abort operation
      I2C0->CR1 &= ~(I2C_CR1_START | I2C_CR1_STOP);
      IICErrorFlag = TRUE;
      IICState = IIC_IDLE;
    }
  }
  else
  {
    //All bytes sent => generate stop condition or repeated start condition
    if(!IICUseRepeatedStart)
    {
      //Generate stop condition and finish
      I2C0->CR1 &= ~(I2C_CR1_START | I2C_CR1_STOP);
      IICState = IIC_IDLE;
    }
    else
    {
      //Generate repeated start => send read address and continue with read operation
      I2C0->CR1 |= I2C_CR1_START;
      I2C0->DR = IICRepeatedReadAddress;
      IICUseRepeatedStart = FALSE;
      IICState = IIC_READ_ADDRESS;
    }
  }
  break;

case IIC_READ_ADDRESS:  //End of address cycle in read operation
  //Switch interface to read
  I2C0->CR1 &= ~I2C_CR1_TX;
  IICReadData[0] = I2C0->DR;  //Dummy read
  if(!(I2C0->SR2 & I2C_SR2_RXNE))
  {
    //ACK from slave => continue read operation
    IICState = IIC_READ_DATA;
  }
  else
  {
    //NAK from slave or slave not present => generate stop condition, abort operation
    I2C0->CR1 &= ~(I2C_CR1_START | I2C_CR1_STOP);
    IICErrorFlag = TRUE;
    IICState = IIC_IDLE;
  }
  break;

case IIC_READ_DATA:  //Reading data
  if(IICReadCount < IICReadLength)
  {
    //Last byte => generate stop condition
    if(IICReadLength - IICReadCount == 1)
    {
      I2C0->CR1 &= ~(I2C_CR1_START | I2C_CR1_STOP);
      IICState = IIC_IDLE;
    }

    //Second to last or last byte => prepare NAK
    if(IICReadLength - IICReadCount <= 2) I2C0->CR1 |= I2C_CR1_NACK;

    //Fetch one byte
    IICReadData[IICReadCount++] = I2C0->DR;
  }
  else
  {
    //IICMasterRead or IICMasterWriteRead called with ReadLength=0 => finish
    I2C0->CR1 &= ~(I2C_CR1_START | I2C_CR1_STOP);
    IICState = IIC_IDLE;
  }
  break;
}
}
*/

/*******************************************************************
   Funktion: InitIIC
   Parameter: ---

   Rückgabewert: ---

   Zweck: Initialisierung des IIC-Interface.
********************************************************************/
static void init(void)
{
    rcu_periph_clock_enable(RCU_I2C0);
    /* I2C0 GPIO ports */
    /* connect PB8 to I2C0_SCL */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_8);
    /* connect PB9 to I2C0_SDA */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_9);

    /* configure I2C0 GPIO */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* configure I2C clock */
    i2c_clock_config(I2C0, 400000, I2C_DTCY_2);
    /* configure I2C address */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x7E);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}


/*******************************************************************
   Funktion: IICMasterWrite
   Parameter: Address: IIC-Adresse des Slave
              Data: Array mit den zu schreibenden Daten
              Length: Anzahl der zu schreibenden Bytes

   Rückgabewert: TRUE: IIC-Transfer erfolgreich
                 FALSE: Fehler

   Zweck: Senden von Daten an einen IIC-Slave und warten, bis
          die Datenübertragung beendet ist.
********************************************************************/
uint8_t IICMasterWrite(uint8_t Address, uint8_t *Data, uint8_t Size)
{
  uint8_t i;
  uint8_t retval;

  retval=TRUE;

  /* wait until I2C bus is idle */
  IICTimeout=IIC_TIMEOUT;
  while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && IICTimeout>0);
  if(IICTimeout>0)
  {
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    IICTimeout=IIC_TIMEOUT;
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && IICTimeout>0);
    if(IICTimeout>0)
    {
      /* send slave address to I2C bus */
      i2c_master_addressing(I2C0, Address, I2C_TRANSMITTER);
      /* wait until ADDSEND bit is set */
      IICTimeout=IIC_TIMEOUT;
      while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && IICTimeout>0);
      if(IICTimeout>0)
      {
        /* clear ADDSEND bit */
        i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
        /* wait until the transmit data buffer is empty */
        IICTimeout=IIC_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && IICTimeout>0);
        if(IICTimeout>0)
        {
          IICTimeout=IIC_TIMEOUT;
          for(i = 0; i < Size; i++)
          {
            /* data transmission */
            i2c_data_transmit(I2C0, Data[i]);
            /* wait until the TBE bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && IICTimeout>0);
            if(IICTimeout==0)
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
   Funktion: IICMasterRead
   Parameter: Address: IIC-Adresse des Slave
              Data: Array für die gelesenen Daten
              Length: Anzahl der zu lesenden Bytes

   Rückgabewert: TRUE: IIC-Transfer erfolgreich
                 FALSE: Fehler

   Zweck: Lesen von Daten von einem IIC-Slave und warten, bis
          die Datenübertragung beendet ist.
********************************************************************/
uint8_t IICMasterRead(uint8_t Address, uint8_t *Data, uint8_t Size)
{
  uint8_t i;
  uint8_t retval;

  retval=TRUE;

  /* wait until I2C bus is idle */
  IICTimeout=IIC_TIMEOUT;
  while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && IICTimeout>0);
  if(IICTimeout>0)
  {
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    IICTimeout=IIC_TIMEOUT;
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && IICTimeout>0);
    if(IICTimeout>0)
    {
      /* send slave address to I2C bus */
      i2c_master_addressing(I2C0, Address, I2C_RECEIVER);

      /* wait until ADDSEND bit is set */
      IICTimeout=IIC_TIMEOUT;
      while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && IICTimeout>0);
      if(IICTimeout>0)
      {
        /* clear ADDSEND bit */
        i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

        IICTimeout=IIC_TIMEOUT;
        for(i = 0; i < Size; i++)
        {
          if(Size-3 == i)
          {
            /* wait until the second last data byte is received into the shift register */
            while(!i2c_flag_get(I2C0, I2C_FLAG_BTC) && IICTimeout>0);
            if(IICTimeout==0)
            {
              retval=FALSE;
              break;
            }
            /* disable acknowledge */
            i2c_ack_config(I2C0, I2C_ACK_DISABLE);
          }
          /* wait until the RBNE bit is set */
          while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE) && IICTimeout>0);
          if(IICTimeout==0)
          {
            retval=FALSE;
            break;
          }
          /* read a data from I2C_DATA */
          Data[i] = i2c_data_receive(I2C0);
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

uint8_t IICMasterWriteRead(uint8_t Address, uint8_t *WriteData, uint8_t WriteLength, uint8_t *ReadData, uint8_t ReadLength)
{
    uint8_t retval;

    retval=TRUE;

    /* wait until I2C bus is idle */
    IICTimeout=IIC_TIMEOUT;
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && IICTimeout>0);
    if(IICTimeout>0)
    {
      /* send a start condition to I2C bus */
      i2c_start_on_bus(I2C0);
      /* wait until SBSEND bit is set */
      IICTimeout=IIC_TIMEOUT;
      while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && IICTimeout>0);
      if(IICTimeout>0)
      {
          Address = Address & 0xFE;  //Adresse ohne Read-Bit senden
        /* send slave address to I2C bus */
        i2c_master_addressing(I2C0, Address, I2C_TRANSMITTER);
        /* wait until ADDSEND bit is set */
        IICTimeout=IIC_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && IICTimeout>0);
        if(IICTimeout>0)
        {
          /* clear ADDSEND bit */
          i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
          /* wait until the transmit data buffer is empty */
          IICTimeout=IIC_TIMEOUT;
          while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && IICTimeout>0);
          if(IICTimeout>0)
          {
            IICTimeout=IIC_TIMEOUT;
            for(uint8_t i = 0; i < WriteLength; i++)
            {
              /* data transmission */
              i2c_data_transmit(I2C0, WriteData[i]);
              /* wait until the TBE bit is set */
              while(!i2c_flag_get(I2C0, I2C_FLAG_TBE) && IICTimeout>0);
              if(IICTimeout==0)
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
    IICTimeout=IIC_TIMEOUT;
    if(IICTimeout>0)
    {
      /* send a start condition to I2C bus */
      i2c_start_on_bus(I2C0);
      /* wait until SBSEND bit is set */
      IICTimeout=IIC_TIMEOUT;
      while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && IICTimeout>0);
      if(IICTimeout>0)
      {
          Address = Address|0x01;  //Adresse mit Read-Bit senden
        /* send slave address to I2C bus */
        i2c_master_addressing(I2C0, Address, I2C_RECEIVER);

        /* wait until ADDSEND bit is set */
        IICTimeout=IIC_TIMEOUT;
        while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && IICTimeout>0);
        if(IICTimeout>0)
        {
            if(ReadLength==1 || ReadLength==2){
                i2c_ack_config(I2C0, I2C_ACK_DISABLE);

            }
          /* clear ADDSEND bit */
          i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
          if(ReadLength==1){
              i2c_stop_on_bus(I2C0);
          }
          IICTimeout=IIC_TIMEOUT;
          for(uint8_t i = 0; i < ReadLength; i++)
          {
              if(ReadLength==2){
                  while(!i2c_flag_get(I2C0, I2C_FLAG_BTC) && IICTimeout>0);
                  i2c_stop_on_bus(I2C0);
                  ReadData[0] = i2c_data_receive(I2C0);
                  ReadData[1] = i2c_data_receive(I2C0);
                  return retval;
              }

            if(ReadLength-3 == i)
            {
              /* wait until the second last data byte is received into the shift register */
              while(!i2c_flag_get(I2C0, I2C_FLAG_BTC) && IICTimeout>0);
              if(IICTimeout==0)
              {
                retval=FALSE;
                break;
              }
              /* disable acknowledge */
              i2c_ack_config(I2C0, I2C_ACK_DISABLE);
            }
            /* wait until the RBNE bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE) && IICTimeout>0);
            if(IICTimeout==0)
            {
              retval=FALSE;
              break;
            }
            /* read a data from I2C_DATA */
            ReadData[i] = i2c_data_receive(I2C0);
            if(ReadLength==1)
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



//***********************************

}

