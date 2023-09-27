/*******************************************************************************
* Copyright © 2023 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "hal/HAL.h"
#include "hal/RS232.h"

#define BUFFER_SIZE  1024
#define INTR_PRI     6

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

static volatile uint8_t
	rxBuffer[BUFFER_SIZE],
	txBuffer[BUFFER_SIZE];

static uint32_t available = 0;

RXTXTypeDef RS232 =
{
	.init            = init,
	.deInit          = deInit,
	.rx              = rx,
	.tx              = tx,
	.rxN             = rxN,
	.txN             = txN,
	.clearBuffers    = clearBuffers,
	.baudRate        = 115200,
	.bytesAvailable  = bytesAvailable
};

static RXTXBufferingTypeDef buffers =
{
	.rx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = rxBuffer
	},

	.tx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = txBuffer
	}
};

void __attribute__ ((interrupt)) USART6_IRQHandler(void);

static void init()
{

}

static void deInit()
{

}

void USART6_IRQHandler(void)
{

}

static void tx(uint8_t ch)
{

}

static uint8_t rx(uint8_t *ch)
{
return 0;
}

static void txN(uint8_t *str, unsigned char number)
{

}

static uint8_t rxN(uint8_t *str, unsigned char number)
{
	return 0;

}

static void clearBuffers(void)
{

}

static uint32_t bytesAvailable()
{
	return 0;

}

