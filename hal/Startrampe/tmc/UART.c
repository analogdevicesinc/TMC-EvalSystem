#include "../../HAL.h"
#include "../../UART.h"

#define BUFFER_SIZE  1024
#define INTR_PRI     6
#define UART_TIMEOUT_VALUE 10

static void init();
static void deInit();
static void tx(uint8 ch);
static uint8 rx(uint8 *ch);
static void txN(uint8 *str, unsigned char number);
static uint8 rxN(uint8 *ch, unsigned char number);
static void clearBuffers(void);
static uint32 bytesAvailable();

static uint8 UARTSendFlag;

static volatile uint8 rxBuffer[BUFFER_SIZE];
static volatile uint8 txBuffer[BUFFER_SIZE];

static volatile uint32 available = 0;

RXTXTypeDef UART =
{
	.init            = init,
	.deInit          = deInit,
	.rx              = rx,
	.tx              = tx,
	.rxN             = rxN,
	.txN             = txN,
	.clearBuffers    = clearBuffers,
	.baudRate        = 115200,
	.bytesAvailable  = bytesAvailable,
	.uart_mode       = UART_MODE_DUAL_WIRE
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

void __attribute__ ((interrupt)) USART2_IRQHandler(void);

static void init()
{
	USART_InitTypeDef UART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_DeInit(USART2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//One wire UART communication needs the TxD pin to be in open drain mode
	//and a pull-up resistor on the RxD pin.
	/*  HAL.IOs->pins->DIO17.configuration.GPIO_Mode = GPIO_Mode_AF;  //TxD (DIO17)
	HAL.IOs->pins->DIO18.configuration.GPIO_Mode = GPIO_Mode_AF;  //RxD (DIO18)
	HAL.IOs->pins->DIO17.configuration.GPIO_OType = GPIO_OType_OD; //TxD as open drain output
	HAL.IOs->pins->DIO18.configuration.GPIO_PuPd  = GPIO_PuPd_UP;  //RxD with pull-up resistor

	HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
	HAL.IOs->config->set(&HAL.IOs->pins->DIO18);

	GPIO_PinAFConfig(HAL.IOs->pins->DIO17.port, HAL.IOs->pins->DIO17.bit, GPIO_AF_USART2);
	GPIO_PinAFConfig(HAL.IOs->pins->DIO18.port, HAL.IOs->pins->DIO18.bit, GPIO_AF_USART2);
	*/
	//GPIOD aktivieren
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//UART2-Pins zuweisen (PD5 und PD6)
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	USART_StructInit(&UART_InitStructure);
	UART_InitStructure.USART_BaudRate = 115200;
	USART_Init(USART2,&UART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel                    = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = INTR_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	USART_ClearFlag(USART2, USART_FLAG_CTS | USART_FLAG_LBD  | USART_FLAG_TXE  |
	                        USART_FLAG_TC  | USART_FLAG_RXNE | USART_FLAG_IDLE |
	                        USART_FLAG_ORE | USART_FLAG_NE   | USART_FLAG_FE   |
	                        USART_FLAG_PE);
	USART_ITConfig(USART2,USART_IT_PE, DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE, ENABLE);
	USART_ITConfig(USART2,USART_IT_TC, ENABLE);
	USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2,USART_IT_IDLE, DISABLE);
	USART_ITConfig(USART2,USART_IT_LBD, DISABLE);
	USART_ITConfig(USART2,USART_IT_CTS, DISABLE);
	USART_ITConfig(USART2,USART_IT_ERR, DISABLE);

	USART_Cmd(USART2, ENABLE);
}

static void deInit()
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	USART_Cmd(USART2, DISABLE);

	NVIC_InitStructure.NVIC_IRQChannel     = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd  = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag(USART2, 0
	                        | USART_FLAG_CTS
	                        | USART_FLAG_LBD
	                        | USART_FLAG_TXE
	                        | USART_FLAG_TC
	                        | USART_FLAG_RXNE
	                        | USART_FLAG_IDLE
	                        | USART_FLAG_ORE
	                        | USART_FLAG_NE
	                        | USART_FLAG_FE
	                        | USART_FLAG_PE
	               );

	clearBuffers();
}

void USART2_IRQHandler(void)
{
	uint8 byte;
	
	// Receive interrupt
	if(USART2->SR & USART_FLAG_RXNE)
	{
		// One-wire UART communication:
		// Ignore received byte when a byte has just been sent (echo).
		byte = USART2->DR;
		if(!UARTSendFlag)
		{	// not sending, received real data instead of echo -> advance ring buffer index and available counter
      buffers.rx.buffer[buffers.rx.wrote]=byte;
			buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
			available++;
		}
	}

	// Transmit buffer empty interrupt => send next byte if there is something
	// to be sent.
	if(USART2->SR & USART_FLAG_TXE)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UARTSendFlag = true;
			USART2->DR  = buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
		}
		else
		{
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}

	// Transmission complete interrupt => do not ignore echo any more
	// after last bit has been sent.
	if(USART2->SR & USART_FLAG_TC)
	{
		//Only if there are no more bytes left in the transmit buffer
		if(buffers.tx.read == buffers.tx.wrote) 
		{
  		byte = USART2->DR;  //Ignore spurios echos of the last sent byte that sometimes occur.
			UARTSendFlag = false;
		}
		USART_ClearITPendingBit(USART2, USART_IT_TC);
	}
}

void UART_readInt(RXTXTypeDef *channel, uint8 slave, uint8 address, int32 *value)
{
	uint8 readData[8], dataRequest[4];
	uint32 timeout;

	dataRequest[0] = 0x05;                        // Sync byte
	dataRequest[1] = slave;                       // Slave address
	dataRequest[2] = address;                     // Register address
	dataRequest[3] = tmc_CRC8(dataRequest, 3, 1); // Cyclic redundancy check

	channel->clearBuffers();
	channel->txN(dataRequest, ARRAY_SIZE(dataRequest));

	// Wait for reply with timeout limit
	timeout = systick_getTick();
	while(channel->bytesAvailable() < ARRAY_SIZE(readData))
		if(timeSince(timeout) > UART_TIMEOUT_VALUE) // Timeout
			return;

	channel->rxN(readData, ARRAY_SIZE(readData));
	// Check if the received data is correct (CRC, Sync, Slave address, Register address)
	// todo CHECK 2: Only keep CRC check? Should be sufficient for wrong transmissions (LH) #1
	if(readData[7] != tmc_CRC8(readData, 7, 1) || readData[0] != 0x05 || readData[1] != 0xFF || readData[2] != address)
		return;

	*value = readData[3] << 24 | readData[4] << 16 | readData[5] << 8 | readData[6];
	return;
}

void UART_writeInt(RXTXTypeDef *channel, uint8 slave, uint8 address, int32 value)
{
	uint8 writeData[8];

	writeData[0] = 0x05;                         // Sync byte
	writeData[1] = slave;                        // Slave address
	writeData[2] = address | TMC_WRITE_BIT;      // Register address with write bit set
	writeData[3] = value >> 24;                  // Register Data
	writeData[4] = value >> 16;                  // Register Data
	writeData[5] = value >> 8;                   // Register Data
	writeData[6] = value & 0xFF;                 // Register Data
	writeData[7] = tmc_CRC8(writeData, 7, 1);    // Cyclic redundancy check

	channel->clearBuffers();
	for(uint32 i = 0; i < ARRAY_SIZE(writeData); i++)
		channel->tx(writeData[i]);

	/* Workaround: Give the UART time to send. Otherwise another write/readRegister can do clearBuffers()
	 * before we're done. This currently is an issue with the IDE when using the Register browser and the
	 * periodic refresh of values gets requested right after the write request.
	 */
	wait(2);
}

static void tx(uint8 ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

static uint8 rx(uint8 *ch)
{
	if(buffers.rx.read == buffers.rx.wrote)
		return 0;

	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;
	available--;

	return 1;
}

static void txN(uint8 *str, unsigned char number)
{
	for(int32 i = 0; i < number; i++)
		tx(str[i]);
}

static uint8 rxN(uint8 *str, unsigned char number)
{
	if(bytesAvailable() < number)
		return 0;

	for(int32 i = 0; i < number; i++)
		rx(&str[i]);

	return 1;
}

static void clearBuffers(void)
{
	__disable_irq();
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
	__enable_irq();
}

static uint32 bytesAvailable()
{
	return available;
}

