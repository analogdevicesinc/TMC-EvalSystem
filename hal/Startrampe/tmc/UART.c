#include "hal/HAL.h"
#include "hal/UART.h"

#define BUFFER_SIZE  1024
#define INTR_PRI     6
#define UART_TIMEOUT_VALUE 10

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

static uint8_t UARTSendFlag;

static volatile uint8_t rxBuffer[BUFFER_SIZE];
static volatile uint8_t txBuffer[BUFFER_SIZE];

static volatile uint32_t available = 0;

UART_Config UART =
{
	.mode = UART_MODE_DUAL_WIRE,
	.pinout = UART_PINS_1,
	.rxtx =
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
	}
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
	uint8_t byte;

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

int UART_readWrite(UART_Config *uart, uint8_t *data, size_t writeLength, uint8_t readLength)
{
	uart->rxtx.clearBuffers();
	uart->rxtx.txN(data, writeLength);
	/* Workaround: Give the UART time to send. Otherwise another write/readRegister can do clearBuffers()
	 * before we're done. This currently is an issue with the IDE when using the Register browser and the
	 * periodic refresh of values gets requested right after the write request.
	 */
	wait(2);

	// Abort early if no data needs to be read back
	if (readLength <= 0)
		return 0;

	// Wait for reply with timeout limit
	uint32_t timestamp = systick_getTick();
	while(uart->rxtx.bytesAvailable() < readLength)
	{
		if(timeSince(timestamp) > UART_TIMEOUT_VALUE)
		{
			// Abort on timeout
			return -1;
		}
	}

	uart->rxtx.rxN(data, readLength);

	return 0;
}

void UART_readInt(UART_Config *channel, uint8_t slave, uint8_t address, int32_t *value)
{
	uint8_t readData[8], dataRequest[4];
	uint32_t timeout;

	dataRequest[0] = 0x05;                        // Sync byte
	dataRequest[1] = slave;                       // Slave address
	dataRequest[2] = address;                     // Register address
	dataRequest[3] = tmc_CRC8(dataRequest, 3, 1); // Cyclic redundancy check

	channel->rxtx.clearBuffers();
	channel->rxtx.txN(dataRequest, ARRAY_SIZE(dataRequest));

	// Wait for reply with timeout limit
	timeout = systick_getTick();
	while(channel->rxtx.bytesAvailable() < ARRAY_SIZE(readData))
		if(timeSince(timeout) > UART_TIMEOUT_VALUE) // Timeout
			return;

	channel->rxtx.rxN(readData, ARRAY_SIZE(readData));
	// Check if the received data is correct (CRC, Sync, Slave address, Register address)
	// todo CHECK 2: Only keep CRC check? Should be sufficient for wrong transmissions (LH) #1
	if(readData[7] != tmc_CRC8(readData, 7, 1) || readData[0] != 0x05 || readData[1] != 0xFF || readData[2] != address)
		return;

	*value = readData[3] << 24 | readData[4] << 16 | readData[5] << 8 | readData[6];
	return;
}

void UART_writeInt(UART_Config *channel, uint8_t slave, uint8_t address, int32_t value)
{
	uint8_t writeData[8];

	writeData[0] = 0x05;                         // Sync byte
	writeData[1] = slave;                        // Slave address
	writeData[2] = address | TMC_WRITE_BIT;      // Register address with write bit set
	writeData[3] = value >> 24;                  // Register Data
	writeData[4] = value >> 16;                  // Register Data
	writeData[5] = value >> 8;                   // Register Data
	writeData[6] = value & 0xFF;                 // Register Data
	writeData[7] = tmc_CRC8(writeData, 7, 1);    // Cyclic redundancy check

	channel->rxtx.clearBuffers();
	for(uint32_t i = 0; i < ARRAY_SIZE(writeData); i++)
		channel->rxtx.tx(writeData[i]);

	/* Workaround: Give the UART time to send. Otherwise another write/readRegister can do clearBuffers()
	 * before we're done. This currently is an issue with the IDE when using the Register browser and the
	 * periodic refresh of values gets requested right after the write request.
	 */
	wait(2);
}

void UART_setEnabled(UART_Config *channel, uint8_t enabled)
{
	UNUSED(channel);

	GPIO_InitTypeDef GPIO_InitStructure;

	if (enabled)
	{
		// UART2-Pins zuweisen (PD5 und PD6)
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
	}
	else
	{
		HAL.IOs->config->reset(&HAL.IOs->pins->DIO17);
		HAL.IOs->config->reset(&HAL.IOs->pins->DIO18);
	}
}

static void tx(uint8_t ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

static uint8_t rx(uint8_t *ch)
{
	if(buffers.rx.read == buffers.rx.wrote)
		return 0;

	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;
	available--;

	return 1;
}

static void txN(uint8_t *str, unsigned char number)
{
	for(int32_t i = 0; i < number; i++)
		tx(str[i]);
}

static uint8_t rxN(uint8_t *str, unsigned char number)
{
	if(bytesAvailable() < number)
		return 0;

	for(int32_t i = 0; i < number; i++)
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

static uint32_t bytesAvailable()
{
	return available;
}

