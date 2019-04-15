#include "hal/HAL.h"
#include "hal/RS232.h"
#include "hal/WLAN.h"
#include "hal/RXTX.h"

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

static volatile uint8_t rxBuffer[BUFFER_SIZE];
static volatile uint8_t txBuffer[BUFFER_SIZE];

static volatile uint32_t available = 0;

RXTXTypeDef WLAN =
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


void __attribute__ ((interrupt)) USART3_IRQHandler(void);

static void init()
{
	USART_InitTypeDef 	UART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	//HAL.IOs->config->toOutput(&HAL.IOs->pins->WIRELESS_NRST);
	//HAL.IOs->config->setLow(&HAL.IOs->pins->WIRELESS_NRST);

	USART_DeInit(USART3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	HAL.IOs->config->reset(&HAL.IOs->pins->WIRELESS_RX);
	HAL.IOs->config->reset(&HAL.IOs->pins->WIRELESS_TX);

	GPIO_PinAFConfig(HAL.IOs->pins->WIRELESS_RX.port, HAL.IOs->pins->WIRELESS_RX.bit, GPIO_AF_USART3);
	GPIO_PinAFConfig(HAL.IOs->pins->WIRELESS_TX.port, HAL.IOs->pins->WIRELESS_TX.bit, GPIO_AF_USART3);

	USART_StructInit(&UART_InitStructure);
	UART_InitStructure.USART_BaudRate = WLAN.baudRate;
	USART_Init(USART3,&UART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel                    = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = INTR_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	USART_ClearFlag(USART3, USART_FLAG_CTS | USART_FLAG_LBD  | USART_FLAG_TXE  |
	                        USART_FLAG_TC  | USART_FLAG_RXNE | USART_FLAG_IDLE |
	                        USART_FLAG_ORE | USART_FLAG_NE   | USART_FLAG_FE |
	                        USART_FLAG_PE);
	USART_ITConfig(USART3,USART_IT_PE, DISABLE);
	USART_ITConfig(USART3,USART_IT_TXE, ENABLE);
	USART_ITConfig(USART3,USART_IT_TC,  ENABLE);
	USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3,USART_IT_IDLE, DISABLE);
	USART_ITConfig(USART3,USART_IT_LBD, DISABLE);
	USART_ITConfig(USART3,USART_IT_CTS, DISABLE);
	USART_ITConfig(USART3,USART_IT_ERR, DISABLE);

	USART_Cmd(USART3, ENABLE);
}

static void deInit()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_Cmd(USART3, DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel     = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd  = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearFlag
	(
		USART3,
		0
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

void USART3_IRQHandler(void)
{
	if(USART3->SR & USART_FLAG_RXNE)
	{
		buffers.rx.buffer[buffers.rx.wrote] = USART3->DR;
		buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
		available++;
	}

	if(USART3->SR & USART_FLAG_TXE)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			USART3->DR	= buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
		}
		else
		{
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
	}

	if(USART3->SR & USART_FLAG_TC)
	{
		USART_ClearITPendingBit(USART3, USART_IT_TC);
	}
}

static void tx(uint8_t ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
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

// todo ADD 3: Implement WLAN Configuration functionality for Startrampe (LH)
uint32_t checkReadyToSend() { return 0; }
void enableWLANCommandMode() {};
uint32_t checkCmdModeEnabled() { return 0; }
uint32_t handleWLANCommand(BufferCommandTypedef cmd, uint32_t value) {UNUSED(cmd); UNUSED(value); return 1; }
uint32_t getCMDReply() { return 0; }
