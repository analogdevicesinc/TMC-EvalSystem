#include "../../HAL.h"
#include "../../RS232.h"
#include "../freescale/Cpu.h"

#define BUFFER_SIZE         1024
#define INTR_PRI            6
#define UART_TIMEOUT_VALUE  5

static void init();
static void deInit();
static void tx(uint8 ch);
static uint8 rx(uint8 *ch);
static void txN(uint8 *str, uint8 number);
static uint8 rxN(uint8 *ch, uint8 number);
static void clearBuffers(void);
static uint32 bytesAvailable();

static volatile uint8
	rxBuffer[BUFFER_SIZE],
	txBuffer[BUFFER_SIZE];

static volatile uint32 available = 0;

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

static void init()
{
	register uint16 ubd;

	SIM_SCGC1 |= (SIM_SCGC1_UART4_MASK);

	HAL.IOs->pins->RS232_RX.configuration.GPIO_Mode = GPIO_Mode_AF3;
	HAL.IOs->pins->RS232_TX.configuration.GPIO_Mode = GPIO_Mode_AF3;

	HAL.IOs->config->set(&HAL.IOs->pins->RS232_RX);
	HAL.IOs->config->set(&HAL.IOs->pins->RS232_TX);
	/* Disable the transmitter and receiver */
	UART_C2_REG(UART4_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	/* Configure the UART for 8-bit mode, no parity */
	/* We need all default settings, so entire register is cleared */
	UART_C1_REG(UART4_BASE_PTR) = 0;

	ubd = (CPU_BUS_CLK_HZ / 16) / (RS232.baudRate);

	UART_BDH_REG(UART4_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
	UART_BDL_REG(UART4_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);

	/* Enable receiver and transmitter */
	UART_C2_REG(UART4_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);

	enable_irq(INT_UART4_RX_TX-16);
}

static void deInit()
{
	SIM_SCGC1 &= ~(SIM_SCGC1_UART4_MASK);

	HAL.IOs->pins->RS232_RX.configuration.GPIO_Mode = GPIO_Mode_IN;
	HAL.IOs->pins->RS232_TX.configuration.GPIO_Mode = GPIO_Mode_IN;

	HAL.IOs->config->set(&HAL.IOs->pins->RS232_RX);
	HAL.IOs->config->set(&HAL.IOs->pins->RS232_TX);

	disable_irq(INT_UART4_RX_TX-16);

	clearBuffers();
}

void UART4_RX_TX_IRQHandler(void)
{
	uint32 status = UART4_S1;

	if(status & UART_S1_RDRF_MASK)
	{
		buffers.rx.buffer[buffers.rx.wrote] = UART4_D;
		buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
		available++;

		UART4_S1 &= ~(UART_S1_RDRF_MASK);
	}

	if(status & UART_S1_TDRE_MASK)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UART4_D	= buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
		}
		else
		{ // empty buffer -> turn off send interrupt
			UART4_C2 &= ~UART_C2_TIE_MASK;
		}
		UART4_S1 &= ~(UART_S1_TDRE_MASK);
	}
}

static void tx(uint8 ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;

	// enable send interrupt
	UART4_C2 |= UART_C2_TIE_MASK;
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

static void txN(uint8 *str, uint8 number)
{
	for(int32 i = 0; i < number; i++)
		tx(str[i]);
}

static uint8 rxN(uint8 *str, uint8 number)
{
	if(available < number)
		return 0;

	for(int32 i = 0; i < number; i++)
		rx(&str[i]);

	return 1;
}

static void clearBuffers(void)
{
	disable_irq(INT_UART4_RX_TX-16);
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
	enable_irq(INT_UART4_RX_TX-16);
}

static uint32 bytesAvailable()
{
	return available;
}

