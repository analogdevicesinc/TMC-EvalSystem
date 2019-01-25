#include "../../HAL.h"
#include "../../UART.h"
#include "../freescale/Cpu.h"

#define BUFFER_SIZE         32
#define INTR_PRI            6
#define UART_TIMEOUT_VALUE  10

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

static void init()
{
	register uint16 ubd;

	SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;

	// One wire UART communication needs the TxD pin to be in open drain mode
	// and a pull-up resistor on the RxD pin.
	if(UART.uart_mode == UART_MODE_DUAL_WIRE) {
		HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
		HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)
		HAL.IOs->pins->DIO17.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
		HAL.IOs->pins->DIO18.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor

		HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
	} else if(UART.uart_mode == UART_MODE_SINGLE_WIRE) {
		HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
		HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)
		HAL.IOs->pins->DIO18.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
		HAL.IOs->pins->DIO17.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor

		HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO18);

	}

	/* Disable the transmitter and receiver */
	UART_C2_REG(UART2_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

	/* Configure the UART for 8-bit mode, no parity */
	/* We need all default settings, so entire register is cleared */
	UART_C1_REG(UART2_BASE_PTR) = 0;

	ubd = (CPU_BUS_CLK_HZ / 16) / UART.baudRate;

	UART_BDH_REG(UART2_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
	UART_BDL_REG(UART2_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);

	/* Enable receiver and transmitter */
	UART_C2_REG(UART2_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);

	if(UART.uart_mode == UART_MODE_SINGLE_WIRE) {

		// Enable single wire UART
		UART_C1_REG(UART2_BASE_PTR) |= (UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK);

		// Set TxD as output in single wire UART
		UART_C3_REG(UART2_BASE_PTR) |= UART_C3_TXDIR_MASK;

	}

	//(void)UART2_D;
	//(void)UART2_S1;

	enable_irq(INT_UART2_RX_TX-16);
}

static void deInit()
{
	SIM_SCGC4 &= ~(SIM_SCGC4_UART2_MASK);

	HAL.IOs->pins->DIO17.configuration.GPIO_Mode = GPIO_Mode_IN;
	HAL.IOs->pins->DIO18.configuration.GPIO_Mode = GPIO_Mode_IN;

	HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
	HAL.IOs->config->set(&HAL.IOs->pins->DIO18);

	disable_irq(INT_UART2_RX_TX-16);

	clearBuffers();
}

void UART2_RX_TX_IRQHandler(void)
{
	static uint8 isSending = FALSE;
	uint32 status = UART2_S1;

	// Receive interrupt
	if(status & UART_S1_RDRF_MASK)
	{
		// One-wire UART communication:
		buffers.rx.buffer[buffers.rx.wrote] = UART2_D;
		if(!isSending) // Only move ring buffer index & available counter when the received byte wasn't the send echo
		{
			buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
			available++;
		}
	}

	// Transmission complete interrupt => do not ignore echo any more
	// after last bit has been sent.
	if(status & UART_S1_TC_MASK)
	{
		// Last bit has been sent
		isSending = FALSE;
		UART2_C2 &= ~UART_C2_TCIE_MASK;
	}

	// Transmit buffer empty interrupt => send next byte if there is something
	// to be sent.
	if(status & UART_S1_TDRE_MASK)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UART2_D = buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;

			isSending = TRUE; // Ignore echo
			UART2_C2 |= UART_C2_TCIE_MASK; // Turn on transmission complete interrupt
		}
		else
		{
			UART2_C2 &= ~UART_C2_TIE_MASK; // empty buffer -> turn off transmit buffer empty interrupt
		}
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

	// enable send interrupt
	UART2_C2 |= UART_C2_TIE_MASK;
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
	disable_irq(INT_UART2_RX_TX-16);
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
	enable_irq(INT_UART2_RX_TX-16);
}

static uint32 bytesAvailable()
{
	return available;
}

