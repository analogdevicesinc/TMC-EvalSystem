#include "hal/HAL.h"
#include "hal/UART.h"
#include "hal/Landungsbruecke/freescale/Cpu.h"

#define BUFFER_SIZE         32
#define INTR_PRI            6
#define UART_TIMEOUT_VALUE  10
#define WRITE_READ_DELAY    10

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, uint8_t number);
static uint8_t rxN(uint8_t *ch, uint8_t number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

static volatile uint8_t
	rxBuffer[BUFFER_SIZE],
	txBuffer[BUFFER_SIZE];

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

static void init()
{
	register uint16_t ubd = (CPU_BUS_CLK_HZ / 16) / UART.rxtx.baudRate;

	// One wire UART communication needs the TxD pin to be in open drain mode
	// and a pull-up resistor on the RxD pin.
	switch(UART.pinout) {
	case UART_PINS_2:
		HAL.IOs->pins->DIO10.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO10)
		HAL.IOs->pins->DIO11.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO11)
		HAL.IOs->pins->DIO10.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
		HAL.IOs->pins->DIO11.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
		HAL.IOs->config->set(&HAL.IOs->pins->DIO10);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO11);
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
		UART_C2_REG(UART0_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		UART_C1_REG(UART0_BASE_PTR) = 0;
		UART_C4_REG(UART0_BASE_PTR) = 0;
		UART_BDH_REG(UART0_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
		UART_BDL_REG(UART0_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);
		UART_C2_REG(UART0_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
		enable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_1:
	default:
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
		UART_C1_REG(UART2_BASE_PTR) = 0;
		switch(UART.mode) {
		case UART_MODE_SINGLE_WIRE:
			HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
			HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)
			HAL.IOs->pins->DIO18.configuration.GPIO_OType = GPIO_OType_OD;  // RxD as open drain output
			HAL.IOs->pins->DIO17.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // TxD with pull-up resistor
			HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
			HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
			// Enable single wire UART
			UART_C1_REG(UART2_BASE_PTR) |= (UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK);
			// Set TxD as output in single wire UART
			UART_C3_REG(UART2_BASE_PTR) |= UART_C3_TXDIR_MASK;
			break;
		case UART_MODE_DUAL_WIRE:
		default:
			HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
			HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)
			HAL.IOs->pins->DIO17.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
			HAL.IOs->pins->DIO18.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
			HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
			HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
			break;
		}
		UART_C2_REG(UART2_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		UART_BDH_REG(UART2_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
		UART_BDL_REG(UART2_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);
		UART_C2_REG(UART2_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
		enable_irq(INT_UART2_RX_TX-16);
		break;
	}

//	/* Disable the transmitter and receiver */
//	UART_C2_REG(UART0_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
//
//	/* Configure the UART for 8-bit mode, no parity */
//	/* We need all default settings, so entire register is cleared */
//	UART_C1_REG(UART0_BASE_PTR) = 0;
//
//	ubd = (CPU_BUS_CLK_HZ / 16) / UART.baudRate;
//
//	UART_BDH_REG(UART0_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
//	UART_BDL_REG(UART0_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);
//
//	/* Enable receiver and transmitter */
//	UART_C2_REG(UART0_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);

}

static void deInit()
{
	switch(UART.pinout) {
	case UART_PINS_2:
		SIM_SCGC4 &= ~(SIM_SCGC4_UART0_MASK);
		HAL.IOs->pins->DIO10.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->pins->DIO11.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->config->set(&HAL.IOs->pins->DIO10);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO11);
		disable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_1:
	default:
		SIM_SCGC4 &= ~(SIM_SCGC4_UART2_MASK);
		HAL.IOs->pins->DIO17.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->pins->DIO18.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
		disable_irq(INT_UART2_RX_TX-16);
		break;
	}

	clearBuffers();
}

void UART0_RX_TX_IRQHandler_UART(void)
{
	static uint8_t isSending = false;
	uint32_t status = UART0_S1;

	// Receive interrupt
	if(status & UART_S1_RDRF_MASK)
	{
		// One-wire UART communication:
		buffers.rx.buffer[buffers.rx.wrote] = UART0_D;
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
		isSending = false;
		UART0_C2 &= ~UART_C2_TCIE_MASK;
	}

	// Transmit buffer empty interrupt => send next byte if there is something
	// to be sent.
	if(status & UART_S1_TDRE_MASK)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UART0_D = buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;

			isSending = true; // Ignore echo
			UART0_C2 |= UART_C2_TCIE_MASK; // Turn on transmission complete interrupt
		}
		else
		{
			UART0_C2 &= ~UART_C2_TIE_MASK; // empty buffer -> turn off transmit buffer empty interrupt
		}
	}
}

void UART2_RX_TX_IRQHandler(void)
{
	static uint8_t isSending = false;
	uint32_t status = UART2_S1;

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
		isSending = false;
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

			isSending = true; // Ignore echo
			UART2_C2 |= UART_C2_TCIE_MASK; // Turn on transmission complete interrupt
		}
		else
		{
			UART2_C2 &= ~UART_C2_TIE_MASK; // empty buffer -> turn off transmit buffer empty interrupt
		}
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
	wait(WRITE_READ_DELAY);

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
	wait(WRITE_READ_DELAY);
}

void UART_setEnabled(UART_Config *channel, uint8_t enabled)
{
	switch(channel->pinout)
	{
	case UART_PINS_2:
		if (enabled)
		{
			HAL.IOs->pins->DIO10.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO10)
			HAL.IOs->pins->DIO11.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO11)
			HAL.IOs->pins->DIO10.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
			HAL.IOs->pins->DIO11.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
			HAL.IOs->config->set(&HAL.IOs->pins->DIO10);
			HAL.IOs->config->set(&HAL.IOs->pins->DIO11);
		}
		else
		{
			HAL.IOs->config->reset(&HAL.IOs->pins->DIO10);
			HAL.IOs->config->reset(&HAL.IOs->pins->DIO11);
		}
		break;
	case UART_PINS_1:
		if (enabled)
		{
			HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
			HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)

			if (channel->mode == UART_MODE_SINGLE_WIRE)
			{
				HAL.IOs->pins->DIO18.configuration.GPIO_OType = GPIO_OType_OD;  // RxD as open drain output
				HAL.IOs->pins->DIO17.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // TxD with pull-up resistor
			}
			else
			{
				HAL.IOs->pins->DIO17.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
				HAL.IOs->pins->DIO18.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
			}

			HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
			HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
		}
		else
		{
			HAL.IOs->config->reset(&HAL.IOs->pins->DIO17);
			HAL.IOs->config->reset(&HAL.IOs->pins->DIO18);
		}
		break;
	default:
		break;

	}
}

static void tx(uint8_t ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;

	// enable send interrupt
	switch(UART.pinout) {
	case UART_PINS_2:
		UART0_C2 |= UART_C2_TIE_MASK;
		break;
	case UART_PINS_1:
	default:
		UART2_C2 |= UART_C2_TIE_MASK;
		break;
	}
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

static void txN(uint8_t *str, uint8_t number)
{
	for(int32_t i = 0; i < number; i++)
		tx(str[i]);
}

static uint8_t rxN(uint8_t *str, uint8_t number)
{
	if(available < number)
		return 0;

	for(int32_t i = 0; i < number; i++)
		rx(&str[i]);

	return 1;
}

static void clearBuffers(void)
{
	switch(UART.pinout) {
	case UART_PINS_2:
		disable_irq(INT_UART0_RX_TX-16);
		available         = 0;
		buffers.rx.read   = 0;
		buffers.rx.wrote  = 0;
		buffers.tx.read   = 0;
		buffers.tx.wrote  = 0;
		enable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_1:
	default:
		disable_irq(INT_UART2_RX_TX-16);
		available         = 0;
		buffers.rx.read   = 0;
		buffers.rx.wrote  = 0;
		buffers.tx.read   = 0;
		buffers.tx.wrote  = 0;
		enable_irq(INT_UART2_RX_TX-16);
		break;
	}
}

static uint32_t bytesAvailable()
{
	return available;
}

