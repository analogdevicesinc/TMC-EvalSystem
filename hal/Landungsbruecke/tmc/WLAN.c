#include "../../HAL.h"
//#include "../../WLAN.h"
#include "../freescale/Cpu.h"

#include <string.h>

#define BUFFER_SIZE           1024
#define WLAN_CMD_BUFFER_SIZE  128 // ascii command string buffer

#define CMDBUFFER_END_CHAR '\0'

#define INTR_PRI 6

#define UART_TIMEOUT_VALUE 5

static void init();
static void deInit();
static void tx(uint8 ch);
static uint8 rx(uint8 *ch);
static void txN(uint8 *str, uint8 number);
static uint8 rxN(uint8 *ch, uint8 number);
static void clearBuffers(void);
static uint32 bytesAvailable();

// ring buffers (used in BufferingTypedef struct)
static volatile uint8 rxBuffer[BUFFER_SIZE];
static volatile uint8 txBuffer[BUFFER_SIZE];

static int8 cmdBuffer[WLAN_CMD_BUFFER_SIZE];
static uint32 cmdBufferSize = 0;
static uint32 cmdEnabledTime; // systick timestamp when command mode sequence has been sent

static WLANStateTypedef wlanState = WLAN_DATA_MODE;

static volatile uint32 available = 0;

uint32 UART0_TimeoutTimer;

RXTXTypeDef WLAN =
{
	.init            = init,
	.deInit          = deInit,
	.rx              = rx,
	.tx              = tx,
	.rxN             = rxN,
	.txN             = txN,
	.clearBuffers    = clearBuffers,
	.baudRate        = 57600,
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

	HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED6);
	HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED6);

	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

	HAL.IOs->pins->WIRELESS_RX.configuration.GPIO_Mode = GPIO_Mode_AF3;
	HAL.IOs->pins->WIRELESS_TX.configuration.GPIO_Mode = GPIO_Mode_AF3;

	HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_RX);
	HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_TX);
	/* Disable the transmitter and receiver */
	UART_C2_REG(UART0_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

	/* Configure the UART for 8-bit mode, no parity */
	/* We need all default settings, so entire register is cleared */
	UART_C1_REG(UART0_BASE_PTR) = 0;

	ubd = (CPU_BUS_CLK_HZ / 16) / (WLAN.baudRate);

	UART_BDH_REG(UART0_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
	UART_BDL_REG(UART0_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);

	/* Enable receiver and transmitter */
	UART_C2_REG(UART0_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);

	enable_irq(INT_UART0_RX_TX-16);
}

static void deInit()
{
	SIM_SCGC4 &= ~(SIM_SCGC4_UART0_MASK);

	HAL.IOs->pins->WIRELESS_RX.configuration.GPIO_Mode = GPIO_Mode_IN;
	HAL.IOs->pins->WIRELESS_TX.configuration.GPIO_Mode = GPIO_Mode_IN;

	HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_RX);
	HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_TX);

	disable_irq(INT_UART0_RX_TX-16);

	clearBuffers();
}

void UART0_RX_TX_IRQHandler(void)
{
	uint32 status = UART0_S1;

	if(status & UART_S1_RDRF_MASK)
	{
		buffers.rx.buffer[buffers.rx.wrote] = UART0_D;
		buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
		available++;

		// reset timeout value
		UART0_TimeoutTimer = UART_TIMEOUT_VALUE;
		UART0_S1 &= ~(UART_S1_RDRF_MASK); // Zurücksetzen InterruptFlag
	}

	if(status & UART_S1_TDRE_MASK)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UART0_D	= buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
		}
		else // empty buffer -> turn off send interrupt
		{
			UART0_C2 &= ~UART_C2_TIE_MASK;
		}
		UART0_S1 &= ~(UART_S1_TDRE_MASK); // Zurücksetzen InterruptFlag
	}
}

// Send without checking for CMD/Data mode
static void rawTx(uint8 ch)
{
	if(wlanState == WLAN_INIT_CMD_MODE)
		return;

	buffers.tx.buffer[buffers.tx.wrote] = ch;

	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;	// Move ring buffer index

	// enable send interrupt
	UART0_C2 |= UART_C2_TIE_MASK;
}

// Wrapper for rawTx, will silently fail if we're not in data mode
// todo CHECK ADD 3: Should tx be given a return type in order to report failure to send? (LH) #1
static void tx(uint8 ch)
{
	if(checkReadyToSend())
		rawTx(ch);
}

static uint8 rawRx(uint8 *ch)
{
	if(buffers.rx.read == buffers.rx.wrote)
		return 0;

	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;	// Move ring buffer index
	available--;

	return 1;
}

static uint8 rx(uint8 *ch)
{
	if(wlanState != WLAN_DATA_MODE)
		return 0;

	return rawRx(ch);
}

// todo CHECK ADD 3: Should txN be given a return type in order to report failure to send? (LH) #2
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
	disable_irq(INT_UART0_RX_TX-16);
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
	enable_irq(INT_UART0_RX_TX-16);
}

static uint32 bytesAvailable()
{
	return available;
}

uint32 checkReadyToSend()
{
	if(checkCmdModeEnabled())
	{
		return FALSE;
	}
	else
	{
		return (wlanState == WLAN_INIT_CMD_MODE)? FALSE:TRUE;
	}
}

void enableWLANCommandMode()
{	/* To enable command mode, the escape character (default: $) needs to be sent 3 times.
	 * Additionally, both before and after that sequence there should be 250ms without data sent to the module
	 * Since the configuration mode is supposed to be used as a simple testing tool,
	 * there is no check for the time span before the write. If the switching fails due to that,
	 * an error will be returned upon attempted command execution, just try to reenter command mode then.
	 */
	wlanState = WLAN_CMD_MODE; // Block external write sources

	clearBuffers();
	rawTx('$'); // txN doesn't work, as WLAN_CMD_MODE prevents tx (which txN calls) from writing to the buffer)
	rawTx('$');
	rawTx('$');
	wlanState = WLAN_INIT_CMD_MODE; // Block all writes
	cmdEnabledTime = systick_getTick();
}

uint32 checkCmdModeEnabled()
{
	if(wlanState == WLAN_CMD_MODE)
		return TRUE;
	else if(wlanState == WLAN_DATA_MODE)
		return FALSE;

	uint8 reply[4] = { 0 };	// expected reply: {'C','M','D'}, we're appending \0 so we have a NULL-terminated string that we can use in strcmp()
	if(rxN(reply, 3))
	{
		if(strcmp((const char *)reply, "CMD") == 0)
		{
			wlanState = WLAN_CMD_MODE;
			return TRUE;
		}
		else
		{	// Unexpected answer - going back to data mode
			wlanState = WLAN_DATA_MODE;
			return FALSE;
		}
	}
	else
	{
		if(timeSince(cmdEnabledTime) > 350)	// 250 ms from chip spec + 100ms, just to be safe
		{	// Too much time passed since attempted cmd mode switching happened - assuming it failed
			wlanState = WLAN_DATA_MODE;
			return FALSE;
		}
		else
		{	// Not enough time passed, we're not in command mode yet but we're still giving the chip time
			return FALSE;
		}
	}
}

uint32 handleWLANCommand(BufferCommandTypedef cmd, uint32 value)
{
	switch(cmd)
	{
	case BUFFER_CLEAR:
		cmdBufferSize = 0;
		break;
	case BUFFER_WRITE:
		while((value & 0xFF) != CMDBUFFER_END_CHAR)
		{
			if(cmdBufferSize == WLAN_CMD_BUFFER_SIZE)
			{
				if((value & 0xFF) != 0)	// Any value still remaining -> too much data for buffer -> return an error
					return 1;
				break;
			}
			cmdBuffer[cmdBufferSize] = value & 0xFF;
			value >>= 8;
			cmdBufferSize++;
		}
		break;
	case BUFFER_EXECUTE:
		// Abort if not in command mode. IDE/User should switch to command mode before executing
		if(!checkCmdModeEnabled())
			return 1;

		for(uint32 i = 0; i < cmdBufferSize; i++)
			rawTx(cmdBuffer[i]); // Can't use txN since its blocked from sending while in command mode
		rawTx('\r'); // End of command character

		cmdBufferSize = 0;
		break;
	}

	return 0;
}

uint32 getCMDReply()
{
	uint8 cmdReply;
	uint32 result = 0;

	for(int i = 0; i < 4; i++)
	{
		if(rawRx(&cmdReply) == 0)
			cmdReply = 0;
		// First character is in the smallest byte of result
		result |= cmdReply << 24;
		result >>= 8;
	}

	return result;
}
