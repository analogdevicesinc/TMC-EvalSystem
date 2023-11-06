/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "hal/HAL.h"
#include "hal/RS232.h"
#include "hal/WLAN.h"
#include "hal/RXTX.h"
#include <string.h>


#define BUFFER_SIZE  1024
#define WLAN_CMD_BUFFER_SIZE  128 // ascii command string buffer

#define INTR_PRI     6

#define CMDBUFFER_END_CHAR '\0'


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

static int8_t cmdBuffer[WLAN_CMD_BUFFER_SIZE];
static uint32_t cmdBufferSize = 0;
static uint32_t cmdEnabledTime; // systick timestamp when command mode sequence has been sent


static WLANStateTypedef wlanState = WLAN_DATA_MODE;


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

void __attribute__ ((interrupt)) USART1_IRQHandler(void);


static void init()
{
	usart_deinit(RCU_USART1);

	HAL.IOs->pins->WIFI_RX.configuration.GPIO_Mode = GPIO_AF_7;
	HAL.IOs->pins->WIFI_TX.configuration.GPIO_Mode = GPIO_AF_7;

	HAL.IOs->config->set(&HAL.IOs->pins->WIFI_RX);
	HAL.IOs->config->set(&HAL.IOs->pins->WIFI_TX);
	gpio_af_set(HAL.IOs->pins->WIFI_RX.port, GPIO_AF_7, HAL.IOs->pins->WIFI_RX.bitWeight);
	gpio_af_set(HAL.IOs->pins->WIFI_TX.port, GPIO_AF_7, HAL.IOs->pins->WIFI_TX.bitWeight);

	rcu_periph_clock_enable(RCU_USART1);

	usart_hardware_flow_rts_config(RCU_USART1, USART_RTS_DISABLE);
	usart_hardware_flow_cts_config(RCU_USART1, USART_CTS_DISABLE);

    usart_baudrate_set(RCU_USART1, 115200);
    usart_word_length_set(RCU_USART1, USART_WL_8BIT);
    usart_stop_bit_set(RCU_USART1, USART_STB_1BIT);
    usart_parity_config(RCU_USART1, USART_PM_NONE);
    usart_receive_config(RCU_USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(RCU_USART1, USART_TRANSMIT_ENABLE);
    usart_enable(RCU_USART1);
    usart_interrupt_enable(RCU_USART1, USART_INT_TBE);
    usart_interrupt_enable(RCU_USART1, USART_INT_TC);
    usart_interrupt_enable(RCU_USART1, USART_INT_RBNE);

    nvic_irq_enable(USART1_IRQn, INTR_PRI, 0);

	usart_flag_clear(RCU_USART1, USART_FLAG_CTS);
	usart_flag_clear(RCU_USART1, USART_FLAG_LBD);
	usart_flag_clear(RCU_USART1, USART_FLAG_TBE);
	usart_flag_clear(RCU_USART1, USART_FLAG_TC);
	usart_flag_clear(RCU_USART1, USART_FLAG_RBNE);
	usart_flag_clear(RCU_USART1, USART_FLAG_IDLE);
	usart_flag_clear(RCU_USART1, USART_FLAG_ORERR);
	usart_flag_clear(RCU_USART1, USART_FLAG_NERR);
	usart_flag_clear(RCU_USART1, USART_FLAG_FERR);
	usart_flag_clear(RCU_USART1, USART_FLAG_PERR);

}

static void deInit()
{
    usart_disable(RCU_USART1);
    nvic_irq_disable(USART1_IRQn);

	usart_flag_clear(RCU_USART1, USART_FLAG_CTS);
	usart_flag_clear(RCU_USART1, USART_FLAG_LBD);
	usart_flag_clear(RCU_USART1, USART_FLAG_TBE);
	usart_flag_clear(RCU_USART1, USART_FLAG_TC);
	usart_flag_clear(RCU_USART1, USART_FLAG_RBNE);
	usart_flag_clear(RCU_USART1, USART_FLAG_IDLE);
	usart_flag_clear(RCU_USART1, USART_FLAG_ORERR);
	usart_flag_clear(RCU_USART1, USART_FLAG_NERR);
	usart_flag_clear(RCU_USART1, USART_FLAG_FERR);
	usart_flag_clear(RCU_USART1, USART_FLAG_PERR);

	clearBuffers();
}


void USART1_IRQHandler(void)
{
	// Receive interrupt
	if(USART_STAT0(RCU_USART1) & USART_STAT0_RBNE)
	{
		buffers.rx.buffer[buffers.rx.wrote] = USART_DATA(RCU_USART1);
		buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
		available++;

	}

	if(USART_STAT0(RCU_USART1) & USART_STAT0_TBE)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			USART_DATA(RCU_USART1)	= buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
		}
		else // empty buffer -> turn off send interrupt
		{
		    usart_interrupt_disable(RCU_USART1, USART_INT_TBE);
		}
	}

	if(USART_STAT0(RCU_USART1) & USART_STAT0_TC)
	{
	    usart_interrupt_flag_clear(RCU_USART1, USART_INT_FLAG_TC);

	}
}


// Send without checking for CMD/Data mode
static void rawTx(uint8_t ch)
{
	if(wlanState == WLAN_INIT_CMD_MODE)
		return;

	buffers.tx.buffer[buffers.tx.wrote] = ch;

	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;	// Move ring buffer index

	// enable send interrupt
    usart_interrupt_enable(RCU_USART1, USART_INT_TBE);
}

static void tx(uint8_t ch)
{
	if(checkReadyToSend())
		rawTx(ch);
}

static uint8_t rawRx(uint8_t *ch)
{
	if(buffers.rx.read == buffers.rx.wrote)
		return 0;

	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;	// Move ring buffer index
	available--;

	return 1;
}

static uint8_t rx(uint8_t *ch)
{
	if(wlanState != WLAN_DATA_MODE)
		return 0;

	return rawRx(ch);}

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
    nvic_irq_disable(USART1_IRQn);
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
    nvic_irq_enable(USART1_IRQn, INTR_PRI, 0);
}

static uint32_t bytesAvailable()
{
	return available;
}

uint32_t checkReadyToSend() {

	if(checkCmdModeEnabled())
	{
		return false;
	}
	else
	{
		return (wlanState == WLAN_INIT_CMD_MODE)? false:true;
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

uint32_t checkCmdModeEnabled()
{
	if(wlanState == WLAN_CMD_MODE)
		return true;
	else if(wlanState == WLAN_DATA_MODE)
		return false;

	uint8_t reply[4] = { 0 };	// expected reply: {'C','M','D'}, we're appending \0 so we have a NULL-terminated string that we can use in strcmp()
	if(rxN(reply, 3))
	{
		if(strcmp((const char *)reply, "CMD") == 0)
		{
			wlanState = WLAN_CMD_MODE;
			return true;
		}
		else
		{	// Unexpected answer - going back to data mode
			wlanState = WLAN_DATA_MODE;
			return false;
		}
	}
	else
	{
		if(timeSince(cmdEnabledTime) > 350)	// 250 ms from chip spec + 100ms, just to be safe
		{	// Too much time passed since attempted cmd mode switching happened - assuming it failed
			wlanState = WLAN_DATA_MODE;
			return false;
		}
		else
		{	// Not enough time passed, we're not in command mode yet but we're still giving the chip time
			return false;
		}
	}
}

uint32_t handleWLANCommand(BufferCommandTypedef cmd, uint32_t value)
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

		for(uint32_t i = 0; i < cmdBufferSize; i++)
			rawTx(cmdBuffer[i]); // Can't use txN since its blocked from sending while in command mode
		rawTx('\r'); // End of command character

		cmdBufferSize = 0;
		break;
	}

	return 0;
}

uint32_t getCMDReply()
{
	uint8_t cmdReply;
	uint32_t result = 0;

	for(uint8_t i = 0; i < 4; i++)
	{
		if(rawRx(&cmdReply) == 0)
			cmdReply = 0;
		// First character is in the smallest byte of result
		result |= cmdReply << 24;
		result >>= 8;
	}

	return result;
}
