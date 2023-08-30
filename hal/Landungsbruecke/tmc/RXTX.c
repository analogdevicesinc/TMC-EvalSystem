/*
 * RXTX.c
 *
 *  Created on: 26.02.2019
 *      Author: LK
 */

#include "hal/RXTX.h"
#include "hal/WLAN.h"
#include "hal/UART.h"

UART0_Interrupt uart0_interrupt = UART0_INTERRUPT_UART;

void UART0_RX_TX_IRQHandler(void)
{
	switch(uart0_interrupt) {
	case UART0_INTERRUPT_WLAN:
		UART0_RX_TX_IRQHandler_WLAN();
		break;
	case UART0_INTERRUPT_UART:
	default:
		UART0_RX_TX_IRQHandler_UART();
		break;
	}
}
