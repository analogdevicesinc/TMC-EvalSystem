/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


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
