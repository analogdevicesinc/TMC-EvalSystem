/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices, Inc.),
*
* Copyright © 2023 Analog Devices, Inc.
*******************************************************************************/


#ifndef __UART_H_
#define __UART_H_

#include "RXTX.h"

// Available Eselsbruecke UART pinouts
typedef enum {
    UART_PINS_DIO17_18, // Legacy UART pinout   (UART_TXD = DIO17, UART_RXD = DIO18)
    UART_PINS_DIO10_11  // Standard UART pinout (UART_TXD = DIO10, UART_RXD = DIO11)
} UART_Pins;

typedef enum {
    UART_TXMODE_PUSH_PULL, // Push Pull output, no pull resistor
    UART_TXMODE_OPEN_DRAIN, // Open-Drain output with pull-up resistor
} UART_TXMode;

typedef struct {
    UART_Pins pinout;
    UART_TXMode txMode;
    uint16_t timeout;
    RXTXTypeDef rxtx;
    bool hideSingleWireEcho;
} UART_Config;

extern UART_Config UART;

void UART0_RX_TX_IRQHandler_UART(void);
int32_t UART_readWrite(UART_Config *uart, uint8_t *data, size_t writeLength, uint8_t readLength);
void UART_setEnabled(UART_Config *channel, uint8_t enabled);

// Waits for the write buffer to drain
void UART_flushWriteBuffer(UART_Config *channel);

#endif /* __UART_H_ */
