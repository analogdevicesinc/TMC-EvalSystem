#ifndef __UART_H_
#define __UART_H_

#include "RXTX.h"

// Switchable UART pin configuration due to pinout changes in TMC2208 v1.2 -> TMC2208 v1.3 aswell as TMC2209
typedef enum {
	UART_PINS_1, // Default UART pinout (<= TMC2208 v1.2, UART_TXD = DIO17, UART_RXD = DIO18)
	UART_PINS_2  // Alternate UART pinout (>= TMC2208 v1.3, UART_TXD = DIO10, UART_RXD = DIO11)
} UART_Pins;

RXTXTypeDef UART;

void UART_init(UART_Pins pinout);
void UART_readInt(RXTXTypeDef *channel, uint8 slave, uint8 address, int32 *value);
void UART_writeInt(RXTXTypeDef *channel, uint8 slave, uint8 address, int32 value);

#endif /* __UART_H_ */
