#ifndef __UART_H_
#define __UART_H_

#include "RXTX.h"
RXTXTypeDef UART;

void UART_readInt(RXTXTypeDef *channel, uint8 slave, uint8 address, int32 *value);
void UART_writeInt(RXTXTypeDef *channel, uint8 slave, uint8 address, int32 value);

#endif /* __UART_H_ */
