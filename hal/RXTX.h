#ifndef RXTX_H_
#define RXTX_H_

#include "tmc/helpers/API_Header.h"

#if defined(Landungsbruecke)
typedef enum {
	UART0_INTERRUPT_UART,
	UART0_INTERRUPT_WLAN
} UART0_Interrupt;
UART0_Interrupt uart0_interrupt;
#endif

typedef struct
{
	void (*init)();
	void (*deInit)(void);
	void (*tx)(uint8 ch);
	uint8 (*rx)(uint8 *ch);
	void (*txN)(uint8 *ch, unsigned char number);
	uint8 (*rxN)(uint8 *ch, unsigned char number);
	void (*clearBuffers)(void);
	uint32 (*bytesAvailable)(void);
	uint32 baudRate;
} RXTXTypeDef;

typedef struct
{
	unsigned int read;
	unsigned int wrote;
	volatile uint8 *buffer;
} BufferingTypeDef;

typedef struct
{
	BufferingTypeDef tx;
	BufferingTypeDef rx;
} RXTXBufferingTypeDef;

#endif /* RXTX_H_ */
