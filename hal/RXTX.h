#ifndef RXTX_H_
#define RXTX_H_

#include "tmc/helpers/API_Header.h"

#if defined(Landungsbruecke)
typedef enum {
	UART0_INTERRUPT_UART,
	UART0_INTERRUPT_WLAN
} UART0_Interrupt;
extern UART0_Interrupt uart0_interrupt;
#endif

typedef struct
{
	void (*init)();
	void (*deInit)(void);
	void (*tx)(uint8_t ch);
	uint8_t (*rx)(uint8_t *ch);
	void (*txN)(uint8_t *ch, unsigned char number);
	uint8_t (*rxN)(uint8_t *ch, unsigned char number);
	void (*clearBuffers)(void);
	uint32_t (*bytesAvailable)(void);
	uint32_t baudRate;
} RXTXTypeDef;

typedef struct
{
	unsigned int read;
	unsigned int wrote;
	volatile uint8_t *buffer;
} BufferingTypeDef;

typedef struct
{
	BufferingTypeDef tx;
	BufferingTypeDef rx;
} RXTXBufferingTypeDef;

#endif /* RXTX_H_ */
