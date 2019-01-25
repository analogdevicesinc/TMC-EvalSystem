#ifndef _HAL_H_
#define _HAL_H_

	#include "derivative.h"
	#include "IOs.h"
	#include "IOMap.h"
	#include "SPI.h"
	#include "ADCs.h"
	#include "USB.h"
	#include "LEDs.h"
	#include "RS232.h"
	#include "WLAN.h"
	#include "Timer.h"
	#include "SysTick.h"
	#include "UART.h"

	typedef struct
	{
		IOsTypeDef       *config;
		IOPinMapTypeDef  *pins;
	} IOsFunctionsTypeDef;

	typedef struct
	{
		void                       (*init) (void);
		void                       (*reset) (uint8 ResetPeripherals);
		void                       (*NVIC_DeInit)(void);
		const IOsFunctionsTypeDef  *IOs;
		SPITypeDef                 *SPI;
		RXTXTypeDef                *USB;
		LEDsTypeDef                *LEDs;
		ADCTypeDef                 *ADCs;
		RXTXTypeDef                *RS232;
		RXTXTypeDef                *WLAN;
		TimerTypeDef               *Timer;
		RXTXTypeDef                *UART;
	} HALTypeDef;

	const HALTypeDef HAL;

	uint8 hwid;

#endif /* _HAL_H_ */
