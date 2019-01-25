#ifndef LEDS_H_
	#define LEDS_H_

#ifdef Startrampe
	#define LED_ON()            *HAL.IOs->pins->LED_STAT.resetBitRegister   = HAL.IOs->pins->LED_STAT.bitWeight
	#define LED_OFF()           *HAL.IOs->pins->LED_STAT.setBitRegister     = HAL.IOs->pins->LED_STAT.bitWeight
	#define LED_TOGGLE()        HAL.IOs->pins->LED_STAT.port->ODR           ^= HAL.IOs->pins->LED_STAT.bitWeight

	#define LED_ERROR_ON()      *HAL.IOs->pins->LED_ERROR.resetBitRegister  = HAL.IOs->pins->LED_ERROR.bitWeight
	#define LED_ERROR_OFF()     *HAL.IOs->pins->LED_ERROR.setBitRegister    = HAL.IOs->pins->LED_ERROR.bitWeight
	#define LED_ERROR_TOGGLE()  HAL.IOs->pins->LED_ERROR.port->ODR          ^= HAL.IOs->pins->LED_ERROR.bitWeight
#else
	#define LED_ON()            *HAL.IOs->pins->LED_STAT.resetBitRegister   = HAL.IOs->pins->LED_STAT.bitWeight
	#define LED_OFF()           *HAL.IOs->pins->LED_STAT.setBitRegister     = HAL.IOs->pins->LED_STAT.bitWeight
	#define LED_TOGGLE()        HAL.IOs->pins->LED_STAT.GPIOBase->PTOR      ^= GPIO_PTOR_PTTO(HAL.IOs->pins->LED_STAT.bitWeight)

	#define LED_ERROR_ON()      *HAL.IOs->pins->LED_ERROR.resetBitRegister  = HAL.IOs->pins->LED_ERROR.bitWeight
	#define LED_ERROR_OFF()     *HAL.IOs->pins->LED_ERROR.setBitRegister    = HAL.IOs->pins->LED_ERROR.bitWeight
	#define LED_ERROR_TOGGLE()  HAL.IOs->pins->LED_ERROR.GPIOBase->PTOR     ^= GPIO_PTOR_PTTO(HAL.IOs->pins->LED_ERROR.bitWeight)
#endif

	#include "IOs.h"

	typedef struct
	{
		void (*on)(void);
		void (*off)(void);
		void (*toggle)(void);
	} LEDTypeDef;

	typedef struct
	{
		void (*init)(void);
		LEDTypeDef stat;
		LEDTypeDef error;
	} LEDsTypeDef;

	LEDsTypeDef LEDs;

#endif /* LEDS_H_ */
