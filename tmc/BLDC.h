/*
 * BLDC.h
 *
 *  Created on: 14 Apr 2020
 *      Author: LH
 */

#ifndef TMC_BLDC_H_
#define TMC_BLDC_H_

#include "tmc/helpers/API_Header.h"
#include "hal/derivative.h"
#include "hal/HAL.h"

void BLDC_init(IOPinTypeDef *hallU, IOPinTypeDef *hallV, IOPinTypeDef *hallW);

void BLDC_enablePWM(uint8_t enable);
uint8_t BLDC_isPWMenabled();

void BLDC_setTargetPWM(int16_t pwm);
int16_t BLDC_getTargetPWM();

int BLDC_getMeasuredCurrent();

typedef enum {
	BLDC_OPENLOOP,
	BLDC_HALL,
} BLDCMode;

void BLDC_setCommutationMode(BLDCMode mode);
BLDCMode BLDC_getCommutationMode();

void BLDC_setOpenloopStepTime(uint16_t stepTime);
uint16_t BLDC_getOpenloopStepTime();

int BLDC_getTargetAngle();
int BLDC_getHallAngle();

void BLDC_setHallOrder(uint8_t order);
uint8_t BLDC_getHallOrder();

void BLDC_setHallInvert(uint8_t invert);
uint8_t BLDC_getHallInvert();

#endif /* TMC_BLDC_H_ */
