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

typedef enum {
	MEASURE_ONE_PHASE,
	MEASURE_THREE_PHASES,
} BLDCMeasurementType;

void BLDC_init(BLDCMeasurementType type, uint32_t currentScaling, IOPinTypeDef *hallU, IOPinTypeDef *hallV, IOPinTypeDef *hallW);
void timer_callback(void);

void BLDC_calibrateADCs();

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

void BLDC_setPolePairs(uint8 polePairs);
uint8_t BLDC_getPolePairs();

void BLDC_setOpenloopStepTime(uint16_t stepTime);
uint16_t BLDC_getOpenloopStepTime();

int BLDC_getTargetAngle();
int BLDC_getHallAngle();

void BLDC_setTargetOpenloopVelocity(uint32_t velocity);
uint32_t BLDC_getTargetOpenloopVelocity();
int32_t BLDC_getActualOpenloopVelocity();
int BLDC_getActualHallVelocity();

void BLDC_setHallOrder(uint8_t order);
uint8_t BLDC_getHallOrder();

void BLDC_setHallInvert(uint8_t invert);
uint8_t BLDC_getHallInvert();

void BLDC_setBBMTime(uint8_t time);
uint8_t BLDC_getBBMTime();

#endif /* TMC_BLDC_H_ */
