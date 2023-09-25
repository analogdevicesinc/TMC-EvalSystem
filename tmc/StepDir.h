/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef STEP_DIR_H_
#define STEP_DIR_H_

	#include "tmc/helpers/API_Header.h"
	#include "tmc/ramp/LinearRamp1.h"

	#include "hal/HAL.h"

	#define STEPDIR_FREQUENCY         (1 << 17)
	#define STEPDIR_MAX_VELOCITY      STEPDIR_FREQUENCY // Limit: 1 Step per interrupt (2^17 Hz) -> 2^17 pps
	#define STEPDIR_MAX_ACCELERATION  2147418111        // Limit: Highest value above accumulator digits (0xFFFE0000).
	                                                    // Any value above would lead to acceleration overflow whenever the accumulator digits overflow

	#define STEPDIR_DEFAULT_ACCELERATION 100000
	#define STEPDIR_DEFAULT_VELOCITY STEPDIR_MAX_VELOCITY

	typedef enum {
		STEPDIR_INTERNAL = 0,
		STEPDIR_EXTERNAL = 1
	} StepDirMode; // Has to be set explicitly here because IDE relies on this number.

	typedef enum {
		STOP_NORMAL,
		STOP_EMERGENCY,
		STOP_STALL
	} StepDirStop;

	typedef enum {
		SYNC_IDLE,                // Sync mechanism not running
		SYNC_SNAPSHOT_REQUESTED,  // Main code saved the new acceleration and is waiting for the interrupt to save the velocity and apply the acceleration (atomically, from the StepDir generator perspective).
		SYNC_SNAPSHOT_SAVED,      // Interrupt saved the velocity
		SYNC_UPDATE_DATA          // Main code calculated an accelerationSteps difference which the interrupt needs to apply.
	} StepDirSync;

	// StepDir status bits
	#define STATUS_EMERGENCY_STOP     0x01  // Halting condition - Emergency Off
	#define STATUS_NO_STEP_PIN        0x02  // Halting condition - No pin set for Step output
	#define STATUS_NO_DIR_PIN         0x04  // Halting condition - No pin set for Direction output
	#define STATUS_STALLED            0x08  // Halting condition - Stall detected (while Stallguard is enabled)
	#define STATUS_TARGET_REACHED     0x10  // Position mode status - target reached
	#define STATUS_STALLGUARD_ACTIVE  0x20  // Stallguard status - Velocity threshold reached, Stallguard enabled
	#define STATUS_MODE               0x40  // 0: Positioning mode, 1: Velocity mode

	typedef struct
	{	// Generic parameters
		uint8_t       haltingCondition;
		// StallGuard
		bool          stallGuardActive;
		int32_t           stallGuardThreshold;
		IOPinTypeDef  *stallGuardPin;
		// StepDir Pins
		IOPinTypeDef  *stepPin;
		IOPinTypeDef  *dirPin;
		// Acceleration updating sync mechanism (see acceleration setter for details)
		StepDirSync   syncFlag; // Synchronisation flag between main code & interrupt
		// Snapshot data
		// Interrupt -> main code
		int32_t       oldVelocity;
		int32_t       oldVelAccu;
		// Main code -> interrupt
		uint32_t      newAcceleration;
		int32_t       stepDifference;
		StepDirMode   mode;
		uint32_t      frequency;

		TMC_LinearRamp ramp;
	} StepDirectionTypedef;

	void StepDir_rotate(uint8_t channel, int32_t velocity);
	void StepDir_moveTo(uint8_t channel, int32_t position);
	void StepDir_periodicJob(uint8_t channel);
	void StepDir_stop(uint8_t channel, StepDirStop stopType);
	uint8_t StepDir_getStatus(uint8_t channel);
	void StepDir_setPins(uint8_t channel, IOPinTypeDef *stepPin, IOPinTypeDef *dirPin, IOPinTypeDef *stallPin);
	void StepDir_stallGuard(uint8_t channel, bool stall);

	// ===== Setters =====
	void StepDir_setActualPosition(uint8_t channel, int32_t actualPosition);
	void StepDir_setAcceleration(uint8_t channel, uint32_t actualAcceleration);
	void StepDir_setVelocityMax(uint8_t channel, int32_t velocityMax);
	void StepDir_setStallGuardThreshold(uint8_t channel, int32_t stallGuardThreshold);
	void StepDir_setMode(uint8_t channel, StepDirMode mode);
	void StepDir_setFrequency(uint8_t channel, uint32_t frequency);
	void StepDir_setPrecision(uint8_t channel, uint32_t precision);
	// ===== Getters =====
	int32_t StepDir_getActualPosition(uint8_t channel);
	int32_t StepDir_getTargetPosition(uint8_t channel);
	int32_t StepDir_getActualVelocity(uint8_t channel);
	int32_t StepDir_getTargetVelocity(uint8_t channel);
	uint32_t StepDir_getAcceleration(uint8_t channel);
	int32_t StepDir_getVelocityMax(uint8_t channel);
	int32_t StepDir_getStallGuardThreshold(uint8_t channel);
	StepDirMode StepDir_getMode(uint8_t channel);
	uint32_t StepDir_getFrequency(uint8_t channel);
	uint32_t StepDir_getPrecision(uint8_t channel);
	int32_t StepDir_getMaxAcceleration(uint8_t channel);

	void StepDir_init(uint32_t precision);
	void StepDir_deInit(void);

#endif /* STEP_DIR_H_ */
