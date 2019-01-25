#ifndef STEP_DIR_H_
#define STEP_DIR_H_

	#include "tmc/helpers/API_Header.h"
	#include "tmc/ramp/LinearRamp1.h"

	// todo API 2: move higher level StepDir functions to API!? (ED)

	#include "../hal/HAL.h"

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
		uint8         targetReached;
		uint8         haltingCondition;
		// StallGuard
		bool          stallGuardActive;
		int           stallGuardThreshold;
		IOPinTypeDef  *stallGuardPin;
		// StepDir Pins
		IOPinTypeDef  *stepPin;
		IOPinTypeDef  *dirPin;
		// Acceleration updating sync mechanism (see acceleration setter for details)
		StepDirSync   syncFlag; // Synchronisation flag between main code & interrupt
		// Snapshot data
		// Interrupt -> main code
		int32         oldVelocity;
		int32         oldVelAccu;
		// Main code -> interrupt
		uint32        newAcceleration;
		int32         stepDifference;
		StepDirMode   mode;
		uint32        frequency;

		TMC_LinearRamp ramp, ramp_old;
	} StepDirectionTypedef;

	void StepDir_rotate(uint8 channel, int velocity);
	void StepDir_moveTo(uint8 channel, int position);
	void StepDir_periodicJob(uint8 channel);
	void StepDir_stop(uint8 channel, StepDirStop stopType);
	uint8 StepDir_getStatus(uint8 channel);
	void StepDir_setPins(uint8 channel, IOPinTypeDef *stepPin, IOPinTypeDef *dirPin, IOPinTypeDef *stallPin);
	void StepDir_stallGuard(uint8 channel, bool sg);

	// ===== Setters =====
	void StepDir_setActualPosition(uint8 channel, int actualPosition);
	void StepDir_setAcceleration(uint8 channel, uint32 actualAcceleration);
	void StepDir_setVelocityMax(uint8 channel, int velocityMax);
	void StepDir_setStallGuardThreshold(uint8 channel, int stallGuardThreshold);
	void StepDir_setMode(uint8 channel, StepDirMode mode);
	void StepDir_setFrequency(uint8 channel, uint32 frequency);
	// ===== Getters =====
	int StepDir_getActualPosition(uint8 channel);
	int StepDir_getTargetPosition(uint8 channel);
	int StepDir_getActualVelocity(uint8 channel);
	int StepDir_getTargetVelocity(uint8 channel);
	uint32 StepDir_getAcceleration(uint8 channel);
	int StepDir_getVelocityMax(uint8 channel);
	int StepDir_getStallGuardThreshold(uint8 channel);
	StepDirMode StepDir_getMode(uint8 channel);
	uint32 StepDir_getFrequency(uint8 channel);
	int32 StepDir_getMaxAcceleration(uint8 channel);

	void StepDir_init();
	void StepDir_deInit(void);

#endif /* STEP_DIR_H_ */
