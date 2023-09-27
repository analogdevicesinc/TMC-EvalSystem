/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/*
 * StepDir.c
 *
 * This is a basic implementation of a StepDir generator, capable of generating
 * velocity (reaching a target velocity with linear acceleration) and position
 * (reach a target position with linear acc-/decceleration and a maximum velocity)
 * ramps.
 *
 * ***** HOW IT WORKS *****
 *
 * General:
 *   A high frequency (2^17 Hz) Interrupt calculates acceleration, velocity and
 *   position. Position and velocity are calculated with 17 binary decimal places
 *   of precision.
 *
 * Velocity mode:
 *   In velocity mode, the generator will accelerate towards a target velocity.
 *   Acceleration and velocity can be changed at any point of the ramp. The position
 *   is tracked and resettable (useful for setting a reference point via a switch).
 *
 * Position mode:
 *   In position mode, a linearly accelerated ramp is used to reach the position.
 *   Parameters for the ramp are acceleration and maximum velocity. The generator
 *   will always increase the velocity towards the maximum velocity until the
 *   remaining distance to the target is required for the deceleration ramp.
 *   Acceleration, maximum velocity and target position can all be changed during
 *   the ramp. Note that decreasing acceleration or changing target position may
 *   lead to overshooting the target. In that case the generator will start a new
 *   ramp to the target, while always staying within the bounds of acceleration and
 *   velocity.
 *
 *   Due to imprecision in the deceleration distance calculations, a small tolerance
 *   window is used, where the motor will set the velocity to zero if the velocity is
 *   small enough and the position is reached (V_STOP). If the position is barely
 *   missed (HOMING_DISTANCE) and the velocity is zero, the generator will home in
 *   towards the target position at a low velocity (V_STOP).
 *   Changing the actual position value is not possible while in position mode
 *   the generator is not idle (target position reached, velocity zero).
 *   Changing the acceleration to zero is not possible in position mode.
 *   Acceleration value changes require a recalculation of the braking distance.
 *   This can result in more frequent near-misses of the target position, which the
 *   generator will compensate with starting new ramps or homing in (see above).
 *
 *   If overshooting the target by any step is not permitted, it is recommended to
 *   drive to the target without changing parameters during the ramp. Alternatively,
 *   driving to a point shortly before the actual target point and then starting
 *   another small ramp allows for parameter changes during the first ramp, only
 *   requiring a small distance drive with 'locked' parameters.
 *
 *   Overshooting from calculation errors is mostly limited to single digit
 *   position differences. Decreasing acceleration or moving the target position
 *   towards the actual position might result in bigger misses of the target.
 *
 * StallGuard:
 *   The StepDir generator supports the StallGuard feature, either by a input pin
 *   signal or with external monitoring. The function periodicJob() will check,
 *   whether the velocity is above the set StallGuard threshold velocity, set a
 *   status flag (usable for external StallGuard monitoring) and - if present -
 *   check the input pin for indicated stalls.
 *   Make sure that periodicJob() is called frequently to allow quick stall
 *   detection. The function can also be called by an interrupt to guarantee
 *   quick detection [1]. The interrupt should have a lower priority than the
 *   Step-Generator interrupt.
 *
 *   When using external monitoring, for example by checking a chip register,
 *   you can use the STATUS_STALLGUARD_ACTIVE bit of getStatus() to see if
 *   StallGuard is active. In case of a stall, calling stop(STOP_STALL) will
 *   trigger the stall mechanism, shutting down the generator without loosing
 *   further steps.
 *   Clearing a stall condition is done by setting the stall velocity threshold
 *   to any value.
 *   Position mode will start a new ramp towards the target after a stall.
 *
 * Emergency Stop:
 *   The stop function implements an emergency stop. This will result in the
 *   channel immediately stopping any movements. No parameters are updated to
 *   allow for diagnostics. The only way to clear the emergency stop event is
 *   to init() the StepDir generator again [2].
 *
 * ***** LIMITATIONS  *****
 *
 * The frequency of the StepDir generator is limited by the processor. On the
 * Landungsbrücke, the worst case of two motors/channels (TMC2041) is able to
 * still run at 2^17 Hz. Since the bulk of the calculation is per-motor/channel,
 * using a chip with only one motor/channel would allow a frequency of 2^18 Hz.
 * (Note that quite a few calculations have to divide by the frequency, so
 *  choosing a power of two simplifies those to right-shifts.)
 *
 * The limit on Step pulses is one generated pulse per interrupt.
 * The maximum velocity therefore is equal to the interrupt frequency:
 *   Max Velocity: 2^17 pps = 131072 pps
 *
 * Each tick the acceleration value gets added to the velocity accumulator
 * variable (uint32_t). The upper 15 digits are added to the velocity, the lower
 * 17 digits are kept in the accumulator between ticks. The maximum
 * acceleration will result in the upper 15 digits being 1 each tick, increasing
 * the velocity by 32767 (0x7FFF) per tick. The accumulator digits stay unchanged,
 * otherwise the overflow of the lower 17 accumulator digits into the upper 15
 * digits would cause the uint32_t to overflow, loosing an acceleration tick.
 * In other words: The upper 15 digits are 1, the lower 17 digits are 0:
 *   Max Acceleration: 0xFFFE0000 = 4294836224 pps^2
 *
 * A change from lowest to highest (or vice-versa) velocity would take 9 ticks at
 * maximum acceleration:
 *   ceil( (VMAX- (-VMAX)) / AMAX) = ceil(8,000244) = 9
 *
 * ***** Side notes ******
 * [1]: Technically periodicJob() is not interrupt-safe, since it updates the
 *      haltingCondition bitfield. Read-Modify-Write cycles of the main code
 *      could result in the changes of interrupts to the bitfield to be lost.
 *      In practice, the interrupt should just rewrite the stall bit on the next
 *      check though due to the nature of StallGuard.
 * [2]: Emergency stop does not have a graceful recovery method by design.
 *      Clearing the emergency stop via init() will result in all channels
 *      being reset.
 */

#include "StepDir.h"
#include "hal/derivative.h"

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
	#define TIMER_INTERRUPT FTM1_IRQHandler
#elif defined(LandungsbrueckeV3)
	#define TIMER_INTERRUPT TIMER2_IRQHandler
#endif

#define STEP_DIR_CHANNELS 2

// Reset value for stallguard threshold. Since Stallguard is motor/application-specific we can't choose a good value here,
// so this value is rather randomly chosen. Leaving it at zero means stall detection turned off.
#define STALLGUARD_THRESHOLD 0

StepDirectionTypedef StepDir[STEP_DIR_CHANNELS];

IOPinTypeDef DummyPin = { .bitWeight = DUMMY_BITWEIGHT };

// Helper functions
static int32_t calculateStepDifference(int32_t velocity, uint32_t oldAccel, uint32_t newAccel);
// These helper functions are for optimizing the interrupt without duplicating
// logic for both interrupt and main loop. We save time in them by omitting
// safety checks needed only for the main loop in these functions.
// The main loop then uses functions that wrap these functions together with the
// necessary safety checks.
static inline void checkStallguard(StepDirectionTypedef *channel, bool stallSignalActive);
static inline void stop(StepDirectionTypedef *channel, StepDirStop stopType);

void TIMER_INTERRUPT()
{
#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
	FTM1_SC &= ~FTM_SC_TOF_MASK; // clear timer overflow flag
#elif defined(LandungsbrueckeV3)
	if(timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP) == RESET)
		return;
	timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
#endif

	for (uint8_t ch = 0; ch < STEP_DIR_CHANNELS; ch++)
	{
		// Temporary variable for the current channel
		StepDirectionTypedef *currCh = &StepDir[ch];

		// If any halting condition is present, abort immediately
		if (currCh->haltingCondition)
			continue;

		// Reset step output (falling edge of last pulse)

		//*currCh->stepPin->resetBitRegister = currCh->stepPin->bitWeight;
		HAL.IOs->config->setLow(currCh->stepPin);

		// Check if StallGuard pin is high
		// Note: If no stall pin is registered, isStallSignalHigh becomes FALSE
		//       and checkStallguard won't do anything.
#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
		bool isStallSignalHigh = (GPIO_PDIR_REG(currCh->stallGuardPin->GPIOBase) & currCh->stallGuardPin->bitWeight) != 0;
#elif defined(LandungsbrueckeV3)
		bool isStallSignalHigh = HAL.IOs->config->isHigh(currCh->stallGuardPin);
#endif
		checkStallguard(currCh, isStallSignalHigh);

		// Compute ramp
		int32_t dx = tmc_ramp_linear_compute(&currCh->ramp);

		// Step
		if (dx == 0) // No change in position -> skip step generation
			goto skipStep;

		// Direction
		*((dx > 0) ? currCh->dirPin->resetBitRegister : currCh->dirPin->setBitRegister) = currCh->dirPin->bitWeight;

		// Set step output (rising edge of step pulse)
		*currCh->stepPin->setBitRegister = currCh->stepPin->bitWeight;

skipStep:
		// Synchronised Acceleration update
		switch(currCh->syncFlag)
		{
		case SYNC_SNAPSHOT_REQUESTED:
			// Apply the new acceleration
			tmc_ramp_linear_set_acceleration(&currCh->ramp, currCh->newAcceleration);
			// Save a snapshot of the velocity
			currCh->oldVelocity  = tmc_ramp_linear_get_rampVelocity(&currCh->ramp);

			currCh->syncFlag = SYNC_SNAPSHOT_SAVED;
			break;
		case SYNC_UPDATE_DATA:
			currCh->ramp.accelerationSteps += currCh->stepDifference;
			currCh->syncFlag = SYNC_IDLE;
			break;
		default:
			break;
		}
	}
}

void StepDir_rotate(uint8_t channel, int32_t velocity)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	// Set the rampmode first - other way around might cause issues
	tmc_ramp_linear_set_mode(&StepDir[channel].ramp, TMC_RAMP_LINEAR_MODE_VELOCITY);
	switch(StepDir[channel].mode) {
	case STEPDIR_INTERNAL:
		tmc_ramp_linear_set_targetVelocity(&StepDir[channel].ramp, MIN(STEPDIR_MAX_VELOCITY, velocity));
		break;
	case STEPDIR_EXTERNAL:
	default:
		tmc_ramp_linear_set_targetVelocity(&StepDir[channel].ramp, velocity);
		break;
	}
}

void StepDir_moveTo(uint8_t channel, int32_t position)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	tmc_ramp_linear_set_mode(&StepDir[channel].ramp, TMC_RAMP_LINEAR_MODE_POSITION);
	tmc_ramp_linear_set_targetPosition(&StepDir[channel].ramp, position);
}

void StepDir_periodicJob(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	// Check stallguard velocity threshold
	if ((StepDir[channel].stallGuardThreshold != 0) && (abs(tmc_ramp_linear_get_rampVelocity(&StepDir[channel].ramp)) >= StepDir[channel].stallGuardThreshold))
	{
		StepDir[channel].stallGuardActive = true;
	}
	else
	{
		StepDir[channel].stallGuardActive = false;
	}
}

void StepDir_stop(uint8_t channel, StepDirStop stopType)
{
	stop(&StepDir[channel], stopType);
}

uint8_t StepDir_getStatus(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	uint8_t status = StepDir[channel].haltingCondition;

	int32_t targetPosition = tmc_ramp_linear_get_targetPosition(&StepDir[channel].ramp);
	int32_t actualPosition = tmc_ramp_linear_get_rampPosition(&StepDir[channel].ramp);

	status |= (targetPosition == actualPosition) ? STATUS_TARGET_REACHED : 0;
	status |= (StepDir[channel].stallGuardActive) ? STATUS_STALLGUARD_ACTIVE : 0;
	status |= (tmc_ramp_linear_get_mode(&StepDir[channel].ramp) == TMC_RAMP_LINEAR_MODE_VELOCITY) ? STATUS_MODE : 0;

	return status;
}

// Register the pins to be used by a StepDir channel. NULL will leave the pin unchanged
void StepDir_setPins(uint8_t channel, IOPinTypeDef *stepPin, IOPinTypeDef *dirPin, IOPinTypeDef *stallPin)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	if (stepPin)
	{
		if (IS_DUMMY_PIN(stepPin))
		{
			// Set the halting condition before changing the pin
			StepDir[channel].haltingCondition |= STATUS_NO_STEP_PIN;
		}
		StepDir[channel].stepPin = stepPin;
		if (!IS_DUMMY_PIN(stepPin))
		{
			// Clear the halting condition after setting the pin
			StepDir[channel].haltingCondition &= ~STATUS_NO_STEP_PIN;
		}
	}

	if (dirPin)
	{
		if (IS_DUMMY_PIN(dirPin))
		{
			// Set the halting condition before changing the pin
			StepDir[channel].haltingCondition |= STATUS_NO_DIR_PIN;
		}
		StepDir[channel].dirPin = dirPin;
		if (!IS_DUMMY_PIN(dirPin))
		{
			// Clear the halting condition after setting the pin
			StepDir[channel].haltingCondition &= ~STATUS_NO_DIR_PIN;
		}
	}

	if (stallPin)
	{
		StepDir[channel].stallGuardPin = stallPin;
	}
}

void StepDir_stallGuard(uint8_t channel, bool stall)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	checkStallguard(&StepDir[channel], stall);
}

// ===== Setters =====
// The setters are responsible to access their respective variables while keeping the ramp generation stable

// Set actual and target position (Not during an active position ramp)
void StepDir_setActualPosition(uint8_t channel, int32_t actualPosition)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	if (tmc_ramp_linear_get_mode(&StepDir[channel].ramp) == TMC_RAMP_LINEAR_MODE_POSITION)
	{
		// In position mode: If we're not idle -> abort
//		if ((StepDir[channel].actualVelocity != 0) ||
//		   (StepDir[channel].actualPosition != StepDir[channel].targetPosition))
//		{
//			return;
//		}

		// todo CHECK 2: Use a haltingCondition to prevent movement instead of VMAX? (LH)
		// Temporarity set VMAX to 0 to prevent movement between setting actualPosition and targetPosition
//		uint32_t tmp = StepDir[channel].velocityMax;
//		StepDir[channel].velocityMax = 0;

		// Also update target position to prevent movement
		tmc_ramp_linear_set_targetPosition(&StepDir[channel].ramp, actualPosition);
		tmc_ramp_linear_set_rampPosition(&StepDir[channel].ramp, actualPosition);

		// Restore VMAX
//		StepDir[channel].velocityMax = tmp;
	}
	else
	{
		// In velocity mode the position is not relevant so we can just update it without precautions
		tmc_ramp_linear_set_rampPosition(&StepDir[channel].ramp, actualPosition);
	}
}

void StepDir_setAcceleration(uint8_t channel, uint32_t acceleration)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	if (tmc_ramp_linear_get_mode(&StepDir[channel].ramp) == TMC_RAMP_LINEAR_MODE_VELOCITY)
	{	// Velocity mode does not require any special actions
		tmc_ramp_linear_set_acceleration(&StepDir[channel].ramp, acceleration);
		return;
	}

	// Position mode does not allow acceleration 0
	if (acceleration == 0)
		return;

	// Store the old acceleration
	uint32_t oldAcceleration = tmc_ramp_linear_get_acceleration(&StepDir[channel].ramp);

	// Update the acceleration
	tmc_ramp_linear_set_acceleration(&StepDir[channel].ramp, acceleration);

	// If the channel is not halted we need to synchronise with the interrupt
	if (StepDir[channel].haltingCondition == 0)
	{
		// Sync mechanism: store the new acceleration value and request
		// a snapshot from the interrupt
		StepDir[channel].newAcceleration = acceleration;
		StepDir[channel].syncFlag = SYNC_SNAPSHOT_REQUESTED;
		// Wait for the flag update from the interrupt.
		while (ACCESS_ONCE(StepDir[channel].syncFlag) != SYNC_SNAPSHOT_SAVED); // todo CHECK 2: Timeout to prevent deadlock? (LH) #1
	}
	else
	{	// Channel is halted -> access data directly without sync mechanism
		//StepDir[channel].acceleration = acceleration;
		tmc_ramp_linear_set_acceleration(&StepDir[channel].ramp, acceleration);
		StepDir[channel].oldVelocity = tmc_ramp_linear_get_rampVelocity(&StepDir[channel].ramp);
	}

	int32_t stepDifference = calculateStepDifference(StepDir[channel].oldVelocity, oldAcceleration, acceleration);

	if (StepDir[channel].haltingCondition == 0)
	{
		StepDir[channel].stepDifference = stepDifference;
		StepDir[channel].syncFlag = SYNC_UPDATE_DATA;

		// Wait for interrupt to set flag to SYNC_IDLE
		while (ACCESS_ONCE(StepDir[channel].syncFlag) != SYNC_IDLE); // todo CHECK 2: Timeout to prevent deadlock? (LH) #2
	}
	else
	{	// Channel is halted -> access data directly without sync mechanism
		StepDir[channel].ramp.accelerationSteps += stepDifference;
	}
}

void StepDir_setVelocityMax(uint8_t channel, int32_t velocityMax)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	tmc_ramp_linear_set_maxVelocity(&StepDir[channel].ramp, velocityMax);
}

// Set the velocity threshold for active StallGuard. Also reset the stall flag
void StepDir_setStallGuardThreshold(uint8_t channel, int32_t stallGuardThreshold)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	StepDir[channel].stallGuardThreshold = stallGuardThreshold;
	StepDir[channel].haltingCondition &= ~STATUS_STALLED;
}

void StepDir_setMode(uint8_t channel, StepDirMode mode)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	StepDir[channel].mode = mode;

	if (mode == STEPDIR_INTERNAL)
	{
		StepDir_setFrequency(channel, STEPDIR_FREQUENCY);
	}
}

void StepDir_setFrequency(uint8_t channel, uint32_t frequency)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	StepDir[channel].frequency = frequency;
}

void StepDir_setPrecision(uint8_t channel, uint32_t precision)
{
	if (channel >= STEP_DIR_CHANNELS)
		return;

	tmc_ramp_linear_set_precision(&StepDir[channel].ramp, precision);
}

// ===== Getters =====
int32_t StepDir_getActualPosition(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_rampPosition(&StepDir[channel].ramp);
}

int32_t StepDir_getTargetPosition(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_targetPosition(&StepDir[channel].ramp);
}

int32_t StepDir_getActualVelocity(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_rampVelocity(&StepDir[channel].ramp);
}

int32_t StepDir_getTargetVelocity(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_targetVelocity(&StepDir[channel].ramp);
}

uint32_t StepDir_getAcceleration(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_acceleration(&StepDir[channel].ramp);
}

int32_t StepDir_getVelocityMax(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_maxVelocity(&StepDir[channel].ramp);
}

int32_t StepDir_getStallGuardThreshold(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return StepDir[channel].stallGuardThreshold;
}

StepDirMode StepDir_getMode(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return StepDir[channel].mode;
}

uint32_t StepDir_getFrequency(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	return StepDir[channel].frequency;
}

uint32_t StepDir_getPrecision(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return 0;

	return tmc_ramp_linear_get_precision(&StepDir[channel].ramp);
}

int32_t StepDir_getMaxAcceleration(uint8_t channel)
{
	if (channel >= STEP_DIR_CHANNELS)
		return -1;

	if (StepDir[channel].mode == STEPDIR_INTERNAL)
		return STEPDIR_MAX_ACCELERATION;

	// STEPDIR_EXTERNAL -> no limitation from this generator
	return s32_MAX;
}

// ===================

void StepDir_init(uint32_t precision)
{
	if (precision == 0)
	{
		// Use default precision
		precision = STEPDIR_FREQUENCY;
	}

	// StepDir Channel initialisation
	for (uint8_t i = 0; i < STEP_DIR_CHANNELS; i++)
	{
		StepDir[i].oldVelAccu           = 0;
		StepDir[i].oldVelocity          = 0;
		StepDir[i].newAcceleration      = 0;

		// Set the no-pin halting conditions before changing the pins
		// to avoid a race condition with the interrupt
		StepDir[i].haltingCondition     = STATUS_NO_STEP_PIN | STATUS_NO_DIR_PIN;
		StepDir[i].stallGuardPin        = &DummyPin;
		StepDir[i].stepPin              = &DummyPin;
		StepDir[i].dirPin               = &DummyPin;

		StepDir[i].stallGuardThreshold  = STALLGUARD_THRESHOLD;

		StepDir[i].mode                 = STEPDIR_INTERNAL;
		StepDir[i].frequency            = precision;

		tmc_ramp_linear_init(&StepDir[i].ramp);
		tmc_ramp_linear_set_precision(&StepDir[i].ramp, precision);
		tmc_ramp_linear_set_maxVelocity(&StepDir[i].ramp, STEPDIR_DEFAULT_VELOCITY);
		tmc_ramp_linear_set_acceleration(&StepDir[i].ramp, STEPDIR_DEFAULT_ACCELERATION);
	}

	// Chip-specific hardware peripheral initialisation
	#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
		// enable clock for FTM1
		SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

		FTM1_MODE |= FTM_MODE_WPDIS_MASK; // disable write protection, FTM specific register are available

		FTM1_MODE |= FTM_MODE_FTMEN_MASK | FTM_MODE_FAULTM_MASK; //enable interrupt and select all faults

		// Timer frequency = Bus clk frequency / (MOD - CNTIN + 1)
		//     => MOD = (f_bus / f_timer) + CNTIN - 1
		// The datasheet documents the FTM using the system/core clock, but it's
		// actually using the bus clock
		FTM1_CNTIN = 0;
		FTM1_MOD   = (48000000 / precision) - 1;

		// Select Bus clock as clock source, set prescaler divisor to 2^0 = 1,
		// enable timer overflow interrupt
		FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(0) | FTM_SC_TOIE_MASK;

		// set FTM1 interrupt handler
		enable_irq(INT_FTM1-16);
	#elif defined(LandungsbrueckeV3)
		rcu_periph_clock_enable(RCU_TIMER2);
		timer_deinit(TIMER2);

		timer_parameter_struct tps;
		timer_struct_para_init(&tps);

		tps.period = 914;

		timer_init(TIMER2, &tps);
		timer_interrupt_enable(TIMER2, TIMER_INT_UP);
		timer_update_event_enable(TIMER2);
		timer_enable(TIMER2);

		nvic_irq_enable(TIMER2_IRQn, 1, 1);
	#endif
}

void StepDir_deInit()
{
	#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
		// Only disable the module if it has been enabled before
		if (SIM_SCGC6 & SIM_SCGC6_FTM1_MASK)
		{
			// Disable interrupt in FTM module
			FTM1_SC &= ~FTM_SC_TOIE_MASK;

			// Disable the FTM module
			FTM1_MODE &= ~FTM_MODE_FTMEN_MASK;

			// Disable the interrupt
			disable_irq(INT_FTM1-16);

			// Ensure that the module is disabled BEFORE clock gating gets disabled.
			// Without this the processor can crash under heavy FTM interrupt load.
			asm volatile("DMB");

			// Disable clock gating for the FTM module
			SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
			SIM_SCGC6 &= ~SIM_SCGC6_FTM1_MASK;
		}
	#endif
}

// ===== Helper function =====
/* The required calculation to do is the difference of the required
 * amount of steps to reach a given velocity, using two different
 * given accelerations with an evenly accelerated ramp.
 *
 * Calculation:
 *   v1: Start velocity
 *   v2: Target velocity
 *   a:  Acceleration
 *   t:  Time required to reach the target velocity
 *   s:  distance traveled while accelerating
 *
 *   t = (v2 - v1) / a
 * The distance can be calculated with the average velocity v_avrg:
 *   v_avrg = (v2 + v1) / 2
 *   s = v_avrg * t
 *     = (v2 + v1) / 2 * (v2 - v1) / a
 *     = (v2 + v1) * (v2 - v1) / (2*a)
 *     = (v2^2 - v1^2) / (2*a)
 *
 * Our calculation assumes that the starting velocity v1 is zero:
 *   v1 := 0
 *   s = (v2^2 - 0^2) / (2*a)
 *     = v2^2 / (2*a)
 *
 * Calculating velocities with an accumulator results in the velocity
 * being equal or up to 1 below the theoretical velocity (distributed evenly).
 * To reduce the maximum error in the result of the step calculation,
 * the velocity will be increased by 0.5, so that the velocity error
 * will be within [0.5, -0.5).
 *   s = (v+0.5)^2 / (2*a)
 * Change to using integer math:
 *   s = ((v+0.5)*2/2)^2 / (2*a)
 *     = ((v*2 + 1)/2)^2 / (2*a)
 *     = (v*2 + 1)^2 / 2^2 / (2*a)
 *     = (v*2 + 1)^2 / (8a)
 *
 * The result we need is the difference s2 - s1, using a2 and a1 respectively.
 * Only changing the acceleration allows us to reuse most of the calculation:
 * We define
 *   x := (v*2 + 1)^2 / 8
 * so that
 *   s = x/a
 *
 * Variables <=> Formula:
 *   oldAccel: a2
 *   newAccel: a1
 *   velocity: v
 *   oldSteps: s1
 *   newSteps: s2
 */
static int32_t calculateStepDifference(int32_t velocity, uint32_t oldAccel, uint32_t newAccel)
{
	int64_t tmp = velocity;
	tmp = tmp * 2 + 1;
	tmp = (tmp * tmp) / 4;
	tmp = tmp / 2;
	uint32_t oldSteps = tmp / oldAccel;
	uint32_t newSteps = tmp / newAccel;

	return newSteps - oldSteps;
}

// This helper function implements stallguard logic that is used both in the
// interrupt and main loop code. It is used to optimize the interrupt case.
static inline void checkStallguard(StepDirectionTypedef *channel, bool stallSignalActive)
{
	if (channel->stallGuardActive && stallSignalActive)
	{
		stop(channel, STOP_STALL);
	}
}

static inline void stop(StepDirectionTypedef *channel, StepDirStop stopType)
{
	switch(stopType)
	{
	case STOP_NORMAL:
		tmc_ramp_linear_set_targetVelocity(&channel->ramp, 0);
		tmc_ramp_linear_set_mode(&channel->ramp, TMC_RAMP_LINEAR_MODE_VELOCITY);
		break;
	case STOP_EMERGENCY:
		channel->haltingCondition |= STATUS_EMERGENCY_STOP;
		break;
	case STOP_STALL:
		channel->haltingCondition |= STATUS_STALLED;
		tmc_ramp_linear_set_rampVelocity(&channel->ramp, 0);
		channel->ramp.accumulatorVelocity = 0;
		tmc_ramp_linear_set_targetVelocity(&channel->ramp, 0);
		channel->ramp.accelerationSteps = 0;
		break;
	}
}
