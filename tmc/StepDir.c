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
 * variable (uint32). The upper 15 digits are added to the velocity, the lower
 * 17 digits are kept in the accumulator between ticks. The maximum
 * acceleration will result in the upper 15 digits being 1 each tick, increasing
 * the velocity by 32767 (0x7FFF) per tick. The accumulator digits stay unchanged,
 * otherwise the overflow of the lower 17 accumulator digits into the upper 15
 * digits would cause the uint32 to overflow, loosing an acceleration tick.
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
#include "../hal/derivative.h"

#if defined(Startrampe)
	#define TIMER_INTERRUPT TIM2_IRQHandler
#elif defined(Landungsbruecke)
	#define TIMER_INTERRUPT FTM1_IRQHandler
#endif

#define STEP_DIR_CHANNELS 2

// Reset value for stallguard threshold. Since Stallguard is motor/application-specific we can't choose a good value here,
// so this value is rather randomly chosen. Leaving it at zero means stall detection turned off.
#define STALLGUARD_THRESHOLD 0

StepDirectionTypedef StepDir[STEP_DIR_CHANNELS];

IOPinTypeDef DummyPin = { .bitWeight = DUMMY_BITWEIGHT };

int calculateStepDifference(int velocity, uint32 oldAccel, uint32 newAccel);

void TIMER_INTERRUPT()
{
#ifdef Startrampe
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET)
		return;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // clear pending flag
#else
	FTM1_SC &= ~FTM_SC_TOF_MASK; // clear timer overflow flag
#endif

	for(uint8 ch = 0; ch < STEP_DIR_CHANNELS; ch++)
	{
		// Temporary variable for the current channel
		StepDirectionTypedef *currCh = &StepDir[ch];

		// If any halting condition is present, abort immediately
		if(currCh->haltingCondition)
			continue;

		// Reset step output (falling edge of last pulse)
		*currCh->stepPin->resetBitRegister = currCh->stepPin->bitWeight;

		// Compute ramp
		if(currCh)
			tmc_ramp_linear_compute(&currCh->ramp, 1); // delta = 1 => velocity unit: steps/delta-tick

		// Step
		if(tmc_ramp_linear_get_dx(&currCh->ramp) == 0) // No change in position -> skip step generation
			goto skipStep;

		// Direction
		*((tmc_ramp_linear_get_dx(&currCh->ramp) > 0) ? currCh->dirPin->resetBitRegister : currCh->dirPin->setBitRegister) = currCh->dirPin->bitWeight;

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

void StepDir_rotate(uint8 channel, int velocity)
{
	if(channel >= STEP_DIR_CHANNELS)
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

void StepDir_moveTo(uint8 channel, int position)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	tmc_ramp_linear_set_mode(&StepDir[channel].ramp, TMC_RAMP_LINEAR_MODE_POSITION);
	tmc_ramp_linear_set_targetPosition(&StepDir[channel].ramp, position);
}

void StepDir_periodicJob(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	if(!IS_DUMMY_PIN(StepDir[channel].stallGuardPin))
		StepDir_stallGuard(channel, HAL.IOs->config->isHigh(StepDir[channel].stallGuardPin));
}

void StepDir_stop(uint8 channel, StepDirStop stopType)
{
	switch(stopType)
	{
	case STOP_NORMAL:
		tmc_ramp_linear_set_targetVelocity(&StepDir[channel].ramp, 0);
		tmc_ramp_linear_set_mode(&StepDir[channel].ramp, TMC_RAMP_LINEAR_MODE_VELOCITY);
		break;
	case STOP_EMERGENCY:
		StepDir[channel].haltingCondition |= STATUS_EMERGENCY_STOP;
		break;
	case STOP_STALL:
		StepDir[channel].haltingCondition |= STATUS_STALLED;
//		tmc_ramp_linear_set_rampVelocity(&StepDir[channel].ramp, 0);
//		StepDir[channel].ramp.accumulatorVelocity = 0;
//		tmc_ramp_linear_set_targetVelocity(&StepDir[channel].ramp, 0);
//		StepDir[channel].ramp.accelerationSteps = 0;
		break;
	}
}

uint8 StepDir_getStatus(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	uint8 status = StepDir[channel].haltingCondition;

	status |= (StepDir[channel].targetReached) ? STATUS_TARGET_REACHED : 0;
	status |= (StepDir[channel].stallGuardActive) ? STATUS_STALLGUARD_ACTIVE : 0;
	status |= (tmc_ramp_linear_get_mode(&StepDir[channel].ramp) == TMC_RAMP_LINEAR_MODE_VELOCITY) ? STATUS_MODE : 0;

	return status;
}

// Register the pins to be used by a StepDir channel. NULL will leave the pin unchanged
void StepDir_setPins(uint8 channel, IOPinTypeDef *stepPin, IOPinTypeDef *dirPin, IOPinTypeDef *stallPin)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	if(stepPin)
	{
		StepDir[channel].stepPin = stepPin;
		if(!IS_DUMMY_PIN(stepPin))
			StepDir[channel].haltingCondition &= ~STATUS_NO_STEP_PIN;
		else
			StepDir[channel].haltingCondition |= STATUS_NO_STEP_PIN;
	}

	if(dirPin)
	{
		StepDir[channel].dirPin = dirPin;
		if(!IS_DUMMY_PIN(dirPin))
			StepDir[channel].haltingCondition &= ~STATUS_NO_DIR_PIN;
		else
			StepDir[channel].haltingCondition |= STATUS_NO_DIR_PIN;
	}

	if(stallPin)
		StepDir[channel].stallGuardPin = stallPin;
}

void StepDir_stallGuard(uint8 channel, bool sg)
{
	static bool sg_old = false;

	if(channel >= STEP_DIR_CHANNELS)
		return;

	if((StepDir[channel].stallGuardThreshold != 0) && (abs(tmc_ramp_linear_get_rampVelocity(&StepDir[channel].ramp)) >= StepDir[channel].stallGuardThreshold)) {
		StepDir[channel].stallGuardActive = true;
		if((!sg_old) && sg)
			StepDir_stop(channel, STOP_STALL);
	} else {
		StepDir[channel].stallGuardActive = false;
	}
	sg_old = sg;
}

// ===== Setters =====
// The setters are responsible to access their respective variables while keeping the ramp generation stable

// Set actual and target position (Not during an active position ramp)
void StepDir_setActualPosition(uint8 channel, int actualPosition)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	if(tmc_ramp_linear_get_mode(&StepDir[channel].ramp) == TMC_RAMP_LINEAR_MODE_POSITION)
	{
		// In position mode: If we're not idle -> abort
//		if((StepDir[channel].actualVelocity != 0) ||
//		   (StepDir[channel].actualPosition != StepDir[channel].targetPosition))
//		{
//			return;
//		}

		// todo CHECK 2: Use a haltingCondition to prevent movement instead of VMAX? (LH)
		// Temporarity set VMAX to 0 to prevent movement between setting actualPosition and targetPosition
//		uint32 tmp = StepDir[channel].velocityMax;
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

void StepDir_setAcceleration(uint8 channel, uint32 acceleration)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	if(tmc_ramp_linear_get_mode(&StepDir[channel].ramp) == TMC_RAMP_LINEAR_MODE_VELOCITY)
	{	// Velocity mode does not require any special actions
		tmc_ramp_linear_set_acceleration(&StepDir[channel].ramp, acceleration);
		return;
	}

	// Position mode does not allow acceleration 0
	if(acceleration == 0)
		return;

	tmc_ramp_linear_set_acceleration(&StepDir[channel].ramp, acceleration);

	// Store the old acceleration
	uint32 oldAcceleration = tmc_ramp_linear_get_acceleration(&StepDir[channel].ramp);

	// If the channel is not halted we need to synchronise with the interrupt
	if(StepDir[channel].haltingCondition == 0)
	{
		// Sync mechanism: store the new acceleration value and request
		// a snapshot from the interrupt
		StepDir[channel].newAcceleration = acceleration;
		StepDir[channel].syncFlag = SYNC_SNAPSHOT_REQUESTED;
		// Wait for the flag update from the interrupt.
		while(ACCESS_ONCE(StepDir[channel].syncFlag) != SYNC_SNAPSHOT_SAVED); // todo CHECK 2: Timeout to prevent deadlock? (LH) #1
	}
	else
	{	// Channel is halted -> access data directly without sync mechanism
		//StepDir[channel].acceleration = acceleration;
		tmc_ramp_linear_set_acceleration(&StepDir[channel].ramp, acceleration);
		StepDir[channel].oldVelocity = tmc_ramp_linear_get_rampVelocity(&StepDir[channel].ramp);
	}

	int32 stepDifference = calculateStepDifference(StepDir[channel].oldVelocity, oldAcceleration, acceleration);

	if(StepDir[channel].haltingCondition == 0)
	{
		StepDir[channel].stepDifference = stepDifference;
		StepDir[channel].syncFlag = SYNC_UPDATE_DATA;

		// Wait for interrupt to set flag to SYNC_IDLE
		while(ACCESS_ONCE(StepDir[channel].syncFlag) != SYNC_IDLE); // todo CHECK 2: Timeout to prevent deadlock? (LH) #2
	}
	else
	{	// Channel is halted -> access data directly without sync mechanism
		StepDir[channel].ramp.accelerationSteps += stepDifference;
	}
}

void StepDir_setVelocityMax(uint8 channel, int velocityMax)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	tmc_ramp_linear_set_maxVelocity(&StepDir[channel].ramp, velocityMax);
}

// Set the velocity threshold for active StallGuard. Also reset the stall flag
void StepDir_setStallGuardThreshold(uint8 channel, int stallGuardThreshold)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	StepDir[channel].stallGuardThreshold = stallGuardThreshold;
	StepDir[channel].haltingCondition &= ~STATUS_STALLED;
}

void StepDir_setMode(uint8 channel, StepDirMode mode)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	StepDir[channel].mode = mode;

	if(mode == STEPDIR_INTERNAL)
		StepDir_setFrequency(channel, STEPDIR_FREQUENCY);
}

void StepDir_setFrequency(uint8 channel, uint32 frequency)
{
	if(channel >= STEP_DIR_CHANNELS)
		return;

	StepDir[channel].frequency = frequency;
}

// ===== Getters =====
int StepDir_getActualPosition(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_rampPosition(&StepDir[channel].ramp);
}

int StepDir_getTargetPosition(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_targetPosition(&StepDir[channel].ramp);
}

int StepDir_getActualVelocity(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_rampVelocity(&StepDir[channel].ramp);
}

int StepDir_getTargetVelocity(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_targetVelocity(&StepDir[channel].ramp);
}

uint32 StepDir_getAcceleration(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_acceleration(&StepDir[channel].ramp);
}

int StepDir_getVelocityMax(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return tmc_ramp_linear_get_maxVelocity(&StepDir[channel].ramp);
}

int StepDir_getStallGuardThreshold(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return StepDir[channel].stallGuardThreshold;
}

StepDirMode StepDir_getMode(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return StepDir[channel].mode;
}

uint32 StepDir_getFrequency(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	return StepDir[channel].frequency;
}

int32 StepDir_getMaxAcceleration(uint8 channel)
{
	if(channel >= STEP_DIR_CHANNELS)
		return -1;

	switch(StepDir[channel].mode) {
	case STEPDIR_INTERNAL:
		return STEPDIR_MAX_ACCELERATION;
		break;
	case STEPDIR_EXTERNAL:
	default:
		return s32_MAX;
		break;
	}
}

// ===================

void StepDir_init()
{
	// StepDir Channel initialisation
	for(int i = 0; i < STEP_DIR_CHANNELS; i++)
	{
		StepDir[i].oldVelAccu = 0;
		StepDir[i].oldVelocity = 0;
		StepDir[i].newAcceleration = 0;

		// Set the no-pin halting conditions before changing the pins
		// to avoid a race condition with the interrupt
		StepDir[i].haltingCondition       = STATUS_NO_STEP_PIN | STATUS_NO_DIR_PIN;

		StepDir[i].stallGuardPin          = &DummyPin;
		StepDir[i].stepPin                = &DummyPin;
		StepDir[i].dirPin                 = &DummyPin;

		StepDir[i].stallGuardThreshold    = STALLGUARD_THRESHOLD;

		StepDir[i].mode                   = STEPDIR_INTERNAL;
		StepDir[i].frequency              = STEPDIR_FREQUENCY;

		tmc_ramp_linear_init(&StepDir[i].ramp);
		tmc_ramp_linear_set_maxVelocity(&StepDir[i].ramp, STEPDIR_DEFAULT_VELOCITY);
		tmc_ramp_linear_set_acceleration(&StepDir[i].ramp, STEPDIR_DEFAULT_ACCELERATION);
	}

	// Chip-specific hardware peripheral initialisation
	#if defined(Startrampe)
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		// Timer 2 konfigurieren (zum Erzeugen von Geschwindigkeiten)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		TIM_DeInit(TIM2);
		TIM_TimeBaseStructure.TIM_Period         = 457; // for 120MHz clock -> 60MHz
		TIM_TimeBaseStructure.TIM_Prescaler      = 0;
		TIM_TimeBaseStructure.TIM_ClockDivision  = 0;
		TIM_TimeBaseStructure.TIM_CounterMode    = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM2, ENABLE);

		// Timer-Interrupt im NVIC freischalten
		NVIC_InitStructure.NVIC_IRQChannel                    = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	#elif defined(Landungsbruecke)
		// enable clock for FTM1
		SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;

		FTM1_MODE |= FTM_MODE_WPDIS_MASK; // disable write protection, FTM specific register are available

		FTM1_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(0); // Clock source System Clock,divided by 0 Prescaler

		FTM1_MODE |= FTM_MODE_FTMEN_MASK | FTM_MODE_FAULTM_MASK; //enable interrupt and select all faults

		// (MOD - CNTIN + 1) / CLKFrequency = Timer Period
		FTM1_CNTIN = 65535 - 366;

		FTM1_CONF |= FTM_CONF_NUMTOF(0); // The TOF bit is set for each counter overflow

		// Edge-Aligned PWM (EPWM) mode
		FTM1_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK; //FTM_CnSC_CHIE_MASK //

		// enable FTM1 Timer Overflow interrupt
		FTM1_SC |= (uint32) (FTM_SC_TOIE_MASK);

		// set FTM1 interrupt handler
		enable_irq(INT_FTM1-16);
	#endif
}

void StepDir_deInit()
{
	#if defined(Startrampe)
		TIM_DeInit(TIM2);
	#elif defined(Landungsbruecke)
		SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
		SIM_SCGC6 &= ~SIM_SCGC6_FTM1_MASK;
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
int calculateStepDifference(int velocity, uint32 oldAccel, uint32 newAccel)
{
	int64 tmp = velocity;
	tmp = tmp * 2 + 1;
	tmp = (tmp * tmp) / 4;
	tmp = tmp / 2;
	uint32 oldSteps = tmp / oldAccel;
	uint32 newSteps = tmp / newAccel;

	return newSteps - oldSteps;
}
