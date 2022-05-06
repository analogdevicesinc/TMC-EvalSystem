#include "Board.h"
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ramp/LinearRamp.h"
#include "tmc/RAMDebug.h"

#define DEFAULT_MOTOR  0
#define TMC4671_MOTORS 1

#define TORQUE_FLUX_MAX 	(int32_t)10000
#define POSITION_SCALE_MAX  (int32_t)65536

static IOPinTypeDef *PIN_DRV_ENN;
static ConfigurationTypeDef *TMC4671_config;
static SPIChannelTypeDef *TMC4671_SPIChannel;

typedef struct
{
	uint16_t  startVoltage;
	uint16_t  initWaitTime;
	uint16_t  actualInitWaitTime;
	uint8_t   initState;
	uint8_t   initMode;
	uint16_t  torqueMeasurementFactor;  // uint8_t.uint8_t
	uint32_t  maximumCurrent;
	uint8_t	  motionMode;
	int32_t   actualVelocityPT1;
	int64_t	  akkuActualVelocity;
	int16_t   actualTorquePT1;
	int64_t   akkuActualTorque;
	int16_t   actualFluxPT1;
	int64_t   akkuActualFlux;
	int32_t   positionScaler;
	int32_t   linearScaler;
	int16_t   hall_phi_e_old;
	int16_t   hall_phi_e_new;
	int16_t   hall_actual_coarse_offset;
	uint16_t  last_Phi_E_Selection;
	uint32_t  last_UQ_UD_EXT;
	int16_t   last_PHI_E_EXT;
	uint8_t	  enableVelocityFeedForward;
} TMinimalMotorConfig;

static TMinimalMotorConfig motorConfig[TMC4671_MOTORS];

// variables for ramp generator support
TMC_LinearRamp rampGenerator[TMC4671_MOTORS];
uint8_t actualMotionMode[TMC4671_MOTORS];
int32_t lastRampTargetPosition[TMC4671_MOTORS];
int32_t lastRampTargetVelocity[TMC4671_MOTORS];

// Helper functions for unit conversion
static int32_t linearPositionToInternalPosition(int32_t position, int32_t scaler, int polePairs);
static int32_t internalPositionToLinearPosition(int32_t position, int32_t scaler, int polePairs);
static int32_t linearVelocityToInternalVelocity(int32_t velocity, int32_t scaler);
static int32_t internalVelocityToLinearVelocity(int32_t velocity, int32_t scaler);

// => SPI wrapper
uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
	if (motor == DEFAULT_MOTOR)
		return TMC4671_SPIChannel->readWrite(data, lastTransfer);
	else
		return 0;
}
// <= SPI wrapper

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	// switch to velocity motion mode
	tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);

	// set target velocity for ramp generator
	rampGenerator[motor].targetVelocity = velocity;

	// remember switched motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_VELOCITY;

	return TMC_ERROR_NONE;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	return rotate(motor, velocity);
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	return rotate(motor, -velocity);
}

static uint32_t stop(uint8_t motor)
{
	return rotate(motor, 0);
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	// scale target position
	position = (float)position * (float)POSITION_SCALE_MAX / (float)motorConfig[motor].positionScaler;

	// switch to position motion mode
	tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

	// set target position for ramp generator
	rampGenerator[motor].targetPosition = position;

	// remember switched motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	// scale position deviation
	int32_t dX = (float)*ticks * (float)POSITION_SCALE_MAX / (float)motorConfig[motor].positionScaler;

	// switch to position motion mode
	tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

	// set target position for ramp generator
	rampGenerator[motor].targetPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL) + dX;

	// remember switched motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;

	return TMC_ERROR_NONE;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 4: // max velocity
		if(readWrite == READ)
		{
			*value = (uint32_t) tmc4671_readInt(motor, TMC4671_PID_VELOCITY_LIMIT);

			// update also ramp generator value
			rampGenerator[motor].maxVelocity = *value;
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_LIMIT, *value);

			// update also ramp generator value
			rampGenerator[motor].maxVelocity = *value;
		}
		break;

	case 6: // max torque/flux
		if (readWrite == READ)
		{
			//*value = motorConfig[motor].maximumCurrent;
			*value = tmc4671_getTorqueFluxLimit_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		}
		else if (readWrite == WRITE)
		{
			if((*value >= 0) && (*value <= TORQUE_FLUX_MAX))
			{
				motorConfig[motor].maximumCurrent = *value;
				tmc4671_setTorqueFluxLimit_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);
			}
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11: // acceleration
		if(readWrite == READ)
		{
			*value = (uint32_t) tmc4671_readInt(motor, TMC4671_PID_ACCELERATION_LIMIT);

			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_ACCELERATION_LIMIT, *value);

			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
		}
		break;
	case 12: // enable velocity ramp
		if(readWrite == READ) {
			*value = rampGenerator[motor].rampEnabled;
		} else if(readWrite == WRITE) {
			rampGenerator[motor].rampEnabled = *value;
		}
		break;
	case 13: // ramp velocity
		if(readWrite == READ)
		{
			*value = rampGenerator[motor].rampVelocity;
		}   else if (readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;

	case 20: // Linear scaler [µm/rotation]
		if (readWrite == READ)
		{
			*value = motorConfig[motor].linearScaler;
		}
		else
		{
			motorConfig[motor].linearScaler = *value;
		}
		break;
	case 21: // Linear maximum speed [µm/s]
		if (readWrite == READ)
		{
			int velocity = (uint32_t) tmc4671_readInt(motor, TMC4671_PID_VELOCITY_LIMIT);

			// update also ramp generator value
			rampGenerator[motor].maxVelocity = velocity;

			// Internal -> linear
			*value = internalVelocityToLinearVelocity(velocity, motorConfig[motor].linearScaler);
		}
		else
		{
			// Linear -> Internal
			tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_LIMIT, linearVelocityToInternalVelocity(*value, motorConfig[motor].linearScaler));

			// Also update ramp generator value
			rampGenerator[motor].maxVelocity = *value;
		}
		break;
	case 22: // Linear acceleration [µm/s/s]
		if (readWrite == READ)
		{
			*value = (uint32_t) tmc4671_readInt(motor, TMC4671_PID_ACCELERATION_LIMIT);

			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;

			// Internal -> linear
			*value = internalVelocityToLinearVelocity(*value, motorConfig[motor].linearScaler);
		}
		else
		{
			// Linear -> internal
			tmc4671_writeInt(motor, TMC4671_PID_ACCELERATION_LIMIT, linearVelocityToInternalVelocity(*value, motorConfig[motor].linearScaler));

			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
		}
		break;
	case 23: // Linear ramp velocity [µm/s]
		if (readWrite == READ)
		{
			*value = rampGenerator[motor].rampVelocity;

			// Internal -> linear
			*value = internalVelocityToLinearVelocity(*value, motorConfig[motor].linearScaler);
		}
		break;
	case 24: // Linear ramp position [µm]
		if (readWrite == READ)
		{
			// Internal -> linear
			int polePairs = TMC4671_FIELD_READ(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, TMC4671_N_POLE_PAIRS_MASK, TMC4671_N_POLE_PAIRS_SHIFT);
			*value = internalPositionToLinearPosition(rampGenerator[motor].rampPosition, motorConfig[motor].linearScaler, polePairs);
		}
		break;
	case 25: // Linear target position [µm]
		if (readWrite == READ)
		{
			// Internal -> linear
			int polePairs = TMC4671_FIELD_READ(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, TMC4671_N_POLE_PAIRS_MASK, TMC4671_N_POLE_PAIRS_SHIFT);
			*value = internalPositionToLinearPosition(tmc4671_readInt(motor, TMC4671_PID_POSITION_TARGET), motorConfig[motor].linearScaler, polePairs);
		}
		else
		{
			int polePairs = TMC4671_FIELD_READ(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, TMC4671_N_POLE_PAIRS_MASK, TMC4671_N_POLE_PAIRS_SHIFT);
			int position = linearPositionToInternalPosition(*value, motorConfig[motor].linearScaler, polePairs);

			// Switch to position motion mode
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

			// Set target position for ramp generator
			rampGenerator[motor].targetPosition = position;

			// Remember switched motion mode
			actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;
		}
		break;
	case 26: // Linear actual velocity [µm/s]
		if (readWrite == READ)
		{
			int velocity = motorConfig[motor].actualVelocityPT1;

			// Internal -> linear
			*value = internalVelocityToLinearVelocity(velocity, motorConfig[motor].linearScaler);
		}
		break;
	case 27: // Linear actual position [µm]
		if (readWrite == READ)
		{
			int position = tmc4671_getActualPosition(motor);

			// Internal -> linear
			int polePairs = TMC4671_FIELD_READ(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, TMC4671_N_POLE_PAIRS_MASK, TMC4671_N_POLE_PAIRS_SHIFT);
			*value = internalPositionToLinearPosition(position, motorConfig[motor].linearScaler, polePairs);
		}
		break;
	case 28: // Linear target velocity [µm/s]
		if (readWrite == READ)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 2);
			int velocity = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);

			// Internal -> linear
			*value = internalVelocityToLinearVelocity(velocity, motorConfig[motor].linearScaler);
		}
		else
		{
			// Switch to velocity motion mode
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);

			// Set target velocity for ramp generator
			// Linear -> internal
			rampGenerator[motor].targetVelocity = linearVelocityToInternalVelocity(*value, motorConfig[motor].linearScaler);

			// Remember switched motion mode
			actualMotionMode[motor] = TMC4671_MOTION_MODE_VELOCITY;
		}
		break;

	case 51: // ramp position
		if (readWrite == READ) {
			*value = (float)rampGenerator[motor].rampPosition * ((float)motorConfig[motor].positionScaler / (float)POSITION_SCALE_MAX);
		}
		break;
	case 52: // feed forward
		if (readWrite == READ) {
			*value = motorConfig[motor].enableVelocityFeedForward;
		} else if (readWrite == WRITE) {
			if ((*value == 0) || (*value == 1))
			{
				motorConfig[motor].enableVelocityFeedForward = *value;
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;

	case 56: // position scaler
		if (readWrite == READ) {
			*value = motorConfig[motor].positionScaler;
		} else if (readWrite == WRITE) {
			if (*value >= 6) {
				motorConfig[motor].positionScaler = *value;
			}
		}
		break;

	case 70: // Torque P
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_TORQUE_P_TORQUE_I, TMC4671_PID_TORQUE_P_MASK, TMC4671_PID_TORQUE_P_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_P_TORQUE_I, TMC4671_PID_TORQUE_P_MASK, TMC4671_PID_TORQUE_P_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 71: // Torque I
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_TORQUE_P_TORQUE_I, TMC4671_PID_TORQUE_I_MASK, TMC4671_PID_TORQUE_I_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_TORQUE_P_TORQUE_I, TMC4671_PID_TORQUE_I_MASK, TMC4671_PID_TORQUE_I_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 72: // Flux P
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_FLUX_P_FLUX_I, TMC4671_PID_FLUX_P_MASK, TMC4671_PID_FLUX_P_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_FLUX_P_FLUX_I, TMC4671_PID_FLUX_P_MASK, TMC4671_PID_FLUX_P_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 73: // Flux I
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_FLUX_P_FLUX_I, TMC4671_PID_FLUX_I_MASK, TMC4671_PID_FLUX_I_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_FLUX_P_FLUX_I, TMC4671_PID_FLUX_I_MASK, TMC4671_PID_FLUX_I_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 74: // Velocity P
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, TMC4671_PID_VELOCITY_P_MASK, TMC4671_PID_VELOCITY_P_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, TMC4671_PID_VELOCITY_P_MASK, TMC4671_PID_VELOCITY_P_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 75: // Velocity I
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, TMC4671_PID_VELOCITY_I_MASK, TMC4671_PID_VELOCITY_I_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, TMC4671_PID_VELOCITY_I_MASK, TMC4671_PID_VELOCITY_I_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 76: // Position P
		if (readWrite == READ)
		{
			*value = (int16_t) TMC4671_FIELD_READ(motor, TMC4671_PID_POSITION_P_POSITION_I, TMC4671_PID_POSITION_P_MASK, TMC4671_PID_POSITION_P_SHIFT);
		}
		else
		{
			if (*value >= 0 && *value < 32768)
			{
				TMC4671_FIELD_UPDATE(motor, TMC4671_PID_POSITION_P_POSITION_I, TMC4671_PID_POSITION_P_MASK, TMC4671_PID_POSITION_P_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;

// add biquad settings here!

	case 174:
		// target position (scaled)
		if(readWrite == READ) {
			*value = (int32_t) ((float)tmc4671_readInt(motor, TMC4671_PID_POSITION_TARGET) * ((float)motorConfig[motor].positionScaler / (float)POSITION_SCALE_MAX));
		} else if(readWrite == WRITE) {
			// scale target position
			int position = (float)*value * (float)POSITION_SCALE_MAX / (float)motorConfig[motor].positionScaler;

			// switch to position motion mode
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

			// set target position for ramp generator
			rampGenerator[motor].targetPosition = position;

			// remember switched motion mode
			actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;
		}
		break;
	case 176:
		// actual torque [mA] (PID_TORQUE_ACTUAL scaled)
		if(readWrite == READ) {
			*value = motorConfig[motor].actualTorquePT1;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 177:
		// actual flux [mA] (PID_FLUX_ACTUAL scaled)
		if(readWrite == READ) {
			*value = motorConfig[motor].actualFluxPT1;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 178:
		// actual velocity
		if(readWrite == READ) {
			*value = motorConfig[motor].actualVelocityPT1;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 179:
		// actual position (scaled)
		if(readWrite == READ) {
			*value = (int32_t) ((float)tmc4671_getActualPosition(motor) * ((float)motorConfig[motor].positionScaler / (float)POSITION_SCALE_MAX));
		}
		else if(readWrite == WRITE)
		{
			// scale position
			int32_t position = (float)*value * ((float)POSITION_SCALE_MAX / (float)motorConfig[motor].positionScaler);

			// update actual position
			tmc4671_writeInt(motor, TMC4671_PID_POSITION_ACTUAL, position);

			// also update linear ramp during clear of actual position
			if (actualMotionMode[motor] == TMC4671_MOTION_MODE_POSITION)
			{
				rampGenerator[motor].targetPosition = position;
				rampGenerator[motor].rampPosition = position;
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, position);
			}
		}
		break;
	case 181: // Actual torque (unfiltered)
		if(readWrite == READ)
		{
			int16_t actualCurrentRaw = tmc4671_readRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
			*value = ((int32_t)actualCurrentRaw * (int32_t)motorConfig[motor].torqueMeasurementFactor) / 256;
		}
		else if(readWrite == WRITE)
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 182: // Actual velocity (unfiltered)
		if (readWrite == READ)
		{
			*value = tmc4671_getActualVelocity(motor);
		}
		else
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;

	case 190:
		// target torque [mA] (PIDIN_TARGET_TORQUE scaled)
		if(readWrite == READ) {
			*value = tmc4671_getTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			tmc4671_setTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);

			// remember switched motion mode by setTargetTorque_mA
			actualMotionMode[motor] = TMC4671_MOTION_MODE_TORQUE;
		}
		break;
	case 191:
		// target flux [mA] (PIDIN_TARGET_FLUX scaled)
		if(readWrite == READ) {
			*value = tmc4671_getTargetFlux_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_TORQUE);

			tmc4671_setTargetFlux_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);

			// remember switched motion mode by setTargetTorque_mA
			actualMotionMode[motor] = TMC4671_MOTION_MODE_TORQUE;
		}
		break;
	case 192:
		// target velocity (PIDIN_TARGET_VELOCITY)
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 2);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		}
		else if(readWrite == WRITE)
		{
			// switch to velocity motion mode
			tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);

			// set target velocity for ramp generator
			rampGenerator[motor].targetVelocity = *value;

			// remember switched motion mode
			actualMotionMode[motor] = TMC4671_MOTION_MODE_VELOCITY;
		}
		break;
	case 251:
		// torque measurement factor
		if(readWrite == READ) {
			*value = motorConfig[motor].torqueMeasurementFactor;
		} else if(readWrite == WRITE) {
			motorConfig[motor].torqueMeasurementFactor = *value;
			// update max torque/flux according new torque measurement factor
			tmc4671_setTorqueFluxLimit_mA(motor, motorConfig[motor].torqueMeasurementFactor, motorConfig[motor].maximumCurrent);
		}
		break;
	case 252:
		// start encoder initialization
		if(readWrite == READ) {
			*value = motorConfig[motor].initMode;
		} else if(readWrite == WRITE) {
			tmc4671_startEncoderInitialization(*value, &motorConfig[motor].initMode, &motorConfig[motor].initState);
		}
		break;
	case 253:
		// encoder init state
		if(readWrite == READ) {
			*value = motorConfig[motor].initState;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 254:
		// actual encoder wait time
		if(readWrite == READ) {
			*value = motorConfig[motor].actualInitWaitTime;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static int32_t linearPositionToInternalPosition(int32_t linearPosition, int32_t scaler, int polePairs)
{
	// Conversion calculation:
	// scaler    = [µm / mechanicalRotation]
	// polePairs = [electricalRotation / mechanicalRotation]
	// internal  = [1/65536 * electricalRotation]
	//
	// linear                              = [µm]
	// linear / scaler                     = [mechanicalRotation]
	// linear / scaler * polePairs         = [electricalRotation]
	// linear / scaler * polePairs * 65536 = [1/65536 * electricalRotation]
	//
	// Do the multiplications first to minimize rounding errors:
	return ((int64_t) linearPosition) * 65536 * polePairs / scaler;
}

static int32_t internalPositionToLinearPosition(int32_t internalPosition, int32_t scaler, int polePairs)
{
	// Conversion calculation:
	// polePairs = [electricalRotation / mechanicalRotation]
	// scaler    = [µm / mechanicalRotation]
	// linear    = [µm]
	//
	// internal                              = [1/65536 * electricalRotation]
	// internal / 65536                      = [electricalRotation]
	// internal / 65536 / polePairs          = [mechanicalRotation]
	// internal / 65536 / polePairs * scaler = [µm]
	//
	// Do the multiplication first to minimize rounding errors:
	return ((int64_t) internalPosition) * scaler / 65536 / polePairs;
}

static int32_t linearVelocityToInternalVelocity(int32_t linearVelocity, int32_t scaler)
{
	// Conversion calculation
	// scaler    = [µm / mechanicalRotation]
	// time:       [60s / minute]
	// internal  = [mechanicalRotation / minute]
	//
	// linear               = [µm / s]
	// linear / scaler      = [mechanicalRotation / s]
	// linear / scaler * 60 = [mechanicalRotation / minute]
	//
	// Do the multiplication first to minimize rounding errors:
	return ((int64_t) linearVelocity) * 60 / scaler;
}

static int32_t internalVelocityToLinearVelocity(int32_t internalVelocity, int32_t scaler)
{
	// Conversion calculation
	// scaler    = [µm / mechanicalRotation]
	// time:       [60s / minute]
	// linear:   = [µm / s]
	//
	// internal               = [mechanicalRotation / minute]
	// internal / 60          = [mechanicalRotation / s]
	// internal / 60 * scaler = [µm / s]
	//
	// Do the multiplication first to minimize rounding errors:
	return ((int64_t) internalVelocity) * scaler / 60;
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = tmc4671_getActualVelocity(motor);

	return TMC_ERROR_NONE;
}

static void periodicJob(uint32_t actualSystick)
{
	int32_t motor;

	// do encoder initialization if necessary
	for(motor = 0; motor < TMC4671_MOTORS; motor++)
	{
		tmc4671_periodicJob(motor, actualSystick, motorConfig[motor].initMode,
				&(motorConfig[motor].initState), motorConfig[motor].initWaitTime,
				&(motorConfig[motor].actualInitWaitTime), motorConfig[motor].startVoltage,
				&(motorConfig[motor].hall_phi_e_old), &(motorConfig[motor].hall_phi_e_new), &(motorConfig[motor].hall_actual_coarse_offset),
				&(motorConfig[motor].last_Phi_E_Selection), &(motorConfig[motor].last_UQ_UD_EXT), &(motorConfig[motor].last_PHI_E_EXT));
	}

	// 1ms velocity ramp handling
	static uint32_t lastSystick;
	if (lastSystick != actualSystick)
	{
		for(motor = 0; motor < TMC4671_MOTORS; motor++)
		{
			// filter actual velocity
			motorConfig[motor].actualVelocityPT1 = tmc_filterPT1(&motorConfig[motor].akkuActualVelocity, tmc4671_getActualVelocity(motor), motorConfig[motor].actualVelocityPT1, 3, 8);

			// filter actual current
			int16_t actualCurrentRaw = TMC4671_FIELD_READ(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL, TMC4671_PID_TORQUE_ACTUAL_MASK, TMC4671_PID_TORQUE_ACTUAL_SHIFT);
			if ((actualCurrentRaw > -32000) && (actualCurrentRaw < 32000))
			{
				int32_t actualCurrent = ((int32_t)actualCurrentRaw * (int32_t)motorConfig[motor].torqueMeasurementFactor) / 256;
				motorConfig[motor].actualTorquePT1 = tmc_filterPT1(&motorConfig[motor].akkuActualTorque , actualCurrent, motorConfig[motor].actualTorquePT1, 4, 8);
			}

			// filter actual flux
			int16_t actualFluxRaw = TMC4671_FIELD_READ(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL, TMC4671_PID_FLUX_ACTUAL_MASK, TMC4671_PID_FLUX_ACTUAL_SHIFT);
			if ((actualFluxRaw > -32000) && (actualFluxRaw < 32000))
			{
				int32_t actualFlux = ((int32_t)actualFluxRaw * (int32_t)motorConfig[motor].torqueMeasurementFactor) / 256;
				motorConfig[motor].actualFluxPT1 = tmc_filterPT1(&motorConfig[motor].akkuActualFlux , actualFlux, motorConfig[motor].actualFluxPT1, 2, 8);
			}
		}

		// do velocity / position ramping for every motor
		for (motor = 0; motor < TMC4671_MOTORS; motor++)
		{
			if (actualMotionMode[motor] == TMC4671_MOTION_MODE_POSITION)
			{
				tmc_linearRamp_computeRampPosition(&rampGenerator[motor]);

				// set new target position (only if changed)
				if (rampGenerator[motor].rampPosition != lastRampTargetPosition[motor])
				{
					tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, rampGenerator[motor].rampPosition);
					lastRampTargetPosition[motor] = rampGenerator[motor].rampPosition;

					// use velocity feed forward
					tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_OFFSET, (motorConfig[motor].enableVelocityFeedForward) ? rampGenerator[motor].rampVelocity : 0);
				}

				// sync ramp velocity by PIDIN_TARGET_VELOCITY if ramp is disabled
				if (!rampGenerator[motor].rampEnabled)
					rampGenerator[motor].rampVelocity = tmc4671_readFieldWithDependency(motor, TMC4671_INTERIM_DATA, TMC4671_INTERIM_ADDR, 2, TMC4671_PIDIN_TARGET_VELOCITY_MASK, TMC4671_PIDIN_TARGET_VELOCITY_SHIFT);
			}
			else if (actualMotionMode[motor] == TMC4671_MOTION_MODE_VELOCITY)
			{
				tmc_linearRamp_computeRampVelocity(&rampGenerator[motor]);

				// set new target velocity (only if changed)
				if (rampGenerator[motor].rampVelocity != lastRampTargetVelocity[motor])
				{
					// set new target velocity
					tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, rampGenerator[motor].rampVelocity);
					lastRampTargetVelocity[motor] = rampGenerator[motor].rampVelocity;

					// turn of velocity feed forward
					tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_OFFSET, 0);
				}

				// keep position ramp and target position on track
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL));
				rampGenerator[motor].rampPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
				rampGenerator[motor].lastdXRest = 0;
			}
			else if (actualMotionMode[motor] == TMC4671_MOTION_MODE_TORQUE)
			{
				// keep position ramp and target position on track
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL));
				rampGenerator[motor].rampPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
				rampGenerator[motor].rampVelocity = tmc4671_getActualVelocity(motor);
				rampGenerator[motor].lastdXRest = 0;
			}
		}

		debug_nextProcess();
		debug_process();
		lastSystick = actualSystick;
	}
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(motor);
	tmc4671_writeInt(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc4671_readInt(DEFAULT_MOTOR, address);
}

static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return 0;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setLow(PIN_DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setHigh(PIN_DRV_ENN);
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->setLow(PIN_DRV_ENN);
};

static uint8_t reset()
{
	// set default polarity for evaluation board's power stage after VW reset
	for(size_t motor = 0; motor < TMC4671_MOTORS; motor++)
	{
		tmc4671_writeInt(motor, TMC4671_PWM_POLARITIES, 0);
		tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, TMC4671_PWM_SV_MASK);
		tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
	}

	return 1;
}

static uint8_t restore()
{
	// set default polarity for evaluation board's power stage after VW reset
	for(size_t motor = 0; motor < TMC4671_MOTORS; motor++)
	{
		tmc4671_writeInt(motor, TMC4671_PWM_POLARITIES, 0);
		tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, TMC4671_PWM_SV_MASK);
		tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
	}

	//enableDriver(DRIVER_DISABLE);
	return 1;
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

void TMC4671_init(void)
{
	// configure ENABLE-PIN for TMC4671
	PIN_DRV_ENN = &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	enableDriver(DRIVER_ENABLE);

	TMC4671_SPIChannel = &HAL.SPI->ch1;
	TMC4671_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	TMC4671_config = Evalboards.ch1.config;

	// connect evalboard functions
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_READY;
	Evalboards.ch1.config->configIndex  = 0;
	Evalboards.ch1.rotate               = rotate;
	Evalboards.ch1.right                = right;
	Evalboards.ch1.left                 = left;
	Evalboards.ch1.stop                 = stop;
	Evalboards.ch1.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch1.GAP                  = GAP;
	Evalboards.ch1.SAP                  = SAP;
	Evalboards.ch1.moveTo               = moveTo;
	Evalboards.ch1.moveBy               = moveBy;
	Evalboards.ch1.writeRegister        = writeRegister;
	Evalboards.ch1.readRegister         = readRegister;
	Evalboards.ch1.periodicJob          = periodicJob;
	Evalboards.ch1.userFunction         = userFunction;
	Evalboards.ch1.enableDriver         = enableDriver;
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = TMC4671_MOTORS;
	Evalboards.ch1.deInit               = deInit;
	Evalboards.ch1.VMMin                = 70;
	Evalboards.ch1.VMMax                = 650;

	// init motor config
	int32_t motor;
	for(motor = 0; motor < TMC4671_MOTORS; motor++)
	{
		motorConfig[motor].initWaitTime             	= 1000;
		motorConfig[motor].startVoltage             	= 6000;
		motorConfig[motor].initMode                 	= 0;
		motorConfig[motor].hall_phi_e_old				= 0;
		motorConfig[motor].hall_phi_e_new				= 0;
		motorConfig[motor].hall_actual_coarse_offset	= 0;
		motorConfig[motor].last_Phi_E_Selection			= 0;
		motorConfig[motor].last_UQ_UD_EXT				= 0;
		motorConfig[motor].last_PHI_E_EXT				= 0;
		motorConfig[motor].torqueMeasurementFactor  	= 256;
		motorConfig[motor].maximumCurrent				= 1000;
		motorConfig[motor].actualVelocityPT1			= 0;
		motorConfig[motor].akkuActualVelocity       	= 0;
		motorConfig[motor].actualTorquePT1				= 0;
		motorConfig[motor].akkuActualTorque         	= 0;
		motorConfig[motor].positionScaler				= POSITION_SCALE_MAX;
		motorConfig[motor].enableVelocityFeedForward 	= true;
		motorConfig[motor].linearScaler             	= 30000; // µm / rotation
	}

	// set default polarity for evaluation board's power stage on init
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_POLARITIES, 0x0);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_SV_CHOP, TMC4671_PWM_SV_MASK);	// enable space vector PWM by default
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);

	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_dsADC_MCLK_B, 0x0);

	// set default acceleration and max velocity
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_ACCELERATION_LIMIT, 2000);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_VELOCITY_LIMIT, 4000);

	// set default max torque/flux
	tmc4671_setTorqueFluxLimit_mA(DEFAULT_MOTOR, motorConfig[DEFAULT_MOTOR].torqueMeasurementFactor, motorConfig[DEFAULT_MOTOR].maximumCurrent);

	// init ramp generator
	for (motor = 0; motor < TMC4671_MOTORS; motor++)
	{
		tmc_linearRamp_init(&rampGenerator[motor]);
		actualMotionMode[motor] = TMC4671_MOTION_MODE_STOPPED;
		lastRampTargetPosition[motor] = 0;
		lastRampTargetVelocity[motor] = 0;

		// update ramp generator default values
		rampGenerator[motor].maxVelocity = (uint32_t)tmc4671_readInt(motor, TMC4671_PID_VELOCITY_LIMIT);
		rampGenerator[motor].acceleration = (uint32_t)tmc4671_readInt(motor, TMC4671_PID_ACCELERATION_LIMIT);
	}
}
