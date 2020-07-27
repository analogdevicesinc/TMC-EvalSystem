#include "Board.h"
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ramp/LinearRamp.h"

#define DEFAULT_MOTOR  0
#define TMC4671_MOTORS 1
#define USE_LINEAR_RAMP
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
	uint8_t	  motionMode;
	int32_t   actualVelocityPT1;
	int64_t	  akkuActualVelocity;
	int16_t   actualTorquePT1;
	int64_t   akkuActualTorque;
	int32_t   positionScaler;
} TMinimalMotorConfig;

TMinimalMotorConfig motorConfig[TMC4671_MOTORS];

#ifdef USE_LINEAR_RAMP
	TMC_LinearRamp rampGenerator[TMC4671_MOTORS];
	uint8_t actualMotionMode[TMC4671_MOTORS];
	int32_t lastRampTargetPosition[TMC4671_MOTORS];
	int32_t lastRampTargetVelocity[TMC4671_MOTORS];
#endif

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

#ifdef USE_LINEAR_RAMP
	if (rampGenerator[motor].rampEnabled)
	{
		// switch to velocity motion mode
		tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);

		// set target velocity for ramp generator
		rampGenerator[motor].targetVelocity = velocity;
	}
	else
	{
		// set target velocity directly
		tmc4671_setTargetVelocity(motor, velocity);
	}

	// remember switched motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_VELOCITY;
#else
	// set target velocity directly
	tmc4671_setTargetVelocity(motor, velocity);
#endif

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

#ifdef USE_LINEAR_RAMP
	if (rampGenerator[motor].rampEnabled)
	{
		// switch to position motion mode
		tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

		// set target position for ramp generator
		rampGenerator[motor].targetPosition = position;
	}
	else
	{
		// set target position directly
		tmc4671_setAbsolutTargetPosition(motor, position);
	}

	// remember switched motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;
#else
	// set target position directly
	tmc4671_setAbsolutTargetPosition(motor, position);
#endif

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	// scale position deviation
	int32_t dX = (float)*ticks * (float)POSITION_SCALE_MAX / (float)motorConfig[motor].positionScaler;

#ifdef USE_LINEAR_RAMP
	if (rampGenerator[motor].rampEnabled)
	{
		// switch to position motion mode
		tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

		// set target position for ramp generator
		rampGenerator[motor].targetPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL) + dX;
	}
	else
	{
		// set target position directly
		tmc4671_setRelativeTargetPosition(motor, dX);
	}

	// remember switched motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;
#else
	// set target position directly
	tmc4671_setRelativeTargetPosition(motor, dX);
#endif

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

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].maxVelocity = *value;
#endif
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_LIMIT, *value);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].maxVelocity = *value;
#endif
		}
		break;
	case 11: // acceleration
		if(readWrite == READ)
		{
			*value = (uint32_t) tmc4671_readInt(motor, TMC4671_PID_ACCELERATION_LIMIT);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
#endif
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_ACCELERATION_LIMIT, *value);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
#endif
		}
		break;
#ifdef USE_LINEAR_RAMP
	case 12: // enable velocity ramp
		if(readWrite == READ) {
			*value = rampGenerator[motor].rampEnabled;
		} else if(readWrite == WRITE) {
			rampGenerator[motor].rampEnabled = *value;
		}
		break;
#endif
	case 13: // ramp velocity
		if(readWrite == READ) {
#ifdef USE_LINEAR_RAMP
			if (rampGenerator[motor].rampEnabled)
				*value = rampGenerator[motor].rampVelocity;
			else
				*value = tmc4671_readInt(motor, TMC4671_PID_VELOCITY_TARGET);
#else
			*value = tmc4671_readInt(motor, TMC4671_PID_VELOCITY_TARGET);
#endif
		}   else if (readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;

#ifdef USE_LINEAR_RAMP
	case 51: // ramp position
		if (readWrite == READ) {
			*value = (float)rampGenerator[motor].rampPosition * ((float)motorConfig[motor].positionScaler / (float)POSITION_SCALE_MAX);
		}
		break;
#endif

	case 56: // position scaler
		if (readWrite == READ) {
			*value = motorConfig[motor].positionScaler;
		} else if (readWrite == WRITE) {
			if (*value >= 6) {
				motorConfig[motor].positionScaler = *value;
			}
		}
		break;

	case 174:
		// target position (scaled)
		if(readWrite == READ) {
			*value = (int32_t) ((float)tmc4671_readInt(motor, TMC4671_PID_POSITION_TARGET) * ((float)motorConfig[motor].positionScaler / (float)POSITION_SCALE_MAX));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
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

#ifdef USE_LINEAR_RAMP
			// also update linear ramp during clear of actual position
			if (actualMotionMode[motor] == TMC4671_MOTION_MODE_POSITION)
			{
				rampGenerator[motor].targetPosition = position;
				rampGenerator[motor].rampPosition = position;
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, position);
			}
#endif
		}
		break;
	case 190:
		// target torque [mA] (PIDIN_TARGET_TORQUE scaled)
		if(readWrite == READ) {
			*value = tmc4671_getTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			tmc4671_setTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);

#ifdef USE_LINEAR_RAMP
			// remember switched motion mode by setTargetTorque_mA
			actualMotionMode[motor] = TMC4671_MOTION_MODE_TORQUE;
#endif
		}
		break;
	case 192:
		// target velocity (PIDIN_TARGET_VELOCITY)
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 2);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 251:
		// torque measurement factor
		if(readWrite == READ) {
			*value = motorConfig[motor].torqueMeasurementFactor;
		} else if(readWrite == WRITE) {
			motorConfig[motor].torqueMeasurementFactor = *value;
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
				&(motorConfig[motor].actualInitWaitTime), motorConfig[motor].startVoltage);
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
			int16_t actualCurrentRaw = 	tmc4671_readRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
			if ((actualCurrentRaw > -32000) && (actualCurrentRaw < 32000))
			{
				int32_t actualCurrent = ((int32_t)actualCurrentRaw * (int32_t)motorConfig[motor].torqueMeasurementFactor) / 256;
				motorConfig[motor].actualTorquePT1 = tmc_filterPT1(&motorConfig[motor].akkuActualTorque , actualCurrent, motorConfig[motor].actualTorquePT1, 4, 8);
			}
		}

#ifdef USE_LINEAR_RAMP
		// do velocity / position ramping for every motor
		for (motor = 0; motor < TMC4671_MOTORS; motor++)
		{
			if (rampGenerator[motor].rampEnabled)
			{
				if (actualMotionMode[motor] == TMC4671_MOTION_MODE_POSITION)
				{
					tmc_linearRamp_computeRampPosition(&rampGenerator[motor]);

					// set new target position (only if changed)
					if (rampGenerator[motor].rampPosition != lastRampTargetPosition[motor])
					{
						tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, rampGenerator[motor].rampPosition);
						lastRampTargetPosition[motor] = rampGenerator[motor].rampPosition;
					}
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
			else
			{
				// keep on track if ramp is disabled
				rampGenerator[motor].rampPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
				rampGenerator[motor].rampVelocity = 0;
				rampGenerator[motor].lastdXRest = 0;
			}
		}
#endif
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
		tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, 0x0);
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
		tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, 0x0);
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
		motorConfig[motor].initWaitTime             = 1000;
		motorConfig[motor].startVoltage             = 6000;
		motorConfig[motor].initMode                 = 0;
		motorConfig[motor].torqueMeasurementFactor  = 256;
		motorConfig[motor].actualVelocityPT1		= 0;
		motorConfig[motor].akkuActualVelocity       = 0;
		motorConfig[motor].actualTorquePT1			= 0;
		motorConfig[motor].akkuActualTorque         = 0;
		motorConfig[motor].positionScaler			= POSITION_SCALE_MAX;
	}

	// set default polarity for evaluation board's power stage on init
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_POLARITIES, 0x0);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_SV_CHOP, 0x0);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);

	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_dsADC_MCLK_B, 0x0);

	// set default acceleration and max velocity
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_ACCELERATION_LIMIT, 2000);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_VELOCITY_LIMIT, 4000);

#ifdef USE_LINEAR_RAMP
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
#endif
}
