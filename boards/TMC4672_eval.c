#include "Board.h"
#include "tmc/ic/TMC4672/TMC4672.h"
#include "tmc/ramp/LinearRamp.h"

#define DEFAULT_MOTOR  0
#define TMC4672_MOTORS 1

static IOPinTypeDef *PIN_DRV_ENN;
static ConfigurationTypeDef *TMC4672_config;
static SPIChannelTypeDef *TMC4672_SPIChannel;

typedef struct
{
	u16  startVoltage;
	u16  initWaitTime;
	u16  actualInitWaitTime;
	u8   initState;
	u8   initMode;
	u16  torqueMeasurementFactor;  // u8.u8
	u8	 motionMode;
} TMinimalMotorConfig;

TMinimalMotorConfig motorConfig[TMC4672_MOTORS];

// => SPI wrapper
u8 tmc4672_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
	if (motor == DEFAULT_MOTOR)
		return TMC4672_SPIChannel->readWrite(data, lastTransfer);
	else
		return 0;
}
// <= SPI wrapper

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= TMC4672_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc4672_setTargetVelocity(motor, velocity);

	return TMC_ERROR_NONE;
}

static uint32 right(uint8 motor, int32 velocity)
{
	return rotate(motor, velocity);
}

static uint32 left(uint8 motor, int32 velocity)
{
	return rotate(motor, -velocity);
}

static uint32 stop(uint8 motor)
{
	return rotate(motor, 0);
}

static uint32 moveTo(uint8 motor, int32 position)
{
	if(motor >= TMC4672_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc4672_setAbsolutTargetPosition(motor, position);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	if(motor >= TMC4672_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc4672_setRelativeTargetPosition(motor, *ticks);

	return TMC_ERROR_NONE;
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;

	if(motor >= TMC4672_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 4: // Maximum speed
		if(readWrite == READ)
		{
			*value = (u32) tmc4672_readInt(motor, TMC4672_PID_VELOCITY_LIMIT);
		}
		else if(readWrite == WRITE)
		{
			tmc4672_writeInt(motor, TMC4672_PID_VELOCITY_LIMIT, *value);
		}
		break;
	case 11: // acceleration
		if(readWrite == READ)
		{
			*value = (u32) tmc4672_readInt(motor, TMC4672_PID_ACCELERATION_LIMIT);
		}
		else if(readWrite == WRITE)
		{
			tmc4672_writeInt(motor, TMC4672_PID_ACCELERATION_LIMIT, *value);
		}
		break;
	case 12: // enable velocity ramp
		if(readWrite == READ)
		{
			*value = 0;
		}
		else if(readWrite == WRITE)
		{
			// nothing to do
		}
		break;
	case 13: // ramp velocity
		if(readWrite == READ) {
			*value = tmc4672_readInt(motor, TMC4672_PID_VELOCITY_TARGET);
		}
		break;


	case 171:
		// PID_TORQUE_TARGET
		if(readWrite == READ)
			*value = (s16) tmc4672_readRegister16BitValue(motor, TMC4672_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4672_writeRegister16BitValue(motor, TMC4672_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31, *value);
		break;
	case 172:
		// PID_FLUX_TARGET
		if(readWrite == READ)
			*value = (s16) tmc4672_readRegister16BitValue(motor, TMC4672_PID_TORQUE_FLUX_TARGET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4672_writeRegister16BitValue(motor, TMC4672_PID_TORQUE_FLUX_TARGET, BIT_0_TO_15, *value);
		break;
	case 173:
		// PID_VELOCITY_TARGET
		if(readWrite == READ) {
			*value = (s32) tmc4672_readInt(motor, TMC4672_PID_VELOCITY_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 174:
		// PID_POSITION_TARGET
		if(readWrite == READ) {
			*value = (s32) tmc4672_readInt(motor, TMC4672_PID_POSITION_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 175:
		// PID_TORQUE_ACTUAL
		if(readWrite == READ) {
			*value = (s16) tmc4672_readRegister16BitValue(motor, TMC4672_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 176:
		// PID_TORQUE_ACTUAL_mA
		if(readWrite == READ) {
			*value = tmc4672_getActualTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 177:
		// PID_FLUX_ACTUAL
		if(readWrite == READ) {
			*value = (s16) tmc4672_readRegister16BitValue(motor, TMC4672_PID_TORQUE_FLUX_ACTUAL, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 178:
		// PID_VELOCITY_ACTUAL
		if(readWrite == READ) {
			*value = tmc4672_getActualVelocity(motor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 179:
		// PID_POSITION_ACTUAL
		if(readWrite == READ)
			*value = tmc4672_getActualPosition(motor);
		else if(readWrite == WRITE)
		{
			tmc4672_writeInt(motor, TMC4672_PID_POSITION_ACTUAL, *value);
		}
		break;
	case 189:
		// PIDIN_TARGET_TORQUE
		if(readWrite == READ) {
			*value = tmc4672_getTargetTorque_raw(motor);
		} else if(readWrite == WRITE) {
			tmc4672_setTargetTorque_raw(motor, *value);
		}
		break;
	case 190:
		// PIDIN_TARGET_TORQUE_mA
		if(readWrite == READ) {
			*value = tmc4672_getTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			tmc4672_setTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);
		}
		break;
	case 191:
		// PIDIN_TARGET_FLUX
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 1);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 192:
		// PIDIN_TARGET_VELOCITY
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 2);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 193:
		// PIDIN_TARGET_POSITION
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 3);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 194:
		// PIDOUT_TARGET_TORQUE
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 4);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// PIDOUT_TARGET_FLUX
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 5);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 196:
		// PIDOUT_TARGET_VELOCITY
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 6);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 197:
		// PIDOUT_TARGET_POSITION
		if(readWrite == READ) {
			tmc4672_writeInt(motor, TMC4672_INTERIM_ADDR, 7);
			*value = tmc4672_readInt(motor, TMC4672_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
//	case 230:
//		// DEBUG_VALUE_0
//		if(readWrite == READ) {
//			*value = debug_getTestVar0();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar0(*value);
//		}
//		break;
//	case 231:
//		// DEBUG_VALUE_1
//		if(readWrite == READ) {
//			*value = debug_getTestVar1();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar1(*value);
//		}
//		break;
//	case 232:
//		// DEBUG_VALUE_2
//		if(readWrite == READ) {
//			*value = debug_getTestVar2();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar2(*value);
//		}
//		break;
//	case 233:
//		// DEBUG_VALUE_3
//		if(readWrite == READ) {
//			*value = debug_getTestVar3();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar3(*value);
//		}
//		break;
//	case 234:
//		// DEBUG_VALUE_4
//		if(readWrite == READ) {
//			*value = debug_getTestVar4();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar4(*value);
//		}
//		break;
//	case 235:
//		// DEBUG_VALUE_5
//		if(readWrite == READ) {
//			*value = debug_getTestVar5();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar5(*value);
//		}
//		break;
//	case 236:
//		// DEBUG_VALUE_6
//		if(readWrite == READ) {
//			*value = debug_getTestVar6();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar6(*value);
//		}
//		break;
//	case 237:
//		// DEBUG_VALUE_7
//		if(readWrite == READ) {
//			*value = debug_getTestVar7();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar7(*value);
//		}
//		break;
//	case 238:
//		// DEBUG_VALUE_8
//		if(readWrite == READ) {
//			*value = debug_getTestVar8();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar8(*value);
//		}
//		break;
//	case 239:
//		// DEBUG_VALUE_9
//		if(readWrite == READ) {
//			*value = debug_getTestVar9();
//		} else if(readWrite == WRITE) {
//			debug_setTestVar9(*value);
//		}
//		break;
	case 251:
		// Torque measurement factor
		if(readWrite == READ) {
			*value = motorConfig[motor].torqueMeasurementFactor;
		} else if(readWrite == WRITE) {
			motorConfig[motor].torqueMeasurementFactor = *value;
		}
		break;
	case 252:
		// Start encoder initialization
		if(readWrite == READ) {
			*value = motorConfig[motor].initMode;
		} else if(readWrite == WRITE) {
			tmc4672_startEncoderInitialization(*value, &motorConfig[motor].initMode, &motorConfig[motor].initState);
		}
		break;
	case 253:
		// Encoder init state
		if(readWrite == READ) {
			*value = motorConfig[motor].initState;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 254:
		// Actual encoder wait time
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

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
{
	if(motor >= TMC4672_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = tmc4672_getActualVelocity(motor);

	return TMC_ERROR_NONE;
}

static void periodicJob(uint32 actualSystick)
{
	int motor;

	// do encoder initialization if necessary
	for(motor = 0; motor < TMC4672_MOTORS; motor++)
	{
		tmc4672_periodicJob(motor, actualSystick, motorConfig[motor].initMode,
				&(motorConfig[motor].initState), motorConfig[motor].initWaitTime,
				&(motorConfig[motor].actualInitWaitTime), motorConfig[motor].startVoltage);
	}
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmc4672_writeInt(DEFAULT_MOTOR, address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmc4672_readInt(DEFAULT_MOTOR, address);
}

static uint32 SAP(uint8 type, uint8 motor, int32 value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32 GAP(uint8 type, uint8 motor, int32 *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
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

static uint8 reset()
{
	// set default polarity for evaluation board's power stage after VW reset
	for(size_t motor = 0; motor < TMC4672_MOTORS; motor++) {
		tmc4672_writeInt(motor, TMC4672_PWM_POLARITIES, 0);
		tmc4672_writeInt(motor, TMC4672_PWM_SV_CHOP, 0x0);
	}

	return 1;
}

static uint8 restore()
{
	// set default polarity for evaluation board's power stage after VW reset
	for(size_t motor = 0; motor < TMC4672_MOTORS; motor++) {
		tmc4672_writeInt(motor, TMC4672_PWM_POLARITIES, 0);
		tmc4672_writeInt(motor, TMC4672_PWM_SV_CHOP, 0x0);
	}

	//enableDriver(DRIVER_DISABLE);
	return 1;
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

void TMC4672_init(void)
{
	// configure ENABLE-PIN for TMC4672
	PIN_DRV_ENN = &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	HAL.IOs->config->setHigh(PIN_DRV_ENN);

	TMC4672_SPIChannel = &HAL.SPI->ch1;
	TMC4672_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	TMC4672_config = Evalboards.ch1.config;

	// connect evalboard functions
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_RESET;//CONFIG_READY;
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
	Evalboards.ch1.numberOfMotors       = TMC4672_MOTORS;
	Evalboards.ch1.deInit               = deInit;
	Evalboards.ch1.VMMin                = 140;
	Evalboards.ch1.VMMax                = 650;

	// init motor config

	int motor;
	for(motor = 0; motor < TMC4672_MOTORS; motor++)
	{
		motorConfig[motor].initWaitTime             = 1000;
		motorConfig[motor].startVoltage             = 6000;
		motorConfig[motor].initMode                 = 0;
		motorConfig[motor].torqueMeasurementFactor  = 256;
	}

	// set default polarity for evaluation board's power stage on init
	tmc4672_writeInt(DEFAULT_MOTOR, TMC4672_PWM_POLARITIES, 0x0);
	tmc4672_writeInt(DEFAULT_MOTOR, TMC4672_PWM_SV_CHOP, 0x0);

	// enable the driver
	enableDriver(DRIVER_ENABLE);
};
