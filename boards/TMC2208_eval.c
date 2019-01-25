#include "Board.h"
#include "tmc/ic/TMC2208/TMC2208.h"
#include "../tmc/StepDir.h"

#undef  TMC2208_MAX_VELOCITY
#define TMC2208_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  390  // VM[V/10] max

#define MOTORS 1

static uint32 right(uint8 motor, int32 velocity);
static uint32 left(uint8 motor, int32 velocity);
static uint32 rotate(uint8 motor, int32 velocity);
static uint32 stop(uint8 motor);
static uint32 moveTo(uint8 motor, int32 position);
static uint32 moveBy(uint8 motor, int32 *ticks);
static uint32 GAP(uint8 type, uint8 motor, int32 *value);
static uint32 SAP(uint8 type, uint8 motor, int32 value);

static void checkErrors (uint32 tick);
static void deInit(void);
static uint32 userFunction(uint8 type, uint8 motor, int32 *value);

static void periodicJob(uint32 tick);
static uint8 reset(void);
static void enableDriver(DriverState state);

static RXTXTypeDef *TMC2208_UARTChannel;
static TMC2208TypeDef TMC2208;
static ConfigurationTypeDef *TMC2208_config;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC2208_CRC(data, length) tmc_CRC8(data, length, 1)

typedef struct
{
	IOPinTypeDef  *DRV_ENN;
	IOPinTypeDef  *STEP;
	IOPinTypeDef  *DIR;
	IOPinTypeDef  *MS1;
	IOPinTypeDef  *MS2;
	IOPinTypeDef  *DIAG;
	IOPinTypeDef  *INDEX;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint8 restore(void);

void tmc2208_writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	UART_writeInt(TMC2208_UARTChannel, tmc2208_get_slave(&TMC2208), address, value);
}

void tmc2208_readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	UART_readInt(TMC2208_UARTChannel, tmc2208_get_slave(&TMC2208), address, value);
}

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_rotate(motor, velocity);

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
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = StepDir_getTargetPosition(motor);
		} else if(readWrite == WRITE) {
			StepDir_moveTo(motor, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = StepDir_getActualPosition(motor);
		} else if(readWrite == WRITE) {
			StepDir_setActualPosition(motor, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = StepDir_getTargetVelocity(motor);
		} else if(readWrite == WRITE) {
			StepDir_rotate(motor, *value);
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = StepDir_getActualVelocity(motor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = StepDir_getVelocityMax(motor);
		} else if(readWrite == WRITE) {
			StepDir_setVelocityMax(motor, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = StepDir_getAcceleration(motor);
		} else if(readWrite == WRITE) {
			StepDir_setAcceleration(motor, *value);
		}
		break;
	case 6:
		// UART slave address
		if(readWrite == READ) {
			*value = tmc2208_get_slave(&TMC2208);
		} else if(readWrite == WRITE) {
			tmc2208_set_slave(&TMC2208, *value);
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}

	return errors;
}

static uint32 SAP(uint8 type, uint8 motor, int32 value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32 GAP(uint8 type, uint8 motor, int32 *value)
{
	return handleParameter(READ, motor, type, value);
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch2.errors = 0;
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	uint32 errors = 0;

	switch(type)
	{
	case 0:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}

	return errors;
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->reset(Pins.DRV_ENN);

	HAL.IOs->config->reset(Pins.STEP);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.MS1);
	HAL.IOs->config->reset(Pins.MS2);
	HAL.IOs->config->reset(Pins.DIAG);
	HAL.IOs->config->reset(Pins.INDEX);

	StepDir_deInit();
}

static uint8 reset()
{
	StepDir_init();
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);

	return tmc2208_reset(&TMC2208);
}

static uint8 restore()
{
	return tmc2208_restore(&TMC2208);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.DRV_ENN);
}

static void periodicJob(uint32 tick)
{
	for(int motor = 0; motor < MOTORS; motor++)
	{
		tmc2208_periodicJob(&TMC2208, tick);
		StepDir_periodicJob(motor);
	}
}

void TMC2208_init(void)
{
	tmc_fillCRC8Table(0x07, TRUE, 1);

	Pins.DRV_ENN  = &HAL.IOs->pins->DIO0;
	Pins.STEP     = &HAL.IOs->pins->DIO6;
	Pins.DIR      = &HAL.IOs->pins->DIO7;
	Pins.MS1      = &HAL.IOs->pins->DIO3;
	Pins.MS2      = &HAL.IOs->pins->DIO4;
	Pins.DIAG     = &HAL.IOs->pins->DIO1;
	Pins.INDEX    = &HAL.IOs->pins->DIO2;

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
	HAL.IOs->config->toOutput(Pins.MS1);
	HAL.IOs->config->toOutput(Pins.MS2);
	HAL.IOs->config->toInput(Pins.DIAG);
	HAL.IOs->config->toInput(Pins.INDEX);

	HAL.UART->init();

	TMC2208_UARTChannel = HAL.UART;

	TMC2208_config = Evalboards.ch2.config;

	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_RESET;
	Evalboards.ch2.config->configIndex  = 0;

	Evalboards.ch2.rotate               = rotate;
	Evalboards.ch2.right                = right;
	Evalboards.ch2.left                 = left;
	Evalboards.ch2.stop                 = stop;
	Evalboards.ch2.GAP                  = GAP;
	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.moveTo               = moveTo;
	Evalboards.ch2.moveBy               = moveBy;
	Evalboards.ch2.writeRegister        = tmc2208_writeRegister;
	Evalboards.ch2.readRegister         = tmc2208_readRegister;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.periodicJob          = periodicJob;

	tmc2208_init(&TMC2208, 0, TMC2208_config, &tmc2208_defaultRegisterResetState[0]);

	StepDir_init();
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 51200);
	StepDir_setAcceleration(0, 51200);

	enableDriver(DRIVER_ENABLE);
};
