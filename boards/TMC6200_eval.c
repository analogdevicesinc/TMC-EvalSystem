#include "Board.h"
#include "tmc/ic/TMC6200/TMC6200.h"

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

#define TMC6200_DEFAULT_MOTOR 0

static uint32 right(uint8 motor, int32 velocity);
static uint32 left(uint8 motor, int32 velocity);
static uint32 rotate(uint8 motor, int32 velocity);
static uint32 stop(uint8 motor);
static uint32 moveTo(uint8 motor, int32 position);
static uint32 moveBy(uint8 motor, int32 *ticks);
static uint32 GAP(uint8 type, uint8 motor, int32 *value);
static uint32 SAP(uint8 type, uint8 motor, int32 value);
static void readRegister(u8 motor, uint8 address, int32 *value);
static void writeRegister(u8 motor, uint8 address, int32 value);
static uint32 getMeasuredSpeed(uint8 motor, int32 *value);

static void periodicJob(uint32 tick);
static void checkErrors(uint32 tick);
static void deInit(void);
static uint32 userFunction(uint8 type, uint8 motor, int32 *value);

static uint8 reset();
static void enableDriver(DriverState state);

SPIChannelTypeDef *TMC6200_SPIChannel;

// => SPI wrapper
u8 tmc6200_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
	if (motor == TMC6200_DEFAULT_MOTOR)
		return TMC6200_SPIChannel->readWrite(data, lastTransfer);
	else
		return 0;
}
// <= SPI wrapper

typedef struct
{
	IOPinTypeDef  *REFL_UC;
	IOPinTypeDef  *REFR_UC;
	IOPinTypeDef  *DRV_ENN_CFG6;
	IOPinTypeDef  *ENCA_DCIN_CFG5;
	IOPinTypeDef  *ENCB_DCEN_CFG4;
	IOPinTypeDef  *ENCN_DCO;
	IOPinTypeDef  *SD_MODE;
	IOPinTypeDef  *SPI_MODE;
	IOPinTypeDef  *SWN_DIAG0;
	IOPinTypeDef  *SWP_DIAG1;
} PinsTypeDef;

//static PinsTypeDef Pins;

static uint32 rotate(uint8 motor, int32 velocity)
{
	UNUSED(velocity);

	if(motor >= TMC6200_MOTORS)
		return TMC_ERROR_MOTOR;

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
	UNUSED(position);

	if(motor >= TMC6200_MOTORS)
		return TMC_ERROR_MOTOR;

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	return moveTo(motor, *ticks);
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	UNUSED(readWrite);
	UNUSED(value);

	u32 errors = TMC_ERROR_NONE;

	if(motor >= TMC6200_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
		// add parameters if needed
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

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
{
	UNUSED(value);

	if(motor >= TMC6200_MOTORS)
		return TMC_ERROR_MOTOR;

	return TMC_ERROR_NONE;
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmc6200_writeInt(TMC6200_DEFAULT_MOTOR, address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmc6200_readInt(TMC6200_DEFAULT_MOTOR, address);
}

static void periodicJob(uint32 tick)
{
	UNUSED(tick);
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch2.errors = 0;
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);

	return 0;
}

static void deInit(void)
{
};

static uint8 reset()
{
	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6200_writeInt(TMC6200_DEFAULT_MOTOR, TMC6200_GCONF, 0x0);

	return 1;
}

static uint8 restore()
{
	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6200_writeInt(TMC6200_DEFAULT_MOTOR, TMC6200_GCONF, 0x0);

	return 1;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

void TMC6200_init(void)
{
	TMC6200_SPIChannel = &HAL.SPI->ch2;
	TMC6200_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

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
	Evalboards.ch2.writeRegister        = writeRegister;
	Evalboards.ch2.readRegister         = readRegister;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = TMC6200_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;

	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6200_writeInt(TMC6200_DEFAULT_MOTOR, TMC6200_GCONF, 0x0);

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
