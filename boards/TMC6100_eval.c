#include "boards/Board.h"
#include "tmc/ic/TMC6100/TMC6100.h"

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         550  // VM[V/10] max

#define TMC6100_DEFAULT_MOTOR 0

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static void readRegister(uint8_t motor, uint8_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint8_t address, int32_t value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

SPIChannelTypeDef *TMC6100_SPIChannel;

// => SPI wrapper
uint8_t tmc6100_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
	if (motor == TMC6100_DEFAULT_MOTOR)
		return TMC6100_SPIChannel->readWrite(data, lastTransfer);
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

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	UNUSED(velocity);

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

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
	UNUSED(position);

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	UNUSED(readWrite);
	UNUSED(value);

	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC6100_MOTORS)
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

static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	UNUSED(value);

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(motor);
	tmc6100_writeInt(TMC6100_DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc6100_readInt(TMC6100_DEFAULT_MOTOR, address);
}

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch2.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);

	return 0;
}

static void deInit(void)
{
};

static uint8_t reset()
{
	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6100_writeInt(TMC6100_DEFAULT_MOTOR, TMC6100_GCONF, 0x40);

	return 1;
}

static uint8_t restore()
{
	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6100_writeInt(TMC6100_DEFAULT_MOTOR, TMC6100_GCONF, 0x40);

	return 1;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

void TMC6100_init(void)
{
	TMC6100_SPIChannel = &HAL.SPI->ch2;
	TMC6100_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

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
	Evalboards.ch2.numberOfMotors       = TMC6100_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;

	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6100_writeInt(TMC6100_DEFAULT_MOTOR, TMC6100_GCONF, 0x40);

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
