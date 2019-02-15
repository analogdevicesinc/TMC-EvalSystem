#include "Board.h"
#include "tmc/ic/TMCC160/TMCC160.h"

#define MOTORS 1 // number of motors for this board

IOPinTypeDef *PIN_DRV_ENN;
ConfigurationTypeDef *C160_config;

// => SPI wrapper
u8 tmcc160_spi_readwriteByte(u8 data, u8 lastTransfer)
{
	return spi_ch1_readWriteByte(data, lastTransfer);
}
// <= SPI wrapper

void tmcc160_writeDatagram(uint8 address, uint8 x3, uint8 x2, uint8 x1, uint8 x0)
{
	tmcc160_spi_readwriteByte(address | 0x80, false);
	tmcc160_spi_readwriteByte(x3, false);
	tmcc160_spi_readwriteByte(x2, false);
	tmcc160_spi_readwriteByte(x1, false);
	tmcc160_spi_readwriteByte(x0, true);
}

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	tmcc160_setTargetVelocity(velocity);

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

	tmcc160_setAbsolutTargetPosition(position);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	tmcc160_setRelativeTargetPosition(*ticks);

	return TMC_ERROR_NONE;
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:  // target position
		if(readWrite == READ)
			*value = tmcc160_getTargetPosition();
		else if(readWrite == WRITE)
			tmcc160_setAbsolutTargetPosition(*value);
		break;
	case 1:  // actual position
		if(readWrite == READ)
			*value = tmcc160_getActualPosition();
		else if(readWrite == WRITE)
			tmcc160_setActualPosition(*value);
		break;
	case 2:  // target velocity
		if(readWrite == READ)
			*value = tmcc160_getTargetVelocity();
		else if(readWrite == WRITE)
			tmcc160_setTargetVelocity(*value);
		break;
	case 3:  // actual velocity
		if(readWrite == READ)
			*value = tmcc160_getActualVelocity();
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 13:  // actual ramp generator velocity
		if(readWrite == READ)
			*value = tmcc160_getActualRampVelocity();
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 150:  // actual motor current
		if(readWrite == READ)
			*value = tmcc160_getActualTorque_mA();
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 155:  // target motor current
		if(readWrite == READ)
			*value = tmcc160_getTargetTorque_mA();
		else if(readWrite == WRITE)
			tmcc160_setTargetTorque_mA(*value);
		break;
	default:  // handle all other types by TMCL tunnel
		// --- write tmcl data ---
		// write Byte 0..3
		if(readWrite == READ)
			tmcc160_writeDatagram(TMCL_REQUEST_BYTE_0123_REG_ADDR | 0x80, 6/*TMCL_GAP*/, type, motor, 0/*TMCL.command->Error*/);
		else if(readWrite == WRITE)
			tmcc160_writeDatagram(TMCL_REQUEST_BYTE_0123_REG_ADDR | 0x80, 5/*TMCL_SAP*/, type, motor, 0/*TMCL.command->Error*/);

		tmcc160_default_spi_delay();

		// write Byte 4..7
		tmcc160_writeInt(TMCL_REQUEST_BYTE_4567_REG_ADDR | 0x80, *value);
		tmcc160_default_spi_delay();

		// --- read tmcl data ---

		// read Byte 0..3
		tmcc160_readInt(TMCL_REPLY_BYTE_0123_REG_ADDR);

		tmcc160_default_spi_delay();

		// read Byte 4..7
		*value	= tmcc160_readInt(TMCL_REPLY_BYTE_4567_REG_ADDR);
		tmcc160_default_spi_delay();

		break;
	}
	return errors;
}

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	*value = tmcc160_getActualVelocity();

	return TMC_ERROR_NONE;
}

static void periodicJob(uint32 actualSystick)
{
	tmcc160_periodicJob(actualSystick);
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmcc160_writeInt(address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmcc160_readInt(address);
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
	if(state== DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE)
		HAL.IOs->config->setLow(PIN_DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setHigh(PIN_DRV_ENN);
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->reset(PIN_DRV_ENN);
};

static uint8 reset()
{
	return 1;
}

static uint8 restore()
{
	return 1;
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

void TMCC160_init(void)
{
	// configure ENABLE-PIN for TMCC160
	PIN_DRV_ENN	= &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	HAL.IOs->config->setHigh(PIN_DRV_ENN);

	C160_config	= Evalboards.ch2.config;

	// connect evalboard functions
	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_RESET;
	Evalboards.ch2.config->configIndex  = 0;

	Evalboards.ch2.rotate               = rotate;
	Evalboards.ch2.right                = right;
	Evalboards.ch2.left                 = left;
	Evalboards.ch2.stop                 = stop;
	Evalboards.ch2.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch2.GAP                  = GAP;
	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.moveTo               = moveTo;
	Evalboards.ch2.moveBy               = moveBy;
	Evalboards.ch2.writeRegister        = writeRegister;
	Evalboards.ch2.readRegister         = readRegister;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.VMMin                = 50;   // VM[V/10] min
	Evalboards.ch2.VMMax                = 280;  // VM[V/10] max +10%

	// init used API functions
	tmcc160_init();

	// enable the driver
	enableDriver(DRIVER_ENABLE);
};
