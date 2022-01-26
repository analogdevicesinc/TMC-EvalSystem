#include "Rhino_standalone.h"

#include "Board.h"
#include "tmc/ic/TMC5130/TMC5130_Register.h"
#include "tmc/StepDir.h"
#include "hal/Timer.h"

//#define VM_MIN  50   // VM[V/10] min
//#define VM_MAX  480  // VM[V/10] max

#undef  TMC2100_MAX_VELOCITY
#define TMC2100_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define MOTORS 1

#define VREF_FULLSCALE 2714 // mV

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

static void periodicJob(uint32_t tick);
//static void checkErrors	(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static uint32_t setStandAloneSettings(uint8_t i, int32_t value);
static uint32_t getStandAloneSettings(uint8_t i, int32_t *value);

static IO_States lastEnable = IOS_OPEN; //TMCRhinoSA.Enable_StandStillPowerDownSettings.ENABLED_034;

static uint16_t vref; // mV

typedef struct
{
	IOPinTypeDef  *CFG3;
	IOPinTypeDef  *CFG2;
	IOPinTypeDef  *CFG1;
	IOPinTypeDef  *CFG0;
	IOPinTypeDef  *STEP;
	IOPinTypeDef  *DIR;
	IOPinTypeDef  *CFG4;
	IOPinTypeDef  *CFG5;
	IOPinTypeDef  *ERROR;
	IOPinTypeDef  *INDEX;
	IOPinTypeDef  *CFG6_ENN;
	IOPinTypeDef  *AIN_REF_SW;
	IOPinTypeDef  *AIN_REF_PWM;

} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_rotate(motor, velocity);

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
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

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
	case 9:
		// VREF
		if (readWrite == READ) {
			*value = vref;
		} else {
			if ((uint32_t) *value < VREF_FULLSCALE) {
				vref = *value;
				Timer.setDuty(TIMER_CHANNEL_1, ((float)vref) / VREF_FULLSCALE);
			} else {
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 11:
		// Position reached flag
		if(readWrite == READ) {
			if(type == 8)
				*value = (StepDir_getStatus(motor) & STATUS_TARGET_REACHED)? 1:0;
			else
				errors |= TMC_ERROR_TYPE;
		} else if(readWrite == WRITE) {
			errors |= setStandAloneSettings(type-6, *value);
		}
		break;
	case 12:
		if(readWrite == READ) {
			*value = (HAL.IOs->config->isHigh(Pins.AIN_REF_SW))? 1 : 0;
		} else if(readWrite == WRITE) {
			if(*value)
				HAL.IOs->config->setHigh(Pins.AIN_REF_SW);
			else
				HAL.IOs->config->setLow(Pins.AIN_REF_SW);
		}
	break;

	case 13:
		if(readWrite == READ) {
			*value = (HAL.IOs->config->isHigh(Pins.AIN_REF_PWM)) ? 1 : 0;
		} else if(readWrite == WRITE) {
			if(((uint32_t) *value) > 10000)
				errors |= TMC_ERROR_TYPE;
			else
				Timer.setDuty(TIMER_CHANNEL_1, ((float)*value) / TIMER_MAX);
		}
	break;

	// getStandaloneSettin APs have been copied from from types 6-11 but also left there for backwards-compatibility

	case 14:
		// Chopper off time
	case 15:
		// Microstep Resolution, Interpolation and Chopper
	case 16:
		// Maximum current
	case 17:
		// Chopper hysteresis
	case 18:
		// Chopper blank time
	case 19:
		// Power down delay
		if(readWrite == READ) {
			getStandAloneSettings(type - 14, value);
		} else if(readWrite == WRITE) {
			errors |= setStandAloneSettings(type - 14, *value);
		}
	break;
	case 50: // StepDir internal(0)/external(1)
		if(readWrite == READ) {
			*value = StepDir_getMode(motor);
		} else if(readWrite == WRITE) {
			StepDir_setMode(motor, *value);
		}
		break;
	case 51: // StepDir interrupt frequency
		if(readWrite == READ) {
			*value = StepDir_getFrequency(motor);
		} else if(readWrite == WRITE) {
			StepDir_setFrequency(motor, *value);
		}
		break;

	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			errors |= TMC_ERROR_TYPE;
		} else if(readWrite == WRITE) {
			// microstep resolution without interpolation

			errors |= getStandAloneSettings(1, value);

			IO_States CFG[7];
			TMCRhinoSA.getPins(CFG);

			if((CFG[2] == IOS_LOW)        && (CFG[1] == IOS_LOW))   *value = 1;
			else if((CFG[2] == IOS_LOW)   && (CFG[1] == IOS_HIGH))  *value = 2;
			else if((CFG[2] == IOS_LOW)   && (CFG[1] == IOS_OPEN))  *value = 2;
			else if((CFG[2] == IOS_HIGH)  && (CFG[1] == IOS_LOW))   *value = 4;
			else if((CFG[2] == IOS_HIGH)  && (CFG[1] == IOS_HIGH))  *value = 16;
			else if((CFG[2] == IOS_HIGH)  && (CFG[1] == IOS_OPEN))  *value = 4;
			else if((CFG[2] == IOS_OPEN)  && (CFG[1] == IOS_LOW))   *value = 16;
			else if((CFG[2] == IOS_OPEN)  && (CFG[1] == IOS_HIGH))  *value = 4;
			else if((CFG[2] == IOS_OPEN)  && (CFG[1] == IOS_OPEN))  *value = 16;
		}
		break;
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

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(address);
	UNUSED(motor);

	TMCRhinoSA.setInt(value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(address);
	UNUSED(motor);

	*value = TMCRhinoSA.getInt();
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	switch(type)
	{
	case 0:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	case 1:  // When running with the TMC4330 v1.2: CFG5 is routed from DIO6 (Motion Controller) to DIO12 (Driver)
		Pins.CFG5 = &HAL.IOs->pins->DIO6;
		TMCRhinoSA.CFGPins[5] = Pins.CFG5;
		HAL.IOs->config->toOutput(TMCRhinoSA.CFGPins[5]);
		HAL.IOs->config->setLow(TMCRhinoSA.CFGPins[5]);
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}

	return errors;
}

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);

	for(int motor = 0; motor < MOTORS; motor++)
	{
		StepDir_periodicJob(motor);
	}
}

static uint32_t setStandAloneSettings(uint8_t i, int32_t value)
{
	TMCRhinoTypeStandAloneConfigDef config;

	uint32_t errors = TMC_ERROR_NONE;

	if(i > 6)
	{
		errors |= TMC_ERROR_TYPE;
		return errors;
	}

	if((value < 0) || (value > 2))
	{
		if((i != 1) || (value > 0x0F))
		{
			errors |= TMC_ERROR_VALUE;
			return errors;
		}
	}

	TMCRhinoSA.getConfig(&config);

	switch(i)
	{
	case 0:
		config.chopperOffTime = value;
		break;
	case 1:
		switch(value)
		{
		case 0:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY1_INTERPOL0;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY1_INTERPOL0;
			break;
		case 1:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY2_INTERPOL0;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY2_INTERPOL0;
			break;
		case 2:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY2_INTERPOL256;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY2_INTERPOL256;
			break;
		case 3:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY4_INTERPOL0;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY4_INTERPOL0;
			break;
		case 4:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY4_INTERPOL256;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY4_INTERPOL256;
			break;
		case 5:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY16_INTERPOL0;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY16_INTERPOL0;
			break;
		case 6:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.BY16_INTERPOL256;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.BY16_INTERPOL256;
			break;
		case 7:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.PWM_CHOP_BY4_INTERPOL256;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.PWM_CHOP_BY4_INTERPOL256;
			break;
		case 8:
			config.microstepResolution1 = TMCRhinoSA.MicrostepResolutionSettings1.PWM_CHOP_BY16_INTERPOL256;
			config.microstepResolution2 = TMCRhinoSA.MicrostepResolutionSettings2.PWM_CHOP_BY16_INTERPOL256;
			break;
		}
		break;
	case 2:
		config.currentSetting = value;
		break;
	case 3:
		config.chopperHysteresis = value;
		break;
	case 4:
		config.chopperBlankTime = value;
		break;
	case 5:
		config.enableStandStillPowerDown = lastEnable = value;
		break;
	}

	TMCRhinoSA.setConfig(&config);

	return errors;
}

static uint32_t getStandAloneSettings(uint8_t i, int32_t *value)
{
	TMCRhinoTypeStandAloneConfigDef config;

	uint32_t errors = TMC_ERROR_NONE;

	if(i > 6)
	{
		errors |= TMC_ERROR_TYPE;
		return errors;
	}

	TMCRhinoSA.getConfig(&config);

	switch(i)
	{
	case 0:
		*value = config.chopperOffTime;
		break;
	case 1:
		if(     (config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY1_INTERPOL0) &&
			    (config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY1_INTERPOL0))
			*value = 0;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY2_INTERPOL0) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY2_INTERPOL0))
			*value = 1;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY2_INTERPOL256) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY2_INTERPOL256))
			*value = 2;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY4_INTERPOL0) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY4_INTERPOL0))
			*value = 3;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY4_INTERPOL256) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY4_INTERPOL256))
			*value = 4;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY16_INTERPOL0) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY16_INTERPOL0))
			*value = 5;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.BY16_INTERPOL256) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.BY16_INTERPOL256))
			*value = 6;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.PWM_CHOP_BY4_INTERPOL256) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.PWM_CHOP_BY4_INTERPOL256))
			*value = 7;
		else if((config.microstepResolution1 == TMCRhinoSA.MicrostepResolutionSettings1.PWM_CHOP_BY16_INTERPOL256) &&
				(config.microstepResolution2 == TMCRhinoSA.MicrostepResolutionSettings2.PWM_CHOP_BY16_INTERPOL256))
			*value = 8;
		else
			errors |= TMC_ERROR_VALUE;
		break;
	case 2:
		*value = config.currentSetting;
		break;
	case 3:
		*value = config.chopperHysteresis;
		break;
	case 4:
		*value = config.chopperBlankTime;
		break;
	case 5:
		*value = config.enableStandStillPowerDown;
		break;
	case 6:
		*value = config.chopperOffTime;
		break;
	}
	return errors;
}

void TMC2100_init(void)
{
	Pins.AIN_REF_PWM  = &HAL.IOs->pins->DIO11;
	Pins.AIN_REF_SW   = &HAL.IOs->pins->DIO10;
	Pins.CFG0         = &HAL.IOs->pins->SPI2_SDO;
	Pins.CFG1         = &HAL.IOs->pins->SPI2_SDI;
	Pins.CFG2         = &HAL.IOs->pins->SPI2_SCK;
	Pins.CFG3         = &HAL.IOs->pins->SPI2_CSN0;
	Pins.CFG4         = &HAL.IOs->pins->DIO13;
	Pins.CFG5         = &HAL.IOs->pins->DIO12;
	Pins.CFG6_ENN     = &HAL.IOs->pins->DIO0;
	Pins.DIR          = &HAL.IOs->pins->DIO7;
	Pins.ERROR        = &HAL.IOs->pins->DIO15;
	Pins.INDEX        = &HAL.IOs->pins->DIO16;
	Pins.STEP         = &HAL.IOs->pins->DIO6;

	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
	HAL.IOs->config->toOutput(Pins.AIN_REF_PWM);
	HAL.IOs->config->toOutput(Pins.AIN_REF_SW);

	HAL.IOs->config->toInput(Pins.ERROR);
	HAL.IOs->config->toInput(Pins.INDEX);

	TMCRhinoSA.CFGPins[0] = Pins.CFG0;
	TMCRhinoSA.CFGPins[1] = Pins.CFG1;
	TMCRhinoSA.CFGPins[2] = Pins.CFG2;
	TMCRhinoSA.CFGPins[3] = Pins.CFG3;
	TMCRhinoSA.CFGPins[4] = Pins.CFG4;
	TMCRhinoSA.CFGPins[5] = Pins.CFG5;
	TMCRhinoSA.CFGPins[6] = Pins.CFG6_ENN;

	for(uint8_t i = 0; i < 7; i++)
	{
		HAL.IOs->config->toOutput(TMCRhinoSA.CFGPins[i]);
		HAL.IOs->config->setLow(TMCRhinoSA.CFGPins[i]);
	}

	Evalboards.ch2.config->state = CONFIG_READY;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 20000);
	StepDir_setAcceleration(0, 25000);

	Evalboards.ch2.rotate          = rotate;
	Evalboards.ch2.right           = right;
	Evalboards.ch2.left            = left;
	Evalboards.ch2.stop            = stop;
	Evalboards.ch2.GAP             = GAP;
	Evalboards.ch2.SAP             = SAP;
	Evalboards.ch2.moveTo          = moveTo;
	Evalboards.ch2.moveBy          = moveBy;
	Evalboards.ch2.writeRegister   = writeRegister;
	Evalboards.ch2.readRegister    = readRegister;
	Evalboards.ch2.userFunction    = userFunction;
	Evalboards.ch2.periodicJob     = periodicJob;
	Evalboards.ch2.enableDriver    = enableDriver;
	Evalboards.ch2.numberOfMotors  = MOTORS;
	Evalboards.ch2.deInit          = deInit;

#if defined(Startrampe)
	Pins.AIN_REF_PWM->configuration.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PinAFConfig(Pins.AIN_REF_PWM->port, Pins.AIN_REF_PWM->bit, GPIO_AF_TIM1);
#elif defined(Landungsbruecke)
	HAL.IOs->config->toOutput(Pins.AIN_REF_PWM);
	Pins.AIN_REF_PWM->configuration.GPIO_Mode = GPIO_Mode_AF4;
#endif

	vref = 2000;
	HAL.IOs->config->set(Pins.AIN_REF_PWM);
	Timer.init();
	Timer.setDuty(TIMER_CHANNEL_1, ((float)vref) / VREF_FULLSCALE);

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
	reset();
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);

	HAL.IOs->config->reset(Pins.AIN_REF_PWM);
	HAL.IOs->config->reset(Pins.AIN_REF_SW);
	HAL.IOs->config->reset(Pins.CFG0);
	HAL.IOs->config->reset(Pins.CFG1);
	HAL.IOs->config->reset(Pins.CFG2);
	HAL.IOs->config->reset(Pins.CFG3);
	HAL.IOs->config->reset(Pins.CFG4);
	HAL.IOs->config->reset(Pins.CFG5);
	HAL.IOs->config->reset(Pins.CFG6_ENN);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.ERROR);
	HAL.IOs->config->reset(Pins.INDEX);
	HAL.IOs->config->reset(Pins.STEP);

	StepDir_deInit();
	Timer.deInit();
}

static uint8_t reset()
{
	if(StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
		return 0;

	TMCRhinoSA.reset();

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 20000);
	StepDir_setAcceleration(0, 25000);

	return 1;
}

static void enableDriver(DriverState state)
{
	TMCRhinoTypeStandAloneConfigDef config;
	TMCRhinoSA.getConfig(&config);

	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
	{
		lastEnable = config.enableStandStillPowerDown;
		config.enableStandStillPowerDown = TMCRhinoSA.EnableStandStillPowerDownSettings.DISABLED;
	}
	else
	{
		config.enableStandStillPowerDown = lastEnable;
	}
}
