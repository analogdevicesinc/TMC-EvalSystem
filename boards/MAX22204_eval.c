/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "tmc/StepDir.h"
#include "boards/Board.h"
#include "tmc/RAMDebug.h"

#define MAX22204_EVAL_VM_MIN  44   // VM[V/10] min 4,5V real
#define MAX22204_EVAL_VM_MAX  680  // VM[V/10] max +5%; 65V real

#define MAX22204_MOTORS 1

#undef  MAX22204_MAX_VELOCITY
#define MAX22204_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define MAX22204_DEFAULT_MOTOR 0

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
#define MAX22204_VREF_TIMER TIMER_CHANNEL_3
#define MAX22204_RAMDEBUG_TIMER TIMER_CHANNEL_1
#elif defined(LandungsbrueckeV3)
#define MAX22204_VREF_TIMER	TIMER_CHANNEL_4
#define MAX22204_RAMDEBUG_TIMER TIMER_CHANNEL_2
#endif

static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value);
static void periodicJob(uint32_t tick);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);
static void deInit(void);
static uint8_t reset();
static uint8_t restore();
static void enableDriver(DriverState state);

//static int32_t measured_velocity = 0;

typedef struct
{
	IOPinTypeDef *ROFF_CTRL;
	IOPinTypeDef *MODE0;
	IOPinTypeDef *MODE1;
	IOPinTypeDef *MODE2;
	IOPinTypeDef *DECAY0;
	IOPinTypeDef *DECAY1;
	IOPinTypeDef *DECAY2;
	IOPinTypeDef *EN_N;
	IOPinTypeDef *STEP;
	IOPinTypeDef *DIR;
	IOPinTypeDef *REF_PWM;
	IOPinTypeDef *SLEEP_N;
	IOPinTypeDef *TRQ;
	IOPinTypeDef *FAULT_N;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= MAX22204_MOTORS)
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
	if(motor >= MAX22204_MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= MAX22204_MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= MAX22204_MOTORS)
		return TMC_ERROR_MOTOR;

	int32_t tempValue;

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
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = (StepDir_getStatus(motor) & STATUS_TARGET_REACHED)? 1:0;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 9:
		// VREF
		if (readWrite == READ) {
			*value = (uint32_t) ((1.0 - Timer.getDuty(MAX22204_VREF_TIMER)) * 100);
		} else {
			if ((uint32_t) *value <= 100) {
				Timer.setDuty(MAX22204_VREF_TIMER, 1.0 - ((float)(*value) / 100));
			} else {
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 10:
		// TRQ
		if (readWrite == READ) {
			*value = (HAL.IOs->config->getState(Pins.TRQ) == IOS_HIGH) ? 1 : 0;
		} else {
			HAL.IOs->config->setToState(Pins.TRQ, (*value) ? IOS_HIGH : IOS_LOW);
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
			*value = 1 << ((((HAL.IOs->config->getState(Pins.MODE2) == IOS_HIGH) ? 1 : 0) << 2)
				| (((HAL.IOs->config->getState(Pins.MODE1) == IOS_HIGH) ? 1 : 0) << 1)
				| (((HAL.IOs->config->getState(Pins.MODE0) == IOS_HIGH) ? 1 : 0) << 0));
		} else if(readWrite == WRITE) {
			tempValue = __builtin_ctz(*value);
			HAL.IOs->config->setToState(Pins.MODE0, (tempValue & 0b001) ? IOS_HIGH : IOS_LOW);
			HAL.IOs->config->setToState(Pins.MODE1, (tempValue & 0b010) ? IOS_HIGH : IOS_LOW);
			HAL.IOs->config->setToState(Pins.MODE2, (tempValue & 0b100) ? IOS_HIGH : IOS_LOW);
		}
		break;
	case 141:
		// Decay Mode
		if(readWrite == READ) {
			*value = (((HAL.IOs->config->getState(Pins.DECAY2) == IOS_HIGH) ? 1 : 0) << 2)
				| (((HAL.IOs->config->getState(Pins.DECAY1) == IOS_HIGH) ? 1 : 0) << 1)
				| (((HAL.IOs->config->getState(Pins.DECAY0) == IOS_HIGH) ? 1 : 0) << 0);
		} else if(readWrite == WRITE) {
			HAL.IOs->config->setToState(Pins.DECAY0, (*value & 0b001) ? IOS_HIGH : IOS_LOW);
			HAL.IOs->config->setToState(Pins.DECAY1, (*value & 0b010) ? IOS_HIGH : IOS_LOW);
			HAL.IOs->config->setToState(Pins.DECAY2, (*value & 0b100) ? IOS_HIGH : IOS_LOW);
		}
		break;
	case 142:
		// Enable
		if(readWrite == READ) {
			*value = (HAL.IOs->config->getState(Pins.EN_N) == IOS_HIGH) ? 1 : 0;
		} else if(readWrite == WRITE) {
			HAL.IOs->config->setToState(Pins.EN_N, (*value == 0) ? IOS_LOW : IOS_HIGH);
		}
		break;
	case 143:
		// Sleep
		if(readWrite == READ) {
			*value = (HAL.IOs->config->getState(Pins.SLEEP_N) == IOS_LOW) ? 1 : 0;
		} else if(readWrite == WRITE) {
			HAL.IOs->config->setToState(Pins.SLEEP_N, (*value == 0) ? IOS_HIGH : IOS_LOW);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = HAL.IOs->config->getState(Pins.ROFF_CTRL);

		} else if(readWrite == WRITE) {
			HAL.IOs->config->setToState(Pins.ROFF_CTRL, *value);

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

static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);

	switch(type) {
	case 0: // REF_MON
		*value = *HAL.ADCs->AIN0;
		break;
	case 1: // ISENA
		*value = *HAL.ADCs->AIN1;
		break;
	case 2: // ISENB
		*value = *HAL.ADCs->AIN2;
		break;
	default:
		return TMC_ERROR_TYPE;
	}

	return TMC_ERROR_NONE;
}

static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	uint32_t errors = TMC_ERROR_NONE;
	switch(type) {
	case 2:
	case 3:
	case 4:
	case 24:
		if(limit == LIMIT_MIN) {
			*value = 0; // TODO: Determine limits here
		} else if(limit == LIMIT_MAX) {
			*value = StepDir_getFrequency(motor);
		}
		break;
	case 5:
		if(limit == LIMIT_MIN) {
			*value = 0; // TODO: Determine limits here
		} else if(limit == LIMIT_MAX) {
			*value = StepDir_getMaxAcceleration(motor);
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value)
{
	return getLimit(LIMIT_MIN, type, motor, value);
}

static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value)
{
	return getLimit(LIMIT_MAX, type, motor, value);
}

static void periodicJob(uint32_t tick)
{
	StepDir_periodicJob(MAX22204_DEFAULT_MOTOR);
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	if(motor >= MAX22204_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(motor)
	{
	case 0:
		//*value = StepDir.ch1->actualVelocity;
		*value = StepDir_getActualVelocity(0);
		break;
	default:
		return TMC_ERROR_MOTOR;
		break;
	}
	return TMC_ERROR_NONE;
}

static void deInit(void)
{
	HAL.IOs->config->reset(Pins.ROFF_CTRL);
	HAL.IOs->config->reset(Pins.MODE0);
	HAL.IOs->config->reset(Pins.MODE1);
	HAL.IOs->config->reset(Pins.MODE2);
	HAL.IOs->config->reset(Pins.DECAY0);
	HAL.IOs->config->reset(Pins.DECAY1);
	HAL.IOs->config->reset(Pins.DECAY2);
	HAL.IOs->config->reset(Pins.STEP);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.REF_PWM);
	HAL.IOs->config->reset(Pins.TRQ);

	StepDir_deInit();
	Timer.deInit();
}

static uint8_t reset()
{
	if(StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
		return 0;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);

	return 1;
}

static uint8_t restore()
{
	return 1;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setLow(Pins.EN_N);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setHigh(Pins.EN_N);
}

void MAX22204_init(void)
{
	// Initialize the hardware pins
	Pins.ROFF_CTRL = &HAL.IOs->pins->DIO19;
	Pins.MODE0 = &HAL.IOs->pins->DIO18;
	Pins.MODE1 = &HAL.IOs->pins->DIO17;
	Pins.MODE2 = &HAL.IOs->pins->DIO16;
	Pins.DECAY0 = &HAL.IOs->pins->DIO15;
	Pins.DECAY1 = &HAL.IOs->pins->DIO14;
	Pins.DECAY2 = &HAL.IOs->pins->DIO13;
	Pins.EN_N = &HAL.IOs->pins->DIO0;
	Pins.STEP = &HAL.IOs->pins->DIO6;
	Pins.DIR = &HAL.IOs->pins->DIO7;
	Pins.REF_PWM = &HAL.IOs->pins->DIO9;
	Pins.SLEEP_N = &HAL.IOs->pins->DIO2;
	Pins.TRQ = &HAL.IOs->pins->DIO12;
	Pins.FAULT_N = &HAL.IOs->pins->DIO4;

 	HAL.IOs->config->toOutput(Pins.ROFF_CTRL);
 	HAL.IOs->config->toOutput(Pins.MODE0);
 	HAL.IOs->config->toOutput(Pins.MODE1);
 	HAL.IOs->config->toOutput(Pins.MODE2);
 	HAL.IOs->config->toOutput(Pins.DECAY0);
 	HAL.IOs->config->toOutput(Pins.DECAY1);
 	HAL.IOs->config->toOutput(Pins.DECAY2);
 	HAL.IOs->config->toOutput(Pins.EN_N);
 	HAL.IOs->config->toOutput(Pins.STEP);
 	HAL.IOs->config->toOutput(Pins.DIR);
 	HAL.IOs->config->toOutput(Pins.REF_PWM);
	HAL.IOs->config->toOutput(Pins.SLEEP_N);
	HAL.IOs->config->toOutput(Pins.TRQ);
	HAL.IOs->config->toInput(Pins.FAULT_N);

 	HAL.IOs->config->setLow(Pins.ROFF_CTRL);
 	HAL.IOs->config->setLow(Pins.MODE0);
 	HAL.IOs->config->setLow(Pins.MODE1);
 	HAL.IOs->config->setLow(Pins.MODE2);
 	HAL.IOs->config->setLow(Pins.DECAY0);
 	HAL.IOs->config->setLow(Pins.DECAY1);
 	HAL.IOs->config->setLow(Pins.DECAY2);
 	HAL.IOs->config->setLow(Pins.EN_N);
 	HAL.IOs->config->setLow(Pins.STEP);
 	HAL.IOs->config->setLow(Pins.DIR);
	HAL.IOs->config->setHigh(Pins.SLEEP_N);
	HAL.IOs->config->setLow(Pins.TRQ);

 	// Initialize the software StepDir generator
 	StepDir_init(STEPDIR_PRECISION);
 	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);

 	Evalboards.ch2.config->callback     = NULL;
 	Evalboards.ch2.config->channel      = 0;
 	Evalboards.ch2.config->configIndex  = 0;
 	Evalboards.ch2.config->state        = CONFIG_READY;
 	Evalboards.ch2.config->reset        = reset;
 	Evalboards.ch2.config->restore      = restore;

 	Evalboards.ch2.rotate               = rotate;
 	Evalboards.ch2.right                = right;
 	Evalboards.ch2.left                 = left;
 	Evalboards.ch2.stop                 = stop;
 	Evalboards.ch2.GAP                  = GAP;
 	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.GIO                  = GIO;
	Evalboards.ch2.moveTo               = moveTo;
 	Evalboards.ch2.moveBy               = moveBy;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.numberOfMotors       = MAX22204_MOTORS;
	Evalboards.ch2.VMMin                = MAX22204_EVAL_VM_MIN;
	Evalboards.ch2.VMMax                = MAX22204_EVAL_VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.getMin               = getMin;
	Evalboards.ch2.getMax               = getMax;

	Evalboards.driverEnable             = DRIVER_DISABLE;

	HAL.IOs->config->toOutput(Pins.REF_PWM);

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
	Pins.REF_PWM->configuration.GPIO_Mode = GPIO_Mode_AF4;
#elif defined(LandungsbrueckeV3)
	Pins.REF_PWM->configuration.GPIO_Mode  = GPIO_MODE_AF;
	gpio_af_set(Pins.REF_PWM->port, GPIO_AF_1, Pins.REF_PWM->bitWeight);
#endif

	HAL.IOs->config->set(Pins.REF_PWM);
	Timer.overflow_callback = debug_nextProcess;
	Timer.init();
	Timer.setPeriodMin(MAX22204_RAMDEBUG_TIMER, 1000);
	Timer.setFrequencyMin(MAX22204_RAMDEBUG_TIMER, 1000);
	Timer.setDuty(MAX22204_RAMDEBUG_TIMER, 0.5);

	//enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
