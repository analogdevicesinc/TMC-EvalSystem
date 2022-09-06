#include "Board.h"
#include "tmc/ic/TMC5130/TMC5130_Register.h"
#include "tmc/StepDir.h"
//#include "hal/Timer.h"

//#define VM_MIN  50   // VM[V/10] min
//#define VM_MAX  480  // VM[V/10] max

#undef  TMC2210_MAX_VELOCITY
#define TMC2210_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define MOTORS 1

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

//static IO_States lastEnable = IOS_OPEN; //TMCRhinoSA.Enable_StandStillPowerDownSettings.ENABLED_034;


typedef struct
{
	IOPinTypeDef  *INDEX;
	IOPinTypeDef  *CFG0;
	IOPinTypeDef  *CFG1;
	IOPinTypeDef  *CFG2;
	IOPinTypeDef  *CFG3;
	IOPinTypeDef  *CFG4;
	IOPinTypeDef  *CFG5;

//	IOPinTypeDef  *CFG6;
//	IOPinTypeDef  *CFG7;

	IOPinTypeDef  *DRV_ENN;

	IOPinTypeDef  *nSLEEP;
	IOPinTypeDef  *STEP;
	IOPinTypeDef  *DIR;
	IOPinTypeDef  *IREF_R2;
	IOPinTypeDef  *IREF_R3;
	IOPinTypeDef  *ERROR;

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


	// getStandaloneSettin APs have been copied from from types 6-11 but also left there for backwards-compatibility

	case 14:
		// Chopper off time

	case 16:
		// Maximum current
	case 17:
		// Chopper hysteresis
	case 18:
		// Chopper blank time
	case 50: // StepDir internal(0)/external(1)
		if(readWrite == READ) {
			*value = StepDir_getMode(motor);
		}
		else if(readWrite == WRITE) {
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

//	case 137:
//			// HoldCurrentReduction
//			if(readWrite == READ) {
//				int val2 = (HAL.IOs->config->isHigh(Pins.CFG6));
//				int val3 = (HAL.IOs->config->isHigh(Pins.CFG7));
//				if (val2 == 0 && val3 == 0){ //1
//					*value = 0;
//				}
//				else if (val2 == 1 && val3 == 0){
//					*value = 1;
//				}
//				else if (val2 == 0 && val3 == 1){
//					*value = 2;
//				}
//				else if (val2 == 1 && val3 == 1){
//					*value = 3;
//				}
//			}
//			else if(readWrite == WRITE) {
//				if(*value == 0) {
//					HAL.IOs->config->setLow(Pins.CFG6);
//					HAL.IOs->config->setLow(Pins.CFG7);
//				}
//				else if(*value == 1) {
//					HAL.IOs->config->setHigh(Pins.CFG6);
//					HAL.IOs->config->setLow(Pins.CFG7);
//				}
//				else if(*value == 2) {
//					HAL.IOs->config->setLow(Pins.CFG6);
//					HAL.IOs->config->setHigh(Pins.CFG7);
//				}
//				else if(*value == 3) {
//					HAL.IOs->config->setHigh(Pins.CFG6);
//					HAL.IOs->config->setHigh(Pins.CFG7);
//				}
//			}
//
//			break;
	case 138:
			// ChopperMode
			if(readWrite == READ) {
				*value = (HAL.IOs->config->isHigh(Pins.CFG5));
			}
			else if(readWrite == WRITE) {
				if(*value == 0) {
					HAL.IOs->config->setLow(Pins.CFG5);
				}
				else if(*value == 1) {
					HAL.IOs->config->setHigh(Pins.CFG5);
				}
			}
			break;
	case 139:
			// DigitalCurrentScale
			if(readWrite == READ) {
				*value = (HAL.IOs->config->isHigh(Pins.CFG4));
			}
			else if(readWrite == WRITE) {
				if(*value == 0) {
					HAL.IOs->config->setLow(Pins.CFG4);
				}
				else if(*value == 1) {
					HAL.IOs->config->setHigh(Pins.CFG4);
				}
			}
			break;


	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			int val0 = (HAL.IOs->config->isHigh(Pins.CFG0));
			int val1 = (HAL.IOs->config->isHigh(Pins.CFG1));
			if (val0 == 0 && val1 == 0){ //8
				*value = 0;
			}
			else if (val0 == 1 && val1 == 0){ //16
				*value = 1;
			}
			else if (val0 == 0 && val1 == 1){ //32
				*value = 2;
			}
			else if (val0 == 1 && val1 == 1){ //64
				*value = 3;
			}
		}
		else if(readWrite == WRITE) {
			// microstep resolution
			if(*value == 0) { //8
				HAL.IOs->config->setLow(Pins.CFG0);
				HAL.IOs->config->setLow(Pins.CFG1);
			}
			else if(*value == 1) { //16
				HAL.IOs->config->setHigh(Pins.CFG0);
				HAL.IOs->config->setLow(Pins.CFG1);
			}
			else if(*value == 2) { //32
				HAL.IOs->config->setLow(Pins.CFG0);
				HAL.IOs->config->setHigh(Pins.CFG1);
			}
			else if(*value == 3) { //64
				HAL.IOs->config->setHigh(Pins.CFG0);
				HAL.IOs->config->setHigh(Pins.CFG1);
			}
		}
		break;
	case 211:
			//ADC Scaling Resitors
			if(readWrite == READ) {
				int val2 = (HAL.IOs->config->isHigh(Pins.IREF_R2));
				int val3 = (HAL.IOs->config->isHigh(Pins.IREF_R3));
				if (val2 == 0 && val3 == 0){ //48k
					*value = 0;
				}
				else if (val2 == 1 && val3 == 0){//24k
					*value = 1;
				}
				else if (val2 == 0 && val3 == 1){//16k
					*value = 2;
				}
				else if (val2 == 1 && val3 == 1){//12k
					*value = 3;
				}
			}
			else if(readWrite == WRITE) {
				if(*value == 0) { //48k
					HAL.IOs->config->toOutput(Pins.IREF_R2);
					HAL.IOs->config->toOutput(Pins.IREF_R3);
					HAL.IOs->config->setLow(Pins.IREF_R2);
					HAL.IOs->config->setLow(Pins.IREF_R3);
					}
				else if(*value == 1) {//24k
					HAL.IOs->config->toOutput(Pins.IREF_R2);
					HAL.IOs->config->toOutput(Pins.IREF_R3);
					HAL.IOs->config->setHigh(Pins.IREF_R2);
					HAL.IOs->config->setLow(Pins.IREF_R3);
					}
				else if(*value == 2) {//16k
					HAL.IOs->config->toOutput(Pins.IREF_R2);
					HAL.IOs->config->toOutput(Pins.IREF_R3);
					HAL.IOs->config->setLow(Pins.IREF_R2);
					HAL.IOs->config->setHigh(Pins.IREF_R3);
					}
				else if(*value == 3) {//12k
					HAL.IOs->config->toOutput(Pins.IREF_R2);
					HAL.IOs->config->toOutput(Pins.IREF_R3);
					HAL.IOs->config->setHigh(Pins.IREF_R2);
					HAL.IOs->config->setHigh(Pins.IREF_R3);
					}
			}
			break;
	case 212:
			// Current range
			if(readWrite == READ) {
				int val2 = (HAL.IOs->config->isHigh(Pins.CFG2));
				int val3 = (HAL.IOs->config->isHigh(Pins.CFG3));
				if (val2 == 0 && val3 == 0){ //1
					*value = 0;
				}
				else if (val2 == 1 && val3 == 0){ //2
					*value = 1;
				}
				else if (val2 == 0 && val3 == 1){ //3
					*value = 2;
				}
			}
			else if(readWrite == WRITE) {
				// microstep resolution
				if(*value == 0) { //1A
					HAL.IOs->config->setLow(Pins.CFG2);
					HAL.IOs->config->setLow(Pins.CFG3);
				}
				else if(*value == 1) { //2A
					HAL.IOs->config->setHigh(Pins.CFG2);
					HAL.IOs->config->setLow(Pins.CFG3);
				}
				else if(*value == 2) { //3A
					HAL.IOs->config->setLow(Pins.CFG2);
					HAL.IOs->config->setHigh(Pins.CFG3);
				}
			}

			break;
	case 223:
		if(readWrite == READ) {
			*value = HAL.IOs->config->isHigh(Pins.nSLEEP);
		}
		else if(readWrite == WRITE) {
			if(*value == 1)
			{
				HAL.IOs->config->toOutput(Pins.nSLEEP);
				HAL.IOs->config->setHigh(Pins.nSLEEP);
			}
			else if(*value == 0)
			{
				HAL.IOs->config->toOutput(Pins.nSLEEP);
				HAL.IOs->config->setLow(Pins.nSLEEP);
			}

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
	UNUSED(value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(address);
	UNUSED(motor);
	UNUSED(value);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

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

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);

	for(int motor = 0; motor < MOTORS; motor++)
	{
		StepDir_periodicJob(motor);
	}
}

void TMC2210_init(void)
{
	/*
	//Dev-Eval
	Pins.CFG0         = &HAL.IOs->pins->DIO12;
	Pins.CFG1         = &HAL.IOs->pins->DIO18;
	Pins.CFG2         = &HAL.IOs->pins->DIO17;
	Pins.CFG3         = &HAL.IOs->pins->DIO19;
	Pins.CFG4         = &HAL.IOs->pins->DIO3;
	Pins.CFG5         = &HAL.IOs->pins->DIO1;
	Pins.CFG6         = &HAL.IOs->pins->DIO4;
	Pins.CFG7         = &HAL.IOs->pins->DIO2;
	Pins.DRV_ENN      = &HAL.IOs->pins->DIO0;
	Pins.DIR          = &HAL.IOs->pins->DIO7;
	Pins.ERROR        = &HAL.IOs->pins->DIO16;
	Pins.INDEX        = &HAL.IOs->pins->DIO15;
	Pins.STEP         = &HAL.IOs->pins->DIO6;
	Pins.IREF_R2      = &HAL.IOs->pins->DIO13;
	Pins.IREF_R3 	  = &HAL.IOs->pins->DIO14;
	Pins.nSLEEP 	  = &HAL.IOs->pins->DIO8;
	*/
	Pins.CFG0         = &HAL.IOs->pins->DIO3;
	Pins.CFG1         = &HAL.IOs->pins->DIO9;
	Pins.CFG2         = &HAL.IOs->pins->DIO10;
	Pins.CFG3         = &HAL.IOs->pins->DIO11;
	Pins.CFG4         = &HAL.IOs->pins->SPI2_CSN1;
	Pins.CFG5         = &HAL.IOs->pins->SPI2_CSN2;

	Pins.DRV_ENN      = &HAL.IOs->pins->DIO0;
	Pins.DIR          = &HAL.IOs->pins->DIO7;
	Pins.ERROR        = &HAL.IOs->pins->DIO16;
	Pins.INDEX        = &HAL.IOs->pins->DIO15;
	Pins.STEP         = &HAL.IOs->pins->DIO6;
	Pins.IREF_R2      = &HAL.IOs->pins->DIO1;
	Pins.IREF_R3 	  = &HAL.IOs->pins->DIO2;
	Pins.nSLEEP 	  = &HAL.IOs->pins->DIO8;

	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
	HAL.IOs->config->toOutput(Pins.IREF_R2);
	HAL.IOs->config->toOutput(Pins.IREF_R3);
	HAL.IOs->config->toOutput(Pins.nSLEEP);
	HAL.IOs->config->toOutput(Pins.DRV_ENN);

	HAL.IOs->config->toOutput(Pins.CFG0);
	HAL.IOs->config->toOutput(Pins.CFG1);
	HAL.IOs->config->toOutput(Pins.CFG2);
	HAL.IOs->config->toOutput(Pins.CFG3);
	HAL.IOs->config->toOutput(Pins.CFG4);
	HAL.IOs->config->toOutput(Pins.CFG5);
//	HAL.IOs->config->toOutput(Pins.CFG6);
//	HAL.IOs->config->toOutput(Pins.CFG7);

	HAL.IOs->config->setLow(Pins.CFG0);
	HAL.IOs->config->setLow(Pins.CFG1);
	HAL.IOs->config->setLow(Pins.CFG2);
	HAL.IOs->config->setLow(Pins.CFG3);
	HAL.IOs->config->setLow(Pins.CFG4);
	HAL.IOs->config->setLow(Pins.CFG5);
//	HAL.IOs->config->setLow(Pins.CFG6);
//	HAL.IOs->config->setLow(Pins.CFG7);

	HAL.IOs->config->setLow(Pins.IREF_R2);
	HAL.IOs->config->setLow(Pins.IREF_R3);
	HAL.IOs->config->setHigh(Pins.nSLEEP);
	HAL.IOs->config->setHigh(Pins.DRV_ENN);


	HAL.IOs->config->toInput(Pins.ERROR);
	HAL.IOs->config->toInput(Pins.INDEX);


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

	reset();

}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->reset(Pins.CFG0);
	HAL.IOs->config->reset(Pins.CFG1);
	HAL.IOs->config->reset(Pins.CFG2);
	HAL.IOs->config->reset(Pins.CFG3);
	HAL.IOs->config->reset(Pins.CFG4);
	HAL.IOs->config->reset(Pins.CFG5);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.ERROR);
	HAL.IOs->config->reset(Pins.INDEX);
	HAL.IOs->config->reset(Pins.STEP);
	HAL.IOs->config->reset(Pins.IREF_R2);
	HAL.IOs->config->reset(Pins.IREF_R3);

	StepDir_deInit();
	//Timer.deInit();
}

static uint8_t reset()
{
	if(StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
		return 0;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 20000);
	StepDir_setAcceleration(0, 25000);
	enableDriver(DRIVER_ENABLE);
	return 1;
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
