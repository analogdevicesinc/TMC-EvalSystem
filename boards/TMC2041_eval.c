#include "Board.h"
#include "tmc/ic/TMC2041/TMC2041.h"

#include "../tmc/StepDir.h"

#define VM_MIN  50   // 5V
#define VM_MAX  286  // 286V (26V + 10%)

// TODO: Limits
#undef  TMC2041_MAX_VELOCITY
#define TMC2041_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

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

static SPIChannelTypeDef *TMC2041_SPIChannel;
static TMC2041TypeDef TMC2041;
static ConfigurationTypeDef *TMC2041_config;

typedef struct
{
	IOPinTypeDef  *REFL1_STEP1;
	IOPinTypeDef  *REFL2_STEP2;
	IOPinTypeDef  *REFR1_DIR1;
	IOPinTypeDef  *REFR2_DIR2;
	IOPinTypeDef  *DRV_ENN;
	IOPinTypeDef  *SWIOP;
	IOPinTypeDef  *SWION;
	IOPinTypeDef  *SWSEL;
	IOPinTypeDef  *INT;
	IOPinTypeDef  *PP;
	IOPinTypeDef  *CSN;
} PinsTypeDef;

static PinsTypeDef Pins;

// Translate motor number to TMC2041TypeDef
// When using multiple ICs you can map them here
static inline TMC2041TypeDef *motorToIC(uint8 motor)
{
	UNUSED(motor);

	return &TMC2041;
}

// Translate channel number to SPI channel
// When using multiple ICs you can map them here
static inline SPIChannelTypeDef *channelToSPI(uint8 channel)
{
	UNUSED(channel);

	return TMC2041_SPIChannel;
}

// => SPI wrapper
void tmc2041_readWriteArray(uint8 channel, uint8 *data, size_t length)
{
	// Map the channel to the corresponding SPI channel
	channelToSPI(channel)->readWriteArray(&data[0], length);
}
// <= SPI wrapper

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= TMC2041_MOTORS)
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
	if(motor >= TMC2041_MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	if(motor >= TMC2041_MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;
	int tempValue;

	if(motor >= TMC2041_MOTORS)
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
		// todo XML 3: Velocity ist signed, nicht unsigned (LH)
		// Target speed
		if(readWrite == READ) {
			*value = StepDir_getTargetVelocity(motor);
		} else if(readWrite == WRITE) {
			StepDir_rotate(motor, *value);
		}
		break;
	case 3:
		// todo CHECK 3: min max actually velocity min and velocity max? (JE) #1
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
		// Maximum current
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_IHOLD_IRUN(motor), TMC2041_IRUN_MASK, TMC2041_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_IHOLD_IRUN(motor), TMC2041_IRUN_MASK, TMC2041_IRUN_SHIFT, *value); // todo CHECK 3: check functionality (ED) #1
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_IHOLD_IRUN(motor), TMC2041_IHOLD_MASK, TMC2041_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_IHOLD_IRUN(motor), TMC2041_IHOLD_MASK, TMC2041_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = (StepDir_getStatus(motor) & STATUS_TARGET_REACHED)? 1:0;
		} else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_VHIGHFS_MASK, TMC2041_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_VHIGHFS_MASK, TMC2041_VHIGHFS_SHIFT, *value);
		}
		break;
	case 29: // todo AP REM 2: Not documented in XML - delete? seems pointless anyways (LH)
		if(readWrite == READ) {
			//*value = StepDir.ch1->actualVelocity;
			*value = StepDir_getActualVelocity(0);
		} else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
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
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_MRES_MASK, TMC2041_MRES_SHIFT);
		} else if(readWrite == WRITE) {
			switch(*value)
			{
			case 1:    *value = 8;   break;
			case 2:    *value = 7;   break;
			case 4:    *value = 6;   break;
			case 8:    *value = 5;   break;
			case 16:   *value = 4;   break;
			case 32:   *value = 3;   break;
			case 64:   *value = 2;   break;
			case 128:  *value = 1;   break;
			case 256:  *value = 0;   break;
			default:   *value = -1;  break;
			}

			if(*value != -1)
			{
				TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_MRES_MASK, TMC2041_MRES_SHIFT, *value);
			}
			//else TMCL.reply->Status = REPLY_INVALID_VALUE;
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TBL_MASK, TMC2041_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TBL_MASK, TMC2041_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_CHM_MASK, TMC2041_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_CHM_MASK, TMC2041_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_DISFDCC_MASK, TMC2041_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_DISFDCC_MASK, TMC2041_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		tempValue = tmc2041_readInt(motorToIC(motor), TMC2041_CHOPCONF(motor));
		if(readWrite == READ) {
			if(tempValue & TMC2041_CHM_MASK) // Chopper hysteresis end
			{
				*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_HEND_MASK, TMC2041_HEND_SHIFT);
			}
			else // fast decay time
			{
				*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TFD_ALL_MASK, TMC2041_TFD_ALL_SHIFT);
				if(tempValue & TMC2041_TFD_3_MASK) // add MSB of fast decay time to *value
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			if(tempValue & TMC2041_CHM_MASK) // Chopper hysteresis end
			{
				TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_HEND_MASK, TMC2041_HEND_SHIFT, *value);
			}
			else // fast decay time
			{
				TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TFD_ALL_MASK, TMC2041_TFD_ALL_SHIFT, *value);
				TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TFD_3_MASK, TMC2041_TFD_3_SHIFT, (*value & (1<<3))? 1:0);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		tempValue = tmc2041_readInt(motorToIC(motor), TMC2041_CHOPCONF(motor));
		if(readWrite == READ) {
			if(tempValue & TMC2041_CHM_MASK) // Chopper hysteresis start
			{
				*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_HSTRT_MASK, TMC2041_HSTRT_SHIFT);
			}
			else // sine wave offset
			{
				*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_OFFSET_MASK, TMC2041_OFFSET_SHIFT);
			}
		} else if(readWrite == WRITE) {
			if(tempValue & TMC2041_CHM_MASK) // Chopper hysteresis start
			{
				TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_HSTRT_MASK, TMC2041_HSTRT_SHIFT, *value);
			}
			else // sine wave offset
			{
				TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_OFFSET_MASK, TMC2041_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TOFF_MASK, TMC2041_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_TOFF_MASK, TMC2041_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEIMIN_MASK, TMC2041_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEIMIN_MASK, TMC2041_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEDN_MASK, TMC2041_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEDN_MASK, TMC2041_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEMAX_MASK, TMC2041_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEMAX_MASK, TMC2041_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEUP_MASK, TMC2041_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEUP_MASK, TMC2041_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEMIN_MASK, TMC2041_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SEMIN_MASK, TMC2041_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SFILT_MASK, TMC2041_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SFILT_MASK, TMC2041_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SGT_MASK, TMC2041_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_COOLCONF(motor), TMC2041_SGT_MASK, TMC2041_SGT_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_VSENSE_MASK, TMC2041_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_VSENSE_MASK, TMC2041_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_DRVSTATUS(motor), TMC2041_CS_ACTUAL_MASK, TMC2041_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 181:
		// smartEnergy stall velocity
		if(readWrite == READ) {
			*value = StepDir_getStallGuardThreshold(motor);
		} else if(readWrite == WRITE) {
			StepDir_setStallGuardThreshold(motor, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_RNDTF_MASK, TMC2041_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2041_FIELD_WRITE(motorToIC(motor), TMC2041_CHOPCONF(motor), TMC2041_RNDTF_MASK, TMC2041_RNDTF_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC2041_FIELD_READ(motorToIC(motor), TMC2041_DRVSTATUS(motor), TMC2041_SG_RESULT_MASK, TMC2041_SG_RESULT_SHIFT);
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

static uint32 SAP(uint8 type, uint8 motor, int32 value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32 GAP(uint8 type, uint8 motor, int32 *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32 getLimit(AxisParameterLimit limit, uint8 type, uint8 motor, int32 *value)
{
	UNUSED(motor);
	u32 errors = TMC_ERROR_NONE;
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

static uint32 getMin(uint8 type, uint8 motor, int32 *value)
{
	return getLimit(LIMIT_MIN, type, motor, value);
}

static uint32 getMax(uint8 type, uint8 motor, int32 *value)
{
	return getLimit(LIMIT_MAX, type, motor, value);
}

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
{
	if(motor >= TMC2041_MOTORS)
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

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	tmc2041_writeInt(motorToIC(motor), address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	*value = tmc2041_readInt(motorToIC(motor), address);
}

static void periodicJob(uint32 tick)
{
	tmc2041_periodicJob(&TMC2041, tick);

	for(size_t motor = 0; motor < TMC2041_MOTORS; motor++)
	{
		StepDir_periodicJob(motor);
		// Read stallGuard status and pass it to StepDir for further handling
		StepDir_stallGuard(motor, TMC2041_FIELD_READ(motorToIC(motor), TMC2041_DRVSTATUS(motor), TMC2041_STALLGUARD_MASK, TMC2041_STALLGUARD_SHIFT) == 1);
	}
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
	case 1:  // read interrupt pin INT
		*value = (HAL.IOs->config->isHigh(Pins.INT)) ? 1 : 0;
		break;
	case 2:  // read position compare pin PP
		*value = (HAL.IOs->config->isHigh(Pins.PP)) ? 1 : 0;
		break;
	case 3:  // Read StepDir status bits
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
	HAL.IOs->config->reset(Pins.INT);
	HAL.IOs->config->reset(Pins.PP);

	StepDir_deInit();
	Timer.deInit();
}

static uint8 reset()
{
	for(uint8 motor = 0; motor < TMC2041_MOTORS; motor++)
		if(StepDir_getActualVelocity(motor) != 0)
			return 0;

	tmc2041_reset(&TMC2041);
	StepDir_init();
	StepDir_setPins(0, Pins.REFL1_STEP1, Pins.REFR1_DIR1, NULL);
	StepDir_setPins(1, Pins.REFL2_STEP2, Pins.REFR2_DIR2, NULL);

	return 1;
}

static uint8 restore()
{
	return tmc2041_restore(&TMC2041);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.DRV_ENN);
}

void TMC2041_init(void)
{
	tmc2041_init(&TMC2041, 1, Evalboards.ch2.config, &tmc2041_defaultRegisterResetState[0]);

	Pins.DRV_ENN      = &HAL.IOs->pins->DIO0;
	Pins.PP           = &HAL.IOs->pins->DIO4;
	Pins.INT          = &HAL.IOs->pins->DIO5;
	Pins.REFL1_STEP1  = &HAL.IOs->pins->DIO6;
	Pins.REFR1_DIR1   = &HAL.IOs->pins->DIO7;
	Pins.REFL2_STEP2  = &HAL.IOs->pins->DIO8;
	Pins.REFR2_DIR2   = &HAL.IOs->pins->DIO9;
	Pins.CSN          = &HAL.IOs->pins->SPI2_CSN0;

	HAL.IOs->config->toOutput(Pins.REFL1_STEP1);
	HAL.IOs->config->toOutput(Pins.REFR1_DIR1);
	HAL.IOs->config->toOutput(Pins.REFL2_STEP2);
	HAL.IOs->config->toOutput(Pins.REFR2_DIR2);
	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.CSN);

	TMC2041_SPIChannel = &HAL.SPI->ch2;
	TMC2041_SPIChannel->CSN = Pins.CSN;

	TMC2041_config = &TMCDriver.config;

	StepDir_init();
	StepDir_setPins(0, Pins.REFL1_STEP1, Pins.REFR1_DIR1, NULL);
	StepDir_setPins(1, Pins.REFL2_STEP2, Pins.REFR2_DIR2, NULL);

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
	Evalboards.ch2.numberOfMotors       = TMC2041_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.getMin               = getMin;
	Evalboards.ch2.getMax               = getMax;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);

	Timer.init();
	Timer.setDuty(TIMER_CHANNEL_1, 0);
}
