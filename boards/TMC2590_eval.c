#include "tmc/StepDir.h"
#include "Board.h"
#include "tmc/ic/TMC2590/TMC2590.h"

#undef  TMC2590_MAX_VELOCITY
#define TMC2590_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  600  // VM[V/10] max +10%

#define MOTORS 1

#define DEFAULT_MOTOR 0

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value);
static void writeRegister(uint8_t motor, uint8_t address, int32_t value);
static void readRegister(uint8_t motor, uint8_t address, int32_t *value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);
static void deInit(void);
static void periodicJob(uint32_t tick);
static uint8_t reset();
static uint8_t restore();
static void enableDriver(DriverState state);

static uint32_t compatibilityMode = 1;

static SPIChannelTypeDef *TMC2590_SPIChannel;
static TMC2590TypeDef TMC2590;
static ConfigurationTypeDef *TMC2590_config;

// Translate motor number to TMC2590TypeDef
// When using multiple ICs you can map them here
static inline TMC2590TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC2590;
}

// Translate channel number to SPI channel
// When using multiple ICs you can map them here
static inline SPIChannelTypeDef *channelToSPI(uint8_t channel)
{
	UNUSED(channel);

	return TMC2590_SPIChannel;
}

// SPI Wrapper for API
void tmc2590_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	if(Evalboards.ch1.fullCover != NULL) {
		UNUSED(channel);
		Evalboards.ch1.fullCover(&data[0], length);
	} else {
		channelToSPI(channel)->readWriteArray(data, length);
	}
}

typedef struct
{
	IOPinTypeDef  *CSN;
	IOPinTypeDef  *STEP;
	IOPinTypeDef  *DIR;
	IOPinTypeDef  *ENN;
	IOPinTypeDef  *SG_TST;
	IOPinTypeDef  *TEMP_BRIDGE;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:	// disable continuos read/write mode - used in BoardAssignment.c for the combination TMC43XX + TMC2590
		// In continuos read/write mode settings will be continously written to TMC2590 and all replies are requested rotatory.
		// It's the default mode to prevent TMC2590 from loosing setting on brownout and being alway up to date with all chip states.
		TMC2590.continuousModeEnable = *value ? 0 : 1;
		break;
	case 1:	// disable compatibility mode
		// per default compability mode is enabled,
		// saying firmware works with orl TMC2590-Eval Tool
		// e.g. stallGuard value is only
		compatibilityMode = *value ? 0 : 1;
		break;
	case 2:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}

	return errors;
}

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
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = TMC2590.runCurrentScale;
		} else if(readWrite == WRITE) {
			TMC2590.runCurrentScale = *value;
			if(TMC2590.isStandStillCurrent == false)
			{
				TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SGCSCONF, TMC2590_CS_MASK, TMC2590_CS_SHIFT, TMC2590.runCurrentScale);
			}
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC2590.standStillCurrentScale;
		} else if(readWrite == WRITE) {
			TMC2590.standStillCurrentScale = *value;
			if(TMC2590.isStandStillCurrent == true)
			{
				TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SGCSCONF, TMC2590_CS_MASK, TMC2590_CS_SHIFT, TMC2590.standStillCurrentScale);
			}
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
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			*value = StepDir_getActualVelocity(motor); // todo CHECK AP 2: Basically a duplicate of AP 3 - remove? (LH)
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
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
			*value = 8 - TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCTRL | TMC2590_WRITE_BIT, TMC2590_MRES_MASK, TMC2590_MRES_SHIFT);
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
				TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCTRL, TMC2590_MRES_MASK, TMC2590_MRES_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 160:
		// Microstep Interpolation
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCTRL | TMC2590_WRITE_BIT, TMC2590_INTPOL_MASK, TMC2590_INTPOL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCTRL, TMC2590_INTPOL_MASK, TMC2590_INTPOL_SHIFT, *value);
		}
		break;
	case 161:
		// Double Edge Steps
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCTRL | TMC2590_WRITE_BIT, TMC2590_DEDGE_MASK, TMC2590_DEDGE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCTRL, TMC2590_DEDGE_MASK, TMC2590_DEDGE_SHIFT, *value);
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF | TMC2590_WRITE_BIT, TMC2590_TBL_MASK, TMC2590_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_TBL_MASK, TMC2590_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF | TMC2590_WRITE_BIT, TMC2590_CHM_MASK, TMC2590_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_CHM_MASK, TMC2590_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF | TMC2590_WRITE_BIT, TMC2590_HDEC_MASK, TMC2590_HDEC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_HDEC_MASK, TMC2590_HDEC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF | TMC2590_WRITE_BIT, TMC2590_HEND_MASK, TMC2590_HEND_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_HEND_MASK, TMC2590_HEND_SHIFT, *value);
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF | TMC2590_WRITE_BIT, TMC2590_HSTRT_MASK, TMC2590_HSTRT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_HSTRT_MASK, TMC2590_HSTRT_SHIFT, *value);
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF | TMC2590_WRITE_BIT, TMC2590_TOFF_MASK, TMC2590_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_TOFF_MASK, TMC2590_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEIMIN_MASK, TMC2590_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEIMIN_MASK, TMC2590_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEDN_MASK, TMC2590_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEDN_MASK, TMC2590_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEMAX_MASK, TMC2590_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEMAX_MASK, TMC2590_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEUP_MASK, TMC2590_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SMARTEN, TMC2590_SEUP_MASK, TMC2590_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC2590.coolStepActiveValue;
		} else if(readWrite == WRITE) {
			TMC2590.coolStepActiveValue = *value;
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_SGCSCONF, TMC2590_SFILT_MASK, TMC2590_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SGCSCONF, TMC2590_SFILT_MASK, TMC2590_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_SGCSCONF, TMC2590_SGT_MASK, TMC2590_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_SGCSCONF, TMC2590_SGT_MASK, TMC2590_SGT_SHIFT, *value);
		}
		break;
	case 175:
		// Slope control, high side
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_SLPH_MASK, TMC2590_SLPH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_SLPH_MASK, TMC2590_SLPH_SHIFT, *value);
		}
		break;
	case 176:
		// Slope control, low side
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_SLPL_MASK, TMC2590_SLPL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_SLPL_MASK, TMC2590_SLPL_SHIFT, *value);
		}
		break;
	case 177:
		// Short to Ground Protection
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_DISS2G_MASK, TMC2590_DISS2G_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_DISS2G_MASK, TMC2590_DISS2G_SHIFT, *value);
		}
		break;
	case 178:
		// Short-to-ground detection timer
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_TS2G_MASK, TMC2590_TS2G_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_TS2G_MASK, TMC2590_TS2G_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_VSENSE_MASK, TMC2590_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_VSENSE_MASK, TMC2590_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_RESPONSE2, TMC2590_SE_MASK, TMC2590_SE_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		if(readWrite == READ) {
			*value = StepDir_getStallGuardThreshold(motor);
		} else if(readWrite == WRITE) {
			StepDir_setStallGuardThreshold(motor, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = TMC2590.coolStepThreshold;
		} else if(readWrite == WRITE) {
			TMC2590.coolStepThreshold = *value;
		}
		break;
	case 183:
		// Disable step/dir interface
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_SDOFF_MASK, TMC2590_SDOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_SDOFF_MASK, TMC2590_SDOFF_SHIFT, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_RNDTF_MASK, TMC2590_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_CHOPCONF, TMC2590_RNDTF_MASK, TMC2590_RNDTF_SHIFT, *value);
		}
		break;
	case 185:
		// Reserved test mode: leave undocumented?
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_DRVCONF, TMC2590_TST_MASK, TMC2590_TST_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2590_FIELD_WRITE(motorToIC(motor), TMC2590_DRVCONF, TMC2590_TST_MASK, TMC2590_TST_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = (compatibilityMode) ?
					TMC2590_FIELD_READ(motorToIC(motor), TMC2590_RESPONSE2, TMC2590_SGU_MASK, TMC2590_SGU_SHIFT)<<5 :
					TMC2590_FIELD_READ(motorToIC(motor), TMC2590_RESPONSE1, TMC2590_SG2_MASK, TMC2590_SG2_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 208:
		// Status Flags
		if(readWrite == READ) {
			*value = TMC2590_FIELD_READ(motorToIC(motor), TMC2590_RESPONSE_LATEST, TMC2590_STATUS_MASK, TMC2590_STATUS_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// Power Down Delay
		if(readWrite == READ) {
			*value = TMC2590.standStillTimeout;
		} else if(readWrite == WRITE) {
			TMC2590.standStillTimeout = *value;
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

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	tmc2590_writeInt(motorToIC(motor), address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc2590_readInt(motorToIC(motor), address);
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(motor)
	{
	case 0:
		*value = StepDir_getActualVelocity(motor);
		break;
	default:
		return TMC_ERROR_MOTOR;
		break;
	}
	return TMC_ERROR_NONE;
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);

	HAL.IOs->config->setHigh(Pins.ENN);

	HAL.IOs->config->reset(Pins.CSN);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.ENN);
	HAL.IOs->config->reset(Pins.SG_TST);
	HAL.IOs->config->reset(Pins.STEP);

	StepDir_deInit();
}

static void periodicJob(uint32_t tick)
{
	static uint8_t lastCoolStepState = 0;

	tmc2590_periodicJob(&TMC2590, tick);
	StepDir_periodicJob(DEFAULT_MOTOR);

	uint8_t currCoolStepState = (abs(StepDir_getActualVelocity(DEFAULT_MOTOR)) >= TMC2590.coolStepThreshold);
	if(currCoolStepState != lastCoolStepState)
	{
		uint8_t value = (currCoolStepState)? TMC2590.coolStepActiveValue : TMC2590.coolStepInactiveValue;
		TMC2590_FIELD_WRITE(&TMC2590, TMC2590_SMARTEN, TMC2590_SEMIN_MASK, TMC2590_SEMIN_SHIFT, value);

		lastCoolStepState = currCoolStepState;
	}
}

static uint8_t reset()
{
	if(StepDir_getActualVelocity(0) != 0)
		return 0;

	tmc2590_reset(&TMC2590);
	compatibilityMode = 1;
	enableDriver(DRIVER_USE_GLOBAL_ENABLE);

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.SG_TST);

	return 1;
}

static uint8_t restore()
{
	return tmc2590_restore(&TMC2590);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.ENN);
}

void TMC2590_init(void)
{
	compatibilityMode = 1;

	tmc2590_init(&TMC2590, 0, Evalboards.ch2.config, &tmc2590_defaultRegisterResetState[0]);

	Pins.ENN     = &HAL.IOs->pins->DIO0;
	Pins.SG_TST  = &HAL.IOs->pins->DIO1;
	Pins.STEP    = &HAL.IOs->pins->DIO6;
	Pins.DIR     = &HAL.IOs->pins->DIO7;
	Pins.CSN     = &HAL.IOs->pins->SPI2_CSN0;

	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
	HAL.IOs->config->toOutput(Pins.ENN);
	HAL.IOs->config->toInput(Pins.SG_TST);
	HAL.IOs->config->toOutput(Pins.CSN);

#if defined(Startrampe)
	Pins.TEMP_BRIDGE	= &HAL.IOs->pins->AIN0;
	HAL.IOs->config->reset(Pins.TEMP_BRIDGE);
#endif

	TMC2590_SPIChannel = &HAL.SPI->ch2;
	TMC2590_SPIChannel->CSN = Pins.CSN;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.SG_TST);

	TMC2590_config = Evalboards.ch2.config;

	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->reset        = reset;

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
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.getMin               = getMin;
	Evalboards.ch2.getMax               = getMax;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
