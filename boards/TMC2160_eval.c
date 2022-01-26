#include "tmc/StepDir.h"
#include "Board.h"
#include "tmc/ic/TMC2160/TMC2160.h"

#define TMC2160_EVAL_VM_MIN  50   // VM[V/10] min
#define TMC2160_EVAL_VM_MAX  480  // VM[V/10] max +5%

#undef  TMC2160_MAX_VELOCITY
#define TMC2160_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define TMC2160_DRVENN_DELAY 2000 // Driver enable delay preventing overload on power-on.

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
static void periodicJob(uint32_t tick);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);
static void deInit(void);
static uint8_t reset();
static uint8_t restore();
static void configCallback(TMC2160TypeDef *tmc2160, ConfigState state);
static void enableDriver(DriverState state);

static uint8_t init_state = 0;

extern IOPinTypeDef DummyPin;

typedef struct
{
	IOPinTypeDef *DRV_ENN;
	IOPinTypeDef *REFL_STEP;
	IOPinTypeDef *REFR_DIR;
	IOPinTypeDef *SPI_MODE;
	IOPinTypeDef *DCIN;
	IOPinTypeDef *DCEN;
	IOPinTypeDef *DCO;
	IOPinTypeDef *DIAG0;
	IOPinTypeDef *DIAG1;
} PinsTypeDef;

static PinsTypeDef Pins;

SPIChannelTypeDef *TMC2160_SPIChannel;
TMC2160TypeDef TMC2160;

// Translate motor number to TMC5130TypeDef
// When using multiple ICs you can map them here
static inline TMC2160TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC2160;
}

// Translate channel number to SPI channel
// When using multiple ICs you can map them here
static inline SPIChannelTypeDef *channelToSPI(uint8_t channel)
{
	UNUSED(channel);

	return TMC2160_SPIChannel;
}

// => SPI wrapper (also takes care of cover mode)
void tmc2160_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	if(Evalboards.ch1.fullCover != NULL)
	{
		UNUSED(channel);
		Evalboards.ch1.fullCover(&data[0], length);
	}
	else
	{
		// Map the channel to the corresponding SPI channel
		channelToSPI(channel)->readWriteArray(&data[0], length);
	}
}
// <= SPI wrapper

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= TMC2160_MOTORS)
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
	if(motor >= TMC2160_MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= TMC2160_MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC2160_MOTORS)
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
			switch(StepDir_getMode(motor)) {
			case STEPDIR_INTERNAL:
				*value = StepDir_getActualVelocity(motor);
				break;
			case STEPDIR_EXTERNAL:
			default:
				tempValue = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)TMC2160_FIELD_READ(motorToIC(motor), TMC2160_TSTEP, TMC2160_TSTEP_MASK, TMC2160_TSTEP_SHIFT));
				*value = (abs(tempValue) < 20) ? 0 : tempValue;
				break;
			}
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
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_IHOLD_IRUN, TMC2160_IRUN_MASK, TMC2160_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_IHOLD_IRUN, TMC2160_IRUN_MASK, TMC2160_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_IHOLD_IRUN, TMC2160_IHOLD_MASK, TMC2160_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_IHOLD_IRUN, TMC2160_IHOLD_MASK, TMC2160_IHOLD_SHIFT, *value);
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
	case 21:
		// todo CHECK 3: Funktionalität prüfen (JE)
		// todo XML 3: XML Beschreibung hinzufügen (JE)
		if(readWrite == READ) {
			errors |= TMC_ERROR_TYPE;
		} else if(readWrite == WRITE) {
			tmc2160_writeInt(motorToIC(motor), TMC2160_TPOWERDOWN, *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			tempValue = tmc2160_readInt(motorToIC(motor), TMC2160_THIGH);
			*value = MIN(0xFFFFF, (1<<24) / ((tempValue)? tempValue:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc2160_writeInt(motorToIC(motor), TMC2160_THIGH, *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc2160_readInt(motorToIC(motor), TMC2160_VDCMIN);
		} else if(readWrite == WRITE) {
			tmc2160_writeInt(motorToIC(motor), TMC2160_VDCMIN, *value);
		}
		break;
	case 26:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_VHIGHFS_MASK, TMC2160_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_VHIGHFS_MASK, TMC2160_VHIGHFS_SHIFT, *value);
		}
		break;
	case 27:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_VHIGHCHM_MASK, TMC2160_VHIGHCHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_VHIGHCHM_MASK, TMC2160_VHIGHCHM_SHIFT, *value);
		}
		break;
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			tempValue = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)TMC2160_FIELD_READ(motorToIC(motor), TMC2160_TSTEP, TMC2160_TSTEP_MASK, TMC2160_TSTEP_SHIFT));
			*value = (abs(tempValue) < 20) ? 0 : tempValue;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 35:
		// Global current scaler
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_GLOBAL_SCALER, TMC2160_GLOBAL_SCALER_MASK, TMC2160_GLOBAL_SCALER_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_GLOBAL_SCALER, TMC2160_GLOBAL_SCALER_MASK, TMC2160_GLOBAL_SCALER_SHIFT, *value);
		}
		break;
	case 50: // StepDir internal(0)/external(1)
		if(readWrite == READ) {
			*value = StepDir_getMode(motor);
		} else if(readWrite == WRITE) {
			StepDir_setMode(motor, *value);

			if (*value == 0)
			{
				// Register the pins
				StepDir_setPins(0, Pins.REFL_STEP, Pins.REFR_DIR, NULL);
			}
			else
			{
				// Unregister the pins
				StepDir_setPins(0, &DummyPin, &DummyPin, NULL);

				// Set the Pins to HIGH - this allows external StepDir input
				HAL.IOs->config->setHigh(Pins.REFL_STEP);
				HAL.IOs->config->setHigh(Pins.REFR_DIR);
			}
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
			*value = 256 >> TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_MRES_MASK, TMC2160_MRES_SHIFT);
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
				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_MRES_MASK, TMC2160_MRES_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_TBL_MASK, TMC2160_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_TBL_MASK, TMC2160_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_CHM_MASK, TMC2160_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_CHM_MASK, TMC2160_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_DISFDCC_MASK, TMC2160_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_DISFDCC_MASK, TMC2160_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			if(tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF) & (1<<14))
			{
				*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_HEND_MASK, TMC2160_HEND_SHIFT);
			}
			else
			{
				tempValue = tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF);
				*value = (tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF) >> 4) & 0x07;
				if(tempValue & (1<<11))
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			if(tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF) & (1<<14))
			{
				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_HEND_MASK, TMC2160_HEND_SHIFT, *value);
			}
			else
			{
				tempValue = tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF);

				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_TFD_3_MASK, TMC2160_TFD_3_SHIFT, (*value & (1<<3))? 1:0);
				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_TFD_ALL_MASK, TMC2160_TFD_ALL_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		if(readWrite == READ) {
			if(tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF) & (1<<14))
			{
				*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_HSTRT_MASK, TMC2160_HSTRT_SHIFT);
			}
			else
			{
				tempValue = tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF);
				*value = (tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF) >> 7) & 0x0F;
				if(tempValue & (1<<11))
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			if(tmc2160_readInt(motorToIC(motor), TMC2160_CHOPCONF) & (1<<14))
			{
				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_HSTRT_MASK, TMC2160_HSTRT_SHIFT, *value);
			}
			else
			{
				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_OFFSET_MASK, TMC2160_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_TOFF_MASK, TMC2160_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_TOFF_MASK, TMC2160_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEIMIN_MASK, TMC2160_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEIMIN_MASK, TMC2160_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEDN_MASK, TMC2160_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEDN_MASK, TMC2160_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEMAX_MASK, TMC2160_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEMAX_MASK, TMC2160_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEUP_MASK, TMC2160_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEUP_MASK, TMC2160_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEMIN_MASK, TMC2160_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SEMIN_MASK, TMC2160_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SFILT_MASK, TMC2160_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SFILT_MASK, TMC2160_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SGT_MASK, TMC2160_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_COOLCONF, TMC2160_SGT_MASK, TMC2160_SGT_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_VSENSE_MASK, TMC2160_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_VSENSE_MASK, TMC2160_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_DRV_STATUS, TMC2160_CS_ACTUAL_MASK, TMC2160_CS_ACTUAL_SHIFT);
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
			tempValue = tmc2160_readInt(motorToIC(motor), TMC2160_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((tempValue)? tempValue:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc2160_writeInt(motorToIC(motor), TMC2160_TCOOLTHRS, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_RNDTF_MASK, TMC2160_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_CHOPCONF, TMC2160_RNDTF_MASK, TMC2160_RNDTF_SHIFT, *value);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			tempValue = tmc2160_readInt(motorToIC(motor), TMC2160_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((tempValue)? tempValue:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc2160_writeInt(motorToIC(motor), TMC2160_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_GRAD_MASK, TMC2160_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_GRAD_MASK, TMC2160_PWM_GRAD_SHIFT, *value);

			// Enable/disable stealthChop accordingly
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_GCONF, TMC2160_EN_PWM_MODE_MASK, TMC2160_EN_PWM_MODE_SHIFT, (*value) ? 1 : 0);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_OFS_MASK, TMC2160_PWM_OFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_OFS_MASK, TMC2160_PWM_OFS_SHIFT, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_FREQ_MASK, TMC2160_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_FREQ_MASK, TMC2160_PWM_FREQ_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 192:
		// PWM autoscale
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_AUTOSCALE_MASK, TMC2160_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_PWMCONF, TMC2160_PWM_AUTOSCALE_MASK, TMC2160_PWM_AUTOSCALE_SHIFT, (*value)? 1:0);
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_PWMCONF, TMC2160_FREEWHEEL_MASK, TMC2160_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2160_FIELD_WRITE(motorToIC(motor), TMC2160_PWMCONF, TMC2160_FREEWHEEL_MASK, TMC2160_FREEWHEEL_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC2160_FIELD_READ(motorToIC(motor), TMC2160_DRV_STATUS, TMC2160_SG_RESULT_MASK, TMC2160_SG_RESULT_SHIFT);
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
	tmc2160_writeInt(motorToIC(motor), address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc2160_readInt(motorToIC(motor), address);
}

static void periodicJob(uint32_t tick)
{
	static uint32_t old_tick = 0;

	switch(init_state) {
	case 1: // Initialization done
		old_tick = tick;
		init_state = 2;
		break;
	case 2: // Caps loaded, enabling MOSFETs
		if((tick - old_tick) > TMC2160_DRVENN_DELAY) {
			enableDriver(DRIVER_USE_GLOBAL_ENABLE);
			init_state = 3;
		}
		break;
	}

	tmc2160_periodicJob(&TMC2160, tick);

	StepDir_periodicJob(0);

	uint8_t status = StepDir_getStatus(0);
	// Already stalled -> skip stallGuard check
	if(status & STATUS_STALLED)
		return;

	// Stallguard not enabled -> skip stallGuard check
	if(!(status & STATUS_STALLGUARD_ACTIVE))
		return;

	// Check stallGuard
	if(tmc2160_readInt(&TMC2160, TMC2160_DRV_STATUS) & TMC2160_STALLGUARD_MASK)
		StepDir_stop(0, STOP_STALL);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	uint32_t uvalue;

	switch(type)
	{
	case 0:	// set analogue current duty
		/*
		 * Current will be defined by analogue value voltage or current signal. In any case this function
		 * will generate a analogue voltage by PWM for up to 50% duty and a switch for the other 50%.
		 * The reference voltage will be AIN_REF = VCC_IO * value/20000 with value = {0..20000}
		 */

		uvalue = (uint32_t) *value;

		if(uvalue <= 20000)
		{
			//HAL.IOs->config->setToState(Pins.AIN_REF_SW, (uvalue > 10000) ? IOS_HIGH : IOS_LOW);
			Timer.setDuty(TIMER_CHANNEL_1, ((float)(uvalue % 10001)) / TIMER_MAX);
		}
		else
		{
			errors |= TMC_ERROR_VALUE;
		}
		break;
	case 1:	// Use internal clock
		/*
		 * Internel clock will be enabled by calling this function with a value != 0 and unpower and repower the motor supply while keeping usb connected.
		 */
		if(*value)
			HAL.IOs->config->setToState(&HAL.IOs->pins->CLK16, IOS_LOW);
		else
			HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
		break;
	case 2:	// writing a register at address = motor with value = value and reading back the value
		// DO NOT USE!
		// todo BUG 3: using the motor for both address and motor value can lead to trying to get struct pointer from an array outside its bounds
		//             and then loading a function pointer from that. That is pretty a much guaranteed crash and/or hard fault!!! (LH) #1

		//tmc2160_writeInt(motorToIC(motor), motor, *value);
		//*value = tmc2160_readInt(motorToIC(motor), motor);
		break;
	case 3:	 // set DC_EN
		HAL.IOs->config->setToState(Pins.DCEN, (*value) ? IOS_HIGH : IOS_LOW);
		break;
	case 4:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	case 5:  // When running with the TMC4330 v1.2: CFG4 is routed from DIO6 (Motion Controller) to DIO13 (Driver)
		Pins.DCEN = &HAL.IOs->pins->DIO6;
		HAL.IOs->config->toOutput(Pins.DCEN);
		break;
	case 6:
		enableDriver(DRIVER_USE_GLOBAL_ENABLE);
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	if(motor >= TMC2160_MOTORS)
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
	HAL.IOs->config->setHigh(Pins.DRV_ENN);	// DISABLE DRIVER

	HAL.IOs->config->reset(Pins.REFL_STEP);
	HAL.IOs->config->reset(Pins.REFR_DIR);
	HAL.IOs->config->reset(Pins.SPI_MODE);
	HAL.IOs->config->reset(Pins.DCIN);
	HAL.IOs->config->reset(Pins.DCEN);
	HAL.IOs->config->reset(Pins.DCO);
	HAL.IOs->config->reset(Pins.DIAG0);
	HAL.IOs->config->reset(Pins.DIAG1);
	//HAL.IOs->config->reset(Pins.DRV_ENN);

	StepDir_deInit();
	Timer.deInit();
	init_state = 0;
}

static uint8_t reset()
{
	if(StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
		return 0;

	tmc2160_reset(&TMC2160);

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.REFL_STEP, Pins.REFR_DIR, NULL);

	return 1;
}

static uint8_t restore()
{
	return tmc2160_restore(&TMC2160);
}

static void configCallback(TMC2160TypeDef *tmc2160, ConfigState state)
{
	if(state == CONFIG_RESET)
	{	// Change hardware-preset registers here
		tmc2160_writeInt(tmc2160, TMC2160_PWMCONF, 0xC40C001E);
		tmc2160_writeInt(tmc2160, TMC2160_DRV_CONF, 0x00080400);
		//tmc2160_writeInt(tmc2160, TMC2160_PWMCONF, 0x000504C8);

		tmc2160_fillShadowRegisters(tmc2160);
	}
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

void TMC2160_init(void)
{
	tmc2160_init(&TMC2160, 1, Evalboards.ch2.config, &tmc2160_defaultRegisterResetState[0]);
	tmc2160_setCallback(&TMC2160, configCallback);

	Pins.DRV_ENN = &HAL.IOs->pins->DIO0;
	Pins.REFL_STEP = &HAL.IOs->pins->DIO6;
	Pins.REFR_DIR = &HAL.IOs->pins->DIO7;
	Pins.SPI_MODE = &HAL.IOs->pins->DIO11;
	Pins.DCIN = &HAL.IOs->pins->DIO12;
	Pins.DCEN = &HAL.IOs->pins->DIO13;
	Pins.DCO = &HAL.IOs->pins->DIO14;
	Pins.DIAG0 = &HAL.IOs->pins->DIO15;
	Pins.DIAG1 = &HAL.IOs->pins->DIO16;

	//HAL.IOs->config->toInput(Pins.DIAG0);
	//HAL.IOs->config->toInput(Pins.DIAG1);
	HAL.IOs->config->toInput(Pins.DCO);

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.REFL_STEP);
	HAL.IOs->config->toOutput(Pins.REFR_DIR);
	HAL.IOs->config->toOutput(Pins.SPI_MODE);
	HAL.IOs->config->toOutput(Pins.DCEN);
	HAL.IOs->config->toOutput(Pins.DCIN);

	HAL.IOs->config->setHigh(Pins.SPI_MODE);

	HAL.IOs->config->setLow(Pins.DCO);
	HAL.IOs->config->setLow(Pins.DCIN);

//	HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
//	HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);

	TMC2160_SPIChannel       = &HAL.SPI->ch2;
	TMC2160_SPIChannel->CSN  = &HAL.IOs->pins->SPI2_CSN0;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.REFL_STEP, Pins.REFR_DIR, NULL);

	Evalboards.ch2.type = (void *)&TMC2160;

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
	Evalboards.ch2.numberOfMotors       = TMC2160_MOTORS;
	Evalboards.ch2.VMMin                = TMC2160_EVAL_VM_MIN;
	Evalboards.ch2.VMMax                = TMC2160_EVAL_VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.getMin               = getMin;
	Evalboards.ch2.getMax               = getMax;

	init_state = 1;

	Timer.init();
	Timer.setDuty(TIMER_CHANNEL_1, 0);
}
