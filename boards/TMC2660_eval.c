#include "tmc/StepDir.h"

#include "Board.h"
#include "tmc/ic/TMC2660/TMC2660.h"

#undef  TMC2660_MAX_VELOCITY
#define TMC2660_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define ERRORS_I_STS          (1<<0)  // stand still current too high
#define ERRORS_I_TIMEOUT_STS  (1<<1)  // current limited in stand still to prevent driver from demage

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  600  // VM[V/10] max +10%

#define MOTORS 1

#define I_STAND_STILL 5
#define T_STAND_STILL 1000

#define DEFAULT_MOTOR 0

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static void readRegister(uint8_t motor, uint8_t address,	int32_t *value);
static void writeRegister(uint8_t motor, uint8_t address, int32_t value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static void on_standstill_changed(uint8_t newStandstill);

void tmc2660_writeInt(uint8_t motor, uint8_t address, int value);
uint32_t tmc2660_readInt(uint8_t motor, uint8_t address);

static uint32_t compatibilityMode = 1;

static SPIChannelTypeDef *TMC2660_SPIChannel;
static TMC2660TypeDef TMC2660;
static ConfigurationTypeDef *TMC2660_config;

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

void readWrite(uint32_t datagram)
{	// sending data (value) via spi to TMC262, coping written and received data to shadow register
	static uint8_t rdsel = 0; // number of expected read response

// if SGCONF should be written, check whether stand still, or run current should be used
//	if(TMC2660_GET_ADDRESS(datagram) == TMC2660_SGCSCONF)
//	{
//		datagram &= ~TMC2660_SET_CS(-1); // clear CS field
//		datagram |= (TMC2660.isStandStillCurrentLimit) ?  TMC2660_SET_CS(TMC2660.standStillCurrentScale) : TMC2660_SET_CS(TMC2660.runCurrentScale); // set current
//	}

// write value and read reply to shadow register
	TMC2660_config->shadowRegister[rdsel]  = TMC2660_SPIChannel->readWrite(datagram>>16, 0);
	TMC2660_config->shadowRegister[rdsel]  <<= 8;
	TMC2660_config->shadowRegister[rdsel]  |= TMC2660_SPIChannel->readWrite(datagram>>8, 0);
	TMC2660_config->shadowRegister[rdsel]  <<= 8;
	TMC2660_config->shadowRegister[rdsel]  |= TMC2660_SPIChannel->readWrite(datagram & 0xFF, 1);
	TMC2660_config->shadowRegister[rdsel]  >>= 4;

	TMC2660_config->shadowRegister[TMC2660_RESPONSE_LATEST] = TMC2660_config->shadowRegister[rdsel]; // copy value to latest field

// set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
	if(TMC2660_GET_ADDRESS(datagram) == TMC2660_DRVCONF)
		rdsel = TMC2660_GET_RDSEL(datagram);

// write store written value to shadow register
	TMC2660_config->shadowRegister[TMC2660_GET_ADDRESS(datagram) | TMC2660_WRITE_BIT ] = datagram;
}

void readImmediately(uint8_t rdsel)
{ // sets desired reply in DRVCONF register, resets it to previous settings whilst reading desired reply
	uint32_t value, drvConf;

// additional reading to keep all replies up to date
	value = tmc2660_readInt(0, TMC2660_WRITE_BIT | TMC2660_DRVCONF);  // buffer value amd  drvConf to write back later
	drvConf = value;
	value &= ~TMC2660_SET_RDSEL(-1);                              // clear RDSEL bits
	value |= TMC2660_SET_RDSEL(rdsel%3);                          // set rdsel
	readWrite(value);                                             // write to chip and readout reply
	readWrite(drvConf);                                           // write to chip and return desired reply
}

// => SPI wrapper
void tmc2660_writeInt(uint8_t motor, uint8_t address, int value)
{
	UNUSED(motor);

	// tmc2660_writeDatagram(address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
	value &= 0x0FFFFF;

	TMC2660_config->shadowRegister[0x7F & (address | TMC2660_WRITE_BIT)] = value;
	if(!TMC2660.continuousModeEnable)
		readWrite(TMC2660_DATAGRAM(address, value));
}

uint32_t tmc2660_readInt(uint8_t motor, uint8_t address)
{
	UNUSED(motor);

	if(!TMC2660.continuousModeEnable && !(address & TMC2660_WRITE_BIT))
		readImmediately(address);

	return TMC2660_config->shadowRegister[0x7F & address];
}

void tmc2660_readWrite(uint8_t motor, uint32_t value)
{
	UNUSED(motor);

	static uint8_t rdsel = 0; // number of expected read response

	// if SGCONF should be written, check whether stand still, or run current should be used
//	if(TMC2660_GET_ADDRESS(value) == TMC2660_SGCSCONF)
//	{
//		value &= ~TMC2660_SET_CS(-1); // clear CS field
//		value |= (TMC2660.isStandStillCurrentLimit) ?  TMC2660_SET_CS(TMC2660.standStillCurrentScale) : TMC2660_SET_CS(TMC2660.runCurrentScale); // set current
//	}

	// write value and read reply to shadow register
	TMC2660_config->shadowRegister[rdsel] = TMC2660_SPIChannel->readWrite(value>>16, 0);
	TMC2660_config->shadowRegister[rdsel] <<= 8;
	TMC2660_config->shadowRegister[rdsel] |= TMC2660_SPIChannel->readWrite(value>>8, 0);
	TMC2660_config->shadowRegister[rdsel] <<= 8;
	TMC2660_config->shadowRegister[rdsel] |= TMC2660_SPIChannel->readWrite(value & 0xFF, 1);
	TMC2660_config->shadowRegister[rdsel] >>= 4;

	TMC2660_config->shadowRegister[TMC2660_RESPONSE_LATEST] = TMC2660_config->shadowRegister[rdsel]; // copy value to latest field

	// set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
	if(TMC2660_GET_ADDRESS(value) == TMC2660_DRVCONF)
		rdsel = TMC2660_GET_RDSEL(value);

	// write store written value to shadow register
	TMC2660_config->shadowRegister[TMC2660_GET_ADDRESS(value) | TMC2660_WRITE_BIT ] = value;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:	// disable continuos read/write mode - used in BoardAssignment.c for the combination TMC43XX + TMC2660
		// In continuos read/write mode settings will be continously written to TMC2660 and all replies are requested rotatory.
		// It's the default mode to prevent TMC2660 from loosing setting on brownout and being alway up to date with all chip states.
		TMC2660.continuousModeEnable = *value ? 0 : 1;
		break;
	case 1:	// disable compatibility mode
		// per default compability mode is enabled,
		// saying firmware works with orl TMC2660-Eval Tool
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

	TMC2660.isStandStillCurrentLimit  = 0;
	TMC2660.isStandStillOverCurrent   = 0;

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
			*value = TMC2660.runCurrentScale;
		} else if(readWrite == WRITE) {
			TMC2660.runCurrentScale = *value;
			if(Evalboards.ch1.fullCover == NULL) {
				if(TMC2660_FIELD_READ(0, TMC2660_DRVCTRL, TMC2660_STST_MASK, TMC2660_STST_SHIFT) == 0)
					TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_CS_MASK, TMC2660_CS_SHIFT, TMC2660.runCurrentScale);
			} else {
				TMC2660.standStillCurrentScale = *value;
				TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_CS_MASK, TMC2660_CS_SHIFT, TMC2660.runCurrentScale);
			}
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC2660.standStillCurrentScale;
		} else if(readWrite == WRITE) {
			TMC2660.standStillCurrentScale = *value;
			if(Evalboards.ch1.fullCover == NULL) {
				if(TMC2660_FIELD_READ(0, TMC2660_DRVCTRL, TMC2660_STST_MASK, TMC2660_STST_SHIFT) == 1)
					TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_CS_MASK, TMC2660_CS_SHIFT, TMC2660.standStillCurrentScale);
			} else {
				TMC2660.runCurrentScale = *value;
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
			*value = 8 - TMC2660_FIELD_READ(0, TMC2660_DRVCTRL | TMC2660_WRITE_BIT, TMC2660_MRES_MASK, TMC2660_MRES_SHIFT);
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
				TMC2660_FIELD_UPDATE(0, TMC2660_DRVCTRL, TMC2660_MRES_MASK, TMC2660_MRES_SHIFT, *value);
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
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCTRL | TMC2660_WRITE_BIT, TMC2660_INTPOL_MASK, TMC2660_INTPOL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCTRL, TMC2660_INTPOL_MASK, TMC2660_INTPOL_SHIFT, *value);
		}
		break;
	case 161:
		// Double Edge Steps
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCTRL | TMC2660_WRITE_BIT, TMC2660_DEDGE_MASK, TMC2660_DEDGE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCTRL, TMC2660_DEDGE_MASK, TMC2660_DEDGE_SHIFT, *value);
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF | TMC2660_WRITE_BIT, TMC2660_TBL_MASK, TMC2660_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_TBL_MASK, TMC2660_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF | TMC2660_WRITE_BIT, TMC2660_CHM_MASK, TMC2660_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_CHM_MASK, TMC2660_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF | TMC2660_WRITE_BIT, TMC2660_HDEC_MASK, TMC2660_HDEC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_HDEC_MASK, TMC2660_HDEC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF | TMC2660_WRITE_BIT, TMC2660_HEND_MASK, TMC2660_HEND_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_HEND_MASK, TMC2660_HEND_SHIFT, *value);
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF | TMC2660_WRITE_BIT, TMC2660_HSTRT_MASK, TMC2660_HSTRT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_HSTRT_MASK, TMC2660_HSTRT_SHIFT, *value);
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF | TMC2660_WRITE_BIT, TMC2660_TOFF_MASK, TMC2660_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_TOFF_MASK, TMC2660_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SMARTEN, TMC2660_SEIMIN_MASK, TMC2660_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SMARTEN, TMC2660_SEIMIN_MASK, TMC2660_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SMARTEN, TMC2660_SEDN_MASK, TMC2660_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SMARTEN, TMC2660_SEDN_MASK, TMC2660_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SMARTEN, TMC2660_SEMAX_MASK, TMC2660_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SMARTEN, TMC2660_SEMAX_MASK, TMC2660_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SMARTEN, TMC2660_SEUP_MASK, TMC2660_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SMARTEN, TMC2660_SEUP_MASK, TMC2660_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SMARTEN, TMC2660_SEMIN_MASK, TMC2660_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SMARTEN, TMC2660_SEMIN_MASK, TMC2660_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SGCSCONF, TMC2660_SFILT_MASK, TMC2660_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_SFILT_MASK, TMC2660_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_SGCSCONF, TMC2660_SGT_MASK, TMC2660_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_SGT_MASK, TMC2660_SGT_SHIFT, *value);
		}
		break;
	case 175:
		// Slope control, high side
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_SLPH_MASK, TMC2660_SLPH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_SLPH_MASK, TMC2660_SLPH_SHIFT, *value);
		}
		break;
	case 176:
		// Slope control, low side
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_SLPL_MASK, TMC2660_SLPL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_SLPL_MASK, TMC2660_SLPL_SHIFT, *value);
		}
		break;
	case 177:
		// Short to Ground Protection
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_DISS2G_MASK, TMC2660_DISS2G_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_DISS2G_MASK, TMC2660_DISS2G_SHIFT, *value);
		}
		break;
	case 178:
		// Short-to-ground detection timer
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_TS2G_MASK, TMC2660_TS2G_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_TS2G_MASK, TMC2660_TS2G_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_VSENSE_MASK, TMC2660_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_VSENSE_MASK, TMC2660_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_RESPONSE2, TMC2660_SE_MASK, TMC2660_SE_SHIFT);
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
			*value = TMC2660.coolStepThreshold;
		} else if(readWrite == WRITE) {
			TMC2660.coolStepThreshold = *value;
		}
		break;
	case 183:
		// Disable step/dir interface
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_SDOFF_MASK, TMC2660_SDOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_SDOFF_MASK, TMC2660_SDOFF_SHIFT, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_CHOPCONF, TMC2660_RNDTF_MASK, TMC2660_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_CHOPCONF, TMC2660_RNDTF_MASK, TMC2660_RNDTF_SHIFT, *value);
		}
		break;
	case 185:
		// Reserved test mode: leave undocumented?
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_DRVCONF, TMC2660_TST_MASK, TMC2660_TST_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2660_FIELD_UPDATE(0, TMC2660_DRVCONF, TMC2660_TST_MASK, TMC2660_TST_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = (compatibilityMode) ?
					TMC2660_FIELD_READ(0, TMC2660_RESPONSE2, TMC2660_SGU_MASK, TMC2660_SGU_SHIFT)<<5 :
					TMC2660_FIELD_READ(0, TMC2660_RESPONSE1, TMC2660_SG2_MASK, TMC2660_SG2_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 208:
		// Status Flags
		if(readWrite == READ) {
			*value = TMC2660_FIELD_READ(0, TMC2660_RESPONSE_LATEST, TMC2660_STATUS_MASK, TMC2660_STATUS_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// Power Down Delay
		if(readWrite == READ) {
			*value = TMC2660.standStillTimeout;
		} else if(readWrite == WRITE) {
			TMC2660.standStillTimeout = *value;
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
	UNUSED(motor);
	tmc2660_writeInt(0, address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc2660_readInt(0, address);
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

static void on_standstill_changed(uint8_t newStandstill)
{
	if(newStandstill == true) {
		TMC2660.runCurrentScale = TMC2660_FIELD_READ(0, 0x0E, TMC2660_CS_MASK, TMC2660_CS_SHIFT);
		TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_CS_MASK, TMC2660_CS_SHIFT, TMC2660.standStillCurrentScale);
	} else if(newStandstill == false) {
		TMC2660.standStillCurrentScale = TMC2660_FIELD_READ(0, 0x0E, TMC2660_CS_MASK, TMC2660_CS_SHIFT);
		TMC2660_FIELD_UPDATE(0, TMC2660_SGCSCONF, TMC2660_CS_MASK, TMC2660_CS_SHIFT, TMC2660.runCurrentScale);
	}
}

static void periodicJob(uint32_t tick)
{
	static uint8_t lastCoolStepState = 0;
	static uint8_t lastStandstillState = 0;
	uint8_t stst;

	if(Evalboards.ch1.fullCover == NULL) { // Standstill detection only when not using an additional motion controller
		// Apply current settings
		if((stst = TMC2660_FIELD_READ(0, TMC2660_DRVCTRL, TMC2660_STST_MASK, TMC2660_STST_SHIFT)) != lastStandstillState) {
			on_standstill_changed(stst);
			lastStandstillState = stst;
		}
	}

	Evalboards.ch2.errors = (TMC2660.isStandStillOverCurrent) 	? (Evalboards.ch2.errors | ERRORS_I_STS) 			: (Evalboards.ch2.errors & ~ERRORS_I_STS);
	Evalboards.ch2.errors = (TMC2660.isStandStillCurrentLimit) 	? (Evalboards.ch2.errors | ERRORS_I_TIMEOUT_STS) 	: (Evalboards.ch2.errors & ~ERRORS_I_TIMEOUT_STS);

	uint8_t currCoolStepState = (abs(StepDir_getActualVelocity(DEFAULT_MOTOR)) >= TMC2660.coolStepThreshold);
	if(currCoolStepState != lastCoolStepState)
	{
		uint8_t value = (currCoolStepState)? TMC2660.coolStepActiveValue : TMC2660.coolStepInactiveValue;
		TMC2660_FIELD_UPDATE(0, TMC2660_SMARTEN, TMC2660_SEMIN_MASK, TMC2660_SEMIN_SHIFT, value);

		lastCoolStepState = currCoolStepState;
	}

	tmc2660_periodicJob(DEFAULT_MOTOR, tick, &TMC2660, TMC2660_config);
	StepDir_periodicJob(DEFAULT_MOTOR);
}

static uint8_t reset()
{
	if(StepDir_getActualVelocity(0) != 0)
		return 0;

	tmc2660_reset(&TMC2660, TMC2660_config);
	compatibilityMode = 1;
	enableDriver(DRIVER_USE_GLOBAL_ENABLE);

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.SG_TST);

	return 1;
}

static uint8_t restore()
{
	return tmc2660_restore(TMC2660_config);
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

void TMC2660_init(void)
{
	compatibilityMode = 1;

	tmc2660_initConfig(&TMC2660);

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

	TMC2660_SPIChannel = &HAL.SPI->ch2;
	TMC2660_SPIChannel->CSN = Pins.CSN;

	TMC2660.standStillCurrentScale  = I_STAND_STILL;
	TMC2660.standStillTimeout       = T_STAND_STILL;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.SG_TST);

	TMC2660_config = Evalboards.ch2.config;

	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->state        = CONFIG_READY; // Not used, leave this as CONFIG_READY to indicate chip not being busy
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
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}

