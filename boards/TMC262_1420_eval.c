#include "../tmc/StepDir.h"

#include "Board.h"
#include "tmc/ic/TMC262_1420/TMC262_1420.h"

#undef  TMC262_1420_MAX_VELOCITY
#define TMC262_1420_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

#define ERRORS_I_STS          (1<<0)  // stand still current too high
#define ERRORS_I_TIMEOUT_STS  (1<<1)  // current limited in stand still to prevent driver from demage

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  293  // VM[V/10] max +10%

#define MOTORS 1

#define I_STAND_STILL 5
#define T_STAND_STILL 1000

#define DEFAULT_MOTOR 0

static uint32 userFunction(uint8 type, uint8 motor, int32 *value);
static uint32 rotate(uint8 motor, int32 velocity);
static uint32 right(uint8 motor, int32 velocity);
static uint32 left(uint8 motor, int32 velocity);
static uint32 stop(uint8 motor);
static uint32 moveTo(uint8 motor, int32 position);
static uint32 moveBy(uint8 motor, int32 *ticks);
static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value);
static uint32 SAP(uint8 type, uint8 motor, int32 value);
static uint32 GAP(uint8 type, uint8 motor, int32 *value);
static uint32 getLimit(AxisParameterLimit limit, uint8 type, uint8 motor, int32 *value);
static uint32 getMin(uint8 type, uint8 motor, int32 *value);
static uint32 getMax(uint8 type, uint8 motor, int32 *value);
static void writeRegister(u8 motor, uint8 address, int32 value);
static void readRegister(u8 motor, uint8 address, int32 *value);
static uint32 getMeasuredSpeed(uint8 motor, int32 *value);
static void deInit(void);
static void periodicJob(uint32 tick);
static uint8 reset();
static uint8 restore();
static void enableDriver(DriverState state);

static void on_standstill_changed(uint8 newStandstill);

static uint32 compatibilityMode = 1;
static uint8 standstill = 1;

static SPIChannelTypeDef *TMC262_1420_SPIChannel;
static TMC262_1420TypeDef TMC262_1420;
static ConfigurationTypeDef *TMC262_1420_config;

// Translate motor number to TMC262_1420TypeDef
// When using multiple ICs you can map them here
static inline TMC262_1420TypeDef *motorToIC(uint8 motor)
{
	UNUSED(motor);

	return &TMC262_1420;
}

// Translate channel number to SPI channel
// When using multiple ICs you can map them here
static inline SPIChannelTypeDef *channelToSPI(uint8 channel)
{
	UNUSED(channel);

	return TMC262_1420_SPIChannel;
}

// SPI Wrapper for API
void tmc262_1420_readWriteArray(uint8 channel, uint8 *data, size_t length)
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

//static void readWrite(uint32 value)
//{	// sending data (value) via spi to TMC262, coping written and received data to shadow register
//	static uint8 rdsel = 0; // number of expected read response
//
//// if SGCONF should be written, check whether stand still, or run current should be used
//	if(TMC262_1420_GET_ADDRESS(value) == TMC262_1420_SGCSCONF)
//	{
//		value &= ~TMC262_1420_SET_CS(-1); // clear CS field
//		value |= (TMC262_1420.isStandStillCurrentLimit) ?  TMC262_1420_SET_CS(TMC262_1420.standStillCurrentScale) : TMC262_1420_SET_CS(TMC262_1420.runCurrentScale); // set current
//	}
//
//// write value and read reply to shadow register
//	TMC262_1420_config->shadowRegister[rdsel]  = TMC262_1420_SPIChannel->readWrite(value>>16, 0);
//	TMC262_1420_config->shadowRegister[rdsel]  <<= 8;
//	TMC262_1420_config->shadowRegister[rdsel]  |= TMC262_1420_SPIChannel->readWrite(value>>8, 0);
//	TMC262_1420_config->shadowRegister[rdsel]  <<= 8;
//	TMC262_1420_config->shadowRegister[rdsel]  |= TMC262_1420_SPIChannel->readWrite(value & 0xFF, 1);
//	TMC262_1420_config->shadowRegister[rdsel]  >>= 4;
//
//	TMC262_1420_config->shadowRegister[TMC262_1420_RESPONSE_LATEST] = TMC262_1420_config->shadowRegister[rdsel]; // copy value to latest field
//
//// set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
//	if(TMC262_1420_GET_ADDRESS(value) == TMC262_1420_DRVCONF)
//		rdsel = TMC262_1420_GET_RDSEL(value);
//
//// write store written value to shadow register
//	TMC262_1420_config->shadowRegister[TMC262_1420_GET_ADDRESS(value) | TMC262_1420_WRITE_BIT ] = value;
//}
//
//static void readImmediately(uint8 rdsel)
//{ // sets desired reply in DRVCONF register, resets it to previous settings whilst reading desired reply
//	uint32 value, drvConf;
//
//// additional reading to keep all replies up to date
//	value = tmc262_1420_readInt(&TMC262_1420, TMC262_1420_WRITE_BIT | TMC262_1420_DRVCONF);  // buffer value amd  drvConf to write back later
//	drvConf = value;
//	value &= ~TMC262_1420_SET_RDSEL(-1);                              // clear RDSEL bits
//	value |= TMC262_1420_SET_RDSEL(rdsel%3);                          // set rdsel
//	readWrite(value);                                             // write to chip and readout reply
//	readWrite(drvConf);                                           // write to chip and return desired reply
//}
//
//// => SPI wrapper
//void tmc262_1420_writeInt(uint8 motor, uint8 address, int value)
//{
//	UNUSED(motor);
//
//	// tmc262_1420_writeDatagram(address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
//	value &= 0x0FFFFF;
//
//	// store desired cs value, this can be overwritten by current limitation
//	if(TMC262_1420_GET_ADDRESS(value) == TMC262_1420_SGCSCONF)
//		TMC262_1420.runCurrentScale = TMC262_1420_GET_CS(value);
//
//	TMC262_1420_config->shadowRegister[0x7F & (address | TMC262_1420_WRITE_BIT)] = value;
//	if(!TMC262_1420.continuousModeEnable)
//		readWrite(value);
//}
//
//uint32 tmc262_1420_readInt(uint8 motor, uint8 address)
//{
//	UNUSED(motor);
//
//	if(!TMC262_1420.continuousModeEnable && !(address & TMC262_1420_WRITE_BIT))
//		readImmediately(address);
//
//	return TMC262_1420_config->shadowRegister[0x7F & address];
//}
//
//void tmc262_1420_readWrite(uint8 motor, uint32 value)
//{
//	UNUSED(motor);
//
//	static uint8 rdsel = 0; // number of expected read response
//
//	// if SGCONF should be written, check whether stand still, or run current should be used
//	if(TMC262_1420_GET_ADDRESS(value) == TMC262_1420_SGCSCONF)
//	{
//		value &= ~TMC262_1420_SET_CS(-1); // clear CS field
//		value |= (TMC262_1420.isStandStillCurrentLimit) ?  TMC262_1420_SET_CS(TMC262_1420.standStillCurrentScale) : TMC262_1420_SET_CS(TMC262_1420.runCurrentScale); // set current
//	}
//
//	// write value and read reply to shadow register
//	TMC262_1420_config->shadowRegister[rdsel] = TMC262_1420_SPIChannel->readWrite(value>>16, 0);
//	TMC262_1420_config->shadowRegister[rdsel] <<= 8;
//	TMC262_1420_config->shadowRegister[rdsel] |= TMC262_1420_SPIChannel->readWrite(value>>8, 0);
//	TMC262_1420_config->shadowRegister[rdsel] <<= 8;
//	TMC262_1420_config->shadowRegister[rdsel] |= TMC262_1420_SPIChannel->readWrite(value & 0xFF, 1);
//	TMC262_1420_config->shadowRegister[rdsel] >>= 4;
//
//	TMC262_1420_config->shadowRegister[TMC262_1420_RESPONSE_LATEST] = TMC262_1420_config->shadowRegister[rdsel]; // copy value to latest field
//
//	// set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
//	if(TMC262_1420_GET_ADDRESS(value) == TMC262_1420_DRVCONF)
//		rdsel = TMC262_1420_GET_RDSEL(value);
//
//	// write store written value to shadow register
//	TMC262_1420_config->shadowRegister[TMC262_1420_GET_ADDRESS(value) | TMC262_1420_WRITE_BIT ] = value;
//}



static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	uint32 errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:
		// disable continuos read/write mode - used in BoardAssignment.c for the combination TMC43XX + TMC262_1420
		// In continuos read/write mode settings will be continously written to TMC262_1420 and all replies are requested rotatory.
		// It's the default mode to prevent TMC262_1420 from loosing setting on brownout and being alway up to date with all chip states.
		TMC262_1420.continuousModeEnable = *value ? 0 : 1;
		break;
	case 1:
		// disable compatibility mode
		// per default compability mode is enabled,
		// saying firmware works with orl TMC262_1420-Eval Tool
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

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	TMC262_1420.isStandStillCurrentLimit  = 0;
	TMC262_1420.isStandStillOverCurrent   = 0;

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
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;

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
			*value = TMC262_1420.runCurrentScale;
		} else if(readWrite == WRITE) {
			TMC262_1420.runCurrentScale = *value;
			if(standstill == false)
				TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SGCSCONF, TMC262_1420_CS_MASK, TMC262_1420_CS_SHIFT, TMC262_1420.runCurrentScale);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC262_1420.standStillCurrentScale;
		} else if(readWrite == WRITE) {
			TMC262_1420.standStillCurrentScale = *value;
			if(standstill == true)
				TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SGCSCONF, TMC262_1420_CS_MASK, TMC262_1420_CS_SHIFT, TMC262_1420.standStillCurrentScale);
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
			*value = 8 - TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCTRL | TMC262_1420_WRITE_BIT, TMC262_1420_MRES_MASK, TMC262_1420_MRES_SHIFT);
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
				TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCTRL, TMC262_1420_MRES_MASK, TMC262_1420_MRES_SHIFT, *value);
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
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCTRL | TMC262_1420_WRITE_BIT, TMC262_1420_INTPOL_MASK, TMC262_1420_INTPOL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCTRL, TMC262_1420_INTPOL_MASK, TMC262_1420_INTPOL_SHIFT, *value);
		}
		break;
	case 161:
		// Double Edge Steps
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCTRL | TMC262_1420_WRITE_BIT, TMC262_1420_DEDGE_MASK, TMC262_1420_DEDGE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCTRL, TMC262_1420_DEDGE_MASK, TMC262_1420_DEDGE_SHIFT, *value);
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT, TMC262_1420_TBL_MASK, TMC262_1420_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_TBL_MASK, TMC262_1420_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT, TMC262_1420_CHM_MASK, TMC262_1420_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_CHM_MASK, TMC262_1420_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT, TMC262_1420_HDEC_MASK, TMC262_1420_HDEC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_HDEC_MASK, TMC262_1420_HDEC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT, TMC262_1420_HEND_MASK, TMC262_1420_HEND_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_HEND_MASK, TMC262_1420_HEND_SHIFT, *value);
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT, TMC262_1420_HSTRT_MASK, TMC262_1420_HSTRT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_HSTRT_MASK, TMC262_1420_HSTRT_SHIFT, *value);
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF | TMC262_1420_WRITE_BIT, TMC262_1420_TOFF_MASK, TMC262_1420_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_TOFF_MASK, TMC262_1420_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEIMIN_MASK, TMC262_1420_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEIMIN_MASK, TMC262_1420_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEDN_MASK, TMC262_1420_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEDN_MASK, TMC262_1420_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEMAX_MASK, TMC262_1420_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEMAX_MASK, TMC262_1420_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEUP_MASK, TMC262_1420_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SMARTEN, TMC262_1420_SEUP_MASK, TMC262_1420_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC262_1420.coolStepActiveValue;
		} else if(readWrite == WRITE) {
			TMC262_1420.coolStepActiveValue = *value;
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_SGCSCONF, TMC262_1420_SFILT_MASK, TMC262_1420_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SGCSCONF, TMC262_1420_SFILT_MASK, TMC262_1420_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_SGCSCONF, TMC262_1420_SGT_MASK, TMC262_1420_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_SGCSCONF, TMC262_1420_SGT_MASK, TMC262_1420_SGT_SHIFT, *value);
		}
		break;
	case 175:
		// Slope control, high side
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_SLPH_MASK, TMC262_1420_SLPH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_SLPH_MASK, TMC262_1420_SLPH_SHIFT, *value);
		}
		break;
	case 176:
		// Slope control, low side
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_SLPL_MASK, TMC262_1420_SLPL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_SLPL_MASK, TMC262_1420_SLPL_SHIFT, *value);
		}
		break;
	case 177:
		// Short to Ground Protection
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_DISS2G_MASK, TMC262_1420_DISS2G_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_DISS2G_MASK, TMC262_1420_DISS2G_SHIFT, *value);
		}
		break;
	case 178:
		// Short-to-ground detection timer
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_TS2G_MASK, TMC262_1420_TS2G_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_TS2G_MASK, TMC262_1420_TS2G_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_VSENSE_MASK, TMC262_1420_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_VSENSE_MASK, TMC262_1420_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_RESPONSE2, TMC262_1420_SE_MASK, TMC262_1420_SE_SHIFT);
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
			*value = TMC262_1420.coolStepThreshold;
		} else if(readWrite == WRITE) {
			TMC262_1420.coolStepThreshold = *value;
		}
		break;
	case 183:
		// Disable step/dir interface
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_SDOFF_MASK, TMC262_1420_SDOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_SDOFF_MASK, TMC262_1420_SDOFF_SHIFT, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_RNDTF_MASK, TMC262_1420_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_CHOPCONF, TMC262_1420_RNDTF_MASK, TMC262_1420_RNDTF_SHIFT, *value);
		}
		break;
	case 185:
		// Reserved test mode: leave undocumented?
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_TST_MASK, TMC262_1420_TST_SHIFT);
		} else if(readWrite == WRITE) {
			TMC262_1420_FIELD_UPDATE(motorToIC(motor), TMC262_1420_DRVCONF, TMC262_1420_TST_MASK, TMC262_1420_TST_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = (compatibilityMode) ?
					TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_RESPONSE2, TMC262_1420_SGU_MASK, TMC262_1420_SGU_SHIFT)<<5 :
					TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_RESPONSE1, TMC262_1420_SG2_MASK, TMC262_1420_SG2_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 208:
		// Status Flags
		if(readWrite == READ) {
			*value = TMC262_1420_FIELD_READ(motorToIC(motor), TMC262_1420_RESPONSE_LATEST, TMC262_1420_STATUS_MASK, TMC262_1420_STATUS_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// Power Down Delay
		if(readWrite == READ) {
			*value = TMC262_1420.standStillTimeout;
		} else if(readWrite == WRITE) {
			TMC262_1420.standStillTimeout = *value;
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

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	tmc262_1420_writeInt(motorToIC(motor), address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	*value = tmc262_1420_readInt(motorToIC(motor), address);
}

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
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

static void on_standstill_changed(uint8 newStandstill)
{
	if(newStandstill == true) {
		TMC262_1420.runCurrentScale = TMC262_1420_FIELD_READ(&TMC262_1420, TMC262_1420_SGCSCONF, TMC262_1420_CS_MASK, TMC262_1420_CS_SHIFT);
		TMC262_1420_FIELD_UPDATE(&TMC262_1420, TMC262_1420_SGCSCONF, TMC262_1420_CS_MASK, TMC262_1420_CS_SHIFT, TMC262_1420.standStillCurrentScale);
	} else if(newStandstill == false) {
		TMC262_1420.standStillCurrentScale = TMC262_1420_FIELD_READ(&TMC262_1420, TMC262_1420_SGCSCONF, TMC262_1420_CS_MASK, TMC262_1420_CS_SHIFT);
		TMC262_1420_FIELD_UPDATE(&TMC262_1420, TMC262_1420_SGCSCONF, TMC262_1420_CS_MASK, TMC262_1420_CS_SHIFT, TMC262_1420.runCurrentScale);
	}
}

static void periodicJob(uint32 tick)
{
	static uint8 lastCoolStepState = 0;
	uint8 stst;

	if((stst = TMC262_1420_FIELD_READ(&TMC262_1420, TMC262_1420_DRVCTRL, TMC262_1420_STST_MASK, TMC262_1420_STST_SHIFT)) != standstill) {
		on_standstill_changed(stst);
		standstill = stst;
	}

	Evalboards.ch2.errors = (TMC262_1420.isStandStillOverCurrent) 	? (Evalboards.ch2.errors | ERRORS_I_STS) 			: (Evalboards.ch2.errors & ~ERRORS_I_STS);
	Evalboards.ch2.errors = (TMC262_1420.isStandStillCurrentLimit) 	? (Evalboards.ch2.errors | ERRORS_I_TIMEOUT_STS) 	: (Evalboards.ch2.errors & ~ERRORS_I_TIMEOUT_STS);

	uint8 currCoolStepState = (abs(StepDir_getActualVelocity(DEFAULT_MOTOR)) >= TMC262_1420.coolStepThreshold);
	if(currCoolStepState != lastCoolStepState)
	{
		uint8 value = (currCoolStepState)? TMC262_1420.coolStepActiveValue : TMC262_1420.coolStepInactiveValue;
		TMC262_1420_FIELD_UPDATE(&TMC262_1420, TMC262_1420_SMARTEN, TMC262_1420_SEMIN_MASK, TMC262_1420_SEMIN_SHIFT, value);

		lastCoolStepState = currCoolStepState;
	}

	tmc262_1420_periodicJob(&TMC262_1420, tick);
	StepDir_periodicJob(DEFAULT_MOTOR);
}

static uint8 reset()
{
	if(StepDir_getActualVelocity(0) != 0)
		return 0;

	tmc262_1420_reset(&TMC262_1420);
	compatibilityMode = 1;
	enableDriver(DRIVER_USE_GLOBAL_ENABLE);

	StepDir_init();
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.SG_TST);

	return 1;
}

static uint8 restore()
{
	return tmc262_1420_restore(&TMC262_1420);
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

void TMC262_1420_init(void)
{
	compatibilityMode = 1;

	tmc262_1420_init(&TMC262_1420, 0, Evalboards.ch2.config, &tmc262_1420_defaultRegisterResetState[0]);

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

	TMC262_1420_SPIChannel = &HAL.SPI->ch2;
	TMC262_1420_SPIChannel->CSN = Pins.CSN;

	TMC262_1420.standStillCurrentScale  = I_STAND_STILL;
	TMC262_1420.standStillTimeout       = T_STAND_STILL;

	StepDir_init();
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.SG_TST);

	TMC262_1420_config = Evalboards.ch2.config;

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
	Evalboards.ch2.getMin               = getMin;
	Evalboards.ch2.getMax               = getMax;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}

