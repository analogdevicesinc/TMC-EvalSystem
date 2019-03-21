#include "Board.h"
#include "tmc/ic/TMC5062/TMC5062.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  222  // VM[V/10] max +10%

#define MOTORS  2

// Map our motor index to IC/channel pair
// We only have one IC, so we always choose that IC and map the motor index directly to the channel
#define MOTOR_TO_IC(motor)       (&TMC5062)
#define MOTOR_TO_CHANNEL(motor)  (motor)

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
static void checkErrors	(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);
static void configCallback(TMC5062TypeDef *tmc5062, ConfigState state);

static SPIChannelTypeDef *TMC5062_SPIChannel;
static TMC5062TypeDef TMC5062;
static ConfigurationTypeDef *TMC5062_config;

// Position and velocity mode both use VMAX. In order to preserve VMAX of
// position mode we store the value when switching to velocity mode and
// reapply the stored value when entering position mode.
// A nonzero value represents a stored value, a zero value represents
// no value being stored (VMAX = 0 is a useless case for position mode, so
// using 0 as special value works). No stored value results in VMAX of
// velocity mode being kept for position mode.
static uint32_t vMaxPosMode[MOTORS] = { 0 };

typedef struct
{
	IOPinTypeDef  *DRV_ENN;
	IOPinTypeDef  *INT_ENCA;
	IOPinTypeDef  *PP_ENCB;
	IOPinTypeDef  *SWSEL;
	IOPinTypeDef  *SWIOP1;
	IOPinTypeDef  *SWIOP2;
	IOPinTypeDef  *SWION;
} PinsTypeDef;

static PinsTypeDef Pins;

// => SPI Wrapper
uint8_t tmc5062_readWrite(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
	if(motor >= MOTORS)
		return 0;

	// Only one IC (TMC5062) -> always the same bus to use
	return TMC5062_SPIChannel->readWrite(data, lastTransfer);
}
// <= SPI Wrapper

// => Motor -> IC/channel translation
inline static int readInt(uint8_t motor, uint8_t address)
{
	TMC5062TypeDef *IC = MOTOR_TO_IC(motor);
	uint8_t channel = MOTOR_TO_CHANNEL(motor);

	return tmc5062_readInt(IC, channel, address);
}

inline static void writeInt(uint8_t motor, uint8_t address, int value)
{
	TMC5062TypeDef *IC = MOTOR_TO_IC(motor);
	uint8_t channel = MOTOR_TO_CHANNEL(motor);

	tmc5062_writeInt(IC, channel, address, value);
}

inline static int readField(uint8_t motor, uint8_t address, uint32_t mask, uint8_t shift)
{
	TMC5062TypeDef *IC = MOTOR_TO_IC(motor);
	uint8_t channel = MOTOR_TO_CHANNEL(motor);

	return TMC5062_FIELD_READ(IC, channel, address, mask, shift);
}

inline static void writeField(uint8_t motor, uint8_t address, uint32_t mask, uint8_t shift, uint32_t value)
{
	TMC5062TypeDef *IC = MOTOR_TO_IC(motor);
	uint8_t channel = MOTOR_TO_CHANNEL(motor);

	TMC5062_FIELD_WRITE(IC, channel, address, mask, shift, value);
}
// <= Motor -> IC/channel translation

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	// Save VMAX if there is no saved value yet (restore when (re-)entering position mode)
	if(vMaxPosMode[motor] == 0)
		vMaxPosMode[motor] = readInt(motor, TMC5062_VMAX(motor));

	writeInt(motor, TMC5062_VMAX(motor), abs(velocity));
	writeInt(motor, TMC5062_RAMPMODE(motor), (velocity >= 0)? TMC5062_MODE_VELPOS:TMC5062_MODE_VELNEG);

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

	// If we have a saved VMAX, apply and then delete ( = 0) the copy
	if(vMaxPosMode[motor])
	{
		writeInt(motor, TMC5062_VMAX(motor), vMaxPosMode[motor]);
		vMaxPosMode[motor] = 0;
	}

	writeInt(motor, TMC5062_XTARGET(motor), position);
	writeInt(motor, TMC5062_RAMPMODE(motor), TMC5062_MODE_POSITION);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	// determine actual position and add numbers of ticks to move
	*ticks = readInt(motor, TMC5062_XACTUAL(motor)) + *ticks;

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;
	int tempValue;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_XTARGET(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_XTARGET(motor), *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_XACTUAL(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_XACTUAL(motor), *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VMAX(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VMAX(motor), abs(*value));
		}
		break;
	case 3:
		// todo CHECK 3: min max actually velocity min and velocity max? (JE) #4
		// Actual speed
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VACTUAL(motor));
			*value = CAST_Sn_TO_S32(*value, 24);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = TMC5062_config->shadowRegister[TMC5062_VMAX(motor)];
		} else if(readWrite == WRITE) {
			TMC5062_config->shadowRegister[TMC5062_VMAX(motor)] = abs(*value);
			if(readInt(motor, TMC5062_RAMPMODE(motor)) == TMC5062_MODE_POSITION)
				writeInt(motor, TMC5062_VMAX(motor), abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_AMAX(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_AMAX(motor), *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_IHOLD_IRUN(motor), TMC5062_IRUN_MASK, TMC5062_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_IHOLD_IRUN(motor), TMC5062_IRUN_MASK, TMC5062_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_IHOLD_IRUN(motor), TMC5062_IHOLD_MASK, TMC5062_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_IHOLD_IRUN(motor), TMC5062_IHOLD_MASK, TMC5062_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_RAMPSTAT(motor), TMC5062_POSITION_REACHED_MASK, TMC5062_POSITION_REACHED_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = !readField(motor, TMC5062_RAMPSTAT(motor), TMC5062_STATUS_STOP_R_MASK, TMC5062_STATUS_STOP_R_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !readField(motor, TMC5062_RAMPSTAT(motor), TMC5062_STATUS_STOP_L_MASK, TMC5062_STATUS_STOP_L_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_SWMODE(motor), TMC5062_STOP_R_ENABLE_MASK, TMC5062_STOP_R_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_SWMODE(motor), TMC5062_STOP_R_ENABLE_MASK, TMC5062_STOP_R_ENABLE_SHIFT, *value);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_SWMODE(motor), TMC5062_STOP_L_ENABLE_MASK, TMC5062_STOP_L_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_SWMODE(motor), TMC5062_STOP_L_ENABLE_MASK, TMC5062_STOP_L_ENABLE_SHIFT, *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_SWMODE(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_SWMODE(motor), *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_A1(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_A1(motor), *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_V1(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_V1(motor), *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_DMAX(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_DMAX(motor), *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_D1(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_D1(motor), *value);
		}
		break;
	case 19:
		// VSTART
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VSTART(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VSTART(motor), *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VSTOP(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VSTOP(motor), *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_TZEROWAIT(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_TZEROWAIT(motor), *value);
		}
		break;
	case 22:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VCOOLTHRS(motor), *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VHIGH(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VHIGH(motor), *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VDCMIN(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VDCMIN(motor), *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_VHIGHFS_MASK, TMC5062_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_VHIGHFS_MASK, TMC5062_VHIGHFS_SHIFT, *value);
		}
		break;
	case 29:
		if(readWrite == READ) {
			*value = TMC5062.velocity[motor];
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 256 >> readField(motor, TMC5062_CHOPCONF(motor), TMC5062_MRES_MASK, TMC5062_MRES_SHIFT);
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
				writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_MRES_MASK, TMC5062_MRES_SHIFT, *value);
			}
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_TBL_MASK, TMC5062_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_TBL_MASK, TMC5062_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_CHM_MASK, TMC5062_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_CHM_MASK, TMC5062_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_DISFDCC_MASK, TMC5062_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_DISFDCC_MASK, TMC5062_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			if(readField(motor, TMC5062_CHOPCONF(motor), TMC5062_CHM_MASK, TMC5062_CHM_SHIFT))
			{
				*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_HEND_MASK, TMC5062_HEND_SHIFT);
			}
			else
			{
				*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_TFD_ALL_MASK, TMC5062_TFD_ALL_SHIFT);
				if(readField(motor, TMC5062_CHOPCONF(motor), TMC5062_TFD_3_MASK, TMC5062_TFD_3_SHIFT))
					*value |= 1<<3; // MSB wird zu value hinzugefügt
			}
		} else if(readWrite == WRITE) {
			if(readInt(motor, TMC5062_CHOPCONF(motor)) & (1<<14))
			{
				writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_HEND_MASK, TMC5062_HEND_SHIFT, *value);
			}
			else
			{
				writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_TFD_3_MASK, TMC5062_TFD_3_SHIFT, (*value & 1<<3)? 1:0);
				writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_TFD_ALL_MASK, TMC5062_TFD_ALL_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		if(readWrite == READ) {
			if(readInt(motor, TMC5062_CHOPCONF(motor)) & (1<<14))
			{
				*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_HSTRT_MASK, TMC5062_HSTRT_SHIFT);
			}
			else
			{
				*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_OFFSET_MASK, TMC5062_OFFSET_SHIFT);
				if(readField(motor, TMC5062_CHOPCONF(motor), TMC5062_TFD_3_MASK, TMC5062_TFD_3_SHIFT))
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			if(readInt(motor, TMC5062_CHOPCONF(motor)) & (1<<14))
			{
				writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_HSTRT_MASK, TMC5062_HSTRT_SHIFT, *value);
			}
			else
			{
				writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_OFFSET_MASK, TMC5062_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_TOFF_MASK, TMC5062_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_TOFF_MASK, TMC5062_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SEIMIN_MASK, TMC5062_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SEIMIN_MASK, TMC5062_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SEDN_MASK, TMC5062_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SEDN_MASK, TMC5062_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SEMAX_MASK, TMC5062_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SEMAX_MASK, TMC5062_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SEUP_MASK, TMC5062_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SEUP_MASK, TMC5062_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SEMIN_MASK, TMC5062_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SEMIN_MASK, TMC5062_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SFILT_MASK, TMC5062_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SFILT_MASK, TMC5062_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_COOLCONF(motor), TMC5062_SGT_MASK, TMC5062_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_COOLCONF(motor), TMC5062_SGT_MASK, TMC5062_SGT_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_VSENSE_MASK, TMC5062_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_VSENSE_MASK, TMC5062_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_DRVSTATUS(motor), TMC5062_CS_ACTUAL_MASK, TMC5062_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// reset stall
		if(readWrite == READ) {
			if( readInt(motor, TMC5062_SWMODE(motor)) & (1<<10))
			{
				*value = readInt(motor, TMC5062_VCOOLTHRS(motor));
			}
			else
				tempValue = 0;
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VCOOLTHRS(motor),*value);
			writeField(motor, TMC5062_SWMODE(motor), TMC5062_SG_STOP_MASK, TMC5062_SG_STOP_SHIFT, (*value)? 1:0);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_VCOOLTHRS(motor),*value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_RNDTF_MASK, TMC5062_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_RNDTF_MASK, TMC5062_RNDTF_SHIFT, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_CHOPCONF(motor), TMC5062_SYNC_MASK, TMC5062_SYNC_SHIFT);
		} else if(readWrite == WRITE) {
			writeField(motor, TMC5062_CHOPCONF(motor), TMC5062_SYNC_MASK, TMC5062_SYNC_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = readField(motor, TMC5062_DRVSTATUS(motor), TMC5062_SG_RESULT_MASK, TMC5062_SG_RESULT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_XENC(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_XENC(motor),*value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = readInt(motor, TMC5062_ENC_CONST(motor));
		} else if(readWrite == WRITE) {
			writeInt(motor, TMC5062_ENC_CONST(motor),*value);
		}
		break;
	case 211:
		if(readWrite == READ) {
			// encoder enable
			switch(motor)
			{
			case 0:
				tempValue = readInt(motor, TMC5062_GCONF);
				tempValue &= (1<<3) | (1<<4);
				*value = (tempValue == (1<<4))? 1 : 0;
				break;
			case 1:
				tempValue = readInt(motor, TMC5062_GCONF);
				tempValue &= (1<<5) | (1<<6);
				*value = (tempValue == ((1<<5) | (0<<6)))? 1 : 0;
				break;
			}
		} else if(readWrite == WRITE) {
			// encoder enable
			switch(motor)
			{
			case 0:
				tempValue = readInt(motor, TMC5062_GCONF);
				if(*value)
					tempValue = (tempValue & ~(1<<3)) | (1<<4);
				else
					tempValue = (tempValue | (1<<3)) & ~(1<<4);
				writeInt(motor, TMC5062_GCONF, tempValue);
				break;
			case 1:		// enable ENCODER2 - disable REF
				tempValue = readInt(motor, TMC5062_GCONF);
				if(*value)
					//tempValue = (tempValue | (1<<5)) & ~(5<<5);
					tempValue = (tempValue | (1<<5)) & ~(1<<6); //todo: CHECK 3: Sind die Änderungen richtig? Codemäßig macht es so Sinn, aber die Bits sind in der Dokumentation als reserved markiert (LH) #3
				else
					//tempValue = (tempValue & ~(1<<6)) | ~(1<<6);
					tempValue = (tempValue & ~(1<<5)) | (1<<6); //todo: CHECK 3: Sind die Änderungen richtig? Codemäßig macht es so Sinn, aber die Bits sind in der Dokumentation als reserved markiert (LH) #4
				writeInt(motor, TMC5062_GCONF, tempValue);
				break;
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

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5062.velocity[motor];

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(motor);
	writeInt(0, address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value	= readInt(0, address);
}

static void periodicJob(uint32_t tick)
{
	tmc5062_periodicJob(&TMC5062, tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:		// enable Single Wire Mode
		if(*value)
			HAL.IOs->config->setHigh(Pins.SWSEL);
		else
			HAL.IOs->config->setLow(Pins.SWSEL);
		break;
	case 1:		// read interrupt pin INT
		*value = (HAL.IOs->config->isHigh(Pins.INT_ENCA))? 1 : 0;
		break;
	case 2:		// read position compare pin PP
		*value = (HAL.IOs->config->isHigh(Pins.PP_ENCB))? 1 : 0;
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
	HAL.IOs->config->reset(Pins.INT_ENCA);
	HAL.IOs->config->reset(Pins.PP_ENCB);
	HAL.IOs->config->reset(Pins.SWION);
	HAL.IOs->config->reset(Pins.SWIOP1);
	HAL.IOs->config->reset(Pins.SWIOP2);
	HAL.IOs->config->reset(Pins.SWSEL);
};

static uint8_t reset()
{
	for(uint8_t motor = 0; motor < MOTORS; motor++)
		if(readInt(motor, TMC5062_VACTUAL(motor)) != 0)
			return 0;

	return tmc5062_reset(&TMC5062);
}

static uint8_t restore()
{
	return tmc5062_restore(&TMC5062);
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

static void configCallback(TMC5062TypeDef *tmc5062, ConfigState state)
{
	if(state == CONFIG_RESET)
	{	// Change hardware-preset registers here
		for(uint8_t motor = 0; motor < TMC5062_MOTORS; motor++)
			tmc5062_writeInt(tmc5062, motor, TMC5062_PWMCONF(motor), 0x000504C8);

		// Fill missing shadow registers (hardware preset registers)
		tmc5062_fillShadowRegisters(&TMC5062);
	}
}

void TMC5062_init(void)
{
	TMC5062_config = Evalboards.ch1.config;
	tmc5062_init(&TMC5062, TMC5062_config, &tmc5062_defaultRegisterResetState[0], 0, 1, 16000000);
	tmc5062_setCallback(&TMC5062, configCallback);

	Pins.DRV_ENN   = &HAL.IOs->pins->DIO0;
	Pins.INT_ENCA  = &HAL.IOs->pins->DIO5;
	Pins.PP_ENCB   = &HAL.IOs->pins->DIO6;
	Pins.SWSEL     = &HAL.IOs->pins->DIO16;
	Pins.SWIOP1    = &HAL.IOs->pins->DIO17;
	Pins.SWIOP2    = &HAL.IOs->pins->DIO18;
	Pins.SWION     = &HAL.IOs->pins->DIO19;

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.SWSEL);

	HAL.IOs->config->setLow(Pins.SWSEL);

	HAL.IOs->config->toInput(Pins.INT_ENCA);
	HAL.IOs->config->toInput(Pins.PP_ENCB);
	HAL.IOs->config->toInput(Pins.SWION);
	HAL.IOs->config->toInput(Pins.SWIOP1);
	HAL.IOs->config->toInput(Pins.SWIOP2);

	TMC5062_SPIChannel = &HAL.SPI->ch1;
	TMC5062_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	TMC5062_MicroStepTable microStepTable;
	microStepTable.LUT_0  = 0xAAAAB554;
	microStepTable.LUT_1  = 0x4A9554AA;
	microStepTable.LUT_2  = 0x24492929;
	microStepTable.LUT_3  = 0x10104222;
	microStepTable.LUT_4  = 0xFBFFFFFF;
	microStepTable.LUT_5  = 0xB5BB777D;
	microStepTable.LUT_6  = 0x49295556;
	microStepTable.LUT_7  = 0x00404222;

	microStepTable.X1  = 0x80;
	microStepTable.X2  = 0xFF;
	microStepTable.X3  = 0xFF;
	microStepTable.W0  = 2;
	microStepTable.W1  = 1;
	microStepTable.W2  = 1;
	microStepTable.W3  = 1;

	microStepTable.START_SIN    = 0x00;
	microStepTable.START_SIN90  = 0xF7;

	setMicroStepTable(&TMC5062, 0, &microStepTable);
	setMicroStepTable(&TMC5062, 1, &microStepTable);

	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_RESET;
	Evalboards.ch1.config->configIndex  = 0;

	Evalboards.ch1.rotate               = rotate;
	Evalboards.ch1.right                = right;
	Evalboards.ch1.left                 = left;
	Evalboards.ch1.stop                 = stop;
	Evalboards.ch1.GAP                  = GAP;
	Evalboards.ch1.SAP                  = SAP;
	Evalboards.ch1.moveTo               = moveTo;
	Evalboards.ch1.moveBy               = moveBy;
	Evalboards.ch1.writeRegister        = writeRegister;
	Evalboards.ch1.readRegister         = readRegister;
	Evalboards.ch1.periodicJob          = periodicJob;
	Evalboards.ch1.userFunction         = userFunction;
	Evalboards.ch1.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch1.enableDriver         = enableDriver;
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	enableDriver(DRIVER_ENABLE);
};
