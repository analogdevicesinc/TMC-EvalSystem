#include "Board.h"
#include "tmc/ic/TMC5041/TMC5041.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  280  // VM[V/10] max +10%

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
static void checkErrors	(uint32 tick);
static void deInit(void);
static uint32 userFunction(uint8 type, uint8 motor, int32 *value);

static uint8 reset();
static void enableDriver(DriverState state);

static SPIChannelTypeDef *TMC5041_SPIChannel;
static TMC5041TypeDef TMC5041;
static ConfigurationTypeDef *TMC5041_config;

typedef struct
{
	IOPinTypeDef  *DRV_ENN;
	IOPinTypeDef  *INT_ENCA;
	IOPinTypeDef  *PP_ENCB;
} PinsTypeDef;

static PinsTypeDef Pins;

// => SPI wrapper
void tmc5041_writeDatagram(u8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{
	UNUSED(motor);
	TMC5041_SPIChannel->readWrite(address|TMC5041_WRITE_BIT, FALSE);
	TMC5041_SPIChannel->readWrite(x1, FALSE);
	TMC5041_SPIChannel->readWrite(x2, FALSE);
	TMC5041_SPIChannel->readWrite(x3, FALSE);
	TMC5041_SPIChannel->readWrite(x4, TRUE);

	int value = x1;
	value <<= 8;
	value |= x2;
	value <<= 8;
	value |= x3;
	value <<= 8;
	value |= x4;

	TMC5041_config->shadowRegister[address & 0x7F] = value;
}

void tmc5041_writeInt(u8 motor, uint8 address, int value)
{
	tmc5041_writeDatagram(motor, address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
}

int tmc5041_readInt(u8 motor, uint8 address)
{
	UNUSED(motor);
	address &= 0x7F;

	// register not readable -> shadow register copy
	if(!TMC_IS_READABLE(TMC5041.registerAccess[address]))
		return TMC5041_config->shadowRegister[address];

	TMC5041_SPIChannel->readWrite(address, FALSE);
	TMC5041_SPIChannel->readWrite(0, FALSE);
	TMC5041_SPIChannel->readWrite(0, FALSE);
	TMC5041_SPIChannel->readWrite(0, FALSE);
	TMC5041_SPIChannel->readWrite(0, TRUE);

	TMC5041_SPIChannel->readWrite(address, FALSE);
	int value = TMC5041_SPIChannel->readWrite(0, FALSE);
	value <<= 8;
	value |= TMC5041_SPIChannel->readWrite(0, FALSE);
	value <<=	8;
	value |= TMC5041_SPIChannel->readWrite(0, FALSE);
	value <<= 8;
	value |= TMC5041_SPIChannel->readWrite(0, TRUE);

	return value;
}
// <= SPI wrapper

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= TMC5041_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc5041_writeInt(motor, TMC5041_VMAX(motor), abs(velocity));
	TMC5041.vMaxModified[motor] = TRUE;
	tmc5041_writeDatagram(motor, TMC5041_RAMPMODE(motor), 0, 0, 0, (velocity >= 0)? TMC5041_MODE_VELPOS:TMC5041_MODE_VELNEG);
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
	if(motor >= TMC5041_MOTORS)
		return TMC_ERROR_MOTOR;

	if(TMC5041.vMaxModified[motor])
	{
		tmc5041_writeInt(motor, TMC5041_VMAX(motor), TMC5041_config->shadowRegister[TMC5041_VMAX(motor)]);
		TMC5041.vMaxModified[motor] = FALSE;
	}
	tmc5041_writeInt(motor, TMC5041_XTARGET(motor), position);
	tmc5041_writeDatagram(motor, TMC5041_RAMPMODE(motor), 0, 0, 0, TMC5041_MODE_POSITION);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	// determine actual position and add numbers of ticks to move
	*ticks = tmc5041_readInt(motor, TMC5041_XACTUAL(motor)) + *ticks;

	return moveTo(motor, *ticks);
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;
	int tempValue;

	if(motor >= TMC5041_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_XTARGET(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_XTARGET(motor), *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_XACTUAL(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_XACTUAL(motor), *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VMAX(motor));
		} else if(readWrite == WRITE) {
			TMC5041.vMaxModified[motor] = TRUE;
			tmc5041_writeInt(motor, TMC5041_VMAX(motor), abs(*value));
		}
		break;
	case 3:
		// todo CHECK 3: min max actually velocity min and velocity max? (JE) #3
		// Actual speed
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VACTUAL(motor));
			*value = CAST_Sn_TO_S32(*value, 24);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value=TMC5041_config->shadowRegister[TMC5041_VMAX(motor)];
		} else if(readWrite == WRITE) {
			TMC5041_config->shadowRegister[TMC5041_VMAX(motor)] = abs(*value);
			if(tmc5041_readInt(motor, TMC5041_RAMPMODE(motor)) == TMC5041_MODE_POSITION)
				tmc5041_writeInt(motor, TMC5041_VMAX(motor), abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value=tmc5041_readInt(motor, TMC5041_AMAX(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_AMAX(motor), *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_IHOLD_IRUN(motor), TMC5041_IRUN_MASK, TMC5041_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_IHOLD_IRUN(motor), TMC5041_IRUN_MASK, TMC5041_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_IHOLD_IRUN(motor), TMC5041_IHOLD_MASK, TMC5041_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_IHOLD_IRUN(motor), TMC5041_IHOLD_MASK, TMC5041_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_RAMPSTAT(motor), TMC5041_POSITION_REACHED_MASK, TMC5041_POSITION_REACHED_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = !TMC5041_FIELD_READ(motor, TMC5041_RAMPSTAT(motor), TMC5041_STATUS_STOP_R_MASK, TMC5041_STATUS_STOP_R_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !TMC5041_FIELD_READ(motor, TMC5041_RAMPSTAT(motor), TMC5041_STATUS_STOP_L_MASK, TMC5041_STATUS_STOP_L_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_SWMODE(motor), TMC5041_STOP_R_ENABLE_MASK, TMC5041_STOP_R_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_SWMODE(motor), TMC5041_STOP_R_ENABLE_MASK, TMC5041_STOP_R_ENABLE_SHIFT, (*value)? 1:0);
			TMC5041_FIELD_UPDATE(motor, TMC5041_SWMODE(motor), TMC5041_POL_STOP_R_MASK, TMC5041_POL_STOP_R_SHIFT, (*value==2)? 1:0);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_SWMODE(motor), TMC5041_STOP_L_ENABLE_MASK, TMC5041_STOP_L_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_SWMODE(motor), TMC5041_STOP_L_ENABLE_MASK, TMC5041_STOP_L_ENABLE_SHIFT, (*value)? 1:0);
			TMC5041_FIELD_UPDATE(motor, TMC5041_SWMODE(motor), TMC5041_POL_STOP_L_MASK, TMC5041_POL_STOP_L_SHIFT, (*value==2)? 1:0);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_SWMODE(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_SWMODE(motor), *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_A1(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_A1(motor), *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_V1(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_V1(motor), *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_DMAX(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_DMAX(motor), *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_D1(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_D1(motor), *value);
		}
		break;
	case 19:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VSTART(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VSTART(motor), *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VSTOP(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VSTOP(motor), *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_TZEROWAIT(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_TZEROWAIT(motor), *value);
		}
		break;
	case 22:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VCOOLTHRS(motor), *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VHIGH(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VHIGH(motor), *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VDCMIN(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VDCMIN(motor), *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_VHIGHFS_MASK, TMC5041_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_VHIGHFS_MASK, TMC5041_VHIGHFS_SHIFT, *value);
		}
		break;
	case 29:	// todo AP XML 2: Beschreibung vom AP fehlt (LH)
		if(readWrite == READ) {
			*value = TMC5041.velocity[motor];
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 256 >> TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_MRES_MASK, TMC5041_MRES_SHIFT);
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
				TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_MRES_MASK, TMC5041_MRES_SHIFT, *value);
			}
			//else TMCL.reply->Status = REPLY_INVALID_VALUE;
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_TBL_MASK, TMC5041_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_TBL_MASK, TMC5041_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_CHM_MASK, TMC5041_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_CHM_MASK, TMC5041_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_DISFDCC_MASK, TMC5041_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_DISFDCC_MASK, TMC5041_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		tempValue = tmc5041_readInt(motor, TMC5041_CHOPCONF(motor));
		if(readWrite == READ) {
			if(tempValue & TMC5041_CHM_MASK)
			{
				*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_HEND_MASK, TMC5041_HEND_SHIFT);
			}
			else
			{
				*value = (tempValue >> TMC5041_TFD_ALL_SHIFT) & TMC5041_TFD_ALL_MASK;
				if(tempValue & TMC5041_TFD_3_MASK)
					*value |= 1<<3;	// Add MSB to value
			}
		} else if(readWrite == WRITE) {
			if(tempValue & TMC5041_CHM_MASK)
			{
				TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_HEND_MASK, TMC5041_HEND_SHIFT, *value);
			}
			else
			{
				TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_TFD_3_MASK, TMC5041_TFD_3_SHIFT, (*value & (1<<3))? 1:0);

				TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_TFD_ALL_MASK, TMC5041_TFD_ALL_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		tempValue = tmc5041_readInt(motor, TMC5041_CHOPCONF(motor));
		if(readWrite == READ) {
			if(tempValue & TMC5041_CHM_MASK)
			{
				*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_HSTRT_MASK, TMC5041_HSTRT_SHIFT);
			}
			else
			{
				*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_OFFSET_MASK, TMC5041_OFFSET_SHIFT);
				if(tempValue & TMC5041_TFD_3_MASK)
					*value |= 1<<3; // MSB wird zu value hinzugefügt
			}
		} else if(readWrite == WRITE) {
			if(tmc5041_readInt(motor, TMC5041_CHOPCONF(motor)) & (1<<14))
			{
				TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_HSTRT_MASK, TMC5041_HSTRT_SHIFT, *value);
			}
			else
			{
				TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_OFFSET_MASK, TMC5041_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_TOFF_MASK, TMC5041_TOFF_MASK);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_TOFF_MASK, TMC5041_TOFF_MASK, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SEIMIN_MASK, TMC5041_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SEIMIN_MASK, TMC5041_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SEDN_MASK, TMC5041_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SEDN_MASK, TMC5041_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SEMAX_MASK, TMC5041_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SEMAX_MASK, TMC5041_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SEUP_MASK, TMC5041_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SEUP_MASK, TMC5041_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SEMIN_MASK, TMC5041_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SEMIN_MASK, TMC5041_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SFILT_MASK, TMC5041_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SFILT_MASK, TMC5041_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_COOLCONF(motor), TMC5041_SGT_MASK, TMC5041_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_COOLCONF(motor), TMC5041_SGT_MASK, TMC5041_SGT_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_VSENSE_MASK, TMC5041_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_VSENSE_MASK, TMC5041_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_DRVSTATUS(motor), TMC5041_CS_ACTUAL_MASK, TMC5041_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VCOOLTHRS(motor),*value);
			TMC5041_FIELD_UPDATE(motor, TMC5041_SWMODE(motor), TMC5041_SG_STOP_MASK, TMC5041_SG_STOP_SHIFT, (*value)? 1:0);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = tmc5041_readInt(motor, TMC5041_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			tmc5041_writeInt(motor, TMC5041_VCOOLTHRS(motor),*value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_CHOPCONF(motor), TMC5041_RNDTF_MASK, TMC5041_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5041_FIELD_UPDATE(motor, TMC5041_CHOPCONF(motor), TMC5041_RNDTF_MASK, TMC5041_RNDTF_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC5041_FIELD_READ(motor, TMC5041_DRVSTATUS(motor), TMC5041_SG_RESULT_MASK, TMC5041_SG_RESULT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209: // todo CHECK REM 3: TMC5041 doesn't have encoder features? Remove this? (LH) #1
		// Encoder position
		break;
	case 210: // todo CHECK REM 3: TMC5041 doesn't have encoder features? Remove this? (LH) #2
		// Encoder Resolution
//			if(readWrite == READ) {
//				*value = tmc5041_readInt(motor, TMC5041_ENC_CONST(motor));
//			} else if(readWrite == WRITE) {
//				tmc5041_writeInt(motor, TMC5041_ENC_CONST(motor),*value);
//			}
		break;
	case 211: // d
		if(readWrite == READ) {
			// encoder enable
			switch(motor)
			{
			case 0:
				tempValue = tmc5041_readInt(motor, TMC5041_GCONF);
				tempValue &= (1<<3) | (1<<4);
				*value = (tempValue == (1<<4))? 1 : 0;
				break;
			case 1:
				tempValue = tmc5041_readInt(motor, TMC5041_GCONF);
				tempValue &= (1<<5) | (1<<6);
				*value = (tempValue == (1<<5))? 1 : 0;
				break;
			}
		} else if(readWrite == WRITE) {
			// encoder enable
			switch(motor)
			{
			case 0:
				tempValue = tmc5041_readInt(motor, TMC5041_GCONF);
				if(*value)
					tempValue = (tempValue & ~(1<<3)) | (1<<4);
				else
					tempValue = (tempValue | (1<<3)) & ~(1<<4);
				tmc5041_writeInt(motor, TMC5041_GCONF, tempValue);
				break;
			case 1:		// enable ENCODER2 - disable REF
				tempValue = tmc5041_readInt(motor, TMC5041_GCONF);
				if(*value)
					//tempValue = (tempValue | (1<<5)) & ~(5<<5);
					tempValue = (tempValue | (1<<5)) & ~(1<<6); //todo: CHECK 3: Sind die Änderungen richtig? Codemäßig macht es so Sinn, aber die Bits sind in der Dokumentation als reserved markiert (LH) #1
				else
					//tempValue = (tempValue & ~(1<<6)) | ~(1<<6);
					tempValue = (tempValue & ~(1<<5)) | (1<<6); //todo: CHECK 3: Sind die Änderungen richtig? Codemäßig macht es so Sinn, aber die Bits sind in der Dokumentation als reserved markiert (LH) #2
				tmc5041_writeInt(motor, TMC5041_GCONF, tempValue);
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

static uint32 SAP(uint8 type, uint8 motor, int32 value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32 GAP(uint8 type, uint8 motor, int32 *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
{
	if(motor >= TMC5041_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5041.velocity[motor];

	return TMC_ERROR_NONE;
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmc5041_writeInt(0, address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmc5041_readInt(0, address);
}

static void periodicJob(uint32 tick)
{
	for(int motor = 0; motor < TMC5041_MOTORS; motor++)
	{
		tmc5041_periodicJob(motor, tick, &TMC5041, TMC5041_config);
	}
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	uint32 errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 1:  // read interrupt pin INT
		*value = (HAL.IOs->config->isHigh(Pins.INT_ENCA))? 1 : 0;
		break;
	case 2:  // read position compare pin PP
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
};

static uint8 reset()
{
	for(uint8 motor = 0; motor < TMC5041_MOTORS; motor++)
		if(tmc5041_readInt(motor, TMC5041_VACTUAL(motor)) != 0)
			return 0;

	return tmc5041_reset(TMC5041_config);
}

static uint8 restore()
{
	return tmc5041_restore(TMC5041_config);
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

void TMC5041_init(void)
{
	tmc5041_initConfig(&TMC5041);

	Pins.DRV_ENN   = &HAL.IOs->pins->DIO0;
	Pins.INT_ENCA  = &HAL.IOs->pins->DIO5;
	Pins.PP_ENCB   = &HAL.IOs->pins->DIO6;

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toInput(Pins.INT_ENCA);
	HAL.IOs->config->toInput(Pins.PP_ENCB);

	TMC5041_SPIChannel = &HAL.SPI->ch1;
	TMC5041_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	TMC5041_config = Evalboards.ch1.config;

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
	Evalboards.ch1.numberOfMotors       = TMC5041_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
