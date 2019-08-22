#include "Board.h"
#include "tmc/ic/TMC5161/TMC5161.h"

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

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
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static SPIChannelTypeDef *TMC5161_SPIChannel;
static TMC5161TypeDef TMC5161;
static uint32_t vmax_position;

static inline TMC5161TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC5161;
}

static inline SPIChannelTypeDef *channelToSPI(uint8_t channel)
{
	UNUSED(channel);

	return TMC5161_SPIChannel;
}

// SPI Wrapper for API
void tmc5161_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	// Map the channel to the corresponding SPI channel
	channelToSPI(channel)->readWriteArray(data, length);
}

typedef struct
{
	IOPinTypeDef  *REFL_UC;
	IOPinTypeDef  *REFR_UC;
	IOPinTypeDef  *DRV_ENN_CFG6;
	IOPinTypeDef  *ENCA_DCIN_CFG5;
	IOPinTypeDef  *ENCB_DCEN_CFG4;
	IOPinTypeDef  *ENCN_DCO;
	IOPinTypeDef  *SD_MODE;
	IOPinTypeDef  *SPI_MODE;
	IOPinTypeDef  *SWN_DIAG0;
	IOPinTypeDef  *SWP_DIAG1;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	tmc5161_rotate(motorToIC(motor), velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	tmc5161_right(motorToIC(motor), velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	tmc5161_left(motorToIC(motor), velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	tmc5161_stop(motorToIC(motor));

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc5161_moveTo(motorToIC(motor), position, vmax_position);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc5161_moveBy(motorToIC(motor), ticks, vmax_position);

	return 0;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC5161_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_XTARGET);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_XTARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_XACTUAL);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_VMAX);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_VMAX, abs(*value));
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_VACTUAL);
			*value = CAST_Sn_TO_S32(*value, 24);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = vmax_position;
		} else if(readWrite == WRITE) {
			vmax_position = abs(*value);
			if(tmc5161_readInt(motorToIC(motor), TMC5161_RAMPMODE) == TMC5161_MODE_POSITION)
				tmc5161_writeInt(motorToIC(motor), TMC5161_VMAX, vmax_position);
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_AMAX);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_AMAX, *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_IHOLD_IRUN, TMC5161_IRUN_MASK, TMC5161_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_IHOLD_IRUN, TMC5161_IRUN_MASK, TMC5161_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_IHOLD_IRUN, TMC5161_IHOLD_MASK, TMC5161_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_IHOLD_IRUN, TMC5161_IHOLD_MASK, TMC5161_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_RAMPSTAT, TMC5161_POSITION_REACHED_MASK, TMC5161_POSITION_REACHED_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = !TMC5161_FIELD_READ(motorToIC(motor), TMC5161_RAMPSTAT, TMC5161_STATUS_STOP_R_MASK, TMC5161_STATUS_STOP_R_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !TMC5161_FIELD_READ(motorToIC(motor), TMC5161_RAMPSTAT, TMC5161_STATUS_STOP_L_MASK, TMC5161_STATUS_STOP_L_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_SWMODE, TMC5161_STOP_R_ENABLE_MASK, TMC5161_STOP_R_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_SWMODE, TMC5161_STOP_R_ENABLE_MASK, TMC5161_STOP_R_ENABLE_SHIFT, *value);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_SWMODE, TMC5161_STOP_L_ENABLE_MASK, TMC5161_STOP_L_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_SWMODE, TMC5161_STOP_L_ENABLE_MASK, TMC5161_STOP_L_ENABLE_SHIFT, *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_SWMODE);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_SWMODE, *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_A1);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_A1, *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_V1);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_V1, *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_DMAX);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_DMAX, *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_D1);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_D1, *value);
		}
		break;
	case 19:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_VSTART);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_VSTART, *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_VSTOP);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_VSTOP, *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_TZEROWAIT);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_TZEROWAIT, *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			buffer = tmc5161_readInt(motorToIC(motor), TMC5161_THIGH);
			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
			tmc5161_writeInt(motorToIC(motor), TMC5161_THIGH, *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_VDCMIN);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_VDCMIN, *value);
		}
		break;
	case 27:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_VHIGHCHM_MASK, TMC5161_VHIGHCHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_VHIGHCHM_MASK, TMC5161_VHIGHCHM_SHIFT, *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_VHIGHFS_MASK, TMC5161_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_VHIGHFS_MASK, TMC5161_VHIGHFS_SHIFT, *value);
		}
		break;
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			*value = TMC5161.velocity;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 33:
		// Analog I Scale
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_GCONF, TMC5161_RECALIBRATE_MASK, TMC5161_RECALIBRATE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_GCONF, TMC5161_RECALIBRATE_MASK, TMC5161_RECALIBRATE_SHIFT, *value);
		}
		break;
	case 34:
		// Internal RSense
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_GCONF, TMC5161_REFR_DIR_MASK, TMC5161_REFR_DIR_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_GCONF, TMC5161_REFR_DIR_MASK, TMC5161_REFR_DIR_SHIFT, *value);
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_MRES_MASK, TMC5161_MRES_SHIFT);
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
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_MRES_MASK, TMC5161_MRES_SHIFT, *value);
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
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_TBL_MASK, TMC5161_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_TBL_MASK, TMC5161_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_CHM_MASK, TMC5161_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_CHM_MASK, TMC5161_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_DISFDCC_MASK, TMC5161_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_DISFDCC_MASK, TMC5161_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		buffer = tmc5161_readInt(motorToIC(motor), TMC5161_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5161_CHM_SHIFT))
			{
				*value = (buffer >> TMC5161_HEND_SHIFT) & TMC5161_HEND_MASK;
			}
			else
			{
				*value = (tmc5161_readInt(motorToIC(motor), TMC5161_CHOPCONF) >> TMC5161_TFD_ALL_SHIFT) & TMC5161_TFD_ALL_MASK;
				if(buffer & TMC5161_TFD_3_SHIFT)
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(tmc5161_readInt(motorToIC(motor), TMC5161_CHOPCONF) & (1<<14))
			{
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_HEND_MASK, TMC5161_HEND_SHIFT, *value);
			}
			else
			{
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_TFD_3_MASK, TMC5161_TFD_3_SHIFT, (*value & (1<<3))); // MSB wird zu value dazugefügt
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_TFD_ALL_MASK, TMC5161_TFD_ALL_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		buffer = tmc5161_readInt(motorToIC(motor), TMC5161_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5161_CHM_SHIFT))
			{
				*value = (buffer >> TMC5161_HSTRT_SHIFT) & TMC5161_HSTRT_MASK;
			}
			else
			{
				*value = (buffer >> TMC5161_OFFSET_SHIFT) & TMC5161_OFFSET_MASK;
				if(buffer & (1 << TMC5161_TFD_3_SHIFT))
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1 << TMC5161_CHM_SHIFT))
			{
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_HSTRT_MASK, TMC5161_HSTRT_SHIFT, *value);
			}
			else
			{
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_OFFSET_MASK, TMC5161_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_TOFF_MASK, TMC5161_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_TOFF_MASK, TMC5161_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEIMIN_MASK, TMC5161_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEIMIN_MASK, TMC5161_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEDN_MASK, TMC5161_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEDN_MASK, TMC5161_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEMAX_MASK, TMC5161_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEMAX_MASK, TMC5161_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEUP_MASK, TMC5161_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEUP_MASK, TMC5161_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEMIN_MASK, TMC5161_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SEMIN_MASK, TMC5161_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SFILT_MASK, TMC5161_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SFILT_MASK, TMC5161_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SGT_MASK, TMC5161_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_COOLCONF, TMC5161_SGT_MASK, TMC5161_SGT_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_DRVSTATUS, TMC5161_CS_ACTUAL_MASK, TMC5161_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		//this function sort of doubles with 182 but is necessary to allow cross chip compliance
		if(readWrite == READ) {
			if(TMC5161_FIELD_READ(motorToIC(motor), TMC5161_SWMODE, TMC5161_SG_STOP_MASK, TMC5161_SG_STOP_SHIFT))
			{
				buffer = tmc5161_readInt(motorToIC(motor), TMC5161_TCOOLTHRS);
				*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
			}
			else
			{
				*value = 0;
			}
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_SWMODE, TMC5161_SG_STOP_MASK, TMC5161_SG_STOP_SHIFT, (*value)? 1:0);

			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5161_writeInt(motorToIC(motor), TMC5161_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			buffer = tmc5161_readInt(motorToIC(motor), TMC5161_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5161_writeInt(motorToIC(motor), TMC5161_TCOOLTHRS, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_RNDTF_MASK, TMC5161_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_CHOPCONF, TMC5161_RNDTF_MASK, TMC5161_RNDTF_SHIFT, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = (tmc5161_readInt(motorToIC(motor), TMC5161_CHOPCONF) >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			buffer = tmc5161_readInt(motorToIC(motor), TMC5161_CHOPCONF);
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			tmc5161_writeInt(motorToIC(motor), TMC5161_CHOPCONF,buffer);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			buffer = tmc5161_readInt(motorToIC(motor), TMC5161_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5161_writeInt(motorToIC(motor), TMC5161_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_GRAD_MASK, TMC5161_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_GRAD_MASK, TMC5161_PWM_GRAD_SHIFT, *value);
			// Enable/disable stealthChop accordingly
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_GCONF, TMC5161_EN_PWM_MODE_MASK, TMC5161_EN_PWM_MODE_SHIFT, (*value) ? 1 : 0);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_OFS_MASK, TMC5161_PWM_OFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_PWMCONF, TMC5161_GLOBAL_SCALER_MASK, TMC5161_GLOBAL_SCALER_SHIFT, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_FREQ_MASK, TMC5161_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_FREQ_MASK, TMC5161_PWM_FREQ_SHIFT, *value);
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
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_AUTOSCALE_MASK, TMC5161_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
			{
				TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_PWMCONF, TMC5161_PWM_AUTOSCALE_MASK, TMC5161_PWM_AUTOSCALE_SHIFT, *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_PWMCONF, TMC5161_FREEWHEEL_MASK, TMC5161_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5161_FIELD_UPDATE(motorToIC(motor), TMC5161_PWMCONF, TMC5161_FREEWHEEL_MASK, TMC5161_FREEWHEEL_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC5161_FIELD_READ(motorToIC(motor), TMC5161_DRVSTATUS, TMC5161_SG_RESULT_MASK, TMC5161_SG_RESULT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_XENC);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_XENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = tmc5161_readInt(motorToIC(motor), TMC5161_ENC_CONST);
		} else if(readWrite == WRITE) {
			tmc5161_writeInt(motorToIC(motor), TMC5161_ENC_CONST, *value);
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
	if(motor >= TMC5161_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5161.velocity;

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(motor);
	tmc5161_writeInt(&TMC5161, address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc5161_readInt(&TMC5161, address);
}

static void periodicJob(uint32_t tick)
{
	tmc5161_periodicJob(&TMC5161, tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:  // simulate reference switches, set high to support external ref swiches
		/*
		 * The the TMC5161 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */
		if(!(*value & ~3))
		{
			if(*value & (1<<0))
			{
				HAL.IOs->config->toInput(Pins.REFR_UC); // pull up -> set it to floating causes high
			}
			else
			{
				HAL.IOs->config->toOutput(Pins.REFR_UC);
				HAL.IOs->config->setLow(Pins.REFR_UC);
			}

			if(*value & (1<<1))
			{
				HAL.IOs->config->toInput(Pins.REFL_UC); // pull up -> set it to floating causes high
			}
			else
			{
				HAL.IOs->config->toOutput(Pins.REFL_UC);
				HAL.IOs->config->setLow(Pins.REFL_UC);
			}
		}
		else
		{
			errors |= TMC_ERROR_VALUE;
		}
		break;
//	case 1:  // set analogue current duty
//		/*
//		 * Current will be defined by analogue *value voltage or current signal. In any case this function
//		 * will generate a analogue voltage by PWM for up to 50% duty and a switch for the other 50%.
//		 * The reference voltage will be AIN_REF = VCC_IO * *value/20000 with *value = {0..20000}
//		 */
//
//		buffer = (uint32_t) *value;
//
//		if(buffer <= 20000)
//		{
//			if(buffer > 10000)HAL.IOs->config->setHigh(Pins.AIN_REF_SW);
//			else HAL.IOs->config->setLow(Pins.AIN_REF_SW);
//
//			Timer.setDuty(buffer%10001);
//		}
//		else errors |= TMC_ERROR_VALUE;
//		break;
	case 2:  // Use internal clock
		/*
		 * Internel clock will be enabled by calling this function with a *value != 0 and unpower and repower the motor supply while keeping usb connected.
		 */
		if(*value)
		{
			HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
			HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);
		}
		else
		{
			HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
		}
		break;
	case 3: // Write/Read SD_MODE pin
		if(motor)
		{	// Write
			// Use Bit 0 here explicitly to allow extension of the UF for more pins if ever needed
			if(*value & 0x00000001)
				HAL.IOs->config->setHigh(Pins.SD_MODE);
			else
				HAL.IOs->config->setLow(Pins.SD_MODE);
		}
		else
		{	// Read
			*value = (HAL.IOs->config->isHigh(Pins.SD_MODE))? 1:0;
		}
		break;
	case 4:  // set or release/read ENCB_[DCEN_CFG4]
		switch(buffer = *value)
		{
		case 0:
			HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
			HAL.IOs->config->setLow(Pins.ENCB_DCEN_CFG4);
			break;
		case 1:
			HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
			HAL.IOs->config->setHigh(Pins.ENCB_DCEN_CFG4);
			break;
		default:
			HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
			buffer = HAL.IOs->config->isHigh(Pins.ENCB_DCEN_CFG4);;
			break;
		}
		*value = buffer;
		break;
	case 5:  // read interrupt pin SWN_DIAG0
		*value = (HAL.IOs->config->isHigh(Pins.SWN_DIAG0))? 1:0;
		break;
	case 6:  // read interrupt pin SWP_DIAG1
		*value = (HAL.IOs->config->isHigh(Pins.SWP_DIAG1))? 1:0;
		break;
//	case 7:  // enable single wire interface (SWSEL)
//			if(*value == 1) HAL.IOs->config->setHigh(Pins.SWSEL);
//			else HAL.IOs->config->setLow(Pins.SWSEL);
//		break;
	case 252:
		if(*value)
		{
			HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
			HAL.IOs->config->setLow(Pins.ENCB_DCEN_CFG4);
		}
		else
		{
			HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static void deInit(void)
{
	HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->setLow(Pins.SD_MODE);
	HAL.IOs->config->setLow(Pins.SPI_MODE);
	HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->reset(Pins.ENCN_DCO);
	HAL.IOs->config->reset(Pins.REFL_UC);
	HAL.IOs->config->reset(Pins.REFR_UC);
	HAL.IOs->config->reset(Pins.SWN_DIAG0);
	HAL.IOs->config->reset(Pins.SWP_DIAG1);
	HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->reset(Pins.SD_MODE);
	HAL.IOs->config->reset(Pins.SPI_MODE);
};

static uint8_t reset()
{
	if(!tmc5161_readInt(&TMC5161, TMC5161_VACTUAL))
		tmc5161_reset(&TMC5161);

	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);

	return 1;
}

static uint8_t restore()
{
	return tmc5161_restore(&TMC5161);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
}

static void configCallback(TMC5161TypeDef *tmc5161, ConfigState state)
{
	if(state == CONFIG_RESET)
	{	// Change hardware-preset registers here
		tmc5161_writeInt(tmc5161, TMC5161_SHORT_CONF, 0x00010C0C);
		tmc5161_writeInt(tmc5161, TMC5161_DRV_CONF, 0x00080200);
		tmc5161_writeInt(tmc5161, TMC5161_PWMCONF, 0xC40C001E);
	}
}

void TMC5161_init(void)
{
	tmc5161_init(&TMC5161, 0, Evalboards.ch1.config, tmc5161_defaultRegisterResetState);
	tmc5161_setCallback(&TMC5161, configCallback);

	Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0;
	Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1;
	Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2;
	Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3;
	Pins.REFL_UC         = &HAL.IOs->pins->DIO6;
	Pins.REFR_UC         = &HAL.IOs->pins->DIO7;
	Pins.SD_MODE         = &HAL.IOs->pins->DIO9;
	Pins.SPI_MODE        = &HAL.IOs->pins->DIO11;
	Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15;
	Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16;

	HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->toOutput(Pins.SD_MODE);
	HAL.IOs->config->toOutput(Pins.SPI_MODE);

	HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->setLow(Pins.SD_MODE);
	HAL.IOs->config->setHigh(Pins.SPI_MODE);

	HAL.IOs->config->toInput(Pins.ENCN_DCO);
	HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->toInput(Pins.SWN_DIAG0);
	HAL.IOs->config->toInput(Pins.SWP_DIAG1);
	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);

	// Disable CLK output -> use internal 12 MHz clock
	// Switchable via user function
	//HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
	//HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);

	TMC5161_SPIChannel = &HAL.SPI->ch1;
	TMC5161_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

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
	Evalboards.ch1.numberOfMotors       = TMC5161_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	vmax_position = TMC5161.config->shadowRegister[TMC5161_VMAX];

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
