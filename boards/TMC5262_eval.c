/*******************************************************************************
* Copyright © 2017 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5262/TMC5262.h"
#include "tmc/RAMDebug.h"
#include "hal/Timer.h"


static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5262_SPIChannel;


void tmc5262_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
	UNUSED(icID);
	TMC5262_SPIChannel->readWriteArray(data, dataLength);
}

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         45   // VM[V/10] min
#define VM_MAX         650  // VM[V/10] max


#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
#define TMC5262_RAMDEBUG_TIMER TIMER_CHANNEL_1
#elif defined(LandungsbrueckeV3)
#define TMC5262_RAMDEBUG_TIMER TIMER_CHANNEL_2
#endif

// Typedefs
typedef struct
{
    ConfigurationTypeDef *config;
    int32_t velocity, oldX;
    uint32_t oldTick;
    uint8_t slaveAddress;
} TMC5262TypeDef;
static TMC5262TypeDef TMC5262;
static bool vMaxModified = false;
static uint32_t vmax_position;
//static uint32_t vMax		   = 1;

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static TMC5262TypeDef TMC5262;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC5262_CRC(data, length) tmc_CRC8(data, length, 1)

// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc5262_CRC8(uint8_t *data, size_t length)
{
	return tmc_CRC8(data, length, 1);
	//TMC5262_CRC(data, length);
}

static inline TMC5262TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &TMC5262;
}


typedef struct
{
	IOPinTypeDef *N_DRN_EN;
	IOPinTypeDef *N_SLEEP;
	IOPinTypeDef *REFL_INT;
	IOPinTypeDef *REFR_INT;
	IOPinTypeDef *IREF_R2;
	IOPinTypeDef *IREF_R3;
	IOPinTypeDef *DIAG0;
	IOPinTypeDef *DIAG1;

} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	tmc5262_rotate(motorToIC(motor), velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	tmc5262_right(motorToIC(motor), velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	tmc5262_left(motorToIC(motor), velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	tmc5262_stop(motorToIC(motor));

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc5262_moveTo(motorToIC(motor), position, vmax_position);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc5262_moveBy(motorToIC(motor), ticks, vmax_position);

	return 0;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC5262_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			readRegister(motor, TMC5262_XTARGET, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_XTARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			readRegister(motor, TMC5262_XACTUAL, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			readRegister(motor, TMC5262_VMAX, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_VMAX, abs(*value));
			vMaxModified = true;
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			readRegister(motor, TMC5262_VACTUAL, value);
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
			readRegister(motor, TMC5262_RAMPMODE, &buffer);
			if(buffer == TMC5262_MODE_POSITION)
				writeRegister(motor, TMC5262_VMAX, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			readRegister(motor, TMC5262_AMAX, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_AMAX, *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_IRUN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_IRUN_FIELD, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_IHOLD_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_IHOLD_FIELD, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_POSITION_REACHED_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_EVENT_STOP_R_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_EVENT_STOP_L_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_EN_VIRTUAL_STOP_R_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_EN_VIRTUAL_STOP_R_FIELD, *value);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_EN_VIRTUAL_STOP_L_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_EN_VIRTUAL_STOP_L_FIELD, *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			readRegister(motor, TMC5262_SW_MODE, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_SW_MODE, *value);
		}
		break;
	case 15:
		// Maximum Deceleration
		if(readWrite == READ) {
			readRegister(motor, TMC5262_DMAX, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_DMAX, *value);
		}
		break;
	case 16:
		// Velocity VSTART
		if(readWrite == READ) {
			readRegister(motor, TMC5262_VSTART, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_VSTART, *value);
		}
		break;
	case 17:
		// Acceleration A1
		if(readWrite == READ) {
			readRegister(motor, TMC5262_A1, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_A1, *value);
		}
		break;
	case 18:
		// Velocity V1
		if(readWrite == READ) {
			readRegister(motor, TMC5262_V1, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_V1, *value);
		}
		break;
	case 19:
		// Deceleration D1
		if(readWrite == READ) {
			readRegister(motor, TMC5262_D1, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_D1, *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			readRegister(motor, TMC5262_VSTOP, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_VSTOP, *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			readRegister(motor, TMC5262_TZEROWAIT, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_TZEROWAIT, *value);
		}
		break;
	case 22:
		// Velocity V2
		if(readWrite == READ) {
			readRegister(motor, TMC5262_V2, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_V2, *value);
		}
		break;
	case 23:
		// Deceleration D2
		if(readWrite == READ) {
			readRegister(motor, TMC5262_D2, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_D2, *value);
		}
		break;
	case 24:
		// Acceleration A2
		if(readWrite == READ) {
			readRegister(motor, TMC5262_A2, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_A2, *value);
		}
		break;
	case 25:
		// TVMAX
		if(readWrite == READ) {
			readRegister(motor, TMC5262_TVMAX, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_TVMAX, *value);
		}
		break;


	case 26:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			readRegister(motor, TMC5262_THIGH, &buffer);
			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
			writeRegister(motor, TMC5262_THIGH, *value);
		}
		break;
	case 27:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			readRegister(motor, TMC5262_TUDCSTEP, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_TUDCSTEP, *value);
		}
		break;
	case 30:
		// Measured Speed
		if(readWrite == READ) {
			*value = TMC5262.velocity;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 31:
		// Current P
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_CUR_P_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_CUR_P_FIELD, *value);
		}
		break;
	case 32:
		// Current I
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_CUR_I_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_CUR_I_FIELD, *value);
		}
		break;
	case 37:
		// Current limit
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_CUR_PI_LIMIT_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_CUR_PI_LIMIT_FIELD, *value);
		}
		break;
	case 40:
		// Measured current amplitude
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_AMPL_MEAS_FIELD);

		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_AMPL_MEAS_FIELD, *value);

		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> tmc5262_fieldRead(DEFAULT_ICID, TMC5262_MRES_FIELD);
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
				tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_MRES_FIELD, *value);
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
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_TBL_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_TBL_FIELD, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_CHM_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_CHM_FIELD, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_DISFDCC_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_DISFDCC_FIELD, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		readRegister(motor, TMC5262_CHOPCONF, &buffer);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5262_CHM_SHIFT))
			{
				*value = (buffer >> TMC5262_HEND_OFFSET_SHIFT) & TMC5262_HEND_OFFSET_MASK;
			}
			else
			{
				*value = (buffer >> TMC5262_HSTRT_TFD210_SHIFT) & TMC5262_HSTRT_TFD210_MASK;
				if(buffer & TMC5262_FD3_SHIFT)
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1<<14))
			{
				tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_HEND_OFFSET_FIELD, *value);
			}
			else
			{
				tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_FD3_FIELD, (*value & (1<<3))); // MSB wird zu value dazugefügt
				tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_HSTRT_TFD210_FIELD, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		readRegister(motor, TMC5262_CHOPCONF, &buffer);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5262_CHM_SHIFT))
			{
				*value = (buffer >> TMC5262_HSTRT_TFD210_SHIFT) & TMC5262_HSTRT_TFD210_MASK;
			}
			else
			{
				*value = (buffer >> TMC5262_HEND_OFFSET_SHIFT) & TMC5262_HEND_OFFSET_MASK;
				if(buffer & (1 << TMC5262_FD3_SHIFT))
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1 << TMC5262_CHM_SHIFT))
			{
				tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_HSTRT_TFD210_FIELD, *value);
			}
			else
			{
				tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_HEND_OFFSET_FIELD, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_TOFF_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_TOFF_FIELD, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SEIMIN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SEIMIN_FIELD, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SEDN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SEDN_FIELD, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SEMAX_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SEMAX_FIELD, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SEUP_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SEUP_FIELD, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SEMIN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SEMIN_FIELD, *value);
		}
		break;
	case 173:
		// stallGuard4 filter enable
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SGP_FILT_EN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SGP_FILT_EN_FIELD, *value);
		}
		break;
	case 174:
		// stallGuard4 threshold
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SGP_THRS_FIELD);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SGP_THRS_FIELD, *value);
		}
		break;
	case 175:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SFILT_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SFILT_FIELD, *value);
		}
		break;
	case 176:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SGT_FIELD);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SGT_FIELD, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_CS_ACTUAL_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		//this function sort of doubles with 182 but is necessary to allow cross chip compliance
		if(readWrite == READ) {
			if(tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SG_STOP_FIELD))
			{
				readRegister(motor, TMC5262_TCOOLTHRS, &buffer);
				*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
			}
			else
			{
				*value = 0;
			}
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SG_STOP_FIELD, (*value)? 1:0);

			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC5262_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC5262_TCOOLTHRS, &buffer);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC5262_TCOOLTHRS, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		readRegister(motor, TMC5262_CHOPCONF, &buffer);
		if(readWrite == READ) {
			*value = (buffer >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			writeRegister(motor, TMC5262_CHOPCONF, buffer);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC5262_TPWMTHRS, &buffer);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC5262_TPWMTHRS, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_PWM_FREQ_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_PWM_FREQ_FIELD, *value);
		}
		break;
	case 194:
		// MSCNT
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_MSCNT_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// MEAS_SD_EN
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SD_ON_MEAS_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_SD_ON_MEAS_FIELD, *value);
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_FREEWHEEL_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_FREEWHEEL_FIELD, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_SG_RESULT_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			readRegister(motor, TMC5262_X_ENC, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_X_ENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			readRegister(motor, TMC5262_ENC_CONST, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5262_ENC_CONST, *value);
		}
		break;
	case 211:
		//ADC Scaling Resitors
		if(readWrite == READ) {
			uint8_t val2 = (HAL.IOs->config->isHigh(Pins.IREF_R2));
			uint8_t val3 = (HAL.IOs->config->isHigh(Pins.IREF_R3));
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
		// Current range from DRV_CONF reg
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_CURRENT_RANGE_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_CURRENT_RANGE_FIELD, *value);
		}
		break;

	case 213:
		// ADCTemperatur
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_ADC_TEMP_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// ADCSupply
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_ADC_VSUPPLY_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 216:
		// Overvoltage Limit ADC value
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_OVERVOLTAGE_VTH_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_OVERVOLTAGE_VTH_FIELD, *value);
		}
		break;
	case 217:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			*value = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_OVERTEMPPREWARNING_VTH_FIELD);
		} else if(readWrite == WRITE) {
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_OVERTEMPPREWARNING_VTH_FIELD, *value);
		}
		break;
	case 218:
		// ADCTemperatur Converted
		if(readWrite == READ) {

			int32_t adc = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_ADC_TEMP_FIELD);
			*value = (int32_t)10*(adc-2038)/77;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 220:
		// ADCSupply
		if(readWrite == READ) {
			int32_t adc = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_ADC_VSUPPLY_FIELD);
			*value = (int32_t)32*3052*adc/10000;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 221:
		// Overvoltage Limit converted
		if(readWrite == READ) {
			int32_t val = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_OVERVOLTAGE_VTH_FIELD);
			*value = (int32_t)32*3052*val/10000;
		} else if(readWrite == WRITE) {
			int32_t val = (int32_t)(*value*10000/(3052*32));
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_OVERVOLTAGE_VTH_FIELD, val);
		}
		break;
	case 222:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			int32_t temp = tmc5262_fieldRead(DEFAULT_ICID, TMC5262_OVERTEMPPREWARNING_VTH_FIELD);
			*value = (int32_t)(temp-2038)/7.7;
		} else if(readWrite == WRITE) {
			float valf  = *value*7.7;
			int32_t val = (int32_t)valf;
			val = val+2038;
			tmc5262_fieldWrite(DEFAULT_ICID, TMC5262_OVERTEMPPREWARNING_VTH_FIELD, val);
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
	if(motor >= TMC5262_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5262.velocity;

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc5262_writeRegister(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc5262_readRegister(DEFAULT_MOTOR, address);
}

static void periodicJob(uint32_t tick)
{
	tmc5262_periodicJob(&TMC5262, tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);

	switch(type) {
	case 0: // Reading analog values at DIAG0
		*value = *HAL.ADCs->AIN0;
		break;
	case 1: // Reading analog values at DIAG1
		*value = *HAL.ADCs->AIN1;
		break;
	default:
		return TMC_ERROR_TYPE;
	}

	return TMC_ERROR_NONE;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:  // simulate reference switches, set high to support external ref swiches
		/*
		 * The the TMC5262 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */
		if(!(*value & ~3))
		{
			if(*value & (1<<0))
			{
				HAL.IOs->config->toInput(Pins.REFR_INT); // pull up -> set it to floating causes high
			}
			else
			{
				HAL.IOs->config->toOutput(Pins.REFR_INT);
				HAL.IOs->config->setLow(Pins.REFR_INT);
			}

			if(*value & (1<<1))
			{
				HAL.IOs->config->toInput(Pins.REFL_INT); // pull up -> set it to floating causes high
			}
			else
			{
				HAL.IOs->config->toOutput(Pins.REFL_INT);
				HAL.IOs->config->setLow(Pins.REFL_INT);
			}
		}
		else
		{
			errors |= TMC_ERROR_VALUE;
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
	HAL.IOs->config->setHigh(Pins.N_DRN_EN);
	HAL.IOs->config->reset(Pins.N_SLEEP);
	HAL.IOs->config->reset(Pins.REFL_INT);
	HAL.IOs->config->reset(Pins.REFR_INT);
	HAL.IOs->config->reset(Pins.IREF_R2);
	HAL.IOs->config->reset(Pins.IREF_R3);
};

static uint8_t reset()
{
	if(!tmc5262_readRegister(DEFAULT_MOTOR, TMC5262_VACTUAL))
		tmc5262_reset(&TMC5262);

	return 1;
}

static uint8_t restore()
{
	return tmc5262_restore(&TMC5262);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.N_DRN_EN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.N_DRN_EN);
}

static void timer_overflow(timer_channel channel)
{
	UNUSED(channel);
	
	// RAMDebug
	debug_nextProcess();
}

void TMC5262_init(void)
{

	Pins.N_DRN_EN = &HAL.IOs->pins->DIO0;
	Pins.N_SLEEP = &HAL.IOs->pins->DIO8;
	Pins.REFL_INT = &HAL.IOs->pins->DIO6;
	Pins.REFR_INT = &HAL.IOs->pins->DIO7;
	Pins.IREF_R2 = &HAL.IOs->pins->DIO13;
	Pins.IREF_R3 = &HAL.IOs->pins->DIO14;
	
	HAL.IOs->config->toOutput(Pins.N_DRN_EN);
	HAL.IOs->config->toOutput(Pins.N_SLEEP);
	HAL.IOs->config->toOutput(Pins.IREF_R2);
	HAL.IOs->config->toOutput(Pins.IREF_R3);

	HAL.IOs->config->setHigh(Pins.N_SLEEP);
	HAL.IOs->config->setHigh(Pins.N_DRN_EN);
	HAL.IOs->config->setHigh(Pins.IREF_R2);
	HAL.IOs->config->setHigh(Pins.IREF_R2);

	HAL.IOs->config->toInput(Pins.REFL_INT);
	HAL.IOs->config->toInput(Pins.REFR_INT);

	TMC5262_SPIChannel = &HAL.SPI->ch1;
	TMC5262_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_RESET;

	tmc5262_init(&TMC5262, 0, Evalboards.ch1.config);

	vmax_position = 0;

	Evalboards.ch1.rotate               = rotate;
	Evalboards.ch1.right                = right;
	Evalboards.ch1.left                 = left;
	Evalboards.ch1.stop                 = stop;
	Evalboards.ch1.GAP                  = GAP;
	Evalboards.ch1.SAP                  = SAP;
	Evalboards.ch1.GIO                  = GIO;
	Evalboards.ch1.moveTo               = moveTo;
	Evalboards.ch1.moveBy               = moveBy;
	Evalboards.ch1.writeRegister        = writeRegister;
	Evalboards.ch1.readRegister         = readRegister;
	Evalboards.ch1.periodicJob          = periodicJob;
	Evalboards.ch1.userFunction         = userFunction;
	Evalboards.ch1.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch1.enableDriver         = enableDriver;
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = TMC5262_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	Timer.overflow_callback = timer_overflow;
	Timer.init();
	Timer.setFrequency(TMC5262_RAMDEBUG_TIMER, 10000);
	debug_updateFrequency(10000);
	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
