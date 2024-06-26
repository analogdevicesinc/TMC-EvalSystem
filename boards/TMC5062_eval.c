/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5062/TMC5062.h"


static TMC5062BusType activeBus = IC_BUS_SPI;
static SPIChannelTypeDef *TMC5062_SPIChannel;
static UART_Config *TMC5062_UARTChannel;

// Usage note: use one TypeDef per IC
typedef struct {
    ConfigurationTypeDef *config;
    uint8_t motors[TMC5062_MOTORS];

    // External frequency supplied to the IC (or 16MHz for internal frequency)
    uint32_t chipFrequency;

    // Velocity estimation (for dcStep)
    uint32_t measurementInterval;
    uint32_t oldTick;
    int32_t oldXActual[TMC5062_MOTORS];
    int32_t velocity[TMC5062_MOTORS];

} TMC5062TypeDef;

´static TMC5062TypeDef TMC5062;

void tmc5062_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC5062_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5062_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
    int32_t status = UART_readWrite(TMC5062_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

TMC5062BusType tmc5062_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  222  // VM[V/10] max +10%


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

//static void init_comm(TMC5062BusType mode);

static void periodicJob(uint32_t tick);
static void checkErrors	(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);
static void configCallback(TMC5062TypeDef *tmc5062, ConfigState state);

static ConfigurationTypeDef *TMC5062_config;

// When using multiple ICs you can map them here
static inline TMC5062TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &TMC5062;
}

typedef struct
{
	IOPinTypeDef  *DRV_ENN;
	IOPinTypeDef  *INT_ENCA;
	IOPinTypeDef  *PP_ENCB;
	IOPinTypeDef  *SWSEL;
	IOPinTypeDef  *SWIOP1;
	IOPinTypeDef  *SWIOP2;
	IOPinTypeDef  *SWION;

	IOPinTypeDef  *CLK;
	IOPinTypeDef  *SDI;
	IOPinTypeDef  *SDO;
	IOPinTypeDef  *SCK;
	IOPinTypeDef  *CS;

} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	tmc5062_rotate(motorToIC(motor),motor, velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	tmc5062_right(motorToIC(motor),motor, velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	tmc5062_left(motorToIC(motor),motor, velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	tmc5062_stop(motorToIC(motor),motor);

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc5062_moveTo(motorToIC(motor),motor, position, TMC5062_config->shadowRegister[TMC5062_VMAX(motor)]);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc5062_moveBy(motorToIC(motor),motor, TMC5062_config->shadowRegister[TMC5062_VMAX(motor)], ticks);

	return 0;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;
	int32_t buffer;

	if(motor >= TMC5062_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			readRegister(motor, TMC5062_XTARGET(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_XTARGET(motor), *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			readRegister(motor, TMC5062_XACTUAL(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_XACTUAL(motor), *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VMAX(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VMAX(motor), abs(*value));
		}
		break;
	case 3:
		// todo CHECK 3: min max actually velocity min and velocity max? (JE) #4
		// Actual speed
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VACTUAL(motor), value);
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
			readRegister(motor, TMC5062_RAMPMODE(motor), &buffer);
			if(buffer == TMC5062_MODE_POSITION)
				writeRegister(motor, TMC5062_VMAX(motor), abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			readRegister(motor,TMC5062_AMAX(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_AMAX(motor), *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_IRUN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_IRUN_FIELD(motor), *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_IHOLD_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_IHOLD_FIELD(motor), *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_POSITION_REACHED_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = !tmc5062_fieldRead(motor, TMC5062_STATUS_STOP_R_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !tmc5062_fieldRead(motor, TMC5062_STATUS_STOP_L_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_STOP_R_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_STOP_R_ENABLE_FIELD(motor), *value);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_STOP_L_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_STOP_L_ENABLE_FIELD(motor), *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			readRegister(motor, TMC5062_SWMODE(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_SWMODE(motor), *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			readRegister(motor, TMC5062_A1(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_A1(motor), *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			readRegister(motor, TMC5062_V1(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_V1(motor), *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			readRegister(motor, TMC5062_DMAX(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_DMAX(motor), *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			readRegister(motor, TMC5062_D1(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_D1(motor), *value);
		}
		break;
	case 19:
		// VSTART
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VSTART(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VSTART(motor), *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VSTOP(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VSTOP(motor), *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			readRegister(motor, TMC5062_TZEROWAIT(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_TZEROWAIT(motor), *value);
		}
		break;
	case 22:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VCOOLTHRS(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VCOOLTHRS(motor), *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VHIGH(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VHIGH(motor), *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VDCMIN(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VDCMIN(motor), *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_VHIGHFS_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_VHIGHFS_FIELD(motor), *value);
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
			*value = 256 >> tmc5062_fieldRead(motor, TMC5062_MRES_FIELD(motor));
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
				tmc5062_fieldWrite(motor, TMC5062_MRES_FIELD(motor), *value);
			}
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_TBL_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_TBL_FIELD(motor), *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_CHM_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_CHM_FIELD(motor), *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_DISFDCC_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_DISFDCC_FIELD(motor), *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			if(tmc5062_fieldRead(motor, TMC5062_CHM_FIELD(motor)))
			{
				*value = tmc5062_fieldRead(motor, TMC5062_HEND_FIELD(motor));
			}
			else
			{
				*value = tmc5062_fieldRead(motor, TMC5062_TFD_ALL_FIELD(motor));
				if(tmc5062_fieldRead(motor, TMC5062_TFD_3_FIELD(motor)))
					*value |= 1<<3; // MSB wird zu value hinzugefügt
			}
		} else if(readWrite == WRITE) {
			readRegister(motor, TMC5062_CHOPCONF(motor), &buffer);
			if(buffer & (1<<14))
			{
				tmc5062_fieldWrite(motor, TMC5062_HEND_FIELD(motor), *value);
			}
			else
			{
				tmc5062_fieldWrite(motor, TMC5062_TFD_3_FIELD(motor), (*value & 1<<3)? 1:0);
				tmc5062_fieldWrite(motor,  TMC5062_TFD_ALL_FIELD(motor), *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		readRegister(motor, TMC5062_CHOPCONF(motor), &buffer);
		if(readWrite == READ) {
			if( buffer & (1<<14))
			{
				*value = tmc5062_fieldRead(motor, TMC5062_HSTRT_FIELD(motor));
			}
			else
			{
				*value = tmc5062_fieldRead(motor, TMC5062_OFFSET_FIELD(motor));
				if(tmc5062_fieldRead(motor, TMC5062_TFD_3_FIELD(motor)))
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1<<14))
			{
				tmc5062_fieldWrite(motor, TMC5062_HSTRT_FIELD(motor), *value);
			}
			else
			{
				tmc5062_fieldWrite(motor, TMC5062_OFFSET_FIELD(motor), *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_TOFF_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_TOFF_FIELD(motor), *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SEIMIN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SEIMIN_FIELD(motor), *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SEDN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SEDN_FIELD(motor), *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SEMAX_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SEMAX_FIELD(motor), *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SEUP_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SEUP_FIELD(motor), *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SEMIN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SEMIN_FIELD(motor), *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SFILT_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SFILT_FIELD(motor), *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SGT_FIELD(motor));
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SGT_FIELD(motor), *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_VSENSE_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_VSENSE_FIELD(motor), *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_CS_ACTUAL_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// reset stall
		if(readWrite == READ) {
			readRegister(motor, TMC5062_SWMODE(motor), &buffer);
			if( buffer & (1<<10))
			{
				readRegister(motor, TMC5062_VCOOLTHRS(motor), value);
			}
			else
				buffer = 0;
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VCOOLTHRS(motor),*value);
			tmc5062_fieldWrite(motor, TMC5062_SG_STOP_FIELD(motor), (*value)? 1:0);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC5062_VCOOLTHRS(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_VCOOLTHRS(motor),*value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_RNDTF_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_RNDTF_FIELD(motor), *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SYNC_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5062_fieldWrite(motor, TMC5062_SYNC_FIELD(motor), *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = tmc5062_fieldRead(motor, TMC5062_SG_RESULT_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			readRegister(motor, TMC5062_XENC(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_XENC(motor),*value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			readRegister(motor, TMC5062_ENC_CONST(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5062_ENC_CONST(motor),*value);
		}
		break;
	case 211:
		readRegister(motor, TMC5062_GCONF, &buffer);
		if(readWrite == READ) {
			// encoder enable
			switch(motor)
			{
			case 0:
				buffer &= (1<<3) | (1<<4);
				*value = (buffer == (1<<4))? 1 : 0;
				break;
			case 1:
				buffer &= (1<<5) | (1<<6);
				*value = (buffer == ((1<<5) | (0<<6)))? 1 : 0;
				break;
			}
		} else if(readWrite == WRITE) {
			// encoder enable
			switch(motor)
			{
			case 0:
				if(*value)
					buffer = (buffer & ~(1<<3)) | (1<<4);
				else
					buffer = (buffer | (1<<3)) & ~(1<<4);
				writeRegister(motor, TMC5062_GCONF, buffer);
				break;
			case 1:		// enable ENCODER2 - disable REF
				if(*value)
					//tempValue = (tempValue | (1<<5)) & ~(5<<5);
					buffer = (buffer | (1<<5)) & ~(1<<6); //todo: CHECK 3: Sind die Änderungen richtig? Codemäßig macht es so Sinn, aber die Bits sind in der Dokumentation als reserved markiert (LH) #3
				else
					//tempValue = (tempValue & ~(1<<6)) | ~(1<<6);
					buffer = (buffer & ~(1<<5)) | (1<<6); //todo: CHECK 3: Sind die Änderungen richtig? Codemäßig macht es so Sinn, aber die Bits sind in der Dokumentation als reserved markiert (LH) #4
				writeRegister(motor, TMC5062_GCONF, buffer);
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
	if(motor >= TMC5062_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5062.velocity[motor];

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    tmc5062_writeRegister(motor, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    *value = tmc5062_readRegister(motor, address);
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
	for(uint8_t motor = 0; motor < TMC5062_MOTORS; motor++)
		if(tmc5062_readRegister(motor, TMC5062_VACTUAL(motor)) != 0)
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
{	UNUSED(tmc5062);
	if(state == CONFIG_RESET)
	{	// Change hardware-preset registers here
		for(uint8_t motor = 0; motor < TMC5062_MOTORS; motor++)
			tmc5062_writeRegister(motor, TMC5062_PWMCONF(motor), 0x000504C8);

		// Fill missing shadow registers (hardware preset registers)
		tmc5062_fillShadowRegisters(&TMC5062);
	}
}

static void init_comm(TMC5062BusType mode)
{
	TMC5062_UARTChannel = HAL.UART;
	switch(mode) {
	case IC_BUS_UART:
		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
		HAL.IOs->config->toOutput(Pins.SCK);
		HAL.IOs->config->toOutput(Pins.SDI);
		HAL.IOs->config->toOutput(Pins.SDO);
		HAL.IOs->config->toOutput(Pins.CS);
		HAL.IOs->config->setLow(Pins.SCK);
		HAL.IOs->config->setLow(Pins.SDI);
		HAL.IOs->config->setLow(Pins.SDO);
		HAL.IOs->config->setLow(Pins.CS);

		HAL.IOs->config->setHigh(Pins.SWSEL);
		TMC5062_UARTChannel -> mode = UART_MODE_SINGLE_WIRE;
		TMC5062_UARTChannel = HAL.UART;
		TMC5062_UARTChannel->rxtx.init();
		break;
	case IC_BUS_SPI:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
		HAL.IOs->config->setLow(Pins.SWSEL);

		SPI.init();
		TMC5062_UARTChannel->rxtx.deInit();
		TMC5062_SPIChannel = &HAL.SPI->ch1;
		TMC5062_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
	default:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
		HAL.IOs->config->setLow(Pins.SWSEL);

		SPI.init();
		TMC5062_UARTChannel->rxtx.deInit();
		TMC5062_SPIChannel = &HAL.SPI->ch1;
		TMC5062_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		activeBus = IC_BUS_SPI;
		break;
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

	Pins.SCK       = &HAL.IOs->pins->SPI1_SCK;
	Pins.SDI       = &HAL.IOs->pins->SPI1_SDI;
	Pins.SDO       = &HAL.IOs->pins->SPI1_SDO;
	Pins.CS        = &HAL.IOs->pins->SPI1_CSN;

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.SWSEL);
	HAL.IOs->config->toInput(Pins.INT_ENCA);
	HAL.IOs->config->toInput(Pins.PP_ENCB);

	HAL.IOs->config->setLow(Pins.SWSEL);

	init_comm(activeBus);

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
	Evalboards.ch1.numberOfMotors       = TMC5062_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	enableDriver(DRIVER_ENABLE);
};
