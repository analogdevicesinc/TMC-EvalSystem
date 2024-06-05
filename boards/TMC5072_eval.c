/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5072/TMC5072.h"

static TMC5072BusType activeBus = IC_BUS_SPI;
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5072_SPIChannel;
static UART_Config *TMC5072_UARTChannel;

#define DEFAULT_MOTOR  0


void tmc5072_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
	UNUSED(icID);
	TMC5072_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5072_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
	UNUSED(icID);
	int32_t status = UART_readWrite(TMC5072_UARTChannel, data, writeLength, readLength);
	if(status == -1)
		return false;
	return true;
}

TMC5072BusType tmc5072_getBusType(uint16_t icID)
{
	UNUSED(icID);

	return activeBus;
}

uint8_t tmc5072_getNodeAddress(uint16_t icID)
{
	UNUSED(icID);

	return nodeAddress;
}

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  280  // VM[V/10] max +10%

#define DEFAULT_CHANNEL 0

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
static void checkErrors	(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void configCallback(TMC5072TypeDef *tmc5072, ConfigState state);
static void enableDriver(DriverState state);

static uint32_t vmax_position[TMC5072_MOTORS];

// Translate motor number to TMC5072TypeDef
// When using multiple ICs you can map them here
static inline TMC5072TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC5072;
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

} PinsTypeDef;

static PinsTypeDef Pins;

// => Functions forwarded to API
static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	tmc5072_rotate(motorToIC(motor), motor, velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	tmc5072_right(motorToIC(motor), motor, velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	tmc5072_left(motorToIC(motor), motor, velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	tmc5072_stop(motorToIC(motor), motor);

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc5072_moveTo(motorToIC(motor), motor, position, vmax_position[motor]);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc5072_moveBy(motorToIC(motor), motor, vmax_position[motor], ticks);

	return 0;
}
// <= Functions forwarded to API

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;
	int32_t tempValue;

	if(motor >= TMC5072_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_XTARGET(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_XTARGET(motor), *value);
		}
		break;
	case 1:
		// actual position
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_XACTUAL(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_XACTUAL(motor), *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VMAX(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VMAX(motor), abs(*value));
		}
		break;
	case 3:
		// todo CHECK 3: min max actually velocity min and velocity max? (JE) #5
		// Actual speed
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VACTUAL(motor));
			*value = CAST_Sn_TO_S32(*value, 24);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = vmax_position[motor];
		} else if(readWrite == WRITE) {
			vmax_position[motor] = abs(*value);
			if(tmc5072_readRegister(motor, TMC5072_RAMPMODE(motor)) == TMC5072_MODE_POSITION)
				tmc5072_writeRegister(motor, TMC5072_VMAX(motor), vmax_position[motor]);
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_AMAX(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_AMAX(motor), *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_IRUN_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_IRUN_FIELD(motor), *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = field_read(motor,  TMC5072_IHOLD_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_IHOLD_FIELD(motor), *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_POSITION_REACHED_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_STATUS_STOP_R_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_STATUS_STOP_L_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_STOP_R_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
			// configure right stop bits
			field_write(motor, TMC5072_STOP_R_ENABLE_FIELD(motor), (*value > 0)? 1:0);
			field_write(motor, TMC5072_POL_STOP_R_FIELD(motor), (*value == 2)? 1:0);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_STOP_L_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
			// configure left stop bits
			field_write(motor, TMC5072_STOP_L_ENABLE_FIELD(motor), (*value > 0)? 1:0);
			field_write(motor, TMC5072_POL_STOP_L_FIELD(motor), (*value == 2)? 1:0);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_SWMODE(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_SWMODE(motor), *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_A1(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_A1(motor), *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_V1(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_V1(motor), *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_DMAX(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_DMAX(motor), *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_D1(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_D1(motor), *value);
		}
		break;
	case 19:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VSTART(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VSTART(motor), *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VSTOP(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VSTOP(motor), *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_TZEROWAIT(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_TZEROWAIT(motor), *value);
		}
		break;
	case 22:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VCOOLTHRS(motor), *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VHIGH(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VHIGH(motor), *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VDCMIN(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VDCMIN(motor), *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_VHIGHFS_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_VHIGHFS_FIELD(motor), *value);
		}
		break;
	case 29:
		if(readWrite == READ) {
			*value = motorToIC(motor)->velocity[motor];
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 256 >> field_read(motor, TMC5072_TOFF_FIELD(motor));
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
				field_write(motor, TMC5072_MRES_FIELD(motor), *value);
			}
			//else TMCL.reply->Status = REPLY_INVALID_VALUE;
		}
		break;
	case 162:
		// Chopper blank time
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_TBL_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_TBL_FIELD(motor), *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_RNDTF_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_RNDTF_FIELD(motor), *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_DISFDCC_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_DISFDCC_FIELD(motor), *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		tempValue = field_read(motor, TMC5072_CHM_FIELD(motor));
		if(readWrite == READ) {
			if(tempValue)
			{
				*value = field_read(motor, TMC5072_HEND_FIELD(motor));
			}
			else
			{
				*value = field_read(motor, TMC5072_TFD_2__0__FIELD(motor));
				*value |= (*value & (1<<11)) << 3;
			}
		} else if(readWrite == WRITE) {
			if(tempValue)
			{
				field_write(motor, TMC5072_HEND_FIELD(motor), *value);
			}
			else
			{
				field_write(motor, TMC5072_TFD___FIELD(motor), (*value & (1<<3))? 1:0);
				field_write(motor, TMC5072_TFD_2__0__FIELD(motor), *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		tempValue = field_read(motor, TMC5072_CHM_FIELD(motor));
		if(readWrite == READ) {
			if(tempValue)
			{
				*value = field_read(motor, TMC5072_HSTRT_FIELD(motor));
			}
			else
			{
				*value = field_read(motor, TMC5072_OFFSET_FIELD(motor));
				tempValue = field_read(motor, TMC5072_TFD___FIELD(motor));
				*value |= tempValue << 3;
			}
		} else if(readWrite == WRITE) {
			if(tempValue)
			{
				field_write(motor, TMC5072_HSTRT_FIELD(motor), *value);
			}
			else
			{
				field_write(motor, TMC5072_OFFSET_FIELD(motor), *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_TOFF_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_TOFF_FIELD(motor), *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SEIMIN_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SEIMIN_FIELD(motor), *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SEDN_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SEDN_FIELD(motor), *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SEMAX_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SEMAX_FIELD(motor), *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SEUP_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SEUP_FIELD(motor), *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SEMIN_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SEMIN_FIELD(motor), *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SFILT_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SFILT_FIELD(motor), *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SGT_FIELD(motor));
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_SGT_FIELD(motor), *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_VSENSE_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_VSENSE_FIELD(motor), *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_CS_ACTUAL_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		if(readWrite == READ) {
			if(field_read(motor, TMC5072_SG_STOP_FIELD(motor)))
			{
				*value = tmc5072_readRegister(motor, TMC5072_VCOOLTHRS(motor));
			}
			else
			{
				*value = 0;
			}
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VCOOLTHRS(motor),*value);
			field_write(motor, TMC5072_SG_STOP_FIELD(motor), (*value)? 1:0);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_VCOOLTHRS(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_VCOOLTHRS(motor),*value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_RNDTF_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_RNDTF_FIELD(motor), *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_SG_RESULT_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = field_read(motor, TMC5072_XENC_FIELD(motor));
		} else if(readWrite == WRITE) {
			field_write(motor, TMC5072_XENC_FIELD(motor), *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = tmc5072_readRegister(motor, TMC5072_ENC_CONST(motor));
		} else if(readWrite == WRITE) {
			tmc5072_writeRegister(motor, TMC5072_ENC_CONST(motor),*value);
		}
		break;
	case 211:
		if(readWrite == READ) {
			// encoder enable
			switch(motor)
			{
			case 0:
				tempValue = tmc5072_readRegister(motor, TMC5072_GCONF);
				tempValue &= 0x18; //(1<<3) | (1<<4);
				*value = (tempValue == 0x10) ? 1 : 0;
				break;
			case 1:
				tempValue = tmc5072_readRegister(motor, TMC5072_GCONF);
				tempValue &= 0x60; //(1<<5) | (1<<6);
				*value = (tempValue == 0x20) ? 1 : 0;
				break;
			}
		} else if(readWrite == WRITE) {
			// encoder enable
			switch(motor)
			{
			case 0:
				field_write(motor, TMC5072_POSCMP_ENABLE_FIELD(motor), (*value) ? 0 : 1);
				field_write(motor, TMC5072_ENC1_REFSEL_FIELD(motor), (*value) ? 1 : 0);
				break;
			case 1:		// enable ENCODER2 - disable REF
				field_write(motor, TMC5072_ENC2_ENABLE_FIELD(motor), (*value) ? 1 : 0);
				field_write(motor, TMC5072_ENC2_REFSEL_FIELD(motor), (*value) ? 0 : 1);
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
	if(motor >= TMC5072_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5072.velocity[motor];

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
	tmc5072_writeRegister(motor, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
	*value = tmc5072_readRegister(motor, (uint8_t) address);
}

static void periodicJob(uint32_t tick)
{
	for(uint8_t motor = 0; motor < TMC5072_MOTORS; motor++)
	{
		tmc5072_periodicJob(motorToIC(motor), tick);
	}
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
	for(uint8_t motor = 0; motor < TMC5072_MOTORS; motor++)
		if(tmc5072_readRegister(motor, TMC5072_VACTUAL(motor)) != 0)
			return 0;

	return tmc5072_reset(&TMC5072);
}

static uint8_t restore()
{
	return tmc5072_restore(&TMC5072);
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

static void configCallback(TMC5072TypeDef *tmc5072, ConfigState state)
{
	if(state == CONFIG_RESET)
	{	// Change hardware-preset registers here
		for(uint8_t motor = 0; motor < TMC5072_MOTORS; motor++)
			tmc5072_writeRegister(tmc5072, TMC5072_PWMCONF(motor), 0x000504C8);

		// Fill missing shadow registers (hardware preset registers)
		tmc5072_fillShadowRegisters(&TMC5072);
	}
}

static void init_comm(TMC5072BusType mode)
{
	TMC5072_UARTChannel = HAL.UART;
	switch(mode) {
	case IC_BUS_UART:

		HAL.IOs->config->reset(Pins.SWION);
		HAL.IOs->config->toInput(Pins.SWIOP1);
		HAL.IOs->config->toInput(Pins.SWIOP2);


		HAL.IOs->config->toOutput(Pins.INT_ENCA);
		HAL.IOs->config->toOutput(Pins.PP_ENCB);


		HAL.IOs->config->setHigh(Pins.SWSEL);
		TMC5072_UARTChannel->mode = UART_MODE_SINGLE_WIRE;
		TMC5072_UARTChannel = HAL.UART;
		TMC5072_UARTChannel->rxtx.init();
		break;
	case IC_BUS_SPI:

		HAL.IOs->config->toInput(Pins.SWION);
		HAL.IOs->config->toInput(Pins.SWIOP1);
		HAL.IOs->config->toInput(Pins.SWIOP2);

		HAL.IOs->config->toInput(Pins.INT_ENCA);
		HAL.IOs->config->toInput(Pins.PP_ENCB);

		SPI.init();
		HAL.IOs->config->setLow(Pins.SWSEL);
		TMC5072_UARTChannel->rxtx.deInit();
		TMC5072_SPIChannel = &HAL.SPI->ch1;
		TMC5072_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
	}
}

void TMC5072_init(void)
{
	tmc5072_init(&TMC5072, 0, Evalboards.ch1.config, &tmc5072_defaultRegisterResetState[0]);
	tmc5072_setCallback(&TMC5072, configCallback);

	Pins.DRV_ENN   = &HAL.IOs->pins->DIO0;
	Pins.INT_ENCA  = &HAL.IOs->pins->DIO5;
	Pins.PP_ENCB   = &HAL.IOs->pins->DIO6;

	Pins.SWSEL     = &HAL.IOs->pins->DIO16;	//Single wire selection
	Pins.SWIOP1    = &HAL.IOs->pins->DIO17;
	Pins.SWIOP2    = &HAL.IOs->pins->DIO18;
	Pins.SWION     = &HAL.IOs->pins->DIO19;

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.SWSEL);




	init_comm(activeBus);

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
	Evalboards.ch1.numberOfMotors       = TMC5072_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	for(uint8_t motor = 0; motor < TMC5072_MOTORS; motor++)
	{
		vmax_position[motor] = motorToIC(motor)->config->shadowRegister[TMC5072_VMAX(motor)];
	}

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
