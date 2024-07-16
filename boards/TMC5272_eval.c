/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5272/TMC5272.h"

const uint8_t tmcCRCTable_Poly7Reflected[256] = {
			0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
			0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
			0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
			0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
			0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
			0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
			0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
			0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
			0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
			0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
			0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
			0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
			0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
			0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
			0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
			0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
};

static TMC5272BusType activeBus = IC_BUS_SPI;
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5272_SPIChannel;
static UART_Config *TMC5272_UARTChannel;

#define DEFAULT_MOTOR  0

// Typedefs
typedef struct
{
    ConfigurationTypeDef *config;
    int32_t oldX[TMC5272_MOTORS];
    int32_t velocity[TMC5272_MOTORS];
    uint32_t oldTick;
} TMC5272TypeDef;

static TMC5272TypeDef TMC5272;

void tmc5272_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
	UNUSED(icID);
	TMC5272_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5272_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
	UNUSED(icID);
	int32_t status = UART_readWrite(TMC5272_UARTChannel, data, writeLength, readLength);
	if(status == -1)
		return false;
	return true;
}

TMC5272BusType tmc5272_getBusType(uint16_t icID)
{
	UNUSED(icID);

	return activeBus;
}

uint8_t tmc5272_getNodeAddress(uint16_t icID)
{
	UNUSED(icID);

	return nodeAddress;
}


#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max


static bool vMaxModified = false;
static uint32_t vmax_position[TMC5272_MOTORS];

static bool noRegResetnSLEEP = false;
static uint32_t nSLEEPTick;
static uint32_t targetAddressUart = 0;

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


static void init_comm(TMC5272BusType mode);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static TMC5272TypeDef TMC5272;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC5272_CRC(data, length) tmc_CRC8(data, length, 1)

// When using multiple ICs you can map them here
static inline TMC5272TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &TMC5272;
}

// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc5272_CRC8(uint8_t *data, size_t length)
{
	return tmc_CRC8(data, length, 1);
	//TMC5272_CRC(data, length);
}


typedef struct
{
	IOPinTypeDef  *REFL_UC;
	IOPinTypeDef  *REFR_UC;
	IOPinTypeDef  *DRV_ENN_CFG6;
	IOPinTypeDef  *ENCA_DCIN_CFG5;
	IOPinTypeDef  *ENCB_DCEN_CFG4;
	IOPinTypeDef  *ENCN_DCO;
	IOPinTypeDef  *UART_MODE;
	IOPinTypeDef  *CLK;
	IOPinTypeDef  *SDI;
	IOPinTypeDef  *SDO;
	IOPinTypeDef  *SCK;
	IOPinTypeDef  *CS;

	IOPinTypeDef  *SWN_DIAG0;
	IOPinTypeDef  *SWP_DIAG1;
	IOPinTypeDef  *nSLEEP;
	IOPinTypeDef  *IREF_R2;
	IOPinTypeDef  *IREF_R3;

} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	tmc5272_rotate(motorToIC(motor),motor, velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	tmc5272_right(motorToIC(motor),motor, velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	tmc5272_left(motorToIC(motor),motor, velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	tmc5272_stop(motorToIC(motor),motor);

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc5272_moveTo(motorToIC(motor),motor, position, vmax_position[motor]);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc5272_moveBy(motorToIC(motor),motor, vmax_position[motor], ticks);

	return 0;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	int32_t buffer;
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC5272_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			readRegister(motor, TMC5272_XTARGET(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_XTARGET(motor), *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			readRegister(motor, TMC5272_XACTUAL(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_XACTUAL(motor), *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			if (motor == 0){
				if (tmc5272_fieldRead(motor, TMC5272_RAMPMODE_M0_RAMPMODE_FIELD) == 2){
				    readRegister(motor, TMC5272_VMAX(motor), value);
					*value = -(*value);
				}
				else
					readRegister(motor, TMC5272_VMAX(motor), value);
			}
			else if (motor == 1){
				if (tmc5272_fieldRead(motor, TMC5272_RAMPMODE_M1_RAMPMODE_FIELD) == 2){
				    readRegister(motor, TMC5272_VMAX(motor), value);
					*value = -(*value);
				}
				else
					readRegister(motor, TMC5272_VMAX(motor), value);
			}
			else
				readRegister(motor, TMC5272_VMAX(motor), value);
		}
		else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_VMAX(motor), abs(*value));
			vMaxModified = true;
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			readRegister(motor, TMC5272_VACTUAL(motor), value);
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
			if(motor == 0)
			{
				if(tmc5272_fieldRead(motor, TMC5272_RAMPMODE_M0_RAMPMODE_FIELD) == TMC5272_MODE_POSITION)
					writeRegister(motor, TMC5272_VMAX(motor), abs(*value));

			}
			else if(motor == 1)
			{
				if(tmc5272_fieldRead(motor, TMC5272_RAMPMODE_M1_RAMPMODE_FIELD) == TMC5272_MODE_POSITION)
					writeRegister(motor, TMC5272_VMAX(motor), abs(*value));
			}
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			readRegister(motor, TMC5272_AMAX(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_AMAX(motor), *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {

			*value = tmc5272_fieldRead(motor, TMC5272_IHOLD_IRUN_IRUN_FIELD(motor));

		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_IHOLD_IRUN_IRUN_FIELD(motor), *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_IHOLD_IRUN_IHOLD_FIELD(motor));
		} else if(readWrite == WRITE) {
		    tmc5272_fieldWrite(motor, TMC5272_IHOLD_IRUN_IHOLD_FIELD(motor), *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_RAMP_STAT_POSITION_REACHED_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
		    *value = !tmc5272_fieldRead(motor, TMC5272_RAMP_STAT_STATUS_STOP_R_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !tmc5272_fieldRead(motor, TMC5272_RAMP_STAT_STATUS_STOP_L_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SW_MODE_STOP_R_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
		    tmc5272_fieldWrite(motor, TMC5272_SW_MODE_STOP_R_ENABLE_FIELD(motor), *value);

		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SW_MODE_STOP_L_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
		    tmc5272_fieldWrite(motor, TMC5272_SW_MODE_STOP_L_ENABLE_FIELD(motor), *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			readRegister(motor, TMC5272_SW_MODE(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_SW_MODE(motor), *value);
		}
		break;
	case 15:
		// Maximum Deceleration
		if(readWrite == READ) {
			readRegister(motor, TMC5272_DMAX(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_DMAX(motor), *value);
		}
		break;
	case 16:
		// Velocity VSTART
		if(readWrite == READ) {
			readRegister(motor, TMC5272_VSTART(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_VSTART(motor), *value);
		}
		break;
	case 17:
		// Acceleration A1
		if(readWrite == READ) {
			readRegister(motor, TMC5272_A1(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_A1(motor), *value);
		}
		break;
	case 18:
		// Velocity V1
		if(readWrite == READ) {
			readRegister(motor, TMC5272_V1(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_V1(motor), *value);
		}
		break;
	case 19:
		// Deceleration D1
		if(readWrite == READ) {
			readRegister(motor, TMC5272_D1(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_D1(motor), *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			readRegister(motor, TMC5272_VSTOP(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_VSTOP(motor), *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			readRegister(motor, TMC5272_TZEROWAIT(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_TZEROWAIT(motor), *value);
		}
		break;
	case 22:
		// Velocity V2
		if(readWrite == READ) {
			readRegister(motor, TMC5272_V2(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_V2(motor), *value);
		}
		break;
	case 23:
		// Deceleration D2
		if(readWrite == READ) {
			readRegister(motor, TMC5272_D2(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_D2(motor), *value);
		}
		break;
	case 24:
		// Acceleration A2
		if(readWrite == READ) {
			readRegister(motor, TMC5272_A2(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_A2(motor), *value);
		}
		break;
	case 25:
		// TVMAX
		if(readWrite == READ) {
			readRegister(motor, TMC5272_TVMAX(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_TVMAX(motor), *value);
		}
		break;
	case 26:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			readRegister(motor, TMC5272_THIGH(motor), value);
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
			writeRegister(motor, TMC5272_THIGH(motor), *value);
		}
		break;
	case 27:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			readRegister(motor, TMC5272_VDCMIN(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_VDCMIN(motor), *value);
		}
		break;
	case 28:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_CHOPCONF_VHIGHCHM_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_VHIGHCHM_FIELD(motor), *value);
		}
		break;
	case 29:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_CHOPCONF_VHIGHFS_FIELD(motor));
		} else if(readWrite == WRITE) {
		    tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_VHIGHFS_FIELD(motor), *value);

		}
		break;
	case 30:
		// Measured Speed
		if(readWrite == READ) {
			*value = (int32_t)TMC5272.velocity[motor];
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;

		//	case 34:
		//		// Internal RSense
		//		if(readWrite == READ) {
		//			*value = TMC5272_tmc5272_fieldRead(motor, TMC5272_GCONF, TMC5272_REFR_DIR_MASK, TMC5272_REFR_DIR_SHIFT);
		//		} else if(readWrite == WRITE) {
		//			TMC5272_tmc5272_fieldWrite(motor, TMC5272_GCONF, TMC5272_REFR_DIR_MASK, TMC5272_REFR_DIR_SHIFT, *value);
		//		}
		//		break;
	case 35:
		// Global current scaler A
		if(readWrite == READ) {
			if(motor ==  0)
				*value = tmc5272_fieldRead(motor,TMC5272_GLOBAL_SCALER_GLOBALSCALER_M0_A_FIELD);
			else if(motor ==  1)
				*value = tmc5272_fieldRead(motor,TMC5272_GLOBAL_SCALER_GLOBALSCALER_M1_A_FIELD);

		} else if(readWrite == WRITE) {
			if(motor ==  0)
			{
				if(*value > 31)
				    tmc5272_fieldWrite(motor,TMC5272_GLOBAL_SCALER_GLOBALSCALER_M0_A_FIELD, *value);
				else
				    tmc5272_fieldWrite(motor,TMC5272_GLOBAL_SCALER_GLOBALSCALER_M0_A_FIELD, *value);
			}
			else if(motor ==  1)
			{
				if(*value > 31)
				    tmc5272_fieldWrite(motor,TMC5272_GLOBAL_SCALER_GLOBALSCALER_M1_A_FIELD, *value);
				else
				    tmc5272_fieldWrite(motor,TMC5272_GLOBAL_SCALER_GLOBALSCALER_M1_A_FIELD, 0);
			}
		}
		break;
	case 36:
		// Global current scaler B
		if(readWrite == READ) {
			if(motor ==  0)
				*value = tmc5272_fieldRead(motor, TMC5272_GLOBAL_SCALER_GLOBALSCALER_M0_B_FIELD);
			else if(motor ==  1)
			    *value = tmc5272_fieldRead(motor, TMC5272_GLOBAL_SCALER_GLOBALSCALER_M1_B_FIELD);
		} else if(readWrite == WRITE) {
			if(motor ==  0)
			{
				if(*value > 31)
				    tmc5272_fieldWrite(motor, TMC5272_GLOBAL_SCALER_GLOBALSCALER_M0_B_FIELD, *value);
				else
				    tmc5272_fieldWrite(motor, TMC5272_GLOBAL_SCALER_GLOBALSCALER_M0_B_FIELD, 0);
			}
			else if(motor ==  1)
			{
				if(*value > 31)
				    tmc5272_fieldWrite(motor, TMC5272_GLOBAL_SCALER_GLOBALSCALER_M1_B_FIELD, *value);
				else
				    tmc5272_fieldWrite(motor, TMC5272_GLOBAL_SCALER_GLOBALSCALER_M1_B_FIELD, 0);
			}
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> tmc5272_fieldRead(motor, TMC5272_CHOPCONF_MRES_FIELD(motor));
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
			    tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_MRES_FIELD(motor), *value);
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
			*value = tmc5272_fieldRead(motor, TMC5272_CHOPCONF_TBL_FIELD(motor));
		} else if(readWrite == WRITE) {
		    tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_TBL_FIELD(motor), *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_CHOPCONF_CHM_FIELD(motor));
		} else if(readWrite == WRITE) {
		    tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_CHM_FIELD(motor), *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_CHOPCONF_DISFDCC_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_DISFDCC_FIELD(motor), *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		readRegister(motor, TMC5272_CHOPCONF(motor), &buffer);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5272_CHOPCONF_CHM_SHIFT))
			{
				*value = (buffer >> TMC5272_CHOPCONF_HEND_OFFSET_SHIFT) & TMC5272_CHOPCONF_HEND_OFFSET_MASK;
			}
			else
			{
				*value = (buffer >> TMC5272_CHOPCONF_HSTRT_TFD210_SHIFT) & TMC5272_CHOPCONF_HSTRT_TFD210_MASK;
				if(buffer & TMC5272_CHOPCONF_HSTRT_TFD210_SHIFT)
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			readRegister(motor, TMC5272_CHOPCONF(motor), &buffer);
			if(buffer & (1<<14))
			{
				tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_HEND_OFFSET_FIELD(motor), *value);

			}
			else
			{
				tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_HSTRT_TFD210_FIELD(motor), (*value & (1<<3))); // MSB wird zu value dazugefügt
				tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_HSTRT_TFD210_FIELD(motor), *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		readRegister(motor, TMC5272_CHOPCONF(motor), &buffer);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5272_CHOPCONF_CHM_SHIFT))
			{
				*value = (buffer >> TMC5272_CHOPCONF_HSTRT_TFD210_SHIFT) & TMC5272_CHOPCONF_HSTRT_TFD210_MASK;
			}
			else
			{
				*value = (buffer >> TMC5272_CHOPCONF_HEND_OFFSET_SHIFT) & TMC5272_CHOPCONF_HEND_OFFSET_MASK;
				if(buffer & (1 << TMC5272_CHOPCONF_FD3_SHIFT))
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1 << TMC5272_CHOPCONF_CHM_SHIFT))
			{
				tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_HSTRT_TFD210_FIELD(motor), *value);
			}
			else
			{
				tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_HEND_OFFSET_FIELD(motor), *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_CHOPCONF_TOFF_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_CHOPCONF_TOFF_FIELD(motor), *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SEIMIN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SEIMIN_FIELD(motor), *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SEDN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SEDN_FIELD(motor), *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SEMAX_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SEMAX_FIELD(motor), *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SEUP_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SEUP_FIELD(motor), *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SEMIN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SEMIN_FIELD(motor), *value);
		}
		break;
	case 173:
		// stallGuard4 filter enable
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_THRS_SG4_FILT_EN_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_SG4_THRS_SG4_FILT_EN_FIELD(motor), *value);
		}
		break;
	case 174:
		// stallGuard4 threshold
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_THRS_SG4_THRS_FIELD(motor));
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_SG4_THRS_SG4_THRS_FIELD(motor), *value);
		}
		break;
	case 175:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SFILT_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SFILT_FIELD(motor), *value);
		}
		break;
	case 176:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_COOLCONF_SGT_FIELD(motor));
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_COOLCONF_SGT_FIELD(motor), *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_DRV_STATUS_CS_ACTUAL_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		//this function sort of doubles with 182 but is necessary to allow cross chip compliance
		if(readWrite == READ) {
			if(tmc5272_fieldRead(motor, TMC5272_SW_MODE_SG_STOP_FIELD(motor)))
			{
				readRegister(motor, TMC5272_TCOOLTHRS(motor), &buffer);
				*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
			}
			else
			{
				*value = 0;
			}
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_SW_MODE_SG_STOP_FIELD(motor), (*value)? 1:0);

			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC5272_TCOOLTHRS(motor), *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC5272_TCOOLTHRS(motor), &buffer);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC5272_TCOOLTHRS(motor), *value);
		}
		break;
	case 184:
		// SG_ANGLE_OFFSET
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_THRS_SG_ANGLE_OFFSET_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_SG4_THRS_SG_ANGLE_OFFSET_FIELD(motor), *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			readRegister(motor, TMC5272_CHOPCONF(motor), value);
			*value = (*value >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			readRegister(motor, TMC5272_CHOPCONF(motor), &buffer);
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			writeRegister(motor, TMC5272_CHOPCONF(motor),buffer);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC5272_TPWMTHRS(motor), &buffer);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC5272_TPWMTHRS(motor), *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_PWM_GRAD_FIELD(motor));
		} else if(readWrite == WRITE) {
			// Set gradient
			tmc5272_fieldWrite(motor, TMC5272_PWMCONF_PWM_GRAD_FIELD(motor), *value);
			// Enable/disable stealthChop accordingly
			if(motor == 0)
				tmc5272_fieldWrite(motor, TMC5272_GCONF_M0_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);
			else if(motor == 1)
				tmc5272_fieldWrite(motor, TMC5272_GCONF_M1_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_PWM_OFS_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_PWMCONF_PWM_OFS_FIELD(motor), *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_PWM_FREQ_FIELD(motor));
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				tmc5272_fieldWrite(motor, TMC5272_PWMCONF_PWM_FREQ_FIELD(motor), *value);
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
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_PWM_AUTOSCALE_FIELD(motor));
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
			{
				tmc5272_fieldWrite(motor, TMC5272_PWMCONF_PWM_AUTOSCALE_FIELD(motor), *value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 193:
		// PWM scale sum
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWM_SCALE_PWM_SCALE_SUM_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 194:
		// MSCNT
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_MSCNT_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// MEAS_SD_EN
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_PWM_MEAS_SD_ENABLE_FIELD(motor));
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
				tmc5272_fieldWrite(motor, TMC5272_PWMCONF_PWM_MEAS_SD_ENABLE_FIELD(motor), *value);
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 196:
		// DIS_REG_STST
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_PWM_DIS_REG_STST_FIELD(motor));
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
				tmc5272_fieldWrite(motor, TMC5272_PWMCONF_PWM_DIS_REG_STST_FIELD(motor), *value);
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_PWMCONF_FREEWHEEL_FIELD(motor));
		} else if(readWrite == WRITE) {
			tmc5272_fieldWrite(motor, TMC5272_PWMCONF_FREEWHEEL_FIELD(motor), *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_DRV_STATUS_SG_RESULT_FIELD(motor));
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			readRegister(motor, TMC5272_X_ENC(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_X_ENC(motor), *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			readRegister(motor, TMC5272_ENC_CONST(motor), value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC5272_ENC_CONST(motor), *value);
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
		// FSR range from DRV_CONF reg
		if(readWrite == READ) {
			if(motor ==  0)
				*value = tmc5272_fieldRead(motor, TMC5272_DRV_CONF_FSR_M0_FIELD);
			else
				*value = tmc5272_fieldRead(motor, TMC5272_DRV_CONF_FSR_M1_FIELD);
		} else if(readWrite == WRITE) {
			if(motor ==  0)
				tmc5272_fieldWrite(motor, TMC5272_DRV_CONF_FSR_M0_FIELD, *value);
			else
			    tmc5272_fieldWrite(motor, TMC5272_DRV_CONF_FSR_M1_FIELD, *value);
		}
		break;
	case 213:
		// ADCTemperatur
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_IOIN_ADC_TEMPERATURE_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// ADCTemperatur Converted
		if(readWrite == READ) {
			int32_t adc = tmc5272_fieldRead(motor, TMC5272_IOIN_ADC_TEMPERATURE_FIELD);
			*value = (int32_t)((2.03*adc)-259);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// Scales the reference current IREF of Axis M0: 0x0: 25 % IREF 0x1: 50 % IREF 0x2: 75 % IREF 0x3: 100% IREF Use this together with FSR_M0 for fine current scaling.
		if(readWrite == READ) {
			if(motor ==  0)
				*value = tmc5272_fieldRead(motor, TMC5272_DRV_CONF_FSR_IREF_M0_FIELD);
			else
				*value = tmc5272_fieldRead(motor, TMC5272_DRV_CONF_FSR_IREF_M1_FIELD);
		} else if(readWrite == WRITE) {
			if(motor ==  0)
				tmc5272_fieldWrite(motor, TMC5272_DRV_CONF_FSR_IREF_M0_FIELD, *value);
			else
				tmc5272_fieldWrite(motor, TMC5272_DRV_CONF_FSR_IREF_M1_FIELD, *value);
		}
		break;
	case 216:
		if(readWrite == READ) {
			*value = HAL.IOs->config->isHigh(Pins.nSLEEP);
		} else if(readWrite == WRITE) {
			if(*value == 1)
			{
				HAL.IOs->config->toOutput(Pins.nSLEEP);
				HAL.IOs->config->setHigh(Pins.nSLEEP);
				noRegResetnSLEEP = true;
				nSLEEPTick = systick_getTick();
			}
			else if(*value == 0)
			{
				HAL.IOs->config->toOutput(Pins.nSLEEP);
				HAL.IOs->config->setLow(Pins.nSLEEP);
			}

		}
		break;
	case 220:
		// MSLUT0
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x00);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x10);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x00);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x10);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;
	case 221:
		// MSLUT1
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x01);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x11);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x01);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x11);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;

	case 222:
		// MSLUT2
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x02);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x12);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x02);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x12);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;

	case 223:
		// MSLUT3
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x03);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x13);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x03);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x13);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;

	case 224:
		// MSLUT4
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x04);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x14);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x04);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x14);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;

	case 225:
		// MSLUT5
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x05);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x15);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x05);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x15);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;
	case 226:
		// MSLUT6
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x06);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x16);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x06);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x16);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;
	case 227:
		// MSLUT7
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x07);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x17);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x07);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x17);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;
	case 228:
		// MSLUT_START
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x08);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x18);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x08);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x18);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;
	case 229:
		// MSLUT_SEL
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x09);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x19);
				readRegister(motor, TMC5272_MSLUT_SEL_START, value);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x09);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x19);
				writeRegister(motor, TMC5272_MSLUT_SEL_START, *value);
			}
		}
		break;
	case 230:
		// START_SIN90
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x08);
                *value = tmc5272_fieldRead(motor, TMC5272_MSLUT_START_START_SIN90_FIELD);

			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x18);
				*value = tmc5272_fieldRead(motor, TMC5272_MSLUT_START_START_SIN90_FIELD);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x08);
				tmc5272_fieldWrite(motor, TMC5272_MSLUT_START_START_SIN90_FIELD, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x18);
				tmc5272_fieldWrite(motor, TMC5272_MSLUT_START_START_SIN90_FIELD, *value);
			}
		}
		break;
	case 231:
		// OFFSET_SIN90
		if(readWrite == READ) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x08);
				*value = tmc5272_fieldRead(motor, TMC5272_MSLUT_START_OFFSET_SIN90_FIELD);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x18);
				*value = tmc5272_fieldRead(motor, TMC5272_MSLUT_START_OFFSET_SIN90_FIELD);
			}
		} else if(readWrite == WRITE) {
			if(motor ==  0){
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x08);
				tmc5272_fieldWrite(motor, TMC5272_MSLUT_START_OFFSET_SIN90_FIELD, *value);
			}
			else{
				writeRegister(motor, TMC5272_MSLUT_ADDR, 0x18);
				tmc5272_fieldWrite(motor, TMC5272_MSLUT_START_OFFSET_SIN90_FIELD, *value);
			}
		}
		break;
	case 232:
		// SG4_IND_0
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_IND_SG4_IND_0_FIELD(motor));
		}
		else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 233:
		// SG4_IND_1
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_IND_SG4_IND_1_FIELD(motor));
		}
		else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 234:
		// SG4_IND_2
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_IND_SG4_IND_2_FIELD(motor));
		}
		else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 235:
		// SG4_IND_3
		if(readWrite == READ) {
			*value = tmc5272_fieldRead(motor, TMC5272_SG4_IND_SG4_IND_3_FIELD(motor));
		}
		else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 255:
		// DEBUG: SPI-FREQ
		if(readWrite == READ) {
			*value = spi_getFrequency(TMC5272_SPIChannel);
		} else if(readWrite == WRITE) {
			spi_setFrequency(TMC5272_SPIChannel, *value);
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
	if(motor >= TMC5272_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5272.velocity[motor];

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
	UNUSED(motor);

	// Encoder deviation warning!
	if (address == TMC5272_ENC_STATUS(0) || address == TMC5272_ENC_STATUS(1)) {
		if (value == 2)
		{
			// Overwrite value to clear encoder N event flag.
			value = 3;
		}
	}

	tmc5272_writeRegister(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
	UNUSED(motor);

	*value = tmc5272_readRegister(DEFAULT_MOTOR, address);
}

static void periodicJob(uint32_t tick)
{
	if(!noRegResetnSLEEP)
	{
		//check if reset after nSLEEP to HIGH was performed
		for(uint8_t motor = 0; motor < TMC5272_MOTORS; motor++)
		{
			tmc5272_periodicJob(&TMC5272, tick);
		}
	}
	else
	{
		//check if minimum time since chip activation passed. Then restore.
		if((systick_getTick()-nSLEEPTick)>20) //
		{
			noRegResetnSLEEP = false;
			enableDriver(DRIVER_ENABLE);
			field_write(DEFAULT_MOTOR, TMC5272_CHOPCONF_TOFF_FIELD(0), 3);
			field_write(DEFAULT_MOTOR, TMC5272_CHOPCONF_TOFF_FIELD(1), 3);
			field_write(DEFAULT_MOTOR, TMC5272_IHOLD_IRUN_IHOLD_FIELD(1), 3);
		}
	}
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
		 * The the TMC5272 ref switch input is pulled high by external resistor an can be pulled low either by
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

		case 8: // Enable UART mode
			if(*value == 1)
				activeBus = IC_BUS_UART;
			else if(*value == 0)
				activeBus = IC_BUS_SPI;
			init_comm(activeBus);
			break;

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
		case 253:
			readRegister(motor, TMC5272_XACTUAL(motor), value);
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
	HAL.IOs->config->setLow(Pins.UART_MODE);
	HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->reset(Pins.ENCN_DCO);
	HAL.IOs->config->reset(Pins.REFL_UC);
	HAL.IOs->config->reset(Pins.REFR_UC);
	HAL.IOs->config->reset(Pins.SWN_DIAG0);
	HAL.IOs->config->reset(Pins.SWP_DIAG1);
	HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->reset(Pins.UART_MODE);
	HAL.IOs->config->reset(Pins.nSLEEP);
	HAL.IOs->config->reset(Pins.IREF_R2);
	HAL.IOs->config->reset(Pins.IREF_R3);

};

static uint8_t reset()
{
	HAL.IOs->config->toOutput(Pins.nSLEEP);
	HAL.IOs->config->setHigh(Pins.nSLEEP);
	wait(50);
	HAL.IOs->config->setLow(Pins.nSLEEP);
	noRegResetnSLEEP = true;
	nSLEEPTick = systick_getTick();
	int32_t value = 0;

	for(uint8_t motor = 0; motor < TMC5272_MOTORS; motor++){
		readRegister(motor, TMC5272_VACTUAL(motor), &value);
		if(value != 0)
			return 0;
	}


	return tmc5272_reset(&TMC5272);
}

static uint8_t restore()
{
	return reset();
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE){
		HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
		tmc5272_fieldWrite(&TMC5272, TMC5272_GCONF_M0_DRV_ENN_FIELD, 1);
		tmc5272_fieldWrite(&TMC5272, TMC5272_GCONF_M1_DRV_ENN_FIELD, 1);
	}
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE)){
		HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
		tmc5272_fieldWrite(&TMC5272, TMC5272_GCONF_M0_DRV_ENN_FIELD, 0);
		tmc5272_fieldWrite(&TMC5272, TMC5272_GCONF_M1_DRV_ENN_FIELD, 0);
	}
}

static void init_comm(TMC5272BusType mode)
{
	TMC5272_UARTChannel = HAL.UART;
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

		HAL.IOs->config->setHigh(Pins.UART_MODE);
		TMC5272_UARTChannel = HAL.UART;
		TMC5272_UARTChannel->pinout = UART_PINS_2;
		TMC5272_UARTChannel->rxtx.init();
		break;
	case IC_BUS_SPI:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC5272_UARTChannel->rxtx.deInit();
		TMC5272_SPIChannel = &HAL.SPI->ch1;
		TMC5272_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
	default:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC5272_UARTChannel->rxtx.deInit();
		TMC5272_SPIChannel = &HAL.SPI->ch1;
		TMC5272_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		activeBus = IC_BUS_SPI;
		break;
	}
}

void TMC5272_init(void)
{
	Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0; //Pin8
	Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1; //Pin9
	Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2; //Pin10
	Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3; //Pin11
	Pins.REFL_UC         = &HAL.IOs->pins->DIO6; //Pin17
	Pins.REFR_UC         = &HAL.IOs->pins->DIO7; //Pin18
	Pins.nSLEEP          = &HAL.IOs->pins->DIO8; //Pin19
	Pins.UART_MODE       = &HAL.IOs->pins->DIO9;//Pin20
	Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15; //Pin37
	Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16; //Pin38
	Pins.IREF_R2         = &HAL.IOs->pins->DIO13; //Pin35
	Pins.IREF_R3         = &HAL.IOs->pins->DIO14; //Pin36
	Pins.SCK             = &HAL.IOs->pins->SPI1_SCK; //Pin31
	Pins.SDI             = &HAL.IOs->pins->SPI1_SDI; //Pin32
	Pins.SDO             = &HAL.IOs->pins->SPI1_SDO; //Pin33
	Pins.CS              = &HAL.IOs->pins->SPI1_CSN; //Pin33


	HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->toOutput(Pins.UART_MODE);
	HAL.IOs->config->toOutput(Pins.IREF_R2);
	HAL.IOs->config->toOutput(Pins.IREF_R3);
	HAL.IOs->config->toOutput(Pins.nSLEEP);

	HAL.IOs->config->setLow(Pins.nSLEEP);
	HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->setLow(Pins.UART_MODE);
	HAL.IOs->config->setLow(Pins.IREF_R2);
	HAL.IOs->config->setLow(Pins.IREF_R2);

	HAL.IOs->config->toInput(Pins.ENCN_DCO);
	HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);

	noRegResetnSLEEP = true;
	nSLEEPTick = systick_getTick();


	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);


	init_comm(activeBus);

	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_RESET;

	tmc5272_init(&TMC5272, 0, Evalboards.ch1.config);

	for(uint8_t motor = 0; motor < TMC5272_MOTORS; motor++)
	{
		vmax_position[motor] = 0;
	}

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
	Evalboards.ch1.numberOfMotors       = TMC5272_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);


};
