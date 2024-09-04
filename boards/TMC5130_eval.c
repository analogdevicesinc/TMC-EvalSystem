/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5130/TMC5130.h"

static TMC5130BusType activeBus = IC_BUS_SPI;	//Checkout README if you want to use UART
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5130_SPIChannel;
static UART_Config *TMC5130_UARTChannel;

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  480  // VM[V/10] max

#define DEFAULT_ICID  0

#define VREF_FULLSCALE 2714 // mV

// Typedefs
typedef struct
{
	ConfigurationTypeDef *config;
	int32_t velocity, oldX;
	uint32_t oldTick;
} TMC5130TypeDef;
static TMC5130TypeDef TMC5130;

typedef struct
{
	IOPinTypeDef  *REFL_UC;
	IOPinTypeDef  *REFR_UC;
	IOPinTypeDef  *DRV_ENN_CFG6;
	IOPinTypeDef  *ENCA_DCIN_CFG5;
	IOPinTypeDef  *ENCB_DCEN_CFG4;
	IOPinTypeDef  *ENCN_DCO;

	IOPinTypeDef  *SWSEL;
	IOPinTypeDef  *SWN_DIAG0;
	IOPinTypeDef  *SWP_DIAG1;

	IOPinTypeDef  *AIN_REF_SW;
	IOPinTypeDef  *AIN_REF_PWM;
	IOPinTypeDef  *CLK;
	IOPinTypeDef  *SDI;
	IOPinTypeDef  *SDO;
	IOPinTypeDef  *SCK;
	IOPinTypeDef  *CS;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);

static void init_comm(TMC5130BusType mode);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static uint32_t vmax_position1;
static uint16_t vref; // mV

void tmc5130_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
	UNUSED(icID);
	TMC5130_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5130_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
	UNUSED(icID);

	int32_t status = UART_readWrite(TMC5130_UARTChannel, data, writeLength, readLength);
	if(status == -1)
		return false;
	return true;
}

TMC5130BusType tmc5130_getBusType(uint16_t icID)
{
	UNUSED(icID);

	return activeBus;
}

uint8_t tmc5130_getNodeAddress(uint16_t icID)
{
	UNUSED(icID);

	return nodeAddress;
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	UNUSED(motor);
	// Set absolute velocity
	tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VMAX, abs(velocity));
	// Set direction
	tmc5130_writeRegister(DEFAULT_ICID, TMC5130_RAMPMODE, (velocity >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG);
	return 0;
}

// Rotate to the right
static uint32_t right(uint8_t motor, int32_t velocity)
{
	rotate( motor, velocity);

	return 0;
}

// Rotate to the left
static uint32_t left(uint8_t motor, int32_t velocity)
{
	rotate( motor, -velocity);

	return 0;
}

// Stop moving
static uint32_t stop(uint8_t motor)
{
	rotate(motor, 0);

	return 0;
}

// Move to a specified position with a given velocity
static uint32_t moveTo(uint8_t motor, int32_t position)
{
	UNUSED(motor);
	tmc5130_writeRegister(DEFAULT_ICID, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

	// VMAX also holds the target velocity in velocity mode.
	// Re-write the position mode maximum velocity here.
	tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VMAX, vmax_position1);

	tmc5130_writeRegister(DEFAULT_ICID, TMC5130_XTARGET, position);

	return 0;
}

// Move by a given amount with a given velocity
// This function will write the absolute target position to *ticks
static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	// determine actual position and add numbers of ticks to move
	*ticks += tmc5130_readRegister(DEFAULT_ICID, TMC5130_XACTUAL);

	moveTo(motor, *ticks);

	return 0;
}
// <= Functions forwarded to API

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC5130_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_XTARGET);
		} else if(readWrite == WRITE) {

			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_XTARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_XACTUAL);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_VMAX);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VMAX, *value);
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_VACTUAL);
			*value = CAST_Sn_TO_S32(*value, 24);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = vmax_position1;
		} else if(readWrite == WRITE) {
			vmax_position1 = abs(*value);
			if(tmc5130_readRegister(DEFAULT_ICID, TMC5130_RAMPMODE) == TMC5130_MODE_POSITION)
				tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VMAX, vmax_position1);
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_AMAX);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_AMAX, *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_IRUN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_IRUN_FIELD, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_IHOLD_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_IHOLD_FIELD, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = (tmc5130_readRegister(DEFAULT_ICID, TMC5130_RAMPSTAT) & TMC5130_RS_POSREACHED)? 1:0;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 9:
		// VREF
		if (readWrite == READ) {
			*value = vref;
		} else {
			if ((uint32_t) *value < VREF_FULLSCALE) {
				vref = *value;
				Timer.setDuty(TIMER_CHANNEL_1, ((float)vref) / VREF_FULLSCALE);
			} else {
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = (tmc5130_readRegister(DEFAULT_ICID, TMC5130_RAMPSTAT) & TMC5130_RS_STOPR)? 0:1;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = (tmc5130_readRegister(DEFAULT_ICID, TMC5130_RAMPSTAT) & TMC5130_RS_STOPL)? 0:1;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = (tmc5130_readRegister(DEFAULT_ICID, TMC5130_SWMODE) & TMC5130_SW_STOPR_ENABLE)? 1:0;
		} else if(readWrite == WRITE) {
			buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_SWMODE);
			if(*value == 0)
				tmc5130_writeRegister(DEFAULT_ICID, TMC5130_SWMODE, buffer | TMC5130_SW_STOPR_ENABLE);
			else
				tmc5130_writeRegister(DEFAULT_ICID, TMC5130_SWMODE, buffer & ~TMC5130_SW_STOPR_ENABLE);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = (tmc5130_readRegister(DEFAULT_ICID, TMC5130_SWMODE) & TMC5130_SW_STOPL_ENABLE)? 1:0;
		} else if(readWrite == WRITE) {
			buffer	= tmc5130_readRegister(DEFAULT_ICID, TMC5130_SWMODE);
			if(*value==0)
				tmc5130_writeRegister(DEFAULT_ICID, TMC5130_SWMODE, buffer | TMC5130_SW_STOPL_ENABLE);
			else
				tmc5130_writeRegister(DEFAULT_ICID, TMC5130_SWMODE, buffer & ~TMC5130_SW_STOPL_ENABLE);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_SWMODE);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_SWMODE, *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_A1);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_A1, *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_V1);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_V1, *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_DMAX);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_DMAX, *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_D1);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_D1, *value);
		}
		break;
	case 19:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_VSTART);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VSTART, *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_VSTOP);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VSTOP, *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_TZEROWAIT);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_TZEROWAIT, *value);
		}
		break;

	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_THIGH);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_THIGH, *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_VDCMIN);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_VDCMIN, *value);
		}
		break;
	case 27:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_VHIGHCHM_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_VHIGHCHM_FIELD, *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_VHIGHFS_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_VHIGHFS_FIELD, *value);
		}
		break;
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			*value = TMC5130.velocity;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 30: // par::RampType
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_RAMPMODE_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_RAMPMODE_FIELD, *value);
		}
		break;
	case 33:
		// Analog I Scale
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_I_SCALE_ANALOG_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_I_SCALE_ANALOG_FIELD, *value);
		}
		break;
	case 34:
		// Internal RSense
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_INTERNAL_RSENSE_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_INTERNAL_RSENSE_FIELD, *value);
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 256 >> tmc5130_fieldRead(DEFAULT_ICID, TMC5130_MRES_FIELD);
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
				tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_MRES_FIELD, *value);
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
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_TBL_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_TBL_FIELD, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_CHM_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_CHM_FIELD, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_DISFDCC_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_DISFDCC_FIELD, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1<<14))
			{
				*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_HEND_FIELD);
			}
			else
			{
				*value = ((buffer >> 4) & 0x07) | (buffer & (1<<11))? (1<<3):0;
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1<<14))
			{
				tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_HEND_FIELD, *value);
			}
			else
			{
				if(*value & (1<<3))
					buffer |= (0x01<<11);
				else
					buffer &= ~(0x01<<11);

				buffer &= ~(0x07<<4);
				buffer |= (*value & 0x0F) << 4;

				tmc5130_writeRegister(DEFAULT_ICID, TMC5130_CHOPCONF,buffer);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1<<14))
			{
				*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_HSTRT_FIELD);
			}
			else
			{
				*value = ((buffer >> 7) & 0x0F) | (buffer & (1<<11))? 1<<3 : 0;
			}
		} else if(readWrite == WRITE) {
			if(tmc5130_readRegister(DEFAULT_ICID, TMC5130_CHOPCONF) & (1<<14))
			{
				tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_HSTRT_FIELD, *value);
			}
			else
			{
				tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_OFFSET_FIELD, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_TOFF_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_TOFF_FIELD, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SEIMIN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SEIMIN_FIELD, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SEDN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SEDN_FIELD , *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SEMAX_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SEMAX_FIELD, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SEUP_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SEUP_FIELD, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SEMIN_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SEMIN_FIELD, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SFILT_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SFILT_FIELD, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SGT_FIELD);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SGT_FIELD, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_VSENSE_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_VSENSE_FIELD, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_CS_ACTUAL_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		//this function sort of doubles with 182 but is necessary to allow cross chip compliance
		if(readWrite == READ)
		{
			if(tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SG_STOP_FIELD))
			{
				buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_TCOOLTHRS);
				*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer : 1));
			}
			else
				*value = 0;
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SG_STOP_FIELD, (*value) ? 1:0);
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			buffer = MIN(0xFFFFF, (1<<24) / ((*value)? *value : 1));
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_TCOOLTHRS, buffer);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_RNDTF_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_RNDTF_FIELD, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SYNC_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_SYNC_FIELD, *value);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			buffer = tmc5130_readRegister(DEFAULT_ICID, TMC5130_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value : 1));
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_PWM_GRAD_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_PWM_GRAD_FIELD, *value);
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_EN_PWM_MODE_FIELD, (*value)? 1:0);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_PWM_AMPL_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_PWM_AMPL_FIELD, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_PWM_FREQ_FIELD);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_PWM_FREQ_FIELD, *value);
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
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_PWM_AUTOSCALE_FIELD);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
			{
				tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_PWM_AUTOSCALE_FIELD, *value);
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
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_FREEWHEEL_FIELD);
		} else if(readWrite == WRITE) {
			tmc5130_fieldWrite(DEFAULT_ICID, TMC5130_FREEWHEEL_FIELD, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = tmc5130_fieldRead(DEFAULT_ICID, TMC5130_SG_RESULT_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_XENC);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_XENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = tmc5130_readRegister(DEFAULT_ICID, TMC5130_ENC_CONST);
		} else if(readWrite == WRITE) {
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_ENC_CONST, *value);
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
	if(motor >= TMC5130_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5130.velocity;

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t icID, uint16_t address, int32_t value)
{
    tmc5130_writeRegister(icID, address, value);
}

static void readRegister(uint8_t icID, uint16_t address, int32_t *value)
{
    *value = tmc5130_readRegister(icID, address);
}

static void tmc5130_writeConfiguration()
{
	uint8_t *ptr = &TMC5130.config->configIndex;
	const int32_t *settings;

	if(TMC5130.config->state == CONFIG_RESTORE)
		{
			settings = *(tmc5130_shadowRegister + 0);
			// Find the next restorable register
        while(*ptr < TMC5130_REGISTER_COUNT)
        {
					// If the register is writable and has been written to, restore it
					if (TMC_IS_WRITABLE(tmc5130_registerAccess[*ptr]) && tmc5130_getDirtyBit(DEFAULT_ICID,*ptr))
					{
						break;
					}

					// Otherwise, check next register
				(*ptr)++;
			}
		}
	else
		{
			settings = tmc5130_sampleRegisterPreset;
			// Find the next resettable register
			while((*ptr < TMC5130_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc5130_registerAccess[*ptr]))
			{
				(*ptr)++;
			}
		}

	if(*ptr < TMC5130_REGISTER_COUNT)
		{
			tmc5130_writeRegister(DEFAULT_ICID, *ptr, settings[*ptr]);
			(*ptr)++;
		}
	else // Finished configuration
		{

			// Configuration reset completed
			// Change hardware preset registers here
			tmc5130_writeRegister(DEFAULT_ICID, TMC5130_PWMCONF, 0x000500C8);

			// Fill missing shadow registers (hardware preset registers)
			tmc5130_initCache();


			TMC5130.config->state = CONFIG_READY;
		}
}

// Call this periodically
static void periodicJob(uint32_t tick)
{
	// Helper function: Configure the next register.
	if(TMC5130.config->state != CONFIG_READY)
	{
		tmc5130_writeConfiguration();
		return;
	}

	int32_t XActual;
	uint32_t tickDiff;

	// Calculate velocity v = dx/dt
	if((tickDiff = tick - TMC5130.oldTick) >= 5)
	{
		XActual = tmc5130_readRegister(DEFAULT_ICID, TMC5130_XACTUAL);
		// ToDo CHECK 2: API Compatibility - write alternative algorithm w/o floating point? (LH)
		TMC5130.velocity = (uint32_t) ((float32_t) ((XActual - TMC5130.oldX) / (float32_t) tickDiff) * (float32_t) 1048.576);

		TMC5130.oldX     = XActual;
		TMC5130.oldTick  = tick;
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
		 * The the TMC5130 ref switch input is pulled high by external resistor an can be pulled low either by
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
	case 1:  // set analogue current duty
		/*
		 * Current will be defined by analogue *value voltage or current signal. In any case this function
		 * will generate a analogue voltage by PWM for up to 50% duty and a switch for the other 50%.
		 * The reference voltage will be AIN_REF = VCC_IO * *value/20000 with *value = {0..20000}
		 */

		if(*value <= 20000)
		{
			if(*value > 10000)
				HAL.IOs->config->setHigh(Pins.AIN_REF_SW);
			else
				HAL.IOs->config->setLow(Pins.AIN_REF_SW);

			Timer.setDuty(TIMER_CHANNEL_1, ((float)(*value % 10001)) / TIMER_MAX);
		}
		else
		{
			errors |= TMC_ERROR_VALUE;
		}
		break;
	case 2:  // Use internal clock
		/*
		 * Internal clock will be enabled by calling this function with a *value != 0 and unpower and repower the motor supply while keeping usb connected.
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
	case 3:
		// Unused
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
			buffer = HAL.IOs->config->isHigh(Pins.ENCB_DCEN_CFG4);
			break;
		}
		*value = buffer;
		break;
	case 5:  // read interrupt pin SWN_DIAG0
		*value = (HAL.IOs->config->isHigh(Pins.SWN_DIAG0))? 1 : 0;
		break;
	case 6:  // read interrupt pin SWP_DIAG1
		*value = (HAL.IOs->config->isHigh(Pins.SWP_DIAG1))? 1 : 0;
		break;
	case 7:  // enable single wire interface (SWSEL)
		if(*value == 1)
			HAL.IOs->config->setHigh(Pins.SWSEL);
		else
			HAL.IOs->config->setLow(Pins.SWSEL);
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
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static void deInit(void)
{
	HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->reset(Pins.AIN_REF_PWM);
	HAL.IOs->config->reset(Pins.AIN_REF_SW);
	HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->reset(Pins.ENCN_DCO);
	HAL.IOs->config->reset(Pins.REFL_UC);
	HAL.IOs->config->reset(Pins.REFR_UC);
	HAL.IOs->config->reset(Pins.SWN_DIAG0);
	HAL.IOs->config->reset(Pins.SWP_DIAG1);
	HAL.IOs->config->reset(Pins.SWSEL);
	HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);

	Timer.deInit();
};

// Reset the TMC5130.
static uint8_t reset()
{
	if(!tmc5130_readRegister(DEFAULT_ICID, TMC5130_VACTUAL))	{
		if(TMC5130.config->state != CONFIG_READY)
			return false;

		// Reset the dirty bits and wipe the shadow registers
		size_t i;
		for(i = 0; i < TMC5130_REGISTER_COUNT; i++)
		{
	        tmc5130_setDirtyBit(DEFAULT_ICID, i, false);
			tmc5130_shadowRegister[DEFAULT_ICID][i] = 0;
		}

		TMC5130.config->state        = CONFIG_RESET;
		TMC5130.config->configIndex  = 0;

		return true;
	}

	HAL.IOs->config->setLow(Pins.AIN_REF_SW);
	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);

	return 1;
}

// Restore the TMC5130 to the state stored in the shadow registers.
// This can be used to recover the IC configuration after a VM power loss.
static uint8_t restore()
{
	if(TMC5130.config->state != CONFIG_READY)
		return false;

	TMC5130.config->state        = CONFIG_RESTORE;
	TMC5130.config->configIndex  = 0;

	return true;
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

// Initialize the communication. Sets the pins. By setting the Pins
// correctly the microcontroller can communicate to the TMC5130 how he wants to communicate.
static void init_comm(TMC5130BusType mode)
{
	TMC5130_UARTChannel = HAL.UART;
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
		TMC5130_UARTChannel = HAL.UART;
		TMC5130_UARTChannel->rxtx.init();
		break;
	case IC_BUS_SPI:
	    HAL.IOs->config->setLow(Pins.SWSEL);
        TMC5130_UARTChannel->rxtx.deInit();

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
	    SPI.init();
        TMC5130_SPIChannel = &HAL.SPI->ch1;
        TMC5130_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
	}
}

// Initialize the microcontroller.
void TMC5130_init(void)
{
	Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0; //Pin8
	Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1; //Pin9
	Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2; //Pin10
	Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3; //Pin11
	Pins.REFL_UC         = &HAL.IOs->pins->DIO6; //Pin17
	Pins.REFR_UC         = &HAL.IOs->pins->DIO7; //Pin18
#if defined(LandungsbrueckeV3)
	Pins.AIN_REF_SW      = &HAL.IOs->pins->DIO10_PWM_WL; //Pin21
	Pins.AIN_REF_PWM     = &HAL.IOs->pins->DIO11_PWM_WH; //Pin22
#else
	Pins.AIN_REF_SW      = &HAL.IOs->pins->DIO10; //Pin21
	Pins.AIN_REF_PWM     = &HAL.IOs->pins->DIO11; //Pin22
#endif
	Pins.CS              = &HAL.IOs->pins->SPI1_CSN; //Pin30
	Pins.SCK             = &HAL.IOs->pins->SPI1_SCK; //Pin31
	Pins.SDI             = &HAL.IOs->pins->SPI1_SDI; //Pin32
	Pins.SDO             = &HAL.IOs->pins->SPI1_SDO; //Pin33
	Pins.SWSEL           = &HAL.IOs->pins->DIO14; //Pin36
	Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15; //Pin37
	Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16; //Pin38

	HAL.IOs->config->toInput(Pins.ENCN_DCO);
	HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->toOutput(Pins.SWSEL);
	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);
	HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->toOutput(Pins.AIN_REF_SW);

	init_comm(activeBus);

	TMC5130.config = Evalboards.ch1.config;
	TMC5130.velocity  = 0;
	TMC5130.oldTick   = 0;
	TMC5130.oldX      = 0;

	TMC5130.config->callback     = NULL;
	TMC5130.config->channel      = 0;
	TMC5130.config->state      = CONFIG_RESET;

	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
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
	Evalboards.ch1.numberOfMotors       = TMC5130_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	tmc5130_cache(DEFAULT_ICID, TMC5130_CACHE_READ, TMC5130_VMAX, &vmax_position1);


#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
	HAL.IOs->config->toOutput(Pins.AIN_REF_PWM);
	Pins.AIN_REF_PWM->configuration.GPIO_Mode = GPIO_Mode_AF4;
#elif defined(LandungsbrueckeV3)
	Pins.AIN_REF_PWM->configuration.GPIO_Mode = GPIO_MODE_AF;
	gpio_af_set(Pins.AIN_REF_PWM->port, GPIO_AF_1, Pins.AIN_REF_PWM->bitWeight);
#endif

	vref = 2000;
	HAL.IOs->config->set(Pins.AIN_REF_PWM);
	Timer.init();
	Timer.setDuty(TIMER_CHANNEL_1, ((float)vref) / VREF_FULLSCALE);

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
