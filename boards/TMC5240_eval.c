#include "Board.h"
#include "tmc/ic/TMC5240/TMC5240.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

#define DEFAULT_MOTOR  0

//#define TMC5240_TIMEOUT 50 // UART Timeout in ms

static bool vMaxModified = false;
static uint32_t vmax_position;
//static uint32_t vMax		   = 1;

static TMC_Board_Comm_Mode commMode = TMC_BOARD_COMM_SPI;
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
static void readRegister(uint8_t motor, uint8_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint8_t address, int32_t value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);

static int32_t tmc5240_UARTreadInt(UART_Config *channel, uint8_t address);
static void tmc5240_UARTwriteInt(UART_Config *channel, uint8_t address, int32_t value);


static void init_comm(TMC_Board_Comm_Mode mode);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static UART_Config *TMC5240_UARTChannel;
static SPIChannelTypeDef *TMC5240_SPIChannel;
static TMC5240TypeDef TMC5240;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC5240_CRC(data, length) tmc_CRC8(data, length, 1)

// When using multiple ICs you can map them here
static inline TMC5240TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &TMC5240;
}

// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc5240_CRC8(uint8_t *data, size_t length)
{
	return tmc_CRC8(data, length, 1);
	//TMC5240_CRC(data, length);
}

int32_t tmc5240_readInt(TMC5240TypeDef *tmc5240, uint8_t address){
	UNUSED(tmc5240);
	if(commMode == TMC_BOARD_COMM_SPI)
		{
		spi_readInt(TMC5240_SPIChannel, address);
		return spi_readInt(TMC5240_SPIChannel, address);
		}
	else if (commMode == TMC_BOARD_COMM_UART){
		return tmc5240_UARTreadInt(TMC5240_UARTChannel,  address);
		}
	return -1;
}

void   tmc5240_writeInt(TMC5240TypeDef *tmc5240, uint8_t address, int32_t value){
	UNUSED(tmc5240);
	if(commMode == TMC_BOARD_COMM_SPI)
	{
		spi_writeInt(TMC5240_SPIChannel,  address,  value);
	}
	else if (commMode == TMC_BOARD_COMM_UART)
	{
		tmc5240_UARTwriteInt(TMC5240_UARTChannel,  address,  value);
	}
}
void tmc5240_UARTwriteInt(UART_Config *channel, uint8_t address, int32_t value)
{
	uint8_t data[8];

	data[0] = 0x05;
	data[1] = targetAddressUart;
	data[2] = address | TMC_WRITE_BIT;
	data[3] = (value >> 24) & 0xFF;
	data[4] = (value >> 16) & 0xFF;
	data[5] = (value >> 8 ) & 0xFF;
	data[6] = (value      ) & 0xFF;
	data[7] = tmc5240_CRC8(data, 7);

	UART_readWrite(channel, &data[0], 8, 0);
}

int32_t tmc5240_UARTreadInt(UART_Config *channel, uint8_t address)
{
	uint8_t data[8] = { 0 };

	address = TMC_ADDRESS(address);

	data[0] = 0x05;
	data[1] = targetAddressUart;
	data[2] = address;
	data[3] = tmc5240_CRC8(data, 3);

	UART_readWrite(channel, data, 4, 8);

	// Byte 0: Sync nibble correct?
	if (data[0] != 0x05)
		return 0;

	// Byte 1: Master address correct?
	if (data[1] != 0xFF)
		return 0;

	// Byte 2: Address correct?
	if (data[2] != address)
		return 0;

	// Byte 7: CRC correct?
	if (data[7] != tmc5240_CRC8(data, 7))
		return 0;

	return ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | (data[5] << 8) | data[6];
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
	tmc5240_rotate(motorToIC(motor), velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	tmc5240_right(motorToIC(motor), velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	tmc5240_left(motorToIC(motor), velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	tmc5240_stop(motorToIC(motor));

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc5240_moveTo(motorToIC(motor), position, vmax_position);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc5240_moveBy(motorToIC(motor), ticks, vmax_position);

	return 0;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC5240_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_XTARGET);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_XTARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_XACTUAL);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_VMAX);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_VMAX, abs(*value));
			vMaxModified = true;
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_VACTUAL);
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
			if(tmc5240_readInt(motorToIC(motor), TMC5240_RAMPMODE) == TMC5240_MODE_POSITION)
				tmc5240_writeInt(motorToIC(motor), TMC5240_VMAX, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_AMAX);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_AMAX, *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_IHOLD_IRUN, TMC5240_IRUN_MASK, TMC5240_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_IHOLD_IRUN, TMC5240_IRUN_MASK, TMC5240_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_IHOLD_IRUN, TMC5240_IHOLD_MASK, TMC5240_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_IHOLD_IRUN, TMC5240_IHOLD_MASK, TMC5240_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_RAMPSTAT, TMC5240_POSITION_REACHED_MASK, TMC5240_POSITION_REACHED_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = !TMC5240_FIELD_READ(motorToIC(motor), TMC5240_RAMPSTAT, TMC5240_STATUS_STOP_R_MASK, TMC5240_STATUS_STOP_R_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !TMC5240_FIELD_READ(motorToIC(motor), TMC5240_RAMPSTAT, TMC5240_STATUS_STOP_L_MASK, TMC5240_STATUS_STOP_L_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_SWMODE, TMC5240_STOP_R_ENABLE_MASK, TMC5240_STOP_R_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_SWMODE, TMC5240_STOP_R_ENABLE_MASK, TMC5240_STOP_R_ENABLE_SHIFT, *value);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_SWMODE, TMC5240_STOP_L_ENABLE_MASK, TMC5240_STOP_L_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_SWMODE, TMC5240_STOP_L_ENABLE_MASK, TMC5240_STOP_L_ENABLE_SHIFT, *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_SWMODE);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_SWMODE, *value);
		}
		break;
	case 15:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_DMAX);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_DMAX, *value);
		}
		break;
	case 16:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_VSTART);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_VSTART, *value);
		}
		break;
	case 17:
		// Acceleration A1
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_A1);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_A1, *value);
		}
		break;
	case 18:
		// Velocity V1
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_V1);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_V1, *value);
		}
		break;
	case 19:
		// Deceleration D1
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_D1);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_D1, *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_VSTOP);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_VSTOP, *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_TZEROWAIT);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_TZEROWAIT, *value);
		}
		break;
	case 22:
		// Velocity V2
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_V2);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_V2, *value);
		}
		break;
	case 23:
		// Deceleration D2
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_D2);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_D2, *value);
		}
		break;
	case 24:
		// Acceleration A2
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_A2);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_A2, *value);
		}
		break;
	case 25:
		// TVMAX
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_TVMAX);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_TVMAX, *value);
		}
		break;


	case 26:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			buffer = tmc5240_readInt(motorToIC(motor), TMC5240_THIGH);
			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
			tmc5240_writeInt(motorToIC(motor), TMC5240_THIGH, *value);
		}
		break;
	case 27:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_VDCMIN);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_VDCMIN, *value);
		}
		break;
	case 28:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_VHIGHCHM_MASK, TMC5240_VHIGHCHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_VHIGHCHM_MASK, TMC5240_VHIGHCHM_SHIFT, *value);
		}
		break;
	case 29:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_VHIGHFS_MASK, TMC5240_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_VHIGHFS_MASK, TMC5240_VHIGHFS_SHIFT, *value);
		}
		break;
	case 30:
		// Measured Speed
		if(readWrite == READ) {
			*value = TMC5240.velocity;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 34:
		// Internal RSense
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_GCONF, TMC5240_REFR_DIR_MASK, TMC5240_REFR_DIR_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_GCONF, TMC5240_REFR_DIR_MASK, TMC5240_REFR_DIR_SHIFT, *value);
		}
		break;
	case 35:
		// Global current scaler
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_GLOBAL_SCALER, TMC5240_GLOBAL_SCALER_MASK, TMC5240_GLOBAL_SCALER_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value > 31)
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_GLOBAL_SCALER, TMC5240_GLOBAL_SCALER_MASK, TMC5240_GLOBAL_SCALER_SHIFT, *value);
			else
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_GLOBAL_SCALER, TMC5240_GLOBAL_SCALER_MASK, TMC5240_GLOBAL_SCALER_SHIFT, 0);
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_MRES_MASK, TMC5240_MRES_SHIFT);
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
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_MRES_MASK, TMC5240_MRES_SHIFT, *value);
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
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_TBL_MASK, TMC5240_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_TBL_MASK, TMC5240_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_CHM_MASK, TMC5240_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_CHM_MASK, TMC5240_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_DISFDCC_MASK, TMC5240_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_DISFDCC_MASK, TMC5240_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		buffer = tmc5240_readInt(motorToIC(motor), TMC5240_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5240_CHM_SHIFT))
			{
				*value = (buffer >> TMC5240_HEND_OFFSET_SHIFT) & TMC5240_HEND_OFFSET_MASK;
			}
			else
			{
				*value = (tmc5240_readInt(motorToIC(motor), TMC5240_CHOPCONF) >> TMC5240_TFD_ALL_SHIFT) & TMC5240_TFD_ALL_MASK;
				if(buffer & TMC5240_FD3_SHIFT)
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(tmc5240_readInt(motorToIC(motor), TMC5240_CHOPCONF) & (1<<14))
			{
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_HEND_OFFSET_MASK, TMC5240_HEND_OFFSET_SHIFT, *value);
			}
			else
			{
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_FD3_MASK, TMC5240_FD3_SHIFT, (*value & (1<<3))); // MSB wird zu value dazugefügt
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_TFD_ALL_MASK, TMC5240_TFD_ALL_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		buffer = tmc5240_readInt(motorToIC(motor), TMC5240_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5240_CHM_SHIFT))
			{
				*value = (buffer >> TMC5240_TFD_ALL_SHIFT) & TMC5240_TFD_ALL_MASK;
			}
			else
			{
				*value = (buffer >> TMC5240_HEND_OFFSET_SHIFT) & TMC5240_HEND_OFFSET_MASK;
				if(buffer & (1 << TMC5240_FD3_SHIFT))
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1 << TMC5240_CHM_SHIFT))
			{
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_TFD_ALL_MASK, TMC5240_TFD_ALL_SHIFT, *value);
			}
			else
			{
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_HEND_OFFSET_MASK, TMC5240_HEND_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_TOFF_MASK, TMC5240_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_CHOPCONF, TMC5240_TOFF_MASK, TMC5240_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEIMIN_MASK, TMC5240_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEIMIN_MASK, TMC5240_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEDN_MASK, TMC5240_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEDN_MASK, TMC5240_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEMAX_MASK, TMC5240_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEMAX_MASK, TMC5240_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEUP_MASK, TMC5240_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEUP_MASK, TMC5240_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEMIN_MASK, TMC5240_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SEMIN_MASK, TMC5240_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard4 filter enable
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SFILT_MASK, TMC5240_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SFILT_MASK, TMC5240_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard4 threshold
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SGT_MASK, TMC5240_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_COOLCONF, TMC5240_SGT_MASK, TMC5240_SGT_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_DRVSTATUS, TMC5240_CS_ACTUAL_MASK, TMC5240_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		//this function sort of doubles with 182 but is necessary to allow cross chip compliance
		if(readWrite == READ) {
			if(TMC5240_FIELD_READ(motorToIC(motor), TMC5240_SWMODE, TMC5240_SG_STOP_MASK, TMC5240_SG_STOP_SHIFT))
			{
				buffer = tmc5240_readInt(motorToIC(motor), TMC5240_TCOOLTHRS);
				*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
			}
			else
			{
				*value = 0;
			}
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_SWMODE, TMC5240_SG_STOP_MASK, TMC5240_SG_STOP_SHIFT, (*value)? 1:0);

			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5240_writeInt(motorToIC(motor), TMC5240_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			buffer = tmc5240_readInt(motorToIC(motor), TMC5240_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5240_writeInt(motorToIC(motor), TMC5240_TCOOLTHRS, *value);
		}
		break;
	case 183:
			// SG_FILT_EN
			if(readWrite == READ) {
				*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_SG4_THRS, TMC5240_SG4_FILT_EN_MASK, TMC5240_SG4_FILT_EN_SHIFT);
			} else if(readWrite == WRITE) {
				if(*value >= 0 && *value < 2){
					TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_SG4_THRS, TMC5240_SG4_FILT_EN_MASK, TMC5240_SG4_FILT_EN_SHIFT, *value);
				}
				else
				{
					errors |= TMC_ERROR_VALUE;
				}
			}
			break;
	case 184:
			// SG_ANGLE_OFFSET
			if(readWrite == READ) {
				*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_SG4_THRS, TMC5240_SG_ANGLE_OFFSET_MASK, TMC5240_SG_ANGLE_OFFSET_SHIFT);
			} else if(readWrite == WRITE) {
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_SG4_THRS, TMC5240_SG_ANGLE_OFFSET_MASK, TMC5240_SG_ANGLE_OFFSET_SHIFT, *value);
			}
			break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = (tmc5240_readInt(motorToIC(motor), TMC5240_CHOPCONF) >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			buffer = tmc5240_readInt(motorToIC(motor), TMC5240_CHOPCONF);
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			tmc5240_writeInt(motorToIC(motor), TMC5240_CHOPCONF,buffer);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			buffer = tmc5240_readInt(motorToIC(motor), TMC5240_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5240_writeInt(motorToIC(motor), TMC5240_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_GRAD_MASK, TMC5240_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_GRAD_MASK, TMC5240_PWM_GRAD_SHIFT, *value);
			// Enable/disable stealthChop accordingly
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_GCONF, TMC5240_EN_PWM_MODE_MASK, TMC5240_EN_PWM_MODE_SHIFT, (*value) ? 1 : 0);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_OFS_MASK, TMC5240_PWM_OFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_OFS_MASK, TMC5240_PWM_OFS_SHIFT, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_FREQ_MASK, TMC5240_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_FREQ_MASK, TMC5240_PWM_FREQ_SHIFT, *value);
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
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_AUTOSCALE_MASK, TMC5240_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
			{
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_AUTOSCALE_MASK, TMC5240_PWM_AUTOSCALE_SHIFT, *value);
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
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMSCALE, TMC5240_PWM_SCALE_SUM_MASK, TMC5240_PWM_SCALE_SUM_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 194:
		// MSCNT
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_MSCNT, TMC5240_MSCNT_MASK, TMC5240_MSCNT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// MEAS_SD_EN
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_MEAS_SD_ENABLE_MASK, TMC5240_PWM_MEAS_SD_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_MEAS_SD_ENABLE_MASK, TMC5240_PWM_MEAS_SD_ENABLE_SHIFT, *value);
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 196:
		// DIS_REG_STST
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_DIS_REG_STST_MASK, TMC5240_PWM_DIS_REG_STST_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
				TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_PWM_DIS_REG_STST_MASK, TMC5240_PWM_DIS_REG_STST_SHIFT, *value);
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_PWMCONF, TMC5240_FREEWHEEL_MASK, TMC5240_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_PWMCONF, TMC5240_FREEWHEEL_MASK, TMC5240_FREEWHEEL_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_DRVSTATUS, TMC5240_SG_RESULT_MASK, TMC5240_SG_RESULT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_XENC);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_XENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = tmc5240_readInt(motorToIC(motor), TMC5240_ENC_CONST);
		} else if(readWrite == WRITE) {
			tmc5240_writeInt(motorToIC(motor), TMC5240_ENC_CONST, *value);
		}
		break;
	case 211:
		//ADC Scaling Resitors
		if(readWrite == READ) {
			int val2 = (HAL.IOs->config->isHigh(Pins.IREF_R2));
			int val3 = (HAL.IOs->config->isHigh(Pins.IREF_R3));
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
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_DRV_CONF, TMC5240_CURRENT_RANGE_MASK, TMC5240_CURRENT_RANGE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_DRV_CONF, TMC5240_CURRENT_RANGE_MASK, TMC5240_CURRENT_RANGE_SHIFT, *value);
		}
		break;

	case 213:
		// ADCTemperatur
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_ADC_TEMP, TMC5240_ADC_TEMP_MASK, TMC5240_ADC_TEMP_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// ADCIN
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_ADC_VSUPPLY_AIN, TMC5240_ADC_AIN_MASK, TMC5240_ADC_AIN_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// ADCSupply
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_ADC_VSUPPLY_AIN, TMC5240_ADC_VSUPPLY_MASK, TMC5240_ADC_VSUPPLY_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 216:
		// Overvoltage Limit ADC value
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERVOLTAGE_VTH_MASK, TMC5240_OVERVOLTAGE_VTH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERVOLTAGE_VTH_MASK, TMC5240_OVERVOLTAGE_VTH_SHIFT, *value);
		}
		break;
	case 217:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			*value = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERTEMPPREWARNING_VTH_MASK, TMC5240_OVERTEMPPREWARNING_VTH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERTEMPPREWARNING_VTH_MASK, TMC5240_OVERTEMPPREWARNING_VTH_SHIFT, *value);
		}
		break;
	case 218:
		// ADCTemperatur Converted
		if(readWrite == READ) {

			int adc = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_ADC_TEMP, TMC5240_ADC_TEMP_MASK, TMC5240_ADC_TEMP_SHIFT);
			*value = (int)10*(adc-2038)/77;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 219:
		// ADCIN converted
		if(readWrite == READ) {
			int adc = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_ADC_VSUPPLY_AIN, TMC5240_ADC_AIN_MASK, TMC5240_ADC_AIN_SHIFT);
			*value = (int)3052*adc/10000;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 220:
		// ADCSupply
		if(readWrite == READ) {
			int adc = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_ADC_VSUPPLY_AIN, TMC5240_ADC_VSUPPLY_MASK, TMC5240_ADC_VSUPPLY_SHIFT);
			*value = (int)32*3052*adc/10000;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 221:
		// Overvoltage Limit converted
		if(readWrite == READ) {
			int val = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERVOLTAGE_VTH_MASK, TMC5240_OVERVOLTAGE_VTH_SHIFT);
			*value = (int)32*3052*val/10000;
		} else if(readWrite == WRITE) {
			int val = (int)(*value*10000/(3052*32));
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERVOLTAGE_VTH_MASK, TMC5240_OVERVOLTAGE_VTH_SHIFT, val);
		}
		break;
	case 222:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			int temp = TMC5240_FIELD_READ(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERTEMPPREWARNING_VTH_MASK, TMC5240_OVERTEMPPREWARNING_VTH_SHIFT);
			*value = (int)(temp-2038)/7.7;
		} else if(readWrite == WRITE) {
			float valf  = *value*7.7;
			int val = (int)valf;
			val = val+2038;
			TMC5240_FIELD_WRITE(motorToIC(motor), TMC5240_OTW_OV_VTH, TMC5240_OVERTEMPPREWARNING_VTH_MASK, TMC5240_OVERTEMPPREWARNING_VTH_SHIFT, val);
		}
		break;

	case 223:
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
//	case 224: // Enable UART mode
//		if (readWrite == WRITE) {
//			if(*value == 1)
//				commMode = TMC_BOARD_COMM_UART;
//			else if(*value == 0)
//				commMode = TMC_BOARD_COMM_SPI;
//			init_comm(commMode);
//		}
//		else if(readWrite == READ) {
//			if(commMode == TMC_BOARD_COMM_UART)
//				*value = 1;
//			else if (commMode == TMC_BOARD_COMM_SPI)
//				*value = 0;
//				}
//			break;
//	case 225: // UART slave address. Remove later
//		if (readWrite == READ) {
//			*value = targetAddressUart;
//		}
//		else if(readWrite == WRITE) {
//			targetAddressUart = *value;
//		}
//		break;

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
	if(motor >= TMC5240_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5240.velocity;

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	tmc5240_writeInt(motorToIC(motor), address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc5240_readInt(motorToIC(motor), address);
}

static void periodicJob(uint32_t tick)
{
	//check if reset after nSLEEP to HIGH was performed
	if(!noRegResetnSLEEP)
	{
		for(int motor = 0; motor < TMC5240_MOTORS; motor++)
			{
				tmc5240_periodicJob(&TMC5240, tick);
			}
	}
	else
	{
		//check if minimum time since chip activation passed. Then restore.
		if((systick_getTick()-nSLEEPTick)>20) //
		{
			tmc5240_restore(&TMC5240);
			noRegResetnSLEEP = false;
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
		 * The the TMC5240 ref switch input is pulled high by external resistor an can be pulled low either by
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
//	case 7:  // enable single wire interface (SWSEL)
//			if(*value == 1) HAL.IOs->config->setHigh(Pins.SWSEL);
//			else HAL.IOs->config->setLow(Pins.SWSEL);
//		break;
	case 8: // Enable UART mode
		if(*value == 1)
			commMode = TMC_BOARD_COMM_UART;
		else if(*value == 0)
			commMode = TMC_BOARD_COMM_SPI;
		init_comm(commMode);
		break;
		/*
	case 9: // Set UART address
		tmc5240_setSlaveAddress()
		break;
*/
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
	HAL.IOs->config->setLow(Pins.UART_MODE);
	//HAL.IOs->config->setLow(Pins.SPI_MODE);
	HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->reset(Pins.ENCN_DCO);
	HAL.IOs->config->reset(Pins.REFL_UC);
	HAL.IOs->config->reset(Pins.REFR_UC);
	HAL.IOs->config->reset(Pins.SWN_DIAG0);
	HAL.IOs->config->reset(Pins.SWP_DIAG1);
	HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->reset(Pins.UART_MODE);
	//HAL.IOs->config->reset(Pins.SPI_MODE);
	HAL.IOs->config->reset(Pins.nSLEEP);
	HAL.IOs->config->reset(Pins.IREF_R2);
	HAL.IOs->config->reset(Pins.IREF_R3);

};

static uint8_t reset()
{
	if(!tmc5240_readInt(&TMC5240, TMC5240_VACTUAL))
		tmc5240_reset(&TMC5240);

	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);

	return 1;
}

static uint8_t restore()
{
	return tmc5240_restore(&TMC5240);
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

static void init_comm(TMC_Board_Comm_Mode mode)
{
	TMC5240_UARTChannel = HAL.UART;
	switch(mode) {
	case TMC_BOARD_COMM_UART:

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
		TMC5240_UARTChannel->pinout = UART_PINS_2;
		TMC5240_UARTChannel->rxtx.init();
		break;
	case TMC_BOARD_COMM_SPI:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC5240_UARTChannel->rxtx.deInit();
		TMC5240_SPIChannel = &HAL.SPI->ch1;
		TMC5240_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
	case TMC_BOARD_COMM_WLAN: // unused
	default:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC5240_UARTChannel->rxtx.deInit();
		TMC5240_SPIChannel = &HAL.SPI->ch1;
		TMC5240_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		commMode = TMC_BOARD_COMM_SPI;
		break;
	}
}

void TMC5240_init(void)
{
	tmc_fillCRC8Table(0x07, true, 1);

	Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0; //Pin8
	Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1; //Pin9
	Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2; //Pin10
	Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3; //Pin11
	Pins.REFL_UC         = &HAL.IOs->pins->DIO6; //Pin17
	Pins.REFR_UC         = &HAL.IOs->pins->DIO7; //Pin18
	Pins.UART_MODE       = &HAL.IOs->pins->DIO9;//Pin20
	Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15; //Pin37
	Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16; //Pin38
	Pins.nSLEEP          = &HAL.IOs->pins->DIO8; //Pin19
	Pins.IREF_R2         = &HAL.IOs->pins->DIO13; //Pin35
	Pins.IREF_R3         = &HAL.IOs->pins->DIO14; //Pin36
	Pins.SCK             = &HAL.IOs->pins->SPI1_SCK; //Pin31
	Pins.SDI             = &HAL.IOs->pins->SPI1_SDI; //Pin32
	Pins.SDO             = &HAL.IOs->pins->SPI1_SDO; //Pin33
	Pins.CS              = &HAL.IOs->pins->SPI1_CSN; //Pin33

	
	HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->toOutput(Pins.UART_MODE);
	HAL.IOs->config->toOutput(Pins.nSLEEP);
	noRegResetnSLEEP = true;
	nSLEEPTick = systick_getTick();
	HAL.IOs->config->toOutput(Pins.IREF_R2);
	HAL.IOs->config->toOutput(Pins.IREF_R3);

	HAL.IOs->config->setHigh(Pins.nSLEEP);
	HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->setLow(Pins.UART_MODE);
	HAL.IOs->config->setLow(Pins.IREF_R2);
	HAL.IOs->config->setLow(Pins.IREF_R2);

	HAL.IOs->config->toInput(Pins.ENCN_DCO);
	HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);
	//HAL.IOs->config->toInput(Pins.SWN_DIAG0);
	//HAL.IOs->config->toInput(Pins.SWP_DIAG1);
	//HAL.IOs->config->toOutput(Pins.SWP_DIAG1);
	//HAL.IOs->config->setToState(Pins.SWP_DIAG1,IOS_OPEN);
	//Pins.SWP_DIAG1->configuration.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	//setPinConfiguration(Pins.SWP_DIAG1);




	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);

	// Disable CLK output -> use internal 12 MHz clock
	//  Switchable via user function



	init_comm(commMode);
	//init_comm(TMC_BOARD_COMM_UART);

	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_RESET;

	tmc5240_init(&TMC5240, 0, Evalboards.ch1.config, tmc5240_defaultRegisterResetState);

	vmax_position = 0;

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
	Evalboards.ch1.numberOfMotors       = TMC5240_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);


};
