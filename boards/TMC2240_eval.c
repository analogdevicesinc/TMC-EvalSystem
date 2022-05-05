#include "tmc/StepDir.h"
#include "Board.h"
#include "tmc/ic/TMC2240/TMC2240.h"

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

#undef  TMC2240_MAX_VELOCITY
#define TMC2240_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define TMC2240_MOTORS 1

static TMC_Board_Comm_Mode commMode = TMC_BOARD_COMM_SPI;
static uint32_t targetAddressUart = 0;
static bool noRegResetnSLEEP = false;
static uint32_t nSLEEPTick;

static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type,
		int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor,
		int32_t *value);
static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value);
static void readRegister(uint8_t motor, uint8_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint8_t address, int32_t value);

static int32_t tmc2240_UARTreadInt(UART_Config *channel, uint8_t address);
static void tmc2240_UARTwriteInt(UART_Config *channel, uint8_t address, int32_t value);


//static void write_uart(uint8_t motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
//static int32_t read_uart(uint8_t motor, uint8_t address);
void	tmc2240_writeInt(TMC2240TypeDef *tmc2240, uint8_t address, int32_t value);
int32_t tmc2240_readInt(TMC2240TypeDef *tmc2240, uint8_t address);
static void checkErrors(uint32_t tick);

static void init_comm(TMC_Board_Comm_Mode mode);

static void periodicJob(uint32_t tick);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);
static void deInit(void);
static uint8_t reset();
static uint8_t restore();
static void configCallback(TMC2240TypeDef *tmc2240, ConfigState state);
static void enableDriver(DriverState state);

static UART_Config *TMC2240_UARTChannel;
static SPIChannelTypeDef *TMC2240_SPIChannel;
static TMC2240TypeDef TMC2240;
#define TMC2240_TIMEOUT 50 // UART Timeout in ms
//static int32_t measured_velocity = 0;

typedef struct {
	IOPinTypeDef *STEP;
	IOPinTypeDef *DIR;
	IOPinTypeDef *DRV_ENN_CFG6;
	IOPinTypeDef *DIAG0;
	IOPinTypeDef *DIAG1;
	//IOPinTypeDef *AIN_REF_SW;
	//IOPinTypeDef *AIN_REF_PWM;
	IOPinTypeDef  *UART_MODE;
	IOPinTypeDef  *nSLEEP;
	IOPinTypeDef  *IREF_R2;
	IOPinTypeDef  *IREF_R3;
	IOPinTypeDef  *SDI;
	IOPinTypeDef  *SDO;
	IOPinTypeDef  *SCK;
	IOPinTypeDef  *CS;
} PinsTypeDef;

static PinsTypeDef Pins;




// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC2240_CRC(data, length) tmc_CRC8(data, length, 1)

// When using multiple ICs you can map them here
static inline TMC2240TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &TMC2240;
}

// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc2240_CRC8(uint8_t *data, size_t length)
{
	return tmc_CRC8(data, length, 1);
	//TMC2240_CRC(data, length);
}

int32_t tmc2240_readInt(TMC2240TypeDef *tmc2240, uint8_t address){
	UNUSED(tmc2240);
	if(commMode == TMC_BOARD_COMM_SPI)
		{
		spi_readInt(TMC2240_SPIChannel, address);
		return spi_readInt(TMC2240_SPIChannel, address);
		}
	else if (commMode == TMC_BOARD_COMM_UART){
		//int32_t *returnvalue = 0;
		//UART_readInt(TMC2240_UARTChannel,0x00, address, &returnvalue);
		//returnvalue = 0;
		//UART_readInt(TMC2240_UARTChannel,targetAddressUart, address, &returnvalue);
		//return returnvalue;
		return tmc2240_UARTreadInt(TMC2240_UARTChannel,  address);
		}
	return -1;
}

void tmc2240_writeInt(TMC2240TypeDef *tmc2240, uint8_t address, int32_t value){
	UNUSED(tmc2240);
	if(commMode == TMC_BOARD_COMM_SPI)
	{
		spi_writeInt(TMC2240_SPIChannel,  address,  value);
	}
	else if (commMode == TMC_BOARD_COMM_UART)
	{
		//UART_writeInt(TMC2240_UARTChannel,  targetAddressUart,  address,  value);
		tmc2240_UARTwriteInt(TMC2240_UARTChannel,  address,  value);
	}
}
void tmc2240_UARTwriteInt(UART_Config *channel, uint8_t address, int32_t value)
{
	uint8_t data[8];

	data[0] = 0x05;
	data[1] = targetAddressUart;
	data[2] = address | TMC_WRITE_BIT;
	data[3] = (value >> 24) & 0xFF;
	data[4] = (value >> 16) & 0xFF;
	data[5] = (value >> 8 ) & 0xFF;
	data[6] = (value      ) & 0xFF;
	data[7] = tmc2240_CRC8(data, 7);

	UART_readWrite(channel, &data[0], 8, 0);
}

int32_t tmc2240_UARTreadInt(UART_Config *channel, uint8_t address)
{
	uint8_t data[8] = { 0 };

	address = TMC_ADDRESS(address);

	data[0] = 0x05;
	data[1] = targetAddressUart;
	data[2] = address;
	data[3] = tmc2240_CRC8(data, 3);

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
	if (data[7] != tmc2240_CRC8(data, 7))
		return 0;

	return ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | (data[5] << 8) | data[6];
}


static uint32_t rotate(uint8_t motor, int32_t velocity) {
	if (motor >= TMC2240_MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_rotate(motor, velocity);

	return TMC_ERROR_NONE;
}

static uint32_t right(uint8_t motor, int32_t velocity) {
	return rotate(motor, velocity);
}

static uint32_t left(uint8_t motor, int32_t velocity) {
	return rotate(motor, -velocity);
}

static uint32_t stop(uint8_t motor) {
	return rotate(motor, 0);
}

static uint32_t moveTo(uint8_t motor, int32_t position) {
	if (motor >= TMC2240_MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks) {
	if (motor >= TMC2240_MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value) {
	uint32_t errors = TMC_ERROR_NONE;
	uint32_t buffer;


	if (motor >= TMC2240_MOTORS)
		return TMC_ERROR_MOTOR;

	int32_t tempValue;

	switch (type) {
	case 0:
		// Target position
		if (readWrite == READ) {
			*value = StepDir_getTargetPosition(motor);
		} else if (readWrite == WRITE) {
			StepDir_moveTo(motor, *value);
		}
		break;
	case 1:
		// Actual position
		if (readWrite == READ) {
			*value = StepDir_getActualPosition(motor);
		} else if (readWrite == WRITE) {
			StepDir_setActualPosition(motor, *value);
		}
		break;
	case 2:
		// Target speed
		if (readWrite == READ) {
			*value = StepDir_getTargetVelocity(motor);
		} else if (readWrite == WRITE) {
			StepDir_rotate(motor, *value);
		}
		break;
	case 3:
		// Actual speed
		if (readWrite == READ) {
			switch (StepDir_getMode(motor)) {
			case STEPDIR_INTERNAL:
				*value = StepDir_getActualVelocity(motor);
				break;
			case STEPDIR_EXTERNAL:
			default:
				tempValue =
						(int32_t)(
								((int64_t) StepDir_getFrequency(motor)
										* (int64_t) 122)
										/ (int64_t)TMC2240_FIELD_READ(motorToIC(motor), TMC2240_TSTEP, TMC2240_TSTEP_MASK, TMC2240_TSTEP_SHIFT));
				*value = (abs(tempValue) < 20) ? 0 : tempValue;
				break;
			}
		} else if (readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if (readWrite == READ) {
			*value = StepDir_getVelocityMax(motor);
		} else if (readWrite == WRITE) {
			StepDir_setVelocityMax(motor, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if (readWrite == READ) {
			*value = StepDir_getAcceleration(motor);
		} else if (readWrite == WRITE) {
			StepDir_setAcceleration(motor, *value);
		}
		break;
	case 6:
		// Maximum current
		if (readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_IHOLD_IRUN,
					TMC2240_IRUN_MASK, TMC2240_IRUN_SHIFT);
		} else if (readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_IHOLD_IRUN,
					TMC2240_IRUN_MASK, TMC2240_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if (readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_IHOLD_IRUN,
					TMC2240_IHOLD_MASK, TMC2240_IHOLD_SHIFT);
		} else if (readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_IHOLD_IRUN,
					TMC2240_IHOLD_MASK, TMC2240_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if (readWrite == READ) {
			*value = (StepDir_getStatus(motor) & STATUS_TARGET_REACHED) ? 1 : 0;
		} else if (readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;



	case 26:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			buffer = tmc2240_readInt(motorToIC(motor), TMC2240_THIGH);
			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
			tmc2240_writeInt(motorToIC(motor), TMC2240_THIGH, *value);
		}
		break;

	case 28:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_VHIGHCHM_MASK, TMC2240_VHIGHCHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_VHIGHCHM_MASK, TMC2240_VHIGHCHM_SHIFT, *value);
		}
		break;
	case 29:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_VHIGHFS_MASK, TMC2240_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_VHIGHFS_MASK, TMC2240_VHIGHFS_SHIFT, *value);
		}
		break;
	case 30:
		// Measured Speed
		if(readWrite == READ) {
			*value = TMC2240.velocity;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 34:
		// Internal RSense
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_GCONF, TMC2240_REFR_DIR_MASK, TMC2240_REFR_DIR_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_GCONF, TMC2240_REFR_DIR_MASK, TMC2240_REFR_DIR_SHIFT, *value);
		}
		break;

	case 35:
		// Global current scaler
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_GLOBAL_SCALER, TMC2240_GLOBALSCALER_MASK, TMC2240_GLOBALSCALER_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value > 31){
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_GLOBAL_SCALER, TMC2240_GLOBALSCALER_MASK, TMC2240_GLOBALSCALER_SHIFT, *value);
			}
			else{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_GLOBAL_SCALER, TMC2240_GLOBALSCALER_MASK, TMC2240_GLOBALSCALER_SHIFT, 0);
			}
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_MRES_MASK, TMC2240_MRES_SHIFT);
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
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_MRES_MASK, TMC2240_MRES_SHIFT, *value);
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
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_TBL_MASK, TMC2240_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_TBL_MASK, TMC2240_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_CHM_MASK, TMC2240_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_CHM_MASK, TMC2240_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_DISFDCC_MASK, TMC2240_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_DISFDCC_MASK, TMC2240_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		buffer = tmc2240_readInt(motorToIC(motor), TMC2240_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC2240_CHM_SHIFT))
			{
				*value = (buffer >> TMC2240_HEND_OFFSET_SHIFT) & TMC2240_HEND_OFFSET_MASK;
			}
			else
			{
				*value = (tmc2240_readInt(motorToIC(motor), TMC2240_CHOPCONF) >> TMC2240_HSTRT_TFD210_SHIFT) & TMC2240_HSTRT_TFD210_MASK;
				if(buffer & TMC2240_FD3_SHIFT)
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(tmc2240_readInt(motorToIC(motor), TMC2240_CHOPCONF) & (1<<14))
			{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_HEND_OFFSET_MASK, TMC2240_HEND_OFFSET_SHIFT, *value);
			}
			else
			{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_FD3_MASK, TMC2240_FD3_SHIFT, (*value & (1<<3))); // MSB wird zu value dazugefügt
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_HSTRT_TFD210_MASK, TMC2240_HSTRT_TFD210_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		buffer = tmc2240_readInt(motorToIC(motor), TMC2240_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC2240_CHM_SHIFT))
			{
				*value = (buffer >> TMC2240_HSTRT_TFD210_SHIFT) & TMC2240_HSTRT_TFD210_MASK;
			}
			else
			{
				*value = (buffer >> TMC2240_HEND_OFFSET_SHIFT) & TMC2240_HEND_OFFSET_MASK;
				if(buffer & (1 << TMC2240_FD3_SHIFT))
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1 << TMC2240_CHM_SHIFT))
			{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_HSTRT_TFD210_MASK, TMC2240_HSTRT_TFD210_SHIFT, *value);
			}
			else
			{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_HEND_OFFSET_MASK, TMC2240_HEND_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_TOFF_MASK, TMC2240_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_CHOPCONF, TMC2240_TOFF_MASK, TMC2240_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEIMIN_MASK, TMC2240_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEIMIN_MASK, TMC2240_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEDN_MASK, TMC2240_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEDN_MASK, TMC2240_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEMAX_MASK, TMC2240_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEMAX_MASK, TMC2240_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEUP_MASK, TMC2240_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEUP_MASK, TMC2240_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEMIN_MASK, TMC2240_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SEMIN_MASK, TMC2240_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard4 filter enable
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SFILT_MASK, TMC2240_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SFILT_MASK, TMC2240_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard4 threshold
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SGT_MASK, TMC2240_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_COOLCONF, TMC2240_SGT_MASK, TMC2240_SGT_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_DRVSTATUS, TMC2240_CS_ACTUAL_MASK, TMC2240_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;

	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			buffer = tmc2240_readInt(motorToIC(motor), TMC2240_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc2240_writeInt(motorToIC(motor), TMC2240_TCOOLTHRS, *value);
		}
		break;
	case 183:
		// SG_FILT_EN
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_SG4_THRS, TMC2240_SG4_FILT_EN_MASK, TMC2240_SG4_FILT_EN_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2){
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_SG4_THRS, TMC2240_SG4_FILT_EN_MASK, TMC2240_SG4_FILT_EN_SHIFT, *value);
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
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_SG4_THRS, TMC2240_SG_ANGLE_OFFSET_MASK, TMC2240_SG_ANGLE_OFFSET_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_SG4_THRS, TMC2240_SG_ANGLE_OFFSET_MASK, TMC2240_SG_ANGLE_OFFSET_SHIFT, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = (tmc2240_readInt(motorToIC(motor), TMC2240_CHOPCONF) >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			buffer = tmc2240_readInt(motorToIC(motor), TMC2240_CHOPCONF);
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			tmc2240_writeInt(motorToIC(motor), TMC2240_CHOPCONF,buffer);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			buffer = tmc2240_readInt(motorToIC(motor), TMC2240_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc2240_writeInt(motorToIC(motor), TMC2240_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_GRAD_MASK, TMC2240_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_GRAD_MASK, TMC2240_PWM_GRAD_SHIFT, *value);
			// Enable/disable stealthChop accordingly
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_GCONF, TMC2240_EN_PWM_MODE_MASK, TMC2240_EN_PWM_MODE_SHIFT, (*value) ? 1 : 0);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_OFS_MASK, TMC2240_PWM_OFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_OFS_MASK, TMC2240_PWM_OFS_SHIFT, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_FREQ_MASK, TMC2240_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_FREQ_MASK, TMC2240_PWM_FREQ_SHIFT, *value);
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
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_AUTOSCALE_MASK, TMC2240_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
			{
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_AUTOSCALE_MASK, TMC2240_PWM_AUTOSCALE_SHIFT, *value);
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
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMSCALE, TMC2240_PWM_SCALE_SUM_MASK, TMC2240_PWM_SCALE_SUM_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 194:
		// MSCNT
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_MSCNT, TMC2240_MSCNT_MASK, TMC2240_MSCNT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// MEAS_SD_EN
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_MEAS_SD_ENABLE_MASK, TMC2240_PWM_MEAS_SD_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_MEAS_SD_ENABLE_MASK, TMC2240_PWM_MEAS_SD_ENABLE_SHIFT, *value);
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 196:
		// DIS_REG_STST
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_DIS_REG_STST_MASK, TMC2240_PWM_DIS_REG_STST_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
				TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_PWM_DIS_REG_STST_MASK, TMC2240_PWM_DIS_REG_STST_SHIFT, *value);
			else
				errors |= TMC_ERROR_TYPE;
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_PWMCONF, TMC2240_FREEWHEEL_MASK, TMC2240_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_PWMCONF, TMC2240_FREEWHEEL_MASK, TMC2240_FREEWHEEL_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_DRVSTATUS, TMC2240_SG_RESULT_MASK, TMC2240_SG_RESULT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = tmc2240_readInt(motorToIC(motor), TMC2240_XENC);
		} else if(readWrite == WRITE) {
			tmc2240_writeInt(motorToIC(motor), TMC2240_XENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = tmc2240_readInt(motorToIC(motor), TMC2240_ENC_CONST);
		} else if(readWrite == WRITE) {
			tmc2240_writeInt(motorToIC(motor), TMC2240_ENC_CONST, *value);
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
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_DRV_CONF, TMC2240_CURRENT_RANGE_MASK, TMC2240_CURRENT_RANGE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_DRV_CONF, TMC2240_CURRENT_RANGE_MASK, TMC2240_CURRENT_RANGE_SHIFT, *value);
		}
		break;

	case 213:
		// ADCTemperatur
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_ADC_TEMP, TMC2240_ADC_TEMP_MASK, TMC2240_ADC_TEMP_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// ADCIN
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_ADC_VSUPPLY_AIN, TMC2240_ADC_AIN_MASK, TMC2240_ADC_AIN_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// ADCSupply
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_ADC_VSUPPLY_AIN, TMC2240_ADC_VSUPPLY_MASK, TMC2240_ADC_VSUPPLY_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 216:
		// Overvoltage Limit ADC value
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERVOLTAGE_VTH_MASK, TMC2240_OVERVOLTAGE_VTH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERVOLTAGE_VTH_MASK, TMC2240_OVERVOLTAGE_VTH_SHIFT, *value);
		}
		break;
	case 217:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			*value = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERTEMPPREWARNING_VTH_MASK, TMC2240_OVERTEMPPREWARNING_VTH_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERTEMPPREWARNING_VTH_MASK, TMC2240_OVERTEMPPREWARNING_VTH_SHIFT, *value);
		}
		break;
	case 218:
		// ADCTemperatur Converted
		if(readWrite == READ) {

			int adc = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_ADC_TEMP, TMC2240_ADC_TEMP_MASK, TMC2240_ADC_TEMP_SHIFT);
			*value = (int)10*(adc-2038)/77;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 219:
		// ADCIN converted
		if(readWrite == READ) {
			int adc = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_ADC_VSUPPLY_AIN, TMC2240_ADC_AIN_MASK, TMC2240_ADC_AIN_SHIFT);
			*value = (int)3052*adc/10000;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 220:
		// ADCSupply
		if(readWrite == READ) {
			int adc = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_ADC_VSUPPLY_AIN, TMC2240_ADC_VSUPPLY_MASK, TMC2240_ADC_VSUPPLY_SHIFT);
			*value = (int)32*3052*adc/10000;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 221:
		// Overvoltage Limit converted
		if(readWrite == READ) {
			int val = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERVOLTAGE_VTH_MASK, TMC2240_OVERVOLTAGE_VTH_SHIFT);
			*value = (int)32*3052*val/10000;
		} else if(readWrite == WRITE) {
			int val = (int)(*value*10000/(3052*32));
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERVOLTAGE_VTH_MASK, TMC2240_OVERVOLTAGE_VTH_SHIFT, val);
		}
		break;
	case 222:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			int temp = TMC2240_FIELD_READ(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERTEMPPREWARNING_VTH_MASK, TMC2240_OVERTEMPPREWARNING_VTH_SHIFT);
			*value = (int)(temp-2038)/7.7;
		} else if(readWrite == WRITE) {
			float valf  = *value*7.7;
			int val = (int)valf;
			val = val+2038;
			TMC2240_FIELD_WRITE(motorToIC(motor), TMC2240_OTW_OV_VTH, TMC2240_OVERTEMPPREWARNING_VTH_MASK, TMC2240_OVERTEMPPREWARNING_VTH_SHIFT, val);
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
	case 224: // Enable UART mode
		if (readWrite == WRITE) {
			if(*value == 1)
				commMode = TMC_BOARD_COMM_UART;
			else if(*value == 0)
				commMode = TMC_BOARD_COMM_SPI;
			init_comm(commMode);
		}
		else if(readWrite == READ) {
			if(commMode == TMC_BOARD_COMM_UART)
				*value = 1;
			else if (commMode == TMC_BOARD_COMM_SPI)
				*value = 0;
				}
			break;
	case 225: // UART slave address. Remove later
		if (readWrite == READ) {
			*value = targetAddressUart;
		}
		else if(readWrite == WRITE) {
			targetAddressUart = *value;
		}
		break;

	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value) {
	return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value) {
	return handleParameter(READ, motor, type, value);
}

static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor,
		int32_t *value) {
	UNUSED(motor);
	uint32_t errors = TMC_ERROR_NONE;
	switch (type) {
	case 2:
	case 3:
	case 4:
	case 24:
		if (limit == LIMIT_MIN) {
			*value = 0; // TODO: Determine limits here
		} else if (limit == LIMIT_MAX) {
			*value = StepDir_getFrequency(motor);
		}
		break;
	case 5:
		if (limit == LIMIT_MIN) {
			*value = 0; // TODO: Determine limits here
		} else if (limit == LIMIT_MAX) {
			*value = StepDir_getMaxAcceleration(motor);
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value) {
	return getLimit(LIMIT_MIN, type, motor, value);
}

static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value) {
	return getLimit(LIMIT_MAX, type, motor, value);
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value) {
	tmc2240_writeInt(motorToIC(motor), address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value) {
	*value = tmc2240_readInt(motorToIC(motor), address);
}
static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}
static void periodicJob(uint32_t tick)
{
	//check if reset after nSLEEP to HIGH was performed
	if(!noRegResetnSLEEP)
	{
		for(int motor = 0; motor < TMC2240_MOTORS; motor++)
			{
				tmc2240_periodicJob(&TMC2240, tick);
				StepDir_periodicJob(motor);
			}
	}
	else
	{
		//check if minimum time since chip activation passed. Then restore.
		if((systick_getTick()-nSLEEPTick)>20) //
		{
			tmc2240_restore(&TMC2240);
			noRegResetnSLEEP = false;
		}
	}
}


static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	case 8: // Enable UART mode
		if(*value == 1)
			commMode = TMC_BOARD_COMM_UART;
		else if(*value == 0)
			commMode = TMC_BOARD_COMM_SPI;
		init_comm(commMode);
		break;

	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value) {
	if (motor >= TMC2240_MOTORS)
		return TMC_ERROR_MOTOR;

	switch (motor) {
	case 0:
		//*value = StepDir.ch1->actualVelocity;
		*value = StepDir_getActualVelocity(0);
		break;
	default:
		return TMC_ERROR_MOTOR;
		break;
	}
	return TMC_ERROR_NONE;
}

static void deInit(void) {
	HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);

	HAL.IOs->config->reset(Pins.STEP);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.DIAG0);
	HAL.IOs->config->reset(Pins.DIAG1);
	HAL.IOs->config->reset(Pins.nSLEEP);
	HAL.IOs->config->reset(Pins.IREF_R2);
	HAL.IOs->config->reset(Pins.IREF_R3);
	HAL.IOs->config->reset(Pins.UART_MODE);

	StepDir_deInit();
	//Timer.deInit();
}

static uint8_t reset() {
	if (StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
		return 0;

	tmc2240_reset(&TMC2240);

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 20000);
	StepDir_setAcceleration(0, 25000);
	enableDriver(DRIVER_ENABLE);
	return 1;
}

static uint8_t restore() {
	return tmc2240_restore(&TMC2240);
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 20000);
	StepDir_setAcceleration(0, 25000);
}

static void configCallback(TMC2240TypeDef *tmc2240, ConfigState completedState) {
	if (completedState == CONFIG_RESET) {
		// Configuration reset completed
		// Change hardware preset registers here
		tmc2240_writeInt(tmc2240, TMC2240_PWMCONF, 0x000504C8);

	}
}

static void enableDriver(DriverState state) {
	if (state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if (state == DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
	else if ((state == DRIVER_ENABLE)
			&& (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
}
static void init_comm(TMC_Board_Comm_Mode mode)
{
	static TMC_Board_Comm_Mode old = TMC_BOARD_COMM_SPI;
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
		TMC2240_UARTChannel = HAL.UART;
		TMC2240_UARTChannel->pinout = UART_PINS_2;
		TMC2240_UARTChannel->rxtx.init();
		break;
	case TMC_BOARD_COMM_SPI:
		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC2240_UARTChannel->rxtx.deInit();
		TMC2240_SPIChannel = &HAL.SPI->ch2;
		TMC2240_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;
		break;
	case TMC_BOARD_COMM_WLAN: // unused
	default:
		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC2240_UARTChannel->rxtx.deInit();
		TMC2240_SPIChannel = &HAL.SPI->ch2;
		TMC2240_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;
		commMode = TMC_BOARD_COMM_SPI;
		break;
	}
}


void TMC2240_init(void) {
	tmc_fillCRC8Table(0x07, true, 1);

	// Initialize the hardware pins
	Pins.DRV_ENN_CFG6 = &HAL.IOs->pins->DIO0;
	Pins.STEP         = &HAL.IOs->pins->DIO6;
	Pins.DIR 	      = &HAL.IOs->pins->DIO7;
	Pins.DIAG0 	 	  = &HAL.IOs->pins->DIO16;
	Pins.DIAG1        = &HAL.IOs->pins->DIO15;
	Pins.nSLEEP       = &HAL.IOs->pins->DIO8;
	Pins.IREF_R2      = &HAL.IOs->pins->DIO1;
	Pins.IREF_R3      = &HAL.IOs->pins->DIO2;
	Pins.UART_MODE    = &HAL.IOs->pins->DIO9;
	Pins.SCK          = &HAL.IOs->pins->SPI2_SCK; //
	Pins.SDI          = &HAL.IOs->pins->SPI2_SDI; //
	Pins.SDO          = &HAL.IOs->pins->SPI2_SDO; //
	Pins.CS           = &HAL.IOs->pins->SPI2_CSN0; //

	HAL.IOs->config->toInput(Pins.DIAG0);
	HAL.IOs->config->toInput(Pins.DIAG1);

	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
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


	// Initialize the SPI channel
	//init_comm((uart_mode) ? TMC_BOARD_COMM_UART : TMC_BOARD_COMM_SPI);
	init_comm(commMode);


	Evalboards.ch2.config->reset = reset;
	Evalboards.ch2.config->restore = restore;
	Evalboards.ch2.config->state = CONFIG_RESET;
	Evalboards.ch2.config->configIndex = 0;

	tmc2240_init(&TMC2240, 0, Evalboards.ch2.config, tmc2240_defaultRegisterResetState);
	// Initialize the software StepDir generator
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 100000);
	StepDir_setAcceleration(0, 25000);

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
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = TMC2240_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
