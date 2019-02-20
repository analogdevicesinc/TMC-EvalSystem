#include "Board.h"
#include "tmc/ic/TMC5160/TMC5160.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

#define DEFAULT_MOTOR  0

#define TMC5160_TIMEOUT 50 // UART Timeout in ms

static bool vMaxModified = false;
//static uint32 vMax		   = 1;
static bool uart_mode = false;

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

void tmc5160_writeDatagram(uint8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
void tmc5160_writeInt(uint8 motor, uint8 address, int value);
int tmc5160_readInt(u8 motor, uint8 address);
static void writeDatagram_spi(uint8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
static int32 readInt_spi(u8 motor, uint8 address);
static void writeDatagram_uart(uint8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4);
static int32 readInt_uart(u8 motor, uint8 address);

static void init_comm(bool mode);

static void periodicJob(uint32 tick);
static void checkErrors(uint32 tick);
static void deInit(void);
static uint32 userFunction(uint8 type, uint8 motor, int32 *value);

static uint8 reset();
static void enableDriver(DriverState state);

static RXTXTypeDef *TMC5160_UARTChannel;
static SPIChannelTypeDef *TMC5160_SPIChannel;
static TMC5160TypeDef TMC5160;
static ConfigurationTypeDef *TMC5160_config;

// Translate motor number to TMC5130TypeDef
// When using multiple ICs you can map them here
//static inline TMC5160TypeDef *motorToIC(uint8 motor)
//{
//	UNUSED(motor);
//
//	return &TMC5160;
//}
//
//// Translate channel number to SPI channel
//// When using multiple ICs you can map them here
//static inline SPIChannelTypeDef *channelToSPI(uint8 channel)
//{
//	UNUSED(channel);
//
//	return TMC5160_SPIChannel;
//}
//
//// SPI Wrapper for API
//void tmc5160_readWriteArray(uint8 channel, uint8 *data, size_t length)
//{
//	// Map the channel to the corresponding SPI channel
//	channelToSPI(channel)->readWriteArray(data, length);
//}

void tmc5160_writeDatagram(uint8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{
	if(uart_mode)
		writeDatagram_uart(motor, address, x1, x2, x3, x4);
	else
		writeDatagram_spi(motor, address, x1, x2, x3, x4);
}

void tmc5160_writeInt(uint8 motor, uint8 address, int value)
{
	tmc5160_writeDatagram(motor, address, 0xFF & (value>>24), 0xFF & (value>>16), 0xFF & (value>>8), 0xFF & (value>>0));
}

int tmc5160_readInt(u8 motor, uint8 address)
{
	int32 r = 0;
	if(uart_mode)
		r = readInt_uart(motor, address);
	else
		r = readInt_spi(motor, address);
	return r;
}

static void writeDatagram_spi(uint8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{
	UNUSED(motor);
	address = TMC_ADDRESS(address);
	TMC5160_SPIChannel->readWrite(address|0x80, false);
	TMC5160_SPIChannel->readWrite(x1, false);
	TMC5160_SPIChannel->readWrite(x2, false);
	TMC5160_SPIChannel->readWrite(x3, false);
	TMC5160_SPIChannel->readWrite(x4, true);

	int value = x1;
	value <<= 8;
	value |= x2;
	value <<= 8;
	value |= x3;
	value <<= 8;
	value |= x4;

	TMC5160_config->shadowRegister[address] = value;
}

static int32 readInt_spi(u8 motor, uint8 address)
{
	UNUSED(motor);
	address = TMC_ADDRESS(address);

	// Register not readable -> shadow register copy
	if(!TMC_IS_READABLE(TMC5160.registerAccess[address]))
		return TMC5160_config->shadowRegister[address];

	TMC5160_SPIChannel->readWrite(address, false);
	TMC5160_SPIChannel->readWrite(0, false);
	TMC5160_SPIChannel->readWrite(0, false);
	TMC5160_SPIChannel->readWrite(0, false);
	TMC5160_SPIChannel->readWrite(0, true);

	TMC5160_SPIChannel->readWrite(address, false);
	int value = TMC5160_SPIChannel->readWrite(0, false);
	value <<= 8;
	value |= TMC5160_SPIChannel->readWrite(0, false);
	value <<=	8;
	value |= TMC5160_SPIChannel->readWrite(0, false);
	value <<= 8;
	value |= TMC5160_SPIChannel->readWrite(0, true);

	return value;
}

static void writeDatagram_uart(uint8 motor, uint8 address, uint8 x1, uint8 x2, uint8 x3, uint8 x4)
{

	address = TMC_ADDRESS(address);
	UNUSED(motor);
	uint8 writeData[8];

	writeData[0] = 0x05;                         // Sync byte
	writeData[1] = 0x00;                         // Slave address
	writeData[2] = address | TMC5160_WRITE_BIT;  // Register address with write bit set
	writeData[3] = x1;                           // Register Data
	writeData[4] = x2;                           // Register Data
	writeData[5] = x3;                           // Register Data
	writeData[6] = x4;                           // Register Data
	writeData[7] = tmc_CRC8(writeData, 7, 1);    // Cyclic redundancy check

	TMC5160_UARTChannel->clearBuffers();
	for(uint32 i = 0; i < ARRAY_SIZE(writeData); i++)
		TMC5160_UARTChannel->tx(writeData[i]);

	/* Workaround: Give the UART time to send. Otherwise another write/readRegister can do clearBuffers()
	 * before we're done. This currently is an issue with the IDE when using the Register browser and the
	 * periodic refresh of values gets requested right after the write request.
	 */
	wait(2);

	TMC5160_config->shadowRegister[address] = _8_32(x1, x2, x3, x4);
}

static int32 readInt_uart(u8 motor, uint8 address)
{
	UNUSED(motor);
	address = TMC_ADDRESS(address);
	uint8 readData[8], dataRequest[4];
	uint32 timeout;

	if(!TMC_IS_READABLE(TMC5160.registerAccess[address]))
	{	// Register not readable - shadowRegister copy
		return TMC5160_config->shadowRegister[address];
	}

	dataRequest[0] = 0x05;                         // Sync byte
	dataRequest[1] = 0x00;                         // Slave address
	dataRequest[2] = address;                      // Register address
	dataRequest[3] = tmc_CRC8(dataRequest, 3, 1);  // Cyclic redundancy check

	TMC5160_UARTChannel->clearBuffers();
	TMC5160_UARTChannel->txN(dataRequest, ARRAY_SIZE(dataRequest));

	// Wait for reply with timeout limit
	timeout = systick_getTick();
	while(TMC5160_UARTChannel->bytesAvailable() < ARRAY_SIZE(readData))
		if(timeSince(timeout) > TMC5160_TIMEOUT) // Timeout
			return -1;

	TMC5160_UARTChannel->rxN(readData, ARRAY_SIZE(readData));
	// Check if the received data is correct (CRC, Sync, Slave address, Register address)
	// todo CHECK 2: Only keep CRC check? Should be sufficient for wrong transmissions (LH) #1
	if(readData[7] != tmc_CRC8(readData, 7, 1) || readData[0] != 0x05 || readData[1] != 0xFF || readData[2] != address)
		return -1;

	return _8_32(readData[3], readData[4], readData[5], readData[6]);
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

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= TMC5160_MOTORS)
		return TMC_ERROR_MOTOR;

	vMaxModified = true;

	// set absolute velocity, independant from direction
	tmc5160_writeInt(motor, TMC5160_VMAX, abs(velocity));

	// signdedness defines velocity mode direction bit in rampmode register
	tmc5160_writeDatagram(motor, TMC5160_RAMPMODE, 0, 0, 0, (velocity >= 0)? 1 : 2);

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
	if(motor >= TMC5160_MOTORS)
		return TMC_ERROR_MOTOR;

	if(vMaxModified)
	{
		tmc5160_writeInt(motor, TMC5160_VMAX, TMC5160_config->shadowRegister[TMC5160_VMAX]);
		vMaxModified = false;
	}

	// set position
	tmc5160_writeInt(motor, TMC5160_XTARGET, position);

	// change to positioning mode
	tmc5160_writeDatagram(motor, TMC5160_RAMPMODE, 0, 0, 0, 0);

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	// determine actual position and add numbers of ticks to move
	*ticks = tmc5160_readInt(motor, TMC5160_XACTUAL) + *ticks;

	return moveTo(motor, *ticks);
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	uint32 buffer;
	u32 errors = TMC_ERROR_NONE;

	if(motor >= TMC5160_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_XTARGET);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_XTARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_XACTUAL);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_VMAX);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_VMAX, abs(*value));
			vMaxModified = true;
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_VACTUAL);
			*value = CAST_Sn_TO_S32(*value, 24);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = TMC5160_config->shadowRegister[TMC5160_VMAX];
		} else if(readWrite == WRITE) {
			TMC5160_config->shadowRegister[TMC5160_VMAX] = abs(*value);
			if(tmc5160_readInt(motor, TMC5160_RAMPMODE) == TMC5160_MODE_POSITION)
				tmc5160_writeInt(motor, TMC5160_VMAX, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_AMAX);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_AMAX, *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_IHOLD_IRUN, TMC5160_IRUN_MASK, TMC5160_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_IHOLD_IRUN, TMC5160_IRUN_MASK, TMC5160_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_IHOLD_IRUN, TMC5160_IHOLD_MASK, TMC5160_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_IHOLD_IRUN, TMC5160_IHOLD_MASK, TMC5160_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_RAMPSTAT, TMC5160_POSITION_REACHED_MASK, TMC5160_POSITION_REACHED_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 10:
		// Right endstop
		if(readWrite == READ) {
			*value = !TMC5160_FIELD_READ(motor, TMC5160_RAMPSTAT, TMC5160_STATUS_STOP_R_MASK, TMC5160_STATUS_STOP_R_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 11:
		// Left endstop
		if(readWrite == READ) {
			*value = !TMC5160_FIELD_READ(motor, TMC5160_RAMPSTAT, TMC5160_STATUS_STOP_L_MASK, TMC5160_STATUS_STOP_L_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 12:
		// Automatic right stop
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_SWMODE, TMC5160_STOP_R_ENABLE_MASK, TMC5160_STOP_R_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_SWMODE, TMC5160_STOP_R_ENABLE_MASK, TMC5160_STOP_R_ENABLE_SHIFT, *value);
		}
		break;
	case 13:
		// Automatic left stop
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_SWMODE, TMC5160_STOP_L_ENABLE_MASK, TMC5160_STOP_L_ENABLE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_SWMODE, TMC5160_STOP_L_ENABLE_MASK, TMC5160_STOP_L_ENABLE_SHIFT, *value);
		}
		break;
	case 14:
		// SW_MODE Register
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_SWMODE);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_SWMODE, *value);
		}
		break;
	case 15:
		// Acceleration A1
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_A1);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_A1, *value);
		}
		break;
	case 16:
		// Velocity V1
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_V1);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_V1, *value);
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_DMAX);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_DMAX, *value);
		}
		break;
	case 18:
		// Deceleration D1
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_D1);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_D1, *value);
		}
		break;
	case 19:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_VSTART);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_VSTART, *value);
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_VSTOP);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_VSTOP, *value);
		}
		break;
	case 21:
		// Waiting time after ramp down
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_TZEROWAIT);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_TZEROWAIT, *value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			buffer = tmc5160_readInt(motor, TMC5160_THIGH);
			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
			tmc5160_writeInt(motor, TMC5160_THIGH, *value);
		}
		break;
	case 24:
		// Minimum speed for switching to dcStep
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_VDCMIN);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_VDCMIN, *value);
		}
		break;
	case 27:
		// High speed chopper mode
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_VHIGHCHM_MASK, TMC5160_VHIGHCHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_VHIGHCHM_MASK, TMC5160_VHIGHCHM_SHIFT, *value);
		}
		break;
	case 28:
		// High speed fullstep mode
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_VHIGHFS_MASK, TMC5160_VHIGHFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_VHIGHFS_MASK, TMC5160_VHIGHFS_SHIFT, *value);
		}
		break;
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			*value = TMC5160.velocity;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 33:
		// Analog I Scale
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_GCONF, TMC5160_RECALIBRATE_MASK, TMC5160_RECALIBRATE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_GCONF, TMC5160_RECALIBRATE_MASK, TMC5160_RECALIBRATE_SHIFT, *value);
		}
		break;
	case 34:
		// Internal RSense
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_GCONF, TMC5160_REFR_DIR_MASK, TMC5160_REFR_DIR_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_GCONF, TMC5160_REFR_DIR_MASK, TMC5160_REFR_DIR_SHIFT, *value);
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_MRES_MASK, TMC5160_MRES_SHIFT);
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
				TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_MRES_MASK, TMC5160_MRES_SHIFT, *value);
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
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_TBL_MASK, TMC5160_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_TBL_MASK, TMC5160_TBL_SHIFT, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_CHM_MASK, TMC5160_CHM_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_CHM_MASK, TMC5160_CHM_SHIFT, *value);
		}
		break;
	case 164:
		// Disable fast decay comparator
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_DISFDCC_MASK, TMC5160_DISFDCC_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_DISFDCC_MASK, TMC5160_DISFDCC_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		buffer = tmc5160_readInt(motor, TMC5160_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5160_CHM_SHIFT))
			{
				*value = (buffer >> TMC5160_HEND_SHIFT) & TMC5160_HEND_MASK;
			}
			else
			{
				*value = (tmc5160_readInt(motor, TMC5160_CHOPCONF) >> TMC5160_TFD_ALL_SHIFT) & TMC5160_TFD_ALL_MASK;
				if(buffer & TMC5160_TFD_3_SHIFT)
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(tmc5160_readInt(motor, TMC5160_CHOPCONF) & (1<<14))
			{
				TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_HEND_MASK, TMC5160_HEND_SHIFT, *value);
			}
			else
			{
				TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_TFD_3_MASK, TMC5160_TFD_3_SHIFT, (*value & (1<<3))); // MSB wird zu value dazugefügt
				TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_TFD_ALL_MASK, TMC5160_TFD_ALL_SHIFT, *value);
			}
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		buffer = tmc5160_readInt(motor, TMC5160_CHOPCONF);
		if(readWrite == READ) {
			if(buffer & (1 << TMC5160_CHM_SHIFT))
			{
				*value = (buffer >> TMC5160_HSTRT_SHIFT) & TMC5160_HSTRT_MASK;
			}
			else
			{
				*value = (buffer >> TMC5160_OFFSET_SHIFT) & TMC5160_OFFSET_MASK;
				if(buffer & (1 << TMC5160_TFD_3_SHIFT))
					*value |= 1<<3; // MSB wird zu value dazugefügt
			}
		} else if(readWrite == WRITE) {
			if(buffer & (1 << TMC5160_CHM_SHIFT))
			{
				TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_HSTRT_MASK, TMC5160_HSTRT_SHIFT, *value);
			}
			else
			{
				TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_OFFSET_MASK, TMC5160_OFFSET_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_TOFF_MASK, TMC5160_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_TOFF_MASK, TMC5160_TOFF_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SEIMIN_MASK, TMC5160_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SEIMIN_MASK, TMC5160_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SEDN_MASK, TMC5160_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SEDN_MASK, TMC5160_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SEMAX_MASK, TMC5160_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SEMAX_MASK, TMC5160_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SEUP_MASK, TMC5160_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SEUP_MASK, TMC5160_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SEMIN_MASK, TMC5160_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SEMIN_MASK, TMC5160_SEMIN_SHIFT, *value);
		}
		break;
	case 173:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SFILT_MASK, TMC5160_SFILT_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SFILT_MASK, TMC5160_SFILT_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_COOLCONF, TMC5160_SGT_MASK, TMC5160_SGT_SHIFT);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_COOLCONF, TMC5160_SGT_MASK, TMC5160_SGT_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_DRVSTATUS, TMC5160_CS_ACTUAL_MASK, TMC5160_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		//this function sort of doubles with 182 but is necessary to allow cross chip compliance
		if(readWrite == READ) {
			if(TMC5160_FIELD_READ(motor, TMC5160_SWMODE, TMC5160_SG_STOP_MASK, TMC5160_SG_STOP_SHIFT))
			{
				buffer = tmc5160_readInt(motor, TMC5160_TCOOLTHRS);
				*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
			}
			else
			{
				*value = 0;
			}
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_SWMODE, TMC5160_SG_STOP_MASK, TMC5160_SG_STOP_SHIFT, (*value)? 1:0);

			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5160_writeInt(motor, TMC5160_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			buffer = tmc5160_readInt(motor, TMC5160_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5160_writeInt(motor, TMC5160_TCOOLTHRS, *value);
		}
		break;
	case 184:
		// Random TOff mode
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_CHOPCONF, TMC5160_RNDTF_MASK, TMC5160_RNDTF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_CHOPCONF, TMC5160_RNDTF_MASK, TMC5160_RNDTF_SHIFT, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		if(readWrite == READ) {
			*value = (tmc5160_readInt(motor, TMC5160_CHOPCONF) >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			buffer = tmc5160_readInt(motor, TMC5160_CHOPCONF);
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			tmc5160_writeInt(motor, TMC5160_CHOPCONF,buffer);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			buffer = tmc5160_readInt(motor, TMC5160_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			tmc5160_writeInt(motor, TMC5160_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_PWMCONF, TMC5160_PWM_GRAD_MASK, TMC5160_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC5160_FIELD_UPDATE(motor, TMC5160_PWMCONF, TMC5160_PWM_GRAD_MASK, TMC5160_PWM_GRAD_SHIFT, *value);

			// Enable/disable stealthChop accordingly
			TMC5160_FIELD_UPDATE(motor, TMC5160_GCONF, TMC5160_EN_PWM_MODE_MASK, TMC5160_EN_PWM_MODE_SHIFT, *value);
		}
		break;
	case 188:
		// PWM amplitude
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_PWMCONF, TMC5160_PWM_OFS_MASK, TMC5160_PWM_OFS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_PWMCONF, TMC5160_GLOBAL_SCALER_MASK, TMC5160_GLOBAL_SCALER_SHIFT, *value);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_PWMCONF, TMC5160_PWM_FREQ_MASK, TMC5160_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC5160_FIELD_UPDATE(motor, TMC5160_PWMCONF, TMC5160_PWM_FREQ_MASK, TMC5160_PWM_FREQ_SHIFT, *value);
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
			*value = TMC5160_FIELD_READ(motor, TMC5160_PWMCONF, TMC5160_PWM_AUTOSCALE_MASK, TMC5160_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 2)
			{
				TMC5160_FIELD_UPDATE(motor, TMC5160_PWMCONF, TMC5160_PWM_AUTOSCALE_MASK, TMC5160_PWM_AUTOSCALE_SHIFT, *value);
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
			*value = TMC5160_FIELD_READ(motor, TMC5160_PWMCONF, TMC5160_FREEWHEEL_MASK, TMC5160_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC5160_FIELD_UPDATE(motor, TMC5160_PWMCONF, TMC5160_FREEWHEEL_MASK, TMC5160_FREEWHEEL_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = TMC5160_FIELD_READ(motor, TMC5160_DRVSTATUS, TMC5160_SG_RESULT_MASK, TMC5160_SG_RESULT_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_XENC);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_XENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			*value = tmc5160_readInt(motor, TMC5160_ENC_CONST);
		} else if(readWrite == WRITE) {
			tmc5160_writeInt(motor, TMC5160_ENC_CONST, *value);
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
	if(motor >= TMC5160_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = TMC5160.velocity;

	return TMC_ERROR_NONE;
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmc5160_writeInt(DEFAULT_MOTOR, address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmc5160_readInt(DEFAULT_MOTOR, address);
}

static void periodicJob(uint32 tick)
{
	for(int motor = 0; motor < TMC5160_MOTORS; motor++)
	{
		tmc5160_periodicJob(motor, tick, &TMC5160, TMC5160_config);
	}
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	uint32 buffer;
	uint32 errors = 0;

	UNUSED(motor);

	switch(type)
	{
	case 0:  // simulate reference switches, set high to support external ref swiches
		/*
		 * The the TMC5160 ref switch input is pulled high by external resistor an can be pulled low either by
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
//		buffer = (uint32) *value;
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
	case 8: // Enable UART mode
		uart_mode = ((*value & 1) == 1);
		init_comm(uart_mode);
		break;
	case 9: // Switch between internal (0) / external (1) clock
		if(*value == 1) {
			HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
			HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);
		} else {
			HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
		}
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

static uint8 reset()
{
	if(!tmc5160_readInt(0, TMC5160_VACTUAL))
		tmc5160_reset(TMC5160_config);

	HAL.IOs->config->toInput(Pins.REFL_UC);
	HAL.IOs->config->toInput(Pins.REFR_UC);

	return 1;
}

static uint8 restore()
{
	return tmc5160_restore(TMC5160_config);
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

static void init_comm(bool mode)
{
	if(mode) {
		HAL.UART->init();
		TMC5160_UARTChannel = HAL.UART;
	} else {
		HAL.UART->deInit();
		TMC5160_SPIChannel = &HAL.SPI->ch1;
		TMC5160_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
	}
}

void TMC5160_init(void)
{
	tmc5160_initConfig(&TMC5160);

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
//	HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
//	HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);

	init_comm(uart_mode);

	TMC5160_config = Evalboards.ch1.config;

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
	Evalboards.ch1.numberOfMotors       = TMC5160_MOTORS;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;
	Evalboards.ch1.deInit               = deInit;

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
