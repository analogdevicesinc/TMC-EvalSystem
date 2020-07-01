#include "Board.h"
#include "tmc/BoardAssignment.h"
#include "tmc/ic/TMC4331/TMC4331.h"
#include "tmc/ic/TMC2660/TMC2660_Macros.h"
#include "tmc/ic/TMC2130/TMC2130.h"
#include "tmc/ic/TMC2160/TMC2160.h"

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

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint8_t reset();
static uint8_t restore();

typedef struct
{
	IOPinTypeDef  *TARGET_REACHED;
	IOPinTypeDef  *NRST;
	IOPinTypeDef  *FREEZE;
	IOPinTypeDef  *START;
	IOPinTypeDef  *HOME_REF;
	IOPinTypeDef  *STOP_R;
	IOPinTypeDef  *STOP_L;
	IOPinTypeDef  *INTR;
	IOPinTypeDef  *STANDBY_CLK;
} PinsTypeDef;

static PinsTypeDef Pins;

static SPIChannelTypeDef *TMC4331_SPIChannel;
static TMC4331TypeDef TMC4331;

static uint32_t vmax_position = 0;

// Translate motor number to TMC4331TypeDef
// When using multiple ICs you can map them here
static inline TMC4331TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC4331;
}

// Translate channel number to SPI channel
// When using multiple ICs you can map them here
static inline SPIChannelTypeDef *channelToSPI(uint8_t channel)
{
	UNUSED(channel);

	return TMC4331_SPIChannel;
}

// => SPI Wrapper
void tmc4331_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	channelToSPI(channel)->readWriteArray(data, length);
}
// <= SPI Wrapper

// Route the generic cover function to the TMC4331 function
// This also provides the TMC4331TypeDef, which the generic
// cover function doesn't know.
static void tmc4331_fullCover(uint8_t *data, size_t length)
{
	tmc4331_readWriteCover(&TMC4331, data, length);
}

// The cover function emulates the SPI readWrite function
static uint8_t tmc4331_cover(uint8_t data, uint8_t lastTransfer)
{
	static uint64_t coverIn = 0;     // read from squirrel
	static uint64_t coverOut = 0;    // write to squirrel
	static uint8_t coverLength = 0;  // data to be written

	uint8_t out = 0; // return value of this function

	// buffer outgoing data
	coverOut <<= 8;    // shift left by one byte to make room for the next byte
	coverOut |= data;  // add new byte to be written
	coverLength++;     // count outgoing bytes

	// return read and buffered byte to be returned
	out = coverIn >> 56;  // output last received byte
	coverIn <<= 8;        // shift by one byte to read this next time

	if(lastTransfer)
	{
		/* Write data to cover register(s). The lower 4 bytes go into the cover low register,
		 * the higher 4 bytes, if present, go into the cover high register.
		 * The datagram needs to be sent twice, otherwise the read buffer will be delayed by
		 * one read/write datagram.
		 */

		// Send the buffered datagram & wait a bit before continuing so the 4361 can complete the datagram to the driver
		// measured delay between COVER_LOW transmission and COVER_DONE flag: ~90µs -> 1 ms more than enough
		// todo CHECK 3: Delay measurement only done on TMC4361, not 4361A - make sure the required delay didnt change (LH) #1
		if(coverLength > 4)
			tmc4331_writeInt(&TMC4331, TMC4331_COVER_HIGH_WR, coverOut >> 32);
		tmc4331_writeInt(&TMC4331, TMC4331_COVER_LOW_WR, coverOut & 0xFFFFFFFF);
		wait(1);

		// Trigger a re-send by writing the low register again
		tmc4331_writeInt(&TMC4331, TMC4331_COVER_LOW_WR, coverOut & 0xFFFFFFFF);

		// Read the reply
		coverIn = 0;
		if(coverLength > 4)
			coverIn |= (uint64_t) tmc4331_readInt(&TMC4331, TMC4331_COVER_DRV_HIGH_RD) << 32;
		coverIn |= tmc4331_readInt(&TMC4331, TMC4331_COVER_DRV_LOW_RD);
		coverIn <<= (8-coverLength) * 8; // Shift the highest byte of the reply to the highest byte of the buffer uint64_t

		// Clear write buffer
		coverOut = 0;
		coverLength=0;
	}

	return out; // return buffered read byte
}

// => Functions forwarded to API
static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	UNUSED(motor);
	tmc4331_rotate(motorToIC(motor), velocity);

	return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	rotate(motor, velocity);

	return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	rotate(motor, -velocity);

	return 0;
}

static uint32_t stop(uint8_t motor)
{
	rotate(motor, 0);

	return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	tmc4331_moveTo(motorToIC(motor), position, vmax_position);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc4331_moveBy(motorToIC(motor), ticks, vmax_position);

	return 0;
}
// <= Functions forwarded to API

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC4331_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_X_TARGET);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331, TMC4331_X_TARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_XACTUAL);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VMAX) >> 8;
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_VMAX, abs(*value));
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VACTUAL);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = vmax_position;
			//*value = tmc4331_readInt(&TMC4331, TMC4331_VMAX) >> 8;
		} else if(readWrite == WRITE) {
			vmax_position = *value;

			if(tmc4331_readInt(motorToIC(motor), TMC4331_RAMPMODE) & TMC4331_RAMP_POSITION)
				tmc4331_writeInt(motorToIC(motor), TMC4331_VMAX, tmc4331_discardVelocityDecimals(vmax_position));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_AMAX)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4331_writeInt(&TMC4331,  TMC4331_AMAX, *value<<2);
			}
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = TMC4331_FIELD_READ(&TMC4331, TMC4331_STATUS, TMC4331_TARGET_REACHED_F_MASK, TMC4331_TARGET_REACHED_F_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 14:
		// Ramp type
		if(readWrite == READ) { // value = true/non-zero: TMC4331_RAMP_SSHAPE, = false/zero: TMC4331_RAMP_TRAPEZ
			*value = TMC4331_FIELD_READ(&TMC4331, TMC4331_RAMPMODE, TMC4331_RAMP_PROFILE_MASK, TMC4331_RAMP_PROFILE_SHIFT) == TMC4331_RAMP_SSHAPE;
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331, TMC4331_RAMPMODE, (*value) ? TMC4331_RAMP_SSHAPE : TMC4331_RAMP_TRAPEZ);
		}
		break;
	case 15:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VSTART);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_VSTART, *value);
		}
		break;
	case 16:
		// Acceleration AStart
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_ASTART)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4331_writeInt(&TMC4331,  TMC4331_ASTART, *value<<2);
			}
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_DMAX)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4331_writeInt(&TMC4331,  TMC4331_DMAX, *value<<2);
			}
		}
		break;
	case 18:
		// Velocity VBreak
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VBREAK);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_VBREAK, *value);
		}
		break;
	case 19:
		// Deceleration DFinal
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_DFINAL)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4331_writeInt(&TMC4331,  TMC4331_DFINAL, *value<<2);
			}
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VSTOP);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_VSTOP, *value);
		}
		break;
	case 21:
		// Deceleration DStop
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_DSTOP);
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4331_writeInt(&TMC4331,  TMC4331_DSTOP, *value);
			}
		}
		break;
	case 22:
		// Bow 1
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_BOW1);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_BOW1, *value);
		}
		break;
	case 23:
		// Bow 2
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_BOW2);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_BOW2, *value);
		}
		break;
	case 24:
		// Bow 3
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_BOW3);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_BOW3, *value);
		}
		break;
	case 25:
		// Bow 4
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_BOW4);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_BOW4, *value);
		}
		break;
	case 26:
		// Virtual stop left
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VIRT_STOP_LEFT);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_VIRT_STOP_LEFT, *value);
		}
		break;
	case 27:
		// Virtual stop right
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_VIRT_STOP_RIGHT);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_VIRT_STOP_RIGHT, *value);
		}
		break;
	case 214:
		// Power Down Delay
		if(readWrite == READ) {
			*value = tmc4331_readInt(&TMC4331, TMC4331_STDBY_DELAY);
		} else if(readWrite == WRITE) {
			tmc4331_writeInt(&TMC4331,  TMC4331_STDBY_DELAY, *value*160000);
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

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	// Notify driver shadows about register changes made via cover
	static int32_t high;
	switch(address) {
	case TMC4331_COVER_HIGH_WR:
		high = value;
		break;
	case TMC4331_COVER_LOW_WR:
		if(Evalboards.ch2.id == ID_TMC2660) // TMC2660 -> 20 bit registers, 8 bit address
			Evalboards.ch2.writeRegister(motor, TMC2660_ADDRESS(value), TMC2660_VALUE(value));
		else // All other drivers -> 32 bit registers, 8 bit address
			Evalboards.ch2.writeRegister(motor, TMC_ADDRESS(high), value);
		break;
	case TMC4331_SCALE_VALUES:
		/* Only possible with IHOLD and only with TMC2130 and TMC2160, since write-only registers changed actively by
		 * the TMC43XX (not via cover datagrams) are impossible to track.
		 */
		switch(Evalboards.ch2.id) {
		case ID_TMC2130:
			TMC2130_FIELD_WRITE((TMC2130TypeDef *)Evalboards.ch2.type, TMC2130_IHOLD_IRUN, TMC2130_IHOLD_MASK, TMC2130_IHOLD_SHIFT, FIELD_GET(value, TMC4331_HOLD_SCALE_VAL_MASK, TMC4331_HOLD_SCALE_VAL_SHIFT));
			break;
		case ID_TMC2160:
			TMC2160_FIELD_WRITE((TMC2160TypeDef *)Evalboards.ch2.type, TMC2160_IHOLD_IRUN, TMC2160_IHOLD_MASK, TMC2160_IHOLD_SHIFT, FIELD_GET(value, TMC4331_HOLD_SCALE_VAL_MASK, TMC4331_HOLD_SCALE_VAL_SHIFT));
			break;
		}
		break;
	}
	tmc4331_writeInt(motorToIC(motor), address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value	= tmc4331_readInt(motorToIC(motor), address);
}

static void periodicJob(uint32_t tick)
{
	tmc4331_periodicJob(&TMC4331, tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	uint32_t errors = 0;

	switch(type)
	{
	case 0:	// simulate left/right reference switches, set high to support external ref swiches
		/*
		 * The the TMC4361 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */
		if(!(*value & ~3))
		{
			if(*value & (1<<0))
			{
				HAL.IOs->config->toInput(Pins.STOP_R); // pull up -> set it to floating causes high
			}
			else
			{
				HAL.IOs->config->toOutput(Pins.STOP_R);
				HAL.IOs->config->setLow(Pins.STOP_R);
			}

			if(*value & (1<<1))
			{
				HAL.IOs->config->toInput(Pins.STOP_L); // pull up -> set it to floating causes high
			}
			else
			{
				HAL.IOs->config->toOutput(Pins.STOP_L);
				HAL.IOs->config->setLow(Pins.STOP_L);
			}
		}
		//else TMCL.reply->Status = REPLY_INVALID_VALUE;
		break;
	case 1:	// simulate reference switche HOME_REF, set high to support external ref swiches
		/*
		 * The the TMC43x1 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */
		if(*value)
		{
			HAL.IOs->config->toInput(Pins.HOME_REF); // pull up -> set it to floating causes high
		}
		else
		{
			HAL.IOs->config->toOutput(Pins.HOME_REF);
			HAL.IOs->config->setLow(Pins.HOME_REF);
		}
		break;
	case 2:	// simulate reference switche FREEZE, set high to support external ref swiches
		/*
		 * The the TMC43x1 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */

		if(*value)
		{
			HAL.IOs->config->toInput(Pins.FREEZE); // pull up -> set it to floating causes high
		}
		else
		{
			HAL.IOs->config->toOutput(Pins.FREEZE);
			HAL.IOs->config->setLow(Pins.FREEZE);
		}
		break;
	case 3:
		errors |= TMC_ERROR_NOT_DONE;
		break;
	case 255:
		Evalboards.ch2.config->reset();
		Evalboards.ch1.config->reset();
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static void deInit(void)
{
	HAL.IOs->config->setLow(Pins.NRST);

	HAL.IOs->config->reset(Pins.STOP_L);
	HAL.IOs->config->reset(Pins.STOP_R);
	HAL.IOs->config->reset(Pins.HOME_REF);
	HAL.IOs->config->reset(Pins.START);
	HAL.IOs->config->reset(Pins.FREEZE);
	HAL.IOs->config->reset(Pins.STANDBY_CLK);
	HAL.IOs->config->reset(Pins.INTR);
	HAL.IOs->config->reset(Pins.TARGET_REACHED);
	HAL.IOs->config->reset(Pins.NRST);

	HAL.SPI->ch2.reset();
}

static uint8_t reset()
{
	// Pulse the low-active hardware reset pin
	HAL.IOs->config->setLow(Pins.NRST);
	wait(1);
	HAL.IOs->config->setHigh(Pins.NRST);

	tmc4331_reset(&TMC4331);

	return 1;
}

static uint8_t restore()
{
	// Pulse the low-active hardware reset pin
	HAL.IOs->config->setLow(Pins.NRST);
	wait(1);
	HAL.IOs->config->setHigh(Pins.NRST);

	tmc4331_restore(&TMC4331);

	return 1;
}

static void configCallback(TMC4331TypeDef *tmc4331, ConfigState state)
{
	uint8_t driver, dataLength;
	uint32_t value;

	// Setup SPI
	switch(Evalboards.ch2.id)
	{
	case ID_TMC2130:
		driver = 0x0C;
		dataLength = 0;
		break;
	case ID_TMC2660:
		driver = 0x0B;
		dataLength = 0;
		break;
	default:
		driver = 0x0F;
		dataLength = 40;
		break;
	}
	value = 0x44400040 | (dataLength << 13) | (driver << 0);
	tmc4331_writeInt(tmc4331, TMC4331_SPIOUT_CONF, value);

	// Reset/Restore driver
	if(state == CONFIG_RESET)
		Evalboards.ch2.config->reset();
	else
		Evalboards.ch2.config->restore();
}

void TMC4331_init(void)
{
	tmc4331_init(&TMC4331, 0, Evalboards.ch1.config, &tmc4331_defaultRegisterResetState[0]);
	tmc4331_setCallback(&TMC4331, configCallback);

	Pins.STANDBY_CLK     = &HAL.IOs->pins->DIO4;
	Pins.INTR            = &HAL.IOs->pins->DIO5;
	Pins.STOP_L          = &HAL.IOs->pins->DIO12;
	Pins.STOP_R          = &HAL.IOs->pins->DIO13;
	Pins.HOME_REF        = &HAL.IOs->pins->DIO14;
	Pins.START           = &HAL.IOs->pins->DIO15;
	Pins.FREEZE          = &HAL.IOs->pins->DIO16;
	Pins.NRST            = &HAL.IOs->pins->DIO17;
	Pins.TARGET_REACHED  = &HAL.IOs->pins->DIO18;

	HAL.IOs->config->toOutput(Pins.NRST);
	HAL.IOs->config->toOutput(Pins.STOP_L);
	HAL.IOs->config->toOutput(Pins.STOP_R);
	HAL.IOs->config->toOutput(Pins.HOME_REF);
	HAL.IOs->config->toOutput(Pins.START);
	HAL.IOs->config->toOutput(Pins.FREEZE);

	HAL.IOs->config->setHigh(Pins.NRST);

	HAL.IOs->config->setHigh(Pins.STOP_L);
	HAL.IOs->config->setHigh(Pins.STOP_R);
	HAL.IOs->config->setHigh(Pins.HOME_REF);
	HAL.IOs->config->setHigh(Pins.START);
	HAL.IOs->config->setHigh(Pins.FREEZE);

	HAL.IOs->config->toInput(Pins.STANDBY_CLK);
	HAL.IOs->config->toInput(Pins.INTR);
	HAL.IOs->config->toInput(Pins.TARGET_REACHED);

	TMC4331_SPIChannel = &HAL.SPI->ch1;
	TMC4331_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	Evalboards.ch1.config->state        = CONFIG_RESET;
	Evalboards.ch1.config->configIndex  = 0;
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;

	Evalboards.ch1.cover                = tmc4331_cover;
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
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = TMC4331_MOTORS;
	Evalboards.ch1.deInit               = deInit;

	// Provide the cover function to the driver channel
	Evalboards.ch1.fullCover            = tmc4331_fullCover;
};
