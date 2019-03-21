#include "Board.h"
#include "tmc/BoardAssignment.h"
#include "tmc/ic/TMC4330/TMC4330.h"

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

static SPIChannelTypeDef *TMC4330_SPIChannel;
static TMC4330TypeDef TMC4330;

static uint32_t vmax_position = 0;

// Translate motor number to TMC4330TypeDef
// When using multiple ICs you can map them here
static inline TMC4330TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC4330;
}

// Translate channel number to SPI channel
// When using multiple ICs you can map them here
static inline SPIChannelTypeDef *channelToSPI(uint8_t channel)
{
	UNUSED(channel);

	return TMC4330_SPIChannel;
}

// => SPI Wrapper
void tmc4330_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	channelToSPI(channel)->readWriteArray(data, length);
}
// <= SPI Wrapper

// => Functions forwarded to API
static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	UNUSED(motor);
	tmc4330_rotate(motorToIC(motor), velocity);

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
	tmc4330_moveTo(motorToIC(motor), position, vmax_position);

	return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	tmc4330_moveBy(motorToIC(motor), ticks, vmax_position);

	return 0;
}
// <= Functions forwarded to API

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;
	uint32_t uvalue;

	if(motor >= TMC4330_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_X_TARGET);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_X_TARGET, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_XACTUAL);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_XACTUAL, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VMAX) >> 8;
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_VMAX, tmc4330_discardVelocityDecimals(*value));
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VACTUAL);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = vmax_position;
			//*value = tmc4330_readInt(motorToIC(motor), TMC4330_VMAX) >> 8;
		} else if(readWrite == WRITE) {
			vmax_position = *value;
			// Write VMAX if already in position mode
			if(tmc4330_readInt(motorToIC(motor), TMC4330_RAMPMODE) & TMC4330_RAMP_POSITION)
				tmc4330_writeInt(motorToIC(motor), TMC4330_VMAX, tmc4330_discardVelocityDecimals(vmax_position));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_AMAX)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4330_writeInt(motorToIC(motor), TMC4330_AMAX, *value<<2);
			}
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = (tmc4330_readInt(motorToIC(motor), TMC4330_STATUS) & (1<<0))? 1:0;
		} else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 14:
		// Ramp type
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_RAMPMODE)<<1;
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_RAMPMODE, (*value) ? TMC4330_RAMP_SSHAPE : TMC4330_RAMP_TRAPEZ);
		}
		break;
	case 15:
		// Velocity VSTART
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VSTART);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_VSTART, *value);
		}
		break;
	case 16:
		// Acceleration AStart
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_ASTART)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4330_writeInt(motorToIC(motor), TMC4330_ASTART, *value<<2);
			}
		}
		break;
	case 17:
		// Maximum Deceleration
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_DMAX)>>2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4330_writeInt(motorToIC(motor), TMC4330_DMAX, *value<<2);
			}
		}
		break;
	case 18:
		// Velocity VBreak
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VBREAK);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_VBREAK, *value);
		}
		break;
	case 19:
		// Deceleration DFinal
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_DFINAL) >> 2;
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4330_writeInt(motorToIC(motor), TMC4330_DFINAL, *value<<2);
			}
		}
		break;
	case 20:
		// Velocity VSTOP
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VSTOP);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_VSTOP, *value);
		}
		break;
	case 21:
		// Deceleration DStop
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_DSTOP);
		} else if(readWrite == WRITE) {
			if(*value & ~0x3FFFFF)
			{
				errors |= TMC_ERROR_VALUE;
			}
			else
			{
				tmc4330_writeInt(motorToIC(motor), TMC4330_DSTOP, *value);
			}
		}
		break;
	case 22:
		// Bow 1
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_BOW1);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_BOW1, *value);
		}
		break;
	case 23:
		// Bow 2
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_BOW2);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_BOW2, *value);
		}
		break;
	case 24:
		// Bow 3
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_BOW3);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_BOW3, *value);
		}
		break;
	case 25:
		// Bow 4
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_BOW4);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_BOW4, *value);
		}
		break;
	case 26:
		// Virtual stop left
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VIRT_STOP_LEFT);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_VIRT_STOP_LEFT, *value);
		}
		break;
	case 27:
		// Virtual stop right
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_VIRT_STOP_RIGHT);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_VIRT_STOP_RIGHT, *value);
		}
		break;
	case 108:
		// CL Gamma VMin
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_VMIN_EMF_WR);		// read from shadow register
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_VMIN_EMF_WR, *value);
		}
		break;
	case 109:
		// CL Gamma VMax
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_VADD_EMF); 	// read from shadow register
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_VADD_EMF, *value);
		}
		break;
	case 110:
		// CL maximum Gamma
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_BETA) >> 16;
		} else if(readWrite == WRITE) {
			uvalue = tmc4330_readInt(motorToIC(motor),  TMC4330_CL_BETA) & 0x000001FF;
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_BETA, uvalue | (*value<<16));
		}
		break;
	case 111:
		// CL beta
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_BETA) & 0xFF;
		} else if(readWrite == WRITE) {
			uvalue = tmc4330_readInt(motorToIC(motor),  TMC4330_CL_BETA) & 0x00FF0000;
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_BETA, uvalue | (*value & 0x1FF));
		}
		break;
	case 112:
		// CL offset
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_OFFSET);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_OFFSET, *value);
		}
		break;
	case 113:
		// CL current minimum
		if(readWrite == READ) {
			*value = (tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) >> 0) & 0xFF;
		} else if(readWrite == WRITE) {
			uvalue = tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) & ~(0xFF<<0);
			uvalue |= (*value & 0xFF) << 0;
			tmc4330_writeInt(motorToIC(motor), TMC4330_SCALE_VALUES, uvalue);
		}
		break;
	case 114:
		// CL current maximum
		if(readWrite == READ) {
			*value = (tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) >> 8) & 0xFF;
		} else if(readWrite == WRITE) {
			uvalue = tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) & ~(0xFF<<8);
			uvalue |= (*value & 0xFF) << 8;
			tmc4330_writeInt(motorToIC(motor), TMC4330_SCALE_VALUES, uvalue);
		}
		break;
	case 115:
		// CL correction velocity P
		if(readWrite == READ) {
			*value = TMC4330.config->shadowRegister[TMC4330_CL_VMAX_CALC_P_WR];
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_VMAX_CALC_P_WR, *value);
		}
		break;
	case 116:
		// CL correction velocity I
		if(readWrite == READ) {
			*value = TMC4330.config->shadowRegister[TMC4330_CL_VMAX_CALC_I_WR];
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_VMAX_CALC_I_WR, *value);
		}
		break;
	case 117:
		// CL correction velocity I clipping
		// todo AP 3: same register as in AP 116 and 115? (BS) #1
		// (JE) : probably register TMC4361_PID_I_CLIP_WR ?
		if(readWrite == READ) {
			*value = TMC4330.config->shadowRegister[TMC4330_PID_I_WR] >> 0;
			*value &= 0x7FFF;
		} else if(readWrite == WRITE) {
			uvalue = TMC4330.config->shadowRegister[TMC4330_PID_I_WR];
			uvalue &= ~(0x7FFF << 0);
			uvalue |= (*value & 0x7FFF) << 0;
			tmc4330_writeInt(motorToIC(motor), TMC4330_PID_I_WR, uvalue);
		}
		break;
	case 118:
		// CL correction velocity DV clock
		// todo AP 3: same register as in AP 116 and 115? (BS) #2
		// (JE) probably register TMC4361_PID_I_CLIP_WR?
		if(readWrite == READ) {
			*value = TMC4330.config->shadowRegister[TMC4330_PID_I_WR] >> 16;
			*value &= 0xFF;
		} else if(readWrite == WRITE) {
			uvalue = TMC4330.config->shadowRegister[TMC4330_PID_I_WR];
			uvalue &= ~(0xFF << 16);
			uvalue |= (*value & 0xFF) << 16;
			tmc4330_writeInt(motorToIC(motor), TMC4330_PID_I_WR, uvalue);
		}
		break;
	case 119:
		// CL correction velocity DV clipping
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_PID_DV_CLIP_WR);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_PID_DV_CLIP_WR, *value);
		}
		break;
	case 124:
		// CL Correction Position P
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_DELTA_P_WR);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_DELTA_P_WR, *value);
		}
		break;
	case 125:
		// CL max. correction tolerance
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_CL_TOLERANCE_WR);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_TOLERANCE_WR, *value);
		}
		break;
	case 126:
		// CL start up
		if(readWrite == READ) {
			*value = (tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) >> 16) & 0xFF;
		} else if(readWrite == WRITE) {
			uvalue = tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) & ~(0xFF<<16);
			uvalue |= (*value & 0xFF) << 16;
			tmc4330_writeInt(motorToIC(motor), TMC4330_SCALE_VALUES, uvalue);
		}
		break;
	case 129: // todo AP 2: merge AP 129 with AP 133? #1
		// Closed Loop Flag
		if(readWrite == READ) {
			// Read for closed loop flag is implemented as AP 133
		} else if(readWrite == WRITE) {
			//Closed loop on/off
			if(*value)
			{
				*value = tmc4330_calibrateClosedLoop(motorToIC(motor), 1);
				if(!*value)
					errors |= TMC_ERROR_NOT_DONE;
			}
			else
			{
				uvalue 	= tmc4330_readInt(motorToIC(motor), TMC4330_ENC_IN_CONF);
				uvalue 	&= ~(1<<22); // closed loop
				tmc4330_writeInt(motorToIC(motor), TMC4330_ENC_IN_CONF, uvalue);
			}
		}
		break;
	case 132:
		// Measured Encoder Speed
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_V_ENC_RD);
		} else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 133: // todo AP 2: merge AP 129 with AP 133? #2
		// Closed Loop Init Flag
		if(readWrite == READ) {
			uvalue 	= tmc4330_readInt(motorToIC(motor), TMC4330_ENC_IN_CONF);
			*value = (((uvalue >> 22) & 3) == 1) ? 1 : 0;
		} else if(readWrite == WRITE) {
			// Write for closed loop flag is implemented as AP 129
		}
		break;
	case 134:
		// todo AP XML 2: Split this into read-only Encoder deviation and write-only CL_TR_TOLERANCE? Having this as one axis parameter is not that intuitive (LH) #1
		// Encoder deviation
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_ENC_POS_DEV_RD);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_CL_TR_TOLERANCE_WR, *value);
		}
		break;
	case 136:
		// Encoder Velocity Delay
		if(readWrite == READ) {
			*value = (TMC4330.config->shadowRegister[TMC4330_ENC_VMEAN_WAIT_WR] >> 0) &  0xFF;
		} else if(readWrite == WRITE) {
			uvalue = TMC4330.config->shadowRegister[TMC4330_ENC_VMEAN_WAIT_WR];
			uvalue &= ~(0xFF << 0);
			uvalue |= (*value & 0x0F) << 0;
			tmc4330_writeInt(motorToIC(motor), TMC4330_ENC_VMEAN_WAIT_WR, uvalue);
		}
		break;
	case 137:
		// Encoder Velocity Filter
		if(readWrite == READ) {
			*value = (TMC4330.config->shadowRegister[TMC4330_ENC_VMEAN_WAIT_WR] >> 8) &  0xF;
		} else if(readWrite == WRITE) {
			uvalue = TMC4330.config->shadowRegister[TMC4330_ENC_VMEAN_WAIT_WR];
			uvalue &= ~(0xF << 8);
			uvalue |= (*value & 0x0F) << 8;
			tmc4330_writeInt(motorToIC(motor), TMC4330_ENC_VMEAN_WAIT_WR, uvalue);
		}
		break;
	case 138:
		// Filter Update Time
		if(readWrite == READ) {
			*value = (TMC4330.config->shadowRegister[TMC4330_ENC_VMEAN_WAIT_WR] >> 16) &  0xFF;
		} else if(readWrite == WRITE) {
			uvalue = TMC4330.config->shadowRegister[TMC4330_ENC_VMEAN_WAIT_WR];
			uvalue &= ~(0xFF << 16);
			uvalue |= (*value & 0x0FF) << 16;
			tmc4330_writeInt(motorToIC(motor), TMC4330_ENC_VMEAN_WAIT_WR, uvalue);
		}
		break;
	case 200:
		// Boost current
		if(readWrite == READ) {
			*value = (tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) >> 0) & 0xFF;
		} else if(readWrite == WRITE) {
			uvalue = tmc4330_readInt(motorToIC(motor), TMC4330_SCALE_VALUES) & ~(0xFF<<0);
			uvalue |= (*value & 0xFF) << 0;
			tmc4330_writeInt(motorToIC(motor), TMC4330_SCALE_VALUES, uvalue);
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_ENC_POS);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_ENC_POS, *value);
		}
		break;
	case 212:
		// Maximum encoder deviation
		if(readWrite == READ) {
			*value = TMC4330.config->shadowRegister[TMC4330_SCALE_VALUES]; // todo CHECK 3: shouldn't this register be TMC4330_ENC_POS_DEV_TOL_WR like below? (BS) #1
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_ENC_POS_DEV_TOL_WR, *value);
		}
		break;
	case 214:
		// Power Down Delay
		if(readWrite == READ) {
			*value = tmc4330_readInt(motorToIC(motor), TMC4330_STDBY_DELAY);
		} else if(readWrite == WRITE) {
			tmc4330_writeInt(motorToIC(motor), TMC4330_STDBY_DELAY, *value*160000);
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
	tmc4330_writeInt(motorToIC(motor), address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value	= tmc4330_readInt(motorToIC(motor), address);
}

static void periodicJob(uint32_t tick)
{
	tmc4330_periodicJob(&TMC4330, tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
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
		*value = tmc4330_calibrateClosedLoop(motorToIC(motor), 1);
		if(!*value)
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

	tmc4330_reset(&TMC4330);

	return 1;
}

static uint8_t restore()
{
	// Pulse the low-active hardware reset pin
	HAL.IOs->config->setLow(Pins.NRST);
	wait(1);
	HAL.IOs->config->setHigh(Pins.NRST);

	tmc4330_restore(&TMC4330);

	return 1;
}

void TMC4330_init(void)
{
	tmc4330_init(&TMC4330, 0, Evalboards.ch1.config, &tmc4330_defaultRegisterResetState[0]);

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

	TMC4330_SPIChannel = &HAL.SPI->ch1;
	TMC4330_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	Evalboards.ch1.config->state        = CONFIG_RESET;
	Evalboards.ch1.config->configIndex  = 0;
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;

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
	Evalboards.ch1.numberOfMotors       = TMC4330_MOTORS;
	Evalboards.ch1.deInit               = deInit;
};
