/*******************************************************************************
* Copyright © 2026 Analog Devices, Inc.
*******************************************************************************/


#include "tmc/StepDir.h"
#include "Board.h"
#include "tmc/ic/TMC3241/TMC3241.h"
#include "tmc/RAMDebug.h"
#include "hal/Timer.h"

#define VM_MIN         45   // VM[V/10] min
#define VM_MAX         650  // VM[V/10] max

#define TMC3241_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)
#define DEFAULT_ICID 0

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
#define TMC3241_RAMDEBUG_TIMER TIMER_CHANNEL_1
#elif defined(LandungsbrueckeV3)
#define TMC3241_RAMDEBUG_TIMER TIMER_CHANNEL_2
#endif

// TMC3241: TMC3241TypeDef struct, which represents one IC
typedef struct
{
    ConfigurationTypeDef *config;
    int32_t velocity, oldX;
    uint32_t oldTick;
    uint8_t slaveAddress;
} TMC3241TypeDef;
static TMC3241TypeDef TMC3241;

typedef struct
{
    IOPinTypeDef *N_DRN_EN;
    IOPinTypeDef *N_SLEEP;
    IOPinTypeDef *DIAG0;
    IOPinTypeDef *DIAG1;
    IOPinTypeDef *STEP_INT;
    IOPinTypeDef *DIR_INT;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t vmax_position;
//static uint32_t vMax         = 1;
static bool internalRamp;

static SPIChannelTypeDef *TMC3241_SPIChannel;

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
static uint8_t reset();
static void enableDriver(DriverState state);

void tmc3241_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
	UNUSED(icID);
	TMC3241_SPIChannel->readWriteArray(data, dataLength);
}

static void writeConfiguration(uint32_t tick)
{
    uint8_t *ptr = &TMC3241.config->configIndex;
    static int32_t prevTick;
    int32_t readData;

    switch(*ptr){
    case 0:
        // Set PLL register to enable all the clocks and external oscillator
        tmc3241_writeRegister(DEFAULT_ICID, TMC3241_PLL, 0x65FF);
        (*ptr)++;
        prevTick = tick;
        break;
    case 1:
        if(tick - prevTick >= 1000)
        {
            // Clear the all the error flags by the PLL in [15:12]
            tmc3241_writeRegister(DEFAULT_ICID, TMC3241_PLL, 0xF5FE);
            // Read PLL back
            readData = tmc3241_readRegister(DEFAULT_ICID, TMC3241_PLL);
            readData = tmc3241_readRegister(DEFAULT_ICID, TMC3241_PLL);
            if((readData & 0xF000) != 0)
            {
                *ptr = 0;
                break;
            }
            (*ptr)++;
        }
        break;
    case 2:
        //Reading ChopConf register
        readData = tmc3241_readRegister(DEFAULT_ICID, TMC3241_CHOPCONF);
        // Set TOFF field to enable the driver
        tmc3241_writeRegister(DEFAULT_ICID, TMC3241_CHOPCONF, (readData & 0xFFFFFFF0) | 0x3);
        (*ptr)++;
        break;
    case 3:
        // Write to GSTAT register to clear the flags
        tmc3241_writeRegister(DEFAULT_ICID, TMC3241_GSTAT, 0x3F);

        // Set MRES to 4 and INTPOL to 1 in ChopConf register
        tmc3241_writeRegister(DEFAULT_ICID, TMC3241_CHOPCONF, 0x10410153);

        TMC3241.config->state = CONFIG_READY;
        *ptr = 0;
        break;
    }
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if (motor >= TMC3241_MOTORS)
		return TMC_ERROR_MOTOR;
    internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;
    if(internalRamp)
    {
        // Set absolute velocity
        tmc3241_writeRegister(0, TMC3241_VMAX, abs(velocity));
        // Set direction
        tmc3241_writeRegister(0, TMC3241_RAMPMODE, (velocity >= 0) ? TMC3241_MODE_VELPOS : TMC3241_MODE_VELNEG);
    }
    else
    {
        StepDir_rotate(motor, velocity);
    }

	return TMC_ERROR_NONE;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	return rotate(motor, velocity);

}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	return rotate(motor, -velocity);

}

static uint32_t stop(uint8_t motor)
{
	return rotate(motor, 0);
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	if (motor >= TMC3241_MOTORS)
		return TMC_ERROR_MOTOR;
    internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;
    if(internalRamp)
    {
        tmc3241_writeRegister(DEFAULT_MOTOR, TMC3241_RAMPMODE, TMC3241_MODE_POSITION);

        // VMAX also holds the target velocity in velocity mode.
        // Re-write the position mode maximum velocity here.
        tmc3241_writeRegister(DEFAULT_MOTOR, TMC3241_VMAX, vmax_position);

        tmc3241_writeRegister(DEFAULT_MOTOR, TMC3241_XTARGET, position);
    }
    else
    {
        StepDir_moveTo(motor, position);
    }

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if (motor >= TMC3241_MOTORS)
		return TMC_ERROR_MOTOR;
    internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;
    if(internalRamp)
    {
        // determine actual position and add numbers of ticks to move
        *ticks += tmc3241_readRegister(DEFAULT_MOTOR, TMC3241_XACTUAL);
    }
    else
    {
        // determine actual position and add numbers of ticks to move
        *ticks += StepDir_getActualPosition(motor);
    }

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t buffer;
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC3241_MOTORS)
		return TMC_ERROR_MOTOR;

	int32_t tempValue;

	switch(type)
	{
	case 0:
		// Target position
        internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;

		if (readWrite == READ) {
            if(internalRamp)
                readRegister(motor, TMC3241_XTARGET, value);
            else
                *value = StepDir_getTargetPosition(motor);

		} else if (readWrite == WRITE) {
            if(internalRamp)
                writeRegister(motor, TMC3241_XTARGET, *value);
            else
                StepDir_moveTo(motor, *value);
		}
		break;
	case 1:
		// Actual position
        internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;

		if (readWrite == READ) {
            if(internalRamp)
                readRegister(motor, TMC3241_XACTUAL, value);
            else
                *value = StepDir_getActualPosition(motor);

		} else if (readWrite == WRITE) {
            if(internalRamp)
                writeRegister(motor, TMC3241_XACTUAL, *value);
            else
                StepDir_setActualPosition(motor, *value);
		}
		break;
	case 2:
		// Target speed
        internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;

		if (readWrite == READ) {
            if(internalRamp)
                readRegister(motor, TMC3241_VMAX, value);
            else
                *value = StepDir_getTargetVelocity(motor);
		} else if (readWrite == WRITE) {
            if(internalRamp)
                writeRegister(motor, TMC3241_VMAX, *value);
            else
                StepDir_rotate(motor, *value);
		}
		break;
	case 3:
		// Actual speed
        internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;

		if (readWrite == READ) {
		    if(internalRamp)
		    {
		        readRegister(motor, TMC3241_VACTUAL, value);
		        *value = CAST_Sn_TO_S32(*value, 24);
		    }
		    else
		    {
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
		                                    / (int64_t)tmc3241_fieldRead(motor, TMC3241_TSTEP_FIELD));
		            *value = (abs(tempValue) < 20) ? 0 : tempValue;
		            break;
		        }
		    }

		} else if (readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
        internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;

		if (readWrite == READ) {
		    if(internalRamp)
	            *value = vmax_position;
		    else
		        *value = StepDir_getVelocityMax(motor);
		} else if (readWrite == WRITE) {
            if(internalRamp)
            {
                vmax_position = abs(*value);
                readRegister(motor, TMC3241_RAMPMODE, (int32_t *)&buffer);
                if(buffer == TMC3241_MODE_POSITION)
                    writeRegister(motor, TMC3241_VMAX, abs(*value));
            }
            else
                StepDir_setVelocityMax(motor, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
        internalRamp = tmc3241_fieldRead(motor, TMC3241_THREEPHASE_MCC_FIELD) == 1;

		if (readWrite == READ) {
            if(internalRamp)
                readRegister(motor, TMC3241_AMAX, value);
            else
                *value = StepDir_getAcceleration(motor);
		} else if (readWrite == WRITE) {
            if(internalRamp)
                writeRegister(motor, TMC3241_AMAX, *value);
            else
                StepDir_setAcceleration(motor, *value);
		}
		break;
	case 6:
		// Maximum current
		if (readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_IRUN_FIELD);
		} else if (readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_IRUN_FIELD, *value);
		}
		break;
	case 7:
		// Standby current
		if (readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_IHOLD_FIELD);
		} else if (readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_IHOLD_FIELD, *value);
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
    case 15:
        // Maximum Deceleration if internal ramp enabled
        if(readWrite == READ) {
            readRegister(motor, TMC3241_DMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_DMAX, *value);
        }
        break;
//	case 14:
//		// SW_MODE Register
//		if(readWrite == READ) {
//			readRegister(motor, TMC3241_SW_MODE, value);
//		} else if(readWrite == WRITE) {
//			writeRegister(motor, TMC3241_SW_MODE, *value);
//		}
//		break;
//	case 26:
//		// Speed threshold for high speed mode
//		if(readWrite == READ) {
//			readRegister(motor, TMC3241_THIGH, (int32_t *)&buffer);
//			*value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
//		} else if(readWrite == WRITE) {
//			*value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
//			writeRegister(motor, TMC3241_THIGH, *value);
//		}
//		break;
//	case 27:
//		// Minimum speed for switching to dcStep
//		if(readWrite == READ) {
//			readRegister(motor, TMC3241_TUDCSTEP, value);
//		} else if(readWrite == WRITE) {
//			writeRegister(motor, TMC3241_TUDCSTEP, *value);
//		}
//		break;
    case 16:
        // Velocity VSTART
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC3241_VSTART, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC3241_VSTART, *value);
        }
        break;
    case 17:
        // Acceleration A1
        if(readWrite == READ) {
            readRegister(motor, TMC3241_A1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_A1, *value);
        }
        break;
    case 18:
        // Velocity V1
        if(readWrite == READ) {
            readRegister(motor, TMC3241_V1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_V1, *value);
        }
        break;
    case 19:
        // Deceleration D1
        if(readWrite == READ) {
            readRegister(motor, TMC3241_D1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_D1, *value);
        }
        break;
    case 20:
        // Velocity VSTOP
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC3241_VSTOP, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC3241_VSTOP, *value);
        }
        break;
    case 21:
        // Waiting time after ramp down
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC3241_TZEROWAIT, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC3241_TZEROWAIT, *value);
        }
        break;
    case 22:
        // Velocity V2
        if(readWrite == READ) {
            readRegister(motor, TMC3241_V2, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_V2, *value);
        }
        break;
    case 23:
        // Deceleration D2
        if(readWrite == READ) {
            readRegister(motor, TMC3241_D2, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_D2, *value);
        }
        break;
    case 24:
        // Acceleration A2
        if(readWrite == READ) {
            readRegister(motor, TMC3241_A2, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC3241_A2, *value);
        }
        break;
    case 25:
        // TVMAX
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC3241_TVMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC3241_TVMAX, *value);
        }
        break;
	case 30:
        if(readWrite == READ) {
            tempValue = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)tmc3241_fieldRead(motor, TMC3241_TSTEP_FIELD));
            *value = (abs(tempValue) < 20) ? 0 : tempValue;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
		break;
//	case 31:
//		// Current P
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_CUR_P_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_CUR_P_FIELD, *value);
//		}
//		break;
//	case 32:
//		// Current I
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_CUR_I_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_CUR_I_FIELD, *value);
//		}
//		break;
//	case 37:
//		// Current limit
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_CUR_PI_LIMIT_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_CUR_PI_LIMIT_FIELD, *value);
//		}
//		break;
//	case 40:
//		// Measured current amplitude
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_AMPL_MEAS_FIELD);
//
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_AMPL_MEAS_FIELD, *value);
//
//		}
//		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 0x100 >> tmc3241_fieldRead(motor, TMC3241_MRES_FIELD);
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
				tmc3241_fieldWrite(motor, TMC3241_MRES_FIELD, *value);
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
			*value = tmc3241_fieldRead(motor, TMC3241_TBL_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_TBL_FIELD, *value);
		}
		break;
	case 163:
		// Constant TOff Mode
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_CHM_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_CHM_FIELD, *value);
		}
		break;
//	case 164:
//		// Disable fast decay comparator
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_DISFDCC_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_DISFDCC_FIELD, *value);
//		}
//		break;
//	case 165:
//		// Chopper hysteresis end / fast decay time
//		readRegister(motor, TMC3241_CHOPCONF, (int32_t *)&buffer);
//		if(readWrite == READ) {
//			if(buffer & (1 << TMC3241_CHM_SHIFT))
//			{
//				*value = (buffer >> TMC3241_HEND_OFFSET_SHIFT) & TMC3241_HEND_OFFSET_MASK;
//			}
//			else
//			{
//				*value = (buffer >> TMC3241_HSTRT_TFD210_SHIFT) & TMC3241_HSTRT_TFD210_MASK;
//				if(buffer & TMC3241_FD3_SHIFT)
//					*value |= 1<<3; // MSB wird zu value dazugefügt
//			}
//		} else if(readWrite == WRITE) {
//			if(buffer & (1<<14))
//			{
//				tmc3241_fieldWrite(motor, TMC3241_HEND_OFFSET_FIELD, *value);
//			}
//			else
//			{
//				tmc3241_fieldWrite(motor, TMC3241_FD3_FIELD, (*value & (1<<3))); // MSB wird zu value dazugefügt
//				tmc3241_fieldWrite(motor, TMC3241_HSTRT_TFD210_FIELD, *value);
//			}
//		}
//		break;
//	case 166:
//		// Chopper hysteresis start / sine wave offset
//		readRegister(motor, TMC3241_CHOPCONF, (int32_t *)&buffer);
//		if(readWrite == READ) {
//			if(buffer & (1 << TMC3241_CHM_SHIFT))
//			{
//				*value = (buffer >> TMC3241_HSTRT_TFD210_SHIFT) & TMC3241_HSTRT_TFD210_MASK;
//			}
//			else
//			{
//				*value = (buffer >> TMC3241_HEND_OFFSET_SHIFT) & TMC3241_HEND_OFFSET_MASK;
//				if(buffer & (1 << TMC3241_FD3_SHIFT))
//					*value |= 1<<3; // MSB wird zu value dazugefügt
//			}
//		} else if(readWrite == WRITE) {
//			if(buffer & (1 << TMC3241_CHM_SHIFT))
//			{
//				tmc3241_fieldWrite(motor, TMC3241_HSTRT_TFD210_FIELD, *value);
//			}
//			else
//			{
//				tmc3241_fieldWrite(motor, TMC3241_HEND_OFFSET_FIELD, *value);
//			}
//		}
//		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_TOFF_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_TOFF_FIELD, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SEIMIN_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SEIMIN_FIELD, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SEDN_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SEDN_FIELD, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SEMAX_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SEMAX_FIELD, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SEUP_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SEUP_FIELD, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SEMIN_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SEMIN_FIELD, *value);
		}
		break;
//	case 173:
//		// stallGuard4 filter enable
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_SGP_FILT_EN_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_SGP_FILT_EN_FIELD, *value);
//		}
//		break;
//	case 174:
//		// stallGuard4 threshold
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_SGP_THRS_FIELD);
//			*value = CAST_Sn_TO_S32(*value, 7);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_SGP_THRS_FIELD, *value);
//		}
//		break;
	case 175:
		// stallGuard2 filter enable
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SFILT_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SFILT_FIELD, *value);
		}
		break;
	case 176:
		// stallGuard2 threshold
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SGT_FIELD);
			*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_SGT_FIELD, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_CS_ACTUAL_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		if(readWrite == READ) {
			*value = StepDir_getStallGuardThreshold(motor);
		} else if(readWrite == WRITE) {
			// Store the threshold value in the internal StepDir generator
			StepDir_setStallGuardThreshold(motor, *value);

			// Convert the value for the TCOOLTHRS register
			// The IC only sends out Stallguard errors while TCOOLTHRS >= TSTEP >= TPWMTHRS
			// The TSTEP value is measured. To prevent measurement inaccuracies hiding
			// a stall signal, we decrease the needed velocity by roughly 12% before converting it.
			*value -= (*value) >> 3;
			if (*value)
			{
				*value = MIN(0x000FFFFF, (1<<24) / (*value));
			}
			else
			{
				*value = 0x000FFFFF;
			}
			writeRegister(motor, TMC3241_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			readRegister(motor, TMC3241_TCOOLTHRS, (int32_t *)&buffer);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
			writeRegister(motor, TMC3241_TCOOLTHRS, *value);
		}
		break;
	case 185:
		// Chopper synchronization
		readRegister(motor, TMC3241_CHOPCONF, (int32_t *)&buffer);
		if(readWrite == READ) {
			*value = (buffer >> 20) & 0x0F;
		} else if(readWrite == WRITE) {
			buffer &= ~(0x0F<<20);
			buffer |= (*value & 0x0F) << 20;
			writeRegister(motor, TMC3241_CHOPCONF, buffer);
		}
		break;
//	case 186:
//		// PWM threshold speed
//		if(readWrite == READ) {
//			readRegister(motor, TMC3241_TPWMTHRS, (int32_t *)&buffer);
//			*value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
//		} else if(readWrite == WRITE) {
//			*value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
//			writeRegister(motor, TMC3241_TPWMTHRS, *value);
//		}
//		break;
//	case 191:
//		// PWM frequency
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_PWM_FREQ_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_PWM_FREQ_FIELD, *value);
//		}
//		break;
	case 194:
		// MSCNT
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_MSCNT_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
		/*	case 195:
		// MEAS_SD_EN
		if(readWrite == READ) {
		 *value = TMC3241_tmc3241_fieldRead(motorToIC(motor), TMC3241_PWMCONF, TMC3241_PWMCONF_SD_ON_MEAS_MASK, TMC3241_PWMCONF_SD_ON_MEAS_SHIFT);
		} else if(readWrite == WRITE) {
			TMC3241_tmc3241_fieldWrite(motorToIC(motor), TMC3241_PWMCONF, TMC3241_PWMCONF_SD_ON_MEAS_MASK, TMC3241_PWMCONF_SD_ON_MEAS_SHIFT, *value);
		}
		break;*/
//	case 204:
//		// Freewheeling mode
//		if(readWrite == READ) {
//			*value = tmc3241_fieldRead(motor, TMC3241_FREEWHEEL_FIELD);
//		} else if(readWrite == WRITE) {
//			tmc3241_fieldWrite(motor, TMC3241_FREEWHEEL_FIELD, *value);
//		}
//		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_SG_RESULT_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// Encoder position
		if(readWrite == READ) {
			readRegister(motor, TMC3241_X_ENC, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC3241_X_ENC, *value);
		}
		break;
	case 210:
		// Encoder Resolution
		if(readWrite == READ) {
			readRegister(motor, TMC3241_ENC_CONST, value);
		} else if(readWrite == WRITE) {
			writeRegister(motor, TMC3241_ENC_CONST, *value);
		}
		break;
	case 212:
		// Current range from DRV_CONF reg
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_CURRENT_RANGE_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_CURRENT_RANGE_FIELD, *value);
		}
		break;
	case 213:
		// ADCTemperatur
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_ADC_TEMP_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// ADCSupply
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_ADC_VSUPPLY_FIELD);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 216:
		// Overvoltage Limit ADC value
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_OVERVOLTAGE_VTH_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_OVERVOLTAGE_VTH_FIELD, *value);
		}
		break;
	case 217:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			*value = tmc3241_fieldRead(motor, TMC3241_OVERTEMPPREWARNING_VTH_FIELD);
		} else if(readWrite == WRITE) {
			tmc3241_fieldWrite(motor, TMC3241_OVERTEMPPREWARNING_VTH_FIELD, *value);
		}
		break;
	case 218:
		// ADCTemperatur Converted
		if(readWrite == READ) {

			int32_t adc = tmc3241_fieldRead(motor, TMC3241_ADC_TEMP_FIELD);
			*value = (int32_t)10*(adc-2038)/77;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 220:
		// ADCSupply
		if(readWrite == READ) {
			int32_t adc = tmc3241_fieldRead(motor, TMC3241_ADC_VSUPPLY_FIELD);
			*value = (int32_t)32*3052*adc/10000;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 221:
		// Overvoltage Limit converted
		if(readWrite == READ) {
			int32_t val = tmc3241_fieldRead(motor, TMC3241_OVERVOLTAGE_VTH_FIELD);
			*value = (int32_t)32*3052*val/10000;
		} else if(readWrite == WRITE) {
			int32_t val = (int32_t)(*value*10000/(3052*32));
			tmc3241_fieldWrite(motor, TMC3241_OVERVOLTAGE_VTH_FIELD, val);
		}
		break;
	case 222:
		// Overtemperature Warning Limit
		if(readWrite == READ) {
			int32_t temp = tmc3241_fieldRead(motor, TMC3241_OVERTEMPPREWARNING_VTH_FIELD);
			*value = (int32_t)(temp-2038)/7.7;
		} else if(readWrite == WRITE) {
			float valf  = *value*7.7;
			int32_t val = (int32_t)valf;
			val = val+2038;
			tmc3241_fieldWrite(motor, TMC3241_OVERTEMPPREWARNING_VTH_FIELD, val);
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
	if(motor >= TMC3241_MOTORS)
		return TMC_ERROR_MOTOR;

	switch (motor) {
	case 0:
		*value = StepDir_getActualVelocity(0);
		break;
	default:
		return TMC_ERROR_MOTOR;
		break;
	}
	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
	UNUSED(motor);
	tmc3241_writeRegister(DEFAULT_MOTOR, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc3241_readRegister(DEFAULT_MOTOR, (uint8_t) address);
}

static void periodicJob(uint32_t tick)
{
    if(TMC3241.config->state != CONFIG_READY)
    {
        writeConfiguration(tick);
    }
    StepDir_periodicJob(0);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch2.errors = 0;
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

static void deInit(void)
{
	HAL.IOs->config->setHigh(Pins.N_DRN_EN);
	HAL.IOs->config->reset(Pins.N_SLEEP);
	StepDir_deInit();
}

static uint8_t reset()
{
	if (StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
		return 0;

    if(TMC3241.config->state != CONFIG_READY)
        return false;

    TMC3241.config->state        = CONFIG_RESET;
    TMC3241.config->configIndex  = 0;

    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.STEP_INT, Pins.DIR_INT, Pins.DIAG1);
    StepDir_setVelocityMax(0, 100000);
    StepDir_setAcceleration(0, 25000);
    enableDriver(DRIVER_ENABLE);

    return 1;
}

static uint8_t restore()
{
    if(TMC3241.config->state != CONFIG_READY)
        return false;

    TMC3241.config->state        = CONFIG_RESTORE;
    TMC3241.config->configIndex  = 0;

    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.STEP_INT, Pins.DIR_INT, Pins.DIAG1);
    StepDir_setVelocityMax(0, 100000);
    StepDir_setAcceleration(0, 25000);

    return 1;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.N_DRN_EN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
	{
		HAL.IOs->config->setLow(Pins.N_DRN_EN);
		//Set NOSD bit by default
        tmc3241_fieldWrite(0, TMC3241_NOSD_FIELD, 1);
	}
}

static void timer_overflow(timer_channel channel)
{
	UNUSED(channel);

	// RAMDebug
	debug_nextProcess();
}

void TMC3241_init(void)
{
	Pins.N_DRN_EN = &HAL.IOs->pins->DIO0;
	Pins.N_SLEEP = &HAL.IOs->pins->DIO8;
	Pins.STEP_INT = &HAL.IOs->pins->DIO6;
	Pins.DIR_INT = &HAL.IOs->pins->DIO7;

	HAL.IOs->config->toOutput(Pins.N_DRN_EN);
	HAL.IOs->config->toOutput(Pins.N_SLEEP);
	HAL.IOs->config->toOutput(Pins.STEP_INT);
	HAL.IOs->config->toOutput(Pins.DIR_INT);

	HAL.IOs->config->setHigh(Pins.N_SLEEP);
	HAL.IOs->config->setHigh(Pins.N_DRN_EN);


	TMC3241_SPIChannel = &HAL.SPI->ch2;
	TMC3241_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_RESET;

    TMC3241.velocity  = 0;
    TMC3241.oldTick   = 0;
    TMC3241.oldX      = 0;

    TMC3241.config               = Evalboards.ch2.config;
    TMC3241.config->channel = 0;    //channel: The channel index, which will be sent back in the SPI callback
    TMC3241.config->configIndex  = 0;
    TMC3241.config->state        = CONFIG_READY;

	// Initialize the software StepDir generator
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP_INT, Pins.DIR_INT, Pins.DIAG1);
	StepDir_setVelocityMax(0, 100000);
	StepDir_setAcceleration(0, 25000);

	vmax_position = 0;

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
	Evalboards.ch2.GIO                  = GIO;
	Evalboards.ch2.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = TMC3241_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;

	Timer.overflow_callback = timer_overflow;
	Timer.init();
	Timer.setFrequency(TMC3241_RAMDEBUG_TIMER, 10000);
	debug_updateFrequency(10000);
	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
