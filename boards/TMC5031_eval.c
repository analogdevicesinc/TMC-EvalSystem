/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5031/TMC5031.h"

static SPIChannelTypeDef *TMC5031_SPIChannel;
extern ConfigurationTypeDef *TMC5031_config;

void tmc5031_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC5031_SPIChannel->readWriteArray(data, dataLength);
}

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  182  // VM[V/10] max +10%

#define MOTORS 2
#define DEFAULT_ICID 0

static int32_t vMax[MOTORS] = {0, 0};
static uint8_t vMaxModified[MOTORS] = {true, true};

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
static void checkErrors    (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);


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

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VMAX(motor), abs(velocity));
    vMaxModified[motor] = true;
    if(velocity >= 0)
    {
        tmc5031_writeRegister(DEFAULT_ICID, TMC5031_RAMPMODE(motor), TMC5031_MODE_VELPOS);
    }
    else
    {
        tmc5031_writeRegister(DEFAULT_ICID, TMC5031_RAMPMODE(motor), TMC5031_MODE_VELNEG);
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
    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    if(vMaxModified[motor])
    {
        tmc5031_writeRegister(mDEFAULT_ICIDotor, TMC5031_VMAX(motor), vMax[motor]);
        vMaxModified[motor] = false;
    }
    tmc5031_writeRegister(DEFAULT_ICID, TMC5031_XTARGET(motor), position);
    tmc5031_writeRegister(DEFAULT_ICID, TMC5031_RAMPMODE(motor), TMC5031_MODE_POSITION);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    // determine actual position and add numbers of ticks to move
    *ticks = tmc5031_readRegister(DEFAULT_ICID, TMC5031_XACTUAL(motor)) + *ticks;

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;
    int32_t tempValue;

    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0:
        // Target position
        if(readWrite == READ) {
            *value=tmc5031_readRegister(DEFAULT_ICID, TMC5031_XTARGET(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_XTARGET(motor), *value);
        }
        break;
    case 1:
        // Actual position
        if(readWrite == READ) {
            *value=tmc5031_readRegister(DEFAULT_ICID, TMC5031_XACTUAL(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_XACTUAL(motor), *value);
        }
        break;
    case 2:
        // Target speed
        if(readWrite == READ) {
            *value=tmc5031_readRegister(DEFAULT_ICID, TMC5031_VMAX(motor));
        } else if(readWrite == WRITE) {
            vMaxModified[motor] = true;
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VMAX(motor), abs(*value));
        }
        break;
    case 3:
        // todo CHECK 3: min max actually velocity min and velocity max? (JE) #2
        // Actual speed
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VACTUAL(motor));
            *value = CAST_Sn_TO_S32(*value, 24);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 4:
        // Maximum speed
        if(readWrite == READ) {
            *value=vMax[motor];
        } else if(readWrite == WRITE) {
            vMax[motor] = abs(*value);
            if(tmc5031_readRegister(DEFAULT_ICID, TMC5031_RAMPMODE(motor)) == TMC5031_MODE_POSITION)
                tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VMAX(motor), abs(*value));
        }
        break;
    case 5:
        // Maximum acceleration
        if(readWrite == READ) {
            *value=tmc5031_readRegister(DEFAULT_ICID, TMC5031_AMAX(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_AMAX(motor), *value);
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_IRUN_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_IRUN_FIELD(motor), *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_IHOLD_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_IHOLD_FIELD(motor), *value);
        }
        break;
    case 8:
        // Position reached flag
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_POSITION_REACHED_FIELD(motor));
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 10:
        // Right endstop
        if(readWrite == READ) {
            *value = !tmc5031_field_read(DEFAULT_ICID,TMC5031_STATUS_STOP_R_FIELD(motor));
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 11:
        // Left endstop
        if(readWrite == READ) {
            *value = !tmc5031_field_read(DEFAULT_ICID, TMC5031_STATUS_STOP_L_FIELD(motor));
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 12:
        // Automatic right stop
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_STOP_R_ENABLE_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID,TMC5031_STOP_R_ENABLE_FIELD(motor), !*value);
        }
        break;
    case 13:
        // Automatic left stop
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_STOP_L_ENABLE_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID,  TMC5031_STOP_L_ENABLE_FIELD(motor), (*value)? 0:1);    // todo CHECK 3: Ist 0:1 sorum richtig? Der alte code hatte das so (LH)
        }
        break;
    case 14:
        // SW_MODE Register
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_SWMODE(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_SWMODE(motor), *value);
        }
        break;
    case 15:
        // Acceleration A1
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_A1(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_A1(motor), *value);
        }
        break;
    case 16:
        // Velocity V1
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_V1(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_V1(motor), *value);
        }
        break;
    case 17:
        // Maximum Deceleration
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_DMAX(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_DMAX(motor), *value);
        }
        break;
    case 18:
        // Deceleration D1
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_D1(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_D1(motor), *value);
        }
        break;
    case 19:
        // Velocity VSTART
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VSTART(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VSTART(motor), *value);
        }
        break;
    case 20:
        // Velocity VSTOP
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VSTOP(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VSTOP(motor), *value);
        }
        break;
    case 21:
        // Waiting time after ramp down
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_TZEROWAIT(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_TZEROWAIT(motor), *value);
        }
        break;
    case 22:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VCOOLTHRS(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VCOOLTHRS(motor), *value);
        }
        break;
    case 23:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VHIGH(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VHIGH(motor), *value);
        }
        break;
    case 24:
        // Minimum speed for switching to dcStep
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VDCMIN(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VDCMIN(motor), *value);
        }
        break;
    case 28:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_VHIGHFS_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_VHIGHFS_FIELD(motor), *value);
        }
        break;
    case 29:
        if(readWrite == READ) {
            *value = TMC5031_config->shadowRegister[TMC5031_VACTUAL(motor)];
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 256 >> tmc5031_field_read(DEFAULT_ICID, TMC5031_MRES_FIELD(motor));
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
                tempValue = tmc5031_readRegister(DEFAULT_ICID, TMC5031_CHOPCONF(motor));
                tempValue &= ~(0x0F<<24);
                tempValue |= (*value & 0xF) << 24;
                tmc5031_writeRegister(DEFAULT_ICID, TMC5031_CHOPCONF(motor),tempValue);
                tmc5031_field_write(DEFAULT_ICID, TMC5031_MRES_FIELD(motor), *value);
            }
            //else TMCL.reply->Status = REPLY_INVALID_VALUE;
        }
        break;
    case 162:
        // Chopper blank time
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_TBL_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_TBL_FIELD(motor), *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_CHM_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_CHM_FIELD(motor), *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_DISFDCC_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_DISFDCC_FIELD(motor), *value);
        }
        break;
    case 165:    // todo CHECK AP 2: Is there a particular reason why HEND and TFD are grouped to one axis parameter and sine offset and HSTART to another, even though HEND and sine offset share a bitfield, and HSTART and TFD another?
        // Chopper hysteresis end / fast decay time
        tempValue = tmc5031_readRegister(DEFAULT_ICID, TMC5031_CHOPCONF(motor));
        if(readWrite == READ) {
            if(tempValue & TMC5031_CHM_MASK) // Chopper Hysteresis
            {
                *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_HEND_FIELD(motor));
            }
            else // fast decay time
            {
                *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_TFD_2__0__FIELD(motor));
                if(tempValue & TMC5031_TFD___MASK) // Add MSB of fast decay time to total value
                    *value |= 1<<3;
            }
        } else if(readWrite == WRITE) {
            if(tempValue & TMC5031_CHM_MASK) // Chopper Hysteresis
            {
                tmc5031_field_write(DEFAULT_ICID, TMC5031_HEND_FIELD(motor), *value);
            }
            else // fast decay time
            {
                if(*value & (1<<3)) // check MSB of fast decay time
                    tempValue |= TMC5031_TFD___MASK;
                else
                    tempValue &= ~TMC5031_TFD___MASK;

                tmc5031_field_write(DEFAULT_ICID, TMC5031_TFD_2__0__FIELD(motor), *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        tempValue = tmc5031_readRegister(motor, TMC5031_CHOPCONF(motor));
        if(readWrite == READ) {
            if(tempValue & TMC5031_CHM_MASK) // Chopper hysteresis start
            {
                *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_HSTRT_FIELD(motor));
            }
            else // sine wave offset
            {
                *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_OFFSET_FIELD(motor));
            }
        } else if(readWrite == WRITE) {
            if(tempValue & TMC5031_CHM_MASK) // Chopper hysteresis start
            {
                tmc5031_field_write(DEFAULT_ICID, TMC5031_HSTRT_FIELD(motor), *value);
            }
            else // sine wave offset
            {
                tmc5031_field_write(DEFAULT_ICID, TMC5031_OFFSET_FIELD(motor), *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_TOFF_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_TOFF_FIELD(motor), *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SEIMIN_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SEIMIN_FIELD(motor), *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SEDN_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SEDN_FIELD(motor), *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SEMAX_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SEMAX_FIELD(motor), *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SEUP_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SEUP_FIELD(motor), *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID,  TMC5031_SEMIN_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SEMIN_FIELD(motor), *value);
        }
        break;
    case 173:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SFILT_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SFILT_FIELD(motor), *value);
        }
        break;
    case 174:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SGT_FIELD(motor));
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_SGT_FIELD(motor), *value);
        }
        break;
    case 179:
        // VSense
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_VSENSE_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID,  TMC5031_VSENSE_FIELD(motor), *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID,  TMC5031_CS_ACTUAL_FIELD(motor));
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 181:
        // reset stall
        tempValue = tmc5031_readRegister(DEFAULT_ICID, TMC5031_SWMODE(motor));
        if(readWrite == READ)
        {
            if(tempValue & TMC5031_SG_STOP_MASK)
            {
                *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VCOOLTHRS(motor));
            }
            else
                *value = 0;
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VCOOLTHRS(motor), *value);

            tmc5031_field_write(DEFAULT_ICID, TMC5031_SG_STOP_FIELD(motor), (*value) ? 1:0);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_VCOOLTHRS(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_VCOOLTHRS(motor),*value);
        }
        break;
    case 184:
        // Random TOff mode
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_RNDTF_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID, TMC5031_RNDTF_FIELD(motor), *value);
        }
        break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID, TMC5031_SYNC_FIELD(motor));
        } else if(readWrite == WRITE) {
            tmc5031_field_write(DEFAULT_ICID,  TMC5031_SYNC_FIELD(motor), *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc5031_field_read(DEFAULT_ICID,  TMC5031_SG_RESULT_FIELD(motor));
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 209:
        // Encoder position
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_XENC(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_XENC(motor),*value);
        }
        break;
    case 210:
        // Encoder Resolution
        if(readWrite == READ) {
            *value = tmc5031_readRegister(DEFAULT_ICID, TMC5031_ENC_CONST(motor));
        } else if(readWrite == WRITE) {
            tmc5031_writeRegister(DEFAULT_ICID, TMC5031_ENC_CONST(motor),*value);
        }
        break;
    case 211:
        if(readWrite == READ) {
            // encoder enable
            switch(motor)
            {
            case 0:
                tempValue = tmc5031_readRegister(motor, TMC5031_GCONF);
                tempValue &= (1<<3) | (1<<4);
                *value = (tempValue == (1<<4))? 1 : 0;
                break;
            case 1:
                tempValue = tmc5031_readRegister(motor, TMC5031_GCONF);
                tempValue &= (1<<5) | (1<<6);
                *value = (tempValue == ((1<<5) | (0<<6)))? 1 : 0;
                break;
            }
        } else if(readWrite == WRITE) {
            // encoder enable
            switch(motor)
            {
            case 0:
                tempValue = tmc5031_readRegister(DEFAULT_ICID, TMC5031_GCONF);
                tempValue = (*value)? tempValue & ~(1<<3) : tempValue | (1<<3);  // poscmp_enable -> ENCODER1 A,B
                tempValue = (*value)? tempValue | (1<<4) : tempValue &~ (1<<4);  // enc1_refsel -> ENCODER1 N to REFL1
                tmc5031_writeRegister(DEFAULT_ICID, TMC5031_GCONF, tempValue);
                break;
            case 1:        // enable ENCODER2 - disable REF
                tempValue = tmc5031_readRegister(DEFAULT_ICID, TMC5031_GCONF);
                tempValue = (*value) ? tempValue | (1<<5) : tempValue &~ (1<<5);  // enc2_enable -> ENCODER2 A,B to REFR1,2
                tempValue = (*value) ? tempValue &~ (1<<6) : tempValue | (1<<6);  // enc2_refse2 -> ENCODER1 N to REFL2
                tmc5031_writeRegister(DEFAULT_ICID, TMC5031_GCONF, tempValue);
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
    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    *value = TMC5031_config->shadowRegister[TMC5031_VACTUAL(motor)];

    return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc5031_writeRegister(DEFAULT_ICID, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc5031_readRegister(DEFAULT_ICID, (uint8_t) address);
}

static void periodicJob(uint32_t tick)
{
    for(uint8_t motor = 0; motor < MOTORS; motor++)
    {
        tmc5031_periodicJob(motor, tick, &TMC5031, TMC5031_config);
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
    case 0:        // enable Single Wire Mode
        if(*value)
            HAL.IOs->config->setHigh(Pins.SWSEL);
        else
            HAL.IOs->config->setLow(Pins.SWSEL);
        break;
    case 1:        // read interrupt pin INT
        *value = (HAL.IOs->config->isHigh(Pins.INT_ENCA))? 1 : 0;
        break;
    case 2:        // read position compare pin PP
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
    for(uint8_t motor = 0; motor < MOTORS; motor++)
        if(tmc5031_readRegister(DEFAULT_ICID, TMC5031_VACTUAL(motor)) != 0)
            return 0;

    return tmc5031_reset(TMC5031_config);
}

static uint8_t restore()
{
    return tmc5031_restore(TMC5031_config);
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

void TMC5031_init(void)
{
    tmc5031_initConfig(&TMC5031);

    Pins.DRV_ENN   = &HAL.IOs->pins->DIO0;
    Pins.INT_ENCA  = &HAL.IOs->pins->DIO5;
    Pins.PP_ENCB   = &HAL.IOs->pins->DIO6;
    Pins.SWSEL     = &HAL.IOs->pins->DIO16;
    Pins.SWIOP1    = &HAL.IOs->pins->DIO17;
    Pins.SWIOP2    = &HAL.IOs->pins->DIO18;
    Pins.SWION     = &HAL.IOs->pins->DIO19;

    HAL.IOs->config->toOutput(Pins.DRV_ENN);
    HAL.IOs->config->toOutput(Pins.SWSEL);

    HAL.IOs->config->setLow(Pins.SWSEL);

    HAL.IOs->config->toInput(Pins.INT_ENCA);
    HAL.IOs->config->toInput(Pins.PP_ENCB);
    HAL.IOs->config->toInput(Pins.SWION);
    HAL.IOs->config->toInput(Pins.SWIOP1);
    HAL.IOs->config->toInput(Pins.SWIOP2);

    TMC5031_SPIChannel = &HAL.SPI->ch1;
    TMC5031_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

    TMC5031_config = Evalboards.ch1.config;

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
    Evalboards.ch1.numberOfMotors       = MOTORS;
    Evalboards.ch1.VMMin                = VM_MIN;
    Evalboards.ch1.VMMax                = VM_MAX;
    Evalboards.ch1.deInit               = deInit;

    enableDriver(DRIVER_ENABLE);
};
