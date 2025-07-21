/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "../TMC-API/tmc/ic/TMC5222/TMC5222.h"
#include "Board.h"


#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)
#define VM_MIN         19   // VM[V/10] min, It should be 2.1V but ADC measurement is not precise, so decreased the voltage limit by 10% below the original value.
#define VM_MAX         176  // VM[V/10] max, It should be 16V but ADC measurement is not precise, so increased the voltage limit by 10% above the original value.
#define DEFAULT_ICID  0

// Typedefs
typedef struct
{
    ConfigurationTypeDef *config;
    int32_t oldX;
    int32_t velocity;
    uint32_t oldTick;
} TMC5222TypeDef;
static TMC5222TypeDef TMC5222;

typedef struct
{
    IOPinTypeDef  *SEL_I2CN;
    IOPinTypeDef  *CLK_LB;
    IOPinTypeDef  *DIAG0_LB;
    IOPinTypeDef  *DIAG1_LB;
    IOPinTypeDef  *SLEEPN_LB;
    IOPinTypeDef  *REFLN_LB;
    IOPinTypeDef  *REFRN_LB;
    IOPinTypeDef  *DRV_EN_LB;
} PinsTypeDef;
static PinsTypeDef Pins;

static TMC5222BusType activeBus = IC_BUS_IIC;
static IICTypeDef *TMC5222_IIC;
static bool vMaxModified = false;
static uint32_t vmax_position[TMC5222_MOTORS];
static bool noRegResetnSLEEP = false;
static bool drvError = true;
static uint32_t nSLEEPTick;
static uint8_t deviceAddress = 0xC0;

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
static void init_comm(TMC5222BusType mode);
static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint8_t reset();
static void enableDriver(DriverState state);

static void delayBlocking(uint32_t microseconds)
{

    uint32_t startTime = systick_getMicrosecondTick();
    while(timeDiff(systick_getMicrosecondTick(), startTime)<=microseconds);

}

bool tmc5222_readWriteIIC(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
   if(IICMasterWriteRead(data[0],&data[1],writeLength,&data[2],readLength))//Device address = 0b1100000W/R
       return true;

    return false;
}

TMC5222BusType tmc5222_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint8_t tmc5222_getDeviceAddress(uint16_t icID)
{
    UNUSED(icID);

    return deviceAddress;
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    if(motor >= TMC5222_MOTORS)
        return TMC_ERROR_MOTOR;

    tmc5222_writeRegister(DEFAULT_ICID, TMC5222_RGR_VMAX, abs(velocity));
    tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_RAMPMODE_FIELD,  (velocity >= 0) ? TMC5222_MODE_VELPOS : TMC5222_MODE_VELNEG);

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
    if(motor >= TMC5222_MOTORS)
        return TMC_ERROR_MOTOR;

    tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_RAMPMODE_FIELD, TMC5222_MODE_POSITION);
    tmc5222_writeRegister(DEFAULT_ICID, TMC5222_RGR_VMAX, vmax_position[motor]);
    tmc5222_writeRegister(DEFAULT_ICID, TMC5222_RGR_XTARGET, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    // determine actual position and add numbers of ticks to move
    *ticks += tmc5222_readRegister(DEFAULT_ICID, TMC5222_RGR_XACTUAL);

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    int32_t buffer;
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= TMC5222_MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0:
        // Target position
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_XTARGET, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_XTARGET, *value);
        }
        break;
    case 1:
        // Actual position
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5222_RGR_XACTUAL, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5222_RGR_XACTUAL, *value);
        }
        break;
    case 2:
        // Target speed
        if(readWrite == READ) {
            if (tmc5222_fieldRead(DEFAULT_ICID, TMC5222_RAMPMODE_FIELD ) == 2){
                readRegister(DEFAULT_ICID,TMC5222_RGR_VMAX, value);
                *value = -(*value);
            }
            else
                readRegister(DEFAULT_ICID,TMC5222_RGR_VMAX, value);
        }
        else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5222_RGR_VMAX, abs(*value));
            vMaxModified = true;
        }
        break;
    case 3:
        // Actual speed
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5222_RGR_VACTUAL, value);
            *value = CAST_Sn_TO_S32(*value, 24);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 4:
        // Maximum speed
        if(readWrite == READ) {
            *value = vmax_position[motor];
        }
        else if(readWrite == WRITE) {
            vmax_position[motor] = abs(*value);
            if(tmc5222_fieldRead(DEFAULT_ICID, TMC5222_RAMPMODE_FIELD) == TMC5222_MODE_POSITION)
                writeRegister(DEFAULT_ICID,TMC5222_RGR_VMAX, abs(*value));
        }
        break;
    case 5:
        // Maximum acceleration
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5222_RGR_AMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5222_RGR_AMAX, *value);
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_IHOLD_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_IHOLD_FIELD, *value);
        }
        break;
    case 8:
        // Position reached flag
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_POSITION_REACHED_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 10:
        // Right endstop
        if(readWrite == READ) {
            *value = !tmc5222_fieldRead(DEFAULT_ICID, TMC5222_STATUS_STOP_R_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 11:
        // Left endstop
        if(readWrite == READ) {
            *value = !tmc5222_fieldRead(DEFAULT_ICID, TMC5222_STATUS_STOP_L_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 12:
        // Automatic right stop
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_STOP_R_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_STOP_R_ENABLE_FIELD, *value);
        }
        break;
    case 13:
        // Automatic left stop
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_POL_STOP_L_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_POL_STOP_L_FIELD, *value);
        }
        break;
    case 14:
        // SW_MODE Register
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5222_RGDR_SW_MODE, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5222_RGDR_SW_MODE, *value);
        }
        break;
    case 15:
        // Maximum Deceleration
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5222_RGR_DMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5222_RGR_DMAX, *value);
        }
        break;
    case 16:
        // Velocity VSTART
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_VSTART, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_VSTART, *value);
        }
        break;
    case 17:
        // Acceleration A1
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_A1, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_A1, *value);
        }
        break;
    case 18:
        // Velocity V1
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_V1, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_V1, *value);
        }
        break;
    case 19:
        // Deceleration D1
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_D1, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_D1, *value);
        }
        break;
    case 20:
        // Velocity VSTOP
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_VSTOP, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_VSTOP, *value);
        }
        break;
    case 21:
        // Waiting time after ramp down
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_TZEROWAIT, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_TZEROWAIT, *value);
        }
        break;
    case 22:
        // Velocity V2
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_V2, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_V2, *value);
        }
        break;
    case 23:
        // Deceleration D2
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_D2, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_D2, *value);
        }
        break;
    case 24:
        // Acceleration A2
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_A2, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_A2, *value);
        }
        break;
    case 25:
        // TVMAX
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_TVMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGR_TVMAX, *value);
        }
        break;
    case 26:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGR_TVMAX, value);
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
            writeRegister(DEFAULT_ICID, TMC5222_VDR_THIGH, *value);
        }
        break;
    case 27:
        // Minimum speed for switching to dcStep
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_RGDR_VDCMIN, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5222_RGDR_VDCMIN, *value);
        }
        break;
    case 28:
        // High speed chopper mode
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_VHIGHCHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_VHIGHCHM_FIELD, *value);
        }
        break;
    case 29:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_VHIGHFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_VHIGHFS_FIELD, *value);
        }
        break;
    case 30:
        // Measured Speed
        if(readWrite == READ) {
            *value = (int32_t)TMC5222.velocity;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;


    case 35:
        // Global current scaler A
        if(readWrite == READ) {
                *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_GLOBALSCALER_A_FIELD);
        } else if(readWrite == WRITE) {
            if(*value > 31)
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_GLOBALSCALER_A_FIELD, *value);
            else
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_GLOBALSCALER_A_FIELD, 0);
        }
        break;
    case 36:
        // Global current scaler B
        if(readWrite == READ) {
                *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_GLOBALSCALER_B_FIELD);
        }
        else if(readWrite == WRITE) {

            if(*value > 31)
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_GLOBALSCALER_B_FIELD, *value);
            else
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_GLOBALSCALER_B_FIELD, 0);


        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 0x100 >> tmc5222_fieldRead(DEFAULT_ICID, TMC5222_MRES_FIELD);
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
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_MRES_FIELD, *value);
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
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_TBL_FIELD, *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_CHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_CHM_FIELD, *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_DISFDCC_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_DISFDCC_FIELD, *value);
        }
        break;
    case 165:
        // Chopper hysteresis end / fast decay time
        readRegister(DEFAULT_ICID, TMC5222_MDR_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5222_CHM_SHIFT))
            {
                *value = (buffer >> TMC5222_HEND_OFFSET_SHIFT) & TMC5222_HEND_OFFSET_MASK;
            }
            else
            {
                *value = (buffer >> TMC5222_HSTRT_TFD210_SHIFT) & TMC5222_HSTRT_TFD210_MASK;

                if(buffer & TMC5222_HSTRT_TFD210_SHIFT)
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            readRegister(DEFAULT_ICID, TMC5222_MDR_CHOPCONF, &buffer);
            if(buffer & (1<<14))
            {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_HEND_OFFSET_FIELD, *value);
            }
            else
            {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_HSTRT_TFD210_FIELD, (*value & (1<<3))); // MSB wird zu value dazugefügt
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_HSTRT_TFD210_FIELD, *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        readRegister(DEFAULT_ICID, TMC5222_MDR_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5222_CHM_SHIFT))
            {
                *value = (buffer >> TMC5222_HSTRT_TFD210_SHIFT) & TMC5222_HSTRT_TFD210_MASK;
            }
            else
            {
                *value = (buffer >> TMC5222_HEND_OFFSET_SHIFT) & TMC5222_HEND_OFFSET_MASK;
                if(buffer & (1 << TMC5222_FD3_SHIFT))
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(buffer & (1 << TMC5222_CHM_SHIFT))
            {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_HSTRT_TFD210_FIELD, *value);
            }
            else
            {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_HEND_OFFSET_FIELD, *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_TOFF_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_TOFF_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SEMIN_FIELD, *value);
        }
        break;
    case 173:
        // stallGuard4 filter enable
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG4_FILT_EN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SG4_FILT_EN_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard4 threshold
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG4_THRS_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SG4_THRS_FIELD, *value);
        }
        break;
    case 175:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SFILT_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SFILT_FIELD, *value);
        }
        break;
    case 176:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SGT_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SGT_FIELD, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_CS_ACTUAL_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 181:
        // smartEnergy stall velocity
        //this function sort of doubles with 182 but is necessary to allow cross chip compliance
        if(readWrite == READ) {
            if(tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG_STOP_FIELD))
            {
                readRegister(DEFAULT_ICID, TMC5222_VDR_TCOOLTHRS, &buffer);
                *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
            }
            else
            {
                *value = 0;
            }
        } else if(readWrite == WRITE) {

            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_SG_STOP_FIELD, (*value)? 1:0);
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(DEFAULT_ICID, TMC5222_VDR_TCOOLTHRS, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_VDR_TCOOLTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(DEFAULT_ICID, TMC5222_VDR_TCOOLTHRS, *value);
        }
        break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_MDR_CHOPCONF, value);
            *value = (*value >> 20) & 0x0F;
        } else if(readWrite == WRITE) {
            readRegister(DEFAULT_ICID, TMC5222_MDR_CHOPCONF, &buffer);
            buffer &= ~(0x0F<<20);
            buffer |= (*value & 0x0F) << 20;
            writeRegister(DEFAULT_ICID, TMC5222_MDR_CHOPCONF, buffer);
        }
        break;
    case 186:
        // PWM threshold speed
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5222_VDR_TPWMTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
             writeRegister(DEFAULT_ICID, TMC5222_VDR_TPWMTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_PWM_GRAD_FIELD, *value);
            // Enable/disable stealthChop accordingly
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);

        }
        break;
    case 188:
        // PWM amplitude
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_OFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_PWM_OFS_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_PWM_FREQ_FIELD, *value);
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
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
            {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_PWM_AUTOSCALE_FIELD, *value);
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
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_SCALE_SUM_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 194:
        // MSCNT
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_MSCNT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 195:
        // MEAS_SD_EN
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_MEAS_SD_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_PWM_MEAS_SD_ENABLE_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 196:
        // DIS_REG_STST
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_PWM_DIS_REG_STST_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_PWM_DIS_REG_STST_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 204:
        // Freewheeling mode
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG_RESULT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 213:
            // ADCTemperatur
            if(readWrite == READ) {
                *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_TEMPERATURE_FIELD);
            } else if(readWrite == WRITE) {
                errors |= TMC_ERROR_TYPE;
            }
            break;
    case 214:
        // ADCTemperatur Converted
        if(readWrite == READ) {
            int32_t adc = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_TEMPERATURE_FIELD);
            *value = (int32_t)((1.017*adc)-259.2);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 216:
        if(readWrite == READ) {
            *value = HAL.IOs->config->isHigh(Pins.SLEEPN_LB);
        } else if(readWrite == WRITE) {
            if(*value == 1)
            {
                HAL.IOs->config->toOutput(Pins.SLEEPN_LB);
                HAL.IOs->config->setHigh(Pins.SLEEPN_LB);
                noRegResetnSLEEP = true;
                nSLEEPTick = systick_getTick();
            }
            else if(*value == 0)
            {
                HAL.IOs->config->toOutput(Pins.SLEEPN_LB);
                HAL.IOs->config->setLow(Pins.SLEEPN_LB);
            }

        }
        break;
    case 220:
        // MSLUT0
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT0, value);

        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT0, *value);
            }
        break;
    case 221:
        // MSLUT1
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT1, value);

        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT1, *value);
            }
        break;

    case 222:
        // MSLUT2
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT2, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT2, *value);
            }
        break;

    case 223:
        // MSLUT3
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT3, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT3, *value);
            }
        break;

    case 224:
        // MSLUT4
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT4, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT4, *value);
            }
        break;

    case 225:
        // MSLUT5
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT5, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT5, *value);
            }
        break;
    case 226:
        // MSLUT6
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT6, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT6, *value);
            }
        break;
    case 227:
        // MSLUT7
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT7, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT7, *value);
            }
        break;
    case 228:
        // MSLUT_START
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT_START, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT_START, *value);
            }
        break;
    case 229:
        // MSLUT_SEL
        if(readWrite == READ) {
                readRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT_SEL, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5222_MICROSTEP_LOOK_UP_TABLE_MSLUT_SEL, *value);
            }
        break;
    case 230:
        // START_SIN90
        if(readWrite == READ) {
                *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_START_SIN90_FIELD);
        } else if(readWrite == WRITE) {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_START_SIN90_FIELD, *value);
            }
        break;
    case 231:
        // OFFSET_SIN90
        if(readWrite == READ) {
                *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_OFFSET_SIN90_FIELD);
        } else if(readWrite == WRITE) {
                tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_OFFSET_SIN90_FIELD, *value);
            }
        break;
    case 232:
        // SG4_IND_0
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG4_IND_0_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 233:
        // SG4_IND_1
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG4_IND_1_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 234:
        // SG4_IND_2
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG4_IND_2_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 235:
        // SG4_IND_3
        if(readWrite == READ) {
            *value = tmc5222_fieldRead(DEFAULT_ICID, TMC5222_SG4_IND_3_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
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
    if(motor >= TMC5222_MOTORS)
        return TMC_ERROR_MOTOR;

    *value = 0;//TMC5222.velocity;

    return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc5222_writeRegister(DEFAULT_ICID, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc5222_readRegister(DEFAULT_ICID, address);
}

static void periodicJob(uint32_t tick)
{
    if(tmc5222_fieldRead(DEFAULT_ICID, TMC5222_DRV_ERR_FIELD) == 0 && drvError){
        drvError = false;
        noRegResetnSLEEP = true;
        nSLEEPTick = systick_getTick();
        return;
    }
    else if(tmc5222_fieldRead(DEFAULT_ICID, TMC5222_DRV_ERR_FIELD) == 1 && !drvError){
        drvError = true;
    }

    if(!noRegResetnSLEEP)
    {
        //check if reset after nSLEEP to HIGH was performed
        uint32_t tickDiff;

        if(TMC5222.config->state != CONFIG_READY)
        {
          TMC5222.config->state = CONFIG_READY;
            return;
        }

        int32_t x;

        // Calculate velocity v = dx/dt
        if((tickDiff = tick - TMC5222.oldTick) >= 5)
        {
            for(uint8_t motor = 0; motor < TMC5222_MOTORS; motor++)
            {
                readRegister(DEFAULT_ICID, TMC5222_RGR_XACTUAL, &x);
                TMC5222.velocity = (uint32_t) ((float32_t) ((x - TMC5222.oldX) / (float32_t) tickDiff) * (float32_t) 1048.576);
                TMC5222.oldX = x;
            }
            TMC5222.oldTick  = tick;
        }
    }
    else
    {
        //check if minimum time since chip activation passed. Then restore.
        if((systick_getTick()-nSLEEPTick)>20) //
        {
            noRegResetnSLEEP = false;
            enableDriver(DRIVER_ENABLE);
            tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_TOFF_FIELD, 3);
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
         * The the TMC5222 ref switch input is pulled high by external resistor an can be pulled low either by
         * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
         */
        if(!(*value & ~3))
        {
            if(*value & (1<<0))
            {
                HAL.IOs->config->toInput(Pins.REFRN_LB); // pull up -> set it to floating causes high
            }
            else
            {
                HAL.IOs->config->toOutput(Pins.REFRN_LB);
                HAL.IOs->config->setHigh(Pins.REFRN_LB);
            }

            if(*value & (1<<1))
            {
                HAL.IOs->config->toInput(Pins.REFLN_LB); // pull up -> set it to floating causes high
            }
            else
            {
                HAL.IOs->config->toOutput(Pins.REFLN_LB);
                HAL.IOs->config->setHigh(Pins.REFLN_LB);
            }
        }
        else
        {
            errors |= TMC_ERROR_VALUE;
        }
        break;
    case 5:  // read interrupt pin DIAG0_LB
        *value = (HAL.IOs->config->isHigh(Pins.DIAG0_LB))? 1:0;
        break;
    case 6:  // read interrupt pin DIAG1_LB
        *value = (HAL.IOs->config->isHigh(Pins.DIAG1_LB))? 1:0;
        break;
//    case 8: // Enable UART mode
//        if(*value == 1)
//            activeBus = IC_BUS_UART;
//        else if(*value == 0)
//            activeBus = IC_BUS_SPI;
//        init_comm(activeBus);
//        break;
    case 9: // Use internal clock
        /*
         * Internal clock will be enabled by calling this function with a value != 0 and power cycle the motor supply while keeping usb connected.
         */
        if(*value)
        {
            // Use Internal clock of 12MHz
            HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
            HAL.IOs->config->setToState(&HAL.IOs->pins->CLK16, IOS_LOW);
        }
        else
        {
            // Use external clock of 16MHz
            HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
#if defined(LandungsbrueckeV3)
            gpio_af_set(HAL.IOs->pins->CLK16.port, GPIO_AF_0, HAL.IOs->pins->CLK16.bitWeight);
#endif
        }
        break;
    case 10:  // Change device address
        deviceAddress = ((*value & 0xFF) << 1);
        break;
    case 11:  // Rising edge L
        HAL.IOs->config->toOutput(Pins.REFLN_LB);
        HAL.IOs->config->setHigh(Pins.REFLN_LB);
        delayBlocking(1);
        HAL.IOs->config->setLow(Pins.REFLN_LB);
        delayBlocking(10);
        HAL.IOs->config->setHigh(Pins.REFLN_LB);
        break;
    case 12:  // Rising edge R
        HAL.IOs->config->toOutput(Pins.REFRN_LB);
        HAL.IOs->config->setHigh(Pins.REFRN_LB);
        delayBlocking(1);
        HAL.IOs->config->setLow(Pins.REFRN_LB);
        delayBlocking(10);
        HAL.IOs->config->setHigh(Pins.REFRN_LB);
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static void deInit(void)
{
    HAL.IOs->config->setLow(Pins.DRV_EN_LB);
    HAL.IOs->config->setHigh(Pins.SEL_I2CN);
    HAL.IOs->config->reset(Pins.REFLN_LB);
    HAL.IOs->config->reset(Pins.REFRN_LB);
    HAL.IOs->config->reset(Pins.DIAG0_LB);
    HAL.IOs->config->reset(Pins.DIAG1_LB);
    HAL.IOs->config->reset(Pins.DRV_EN_LB);
    HAL.IOs->config->reset(Pins.SEL_I2CN);
    HAL.IOs->config->reset(Pins.SLEEPN_LB);
};

static uint8_t reset()
{

    HAL.IOs->config->toOutput(Pins.SLEEPN_LB);
    HAL.IOs->config->setLow(Pins.SLEEPN_LB);
    wait(50);
    HAL.IOs->config->setHigh(Pins.SLEEPN_LB);
    noRegResetnSLEEP = true;
    nSLEEPTick = systick_getTick();
    int32_t value = 0;

    for(uint8_t motor = 0; motor < TMC5222_MOTORS; motor++){
        readRegister(DEFAULT_ICID, TMC5222_RGR_VACTUAL, &value);
        if(value != 0)
            return 0;
    }

    if(TMC5222.config->state != CONFIG_READY)
        return false;

    TMC5222.config->state        = CONFIG_RESET;
    TMC5222.config->configIndex  = 0;

    return true;
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
        HAL.IOs->config->setLow(Pins.DRV_EN_LB);
        tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_DRV_EN_SW_FIELD, 0);
    }
    else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE)){
        HAL.IOs->config->setHigh(Pins.DRV_EN_LB);
        tmc5222_fieldWrite(DEFAULT_ICID, TMC5222_DRV_EN_SW_FIELD, 1);
    }
}

static void init_comm(TMC5222BusType mode)
{
    switch(mode) {
    case IC_BUS_IIC:
    default:
        IIC.init();
        HAL.IOs->config->setLow(Pins.SEL_I2CN);
        TMC5222_IIC = HAL.IIC;
        break;
    }
}

void TMC5222_init(void)
{
    Pins.DRV_EN_LB      = &HAL.IOs->pins->DIO0; //Pin8
    Pins.REFLN_LB       = &HAL.IOs->pins->DIO6; //Pin17
    Pins.REFRN_LB       = &HAL.IOs->pins->DIO7; //Pin18
    Pins.SLEEPN_LB      = &HAL.IOs->pins->DIO8; //Pin19
    Pins.SEL_I2CN       = &HAL.IOs->pins->DIO9;//Pin20
    Pins.DIAG1_LB       = &HAL.IOs->pins->DIO15; //Pin37
    Pins.DIAG0_LB       = &HAL.IOs->pins->DIO16; //Pin38

    HAL.IOs->config->toOutput(Pins.DRV_EN_LB);
    HAL.IOs->config->toOutput(Pins.SEL_I2CN);
    HAL.IOs->config->toOutput(Pins.SLEEPN_LB);

    HAL.IOs->config->setLow(Pins.SLEEPN_LB);
    HAL.IOs->config->setLow(Pins.DRV_EN_LB);

    // use internal clock of the IC and not from LB -> 12 MHz clock
    // Switchable via user function
    HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
    HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);

    HAL.IOs->config->toInput(Pins.REFLN_LB);
    HAL.IOs->config->toInput(Pins.REFRN_LB);

    init_comm(activeBus);

    Evalboards.ch1.config->reset        = reset;
    Evalboards.ch1.config->restore      = restore;
    Evalboards.ch1.config->state        = CONFIG_RESET;

    TMC5222.velocity = 0;
    TMC5222.oldX = 0;

    TMC5222.config               = Evalboards.ch1.config;
    TMC5222.config->callback     = NULL;
    TMC5222.config->channel      = 0;
    TMC5222.config->configIndex  = 0;
    TMC5222.config->state        = CONFIG_READY;

    for(uint8_t motor = 0; motor < TMC5222_MOTORS; motor++)
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
    Evalboards.ch1.numberOfMotors       = TMC5222_MOTORS;
    Evalboards.ch1.VMMin                = VM_MIN;
    Evalboards.ch1.VMMax                = VM_MAX;
    Evalboards.ch1.deInit               = deInit;

    enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
