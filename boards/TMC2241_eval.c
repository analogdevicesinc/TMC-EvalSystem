/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "tmc/StepDir.h"
#include "Board.h"
#include "tmc/ic/TMC2241/TMC2241.h"
//#define BoardVersion2241EvalDEV  //Version 1 Trinamic-Logo, Version 2 ADI-Logo

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

#undef  TMC2241_MAX_VELOCITY
#define TMC2241_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)
#define TMC2241_TIMEOUT 50 // UART Timeout in ms

#define TMC2241_MOTORS 1
#define DEFAULT_MOTOR  0
#define DEFAULT_ICID   0

static TMC2241BusType activeBus = IC_BUS_SPI;
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC2241_SPIChannel;
static UART_Config *TMC2241_UARTChannel;

static bool noRegResetnSLEEP = false;
static uint32_t nSLEEPTick;

typedef struct
{
    ConfigurationTypeDef *config;
    int32_t velocity, oldX;
    uint32_t oldTick;
    uint8_t slaveAddress;
} TMC2241TypeDef;

static TMC2241TypeDef TMC2241;

static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);

static void checkErrors(uint32_t tick);

static void init_comm(TMC2241BusType mode);
static void writeConfiguration();
static void periodicJob(uint32_t tick);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);
static void deInit(void);
static uint8_t reset();
static uint8_t restore();
static void enableDriver(DriverState state);

typedef struct {
    IOPinTypeDef *STEP;
    IOPinTypeDef *DIR;
    IOPinTypeDef *DRV_ENN;
    IOPinTypeDef *DIAG0;
    IOPinTypeDef *DIAG1;
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


void tmc2241_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC2241_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc2241_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
    int32_t status = UART_readWrite(TMC2241_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

TMC2241BusType tmc2241_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint8_t tmc2241_getNodeAddress(uint16_t icID)
{
    UNUSED(icID);

    return nodeAddress;
}


static uint32_t rotate(uint8_t motor, int32_t velocity) {
    if (motor >= TMC2241_MOTORS)
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
    if (motor >= TMC2241_MOTORS)
        return TMC_ERROR_MOTOR;

    StepDir_moveTo(motor, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks) {
    if (motor >= TMC2241_MOTORS)
        return TMC_ERROR_MOTOR;

    // determine actual position and add numbers of ticks to move
    *ticks += StepDir_getActualPosition(motor);

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value) {
    uint32_t errors = TMC_ERROR_NONE;
    uint32_t buffer;


    if (motor >= TMC2241_MOTORS)
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
                                        / (int64_t)tmc2241_fieldRead(DEFAULT_ICID, TMC2241_TSTEP_FIELD));
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
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_IRUN_FIELD);
        } else if (readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if (readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_IHOLD_FIELD);
        } else if (readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_IHOLD_FIELD, *value);
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
            buffer = tmc2241_readRegister(DEFAULT_ICID, TMC2241_THIGH);
            *value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_THIGH, *value);
        }

        break;

    case 28:
        // High speed chopper mode
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_VHIGHCHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_VHIGHCHM_FIELD, *value);
        }
        break;
    case 29:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_VHIGHFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_VHIGHFS_FIELD, *value);
        }
        break;
    case 30:
        // Measured Speed
        if(readWrite == READ) {
            *value = TMC2241.velocity;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 34:
        // Internal RSense
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_FAST_STANDSTILL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_FAST_STANDSTILL_FIELD, *value);
        }
        break;

    case 35:
        // Global current scaler
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_GLOBAL_SCALER_FIELD);
        } else if(readWrite == WRITE) {
            if(*value > 31){
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_GLOBAL_SCALER_FIELD, *value);
            }
            else{
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_GLOBAL_SCALER_FIELD, 0);
            }
        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 0x100 >> tmc2241_fieldRead(DEFAULT_ICID,TMC2241_MRES_FIELD);
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
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_MRES_FIELD, *value);
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
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_TBL_FIELD, *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_CHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_CHM_FIELD, *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_DISFDCC_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_DISFDCC_FIELD, *value);
        }
        break;
    case 165:
        // Chopper hysteresis end / fast decay time
        buffer = tmc2241_readRegister(DEFAULT_ICID, TMC2241_CHOPCONF);
        if(readWrite == READ) {
            if(buffer & (1 << TMC2241_CHM_SHIFT))
            {
                *value = (buffer >> TMC2241_HEND_OFFSET_SHIFT) & TMC2241_HEND_OFFSET_MASK;
            }
            else
            {
                *value = (tmc2241_readRegister(DEFAULT_ICID, TMC2241_CHOPCONF) >> TMC2241_HSTRT_TFD210_SHIFT) & TMC2241_HSTRT_TFD210_MASK;
                if(buffer & TMC2241_FD3_SHIFT)
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(tmc2241_readRegister(DEFAULT_ICID, TMC2241_CHOPCONF) & (1<<14))
            {
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_HEND_OFFSET_FIELD, *value);
            }
            else
            {
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_FD3_FIELD, (*value & (1<<3))); // MSB wird zu value dazugefügt
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_HSTRT_TFD210_FIELD, *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        buffer = tmc2241_readRegister(DEFAULT_ICID, TMC2241_CHOPCONF);
        if(readWrite == READ) {
            if(buffer & (1 << TMC2241_CHM_SHIFT))
            {
                *value = (buffer >> TMC2241_HSTRT_TFD210_SHIFT) & TMC2241_HSTRT_TFD210_MASK;
            }
            else
            {
                *value = (buffer >> TMC2241_HEND_OFFSET_SHIFT) & TMC2241_HEND_OFFSET_MASK;
                if(buffer & (1 << TMC2241_FD3_SHIFT))
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(buffer & (1 << TMC2241_CHM_SHIFT))
            {
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_HSTRT_TFD210_FIELD, *value);
            }
            else
            {
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_HSTRT_TFD210_FIELD, *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_TOFF_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_TOFF_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID,  TMC2241_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SEMIN_FIELD, *value);
        }
        break;
    case 173:
        // stallGuard4 filter enable
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID,  TMC2241_SG4_FILT_EN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SG4_FILT_EN_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard4 threshold
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SG4_THRS_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SG4_THRS_FIELD, *value);
        }
        break;
    case 175:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID,  TMC2241_SFILT_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SFILT_FIELD, *value);
        }
        break;
    case 176:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID,  TMC2241_SGT_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_SGT_FIELD, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_CS_ACTUAL_FIELD);
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
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_TCOOLTHRS, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            buffer = tmc2241_readRegister(DEFAULT_ICID, TMC2241_TCOOLTHRS);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_TCOOLTHRS, *value);
        }
        break;
    case 184:
        // SG_ANGLE_OFFSET
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SG_ANGLE_OFFSET_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID,TMC2241_SG_ANGLE_OFFSET_FIELD, *value);
        }
        break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            *value = (tmc2241_readRegister(DEFAULT_ICID, TMC2241_CHOPCONF) >> 20) & 0x0F;
        } else if(readWrite == WRITE) {
            buffer = tmc2241_readRegister(DEFAULT_ICID, TMC2241_CHOPCONF);
            buffer &= ~(0x0F<<20);
            buffer |= (*value & 0x0F) << 20;
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_CHOPCONF,buffer);
        }
        break;
    case 186:
        // PWM threshold speed
        if(readWrite == READ) {
            buffer = tmc2241_readRegister(DEFAULT_ICID, TMC2241_TPWMTHRS);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_TPWMTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_PWM_GRAD_FIELD, *value);
            // Enable/disable stealthChop accordingly
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_PWM_GRAD_FIELD, (*value) ? 1 : 0);
        }
        break;
    case 188:
        // PWM amplitude
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_PWM_OFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID,  TMC2241_PWM_OFS_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID,  TMC2241_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_PWM_FREQ_FIELD, *value);
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
            *value = tmc2241_fieldRead(DEFAULT_ICID,  TMC2241_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
            {
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_PWM_AUTOSCALE_FIELD, *value);
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
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_PWM_SCALE_SUM_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 194:
        // MSCNT
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_MSCNT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 195:
        // MEAS_SD_EN
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_PWM_MEAS_SD_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_PWM_MEAS_SD_ENABLE_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 196:
        // DIS_REG_STST
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_PWM_DIS_REG_STST_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_PWM_DIS_REG_STST_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 204:
        // Freewheeling mode
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_SG_RESULT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 209:
        // Encoder position
        if(readWrite == READ) {
            *value = tmc2241_readRegister(DEFAULT_ICID, TMC2241_X_ENC);
        } else if(readWrite == WRITE) {
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_X_ENC, *value);
        }
        break;
    case 210:
        // Encoder Resolution
        if(readWrite == READ) {
            *value = tmc2241_readRegister(DEFAULT_ICID, TMC2241_ENC_CONST);
        } else if(readWrite == WRITE) {
            tmc2241_writeRegister(DEFAULT_ICID, TMC2241_ENC_CONST, *value);
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
        // Current range from DRV_CONF reg
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_CURRENT_RANGE_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_CURRENT_RANGE_FIELD, *value);
        }
        break;

    case 213:
        // ADCTemperatur
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID,  TMC2241_ADC_TEMP_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 214:
        // ADCIN
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_ADC_AIN_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 215:
        // ADCSupply
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_ADC_VSUPPLY_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 216:
        // Overvoltage Limit ADC value
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_OVERVOLTAGE_VTH_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_OVERVOLTAGE_VTH_FIELD, *value);
        }
        break;
    case 217:
        // Overtemperature Warning Limit
        if(readWrite == READ) {
            *value = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_OVERTEMPPREWARNING_VTH_FIELD);
        } else if(readWrite == WRITE) {
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_OVERTEMPPREWARNING_VTH_FIELD, *value);
        }
        break;
    case 218:
        // ADCTemperatur Converted
        if(readWrite == READ) {

            int32_t adc = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_ADC_TEMP_FIELD);
            *value = (int32_t)10*(adc-2038)/77;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 219:
        // ADCIN converted
        if(readWrite == READ) {
            int32_t adc = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_ADC_AIN_FIELD);
            *value = (int32_t)3052*adc/10000;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 220:
        // ADCSupply
        if(readWrite == READ) {
            int32_t adc = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_ADC_VSUPPLY_FIELD);
            *value = (int32_t)58*3052*adc/10000;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 221:
        // Overvoltage Limit converted
        if(readWrite == READ) {
            int32_t val = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_OVERVOLTAGE_VTH_FIELD);
            *value = (int32_t)58*3052*val/10000;
        } else if(readWrite == WRITE) {
            int32_t val = (int32_t)(*value*10000/(3052*58));
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_OVERVOLTAGE_VTH_FIELD, val);
        }
        break;
    case 222:
        // Overtemperature Warning Limit
        if(readWrite == READ) {
            int32_t temp = tmc2241_fieldRead(DEFAULT_ICID, TMC2241_OVERTEMPPREWARNING_VTH_FIELD);
            *value = (int32_t)(temp-2038)/7.7;
        } else if(readWrite == WRITE) {
            float valf  = *value*7.7;
            int32_t val = (int32_t)valf;
            val = val+2038;
            tmc2241_fieldWrite(DEFAULT_ICID, TMC2241_OVERTEMPPREWARNING_VTH_FIELD, val);
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
                activeBus = IC_BUS_UART;
            else if(*value == 0)
                activeBus = IC_BUS_SPI;
            init_comm(activeBus);
        }
        else if(readWrite == READ) {
            if(activeBus == IC_BUS_UART)
                *value = 1;
            else if (activeBus == IC_BUS_SPI)
                *value = 0;
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


static void writeRegister(uint8_t motor, uint16_t address, int32_t value) {
    UNUSED(motor);
    tmc2241_writeRegister(DEFAULT_ICID, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value) {
    UNUSED(motor);
    *value = tmc2241_readRegister(DEFAULT_ICID, (uint8_t) address);
}
static void checkErrors(uint32_t tick)
{
    UNUSED(tick);
    Evalboards.ch1.errors = 0;
}
// Helper function: Configure the next register.
static void writeConfiguration()
{
    uint8_t *ptr = &TMC2241.config->configIndex;
    const int32_t *settings;

    if(TMC2241.config->state == CONFIG_RESTORE)
    {
        settings = *(tmc2241_shadowRegister+0);
        // Find the next restorable register
        while(*ptr < TMC2241_REGISTER_COUNT)
        {
            // If the register is writable and has been written to, restore it
            if (TMC_IS_WRITABLE(tmc2241_registerAccess[*ptr]) && tmc2241_getDirtyBit(DEFAULT_ICID,*ptr))
            {
                break;
            }

            // Otherwise, check next register
            (*ptr)++;
        }
    }
    else
    {
        settings = tmc2241_sampleRegisterPreset;
        // Find the next resettable register
        while((*ptr < TMC2241_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc2241_registerAccess[*ptr]))
        {
            (*ptr)++;
        }
    }
    if(*ptr < TMC2241_REGISTER_COUNT)
    {
        tmc2241_writeRegister(DEFAULT_ICID, *ptr, settings[*ptr]);
        (*ptr)++;
    }
    else // Finished configuration
    {
        if(TMC2241.config->state == CONFIG_RESET)
        {
            // Fill missing shadow registers (hardware preset registers)
            tmc2241_initCache();
        }

        TMC2241.config->state = CONFIG_READY;
    }
}
static void periodicJob(uint32_t tick)
{
    UNUSED(tick);

    //check if reset after nSLEEP to HIGH was performed
    if(!noRegResetnSLEEP)
    {
        if(TMC2241.config->state != CONFIG_READY)
        {
            writeConfiguration(TMC2241);
        }
        StepDir_periodicJob(DEFAULT_MOTOR);
    }
    else
    {
        //check if minimum time since chip activation passed. Then restore.
        if((systick_getTick()-nSLEEPTick)>5000) //
        {
            restore();

            noRegResetnSLEEP = false;
        }
    }
}


static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    uint32_t errors = 0;

    UNUSED(motor);

    switch(type)
    {
    case 0:  // Read StepDir status bits
        *value = StepDir_getStatus(motor);
        break;
    case 8: // Enable UART mode
        if(*value == 1)
            activeBus = IC_BUS_UART;
        else if(*value == 0)
            activeBus = IC_BUS_SPI;
        init_comm(activeBus);
        break;

    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value) {
    if (motor >= TMC2241_MOTORS)
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

static void deInit(void) {
    HAL.IOs->config->reset(Pins.DRV_ENN);

    HAL.IOs->config->reset(Pins.STEP);
    HAL.IOs->config->reset(Pins.DIR);
    HAL.IOs->config->reset(Pins.DIAG0);
    HAL.IOs->config->reset(Pins.DIAG1);
    HAL.IOs->config->reset(Pins.nSLEEP);
    HAL.IOs->config->reset(Pins.IREF_R2);
    HAL.IOs->config->reset(Pins.IREF_R3);
    HAL.IOs->config->reset(Pins.UART_MODE);

    StepDir_deInit();
}

static uint8_t reset() {

    if (StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
        return 0;

    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.DIAG1);
    StepDir_setVelocityMax(0, 100000);
    StepDir_setAcceleration(0, 25000);
    enableDriver(DRIVER_ENABLE);

    if(TMC2241.config->state != CONFIG_READY)
        return 0;

    // Reset the dirty bits and wipe the shadow registers
    for(size_t i = 0; i < TMC2241_REGISTER_COUNT; i++)
    {
        tmc2241_setDirtyBit(DEFAULT_ICID, i, false);
        tmc2241_shadowRegister[DEFAULT_ICID][i] = 0;
    }

    TMC2241.config->state        = CONFIG_RESET;
    TMC2241.config->configIndex  = 0;

    return 1;
}

static uint8_t restore() {
    if(TMC2241.config->state != CONFIG_READY)
        return 0;

    TMC2241.config->state        = CONFIG_RESTORE;
    TMC2241.config->configIndex  = 0;
    return 1;
}


static void enableDriver(DriverState state) {
    if (state == DRIVER_USE_GLOBAL_ENABLE)
        state = Evalboards.driverEnable;

    if (state == DRIVER_DISABLE)
        HAL.IOs->config->setHigh(Pins.DRV_ENN);
    else if ((state == DRIVER_ENABLE)
            && (Evalboards.driverEnable == DRIVER_ENABLE))
        HAL.IOs->config->setLow(Pins.DRV_ENN);
}
static void init_comm(TMC2241BusType mode)
{
    TMC2241_UARTChannel = HAL.UART;
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
        TMC2241_UARTChannel->pinout = UART_PINS_2;
        TMC2241_UARTChannel->rxtx.init();
        break;
    case IC_BUS_SPI:
        HAL.IOs->config->reset(Pins.SCK);
        HAL.IOs->config->reset(Pins.SDI);
        HAL.IOs->config->reset(Pins.SDO);
        HAL.IOs->config->reset(Pins.CS);
        SPI.init();
        HAL.IOs->config->setLow(Pins.UART_MODE);
        TMC2241_UARTChannel->rxtx.deInit();
#ifndef BoardVersion2241EvalDEV
        TMC2241_SPIChannel = &HAL.SPI->ch2;
        TMC2241_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;
#else
        TMC2241_SPIChannel = &HAL.SPI->ch1;
        TMC2241_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
#endif
        break;
    case IC_BUS_WLAN: // unused
    default:
        HAL.IOs->config->reset(Pins.SCK);
        HAL.IOs->config->reset(Pins.SDI);
        HAL.IOs->config->reset(Pins.SDO);
        HAL.IOs->config->reset(Pins.CS);
        SPI.init();
        HAL.IOs->config->setLow(Pins.UART_MODE);
        TMC2241_UARTChannel->rxtx.deInit();
        TMC2241_SPIChannel = &HAL.SPI->ch2;
#ifndef BoardVersion2241EvalDEV
        TMC2241_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;
        activeBus = IC_BUS_SPI;

#else
        TMC2241_SPIChannel = &HAL.SPI->ch1;
        TMC2241_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
#endif
        break;
    }
}

void TMC2241_init(void) {
    tmc_fillCRC8Table(0x07, true, 1);

#ifndef BoardVersion2241EvalDEV

    // Initialize the hardware pins
    Pins.DRV_ENN = &HAL.IOs->pins->DIO0;
    Pins.STEP         = &HAL.IOs->pins->DIO6;
    Pins.DIR          = &HAL.IOs->pins->DIO7;
    Pins.DIAG0        = &HAL.IOs->pins->DIO16;
    Pins.DIAG1        = &HAL.IOs->pins->DIO15;
    Pins.nSLEEP       = &HAL.IOs->pins->DIO8;
    Pins.IREF_R2      = &HAL.IOs->pins->DIO1;
    Pins.IREF_R3      = &HAL.IOs->pins->DIO2;
    Pins.UART_MODE    = &HAL.IOs->pins->DIO9;
    Pins.SCK          = &HAL.IOs->pins->SPI2_SCK; //
    Pins.SDI          = &HAL.IOs->pins->SPI2_SDI; //
    Pins.SDO          = &HAL.IOs->pins->SPI2_SDO; //
    Pins.CS           = &HAL.IOs->pins->SPI2_CSN0; //

#else
    Pins.DRV_ENN = &HAL.IOs->pins->DIO0;
    Pins.STEP         = &HAL.IOs->pins->DIO6;
    Pins.DIR          = &HAL.IOs->pins->DIO7;
    Pins.DIAG0        = &HAL.IOs->pins->DIO16;
    Pins.DIAG1        = &HAL.IOs->pins->DIO15;
    Pins.nSLEEP       = &HAL.IOs->pins->DIO8;
    Pins.IREF_R2      = &HAL.IOs->pins->DIO13;
    Pins.IREF_R3      = &HAL.IOs->pins->DIO14;
    Pins.UART_MODE    = &HAL.IOs->pins->DIO9;
    Pins.SCK          = &HAL.IOs->pins->SPI1_SCK; //Pin31
    Pins.SDI          = &HAL.IOs->pins->SPI1_SDI; //Pin32
    Pins.SDO          = &HAL.IOs->pins->SPI1_SDO; //Pin33
    Pins.CS           = &HAL.IOs->pins->SPI1_CSN; //Pin33

#endif
    HAL.IOs->config->toInput(Pins.DIAG0);
    HAL.IOs->config->toInput(Pins.DIAG1);

    HAL.IOs->config->toOutput(Pins.STEP);
    HAL.IOs->config->toOutput(Pins.DIR);
    HAL.IOs->config->toOutput(Pins.DRV_ENN);
    HAL.IOs->config->toOutput(Pins.UART_MODE);
    HAL.IOs->config->toOutput(Pins.nSLEEP);
    noRegResetnSLEEP = true;
    nSLEEPTick = systick_getTick();
    HAL.IOs->config->toOutput(Pins.IREF_R2);
    HAL.IOs->config->toOutput(Pins.IREF_R3);

    HAL.IOs->config->setHigh(Pins.nSLEEP);
    HAL.IOs->config->setHigh(Pins.DRV_ENN);
    HAL.IOs->config->setLow(Pins.UART_MODE);
    HAL.IOs->config->setLow(Pins.IREF_R2);
    HAL.IOs->config->setLow(Pins.IREF_R2);

    // Initialize the SPI channel
    init_comm(activeBus);

    TMC2241.config   = Evalboards.ch2.config;
    Evalboards.ch2.config->reset = reset;
    Evalboards.ch2.config->restore = restore;
    Evalboards.ch2.config->state = CONFIG_RESET;
    Evalboards.ch2.config->configIndex = 0;
    Evalboards.ch2.config->callback     = NULL;
    Evalboards.ch2.config->channel      = 0;

    TMC2241.velocity  = 0;
    TMC2241.oldTick   = 0;
    TMC2241.oldX      = 0;

    // Initialize the software StepDir generator
    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.DIAG1);
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
    Evalboards.ch2.numberOfMotors       = TMC2241_MOTORS;
    Evalboards.ch2.VMMin                = VM_MIN;
    Evalboards.ch2.VMMax                = VM_MAX;
    Evalboards.ch2.deInit               = deInit;

    enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
