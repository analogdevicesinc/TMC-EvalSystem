/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "tmc/StepDir.h"
#include "Board.h"
#include "tmc/ic/TMC2130/TMC2130.h"

static SPIChannelTypeDef *TMC2130_SPIChannel;

#define DEFAULT_ICID  0

#define TMC2130_EVAL_VM_MIN  50   // VM[V/10] min
#define TMC2130_EVAL_VM_MAX  480  // VM[V/10] max +5%

#undef  TMC2130_MAX_VELOCITY
#define TMC2130_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define VREF_FULLSCALE 2714 // mV

static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static void writeConfiguration();
static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);
static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void periodicJob(uint32_t tick);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);
static void deInit(void);
static uint8_t reset();
static uint8_t restore();
static void enableDriver(DriverState state);

//static int32_t measured_velocity = 0;

typedef struct
{
    IOPinTypeDef *REFL_STEP;
    IOPinTypeDef *REFR_DIR;
    IOPinTypeDef *DRV_ENN_CFG6;
    IOPinTypeDef *ENCA_DCIN_CFG5;
    IOPinTypeDef *ENCB_DCEN_CFG4;
    IOPinTypeDef *ENCN_DCO;
    IOPinTypeDef *DIAG0;
    IOPinTypeDef *DIAG1;
    IOPinTypeDef *AIN_REF_SW;
    IOPinTypeDef *AIN_REF_PWM;
} PinsTypeDef;

static PinsTypeDef Pins;


// Typedefs
typedef struct
{
    ConfigurationTypeDef *config;
} TMC2130TypeDef;

static TMC2130TypeDef TMC2130;

static uint16_t vref; // mV

// => SPI wrapper
void tmc2130_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC2130_SPIChannel-> readWriteArray(data, dataLength);
}
// <= SPI wrapper

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    if(motor >= TMC2130_MOTORS)
        return TMC_ERROR_MOTOR;

    StepDir_rotate(motor, velocity);

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
    if(motor >= TMC2130_MOTORS)
        return TMC_ERROR_MOTOR;

    StepDir_moveTo(motor, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    if(motor >= TMC2130_MOTORS)
        return TMC_ERROR_MOTOR;

    // determine actual position and add numbers of ticks to move
    *ticks += StepDir_getActualPosition(motor);

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= TMC2130_MOTORS)
        return TMC_ERROR_MOTOR;

    int32_t tempValue;

    switch(type)
    {
    case 0:
        // Target position
        if(readWrite == READ) {
            *value = StepDir_getTargetPosition(motor);
        } else if(readWrite == WRITE) {
            StepDir_moveTo(motor, *value);
        }
        break;
    case 1:
        // Actual position
        if(readWrite == READ) {
            *value = StepDir_getActualPosition(motor);
        } else if(readWrite == WRITE) {
            StepDir_setActualPosition(motor, *value);
        }
        break;
    case 2:
        // Target speed
        if(readWrite == READ) {
            *value = StepDir_getTargetVelocity(motor);
        } else if(readWrite == WRITE) {
            StepDir_rotate(motor, *value);
        }
        break;
    case 3:
        // Actual speed
        if(readWrite == READ) {
            switch(StepDir_getMode(motor)) {
            case STEPDIR_INTERNAL:
                *value = StepDir_getActualVelocity(motor);
                break;
            case STEPDIR_EXTERNAL:
            default:
                tempValue = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)tmc2130_fieldRead(DEFAULT_ICID, TMC2130_TSTEP_FIELD));
                *value = (abs(tempValue) < 20) ? 0 : tempValue;
                break;
            }
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 4:
        // Maximum speed
        if(readWrite == READ) {
            *value = StepDir_getVelocityMax(motor);
        } else if(readWrite == WRITE) {
            StepDir_setVelocityMax(motor, abs(*value));
        }
        break;
    case 5:
        // Maximum acceleration
        if(readWrite == READ) {
            *value = StepDir_getAcceleration(motor);
        } else if(readWrite == WRITE) {
            StepDir_setAcceleration(motor, *value);
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_IHOLD_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_IHOLD_FIELD, *value);
        }
        break;
    case 8:
        // Position reached flag
        if(readWrite == READ) {
            *value = (StepDir_getStatus(motor) & STATUS_TARGET_REACHED)? 1:0;
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
    case 21:
        // todo CHECK 3: Funktionalität prüfen (JE)
        // todo XML 3: XML Beschreibung hinzufügen (JE)
        if(readWrite == READ) {
            errors |= TMC_ERROR_TYPE;
        } else if(readWrite == WRITE) {
            tmc2130_writeRegister(DEFAULT_ICID, TMC2130_TPOWERDOWN, *value);
        }
        break;
    case 23:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            tempValue = tmc2130_readRegister(DEFAULT_ICID, TMC2130_THIGH);
            *value = MIN(0xFFFFF, (1<<24) / ((tempValue)? tempValue:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            tmc2130_writeRegister(DEFAULT_ICID, TMC2130_THIGH, *value);
        }
        break;
    case 24:
        // Minimum speed for switching to dcStep
        if(readWrite == READ) {
            *value = tmc2130_readRegister(DEFAULT_ICID, TMC2130_VDCMIN);
        } else if(readWrite == WRITE) {
            tmc2130_writeRegister(DEFAULT_ICID, TMC2130_VDCMIN, *value);
        }
        break;
    case 26:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_VHIGHFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_VHIGHFS_FIELD, *value);
        }
        break;
    case 27:
        // High speed chopper mode
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_VHIGHCHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_VHIGHCHM_FIELD, *value);
        }
        break;
    case 28:
        // Internal RSense
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_INTERNAL_RSENSE_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_INTERNAL_RSENSE_FIELD, *value);
        }
        break;
    case 29:
        // Measured Speed
        if(readWrite == READ) {
            tempValue = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)tmc2130_fieldRead(DEFAULT_ICID, TMC2130_TSTEP_FIELD));
            *value = (abs(tempValue) < 20) ? 0 : tempValue;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 50: // StepDir internal(0)/external(1)
        if(readWrite == READ) {
            *value = StepDir_getMode(motor);
        } else if(readWrite == WRITE) {
            StepDir_setMode(motor, *value);
        }
        break;
    case 51: // StepDir interrupt frequency
        if(readWrite == READ) {
            *value = StepDir_getFrequency(motor);
        } else if(readWrite == WRITE) {
            StepDir_setFrequency(motor, *value);
        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 256 >> tmc2130_fieldRead(DEFAULT_ICID, TMC2130_MRES_FIELD);
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
                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_MRES_FIELD, *value);
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
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_TBL_FIELD, *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_CHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_CHM_FIELD, *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_DISFDCC_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_DISFDCC_FIELD, *value);
        }
        break;
    case 165:
        // Chopper hysteresis end / fast decay time
        if(readWrite == READ) {
            if(tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF) & (1<<14))
            {
                *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_HEND_FIELD);
            }
            else
            {
                tempValue = tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF);
                *value = (tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF) >> 4) & 0x07;
                if(tempValue & (1<<11))
                    *value |= 1<<3;
            }
        } else if(readWrite == WRITE) {
            if(tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF) & (1<<14))
            {
                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_HEND_FIELD, *value);
            }
            else
            {
                tempValue = tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF);

                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_TFD___FIELD, (*value & (1<<3))? 1:0);
                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_TFD_2__0__FIELD, *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        if(readWrite == READ) {
            if(tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF) & (1<<14))
            {
                *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_HSTRT_FIELD);
            }
            else
            {
                tempValue = tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF);
                *value = (tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF) >> 7) & 0x0F;
                if(tempValue & (1<<11))
                    *value |= 1<<3;
            }
        } else if(readWrite == WRITE) {
            if(tmc2130_readRegister(DEFAULT_ICID, TMC2130_CHOPCONF) & (1<<14))
            {
                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_HSTRT_FIELD, *value);
            }
            else
            {
                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_OFFSET_FIELD, *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_TOFF_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_TOFF_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID , TMC2130_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SEMIN_FIELD, *value);
        }
        break;
    case 173:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SFILT_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SFILT_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SGT_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SGT_FIELD, *value);
        }
        break;
    case 179:
        // VSense
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_VSENSE_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_VSENSE_FIELD, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_CS_ACTUAL_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 181:
        // smartEnergy stall velocity
        if(readWrite == READ) {
            *value = StepDir_getStallGuardThreshold(motor);
        } else if(readWrite == WRITE) {
            StepDir_setStallGuardThreshold(motor, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            tempValue = tmc2130_readRegister(DEFAULT_ICID, TMC2130_TCOOLTHRS);
            *value = MIN(0xFFFFF, (1<<24) / ((tempValue)? tempValue:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            tmc2130_writeRegister(DEFAULT_ICID, TMC2130_TCOOLTHRS, *value);
        }
        break;
    case 184:
        // Random TOff mode
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_RNDTF_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_RNDTF_FIELD, *value);
        }
        break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SYNC_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_SYNC_FIELD, *value);
        }
        break;
    case 186:
        // PWM threshold speed
        if(readWrite == READ) {
            tempValue = tmc2130_readRegister(DEFAULT_ICID, TMC2130_TPWMTHRS);
            *value = MIN(0xFFFFF, (1<<24) / ((tempValue)? tempValue:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            tmc2130_writeRegister(DEFAULT_ICID, TMC2130_TPWMTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_PWM_GRAD_FIELD, *value);

            // Enable/disable stealthChop accordingly
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);
        }
        break;
    case 188:
        // PWM amplitude
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_PWM_AMPL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_PWM_AMPL_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_PWM_FREQ_FIELD, *value);
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
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_PWM_AUTOSCALE_FIELD, (*value)? 1:0);
        }
        break;
    case 204:
        // Freewheeling mode
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc2130_fieldRead(DEFAULT_ICID, TMC2130_SG_RESULT_FIELD);
        } else if(readWrite == WRITE) {
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

// Helper function: Configure the next register.
static void writeConfiguration()
{

    uint8_t *ptr = &TMC2130.config->configIndex;
    const int32_t *settings;

    if(TMC2130.config->state == CONFIG_RESTORE)
    {
        settings = *(tmc2130_shadowRegister+0);
        // Find the next restorable register
        while(*ptr < TMC2130_REGISTER_COUNT)
        {
            // If the register is writable and has been written to, restore it
            if (TMC_IS_WRITABLE(tmc2130_registerAccess[*ptr]) && tmc2130_getDirtyBit(DEFAULT_MOTOR, *ptr))
            {
                break;
            }

            // Otherwise, check next register
            (*ptr)++;
        }
    }
    else
    {
        settings = tmc2130_sampleRegisterPreset;
        // Find the next resettable register
        while((*ptr < TMC2130_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc2130_registerAccess[*ptr]))
        {
            (*ptr)++;
        }
    }

    if(*ptr < TMC2130_REGISTER_COUNT)
    {
        tmc2130_writeRegister(DEFAULT_MOTOR, *ptr, settings[*ptr]);
        (*ptr)++;
    }
    else // Finished configuration
    {

        if(TMC2130.config->state == CONFIG_RESET)
        {
            // Change hardware preset registers here
            tmc2130_writeRegister(DEFAULT_ICID, TMC2130_PWMCONF, 0x000504C8);

            // Fill missing shadow registers (hardware preset registers)
            tmc2130_initCache();
        }

        TMC2130.config->state = CONFIG_READY;
    }
}

static uint32_t getLimit(AxisParameterLimit limit, uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    uint32_t errors = TMC_ERROR_NONE;
    switch(type) {
    case 2:
    case 3:
    case 4:
    case 24:
        if(limit == LIMIT_MIN) {
            *value = 0; // TODO: Determine limits here
        } else if(limit == LIMIT_MAX) {
            *value = StepDir_getFrequency(motor);
        }
        break;
    case 5:
        if(limit == LIMIT_MIN) {
            *value = 0; // TODO: Determine limits here
        } else if(limit == LIMIT_MAX) {
            *value = StepDir_getMaxAcceleration(motor);
        }
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static uint32_t getMin(uint8_t type, uint8_t motor, int32_t *value)
{
    return getLimit(LIMIT_MIN, type, motor, value);
}

static uint32_t getMax(uint8_t type, uint8_t motor, int32_t *value)
{
    return getLimit(LIMIT_MAX, type, motor, value);
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc2130_writeRegister(DEFAULT_ICID, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc2130_readRegister(DEFAULT_ICID, (uint8_t) address);
}

static void periodicJob(uint32_t tick)
{
    UNUSED(tick);
    if(TMC2130.config->state != CONFIG_READY)
    {
        writeConfiguration();
        return;
    }
    StepDir_periodicJob(DEFAULT_MOTOR);
    StepDir_stallGuard(DEFAULT_MOTOR, tmc2130_fieldRead(DEFAULT_MOTOR, TMC2130_STALLGUARD_FIELD) == 1);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    uint32_t errors = 0;

    uint32_t uvalue;

    switch(type)
    {
    case 0:    // set analogue current duty
        /*
         * Current will be defined by analogue value voltage or current signal. In any case this function
         * will generate a analogue voltage by PWM for up to 50% duty and a switch for the other 50%.
         * The reference voltage will be AIN_REF = VCC_IO * value/20000 with value = {0..20000}
         */

        uvalue = (uint32_t) *value;

        if(uvalue <= 20000)
        {
            HAL.IOs->config->setToState(Pins.AIN_REF_SW, (uvalue > 10000) ? IOS_HIGH : IOS_LOW);
            Timer.setDuty(TIMER_CHANNEL_1, ((float)(uvalue % 10001)) / TIMER_MAX);
        }
        else
        {
            errors |= TMC_ERROR_VALUE;
        }
        break;
    case 1:    // Use internal clock
        /*
         * Internel clock will be enabled by calling this function with a value != 0 and unpower and repower the motor supply while keeping usb connected.
         */
        if(*value)
            HAL.IOs->config->setToState(&HAL.IOs->pins->CLK16, IOS_LOW);
        else
            HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
        break;
    case 2:    // writing a register at address = motor with value = value and reading back the value
        // DO NOT USE!
        // todo BUG 3: using the motor for both address and motor value can lead to trying to get struct pointer from an array outside its bounds
        //             and then loading a function pointer from that. That is pretty a much guaranteed crash and/or hard fault!!! (LH) #1

        //tmc2130_writeRegister(DEFAULT_ICID, motor, *value);
        //*value = tmc2130_readRegister(DEFAULT_ICID, motor);
        break;
    case 3:     // set DC_EN
        HAL.IOs->config->setToState(Pins.ENCB_DCEN_CFG4, (*value) ? IOS_HIGH : IOS_LOW);
        break;
    case 4:  // Read StepDir status bits
        *value = StepDir_getStatus(motor);
        break;
    case 5:  // When running with the TMC4330 v1.2: CFG4 is routed from DIO6 (Motion Controller) to DIO13 (Driver)
        Pins.ENCB_DCEN_CFG4 = &HAL.IOs->pins->DIO6;
        HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
        break;
    case 6:
        // When running with any of the TMC43XX Evals, CFG5 is not routed through.
        // The DIO12 pin not usable by the TMC2130 and is reset to a HIGH level.
        Pins.ENCA_DCIN_CFG5 = &HAL.IOs->pins->DUMMY;
        HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO12);
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
    if(motor >= TMC2130_MOTORS)
        return TMC_ERROR_MOTOR;

    switch(motor)
    {
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

static void deInit(void)
{
    HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);    // DISABLE DRIVER

    HAL.IOs->config->reset(Pins.REFL_STEP);
    HAL.IOs->config->reset(Pins.REFR_DIR);
    HAL.IOs->config->reset(Pins.AIN_REF_SW);
    HAL.IOs->config->reset(Pins.AIN_REF_PWM);
    HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
    HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->reset(Pins.ENCN_DCO);
    HAL.IOs->config->reset(Pins.DIAG0);
    HAL.IOs->config->reset(Pins.DIAG1);
    HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);

    StepDir_deInit();
    Timer.deInit();
}

static uint8_t reset()
{
    if(StepDir_getActualVelocity(0) && !VitalSignsMonitor.brownOut)
        return 0;

    if(TMC2130.config->state == CONFIG_READY)
    {
        // Reset the dirty bits and wipe the shadow registers
        size_t i;
        for(i = 0; i < TMC2130_REGISTER_COUNT; i++)
        {
            tmc2130_setDirtyBit(DEFAULT_ICID, i, false);
            tmc2130_shadowRegister[DEFAULT_ICID][i] = 0;
        }

        TMC2130.config->state        = CONFIG_RESET;
        TMC2130.config->configIndex  = 0;
    }

    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.REFL_STEP, Pins.REFR_DIR, NULL);

    return 1;
}

static uint8_t restore()
{
    // Restore the TMC2130 to the state stored in the shadow registers.
    // This can be used to recover the IC configuration after a VM power loss.

    if(TMC2130.config->state != CONFIG_READY)
        return false;

    TMC2130.config->state        = CONFIG_RESTORE;
    TMC2130.config->configIndex  = 0;

    return true;
}

static void enableDriver(DriverState state)
{
    if(state == DRIVER_USE_GLOBAL_ENABLE)
        state = Evalboards.driverEnable;

    if(state == DRIVER_DISABLE)
        HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
    else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
        HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
}

void TMC2130_init(void)
{
    TMC2130.config   = Evalboards.ch2.config;

    // Initialize the hardware pins
    Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0;
    Pins.REFL_STEP       = &HAL.IOs->pins->DIO6;
    Pins.REFR_DIR        = &HAL.IOs->pins->DIO7;
#if defined(LandungsbrueckeV3)
    Pins.AIN_REF_SW      = &HAL.IOs->pins->DIO10_PWM_WL;
    Pins.AIN_REF_PWM     = &HAL.IOs->pins->DIO11_PWM_WH;
#else
    Pins.AIN_REF_SW      = &HAL.IOs->pins->DIO10;
    Pins.AIN_REF_PWM     = &HAL.IOs->pins->DIO11;
#endif
    Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO12;
    Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO13;
    Pins.ENCN_DCO        = &HAL.IOs->pins->DIO14;
    Pins.DIAG0           = &HAL.IOs->pins->DIO15;
    Pins.DIAG1           = &HAL.IOs->pins->DIO16;

    HAL.IOs->config->toInput(Pins.DIAG0);
    HAL.IOs->config->toInput(Pins.DIAG1);
    HAL.IOs->config->toInput(Pins.ENCN_DCO);

    HAL.IOs->config->toOutput(Pins.REFL_STEP);
    HAL.IOs->config->toOutput(Pins.REFR_DIR);
    HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->toOutput(Pins.ENCA_DCIN_CFG5);
    HAL.IOs->config->toOutput(Pins.AIN_REF_PWM);
    HAL.IOs->config->toOutput(Pins.AIN_REF_SW);

    HAL.IOs->config->setLow(Pins.AIN_REF_SW);
    HAL.IOs->config->setLow(Pins.ENCN_DCO);
    HAL.IOs->config->setLow(Pins.ENCA_DCIN_CFG5);

    // Initialize the SPI channel
    TMC2130_SPIChannel       = &HAL.SPI->ch2;
    TMC2130_SPIChannel->CSN  = &HAL.IOs->pins->SPI2_CSN0;

    // Initialize the software StepDir generator
    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.REFL_STEP, Pins.REFR_DIR, NULL);

    Evalboards.ch2.type = (void *)&TMC2130;

    TMC2130.config->callback     = NULL;
    TMC2130.config->channel      = 1;

    Evalboards.ch2.config->reset        = reset;
    Evalboards.ch2.config->restore      = restore;
    Evalboards.ch2.config->state        = CONFIG_RESET;

    Evalboards.ch2.config->configIndex  = 0;

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
    Evalboards.ch2.numberOfMotors       = TMC2130_MOTORS;
    Evalboards.ch2.VMMin                = TMC2130_EVAL_VM_MIN;
    Evalboards.ch2.VMMax                = TMC2130_EVAL_VM_MAX;
    Evalboards.ch2.deInit               = deInit;
    Evalboards.ch2.getMin               = getMin;
    Evalboards.ch2.getMax               = getMax;


#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
    HAL.IOs->config->toOutput(Pins.AIN_REF_PWM);
    Pins.AIN_REF_PWM->configuration.GPIO_Mode = GPIO_Mode_AF4;
#elif defined(LandungsbrueckeV3)
    //Set MUX_1 and MUX_2 to one to connect DIO10 and DIO11 to PWM pins DIO10_A and DIO11_A respectively.
    HAL.IOs->config->toOutput(&HAL.IOs->pins->SW_UART_PWM);
    HAL.IOs->config->setHigh(&HAL.IOs->pins->SW_UART_PWM);

    Pins.AIN_REF_PWM->configuration.GPIO_Mode = GPIO_MODE_AF;
    gpio_af_set(Pins.AIN_REF_PWM->port, GPIO_AF_1, Pins.AIN_REF_PWM->bitWeight);
#endif

    vref = 2000;
    HAL.IOs->config->set(Pins.AIN_REF_PWM);
    Timer.init();
    Timer.setDuty(TIMER_CHANNEL_1, ((float)vref) / VREF_FULLSCALE);

    enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}
