/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/*
 * WARNING: Currently the TMC2300-Eval will send current through the motor for
 * approximately 23ms when the Landungsbruecke is powered on and the TMC2300 has
 * connected supply voltage. This is due to the default driver enable polarity
 * turning the TMC2300 on before the ID detection calls the TMC2300_init()
 * function.
 * Either disconnect the power or the motor prior to startup if your motor is
 * small to prevent damage to it.
 */

#include "boards/Board.h"
#include "tmc/ic/TMC2300/TMC2300.h"
#include "tmc/StepDir.h"

#undef  TMC2300_MAX_VELOCITY
#define TMC2300_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define VM_MIN  18   // VM[V/10] min
#define VM_MAX  121  // VM[V/10] max

#define MOTORS 1
#define DEFAULT_MOTOR  0
#define DEFAULT_ICID  0

#define TIMEOUT_VALUE 10 // 10 ms

// Eval Error defines
#define ERROR_INCONSISTENT_MASK (1 << 0)
#define ERROR_INCONSISTENT_SHIFT 0

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);

static void checkErrors (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static void setStandby(uint8_t enableStandby);
static uint8_t onPinChange(IOPinTypeDef *pin, IO_States state);

static uint8_t restore(void);

static void writeConfiguration();
static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static void enableDriver(DriverState state);

static uint8_t nodeAddress = 0;
static UART_Config *TMC2300_UARTChannel;
static int32_t thigh;

typedef struct
{
    IOPinTypeDef  *DRV_EN;
    IOPinTypeDef  *STEP;
    IOPinTypeDef  *DIR;
    IOPinTypeDef  *MS1;
    IOPinTypeDef  *MS2;
    IOPinTypeDef  *MODE;
    IOPinTypeDef  *DIAG;
    IOPinTypeDef  *STDBY;
} PinsTypeDef;

static PinsTypeDef Pins;

// Usage note: use 1 TypeDef per IC
typedef struct {
    ConfigurationTypeDef *config;

    int32_t registerResetState[TMC2300_REGISTER_COUNT];
    uint8_t registerAccess[TMC2300_REGISTER_COUNT];

    uint8_t slaveAddress;
    uint8_t standbyEnabled;
    uint8_t brownout;
    uint32_t oldTick;
} TMC2300TypeDef;

TMC2300TypeDef TMC2300;


bool tmc2300_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);

    // When we are in standby or in the reset procedure we do not actually write
    // to the IC - we only update the shadow registers. After exiting standby or
    // completing a reset we transition into a restore, which pushes the shadow
    // register contents into the chip.
    if (TMC2300.standbyEnabled)
        return false;
    if (TMC2300.config->state == CONFIG_RESET && readLength== 0 )
        return false;


    int32_t status = UART_readWrite(TMC2300_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

uint8_t tmc2300_getNodeAddress(uint16_t icID)
{
    UNUSED(icID);

    return nodeAddress;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc2300_writeRegister(DEFAULT_ICID, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc2300_readRegister(DEFAULT_ICID, (uint8_t) address);
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    if(motor >= MOTORS)
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
    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    StepDir_moveTo(motor, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    // determine actual position and add numbers of ticks to move
    *ticks += StepDir_getActualPosition(motor);

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;
    int32_t buffer = 0;

    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

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
            *value = StepDir_getActualVelocity(motor);
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
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_IHOLD_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_IHOLD_FIELD, *value);
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
        // Standby
        if (readWrite == READ)
        {
            *value = TMC2300.standbyEnabled;
        } else if (readWrite == WRITE) {
            setStandby(*value);
        }
        break;
    case 23:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            buffer = thigh;
            *value = MIN(0xFFFFF, (1<<24) / ((buffer) ? buffer : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value) ? *value : 1));
            thigh = *value;
        }
        break;
    case 29:
        // Measured Speed
        if(readWrite == READ) {
            buffer = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)tmc2300_fieldRead(DEFAULT_ICID, TMC2300_TSTEP_FIELD));
            *value = (abs(buffer) < 20) ? 0 : buffer;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 30: // UART slave address
        if (readWrite == READ) {
            *value = TMC2300.slaveAddress;
        } else {
            if (*value >= 0 && *value <= 3) {
                TMC2300.slaveAddress = *value;
            } else {
                errors |= TMC_ERROR_VALUE;
            }
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
            *value = 256 >> tmc2300_fieldRead(DEFAULT_ICID, TMC2300_MRES_FIELD);
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
                tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_MRES_FIELD, *value);
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
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_TBL_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID,TMC2300_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID,TMC2300_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_SEMIN_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard2 threshold
        if(readWrite == READ) {
            //*value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_COOLCONF, TMC2300_SGT_MASK, TMC2300_SGT_SHIFT);
            //*value = StepDir_getStallGuardThreshold(motor);
            *value = tmc2300_readRegister(DEFAULT_ICID, TMC2300_SGTHRS);
            //*value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc2300_writeRegister(DEFAULT_ICID, TMC2300_SGTHRS, *value);
            //StepDir_setStallGuardThreshold(motor, *value);
            //tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_COOLCONF, TMC2300_SGT_MASK, TMC2300_SGT_SHIFT, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_CS_ACTUAL_FIELD);
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
            tmc2300_writeRegister(DEFAULT_ICID, TMC2300_TCOOLTHRS, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            buffer = tmc2300_readRegister(DEFAULT_ICID, TMC2300_TCOOLTHRS);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer) ? buffer : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value) ? *value : 1));
            tmc2300_writeRegister(DEFAULT_ICID, TMC2300_TCOOLTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID,TMC2300_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_PWM_GRAD_FIELD, *value);

            // Enable/disable stealthChop accordingly
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_EN_SPREADCYCLE_SHIFT_FIELD, (*value > 0) ? 0 : 1);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID,TMC2300_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_PWM_FREQ_FIELD, *value);
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
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_PWM_AUTOSCALE_FIELD, (*value)? 1:0);
        }
        break;
    case 204:
        // Freewheeling mode
        if(readWrite == READ) {
            *value = tmc2300_fieldRead(DEFAULT_ICID, TMC2300_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            tmc2300_fieldWrite(DEFAULT_ICID, TMC2300_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc2300_readRegister(DEFAULT_ICID, TMC2300_SG_VALUE);
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

static void checkErrors(uint32_t tick)
{
    UNUSED(tick);

    // Error detected -> disable driver for safety
    if(Evalboards.ch2.errors)
        enableDriver(DRIVER_DISABLE);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    uint32_t errors = 0;

    switch(type)
    {
    case 0:  // Read StepDir status bits
        *value = StepDir_getStatus(motor);
        break;
    case 3:
        restore();
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
    HAL.IOs->config->reset(Pins.DRV_EN);

    HAL.IOs->config->reset(Pins.STEP);
    HAL.IOs->config->reset(Pins.DIR);
    HAL.IOs->config->reset(Pins.MS1);
    HAL.IOs->config->reset(Pins.MS2);
    HAL.IOs->config->reset(Pins.DIAG);
    HAL.IOs->config->reset(Pins.STDBY);

    StepDir_deInit();
}

static void setStandby(uint8_t enableStandby)
{
    // En/disable the UART pins depending on standby state
    UART_setEnabled(TMC2300_UARTChannel, !enableStandby);

    HAL.IOs->config->setToState(Pins.STDBY, (enableStandby) ? IOS_LOW : IOS_HIGH);

    if (enableStandby)
    {
        // Just entered standby -> disable the driver
        enableDriver(DRIVER_DISABLE);
    }
    else
    {
        // ToDo: Needed?
        wait(10);
    }
    // start up after standby
    if (TMC2300.standbyEnabled && !enableStandby)
    {
        // Just exited standby -> call the restore
        restore();
    }
    TMC2300.standbyEnabled = enableStandby;
}

static uint8_t onPinChange(IOPinTypeDef *pin, IO_States state)
{
    UNUSED(state);
    return !(pin == Pins.DRV_EN || pin == Pins.STDBY);
}

static uint8_t reset()
{
    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.DIAG);

    // A reset can always happen - even during another reset or restore
    // Reset the dirty bits and wipe the shadow registers
    size_t i;
    for(i = 0; i < TMC2300_REGISTER_COUNT; i++)
    {
        tmc2300_setDirtyBit(DEFAULT_ICID, i, false);
        tmc2300_shadowRegister[DEFAULT_ICID][i] = 0;
    }
    tmc2300_initCache();

    // Activate the reset config mechanism
    TMC2300.config->state        = CONFIG_RESET;
    TMC2300.config->configIndex  = 0;

    return 1;
}

static uint8_t restore()
{
    // Do not interrupt a reset
    // A reset will transition into a restore anyways
    if(TMC2300.config->state == CONFIG_RESET)
        return 0;

    TMC2300.config->state        = CONFIG_RESTORE;
    TMC2300.config->configIndex  = 0;

    return 1;
}

static void enableDriver(DriverState state)
{
    if(state == DRIVER_USE_GLOBAL_ENABLE)
        state = Evalboards.driverEnable;

    if(state == DRIVER_DISABLE)
        HAL.IOs->config->setLow(Pins.DRV_EN);
    else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
        HAL.IOs->config->setHigh(Pins.DRV_EN);
}

static void writeConfiguration()
{
    uint8_t *ptr = &TMC2300.config->configIndex;
    const int32_t *settings;

    if(TMC2300.config->state == CONFIG_RESTORE)
    {
        // Do not restore while in standby
        if (TMC2300.standbyEnabled)
            return;

        settings = *(tmc2300_shadowRegister+0);
        // Find the next restorable register
        while(*ptr < TMC2300_REGISTER_COUNT)
        {
            // If the register is writable and has been written to, restore it
            if (TMC_IS_WRITABLE(tmc2300_registerAccess[*ptr]) && tmc2300_getDirtyBit(DEFAULT_ICID,*ptr))
            {
                break;
            }
            // Otherwise, check next register
            (*ptr)++;
        }

        // Configuration restore complete
        // The driver may only be enabled once the configuration is done
        enableDriver(DRIVER_USE_GLOBAL_ENABLE);
    }
    else
    {
        settings = tmc2300_sampleRegisterPreset;
        // Find the next resettable register
        while((*ptr < TMC2300_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc2300_registerAccess[*ptr]))
        {
            (*ptr)++;
        }
    }

    if(*ptr < TMC2300_REGISTER_COUNT)
    {
        // Reset/restore the found register
        tmc2300_writeRegister(0, *ptr, settings[*ptr]);
        (*ptr)++;
    }
    else
    {

        if (TMC2300.config->state == CONFIG_RESET)
        {
            // Reset done -> Perform a restore
            TMC2300.config->state        = CONFIG_RESTORE;
            TMC2300.config->configIndex  = 0;
            tmc2300_initCache();
        }
        else
        {
            // Restore done -> configuration complete
            TMC2300.config->state = CONFIG_READY;
        }
    }
}

static void periodicJob(uint32_t tick)
{
    UNUSED(tick);
    StepDir_periodicJob(0);

    if(TMC2300.config->state != CONFIG_READY)
    {
        writeConfiguration();
        return;
    }
    // check if IC is brownout
    //Is the IC in an active state an is ready?
    if ((TMC2300.standbyEnabled == 0) && (Evalboards.driverEnable == DRIVER_ENABLE) && (TMC2300.config->state == CONFIG_READY) && (tick - TMC2300.oldTick >= 100))
    {
        //Check if the IOIN register is like expected filled with data
        uint32_t ioin = tmc2300_readRegister(DEFAULT_ICID,  TMC7300_IOIN);
        if (ioin == 0)
        {
            TMC2300.brownout = 1;
            enableDriver(DRIVER_DISABLE);
        }

        // If IC is in bownout and comes back. Go in restore
        else if ((TMC2300.brownout == 1) && (ioin != 0))
        {
            TMC2300.brownout = 0;
            restore();
        }
    }
}

void TMC2300_init(void)
{
    TMC2300.config   = Evalboards.ch2.config;

    // Default slave address: 0
    TMC2300.slaveAddress = 0;

    // Start in standby
    TMC2300.standbyEnabled = 1;

    TMC2300.oldTick      = 0;

    Pins.DRV_EN   = &HAL.IOs->pins->DIO0;
    Pins.DIAG     = &HAL.IOs->pins->DIO1;
    Pins.STDBY    = &HAL.IOs->pins->DIO2;
    Pins.MS1      = &HAL.IOs->pins->DIO3;
    Pins.MS2      = &HAL.IOs->pins->DIO4;
    Pins.MODE     = &HAL.IOs->pins->DIO5;
    Pins.STEP     = &HAL.IOs->pins->DIO6;
    Pins.DIR      = &HAL.IOs->pins->DIO7;

    HAL.IOs->config->toOutput(Pins.DRV_EN);
    HAL.IOs->config->toOutput(Pins.STEP);
    HAL.IOs->config->toOutput(Pins.DIR);
    HAL.IOs->config->toOutput(Pins.MS1);
    HAL.IOs->config->toOutput(Pins.MS2);
    HAL.IOs->config->toOutput(Pins.MODE);
    HAL.IOs->config->toOutput(Pins.STDBY);
    HAL.IOs->config->toInput(Pins.DIAG);

    HAL.IOs->config->setHigh(Pins.STDBY);
    HAL.IOs->config->setLow(Pins.MS1);
    HAL.IOs->config->setLow(Pins.MS2);
    HAL.IOs->config->setLow(Pins.MODE);

    TMC2300_UARTChannel = HAL.UART;
    TMC2300_UARTChannel->pinout = UART_PINS_2;
    TMC2300_UARTChannel->rxtx.baudRate = 57600;
    TMC2300_UARTChannel->rxtx.init();

    Evalboards.ch2.config->reset        = reset;
    Evalboards.ch2.config->restore      = restore;
    Evalboards.ch2.config->state        = CONFIG_RESET;
    Evalboards.ch2.config->configIndex  = 0;

    Evalboards.ch2.config->callback     = NULL;
    Evalboards.ch2.config->channel      = 0;
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
    Evalboards.ch2.userFunction         = userFunction;
    Evalboards.ch2.enableDriver         = enableDriver;
    Evalboards.ch2.checkErrors          = checkErrors;
    Evalboards.ch2.numberOfMotors       = MOTORS;
    Evalboards.ch2.VMMin                = 0; // Set to 0 instead of VM_MIN here since the VM supply isn't connected to the Landungsbruecke
    Evalboards.ch2.VMMax                = VM_MAX;
    Evalboards.ch2.deInit               = deInit;
    Evalboards.ch2.periodicJob          = periodicJob;
    Evalboards.ch2.onPinChange          = onPinChange;


    StepDir_init(STEPDIR_PRECISION);
    StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.DIAG);
    StepDir_setVelocityMax(0, 51200);
    StepDir_setAcceleration(0, 51200);

    // Enter standby at the start
    setStandby(1);

    // The driver will be enabled in configCallback() once the IC is configured
    enableDriver(DRIVER_DISABLE);
};
