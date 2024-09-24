/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/*
 * WARNING: Currently the TMC7300-Eval will send current through the motor for
 * approximately 23ms when the Landungsbruecke is powered on and the TMC7300 has
 * connected supply voltage. This is due to the default driver enable polarity
 * turning the TMC7300 on before the ID detection calls the TMC7300_init()
 * function.
 * Either disconnect the power or the motor prior to startup if your motor is
 * small to prevent damage to it.
 *
 */

#include "boards/Board.h"
#include "tmc/ic/TMC7300/TMC7300.h"

#define VM_MIN  18   // VM[V/10] min
#define VM_MAX  110  // VM[V/10] max

#define MOTORS          1
#define DEFAULT_MOTOR   0
#define DEFAULT_ICID    0

#define TIMEOUT_VALUE 10 // 10 ms

#define CONSISTENCY_CHECK_INTERVAL 1000 // [ms]

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

static uint8_t tmc7300_consistencyCheck();
static void checkErrors (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static void setStandby(uint8_t enabled);
static void writeConfiguration();
static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static uint8_t restore(void);
static void enableDriver(DriverState state);

static uint8_t nodeAddress = 0;
static UART_Config *TMC7300_UARTChannel;

typedef struct
{
    IOPinTypeDef  *DRV_EN;
    IOPinTypeDef  *MS1;
    IOPinTypeDef  *MS2;
    IOPinTypeDef  *DIAG;
    IOPinTypeDef  *STDBY;
} PinsTypeDef;

static PinsTypeDef Pins;

// Usage note: use 1 TypeDef per IC
typedef struct {
    ConfigurationTypeDef *config;
    uint8_t slaveAddress;
    uint8_t standbyEnabled;
    uint8_t brownout;
    uint32_t oldTick;
} TMC7300TypeDef;

TMC7300TypeDef TMC7300;


bool tmc7300_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{

    // When we are in standby or in the reset procedure we do not actually write
    // to the IC - we only update the shadow registers. After exiting standby or
    // completing a reset we transition into a restore, which pushes the shadow
    // register contents into the chip.
    if (TMC7300.standbyEnabled)
        return false;
    if (TMC7300.config->state == CONFIG_RESET && readLength== 0 )
                return false;

    UNUSED(icID);

    int32_t status = UART_readWrite(TMC7300_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

uint8_t tmc7300_getNodeAddress(uint16_t icID)
{
    UNUSED(icID);

    return nodeAddress;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc7300_writeRegister(DEFAULT_ICID, (uint8_t) address, value );
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc7300_readRegister(DEFAULT_ICID, (uint8_t) address);
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    UNUSED(velocity);

    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

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
    UNUSED(position);

    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0:
        // Target PWM A duty cycle (in %)
        if (readWrite == READ) {
            int32_t tmp = tmc7300_fieldRead(DEFAULT_ICID, TMC7300_PWM_A_FIELD);

            // Change the unsigned read to a signed value
            tmp = CAST_Sn_TO_S32(tmp, 9);
            // Scale the internal value [-255 ; +255] to percent [-100% ; +100%]
            // For that we need to round away from zero - since the division in
            // the write rounded towards zero.
            if (tmp >= 0) {
                // Round up for positive values
                *value = (tmp * 100 + 254) / 255;
            } else {
                // Round down for negative values
                *value = (tmp * 100 - 254) / 255;
            }
        } else if(readWrite == WRITE) {
            if (abs(*value) <= 100) {
                // Scale the duty cycle in percent [-100% ; +100%] to internal values [-255 ; +255]
                *value = *value * 255 / 100;
                tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_PWM_A_FIELD, *value);
            } else {
                errors |= TMC_ERROR_VALUE;
            }
        }
        break;
    case 1:
        // Target PWM B duty cycle (in %)
        if (readWrite == READ) {
            int32_t tmp = tmc7300_fieldRead(DEFAULT_ICID, TMC7300_PWM_B_FIELD);

            // Change the unsigned read to a signed value
            tmp = CAST_Sn_TO_S32(tmp, 9);
            // Scale the internal value [-255 ; +255] to percent [-100% ; +100%]
            // For that we need to round away from zero - since the division in
            // the write rounded towards zero.
            if (tmp >= 0) {
                // Round up for positive values
                *value = (tmp * 100 + 254) / 255;
            } else {
                // Round down for negative values
                *value = (tmp * 100 - 254) / 255;
            }
        } else if(readWrite == WRITE) {
            if (abs(*value) <= 100) {
                // Scale the duty cycle in percent [-100% ; +100%] to internal values [-255 ; +255]
                *value = *value * 255 / 100;
                tmc7300_fieldWrite(DEFAULT_ICID,TMC7300_PWM_B_FIELD, *value);
            } else {
                errors |= TMC_ERROR_VALUE;
            }
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = tmc7300_fieldRead(DEFAULT_ICID, TMC7300_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby
        if (readWrite == READ) {
            *value = TMC7300.standbyEnabled;
        } else if(readWrite == WRITE) {
            setStandby(*value);
        }
        break;
    case 8:
        // Double motor mode
        if (readWrite == READ) {
            *value = !tmc7300_fieldRead(DEFAULT_ICID, TMC7300_PAR_MODE_FIELD);
        } else if (readWrite == WRITE) {
            tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_PAR_MODE_FIELD, (*value)? 0:1);
        }
        break;
    case 162:
        // Chopper blank time
        if(readWrite == READ) {
            *value = tmc7300_fieldRead(DEFAULT_ICID, TMC7300_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_TBL_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc7300_fieldRead(DEFAULT_ICID, TMC7300_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_PWM_FREQ_FIELD, *value);
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
            if (tmc7300_fieldRead(DEFAULT_ICID, TMC7300_MOTORRUN_FIELD) == 0)
            {
                // Motorrun == 0 -> Freewheeling is locked to normal mode (0)
                *value = 0;
            }
            else
            {
                // Motorrun == 1 -> Freewheeling controlled by the freewheel field
                *value = tmc7300_fieldRead(DEFAULT_ICID,TMC7300_FREEWHEEL_FIELD);
            }
        } else if(readWrite == WRITE) {
            // Unlock the freewheeling options by setting motorrun to 1
            tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_MOTORRUN_FIELD, 1);
            // Set the freewheeling option
            tmc7300_fieldWrite(DEFAULT_ICID, TMC7300_FREEWHEEL_FIELD, *value);
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

static uint8_t tmc7300_consistencyCheck()
{
    // Config has not yet been written -> it can't be consistent
    if(TMC7300.config->state != CONFIG_READY)
        return 0;

    // Standby is enabled -> registers can't be accessed
    if(TMC7300.standbyEnabled)
        return 0;

    // If the PWM_DIRECT bit is no longer set, report an error
    if (tmc7300_fieldRead(DEFAULT_ICID, TMC7300_PWM_DIRECT_FIELD) == 0)
        return 1;

    // No inconsistency detected
    return 0;
}
static void checkErrors(uint32_t tick)
{
    static uint32_t tick_old = 0;
    if((tick - tick_old) >= CONSISTENCY_CHECK_INTERVAL) {
        Evalboards.ch2.errors = FIELD_SET(Evalboards.ch2.errors, ERROR_INCONSISTENT_MASK, ERROR_INCONSISTENT_SHIFT, tmc7300_consistencyCheck());
        tick_old = tick;
    }

    if(Evalboards.ch2.errors)
        enableDriver(DRIVER_DISABLE);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    uint32_t errors = 0;

    switch(type)
    {
    case 1:
        TMC7300.slaveAddress = (*value) & 0xFF;
        break;
    case 2:
        *value = TMC7300.slaveAddress;
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

    HAL.IOs->config->reset(Pins.MS1);
    HAL.IOs->config->reset(Pins.MS2);
    HAL.IOs->config->reset(Pins.DIAG);
    HAL.IOs->config->reset(Pins.STDBY);
}

static void setStandby(uint8_t enableStandby)
{
    // En/disable the UART pins depending on standby state
    UART_setEnabled(TMC7300_UARTChannel, !enableStandby);

    HAL.IOs->config->setToState(Pins.STDBY, (enableStandby)? IOS_LOW:IOS_HIGH);

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
    // Update the APIs internal standby state
    if (TMC7300.standbyEnabled && !enableStandby)
    {
        // Just exited standby -> call the restore
        restore();
    }
    TMC7300.standbyEnabled = enableStandby;
}

static uint8_t reset()
{
    // A reset can always happen - even during another reset or restore

    // Reset the dirty bits and wipe the shadow registers
    size_t i;
    for(i = 0; i < TMC7300_REGISTER_COUNT; i++)
    {
        tmc7300_setDirtyBit(DEFAULT_ICID, i, false);
        tmc7300_shadowRegister[DEFAULT_ICID][i] = 0;
    }
    tmc7300_initCache();

    // Activate the reset config mechanism
    TMC7300.config->state        = CONFIG_RESET;
    TMC7300.config->configIndex  = 0;

    return 1;
}

static uint8_t restore()
{
    // Do not interrupt a reset
    // A reset will transition into a restore anyways
    if(TMC7300.config->state == CONFIG_RESET)
    {
        return 0;
    }

    TMC7300.config->state        = CONFIG_RESTORE;
    TMC7300.config->configIndex  = 0;
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
    uint8_t *ptr = &TMC7300.config->configIndex;
    const int32_t *settings;

    if(TMC7300.config->state == CONFIG_RESTORE)
    {
        // Do not restore while in standby
        if (TMC7300.standbyEnabled)
            return;

        settings = *(tmc7300_shadowRegister+0);
        // Find the next restorable register
        while(*ptr < TMC7300_REGISTER_COUNT)
        {
            // If the register is writable and has been written to, restore it
            if (TMC_IS_WRITABLE(tmc7300_registerAccess[*ptr]) && tmc7300_getDirtyBit(DEFAULT_ICID,*ptr))
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
        settings = tmc7300_sampleRegisterPreset;
        // Find the next resettable register
        while((*ptr < TMC7300_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc7300_registerAccess[*ptr]))
        {
            (*ptr)++;
        }
    }

    if(*ptr < TMC7300_REGISTER_COUNT)
    {
        // Reset/restore the found register
        tmc7300_writeRegister(DEFAULT_ICID, *ptr, settings[*ptr]);
        (*ptr)++;
    }
    else
    {

        if (TMC7300.config->state == CONFIG_RESET)
        {
            // Reset done -> Perform a restore
            TMC7300.config->state        = CONFIG_RESTORE;
            TMC7300.config->configIndex  = 0;
            tmc7300_initCache();
        }
        else
        {
            // Restore done -> configuration complete
            TMC7300.config->state = CONFIG_READY;
        }
    }
}

static void periodicJob(uint32_t tick)
{
    // Call the periodic API function of the TMC7300
    if(TMC7300.config->state != CONFIG_READY)
    {
        writeConfiguration();
    }


    // check if IC is brownout
    //Is the IC in an active state an is ready?
    if ((TMC7300.standbyEnabled == 0) && (Evalboards.driverEnable == DRIVER_ENABLE) && (TMC7300.config->state == CONFIG_READY) && (tick - TMC7300.oldTick >= 100))
    {
        //Check if the IOIN register is like expected filled with data
        uint32_t ioin = tmc7300_readRegister(DEFAULT_ICID,  TMC7300_IOIN);
        if (ioin == 0)
        {
            TMC7300.brownout = 1;
            enableDriver(DRIVER_DISABLE);
        }

        // If IC is in bownout and comes back. Go in restore
        else if ((TMC7300.brownout == 1) && (ioin != 0))
        {
            TMC7300.brownout = 0;
            restore();
        }
        TMC7300.oldTick = tick;
    }

}

void TMC7300_init(void)
{
    TMC7300.config   = Evalboards.ch2.config;

    Pins.DRV_EN   = &HAL.IOs->pins->DIO0;
    Pins.MS1      = &HAL.IOs->pins->DIO3;
    Pins.MS2      = &HAL.IOs->pins->DIO4;
    Pins.DIAG     = &HAL.IOs->pins->DIO1;
    Pins.STDBY    = &HAL.IOs->pins->DIO2;

    TMC7300.oldTick      = 0;


    HAL.IOs->config->toOutput(Pins.DRV_EN);
    HAL.IOs->config->toOutput(Pins.STDBY);
    HAL.IOs->config->toOutput(Pins.MS1);
    HAL.IOs->config->toOutput(Pins.MS2);
    HAL.IOs->config->toInput(Pins.DIAG);

    HAL.IOs->config->setLow(Pins.STDBY);
    HAL.IOs->config->setLow(Pins.MS1);
    HAL.IOs->config->setLow(Pins.MS2);

    TMC7300_UARTChannel = HAL.UART;
    TMC7300_UARTChannel->pinout = UART_PINS_2;
    TMC7300_UARTChannel->rxtx.baudRate = 57600;
    TMC7300_UARTChannel->rxtx.init();

    Evalboards.ch2.config->reset        = reset;
    Evalboards.ch2.config->restore      = restore;
    Evalboards.ch2.config->state        = CONFIG_RESET;
    Evalboards.ch2.config->configIndex  = 0;
    Evalboards.ch2.config->callback     = NULL;
    Evalboards.ch2.config->channel      = 0;
    Evalboards.ch2.config->configIndex  = 0;


    // Default slave address: 0
    TMC7300.slaveAddress = 0;

    // Start in standby
    TMC7300.standbyEnabled = 1;

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

    // Enter standby at the start
    setStandby(1);

    // The driver will be enabled in configCallback() once the IC is configured
    enableDriver(DRIVER_DISABLE);
};
