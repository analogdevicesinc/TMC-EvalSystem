/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"
#include "../TMC-API/tmc/ic/MAX22215/MAX22215.h"

#define DEFAULT_ICID    0
#define MAX22215_MOTORS 1
#define VM_MIN          80  // VM[V/10] min
#define VM_MAX          480 // VM[V/10] max

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
#define ADC_VM_RES 65535
#elif defined(LandungsbrueckeV3)
#define ADC_VM_RES 4095
#endif

static float dutyCycle = 0.5;
static uint8_t GAIN    = 50;
static timer_channel timerChannel1, timerChannel2;
static MAX22215BusType activeBus = IC_BUS_I2C;
static I2CTypeDef *MAX22215_I2C;
static uint8_t deviceAddress = 0x20;

typedef struct
{

    IOPinTypeDef *SLEEPN;
    IOPinTypeDef *PWM_INT;
    IOPinTypeDef *A0;
    IOPinTypeDef *A1;
    IOPinTypeDef *RLSBRK;
    IOPinTypeDef *DIAG;
} PinsTypeDef;
static PinsTypeDef Pins;

typedef struct
{
    ConfigurationTypeDef *config;
} MAX22215TypeDef;
static MAX22215TypeDef MAX22215;

static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t getVISEN();

/*
 * PWM signal at pin PWM_INT controls the current value at ISENS
 * ISENS value is measured through ADC AIN1. We want to capture
 * the ADC value at the center of PWM signal. IF DC > 50%, we
 * will sample at the midPoint of the HIGH level, and for
 * DC < 50%, we will sample it at the midPoint of the LOW level. We use two
 * channels of the same TIMER, one for duty cycle control and
 * value of the other channel set the ADC capture
 * period through external trigger.
 */
static float midPoint_PWM(float dc);

bool max22215_readWriteI2C(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);

    if (I2CMasterWriteRead(data[0], &data[1], writeLength, &data[2], readLength))
        return true;

    return false;
}

MAX22215BusType max22215_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint8_t max22215_getDeviceAddress(uint16_t icID)
{
    UNUSED(icID);

    return deviceAddress;
}

static void setDutyCycle(float dc)
{
    Timer.setDuty(timerChannel1, dc);

    // Calculate the mid point of the timerChannerl 1 PWM
    float midPWM = midPoint_PWM(dc);
    // Set Timer channel to capture ADC value when timerChannel2 equals to midPoint value
    Timer.setDuty(timerChannel2, midPWM);
}

static uint32_t getVISEN()
{

    uint32_t i = 0, v1 = 0;
    v1 = *HAL.ADCs->AIN1;

    while (i < 1000)
    {
        // Reading raw ADC VISEN values
        v1 += *HAL.ADCs->AIN1;
        i++;
    }
    // 0.66 factor comes from the ISENS amplifier on MAX22215 IC and the resistor divider on LB
    // Value return is multiple of 100 mV
    return ((v1 / (i + 1)) * 1000 * 3.3 * 100) / (ADC_VM_RES * 0.66 * GAIN);
}

static float midPoint_PWM(float dc)
{
    if (dc <= 0.5)
        return (dc + 1) / 2;
    else
        return (dc) / 2;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;

    if (motor >= MAX22215_MOTORS)
        return TMC_ERROR_MOTOR;

    switch (type)
    {
    case 0:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_RLS_BRK_FIELD);
        }
        else if (readWrite == WRITE)
        {
            max22215_fieldWrite(DEFAULT_ICID, MAX22215_RLS_BRK_FIELD, *value);
        }
        break;

    case 1:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_NSLEEP_FIELD);
        }
        else if (readWrite == WRITE)
        {
            max22215_fieldWrite(DEFAULT_ICID, MAX22215_NSLEEP_FIELD, *value);
        }
        break;
    case 2:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_SW_HW_FIELD);
        }
        else if (readWrite == WRITE)
        {
            max22215_fieldWrite(DEFAULT_ICID, MAX22215_SW_HW_FIELD, *value);
        }
        break;
    case 3:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_RESET_FIELD);
        }
        else if (readWrite == WRITE)
        {
            max22215_fieldWrite(DEFAULT_ICID, MAX22215_RESET_FIELD, *value);
        }
        break;
    case 4:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_SLEEP_FIELD);
        }
        break;
    case 5:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_BRK_FIELD);
        }
        break;
    case 6:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_RLS_FIELD);
        }
        break;
    case 7:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_DIAG_FIELD);
        }
        break;
    case 8:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_ENF_DMG_FIELD);
        }
        break;

    case 9:
        // DutyCycle
        dutyCycle = 1.0 - ((float) *value) / 100.0;
        setDutyCycle(dutyCycle);
        break;

    case 10:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_SAFE_DMG_FIELD);
        }
        break;

    case 11:
        if (readWrite == READ)
        {
            *value = max22215_fieldRead(DEFAULT_ICID, MAX22215_STATUS_FLT_PWP_FIELD);
        }
        break;

    case 12:
        if (readWrite == READ)
        {
            *value = getVISEN();
        }
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}
static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);

    switch (type)
    {

    case 0: // VISEN (100 mV)
        *value = getVISEN();
        break;

    case 1: // RAWADC
        *value = *HAL.ADCs->AIN1;
        break;

    default:
        return TMC_ERROR_TYPE;
    }

    return TMC_ERROR_NONE;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);

    uint32_t errors = TMC_ERROR_NONE;

    switch (type)
    {
    case 5: // DutyCycle
        dutyCycle = 1.0 - ((float) *value) / 100.0;
        setDutyCycle(dutyCycle);
        break;

    case 6: // Read current duty cycle
        *value = (int32_t) (dutyCycle * 100);
        break;

    case 8: // Gain
        max22215_fieldWrite(DEFAULT_ICID, MAX22215_GAIN_FIELD, *value);
        if (*value == 0)
            GAIN = 25;
        else if (*value == 1)
            GAIN = 50;
        else if (*value == 2)
            GAIN = 100;
        else if (*value == 3)
            GAIN = 200;
        break;

    case 10: // PWM Frequency
        Timer.setFrequency(timerChannel1, (float) *value);
        setDutyCycle(dutyCycle);
        break;

    case 11: // ODM
        max22215_fieldWrite(DEFAULT_ICID, MAX22215_ODM_FIELD, *value);
        break;

    case 12: // Slew rate
        max22215_fieldWrite(DEFAULT_ICID, MAX22215_SR_FIELD, *value);
        break;

    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    max22215_writeRegister(DEFAULT_ICID, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = max22215_readRegister(DEFAULT_ICID, address);
}
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
    return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
    return handleParameter(READ, motor, type, value);
}

static uint8_t reset()
{
    max22215_fieldWrite(DEFAULT_ICID, MAX22215_SW_HW_FIELD, 1);
    max22215_fieldWrite(DEFAULT_ICID, MAX22215_RESET_FIELD, 1);
    max22215_fieldWrite(DEFAULT_ICID, MAX22215_NSLEEP_FIELD, 1);

    MAX22215.config->state = CONFIG_RESET;

    return true;
}

static uint8_t restore()
{
    max22215_fieldWrite(DEFAULT_ICID, MAX22215_SW_HW_FIELD, 1);
    max22215_fieldWrite(DEFAULT_ICID, MAX22215_RESET_FIELD, 1);
    max22215_fieldWrite(DEFAULT_ICID, MAX22215_NSLEEP_FIELD, 1);

    MAX22215.config->state        = CONFIG_RESTORE;

    return true;
}
static void max22215_writeConfiguration()
{
    if(MAX22215.config->state != CONFIG_READY)
    {
        max22215_fieldWrite(DEFAULT_ICID, MAX22215_SW_HW_FIELD, 1);
        max22215_fieldWrite(DEFAULT_ICID, MAX22215_RESET_FIELD, 1);
        max22215_fieldWrite(DEFAULT_ICID, MAX22215_NSLEEP_FIELD, 1);
    }

    MAX22215.config->state = CONFIG_READY;
}

// Call this periodically
static void periodicJob(uint32_t tick)
{
    UNUSED(tick);
    // Helper function: Configure the next register.
    if(MAX22215.config->state != CONFIG_READY)
    {
        max22215_writeConfiguration();
        return;
    }

}

void MAX22215_init(void)
{
    Pins.SLEEPN  = &HAL.IOs->pins->DIO8;
    Pins.PWM_INT = &HAL.IOs->pins->DIO9;
    Pins.A0      = &HAL.IOs->pins->DIO6;
    Pins.A1      = &HAL.IOs->pins->DIO7;
    Pins.RLSBRK  = &HAL.IOs->pins->DIO14;

    HAL.IOs->config->toOutput(Pins.SLEEPN);
    HAL.IOs->config->toOutput(Pins.A0);
    HAL.IOs->config->toOutput(Pins.A1);
    HAL.IOs->config->toOutput(Pins.RLSBRK);
    HAL.IOs->config->toOutput(Pins.PWM_INT);

    I2C.init();
    MAX22215_I2C = HAL.I2C;

    MAX22215.config        = Evalboards.ch2.config;
    MAX22215.config->state = CONFIG_READY;

    MAX22215.config->reset   = reset;
    MAX22215.config->restore   = restore;

    Evalboards.ch2.userFunction  = userFunction;
    Evalboards.ch2.writeRegister = writeRegister;
    Evalboards.ch2.readRegister  = readRegister;
    Evalboards.ch2.periodicJob   = periodicJob;
    Evalboards.ch2.GAP           = GAP;
    Evalboards.ch2.SAP           = SAP;
    Evalboards.ch2.GIO           = GIO;
    Evalboards.ch2.VMMin         = VM_MIN;
    Evalboards.ch2.VMMax         = VM_MAX;

    HAL.IOs->config->setHigh(Pins.SLEEPN);
    HAL.IOs->config->setLow(Pins.RLSBRK);

    //Setting the slave ID to 0x10
    HAL.IOs->config->setLow(Pins.A0);
    HAL.IOs->config->setLow(Pins.A1);

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
    timerChannel1 = TIMER_CHANNEL_3;
    timerChannel2 = TIMER_CHANNEL_2;
#elif defined(LandungsbrueckeV3)
    timerChannel1 = TIMER_CHANNEL_4;
    timerChannel2 = TIMER_CHANNEL_1;
#endif

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
    Pins.PWM_INT->configuration.GPIO_Mode = GPIO_Mode_AF4;
#elif defined(LandungsbrueckeV3)
    Pins.PWM_INT->configuration.GPIO_Mode = GPIO_MODE_AF;
    gpio_af_set(Pins.PWM_INT->port, GPIO_AF_1, Pins.PWM_INT->bitWeight);
#endif

    HAL.IOs->config->set(Pins.PWM_INT);

    Timer.init();

    // For PWM generation
    Timer.setPeriodMin(timerChannel1, 1000);
    Timer.setFrequencyMin(timerChannel1, 1000);
    Timer.setDuty(timerChannel1, 0.5);

    // For ADC Capture on PWM middle point
    Timer.setTimerAdcTrigger(timerChannel2);
}
