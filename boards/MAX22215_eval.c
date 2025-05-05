/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"
#include "../TMC-API/tmc/ic/MAX22215/MAX22215.h"

#define DEFAULT_ICID  0
#define MAX22215_MOTORS 1
static timer_channel timerChannel1, timerChannel2;
static float dutyCycle = 0.5;
static float midPWM;

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
    #define ADC_VM_RES 65535
#elif defined(LandungsbrueckeV3)
    #define ADC_VM_RES 4095
#endif

typedef struct
{

    IOPinTypeDef  *SLEEPN;
    IOPinTypeDef  *PWM_INT;
    IOPinTypeDef  *A0;
    IOPinTypeDef  *A1;
    IOPinTypeDef  *RLSBRK;
    IOPinTypeDef  *DIAG;
    IOPinTypeDef  *adcMid;
} PinsTypeDef;

static PinsTypeDef Pins;
static MAX22215BusType activeBus = IC_BUS_I2C;
static I2CTypeDef *MAX22215_I2C;
static uint8_t deviceAddress = 0x20;
static uint32_t getFrequency;
static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);
static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static float midPoint_PWM(float dutyCycle);

bool max22215_readWriteI2C(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);

    if(I2CMasterWriteRead(data[0],&data[1],writeLength,&data[2],readLength))
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

static void setDutyCycle(float dutyCycle)
{

    Timer.setDuty(timerChannel1, dutyCycle);
    midPWM = midPoint_PWM(dutyCycle);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, midPWM);

}

static float midPoint_PWM(float dutyCycle)
{
    if(dutyCycle <= 0.5)
        return (((dutyCycle+1)/2) * Timer.getPeriod(timerChannel2));
    else
        return (((dutyCycle)/2) * Timer.getPeriod(timerChannel2));
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= MAX22215_MOTORS)
        return TMC_ERROR_MOTOR;

    int32_t tempValue;

    switch(type)
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

    case 9:
        // DutyCycle
        dutyCycle = ((float)*value) / 100.0;
        setDutyCycle(dutyCycle);
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, midPWM);

        break;

    }
    return errors;
}
static uint32_t GIO(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);

    switch(type) {

    case 0: // VISEN (mV)
        uint32_t v1 = *HAL.ADCs->AIN0;
        *value = ((v1)*1000)/ADC_VM_RES;
        break;
    case 1: // Vm (mV)
            uint32_t Vm = *HAL.ADCs->VM;
            *value = ((((Vm)*73330)/ADC_VM_RES)+200);
            break;

    default:
        return TMC_ERROR_TYPE;
    }

    return TMC_ERROR_NONE;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    UNUSED(*value);
    uint32_t errors = TMC_ERROR_NONE;

    switch(type)
    {
    case 5: // DutyCycle
       dutyCycle = ((float)*value) / 100.0;
       setDutyCycle(dutyCycle);
       break;

    case 10: // PWM Frequency
        Timer.setFrequency(timerChannel1, (float)*value);
        setDutyCycle(dutyCycle);
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

static void timer_overflow(timer_channel channel)
{
    UNUSED(channel);

    // RAMDebug
    debug_nextProcess();
}

void MAX22215_init(void)
{
    Pins.SLEEPN          = &HAL.IOs->pins->DIO8;
    Pins.PWM_INT         = &HAL.IOs->pins->DIO9;
    Pins.A0              = &HAL.IOs->pins->DIO6;
    Pins.A1              = &HAL.IOs->pins->DIO7;
    Pins.RLSBRK          = &HAL.IOs->pins->DIO14;
    Pins.adcMid          = &HAL.IOs->pins->DIO11_PWM_WH;

    HAL.IOs->config->toOutput(Pins.SLEEPN);
    HAL.IOs->config->toOutput(Pins.A0);
    HAL.IOs->config->toOutput(Pins.A1);
    HAL.IOs->config->toOutput(Pins.RLSBRK);
    HAL.IOs->config->toOutput(Pins.PWM_INT);
    HAL.IOs->config->toOutput(Pins.adcMid);

    I2C.init();
	MAX22215_I2C = HAL.I2C;

    Evalboards.ch2.userFunction                  = userFunction;
    Evalboards.ch2.writeRegister                 = writeRegister;
    Evalboards.ch2.readRegister                  = readRegister;
    Evalboards.ch2.GAP                           = GAP;
    Evalboards.ch2.SAP                           = SAP;
    Evalboards.ch2.GIO                           = GIO;

    HAL.IOs->config->setHigh(Pins.SLEEPN);
    HAL.IOs->config->setLow(Pins.RLSBRK);
    HAL.IOs->config->setLow(Pins.PWM_INT);

    //Setting the slave ID to 0x10
    HAL.IOs->config->setLow(Pins.A0);
    HAL.IOs->config->setLow(Pins.A1);

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
    timerChannel1 = TIMER_CHANNEL_3;
#elif defined(LandungsbrueckeV3)
    timerChannel1 = TIMER_CHANNEL_4;
    timerChannel2 = TIMER_CHANNEL_1;
#endif

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
    Pins.PWM_INT->configuration.GPIO_Mode = GPIO_Mode_AF4;
#elif defined(LandungsbrueckeV3)
    Pins.PWM_INT->configuration.GPIO_Mode  = GPIO_MODE_AF;
    gpio_af_set(Pins.PWM_INT->port, GPIO_AF_1, Pins.PWM_INT->bitWeight);

    Pins.adcMid->configuration.GPIO_Mode  = GPIO_MODE_AF;
    gpio_af_set(Pins.adcMid->port, GPIO_AF_1, Pins.adcMid->bitWeight);
#endif

    HAL.IOs->config->set(Pins.PWM_INT);
    HAL.IOs->config->set(Pins.adcMid);

    Timer.overflow_callback = timer_overflow;
    Timer.init();

    // For PWM generation
    Timer.setPeriodMin(timerChannel1, 1000);
    Timer.setFrequencyMin(timerChannel1, 1000);
    Timer.setDuty(timerChannel1, 0.5);

    // For ADC Capture on PWM middle point
    Timer.setDuty(timerChannel2, 0.5);

    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_O2CPRE);

    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL, ADC_EXTTRIG_ROUTINE_T0_CH2);
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);

}
