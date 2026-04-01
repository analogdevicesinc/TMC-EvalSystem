/*******************************************************************************
* Copyright © 2025 Analog Devices, Inc.
*******************************************************************************/

#include <stdbool.h>

#include "TMC-API/tmc/ic/TMC6460/TMC6460.h"
#include "boards/Board.h"
#include "tmc/BoardAssignment.h" // For the Board IDs
#include "tmc/RAMDebug.h"

#define DEFAULT_MOTOR   0
#define TMC6460_MOTORS  1

#define TMC6460_VM_MIN 43  // [V/10], 4.5V - 5% margin
#define TMC6460_VM_MAX 396 // [V/10], 36V + 10% margin

// The minimum mantissa needed for the LandungsbrueckeV3 to support the UART
#define UART_MIN_MANTISSA 8

#undef RAMDEBUG_FREQUENCY
#define RAMDEBUG_FREQUENCY 10000 // [Hz]

#define RTMI_MAX_CHANNELS 8

static bool useRTMIRAMDebug = true;

typedef struct
{
    IOPinTypeDef  *DRV_EN;
    IOPinTypeDef  *nSLEEP;
} PinsTypeDef;

static PinsTypeDef Pins;

static ConfigurationTypeDef *TMC6460_config;
static SPIChannelTypeDef *TMC6460_SPIChannel;
static UART_Config *TMC6460_UARTChannel;

static bool isRTMIEnabled = false;
static uint32_t uartControlRegister = 0;
static uint32_t uartBaudrateSetting = 0;
static bool     uartUseAutobaud = false;
static uint32_t uartErrorCount = 0;
static bool     uartIsNormalCRCEnabled = false;
static bool     uartIsRTMICRCEnabled = false;
static bool isBaudrateAvailable(uint32_t mantissaSetting);
static bool isUARTSettingsValid(bool isCommsUART, bool isAutobaud, bool isRTMI);
static void updateUARTInterface();
static void updateCRCSettings(bool enableNormalCRC, bool enableRTMICRC);
static bool enableRTMI(bool enable);
static uint8_t getRegisterRTMIChannel(uint16_t address);

static void timer_overflow(timer_channel channel);
static bool fwdTmclCommand(TMCLCommandTypeDef *ActualCommand, TMCLReplyTypeDef *ActualReply);

static void rtmiramdebug_process();

static uint32_t rtmiramdebug_divisor = 0;

static enum TMC6460BusType activeBus = TMC6460_BUS_SPI;

// => TMC-API wrapper
void tmc6460_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);

    TMC6460_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc6460_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);

    // UART - the helper function UART_readWrite causes trouble in multiple ways
    // It flushes the buffers, causing RTMI trouble. It also has a workaround 2ms delay, making RAMDebug not perform well
    // For now we just reimplement it, long-term we should make UART_readWrite behave better
    TMC6460_UARTChannel->rxtx.txN(data, writeLength);

    // Abort early if no data needs to be read back
    if (readLength <= 0)
        return true;

    // Wait for reply with timeout limit
    uint32_t timestamp = systick_getTick();
    while(TMC6460_UARTChannel->rxtx.bytesAvailable() < readLength)
    {
        if(timeSince(timestamp) > TMC6460_UARTChannel->timeout)
        {
            // UART timeout -> Flush the UART buffer
            TMC6460_UARTChannel->rxtx.clearBuffers();

            // Bump the error counter
            uartErrorCount++;

            // Report the timeout
            return false;
        }
    }

    // Grab the read data
    TMC6460_UARTChannel->rxtx.rxN(data, readLength);

    return true;
}

enum TMC6460BusType tmc6460_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint32_t tmc6460_availableBytes(uint16_t icID)
{
    UNUSED(icID);

    return TMC6460_UARTChannel->rxtx.bytesAvailable();
}

bool tmc6460_isNormalCRCEnabled(uint16_t icID)
{
    UNUSED(icID);

    return uartIsNormalCRCEnabled;
}

bool tmc6460_isRTMICRCEnabled(uint16_t icID)
{
    UNUSED(icID);

    return uartIsRTMICRCEnabled;
}
// <= TMC-API wrapper

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    UNUSED(motor);
    UNUSED(velocity);
    return 0;
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
    UNUSED(motor);
    UNUSED(position);
    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    UNUSED(motor);
    UNUSED(ticks);
    return TMC_ERROR_NONE;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    UNUSED(readWrite);
    UNUSED(value);

    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= TMC6460_MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0: // Bus type
        if (readWrite == READ)
        {
            *value = activeBus;
        }
        else if (readWrite == WRITE)
        {
            if (*value >= TMC6460_BUS_END_)
                return TMC_ERROR_VALUE;

            if (!isUARTSettingsValid(*value == TMC6460_BUS_UART, uartUseAutobaud, useRTMIRAMDebug))
            {
                // RTMI won't send out data if autobaud is enabled and no UART data is sent
                // to the TMC6460 due to SPI being the active bus
                *value = 0;
                return TMC_ERROR_NOT_AVAILABLE;
            }

            if (*value == TMC6460_BUS_UART && !isBaudrateAvailable(uartBaudrateSetting))
            {
                // We're currently configured at a too fast baudrate for the Landungsbruecke
                // We cannot enable Landungsbruecke UART based features.
                *value = 1;
                return TMC_ERROR_NOT_AVAILABLE;
            }

            activeBus = *value;

            // Ensure UART interface is in the right state
            updateUARTInterface();
        }
        break;
    case 1: // RTMI-based RAMDebug
        if (readWrite == READ)
        {
            *value = useRTMIRAMDebug;
        }
        else if (readWrite == WRITE)
        {
            if (!isUARTSettingsValid(activeBus == TMC6460_BUS_UART, uartUseAutobaud, *value))
            {
                // RTMI won't send out data if autobaud is enabled and no UART data is sent
                // to the TMC6460 due to SPI being the active bus
                *value = 0;
                return TMC_ERROR_NOT_AVAILABLE;
            }

            if (*value == 1 && !isBaudrateAvailable(uartBaudrateSetting))
            {
                // We're currently configured at a too fast baudrate for the Landungsbruecke
                // We cannot enable Landungsbruecke UART based features.
                *value = 1;
                return TMC_ERROR_NOT_AVAILABLE;
            }

            if (*value == 0)
            {
                // In case RTMI was turned on, turn it off
                // Note: If we didn't turn RTMI on, this won't do anything.
                enableRTMI(false);
            }

            useRTMIRAMDebug = *value != 0;

            // Ensure UART interface is in the right state
            updateUARTInterface();
        }
        break;
    case 2: // Internal/External clock
        if (readWrite == READ)
        {
            tmc6460_readField(DEFAULT_MOTOR, TMC6460_CLK_CTRL_CONFIG_PLL_SRC_FIELD, (uint32_t *) value);
        }
        else if (readWrite == WRITE)
        {
            // Configure either external 16 MHz clock (CLK_IN pin),
            // or the internal 15 MHz oscillator as PLL input.
            bool enableExtClock = *value != 0;
            uint8_t divider = enableExtClock? (16-1): (15-1);

            uint32_t clkConfig = 0;
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_CLOCK_DIVIDER_FIELD, divider);
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_CLK_FSM_EN_FIELD,    1);
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_PWM_CLK_EN_FIELD,    1);
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_ADC_CLK_EN_FIELD,    1);
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_PLL_EN_FIELD,        1);
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_PLL_SRC_FIELD,       enableExtClock);
            clkConfig = tmc6460_updateField(clkConfig, TMC6460_CLK_CTRL_CONFIG_COMMIT_FIELD,        1);

            // Send the clock update
            tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_CLK_CTRL_CONFIG, clkConfig);

            // Wait for the update to complete
            wait(1);

            // Reset the error counter
            uartErrorCount = 0;

            // Ensure communication buffers are empty
            TMC6460_UARTChannel->rxtx.clearBuffers();
        }
        break;
    case 3: // Baudrate configuration
        if (readWrite == READ)
        {
            *value = uartBaudrateSetting;
        }
        else if (readWrite == WRITE)
        {
            // Minimum Mantissa limit value is 5
            if (*value < 5)
                return TMC_ERROR_VALUE;

            // Maximum value is whatever can fit in the register field
            if (!tmc6460_fieldInRange(TMC6460_UART_CONTROL_MANTISSA_LIMIT_FIELD, *value))
                return TMC_ERROR_VALUE;

            if (activeBus == TMC6460_BUS_UART || useRTMIRAMDebug)
            {
                // The Landungsbruecke is using UART to communicate.
                // So we have to limit the baudrate to whatever the Landungsbruecke supports
                if (!isBaudrateAvailable(*value))
                    return TMC_ERROR_NOT_AVAILABLE;
            }

            // Do not allow baudrate switches while RTMI is active
            if (isRTMIEnabled)
                return TMC_ERROR_NOT_AVAILABLE;

            // Configure the mantissa limit
            uartControlRegister = tmc6460_updateField(
                    uartControlRegister,
                    TMC6460_UART_CONTROL_MANTISSA_LIMIT_FIELD,
                    *value
            );

            // Update the UART baudrate settings
            tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, uartControlRegister);

            // Update the Landungsbruecke UART settings
            uint32_t baudrate = 60000000 / *value;
            // ToDo: Get a dedicated baudrate setter function?
            TMC6460_UARTChannel->rxtx.baudRate = baudrate;
            TMC6460_UARTChannel->rxtx.init();

            uartBaudrateSetting = *value;

            // Reset the error counter
            uartErrorCount = 0;
        }
        break;
    case 4: // UART error rate
        if (readWrite == READ)
        {
            *value = uartErrorCount;
        }
        break;
    case 5: // UART autobaud
        if (readWrite == READ)
        {
            *value = uartUseAutobaud;
        }
        else
        {
            if (!isUARTSettingsValid(activeBus == TMC6460_BUS_UART, *value, useRTMIRAMDebug))
            {
                // RTMI won't send out data if autobaud is enabled and no UART data is sent
                // to the TMC6460 due to SPI being the active bus
                *value = 0;
                return TMC_ERROR_NOT_AVAILABLE;
            }

            // Do not allow autobaud switches while RTMI is active
            if (isRTMIEnabled)
            {
                *value = 1;
                return TMC_ERROR_NOT_AVAILABLE;
            }

            uartUseAutobaud = (*value) != 0;

            // Update the UART autobaud settings
            uartControlRegister = tmc6460_updateField(
                    uartControlRegister,
                    TMC6460_UART_CONTROL_AUTOBAUD_EN_FIELD,
                    uartUseAutobaud
            );
            tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, uartControlRegister);

            // Reset the error counter
            uartErrorCount = 0;
        }
        break;
    case 6: // UART normal CRC
        if (readWrite == READ)
        {
            *value = uartIsNormalCRCEnabled;
        }
        else
        {
            updateCRCSettings((*value) != 0, uartIsRTMICRCEnabled);
        }
        break;
    case 7: // UART RTMI CRC
        if (readWrite == READ)
        {
            *value = uartIsRTMICRCEnabled;
        }
        else
        {
            if (isRTMIEnabled)
            {
                // We do not allow changing RTMI CRC setting while RTMI is active.
                // While this is in principle possible, it would require
                // synchronizing with the RTMI data stream by only considering
                // RTMI active once the write counter increments.
                return TMC_ERROR_NOT_AVAILABLE;
            }

            updateCRCSettings(uartIsNormalCRCEnabled, (*value) != 0);
        }
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    UNUSED(value);
    return 0;
}

static void periodicJob(uint32_t actualSystick)
{
    UNUSED(actualSystick);

    // Process any RTMI packets
    rtmiramdebug_process();
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);

    if (isRTMIEnabled && activeBus == TMC6460_BUS_UART)
    {
        // If we're currently using the RTMI stream and are about
        // to communicate via UART, try using the stream write
        // instead of the regular write.
        // This is slightly less data on the UART (5 instead of 6 bytes)
        uint8_t rtmi_channel = getRegisterRTMIChannel(address);
        if (rtmi_channel != RTMI_MAX_CHANNELS)
        {
            // Target register is configured as an RTMI target
            // -> Send a stream write request
            tmc6460_writeRTMIStreamedRegister(DEFAULT_MOTOR, rtmi_channel, value);
            return;
        }
    }

    tmc6460_writeRegister(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);

    tmc6460_readRegister(DEFAULT_MOTOR, address, (uint32_t *) value);
}

static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
    return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
    return handleParameter(READ, motor, type, value);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(type);
    UNUSED(motor);
    UNUSED(value);

    return TMC_ERROR_TYPE;
}

static bool isBaudrateAvailable(uint32_t mantissaSetting)
{
    return mantissaSetting >= UART_MIN_MANTISSA;
}

static bool isUARTSettingsValid(bool isCommsUART, bool isAutobaud, bool isRTMI)
{
    // The combo of using SPI for communication, autobaud, and RTMI is not allowed.
    // This is due to the autobaud not defaulting to a baudrate before any UART
    // datagram being received. In SPI mode there won't be any datagrams to
    // initialize autobaud with, and RTMI won't ever send any data.
    return isCommsUART || !isAutobaud || !isRTMI;
}

static void updateUARTInterface()
{
    bool uartInUse = (useRTMIRAMDebug || activeBus == TMC6460_BUS_UART);

    UART_setEnabled(TMC6460_UARTChannel, uartInUse);
}

static void updateCRCSettings(bool enableNormalCRC, bool enableRTMICRC)
{
    // The normal CRC setting en/disables CRC bytes for:
    // - Read requests
    // - Read replies
    // - Normal write requests (but not replies!)
    // The RTMI CRC setting en/disables CRC bytes for:
    // - RTMI datagrams
    // - Normal write replies
    // - RTMI write requests // Todo: Check

    // If no setting changed, we don't need to do anything
    if (enableNormalCRC == uartIsNormalCRCEnabled && enableRTMICRC == uartIsRTMICRCEnabled)
        return;

    // Modify the setting bits
    uartControlRegister = tmc6460_updateField(
            uartControlRegister,
            TMC6460_UART_CONTROL_NORMAL_CRC_EN_FIELD,
            enableNormalCRC
    );
    uartControlRegister = tmc6460_updateField(
            uartControlRegister,
            TMC6460_UART_CONTROL_RTMI_CRC_EN_FIELD,
            enableRTMICRC
    );

    // Update the RTMI CRC setting
    // -> The write request still happens with the old normal CRC setting
    // -> The write reply will be received with the new RTMI CRC setting
    uartIsRTMICRCEnabled = enableRTMICRC;

    // Perform the updating write request
    tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, uartControlRegister);

    // Update the Normal CRC setting
    uartIsNormalCRCEnabled = enableNormalCRC;
}

static bool enableRTMI(bool enable)
{
    if (isRTMIEnabled == enable)
        return true;

    bool retVal = true;

    if (enable)
    {
        // Grab the UART->CONTROL register and store it before enabling RTMI
        // Once the RTMI is on we might not be able to do a normal read without overloading
        // the UART bus.

        tmc6460_readRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, &uartControlRegister);

        // Configure the downsampling
        uartControlRegister = tmc6460_updateField(
                uartControlRegister,
                TMC6460_UART_CONTROL_RTMI_SAMPLING_FIELD,
                rtmiramdebug_divisor
        );

        // Clear the RTMI_INTERRUPT flag
        tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_EVENTS, TMC6460_UART_EVENTS_RTMI_INTERRUPT_EVENT_MASK);

        // Enable the RTMI
        tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, uartControlRegister | TMC6460_UART_CONTROL_RTMI_EN_MASK);
    }
    else
    {
        // Disable the RTMI
        tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, uartControlRegister & ~TMC6460_UART_CONTROL_RTMI_EN_MASK);

        // Wait a short moment. When this disabling-RTMI-sequence is run
        // via UART, the above write won't generated a response.
        // Without this delay, we would immediately send the read request,
        // and the TMC6460 will sometimes not respond to that.
        for (volatile int i = 0; i < 1000; i++);

        // When using UART, the below read request serves as a sequence point.
        // Once the read request gets acknowledged by the reply arriving,
        // we know the above write has hit the TMC6460.
        // This allows us to detect the end of the RTMI data stream reliably
        uint32_t tmp = 0;
        tmc6460_readField(DEFAULT_MOTOR, TMC6460_UART_EVENTS_RTMI_INTERRUPT_EVENT_FIELD, &tmp);
        retVal = (tmp == 0);
    }

    isRTMIEnabled = enable;

    return retVal;
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

static void deInit(void)
{
    enableDriver(DRIVER_DISABLE);
    HAL.IOs->config->setLow(Pins.DRV_EN);
};

static uint8_t reset()
{
    // Toggle the TMC6460 nSLEEP pin to reset it
    HAL.IOs->config->setLow(Pins.nSLEEP);
    wait(1);
    HAL.IOs->config->setHigh(Pins.nSLEEP);

    // Wait for the chip to finish startup
    wait(10);

    // Back up the CRC settings
    bool normalCRC = uartIsNormalCRCEnabled;
    bool rtmiCRC   = uartIsRTMICRCEnabled;

    // IC Startup happens with CRC disabled
    uartIsNormalCRCEnabled = false;
    uartIsRTMICRCEnabled   = false;

    // Ensure communication buffers are empty
    TMC6460_UARTChannel->rxtx.clearBuffers();

    // Grab the UART->CONTROL register and store it
    tmc6460_readRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, &uartControlRegister);

    // Configure the mantissa limit and autobaud settings
    uartControlRegister = tmc6460_updateField(
            uartControlRegister,
            TMC6460_UART_CONTROL_MANTISSA_LIMIT_FIELD,
            uartBaudrateSetting
    );
    uartControlRegister = tmc6460_updateField(
            uartControlRegister,
            TMC6460_UART_CONTROL_AUTOBAUD_EN_FIELD,
            uartUseAutobaud
    );

    // Update the UART baudrate settings
    tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_CONTROL, uartControlRegister);

    // Update the CRC settings to their old values
    updateCRCSettings(normalCRC, rtmiCRC);

    // Reset the error counter
    uartErrorCount = 0;

    return 1;
}

static uint8_t restore()
{
    return 1;
}

static void checkErrors(uint32_t tick)
{
    UNUSED(tick);
    Evalboards.ch1.errors = 0;
}

static void timer_overflow(timer_channel channel)
{
    UNUSED(channel);

    if (useRTMIRAMDebug)
        return;

    // RAMDebug
    debug_nextProcess();
}

void TMC6460_init(void)
{
    Pins.DRV_EN     = &HAL.IOs->pins->DIO0; //Pin8
    Pins.nSLEEP     = &HAL.IOs->pins->DIO8; //Pin19

    HAL.IOs->config->toOutput(Pins.DRV_EN);
    HAL.IOs->config->toOutput(Pins.nSLEEP);
    HAL.IOs->config->setHigh(Pins.DRV_EN);

    TMC6460_SPIChannel = &HAL.SPI->ch1;
    TMC6460_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
    spi_setMode(TMC6460_SPIChannel, 1);
    spi_setFrequency(TMC6460_SPIChannel, 7500000);//7.5MHz

    TMC6460_UARTChannel = HAL.UART;
    if (Evalboards.ch1.hwVersion.major == 0 && Evalboards.ch1.id == ID_TMC6460)
    {
        // Evalboard v0.X uses UART on DIO17/18
        TMC6460_UARTChannel->pinout = UART_PINS_1;
    }
    else
    {
        // Evalboard v1.0 switched UART to DIO10/11
        // The BoB v0.1 also uses UART DIO10/11
        TMC6460_UARTChannel->pinout = UART_PINS_2;
    }

    TMC6460_UARTChannel->mode = UART_MODE_DUAL_WIRE_PushPull;
    TMC6460_UARTChannel->rxtx.baudRate = 6000000; // 6Mbps
    TMC6460_UARTChannel->rxtx.init();
    // Calculate the TMC6460 Mantissa setting for the target baudrate
    uartBaudrateSetting = 60000000 / TMC6460_UARTChannel->rxtx.baudRate;

    TMC6460_config = Evalboards.ch1.config;

    // connect evalboard functions
    Evalboards.ch1.config->reset        = reset;
    Evalboards.ch1.config->restore      = restore;
    Evalboards.ch1.config->state        = CONFIG_READY;
    Evalboards.ch1.config->configIndex  = 0;
    Evalboards.ch1.rotate               = rotate;
    Evalboards.ch1.right                = right;
    Evalboards.ch1.left                 = left;
    Evalboards.ch1.stop                 = stop;
    Evalboards.ch1.getMeasuredSpeed     = getMeasuredSpeed;
    Evalboards.ch1.GAP                  = GAP;
    Evalboards.ch1.SAP                  = SAP;
    Evalboards.ch1.moveTo               = moveTo;
    Evalboards.ch1.moveBy               = moveBy;
    Evalboards.ch1.writeRegister        = writeRegister;
    Evalboards.ch1.readRegister         = readRegister;
    Evalboards.ch1.periodicJob          = periodicJob;
    Evalboards.ch1.userFunction         = userFunction;
    Evalboards.ch1.enableDriver         = enableDriver;
    Evalboards.ch1.checkErrors          = checkErrors;
    Evalboards.ch1.numberOfMotors       = TMC6460_MOTORS;
    Evalboards.ch1.deInit               = deInit;
    Evalboards.ch1.VMMin                = TMC6460_VM_MIN;
    Evalboards.ch1.VMMax                = TMC6460_VM_MAX;

    Evalboards.ch1.fwdTmclCommand       = fwdTmclCommand; // Used to intercept RAMDebug

    // Reset the TMC6460
    reset();

    // Enable the driver
    enableDriver(DRIVER_ENABLE);

    // Setup RAM debug measurement
    Timer.overflow_callback = timer_overflow;
    Timer.init();
    Timer.setFrequency(TIMER_CHANNEL_2, RAMDEBUG_FREQUENCY);
    debug_updateFrequency(RAMDEBUG_FREQUENCY);
};

/* RAMDebug <-> RTMI mapping **************************************************/
static uint16_t rtmi_channels[RTMI_MAX_CHANNELS] = { 0 };
#define RTMI_INVALID_CHANNEL_ADDRESS 0xFFFF // TMC6460 addresses are 12 bit, so this value can never be a valid channel

static RAMDebugState rtmiramdebug_state = RAMDEBUG_IDLE;

// Reuse RAMDebug buffer from RAMDebug.c
extern uint32_t debug_buffer[RAMDEBUG_BUFFER_ELEMENTS];
static uint32_t rtmiramdebug_write_index = 0;
static uint32_t rtmiramdebug_start_index = 0;
static uint32_t rtmiramdebug_sampleCount = RAMDEBUG_BUFFER_ELEMENTS;
static uint32_t rtmiramdebug_preTriggerSampleCount = 0;

typedef struct {
    uint8_t          channel; // Index of rtmi_channels to trigger on, or 0xFF if disabled.
    uint8_t          registerIndex; // RTMI channel the trigger channel runs on.
    RAMDebugTrigger  type;
    uint32_t         threshold;
    uint32_t         mask;
    uint8_t          shift;

    uint32_t         signedMsbMask; // This value is only set for signed triggers. Its used for sign-extension and correct triggering there.
} Trigger;
static Trigger trigger;

bool rtmiramdebug_wasAbove;

static bool checkTriggerThreshold(uint32_t value)
{
    // Apply mask/shift values
    value = (value & trigger.mask) >> trigger.shift;

    // Sign-extend for signed values
    // Note: For unconditional and unsigned triggers, signedMsbMask is 0
    value ^= trigger.signedMsbMask;
    value -= trigger.signedMsbMask;

    // For unsigned triggers, we just compare (signedMsbMask is zero there)
    // For signed triggers, this subtraction moves the values that are compared around,
    // so that an unsigned comparison yields the same result as a signed comparison would.
    return (value - trigger.signedMsbMask) > (trigger.threshold - trigger.signedMsbMask);
}

bool hasTriggered(uint8_t register_index, uint32_t value)
{
    // Abort if not in the right state
    if (rtmiramdebug_state != RAMDEBUG_TRIGGER)
        return false;

    // Unconditional capture -> Immediately 'trigger'
    if (trigger.type == TRIGGER_UNCONDITIONAL)
        return true;

    // Wrong channel -> Don't trigger
    if (register_index != trigger.registerIndex)
        return false;

    // Correct channel -> Check threshold against the edge type
    bool isAbove = checkTriggerThreshold(value);
    switch(trigger.type)
    {
    case TRIGGER_RISING_EDGE_SIGNED:
    case TRIGGER_RISING_EDGE_UNSIGNED:
        if (!rtmiramdebug_wasAbove && isAbove)
            return true;
        break;

    case TRIGGER_FALLING_EDGE_SIGNED:
    case TRIGGER_FALLING_EDGE_UNSIGNED:
        if (rtmiramdebug_wasAbove && !isAbove)
            return true;
        break;

    case TRIGGER_DUAL_EDGE_SIGNED:
    case TRIGGER_DUAL_EDGE_UNSIGNED:
        if (rtmiramdebug_wasAbove != isAbove)
            return true;
        break;

    default:
        break;
    }

    // Update the last threshold state
    rtmiramdebug_wasAbove = isAbove;

    // We didn't trigger
    return false;
}

// => TMC-API wrapper
bool tmc6460_RTMIDataCallback(uint16_t icID, uint8_t status, uint32_t data)
{
    UNUSED(icID);
    // No measurement active -> Throw the data away
    if (rtmiramdebug_state == RAMDEBUG_IDLE)
        return false;
    if (rtmiramdebug_state == RAMDEBUG_COMPLETE)
        return false;

    // Unpack the RTMI status
    //uint8_t write_counter  = (status >> 4) & 7;
    uint8_t register_index = (status >> 1) & 7;

    if (rtmiramdebug_state == RAMDEBUG_PRETRIGGER)
    {
        // Check if the requested pretrigger data has been captured
        // Note that this does not cover the case when the amount of pretrigger
        // samples equals the amount of possible samples. In that case the write
        // index wraps back to zero and does not allow this check to succeed.
        // For that case the moving write pointer logic takes care of updating
        // the state.
        if (rtmiramdebug_write_index >= rtmiramdebug_preTriggerSampleCount)
        {
            rtmiramdebug_state = RAMDEBUG_TRIGGER;
        }
    }

    if (hasTriggered(register_index, data))
    {
        // Triggered -> Advance the state and store the trigger point
        rtmiramdebug_state = RAMDEBUG_CAPTURE;
        // Since the trigger might not be the first channel, we must add the amount of valid samples that are part
        // of this RTMI group
        uint8_t priorSamples = (trigger.channel != 0xFF)? trigger.channel : 0;
        rtmiramdebug_start_index = (rtmiramdebug_write_index - priorSamples - rtmiramdebug_preTriggerSampleCount + RAMDEBUG_BUFFER_ELEMENTS) % RAMDEBUG_BUFFER_ELEMENTS;
    }

    // Store the data
    debug_buffer[rtmiramdebug_write_index] = data;

    // Move the write index
    if (++rtmiramdebug_write_index == RAMDEBUG_BUFFER_ELEMENTS)
    {
        rtmiramdebug_write_index = 0;

        // If we filled the entire buffer, the pretrigger phase is finished
        if (rtmiramdebug_state == RAMDEBUG_PRETRIGGER)
        {
            rtmiramdebug_state = RAMDEBUG_TRIGGER;
        }
    }

    if (rtmiramdebug_state == RAMDEBUG_CAPTURE)
    {
        uint32_t samplesWritten = (rtmiramdebug_write_index - rtmiramdebug_start_index + RAMDEBUG_BUFFER_ELEMENTS) % RAMDEBUG_BUFFER_ELEMENTS;
        if (samplesWritten == 0 || samplesWritten >= rtmiramdebug_sampleCount)
        {
            // RAMDebug capture complete
            rtmiramdebug_state = RAMDEBUG_COMPLETE;
            // End the capture - this must be after updating rtmiramdebug_state
            if (!enableRTMI(false))
            {
                // RTMI shutdown reported an error -> Measurement failed
                rtmiramdebug_state = RAMDEBUG_ERROR;
            }
        }
    }

    return false;
}

bool tmc6460_isRTMIEnabled(uint16_t icID)
{
    UNUSED(icID);
    return isRTMIEnabled;
}
// <= TMC-API wrapper

static void rtmiramdebug_process()
{
    if (!tmc6460_processRTMI(DEFAULT_MOTOR, 0))
    {
        // RTMI stream error - abort the measurement
        enableRTMI(false);
        rtmiramdebug_state = RAMDEBUG_ERROR;
    }
}

static void rtmiramdebug_init()
{
    // Ensure RTMI is off // ToDo: Review mid-capture inits/resets
    enableRTMI(false);

    // Reset the state
    rtmiramdebug_state = RAMDEBUG_IDLE;

    // Set default values for the capture configuration
    rtmiramdebug_sampleCount = RAMDEBUG_BUFFER_ELEMENTS;
    rtmiramdebug_divisor = 0;

    // Clear channel settings
    for (uint32_t i = 0; i < ARRAY_SIZE(rtmi_channels); i++)
    {
        rtmi_channels[i] = RTMI_INVALID_CHANNEL_ADDRESS;
    }

    // Clear trigger settings
    trigger.channel = 0xFF;
    trigger.mask    = 0xFFFFFFFF;
    trigger.shift   = 0;

    // Clear the capture buffer
    rtmiramdebug_write_index     = 0;
    rtmiramdebug_start_index     = 0;
}

static bool rtmiramdebug_setChannel(uint8_t type, uint32_t channel_value)
{
    if (type != CAPTURE_REGISTER)
        return false;

    if (rtmiramdebug_state != RAMDEBUG_IDLE)
        return false;

    // Only allow 12-bit register addresses
    if (channel_value > 0x3FF)
        return false;

    for (int i = 0; i < RTMI_MAX_CHANNELS; i++)
    {
        if (rtmi_channels[i] != RTMI_INVALID_CHANNEL_ADDRESS)
            continue;

        // Add the channel
        rtmi_channels[i] = channel_value;
        return true;
    }

    // Failed to find a free channel
    return false;
}

static bool rtmiramdebug_setTriggerChannel(uint8_t type, uint32_t channel_value)
{
    if (type != CAPTURE_REGISTER)
        return false;

    if (rtmiramdebug_state != RAMDEBUG_IDLE)
        return false;

    // Only allow 12-bit register addresses
    if (channel_value > 0x3FF)
        return false;

    for (int i = 0; i < RTMI_MAX_CHANNELS; i++)
    {
        if (rtmi_channels[i] != channel_value)
            continue;

        // Found the trigger channel in the capture list
        trigger.channel = i;
        return true;
    }

    return false;
}

static int32_t rtmiramdebug_getChannelAddress(uint8_t index, uint32_t *address)
{
    if (index >= RTMI_MAX_CHANNELS)
    {
        if (index != 0xFF)
            return false;

        // Trigger channel
        if (trigger.channel == 0xFF)
        {
            // No trigger
            *address = 0;
        }
        else
        {
            *address = rtmi_channels[trigger.channel];
        }
    }
    else
    {
        // Normal channel
        *address = rtmi_channels[index];
    }

    // RAMDebug returns 0 for not-yet-configured channels
    if (*address == RTMI_INVALID_CHANNEL_ADDRESS)
    {
        *address = 0;
    }

    return true;
}


static void rtmiramdebug_setTriggerMaskShift(uint32_t mask, uint8_t shift)
{
    trigger.mask  = mask;
    trigger.shift = shift;
}

static int32_t rtmiramdebug_enableTrigger(uint8_t type, uint32_t threshold)
{
    // Parameter validation
    if (type >= TRIGGER_END)
        return false;

    if (rtmiramdebug_state != RAMDEBUG_IDLE)
        return false;

    // Do not allow the edge triggers with channel still missing
    if (type != TRIGGER_UNCONDITIONAL && trigger.channel == 0xFF)
        return false;

    // Store the trigger information
    trigger.type = type;
    trigger.threshold = threshold;

    if (trigger.type > TRIGGER_UNCONDITIONAL && trigger.type <= TRIGGER_DUAL_EDGE_SIGNED)
    {
        // Pre-calculate the signed MSB mask
        uint32_t baseMask = trigger.mask >> trigger.shift;
        trigger.signedMsbMask = baseMask & (~(baseMask >> 1));
    }
    else
    {
        trigger.signedMsbMask = 0;
    }

    // Enable the trigger
    rtmiramdebug_state = RAMDEBUG_PRETRIGGER;

    // Upload the config to RTMI
    int32_t channelIndex = 7;
    for (int i = RTMI_MAX_CHANNELS-1; i >= 0; i--)
    {
        if (rtmi_channels[i] == RTMI_INVALID_CHANNEL_ADDRESS)
            continue;

        uint32_t configValue = rtmi_channels[i] | TMC6460_UART_RTMI_CH_0_CH_0_EN_MASK;

        if (i == trigger.channel)
        {
            // Cache which RTMI channel the trigger value runs on
            trigger.registerIndex = channelIndex;
        }

        // Fill RTMI channels backwards
        tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_RTMI_CH_0 + channelIndex, configValue);
        channelIndex--;
    }

    // Turn off any other channels
    while (channelIndex >= 0)
    {
        tmc6460_writeRegister(DEFAULT_MOTOR, TMC6460_UART_RTMI_CH_0 + channelIndex, 0);
        channelIndex--;
    }

    // If we have a trigger, preload whether we're above or below the threshold
    if (type != TRIGGER_UNCONDITIONAL)
    {
        uint32_t triggerChannelValue = 0;
        tmc6460_readRegister(DEFAULT_MOTOR, rtmi_channels[trigger.channel], &triggerChannelValue);
        rtmiramdebug_wasAbove = checkTriggerThreshold(triggerChannelValue);
    }

    // Enable RTMI
    enableRTMI(true);

    return true;
}


static void rtmiramdebug_setPrescaler(uint32_t divider)
{
    if (divider <= 255)
    {
        rtmiramdebug_divisor = divider;
    }
}

static void rtmiramdebug_setSampleCount(uint32_t count)
{
    if (count > RAMDEBUG_BUFFER_ELEMENTS)
    {
        count = RAMDEBUG_BUFFER_ELEMENTS;
    }

    rtmiramdebug_sampleCount = count;
}

//static uint32_t rtmiramdebug_getSampleCount()
//{
//    return rtmiramdebug_sampleCount;
//}
//
static void rtmiramdebug_setPretriggerSampleCount(uint32_t count)
{
    if (rtmiramdebug_state != RAMDEBUG_IDLE)
        return;

    rtmiramdebug_preTriggerSampleCount = count;
}

static uint32_t rtmiramdebug_getPretriggerSampleCount()
{
    return rtmiramdebug_preTriggerSampleCount;
}


static int32_t rtmiramdebug_getSample(uint32_t index, uint32_t *value)
{
    if (rtmiramdebug_state != RAMDEBUG_COMPLETE)
    {
        if (rtmiramdebug_state != RAMDEBUG_CAPTURE)
            return false;

        // If we are in CAPTURE state and the user requested data
        // thats already captured, allow the access
        // ToDo: Add early download support
        //if (index > ((debug_write_index - debug_start_index) % RAMDEBUG_BUFFER_ELEMENTS))
            return false;
    }

    if (index >= rtmiramdebug_sampleCount)
        return false;

    uint32_t readIndex = (index + rtmiramdebug_start_index) % RAMDEBUG_BUFFER_ELEMENTS;
    *value = debug_buffer[readIndex];

    return true;
}

static bool rtmiramdebug_bulkDownload(uint32_t index, uint32_t *samplesToSend)
{
    uint32_t extraDataLimit = tmcl_getExtraDataLimit();

    // If there isn't at least space for two samples,
    // report that bulk download isn't supported.
    // Since the bulk download mechanism uses a normal command
    // plus the bulk data, just a single bulk data sample
    // would only make things slower.
    if (extraDataLimit < (2*sizeof(uint32_t)))
        return false;

    uint32_t indexInBuffer = (index + rtmiramdebug_start_index) % RAMDEBUG_BUFFER_ELEMENTS;
    uint32_t leftSamples = rtmiramdebug_sampleCount - index;

    *samplesToSend = MIN(leftSamples, extraDataLimit / sizeof(uint32_t));

    uint32_t extraBytes = *samplesToSend * sizeof(uint32_t);
    uint32_t bytesUntilWraparound = (RAMDEBUG_BUFFER_ELEMENTS - indexInBuffer) * 4;

    tmcl_appendData((uint8_t *) &debug_buffer[indexInBuffer], MIN(extraBytes, bytesUntilWraparound));

    if (extraBytes > bytesUntilWraparound)
    {
        // We only copied data until the end of the ring buffer.
        // Copy the rest in from the start of the ring buffer.
        tmcl_appendData((uint8_t *) &debug_buffer[0], extraBytes - bytesUntilWraparound);
    }

    return true;
}

static int32_t rtmiramdebug_getState(void)
{
    return rtmiramdebug_state;
}

static int32_t rtmiramdebug_getInfo(uint32_t type)
{
    switch(type)
    {
    case 0: // Maximum channels
        return RTMI_MAX_CHANNELS; // ToDo: Configure based on interface speed?
        break;
    case 1: // Maximum total samples
        return RAMDEBUG_BUFFER_ELEMENTS;
        break;
    case 2: // Sampling Frequency
        uint32_t mcc_config_pwm_period = 0;
        tmc6460_readField(DEFAULT_MOTOR, TMC6460_MCC_CONFIG_PWM_PERIOD_MAX_COUNT_FIELD, &mcc_config_pwm_period);

        // In case of communication failure we'll fall back to report the base frequency
        // This is technically not fully correct, but the easiest approach to take for typical usage
        if (mcc_config_pwm_period == 0)
            return 25000;

        return 120000000 / mcc_config_pwm_period ;
        break;
    case 3: // Amount of samples already captured
        return (rtmiramdebug_write_index - rtmiramdebug_start_index) % RAMDEBUG_BUFFER_ELEMENTS;
        break;
    case 4: // Communication speed upper limit (samples per second)
        // Tooling can use this value to ensure that we are not overwhelming the interface resulting in errors.
        // Calculate maximum samples per second based on UART bandwidth
        
        uint32_t uart_baudrate = TMC6460_UARTChannel->rxtx.baudRate;
        uint32_t bytes_per_second = uart_baudrate / 10; // 10 bits per byte (8 + start/stop)
        uint32_t bytes_per_sample = uartIsRTMICRCEnabled? 6:5;
        uint32_t max_samples_per_second = bytes_per_second / bytes_per_sample;

        return max_samples_per_second;

        break;
    default:
        break;
    }

    return -1;
}

// TMCL processing function
static int rtmiramdebug_processTMCL(uint8_t type, uint8_t motor, uint32_t *data)
{
    switch (type)
    {
    case 0:
        rtmiramdebug_init();
        break;
    case 1:
        rtmiramdebug_setSampleCount(*data);
        break;
    case 2:
        /* Placeholder: Set sampling time reference*/
        if (*data != 0)
            return REPLY_INVALID_VALUE;
        break;
    case 3:
        rtmiramdebug_setPrescaler(*data);
        break;
    case 4:
        if (!rtmiramdebug_setChannel(motor, *data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 5:
        if (!rtmiramdebug_setTriggerChannel(motor, *data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 6:
        rtmiramdebug_setTriggerMaskShift(*data, motor);
        break;
    case 7:
        rtmiramdebug_enableTrigger(motor, *data);
        break;
    case 8:
        *data = rtmiramdebug_getState();
        break;
    case 9:
        if (!rtmiramdebug_getSample(*data, data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 10:
        *data = rtmiramdebug_getInfo(*data);
        break;
    case 11:
//        if (!rtmiramdebug_getChannelType(motor, (uint8_t *) data))
//            return REPLY_MAX_EXCEEDED;
        break;
    case 12:
        if (!rtmiramdebug_getChannelAddress(motor, data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 13:
        rtmiramdebug_setPretriggerSampleCount(*data);
        break;
    case 14:
        *data = rtmiramdebug_getPretriggerSampleCount();
        break;
    case 22:
        if (!rtmiramdebug_bulkDownload(*data, data))
            return REPLY_CMD_NOT_AVAILABLE;
        break;
    default:
        return REPLY_INVALID_TYPE;
        break;
    }

    return REPLY_OK;
}

static bool fwdTmclCommand(TMCLCommandTypeDef *ActualCommand, TMCLReplyTypeDef *ActualReply)
{
    if (ActualCommand->ModuleId != 1)
        return false;

    if (!useRTMIRAMDebug)
        return false;

    if (ActualCommand->Opcode == TMCL_RamDebug)
    {
        // Send all RAMDebug commands to the RTMI RAMDebug translation layer
        ActualReply->Status = rtmiramdebug_processTMCL(ActualCommand->Type, ActualCommand->Motor, &ActualReply->Value.UInt32);

        // Report the command as already handled
        return true;
    }

    // Let normal TMCL handle this command
    return false;
}

// Helper function: Find the RTMI channel a register is configured on
// Returns the channel index if found, or RTMI_MAX_CHANNELS if not found.
static uint8_t getRegisterRTMIChannel(uint16_t address)
{
    int32_t channelIndex = 7;
    for (int i = RTMI_MAX_CHANNELS-1; i >= 0; i--)
    {
        if (rtmi_channels[i] == RTMI_INVALID_CHANNEL_ADDRESS)
            continue;

        if (rtmi_channels[i] == address)
            return channelIndex;

        channelIndex--;
    }

    return RTMI_MAX_CHANNELS;
}
