/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "tmc/BoardAssignment.h"
#include "tmc/ic/TMC4361A/TMC4361A.h"
#include "tmc/ic/TMC2660/TMC2660_Macros.h"
#include "tmc/ic/TMC2130/TMC2130.h"
#include "tmc/ic/TMC2160/TMC2160.h"

#define DEFAULT_ICID 0

// Typedefs
typedef struct
{
    ConfigurationTypeDef *config;
    int32_t velocity;
    int32_t oldX;
    uint32_t oldTick;
    uint8_t status;
} TMC4361ATypeDef;
static TMC4361ATypeDef TMC4361A;

typedef struct
{
    IOPinTypeDef *TARGET_REACHED;
    IOPinTypeDef *NRST;
    IOPinTypeDef *FREEZE;
    IOPinTypeDef *START;
    IOPinTypeDef *HOME_REF;
    IOPinTypeDef *STOP_R;
    IOPinTypeDef *STOP_L;
    IOPinTypeDef *INTR;
    IOPinTypeDef *STANDBY_CLK;
} PinsTypeDef;
static PinsTypeDef Pins;

typedef void (*tmc4361A_callback)(TMC4361ATypeDef *, ConfigState);

static SPIChannelTypeDef *TMC4361A_SPIChannel;
static uint32_t vmax_position = 0;

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
static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);
static uint8_t reset();
static uint8_t restore();

void tmc4361A_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC4361A_SPIChannel->readWriteArray(data, dataLength);
}

void tmc4361A_setStatus(uint16_t icID, uint8_t *data)
{
    UNUSED(icID);
    TMC4361A.status = data[0];
}

static void tmc4361A_writeConfiguration()
{
    uint8_t *ptr = &TMC4361A.config->configIndex;
    const int32_t *settings;

    if (TMC4361A.config->state == CONFIG_RESTORE)
    {
        settings = *(tmc4361A_shadowRegister + 0);
        // Find the next restorable register
        while (*ptr < TMC4361A_REGISTER_COUNT)
        {
            // If the register is writable and has been written to, restore it
            if (TMC_IS_WRITABLE(tmc4361A_registerAccess[*ptr]) && tmc4361A_getDirtyBit(DEFAULT_ICID, *ptr))
            {
                break;
            }
            (*ptr)++;
        }
    }
    else
    {
        settings = tmc4361A_sampleRegisterPreset;
        // Find the next resettable register
        while ((*ptr < TMC4361A_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc4361A_registerAccess[*ptr])) (*ptr)++;
    }

    if (*ptr < TMC4361A_REGISTER_COUNT)
    {
        tmc4361A_writeRegister(DEFAULT_ICID, *ptr, settings[*ptr]);
        (*ptr)++;
    }
    else
    {
        if (TMC4361A.config->callback)
        {
            ((tmc4361A_callback) TMC4361A.config->callback)(&TMC4361A, TMC4361A.config->state);
        }

        TMC4361A.config->state = CONFIG_READY;
    }
}

int32_t tmc4361A_discardVelocityDecimals(int32_t value)
{
    if (abs(value) > 8000000)
    {
        value = (value < 0) ? -8000000 : 8000000;
    }
    return value << 8;
}

static uint8_t tmc4361A_moveToNextFullstep()
{
    int32_t stepCount;

    // Motor must be stopped
    if (tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_VACTUAL) != 0)
    {
        // Not stopped
        return 0;
    }

    // Position mode, hold mode, low velocity
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_RAMPMODE, 4);
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VMAX, 10000 << 8);

    // Current step count
    stepCount = tmc4361A_fieldRead(DEFAULT_ICID, TMC4361A_MSCNT_FIELD);
    // Get microstep value of step count (lowest 8 bits)
    stepCount = stepCount % 256;
    // Assume: 256 microsteps -> Fullsteps are at 128 + n*256
    stepCount = 128 - stepCount;

    if (stepCount == 0)
    {
        // Fullstep reached
        return 1;
    }

    // Fullstep not reached -> calculate next fullstep position
    stepCount += tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_XACTUAL);
    // Move to next fullstep position
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_XTARGET, stepCount);

    return 0;
}

uint8_t tmc4361A_calibrateClosedLoop(uint8_t worker0master1)
{
    static uint8_t state = 0;
    static uint32_t oldRamp;

    uint32_t amax = 0;
    uint32_t dmax = 0;

    if (worker0master1 && state == 0)
        state = 1;

    switch (state)
    {
    case 1:
        amax = tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_AMAX);
        dmax = tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_DMAX);

        // Set ramp and motion parameters
        oldRamp = tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_RAMPMODE);
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_RAMPMODE, TMC4361A_RAMP_POSITION | TMC4361A_RAMP_HOLD);
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_AMAX, MAX(amax, 1000));
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_DMAX, MAX(dmax, 1000));
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VMAX, 0);

        state = 2;
        break;
    case 2:
        // Clear encoder calibration bit
        tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_CL_CALIBRATION_EN_FIELD, 0);

        // Disable internal data regulation for closed loop operation in encoder config
        tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_REGULATION_MODUS_FIELD, 1);

        if (tmc4361A_moveToNextFullstep()) // move to next fullstep, motor must be stopped, poll until finished
            state = 3;
        break;
    case 3:
        // Start encoder calibration
        tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_CL_CALIBRATION_EN_FIELD, 1);

        state = 4;
        break;
    case 4:
        if (worker0master1)
            break;

        // Stop encoder calibration
        tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_CL_CALIBRATION_EN_FIELD, 0);
        // Enable closed loop in encoder config
        tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_REGULATION_MODUS_FIELD, 1);
        // Restore old ramp mode, enable position mode
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_RAMPMODE, TMC4361A_RAMP_POSITION | oldRamp);

        state = 5;
        break;
    case 5:
        state = 0;
        return 1;
        break;
    default:
        break;
    }
    return 0;
}

// Route the generic cover function to the TMC4361A function
// This also provides the TMC4361ATypeDef, which the generic
// cover function doesn't know.
static void tmc4361A_fullCover(uint8_t *data, size_t length)
{
    // Buffering old values to not interrupt manual covering
    uint32_t old_high = 0;
    uint32_t old_low  = 0;
    tmc4361A_cache(DEFAULT_ICID, TMC4361A_CACHE_READ, TMC4361A_COVER_HIGH, &old_high);
    tmc4361A_cache(DEFAULT_ICID, TMC4361A_CACHE_READ, TMC4361A_COVER_LOW, &old_low);

    // Check if datagram length is valid
    if (length == 0 || length > 8)
        return;

    uint8_t bytes[8] = {0};
    uint32_t tmp;
    size_t i;
    uint32_t dataMSB = 0;
    uint32_t dataLSB = 0;

    // Copy data into buffer of maximum cover datagram length (8 bytes)
    for (i = 0; i < length; i++) bytes[i] = data[length - i - 1];

    // Send the datagram
    if (length > 4)
    {
        dataMSB = (bytes[7] << 24) | (bytes[6] << 16) | (bytes[5] << 8) | bytes[4];
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_COVER_HIGH, dataMSB);
    }

    dataLSB = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_COVER_LOW, dataLSB);

    // Wait for datagram completion
    // TODO CHECK 3: Get the waiting for cover completion done properly (LH)
    for (i = 0; i < 100; i++) tmp = ACCESS_ONCE(i);

    // Read the reply
    if (length > 4)
    {
        tmp      = tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_COVER_DRV_HIGH);
        bytes[4] = 0XFF & (tmp >> 0);
        bytes[5] = 0XFF & (tmp >> 8);
        bytes[6] = 0XFF & (tmp >> 16);
        bytes[7] = 0XFF & (tmp >> 24);
    }

    tmp      = tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_COVER_DRV_LOW);
    bytes[0] = 0XFF & (tmp >> 0);
    bytes[1] = 0XFF & (tmp >> 8);
    bytes[2] = 0XFF & (tmp >> 16);
    bytes[3] = 0XFF & (tmp >> 24);

    // Write the reply to the data array
    for (i = 0; i < length; i++) { data[length - i - 1] = bytes[i]; }

    // Rewriting old values to prevent interrupting manual covering. Imitating unchanged values and state.
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_COVER_HIGH, old_high);
    tmc4361A_cache(DEFAULT_ICID, TMC4361A_CACHE_WRITE, TMC4361A_COVER_LOW, &old_low);
}

// The cover function emulates the SPI readWrite function
static uint8_t tmc4361A_cover(uint8_t data, uint8_t lastTransfer)
{
    static uint64_t coverIn    = 0; // read from squirrel
    static uint64_t coverOut   = 0; // write to squirrel
    static uint8_t coverLength = 0; // data to be written

    uint8_t out = 0; // return value of this function

    // buffer outgoing data
    coverOut <<= 8;   // shift left by one byte to make room for the next byte
    coverOut |= data; // add new byte to be written
    coverLength++;    // count outgoing bytes

    // return read and buffered byte to be returned
    out = coverIn >> 56; // output last received byte
    coverIn <<= 8;       // shift by one byte to read this next time

    if (lastTransfer)
    {
        /* Write data to cover register(s). The lower 4 bytes go into the cover low register,
		 * the higher 4 bytes, if present, go into the cover high register.
		 * The datagram needs to be sent twice, otherwise the read buffer will be delayed by
		 * one read/write datagram.
		 */

        // Send the buffered datagram & wait a bit before continuing so the 4361 can complete the datagram to the driver
        // measured delay between COVER_LOW transmission and COVER_DONE flag: ~90µs -> 1 ms more than enough
        // todo CHECK 3: Delay measurement only done on TMC4361, not 4361A - make sure the required delay didnt change (LH) #1
        if (coverLength > 4)
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_COVER_HIGH, coverOut >> 32);

        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_COVER_LOW, coverOut & 0xFFFFFFFF);

        wait(1);

        // Trigger a re-send by writing the low register again
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_COVER_LOW, coverOut & 0xFFFFFFFF);

        // Read the reply
        coverIn = 0;
        if (coverLength > 4)
            coverIn |= (uint64_t) tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_COVER_DRV_HIGH) << 32;

        coverIn |= tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_COVER_DRV_LOW);
        coverIn <<=
            (8 - coverLength) * 8; // Shift the highest byte of the reply to the highest byte of the buffer uint64_t

        // Clear write buffer
        coverOut    = 0;
        coverLength = 0;
    }

    return out; // return buffered read byte
}

// => Functions forwarded to API
static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    UNUSED(motor);

    // Disable Position Mode
    tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_OPERATION_MODE_FIELD, 0);

    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VMAX, tmc4361A_discardVelocityDecimals(velocity));

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
    UNUSED(motor);

    // Enable Position Mode
    tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_OPERATION_MODE_FIELD, 1);
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VMAX, tmc4361A_discardVelocityDecimals(vmax_position));
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_XTARGET, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    // determine actual position and add numbers of ticks to move
    *ticks += tmc4361A_readRegister(DEFAULT_ICID, TMC4361A_XACTUAL);

    return moveTo(motor, *ticks);
}
// <= Functions forwarded to API

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t errors = TMC_ERROR_NONE;
    int32_t uvalue;
    int32_t buffer;

    if (motor >= TMC4361A_MOTORS)
        return TMC_ERROR_MOTOR;

    switch (type)
    {
    case 0:
        // Target position
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_XTARGET, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_XTARGET, *value);
        }
        break;
    case 1:
        // Actual position
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_XACTUAL, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_XACTUAL, *value);
        }
        break;
    case 2:
        // Target speed
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VMAX, &buffer);
            *value = buffer >> 8;
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VMAX, tmc4361A_discardVelocityDecimals(*value));
        }
        break;
    case 3:
        // Actual speed
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VACTUAL, value);
        }
        else if (readWrite == WRITE)
        {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 4:
        // Maximum speed
        if (readWrite == READ)
        {
            *value = vmax_position;
            //*value = readRegister(DEFAULT_ICID, TMC4361A_VMAX, &buffer);
            //*value = buffer >> 8;
        }
        else if (readWrite == WRITE)
        {
            vmax_position = *value;
            // Write VMAX if already in position mode
            readRegister(DEFAULT_ICID, TMC4361A_RAMPMODE, &buffer);
            if (buffer & TMC4361A_RAMP_POSITION)
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VMAX, tmc4361A_discardVelocityDecimals(vmax_position));
        }
        break;
    case 5:
        // Maximum acceleration
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_AMAX, &buffer);
            *value = buffer >> 2;
        }
        else if (readWrite == WRITE)
        {
            if (*value & ~0x3FFFFF)
            {
                errors |= TMC_ERROR_VALUE;
            }
            else
            {
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_AMAX, *value << 2);
            }
        }
        break;
    case 8:
        // Position reached flag
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_STATUS, &buffer);
            *value = (buffer & (1 << 0)) ? 1 : 0;
        }
        else if (readWrite == WRITE)
            errors |= TMC_ERROR_TYPE;
        break;
    case 14:
        // Ramp type
        if (readWrite == READ)
        {
            uint32_t rampmode = tmc4361A_fieldRead(DEFAULT_ICID, TMC4361A_RAMP_PROFILE_FIELD);
            *value            = (rampmode == TMC4361A_RAMP_SSHAPE);
        }
        else if (readWrite == WRITE)
        {
            uint32_t rampmode = (*value) ? TMC4361A_RAMP_SSHAPE : TMC4361A_RAMP_TRAPEZ;
            tmc4361A_fieldWrite(DEFAULT_ICID, TMC4361A_RAMP_PROFILE_FIELD, rampmode);
        }
        break;
    case 15:
        // Velocity VSTART
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VSTART, &buffer);
            *value = buffer * 256;
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VSTART, (*value) / 256);
        }
        break;
    case 16:
        // Acceleration AStart
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ASTART, &buffer);
            *value = buffer >> 2;
        }
        else if (readWrite == WRITE)
        {
            if (*value & ~0x3FFFFF)
            {
                errors |= TMC_ERROR_VALUE;
            }
            else
            {
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ASTART, *value << 2);
            }
        }
        break;
    case 17:
        // Maximum Deceleration
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_DMAX, &buffer);
            *value = buffer >> 2;
        }
        else if (readWrite == WRITE)
        {
            if (*value & ~0x3FFFFF)
            {
                errors |= TMC_ERROR_VALUE;
            }
            else
            {
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_DMAX, *value << 2);
            }
        }
        break;
    case 18:
        // Velocity VBreak
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VBREAK, &buffer);
            *value = buffer * 256;
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VBREAK, (*value) / 256);
        }
        break;
    case 19:
        // Deceleration DFinal
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_DFINAL, &buffer);
            *value = buffer >> 2;
        }
        else if (readWrite == WRITE)
        {
            if (*value & ~0x3FFFFF)
            {
                errors |= TMC_ERROR_VALUE;
            }
            else
            {
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_DFINAL, *value << 2);
            }
        }
        break;
    case 20:
        // Velocity VSTOP
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VSTOP, &buffer);
            *value = buffer * 256;
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VSTOP, (*value) / 256);
        }
        break;
    case 21:
        // Deceleration DStop
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_DSTOP, &buffer);
            *value = buffer * 4;
        }
        else if (readWrite == WRITE)
        {
            if (*value & ~0x3FFFFF)
            {
                errors |= TMC_ERROR_VALUE;
            }
            else
            {
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_DSTOP, (*value) / 4);
            }
        }
        break;
    case 22:
        // Bow 1
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_BOW1, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_BOW1, *value);
        }
        break;
    case 23:
        // Bow 2
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_BOW2, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_BOW2, *value);
        }
        break;
    case 24:
        // Bow 3
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_BOW3, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_BOW3, *value);
        }
        break;
    case 25:
        // Bow 4
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_BOW4, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_BOW4, *value);
        }
        break;
    case 26:
        // Virtual stop left
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VIRT_STOP_LEFT, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VIRT_STOP_LEFT, *value);
        }
        break;
    case 27:
        // Virtual stop right
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_VIRT_STOP_RIGHT, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_VIRT_STOP_RIGHT, *value);
        }
        break;
    case 108:
        // CL Gamma VMin
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_VMIN_EMF, value); // read from shadow register
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_VMIN_EMF, *value);
        }
        break;
    case 109:
        // CL Gamma VMax
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_VADD_EMF, value); // read from shadow register
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_VADD_EMF, *value);
        }
        break;
    case 110:
        // CL maximum Gamma
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_ANGLES, &buffer);
            *value = buffer >> 16;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_ANGLES, &buffer);
            uvalue = buffer & 0x000001FF;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_ANGLES, uvalue | (*value << 16));
        }
        break;
    case 111:
        readRegister(DEFAULT_ICID, TMC4361A_CL_ANGLES, &buffer);
        // CL beta
        if (readWrite == READ)
        {
            *value = buffer & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            uvalue = buffer & 0x00FF0000;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_ANGLES, uvalue | (*value & 0x1FF));
        }
        break;
    case 112:
        // CL offset
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_OFFSET, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_OFFSET, *value);
        }
        break;
    case 113:
        // CL current minimum
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, &buffer);
            *value = (buffer >> 0) & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_SPI_OUT_CONF, &buffer);
            uint32_t spiOutFormat = buffer & TMC4361A_SPI_OUTPUT_FORMAT_MASK;
            int32_t minLimit, maxLimit;
            switch (spiOutFormat)
            { //S/D Mode:
            case 0x0B:
            case 0x0C:
            case 0x0F:
                minLimit = 0;
                maxLimit = 31;
                break;
            //SPI Mode:
            case 0x08:
            case 0x09:
            case 0x0A:
            case 0x0D:
                minLimit = 0;
                maxLimit = 255;
                break;
            default:
                minLimit = 0;
                maxLimit = 0;
                break;
            }

            if (*value < minLimit)
            {
                *value = minLimit;
            }
            else if (*value > maxLimit)
            {
                *value = maxLimit;
            }
            readRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, &buffer);
            uvalue = buffer & ~(0xFF << 0);
            uvalue |= (*value & 0xFF) << 0;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, uvalue);
        }
        break;
    case 114:
        // CL current maximum
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, &buffer);
            *value = (buffer >> 8) & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_SPI_OUT_CONF, &buffer);
            uint32_t spiOutFormat = buffer & TMC4361A_SPI_OUTPUT_FORMAT_MASK;
            int32_t minLimit, maxLimit;
            switch (spiOutFormat)
            { //S/D Mode:
            case 0x0B:
            case 0x0C:
            case 0x0F:
                minLimit = 0;
                maxLimit = 31;
                break;
            //SPI Mode:
            case 0x08:
            case 0x09:
            case 0x0A:
            case 0x0D:
                minLimit = 0;
                maxLimit = 255;
                break;
            default:
                minLimit = 0;
                maxLimit = 0;
                break;
            }

            if (*value < minLimit)
            {
                *value = minLimit;
            }
            else if (*value > maxLimit)
            {
                *value = maxLimit;
            }
            readRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, &buffer);
            uvalue = buffer & ~(0xFF << 8);
            uvalue |= (*value & 0xFF) << 8;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, uvalue);
        }
        break;
    case 115:
        // CL correction velocity P
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_VMAX_CALC_P, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_VMAX_CALC_P, *value);
        }
        break;
    case 116:
        // CL correction velocity I
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_VMAX_CALC_I, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_VMAX_CALC_I, *value);
        }
        break;
    case 117:
        // CL correction velocity I clipping
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_PID_I_CLIP, &buffer);
            *value = buffer & 0x7FFF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_PID_I_CLIP, &uvalue);
            uvalue &= ~(0x7FFF << 0);
            uvalue |= (*value & 0x7FFF) << 0;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_PID_I_CLIP, uvalue);
        }
        break;
    case 118:
        // CL correction velocity DV clock
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_PID_D_CLKDIV, &buffer);
            *value = buffer >> 16;
            *value &= 0xFF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_PID_D_CLKDIV, &uvalue);
            uvalue &= ~(0xFF << 16);
            uvalue |= (*value & 0xFF) << 16;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_PID_D_CLKDIV, uvalue);
        }
        break;
    case 119:
        // CL correction velocity DV clipping
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_PID_DV_CLIP, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_PID_DV_CLIP, *value);
        }
        break;
    case 120:
        // CL upscale delay
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_UPSCALE_DELAY, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_UPSCALE_DELAY, *value);
        }
        break;
    case 121:
        // CL Downscale delay
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_DNSCALE_DELAY, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_DNSCALE_DELAY, *value);
        }
        break;
    case 123:
        // Actual Scalar Value
        if (readWrite == READ)
        {
            // Read-only
            readRegister(DEFAULT_ICID, TMC4361A_SCALE_PARAM, value);
        }
        else if (readWrite == WRITE)
        {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 124:
        // CL Correction Position P
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_DELTA_P, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_DELTA_P, *value);
        }
        break;
    case 125:
        // CL max. correction tolerance
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_TOLERANCE, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_TOLERANCE, *value);
        }
        break;
    case 126:
        // CL start up
        readRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, &buffer);
        if (readWrite == READ)
        {
            *value = (buffer >> 16) & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            uvalue = buffer & ~(0xFF << 16);
            uvalue |= (*value & 0xFF) << 16;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, uvalue);
        }
        break;
    case 129: // todo AP 2: merge AP 129 with AP 133? #1
        // Closed Loop Flag
        if (readWrite == READ)
        {
            // Read for closed loop flag is implemented as AP 133
        }
        else if (readWrite == WRITE)
        {
            //Closed loop on/off
            if (*value)
            {
                *value = tmc4361A_calibrateClosedLoop(1);
                if (!*value)
                    errors |= TMC_ERROR_NOT_DONE;
            }
            else
            {
                readRegister(DEFAULT_ICID, TMC4361A_ENC_IN_CONF, &buffer);
                buffer &= ~(1 << 22); // closed loop
                tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ENC_IN_CONF, buffer);
            }
        }
        break;
    case 132:
        // Measured Encoder Speed
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_V_ENC, value);
        }
        else if (readWrite == WRITE)
            errors |= TMC_ERROR_TYPE;
        break;
    case 133: // todo AP 2: merge AP 129 with AP 133? #2
        // Closed Loop Init Flag
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_IN_CONF, &buffer);
            *value = (((buffer >> 22) & 3) == 1) ? 1 : 0;
        }
        else if (readWrite == WRITE)
        {
            // Write for closed loop flag is implemented as AP 129
        }
        break;
    case 134:
        // Encoder deviation
        if (readWrite == READ)
        {
            // Read-only encoder deviation
            readRegister(DEFAULT_ICID, TMC4361A_ENC_POS_DEV, value);
        }
        else if (readWrite == WRITE)
        {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 135:
        // Closed loop target reached tolerance
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_CL_TR_TOLERANCE, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CL_TR_TOLERANCE, *value);
        }
        break;
    case 136:
        // Encoder Velocity Delay
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, &buffer);
            *value = (buffer >> 0) & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, &uvalue);
            uvalue &= ~(0xFF << 0);
            uvalue |= (*value & 0x0F) << 0;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, uvalue);
        }
        break;
    case 137:
        // Encoder Velocity Filter
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, &buffer);
            *value = (buffer >> 8) & 0xF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, &uvalue);
            ;
            uvalue &= ~(0xF << 8);
            uvalue |= (*value & 0x0F) << 8;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, uvalue);
        }
        break;
    case 138:
        // Filter Update Time
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, &buffer);
            *value = (buffer >> 16) & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, &uvalue);
            uvalue &= ~(0xFF << 16);
            uvalue |= (*value & 0x0FF) << 16;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ENC_VMEAN_WAIT, uvalue);
        }
        break;
    case 200:
        // Boost current
        readRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, &buffer);
        if (readWrite == READ)
        {
            *value = (buffer >> 0) & 0xFF;
        }
        else if (readWrite == WRITE)
        {
            uvalue = buffer & ~(0xFF << 0);
            uvalue |= (*value & 0xFF) << 0;
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, uvalue);
        }
        break;
    case 209:
        // Encoder position
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_POS, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ENC_POS, *value);
        }
        break;
    case 214:
        // Power Down Delay
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_STDBY_DELAY, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_STDBY_DELAY, (*value) * 160000);
        }
        break;
    case 225:
        // Maximum encoder deviation
        if (readWrite == READ)
        {
            readRegister(DEFAULT_ICID, TMC4361A_ENC_POS_DEV_TOL, value);
        }
        else if (readWrite == WRITE)
        {
            tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_ENC_POS_DEV_TOL, *value);
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

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    // Notify driver shadows about register changes made via cover
    static int32_t high;

    switch ((uint8_t) address)
    {

    case TMC4361A_COVER_HIGH:
        high = value;
        break;
    case TMC4361A_COVER_LOW:
        if (Evalboards.ch2.id == ID_TMC2660) // TMC2660 -> 20 bit registers, 8 bit address
            Evalboards.ch2.writeRegister(motor, TMC2660_ADDRESS(value), TMC2660_VALUE(value));
        else // All other drivers -> 32 bit registers, 8 bit address
            Evalboards.ch2.writeRegister(motor, TMC_ADDRESS(high), value);
        break;
    case TMC4361A_SCALE_VALUES:
        /* Only possible with IHOLD and only with TMC2130 and TMC2160, since write-only registers changed actively by
		 * the TMC43XX (not via cover datagrams) are impossible to track.
		 */
        uint32_t temp;
        temp = tmc4361A_fieldExtract(value, TMC4361A_HOLD_SCALE_VAL_FIELD);
        switch (Evalboards.ch2.id)
        {
        case ID_TMC2130:
            tmc2130_fieldWrite(DEFAULT_ICID, TMC2130_IHOLD_FIELD, temp);
            break;
        case ID_TMC2160:
            tmc2160_fieldWrite(DEFAULT_ICID, TMC2160_IHOLD_FIELD, temp);
            break;
        }
        break;
    }
    tmc4361A_writeRegister(DEFAULT_ICID, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc4361A_readRegister(DEFAULT_ICID, (uint8_t) address);
}

static void periodicJob(uint32_t tick)
{
    if (TMC4361A.config->state != CONFIG_READY)
    {
        tmc4361A_writeConfiguration();
        return;
    }

    if ((tick - TMC4361A.oldTick) != 0)
    {
        tmc4361A_calibrateClosedLoop(0);
        TMC4361A.oldTick = tick;
    }
}

static void checkErrors(uint32_t tick)
{
    UNUSED(tick);
    Evalboards.ch1.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    uint32_t errors = 0;

    switch (type)
    {
    case 0: // simulate left/right reference switches, set high to support external ref swiches
        /*
		 * The the TMC4361 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */
        if (!(*value & ~3))
        {
            if (*value & (1 << 0))
            {
                HAL.IOs->config->toInput(Pins.STOP_R); // pull up -> set it to floating causes high
            }
            else
            {
                HAL.IOs->config->toOutput(Pins.STOP_R);
                HAL.IOs->config->setLow(Pins.STOP_R);
            }

            if (*value & (1 << 1))
            {
                HAL.IOs->config->toInput(Pins.STOP_L); // pull up -> set it to floating causes high
            }
            else
            {
                HAL.IOs->config->toOutput(Pins.STOP_L);
                HAL.IOs->config->setLow(Pins.STOP_L);
            }
        }
        //else TMCL.reply->Status = REPLY_INVALID_VALUE;
        break;
    case 1: // simulate reference switche HOME_REF, set high to support external ref swiches
        /*
		 * The the TMC43x1 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */
        if (*value)
        {
            HAL.IOs->config->toInput(Pins.HOME_REF); // pull up -> set it to floating causes high
        }
        else
        {
            HAL.IOs->config->toOutput(Pins.HOME_REF);
            HAL.IOs->config->setLow(Pins.HOME_REF);
        }
        break;
    case 2: // simulate reference switche FREEZE, set high to support external ref swiches
        /*
		 * The the TMC43x1 ref switch input is pulled high by external resistor an can be pulled low either by
		 * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
		 */

        if (*value)
        {
            HAL.IOs->config->toInput(Pins.FREEZE); // pull up -> set it to floating causes high
        }
        else
        {
            HAL.IOs->config->toOutput(Pins.FREEZE);
            HAL.IOs->config->setLow(Pins.FREEZE);
        }
        break;
    case 3:
        *value = tmc4361A_calibrateClosedLoop(1);
        if (!*value)
            errors |= TMC_ERROR_NOT_DONE;
        break;
    case 255:
        Evalboards.ch2.config->reset();
        Evalboards.ch1.config->reset();
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static void deInit(void)
{
    HAL.IOs->config->setLow(Pins.NRST);

    HAL.IOs->config->reset(Pins.STOP_L);
    HAL.IOs->config->reset(Pins.STOP_R);
    HAL.IOs->config->reset(Pins.HOME_REF);
    HAL.IOs->config->reset(Pins.START);
    HAL.IOs->config->reset(Pins.FREEZE);
    HAL.IOs->config->reset(Pins.STANDBY_CLK);
    HAL.IOs->config->reset(Pins.INTR);
    HAL.IOs->config->reset(Pins.TARGET_REACHED);
    HAL.IOs->config->reset(Pins.NRST);

    HAL.SPI->ch2.reset();
}

static uint8_t reset()
{
    // Pulse the low-active hardware reset pin
    HAL.IOs->config->setLow(Pins.NRST);
    wait(1);
    HAL.IOs->config->setHigh(Pins.NRST);

    if (TMC4361A.config->state != CONFIG_READY)
        return false;

    int32_t i;

    // Reset the dirty bits and wipe shadow registers
    for (i = 0; i < TMC4361A_REGISTER_COUNT; i++)
    {
        tmc4361A_setDirtyBit(DEFAULT_ICID, i, false);
        tmc4361A_shadowRegister[DEFAULT_ICID][i] = 0;
    }

    TMC4361A.config->state       = CONFIG_RESET;
    TMC4361A.config->configIndex = 0;

    return true;
}

static uint8_t restore()
{
    // Pulse the low-active hardware reset pin
    HAL.IOs->config->setLow(Pins.NRST);
    wait(1);
    HAL.IOs->config->setHigh(Pins.NRST);

    if (TMC4361A.config->state != CONFIG_READY)
        return 0;

    TMC4361A.config->state       = CONFIG_RESTORE;
    TMC4361A.config->configIndex = 0;

    return 1;
}

static void configCallback(TMC4361ATypeDef *tmc4361A, ConfigState state)
{
    UNUSED(tmc4361A);
    uint8_t driver, dataLength;
    uint32_t value;

    // Setup SPI
    switch (Evalboards.ch2.id)
    {
    case ID_TMC2130:
    case ID_TMC2160:
        driver     = 0x0C;
        dataLength = 0;
        break;
    case ID_TMC2660:
        driver     = 0x0B;
        dataLength = 0;
        break;
    default:
        driver     = 0x0F;
        dataLength = 40;
        break;
    }
    value = 0x44400040 | (dataLength << 13) | (driver << 0);
    tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_SPI_OUT_CONF, value);

    if (Evalboards.ch2.id == ID_TMC2160)
    {
        Evalboards.ch1.writeRegister(DEFAULT_ICID, TMC4361A_STP_LENGTH_ADD, 0x60002);
    }

    // Reset/Restore driver
    if (state == CONFIG_RESET)
    {
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_CURRENT_CONF, 0x00000003);
        tmc4361A_writeRegister(DEFAULT_ICID, TMC4361A_SCALE_VALUES, 0x00000000);
        tmc4361A_initCache();
        Evalboards.ch2.config->reset();
    }
    else
        Evalboards.ch2.config->restore();
}

void TMC4361A_init(void)
{
    TMC4361A.velocity = 0;
    TMC4361A.oldTick  = 0;
    TMC4361A.oldX     = 0;
    TMC4361A.config   = Evalboards.ch1.config;

    TMC4361A.config->channel     = 0;
    TMC4361A.config->configIndex = 0;
    TMC4361A.config->state       = CONFIG_READY;
    TMC4361A.config->callback    = (tmc_callback_config) configCallback;

    Pins.STANDBY_CLK    = &HAL.IOs->pins->DIO4;
    Pins.INTR           = &HAL.IOs->pins->DIO5;
    Pins.STOP_L         = &HAL.IOs->pins->DIO12;
    Pins.STOP_R         = &HAL.IOs->pins->DIO13;
    Pins.HOME_REF       = &HAL.IOs->pins->DIO14;
    Pins.START          = &HAL.IOs->pins->DIO15;
    Pins.FREEZE         = &HAL.IOs->pins->DIO16;
    Pins.NRST           = &HAL.IOs->pins->DIO17;
    Pins.TARGET_REACHED = &HAL.IOs->pins->DIO18;

    HAL.IOs->config->toOutput(Pins.NRST);
    HAL.IOs->config->toOutput(Pins.STOP_L);
    HAL.IOs->config->toOutput(Pins.STOP_R);
    HAL.IOs->config->toOutput(Pins.HOME_REF);
    HAL.IOs->config->toOutput(Pins.START);
    HAL.IOs->config->toOutput(Pins.FREEZE);

    HAL.IOs->config->setHigh(Pins.NRST);

    HAL.IOs->config->setHigh(Pins.STOP_L);
    HAL.IOs->config->setHigh(Pins.STOP_R);
    HAL.IOs->config->setHigh(Pins.HOME_REF);
    HAL.IOs->config->setHigh(Pins.START);
    HAL.IOs->config->setHigh(Pins.FREEZE);

    HAL.IOs->config->toInput(Pins.STANDBY_CLK);
    HAL.IOs->config->toInput(Pins.INTR);
    HAL.IOs->config->toInput(Pins.TARGET_REACHED);

    TMC4361A_SPIChannel      = &HAL.SPI->ch1;
    TMC4361A_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

    Evalboards.ch1.config->state       = CONFIG_RESET;
    Evalboards.ch1.config->configIndex = 0;
    Evalboards.ch1.config->reset       = reset;
    Evalboards.ch1.config->restore     = restore;

    Evalboards.ch1.cover          = tmc4361A_cover;
    Evalboards.ch1.rotate         = rotate;
    Evalboards.ch1.right          = right;
    Evalboards.ch1.left           = left;
    Evalboards.ch1.stop           = stop;
    Evalboards.ch1.GAP            = GAP;
    Evalboards.ch1.SAP            = SAP;
    Evalboards.ch1.moveTo         = moveTo;
    Evalboards.ch1.moveBy         = moveBy;
    Evalboards.ch1.writeRegister  = writeRegister;
    Evalboards.ch1.readRegister   = readRegister;
    Evalboards.ch1.periodicJob    = periodicJob;
    Evalboards.ch1.userFunction   = userFunction;
    Evalboards.ch1.checkErrors    = checkErrors;
    Evalboards.ch1.numberOfMotors = TMC4361A_MOTORS;
    Evalboards.ch1.deInit         = deInit;

    // Provide the cover function to the driver channel
    Evalboards.ch1.fullCover = tmc4361A_fullCover;
};
