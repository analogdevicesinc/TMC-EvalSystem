/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5160/TMC5160.h"


#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         80   // VM[V/10] min
#define VM_MAX         590  // VM[V/10] max

#define TMC5160_TIMEOUT 50 // UART Timeout in ms
#define DEFAULT_ICID  0

static bool vMaxModified = false;
static uint32_t vmax_position;

// Typedefs
typedef struct
{
    ConfigurationTypeDef *config;
    int32_t velocity, oldX;
    uint32_t oldTick;

} TMC5160TypeDef;

static TMC5160TypeDef TMC5160;

typedef struct
{
    IOPinTypeDef  *REFL_UC;
    IOPinTypeDef  *REFR_UC;
    IOPinTypeDef  *DRV_ENN_CFG6;
    IOPinTypeDef  *ENCA_DCIN_CFG5;
    IOPinTypeDef  *ENCB_DCEN_CFG4;
    IOPinTypeDef  *ENCN_DCO;
    IOPinTypeDef  *SD_MODE;
    IOPinTypeDef  *SPI_MODE;
    IOPinTypeDef  *SWN_DIAG0;
    IOPinTypeDef  *SWP_DIAG1;

    IOPinTypeDef  *CLK;
    IOPinTypeDef  *SDI;
    IOPinTypeDef  *SDO;
    IOPinTypeDef  *SCK;
    IOPinTypeDef  *CS;
} PinsTypeDef;

static PinsTypeDef Pins;

static TMC5160BusType activeBus = IC_BUS_SPI;
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5160_SPIChannel;
static UART_Config *TMC5160_UARTChannel;

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
static void init_comm(TMC5160BusType mode);
static void periodicJob(uint32_t tick, uint8_t motor);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);


void tmc5160_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC5160_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5160_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
    int32_t status = UART_readWrite(TMC5160_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

TMC5160BusType tmc5160_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint8_t tmc5160_getNodeAddress(uint16_t icID)
{
    UNUSED(icID);

    return nodeAddress;
}

static void writeConfiguration()
{
    uint8_t *ptr = &TMC5160.config->configIndex;
    const int32_t *settings;

    if(TMC5160.config->state == CONFIG_RESTORE)
    {
        settings = *(tmc5160_shadowRegister + 0);
        // Find the next restorable register
        while(*ptr < TMC5160_REGISTER_COUNT)
        {
            // If the register is writable and has been written to, restore it
            if (TMC_IS_WRITABLE(tmc5160_registerAccess[*ptr]) && tmc5160_getDirtyBit(DEFAULT_ICID ,*ptr))
            {
                break;
            }
            (*ptr)++;
        }
    }
    else
    {
        settings = tmc5160_sampleRegisterPreset;
        while((*ptr < TMC5160_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc5160_registerAccess[*ptr]))
        {
            (*ptr)++;
        }
    }

    if(*ptr < TMC5160_REGISTER_COUNT)
    {
        if(*ptr == TMC5160_FACTORY_CONF){

            // Reading reset default value for FCLKTRIM (otp0.0 to otp0.4)
            int32_t otpFclkTrim = tmc5160_readRegister(DEFAULT_ICID, TMC5160_OTP_READ) & TMC5160_OTP_FCLKTRIM_MASK;
            // Writing the reset default value to FCLKTRIM
            tmc5160_writeRegister(DEFAULT_ICID, *ptr, otpFclkTrim);

        }else{
            tmc5160_writeRegister(DEFAULT_ICID, *ptr, settings[*ptr]);
        }
        (*ptr)++;
    }
    else // Finished configuration
    {
        if( TMC5160.config->state == CONFIG_RESET)
        {
            // Fill missing shadow registers (hardware preset registers)
            tmc5160_initCache();
        }
        TMC5160.config->state = CONFIG_READY;
    }
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    // Set absolute velocity
    tmc5160_writeRegister(motor, TMC5160_VMAX, abs(velocity));
    // Set direction
    tmc5160_writeRegister(motor, TMC5160_RAMPMODE, (velocity >= 0) ? TMC5160_MODE_VELPOS : TMC5160_MODE_VELNEG);

    return 0;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
    rotate(motor, velocity);

    return 0;
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
    rotate(motor, -velocity);

    return 0;
}

static uint32_t stop(uint8_t motor)
{
    rotate(motor, 0);

    return 0;
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
    tmc5160_writeRegister(motor, TMC5160_RAMPMODE, TMC5160_MODE_POSITION);

    // VMAX also holds the target velocity in velocity mode.
    // Re-write the position mode maximum velocity here.
    tmc5160_writeRegister(motor, TMC5160_VMAX, vmax_position);

    tmc5160_writeRegister(motor, TMC5160_XTARGET, position);

    return 0;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    // determine actual position and add numbers of ticks to move
    *ticks += tmc5160_readRegister(motor, TMC5160_XACTUAL);

    moveTo(motor, vmax_position);

    return 0;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    uint32_t buffer;
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= TMC5160_MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0:
        // Target position
        if(readWrite == READ) {
            readRegister(motor, TMC5160_XTARGET, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_XTARGET, *value);
        }
        break;
    case 1:
        // Actual position
        if(readWrite == READ) {
            readRegister(motor, TMC5160_XACTUAL, value);

        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_XACTUAL, *value);
        }
        break;
    case 2:
        // Target speed
        if(readWrite == READ) {
            readRegister(motor, TMC5160_VMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_VMAX, abs(*value));
            vMaxModified = true;
        }
        break;
    case 3:
        // Actual speed
        if(readWrite == READ) {
            readRegister(motor, TMC5160_VACTUAL, value);
            *value = CAST_Sn_TO_S32(*value, 24);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 4:
        // Maximum speed
        if(readWrite == READ) {
            *value = vmax_position;
        } else if(readWrite == WRITE) {
            vmax_position = abs(*value);
            readRegister(motor, TMC5160_RAMPMODE, &buffer);
            if(buffer == TMC5160_MODE_POSITION)
                writeRegister(motor, TMC5160_VMAX, abs(*value));
        }
        break;
    case 5:
        // Maximum acceleration
        if(readWrite == READ) {
            readRegister(motor, TMC5160_AMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_AMAX, *value);
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_IHOLD_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_IHOLD_FIELD, *value);
        }
        break;
    case 8:
        // Position reached flag
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_POSITION_REACHED_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 10:
        // Right endstop
        if(readWrite == READ) {
            *value = !tmc5160_fieldRead(motor, TMC5160_STATUS_STOP_R_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 11:
        // Left endstop
        if(readWrite == READ) {
            *value = !tmc5160_fieldRead(motor, TMC5160_STATUS_STOP_L_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 12:
        // Automatic right stop
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_STOP_R_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_STOP_R_ENABLE_FIELD, *value);
        }
        break;
    case 13:
        // Automatic left stop
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_STOP_L_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_STOP_L_ENABLE_FIELD, *value);
        }
        break;
    case 14:
        // SW_MODE Register
        if(readWrite == READ) {
            readRegister(motor, TMC5160_SWMODE, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_SWMODE, *value);
        }
        break;
    case 15:
        // Acceleration A1
        if(readWrite == READ) {
            readRegister(motor, TMC5160_A1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_A1, *value);
        }
        break;
    case 16:
        // Velocity V1
        if(readWrite == READ) {
            readRegister(motor, TMC5160_V1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_V1, *value);
        }
        break;
    case 17:
        // Maximum Deceleration
        if(readWrite == READ) {
            readRegister(motor, TMC5160_DMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_DMAX, *value);
        }
        break;
    case 18:
        // Deceleration D1
        if(readWrite == READ) {
            readRegister(motor, TMC5160_D1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_D1, *value);
        }
        break;
    case 19:
        // Velocity VSTART
        if(readWrite == READ) {
            readRegister(motor, TMC5160_VSTART, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_VSTART, *value);
        }
        break;
    case 20:
        // Velocity VSTOP
        if(readWrite == READ) {
            readRegister(motor, TMC5160_VSTOP, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_VSTOP, *value);
        }
        break;
    case 21:
        // Waiting time after ramp down
        if(readWrite == READ) {
            readRegister(motor, TMC5160_TZEROWAIT, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_TZEROWAIT, *value);
        }
        break;
    case 23:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            readRegister(motor, TMC5160_THIGH, &buffer);
            *value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
            writeRegister(motor, TMC5160_THIGH, *value);
        }
        break;
    case 24:
        // Minimum speed for switching to dcStep
        if(readWrite == READ) {
            readRegister(motor, TMC5160_VDCMIN, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_VDCMIN, *value);
        }
        break;
    case 27:
        // High speed chopper mode
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_VHIGHCHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_VHIGHCHM_FIELD, *value);
        }
        break;
    case 28:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_VHIGHFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_VHIGHFS_FIELD, *value);
        }
        break;
    case 29:
        // Measured Speed
        if(readWrite == READ) {
            *value = TMC5160.velocity;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 33:
        // Analog I Scale
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_RECALIBRATE_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_RECALIBRATE_FIELD, *value);
        }
        break;
    case 34:
        // Internal RSense
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_REFR_DIR_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_REFR_DIR_FIELD, *value);
        }
        break;
    case 35:
        // Global current scaler
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_GLOBAL_SCALER_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_GLOBAL_SCALER_FIELD, *value);
        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 0x100 >> tmc5160_fieldRead(motor, TMC5160_MRES_FIELD);
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
                tmc5160_fieldWrite(motor, TMC5160_MRES_FIELD, *value);
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
            *value = tmc5160_fieldRead(motor, TMC5160_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_TBL_FIELD, *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_CHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_CHM_FIELD, *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_DISFDCC_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_DISFDCC_FIELD, *value);
        }
        break;
    case 165:
        // Chopper hysteresis end / fast decay time
        readRegister(motor, TMC5160_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5160_CHM_SHIFT))
            {
                *value = (buffer >> TMC5160_HEND_SHIFT) & TMC5160_HEND_MASK;
            }
            else
            {
                *value = (buffer >> TMC5160_TFD_ALL_SHIFT) & TMC5160_TFD_ALL_MASK;
                if(buffer & TMC5160_TFD_3_SHIFT)
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(buffer & (1<<14))
            {
                tmc5160_fieldWrite(motor, TMC5160_HEND_FIELD, *value);
            }
            else
            {
                tmc5160_fieldWrite(motor, TMC5160_TFD_3_FIELD, (*value & (1<<3))); // MSB wird zu value dazugefügt
                tmc5160_fieldWrite(motor, TMC5160_TFD_ALL_FIELD, *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        readRegister(motor, TMC5160_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5160_CHM_SHIFT))
            {
                *value = (buffer >> TMC5160_HSTRT_SHIFT) & TMC5160_HSTRT_MASK;
            }
            else
            {
                *value = (buffer >> TMC5160_OFFSET_SHIFT) & TMC5160_OFFSET_MASK;
                if(buffer & (1 << TMC5160_TFD_3_SHIFT))
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(buffer & (1 << TMC5160_CHM_SHIFT))
            {
                tmc5160_fieldWrite(motor, TMC5160_HSTRT_FIELD, *value);
            }
            else
            {
                tmc5160_fieldWrite(motor, TMC5160_OFFSET_FIELD, *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_TOFF_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_TOFF_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SEMIN_FIELD, *value);
        }
        break;
    case 173:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SFILT_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SFILT_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SGT_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SGT_FIELD, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_CS_ACTUAL_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 181:
        // smartEnergy stall velocity
        //this function sort of doubles with 182 but is necessary to allow cross chip compliance
        if(readWrite == READ) {
            if(tmc5160_fieldRead(motor, TMC5160_SG_STOP_FIELD))
            {
                readRegister(motor, TMC5160_TCOOLTHRS, &buffer);
                *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
            }
            else
            {
                *value = 0;
            }
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_SG_STOP_FIELD, (*value)? 1:0);

            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(motor, TMC5160_TCOOLTHRS, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            readRegister(motor, TMC5160_TCOOLTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(motor, TMC5160_TCOOLTHRS, *value);
        }
        break;
    case 184:
        // Random TOff mode
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_RNDTF_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_RNDTF_FIELD, *value);
        }
        break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            readRegister(motor, TMC5160_CHOPCONF, value);
            *value = (*value >> 20) & 0x0F;
        } else if(readWrite == WRITE) {
            readRegister(motor, TMC5160_CHOPCONF, &buffer);
            buffer &= ~(0x0F<<20);
            buffer |= (*value & 0x0F) << 20;
            writeRegister(motor, TMC5160_CHOPCONF, buffer);
        }
        break;
    case 186:
        // PWM threshold speed
        if(readWrite == READ) {
            readRegister(motor, TMC5160_TPWMTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(motor, TMC5160_TPWMTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            tmc5160_fieldWrite(motor, TMC5160_PWM_GRAD_FIELD, *value);
            // Enable/disable stealthChop accordingly
            tmc5160_fieldWrite(motor, TMC5160_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);
        }
        break;
    case 188:
        // PWM amplitude
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_PWM_OFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_PWM_OFS_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc5160_fieldWrite(motor, TMC5160_PWM_FREQ_FIELD, *value);
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
            *value = tmc5160_fieldRead(motor, TMC5160_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
            {
                tmc5160_fieldWrite(motor, TMC5160_PWM_AUTOSCALE_FIELD, *value);
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
            *value = tmc5160_fieldRead(motor, TMC5160_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            tmc5160_fieldWrite(motor, TMC5160_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc5160_fieldRead(motor, TMC5160_SG_RESULT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 209:
        // Encoder position
        if(readWrite == READ) {
            readRegister(motor, TMC5160_XENC, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_XENC, *value);
        }
        break;
    case 210:
        // Encoder Resolution
        if(readWrite == READ) {
            readRegister(motor, TMC5160_ENC_CONST, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5160_ENC_CONST, *value);
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
    if(motor >= TMC5160_MOTORS)
        return TMC_ERROR_MOTOR;

    *value = TMC5160.velocity;

    return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc5160_writeRegister(DEFAULT_ICID, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc5160_readRegister(DEFAULT_ICID, address );
}

static void periodicJob(uint32_t tick, uint8_t motor)
{
    if(TMC5160.config->state != CONFIG_READY)
    {
        writeConfiguration();
        return;
    }

    int32_t XActual;
    uint32_t tickDiff;

    // Calculate velocity v = dx/dt
    if((tickDiff = tick - TMC5160.oldTick) >= 5)
    {
        XActual = tmc5160_readRegister(motor, TMC5160_XACTUAL);
        // ToDo CHECK 2: API Compatibility - write alternative algorithm w/o floating point? (LH)
        TMC5160.velocity = (uint32_t) ((float32_t) ((XActual - TMC5160.oldX) / (float32_t) tickDiff) * (float32_t) 1048.576);

        TMC5160.oldX     = XActual;
        TMC5160.oldTick  = tick;
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
         * The the TMC5160 ref switch input is pulled high by external resistor an can be pulled low either by
         * this µC or external signal. To use external signal make sure the signals from µC are high or floating.
         */
        if(!(*value & ~3))
        {
            if(*value & (1<<0))
            {
                HAL.IOs->config->toInput(Pins.REFR_UC); // pull up -> set it to floating causes high
            }
            else
            {
                HAL.IOs->config->toOutput(Pins.REFR_UC);
                HAL.IOs->config->setLow(Pins.REFR_UC);
            }

            if(*value & (1<<1))
            {
                HAL.IOs->config->toInput(Pins.REFL_UC); // pull up -> set it to floating causes high
            }
            else
            {
                HAL.IOs->config->toOutput(Pins.REFL_UC);
                HAL.IOs->config->setLow(Pins.REFL_UC);
            }
        }
        else
        {
            errors |= TMC_ERROR_VALUE;
        }
        break;
//  case 1:  // set analogue current duty
//      /*
//       * Current will be defined by analogue *value voltage or current signal. In any case this function
//       * will generate a analogue voltage by PWM for up to 50% duty and a switch for the other 50%.
//       * The reference voltage will be AIN_REF = VCC_IO * *value/20000 with *value = {0..20000}
//       */
//
//      buffer = (uint32_t) *value;
//
//      if(buffer <= 20000)
//      {
//          if(buffer > 10000)HAL.IOs->config->setHigh(Pins.AIN_REF_SW);
//          else HAL.IOs->config->setLow(Pins.AIN_REF_SW);
//
//          Timer.setDuty(buffer%10001);
//      }
//      else errors |= TMC_ERROR_VALUE;
//      break;
    case 2:  // Use internal clock
        /*
         * Internel clock will be enabled by calling this function with a *value != 0 and unpower and repower the motor supply while keeping usb connected.
         */
        if(*value)
        {
            HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
            HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);
        }
        else
        {
            HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
        }
        break;
    case 3: // Write/Read SD_MODE pin
        if(motor)
        {   // Write
            // Use Bit 0 here explicitly to allow extension of the UF for more pins if ever needed
            if(*value & 0x00000001)
                HAL.IOs->config->setHigh(Pins.SD_MODE);
            else
                HAL.IOs->config->setLow(Pins.SD_MODE);
        }
        else
        {   // Read
            *value = (HAL.IOs->config->isHigh(Pins.SD_MODE))? 1:0;
        }
        break;
    case 4:  // set or release/read ENCB_[DCEN_CFG4]
        switch(buffer = *value)
        {
        case 0:
            HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
            HAL.IOs->config->setLow(Pins.ENCB_DCEN_CFG4);
            break;
        case 1:
            HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
            HAL.IOs->config->setHigh(Pins.ENCB_DCEN_CFG4);
            break;
        default:
            HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
            buffer = HAL.IOs->config->isHigh(Pins.ENCB_DCEN_CFG4);;
            break;
        }
        *value = buffer;
        break;
    case 5:  // read interrupt pin SWN_DIAG0
        *value = (HAL.IOs->config->isHigh(Pins.SWN_DIAG0))? 1:0;
        break;
    case 6:  // read interrupt pin SWP_DIAG1
        *value = (HAL.IOs->config->isHigh(Pins.SWP_DIAG1))? 1:0;
        break;
//  case 7:  // enable single wire interface (SWSEL)
//          if(*value == 1) HAL.IOs->config->setHigh(Pins.SWSEL);
//          else HAL.IOs->config->setLow(Pins.SWSEL);
//      break;
    case 8: // Enable UART mode
        if(*value == 1)
            activeBus = IC_BUS_UART;
        else if(*value == 0)
            activeBus = IC_BUS_SPI;
        init_comm(activeBus);
        break;
    case 9: // Switch between internal (0) / external (1) clock
        if(*value == 1) {
            HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
            HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);
        } else {
            HAL.IOs->config->reset(&HAL.IOs->pins->CLK16);
        }
        break;
    case 252:
        if(*value)
        {
            HAL.IOs->config->toOutput(Pins.ENCB_DCEN_CFG4);
            HAL.IOs->config->setLow(Pins.ENCB_DCEN_CFG4);
        }
        else
        {
            HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
        }
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static void deInit(void)
{
    HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->setLow(Pins.SD_MODE);
    HAL.IOs->config->setLow(Pins.SPI_MODE);
    HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
    HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->reset(Pins.ENCN_DCO);
    HAL.IOs->config->reset(Pins.REFL_UC);
    HAL.IOs->config->reset(Pins.REFR_UC);
    HAL.IOs->config->reset(Pins.SWN_DIAG0);
    HAL.IOs->config->reset(Pins.SWP_DIAG1);
    HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->reset(Pins.SD_MODE);
    HAL.IOs->config->reset(Pins.SPI_MODE);
};

static uint8_t reset()
{
    if(!tmc5160_readRegister(DEFAULT_ICID, TMC5160_VACTUAL)){
        if(TMC5160.config->state != CONFIG_READY)
            return false;

        // Reset the dirty bits and wipe the shadow registers
        size_t i;
        for(i = 0; i < TMC5160_REGISTER_COUNT; i++)
        {
            tmc5160_setDirtyBit(DEFAULT_ICID, i, false);
            tmc5160_shadowRegister[DEFAULT_ICID][i] = 0;
        }

        TMC5160.config->state        = CONFIG_RESET;
        TMC5160.config->configIndex  = 0;

        return true;
    }
    HAL.IOs->config->toInput(Pins.REFL_UC);
    HAL.IOs->config->toInput(Pins.REFR_UC);

    return 1;
}

static uint8_t restore()
{
    if(TMC5160.config->state != CONFIG_READY)
        return false;

    TMC5160.config->state        = CONFIG_RESTORE;
    TMC5160.config->configIndex  = 0;

    return true;
}

static void enableDriver(DriverState state)
{
    if(state == DRIVER_USE_GLOBAL_ENABLE)
        state = Evalboards.driverEnable;

    if(state ==  DRIVER_DISABLE)
        HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
    else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
        HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
}

static void init_comm(TMC5160BusType mode)
{
	TMC5160_UARTChannel = HAL.UART;
    switch(mode) {
    case IC_BUS_UART:
		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);
		HAL.IOs->config->toOutput(Pins.SDI);
		HAL.IOs->config->toOutput(Pins.SDO);
		HAL.IOs->config->toOutput(Pins.SCK);
		HAL.IOs->config->toOutput(Pins.CS);

		HAL.IOs->config->setLow(Pins.SDI); //NAI
		HAL.IOs->config->setLow(Pins.SDO); //NAO
		HAL.IOs->config->setLow(Pins.CS);
		HAL.IOs->config->setLow(Pins.SCK);
		HAL.IOs->config->setLow(Pins.SPI_MODE);
		HAL.IOs->config->setLow(Pins.SD_MODE);

		TMC5160_UARTChannel = HAL.UART;
		TMC5160_UARTChannel->rxtx.init();
        break;
    case IC_BUS_SPI:
		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setHigh(Pins.SPI_MODE);
		TMC5160_UARTChannel->rxtx.deInit();
		TMC5160_SPIChannel = &HAL.SPI->ch1;
		TMC5160_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
    default:
		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setHigh(Pins.SPI_MODE);
		TMC5160_UARTChannel->rxtx.deInit();
		TMC5160_SPIChannel = &HAL.SPI->ch1;
		TMC5160_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		activeBus = IC_BUS_SPI;
		break;
    }
}

void TMC5160_init(void)
{
    Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0;
    Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1;
    Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2;
    Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3;
    Pins.REFL_UC         = &HAL.IOs->pins->DIO6;
    Pins.REFR_UC         = &HAL.IOs->pins->DIO7;
    Pins.SD_MODE         = &HAL.IOs->pins->DIO9;

	Pins.SCK             = &HAL.IOs->pins->SPI1_SCK;
	Pins.SDI             = &HAL.IOs->pins->SPI1_SDI;
	Pins.SDO             = &HAL.IOs->pins->SPI1_SDO;
	Pins.CS              = &HAL.IOs->pins->SPI1_CSN;

#if defined(LandungsbrueckeV3)
    Pins.SPI_MODE        = &HAL.IOs->pins->DIO11_PWM_WH;
#else
    Pins.SPI_MODE        = &HAL.IOs->pins->DIO11;
#endif
    Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15;
    Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16;

    HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->toOutput(Pins.SD_MODE);
    HAL.IOs->config->toOutput(Pins.SPI_MODE);

    HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->setLow(Pins.SD_MODE);

    HAL.IOs->config->toInput(Pins.ENCN_DCO);
    HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);
    HAL.IOs->config->toInput(Pins.SWN_DIAG0);
    HAL.IOs->config->toInput(Pins.SWP_DIAG1);
    HAL.IOs->config->toInput(Pins.REFL_UC);
    HAL.IOs->config->toInput(Pins.REFR_UC);

    // Disable CLK output -> use internal 12 MHz clock
    // Switchable via user function
    //  HAL.IOs->config->toOutput(&HAL.IOs->pins->CLK16);
    //  HAL.IOs->config->setLow(&HAL.IOs->pins->CLK16);

    init_comm(activeBus);

    TMC5160.velocity  = 0;
    TMC5160.oldTick   = 0;
    TMC5160.oldX      = 0;

    TMC5160.config = Evalboards.ch1.config;
    TMC5160.config->callback     = NULL;
    TMC5160.config->channel      = 0;
    TMC5160.config->configIndex  = 0;
    TMC5160.config->state        = CONFIG_READY;

    Evalboards.ch1.config->reset        = reset;
    Evalboards.ch1.config->restore      = restore;
    Evalboards.ch1.config->state        = CONFIG_RESET;

    vmax_position = 0;

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
    Evalboards.ch1.numberOfMotors       = TMC5160_MOTORS;
    Evalboards.ch1.VMMin                = VM_MIN;
    Evalboards.ch1.VMMax                = VM_MAX;
    Evalboards.ch1.deInit               = deInit;

    enableDriver(DRIVER_USE_GLOBAL_ENABLE);
};
