/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5240/TMC5240.h"

static TMC5240BusType activeBus = IC_BUS_SPI;
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5240_SPIChannel;
static UART_Config *TMC5240_UARTChannel;

// Typedefs
typedef struct
void tmc5240_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC5240_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5240_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    ConfigurationTypeDef *config;
    int32_t velocity, oldX;
    uint32_t oldTick;
    uint8_t slaveAddress;
} TMC5240TypeDef;
static TMC5240TypeDef TMC5240;
    UNUSED(icID);
    int32_t status = UART_readWrite(TMC5240_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

TMC5240BusType tmc5240_getBusType(uint16_t icID)
{
    UNUSED(icID);

    return activeBus;
}

uint8_t tmc5240_getNodeAddress(uint16_t icID)
{
    UNUSED(icID);

    return nodeAddress;
}


#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

//#define TMC5240_TIMEOUT 50 // UART Timeout in ms

static bool vMaxModified = false;
static uint32_t vmax_position;
//static uint32_t vMax         = 1;

static bool noRegResetnSLEEP = false;
static uint32_t nSLEEPTick;
static uint32_t targetAddressUart = 0;
//uint8_t tmc5240_CRC8(uint8_t *data, size_t length);

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

static void init_comm(TMC5240BusType mode);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static uint8_t restore();
static void enableDriver(DriverState state);

static TMC5240TypeDef TMC5240;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC5240_CRC(data, length) tmc_CRC8(data, length, 1)

static void writeConfiguration()
// When using multiple ICs you can map them here
static inline TMC5240TypeDef *motorToIC(uint8_t motor)
{
    uint8_t *ptr = &TMC5240.config->configIndex;
    const int32_t *settings;

    settings = tmc5240_sampleRegisterPreset;
    // Find the next resettable register
    while((*ptr < TMC5240_REGISTER_COUNT) && !TMC_IS_RESETTABLE(tmc5240_registerAccess[*ptr]))
    {
        (*ptr)++;
    }

    if(*ptr < TMC5240_REGISTER_COUNT)
    {
        tmc5240_writeRegister(DEFAULT_ICID, *ptr, settings[*ptr]);
        (*ptr)++;
    }
    else // Finished configuration
    {
        TMC5240.config->state = CONFIG_READY;
    }
    UNUSED(motor);
    return &TMC5240;
}

// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc5240_CRC8(uint8_t *data, size_t length)
{
    return tmc_CRC8(data, length, 1);
    //TMC5240_CRC(data, length);
}


typedef struct
{
    IOPinTypeDef  *REFL_UC;
    IOPinTypeDef  *REFR_UC;
    IOPinTypeDef  *DRV_ENN_CFG6;
    IOPinTypeDef  *ENCA_DCIN_CFG5;
    IOPinTypeDef  *ENCB_DCEN_CFG4;
    IOPinTypeDef  *ENCN_DCO;
    IOPinTypeDef  *UART_MODE;
    IOPinTypeDef  *SDI;
    IOPinTypeDef  *SDO;
    IOPinTypeDef  *SCK;
    IOPinTypeDef  *CS;

    IOPinTypeDef  *SWN_DIAG0;
    IOPinTypeDef  *SWP_DIAG1;
    IOPinTypeDef  *nSLEEP;
    IOPinTypeDef  *IREF_R2;
    IOPinTypeDef  *IREF_R3;

} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
    if(motor >= TMC5240_MOTORS)
        return TMC_ERROR_MOTOR;

    tmc5240_rotateMotor(DEFAULT_ICID, motor, velocity);

    return TMC_ERROR_NONE;;
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
    tmc5240_writeRegister(DEFAULT_ICID, TMC5240_RAMPMODE, TMC5240_MODE_POSITION);

    // VMAX also holds the target velocity in velocity mode.
    // Re-write the position mode maximum velocity here.
    tmc5240_writeRegister(DEFAULT_ICID, TMC5240_VMAX, vmax_position);

    tmc5240_writeRegister(DEFAULT_ICID, TMC5240_XTARGET, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    // determine actual position and add numbers of ticks to move
    *ticks += tmc5240_readRegister(DEFAULT_ICID, TMC5240_XACTUAL);

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    int32_t buffer;
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= TMC5240_MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0:
        // Target position
        if(readWrite == READ) {
            readRegister(motor, TMC5240_XTARGET, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_XTARGET, *value);
        }
        break;
    case 1:
        // Actual position
        if(readWrite == READ) {
            readRegister(motor, TMC5240_XACTUAL, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_XACTUAL, *value);
        }
        break;
    case 2:
        // Target speed
        if(readWrite == READ) {
            readRegister(motor, TMC5240_VMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_VMAX, abs(*value));
            vMaxModified = true;
        }
        break;
    case 3:
        // Actual speed
        if(readWrite == READ) {
            readRegister(motor, TMC5240_VACTUAL, value);
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
            if(field_read(motor, TMC5240_RAMPMODE_FIELD) == TMC5240_MODE_POSITION)
                writeRegister(motor, TMC5240_VMAX, abs(*value));
        }
        break;
    case 5:
        // Maximum acceleration
        if(readWrite == READ) {
            readRegister(motor, TMC5240_AMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_AMAX, *value);
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_IHOLD_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_IHOLD_FIELD, *value);
        }
        break;
    case 8:
        // Position reached flag
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_POSITION_REACHED_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 10:
        // Right endstop
        if(readWrite == READ) {
            *value = !field_read(motor, TMC5240_STATUS_STOP_R_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 11:
        // Left endstop
        if(readWrite == READ) {
            *value = !field_read(motor, TMC5240_STATUS_STOP_L_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 12:
        // Automatic right stop
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_STOP_R_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_STOP_R_ENABLE_FIELD, *value);
        }
        break;
    case 13:
        // Automatic left stop
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_STOP_L_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_STOP_L_ENABLE_FIELD, *value);
        }
        break;
    case 14:
        // SW_MODE Register
        if(readWrite == READ) {
            readRegister(motor, TMC5240_SWMODE, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_SWMODE, *value);
        }
        break;
    case 15:
        // Maximum Deceleration
        if(readWrite == READ) {
            readRegister(motor, TMC5240_DMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_DMAX, *value);
        }
        break;
    case 16:
        // Velocity VSTART
        if(readWrite == READ) {
            readRegister(motor, TMC5240_VSTART, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_VSTART, *value);
        }
        break;
    case 17:
        // Acceleration A1
        if(readWrite == READ) {
            readRegister(motor, TMC5240_A1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_A1, *value);
        }
        break;
    case 18:
        // Velocity V1
        if(readWrite == READ) {
            readRegister(motor, TMC5240_V1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_V1, *value);
        }
        break;
    case 19:
        // Deceleration D1
        if(readWrite == READ) {
            readRegister(motor, TMC5240_D1, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_D1, *value);
        }
        break;
    case 20:
        // Velocity VSTOP
        if(readWrite == READ) {
            readRegister(motor, TMC5240_VSTOP, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_VSTOP, *value);
        }
        break;
    case 21:
        // Waiting time after ramp down
        if(readWrite == READ) {
            readRegister(motor, TMC5240_TZEROWAIT, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_TZEROWAIT, *value);
        }
        break;
    case 22:
        // Velocity V2
        if(readWrite == READ) {
            readRegister(motor, TMC5240_V2, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_V2, *value);
        }
        break;
    case 23:
        // Deceleration D2
        if(readWrite == READ) {
            readRegister(motor, TMC5240_D2, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_D2, *value);
        }
        break;
    case 24:
        // Acceleration A2
        if(readWrite == READ) {
            readRegister(motor, TMC5240_A2, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_A2, *value);
        }
        break;
    case 25:
        // TVMAX
        if(readWrite == READ) {
            readRegister(motor, TMC5240_TVMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_TVMAX, *value);
        }
        break;


    case 26:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            readRegister(motor, TMC5240_THIGH, &buffer);
            *value = MIN(0xFFFFF, (1 << 24) / ((buffer)? buffer : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
            writeRegister(motor, TMC5240_THIGH, *value);
        }
        break;
    case 27:
        // Minimum speed for switching to dcStep
        if(readWrite == READ) {
            readRegister(motor, TMC5240_VDCMIN, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_VDCMIN, *value);
        }
        break;
    case 28:
        // High speed chopper mode
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_VHIGHCHM_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_VHIGHCHM_FIELD, *value);
        }
        break;
    case 29:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_VHIGHFS_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_VHIGHFS_FIELD, *value);
        }
        break;
    case 30:
        // Measured Speed
        if(readWrite == READ) {
            *value = TMC5240.velocity;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 34:
        // Internal RSense
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_REFR_DIR_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_REFR_DIR_FIELD, *value);
        }
        break;
    case 35:
        // Global current scaler
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_GLOBAL_SCALER_FIELD);
        } else if(readWrite == WRITE) {
            if(*value > 31)
                field_write(motor, TMC5240_GLOBAL_SCALER_FIELD, *value);
            else
                field_write(motor, TMC5240_GLOBAL_SCALER_FIELD, 0);
        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 0x100 >> field_read(motor, TMC5240_MRES_FIELD);
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
                field_write(motor, TMC5240_MRES_FIELD, *value);
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
            *value = field_read(motor, TMC5240_TBL_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_TBL_FIELD, *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_CHM_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_CHM_FIELD, *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_DISFDCC_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_DISFDCC_FIELD, *value);
        }
        break;
    case 165:
        // Chopper hysteresis end / fast decay time
        readRegister(motor, TMC5240_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5240_CHM_SHIFT))
            {
                *value = (buffer >> TMC5240_HEND_OFFSET_SHIFT) & TMC5240_HEND_OFFSET_MASK;
            }
            else
            {
                readRegister(motor, TMC5240_CHOPCONF >> TMC5240_TFD_ALL_SHIFT, value);
                *value = *value & TMC5240_TFD_ALL_MASK;
                if(buffer & TMC5240_FD3_SHIFT)
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            readRegister(motor, TMC5240_CHOPCONF, &buffer);
            if(buffer & (1<<14))
            {
                field_write(motor, TMC5240_HEND_OFFSET_FIELD, *value);
            }
            else
            {
                field_write(motor, TMC5240_HEND_OFFSET_FIELD, (*value & (1<<3)));// MSB wird zu value dazugefügt
                field_write(motor, TMC5240_TFD_ALL_FIELD, *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        readRegister(motor, TMC5240_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5240_CHM_SHIFT))
            {
                *value = (buffer >> TMC5240_TFD_ALL_SHIFT) & TMC5240_TFD_ALL_MASK;
            }
            else
            {
                *value = (buffer >> TMC5240_HEND_OFFSET_SHIFT) & TMC5240_HEND_OFFSET_MASK;
                if(buffer & (1 << TMC5240_FD3_SHIFT))
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(buffer & (1 << TMC5240_CHM_SHIFT))
            {
                field_write(motor, TMC5240_TFD_ALL_FIELD, *value);
            }
            else
            {
                field_write(motor, TMC5240_HEND_OFFSET_FIELD, *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_TOFF_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_TOFF_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SEMIN_FIELD, *value);
        }
        break;
    case 173:
        // stallGuard4 filter enable
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SG4_FILT_EN_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SG4_FILT_EN_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard4 threshold
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SG4_THRS_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SG4_THRS_FIELD, *value);
        }
        break;
    case 175:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SFILT_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SFILT_FIELD, *value);
        }
        break;
    case 176:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SGT_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SGT_FIELD, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_CS_ACTUAL_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 181:
        // smartEnergy stall velocity
        //this function sort of doubles with 182 but is necessary to allow cross chip compliance
        if(readWrite == READ) {
            if(field_read(motor, TMC5240_SG_STOP_FIELD))
            {
                readRegister(motor, TMC5240_TCOOLTHRS, &buffer);
                *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
            }
            else
            {
                *value = 0;
            }
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_SG_STOP_FIELD, (*value)? 1:0);
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(motor, TMC5240_TCOOLTHRS, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            readRegister(motor, TMC5240_TCOOLTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(motor, TMC5240_TCOOLTHRS, *value);
        }
        break;
    case 184:
            // SG_ANGLE_OFFSET
            if(readWrite == READ) {
                *value = field_read(motor, TMC5240_SG_ANGLE_OFFSET_FIELD);
            } else if(readWrite == WRITE) {
                field_write(motor, TMC5240_SG_ANGLE_OFFSET_FIELD, *value);
            }
            break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            readRegister(motor, TMC5240_CHOPCONF, value);
            *value = (*value >> 20) & 0x0F;
        } else if(readWrite == WRITE) {
            readRegister(motor, TMC5240_CHOPCONF, &buffer);
            buffer &= ~(0x0F<<20);
            buffer |= (*value & 0x0F) << 20;
            writeRegister(motor, TMC5240_CHOPCONF,buffer);
        }
        break;
    case 186:
        // PWM threshold speed
        if(readWrite == READ) {
            readRegister(motor, TMC5240_TPWMTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(motor, TMC5240_TPWMTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            field_write(motor, TMC5240_PWM_GRAD_FIELD, *value);
            // Enable/disable stealthChop accordingly
            field_write(motor, TMC5240_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);
        }
        break;
    case 188:
        // PWM amplitude
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_PWM_OFS_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_PWM_OFS_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                field_write(motor, TMC5240_PWM_FREQ_FIELD, *value);
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
            *value = field_read(motor, TMC5240_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
            {
                field_write(motor, TMC5240_PWM_AUTOSCALE_FIELD, *value);
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
            *value = field_read(motor, TMC5240_PWM_SCALE_SUM_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 194:
        // MSCNT
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_MSCNT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 195:
        // MEAS_SD_EN
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_PWM_MEAS_SD_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                field_write(motor, TMC5240_PWM_MEAS_SD_ENABLE_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 196:
        // DIS_REG_STST
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_PWM_DIS_REG_STST_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                field_write(motor, TMC5240_PWM_DIS_REG_STST_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 204:
        // Freewheeling mode
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_SG_RESULT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 209:
        // Encoder position
        if(readWrite == READ) {
            readRegister(motor, TMC5240_XENC, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_XENC, *value);
        }
        break;
    case 210:
        // Encoder Resolution
        if(readWrite == READ) {
            readRegister(motor, TMC5240_ENC_CONST, value);
        } else if(readWrite == WRITE) {
            writeRegister(motor, TMC5240_ENC_CONST, *value);
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
            *value = field_read(motor, TMC5240_CURRENT_RANGE_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_CURRENT_RANGE_FIELD, *value);
        }
        break;

    case 213:
        // ADCTemperatur
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_ADC_TEMP_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 214:
        // ADCIN
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_ADC_AIN_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 215:
        // ADCSupply
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_ADC_VSUPPLY_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 216:
        // Overvoltage Limit ADC value
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_OVERVOLTAGE_VTH_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_OVERVOLTAGE_VTH_FIELD, *value);
        }
        break;
    case 217:
        // Overtemperature Warning Limit
        if(readWrite == READ) {
            *value = field_read(motor, TMC5240_OVERTEMPPREWARNING_VTH_FIELD);
        } else if(readWrite == WRITE) {
            field_write(motor, TMC5240_OVERTEMPPREWARNING_VTH_FIELD, *value);
        }
        break;
    case 218:
        // ADCTemperatur Converted
        if(readWrite == READ) {

            int32_t adc = field_read(motor, TMC5240_ADC_TEMP_FIELD);
            *value = (int32_t)10*(adc-2038)/77;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 219:
        // ADCIN converted
        if(readWrite == READ) {
            int32_t adc = field_read(motor, TMC5240_ADC_AIN_FIELD);
            *value = (int32_t)3052*adc/10000;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 220:
        // ADCSupply
        if(readWrite == READ) {
            int32_t adc = field_read(motor, TMC5240_ADC_VSUPPLY_FIELD);
            *value = (int32_t)32*3052*adc/10000;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 221:
        // Overvoltage Limit converted
        if(readWrite == READ) {
            int32_t val = field_read(motor, TMC5240_OVERVOLTAGE_VTH_FIELD);
            *value = (int32_t)32*3052*val/10000;
        } else if(readWrite == WRITE) {
            int32_t val = (int32_t)(*value*10000/(3052*32));
            field_write(motor, TMC5240_OVERVOLTAGE_VTH_FIELD, val);
        }
        break;
    case 222:
        // Overtemperature Warning Limit
        if(readWrite == READ) {
            int32_t temp = field_read(motor, TMC5240_OVERTEMPPREWARNING_VTH_FIELD);
            *value = (int32_t)(temp-2038)/7.7;
        } else if(readWrite == WRITE) {
            float valf  = *value*7.7;
            int32_t val = (int32_t)valf;
            val = val+2038;
            field_write(motor, TMC5240_OVERTEMPPREWARNING_VTH_FIELD, val);
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
    if(motor >= TMC5240_MOTORS)
        return TMC_ERROR_MOTOR;

    *value = TMC5240.velocity;

    return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
    UNUSED(motor);
    tmc5240_writeRegister(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
    UNUSED(motor);
    *value = tmc5240_readRegister(DEFAULT_MOTOR, address);
}

static void periodicJob(uint32_t tick)
{
    //check if reset after nSLEEP to HIGH was performed
    if(!noRegResetnSLEEP)
    {
        if(TMC5240.config->state != CONFIG_READY)
        {
            writeConfiguration();
            return;
        }

        int32_t XActual;
        uint32_t tickDiff;

        // Calculate velocity v = dx/dt
        if((tickDiff = tick - TMC5240.oldTick) >= 5)
        {
            XActual = tmc5240_readRegister(DEFAULT_ICID, TMC5240_XACTUAL);
            // ToDo CHECK 2: API Compatibility - write alternative algorithm w/o floating point? (LH)
            TMC5240.velocity = (uint32_t) ((float32_t) ((XActual - TMC5240.oldX) / (float32_t) tickDiff) * (float32_t) 1048.576);

            TMC5240.oldX     = XActual;
            TMC5240.oldTick  = tick;
        }
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
         * The the TMC5240 ref switch input is pulled high by external resistor an can be pulled low either by
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
        /*
    case 9: // Set UART address
        tmc5240_setSlaveAddress()
        break;
*/
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
    HAL.IOs->config->setLow(Pins.UART_MODE);
    //HAL.IOs->config->setLow(Pins.SPI_MODE);
    HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
    HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->reset(Pins.ENCN_DCO);
    HAL.IOs->config->reset(Pins.REFL_UC);
    HAL.IOs->config->reset(Pins.REFR_UC);
    HAL.IOs->config->reset(Pins.SWN_DIAG0);
    HAL.IOs->config->reset(Pins.SWP_DIAG1);
    HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->reset(Pins.UART_MODE);
    //HAL.IOs->config->reset(Pins.SPI_MODE);
    HAL.IOs->config->reset(Pins.nSLEEP);
    HAL.IOs->config->reset(Pins.IREF_R2);
    HAL.IOs->config->reset(Pins.IREF_R3);

};

static uint8_t reset()
{
    int32_t value = 0;
    readRegister(DEFAULT_MOTOR, TMC5240_VACTUAL, &value);

    if(!value)
    {
        if(TMC5240.config->state != CONFIG_READY)
            return false;

        TMC5240.config->state        = CONFIG_RESET;
        TMC5240.config->configIndex  = 0;

        return true;
    }

    HAL.IOs->config->toInput(Pins.REFL_UC);
    HAL.IOs->config->toInput(Pins.REFR_UC);

    return 1;
}

// Restore the TMC5240 to the state stored in the shadow registers.
// This can be used to recover the IC configuration after a VM power loss.
static uint8_t restore()
{
    if(TMC5240.config->state != CONFIG_READY)
        return false;

    TMC5240.config->state        = CONFIG_RESTORE;
    TMC5240.config->configIndex  = 0;

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

static void init_comm(TMC5240BusType mode)
{
    TMC5240_UARTChannel = HAL.UART;
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
        TMC5240_UARTChannel->pinout = UART_PINS_2;
        TMC5240_UARTChannel->rxtx.init();
        break;
    case IC_BUS_SPI:

        HAL.IOs->config->reset(Pins.SCK);
        HAL.IOs->config->reset(Pins.SDI);
        HAL.IOs->config->reset(Pins.SDO);
        HAL.IOs->config->reset(Pins.CS);

        SPI.init();
        HAL.IOs->config->setLow(Pins.UART_MODE);
        TMC5240_UARTChannel->rxtx.deInit();
        TMC5240_SPIChannel = &HAL.SPI->ch1;
        TMC5240_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
        break;
    case IC_BUS_WLAN: // unused
    default:

        HAL.IOs->config->reset(Pins.SCK);
        HAL.IOs->config->reset(Pins.SDI);
        HAL.IOs->config->reset(Pins.SDO);
        HAL.IOs->config->reset(Pins.CS);

        SPI.init();
        HAL.IOs->config->setLow(Pins.UART_MODE);
        TMC5240_UARTChannel->rxtx.deInit();
        TMC5240_SPIChannel = &HAL.SPI->ch1;
        TMC5240_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
        activeBus = IC_BUS_SPI;
        break;
    }
}

void TMC5240_init(void)
{
    Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0; //Pin8
    Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1; //Pin9
    Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2; //Pin10
    Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3; //Pin11
    Pins.REFL_UC         = &HAL.IOs->pins->DIO6; //Pin17
    Pins.REFR_UC         = &HAL.IOs->pins->DIO7; //Pin18
    Pins.UART_MODE       = &HAL.IOs->pins->DIO9;//Pin20
    Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15; //Pin37
    Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16; //Pin38
    Pins.nSLEEP          = &HAL.IOs->pins->DIO8; //Pin19
    Pins.IREF_R2         = &HAL.IOs->pins->DIO13; //Pin35
    Pins.IREF_R3         = &HAL.IOs->pins->DIO14; //Pin36
    Pins.SCK             = &HAL.IOs->pins->SPI1_SCK; //Pin31
    Pins.SDI             = &HAL.IOs->pins->SPI1_SDI; //Pin32
    Pins.SDO             = &HAL.IOs->pins->SPI1_SDO; //Pin33
    Pins.CS              = &HAL.IOs->pins->SPI1_CSN; //Pin33


    HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->toOutput(Pins.UART_MODE);
    HAL.IOs->config->toOutput(Pins.nSLEEP);
    noRegResetnSLEEP = true;
    nSLEEPTick = systick_getTick();
    HAL.IOs->config->toOutput(Pins.IREF_R2);
    HAL.IOs->config->toOutput(Pins.IREF_R3);

    HAL.IOs->config->setHigh(Pins.nSLEEP);
    HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->setLow(Pins.UART_MODE);
    HAL.IOs->config->setLow(Pins.IREF_R2);
    HAL.IOs->config->setLow(Pins.IREF_R2);

    HAL.IOs->config->toInput(Pins.ENCN_DCO);
    HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);
    //HAL.IOs->config->toInput(Pins.SWN_DIAG0);
    //HAL.IOs->config->toInput(Pins.SWP_DIAG1);
    //HAL.IOs->config->toOutput(Pins.SWP_DIAG1);
    //HAL.IOs->config->setToState(Pins.SWP_DIAG1,IOS_OPEN);
    //Pins.SWP_DIAG1->configuration.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    //setPinConfiguration(Pins.SWP_DIAG1);




    HAL.IOs->config->toInput(Pins.REFL_UC);
    HAL.IOs->config->toInput(Pins.REFR_UC);

    // Disable CLK output -> use internal 12 MHz clock
    //  Switchable via user function



    init_comm(activeBus);
    //init_comm(IC_BUS_UART);

    Evalboards.ch1.config->reset        = reset;
    Evalboards.ch1.config->restore      = restore;
    Evalboards.ch1.config->state        = CONFIG_RESET;

    TMC5240.velocity  = 0;
    TMC5240.oldTick   = 0;
    TMC5240.oldX      = 0;

    TMC5240.config               = Evalboards.ch1.config;
    TMC5240.config->callback     = NULL;
    TMC5240.config->channel      = 0;
    TMC5240.config->configIndex  = 0;
    TMC5240.config->state        = CONFIG_READY;

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
    Evalboards.ch1.numberOfMotors       = TMC5240_MOTORS;
    Evalboards.ch1.VMMin                = VM_MIN;
    Evalboards.ch1.VMMax                = VM_MAX;
    Evalboards.ch1.deInit               = deInit;

    enableDriver(DRIVER_USE_GLOBAL_ENABLE);


};
