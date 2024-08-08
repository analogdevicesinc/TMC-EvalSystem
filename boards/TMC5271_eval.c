/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"
#include "tmc/ic/TMC5271/TMC5271.h"

static TMC5271BusType activeBus = IC_BUS_SPI;
static uint8_t nodeAddress = 0;
static SPIChannelTypeDef *TMC5271_SPIChannel;
static UART_Config *TMC5271_UARTChannel;

#define DEFAULT_MOTOR  0

void tmc5271_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    UNUSED(icID);
    TMC5271_SPIChannel->readWriteArray(data, dataLength);
}

bool tmc5271_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
    int32_t status = UART_readWrite(TMC5271_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}
#define DEFAULT_ICID  0

// Typedefs
typedef struct
TMC5271BusType tmc5271_getBusType(uint16_t icID)
{
    ConfigurationTypeDef *config;
    int32_t oldX;
    int32_t velocity;
    uint32_t oldTick;
} TMC5271TypeDef;
static TMC5271TypeDef TMC5271;
    UNUSED(icID);

    return activeBus;
}

uint8_t tmc5271_getNodeAddress(uint16_t icID)
{
    UNUSED(icID);

    return nodeAddress;
}


#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         660  // VM[V/10] max

static bool vMaxModified = false;
static uint32_t vmax_position[TMC5271_MOTORS];

static bool noRegResetnSLEEP = false;
static uint32_t nSLEEPTick;
static uint32_t targetAddressUart = 0;

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

static void init_comm(TMC5271BusType mode);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static TMC5271TypeDef TMC5271;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC5271_CRC(data, length) tmc_CRC8(data, length, 1)

// When using multiple ICs you can map them here
static inline TMC5271TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &TMC5271;
}

// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc5271_CRC8(uint8_t *data, size_t length)
{
	return tmc_CRC8(data, length, 1);
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
	IOPinTypeDef  *CLK;
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
    if(motor >= TMC5271_MOTORS)
        return TMC_ERROR_MOTOR;

    tmc5271_writeRegister(DEFAULT_ICID, TMC5271_VMAX, abs(velocity));
    tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_RAMPMODE_FIELD,  (velocity >= 0) ? TMC5271_MODE_VELPOS : TMC5271_MODE_VELNEG);

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
    if(motor >= TMC5271_MOTORS)
        return TMC_ERROR_MOTOR;

    tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_RAMPMODE_FIELD, TMC5271_MODE_POSITION);
    tmc5271_writeRegister(DEFAULT_ICID, TMC5271_VMAX, vmax_position[motor]);
    tmc5271_writeRegister(DEFAULT_ICID, TMC5271_XTARGET, position);

    return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
    // determine actual position and add numbers of ticks to move
    *ticks += tmc5271_readRegister(DEFAULT_ICID, TMC5271_XACTUAL);

    return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
    int32_t buffer;
    uint32_t errors = TMC_ERROR_NONE;

    if(motor >= TMC5271_MOTORS)
        return TMC_ERROR_MOTOR;

    switch(type)
    {
    case 0:
        // Target position
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_XTARGET, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_XTARGET, *value);
        }
        break;
    case 1:
        // Actual position
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5271_XACTUAL, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5271_XACTUAL, *value);
        }
        break;
    case 2:
        // Target speed
        if(readWrite == READ) {
            if (tmc5271_fieldRead(DEFAULT_ICID, TMC5271_RAMPMODE_FIELD ) == 2){
                readRegister(DEFAULT_ICID,TMC5271_VMAX, value);
                *value = -(*value);
            }
            else
                readRegister(DEFAULT_ICID,TMC5271_VMAX, value);
        }
        else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5271_VMAX, abs(*value));
            vMaxModified = true;
        }
        break;
    case 3:
        // Actual speed
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5271_VACTUAL, value);
            *value = CAST_Sn_TO_S32(*value, 24);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 4:
        // Maximum speed
        if(readWrite == READ) {
            *value = vmax_position[motor];
        }
        else if(readWrite == WRITE) {
            vmax_position[motor] = abs(*value);
            if(tmc5271_fieldRead(DEFAULT_ICID, TMC5271_RAMPMODE_FIELD) == TMC5271_MODE_POSITION)
                writeRegister(DEFAULT_ICID,TMC5271_VMAX, abs(*value));
        }
        break;
    case 5:
        // Maximum acceleration
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5271_AMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5271_AMAX, *value);
        }
        break;
    case 6:
        // Maximum current
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_IRUN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_IRUN_FIELD, *value);
        }
        break;
    case 7:
        // Standby current
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_IHOLD_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_IHOLD_FIELD, *value);
        }
        break;
    case 8:
        // Position reached flag
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_POSITION_REACHED_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 10:
        // Right endstop
        if(readWrite == READ) {
            *value = !tmc5271_fieldRead(DEFAULT_ICID, TMC5271_STATUS_STOP_R_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 11:
        // Left endstop
        if(readWrite == READ) {
            *value = !tmc5271_fieldRead(DEFAULT_ICID, TMC5271_STATUS_STOP_L_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 12:
        // Automatic right stop
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SW_MODE_STOP_R_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SW_MODE_STOP_R_ENABLE_FIELD, *value);
        }
        break;
    case 13:
        // Automatic left stop
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SW_MODE_STOP_L_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SW_MODE_STOP_L_ENABLE_FIELD, *value);
        }
        break;
    case 14:
        // SW_MODE Register
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5271_SW_MODE, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5271_SW_MODE, *value);
        }
        break;
    case 15:
        // Maximum Deceleration
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID,TMC5271_DMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID,TMC5271_DMAX, *value);
        }
        break;
    case 16:
        // Velocity VSTART
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_VSTART, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_VSTART, *value);
        }
        break;
    case 17:
        // Acceleration A1
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_A1, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_A1, *value);
        }
        break;
    case 18:
        // Velocity V1
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_V1, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_V1, *value);
        }
        break;
    case 19:
        // Deceleration D1
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_V1, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_V1, *value);
        }
        break;
    case 20:
        // Velocity VSTOP
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_VSTOP, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_VSTOP, *value);
        }
        break;
    case 21:
        // Waiting time after ramp down
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_TZEROWAIT, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_TZEROWAIT, *value);
        }
        break;
    case 22:
        // Velocity V2
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_V2, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_V2, *value);
        }
        break;
    case 23:
        // Deceleration D2
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_D2, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_D2, *value);
        }
        break;
    case 24:
        // Acceleration A2
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_A2, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_A2, *value);
        }
        break;
    case 25:
        // TVMAX
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_TVMAX, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_TVMAX, *value);
        }
        break;
    case 26:
        // Speed threshold for high speed mode
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_TVMAX, value);
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value : 1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1 << 24) / ((*value)? *value:1));
            writeRegister(DEFAULT_ICID, TMC5271_THIGH, *value);
        }
        break;
    case 27:
        // Minimum speed for switching to dcStep
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_VDCMIN, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_VDCMIN, *value);
        }
        break;
    case 28:
        // High speed chopper mode
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_VHIGHCHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_VHIGHCHM_FIELD, *value);
        }
        break;
    case 29:
        // High speed fullstep mode
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_VHIGHFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_VHIGHFS_FIELD, *value);
        }
        break;
    case 30:
        // Measured Speed
        if(readWrite == READ) {
            *value = (int32_t)TMC5271.velocity;
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;


    case 35:
        // Global current scaler A
        if(readWrite == READ) {
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_GLOBALSCALER_A_FIELD);
        } else if(readWrite == WRITE) {
            if(*value > 31)
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_GLOBALSCALER_A_FIELD, *value);
            else
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_GLOBALSCALER_A_FIELD, 0);
        }
        break;
    case 36:
        // Global current scaler B
        if(readWrite == READ) {
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_GLOBALSCALER_B_FIELD);
        }
        else if(readWrite == WRITE) {

            if(*value > 31)
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_GLOBALSCALER_B_FIELD, *value);
            else
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_GLOBALSCALER_B_FIELD, 0);


        }
        break;
    case 140:
        // Microstep Resolution
        if(readWrite == READ) {
            *value = 0x100 >> tmc5271_fieldRead(DEFAULT_ICID, TMC5271_MRES_FIELD);
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
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_MRES_FIELD, *value);
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
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_TBL_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_TBL_FIELD, *value);
        }
        break;
    case 163:
        // Constant TOff Mode
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_CHM_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_CHM_FIELD, *value);
        }
        break;
    case 164:
        // Disable fast decay comparator
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_DISFDCC_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_DISFDCC_FIELD, *value);
        }
        break;
    case 165:
        // Chopper hysteresis end / fast decay time
        readRegister(DEFAULT_ICID, TMC5271_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5271_CHM_SHIFT))
            {
                *value = (buffer >> TMC5271_HEND_OFFSET_SHIFT) & TMC5271_HEND_OFFSET_MASK;
            }
            else
            {
                *value = (buffer >> TMC5271_HSTRT_TFD210_SHIFT) & TMC5271_HSTRT_TFD210_MASK;

                if(buffer & TMC5271_HSTRT_TFD210_SHIFT)
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            readRegister(DEFAULT_ICID, TMC5271_CHOPCONF, &buffer);
            if(buffer & (1<<14))
            {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_HEND_OFFSET_FIELD, *value);
            }
            else
            {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_HSTRT_TFD210_FIELD, (*value & (1<<3))); // MSB wird zu value dazugefügt
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_HSTRT_TFD210_FIELD, *value);
            }
        }
        break;
    case 166:
        // Chopper hysteresis start / sine wave offset
        readRegister(DEFAULT_ICID, TMC5271_CHOPCONF, &buffer);
        if(readWrite == READ) {
            if(buffer & (1 << TMC5271_CHM_SHIFT))
            {
                *value = (buffer >> TMC5271_HSTRT_TFD210_SHIFT) & TMC5271_HSTRT_TFD210_MASK;
            }
            else
            {
                *value = (buffer >> TMC5271_HEND_OFFSET_SHIFT) & TMC5271_HEND_OFFSET_MASK;
                if(buffer & (1 << TMC5271_FD3_SHIFT))
                    *value |= 1<<3; // MSB wird zu value dazugefügt
            }
        } else if(readWrite == WRITE) {
            if(buffer & (1 << TMC5271_CHM_SHIFT))
            {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_HSTRT_TFD210_FIELD, *value);
            }
            else
            {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_HEND_OFFSET_FIELD, *value);
            }
        }
        break;
    case 167:
        // Chopper off time
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_TOFF_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_TOFF_FIELD, *value);
        }
        break;
    case 168:
        // smartEnergy current minimum (SEIMIN)
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SEIMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SEIMIN_FIELD, *value);
        }
        break;
    case 169:
        // smartEnergy current down step
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SEDN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SEDN_FIELD, *value);
        }
        break;
    case 170:
        // smartEnergy hysteresis
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SEMAX_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SEMAX_FIELD, *value);
        }
        break;
    case 171:
        // smartEnergy current up step
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SEUP_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SEUP_FIELD, *value);
        }
        break;
    case 172:
        // smartEnergy hysteresis start
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SEMIN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SEMIN_FIELD, *value);
        }
        break;
    case 173:
        // stallGuard4 filter enable
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SG4_FILT_EN_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SG4_FILT_EN_FIELD, *value);
        }
        break;
    case 174:
        // stallGuard4 threshold
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SG4_THRS_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SG4_THRS_FIELD, *value);
        }
        break;
    case 175:
        // stallGuard2 filter enable
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SFILT_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SFILT_FIELD, *value);
        }
        break;
    case 176:
        // stallGuard2 threshold
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SGT_FIELD);
            *value = CAST_Sn_TO_S32(*value, 7);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SGT_FIELD, *value);
        }
        break;
    case 180:
        // smartEnergy actual current
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_CS_ACTUAL_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 181:
        // smartEnergy stall velocity
        //this function sort of doubles with 182 but is necessary to allow cross chip compliance
        if(readWrite == READ) {
            if(tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SW_MODE_SG_STOP_FIELD))
            {
                readRegister(DEFAULT_ICID, TMC5271_TCOOLTHRS, &buffer);
                *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
            }
            else
            {
                *value = 0;
            }
        } else if(readWrite == WRITE) {

            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SW_MODE_SG_STOP_FIELD, (*value)? 1:0);
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(DEFAULT_ICID, TMC5271_TCOOLTHRS, *value);
        }
        break;
    case 182:
        // smartEnergy threshold speed
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_TCOOLTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
            writeRegister(DEFAULT_ICID, TMC5271_TCOOLTHRS, *value);
        }
        break;
    case 184:
            // SG_ANGLE_OFFSET
            if(readWrite == READ) {
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SG_ANGLE_OFFSET_FIELD);
            } else if(readWrite == WRITE) {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_SG_ANGLE_OFFSET_FIELD, *value);
            }
            break;
    case 185:
        // Chopper synchronization
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_CHOPCONF, value);
            *value = (*value >> 20) & 0x0F;
        } else if(readWrite == WRITE) {
            readRegister(DEFAULT_ICID, TMC5271_CHOPCONF, &buffer);
            buffer &= ~(0x0F<<20);
            buffer |= (*value & 0x0F) << 20;
            writeRegister(DEFAULT_ICID, TMC5271_CHOPCONF, buffer);
        }
        break;
    case 186:
        // PWM threshold speed
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_TPWMTHRS, &buffer);
            *value = MIN(0xFFFFF, (1<<24) / ((buffer)? buffer:1));
        } else if(readWrite == WRITE) {
            *value = MIN(0xFFFFF, (1<<24) / ((*value)? *value:1));
             writeRegister(DEFAULT_ICID, TMC5271_TPWMTHRS, *value);
        }
        break;
    case 187:
        // PWM gradient
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_GRAD_FIELD);
        } else if(readWrite == WRITE) {
            // Set gradient
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_PWM_GRAD_FIELD, *value);
            // Enable/disable stealthChop accordingly
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_EN_PWM_MODE_FIELD, (*value) ? 1 : 0);

        }
        break;
    case 188:
        // PWM amplitude
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_OFS_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_PWM_OFS_FIELD, *value);
        }
        break;
    case 191:
        // PWM frequency
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_FREQ_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 4)
            {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_PWM_FREQ_FIELD, *value);
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
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_AUTOSCALE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
            {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_PWM_AUTOSCALE_FIELD, *value);
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
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_SCALE_SUM_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 194:
        // MSCNT
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_MSCNT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 195:
        // MEAS_SD_EN
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_MEAS_SD_ENABLE_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_PWM_MEAS_SD_ENABLE_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 196:
        // DIS_REG_STST
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_PWM_DIS_REG_STST_FIELD);
        } else if(readWrite == WRITE) {
            if(*value >= 0 && *value < 2)
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_PWM_DIS_REG_STST_FIELD, *value);
            else
                errors |= TMC_ERROR_TYPE;
        }
        break;
    case 204:
        // Freewheeling mode
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_FREEWHEEL_FIELD);
        } else if(readWrite == WRITE) {
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_FREEWHEEL_FIELD, *value);
        }
        break;
    case 206:
        // Load value
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_SG_RESULT_FIELD);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 209:
        // Encoder position
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_X_ENC, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_X_ENC, *value);
        }
        break;
    case 210:
        // Encoder Resolution
        if(readWrite == READ) {
            readRegister(DEFAULT_ICID, TMC5271_ENC_CONST, value);
        } else if(readWrite == WRITE) {
            writeRegister(DEFAULT_ICID, TMC5271_X_ENC, *value);
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
        // FSR range from DRV_CONF reg
        if(readWrite == READ) {
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_FSR_FIELD);

        } else if(readWrite == WRITE) {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_FSR_FIELD, *value);

        }
        break;
    case 213:
            // ADCTemperatur
            if(readWrite == READ) {
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_ADC_TEMPERATURE_FIELD);
            } else if(readWrite == WRITE) {
                errors |= TMC_ERROR_TYPE;
            }
            break;
    case 214:
        // ADCTemperatur Converted
        if(readWrite == READ) {
            int32_t adc = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_ADC_TEMPERATURE_FIELD);
            *value = (int32_t)((2.03*adc)-259);
        } else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
        }
        break;
    case 215:
        // Scales the reference current IREF of Axis M0: 0x0: 25 % IREF 0x1: 50 % IREF 0x2: 75 % IREF 0x3: 100% IREF Use this together with FSR_M0 for fine current scaling.
        if(readWrite == READ) {
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_FSR_IREF_FIELD);

        } else if(readWrite == WRITE) {
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_FSR_IREF_FIELD, *value);
        }
        break;
    case 216:
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
    case 220:
        // MSLUT0
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x00);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);

        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x00);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;
    case 221:
        // MSLUT1
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x01);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);

        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x01);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;

    case 222:
        // MSLUT2
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x02);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x02);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;

    case 223:
        // MSLUT3
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x03);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x03);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;

    case 224:
        // MSLUT4
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x04);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x04);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;

    case 225:
        // MSLUT5
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x05);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x05);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;
    case 226:
        // MSLUT6
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x06);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x06);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;
    case 227:
        // MSLUT7
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x07);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x07);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;
    case 228:
        // MSLUT_START
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x08);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x08);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;
    case 229:
        // MSLUT_SEL
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x09);
                readRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, value);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x09);
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_DATA, *value);
            }
        break;
    case 230:
        // START_SIN90
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x08);
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_MSLUT_DATA_START_SIN90_FIELD);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x08);
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_MSLUT_DATA_START_SIN90_FIELD, *value);
            }
        break;
    case 231:
        // OFFSET_SIN90
        if(readWrite == READ) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x08);
                *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_MSLUT_DATA_OFFSET_SIN90_FIELD);
        } else if(readWrite == WRITE) {
                writeRegister(DEFAULT_ICID, TMC5271_MSLUT_ADDR, 0x08);
                tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_MSLUT_DATA_OFFSET_SIN90_FIELD, *value);
            }
        break;
    case 232:
        // SG4_IND_0
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_IND_0_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 233:
        // SG4_IND_1
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_IND_1_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 234:
        // SG4_IND_2
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_IND_2_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 235:
        // SG4_IND_3
        if(readWrite == READ) {
            *value = tmc5271_fieldRead(DEFAULT_ICID, TMC5271_IND_3_FIELD);
        }
        else if(readWrite == WRITE) {
            errors |= TMC_ERROR_TYPE;
    }
        break;
    case 255:
        // DEBUG: SPI-FREQ
        if(readWrite == READ) {
            *value = spi_getFrequency(TMC5271_SPIChannel);
        } else if(readWrite == WRITE) {
            spi_setFrequency(TMC5271_SPIChannel, *value);
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
    if(motor >= TMC5271_MOTORS)
        return TMC_ERROR_MOTOR;

    *value = 0;//TMC5271.velocity;

    return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t icID, uint16_t address, int32_t value)
{
    tmc5271_writeRegister(DEFAULT_ICID, address, value);
}

static void readRegister(uint8_t icID, uint16_t address, int32_t *value)
{
    *value = tmc5271_readRegister(DEFAULT_ICID, address);
}

static void periodicJob(uint32_t tick)
{
    if(!noRegResetnSLEEP)
    {
        //check if reset after nSLEEP to HIGH was performed
        uint32_t tickDiff;

        if(TMC5271.config->state != CONFIG_READY)
        {
            TMC5271.config->state = CONFIG_READY;;
            return;
        }

        int32_t x;

        // Calculate velocity v = dx/dt
        if((tickDiff = tick - TMC5271.oldTick) >= 5)
        {
            for(uint8_t motor = 0; motor < TMC5271_MOTORS; motor++)
            {
                x = tmc5271_readRegister(DEFAULT_ICID, TMC5271_XACTUAL);
                TMC5271.velocity = (uint32_t) ((float32_t) ((x - TMC5271.oldX) / (float32_t) tickDiff) * (float32_t) 1048.576);
                TMC5271.oldX = x;
            }
            TMC5271.oldTick  = tick;
        }
    }
    else
    {
        //check if minimum time since chip activation passed. Then restore.
        if((systick_getTick()-nSLEEPTick)>20) //
        {
            noRegResetnSLEEP = false;
            enableDriver(DRIVER_ENABLE);
            tmc5271_fieldWrite(DEFAULT_ICID, TMC5271_TOFF_FIELD, 3);
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
		 * The the TMC5271 ref switch input is pulled high by external resistor an can be pulled low either by
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

	case 8: // Enable UART mode
		if(*value == 1)
			activeBus = IC_BUS_UART;
		else if(*value == 0)
			activeBus = IC_BUS_SPI;
		init_comm(activeBus);
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
	HAL.IOs->config->setLow(Pins.UART_MODE);
	HAL.IOs->config->reset(Pins.ENCA_DCIN_CFG5);
	HAL.IOs->config->reset(Pins.ENCB_DCEN_CFG4);
	HAL.IOs->config->reset(Pins.ENCN_DCO);
	HAL.IOs->config->reset(Pins.REFL_UC);
	HAL.IOs->config->reset(Pins.REFR_UC);
	HAL.IOs->config->reset(Pins.SWN_DIAG0);
	HAL.IOs->config->reset(Pins.SWP_DIAG1);
	HAL.IOs->config->reset(Pins.DRV_ENN_CFG6);
	HAL.IOs->config->reset(Pins.UART_MODE);
	HAL.IOs->config->reset(Pins.nSLEEP);
	HAL.IOs->config->reset(Pins.IREF_R2);
	HAL.IOs->config->reset(Pins.IREF_R3);

};

static uint8_t reset()
{

    HAL.IOs->config->toOutput(Pins.nSLEEP);
    HAL.IOs->config->setHigh(Pins.nSLEEP);
    wait(50);
    HAL.IOs->config->setLow(Pins.nSLEEP);
    noRegResetnSLEEP = true;
    nSLEEPTick = systick_getTick();
    int32_t value = 0;

    for(uint8_t motor = 0; motor < TMC5271_MOTORS; motor++){
        readRegister(DEFAULT_ICID, TMC5271_VACTUAL, &value);
        if(value != 0)
            return 0;
    }

    if(TMC5271.config->state != CONFIG_READY)
        return false;

    TMC5271.config->state        = CONFIG_RESET;
    TMC5271.config->configIndex  = 0;

    return true;
}

static uint8_t restore()
{
	return reset();
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state ==  DRIVER_DISABLE){
		HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
		field_write(&TMC5271, TMC5271_DRV_ENN_FIELD, 1);
	}
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE)){
		HAL.IOs->config->setLow(Pins.DRV_ENN_CFG6);
		field_write(&TMC5271, TMC5271_DRV_ENN_FIELD, 0);
	}
}

static void init_comm(TMC5271BusType mode)
{
	TMC5271_UARTChannel = HAL.UART;
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
		TMC5271_UARTChannel = HAL.UART;
		TMC5271_UARTChannel->pinout = UART_PINS_2;
		TMC5271_UARTChannel->rxtx.init();
		break;
	case IC_BUS_SPI:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC5271_UARTChannel->rxtx.deInit();
		TMC5271_SPIChannel = &HAL.SPI->ch1;
		TMC5271_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		break;
	case IC_BUS_WLAN: // unused
	default:

		HAL.IOs->config->reset(Pins.SCK);
		HAL.IOs->config->reset(Pins.SDI);
		HAL.IOs->config->reset(Pins.SDO);
		HAL.IOs->config->reset(Pins.CS);

		SPI.init();
		HAL.IOs->config->setLow(Pins.UART_MODE);
		TMC5271_UARTChannel->rxtx.deInit();
		TMC5271_SPIChannel = &HAL.SPI->ch1;
		TMC5271_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;
		activeBus = IC_BUS_SPI;
		break;
	}
}

void TMC5271_init(void)
{
    Pins.DRV_ENN_CFG6    = &HAL.IOs->pins->DIO0; //Pin8
    Pins.ENCN_DCO        = &HAL.IOs->pins->DIO1; //Pin9
    Pins.ENCA_DCIN_CFG5  = &HAL.IOs->pins->DIO2; //Pin10
    Pins.ENCB_DCEN_CFG4  = &HAL.IOs->pins->DIO3; //Pin11
    Pins.REFL_UC         = &HAL.IOs->pins->DIO6; //Pin17
    Pins.REFR_UC         = &HAL.IOs->pins->DIO7; //Pin18
    Pins.nSLEEP          = &HAL.IOs->pins->DIO8; //Pin19
    Pins.UART_MODE       = &HAL.IOs->pins->DIO9;//Pin20
    Pins.SWP_DIAG1       = &HAL.IOs->pins->DIO15; //Pin37
    Pins.SWN_DIAG0       = &HAL.IOs->pins->DIO16; //Pin38
    Pins.IREF_R2         = &HAL.IOs->pins->DIO13; //Pin35
    Pins.IREF_R3         = &HAL.IOs->pins->DIO14; //Pin36
    Pins.SCK             = &HAL.IOs->pins->SPI1_SCK; //Pin31
    Pins.SDI             = &HAL.IOs->pins->SPI1_SDI; //Pin32
    Pins.SDO             = &HAL.IOs->pins->SPI1_SDO; //Pin33
    Pins.CS              = &HAL.IOs->pins->SPI1_CSN; //Pin33


    HAL.IOs->config->toOutput(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->toOutput(Pins.UART_MODE);
    HAL.IOs->config->toOutput(Pins.IREF_R2);
    HAL.IOs->config->toOutput(Pins.IREF_R3);
    HAL.IOs->config->toOutput(Pins.nSLEEP);

    HAL.IOs->config->setLow(Pins.nSLEEP);
    HAL.IOs->config->setHigh(Pins.DRV_ENN_CFG6);
    HAL.IOs->config->setLow(Pins.UART_MODE);
    HAL.IOs->config->setLow(Pins.IREF_R2);
    HAL.IOs->config->setLow(Pins.IREF_R2);

    HAL.IOs->config->toInput(Pins.ENCN_DCO);
    HAL.IOs->config->toInput(Pins.ENCB_DCEN_CFG4);
    HAL.IOs->config->toInput(Pins.ENCA_DCIN_CFG5);

    noRegResetnSLEEP = true;
    nSLEEPTick = systick_getTick();


    HAL.IOs->config->toInput(Pins.REFL_UC);
    HAL.IOs->config->toInput(Pins.REFR_UC);

    // Disable CLK output -> use internal 12 MHz clock
    //  Switchable via user function

    init_comm(activeBus);

    Evalboards.ch1.config->reset        = reset;
    Evalboards.ch1.config->restore      = restore;
    Evalboards.ch1.config->state        = CONFIG_RESET;

    TMC5271.velocity = 0;
    TMC5271.oldX = 0;

    TMC5271.config               = Evalboards.ch1.config;
    TMC5271.config->callback     = NULL;
    TMC5271.config->channel      = 0;
    TMC5271.config->configIndex  = 0;
    TMC5271.config->state        = CONFIG_READY;

    for(uint8_t motor = 0; motor < TMC5271_MOTORS; motor++)
    {
        vmax_position[motor] = 0;
    }

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
    Evalboards.ch1.numberOfMotors       = TMC5271_MOTORS;
    Evalboards.ch1.VMMin                = VM_MIN;
    Evalboards.ch1.VMMax                = VM_MAX;
    Evalboards.ch1.deInit               = deInit;

    enableDriver(DRIVER_USE_GLOBAL_ENABLE);


};
