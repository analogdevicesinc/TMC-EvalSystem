/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "TMCL.h"

#include "ProjectConfig.h"
#include "GitInfo.h"

#include "BoardAssignment.h"
#include "hal/derivative.h"
#include "IdDetection.h"
#include "VitalSignsMonitor.h"
#include "tmc/StepDir.h"
#include "EEPROM.h"
#include "RAMDebug.h"
#include "hal/Timer.h"

// Ensure that any non-local release has corresponding
// version control metadata available and doesn't have
// any local outstanding changes.
#if GETINFO_RELEASE_TYPE != GETINFO_FW_RELEASE_TYPE_LOCAL
#if GIT_VERSION_INFO == 0xFFFFFFFF
#error "Git info is required for nonlocal builds!"
#endif
#if GIT_DIRTY_FLAG != 0
#error "Dirty builds are not allowed for nonlocal builds!"
#endif
#endif

// these addresses are fixed
#define SERIAL_MODULE_ADDRESS  1
#define SERIAL_HOST_ADDRESS    2

// todo CHECK 2: these are unused - delete? (LH) #11
// tmcl interpreter states
#define TM_IDLE      0
#define TM_RUN       1
#define TM_STEP      2
#define TM_RESET     3 // unused
#define TM_DOWNLOAD  4
#define TM_DEBUG     5 // wie TM_IDLE, es wird jedoch der Akku nicht modifiziert bei GAP etc.

// todo CHECK 2: these are unused - delete? (LH) #12
#define TCS_IDLE           0
#define TCS_CAN7           1
#define TCS_CAN8           2
#define TCS_UART           3
#define TCS_UART_ERROR     4
#define TCS_UART_II        5
#define TCS_UART_II_ERROR  6
#define TCS_USB            7
#define TCS_USB_ERROR      8
#define TCS_MEM            9


// Command type variants
#define MVP_ABS  0
#define MVP_REL  1
#define MVP_PRF  2

// GetVersion() Format types
#define VERSION_FORMAT_ASCII      0
#define VERSION_FORMAT_BINARY     1
#define VERSION_BOOTLOADER        2 // todo CHECK 2: implemented this way in IDE - probably means getting the bootloader version. Not implemented in firmware (LH)
#define VERSION_SIGNATURE         3 // todo CHECK 2: implemented under "Signature" in IDE. Not sure what to return for that. Not implemented in firmware (LH)
#define VERSION_BOARD_DETECT_SRC  4 // todo CHECK 2: This doesn't really fit under GetVersion, but its implemented there in the IDE - change or leave this way? (LH)
#define VERSION_BUILD             5
#define NUMBER_OF_MOTORS          6 //is returning the amount of motors that could be connected to the EVAL - board

//Statuscodes
#define REPLY_OK                     100
#define REPLY_CMD_LOADED             101
#define REPLY_CHKERR                 1
#define REPLY_INVALID_CMD            2
#define REPLY_INVALID_TYPE           3
#define REPLY_INVALID_VALUE          4
#define REPLY_EEPROM_LOCKED          5
#define REPLY_CMD_NOT_AVAILABLE      6
#define REPLY_CMD_LOAD_ERROR         7
#define REPLY_WRITE_PROTECTED        8
#define REPLY_MAX_EXCEEDED           9
#define REPLY_DOWNLOAD_NOT_POSSIBLE  10
#define REPLY_CHIP_READ_FAILED       11
#define REPLY_DELAYED                128
#define REPLY_ACTIVE_COMM            129

// TMCL communication status
#define TMCL_RX_ERROR_NONE      0
#define TMCL_RX_ERROR_NODATA    1
#define TMCL_RX_ERROR_CHECKSUM  2

extern const char VersionString[8];
extern void enterBootloader();

void ExecuteActualCommand();
uint8_t setTMCLStatus(uint8_t evalError);
void rx(RXTXTypeDef *RXTX);
void tx(RXTXTypeDef *RXTX);

static uint16_t getExtendedAddress(TMCLCommandTypeDef *tmclCommand)
{
    return (((uint16_t) tmclCommand->Motor >> 4) << 8) | tmclCommand->Type;
}

// Helper functions - used to prevent ExecuteActualCommand() from getting too big.
// No parameters or return value are used.
static void readIdEeprom(void);
static void writeIdEeprom(void);
static void SetGlobalParameter(void);
static void GetGlobalParameter(void);
static void boardAssignment(void);
static void boardsErrors(void);
static void boardsReset(void);
static void boardsMeasuredSpeed(void);
static void setDriversEnable(void);
static void checkIDs(void);
static bool checkBoardTypes();
static void SoftwareReset(void);
static void GetVersion(void);
static void GetInput(void);
static void SetOutput(void);
static void HandleWlanCommand(void);
static int handleRamDebug(uint8_t type, uint8_t motor, uint32_t *data);
static void handleGetInfo(void);
static void handleOTP(void);

TMCLCommandTypeDef ActualCommand;
TMCLReplyTypeDef ActualReply;
RXTXTypeDef interfaces[4];
uint32_t numberOfInterfaces;
uint32_t resetRequest = 0;

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall) || defined(LandungsbrueckeV3)
    extern struct BootloaderConfig BLConfig;
#endif

// Sets TMCL status from Evalboard error. Returns the parameter given to allow for compact error handling
uint8_t setTMCLStatus(uint8_t evalError)
{
    if(evalError == TMC_ERROR_NONE)          ActualReply.Status = REPLY_OK;
    else if(evalError & TMC_ERROR_FUNCTION)  ActualReply.Status = REPLY_INVALID_CMD;
    else if(evalError & TMC_ERROR_TYPE)      ActualReply.Status = REPLY_INVALID_TYPE;
    else if(evalError & TMC_ERROR_MOTOR)     ActualReply.Status = REPLY_INVALID_TYPE; // todo CHECK ADD 2: Different errors for Evalboard type/motor errors? (LH) #1
    else if(evalError & TMC_ERROR_VALUE)     ActualReply.Status = REPLY_INVALID_VALUE;
    else if(evalError & TMC_ERROR_NOT_DONE)  ActualReply.Status = REPLY_DELAYED;
    else if(evalError & TMC_ERROR_CHIP)      ActualReply.Status = REPLY_EEPROM_LOCKED;
    return evalError;
}

void ExecuteActualCommand()
{
    ActualReply.ModuleId = ActualCommand.ModuleId;
    ActualReply.Opcode = ActualCommand.Opcode;
    ActualReply.Status = REPLY_OK;
    ActualReply.Value.Int32 = ActualCommand.Value.Int32;

    if(ActualCommand.Error == TMCL_RX_ERROR_CHECKSUM)
    {
        ActualReply.Value.Int32  = 0;
        ActualReply.Status       = REPLY_CHKERR;
        return;
    }

    if(Evalboards.ch1.fwdTmclCommand)
    {
        if (Evalboards.ch1.fwdTmclCommand(&ActualCommand, &ActualReply))
        {
            return;
        }
    }

    if (ActualCommand.ModuleId != SERIAL_MODULE_ADDRESS)
    {
        // Datagram is not addressed to us, ignore it
        ActualCommand.Error = TMCL_RX_ERROR_NODATA;
        return;
    }

    switch(ActualCommand.Opcode)
    {
    case TMCL_ROR:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.right(ActualCommand.Motor, ActualCommand.Value.Int32)) & TMC_ERROR_FUNCTION)
        {
            setTMCLStatus(Evalboards.ch2.right(ActualCommand.Motor, ActualCommand.Value.Int32));
        }
        break;
    case TMCL_ROL:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.left(ActualCommand.Motor, ActualCommand.Value.Int32)) & TMC_ERROR_FUNCTION)
        {
            setTMCLStatus(Evalboards.ch2.left(ActualCommand.Motor, ActualCommand.Value.Int32));
        }
        break;
    case TMCL_MST:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.stop(ActualCommand.Motor)) & TMC_ERROR_FUNCTION)
        {
            setTMCLStatus(Evalboards.ch2.stop(ActualCommand.Motor));
        }
        break;
    case TMCL_MVP:
        // if function doesn't exist for ch1 try ch2
        switch(ActualCommand.Type)
        {
        case MVP_ABS: // move absolute
            if(setTMCLStatus(Evalboards.ch1.moveTo(ActualCommand.Motor, ActualCommand.Value.Int32)) & TMC_ERROR_FUNCTION)
            {
                setTMCLStatus(Evalboards.ch2.moveTo(ActualCommand.Motor, ActualCommand.Value.Int32));
            }
            break;
        case MVP_REL: // move relative
            if(setTMCLStatus(Evalboards.ch1.moveBy(ActualCommand.Motor, &ActualCommand.Value.Int32)) & TMC_ERROR_FUNCTION)
            {
                setTMCLStatus(Evalboards.ch2.moveBy(ActualCommand.Motor, &ActualCommand.Value.Int32));
            }
            ActualReply.Value.Int32 = ActualCommand.Value.Int32;
            break;
        case MVP_PRF:
            if(setTMCLStatus(Evalboards.ch1.moveProfile(ActualCommand.Motor, ActualCommand.Value.Int32)) & TMC_ERROR_FUNCTION)
            {
                setTMCLStatus(Evalboards.ch2.moveProfile(ActualCommand.Motor, ActualCommand.Value.Int32));
            }
            break;
        default:
            ActualReply.Status = REPLY_INVALID_TYPE;
            break;
        }
        break;
    case TMCL_SAP:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.SAP(ActualCommand.Type, ActualCommand.Motor, ActualCommand.Value.Int32)) & (TMC_ERROR_TYPE | TMC_ERROR_FUNCTION))
        {
            setTMCLStatus(Evalboards.ch2.SAP(ActualCommand.Type, ActualCommand.Motor, ActualCommand.Value.Int32));
        }
        break;
    case TMCL_GAP:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.GAP(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32)) & (TMC_ERROR_TYPE | TMC_ERROR_FUNCTION))
        {
            setTMCLStatus(Evalboards.ch2.GAP(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32));
        }
        break;
    case TMCL_STAP:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.STAP(ActualCommand.Type, ActualCommand.Motor, ActualCommand.Value.Int32)) & (TMC_ERROR_TYPE | TMC_ERROR_FUNCTION))
        {
            setTMCLStatus(Evalboards.ch2.STAP(ActualCommand.Type, ActualCommand.Motor, ActualCommand.Value.Int32));
        }
        break;
    case TMCL_SGP:
        SetGlobalParameter();
        break;
    case TMCL_GGP:
        GetGlobalParameter();
        break;
    case TMCL_SIO:
        SetOutput();
        break;
    case TMCL_GIO:
        GetInput();
        break;
    case TMCL_UF0:
        setDriversEnable();
        break;
    case TMCL_UF1:
        readIdEeprom();
        break;
    case TMCL_UF2:
        writeIdEeprom();
        break;
    case TMCL_UF4:
        // if function doesn't exist for ch1 try ch2
        if(setTMCLStatus(Evalboards.ch1.getMeasuredSpeed(ActualCommand.Motor, &ActualReply.Value.Int32)) & TMC_ERROR_FUNCTION)
        {
            setTMCLStatus(Evalboards.ch2.getMeasuredSpeed(ActualCommand.Motor, &ActualReply.Value.Int32));
        }
        break;
    case TMCL_UF5:
        // if function doesn't exist for ch1 try ch2 // todo CHECK REM 2: We have TMCL_writeRegisterChannel_1, we dont need this. Make sure it isnt used in IDE (LH) #1
        Evalboards.ch1.writeRegister(ActualCommand.Motor & 0x0F, getExtendedAddress(&ActualCommand), ActualCommand.Value.Int32);
        break;
    case TMCL_UF6:
        // if function doesn't exist for ch1 try ch2 // todo CHECK REM 2: We have TMCL_readRegisterChannel_1, we dont need this. Make sure it isnt used in IDE (LH) #2
        Evalboards.ch1.readRegister(ActualCommand.Motor & 0x0F, getExtendedAddress(&ActualCommand), &ActualReply.Value.Int32);
        break;
    case TMCL_UF8:
        // user function for reading Motor0_XActual and Motor1_XActual
        Evalboards.ch1.userFunction(ActualCommand.Type, 0, &ActualCommand.Value.Int32);
        int32_t m0XActual = ActualCommand.Value.Int32;
        Evalboards.ch1.userFunction(ActualCommand.Type, 1, &ActualCommand.Value.Int32);
        int32_t m1XActual = ActualCommand.Value.Int32;
        ActualReply.Value.Byte[0]= m1XActual & 0xFF;
        ActualReply.Value.Byte[1]= (m1XActual & 0xFF00)>>8;
        ActualReply.Value.Byte[2]= (m1XActual & 0xFF0000)>>16;
        ActualReply.Value.Byte[3]= m0XActual & 0xFF;
        ActualReply.Opcode= (m0XActual & 0xFF00)>>8;
        ActualReply.Status= (m0XActual & 0xFF0000)>>16;
        break;
    case TMCL_GetVersion:
        GetVersion();
        break;
    case TMCL_GetIds:
        boardAssignment();
        break;
    case TMCL_UF_CH1:
        // user function for motionController board
        setTMCLStatus(Evalboards.ch1.userFunction(ActualCommand.Type, ActualCommand.Motor, &ActualCommand.Value.Int32));
        ActualReply.Value.Int32 = ActualCommand.Value.Int32;
        break;
    case TMCL_UF_CH2:
        // user function for driver board
        setTMCLStatus(Evalboards.ch2.userFunction(ActualCommand.Type, ActualCommand.Motor, &ActualCommand.Value.Int32));
        ActualReply.Value.Int32 = ActualCommand.Value.Int32;
        break;
    case TMCL_writeRegisterChannel_1:
            Evalboards.ch1.writeRegister(ActualCommand.Motor & 0x0F, getExtendedAddress(&ActualCommand), ActualCommand.Value.Int32);
        break;
    case TMCL_writeRegisterChannel_2:
        Evalboards.ch2.writeRegister(ActualCommand.Motor & 0x0F, getExtendedAddress(&ActualCommand), ActualCommand.Value.Int32);
        break;
    case TMCL_readRegisterChannel_1:
        // Do not allow reads during brownout to prevent garbage data being used
        // in read-modify-write operations. Bypass this safety with motor = 255
        if ((VitalSignsMonitor.brownOut & VSM_ERRORS_BROWNOUT_CH1) && ActualCommand.Motor != 255)
        {
            ActualReply.Status = REPLY_CHIP_READ_FAILED;
        }
        else
        {
                Evalboards.ch1.readRegister(ActualCommand.Motor & 0x0F, getExtendedAddress(&ActualCommand), &ActualReply.Value.Int32);
        }
        break;
    case TMCL_readRegisterChannel_2:
        // Do not allow reads during brownout to prevent garbage data being used
        // in read-modify-write operations. Bypass this safety with motor = 255
        if ((VitalSignsMonitor.brownOut & VSM_ERRORS_BROWNOUT_CH2) && ActualCommand.Motor != 255)
            ActualReply.Status = REPLY_CHIP_READ_FAILED;
        else
        {

            Evalboards.ch2.readRegister(ActualCommand.Motor & 0x0F, getExtendedAddress(&ActualCommand), &ActualReply.Value.Int32);
        }
        break;
    case TMCL_BoardMeasuredSpeed:
        // measured speed from motionController board or driver board depending on type
        boardsMeasuredSpeed();
        break;
    case TMCL_BoardError:
        // errors of motionController board or driver board depending on type
        boardsErrors();
        break;
    case TMCL_BoardReset:
        // reset of motionController board or driver board depending on type
        boardsReset();
        break;
    case TMCL_GetInfo:
        handleGetInfo();
        break;
    case TMCL_WLAN:
        HandleWlanCommand();
        break;
    case TMCL_RamDebug:
        ActualReply.Status = handleRamDebug(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.UInt32);
        break;
    case TMCL_OTP:
        handleOTP();
        break;
    case TMCL_MIN:
        if(setTMCLStatus(Evalboards.ch1.getMin(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32)) & (TMC_ERROR_TYPE | TMC_ERROR_FUNCTION))
        {
            setTMCLStatus(Evalboards.ch2.getMin(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32));
        }
        break;
    case TMCL_MAX:
        if(setTMCLStatus(Evalboards.ch1.getMax(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32)) & (TMC_ERROR_TYPE | TMC_ERROR_FUNCTION))
        {
            setTMCLStatus(Evalboards.ch2.getMax(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32));
        }
        break;
    case TMCL_Boot:
        if(ActualCommand.Type           != 0x81)  break;
        if(ActualCommand.Motor          != 0x92)  break;
        if(ActualCommand.Value.Byte[3]  != 0xA3)  break;
        if(ActualCommand.Value.Byte[2]  != 0xB4)  break;
        if(ActualCommand.Value.Byte[1]  != 0xC5)  break;
        if(ActualCommand.Value.Byte[0]  != 0xD6)  break;
        enterBootloader();
        break;
    case TMCL_SoftwareReset:
        SoftwareReset();
        break;
    default:
        ActualReply.Status = REPLY_INVALID_CMD;
        break;
    }
}

void tmcl_init()
{
    ActualCommand.Error  = TMCL_RX_ERROR_NODATA;
    interfaces[0]        = *HAL.USB;
    interfaces[1]        = *HAL.RS232;
    interfaces[2]        = *HAL.WLAN;
    numberOfInterfaces   = 3;
}

void tmcl_process()
{
    static int32_t currentInterface = 0;

    if(ActualCommand.Error != TMCL_RX_ERROR_NODATA)
        tx(&interfaces[currentInterface]);

    if(resetRequest)
        HAL.reset(true);

    ActualReply.IsSpecial = 0;

    for(uint32_t i = 0; i < numberOfInterfaces; i++)
    {
        rx(&interfaces[i]);
        if(ActualCommand.Error != TMCL_RX_ERROR_NODATA)
        {
            currentInterface = i;
            ExecuteActualCommand();
            return;
        }
    }
}

void tx(RXTXTypeDef *RXTX)
{
    uint8_t checkSum = 0;

    uint8_t reply[9];

    if(ActualReply.IsSpecial)
    {
        for(uint8_t i = 0; i < 9; i++)
            reply[i] = ActualReply.Special[i];
    }
    else
    {
        checkSum += SERIAL_HOST_ADDRESS;
        checkSum += ActualReply.ModuleId;
        checkSum += ActualReply.Status;
        checkSum += ActualReply.Opcode;
        checkSum += ActualReply.Value.Byte[3];
        checkSum += ActualReply.Value.Byte[2];
        checkSum += ActualReply.Value.Byte[1];
        checkSum += ActualReply.Value.Byte[0];

        reply[0] = SERIAL_HOST_ADDRESS;
        reply[1] = ActualReply.ModuleId;
        reply[2] = ActualReply.Status;
        reply[3] = ActualReply.Opcode;
        reply[4] = ActualReply.Value.Byte[3];
        reply[5] = ActualReply.Value.Byte[2];
        reply[6] = ActualReply.Value.Byte[1];
        reply[7] = ActualReply.Value.Byte[0];
        reply[8] = checkSum;
    }

    RXTX->txN(reply, 9);
}

void rx(RXTXTypeDef *RXTX)
{
    uint8_t checkSum = 0;
    uint8_t cmd[9];

    if(!RXTX->rxN(cmd, 9))
    {
        ActualCommand.Error = TMCL_RX_ERROR_NODATA;
        return;
    }

    // todo ADD CHECK 2: check for SERIAL_MODULE_ADDRESS byte ( cmd[0] ) ? (LH)

    for(uint8_t i = 0; i < 8; i++)
        checkSum += cmd[i];

    if(checkSum != cmd[8])
    {
        ActualCommand.Error	= TMCL_RX_ERROR_CHECKSUM;
        return;
    }

    ActualCommand.ModuleId       = cmd[0];
    ActualCommand.Opcode         = cmd[1];
    ActualCommand.Type           = cmd[2];
    ActualCommand.Motor          = cmd[3];
    ActualCommand.Value.Byte[3]  = cmd[4];
    ActualCommand.Value.Byte[2]  = cmd[5];
    ActualCommand.Value.Byte[1]  = cmd[6];
    ActualCommand.Value.Byte[0]  = cmd[7];
    ActualCommand.Error          = TMCL_RX_ERROR_NONE;
}


/*
 * Reads four bytes from the eeprom.
 *
 * @param channel Id of SPI channel to be used with 1 = SPI.ch1 and 2 = SPI.ch2
 * @param address Address the byte should be read from.
 *
 * @return The bytes read with byte0 = lowest byte or 0 if reading went unsuccessful
 */
static void readIdEeprom(void)
{
    SPIChannelTypeDef *spi;
    if(ActualCommand.Type == 1)
        spi = &SPI.ch1;
    else if(ActualCommand.Type == 2)
        spi = &SPI.ch2;
    else
    {
        ActualReply.Status = REPLY_INVALID_TYPE;
        return;
    }

    uint8_t array[4];
    eeprom_read_array(spi, ActualCommand.Value.Int32, array, 4);
    ActualReply.Value.Int32 = array[3] << 24 | array[2] << 16 | array[1] << 8 | array[0];
}

/*
 * Writes one byte into the eeprom for id detection.
 *
 * @param channel Id of SPI channel to be used with 1 = SPI.ch1 and 2 = SPI.ch2
 * @param address Address the byte should be written to.
 * @param bytes Pointer to byte array that are to be written. The first byte is always written
 * 				following bytes are written as long as they are not null.
 *
 * @return false if everything went successful
 * 		   1 if selected channel is not available
 * 		   the status bits of the eeprom if eeprom is not ready
 */
static void writeIdEeprom(void)
{
    SPIChannelTypeDef *spi;
    if(ActualCommand.Type == 1)
        spi = &SPI.ch1;
    else if(ActualCommand.Type == 2)
        spi = &SPI.ch2;
    else
    {
        ActualReply.Status = REPLY_INVALID_TYPE;
        return;
    }

    uint8_t out = eeprom_check(spi);
    // ignore when check did not find magic number, quit on other errors
    if(out != ID_CHECKERROR_MAGICNUMBER && out != 0)
    {
        ActualReply.Status = REPLY_EEPROM_LOCKED; // todo CHECK 2: Not sure which error to send here, this one sounded ok (LH)
        return;
    }

    eeprom_write_byte(spi, ActualCommand.Value.Int32, ActualCommand.Motor);

    return;
}

static void SetGlobalParameter()
{
    switch(ActualCommand.Type)
    {
    case 1:
        VitalSignsMonitor.errorMask = ActualCommand.Value.Int32;
        break;
    case 2:
        setDriversEnable();
        break;
    case 3:
        switch(ActualCommand.Value.Int32)
        {
        case 0: // normal operation
            VitalSignsMonitor.debugMode = 0;
            break;
        case 1: // FREE ERROR LED
            VitalSignsMonitor.debugMode = 1;
            HAL.LEDs->error.off();
            break;
        default:
            ActualReply.Status = REPLY_INVALID_TYPE;
            break;
        }
        break;
        case 6:
            if(Evalboards.ch1.onPinChange(HAL.IOs->pins->pins[ActualCommand.Motor], ActualCommand.Value.UInt32)
                    && Evalboards.ch2.onPinChange(HAL.IOs->pins->pins[ActualCommand.Motor], ActualCommand.Value.UInt32))
                HAL.IOs->config->setToState(HAL.IOs->pins->pins[ActualCommand.Motor], ActualCommand.Value.UInt32);
            break;
        case 7:
            ActualReply.Value.UInt32 = spi_setFrequency(&HAL.SPI->ch1, ActualCommand.Value.UInt32);
            break;
        case 8:
            ActualReply.Value.UInt32 = spi_setFrequency(&HAL.SPI->ch2, ActualCommand.Value.UInt32);
            break;
        case 9:
            if (!spi_setMode(&HAL.SPI->ch1, ActualCommand.Value.UInt32))
            {
                ActualReply.Status = REPLY_INVALID_VALUE;
                break;
            }
            break;
        case 10:
            if (!spi_setMode(&HAL.SPI->ch2, ActualCommand.Value.UInt32))
            {
                ActualReply.Status = REPLY_INVALID_VALUE;
                break;
            }
            break;
        default:
            ActualReply.Status = REPLY_INVALID_TYPE;
            break;
    }
}

static void GetGlobalParameter()
{
    switch(ActualCommand.Type)
    {
    case 1:
        ActualReply.Value.Int32 = VitalSignsMonitor.errors;
        break;
    case 2:
        ActualReply.Value.Int32 = (Evalboards.driverEnable == DRIVER_ENABLE)? 1:0;
        break;
    case 3:
        ActualReply.Value.Int32 = VitalSignsMonitor.debugMode;
        break;
    case 4:
    {
        IdAssignmentTypeDef ids;
        ids.ch1.id = Evalboards.ch1.id;
        ids.ch2.id = Evalboards.ch2.id;
        ActualReply.Value.Int32 = Board_supported(&ids);
    }
    break;
    case 5: // Get hardware ID
        ActualReply.Value.Int32 = hwid;
        break;
    case 6:
        ActualReply.Value.UInt32 = HAL.IOs->config->getState(HAL.IOs->pins->pins[ActualCommand.Motor]);
        break;
    case 7:
        ActualReply.Value.UInt32 = spi_getFrequency(&HAL.SPI->ch1);
        break;
    case 8:
        ActualReply.Value.UInt32 = spi_getFrequency(&HAL.SPI->ch2);
        break;
    case 9:
        ActualReply.Value.UInt32 = spi_getMode(&HAL.SPI->ch1);
        break;
    case 10:
        ActualReply.Value.UInt32 = spi_getMode(&HAL.SPI->ch2);
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static void boardAssignment(void)
{
    uint8_t testOnly = 0;

    IdAssignmentTypeDef ids;
    ids.ch1.id     = (ActualCommand.Value.Int32 >> 0)   & 0xFF;
    ids.ch1.state  = (ActualCommand.Value.Int32 >> 8)   & 0xFF;
    ids.ch2.id     = (ActualCommand.Value.Int32 >> 16)  & 0xFF;
    ids.ch2.state  = (ActualCommand.Value.Int32 >> 24)  & 0xFF;

    switch(ActualCommand.Type)
    {
    case 0:  // auto detect and assign
        checkIDs();
        return;
        break;
    case 1:  // id for channel 2 not changed, reset maybe
        ids.ch2.id     = Evalboards.ch2.id;
        ids.ch2.state  = ID_STATE_WAIT_LOW;
        break;
    case 2:  // id for channel 1 not changed, reset maybe
        ids.ch2.id     = (ActualCommand.Value.Int32 >> 0)  & 0xFF;
        ids.ch2.state  = (ActualCommand.Value.Int32 >> 8)  & 0xFF;
        ids.ch1.id     = Evalboards.ch1.id;
        ids.ch1.state  = ID_STATE_WAIT_LOW;
        break;
    case 3:  // id for both channels
        break;
    case 4:  // test if ids are in firmware
        testOnly = 1;
        if(ActualReply.Value.Int32 == 0)
        {
            ids.ch1.id = Evalboards.ch1.id;
            ids.ch2.id = Evalboards.ch2.id;
        }
        break;
    case 5:  // Re-Check board types - but do not [de]init any boards
        checkBoardTypes();
        return;
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        return;
        break;
    }

    IdAssignmentTypeDef ids_buff;
    ids_buff.ch1.id     = ids.ch1.id;
    ids_buff.ch1.state  = ID_STATE_DONE;
    ids_buff.ch2.id     = ids.ch2.id;
    ids_buff.ch2.state  = ID_STATE_DONE;

    if(!testOnly)
        ActualReply.Value.Int32 = Board_assign(&ids_buff);
    else
        ActualReply.Value.Int32 = Board_supported(&ids_buff);
}

static void boardsErrors(void)
{
    switch(ActualCommand.Type)
    {
    case 0:
        ActualReply.Value.Int32 = Evalboards.ch1.errors;
        break;
    case 1:
        ActualReply.Value.Int32 = Evalboards.ch2.errors;
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static void boardsReset(void)
{
    switch(ActualCommand.Type)
    {
    case 0:
        if(!Evalboards.ch1.config->reset())
            ActualReply.Status = REPLY_WRITE_PROTECTED;
        break;
    case 1:
        if(!Evalboards.ch2.config->reset())
            ActualReply.Status = REPLY_WRITE_PROTECTED;
        break;
    case 2:
        if(!Evalboards.ch1.config->reset())
            ActualReply.Status = REPLY_WRITE_PROTECTED;
        if(!Evalboards.ch2.config->reset())
            ActualReply.Status = REPLY_WRITE_PROTECTED;
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static void boardsMeasuredSpeed(void)
{
    switch(ActualCommand.Type)
    {
    case 0:
        ActualReply.Status = Evalboards.ch1.getMeasuredSpeed(ActualCommand.Motor, &ActualReply.Value.Int32);
        break;
    case 1:
        ActualReply.Status = Evalboards.ch2.getMeasuredSpeed(ActualCommand.Motor, &ActualReply.Value.Int32);
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static void setDriversEnable()
{
    vitalsignsmonitor_clearOvervoltageErrors();

    Evalboards.driverEnable = (ActualCommand.Value.Int32) ? DRIVER_ENABLE : DRIVER_DISABLE;
    Evalboards.ch1.enableDriver(DRIVER_USE_GLOBAL_ENABLE);
    Evalboards.ch2.enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}

static void checkIDs(void)
{
    IdAssignmentTypeDef ids = { 0 };

    // Backwards compatibility:
    // For now we do this *before* scanning the bus since TMCL-IDE <= 4.6.0 will
    // use this command (TMCL_GetIDs type 0) to switch the ID from TMC9660
    // bootloader to param/reg. Later this will change to always do the generic
    // ID detection first followed by identifying the board type here and using
    // a different command (TMCL_GetIDs type 5) - but that change will break the
    // old TMCL-IDE mechanism.
    if (checkBoardTypes())
        return;

    Evalboards.ch1.deInit();
    Evalboards.ch2.deInit();
    Evalboards.ch1.id = 0;
    Evalboards.ch2.id = 0;

    // Try detecting the IDs
    if (!IDDetection_detect(&ids))
    {
        // Monoflop detection not yet finished
        ActualReply.Status = REPLY_DELAYED;
        return;
    }

    // ID detection completed -> Assign the board
    Board_assign(&ids);
    ActualReply.Value.UInt32 = (uint32_t)(
            (ids.ch1.id)
            | (ids.ch1.state << 8)
            | (ids.ch2.id    << 16)
            | (ids.ch2.state << 24)
    );
}

static bool checkBoardTypes()
{
    IdAssignmentTypeDef ids = { 0 };

    switch(Evalboards.ch1.id)
    {
    case ID_TMC9660_3PH_BL_EVAL:
    case ID_TMC9660_STEPPER_BL_EVAL:
    case ID_TMC9660_3PH_PARAM_EVAL:
    case ID_TMC9660_STEPPER_PARAM_EVAL:
    case ID_TMC9660_3PH_REG_EVAL:
    case ID_TMC9660_STEPPER_REG_EVAL:
    {
        int32_t val = 0;
        bool isStepper = Evalboards.ch1.id >= ID_TMC9660_STEPPER_BL_EVAL;

        Evalboards.ch1.userFunction(0,0,&val);
        if(val == TM01)
        {
            // Bootloader responded -> Chip is in bootloader mode
            ids.ch1.id = (isStepper) ? ID_TMC9660_STEPPER_BL_EVAL : ID_TMC9660_3PH_BL_EVAL;
        }
        else
        {
            val = 0;
            Evalboards.ch1.userFunction(2,0,&val);
            if (val == 51)
            {
                // Chip responded with parameter mode ID -> Chip is in parameter mode
                ids.ch1.id = (isStepper) ? ID_TMC9660_STEPPER_PARAM_EVAL : ID_TMC9660_3PH_PARAM_EVAL;
            }
            else if (val == 17)
            {
                ids.ch1.id = (isStepper) ? ID_TMC9660_STEPPER_REG_EVAL : ID_TMC9660_3PH_REG_EVAL;
            }
            else
            {
                // Unknown
                return false;
            }
        }
        Evalboards.ch1.errors &= ~0x07;

        Evalboards.ch1.id = ids.ch1.id;
        ids.ch1.state = ID_STATE_DONE;
        ActualReply.Value.Int32 = (uint32_t)(
                (Evalboards.ch1.id)
                | (ids.ch1.state << 8)
                | (ids.ch2.id    << 16)
                | (ids.ch2.state << 24)
        );
        return true;
        break;
    }
    default:
        return false;
    }
}

static void SoftwareReset(void)
{
    if(ActualCommand.Value.Int32 == 1234)
        resetRequest = true;
}

static void GetVersion(void)
{
    if(ActualCommand.Type == VERSION_FORMAT_ASCII)
    {
        ActualReply.IsSpecial   = 1;
        ActualReply.Special[0]  = SERIAL_HOST_ADDRESS;

        for(uint8_t i = 0; i < 8; i++)
            ActualReply.Special[i+1] = VersionString[i];
    }
    else if(ActualCommand.Type == VERSION_FORMAT_BINARY)
    {
        // module version high
        ActualReply.Value.Byte[3] = MODULE_ID / 100;

        // module version low
        ActualReply.Value.Byte[2] = MODULE_ID % 100;

        // fw version high
        ActualReply.Value.Byte[1] = VERSION_MAJOR;

        // fw version low
        ActualReply.Value.Byte[0] = VERSION_MINOR;
    }
    //how were the boards detected?	// todo CHECK 2: Doesn't fit into GetVersion. Move somewhere else? Or maybe change GetVersion to GetBoardInfo or something (LH)
    else if(ActualCommand.Type == VERSION_BOARD_DETECT_SRC)
    {
        ActualReply.Value.Byte[0] = IdState.ch1.detectedBy;
        ActualReply.Value.Byte[1] = IdState.ch2.detectedBy;
    }
    else if(ActualCommand.Type == VERSION_BUILD) {
        ActualReply.Value.UInt32 = BUILD_VERSION;
    }
    else if(ActualCommand.Type == NUMBER_OF_MOTORS){
        ActualReply.Value.UInt32 = Evalboards.ch1.numberOfMotors + Evalboards.ch2.numberOfMotors;
    }
}

static void handleGetInfo(void)
{
    switch (ActualCommand.Type)
    {
    case 0: // FWModuleID
        ActualReply.Value.Int32 = MODULE_ID;
        break;

    case 1: // FWVersion
        // Major version
        ActualReply.Value.Byte[2] = VERSION_MAJOR;

        // Minor version
        ActualReply.Value.Byte[0] = VERSION_MINOR;
        break;

    case 2: // FWCapability
        ActualReply.Value.Int32 = (GETINFO_FW_CAPABILITY_BITMASK_TMCL); // TMCL only firmware
        break;

    case 3: // FWReleaseType
        ActualReply.Value.Int32 = GETINFO_RELEASE_TYPE;
        break;

    case 20: // APIndexBitWidth
        ActualReply.Value.Int32 = 8;
        break;

    case 21: // RegAddrBitWidth
        ActualReply.Value.UInt32 = 12;
        break;

    case 30: // Git info
        if (GIT_VERSION_INFO == 0xFFFFFFFF)
        {
            // Illegal GIT_VERSION_INFO value -> Git info is disabled
            ActualReply.Status = REPLY_INVALID_TYPE;
            break;
        }

        ActualReply.Value.Int32 = GIT_VERSION_INFO;
        break;

    case 200: // DeviceSpecificArea: Patch version
        ActualReply.Value.UInt32 = VERSION_PATCH;
        break;

    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static void SetOutput(void)
{
    if((Evalboards.ch1.SIO(ActualCommand.Type, ActualCommand.Motor, ActualCommand.Value.Int32) == TMC_ERROR_NONE)
        || (Evalboards.ch2.SIO(ActualCommand.Type, ActualCommand.Motor, ActualCommand.Value.Int32) == TMC_ERROR_NONE))
        return;
}

static void GetInput(void)
{
    if((Evalboards.ch1.GIO(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32) == TMC_ERROR_NONE)
        || (Evalboards.ch2.GIO(ActualCommand.Type, ActualCommand.Motor, &ActualReply.Value.Int32) == TMC_ERROR_NONE))
        return;

    switch(ActualCommand.Type)
    {
    case 0:
        ActualReply.Value.Int32 = *HAL.ADCs->AIN0;
        break;
    case 1:
        ActualReply.Value.Int32 = *HAL.ADCs->AIN1;
        break;
    case 2:
        ActualReply.Value.Int32 = *HAL.ADCs->AIN2;
        break;
    case 3:
        ActualReply.Value.Int32 = *HAL.ADCs->DIO4;
        break;
    case 4:
        ActualReply.Value.Int32 = *HAL.ADCs->DIO5;
        break;
    case 5:
        ActualReply.Value.Int32 = VitalSignsMonitor.VM;
        break;
    case 6:	// Raw VM ADC value, no scaling calculation done // todo QOL 2: Switch this case with case 5? That way we have the raw Values from 0-5, then 6 for scaled VM value. Requires IDE changes (LH)
        ActualReply.Value.Int32 = *HAL.ADCs->VM;
        break;
    case 7:
        ActualReply.Value.Int32 = *HAL.ADCs->AIN_EXT;
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static void HandleWlanCommand(void)
{
    switch(ActualCommand.Type)
    {
    case 0:
        ActualReply.Value.Int32 = handleWLANCommand(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;
    case 1:
        enableWLANCommandMode();
        break;
    case 2:
        ActualReply.Value.Int32 = checkReadyToSend();
        break;
    case 3:
        ActualReply.Value.Int32 = checkCmdModeEnabled();
        break;
    case 4:
        ActualReply.Value.Int32 = getCMDReply();
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}

static int handleRamDebug(uint8_t type, uint8_t motor, uint32_t *data)
{
    switch (type)
    {
    case 0:
        debug_init();
        break;
    case 1:
        debug_setSampleCount(*data);
        break;
    case 2:
        /* Placeholder: Set sampling time reference*/
        if (*data != 0)
            return REPLY_INVALID_VALUE;
        break;
    case 3:
        // RAMDebug expects a divisor value where 1 is original capture frequency,
        // 2 is halved capture frequency etc.
        // The TMCL-IDE sends prescaling values that are one lower than that.
        debug_setPrescaler(*data + 1);
        break;
    case 4:
        if (!debug_setChannel(motor, *data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 5:
        if (!debug_setTriggerChannel(motor, *data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 6:
        debug_setTriggerMaskShift(*data, motor);
        break;
    case 7:
        debug_enableTrigger(motor, *data);
        break;
    case 8:
        *data = debug_getState();
        break;
    case 9:
        if (!debug_getSample(*data, data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 10:
        if (!debug_getInfo(*data, data))
            ActualReply.Status = REPLY_MAX_EXCEEDED;
        break;
    case 11:
        if (!debug_getChannelType(motor, (uint8_t *) data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 12:
        if (!debug_getChannelAddress(motor, data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 13:
        debug_setPretriggerSampleCount(*data);
        break;
    case 14:
        *data = debug_getPretriggerSampleCount();
        break;
    case 15:
        if(Timer.initialized) {
            Timer.setFrequency(TIMER_CHANNEL_2, *data);
            *data = Timer.getPeriod(TIMER_CHANNEL_2);
        }
        break;
    case 16:
        if (!debug_setType(*data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 17:
        if (!debug_setEvalChannel(*data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 18:
        if (!debug_setAddress(*data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 19:
        if (!debug_setTriggerType(*data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 20:
        if (!debug_setTriggerEvalChannel(*data))
            return REPLY_MAX_EXCEEDED;
        break;
    case 21:
        if (!debug_setTriggerAddress(*data))
            return REPLY_MAX_EXCEEDED;
        break;
    default:
        return REPLY_INVALID_TYPE;
        break;
    }

    return REPLY_OK;
}

static void handleOTP(void)
{
    switch (ActualCommand.Type)
    {
    case 0: // OTP_INIT
        ((ActualCommand.Motor == 1) ? &Evalboards.ch2 : &Evalboards.ch1)->OTP_init();
        break;
    case 1: // OTP_ADDRESS
        ((ActualCommand.Motor == 1) ? &Evalboards.ch2 : &Evalboards.ch1)->OTP_address(ActualCommand.Value.UInt32);
        break;
    case 2: // OTP_VALUE
        ((ActualCommand.Motor == 1) ? &Evalboards.ch2 : &Evalboards.ch1)->OTP_value(ActualCommand.Value.UInt32);
        break;
    case 3: // OTP_PROGRAM
        ((ActualCommand.Motor == 1) ? &Evalboards.ch2 : &Evalboards.ch1)->OTP_program();
        break;
    case 4: // OTP_LOCK
        ((ActualCommand.Motor == 1) ? &Evalboards.ch2 : &Evalboards.ch1)->OTP_lock();
        break;
    case 5: // OTP_STATUS
        ActualReply.Value.UInt32 = ((ActualCommand.Motor == 1) ? &Evalboards.ch2 : &Evalboards.ch1)->OTP_status();
        break;
    default:
        ActualReply.Status = REPLY_INVALID_TYPE;
        break;
    }
}
