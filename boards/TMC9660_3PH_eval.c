/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"
#include "tmc/BoardAssignment.h" // For the Board IDs
#include "tmc/ic/TMC9660/TMC9660.h"
#include "tmc/ic/TMC9660/TMC9660_PARAM_HW_Abstraction.h"

static SPIChannelTypeDef *TMC9660_3PH_SPIChannel;
TMC9660BusType activeBus = TMC9660_BUS_UART;
TMC9660BusAddresses busAddresses;
UART_Config *TMC9660_UARTChannel;
static uint8_t lastStatus;

// Evalboard errors reported in Evalboards.ch1.errors
#define EVAL_ERROR_WRONG_MODULE_ID   (1<<0)
#define EVAL_ERROR_NOT_BOOTSTRAPPED  (1<<1)
#define EVAL_ERROR_NOT_IN_BOOTLOADER (1<<2)

#define DEFAULT_ICID  0

typedef struct
{
    IOPinTypeDef *SPI1_MOSI;
    IOPinTypeDef *SPI1_MISO;
    IOPinTypeDef *SPI1_SCK;
    IOPinTypeDef *SPI1_CSN;
    IOPinTypeDef *HOLDN_FLASH;
    IOPinTypeDef *UART_RX;
    IOPinTypeDef *UART_TX;
    IOPinTypeDef *RESET_LB;
    IOPinTypeDef *DRV_ENABLE;
    IOPinTypeDef *WAKEN_LB;
    IOPinTypeDef *FAULTN_LB;
    IOPinTypeDef *GPIO18_LB;
    IOPinTypeDef *GPIO17_LB;
} PinsTypeDef;

static PinsTypeDef Pins;

bool tmc9660_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(icID);
    int32_t status = UART_readWrite(TMC9660_UARTChannel, data, writeLength, readLength);
    if(status == -1)
        return false;
    return true;
}

uint32_t tmc_getMicrosecondTimestamp()
{
  return systick_getMicrosecondTick();
}

TMC9660BusType tmc9660_getBusType(uint16_t icID)
{
    UNUSED(icID);
    return activeBus;
}

TMC9660BusAddresses tmc9660_getBusAddresses(uint16_t icID)
{
    UNUSED(icID);
    return busAddresses;
}

static void deInit(void)
{
    HAL.IOs->config->setHigh(Pins.RESET_LB);
    TMC9660_3PH_SPIChannel->setEnabled(1);
}

static void initTunnel(void)
{
    //Deinit SPI
    TMC9660_3PH_SPIChannel->setEnabled(0);

    TMC9660_UARTChannel         = HAL.UART;
    TMC9660_UARTChannel->pinout = UART_PINS_2;
    TMC9660_UARTChannel->rxtx.init();
    // Some forwarded commands can take longer than the default 10ms timeout
    // like the TMCLScript download sequences. For now, we just greatly
    // increase this timeout, later we should make this a more fine-grained
    // selection.
    TMC9660_UARTChannel->timeout = 250; // [ms]

    HAL.IOs->config->setHigh(Pins.HOLDN_FLASH);
    //    HAL.IOs->config->setHigh(Pins.SPI_EN);
    //    HAL.IOs->config->setLow(Pins.I2C_EN);
    //    HAL.IOs->config->setLow(Pins.RESETN);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    uint32_t errors = TMC_ERROR_NONE;
    uint32_t readValue;

    switch (type)
    {
    case 0:
        // Process Tunnel Commands
        lastStatus = tmc9660_bl_sendCommand(DEFAULT_ICID, motor, *value, &readValue);
        *value = readValue;
        break;
    case 1:
        // Return status byte
        *value = lastStatus;
        break;
    case 2:
        // Get Module ID of App
        tmc9660_param_sendCommand(DEFAULT_ICID, TMC9660_CMD_GET_VERSION, 1, 0, *value, &readValue);
        *value = readValue;
        // The module ID is the upper 16 bits of the reply
        *(uint32_t *)value >>= 16;
        break;
    default:
        errors |= TMC_ERROR_TYPE;
        break;
    }
    return errors;
}

static void verifyTMC9660Mode()
{
    uint32_t readValue;
    tmc9660_bl_sendCommand(DEFAULT_ICID, TMC9660_BLCMD_GET_INFO, 0, &readValue);

    // Check if the TMC9660 bootloader is active
    if (readValue == 0x544D0001)
    {
        if (Evalboards.ch1.id != ID_TMC9660_3PH_BL_EVAL)
        {
            Evalboards.ch1.errors |= EVAL_ERROR_NOT_BOOTSTRAPPED;
            return;
        }
    }
    // Bootloader did not respond -> Either the chip is inactive (no VM, reset, ...),
    // or the chip is waiting for TMCL commands
    uint32_t moduleID = 0;
    int32_t tunnelStatus = tmc9660_param_sendCommand(DEFAULT_ICID, TMC9660_CMD_GET_VERSION, 1, 0, moduleID, &moduleID);
    moduleID >>= 16;

    if (tunnelStatus == -2)
    {
        // No response from TMC9660
    }
    else if (tunnelStatus == -5)
    {
        // Response with checksum error
        // We ignore this, we only seek to find definite mismatches
    }
    else if (tunnelStatus == 0)
    {
        // TMC9660 responded -> Check if mode is right
        if (Evalboards.ch1.id == ID_TMC9660_3PH_BL_EVAL)
        {
            // Bootloader mode expected -> Raise an error
            Evalboards.ch1.errors |= EVAL_ERROR_NOT_IN_BOOTLOADER;
            return;
        }

        // Verify correct module ID
        uint32_t expectedModuleID = (Evalboards.ch1.id == ID_TMC9660_3PH_REG_EVAL)? 17 : 51;
        if (moduleID != expectedModuleID)
        {
            Evalboards.ch1.errors |= EVAL_ERROR_WRONG_MODULE_ID;
        }
    }
}

static bool fwdTmclCommand(TMCLCommandTypeDef *ActualCommand, TMCLReplyTypeDef *ActualReply)
{
    if(ActualCommand->ModuleId != 3)
    {
        return false;
    }

    uint32_t readValue;
    int32_t status = 0;

    if (Evalboards.ch1.id == ID_TMC9660_3PH_PARAM_EVAL)
    {
        uint8_t opcode = ActualCommand->Opcode;
        uint16_t type  = ActualCommand->Type | (((uint16_t) ActualCommand->Motor >> 4) << 8);
        uint8_t index  = ActualCommand->Motor & 0x0F;
        uint32_t value = ActualCommand->Value.UInt32;

        if (opcode == TMC9660_CMD_GET_VERSION && ActualCommand->Type == 0)
        {
            // Special case handling: GetVersion ASCII command has a specific TMC-API function
            //                        since this command returns a nonstandard response format.
            ActualReply->IsSpecial   = 1;
            ActualReply->Special[0]  = 2; // SERIAL_HOST_ADDRESS;
            tmc9660_param_getVersionASCII(DEFAULT_ICID, &ActualReply->Special[1]);
            return true;
        }

        if (opcode == TMC9660_CMD_BOOT && ActualCommand->Type == 0x81 && ActualCommand->Motor == 0x9F && value == 0xA3B4C5D6)
        {
            // Special case handling: Boot command has a specific TMC-API function
            //                        since this command doesn't return a reply.
            tmc9660_param_returnToBootloader(DEFAULT_ICID);
            // Mark this request as invalid to suppress the Evalsystem TMCL stack sending a reply
            ActualCommand->Error = 1; // TMCL_RX_ERROR_NODATA
            return true;
        }

        status = tmc9660_param_sendCommand(DEFAULT_ICID, opcode, type, index, value, &readValue);
    }
    else if (Evalboards.ch1.id == ID_TMC9660_3PH_REG_EVAL)
    {
        uint8_t opcode          = ActualCommand->Opcode;
        uint16_t registerOffset = ActualCommand->Type | (((uint16_t) ActualCommand->Motor >> 5) << 8);
        uint8_t registerBlock   = ActualCommand->Motor & 0x1F;
        uint32_t value          = ActualCommand->Value.UInt32;

        if (opcode == TMC9660_CMD_GET_VERSION && ActualCommand->Type == 0)
        {
            // Special case handling: GetVersion ASCII command has a specific TMC-API function
            //                        since this command returns a nonstandard response format.
            ActualReply->IsSpecial   = 1;
            ActualReply->Special[0]  = 2; // SERIAL_HOST_ADDRESS;
            tmc9660_reg_getVersionASCII(DEFAULT_ICID, &ActualReply->Special[1]);
            return true;
        }

        if (opcode == TMC9660_CMD_BOOT && ActualCommand->Type == 0x81 && ActualCommand->Motor == 0x9F && value == 0xA3B4C5D6)
        {
            // Special case handling: Boot command has a specific TMC-API function
            //                        since this command doesn't return a reply.
            tmc9660_reg_returnToBootloader(DEFAULT_ICID);
            // Mark this request as invalid to suppress the Evalsystem TMCL stack sending a reply
            ActualCommand->Error = 1; // TMCL_RX_ERROR_NODATA
            return true;
        }

        status = tmc9660_reg_sendCommand(DEFAULT_ICID, opcode, registerOffset, registerBlock, value, &readValue);
    }

    // Timeout?
    if(status == -2)
    {
        // ToDo: Send back a different error code for timeouts?
        ActualReply->Status = 1; // REPLY_CHKERR
        return true;
    }
    else if (status == -5)
    {    // Byte 8: CRC correct?
        ActualReply->Status = 1; // REPLY_CHKERR
        return true;
    }

    // Normal datagrams: Fix up the adjusted module ID in the reply
    ActualReply->ModuleId = 3;
    ActualReply->Status = status;
    ActualReply->Value.Int32 = readValue;

    return true;
}

void TMC9660_3PH_init(void)
{
    Pins.SPI1_SCK    = &HAL.IOs->pins->SPI1_SCK;
    Pins.SPI1_MOSI   = &HAL.IOs->pins->SPI1_SDI;
    Pins.SPI1_MISO   = &HAL.IOs->pins->SPI1_SDO;
    Pins.SPI1_CSN    = &HAL.IOs->pins->SPI1_CSN;
    Pins.HOLDN_FLASH = &HAL.IOs->pins->DIO12;
    Pins.WAKEN_LB    = &HAL.IOs->pins->DIO9;
    Pins.RESET_LB    = &HAL.IOs->pins->DIO8;
    Pins.FAULTN_LB   = &HAL.IOs->pins->DIO7;
    Pins.DRV_ENABLE  = &HAL.IOs->pins->DIO6;

#if defined(LandungsbrueckeV3)
    Pins.UART_RX = &HAL.IOs->pins->DIO10_UART_TX; //Pin21
    Pins.UART_TX = &HAL.IOs->pins->DIO11_UART_RX; //Pin22

    //Set MUX_1 and MUX_2 to zero to connect DIO10 and DIO11 to UART pins DIO10_UART_TX and DIO11_UART_RX respectively.
    *HAL.IOs->pins->SW_UART_PWM.resetBitRegister = HAL.IOs->pins->SW_UART_PWM.bitWeight;
#else
    Pins.UART_RX = &HAL.IOs->pins->DIO10; //Pin21
    Pins.UART_TX = &HAL.IOs->pins->DIO11; //Pin22
#endif

    HAL.IOs->config->toOutput(Pins.HOLDN_FLASH);
    HAL.IOs->config->toOutput(Pins.RESET_LB);
    //    HAL.IOs->config->toOutput(Pins.WAKEN_LB);
    //    HAL.IOs->config->toOutput(Pins.DRV_ENABLE);

    //    SPI.init();
    TMC9660_3PH_SPIChannel = &HAL.SPI->ch1;
    TMC9660_3PH_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

    initTunnel();

    Evalboards.ch1.userFunction         = userFunction;
    Evalboards.ch1.fwdTmclCommand       = fwdTmclCommand;
    Evalboards.ch1.deInit               = deInit;

    busAddresses.device = 1;
    busAddresses.host = 255;

    // Pull the chip out of reset
    HAL.IOs->config->setLow(Pins.RESET_LB);

    // Wait a bit - letting the fault pin assert as part of the TMC9660 boot sequence
    wait(100);
    // Check if we have a mode mismatch with the running TMC9660 (bootloader or wrong app mode)
    verifyTMC9660Mode();
}
