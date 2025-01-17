/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include "Board.h"
#include "tmc/BoardAssignment.h" // For the Board IDs

static SPIChannelTypeDef *TMC9660_STEPPER_SPIChannel;
static UART_Config *TMC9660_STEPPER_UARTChannel;
static uint8_t lastStatus;

// Evalboard errors reported in Evalboards.ch1.errors
#define EVAL_ERROR_WRONG_MODULE_ID   (1<<0)
#define EVAL_ERROR_NOT_BOOTSTRAPPED  (1<<1)
#define EVAL_ERROR_NOT_IN_BOOTLOADER (1<<2)
#define EVAL_ERROR_FAULT_DURING_BOOT (1<<3)

#ifdef TMC_API_EXTERNAL_CRC_TABLE
extern const uint8_t tmcCRCTable_Poly7Reflected[256];
#else
const uint8_t tmcCRCTable_Poly7Reflected[256] = {
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, 0x1C, 0x8D, 0xFF,
    0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE,
    0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, 0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A,
    0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, 0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C,
    0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86,
    0x17, 0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 0x54, 0xC5,
    0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 0xE0, 0x71, 0x03, 0x92, 0xE7,
    0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
    0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35,
    0xA4, 0xD1, 0x40, 0x32, 0xA3, 0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C,
    0x2E, 0xBF, 0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 0x8C,
    0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8, 0x39, 0x4B, 0xDA,
    0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, 0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50,
    0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
};
#endif

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

static uint8_t CRC8(uint8_t *data, uint32_t bytes)
{
    uint8_t result = 0;

    while (bytes--) result = tmcCRCTable_Poly7Reflected[result ^ *data++];

    // Flip the result around
    // swap odd and even bits
    result = ((result >> 1) & 0x55) | ((result & 0x55) << 1);
    // swap consecutive pairs
    result = ((result >> 2) & 0x33) | ((result & 0x33) << 2);
    // swap nibbles ...
    result = ((result >> 4) & 0x0F) | ((result & 0x0F) << 4);

    return result;
}

static int32_t processTunnelBL(uint8_t command, int32_t value)
{
    uint8_t data[8] = {0};

    data[0] = 0x55;    // Sync byte
    data[1] = 0x01;    // Device Address
    data[2] = command; // Command
    data[3] = (value >> 24) & 0xFF;
    data[4] = (value >> 16) & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = (value) & 0xFF;
    data[7] = CRC8(data, 7);

    UART_readWrite(TMC9660_STEPPER_UARTChannel, &data[0], 8, 8);

    lastStatus = data[2];

    return ((uint32_t) data[3] << 24) | ((uint32_t) data[4] << 16) | ((uint32_t) data[5] << 8) | data[6];
}

static uint8_t calcCheckSum(uint8_t *data, uint32_t bytes)
{
    uint8_t checkSum = 0;

    for (uint32_t i = 0; i < bytes; i++) { checkSum += data[i]; }
    return checkSum;
}

static int32_t processTunnelApp(uint8_t operation, uint8_t type, uint8_t motor, uint32_t *value, uint8_t *status)
{
    uint8_t data[9] = {0};

    data[0] = 0x01;      // Module Address
    data[1] = operation; //Operation
    data[2] = type;      //type
    data[3] = motor;     //motor
    data[4] = (*value >> 24) & 0xFF;
    data[5] = (*value >> 16) & 0xFF;
    data[6] = (*value >> 8) & 0xFF;
    data[7] = (*value) & 0xFF;
    data[8] = calcCheckSum(data, 8);

    int32_t uartStatus = UART_readWrite(TMC9660_STEPPER_UARTChannel, &data[0], 9, 9);

    // Timeout?
    if (uartStatus == -1)
        return -1;

    // Byte 8: CRC correct?
    if (data[8] != calcCheckSum(data, 8))
        return -2;

    if (status != 0)
    {
        *status = data[2];
    }

    *value = ((uint32_t) data[4] << 24) | ((uint32_t) data[5] << 16) | ((uint32_t) data[6] << 8) | data[7];

    return 0;
}

static void deInit(void)
{
    HAL.IOs->config->setHigh(Pins.RESET_LB);
    TMC9660_STEPPER_SPIChannel->setEnabled(1);
}

static void initTunnel(void)
{
    //Deinit SPI
    TMC9660_STEPPER_SPIChannel->setEnabled(0);

    TMC9660_STEPPER_UARTChannel         = HAL.UART;
    TMC9660_STEPPER_UARTChannel->pinout = UART_PINS_2;
    TMC9660_STEPPER_UARTChannel->rxtx.init();
    // Some forwarded commands can take longer than the default 10ms timeout
    // like the TMCLScript download sequences. For now, we just greatly
    // increase this timeout, later we should make this a more fine-grained
    // selection.
    TMC9660_STEPPER_UARTChannel->timeout = 250; // [ms]

    HAL.IOs->config->setHigh(Pins.HOLDN_FLASH);

    //    HAL.IOs->config->setHigh(Pins.SPI_EN);
    //    HAL.IOs->config->setLow(Pins.I2C_EN);
    //    HAL.IOs->config->setLow(Pins.RESETN);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
    UNUSED(motor);
    uint32_t errors = TMC_ERROR_NONE;

    switch (type)
    {
    case 0:
        // Process Tunnel Commands
        *value = processTunnelBL(motor, *value);
        break;
    case 1:
        // Return status byte
        *value = lastStatus;
        break;
    case 2:
        // Get Module ID of App
        uint8_t status;
        processTunnelApp(136, 1, 0, (uint32_t *) value, &status);
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
    // Check if the TMC9660 bootloader is active
    if (processTunnelBL(0, 0) == 0x544D0001)
    {
        if (Evalboards.ch1.id != ID_TMC9660_STEPPER_BL_EVAL)
        {
            Evalboards.ch1.errors |= EVAL_ERROR_NOT_BOOTSTRAPPED;
            return;
        }
    }
    // Bootloader did not respond -> Either the chip is inactive (no VM, reset, ...),
    // or the chip is waiting for TMCL commands
    uint32_t moduleID = 0;
    uint8_t status = 0;
    int32_t tunnelStatus = processTunnelApp(136, 1, 0, &moduleID, &status);
    moduleID >>= 16;

    if (tunnelStatus == -1)
    {
        // No response from TMC9660
    }
    else if (tunnelStatus == -2)
    {
        // Response with checksum error
        // We ignore this, we only seek to find definite mismatches
    }
    else if (tunnelStatus == 0)
    {
        // TMC9660 responded -> Check if mode is right
        if (Evalboards.ch1.id == ID_TMC9660_STEPPER_BL_EVAL)
        {
            // Bootloader mode expected -> Raise an error
            Evalboards.ch1.errors |= EVAL_ERROR_NOT_IN_BOOTLOADER;
            return;
        }

        // Verify correct module ID
        uint32_t expectedModuleID = (Evalboards.ch1.id == ID_TMC9660_STEPPER_REG_EVAL)? 17 : 51;
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

    uint8_t data[9] = { 0 };

    data[0] = 0x01; // Module Address forced to 1 to reach the TMC9660-3PH-EVAL // ToDo: Make configurable
    data[1] = ActualCommand->Opcode; //Operation
    data[2] = ActualCommand->Type; //type
    data[3] = ActualCommand->Motor; //motor
    data[4] = (ActualCommand->Value.Int32 >> 24) & 0xFF;
    data[5] = (ActualCommand->Value.Int32 >> 16) & 0xFF;
    data[6] = (ActualCommand->Value.Int32 >> 8 ) & 0xFF;
    data[7] = (ActualCommand->Value.Int32      ) & 0xFF;
    data[8] = calcCheckSum(data, 8);

    int32_t uartStatus = UART_readWrite(TMC9660_STEPPER_UARTChannel, &data[0], 9, 9);

    // Timeout?
    if(uartStatus == -1)
    {
        // ToDo: Send back a different error code for timeouts?
        ActualReply->Status = 1; // REPLY_CHKERR
        return 1;
    }

    // Byte 8: CRC correct?
    if (data[8] != calcCheckSum(data, 8))
    {
        ActualReply->Status = 1; // REPLY_CHKERR
        return 1;
    }

    if (ActualCommand->Opcode == 0x88 && ActualCommand->Type == 0)
    {
        // ASCII GetVersion special case
        // ...
        ActualReply->IsSpecial   = 1;
        ActualReply->Special[0]  = 2; // SERIAL_HOST_ADDRESS

        for(uint8_t i = 0; i < 8; i++)
        {
            ActualReply->Special[i+1] = data[i+1];
        }

        return true;
    }

    // Normal datagrams: Fix up the adjusted module ID in the reply
    ActualReply->ModuleId = 3;
    ActualReply->Status = data[2];
    ActualReply->Value.Int32 = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | data[7];

    return true;
}

void TMC9660_STEPPER_init(void)
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
        TMC9660_STEPPER_SPIChannel = &HAL.SPI->ch1;
        TMC9660_STEPPER_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

    initTunnel();

    Evalboards.ch1.userFunction         = userFunction;
    Evalboards.ch1.fwdTmclCommand       = fwdTmclCommand;
    Evalboards.ch1.deInit               = deInit;

    // Pull the chip out of reset
    HAL.IOs->config->setLow(Pins.RESET_LB);

    // Wait a bit - letting the fault pin assert as part of the TMC9660 boot sequence
    wait(1);

    // Wait for the fault pin to deassert, signaling a finished boot
    uint32_t timestamp = systick_getTick();
    while (!HAL.IOs->config->isHigh(Pins.FAULTN_LB))
    {
        if (timeSince(timestamp) > 100)
        {
            Evalboards.ch1.errors |= EVAL_ERROR_FAULT_DURING_BOOT;
            break;
        }
    }

    if (Evalboards.ch1.errors == 0)
    {
        // Check if we have a mode mismatch with the running TMC9660 (bootloader or wrong app mode)
        verifyTMC9660Mode();
    }
}
