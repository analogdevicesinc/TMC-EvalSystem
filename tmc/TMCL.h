/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef TMCL_H
#define TMCL_H

#include "tmc/helpers/API_Header.h"

#define	BL_MAGIC_VALUE_BL_NEW		0x43215678
#define BL_MAGIC_VALUE_APP_NEW		0x12348765
#define BL_MAGIC_VALUE_OLD			0x12345678

// TMCL commands
#define TMCL_ROR                     1
#define TMCL_ROL                     2
#define TMCL_MST                     3
#define TMCL_MVP                     4
#define TMCL_SAP                     5
#define TMCL_GAP                     6
#define TMCL_STAP                    7
#define TMCL_RSAP                    8
#define TMCL_SGP                     9
#define TMCL_GGP                     10
#define TMCL_STGP                    11
#define TMCL_RSGP                    12
#define TMCL_RFS                     13
#define TMCL_SIO                     14
#define TMCL_GIO                     15
#define TMCL_CALC                    19
#define TMCL_COMP                    20
#define TMCL_JC                      21
#define TMCL_JA                      22
#define TMCL_CSUB                    23
#define TMCL_RSUB                    24
#define TMCL_EI                      25
#define TMCL_DI                      26
#define TMCL_WAIT                    27
#define TMCL_STOP                    28
#define TMCL_SAC                     29
#define TMCL_SCO                     30
#define TMCL_GCO                     31
#define TMCL_CCO                     32
#define TMCL_CALCX                   33
#define TMCL_AAP                     34
#define TMCL_AGP                     35
#define TMCL_CLE                     36
#define TMCL_VECT                    37
#define TMCL_RETI                    38
#define TMCL_ACO                     39

#define TMCL_UF0                     64
#define TMCL_UF1                     65
#define TMCL_UF2                     66
#define TMCL_UF3                     67
#define TMCL_UF4                     68
#define TMCL_UF5                     69
#define TMCL_UF6                     70
#define TMCL_UF7                     71
#define TMCL_UF8                     72

#define TMCL_ApplStop                128
#define TMCL_ApplRun                 129
#define TMCL_ApplStep                130
#define TMCL_ApplReset               131
#define TMCL_DownloadStart           132
#define TMCL_DownloadEnd             133
#define TMCL_ReadMem                 134
#define TMCL_GetStatus               135
#define TMCL_GetVersion              136
#define TMCL_FactoryDefault          137
#define TMCL_SetEvent                138
#define TMCL_SetASCII                139
#define TMCL_SecurityCode            140
#define TMCL_Breakpoint              141
#define TMCL_RamDebug                142
#define TMCL_GetIds                  143
#define TMCL_UF_CH1                  144
#define TMCL_UF_CH2                  145
#define TMCL_writeRegisterChannel_1  146
#define TMCL_writeRegisterChannel_2  147
#define TMCL_readRegisterChannel_1   148
#define TMCL_readRegisterChannel_2   149

#define TMCL_BoardMeasuredSpeed      150
#define TMCL_BoardError              151
#define TMCL_BoardReset              152
#define TMCL_GetInfo                 157

#define TMCL_WLAN                    160
#define TMCL_WLAN_CMD                160
#define TMCL_WLAN_IS_RTS             161
#define TMCL_WLAN_CMDMODE_EN         162
#define TMCL_WLAN_IS_CMDMODE         163

#define TMCL_MIN                     170
#define TMCL_MAX                     171
#define TMCL_OTP                     172

#define TMCL_Boot                    242
#define TMCL_SoftwareReset           255

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall) || defined(LandungsbrueckeV3)
    struct BootloaderConfig {
        uint32_t BLMagic;
        uint32_t drvEnableResetValue;
    };
#endif

// TMCL request
typedef struct
{
    uint8_t   ModuleId;
    uint8_t   Opcode;
    uint8_t   Type;
    uint8_t   Motor;
    uint32_t  Error;
    union
    {
        uint8_t Byte[4];
        uint32_t UInt32;
        int32_t Int32;
        float32_t Float32;
    } Value;
} TMCLCommandTypeDef;

// TMCL reply
typedef struct
{
    uint8_t moduleId;
    uint8_t Status;
    uint8_t Opcode;
    union
    {
        uint8_t Byte[4];
        uint32_t UInt32;
        int32_t Int32;
        float32_t Float32;
    } Value;

    uint8_t Special[9];
    uint8_t IsSpecial;  // next transfer will not use the serial address and the checksum bytes - instead the whole datagram is filled with data (used to transmit ASCII version string)
} TMCLReplyTypeDef;

void tmcl_init();
void tmcl_process();
void tmcl_boot();

#endif /* TMCL_H */
