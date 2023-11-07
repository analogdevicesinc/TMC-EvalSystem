/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef BOARD_H
#define BOARD_H

#include "tmc/helpers/API_Header.h"

#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/VitalSignsMonitor.h"

#include "tmc/ic/TMC2130/TMC2130.h"
#include "tmc/ic/TMC2160/TMC2160.h"
#include "tmc/ic/TMC2208/TMC2208.h"
#include "tmc/ic/TMC2224/TMC2224.h"
#include "tmc/ic/TMC2590/TMC2590.h"
#include "tmc/ic/TMC2660/TMC2660.h"
#include "tmc/ic/TMC6100/TMC6100.h"
#include "tmc/ic/TMC6200/TMC6200.h"
#include "tmc/ic/TMC7300/TMC7300.h"


#include "tmc/ic/TMC2209/TMC2209.h"
#include "tmc/ic/TMC2225/TMC2225.h"
#include "tmc/ic/TMC2226/TMC2226.h"
#include "tmc/ic/TMC2300/TMC2300.h"
#include "tmc/ic/MAX22216/MAX22216.h"
#include "tmc/ic/TMC4361A/TMC4361A.h"
#include "tmc/ic/TMC5031/TMC5031.h"
#include "tmc/ic/TMC5041/TMC5041.h"
#include "tmc/ic/TMC5062/TMC5062.h"
#include "tmc/ic/TMC5072/TMC5072.h"
#include "tmc/ic/TMC5130/TMC5130.h"
#include "tmc/ic/TMC5160/TMC5160.h"
#include "tmc/ic/TMC8461/TMC8461.h"
#include "tmc/ic/TMC8462/TMC8462.h"

// parameter access (for axis parameters)
#define READ   0
#define WRITE  1

typedef enum {
	LIMIT_MIN,
	LIMIT_MAX
} AxisParameterLimit;

typedef enum {
	DRIVER_DISABLE,
	DRIVER_ENABLE,
	DRIVER_USE_GLOBAL_ENABLE
} DriverState;

typedef enum {
	OTP_STATUS_IDLE = 0,
	OTP_STATUS_PROGRAMMING = 1,
	OTP_STATUS_DONE = 2,
	OTP_STATUS_FAILED = 3
} OTP_Status;

// Evalboard channel struct
typedef struct
{
	void *type;
	uint8_t  id;
	uint32_t errors;
	int32_t VMMax;
	int32_t VMMin;
	unsigned char numberOfMotors;
	ConfigurationTypeDef *config;
	uint32_t (*left)                (uint8_t motor, int32_t velocity);            // move left with velocity <velocity>
	uint32_t (*right)               (uint8_t motor, int32_t velocity);            // move right with velocity <velocity>
	uint32_t (*rotate)              (uint8_t motor, int32_t velocity);            // move right with velocity <velocity>
	uint32_t (*stop)                (uint8_t motor);                            // stop motor
	uint32_t (*moveTo)              (uint8_t motor, int32_t position);            // move to position <position>
	uint32_t (*moveBy)              (uint8_t motor, int32_t *ticks);              // move by <ticks>, changes ticks to absolute target
	uint32_t (*moveProfile)         (uint8_t motor, int32_t position);            // move profile <position>
	uint32_t (*SAP)                 (uint8_t type, uint8_t motor, int32_t value);   // set axis parameter -> TMCL conformance
	uint32_t (*GAP)                 (uint8_t type, uint8_t motor, int32_t *value);  // get axis parameter -> TMCL conformance
	uint32_t (*STAP)                (uint8_t type, uint8_t motor, int32_t value);   // store axis parameter -> TMCL conformance
	uint32_t (*RSAP)                (uint8_t type, uint8_t motor, int32_t value);   // restore axis parameter -> TMCL conformance
	uint32_t (*SIO)                 (uint8_t type, uint8_t motor, int32_t value);
	uint32_t (*GIO)                 (uint8_t type, uint8_t motor, int32_t *value);
	void (*readRegister)          (uint8_t motor, uint16_t address, int32_t *value);  // Motor needed since some chips utilize it as a switch between low and high values
	void (*writeRegister)         (uint8_t motor, uint16_t address, int32_t value);   // Motor needed since some chips utilize it as a switch between low and high values
	uint32_t (*getMeasuredSpeed)    (uint8_t motor, int32_t *value);
	uint32_t (*userFunction)        (uint8_t type, uint8_t motor, int32_t *value);

	void (*periodicJob)           (uint32_t tick);
	void (*deInit)                (void);

	void (*checkErrors)           (uint32_t tick);
	void (*enableDriver)          (DriverState state);

	uint8_t (*cover)                (uint8_t data, uint8_t lastTransfer);
	void  (*fullCover)            (uint8_t *data, size_t length);

	uint32_t (*getMin)              (uint8_t type, uint8_t motor, int32_t *value);
	uint32_t (*getMax)              (uint8_t type, uint8_t motor, int32_t *value);

	uint8_t (*onPinChange)(IOPinTypeDef *pin, IO_States state);

	void (*OTP_init)(void);
	void (*OTP_address)(uint32_t address);
	void (*OTP_value)(uint32_t value);
	void (*OTP_program)(void);
	void (*OTP_lock)(void);
	OTP_Status (*OTP_status)(void);
} EvalboardFunctionsTypeDef;


// "hash" function to resolve API error <=> Map index
inline uint8_t error_index(uint8_t error)
{
	uint8_t i = 0;
	for(; error != 1; i++)
		error >>= 1;
	return i;
}

// Evalboard errors
// TODO: Extends API Error bits. For more information, see comment in TMCError typedef.
typedef enum {
	TMC_ERROR_TYPE = 0x04,
	TMC_ERROR_ADDRESS = 0x04,
	TMC_ERROR_NOT_DONE = 0x20
} EvalboardErrorBit;

// Channel identifiers required to switch between channels in readWrite
typedef enum {
	CHANNEL_1,
	CHANNEL_2
} EvalboardChannels;

// struct for our Evalsystem, with two available Evalboard channels
typedef struct
{
	EvalboardFunctionsTypeDef ch1;
	EvalboardFunctionsTypeDef ch2;
	DriverState driverEnable; // global driver status
} EvalboardsTypeDef;

extern EvalboardsTypeDef Evalboards;

typedef enum {
	TMC_BOARD_COMM_DEFAULT,
	TMC_BOARD_COMM_SPI,
	TMC_BOARD_COMM_UART,
	TMC_BOARD_COMM_WLAN
} TMC_Board_Comm_Mode;

// Group all the motion controller chip objects into a single union to save memory,
// since we will only ever use one driver at a time
typedef union {
    TMC4361ATypeDef tmc4361A;
    TMC5031TypeDef tmc5031;
    TMC5041TypeDef tmc5041;
    TMC5062TypeDef tmc5062;
    TMC5072TypeDef tmc5072;
    TMC5130TypeDef tmc5130;
    TMC5160TypeDef tmc5160;
    TMC8461TypeDef tmc8461;
    TMC8462TypeDef tmc8462;
} MotionControllerBoards;
extern MotionControllerBoards motionControllerBoards;

// Group all the driver chip objects into a single union to save memory,
// since we will only ever use one motion controller at a time
typedef union {
    TMC2130TypeDef tmc2130;
    TMC2160TypeDef tmc2160;
    TMC2208TypeDef tmc2208;
    TMC2224TypeDef tmc2224;
    TMC2590TypeDef tmc2590;
    TMC2660TypeDef tmc2660;
    TMC7300TypeDef tmc7300;
    TMC2209TypeDef tmc2209;
    TMC2225TypeDef tmc2225;
    TMC2226TypeDef tmc2226;
    TMC2300TypeDef tmc2300;
	MAX22216TypeDef max22216;
} DriverBoards;
extern DriverBoards driverBoards;

void periodicJobDummy(uint32_t tick);
void board_setDummyFunctions(EvalboardFunctionsTypeDef *channel);

#include "TMCDriver.h"
#include "TMCMotionController.h"

#endif /* BOARD_H */
