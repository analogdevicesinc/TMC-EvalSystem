#ifndef BOARD_H
#define BOARD_H

#include "tmc/helpers/API_Header.h"

#include "../hal/derivative.h"
#include "../hal/HAL.h"
#include "../tmc/VitalSignsMonitor.h"

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

// Evalboard channel struct
typedef struct
{
	uint8  id;
	uint32 errors;
	uint32 VMMax;
	uint32 VMMin;
	unsigned char numberOfMotors;
	ConfigurationTypeDef *config;
	uint32 (*left)                (uint8 motor, int32 velocity);            // move left with velocity <velocity>
	uint32 (*right)               (uint8 motor, int32 velocity);            // move right with velocity <velocity>
	uint32 (*rotate)              (uint8 motor, int32 velocity);            // move right with velocity <velocity>
	uint32 (*stop)                (uint8 motor);                            // stop motor
	uint32 (*moveTo)              (uint8 motor, int32 position);            // move to position <position>
	uint32 (*moveBy)              (uint8 motor, int32 *ticks);              // move by <ticks>, changes ticks to absolute target
	uint32 (*moveProfile)         (uint8 motor, int32 position);            // move profile <position>
	uint32 (*SAP)                 (uint8 type, uint8 motor, int32 value);   // set axis parameter -> TMCL conformance
	uint32 (*GAP)                 (uint8 type, uint8 motor, int32 *value);  // get axis parameter -> TMCL conformance
	uint32 (*STAP)                (uint8 type, uint8 motor, int32 value);   // store axis parameter -> TMCL conformance
	uint32 (*RSAP)                (uint8 type, uint8 motor, int32 value);   // restore axis parameter -> TMCL conformance
	void (*readRegister)          (u8 motor, uint8 address, int32 *value);  // Motor needed since some chips utilize it as a switch between low and high values
	void (*writeRegister)         (u8 motor, uint8 address, int32 value);   // Motor needed since some chips utilize it as a switch between low and high values
	uint32 (*getMeasuredSpeed)    (uint8 motor, int32 *value);
	uint32 (*userFunction)        (uint8 type, uint8 motor, int32 *value);

	void (*periodicJob)           (uint32 tick);
	void (*deInit)                (void);

	void (*checkErrors)           (uint32 tick);
	void (*enableDriver)          (DriverState state);

	uint8 (*cover)                (uint8 data, uint8 lastTransfer);
	void  (*fullCover)            (uint8 *data, size_t length);

	uint32 (*getMin)              (uint8 type, uint8 motor, int32 *value);
	uint32 (*getMax)              (uint8 type, uint8 motor, int32 *value);
} EvalboardFunctionsTypeDef;


// "hash" function to resolve API error <=> Map index
inline u8 error_index(u8 error)
{
	u8 i = 0;
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

EvalboardsTypeDef Evalboards;

void periodicJobDummy(uint32 tick);
void board_setDummyFunctions(EvalboardFunctionsTypeDef *channel);

#include "TMCDriver.h"
#include "TMCMotionController.h"

#endif /* BOARD_H */
