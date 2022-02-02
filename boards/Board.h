#ifndef BOARD_H
#define BOARD_H

#include "tmc/helpers/API_Header.h"

#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/VitalSignsMonitor.h"

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
	void (*readRegister)          (uint8_t motor, uint8_t address, int32_t *value);  // Motor needed since some chips utilize it as a switch between low and high values
	void (*writeRegister)         (uint8_t motor, uint8_t address, int32_t value);   // Motor needed since some chips utilize it as a switch between low and high values
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

void periodicJobDummy(uint32_t tick);
void board_setDummyFunctions(EvalboardFunctionsTypeDef *channel);

#include "TMCDriver.h"
#include "TMCMotionController.h"

#endif /* BOARD_H */
