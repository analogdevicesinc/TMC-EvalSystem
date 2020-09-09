#include "boards/Board.h"
#include "tmc/BLDC.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  18   // VM[V/10] min
#define VM_MAX  121  // VM[V/10] max (11V + 10%)

#define MOTORS 1

// todo: Calculate this and describe the calculation instead of measuring it
#define CURRENT_SCALING_FACTOR 10712

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);

static void checkErrors (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static void enableDriver(DriverState state);

typedef struct
{
	IOPinTypeDef  *VBAT_MEASURE;
	IOPinTypeDef  *HALL_U;
	IOPinTypeDef  *HALL_V;
	IOPinTypeDef  *HALL_W;
	IOPinTypeDef  *DIAG;
	IOPinTypeDef  *STDBY;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 1: // Target angle
		if (readWrite == READ)
		{
			*value = BLDC_getTargetAngle();
		}
		else
		{
			errors |= TMC_ERROR_TYPE;
		}

		break;
	case 2: // Hall angle
		if (readWrite == READ)
		{
			*value = BLDC_getHallAngle();
		}
		else
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 3: // Actual PWM
		if (readWrite == READ)
		{
			if (BLDC_isPWMenabled())
			{
				*value = BLDC_getTargetPWM();
			}
			else
			{
				*value = 0;
			}
		}
		else
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4: // Target PWM
		if (readWrite == READ)
		{
			*value = BLDC_getTargetPWM();
		}
		else
		{
			if (*value <= s16_MAX && *value >= s16_MIN)
			{
				BLDC_setTargetPWM(*value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 5: // Commutation mode
		if (readWrite == READ)
		{
			*value = BLDC_getCommutationMode();
		}
		else
		{
			if (*value >= 0 && *value < 2)
			{
				BLDC_setCommutationMode(*value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 6: // Open loop velocity
		if (readWrite == READ)
		{
			*value = BLDC_getOpenloopStepTime();
		}
		else
		{
			if (*value > 0 && *value < u16_MAX)
			{
				BLDC_setOpenloopStepTime(*value);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 7:  // Measured current
		if (readWrite == READ)
		{
			*value = BLDC_getMeasuredCurrent();
		}
		break;
	case 8: // hall invert
		if (readWrite == READ)
		{
			*value = BLDC_getHallInvert();
		}
		else
		{
			BLDC_setHallInvert(*value);
		}
		break;
	case 9: // hall order
		if (readWrite == READ)
		{
			*value = BLDC_getHallOrder();
		}
		else
		{
			if (*value >= 0 && *value < 3)
			{
				BLDC_setHallOrder(*value);
			}
			else
			{
				errors = TMC_ERROR_VALUE;
			}
		}
		break;
	case 10: // Standby
		if (readWrite == READ)
		{
			*value = !HAL.IOs->config->isHigh(Pins.STDBY);
		}
		else
		{
			if (*value)
			{
				// Disable the driver
				enableDriver(DRIVER_DISABLE);

				// Enable standby
				HAL.IOs->config->setLow(Pins.STDBY);
			}
			else
			{
				// Disable standby
				HAL.IOs->config->setHigh(Pins.STDBY);

				// Enable the driver
				enableDriver(DRIVER_ENABLE);
			}
		}
		break;
	case 11: // Battery voltage measurement control
		if (readWrite == READ)
		{
			*value = HAL.IOs->config->isHigh(Pins.VBAT_MEASURE);
		}
		else
		{
			if (*value)
			{
				// Enable battery voltage measurement via the Landungsbruecke
				HAL.IOs->config->setHigh(Pins.VBAT_MEASURE);

				// Set the minimum voltage
				Evalboards.ch2.VMMin = VM_MIN;
			}
			else
			{
				// Disable battery voltage measurement via the Landungsbruecke
				HAL.IOs->config->setLow(Pins.VBAT_MEASURE);

				// Disable the minimum voltage check
				Evalboards.ch2.VMMin = 0;
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

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch2.errors = !(HAL.IOs->config->isHigh(Pins.STDBY));
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	UNUSED(value);

	uint32_t errors = 0;

	switch(type)
	{
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}

	return errors;
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);

	HAL.IOs->config->reset(Pins.DIAG);
	HAL.IOs->config->reset(Pins.STDBY);

	// Re-enable the previously disabled timer module
	Timer.init();
}

static uint8_t reset()
{
	BLDC_setTargetPWM(0);
	BLDC_setCommutationMode(BLDC_OPENLOOP);

	return 0;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
	{
		BLDC_enablePWM(false);
	}
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
	{
		BLDC_enablePWM(true);
	}
}

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);
}

void TMC6300_init(void)
{
	Pins.DIAG         = &HAL.IOs->pins->DIO1;
	Pins.STDBY        = &HAL.IOs->pins->DIO2;
	Pins.HALL_U       = &HAL.IOs->pins->DIO4;
	Pins.HALL_V       = &HAL.IOs->pins->DIO3;
	Pins.HALL_W       = &HAL.IOs->pins->DIO5;
	Pins.VBAT_MEASURE = &HAL.IOs->pins->DIO12;

	// PWM Pins used by BLDC logic:
	// PWM_UL: DIO6
	// PWM_UH: DIO7
	// PWM_VL: DIO8
	// PWM_VH: DIO9
	// PWM_WL: DIO10
	// PWM_WH: DIO11


	HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO17);
	HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO18);

	HAL.IOs->config->toOutput(Pins.STDBY);
	HAL.IOs->config->toOutput(Pins.VBAT_MEASURE);

	HAL.IOs->config->setHigh(Pins.STDBY);
	HAL.IOs->config->setHigh(Pins.VBAT_MEASURE);

	HAL.IOs->config->toInput(Pins.DIAG);

	Evalboards.ch2.config->state        = CONFIG_READY;
	Evalboards.ch2.config->reset        = reset;

	Evalboards.ch2.GAP                  = GAP;
	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.periodicJob          = periodicJob;

	BLDC_init(MEASURE_ONE_PHASE, CURRENT_SCALING_FACTOR, Pins.HALL_U, Pins.HALL_V, Pins.HALL_W);

	enableDriver(DRIVER_ENABLE);
};
