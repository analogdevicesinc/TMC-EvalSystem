#include "Board.h"
#include "tmc/ic/TMC2225/TMC2225.h"
#include "tmc/StepDir.h"

#undef  TMC2225_MAX_VELOCITY
#define TMC2225_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  390  // VM[V/10] max

#define MOTORS 1

#define VREF_FULLSCALE 2714 // mV

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);

static void checkErrors (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static void enableDriver(DriverState state);

static UART_Config *TMC2225_UARTChannel;
static TMC2225TypeDef TMC2225;
static ConfigurationTypeDef *TMC2225_config;

static uint16_t vref; // mV

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC2225_CRC(data, length) tmc_CRC8(data, length, 1)

typedef struct
{
	IOPinTypeDef  *DRV_ENN;
	IOPinTypeDef  *STEP;
	IOPinTypeDef  *DIR;
	IOPinTypeDef  *MS1;
	IOPinTypeDef  *MS2;
	IOPinTypeDef  *DIAG;
	IOPinTypeDef  *INDEX;
	IOPinTypeDef  *UC_PWM;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint8_t restore(void);

static inline TMC2225TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC2225;
}

static inline UART_Config *channelToUART(uint8_t channel)
{
	UNUSED(channel);

	return TMC2225_UARTChannel;
}

// => UART wrapper
// Write [writeLength] bytes from the [data] array.
// If [readLength] is greater than zero, read [readLength] bytes from the
// [data] array.
void tmc2225_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength)
{
	UART_readWrite(channelToUART(channel), data, writeLength, readLength);
}
// <= UART wrapper

// => CRC wrapper
// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc2225_CRC8(uint8_t *data, size_t length)
{
	return TMC2225_CRC(data, length);
}
// <= CRC wrapper

void tmc2225_writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	tmc2225_writeInt(motorToIC(motor), address, value);
}

void tmc2225_readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc2225_readInt(motorToIC(motor), address);
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_rotate(motor, velocity);

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
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;
	int32_t buffer = 0;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 0:
		// Target position
		if(readWrite == READ) {
			*value = StepDir_getTargetPosition(motor);
		} else if(readWrite == WRITE) {
			StepDir_moveTo(motor, *value);
		}
		break;
	case 1:
		// Actual position
		if(readWrite == READ) {
			*value = StepDir_getActualPosition(motor);
		} else if(readWrite == WRITE) {
			StepDir_setActualPosition(motor, *value);
		}
		break;
	case 2:
		// Target speed
		if(readWrite == READ) {
			*value = StepDir_getTargetVelocity(motor);
		} else if(readWrite == WRITE) {
			StepDir_rotate(motor, *value);
		}
		break;
	case 3:
		// Actual speed
		if(readWrite == READ) {
			*value = StepDir_getActualVelocity(motor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 4:
		// Maximum speed
		if(readWrite == READ) {
			*value = StepDir_getVelocityMax(motor);
		} else if(readWrite == WRITE) {
			StepDir_setVelocityMax(motor, abs(*value));
		}
		break;
	case 5:
		// Maximum acceleration
		if(readWrite == READ) {
			*value = StepDir_getAcceleration(motor);
		} else if(readWrite == WRITE) {
			StepDir_setAcceleration(motor, *value);
		}
		break;
	case 6:
		// Maximum current
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_IHOLD_IRUN, TMC2225_IRUN_MASK, TMC2225_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_IHOLD_IRUN, TMC2225_IRUN_MASK, TMC2225_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_IHOLD_IRUN, TMC2225_IHOLD_MASK, TMC2225_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_IHOLD_IRUN, TMC2225_IHOLD_MASK, TMC2225_IHOLD_SHIFT, *value);
		}
		break;
	case 8:
		// Position reached flag
		if(readWrite == READ) {
			*value = (StepDir_getStatus(motor) & STATUS_TARGET_REACHED)? 1:0;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 9:
		// VREF
		if (readWrite == READ) {
			*value = vref;
		} else {
			if ((uint32_t) *value < VREF_FULLSCALE) {
				vref = *value;
				Timer.setDuty(TIMER_CHANNEL_3, vref * TIMER_MAX / VREF_FULLSCALE);
			} else {
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 28:
		// Internal RSense
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_GCONF, TMC2225_INTERNAL_RSENSE_MASK, TMC2225_INTERNAL_RSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_GCONF, TMC2225_INTERNAL_RSENSE_MASK, TMC2225_INTERNAL_RSENSE_SHIFT, *value);
		}
		break;
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			buffer = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)TMC2225_FIELD_READ(motorToIC(motor), TMC2225_TSTEP, TMC2225_TSTEP_MASK, TMC2225_TSTEP_SHIFT));
			*value = (abs(buffer) < 20) ? 0 : buffer;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 50: // StepDir internal(0)/external(1)
		if(readWrite == READ) {
			*value = StepDir_getMode(motor);
		} else if(readWrite == WRITE) {
			StepDir_setMode(motor, *value);
		}
		break;
	case 51: // StepDir interrupt frequency
		if(readWrite == READ) {
			*value = StepDir_getFrequency(motor);
		} else if(readWrite == WRITE) {
			StepDir_setFrequency(motor, *value);
		}
		break;
	case 140:
		// Microstep Resolution
		if(readWrite == READ) {
			*value = 256 >> TMC2225_FIELD_READ(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_MRES_MASK, TMC2225_MRES_SHIFT);
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
				TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_MRES_MASK, TMC2225_MRES_SHIFT, *value);
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
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_TBL_MASK, TMC2225_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_TBL_MASK, TMC2225_TBL_SHIFT, *value);
		}
		break;
	case 165:
		// Chopper hysteresis end / fast decay time
		if(readWrite == READ) {
			if(tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF) & (1<<14))
			{
				*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_HEND_MASK, TMC2225_HEND_SHIFT);
			}
			else
			{
				buffer = tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF);
				*value = (tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF) >> 4) & 0x07;
				if(buffer & (1<<11))
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 166:
		// Chopper hysteresis start / sine wave offset
		if(readWrite == READ) {
			if(tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF) & (1<<14))
			{
				*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_HSTRT_MASK, TMC2225_HSTRT_SHIFT);
			}
			else
			{
				buffer = tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF);
				*value = (tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF) >> 7) & 0x0F;
				if(buffer & (1<<11))
					*value |= 1<<3;
			}
		} else if(readWrite == WRITE) {
			if(tmc2225_readInt(motorToIC(motor), TMC2225_CHOPCONF) & (1<<14))
			{
				TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_HSTRT_MASK, TMC2225_HSTRT_SHIFT, *value);
			}
			else
			{
				TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_HEND_MASK, TMC2225_HEND_SHIFT, *value);
			}
		}
		break;
	case 167:
		// Chopper off time
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_TOFF_MASK, TMC2225_TOFF_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_TOFF_MASK, TMC2225_TOFF_SHIFT, *value);
		}
		break;
	case 179:
		// VSense
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_VSENSE_MASK, TMC2225_VSENSE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_CHOPCONF, TMC2225_VSENSE_MASK, TMC2225_VSENSE_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_DRVSTATUS, TMC2225_CS_ACTUAL_MASK, TMC2225_CS_ACTUAL_SHIFT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// smartEnergy stall velocity
		if(readWrite == READ) {
			*value = StepDir_getStallGuardThreshold(motor);
		} else if(readWrite == WRITE) {
			// Store the threshold value in the internal StepDir generator
			StepDir_setStallGuardThreshold(motor, *value);
		}
		break;
	case 186:
		// PWM threshold speed
		if(readWrite == READ) {
			buffer = tmc2225_readInt(motorToIC(motor), TMC2225_TPWMTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer) ? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value) ? *value : 1));
			tmc2225_writeInt(motorToIC(motor), TMC2225_TPWMTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_PWMCONF, TMC2225_PWM_GRAD_MASK, TMC2225_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_PWMCONF, TMC2225_PWM_GRAD_MASK, TMC2225_PWM_GRAD_SHIFT, *value);

			// Enable/disable stealthChop accordingly
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_GCONF, TMC2225_EN_SPREADCYCLE_MASK, TMC2225_EN_SPREADCYCLE_SHIFT, (*value > 0) ? 0 : 1);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_PWMCONF, TMC2225_PWM_FREQ_MASK, TMC2225_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_PWMCONF, TMC2225_PWM_FREQ_MASK, TMC2225_PWM_FREQ_SHIFT, *value);
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
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_PWMCONF, TMC2225_PWM_AUTOSCALE_MASK, TMC2225_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_PWMCONF, TMC2225_PWM_AUTOSCALE_MASK, TMC2225_PWM_AUTOSCALE_SHIFT, (*value)? 1:0);
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = TMC2225_FIELD_READ(motorToIC(motor), TMC2225_PWMCONF, TMC2225_FREEWHEEL_MASK, TMC2225_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2225_FIELD_UPDATE(motorToIC(motor), TMC2225_PWMCONF, TMC2225_FREEWHEEL_MASK, TMC2225_FREEWHEEL_SHIFT, *value);
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
	Evalboards.ch2.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	switch(type)
	{
	case 0:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	case 1:
		tmc2225_set_slave(motorToIC(motor), (*value) & 0xFF);
		break;
	case 2:
		*value = tmc2225_get_slave(motorToIC(motor));
		break;
	case 3:
		*value = Timer.getDuty(TIMER_CHANNEL_3) * 100 / TIMER_MAX;
		break;
	case 4:
		Timer.setDuty(TIMER_CHANNEL_3, (uint32_t) ((uint32_t)(*value) * (uint32_t)TIMER_MAX) / (uint32_t)100);
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}

	return errors;
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->reset(Pins.DRV_ENN);

	HAL.IOs->config->reset(Pins.STEP);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.MS1);
	HAL.IOs->config->reset(Pins.MS2);
	HAL.IOs->config->reset(Pins.DIAG);
	HAL.IOs->config->reset(Pins.INDEX);

	StepDir_deInit();
}

static uint8_t reset()
{
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);

	return tmc2225_reset(&TMC2225);
}

static uint8_t restore()
{
	return tmc2225_restore(&TMC2225);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setHigh(Pins.DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setLow(Pins.DRV_ENN);
}

static void periodicJob(uint32_t tick)
{
	for(int motor = 0; motor < MOTORS; motor++)
	{
		tmc2225_periodicJob(&TMC2225, tick);
		StepDir_periodicJob(motor);
	}
}

void TMC2225_init(void)
{
	tmc_fillCRC8Table(0x07, true, 1);

	Pins.DRV_ENN  = &HAL.IOs->pins->DIO0;
	Pins.STEP     = &HAL.IOs->pins->DIO6;
	Pins.DIR      = &HAL.IOs->pins->DIO7;
	Pins.MS1      = &HAL.IOs->pins->DIO3;
	Pins.MS2      = &HAL.IOs->pins->DIO4;
	Pins.DIAG     = &HAL.IOs->pins->DIO1;
	Pins.INDEX    = &HAL.IOs->pins->DIO2;
	Pins.UC_PWM   = &HAL.IOs->pins->DIO9;

	HAL.IOs->config->toOutput(Pins.DRV_ENN);
	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
	HAL.IOs->config->toOutput(Pins.MS1);
	HAL.IOs->config->toOutput(Pins.MS2);
	HAL.IOs->config->toInput(Pins.DIAG);
	HAL.IOs->config->toInput(Pins.INDEX);

	TMC2225_UARTChannel = HAL.UART;
	TMC2225_UARTChannel->pinout = UART_PINS_2;
	TMC2225_UARTChannel->rxtx.init();

	TMC2225_config = Evalboards.ch2.config;

	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_RESET;
	Evalboards.ch2.config->configIndex  = 0;

	Evalboards.ch2.rotate               = rotate;
	Evalboards.ch2.right                = right;
	Evalboards.ch2.left                 = left;
	Evalboards.ch2.stop                 = stop;
	Evalboards.ch2.GAP                  = GAP;
	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.moveTo               = moveTo;
	Evalboards.ch2.moveBy               = moveBy;
	Evalboards.ch2.writeRegister        = tmc2225_writeRegister;
	Evalboards.ch2.readRegister         = tmc2225_readRegister;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.periodicJob          = periodicJob;

	tmc2225_init(&TMC2225, 0, TMC2225_config, &tmc2225_defaultRegisterResetState[0]);

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);
	StepDir_setVelocityMax(0, 51200);
	StepDir_setAcceleration(0, 51200);

#if defined(Startrampe)
	Pins.UC_PWM->configuration.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PinAFConfig(Pins.UC_PWM->port, Pins.UC_PWM->bit, GPIO_AF_TIM1);
#elif defined(Landungsbruecke)
	HAL.IOs->config->toOutput(Pins.UC_PWM);
	Pins.UC_PWM->configuration.GPIO_Mode = GPIO_Mode_AF4;
#endif

	vref = 2000;
	HAL.IOs->config->set(Pins.UC_PWM);
	Timer.init();
	Timer.setDuty(TIMER_CHANNEL_3, vref * TIMER_MAX / VREF_FULLSCALE);

	enableDriver(DRIVER_ENABLE);
};
