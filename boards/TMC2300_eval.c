/*
 * WARNING: Currently the TMC2300-Eval will send current through the motor for
 * approximately 23ms when the Landungsbruecke is powered on and the TMC2300 has
 * connected supply voltage. This is due to the default driver enable polarity
 * turning the TMC2300 on before the ID detection calls the TMC2300_init()
 * function.
 * Either disconnect the power or the motor prior to startup if your motor is
 * small to prevent damage to it.
 */

#include "boards/Board.h"
#include "tmc/ic/TMC2300/TMC2300.h"
#include "tmc/StepDir.h"

#undef  TMC2300_MAX_VELOCITY
#define TMC2300_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define VM_MIN  18   // VM[V/10] min
#define VM_MAX  121  // VM[V/10] max

#define MOTORS 1

#define TIMEOUT_VALUE 10 // 10 ms

#define CONSISTENCY_CHECK_INTERVAL 0 // ticks

// Eval Error defines
#define ERROR_INCONSISTENT_MASK (1 << 0)
#define ERROR_INCONSISTENT_SHIFT 0

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

static void setStandby(uint8_t enableStandby);
static uint8_t onPinChange(IOPinTypeDef *pin, IO_States state);

static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static void enableDriver(DriverState state);

static UART_Config *TMC2300_UARTChannel;
static TMC2300TypeDef TMC2300;

static int32_t thigh;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC2300_CRC(data, length) tmc_CRC8(data, length, 1)

typedef struct
{
	IOPinTypeDef  *DRV_EN;
	IOPinTypeDef  *STEP;
	IOPinTypeDef  *DIR;
	IOPinTypeDef  *MS1;
	IOPinTypeDef  *MS2;
	IOPinTypeDef  *MODE;
	IOPinTypeDef  *DIAG;
	IOPinTypeDef  *STDBY;
} PinsTypeDef;

static PinsTypeDef Pins;

static uint8_t restore(void);

static inline TMC2300TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);

	return &TMC2300;
}

static inline UART_Config *channelToUART(uint8_t channel)
{
	UNUSED(channel);

	return TMC2300_UARTChannel;
}

// => UART wrapper
// Write [writeLength] bytes from the [data] array.
// If [readLength] is greater than zero, read [readLength] bytes from the
// [data] array.
void tmc2300_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength)
{
	UART_readWrite(channelToUART(channel), data, writeLength, readLength);
}
// <= UART wrapper

// => CRC wrapper
// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc2300_CRC8(uint8_t *data, size_t length)
{
	return TMC2300_CRC(data, length);
}
// <= CRC wrapper

void tmc2300_writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	tmc2300_writeInt(motorToIC(motor), address, value);
}

void tmc2300_readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc2300_readInt(motorToIC(motor), address);
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
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_IHOLD_IRUN, TMC2300_IRUN_MASK, TMC2300_IRUN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_IHOLD_IRUN, TMC2300_IRUN_MASK, TMC2300_IRUN_SHIFT, *value);
		}
		break;
	case 7:
		// Standby current
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_IHOLD_IRUN, TMC2300_IHOLD_MASK, TMC2300_IHOLD_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_IHOLD_IRUN, TMC2300_IHOLD_MASK, TMC2300_IHOLD_SHIFT, *value);
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
		// Standby
		if (readWrite == READ)
		{
			*value = tmc2300_getStandby(motorToIC(motor));
		} else if (readWrite == WRITE) {
			setStandby(*value);
		}
		break;
	case 23:
		// Speed threshold for high speed mode
		if(readWrite == READ) {
			buffer = thigh;
			*value = MIN(0xFFFFF, (1<<24) / ((buffer) ? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value) ? *value : 1));
			thigh = *value;
		}
		break;
	case 29:
		// Measured Speed
		if(readWrite == READ) {
			buffer = (int32_t)(((int64_t)StepDir_getFrequency(motor) * (int64_t)122) / (int64_t)TMC2300_FIELD_READ(motorToIC(motor), TMC2300_TSTEP, TMC2300_TSTEP_MASK, TMC2300_TSTEP_SHIFT));
			*value = (abs(buffer) < 20) ? 0 : buffer;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 30: // UART slave address
		if (readWrite == READ) {
			*value = tmc2300_getSlaveAddress(motorToIC(motor));
		} else {
			if (*value >= 0 && *value <= 3) {
				tmc2300_setSlaveAddress(motorToIC(motor), *value);
			} else {
				errors |= TMC_ERROR_VALUE;
			}
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
			*value = 256 >> TMC2300_FIELD_READ(motorToIC(motor), TMC2300_CHOPCONF, TMC2300_MRES_MASK, TMC2300_MRES_SHIFT);
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
				TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_CHOPCONF, TMC2300_MRES_MASK, TMC2300_MRES_SHIFT, *value);
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
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_CHOPCONF, TMC2300_TBL_MASK, TMC2300_TBL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_CHOPCONF, TMC2300_TBL_MASK, TMC2300_TBL_SHIFT, *value);
		}
		break;
	case 168:
		// smartEnergy current minimum (SEIMIN)
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEIMIN_MASK, TMC2300_SEIMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEIMIN_MASK, TMC2300_SEIMIN_SHIFT, *value);
		}
		break;
	case 169:
		// smartEnergy current down step
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEDN_MASK, TMC2300_SEDN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEDN_MASK, TMC2300_SEDN_SHIFT, *value);
		}
		break;
	case 170:
		// smartEnergy hysteresis
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEMAX_MASK, TMC2300_SEMAX_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEMAX_MASK, TMC2300_SEMAX_SHIFT, *value);
		}
		break;
	case 171:
		// smartEnergy current up step
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEUP_MASK, TMC2300_SEUP_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEUP_MASK, TMC2300_SEUP_SHIFT, *value);
		}
		break;
	case 172:
		// smartEnergy hysteresis start
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEMIN_MASK, TMC2300_SEMIN_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SEMIN_MASK, TMC2300_SEMIN_SHIFT, *value);
		}
		break;
	case 174:
		// stallGuard2 threshold
		if(readWrite == READ) {
			//*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SGT_MASK, TMC2300_SGT_SHIFT);
			//*value = StepDir_getStallGuardThreshold(motor);
			*value = tmc2300_readInt(motorToIC(motor), TMC2300_SGTHRS);
			//*value = CAST_Sn_TO_S32(*value, 7);
		} else if(readWrite == WRITE) {
			tmc2300_writeInt(motorToIC(motor), TMC2300_SGTHRS, *value);
			//StepDir_setStallGuardThreshold(motor, *value);
			//TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_COOLCONF, TMC2300_SGT_MASK, TMC2300_SGT_SHIFT, *value);
		}
		break;
	case 180:
		// smartEnergy actual current
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_DRVSTATUS, TMC2300_CS_ACTUAL_MASK, TMC2300_CS_ACTUAL_SHIFT);
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

			// Convert the value for the TCOOLTHRS register
			// The IC only sends out Stallguard errors while TCOOLTHRS >= TSTEP >= TPWMTHRS
			// The TSTEP value is measured. To prevent measurement inaccuracies hiding
			// a stall signal, we decrease the needed velocity by roughly 12% before converting it.
			*value -= (*value) >> 3;
			if (*value)
			{
				*value = MIN(0x000FFFFF, (1<<24) / (*value));
			}
			else
			{
				*value = 0x000FFFFF;
			}
			tmc2300_writeInt(motorToIC(motor), TMC2300_TCOOLTHRS, *value);
		}
		break;
	case 182:
		// smartEnergy threshold speed
		if(readWrite == READ) {
			buffer = tmc2300_readInt(motorToIC(motor), TMC2300_TCOOLTHRS);
			*value = MIN(0xFFFFF, (1<<24) / ((buffer) ? buffer : 1));
		} else if(readWrite == WRITE) {
			*value = MIN(0xFFFFF, (1<<24) / ((*value) ? *value : 1));
			tmc2300_writeInt(motorToIC(motor), TMC2300_TCOOLTHRS, *value);
		}
		break;
	case 187:
		// PWM gradient
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_PWMCONF, TMC2300_PWM_GRAD_MASK, TMC2300_PWM_GRAD_SHIFT);
		} else if(readWrite == WRITE) {
			// Set gradient
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_PWMCONF, TMC2300_PWM_GRAD_MASK, TMC2300_PWM_GRAD_SHIFT, *value);

			// Enable/disable stealthChop accordingly
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_GCONF, TMC2300_EN_SPREADCYCLE_MASK, TMC2300_EN_SPREADCYCLE_SHIFT, (*value > 0) ? 0 : 1);
		}
		break;
	case 191:
		// PWM frequency
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_PWMCONF, TMC2300_PWM_FREQ_MASK, TMC2300_PWM_FREQ_SHIFT);
		} else if(readWrite == WRITE) {
			if(*value >= 0 && *value < 4)
			{
				TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_PWMCONF, TMC2300_PWM_FREQ_MASK, TMC2300_PWM_FREQ_SHIFT, *value);
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
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_PWMCONF, TMC2300_PWM_AUTOSCALE_MASK, TMC2300_PWM_AUTOSCALE_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_PWMCONF, TMC2300_PWM_AUTOSCALE_MASK, TMC2300_PWM_AUTOSCALE_SHIFT, (*value)? 1:0);
		}
		break;
	case 204:
		// Freewheeling mode
		if(readWrite == READ) {
			*value = TMC2300_FIELD_READ(motorToIC(motor), TMC2300_PWMCONF, TMC2300_FREEWHEEL_MASK, TMC2300_FREEWHEEL_SHIFT);
		} else if(readWrite == WRITE) {
			TMC2300_FIELD_WRITE(motorToIC(motor), TMC2300_PWMCONF, TMC2300_FREEWHEEL_MASK, TMC2300_FREEWHEEL_SHIFT, *value);
		}
		break;
	case 206:
		// Load value
		if(readWrite == READ) {
			*value = tmc2300_readInt(motorToIC(motor), TMC2300_SG_VALUE);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
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
	static uint32_t tick_old = 0;
	if((tick - tick_old) >= CONSISTENCY_CHECK_INTERVAL) {
		Evalboards.ch2.errors = FIELD_SET(Evalboards.ch2.errors, ERROR_INCONSISTENT_MASK, ERROR_INCONSISTENT_SHIFT, tmc2300_consistencyCheck(&TMC2300));
		tick_old = tick;
	}

	// Error detected -> disable driver for safety
	if(Evalboards.ch2.errors)
		enableDriver(DRIVER_DISABLE);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	uint32_t errors = 0;

	switch(type)
	{
	case 0:  // Read StepDir status bits
		*value = StepDir_getStatus(motor);
		break;
	case 3:
		restore();
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
	HAL.IOs->config->reset(Pins.DRV_EN);

	HAL.IOs->config->reset(Pins.STEP);
	HAL.IOs->config->reset(Pins.DIR);
	HAL.IOs->config->reset(Pins.MS1);
	HAL.IOs->config->reset(Pins.MS2);
	HAL.IOs->config->reset(Pins.DIAG);
	HAL.IOs->config->reset(Pins.STDBY);

	StepDir_deInit();
}

static void setStandby(uint8_t enableStandby)
{
	// En/disable the UART pins depending on standby state
	UART_setEnabled(TMC2300_UARTChannel, !enableStandby);

	HAL.IOs->config->setToState(Pins.STDBY, (enableStandby) ? IOS_LOW : IOS_HIGH);

	if (enableStandby)
	{
		// Just entered standby -> disable the driver
		enableDriver(DRIVER_DISABLE);
	}
	else
	{
		// ToDo: Needed?
		wait(10);
	}
	// Update the APIs internal standby state
	tmc2300_setStandby(&TMC2300, enableStandby);
}

static uint8_t onPinChange(IOPinTypeDef *pin, IO_States state)
{
	UNUSED(state);
	return !(pin == Pins.DRV_EN || pin == Pins.STDBY);
}

static uint8_t reset()
{
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, NULL);

	return tmc2300_reset(&TMC2300);
}

static uint8_t restore()
{
	return tmc2300_restore(&TMC2300);
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setLow(Pins.DRV_EN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setHigh(Pins.DRV_EN);
}

static void periodicJob(uint32_t tick)
{
	static uint32_t lastTick = 0;

	tmc2300_periodicJob(&TMC2300, tick);
	StepDir_periodicJob(0);

	if (tick - lastTick >= 1)
	{
		// Only do the IC read every ms. Otherwise the TMCL communication gets slowed drastically.
		StepDir_stallGuard(0, TMC2300_FIELD_READ(&TMC2300, TMC2300_IOIN, TMC2300_DIAG_MASK, TMC2300_DIAG_SHIFT) == 1);

		lastTick = tick;
	}
}

static void configCallback(TMC2300TypeDef *tmc2300, ConfigState state)
{
	UNUSED(tmc2300);

	if (state == CONFIG_RESET)
	{
		// Configuration reset completed
		// Change hardware preset registers here

		// Lower the default run and standstill currents
		tmc2300_writeInt(tmc2300, TMC2300_IHOLD_IRUN, 0x00010402);
	}
	else
	{
		// Configuration restore complete

		// The driver may only be enabled once the configuration is done
		enableDriver(DRIVER_USE_GLOBAL_ENABLE);
	}
}

void TMC2300_init(void)
{
	tmc_fillCRC8Table(0x07, true, 1);

	tmc2300_init(&TMC2300, 0, Evalboards.ch2.config, &tmc2300_defaultRegisterResetState[0]);
	tmc2300_setCallback(&TMC2300, configCallback);

	Pins.DRV_EN   = &HAL.IOs->pins->DIO0;
	Pins.DIAG     = &HAL.IOs->pins->DIO1;
	Pins.STDBY    = &HAL.IOs->pins->DIO2;
	Pins.MS1      = &HAL.IOs->pins->DIO3;
	Pins.MS2      = &HAL.IOs->pins->DIO4;
	Pins.MODE     = &HAL.IOs->pins->DIO5;
	Pins.STEP     = &HAL.IOs->pins->DIO6;
	Pins.DIR      = &HAL.IOs->pins->DIO7;

	HAL.IOs->config->toOutput(Pins.DRV_EN);
	HAL.IOs->config->toOutput(Pins.STEP);
	HAL.IOs->config->toOutput(Pins.DIR);
	HAL.IOs->config->toOutput(Pins.MS1);
	HAL.IOs->config->toOutput(Pins.MS2);
	HAL.IOs->config->toOutput(Pins.MODE);
	HAL.IOs->config->toOutput(Pins.STDBY);
	HAL.IOs->config->toInput(Pins.DIAG);

	HAL.IOs->config->setHigh(Pins.STDBY);
	HAL.IOs->config->setLow(Pins.MS1);
	HAL.IOs->config->setLow(Pins.MS2);
	HAL.IOs->config->setLow(Pins.MODE);

	TMC2300_UARTChannel = HAL.UART;
	TMC2300_UARTChannel->pinout = UART_PINS_2;
	TMC2300_UARTChannel->rxtx.init();

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
	Evalboards.ch2.writeRegister        = tmc2300_writeRegister;
	Evalboards.ch2.readRegister         = tmc2300_readRegister;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = 0; // Set to 0 instead of VM_MIN here since the VM supply isn't connected to the Landungsbruecke
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.onPinChange          = onPinChange;

	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(0, Pins.STEP, Pins.DIR, Pins.DIAG);
	StepDir_setVelocityMax(0, 51200);
	StepDir_setAcceleration(0, 51200);

	// Enter standby at the start
	setStandby(1);

	// The driver will be enabled in configCallback() once the IC is configured
	enableDriver(DRIVER_DISABLE);
};
