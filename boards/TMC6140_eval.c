#include "boards/Board.h"
#include "tmc/BLDC.h"

#define ERRORS_VM        (1<<0)
#define ERRORS_VM_UNDER  (1<<1)
#define ERRORS_VM_OVER   (1<<2)

#define VM_MIN  18   // VM[V/10] min
#define VM_MAX  330  // VM[V/10] max (30V + 10%)

#define MOTORS 1

// todo: Calculate this and describe the calculation instead of measuring it
#define CURRENT_SCALING_FACTOR 82382

#define TMC6140_UARTERROR_SHW         0x01
#define TMC6140_UARTERROR_SHV         0x02
#define TMC6140_UARTERROR_SHU         0x04
#define TMC6140_UARTERROR_S2G         0x08
#define TMC6140_UARTERROR_S2VS        0x10
#define TMC6140_UARTERROR_SC          0x20
#define TMC6140_UARTERROR_OTUV_MASK   0xC0
#define TMC6140_UARTERROR_OTUV_SHIFT  6

#define ERROR_SHORT_U_GND           0x00000001
#define ERROR_SHORT_V_GND           0x00000002
#define ERROR_SHORT_W_GND           0x00000004
#define ERROR_SHORT_U_VS            0x00000008
#define ERROR_SHORT_V_VS            0x00000010
#define ERROR_SHORT_W_VS            0x00000020
#define ERROR_3V3_REGULATOR         0x00000040
#define ERROR_OVERTEMPERATURE       0x00000080
#define ERROR_OVERTEMPERATURE_WARN  0x00000100
#define ERROR_UNDERVOLTAGE          0x00000200

uint8_t standbyOnDrvOff = 1;

// Timestamp for delayed ADC calibration after driver enable.
// A value of 0 indicates an invalid timestamp.
uint32_t enableTimestamp = 0;

typedef enum {
	SEARCHING_STOP_SEQUENCE,
	SYNC_START_BIT,
	SYNC_DATA_0_6,
	SYNC_DATA_7,
	SYNC_STOP_BIT,
	ERROR_DATA_BITS,

	SEQUENCE_ERROR,
} DiagUARTStates;

DiagUARTStates uartState = SEARCHING_STOP_SEQUENCE;

uint32_t uartStatus = 0;

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);

static void checkErrors (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static uint8_t reset(void);
static void enableDriver(DriverState state);

static void resetUARTErrors(uint32_t mask);
static uint32_t decodeUARTErrorFlags(uint32_t errorBits);

typedef struct
{
	IOPinTypeDef  *HALL_U;
	IOPinTypeDef  *HALL_V;
	IOPinTypeDef  *HALL_W;
	IOPinTypeDef  *LED_ON;
	IOPinTypeDef  *DRV_STRENGTH;
	IOPinTypeDef  *GAIN;
	IOPinTypeDef  *BRAKE_0;
	IOPinTypeDef  *BRAKE_1;
	IOPinTypeDef  *DIAG;
	IOPinTypeDef  *DRV_EN;
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
	case 3: // Target PWM
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
	case 4: // Actual PWM
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
	case 6:  // Measured current
		if (readWrite == READ)
		{
			if (BLDC_isPWMenabled() && HAL.IOs->config->isHigh(Pins.DRV_EN) && enableTimestamp == 0)
			{
				*value = BLDC_getMeasuredCurrent();
			}
			else
			{
				errors |= TMC_ERROR_NOT_DONE;
				*value = 0;
			}
		}
		break;
	case 7: // hall invert
		if (readWrite == READ)
		{
			*value = BLDC_getHallInvert();
		}
		else
		{
			BLDC_setHallInvert(*value);
		}
		break;
	case 8: // hall order
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
	case 9: // UART error status
		if (readWrite == READ)
		{
			*value = uartStatus;
		}
		else
		{
			resetUARTErrors(*value);
		}
		break;
	case 10: // Standby on driver disable
		if (readWrite == READ)
		{
			*value = standbyOnDrvOff;
		}
		else
		{
			standbyOnDrvOff = (*value)? 1:0;
		}
		break;
	case 11: // DrvStrength pin control
		if (readWrite == READ)
		{
			*value = HAL.IOs->config->isHigh(Pins.DRV_STRENGTH)? 1000:500;
		}
		else
		{
			if (*value == 500)
			{
				HAL.IOs->config->setLow(Pins.DRV_STRENGTH);
			}
			else if (*value == 1000)
			{
				HAL.IOs->config->setHigh(Pins.DRV_STRENGTH);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 12: // Brake pins control
		if (readWrite == READ)
		{
			uint8_t brake0 = HAL.IOs->config->isHigh(Pins.BRAKE_0);
			uint8_t brake1 = HAL.IOs->config->isHigh(Pins.BRAKE_1);

			*value = ((brake0)? 1:0) + ((brake1)? 1:0);
		}
		else
		{
			if (*value >= 0 && *value < 3)
			{
				HAL.IOs->config->setToState(Pins.BRAKE_0, (*value > 0)? IOS_HIGH:IOS_LOW);
				HAL.IOs->config->setToState(Pins.BRAKE_1, (*value > 1)? IOS_HIGH:IOS_LOW);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 13: // RSense amplifier gain
		if (readWrite == READ)
		{
			*value = HAL.IOs->config->isHigh(Pins.GAIN)? 20:50;
		}
		else
		{
			if (*value == 50)
			{
				HAL.IOs->config->setToState(Pins.GAIN, IOS_LOW);
			}
			else if (*value == 20)
			{
				HAL.IOs->config->setToState(Pins.GAIN, IOS_HIGH);
			}
			else
			{
				errors |= TMC_ERROR_VALUE;
			}
		}
		break;
	case 14: // LED pin control
		if (readWrite == READ)
		{
			*value = HAL.IOs->config->isHigh(Pins.LED_ON);
		}
		else
		{
			HAL.IOs->config->setToState(Pins.LED_ON, (*value)? IOS_HIGH:IOS_LOW);
		}
		break;
	case 15: // BBM time control [1/48MHz]
		if (readWrite == READ)
		{
			*value = BLDC_getBBMTime();
		}
		else
		{
			BLDC_setBBMTime(*value);
		}
		break;
	case 16: // Actual hall velocity [RPM]
		if (readWrite == READ)
		{
			*value = BLDC_getActualHallVelocity();
		}
		else
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 17: // Target openloop velocity [RPM]
		if (readWrite == READ)
		{
			*value = BLDC_getTargetOpenloopVelocity();
		}
		else
		{
			BLDC_setTargetOpenloopVelocity(*value);
		}
		break;
	case 18: // Actual openloop velocity [RPM]
		if (readWrite == READ)
		{
			*value = BLDC_getActualOpenloopVelocity();
		}
		else
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 19: // Motor pole pairs
		if (readWrite == READ)
		{
			*value = BLDC_getPolePairs();
		}
		else
		{
			BLDC_setPolePairs(*value);
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

	// Re-enable the previously disabled timer module
	Timer.init();
}

static uint8_t reset()
{
	BLDC_setTargetPWM(0);
	BLDC_setCommutationMode(BLDC_OPENLOOP);

	return 0;
}

static uint8_t restore()
{
	resetUARTErrors(~0);

	BLDC_calibrateADCs();

	return 0;
}

static void resetUARTErrors(uint32_t mask)
{
	// Clear hardware errors by disabling the drivers for 10ms
	uint8_t oldState = HAL.IOs->config->isHigh(Pins.DRV_EN);
	HAL.IOs->config->setLow(Pins.DRV_EN);
	wait(10);
	HAL.IOs->config->setToState(Pins.DRV_EN, oldState? IOS_HIGH:IOS_LOW);

	// Clear the error bits
	uartStatus &= ~mask;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
	{
		if (standbyOnDrvOff)
		{
			// Full standby
			BLDC_enablePWM(false);
		}

		HAL.IOs->config->setLow(Pins.DRV_EN);
	}
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
	{
		BLDC_enablePWM(false);

		HAL.IOs->config->setLow(Pins.BRAKE_0);
		HAL.IOs->config->setLow(Pins.BRAKE_1);
		wait(10);

		HAL.IOs->config->setHigh(Pins.DRV_EN);
		wait(10);

		BLDC_enablePWM(true);
		wait(10);

		HAL.IOs->config->setHigh(Pins.BRAKE_0);
		HAL.IOs->config->setHigh(Pins.BRAKE_1);

		// Store the enable time to allow the delayed ADC calibration
		// The 0 value is used as a special case to indicate the calibration was
		// already done. In that case, store the next tick value (a 1) instead.
		enableTimestamp = systick_getTick();
		if (enableTimestamp == 0)
		{
			enableTimestamp++;
		}
	}
}

static void periodicJob(uint32_t tick)
{
	if (enableTimestamp != 0 && timeDiff(tick, enableTimestamp) > 5000)
	{
		// Clear the UART error
		uartStatus = 0;

		BLDC_calibrateADCs();
		// Mark the timestamp as invalid
		enableTimestamp = 0;
	}
}

// Interrupt: Edge on GPIO Port C
void PORTC_IRQHandler(void)
{
	static uint32_t bitTime = 0;
	static uint32_t timestamp_sync_start = 0;
	static uint32_t timestamp_data_start = 0;
	static uint32_t data_time = 0;
	static uint8_t statusBits = 0;
	static uint8_t statusBitCount = 0;

	static uint32_t lastTime = 0;

	// Clear the interrupt
	PORTC_ISFR = PORT_ISFR_ISF_MASK;

	// Store the edge type
	uint8_t isRisingEdge = HAL.IOs->config->isHigh(Pins.DIAG);

	// Calculate the time since the last edge
	uint32_t time = systick_getMicrosecondTick();
	uint32_t timeDiff = time - lastTime;
	lastTime = time;

	uint32_t timeSinceStart = time - timestamp_data_start;

	switch (uartState)
	{
	case SEARCHING_STOP_SEQUENCE: // Falling edge
		// We need to find 44 high bits
		// At 9600 Hz this is ~4583µs
		// At 14kHz this is ~3143µs
		// If we have a falling edge, the previous bits were high
		if ((timeDiff > 3000) && (!isRisingEdge))
		{
			uartState = SYNC_START_BIT;
			timestamp_sync_start = time;
		}
		break;
	case SYNC_START_BIT: // Rising edge
		// Verify edge type
		if (!isRisingEdge)
		{
			uartState = SEQUENCE_ERROR;
			break;
		}

		uartState = SYNC_DATA_0_6;
		break;
	case SYNC_DATA_0_6: // Falling edge
		// Verify edge type
		if (isRisingEdge)
		{
			uartState = SEQUENCE_ERROR;
			break;
		}

		// Calculate the time for each bit by averaging over the last 8 bits
		bitTime = (time - timestamp_sync_start)/8;
		uartState = SYNC_DATA_7;
		break;
	case SYNC_DATA_7: // Rising edge
		// Verify edge type
		if (!isRisingEdge)
		{
			uartState = SEQUENCE_ERROR;
			break;
		}

		// Check that the bit time roughly matches
		if (abs(timeDiff - bitTime) < bitTime/2)
		{
			uartState = SYNC_STOP_BIT;
		}
		else
		{
			uartState = SEQUENCE_ERROR;
		}
		break;
	case SYNC_STOP_BIT: // Falling edge
		// Verify edge type
		if (isRisingEdge)
		{
			uartState = SEQUENCE_ERROR;
			break;
		}

		// Check that the bit time roughly matches
		if (abs(timeDiff - bitTime) < bitTime/2)
		{
			timestamp_data_start = time;
			data_time = bitTime * 3 / 2;
			uartState = ERROR_DATA_BITS;
			statusBits = 0;
			statusBitCount = 0;
		}
		else
		{
			uartState = SEQUENCE_ERROR;
		}
		break;
	case ERROR_DATA_BITS: // Rising/Falling edges
		for (; data_time < timeSinceStart; data_time+=bitTime)
		{
			// Append a bit of error data
			statusBits <<=1;
			statusBits |= isRisingEdge? 0:1;
			statusBitCount++;
			if (statusBitCount >= 8)
			{
				// Once we collected 8 bits, we're done reading the error data
				// Update the software status flags with the received error data
				uartStatus = decodeUARTErrorFlags(statusBits);

				// Note that if the final bit of the datagram is a 1,
				// we will reach this part once the sync start bit of the next
				// datagram occurs - after the stop bit sequence.
				// As a result we will only observe every other datagram in this case.
				// This is fine as the datagram repeats anyways.
				uartState = SEARCHING_STOP_SEQUENCE;

				break;
			}
		}
		break;
	case SEQUENCE_ERROR:
		// Reset back to the initial state
		uartState = SEARCHING_STOP_SEQUENCE;
		break;
	}
}

static uint32_t decodeUARTErrorFlags(uint32_t errorBits)
{
	uint32_t status = 0;

	// Helper variables for readability
	uint32_t isShortToGND = errorBits & TMC6140_UARTERROR_S2G;
	uint32_t isShortToVS  = errorBits & TMC6140_UARTERROR_S2VS;
	uint32_t isUShorted   = errorBits & TMC6140_UARTERROR_SHU;
	uint32_t isVShorted   = errorBits & TMC6140_UARTERROR_SHV;
	uint32_t isWShorted   = errorBits & TMC6140_UARTERROR_SHW;
	uint32_t otuv         = FIELD_GET(errorBits, TMC6140_UARTERROR_OTUV_MASK, TMC6140_UARTERROR_OTUV_SHIFT);

	if (isUShorted && isShortToGND) status |= ERROR_SHORT_U_GND;
	if (isVShorted && isShortToGND) status |= ERROR_SHORT_V_GND;
	if (isWShorted && isShortToGND) status |= ERROR_SHORT_W_GND;
	if (isUShorted && isShortToVS)  status |= ERROR_SHORT_U_VS;
	if (isVShorted && isShortToVS)  status |= ERROR_SHORT_V_VS;
	if (isWShorted && isShortToVS)  status |= ERROR_SHORT_W_VS;

	if (errorBits & TMC6140_UARTERROR_SC)
	{
		status |= ERROR_3V3_REGULATOR;
	}

	switch (otuv)
	{
	case 3:
		status |= ERROR_OVERTEMPERATURE;
		status |= ERROR_OVERTEMPERATURE_WARN;
		break;
	case 2:
		status |= ERROR_UNDERVOLTAGE;
		break;
	case 1:
		status |= ERROR_OVERTEMPERATURE_WARN;
		break;
	case 0:
		break;
	}

	return status;
}

void TMC6140_init(void)
{
	Pins.DRV_EN       = &HAL.IOs->pins->DIO0;
	Pins.HALL_U       = &HAL.IOs->pins->DIO2;
	Pins.HALL_V       = &HAL.IOs->pins->DIO3;
	Pins.HALL_W       = &HAL.IOs->pins->DIO4;
	Pins.DIAG         = &HAL.IOs->pins->DIO12;
	Pins.GAIN         = &HAL.IOs->pins->DIO13;
	Pins.BRAKE_1      = &HAL.IOs->pins->DIO14;
	Pins.DRV_STRENGTH = &HAL.IOs->pins->DIO15;
	Pins.BRAKE_0      = &HAL.IOs->pins->DIO16;
	Pins.LED_ON       = &HAL.IOs->pins->DIO17;

	HAL.IOs->config->toOutput(Pins.DRV_EN);
	HAL.IOs->config->toOutput(Pins.GAIN);
	HAL.IOs->config->toOutput(Pins.BRAKE_0);
	HAL.IOs->config->toOutput(Pins.BRAKE_1);
	HAL.IOs->config->toOutput(Pins.DRV_STRENGTH);
	HAL.IOs->config->toOutput(Pins.LED_ON);

	HAL.IOs->config->setLow(Pins.DRV_STRENGTH);

	HAL.IOs->config->setHigh(Pins.LED_ON);

	HAL.IOs->config->toInput(Pins.HALL_U);
	HAL.IOs->config->toInput(Pins.HALL_V);
	HAL.IOs->config->toInput(Pins.HALL_W);

	Evalboards.ch2.config->state        = CONFIG_READY;
	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;

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

	BLDC_init(MEASURE_THREE_PHASES, CURRENT_SCALING_FACTOR, Pins.HALL_U, Pins.HALL_V, Pins.HALL_W);

	// Configure DIAG Pin
	HAL.IOs->config->toInput(Pins.DIAG);

	// Enable GPIO, edge-triggered interrupt, and PullUp resistor
	PORT_PCR_REG(Pins.DIAG->portBase, Pins.DIAG->bit)  = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	// Clear interrupt flags
	PORTC_ISFR = -1;

	// Enable interrupt
	enable_irq(INT_PORTC - 16);

	enableDriver(DRIVER_ENABLE);
};
