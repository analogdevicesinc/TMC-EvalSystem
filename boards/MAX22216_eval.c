#include "boards/Board.h"
#include "tmc/ic/MAX22216/MAX22216.h"
#include "tmc/StepDir.h"
#include "tmc/RAMDebug.h"
#include "hal/Timer.h"

#undef  MAX22216_MAX_VELOCITY
#define MAX22216_MAX_VELOCITY  STEPDIR_MAX_VELOCITY

// Stepdir precision: 2^17 -> 17 digits of precision
#define STEPDIR_PRECISION (1 << 17)

#define VM_MIN  51   // VM[V/10] min
#define VM_MAX  241  // VM[V/10] max

#define MOTORS 1

#define TIMEOUT_VALUE 10 // 10 ms

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);

void MAX22216_init(void);
static void checkErrors (uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static void OTP_init(void);
static void OTP_address(uint32_t address);
static void OTP_value(uint32_t value);
static void OTP_program(void);
static OTP_Status OTP_status(void);
static uint8_t reset(void);
static void enableDriver(DriverState state);
static void timer_overflow(void);

static SPIChannelTypeDef *MAX22216_SPIChannel;
static MAX22216TypeDef MAX22216;

static bool rapid_fire_enabled[4] = { false, false, false, false };
static uint32_t overflow_count_on[4] = { 0, 0, 0, 0 };
static uint32_t overflow_count_off[4] = { 0, 0, 0, 0 };
static uint32_t overflow_count[4] = { 0, 0, 0, 0 };
static uint32_t rapid_fire_count_max[4] = { 0, 0, 0, 0 };
static uint32_t rapid_fire_count[4] = { 0, 0, 0, 0 };

static OTP_Status otp_status = OTP_STATUS_IDLE;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define MAX22216_CRC(data, length) tmc_CRC8(data, length, 1)

typedef struct
{
	IOPinTypeDef  *CNTL[4];
	IOPinTypeDef  *CRC_EN;
} PinsTypeDef;

typedef enum
{
	RAPID_FIRE_OFF = 0,
	RAPID_FIRE_ON = 1
} rapid_fire_state_t;

static rapid_fire_state_t rapid_fire_state[4] = { RAPID_FIRE_ON, RAPID_FIRE_ON, RAPID_FIRE_ON, RAPID_FIRE_ON };
static rapid_fire_state_t rapid_fire_stop_state[4] = { RAPID_FIRE_OFF, RAPID_FIRE_OFF, RAPID_FIRE_OFF, RAPID_FIRE_OFF };

static PinsTypeDef Pins;

static uint8_t restore(void);

static inline MAX22216TypeDef *motorToIC(uint8_t motor)
{
	UNUSED(motor);
	return &MAX22216;
}

static inline SPIChannelTypeDef *channelToSPI(uint8_t channel)
{
	UNUSED(channel);

	return MAX22216_SPIChannel;
}

void max22216_readWriteArray(uint8_t channel, uint8_t *data, size_t length)
{
	channelToSPI(channel)->readWriteArray(data, length);
}

// => CRC wrapper
// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t max22216_CRC8(uint8_t *data, size_t length)
{
	return MAX22216_CRC(data, length);
}
// <= CRC wrapper

void max22216_writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	max22216_writeInt(motorToIC(motor), address, value);
}

void max22216_readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = max22216_readInt(motorToIC(motor), address);
}

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	UNUSED(motor);
	UNUSED(velocity);
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
	UNUSED(motor);
	UNUSED(position);
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
	//int32_t buffer = 0;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 50: // Enable/disable rapid fire mode
		if (readWrite == READ) {
			*value = rapid_fire_enabled[motor] ? 1 : 0;
		} else {
			rapid_fire_enabled[motor] = false;
			if(*value) {
				HAL.IOs->config->toOutput(Pins.CNTL[motor]);
			} else {
				overflow_count[motor] = 0;
				rapid_fire_state[motor] = RAPID_FIRE_ON;
				rapid_fire_count[motor] = 0;
				switch(rapid_fire_stop_state[motor]) {
				case RAPID_FIRE_ON:
					HAL.IOs->config->setHigh(Pins.CNTL[motor]);
					break;
				case RAPID_FIRE_OFF:
				default:
					HAL.IOs->config->setLow(Pins.CNTL[motor]);
					break;
				}
			}
			rapid_fire_enabled[motor] = *value;
			for(size_t channel = 0; channel < ARRAY_SIZE(rapid_fire_enabled); channel++)
				if(rapid_fire_enabled[channel])
					Timer.setFrequency(6000);
		}
		break;
	case 51: // Rapid fire on-time
		if(readWrite == READ) {
			*value = overflow_count_on[motor];
		} else if(readWrite == WRITE) {
			overflow_count_on[motor] = *value;
		}
		break;
	case 52: // Rapid fire off-time
		if(readWrite == READ) {
			*value = overflow_count_off[motor];
		} else if(readWrite == WRITE) {
			overflow_count_off[motor] = *value;
		}
		break;
	case 53: // Rapid fire count
		if(readWrite == READ) {
			*value = rapid_fire_count_max[motor];
		} else if(readWrite == WRITE) {
			rapid_fire_count_max[motor] = *value;
		}
		break;
	case 54: // Rapid fire stop state
		if(readWrite == READ) {
			*value = rapid_fire_stop_state[motor];
		} else if(readWrite == WRITE) {
			rapid_fire_stop_state[motor] = *value;
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

	Evalboards.ch1.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	switch(type) {
	case 0:
		if(*value) {
			HAL.IOs->config->setHigh(Pins.CRC_EN);
			MAX22216.crc_en = 1;
		} else {
			HAL.IOs->config->setLow(Pins.CRC_EN);
			MAX22216.crc_en = 0;
		}
		return TMC_ERROR_NONE;
	}
	return TMC_ERROR_TYPE;
}

static void deInit(void)
{
}

static uint8_t reset()
{
	return 0;
}

static uint8_t restore()
{
	return 0;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);
}

static void OTP_init(void)
{
	// Activate the part
	MAX22216_FIELD_WRITE(motorToIC(0), 0x01, 0x8000, 15, 1);

	// Drive CRC Pin low
	HAL.IOs->config->setLow(Pins.CRC_EN);
	MAX22216.crc_en = 0;

	// Write preamble
	uint8_t data[3];
	data[0] = 0xFD;
	data[1] = 0x12;
	data[2] = 0xA7;
	max22216_readWriteArray(MAX22216.channel, data, 3);
	data[0] = 0xF8;
	data[1] = 0x00;
	data[2] = 0x1B;
	max22216_readWriteArray(MAX22216.channel, data, 3);
}

static void OTP_address(uint32_t address)
{
	MAX22216_FIELD_WRITE(motorToIC(0), 0x7C, 0xFFFF, 0, address);
}

static void OTP_value(uint32_t value)
{
	MAX22216_FIELD_WRITE(motorToIC(0), 0x7A, 0xFFFF, 0, value & 0xFF);
	MAX22216_FIELD_WRITE(motorToIC(0), 0x7B, 0xFFFF, 0, (value >> 8) & 0xFF);
}

static void OTP_program(void)
{
	MAX22216_FIELD_WRITE(motorToIC(0), 0x68, 0x0001, 0, 1);
	otp_status = OTP_STATUS_PROGRAMMING;
}

static OTP_Status OTP_status(void)
{
	if(otp_status == OTP_STATUS_PROGRAMMING) {
		if(MAX22216_FIELD_READ(motorToIC(0), 0x69, 0x0080, 7) == 1)
			otp_status = OTP_STATUS_DONE;
		if(MAX22216_FIELD_READ(motorToIC(0), 0x69, 0xFF7F, 0) != 0)
			otp_status = OTP_STATUS_FAILED;
	}
	return otp_status;
}

static void timer_overflow(void)
{
	// RAMDebug
	debug_nextProcess();

	// Rapid fire
	for(uint8_t motor = 0; motor < 4; motor++)
	{
		if(rapid_fire_enabled[motor])
		{
			switch(rapid_fire_state[motor]) {
			case RAPID_FIRE_ON:
				HAL.IOs->config->setHigh(Pins.CNTL[motor]);
				overflow_count[motor]++;
				if(overflow_count[motor] >= overflow_count_on[motor]) {
					rapid_fire_state[motor] = RAPID_FIRE_OFF;
					overflow_count[motor] = 0;
				}
				break;
			case RAPID_FIRE_OFF:
			default:
				HAL.IOs->config->setLow(Pins.CNTL[motor]);
				overflow_count[motor]++;
				if(overflow_count[motor] >= overflow_count_off[motor]) {
					rapid_fire_count[motor]++;
					if((rapid_fire_count_max[motor] != 0) && (rapid_fire_count[motor] >= rapid_fire_count_max[motor])) {
						rapid_fire_enabled[motor] = false;
						rapid_fire_count[motor] = 0;
						switch(rapid_fire_stop_state[motor]) {
						case RAPID_FIRE_ON:
							HAL.IOs->config->setHigh(Pins.CNTL[motor]);
							break;
						case RAPID_FIRE_OFF:
						default:
							HAL.IOs->config->setLow(Pins.CNTL[motor]);
							break;
						}
					}
					rapid_fire_state[motor] = RAPID_FIRE_ON;
					overflow_count[motor] = 0;
				}
				break;
			}
		}
	}
}

void MAX22216_init(void)
{
	tmc_fillCRC8Table(0x07, true, 1);
	Pins.CNTL[0] = &HAL.IOs->pins->DIO8;
	Pins.CNTL[1] = &HAL.IOs->pins->DIO9;
	Pins.CNTL[2] = &HAL.IOs->pins->DIO6;
	Pins.CNTL[3] = &HAL.IOs->pins->DIO7;
	Pins.CRC_EN = &HAL.IOs->pins->DIO4;

	MAX22216_SPIChannel = &HAL.SPI->ch2;
	MAX22216_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

	spi_setFrequency(MAX22216_SPIChannel, 12000000);

	HAL.IOs->config->toOutput(Pins.CNTL[0]);
	HAL.IOs->config->toOutput(Pins.CNTL[1]);
	HAL.IOs->config->toOutput(Pins.CNTL[2]);
	HAL.IOs->config->toOutput(Pins.CNTL[3]);

	HAL.IOs->config->setLow(Pins.CNTL[0]);
	HAL.IOs->config->setLow(Pins.CNTL[1]);
	HAL.IOs->config->setLow(Pins.CNTL[2]);
	HAL.IOs->config->setLow(Pins.CNTL[3]);

	HAL.IOs->config->toOutput(Pins.CRC_EN);

	HAL.IOs->config->setLow(Pins.CRC_EN);

	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_READY;
	Evalboards.ch2.config->configIndex  = 0;

	Evalboards.ch2.rotate               = rotate;
	Evalboards.ch2.right                = right;
	Evalboards.ch2.left                 = left;
	Evalboards.ch2.stop                 = stop;
	Evalboards.ch2.GAP                  = GAP;
	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.moveTo               = moveTo;
	Evalboards.ch2.moveBy               = moveBy;
	Evalboards.ch2.writeRegister        = max22216_writeRegister;
	Evalboards.ch2.readRegister         = max22216_readRegister;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.OTP_init             = OTP_init;
	Evalboards.ch2.OTP_address          = OTP_address;
	Evalboards.ch2.OTP_value            = OTP_value;
	Evalboards.ch2.OTP_program          = OTP_program;
	Evalboards.ch2.OTP_status           = OTP_status;

	max22216_init(&MAX22216, 0);

	Timer.overflow_callback = timer_overflow;
	Timer.init();

	wait(100);
}
