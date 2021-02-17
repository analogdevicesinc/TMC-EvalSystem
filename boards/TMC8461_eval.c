/*
 * TMC8461_eval.c
 *
 *  Created on: 30.05.2018
 *      Author: LK
 */

#include "Board.h"
#include "tmc/ic/TMC8461/TMC8461.h"

/*
 * 255 motors needed here to prevent errors in lower level when passing the motor number to register read/write functions.
 * Its used for 64 bit access to switch between low and high registers.
 */
#define TMC8461_MOTORS 0

#define TMC8461_MOTOR_LOW 0
#define TMC8461_MOTOR_HIGH 255

#define TMC8461_MFC(address) ((address) << 4)

typedef enum
{
	UF_PDI_RESET,
	UF_EEP_READ,
	UF_EEP_WRITE
} tmc8461_user_functions;

void TMC8461_init_ch1(void);
void TMC8461_init_ch2(void);
static void periodicJob(uint32_t actualSystick);
static void register_write(uint8_t motor, uint8_t address, int32_t value);
static void register_read(uint8_t motor, uint8_t address, int32_t *value);
static void memory_read(uint8_t motor, uint8_t address, int32_t *value);
static void memory_write(uint8_t motor, uint8_t address, int32_t value);
static void pdi_reset(void);
static uint32_t eep_read(int32_t *value);
static uint32_t eep_write(int32_t value);
static uint32_t user_function(uint8_t type, uint8_t motor, int32_t *value);
static void enableDriver(DriverState state);
static void deInit(void);
static uint8_t reset();
static uint8_t restore();
static void checkErrors(uint32_t tick);

static IOPinTypeDef *PIN_DRV_ENN;
TMC8461TypeDef tmc8461;
SPIChannelTypeDef *tmc8461_spi_mfc, *tmc8461_spi_esc;

/*
 * Usage note:
 * There are 2 possible modes to use the API for this chip.
 * 1. Use two different configurations for each channel and switch between ESC and MFC by using the API functions.
 * 2. Use a single configuration for channel 1 and use local wrapper, setting the chip select there on each r/w.
 *
 * Here the first method is used.
 */
uint8_t tmc8461_readWrite(uint8_t channel, uint8_t data, uint8_t lastTransfer)
{
	uint8_t out = 0;

	switch(channel)
	{
	case CHANNEL_1: // Channel 1 => readWrite on ESC
		out = tmc8461_spi_esc->readWrite(data, lastTransfer);
		break;
	case CHANNEL_2: // Channel 2 => readWrite on MFC
		out = tmc8461_spi_mfc->readWrite(data, lastTransfer);
		break;
	}

	return out;
}

static void periodicJob(uint32_t actualSystick)
{
	UNUSED(actualSystick);
}

/*
 * Register access types: 32-bit and 64-bit
 * default: 64-bit
 * 32-bit access on: TMC8461_MFC_SPI_RX_DATA, TMC8461_MFC_SPI_TX_DATA, TMC8461_MFC_PWM4
 */

static void register_write(uint8_t motor, uint8_t address, int32_t value)
{
	static uint8_t write_buffer[8];
	uint16_t _address = TMC8461_MFC(address);

	switch(motor)
	{
	case TMC8461_MOTOR_LOW:
		write_buffer[0] = BYTE(value, 0);
		write_buffer[1] = BYTE(value, 1);
		write_buffer[2] = BYTE(value, 2);
		write_buffer[3] = BYTE(value, 3);
		break;
	case TMC8461_MOTOR_HIGH:
		write_buffer[4] = BYTE(value, 0);
		write_buffer[5] = BYTE(value, 1);
		write_buffer[6] = BYTE(value, 2);
		write_buffer[7] = BYTE(value, 3);
		break;
	default:
		write_buffer[0] = BYTE(value, 0);
		write_buffer[1] = BYTE(value, 1);
		write_buffer[2] = BYTE(value, 2);
		write_buffer[3] = BYTE(value, 3);
		break;
	}

	tmc8461_mfc_write_auto(&tmc8461, _address, write_buffer);
}

static void register_read(uint8_t motor, uint8_t address, int32_t *value)
{
	static uint8_t readBuffer[8];

	address = TMC8461_MFC(address);

	tmc8461_mfc_read_auto(&tmc8461, address, readBuffer);

	switch (motor)
	{
	case TMC8461_MOTOR_LOW:
		*value = (readBuffer[3] << 24) | (readBuffer[2] << 16) | (readBuffer[1] << 8) | readBuffer[0];
		break;
	case TMC8461_MOTOR_HIGH:
		*value = (readBuffer[7] << 24) | (readBuffer[6] << 16) | (readBuffer[5] << 8) | readBuffer[4];
		break;
	default:
		*value = (readBuffer[3] << 24) | (readBuffer[2] << 16) | (readBuffer[1] << 8) | readBuffer[0];
		break;
	}
}

static void memory_read(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc8461_esc_read_16(&tmc8461, (motor << 8) | address);
}

static void memory_write(uint8_t motor, uint8_t address, int32_t value)
{
	tmc8461_esc_write_8(&tmc8461, (motor << 8) | address, BYTE(value, 0));
}

static void pdi_reset(void)
{
	tmc8461_esc_write_8(&tmc8461, TMC8461_ESC_RESET_PDI, TMC8461_MAGIC_RESET_0);
	tmc8461_esc_write_8(&tmc8461, TMC8461_ESC_RESET_PDI, TMC8461_MAGIC_RESET_1);
	tmc8461_esc_write_8(&tmc8461, TMC8461_ESC_RESET_PDI, TMC8461_MAGIC_RESET_2);
}

static uint32_t eep_read(int32_t *value)
{
	// Check if PDI has EEPROM control offered
	if (!TMC8461_FIELD_READ(&tmc8461, tmc8461_esc_read_8, TMC8461_ESC_EEP_CFG, TMC8461_ESC_EEP_PDI_MASK, TMC8461_ESC_EEP_PDI_SHIFT))
		return TMC_ERROR_CHIP;

	// PDI takes EEPROM control
	TMC8461_FIELD_WRITE(&tmc8461, tmc8461_esc_read_8, tmc8461_esc_write_8, TMC8461_ESC_EEP_PDI_ACCESS, TMC8461_ESC_PDI_ACCESS_MASK, TMC8461_ESC_PDI_ACCESS_SHIFT, true);

	// Wait until EEPROM is idle
	while (TMC8461_FIELD_READ(&tmc8461, tmc8461_esc_read_16, TMC8461_ESC_EEP_STATUS, TMC8461_ESC_EEP_BUSY_MASK, TMC8461_ESC_EEP_BUSY_SHIFT));

	// Write the read address
	tmc8461_esc_write_32(&tmc8461, TMC8461_ESC_EEP_ADDRESS, *value);

	// Send the read command
	TMC8461_FIELD_WRITE(&tmc8461, tmc8461_esc_read_16, tmc8461_esc_write_16, TMC8461_ESC_EEP_STATUS, TMC8461_ESC_EEP_CMD_MASK, TMC8461_ESC_EEP_CMD_SHIFT, TMC8461_ESC_EEP_CMD_READ);

	// Wait until EEPROM is idle
	while (TMC8461_FIELD_READ(&tmc8461, tmc8461_esc_read_16, TMC8461_ESC_EEP_STATUS, TMC8461_ESC_EEP_BUSY_MASK, TMC8461_ESC_EEP_BUSY_SHIFT));

	// Read the data
	*value = tmc8461_esc_read_32(&tmc8461, TMC8461_ESC_EEP_DATA);

	return TMC_ERROR_NONE;
}

static uint32_t eep_write(int32_t value)
{
	// Check if PDI has EEPROM control offered
	if (!TMC8461_FIELD_READ(&tmc8461, tmc8461_esc_read_8, TMC8461_ESC_EEP_CFG, TMC8461_ESC_EEP_PDI_MASK, TMC8461_ESC_EEP_PDI_SHIFT))
		return TMC_ERROR_CHIP;

	// PDI takes EEPROM control
	TMC8461_FIELD_WRITE(&tmc8461, tmc8461_esc_read_8, tmc8461_esc_write_8, TMC8461_ESC_EEP_PDI_ACCESS, TMC8461_ESC_PDI_ACCESS_MASK, TMC8461_ESC_PDI_ACCESS_SHIFT, true);

	// Wait until EEPROM is idle
	while (TMC8461_FIELD_READ(&tmc8461, tmc8461_esc_read_16, TMC8461_ESC_EEP_STATUS, TMC8461_ESC_EEP_BUSY_MASK, TMC8461_ESC_EEP_BUSY_SHIFT));

	// Write the address and data
	tmc8461_esc_write_32(&tmc8461, TMC8461_ESC_EEP_ADDRESS, SHORT(value, 1));
	tmc8461_esc_write_32(&tmc8461, TMC8461_ESC_EEP_DATA, SHORT(value, 0));

	// Send the write command
	TMC8461_FIELD_WRITE(&tmc8461, tmc8461_esc_read_16, tmc8461_esc_write_16, TMC8461_ESC_EEP_STATUS, TMC8461_ESC_EEP_CMD_MASK, TMC8461_ESC_EEP_CMD_SHIFT, TMC8461_ESC_EEP_CMD_WRITE);

	// Wait until EEPROM is idle
	while (TMC8461_FIELD_READ(&tmc8461, tmc8461_esc_read_16, TMC8461_ESC_EEP_STATUS, TMC8461_ESC_EEP_BUSY_MASK, TMC8461_ESC_EEP_BUSY_SHIFT));

	return TMC_ERROR_NONE;
}

static uint32_t user_function(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	uint32_t reply = TMC_ERROR_NONE;

	switch(type)
	{
	case UF_PDI_RESET:
		pdi_reset();
		break;
	case UF_EEP_READ:
		reply = eep_read(value);
		break;
	case UF_EEP_WRITE:
		reply = eep_write(*value);
		break;
	default:
		*value = TMC_ERROR_TYPE;
		reply = TMC_ERROR_TYPE;
		break;
	}

	return reply;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

static void deInit(void)
{

}

static uint8_t reset()
{
	return 1;
}

static uint8_t restore()
{
	return 1;
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);

	Evalboards.ch1.errors = 0;
	Evalboards.ch2.errors = 0;
}

void TMC8461_init_ch1(void)
{
	// configure ENABLE-PIN for TMC8690
	PIN_DRV_ENN = &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	HAL.IOs->config->setHigh(PIN_DRV_ENN);

	tmc8461_spi_esc       = &HAL.SPI->ch1;
	tmc8461_spi_esc->CSN  = &HAL.IOs->pins->SPI1_CSN;

	// connect evalboard functions
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_READY;
	Evalboards.ch1.config->configIndex  = 0;
	Evalboards.ch1.writeRegister        = register_write;
	Evalboards.ch1.readRegister         = register_read;
	Evalboards.ch1.periodicJob          = periodicJob;
	Evalboards.ch1.userFunction         = user_function;
	Evalboards.ch1.enableDriver         = enableDriver;
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = TMC8461_MOTORS;
	Evalboards.ch1.deInit               = deInit;
	Evalboards.ch1.VMMin                = 0;
	Evalboards.ch1.VMMax                = ~0;

	// Call this function manually here since ID detection on channel 2 does not work for this board.
	TMC8461_init_ch2();
}

void TMC8461_init_ch2(void)
{
	/*
	 * 2 Options (both use SPI2 chip select):
	 * 1. Setting tmc8461_spi_mfc to channel 1 and use SPI2_CSN0
	 * 2. Setting tmc8461_spi_mfc to channel 2 and use SPI2_CSN0
	 *
	 * Switchable in hardware by jumper.
	 */
	tmc8461_spi_mfc       = &HAL.SPI->ch2;
	tmc8461_spi_mfc->CSN  = &HAL.IOs->pins->SPI2_CSN0;

	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_READY; // Config instantly ready since we dont write anything for now
	Evalboards.ch2.config->configIndex  = 0;
	Evalboards.ch2.writeRegister        = memory_write;
	Evalboards.ch2.readRegister         = memory_read;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.userFunction         = user_function;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = TMC8461_MOTORS;
	Evalboards.ch2.deInit               = deInit;
	Evalboards.ch2.VMMin                = 0;
	Evalboards.ch2.VMMax                = ~0;

	tmc8461_initConfig(&tmc8461, Evalboards.ch1.config, Evalboards.ch2.config);
}
