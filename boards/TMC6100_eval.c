/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "boards/Board.h"
#include "tmc/ic/TMC6100/TMC6100.h"

#define VM_MIN         50   // VM[V/10] min
#define VM_MAX         550  // VM[V/10] max

#define DEFAULT_MOTOR 0
#define DEFAULT_ICID  0

// use this define for TMC4671-TMC6100-BOB
//#define COMPILE_FOR_TMC4671_TMC6100_BOB

static uint32_t right(uint8_t motor, int32_t velocity);
static uint32_t left(uint8_t motor, int32_t velocity);
static uint32_t rotate(uint8_t motor, int32_t velocity);
static uint32_t stop(uint8_t motor);
static uint32_t moveTo(uint8_t motor, int32_t position);
static uint32_t moveBy(uint8_t motor, int32_t *ticks);
static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value);
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value);
static void readRegister(uint8_t motor, uint16_t address, int32_t *value);
static void writeRegister(uint8_t motor, uint16_t address, int32_t value);
static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value);

static void periodicJob(uint32_t tick);
static void checkErrors(uint32_t tick);
static void deInit(void);
static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value);

static uint8_t reset();
static void enableDriver(DriverState state);

static SPIChannelTypeDef *TMC6100_SPIChannel;
IOPinTypeDef *TMC6100_SPIchipSelect = NULL;

void tmc6100_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
   UNUSED(icID);

   IOPinTypeDef *tmp = NULL;

   // For the BOB version:
   // Do not override the CSN in the SPIChannel - this would break the motion controller using SPI1
   // Instead store the pin in a separate variable
   if (TMC6100_SPIchipSelect)
   {
       // Swap to the TMC6100's CSN
       tmp = TMC6100_SPIChannel->CSN;
       TMC6100_SPIChannel->CSN = TMC6100_SPIchipSelect;
   }
   TMC6100_SPIChannel->readWriteArray(data, dataLength);

   if (TMC6100_SPIchipSelect)
   {
       // Swap back to the original CSN
       TMC6100_SPIChannel->CSN = tmp;
   }
}

typedef struct
{
	IOPinTypeDef  *REFL_UC;
	IOPinTypeDef  *REFR_UC;
	IOPinTypeDef  *DRV_ENN_CFG6;
	IOPinTypeDef  *ENCA_DCIN_CFG5;
	IOPinTypeDef  *ENCB_DCEN_CFG4;
	IOPinTypeDef  *ENCN_DCO;
	IOPinTypeDef  *SD_MODE;
	IOPinTypeDef  *SPI_MODE;
	IOPinTypeDef  *SWN_DIAG0;
	IOPinTypeDef  *SWP_DIAG1;
} PinsTypeDef;

//static PinsTypeDef Pins;

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	UNUSED(velocity);

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

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
	UNUSED(position);

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	return moveTo(motor, *ticks);
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	UNUSED(readWrite);
	UNUSED(value);

	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
		// add parameters if needed
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

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	UNUSED(value);

	if(motor >= TMC6100_MOTORS)
		return TMC_ERROR_MOTOR;

	return TMC_ERROR_NONE;
}

static void writeRegister(uint8_t motor, uint16_t address, int32_t value)
{
	UNUSED(motor);
	tmc6100_writeRegister(DEFAULT_ICID, (uint8_t) address, value);
}

static void readRegister(uint8_t motor, uint16_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc6100_readRegister(DEFAULT_ICID, (uint8_t) address);
}

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch2.errors = 0;
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);

	return 0;
}

static void deInit(void)
{
};

static uint8_t reset()
{
	// set default PWM configuration for evaluation board use with TMC467x-EVAL
    tmc6100_writeRegister(DEFAULT_ICID, TMC6100_GCONF, 0x40);

	return 1;
}

static uint8_t restore()
{
	// set default PWM configuration for evaluation board use with TMC467x-EVAL
    tmc6100_writeRegister(DEFAULT_ICID, TMC6100_GCONF, 0x40);

	return 1;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

void TMC6100_init(void)
{
	TMC6100_SPIChannel = &HAL.SPI->ch2;
	TMC6100_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

#ifdef COMPILE_FOR_TMC4671_TMC6100_BOB

	#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
		TMC6100_SPIChannel->periphery       = SPI1_BASE_PTR;
	#endif

#endif

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
	Evalboards.ch2.writeRegister        = writeRegister;
	Evalboards.ch2.readRegister         = readRegister;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = TMC6100_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;

	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6100_writeRegister(DEFAULT_ICID, TMC6100_GCONF, 0x40);

	enableDriver(DRIVER_USE_GLOBAL_ENABLE);
}

// This function is used to initialize the TMC6100 when it is part of the TMC4671+TMC6100-BOB.
// The only difference to a TMC6100-Eval is the SPI channel - instead of SPI2, SPI1 is used.
// To use the TMC4671+TMC6100-BOB with the Evaluation system, connect the BOB like this:
//   Eselsbruecke | BOB
//   5V_USB       | +5V
//   GND          | GND
//   SPI1_CSN     | CS_CTRL
//   SPI2_CSN0    | CS_DRV
//   SPI1_SCK     | SPI_SCK
//   SPI1_SDI     | SPI_MOSI
//   SPI1_SDO     | SPI_MISO
//   DIO0         | CTRL_EN
// Additionally you need to connect the following BOB pins:
//   - 3V3: Connect this to a 3V3 regulator output
//   - VM, GND: Connect this to your power supply
//   - U, V, W: Connect this to your BLDC motor
// All other BOB pins are optional

void TMC6100_BOB_init(void)
{
	// Run the normal init first...
	TMC6100_init();

	// ...then override the SPI channel to use channel 1
	TMC6100_SPIChannel = &HAL.SPI->ch1;
	// Do not override the CSN in the SPIChannel - this would break the motion controller using SPI1
	// Instead store the pin in a separate variable
	TMC6100_SPIchipSelect = &HAL.IOs->pins->SPI2_CSN0;


	spi_setFrequency(TMC6100_SPIChannel, 1000000);

}
