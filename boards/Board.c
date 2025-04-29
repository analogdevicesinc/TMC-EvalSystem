/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "Board.h"

static void deInit(void) {}

// Evalboard channel function dummies
static uint32_t dummy_Motor(uint8_t motor)
{
	UNUSED(motor);
	return TMC_ERROR_FUNCTION;
}

static uint32_t dummy_MotorValue(uint8_t motor, int32_t value)
{
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static void dummy_AddressRef(uint8_t motor, uint16_t address, int32_t *value)
{
	UNUSED(motor);
	UNUSED(address);
	UNUSED(value);
}

static void dummy_AddressValue(uint8_t motor, uint16_t address, int32_t value)
{
	UNUSED(motor);
	UNUSED(address);
	UNUSED(value);
}

static uint32_t dummy_MotorRef(uint8_t motor, int32_t *value)
{
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint32_t dummy_TypeMotorValue(uint8_t type, uint8_t motor, int32_t value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint32_t dummy_TypeMotorRef(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint32_t dummy_getLimit(uint8_t type, uint8_t motor, int32_t *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint8_t dummy_onPinChange(IOPinTypeDef *pin, IO_States state)
{
	UNUSED(pin);
	UNUSED(state);
	return 1;
}

static void dummy_OTP_init(void)
{
	return;
}

static void dummy_OTP_address(uint32_t address)
{
	UNUSED(address);
	return;
}

static void dummy_OTP_value(uint32_t value)
{
	UNUSED(value);
	return;
}

static void dummy_OTP_program(void)
{
	return;
}

static void dummy_OTP_lock(void)
{
	return;
}

static OTP_Status dummy_OTP_status(void)
{
	return OTP_STATUS_IDLE;
}

static uint8_t delegationReturn(void)
{
	return 1;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

static void periodicJob(uint32_t tick)
{
	UNUSED(tick);
}

void board_setDummyFunctions(EvalboardFunctionsTypeDef *channel)
{
	channel->config->reset     = delegationReturn;
	channel->config->restore   = delegationReturn;

	channel->deInit            = deInit;
	channel->periodicJob       = periodicJob;
	channel->left              = dummy_MotorValue;
	channel->stop              = dummy_Motor;
	channel->moveTo            = dummy_MotorValue;
	channel->moveBy            = dummy_MotorRef;
	channel->moveProfile       = dummy_MotorValue;
	channel->right             = dummy_MotorValue;
	channel->GAP               = dummy_TypeMotorRef;
	channel->readRegister      = dummy_AddressRef;
	channel->writeRegister     = dummy_AddressValue;
	channel->SAP               = dummy_TypeMotorValue;
	channel->SIO               = dummy_TypeMotorValue;
	channel->GIO               = dummy_TypeMotorRef;
	channel->STAP              = dummy_TypeMotorValue;
	channel->RSAP              = dummy_TypeMotorValue;
	channel->userFunction      = dummy_TypeMotorRef;
	channel->getMeasuredSpeed  = dummy_MotorRef;
	channel->checkErrors       = periodicJob;
	channel->enableDriver      = enableDriver;
	channel->fwdTmclCommand    = NULL;

	channel->fullCover         = NULL;
	channel->getMin            = dummy_getLimit;
	channel->getMax            = dummy_getLimit;
	channel->onPinChange       = dummy_onPinChange;

	channel->OTP_init          = dummy_OTP_init;
	channel->OTP_address       = dummy_OTP_address;
	channel->OTP_value         = dummy_OTP_value;
	channel->OTP_program       = dummy_OTP_program;
	channel->OTP_status        = dummy_OTP_status;
	channel->OTP_lock          = dummy_OTP_lock;
}

void periodicJobDummy(uint32_t tick)
{
	UNUSED(tick);
}

// CRC table used by TMC-API
const uint8_t tmcCRCTable_Poly7Reflected[256] = {
        0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
        0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
        0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
        0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
        0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
        0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
        0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
        0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
        0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
        0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
        0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
        0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
        0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
        0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
        0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
        0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
};
