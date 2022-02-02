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

static void dummy_AddressRef(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	UNUSED(address);
	UNUSED(value);
}

static void dummy_AddressValue(uint8_t motor, uint8_t address, int32_t value)
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

	channel->fullCover         = NULL;
	channel->getMin            = dummy_getLimit;
	channel->getMax            = dummy_getLimit;
	channel->onPinChange       = dummy_onPinChange;

	channel->OTP_init          = dummy_OTP_init;
	channel->OTP_address       = dummy_OTP_address;
	channel->OTP_value         = dummy_OTP_value;
	channel->OTP_program       = dummy_OTP_program;
	channel->OTP_status        = dummy_OTP_status;
}

void periodicJobDummy(uint32_t tick)
{
	UNUSED(tick);
}
