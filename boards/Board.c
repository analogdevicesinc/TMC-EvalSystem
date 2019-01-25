#include "Board.h"

static void deInit(void) {}

// Evalboard channel function dummies
static uint32 dummy_Motor(uint8 motor)
{
	UNUSED(motor);
	return TMC_ERROR_FUNCTION;
}

static uint32 dummy_MotorValue(uint8 motor, int32 value)
{
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static void dummy_AddressRef(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	UNUSED(address);
	UNUSED(value);
}

static void dummy_AddressValue(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	UNUSED(address);
	UNUSED(value);
}

static uint32 dummy_MotorRef(uint8 motor, int32 *value)
{
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint32 dummy_TypeMotorValue(uint8 type, uint8 motor, int32 value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint32 dummy_TypeMotorRef(uint8 type, uint8 motor, int32 *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint32 dummy_getLimit(uint8 type, uint8 motor, int32 *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return TMC_ERROR_FUNCTION;
}

static uint8 delegationReturn(void)
{
	return 1;
}

static void enableDriver(DriverState state)
{
	UNUSED(state);
}

static void periodicJob(uint32 tick)
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
	channel->STAP              = dummy_TypeMotorValue;
	channel->RSAP              = dummy_TypeMotorValue;
	channel->userFunction      = dummy_TypeMotorRef;
	channel->getMeasuredSpeed  = dummy_MotorRef;
	channel->checkErrors       = periodicJob;
	channel->enableDriver      = enableDriver;

	channel->fullCover         = NULL;
	channel->getMin            = dummy_getLimit;
	channel->getMax            = dummy_getLimit;
}

void periodicJobDummy(uint32 tick)
{
	UNUSED(tick);
}
