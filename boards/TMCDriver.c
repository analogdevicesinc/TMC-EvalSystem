#include "TMCDriver.h"

EvalBoardDriverTypeDef TMCDriver =
{
	.config	=
	{
		.state           = CONFIG_READY,
		.configIndex     = 0,
		.shadowRegister  = { 0 }
	}
};

void tmcdriver_init()
{
	Evalboards.ch2.config               = &TMCDriver.config;
	Evalboards.ch2.config->state        = CONFIG_READY;
	Evalboards.ch2.config->configIndex  = 0;

	// Set the minimum required voltage to 0.1V here.
	// A value of 0 indicates the Evalboard not connecting the VM line,
	// resulting in skipped global minimum voltage checks.
	Evalboards.ch2.VMMin                = 1;
	Evalboards.ch2.VMMax                = -1;

	Evalboards.ch2.numberOfMotors       = 0;
	Evalboards.ch2.errors               = 0;

	Evalboards.ch2.config->channel      = CHANNEL_2;

	board_setDummyFunctions(&Evalboards.ch2);
}
