#include "TMCMotionController.h"

EvalBoardMotionControllerTypeDef TMCMotionController =
{
	.config	=
	{
		.state           = CONFIG_READY,
		.configIndex     = 0,
		.shadowRegister  = { 0 }
	}
};

void tmcmotioncontroller_init()
{
	Evalboards.ch1.config               = &TMCMotionController.config;
	Evalboards.ch1.config->state        = CONFIG_READY;
	Evalboards.ch1.config->configIndex  = 0;

	// Set the minimum required voltage to 0.1V here.
	// A value of 0 indicates the Evalboard not connecting the VM line,
	// resulting in skipped global minimum voltage checks.
	Evalboards.ch1.VMMin                = 1;
	Evalboards.ch1.VMMax                = -1;

	Evalboards.ch1.numberOfMotors       = 0;
	Evalboards.ch1.errors               = 0;

	Evalboards.ch1.config->channel      = CHANNEL_1;

	board_setDummyFunctions(&Evalboards.ch1);
}
