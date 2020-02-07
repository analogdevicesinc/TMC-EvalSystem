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

	// A value of 0 indicates the Evalboard not connecting the VM line,
	// resulting in skipped global minimum voltage checks.
	// A negative value indicates no board being connected, which skips the
	// minimum voltage check for that channel
	Evalboards.ch1.VMMin                = -1;
	Evalboards.ch1.VMMax                = s32_MAX;

	Evalboards.ch1.numberOfMotors       = 0;
	Evalboards.ch1.errors               = 0;

	Evalboards.ch1.config->channel      = CHANNEL_1;

	board_setDummyFunctions(&Evalboards.ch1);
}
