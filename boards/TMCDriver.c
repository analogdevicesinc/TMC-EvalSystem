/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


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

	// A value of 0 indicates the Evalboard not connecting the VM line,
	// resulting in skipped global minimum voltage checks.
	// A negative value indicates no board being connected, which skips the
	// minimum voltage check for that channel
	Evalboards.ch2.VMMin                = -1;
	Evalboards.ch2.VMMax                = s32_MAX;

	Evalboards.ch2.numberOfMotors       = 0;
	Evalboards.ch2.errors               = 0;

	Evalboards.ch2.config->channel      = CHANNEL_2;

	board_setDummyFunctions(&Evalboards.ch2);
}
