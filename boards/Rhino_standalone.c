#include "Rhino_standalone.h"

#include "Board.h"
#include "tmc/ic/TMC5130/TMC5130_Register.h"

static uint32 getInt();
static void set();
static void reset();
static void setConfig(TMCRhinoTypeStandAloneConfigDef *config);
static void getConfig(TMCRhinoTypeStandAloneConfigDef *config);
static void setPins(IO_States *CFG);
static void getPins(IO_States *CFG);
static void setInt(uint32 state);

static IOPinTypeDef IODummy = { .bitWeight = DUMMY_BITWEIGHT };
static IOPinTypeDef *CFGPins[] = { &IODummy, &IODummy, &IODummy, &IODummy, &IODummy, &IODummy, &IODummy};

static IO_States settings[7] = {IOS_LOW, IOS_LOW, IOS_LOW, IOS_LOW, IOS_LOW, IOS_LOW, IOS_LOW};

TMCRhinoTypeStandAloneDef TMCRhinoSA =
{
	.reset      = reset,
	.setConfig  = setConfig,
	.getConfig  = getConfig,
	.setPins    = setPins,
	.getPins    = getPins,
	.setInt     = setInt,
	.getInt     = getInt,

	.MicrostepResolutionSettings1 =
	{ //	CFG1
		.BY1_INTERPOL0              = IOS_LOW,
		.BY2_INTERPOL0              = IOS_HIGH,
		.BY2_INTERPOL256            = IOS_OPEN,
		.BY4_INTERPOL0              = IOS_LOW,
		.BY4_INTERPOL256            = IOS_OPEN,
		.BY16_INTERPOL0             = IOS_HIGH,
		.BY16_INTERPOL256           = IOS_LOW,
		.PWM_CHOP_BY4_INTERPOL256   = IOS_HIGH,
		.PWM_CHOP_BY16_INTERPOL256  = IOS_OPEN
	},

	.MicrostepResolutionSettings2 =
	{ // CFG2
		.BY1_INTERPOL0              = IOS_LOW,
		.BY2_INTERPOL0              = IOS_LOW,
		.BY2_INTERPOL256            = IOS_LOW,
		.BY4_INTERPOL0              = IOS_HIGH,
		.BY4_INTERPOL256            = IOS_HIGH,
		.BY16_INTERPOL0             = IOS_HIGH,
		.BY16_INTERPOL256           = IOS_OPEN,
		.PWM_CHOP_BY4_INTERPOL256   = IOS_OPEN,
		.PWM_CHOP_BY16_INTERPOL256  = IOS_OPEN,
	},

	.resetSettings =
	(
		0
		| (IOS_LOW	<< 0)   // CFG0	= TCLKx140                   : ChopperOffTimeSettings
		| (IOS_LOW	<< 2)   // CFG1 = PWM_CHOP_BY16_INTERPOL256  : MicrosteResolutionSettings 1
		| (IOS_OPEN	<< 4)   // CFG2 = PWM_CHOP_BY16_INTERPOL256  : MicrosteResolutionSettings 2
		| (IOS_LOW	<< 6)   // CFG3 = RSENSE_ONLY                : CurrentSettings
		| (IOS_LOW	<< 8)   // CFG4 = TOFF5                      : ChopperHysteresisSettings
		| (IOS_LOW	<< 10)  // CFG5 = BLANK16                    : ChopperBlankTimeSettings
		| (IOS_OPEN	<< 12)  // CFG6 = ENABLED_034                : Enable_StandStillPowerDownSettings
	),
	.CFGPins = CFGPins

};

static void reset()
{
	setInt(TMCRhinoSA.resetSettings);
}

static void setInt(uint32 state)
{
	for(uint8 i = 0; i < 7; i++)
	{
		settings[i] = state & 0x03;
		state >>= 2;
	}
	set();
}

static uint32 getInt()
{
	uint32 state = 0;

	for(uint8 i = 0; i < 7; i++)
	{
		state |= (settings[i] & 0x03) << (i<<1);
	}

	return state;
}

static void setPins(IO_States *CFG)
{
	for(uint8 i = 0; i < 7; i++)
		settings[i] = CFG[i] & 0x03;
	set();
}

static void getPins(IO_States *CFG)
{
	for(uint8 i = 0; i < 7; i++)
		CFG[i] = settings[i] & 0x03;
}

static void setConfig(TMCRhinoTypeStandAloneConfigDef *config)
{
	settings[0] = config->chopperOffTime;
	settings[1] = config->microstepResolution1;
	settings[2] = config->microstepResolution2;
	settings[3] = config->currentSetting;
	settings[4] = config->chopperHysteresis;
	settings[5] = config->chopperBlankTime;
	settings[6] = config->enableStandStillPowerDown;
	set();
}

static void getConfig(TMCRhinoTypeStandAloneConfigDef *config)
{
	config->chopperOffTime             = settings[0];
	config->microstepResolution1       = settings[1];
	config->microstepResolution2       = settings[2];
	config->currentSetting             = settings[3];
	config->chopperHysteresis          = settings[4];
	config->chopperBlankTime           = settings[5];
	config->enableStandStillPowerDown  = settings[6];
}

static void set()
{
	for(uint8 i = 0; i < 7; i++)
		HAL.IOs->config->setToState(TMCRhinoSA.CFGPins[i], settings[i] & 0x03);
}

