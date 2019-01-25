#ifndef RHINO_STANDALONE_H
	#define RHINO_STANDALONE_H

	#include "Board.h"

	typedef struct {
		const IO_States TCLKx140;
		const IO_States TCLKx236;
		const IO_States TCLKx332;
	} TMCRhinoChopperOffTimeTypeDef;

	typedef struct {
		const IO_States RSENSE_ONLY;
		const IO_States INTERNAL_RSENSE;
		const IO_States I_SCALE_ANALOG;
	} TMCRhinoCurrentSettingTypeDef;

	typedef struct {
		const IO_States TOFF5;
		const IO_States TOFF9;
		const IO_States I_SCALE_ANALOG;
	} TMCRhinoChopperHysteresisTypeDef;

	typedef struct {
		const IO_States BLANK16;
		const IO_States BLANK24;
		const IO_States BLANK36;
	} TMCRhinoChopperBlankTimeTypeDef;

	typedef struct {
		const IO_States ENABLED_MAX;
		const IO_States DISABLED;
		const IO_States ENABLED_034;
	} TMCRhinoEnableStStPwrDnTypeDef;

	typedef struct {
		const IO_States BY1_INTERPOL0;
		const IO_States BY2_INTERPOL0;
		const IO_States BY2_INTERPOL256;
		const IO_States BY4_INTERPOL0;
		const IO_States BY4_INTERPOL256;
		const IO_States PWM_CHOP_BY4_INTERPOL256;
		const IO_States BY16_INTERPOL0;
		const IO_States BY16_INTERPOL256;
		const IO_States PWM_CHOP_BY16_INTERPOL256;
	} TMCRhinoMicrostepResolutionTypeDef;

	typedef struct
	{
		IO_States chopperOffTime;
		IO_States	microstepResolution1;
		IO_States	microstepResolution2;
		IO_States currentSetting;
		IO_States chopperHysteresis;
		IO_States chopperBlankTime;
		IO_States enableStandStillPowerDown;
	} TMCRhinoTypeStandAloneConfigDef;

	typedef struct
	{
		const TMCRhinoChopperOffTimeTypeDef       ChopperOffTimeSettings;
		const TMCRhinoMicrostepResolutionTypeDef  MicrostepResolutionSettings1;
		const TMCRhinoMicrostepResolutionTypeDef  MicrostepResolutionSettings2;
		const TMCRhinoCurrentSettingTypeDef       CurrentSettings;
		const TMCRhinoChopperHysteresisTypeDef    ChopperHysteresisSettings;
		const TMCRhinoChopperBlankTimeTypeDef     ChopperBlankTimeSettings;
		const TMCRhinoEnableStStPwrDnTypeDef      EnableStandStillPowerDownSettings;
		const uint32                              resetSettings;

		void (*reset) (void);
		void (*setConfig) (TMCRhinoTypeStandAloneConfigDef *config);
		void (*getConfig) (TMCRhinoTypeStandAloneConfigDef *config);
		void (*setPins) (IO_States *CFG);
		void (*getPins) (IO_States *CFG);
		void (*setInt) (uint32 state);
		uint32 (*getInt) (void);
		IOPinTypeDef **CFGPins;

	} TMCRhinoTypeStandAloneDef;

	TMCRhinoTypeStandAloneDef TMCRhinoSA;

#endif /* RHINO_STANDALONE_H */
