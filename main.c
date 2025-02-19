/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "boards/Board.h"
#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/IdDetection.h"
#include "tmc/TMCL.h"
#include "tmc/VitalSignsMonitor.h"
#include "tmc/BoardAssignment.h"
#include "tmc/RAMDebug.h"

const char *VersionString = MODULE_ID "V310"; // module id and version of the firmware shown in the TMCL-IDE

EvalboardsTypeDef Evalboards;

// Forward declaration
void enterBootloader();

/* Keep as is! This lines are important for the update functionality. */
#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall)
	const uint8_t Protection[] __attribute__ ((section(".cfmconfig")))=
	{
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //Backdoor key
		0xFF, 0xFF, 0xFF, 0xFF,                          //Flash protection (FPPROT)
		0x7E,                                            //Flash security   (FSEC) => nach Image-Generierung manuell auf 0x40 setzen im Image
		0xF9,                                            //Flash option     (FOPT) (NMI ausgeschaltet, EzPort ausgeschaltet, Normal power)
		0xFF,                                            //reserved
		0xFF                                             //reserved
	};
#endif

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall) || defined(LandungsbrueckeV3)
	// This struct gets placed at a specific address by the linker
	struct BootloaderConfig __attribute__ ((section(".bldata"))) BLConfig;
#endif


/* Check if jumping into bootloader is forced                                           */
/*                                                                                      */
/* In order to jump to bootloader e.g. because of an accidental infinite loop           */
/* in a modified firmware you may short ID_CLK and ID_CH0 pins on start up.             */
/* This will force the entrance into bootloader mode and allow to replace bad firmware. */
void shallForceBoot()
{
	// toggle each pin and see if you can read the state on the other
	// leave if not, because this means that the pins are not tied together
	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CLK);
	if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
		return;

	HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CLK);
	if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
		return;

	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH0);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CLK);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CH0);
	if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
		return;

	HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CH0);
	if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
		return;

	// not returned, this means pins are tied together
	enterBootloader();
}

/* Call all standard initialization routines. */
static void init()
{
	HAL.init();                  // Initialize Hardware Abstraction Layer

	tmcdriver_init();            // Initialize dummy driver board --> preset EvalBoards.ch2
	tmcmotioncontroller_init();  // Initialize dummy motion controller board  --> preset EvalBoards.ch1

#if defined(LandungsbrueckeV3)
	// Check for the button-based return-to-bootloader request
	if (HAL.IOs->config->isHigh(&HAL.IOs->pins->BUTTON))
	{
		enterBootloader();
	}
#endif

	IDDetection_init();          // Initialize board detection
	tmcl_init();                 // Initialize TMCL communication
	VitalSignsMonitor.busy = 1;  // Put state to busy
	Evalboards.driverEnable = DRIVER_ENABLE;
	Evalboards.ch1.id = 0;       // preset id for driver board to 0 --> error/not found
	Evalboards.ch2.id = 0;       // preset id for driver board to 0 --> error/not found

	// We disable the drivers before configurating anything
	HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO0);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO0);

	IdAssignmentTypeDef ids = { 0 };
	IDDetection_initialScan(&ids);  // start initial board detection
	IDDetection_initialScan(&ids);  // start second time, first time not 100% reliable, not sure why - too fast after startup?
	if(!ids.ch1.id && !ids.ch2.id)
	{
		shallForceBoot();           // only checking to force jump into bootloader if there are no boards attached
		// todo CHECK 2: Workaround: shallForceBoot() changes pin settings - change them again here, since otherwise IDDetection partially breaks (LH)
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
		HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

	}

	if (ID_CH1_DEFAULT && (!ids.ch1.id || ID_CH1_OVERRIDE))
	{
		ids.ch1.id = ID_CH1_DEFAULT;
		ids.ch1.state = ID_STATE_DONE;
	}

	if (ID_CH2_DEFAULT && (!ids.ch2.id || ID_CH2_OVERRIDE))
	{
		ids.ch2.id = ID_CH2_DEFAULT;
		ids.ch2.state = ID_STATE_DONE;
	}

	Board_assign(&ids);             // assign boards with detected id

	VitalSignsMonitor.busy 	= 0;    // not busy any more!
}

/* main function */
int main(void)
{
	// Start all initialization routines
	init();

	// Main loop
	while(1)
	{
		// Check all parameters and life signs and mark errors
		vitalsignsmonitor_checkVitalSigns();

		// handle RAMDebug
		debug_process();

		// Perodic jobs of Motion controller/Driver boards
		Evalboards.ch1.periodicJob(systick_getTick());
		Evalboards.ch2.periodicJob(systick_getTick());

		// Process TMCL communication
		tmcl_process();
	}

	return 0;
}

void enterBootloader()
{
#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall) || defined(LandungsbrueckeV3)
    if(Evalboards.ch1.id == ID_TMC4671)
    {
        // Driver Enable has to be set low by the bootloader for these ICs
        BLConfig.drvEnableResetValue = 0;
    }
    else
    {
        // Default: Driver Enable is set to high
        BLConfig.drvEnableResetValue = 1;
    }
#endif
    Evalboards.driverEnable = DRIVER_DISABLE;
    Evalboards.ch1.enableDriver(DRIVER_DISABLE); // todo CHECK 2: the ch1/2 deInit() calls should already disable the drivers - keep this driver disabling to be sure or remove it and leave the disabling to deInit? (LH)
    Evalboards.ch2.enableDriver(DRIVER_DISABLE);

    Evalboards.ch1.deInit();
    Evalboards.ch2.deInit();

    HAL.USB->deInit();

    wait(500);

    HAL.Timer->deInit();
    HAL.RS232->deInit();
    HAL.WLAN->deInit();
    HAL.ADCs->deInit();

    // todo: CHECK 2: Muss api_deInit hier dazu? (ED)
    StepDir_deInit();

    IDDetection_deInit();

    HAL.NVIC_DeInit();

#if defined(Landungsbruecke) || defined(LandungsbrueckeSmall) || defined(LandungsbrueckeV3)
    bool isBLNew = (BLConfig.BLMagic == BL_MAGIC_VALUE_BL_NEW);
    BLConfig.BLMagic = isBLNew ? BL_MAGIC_VALUE_APP_NEW : BL_MAGIC_VALUE_OLD;
    HAL.reset(true);
#endif
}
