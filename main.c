
#include "boards/Board.h"
#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/IdDetection.h"
#include "tmc/TMCL.h"
#include "tmc/VitalSignsMonitor.h"
#include "tmc/BoardAssignment.h"
#include "tmc/RAMDebug.h"

const char *VersionString = MODULE_ID"V308"; // module id and version of the firmware shown in the TMCL-IDE

EvalboardsTypeDef Evalboards;

/* Keep as is! This lines are important for the update functionality. */
#if defined(Landungsbruecke)
	const uint8_t Protection[] __attribute__ ((section(".cfmconfig")))=
	{
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //Backdoor key
		0xFF, 0xFF, 0xFF, 0xFF,                          //Flash protection (FPPROT)
		0x7E,                                            //Flash security   (FSEC) => nach Image-Generierung manuell auf 0x40 setzen im Image
		0xF9,                                            //Flash option     (FOPT) (NMI ausgeschaltet, EzPort ausgeschaltet, Normal power)
		0xFF,                                            //reserved
		0xFF                                             //reserved
	};

	struct BootloaderConfig {
		uint32_t BLMagic;
		uint32_t drvEnableResetValue;
	};

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
	tmcl_boot();
}

/* Call all standard initialization routines. */
static void init()
{
#if defined(Landungsbruecke)
	// Default value: Driver enable gets set high by the bootloader
	BLConfig.drvEnableResetValue = 1;
#endif

	HAL.init();                  // Initialize Hardware Abstraction Layer
	IDDetection_init();          // Initialize board detection
	tmcl_init();                 // Initialize TMCL communication

	tmcdriver_init();            // Initialize dummy driver board --> preset EvalBoards.ch2
	tmcmotioncontroller_init();  // Initialize dummy motion controller board  --> preset EvalBoards.ch1

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
