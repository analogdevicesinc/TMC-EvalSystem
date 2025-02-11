# TMC-EvalSystem

Trinamic Evaluation System ([home page](https://www.analog.com/en/resources/evaluation-hardware-and-software/motor-motion-control-software/landungsbrucke-eval-system.html))

[AN038: Using TRINAMIC’s IC Software API and Examples](https://www.analog.com/media/en/technical-documentation/app-notes/an-038.pdf)

## Setup
To clone this repository, use SSH or HTTPS in the following way to clone submodules recursively:  

Via SSH:

`git clone --recurse-submodules git@github.com:analogdevicesinc/TMC-EvalSystem.git`

Via HTTPS:

`git clone --recurse-submodules https://github.com/analogdevicesinc/TMC-EvalSystem.git`

## Build Commands
Following build commands could be used for generating hex file for different Landungsbruecke board versions:
 * `make -s all LINK=<BL version> DEVICE=<Landungsbruecke version>`

| **\<BL version\>** |
|:----------------:|
|        BL        |
|       NOBL       |

| **\<Landungsbruecke version\>** |
|:-----------------------------:|
|        Landungsbruecke        |
|      LandungsbrueckeSmall     |
|       LandungsbrueckeV3       |

The clean command deletes the whole build directory:
 * `make clean`

## Changelog

For detailed changelog, see commit history.

### Version 3.10.9: (Released)
 * TMC9660: Fixed the ID detection issue.
 * Released API rework for TMC2660 and MAX22216.
 * Bug fix for TMC5271.

### Version 3.10.8: (Released)
 * TMC9660: Fixed the IC's script storage feature not working due to a too short communication timeout delay.
 * Removed TMC8461-Eval and TMC8462-Eval from the project.
 * Corrected ADC factor for LBV3 VM measurement.
 
### Version 3.10.7: (Released)
 * Various bug fixes for TMC9660-3PH-PARAM-Eval and TMC9660-STEPPER-PARAM-Eval.
 * Removed TMC9660-Eval from the project.
 
### Version 3.10.6: (Pre-release)
 * Added support for STAP command for TMC9660-3PH-Eval and TMC9660-STEPPER-Eval.
 
### Version 3.10.5: (Released)
 * Included TMC9660-3PH-Eval and TMC9660-STEPPER-Eval to the project.
 * Removed TMC2590 from both API and TMC-EvalSystem project because the IC is available for last time buy only. Whereas the eval is not available anymore.
 
### Version 3.10.4: (Released)
 * Finished the API rework for TMC5160, TMC5130, TMC5062, TMC5072, TMC5240, TMC2224, TMC2225, TMC2226, TMC2130, TMC2160, TMC5031, TMC2208, TMC5041, TMC2300, TMC2240, TMC7300, TMC6100, TMC6200 and TMC4671. For further information click [here](https://github.com/analogdevicesinc/TMC-API/issues/53).
 * Various bugfixes for MAX22204 and MAX22210.
 * Code cleanup.
 
### Version 3.10.3: (Released)
 * Added functionality in the userFunction (UF) of TMC8100-Eval.
 * Completed the API rework for TMC2209 (both phase-1 & 2). For further information click [here](https://github.com/analogdevicesinc/TMC-API/blob/e58c4df79796d2d25a27583a5c59e88415263111/tmc/ic/TMC2209/README.md)
 * Various bugfixes for TMC2262-Eval.
 
### Version 3.10.2: (Released)
 * Various bugfixes for MAX22216-Eval.
 
### Version 3.10.1: (Released)
 * Various bugfixes for TMC4671-Eval, TMC4361A-Eval and TMC2130-Eval.
 
### Version 3.10: (Released)
 * Added TMC5262-Eval, TMC2262-Eval and TMC8100-Eval.
 * Various bugfixes & cleanups

### Version 3.09.6: (Pre-release)
 * Added TMC6140-Eval
 * Various bugfixes & cleanups

### Version 3.09: (Pre-release)
 * Added support for Landungsbruecke with GigaDevice MCU
 * Added MAX22210-Eval and MAX22204-Eval boards
 * Removed TMC4331-Eval, TMC4361-Eval, TMC4330-Eval, TMC5241-Eval and Startrampe boards
 * Various bugfixes & cleanups

### Version 3.08: (Released)
 * Added TMC2210, TMC2226, TMC2240, TMC2300, TMC6100-BOB, TMC6140, TMC6300 and TMC7300
 * Added support for Landungsbruecke Small (same PCB, different µC with less memory)
 * Added RAMDebug for TMC4671 to support automatic PI tuning
 * Removed TMC5161, TMC8690, TMCC160
 * Updated board detection mechanism
 * Various bugfixes & cleanups

### Version 3.07: (Released)
 * Added TMC2225 and TMC6100
 * Changed license from GPL to MIT
 * Improved API structure for UART-based boards
 * Various bugfixes & cleanups

### Version 3.06: (Released)
* Full TMC2160 support
* Landungsbruecke HW v2.0 support with internal HWID detection for future revisions
* Internal/external step/dir generator handling in FW and IDE
* Various bugfixes
* Internal optimizations

### Version 3.05: (Released)
* TMC5072-EVAL Critical bugfix at register access
* TMC2160-EVAL Corrected pin layout

### Version 3.02: (Released)
* Reworked chip register configuration mechanisms (generic reset/restore procedures implemented in API)
* Changed register accesses to use a unified Mask/Shift macro approach
* Various bugfixes

## Repository re-release
Due to licensing issues, the code for the Startrampe board has been purged from the history.
As a side effect, the old git tree is now set to private and this git tree is based on a squashed version of the old tree.
For further questions regarding the old tree, please reach out via an issue in this repository.
