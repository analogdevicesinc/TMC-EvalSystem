# TMC-EvalSystem

Trinamic Evaluation System ([home page](https://www.trinamic.com/support/eval-kits/))

[AN038: Using TRINAMIC’s IC Software API and Examples](https://www.trinamic.com/fileadmin/assets/Support/AppNotes/AN038_Using_TRINAMICs_IC_Software_API_and_Examples.pdf)

## Setup
To clone this repository, simply use the following command in order to clone submodules recursively:  
`git clone --recurse-submodules git@github.com:trinamic/TMC-EvalSystem.git`

## Changelog

For detailed changelog, see commit history.

### Version 3.06: (Beta)
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
