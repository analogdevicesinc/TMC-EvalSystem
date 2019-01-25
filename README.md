# TMC-Evaluation-System

## Setup
To clone this repository, simply use the following command in order to clone submodules recursively:  
`git clone --recurse-submodules git@github.com:trinamic/TMC-EvalSystem.git`

## Changelog

### Version 3.06: (Beta)
* Full TMC2160 support
* Landungsbruecke HW v2.0 support with internal HWID detection for future revisions
* Internal/external step/dir generator handling in FW and IDE
* Begin TMC7531-EVAL support
* Begin TMC4672-EVAL support
* Begin TMC2590-EVAL support
* Begin TMC-Minion-EVAL support
* Various bugfixes
* Internal optimizations

### Version 3.05: (Released)
* TMC5072-EVAL Critical bugfix at register access
* TMC2160-EVAL Corrected pin layout

### Version 3.02: (Released)
* Reworked chip register configuration mechanisms (generic reset/restore procedures implemented in API)
* Changed register accesses to use a unified Mask/Shift macro approach
* Various bugfixes