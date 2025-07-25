/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#ifndef BOARD_ASSIGNMENT_H
#define BOARD_ASSIGNMENT_H

#include "boards/Board.h"

typedef enum
{
    FOUND_BY_NONE,
    FOUND_BY_MONOFLOP,
    FOUND_BY_EEPROM
} IDFinder;

typedef struct
{
    uint8_t state;       // detection state of this board
    uint8_t id;          // id of board
    IDFinder detectedBy; // Holds the method used to detect the ID (Monoflop or EEPROM)
    uint32_t counter_1;  // Timer cycles elapsed on ID pulse rising edge
    uint32_t counter_2;  // Timer cycles elapsed on ID pulse falling edge
    uint32_t timer_1;    // Current timer value on ID pulse rising edge
    uint32_t timer_2;    // Current timer value on ID pulse falling edge
} IdStateTypeDef;        // interface for id and detection state of a board

typedef struct
{
    IdStateTypeDef ch1; // interface for id and detection state for the driver board
    IdStateTypeDef ch2; // interface for id and detection state for the motion controller board
} IdAssignmentTypeDef;  // interface for id and detection state of driver and motion controller board

extern IdAssignmentTypeDef IdState;

int32_t Board_assign(IdAssignmentTypeDef *ids);    // ids and states of assigned driver and motion controller board
int32_t Board_supported(IdAssignmentTypeDef *ids); // ids and states of supported driver and motion controller board

#include "boards/SelfTest.h"

// ids for channel 0
#define ID_ERROR           0
#define ID_TMC5031         2
#define ID_TMC5130         5
#define ID_TMC5041         6
#define ID_TMC5072         7
#define ID_TMC4361A        11
#define ID_TMC4671         13
#define ID_TMC5160         16
#define ID_TMC5062         25
#define ID_TMC8462         27 // Exists but not in this project.
#define ID_TMC5240		   28
#define ID_TMC5272		   29
#define ID_TMC5262         30
#define ID_TMC5271		   31
#define ID_TMC9660_3PH_BL_EVAL              33
#define ID_TMC9660_3PH_REG_EVAL             34
#define ID_TMC9660_3PH_PARAM_EVAL           35
#define ID_TMC9660_STEPPER_BL_EVAL          36
#define ID_TMC9660_STEPPER_REG_EVAL         37
#define ID_TMC9660_STEPPER_PARAM_EVAL       38
#define ID_TMC2130_TQFP48  0xFE
#define ID_SELFTEST        255

// ids for channel 1
#define ID_TMC2660         1
#define ID_TMC2130         3
#define ID_TMC2100         4
#define ID_TMC2208         6
#define ID_TMC2224         7
#define ID_TMC2209         8
#define ID_TMC6200        10
#define ID_TMC2160        11
#define ID_TMC7300        12
#define ID_TMC2300        14
#define ID_TMC2225        18
#define ID_TMC6100        19
#define ID_TMC6300        21
#define ID_TMC2226        22
#define ID_TMC6140        23
#define ID_TMC6100_BOB    25 // For the TMC4671+TMC6100-BOB
#define ID_TMC2240        28
#define ID_TMC2210		  29
#define ID_MAX22216_EVAL 	30
#define ID_MAX22216_BOB 	31
#define ID_MAX22204_EVAL 	32
#define ID_MAX22210_EVAL 	33
#define ID_TMC8100        34
#define ID_TMC2262        35
#define ID_MAX22215       36
#define ID_MAX22200_EVAL 42
#define TM01       1414332417

#define ID_GROUP_TMC9660_STEPPER(id) ((id) >= ID_TMC9660_STEPPER_BL_EVAL && (id) <= ID_TMC9660_STEPPER_PARAM_EVAL)
#define ID_GROUP_TMC9660_3PH(id) ((id) >= ID_TMC9660_3PH_BL_EVAL && (id) <= ID_TMC9660_3PH_PARAM_EVAL)

// init() functions for all boards - function definitions are in the respective _eval file of a chip
extern void MAX22200_init();
extern void MAX22216_init();
extern void MAX22204_init();
extern void MAX22210_init();
extern void TMC2100_init();
extern void TMC2130_init();
extern void TMC2160_init();
extern void TMC2208_init();
extern void TMC2209_init();
extern void TMC2210_init();
extern void TMC2224_init();
extern void TMC2225_init();
extern void TMC2226_init();
extern void TMC2240_init();
extern void TMC2300_init();
extern void TMC2660_init();
extern void TMC4361A_init();
extern void TMC4671_init();
extern void TMC5031_init();
extern void TMC5041_init();
extern void TMC5062_init();
extern void TMC5072_init();
extern void TMC5130_init();
extern void TMC5160_init();
extern void TMC5240_init();
extern void TMC5271_init();
extern void TMC5272_init();
extern void TMC5262_init();
extern void TMC6100_init();
extern void TMC6100_BOB_init();
extern void TMC6140_init();
extern void TMC6200_init();
extern void TMC6300_init();
extern void TMC7300_init();
extern void TMC8100_init();
extern void TMC2262_init();
extern void TMC9660_3PH_init();
extern void TMC9660_STEPPER_init();
extern void MAX22215_init();
extern void SelfTest_init();

#if defined(LandungsbrueckeV3)
extern void PD8_IRQHandler();
#endif

typedef struct
{
    uint16_t id;
    void (*init)(void);
} init_assignment;

static const init_assignment init_ch1[] =
{
	{ .id = ID_TMC5031,     .init = TMC5031_init     },
	{ .id = ID_TMC5130,     .init = TMC5130_init     },
	{ .id = ID_TMC5041,     .init = TMC5041_init     },
	{ .id = ID_TMC5072,     .init = TMC5072_init     },
	{ .id = ID_TMC4361A,    .init = TMC4361A_init    },
	{ .id = ID_TMC4671,     .init = TMC4671_init     },
	{ .id = ID_TMC5160,     .init = TMC5160_init     },
	{ .id = ID_TMC5240,     .init = TMC5240_init     },
	{ .id = ID_TMC5271,     .init = TMC5271_init     },
	{ .id = ID_TMC5272,     .init = TMC5272_init     },
	{ .id = ID_TMC5062,     .init = TMC5062_init     },
    { .id = ID_TMC5262,     .init = TMC5262_init     },
    { .id = ID_TMC9660_3PH_BL_EVAL,     .init = TMC9660_3PH_init     },
    { .id = ID_TMC9660_3PH_REG_EVAL,    .init = TMC9660_3PH_init     },
    { .id = ID_TMC9660_3PH_PARAM_EVAL,  .init = TMC9660_3PH_init     },
    { .id = ID_TMC9660_STEPPER_BL_EVAL, .init = TMC9660_STEPPER_init },
    { .id = ID_TMC9660_STEPPER_REG_EVAL, .init = TMC9660_STEPPER_init },
    { .id = ID_TMC9660_STEPPER_PARAM_EVAL, .init = TMC9660_STEPPER_init },
	{ .id = ID_SELFTEST,    .init = SelfTest_init    }
};

static const init_assignment init_ch2[] =
{
	{ .id = ID_TMC2660,       .init = TMC2660_init     },
	{ .id = ID_TMC2130,       .init = TMC2130_init     },
	{ .id = ID_TMC2100,       .init = TMC2100_init     },
	{ .id = ID_TMC2208,       .init = TMC2208_init     },
	{ .id = ID_TMC2224,       .init = TMC2224_init     },
	{ .id = ID_TMC2240,       .init = TMC2240_init     },
	{ .id = ID_TMC6100,       .init = TMC6100_init     },
	{ .id = ID_TMC6100_BOB,   .init = TMC6100_BOB_init },
	{ .id = ID_TMC6200,       .init = TMC6200_init     },
	{ .id = ID_TMC7300,       .init = TMC7300_init     },
	{ .id = ID_TMC2160,       .init = TMC2160_init     },
	{ .id = ID_TMC2210,       .init = TMC2210_init     },
	{ .id = ID_MAX22216_EVAL, .init = MAX22216_init    },
	{ .id = ID_MAX22216_BOB,  .init = MAX22216_init    },
	{ .id = ID_MAX22204_EVAL, .init = MAX22204_init    },
	{ .id = ID_MAX22210_EVAL, .init = MAX22210_init    },
	{ .id = ID_MAX22200_EVAL, .init = MAX22200_init    },
	{ .id = ID_TMC2209,     .init = TMC2209_init     },
	{ .id = ID_TMC2225,     .init = TMC2225_init     },
	{ .id = ID_TMC2226,     .init = TMC2226_init     },
	{ .id = ID_TMC2300,     .init = TMC2300_init     },
	{ .id = ID_TMC6300,     .init = TMC6300_init     },
	{ .id = ID_TMC6140,     .init = TMC6140_init     },
	{ .id = ID_TMC8100,     .init = TMC8100_init     },
	{ .id = ID_TMC2262,     .init = TMC2262_init     },
    { .id = ID_MAX22215,    .init = MAX22215_init    },
};

#endif /* BOARD_ASSIGNMENT_H */
