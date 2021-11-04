/*
 * RAMDebug.h
 *
 *  Created on: 07.10.2020
 *      Author: LH
 */

#ifndef RAMDEBUG_H
#define RAMDEBUG_H

#include <stdint.h>

#define RAMDEBUG_FREQUENCY 1000/*0*/

// Capture state
typedef enum {
	RAMDEBUG_IDLE     = 0,
	RAMDEBUG_TRIGGER  = 1,
	RAMDEBUG_CAPTURE  = 2,
	RAMDEBUG_COMPLETE = 3
} RAMDebugState;

// Capture channel configuration
typedef enum {
	CAPTURE_DISABLED            = 0,

	CAPTURE_PARAMETER           = 1,
	CAPTURE_REGISTER            = 2,
	CAPTURE_STACKED_REGISTER    = 3,
	CAPTURE_SYSTICK             = 4,
	CAPTURE_RAMDEBUG_PARAMETER  = 5, // For modules that do not support registers

	CAPTURE_END
} RAMDebugSource;

// Trigger configuration
typedef enum {
	TRIGGER_UNCONDITIONAL          = 0,
	TRIGGER_RISING_EDGE_SIGNED     = 1,
	TRIGGER_FALLING_EDGE_SIGNED    = 2,
	TRIGGER_DUAL_EDGE_SIGNED       = 3,
	TRIGGER_RISING_EDGE_UNSIGNED   = 4,
	TRIGGER_FALLING_EDGE_UNSIGNED  = 5,
	TRIGGER_DUAL_EDGE_UNSIGNED     = 6,

	TRIGGER_END
} RAMDebugTrigger;

void debug_init();
void debug_process();
int debug_setChannel(uint8_t type, uint32_t address);
int debug_getChannelType(uint8_t index, uint8_t *type);
int debug_getChannelAddress(uint8_t index, uint32_t *address);

int debug_setTriggerChannel(uint8_t type, uint32_t threshold);
void debug_setTriggerMaskShift(uint32_t mask, uint8_t shift);
int debug_enableTrigger(uint8_t type, uint32_t threshold);

void debug_setPrescaler(uint32_t divider);
void debug_setSampleCount(uint32_t count);
uint32_t debug_getSampleCount();

int debug_getSample(uint32_t index, uint32_t *value);
int debug_getState(void);
int debug_getInfo(uint32_t type);

#endif /* RAMDEBUG_H */
