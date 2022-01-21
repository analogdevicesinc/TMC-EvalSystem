/*
 * RAMDebug.c
 *
 *  Created on: 07.10.2020
 *      Author: LH
 */
#include "RAMDebug.h"

#include "boards/Board.h"
#include "hal/SysTick.h"
#include "hal/HAL.h"

#include <string.h>

// === RAM debugging ===========================================================

// Debug parameters
#define RAMDEBUG_MAX_CHANNELS     4
#define RAMDEBUG_BUFFER_SIZE      32768
#define RAMDEBUG_BUFFER_ELEMENTS  (RAMDEBUG_BUFFER_SIZE / 4)

bool captureEnabled = false;

// Sample Buffer
uint32_t debug_buffer[RAMDEBUG_BUFFER_ELEMENTS] = { 0 };
uint32_t debug_write_index = 0;
uint32_t debug_read_index  = 0;
uint32_t pre_index = 0;

RAMDebugState state = RAMDEBUG_IDLE;

static bool global_enable = false;
static bool processing = false;
static bool use_next_process = true;
static bool next_process = false;

// Sampling options
static uint32_t prescaler   = 1;
static uint32_t sampleCount = RAMDEBUG_BUFFER_ELEMENTS;
static uint32_t sampleCountPre = 0;

typedef struct {
	RAMDebugSource type;
	uint8_t eval_channel;
	uint32_t address;
} Channel;

Channel channels[RAMDEBUG_MAX_CHANNELS];

typedef struct {
	Channel          channel;
	RAMDebugTrigger  type;
	uint32_t         threshold;
	uint32_t         mask;
	uint8_t          shift;
} Trigger;

Trigger trigger;

// Store whether the last sampling point was above or below the trigger threshold
static bool wasAboveSigned   = 0;
static bool wasAboveUnsigned = 0;

// Function declarations
static uint32_t readChannel(Channel channel);

// === Capture and trigger logic ===============================================

// This function only gets called by the interrupt handler.
void handleTriggering()
{
	// Abort if not in the right state
	if (state != RAMDEBUG_TRIGGER)
		return;

	// Read the trigger channel value and apply mask/shift values
	uint32_t value_raw = readChannel(trigger.channel);
	value_raw = (value_raw & trigger.mask) >> trigger.shift;
	int32_t value = SIGN_EXTEND(value_raw, __builtin_ctz(BIT31 >> __builtin_clz(trigger.mask >> trigger.shift)), int32_t);

	bool isAboveSigned   = (int32_t)  value > (int32_t)  trigger.threshold;
	bool isAboveUnsigned = (uint32_t) value > (uint32_t) trigger.threshold;

	switch(trigger.type)
	{
	case TRIGGER_UNCONDITIONAL:
		state = RAMDEBUG_CAPTURE;
		break;
	case TRIGGER_RISING_EDGE_SIGNED:
		if (!wasAboveSigned && isAboveSigned)
		{
			state = RAMDEBUG_CAPTURE;
		}
		break;
	case TRIGGER_FALLING_EDGE_SIGNED:
		if (wasAboveSigned && !isAboveSigned)
		{
			state = RAMDEBUG_CAPTURE;
		}
		break;
	case TRIGGER_DUAL_EDGE_SIGNED:
		if (wasAboveSigned != isAboveSigned)
		{
			state = RAMDEBUG_CAPTURE;
		}
		break;
	case TRIGGER_RISING_EDGE_UNSIGNED:
		if (!wasAboveUnsigned && isAboveUnsigned)
		{
			state = RAMDEBUG_CAPTURE;
		}
		break;
	case TRIGGER_FALLING_EDGE_UNSIGNED:
		if (wasAboveUnsigned && !isAboveUnsigned)
		{
			state = RAMDEBUG_CAPTURE;
		}
		break;
	case TRIGGER_DUAL_EDGE_UNSIGNED:
		if (wasAboveUnsigned != isAboveUnsigned)
		{
			state = RAMDEBUG_CAPTURE;
		}
		break;
	default:
		break;
	}

	// Store the last threshold comparison value
	wasAboveSigned   = isAboveSigned;
	wasAboveUnsigned = isAboveUnsigned;
}

// This function only gets called by the interrupt handler.
void handleDebugging()
{
	int i;

	for (i = 0; i < RAMDEBUG_MAX_CHANNELS; i++)
	{
		if (channels[i].type == CAPTURE_DISABLED)
			continue;

        if(state == RAMDEBUG_CAPTURE)
        {
            // Add the sample value to the buffer
            debug_buffer[debug_write_index++] = readChannel(channels[i]);

            if (debug_write_index >= sampleCount)
            {
                // End the capture
                state = RAMDEBUG_COMPLETE;
                captureEnabled = false;
                break;
            }
        }
        else if((state != RAMDEBUG_COMPLETE) && (sampleCountPre > 0))
        {
            debug_buffer[pre_index] = readChannel(channels[i]);
            pre_index = (pre_index + 1) % sampleCountPre;
        }
	}
}

void debug_process()
{
	static uint32_t prescalerCount = 0;

	if(!global_enable)
		return;

	if(processing)
		return;

	if(use_next_process && (!next_process))
		return;

	if (captureEnabled == false)		// unsure here! (ED)
		return;

	handleTriggering();

	// Increment and check the prescaler counter
	if (++prescalerCount < prescaler)
		return;

    processing = true;

	// Reset the prescaler counter
	prescalerCount = 0;

	handleDebugging();
	next_process = false;
	processing = false;
}

static inline uint32_t readChannel(Channel channel)
{
	uint32_t sample = 0;

	switch (channel.type)
	{
	case CAPTURE_PARAMETER:
	{
		uint8_t motor   = channel.address >> 24;
		uint8_t type    = channel.address >> 0;

		((channel.eval_channel == 1) ? (&Evalboards.ch2) : (&Evalboards.ch1))->GAP(type, motor, (int32_t *)&sample);

		break;
	}
	case CAPTURE_REGISTER:
	{
		uint8_t motor = channel.address >> 24;

		((channel.eval_channel == 1) ? (&Evalboards.ch2) : (&Evalboards.ch1))->readRegister(motor, channel.address, (int32_t *)&sample);

		break;
	}
	case CAPTURE_STACKED_REGISTER:
	{
		uint8_t motor                  = channel.address >> 24;
		uint8_t stackedRegisterValue   = channel.address >> 16;
		uint8_t stackedRegisterAddress = channel.address >> 8;
		uint8_t dataRegisterAddress    = channel.address >> 0;

		EvalboardFunctionsTypeDef *ch = (channel.eval_channel == 1) ? (&Evalboards.ch2) : (&Evalboards.ch1);

		// Backup the stacked address
		uint32_t oldAddress = 0;
		ch->readRegister(motor, stackedRegisterAddress, (int32_t *)&oldAddress);

		// Write the new stacked address
		ch->writeRegister(motor, stackedRegisterAddress, stackedRegisterValue);

		// Read the stacked data register
		ch->readRegister(motor, dataRegisterAddress, (int32_t *)&sample);

		// Restore the stacked address
		ch->writeRegister(motor, stackedRegisterAddress, oldAddress);
		break;
	}
	case CAPTURE_SYSTICK:
		sample = systick_getTick();//systick_getTimer10ms();
		break;
	case CAPTURE_ANALOG_INPUT:
		// Use same indices as in TMCL.c GetInput()
		switch(channel.address) {
		case 0:
			sample = *HAL.ADCs->AIN0;
			break;
		case 1:
			sample = *HAL.ADCs->AIN1;
			break;
		case 2:
			sample = *HAL.ADCs->AIN2;
			break;
		case 3:
			sample = *HAL.ADCs->DIO4;
			break;
		case 4:
			sample = *HAL.ADCs->DIO5;
			break;
		case 6:
			sample = *HAL.ADCs->VM;
			break;
		}
		break;
	default:
		sample = 0;
		break;
	}

	return sample;
}

// === Interfacing with the debugger ===========================================
void debug_init()
{
	int i;

	// Disable data capture before changing the configuration
	captureEnabled = false;

	// Reset the RAMDebug state
	state = RAMDEBUG_IDLE;

	// Wipe the RAM debug buffer
	for (i = 0; i < RAMDEBUG_BUFFER_ELEMENTS; i++)
	{
		debug_buffer[i] = 0;
	}
	debug_read_index   = 0;
	debug_write_index  = 0;

	// Set default values for the capture configuration
	prescaler   = 1;
	sampleCount = RAMDEBUG_BUFFER_ELEMENTS;
    sampleCountPre = 0;

	// Reset the channel configuration
	for (i = 0; i < RAMDEBUG_MAX_CHANNELS; i++)
	{
		channels[i].type = CAPTURE_DISABLED;
		channels[i].eval_channel = 0;
		channels[i].address = 0;
	}

	// Reset the trigger
	trigger.channel.type     = CAPTURE_DISABLED;
	trigger.channel.address  = 0;
	trigger.mask             = 0xFFFFFFFF;
	trigger.shift            = 0;

	global_enable = true;
}

bool debug_setType(uint8_t type)
{
	int i;

	if (type >= CAPTURE_END)
		return false;

	if (state != RAMDEBUG_IDLE)
		return false;

	// ToDo: Type-specific address verification logic?

	for (i = 0; i < RAMDEBUG_MAX_CHANNELS; i++)
	{
		if (channels[i].type != CAPTURE_DISABLED)
			continue;

		// Add the configuration to the found channel
		channels[i].type     = type;

		return true;
	}

	return false;
}

bool debug_setEvalChannel(uint8_t eval_channel)
{
	int i;

	if (state != RAMDEBUG_IDLE)
		return false;

	// ToDo: Type-specific address verification logic?

	for (i = 0; i < RAMDEBUG_MAX_CHANNELS; i++)
	{
		if (channels[i].type != CAPTURE_DISABLED)
			continue;

		// Add the configuration to the found channel
		channels[i].eval_channel     = eval_channel;

		return true;
	}

	return false;
}

bool debug_setAddress(uint32_t address)
{
	int i;

	if (state != RAMDEBUG_IDLE)
		return false;

	// ToDo: Type-specific address verification logic?

	for (i = 0; i < RAMDEBUG_MAX_CHANNELS; i++)
	{
		if (channels[i].type != CAPTURE_DISABLED)
			continue;

		// Add the configuration to the found channel
		channels[i].address  = address;

		return true;
	}

	return false;
}

int debug_getChannelType(uint8_t index, uint8_t *type)
{
	if (index == 0xFF)
	{
		*type = trigger.channel.type;
		return 1;
	}

	if (index >= RAMDEBUG_MAX_CHANNELS)
		return 0;

	*type = channels[index].type;

	return 1;
}

int debug_getChannelAddress(uint8_t index, uint32_t *address)
{
	if (index == 0xFF)
	{
		*address = trigger.channel.address;
		return 1;
	}

	if (index >= RAMDEBUG_MAX_CHANNELS)
		return 0;

	*address = channels[index].address;

	return 1;
}

bool debug_setTriggerType(uint8_t type)
{
	if (type >= CAPTURE_END)
		return false;

	if (state != RAMDEBUG_IDLE)
		return false;

	// ToDo: Type-specific address verification logic?

	// Store the trigger configuration
	trigger.channel.type     = type;

	return true;
}

bool debug_setTriggerEvalChannel(uint8_t eval_channel)
{
	if (state != RAMDEBUG_IDLE)
		return false;

	// ToDo: Type-specific address verification logic?

	// Store the trigger configuration
	trigger.channel.eval_channel     = eval_channel;

	return true;
}

bool debug_setTriggerAddress(uint32_t address)
{
	if (state != RAMDEBUG_IDLE)
		return false;

	// ToDo: Type-specific address verification logic?

	// Store the trigger configuration
	trigger.channel.address     = address;

	return true;
}

void debug_setTriggerMaskShift(uint32_t mask, uint8_t shift)
{
	trigger.mask  = mask;
	trigger.shift = shift;
}

int debug_enableTrigger(uint8_t type, uint32_t threshold)
{
	// Parameter validation
	if (type >= TRIGGER_END)
		return 0;

	if (state != RAMDEBUG_IDLE)
		return 0;

	// Do not allow the edge triggers with channel still missing
	if (type != TRIGGER_UNCONDITIONAL && trigger.channel.type == CAPTURE_DISABLED)
		return 0;

	// Store the trigger configuration
	trigger.type = type;
	trigger.threshold = threshold;

	// Initialize the trigger helper variable
	// Read out the trigger value and apply the mask/shift
	int32_t triggerValue = (readChannel(trigger.channel) & trigger.mask) >> trigger.shift;
	wasAboveSigned   = (int32_t)  triggerValue > (int32_t)  trigger.threshold;
	wasAboveUnsigned = (uint32_t) triggerValue > (uint32_t) trigger.threshold;

	// Enable the trigger
	state = RAMDEBUG_TRIGGER;

	// Enable the capturing IRQ
	captureEnabled = true;

	return 1;
}

void debug_setPrescaler(uint32_t divider)
{
	prescaler = divider;
}

void debug_setSampleCount(uint32_t count)
{
	if (count > RAMDEBUG_BUFFER_ELEMENTS)
		count = RAMDEBUG_BUFFER_ELEMENTS;

	sampleCount = count;
}

uint32_t debug_getSampleCount()
{
	return sampleCount;
}

void debug_setPretriggerSampleCount(uint32_t count)
{
    if (count > sampleCount)
		count = sampleCount;

    sampleCountPre = count;
    debug_write_index = count;
}

uint32_t debug_getPretriggerSampleCount()
{
    return sampleCountPre;
}

int debug_getSample(uint32_t index, uint32_t *value)
{
	if (index >= debug_write_index)
		return 0;

    if(index < sampleCountPre)
    {
        *value = debug_buffer[(pre_index + index) % sampleCountPre];
    }
    else
    {
        *value = debug_buffer[index];
    }

	return 1;
}

int debug_getState(void)
{
	return state;
}

int debug_getInfo(uint32_t type)
{
	switch(type)
	{
	case 0:
		return RAMDEBUG_MAX_CHANNELS;
		break;
	case 1:
		return RAMDEBUG_BUFFER_ELEMENTS;
		break;
	case 2:
		// PWM/Sampling Frequency
		return RAMDEBUG_FREQUENCY;
		break;
	case 3:
		return debug_write_index;
		break;
	default:
		break;
	}

	return -1;
}

void debug_useNextProcess(bool enable)
{
	use_next_process = enable;
}

void debug_nextProcess(void)
{
	next_process = true;
}

void debug_setGlobalEnable(bool enable)
{
	global_enable = enable;
}
