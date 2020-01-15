/*
 * IdDetection_Landungsbruecke.c
 *
 *  Created on:  24.10.2013
 *  Author:      ernst
 *
 *  Calling IDDetection_detect(IdAssignmentTypeDef *result) will start the ID detection process.
 *  The function returns the ID Results through the IdAssignmentTypeDef struct result points to.
 *  The detection will be done by monoflop pulse duration measurement or via EEPROM readout.
 *  While this process is still ongoing the function will return ID_STATE_WAIT_HIGH. Once the
 *  ID detection of both channels has been finished, ID_STATE_DONE will be returned.
 *
 *  Calling the function again after the detection has finished will start another scan.
 */

#include "tmc/helpers/API_Header.h"

#include "hal/derivative.h"
#include "hal/HAL.h"
#include "BoardAssignment.h"
#include "EEPROM.h"
#include "IdDetection.h"
#include "VitalSignsMonitor.h"
#include "TMCL.h"

// Helper functions
static int detectID_Monoflop(IdAssignmentTypeDef *ids);
static int detectID_EEPROM(IdAssignmentTypeDef *ids);

// Helper macros
#define ID_CLK_LOW()   HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CLK);   // set id clk signal to low
#define ID_CLK_HIGH()  HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CLK);  // set id clk signal to high
#define IDSTATE_SCAN_DONE(ID_STATE)                             \
            (                                                   \
                (ID_STATE.ch1.state != ID_STATE_WAIT_LOW)  &&   \
                (ID_STATE.ch2.state != ID_STATE_WAIT_LOW)  &&   \
                (ID_STATE.ch1.state != ID_STATE_WAIT_HIGH) &&   \
                (ID_STATE.ch2.state != ID_STATE_WAIT_HIGH)      \
            )

static uint8_t assign(uint32_t pulse);

typedef enum {
	MONOFLOP_INIT,
	MONOFLOP_SCANNING,
	MONOFLOP_DONE,

	MONOFLOP_END
} State_MonoflopDetection;

State_MonoflopDetection monoflopState = MONOFLOP_INIT;

/* Timer Frequency:
 *  System Clock   48MHz
 *  ------------ = ----- = 6MHz
 *    Prescaler      8
 *
 * Calculate time interval (t_id) in 0.1µs from the timer tick difference (ticks):
 *                 0.1s                  10s
 *  t_id * 0.1µs = ---- = ticks * 6MHz * ---     <--- Tick Conversion factor: 10/6
 *                 10^6                   6
 */
#define TICK_FACTOR 10/6

IdAssignmentTypeDef IdState = { 0 };

// Interrupt: Edge on GPIO Port B
void PORTB_IRQHandler(void)
{
	// Store the timing values
	uint32_t timerVal = FTM2_CNT;

	// Store the interrupt flag state and then reset the flags
	uint32_t interruptFlags = PORTB_ISFR;
	PORTB_ISFR = PORT_ISFR_ISF_MASK;

	// Abort if we're not scanning
	if(monoflopState != MONOFLOP_SCANNING)
		return;

	// ======== CH0 ==========
	// Check if Pin ID_CH0 generated the interrupt
	if(interruptFlags & PORT_ISFR_ISF(HAL.IOs->pins->ID_CH0.bitWeight))
	{
		if(IdState.ch1.state == ID_STATE_WAIT_HIGH)
		{	// Second ID pulse edge - store timer values -> state DONE
			IdState.ch1.timer_2    = timerVal;
			IdState.ch1.counter_2  = 0;
			IdState.ch1.state      = ID_STATE_DONE;
		}
		else
		{	// First ID pulse edge - store timer values -> state WAIT_HIGH
			IdState.ch1.timer_1    = timerVal;
			IdState.ch1.counter_1  = 0;
			IdState.ch1.state      = ID_STATE_WAIT_HIGH;
		}
	}

	// ======== CH1 ==========
	// Check if Pin ID_CH1 generated the interrupt
	if(interruptFlags & PORT_ISFR_ISF(HAL.IOs->pins->ID_CH1.bitWeight))
	{
		if(IdState.ch2.state == ID_STATE_WAIT_HIGH)
		{	// Second ID pulse edge - store timer values -> state DONE
			IdState.ch2.timer_2    = timerVal;
			IdState.ch2.counter_2  = 0;
			IdState.ch2.state      = ID_STATE_DONE;
		}
		else
		{	// First ID pulse edge - store timer values -> state WAIT_HIGH
			IdState.ch2.timer_1    = timerVal;
			IdState.ch2.counter_1  = 0;
			IdState.ch2.state      = ID_STATE_WAIT_HIGH;
		}
	}
}

void FTM2_IRQHandler()
{
	// clear timer overflow flag
	FTM2_SC &= ~FTM_SC_TOF_MASK;

	// Stop the timer
	FTM2_SC &= ~FTM_SC_CLKS_MASK;

	// Abort if we're not scanning
	if(monoflopState == MONOFLOP_SCANNING)
	{
		monoflopState = MONOFLOP_DONE;
			return;
	}
}

void IDDetection_init(void)
{
	monoflopState = MONOFLOP_INIT;


	// ====== Timer initialisation =======
	// Enable clock for FTM2
	SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;

	// Disable write protection, FTM specific registers are available
	FTM2_MODE |= FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK | FTM_MODE_FAULTM_MASK;

	// Clear the CLKS field to avoid buffered write issues for MOD
	FTM2_SC &= ~FTM_SC_CLKS_MASK;

	// Use the full time period available
	FTM2_MOD   = 0xFFFF;
	FTM2_CNTIN = 0;
	FTM2_CNT   = 0;

	// Clock source: System Clock (48 MHz), Prescaler: 8 -> 6 MHz Timer clock
	FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(3);

	// The TOF bit is set for each counter overflow
	FTM2_CONF |= FTM_CONF_NUMTOF(0);

	// Edge-Aligned PWM (EPWM) mode
	FTM2_C0SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

	// Enable FTM2 Timer Overflow interrupt
	FTM2_SC |= FTM_SC_TOIE_MASK;

	// Set FTM2 interrupt handler
	enable_irq(INT_FTM2 - 16);

	// ====== Pin initialisation ======
	// Enable Clock gating on port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Configure Pin
	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);

	// Enable GPIO, edge-triggered interrupt, and PullUp/PullDown
	PORT_PCR_REG(HAL.IOs->pins->ID_CH0.portBase, HAL.IOs->pins->ID_CH0.bit)  = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | 0x000000;
	PORT_PCR_REG(HAL.IOs->pins->ID_CH1.portBase, HAL.IOs->pins->ID_CH1.bit)  = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | 0x000000;

	// Clear interrupt flags
	PORTB_ISFR = -1;

	// Enable interrupt
	enable_irq(INT_PORTB - 16);
}

void IDDetection_deInit()
{
	disable_irq(INT_FTM2 - 16);
	FTM2_SC &= ~FTM_SC_CLKS_MASK;
	FTM2_SC &= ~FTM_SC_TOF_MASK;

	disable_irq(INT_PORTB - 16);
	PORTB_ISFR = -1;
}

//returns ID assigned to given pulse (length in 0.1us)
static uint8_t assign(uint32_t pulse)
{
	if(     pulse < 5      ) return 0; // error
	else if(pulse < 110    ) return 1;
	else if(pulse < 135    ) return 2;
	else if(pulse < 165    ) return 3;
	else if(pulse < 200    ) return 4;
	else if(pulse < 245    ) return 5;
	else if(pulse <	300    ) return 6;
	else if(pulse <	360    ) return 7;
	else if(pulse <	430    ) return 8;
	else if(pulse <	515    ) return 9;
	else if(pulse <	620    ) return 10;
	else if(pulse <	750    ) return 11;
	else if(pulse <	910    ) return 12;
	else if(pulse <	1100   ) return 13;
	else if(pulse <	1350   ) return 14;
	else if(pulse <	1650   ) return 15;
	else if(pulse <	2000   ) return 16;
	else if(pulse <	2450   ) return 17;
	else if(pulse <	3000   ) return 18;
	else if(pulse <	3600   ) return 19;
	else if(pulse <	4300   ) return 20;
	else if(pulse <	5150   ) return 21;
	else if(pulse <	6200   ) return 22;
	else if(pulse <	7500   ) return 23;
	else if(pulse <	9100   ) return 24;
	else if(pulse <	11000  ) return 25;
	else if(pulse <	13500  ) return 26;
	else if(pulse <	16500  ) return 27;
	else if(pulse <	20000  ) return 28;
	else if(pulse <	24500  ) return 29;
	else if(pulse <	30000  ) return 30;
	else if(pulse <	36000  ) return 31;
	else if(pulse <	43000  ) return 32;
	else if(pulse <	51500  ) return 33;
	else if(pulse <	62000  ) return 34;
	else if(pulse <	75000  ) return 35;
	else if(pulse <	91000  ) return 36;

	return 0; // error
}

// Detect IDs of attached boards - returns true when done
uint8_t IDDetection_detect(IdAssignmentTypeDef *out)
{
	// Try to identify the IDs via monoflop pulse duration
	if (!detectID_Monoflop(out))
		return false;

	// Try to identify the IDs via EEPROM readout
	detectID_EEPROM(out);

	// Detection finished
	return true;
}

void IDDetection_initialScan(IdAssignmentTypeDef *ids)
{
	while(!IDDetection_detect(ids))
	{
		vitalsignsmonitor_checkVitalSigns();
		tmcl_process();
	}
}


// Helper functions
static int detectID_Monoflop(IdAssignmentTypeDef *ids)
{
	switch (monoflopState)
	{
	case MONOFLOP_INIT:
		FTM2_SC &= ~FTM_SC_CLKS_MASK;  // stop timer
		FTM2_CNT = 0;                  // clear counter

		IdState.ch1.state       = ID_STATE_WAIT_LOW;
		IdState.ch1.detectedBy  = FOUND_BY_NONE;
		IdState.ch2.state       = ID_STATE_WAIT_LOW;
		IdState.ch2.detectedBy  = FOUND_BY_NONE;

		// Update the monoflop state before activating the timer. Otherwise bad
		// luck with other unrelated interrupts might cause enough delay to
		// trigger the timer overflow after starting the timer before updating
		// this state - which results in the timeout no longer working.
		monoflopState = MONOFLOP_SCANNING;

		FTM2_SC |= FTM_SC_CLKS(1);  // start timer
		ID_CLK_HIGH();
		break;
	case MONOFLOP_SCANNING:
		if(IDSTATE_SCAN_DONE(IdState))
		{
			monoflopState = MONOFLOP_DONE;
		}
		break;
	case MONOFLOP_DONE:
		// Scan complete
		ID_CLK_LOW();
		FTM2_SC &= ~FTM_SC_CLKS_MASK; // stop timer

		// ======== CH0 ==========
		// Assign ID detection state for this channel
		ids->ch1.state = IdState.ch1.state;

		if(IdState.ch1.state == ID_STATE_DONE)
		{
			// Assign the ID derived from the ID pulse duration
			uint32_t tickDiff = IdState.ch1.timer_2 - IdState.ch1.timer_1;
			ids->ch1.id = assign(tickDiff * TICK_FACTOR);

			if(ids->ch1.id)
				IdState.ch1.detectedBy = FOUND_BY_MONOFLOP;
			else
				ids->ch1.state = ID_STATE_INVALID; // Invalid ID pulse detected
		}
		else if(IdState.ch1.state == ID_STATE_WAIT_HIGH)
		{	// Only detected ID pulse rising edge -> Timeout
			ids->ch1.state = ID_STATE_TIMEOUT;
		}
		else if(IdState.ch1.state == ID_STATE_WAIT_LOW)
		{	// Did not detect any edge -> No answer
			ids->ch1.state = ID_STATE_NO_ANSWER;
		}
		else
		{
			ids->ch1.id = 0;
		}

		// ======== CH1 ==========
		// Assign ID detection state for this channel
		ids->ch2.state 	= IdState.ch2.state;

		if(IdState.ch2.state == ID_STATE_DONE)
		{
			// Assign the ID derived from the ID pulse duration
			uint32_t tickDiff = IdState.ch2.timer_2 - IdState.ch2.timer_1;
			ids->ch2.id = assign(tickDiff * TICK_FACTOR);

			if(ids->ch2.id)
				IdState.ch2.detectedBy = FOUND_BY_MONOFLOP;
			else
				ids->ch2.state = ID_STATE_INVALID; // Invalid ID pulse detected
		}
		else if(IdState.ch2.state == ID_STATE_WAIT_HIGH)
		{	// Only detected ID pulse rising edge -> Timeout
			ids->ch2.state = ID_STATE_TIMEOUT;
		}
		else if(IdState.ch2.state == ID_STATE_WAIT_LOW)
		{	// Did not detect any edge -> No answer
			ids->ch2.state = ID_STATE_NO_ANSWER;
		}
		else
		{
			ids->ch2.id = 0;
		}

		monoflopState = MONOFLOP_INIT;
		return true;
		break;
	default:
		break;
	}

	return false;
}

static int detectID_EEPROM(IdAssignmentTypeDef *ids)
{
	// ====== EEPROM Check ======
	// EEPROM spec reserves 2 bytes for the ID buffer.
	// Currently we only use one byte for IDs, both here in the firmware
	// and in the IDE - once we deplete that ID pool, this needs to be extended
	// (uint8_t to uint16_t and change EEPROM read to read two bytes instead of one)
	uint8_t idBuffer[2];
	// ====== CH1 ======
	if(ids->ch1.state != ID_STATE_DONE)
	{
		// EEPROM is not ready -> assume it is not connected -> skip EEPROM ID read
		if(!eeprom_check(&SPI.ch1))
		{
			eeprom_read_array(&SPI.ch1, EEPROM_ADDR_ID, &idBuffer[0], 1);
			ids->ch1.id = idBuffer[0];
			// ID was correctly detected via EEPROM
			if(ids->ch1.id)
			{
				ids->ch1.state = ID_STATE_DONE;
				IdState.ch1.detectedBy = FOUND_BY_EEPROM;
			}
		}
		// EEPROM access changes the ID_CH0 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #1
		PORT_PCR_REG(HAL.IOs->pins->ID_CH0.portBase, HAL.IOs->pins->ID_CH0.bit) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | 0x000000;
	}

	// ====== CH2 ======
	if(ids->ch2.state != ID_STATE_DONE)
	{
		// EEPROM is not ready -> assume it is not connected -> skip EEPROM ID read
		if(!eeprom_check(&SPI.ch2))
		{
			eeprom_read_array(&SPI.ch2, EEPROM_ADDR_ID, &idBuffer[0], 1);
			ids->ch2.id = idBuffer[0];
			//id was correctly detected via EEPROM
			if(ids->ch2.id)
			{
				ids->ch2.state = ID_STATE_DONE;
				IdState.ch2.detectedBy = FOUND_BY_EEPROM;
			}
		}
		// EEPROM access changes the ID_CH1 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #2
		PORT_PCR_REG(HAL.IOs->pins->ID_CH1.portBase, HAL.IOs->pins->ID_CH1.bit) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | 0x000000;
	}

	return true;
}
