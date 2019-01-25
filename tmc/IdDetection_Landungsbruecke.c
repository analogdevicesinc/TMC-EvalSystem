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

#include "../hal/derivative.h"
#include "../hal/HAL.h"
#include "IdEeprom.h"
#include "BoardAssignment.h"
#include "IdDetection.h"
#include "VitalSignsMonitor.h"
#include "TMCL.h"

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

static uint8 assign(uint32 pulse);

#define TIMER_START  5537
#define FULLCOUNTER  60000 // 65536 - TIMER_START + 1
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

static uint32 counter = 0;

static bool isScanning;
IdAssignmentTypeDef IdState = { 0 };

// Interrupt: Edge on GPIO Port B
void PORTB_IRQHandler(void)
{
	// Store the timing values
	uint32 timerVal = FTM2_CNT;
	uint32 counterVal = counter;

	// Store the interrupt flag state and then reset the flags
	uint32 interruptFlags = PORTB_ISFR;
	PORTB_ISFR = PORT_ISFR_ISF_MASK;

	// Abort if we're not scanning
	if(!isScanning)
		return;

	// ======== CH0 ==========
	// Check if Pin ID_CH0 generated the interrupt
	if(interruptFlags & PORT_ISFR_ISF(HAL.IOs->pins->ID_CH0.bitWeight))
	{
		if(IdState.ch1.state == ID_STATE_WAIT_HIGH)
		{	// Second ID pulse edge - store timer values -> state DONE
			IdState.ch1.timer_2    = timerVal;
			IdState.ch1.counter_2  = counterVal;
			IdState.ch1.state      = ID_STATE_DONE;
		}
		else
		{	// First ID pulse edge - store timer values -> state WAIT_HIGH
			IdState.ch1.timer_1    = timerVal;
			IdState.ch1.counter_1  = counterVal;
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
			IdState.ch2.counter_2  = counterVal;
			IdState.ch2.state      = ID_STATE_DONE;
		}
		else
		{	// First ID pulse edge - store timer values -> state WAIT_HIGH
			IdState.ch2.timer_1    = timerVal;
			IdState.ch2.counter_1  = counterVal;
			IdState.ch2.state      = ID_STATE_WAIT_HIGH;
		}
	}
}

void FTM2_IRQHandler()
{
	// clear timer overflow flag
	FTM2_SC &= ~FTM_SC_TOF_MASK;

	counter++;

	// Abort if timeout limit hasn't been reached yet
	if(counter < 100)
		return;

	// Reset counter and stop the timer
	counter = 0;
	FTM2_SC &= ~FTM_SC_CLKS_MASK;

	// Abort if we're not scanning
	if(!isScanning)
		return;

	// Set the Timeout states
	if(IdState.ch1.state == ID_STATE_WAIT_HIGH)
	{	// Only detected ID pulse rising edge -> Timeout
		IdState.ch1.state = ID_STATE_TIMEOUT;
	}
	else if(IdState.ch1.state == ID_STATE_WAIT_LOW)
	{	// Did not detect any edge -> No answer
		IdState.ch1.state = ID_STATE_NO_ANSWER;
	}

	if(IdState.ch2.state == ID_STATE_WAIT_HIGH)
	{	// Only detected ID pulse rising edge -> Timeout
		IdState.ch2.state = ID_STATE_TIMEOUT;
	}
	else if(IdState.ch2.state == ID_STATE_WAIT_LOW)
	{	// Did not detect any edge -> No answer
		IdState.ch2.state = ID_STATE_NO_ANSWER;
	}
}

void IDDetection_init(void)
{
	isScanning = FALSE;

	// ====== Timer initialisation =======
	// Enable clock for FTM2
	SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;

	// Disable write protection, FTM specific registers are available
	FTM2_MODE |= FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK | FTM_MODE_FAULTM_MASK;

	// Clock source: System Clock (48 MHz), Prescaler: 8 -> 6 MHz Timer clock
	FTM2_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(3);

	// ( MOD  - CNTIN + 1) / 6 MHz = Timer period
	// (65536 -  5537 + 1) / 6 MHz = 10 ms
	FTM2_CNTIN = TIMER_START;

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
static uint8 assign(uint32 pulse)
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
uint8 IDDetection_detect(IdAssignmentTypeDef *out)
{
	if(!isScanning)
	{
		FTM2_SC &= ~FTM_SC_CLKS_MASK;  // stop timer
		FTM2_CNTIN = TIMER_START;      // clear counter

		IdState.ch1.state       = ID_STATE_WAIT_LOW;
		IdState.ch1.detectedBy  = FOUND_BY_NONE;
		IdState.ch2.state       = ID_STATE_WAIT_LOW;
		IdState.ch2.detectedBy  = FOUND_BY_NONE;
		isScanning = TRUE;

		FTM2_SC |= FTM_SC_CLKS(1);  // start timer
		ID_CLK_HIGH();

		return FALSE;
	}

	if(!IDSTATE_SCAN_DONE(IdState))
		return FALSE;

	// Scan complete
	isScanning = FALSE;
	ID_CLK_LOW();
	FTM2_SC &= ~FTM_SC_CLKS_MASK; // stop timer

	// ======== CH0 ==========
	// Assign ID detection state for this channel
	out->ch1.state = IdState.ch1.state;

	if(IdState.ch1.state == ID_STATE_DONE)
	{
		// Assign the ID derived from the ID pulse duration
		uint32 tickDiff =    (IdState.ch1.counter_2 - IdState.ch1.counter_1) * FULLCOUNTER
						   + (IdState.ch1.timer_2   - IdState.ch1.timer_1);
		out->ch1.id = assign(tickDiff * TICK_FACTOR);

		if(out->ch1.id)
			IdState.ch1.detectedBy = FOUND_BY_MONOFLOP;
		else
			out->ch1.state = ID_STATE_INVALID; // Invalid ID pulse detected
	}
	else
	{
		out->ch1.id = 0;
	}

	// ======== CH1 ==========
	// Assign ID detection state for this channel
	out->ch2.state 	= IdState.ch2.state;

	if(IdState.ch2.state == ID_STATE_DONE)
	{
		// Assign the ID derived from the ID pulse duration
		uint32 tickDiff =    (IdState.ch2.counter_2 - IdState.ch2.counter_1) * FULLCOUNTER
						   + (IdState.ch2.timer_2   - IdState.ch2.timer_1);
		out->ch2.id = assign(tickDiff * TICK_FACTOR);

		if(out->ch2.id)
			IdState.ch2.detectedBy = FOUND_BY_MONOFLOP;
		else
			out->ch2.state = ID_STATE_INVALID; // Invalid ID pulse detected
	}
	else
	{
		out->ch2.id = 0;
	}

	// ====== EEPROM Check ======
	// EEPROM spec reserves 2 bytes for the ID buffer.
	// Currently we only use one byte for IDs, both here in the firmware
	// and in the IDE - once we deplete that ID pool, this needs to be extended
	// (uint8 to uint16 and change EEPROM read to read two bytes instead of one)
	uint8 idBuffer[2];
	// ====== CH1 ======
	if(!out->ch1.id)
	{
		// EEPROM is not ready -> assume it is not connected -> skip EEPROM ID read
		if(!checkEeprom(&SPI.ch1))
		{
			readBoardIdEepromBlock(&SPI.ch1, IDEEPROM_ADDR_ID, &idBuffer[0], 1);
			out->ch1.id = idBuffer[0];
			// ID was correctly detected via EEPROM
			if(out->ch1.id)
			{
				out->ch1.state = ID_STATE_DONE;
				IdState.ch1.detectedBy = FOUND_BY_EEPROM;
			}
		}
		// EEPROM access changes the ID_CH0 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #1
		PORT_PCR_REG(HAL.IOs->pins->ID_CH0.portBase, HAL.IOs->pins->ID_CH0.bit) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | 0x000000;
	}

	// ====== CH2 ======
	if(!out->ch2.id)
	{
		// EEPROM is not ready -> assume it is not connected -> skip EEPROM ID read
		if(!checkEeprom(&SPI.ch2))
		{
			readBoardIdEepromBlock(&SPI.ch2, IDEEPROM_ADDR_ID, &idBuffer[0], 1);
			out->ch2.id = idBuffer[0];
			//id was correctly detected via EEPROM
			if(out->ch2.id)
			{
				out->ch2.state = ID_STATE_DONE;
				IdState.ch2.detectedBy = FOUND_BY_EEPROM;
			}
		}
		// EEPROM access changes the ID_CH1 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #2
		PORT_PCR_REG(HAL.IOs->pins->ID_CH1.portBase, HAL.IOs->pins->ID_CH1.bit) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0B) | PORT_PCR_PE_MASK | 0x000000;
	}

	return TRUE;
}

void IDDetection_initialScan(IdAssignmentTypeDef *ids)
{
	while(!IDDetection_detect(ids))
	{
		vitalsignsmonitor_checkVitalSigns();
		tmcl_process();
	}
}

