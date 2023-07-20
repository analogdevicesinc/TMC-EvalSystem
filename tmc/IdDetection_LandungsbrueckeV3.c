/*
 *
 *  Created on:  07.02.2023
 *  Author:      ASU
 *
 *  Calling IDDetection_detect(IdAssignmentTypeDef *result) will start the ID detection process.
 *  The function returns the ID Results through the IdAssignmentTypeDef struct result points to.
 *  The detection will be done by monoflop pulse duration measurement or via EEPROM readout.
 *  While this process is still ongoing the function will return ID_STATE_WAIT_HIGH. Once the
 *  ID detection of both channels has been finished, ID_STATE_DONE will be returned.
 *
 *  Calling the function again after the detection has finished will start another scan.
 */

#include "IdDetection.h"

#include "hal/derivative.h"
#include "hal/HAL.h"
#include "BoardAssignment.h"
#include "EEPROM.h"
#include "IdDetection.h"
#include "VitalSignsMonitor.h"
#include "TMCL.h"

// Helper functions
static int32_t detectID_Monoflop(IdAssignmentTypeDef *ids);
static int32_t detectID_EEPROM(IdAssignmentTypeDef *ids);

// Helper macros
#define ID_CLK_LOW()   HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CLK)    // set id clk signal to low
#define ID_CLK_HIGH()  HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CLK)    // set id clk signal to high
#define ID_CLK_STATE   (HAL.IOs->config->getState(&HAL.IOs->pins->ID_CLK) == IOS_HIGH)  // get id clk signal level
#define ID_CH0_STATE   (HAL.IOs->config->getState(&HAL.IOs->pins->ID_CH0) == IOS_HIGH)  // get id signal level for this channel
#define ID_CH1_STATE   (HAL.IOs->config->getState(&HAL.IOs->pins->ID_CH1) == IOS_HIGH)  // get id signal level for this channel
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

IdAssignmentTypeDef IdState = { 0 };

/* pin changed interrupt to detect edges of ID pulse for both channels */
void PC7_8_IRQHandler(void)
{
	// Abort if the EXTI line isn't set
	if((exti_flag_get(EXTI_8) != SET) && (exti_flag_get(EXTI_7) != SET)){
		return;
	}

	if(exti_flag_get(EXTI_8) == SET)
	{
		exti_flag_clear(EXTI_8);

			// Abort if we're not scanning
			if(monoflopState != MONOFLOP_SCANNING)
				return;

			if(ID_CH0_STATE) // Capture time of rising edge on ID_CH0
			{
				TIMER_SWEVG(TIMER1) |= TIMER_SWEVG_CH0G;
				IdState.ch1.state = ID_STATE_WAIT_HIGH;
			}
			else // Capture time of falling edge on ID_CH0
			{
				TIMER_SWEVG(TIMER1) |= TIMER_SWEVG_CH1G;
				IdState.ch1.state = ID_STATE_DONE;
			}
	}
	else if(exti_flag_get(EXTI_7) == SET)
	{
		exti_flag_clear(EXTI_7);

			// Abort if we're not scanning
			if(monoflopState != MONOFLOP_SCANNING)
				return;

			if(ID_CH1_STATE) // Capture time of rising edge on ID_CH1
			{
				TIMER_SWEVG(TIMER1) = TIMER_SWEVG_CH2G;
				IdState.ch2.state = ID_STATE_WAIT_HIGH;
			}
			else // Capture time of falling edge on ID_CH1
			{
				TIMER_SWEVG(TIMER1) = TIMER_SWEVG_CH3G;
				IdState.ch2.state = ID_STATE_DONE;
			}
	}



}

void __attribute__ ((interrupt)) EXTI5_9_IRQHandler(void)
{

	if(GET_BITS(SYSCFG_EXTISS2,0,3) == 3){
		PD8_IRQHandler(); // For TMC6140-eval diagnostics
	}
	else if(GET_BITS(SYSCFG_EXTISS2,0,3) == 2 || GET_BITS(SYSCFG_EXTISS1,12,15) == 2){
		PC7_8_IRQHandler(); // For idDetection
	}
}

/* timer interrupt to determine timeout on ID detection (missing boards, errors) */
void TIMER1_IRQHandler(void)
{
	// Abort if the interrupt isn't set
	if(timer_flag_get(TIMER1, TIMER_FLAG_UP) != SET)
		return;

	timer_flag_clear(TIMER1, TIMER_FLAG_UP);

	if(monoflopState == MONOFLOP_SCANNING)
	{
		monoflopState = MONOFLOP_DONE;
		return;
	}
}

/* Initialise timer and GPIO edge-triggered interrupts */
void IDDetection_init(void)
{
	monoflopState = MONOFLOP_INIT;

	// ====== Pin initialisation ======
	// Pin ID_CLK
	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);

	// Pin ID_CH0
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

	syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN8);

	exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	nvic_irq_enable(EXTI5_9_IRQn, 0, 1);
	exti_interrupt_flag_clear(EXTI_8);

	// Pin ID_CH1
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH1);

	syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN7);

	exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	nvic_irq_enable(EXTI5_9_IRQn, 0, 1);
	exti_interrupt_flag_clear(EXTI_7);

	// ====== Timer initialisation

	/* Timer Frequency:
	 *   CLK_TIMER      120MHz
	 *  -----------  =  ----- = 10MHz = 0.1us <= COUNTER_CLK
	 *   Prescaler        12
	 *
	 */

	rcu_periph_clock_enable(RCU_TIMER1);
	rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL2); // CK_TIMER1 = 2 x CK_APB1
	timer_internal_clock_config(TIMER1);
	timer_deinit(TIMER1);

	timer_counter_value_config(TIMER1, 0);



	timer_ic_parameter_struct *icpara_rising;
	timer_ic_parameter_struct *icpara_falling;

	icpara_rising->icpolarity  = TIMER_IC_POLARITY_RISING;
	icpara_rising->icselection = TIMER_IC_SELECTION_DIRECTTI;
	icpara_rising->icprescaler = TIMER_IC_PSC_DIV1;
	icpara_rising->icfilter    = 0U;

	icpara_falling->icpolarity  = TIMER_IC_POLARITY_FALLING;
	icpara_falling->icselection = TIMER_IC_SELECTION_DIRECTTI;
	icpara_falling->icprescaler = TIMER_IC_PSC_DIV1;
	icpara_falling->icfilter    = 0U;

	timer_input_capture_config(TIMER1, TIMER_CH_0, icpara_rising);
	timer_input_capture_config(TIMER1, TIMER_CH_1, icpara_falling);
	timer_input_capture_config(TIMER1, TIMER_CH_2, icpara_rising);
	timer_input_capture_config(TIMER1, TIMER_CH_3, icpara_falling);

	// Setting the prescaler i.e 11
	timer_prescaler_config(TIMER1, 11, TIMER_PSC_RELOAD_NOW); // Divides the timer freq by (prescaler + 1)

	timer_autoreload_value_config(TIMER1,100000); // timeout   -> 10ms --TIM auto-reload register


	timer_update_event_enable(TIMER1);
    TIMER_SWEVG(TIMER1) |= (uint32_t)TIMER_SWEVG_UPG; //generate update event

	timer_interrupt_enable(TIMER1, TIMER_INT_UP);

	// Enable timer interrupt
	nvic_irq_enable(TIMER1_IRQn, 0xF, 0xF);


}

void IDDetection_deInit()
{

	nvic_irq_disable(TIMER1_IRQn);
	nvic_irq_disable(EXTI5_9_IRQn);
	timer_deinit(TIMER1);
	exti_deinit();
}

// Returns ID assigned to given pulse (length in 0.1us)
static uint8_t assign(uint32_t pulse)
{
	if(     pulse < 5)      return 0; // error
	else if(pulse < 110)    return 1;
	else if(pulse < 135)    return 2;
	else if(pulse < 165)    return 3;
	else if(pulse < 200)    return 4;
	else if(pulse < 245)    return 5;
	else if(pulse < 300)    return 6;
	else if(pulse < 360)    return 7;
	else if(pulse < 430)    return 8;
	else if(pulse < 515)    return 9;
	else if(pulse < 620)    return 10;
	else if(pulse < 750)    return 11;
	else if(pulse < 910)    return 12;
	else if(pulse < 1100)   return 13;
	else if(pulse < 1350)   return 14;
	else if(pulse < 1650)   return 15;
	else if(pulse < 2000)   return 16;
	else if(pulse < 2450)   return 17;
	else if(pulse < 3000)   return 18;
	else if(pulse < 3600)   return 19;
	else if(pulse < 4300)   return 20;
	else if(pulse < 5150)   return 21;
	else if(pulse < 6200)   return 22;
	else if(pulse < 7500)   return 23;
	else if(pulse < 9100)   return 24;
	else if(pulse < 11000)  return 25;
	else if(pulse < 13500)  return 26;
	else if(pulse < 16500)  return 27;
	else if(pulse < 20000)  return 28;
	else if(pulse < 24500)  return 29;
	else if(pulse < 30000)  return 30;
	else if(pulse < 36000)  return 31;
	else if(pulse < 43000)  return 32;
	else if(pulse < 51500)  return 33;
	else if(pulse < 62000)  return 34;
	else if(pulse < 75000)  return 35;
	else if(pulse < 91000)  return 36;

	return 0; // error
}

// Detect IDs of attached boards - returns true when done
uint8_t IDDetection_detect(IdAssignmentTypeDef *ids)
{
	// Change the exti source back to PC8 (It could be changed by TMC6140-eval to PD8)
//	syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN8);

	// Try to identify the IDs via monoflop pulse duration
	if (!detectID_Monoflop(ids))
		return false;

	// Try to identify the IDs via EEPROM readout
	detectID_EEPROM(ids);

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

static int32_t detectID_Monoflop(IdAssignmentTypeDef *ids)
{
	switch(monoflopState)
	{
	case MONOFLOP_INIT:
		timer_disable(TIMER1); // stop the timer
		timer_counter_value_config(TIMER1, 0); // clear counter

		IdState.ch1.state       = ID_STATE_WAIT_LOW;
		IdState.ch1.detectedBy  = FOUND_BY_NONE;
		IdState.ch2.state       = ID_STATE_WAIT_LOW;
		IdState.ch2.detectedBy  = FOUND_BY_NONE;

		// Update the monoflop state before activating the timer. Otherwise bad
		// luck with other unrelated interrupts might cause enough delay to
		// trigger the timer overflow after starting the timer before updating
		// this state - which results in the timeout no longer working.
		monoflopState = MONOFLOP_SCANNING;

		timer_enable(TIMER1);   // start timer
		ID_CLK_HIGH();
		break;
	case MONOFLOP_SCANNING:
		if (IDSTATE_SCAN_DONE(IdState))
		{
			monoflopState = MONOFLOP_DONE;
		}
		break;
	case MONOFLOP_DONE:
		// Scan complete, disable ID_CLK and the timer
		ID_CLK_LOW();
		timer_disable(TIMER1);

		// ======== CH1 ==========
		// Assign ID detection state for this channel
		ids->ch1.state 	= IdState.ch1.state;

		if(IdState.ch1.state == ID_STATE_DONE)
		{
			 uint32_t pulse=timer_channel_capture_value_register_read(TIMER1, TIMER_CH_1) - timer_channel_capture_value_register_read(TIMER1, TIMER_CH_0);

			// Assign the ID derived from the ID pulse duration
			ids->ch1.id = assign(pulse);


			if(ids->ch1.id)
				IdState.ch1.detectedBy = FOUND_BY_MONOFLOP;
			else
				ids->ch1.state = ID_STATE_INVALID; // Invalid ID pulse detected
		}
		else if(IdState.ch1.state == ID_STATE_WAIT_HIGH)
		{	// Only detected ID pulse rising edge -> Timeout
			IdState.ch1.state = ID_STATE_TIMEOUT;
		}
		else if(IdState.ch1.state == ID_STATE_WAIT_LOW)
		{	// Did not detect any edge -> No answer
			IdState.ch1.state = ID_STATE_NO_ANSWER;
		}
		else
		{
			ids->ch1.id = 0;
		}



		// ======== CH2 ==========
		// Assign ID detection state for this channel
		ids->ch2.state 	= IdState.ch2.state;

		if(IdState.ch2.state == ID_STATE_DONE)
		{
			// Assign the ID derived from the ID pulse duration
			ids->ch2.id = assign(timer_channel_capture_value_register_read(TIMER1, TIMER_CH_3) - timer_channel_capture_value_register_read(TIMER1, TIMER_CH_2));

			if(ids->ch2.id)
				IdState.ch2.detectedBy = FOUND_BY_MONOFLOP;
			else
				ids->ch2.state = ID_STATE_INVALID; // Invalid ID pulse detected
		}
		else if(IdState.ch2.state == ID_STATE_WAIT_HIGH)
		{	// Only detected ID pulse rising edge -> Timeout
			IdState.ch2.state = ID_STATE_TIMEOUT;
		}
		else if(IdState.ch2.state == ID_STATE_WAIT_LOW)
		{	// Did not detect any edge -> No answer
			IdState.ch2.state = ID_STATE_NO_ANSWER;
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

static int32_t detectID_EEPROM(IdAssignmentTypeDef *ids)
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
		// EEPROM access changes the ID_CH0 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #3
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH0);
	}

	// ====== CH2 ======
	if(ids->ch2.state != ID_STATE_DONE)
	{
		// EEPROM is not ready -> assume it is not connected -> skip EEPROM ID read
		if(!eeprom_check(&SPI.ch2))
		{
			eeprom_read_array(&SPI.ch2, EEPROM_ADDR_ID, &idBuffer[0], 1);
			ids->ch2.id = idBuffer[0];
			// ID was correctly detected via EEPROM
			if(ids->ch2.id)
			{
				ids->ch2.state = ID_STATE_DONE;
				IdState.ch2.detectedBy = FOUND_BY_EEPROM;
			}
		}
		// EEPROM access changes the ID_CH1 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #4
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH1);
	}

	return true;
}
