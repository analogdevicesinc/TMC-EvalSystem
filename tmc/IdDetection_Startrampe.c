/*
 * IdDetection_Startrampe.c
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

#include "IdDetection.h"

#include "../hal/derivative.h"
#include "../hal/HAL.h"
#include "IdEeprom.h"
#include "BoardAssignment.h"
#include "IdDetection.h"
#include "VitalSignsMonitor.h"
#include "TMCL.h"

// Helper macros
#define ID_CLK_LOW()   GPIOC->BSRRH=BIT9    // set id clk signal to low
#define ID_CLK_HIGH()  GPIOC->BSRRL=BIT9    // set id clk signal to high
#define ID_CLK_STATE   (GPIOC->ODR & BIT9)  // get id clk signal level
#define ID_CH0_STATE   (GPIOC->IDR & BIT0)  // get id signal level for this channel
#define ID_CH1_STATE   (GPIOC->IDR & BIT1)  // get id signal level for this channel
#define IDSTATE_SCAN_DONE(ID_STATE)                             \
            (                                                   \
                (ID_STATE.ch1.state != ID_STATE_WAIT_LOW)  &&   \
                (ID_STATE.ch2.state != ID_STATE_WAIT_LOW)  &&   \
                (ID_STATE.ch1.state != ID_STATE_WAIT_HIGH) &&   \
                (ID_STATE.ch2.state != ID_STATE_WAIT_HIGH)      \
            )

static uint8 assign(uint32 pulse);

static bool isScanning;
IdAssignmentTypeDef IdState = { 0 };

/* pin changed interrupt to detect edges of ID pulse for this channel */
void __attribute__ ((interrupt)) EXTI0_IRQHandler(void)
{
	// Abort if the EXTI line isn't set
	if(EXTI_GetITStatus(EXTI_Line0) != SET)
		return;

	EXTI_ClearITPendingBit(EXTI_Line0);

	// Abort if we're not scanning
	if(!isScanning)
		return;

	if(ID_CH0_STATE) // Capture time of rising edge on ID_CH0
	{
		TIM5->EGR |= TIM_EGR_CC1G;
		IdState.ch1.state = ID_STATE_WAIT_HIGH;
	}
	else // Capture time of falling edge on ID_CH0
	{
		TIM5->EGR |= TIM_EGR_CC2G;
		IdState.ch1.state = ID_STATE_DONE;
	}
}

/* pin changed interrupt to detect edges of ID pulse for this channel */
void __attribute__ ((interrupt)) EXTI1_IRQHandler(void)
{
	// Abort if the EXTI line isn't set
	if(EXTI_GetITStatus(EXTI_Line1) != SET)
		return;

	EXTI_ClearITPendingBit(EXTI_Line1);

	// Abort if we're not scanning
	if(!isScanning)
		return;

	if(ID_CH1_STATE) // Capture time of rising edge on ID_CH1
	{
		TIM5->EGR |= TIM_EGR_CC3G;
		IdState.ch2.state = ID_STATE_WAIT_HIGH;
	}
	else // Capture time of falling edge on ID_CH1
	{
		TIM5->EGR |= TIM_EGR_CC4G;
		IdState.ch2.state = ID_STATE_DONE;
	}
}

/* timer interrupt to determine timeout on ID detection (missing boards, errors) */
void TIM5_IRQHandler(void)
{
	// Abort if the interrupt isn't set
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != SET)
		return;

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

	if(!isScanning)
		return;

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

/* Initialise timer and GPIO edge-triggered interrupts */
void IDDetection_init(void)
{
	isScanning = FALSE;

	// ====== Pin initialisation ======
	// Pin ID_CLK
	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);

	// Pin ID_CH0
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line     = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode     = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger  = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd  = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel                    = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line0);

	// Pin ID_CH1
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH1);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

	EXTI_InitStructure.EXTI_Line     = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode     = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger  = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd  = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel                    = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line1);

	// ====== Timer initialisation
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_DeInit(TIM5);

	TIM5->CNT = 0;

	TIM5->CR1 |= (uint16)
		(
				TIM_CR1_ARPE  // buffered auto reload register
				//| TIM_CR1_OPM  // stop on update
		);

	TIM5->DIER |= (uint16)
		(
				TIM_DIER_UIE  // update interrupt
				//| TIM_DIER_CC1IE
		);


	TIM5->CCMR1 |= (uint16)
		(
				TIM_CCMR1_CC1S  // channel as input
				| TIM_CCMR1_CC2S  // channel as input
		);

	TIM5->CCMR2 |= (uint16)
		(
				TIM_CCMR2_CC3S  // channel as input
				| TIM_CCMR2_CC4S  // channel as input
		);

	// Set CCER after CCMR1 and CCMR2
	TIM5->CCER |= (uint16)
		(
				TIM_CCER_CC1E   // capture enable
				| TIM_CCER_CC2E // capture enable
				| TIM_CCER_CC3E // capture enable
				| TIM_CCER_CC4E // capture enable
		);

	TIM5->PSC = 5;         // prescaler -> 0.1us
	TIM5->ARR = 13500000;  // timeout   -> 1.35s

	TIM5->EGR |= (uint16)
		(
			TIM_EGR_UG // update event to initialize
		);

	// Enable timer interrupt
	NVIC_InitStructure.NVIC_IRQChannel                    = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void IDDetection_deInit()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel     = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd  = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel     = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd  = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel     = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd  = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit(TIM5);
	EXTI_DeInit();
}

// Returns ID assigned to given pulse (length in 0.1us)
static uint8 assign(uint32 pulse)
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
uint8 IDDetection_detect(IdAssignmentTypeDef *out)
{
	if(!isScanning)
	{
		TIM_Cmd(TIM5, DISABLE);  // stop timer
		TIM5->CNT = 0;           // clear counter

		IdState.ch1.state       = ID_STATE_WAIT_LOW;
		IdState.ch1.detectedBy  = FOUND_BY_NONE;
		IdState.ch2.state       = ID_STATE_WAIT_LOW;
		IdState.ch2.detectedBy  = FOUND_BY_NONE;
		isScanning = TRUE;

		TIM_Cmd(TIM5, ENABLE);   // start timer
		ID_CLK_HIGH();

		return FALSE;
	}

	// Abort if we're still scanning
	if(!IDSTATE_SCAN_DONE(IdState))
		return FALSE;

	// Scan done, disable ID_CLK and the timer
	isScanning = FALSE;
	ID_CLK_LOW();
	TIM_Cmd(TIM5, DISABLE);

	// ======== CH1 ==========
	// Assign ID detection state for this channel
	out->ch1.state 	= IdState.ch1.state;

	if(IdState.ch1.state == ID_STATE_DONE)
	{
		// Assign the ID derived from the ID pulse duration
		out->ch1.id = assign(TIM5->CCR2 - TIM5->CCR1);

		if(out->ch1.id)
			IdState.ch1.detectedBy = FOUND_BY_MONOFLOP;
		else
			out->ch1.state = ID_STATE_INVALID; // Invalid ID pulse detected
	}
	else
	{
		out->ch1.id = 0;
	}

	// ======== CH2 ==========
	// Assign ID detection state for this channel
	out->ch2.state 	= IdState.ch2.state;

	if(IdState.ch2.state == ID_STATE_DONE)
	{
		// Assign the ID derived from the ID pulse duration
		out->ch2.id = assign(TIM5->CCR4 - TIM5->CCR3);

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
		// EEPROM access changes the ID_CH0 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #3
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH0);
	}

	// ====== CH2 ======
	if(!out->ch2.id)
	{
		// EEPROM is not ready -> assume it is not connected -> skip EEPROM ID read
		if(!checkEeprom(&SPI.ch2))
		{
			readBoardIdEepromBlock(&SPI.ch2, IDEEPROM_ADDR_ID, &idBuffer[0], 1);
			out->ch2.id = idBuffer[0];
			// ID was correctly detected via EEPROM
			if(out->ch2.id)
			{
				out->ch2.state = ID_STATE_DONE;
				IdState.ch2.detectedBy = FOUND_BY_EEPROM;
			}
		}
		// EEPROM access changes the ID_CH1 pin configuration -> write it again // todo CHECK 2: workaround, do this better later (LH) #4
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH1);
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
