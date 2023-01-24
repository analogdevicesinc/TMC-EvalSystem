/*
 * IdDetection_Startrampe.c
 *
 *  Created on:  20.01.2023
 *  Author:      LK
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
static int detectID_Monoflop(IdAssignmentTypeDef *ids);
static int detectID_EEPROM(IdAssignmentTypeDef *ids);

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

/* Initialise timer and GPIO edge-triggered interrupts */
void IDDetection_init(void)
{

}

void IDDetection_deInit()
{

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
	return true;

}

void IDDetection_initialScan(IdAssignmentTypeDef *ids)
{

}

static int detectID_Monoflop(IdAssignmentTypeDef *ids)
{
	return false;
}

static int detectID_EEPROM(IdAssignmentTypeDef *ids)
{
	return true;
}
