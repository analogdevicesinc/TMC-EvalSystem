#ifndef VITAL_SIGNS_MONITOR_H
#define VITAL_SIGNS_MONITOR_H

#include "tmc/helpers/API_Header.h"

typedef struct
{
	int8_t    debugMode;    // while debugMode is set, the error LED does not get set by VSM and status LED heartrate does not get updated
	int8_t    busy;         // if module is busy, the status LED is blinking fast
	uint8_t   brownOut;     // Undervoltage condition flags
	uint8_t   overVoltage;  // Overvoltage condition flags
	int32_t   errorMask;    // error mask, each bit stands for one error bit, 1 means error will be reported
	int32_t   errors;       // actual error bits
	uint32_t  heartRate;    // status LED blinking frequency
	uint32_t  VM;           // actual measured motor supply VM
} VitalSignsMonitorTypeDef;

extern VitalSignsMonitorTypeDef VitalSignsMonitor; // global implementation of interface for system

// error bits
#define VSM_CHX                     (1<<0)   // any errors on the evalboards
#define VSM_CH1                     (1<<1)   // any errors on motion controller board
#define VSM_CH2                     (1<<2)   // any errors on driver board

#define VSM_ERRORS_VM               (1<<0)   // something's wrong with the motor supply VM for any board
#define VSM_ERRORS_CH1              (1<<1)   // something's wrong with the motor supply VM for motion controller board
#define VSM_ERRORS_CH2              (1<<2)   // something's wrong with the motor supply VM for driver board
#define VSM_BUSY                    (1<<3)   // any board is busy
#define VSM_BUSY_CH1                (1<<4)   // motion controller board is busy
#define VSM_BUSY_CH2                (1<<5)   // driver board is busy
#define VSM_WARNING_CPU_SUPPLY_LOW  (1<<6)   // motor supply VM is to low for the processor -> Supply over USB needed
#define VSM_ERRORS_BROWNOUT_CH1     (1<<7)   // motor supply VM is to low for motion controller board
#define VSM_ERRORS_BROWNOUT_CH2     (1<<8)   // motor supply VM is to low for driver board
#define VSM_ERRORS_OVERVOLTAGE      (1<<9)   // motor supply VM is to high for any board
#define VSM_ERRORS_OVERVOLTAGE_CH1  (1<<10)  // motor supply VM is to high for motion controller board
#define VSM_ERRORS_OVERVOLTAGE_CH2  (1<<11)  // motor supply VM is to high for driver board

#define VSM_ERRORS_VIO_LOW          (1 << 1)

void vitalsignsmonitor_checkVitalSigns();
void vitalsignsmonitor_clearOvervoltageErrors();

#endif /* VITAL_SIGNS_MONITOR_H */
