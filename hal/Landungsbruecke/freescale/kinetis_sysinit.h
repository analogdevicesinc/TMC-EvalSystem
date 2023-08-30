/*
  FILE    : kinetis_sysinit.h
  PURPOSE : system initialization header for Kinetis ARM architecture
  LANGUAGE: C
  Copyright © 2012 Freescale semiConductor Inc. All Rights Reserved.
*/
#ifndef KINETIS_SYSINIT_H
#define KINETIS_SYSINIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tmc/helpers/API_Header.h"

/* Word to be written in SIM_COP in order to disable the Watchdog */
#define KINETIS_WDOG_DISABLED_CTRL	0x0

/*
	Initializes the Kinetis hardware: e.g. disables the Watchdog
*/
void __init_hardware(void);

/*
** ===================================================================
**     Method      :  Default_Handler
**
**     Description :
**         The default interrupt handler.
** ===================================================================
*/
void Default_Handler(void);

void SetBASEPRI(uint32 Level) ;


#ifdef __cplusplus
}
#endif

#endif /* #ifndef KINETIS_SYSINIT_H */
