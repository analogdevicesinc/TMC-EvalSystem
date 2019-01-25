#ifndef _DERIVATIVE_H_
	#define  _DERIVATIVE_H_

/* This is purely for the Eclipse parsing.
 * while working on Board-specific code you can
 * select one of the two boards here.
 * The Build process already selects one via makefile & makeFor.h files,
 * this choice will therefore not influence the build process.
 */
#if !defined(Landungsbruecke) && !defined(Startrampe)
#warning "No Board selected by makefile, defining one for debug purposes"
#define Landungsbruecke
//#define Startrampe
#endif

	#include "tmc/helpers/API_Header.h"

	#if defined(Startrampe)
		#define MODULE_ID "0011"
		#include "stm32f2xx.h"
	#elif defined(Landungsbruecke)
		#define MODULE_ID "0012"
		#include <MK20D10.h>
		#include "hal/Landungsbruecke/freescale/Cpu.h"
		#include "hal/Landungsbruecke/freescale/nvic.h"
		#define CPU_LITTLE_ENDIAN
		#define __MK_xxx_H__
	#else
	#error "No Board selected"
	#endif

#endif /* _DERIVATIVE_H_ */
