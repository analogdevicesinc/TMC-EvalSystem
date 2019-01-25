/*
 * nvic.h
 *
 *  Created on: Nov 6, 2012
 *      Author: B34443
 */

#ifndef NVIC_H_
#define NVIC_H_

	/* ARM Cortex M0 implementation for interrupt priority shift */
	#define ARM_INTERRUPT_LEVEL_BITS          2

#ifndef EnableInterrupts
	#define EnableInterrupts asm(" CPSIE i")
#endif

#ifndef DisableInterrupts
	#define DisableInterrupts asm(" CPSID i")
#endif

	void enable_irq(int);
	void disable_irq(int);
	void set_irq_priority(int, int);

#endif /* NVIC_H_ */
