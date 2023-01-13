/** \file startup_gd32f405_425_gas.S ***************************************
 *
 *             Startup code for GD32F405 / GD32F425
 *             To be used with GCC / GNU Assembler
 *             and linker scripts gd32f425.ld / gd32f425-tmcm.ld.
 *
 *    Revision History:
 *                    2023_01_05    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

  .syntax unified
	.cpu cortex-m3
	.fpu softvfp
	.thumb

  .global  g_pfnVectors
        
  .word _sidata
  .word _sdata
  .word _edata
  .word _sbss
  .word _ebss
        
  .section	.text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function

Reset_Handler:  
/* Copy the data segment initializers from flash to SRAM */  
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss

/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss
/* Call the clock system intitialization function.*/
  bl  SystemInit   
/* Call the application's entry point.*/
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler

  .section  .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors
        

g_pfnVectors:
  .word     _estack                           /* top of stack */
  .word     Reset_Handler                     /* Reset Handler */
  .word     NMI_Handler                       /* NMI Handler */
  .word     HardFault_Handler                 /* Hard Fault Handler */
  .word     MemManage_Handler                 /* MPU Fault Handler */
  .word     BusFault_Handler                  /* Bus Fault Handler */
  .word     UsageFault_Handler                /* Usage Fault Handler */
  .word     0                                 /* Reserved */
  .word     0                                 /* Reserved */
  .word     0                                 /* Reserved */
  .word     0                                 /* Reserved */
  .word     SVC_Handler                       /* SVCall Handler */
  .word     DebugMon_Handler                  /* Debug Monitor Handler */
  .word     0                                 /* Reserved */
  .word     PendSV_Handler                    /* PendSV Handler */
  .word     SysTick_Handler                   /* SysTick Handler */
                                            
  /* External Interrupts */
  .word     WWDGT_IRQHandler                  /* Vector Number 16,Window Watchdog Timer */
  .word     LVD_IRQHandler                    /* Vector Number 17,LVD through EXTI Line detect */
  .word     TAMPER_STAMP_IRQHandler           /* Vector Number 18,Tamper and TimeStamp Interrupt */
  .word     RTC_WKUP_IRQHandler               /* Vector Number 19,RTC Wakeup through EXTI Line */
  .word     FMC_IRQHandler                    /* Vector Number 20,FMC */
  .word     RCU_CTC_IRQHandler                /* Vector Number 21,RCU and CTC*/
  .word     EXTI0_IRQHandler                  /* Vector Number 22,EXTI Line 0 */
  .word     EXTI1_IRQHandler                  /* Vector Number 23,EXTI Line 1 */
  .word     EXTI2_IRQHandler                  /* Vector Number 24,EXTI Line 2 */
  .word     EXTI3_IRQHandler                  /* Vector Number 25,EXTI Line 3 */
  .word     EXTI4_IRQHandler                  /* Vector Number 26,EXTI Line 4 */
  .word     DMA0_Channel0_IRQHandler          /* Vector Number 27,DMA0 Channel 0 */
  .word     DMA0_Channel1_IRQHandler          /* Vector Number 28,DMA0 Channel 1 */
  .word     DMA0_Channel2_IRQHandler          /* Vector Number 29,DMA0 Channel 2 */
  .word     DMA0_Channel3_IRQHandler          /* Vector Number 30,DMA0 Channel 3 */
  .word     DMA0_Channel4_IRQHandler          /* Vector Number 31,DMA0 Channel 4 */
  .word     DMA0_Channel5_IRQHandler          /* Vector Number 32,DMA0 Channel 5 */
  .word     DMA0_Channel6_IRQHandler          /* Vector Number 33,DMA0 Channel 6 */
  .word     ADC_IRQHandler                    /* Vector Number 34,ADC0 and ADC1 */
  .word     CAN0_TX_IRQHandler                /* Vector Number 35,CAN0 TX */
  .word     CAN0_RX0_IRQHandler               /* Vector Number 36,CAN0 RX0 */
  .word     CAN0_RX1_IRQHandler               /* Vector Number 37,CAN0 RX1 */
  .word     CAN0_EWMC_IRQHandler              /* Vector Number 38,CAN0 EWMC */
  .word     EXTI5_9_IRQHandler                /* Vector Number 39,EXTI Line 5 to EXTI Line 9 */
  .word     TIMER0_BRK_TIMER8_IRQHandler      /* Vector Number 40,TIMER0 Break and TIMER8 global */
  .word     TIMER0_UP_TIMER9_IRQHandler       /* Vector Number 41,TIMER0 Update and TIMER9 global */
  .word     TIMER0_TRG_CMT_TIMER10_IRQHandler /* Vector Number 42,TIMER0 Trigger and Commutation and TIMER10 global */
  .word     TIMER0_Channel_IRQHandler         /* Vector Number 43,TIMER0 Channel Capture Compare */
  .word     TIMER1_IRQHandler                 /* Vector Number 44,TIMER1 */
  .word     TIMER2_IRQHandler                 /* Vector Number 45,TIMER2 */
  .word     TIMER3_IRQHandler                 /* Vector Number 46,TIMER3 */
  .word     I2C0_EV_IRQHandler                /* Vector Number 47,I2C0 Event */
  .word     I2C0_ER_IRQHandler                /* Vector Number 48,I2C0 Error */
  .word     I2C1_EV_IRQHandler                /* Vector Number 49,I2C1 Event */
  .word     I2C1_ER_IRQHandler                /* Vector Number 50,I2C1 Error */
  .word     SPI0_IRQHandler                   /* Vector Number 51,SPI0 */
  .word     SPI1_IRQHandler                   /* Vector Number 52,SPI1 */
  .word     USART0_IRQHandler                 /* Vector Number 53,USART0 */
  .word     USART1_IRQHandler                 /* Vector Number 54,USART1 */
  .word     USART2_IRQHandler                 /* Vector Number 55,USART2 */
  .word     EXTI10_15_IRQHandler              /* Vector Number 56,EXTI Line 10 to EXTI Line 15 */
  .word     RTC_Alarm_IRQHandler              /* Vector Number 57,RTC Alarm through EXTI Line */
  .word     USBFS_WKUP_IRQHandler             /* Vector Number 58,USBFS WakeUp from suspend through EXTI Line */
  .word     TIMER7_BRK_TIMER11_IRQHandler     /* Vector Number 59,TIMER7 Break Interrupt and TIMER11 global */
  .word     TIMER7_UP_TIMER12_IRQHandler      /* Vector Number 60,TIMER7 Update Interrupt and TIMER12 global */
  .word     TIMER7_TRG_CMT_TIMER13_IRQHandler /* Vector Number 61,TIMER7 Trigger and Commutation Interrupt and TIMER13 */
  .word     TIMER7_Channel_IRQHandler         /* Vector Number 62,TIMER7 Channel Capture Compare */
  .word     DMA0_Channel7_IRQHandler          /* Vector Number 63,DMA0 Channel7 */
  .word     0                                 /* Vector Number 64,Reserverd */
  .word     SDIO_IRQHandler                   /* Vector Number 65,SDIO */
  .word     TIMER4_IRQHandler                 /* Vector Number 66,TIMER4 */
  .word     SPI2_IRQHandler                   /* Vector Number 67,SPI2 */
  .word     UART3_IRQHandler                  /* Vector Number 68,UART3 */
  .word     UART4_IRQHandler                  /* Vector Number 69,UART4 */
  .word     TIMER5_DAC_IRQHandler             /* Vector Number 70,TIMER5 and DAC */
  .word     TIMER6_IRQHandler                 /* Vector Number 71,TIMER6 */
  .word     DMA1_Channel0_IRQHandler          /* Vector Number 72,DMA1 Channel0 */
  .word     DMA1_Channel1_IRQHandler          /* Vector Number 73,DMA1 Channel1 */
  .word     DMA1_Channel2_IRQHandler          /* Vector Number 74,DMA1 Channel2 */
  .word     DMA1_Channel3_IRQHandler          /* Vector Number 75,DMA1 Channel3 */
  .word     DMA1_Channel4_IRQHandler          /* Vector Number 76,DMA1 Channel4 */
  .word     0                                 /* Vector Number 77,Reserved */
  .word     0                                 /* Vector Number 78,Reserved */
  .word     CAN1_TX_IRQHandler                /* Vector Number 79,CAN1 TX */
  .word     CAN1_RX0_IRQHandler               /* Vector Number 80,CAN1 RX 0*/
  .word     CAN1_RX1_IRQHandler               /* Vector Number 81,CAN1 RX1 */
  .word     CAN1_EWMC_IRQHandler              /* Vector Number 82,CAN1 EWMC */
  .word     USBFS_IRQHandler                  /* Vector Number 83,USBFS */
  .word     DMA1_Channel5_IRQHandler          /* Vector Number 84,DMA1 Channel5 */
  .word     DMA1_Channel6_IRQHandler          /* Vector Number 85,DMA1 Channel6 */
  .word     DMA1_Channel7_IRQHandler          /* Vector Number 86,DMA1 Channel7 */
  .word     USART5_IRQHandler                 /* Vector Number 87,USART5 */
  .word     I2C2_EV_IRQHandler                /* Vector Number 88,I2C2 Event */
  .word     I2C2_ER_IRQHandler                /* Vector Number 89,I2C2 Error */
  .word     USBHS_EP1_Out_IRQHandler          /* Vector Number 90,USBHS Endpoint 1 Out */
  .word     USBHS_EP1_In_IRQHandler           /* Vector Number 91,USBHS Endpoint 1 In */
  .word     USBHS_WKUP_IRQHandler             /* Vector Number 92,USBHS Wakeup through ETXI Line */
  .word     USBHS_IRQHandler                  /* Vector Number 93,USBHS */
  .word     DCI_IRQHandler                    /* Vector Number 94,DCI */
  .word     0                                 /* Vector Number 95,Reserved */
  .word     TRNG_IRQHandler                   /* Vector Number 96,TRNG */
  .word     FPU_IRQHandler                    /* Vector Number 97,FPU */
        
/************************************
 * Default interrupt handlers.
*************************************/

  .weak NMI_Handler
NMI_Handler:
  b NMI_Handler
          
  .weak HardFault_Handler
HardFault_Handler:
  b HardFault_Handler
        
  .weak MemManage_Handler
MemManage_Handler:
  b MemManage_Handler

  .weak BusFault_Handler
BusFault_Handler:
  b BusFault_Handler

  .weak UsageFault_Handler
UsageFault_Handler:
  b UsageFault_Handler
  
  .weak SVC_Handler
SVC_Handler:
  b SVC_Handler
 
  .weak DebugMon_Handler
DebugMon_Handler:
  b DebugMon_Handler
          
  .weak PendSV_Handler
PendSV_Handler:
  b PendSV_Handler
          
  .weak SysTick_Handler
SysTick_Handler:
  b SysTick_Handler
          
  .weak WWDGT_IRQHandler
WWDGT_IRQHandler:
  b WWDGT_IRQHandler
                  
  .weak LVD_IRQHandler
LVD_IRQHandler:
  b LVD_IRQHandler
  
  .weak TAMPER_STAMP_IRQHandler
TAMPER__STAMP_IRQHandler:
  b TAMPER_STAMP_IRQHandler

  .weak RTC_WKUP_IRQHandler
RTC_WKUP_IRQHandler:
  b RTC_WKUP_IRQHandler
                 
  .weak FMC_IRQHandler
FMC_IRQHandler:
  b FMC_IRQHandler
                  
  .weak RCU_CTC_IRQHandler
RCU_CTC_IRQHandler:
  b RCU_CTC_IRQHandler
                  
  .weak EXTI0_IRQHandler
EXTI0_IRQHandler:
  b EXTI0_IRQHandler
                  
  .weak EXTI1_IRQHandler
EXTI1_IRQHandler:
  b EXTI1_IRQHandler
                  
  .weak EXTI2_IRQHandler
EXTI2_IRQHandler:
  b EXTI2_IRQHandler

  .weak EXTI3_IRQHandler
EXTI3_IRQHandler:
  b EXTI3_IRQHandler

  .weak EXTI4_IRQHandler
EXTI4_IRQHandler:
  b EXTI4_IRQHandler
                  
  .weak DMA0_Channel0_IRQHandler
DMA0_Channel0_IRQHandler:
  b DMA0_Channel0_IRQHandler
          
  .weak DMA0_Channel1_IRQHandler
DMA0_Channel1_IRQHandler:
  b DMA0_Channel1_IRQHandler

  .weak DMA0_Channel2_IRQHandler
DMA0_Channel2_IRQHandler:
  b DMA0_Channel2_IRQHandler
                  
  .weak DMA0_Channel3_IRQHandler
DMA0_Channel3_IRQHandler:
  b DMA0_Channel3_IRQHandler

  .weak DMA0_Channel4_IRQHandler
DMA0_Channel4_IRQHandler:
  b DMA0_Channel4_IRQHandler
  
  .weak DMA0_Channel5_IRQHandler
DMA0_Channel5_IRQHandler:
  b DMA0_Channel5_IRQHandler

  .weak DMA0_Channel6_IRQHandler
DMA0_Channel6_IRQHandler:
  b DMA0_Channel6_IRQHandler
          
  .weak ADC_IRQHandler
ADC_IRQHandler:
  b ADC_IRQHandler
                   
  .weak CAN0_TX_IRQHandler
CAN0_TX_IRQHandler:
  b CAN0_TX_IRQHandler
                  
  .weak CAN0_RX0_IRQHandler
CAN0_RX0_IRQHandler:
  b CAN0_RX0_IRQHandler
                  
  .weak CAN0_RX1_IRQHandler
CAN0_RX1_IRQHandler:
  b CAN0_RX1_IRQHandler
                  
  .weak CAN0_EWMC_IRQHandler
CAN0_EWMC_IRQHandler:
  b CAN0_EWMC_IRQHandler
                  
  .weak EXTI5_9_IRQHandler
EXTI5_9_IRQHandler:
  b EXTI5_9_IRQHandler
  
  .weak TIMER0_BRK_TIMER8_IRQHandler
TIMER0_BRK_TIMER8_IRQHandler:
  b TIMER0_BRK_TIMER8_IRQHandler
                  
  .weak TIMER0_UP_TIMER9_IRQHandler
TIMER0_UP_TIMER9_IRQHandler:
  b TIMER0_UP_TIMER9_IRQHandler
  
  .weak TIMER0_TRG_CMT_TIMER10_IRQHandler
TIMER0_TRG_CMT_TIMER10_IRQHandler:
  b TIMER0_TRG_CMT_TIMER10_IRQHandler
                 
  .weak TIMER0_Channel_IRQHandler
TIMER0_Channel_IRQHandler:
  b TIMER0_Channel_IRQHandler
                  
  .weak TIMER1_IRQHandler
TIMER1_IRQHandler:
  b TIMER1_IRQHandler
                  
  .weak TIMER2_IRQHandler
TIMER2_IRQHandler:
  b TIMER2_IRQHandler
  
  .weak TIMER3_IRQHandler
TIMER3_IRQHandler:
  b TIMER3_IRQHandler
                  
  .weak I2C0_EV_IRQHandler
I2C0_EV_IRQHandler:
  b I2C0_EV_IRQHandler
          
  .weak I2C0_ER_IRQHandler
I2C0_ER_IRQHandler:
  b I2C0_ER_IRQHandler
                  
  .weak I2C1_EV_IRQHandler
I2C1_EV_IRQHandler:
  b I2C1_EV_IRQHandler
                 
  .weak I2C1_ER_IRQHandler
I2C1_ER_IRQHandler:
  b I2C1_ER_IRQHandler
                  
  .weak SPI0_IRQHandler
SPI0_IRQHandler:
  b SPI0_IRQHandler
                  
  .weak SPI1_IRQHandler
SPI1_IRQHandler:
  b SPI1_IRQHandler
  
  .weak USART0_IRQHandler
USART0_IRQHandler:
  b USART0_IRQHandler
  
  .weak USART1_IRQHandler
USART1_IRQHandler:
  b USART1_IRQHandler
  
  .weak USART2_IRQHandler
USART2_IRQHandler:
  b USART2_IRQHandler

  .weak EXTI10_15_IRQHandler
EXTI10_15_IRQHandler:
  b EXTI10_15_IRQHandler
  
  .weak RTC_Alarm_IRQHandler
RTC_Alarm_IRQHandler:
  b RTC_Alarm_IRQHandler
  
  .weak USBFS_WKUP_IRQHandler
USBFS_WKUP_IRQHandler:
  b USBFS_WKUP_IRQHandler
  
  .weak TIMER7_BRK_TIMER11_IRQHandler
TIMER7_BRK_TIMER11_IRQHandler:
  b TIMER7_BRK_TIMER11_IRQHandler

  .weak TIMER7_UP_TIMER12_IRQHandler
TIMER7_UP_TIMER12_IRQHandler:
  b TIMER7_UP_TIMER12_IRQHandler

  .weak TIMER7_TRG_CMT_TIMER13_IRQHandler
TIMER7_TRG_CMT_TIMER13_IRQHandler:
  b TIMER7_TRG_CMT_TIMER13_IRQHandler

  .weak TIMER7_Channel_IRQHandler
TIMER7_Channel_IRQHandler:
  b TIMER7_Channel_IRQHandler

  .weak DMA0_Channel7_IRQHandler
DMA0_Channel7_IRQHandler:
  b DMA0_Channel7_IRQHandler

  .weak SDIO_IRQHandler
SDIO_IRQHandler:
  b SDIO_IRQHandler

  .weak TIMER4_IRQHandler
TIMER4_IRQHandler:
  b TIMER4_IRQHandler

  .weak SPI2_IRQHandler
SPI2_IRQHandler:
  b SPI2_IRQHandler

  .weak UART3_IRQHandler
UART3_IRQHandler:
  b UART3_IRQHandler

  .weak UART4_IRQHandler
UART4_IRQHandler:
  b UART4_IRQHandler

  .weak TIMER5_DAC_IRQHandler
TIMER5_DAC_IRQHandler:
  b TIMER5_DAC_IRQHandler

  .weak TIMER6_IRQHandler
TIMER6_IRQHandler:
  b TIMER6_IRQHandler

  .weak DMA1_Channel0_IRQHandler
DMA1_Channel0_IRQHandler:
  b DMA1_Channel0_IRQHandler

  .weak DMA1_Channel1_IRQHandler
DMA1_Channel1_IRQHandler:
  b DMA1_Channel1_IRQHandler
  
  .weak DMA1_Channel2_IRQHandler
DMA1_Channel2_IRQHandler:
  b DMA1_Channel2_IRQHandler

  .weak DMA1_Channel3_IRQHandler
DMA1_Channel3_IRQHandler:
  b DMA1_Channel3_IRQHandler

  .weak DMA1_Channel4_IRQHandler
DMA1_Channel4_IRQHandler:
  b DMA1_Channel4_IRQHandler

  .weak CAN1_TX_IRQHandler
CAN1_TX_IRQHandler:
  b CAN1_TX_IRQHandler

  .weak CAN1_RX0_IRQHandler
CAN1_RX0_IRQHandler:
  b CAN1_RX0_IRQHandler

  .weak CAN1_RX1_IRQHandler
CAN1_RX1_IRQHandler:
  b CAN1_RX1_IRQHandler

  .weak CAN1_EWMC_IRQHandler
CAN1_EWMC_IRQHandler:
  b CAN1_EWMC_IRQHandler

  .weak USBFS_IRQHandler
USBFS_IRQHandler:
  b USBFS_IRQHandler
  
  .weak DMA1_Channel5_IRQHandler
DMA1_Channel5_IRQHandler:
  b DMA1_Channel5_IRQHandler
  
  .weak DMA1_Channel6_IRQHandler
DMA1_Channel6_IRQHandler:
  b DMA1_Channel6_IRQHandler
  
  .weak DMA1_Channel7_IRQHandler
DMA1_Channel7_IRQHandler:
  b DMA1_Channel7_IRQHandler
  
  .weak USART5_IRQHandler
USART5_IRQHandler:
  b USART5_IRQHandler
  
  .weak I2C2_EV_IRQHandler
I2C2_EV_IRQHandler:
  b I2C2_EV_IRQHandler
  
  .weak I2C2_ER_IRQHandler
I2C2_ER_IRQHandler:
  b I2C2_ER_IRQHandler
  
  .weak USBHS_EP1_Out_IRQHandler
USBHS_EP1_Out_IRQHandler:
  b USBHS_EP1_Out_IRQHandler
                              
  .weak USBHS_EP1_In_IRQHandler
USBHS_EP1_In_IRQHandler:
  b USBHS_EP1_In_IRQHandler

  .weak USBHS_WKUP_IRQHandler
USBHS_WKUP_IRQHandler:
  b USBHS_WKUP_IRQHandler

  .weak USBHS_IRQHandler
USBHS_IRQHandler:
  b USBHS_IRQHandler

  .weak DCI_IRQHandler
DCI_IRQHandler:
  b DCI_IRQHandler
  
  .weak TRNG_IRQHandler
TRNG_IRQHandler:
  b TRNG_IRQHandler
  
  .weak FPU_IRQHandler
FPU_IRQHandler:
  b FPU_IRQHandler
