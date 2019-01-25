#include "../../HAL.h"
#include "../../ADCs.h"

/* analog channel selection values (ADCx_SC1n Register, DIFF & ADCH fields) */

//             ADCH  // DIFF = 0                   | DIFF = 1
#define DAD0   0x00  // DADP0                      | DAD0
#define DAD1   0x01  // DADP1                      | DAD1
#define DAD2   0x02  // DADP2                      | DAD2
#define DAD3   0x03  // DADP3                      | DAD3
#define AD04   0x04  // AD4                        | Reserved
#define AD05   0x05  // AD5                        | Reserved
#define AD06   0x06  // AD6                        | Reserved
#define AD07   0x07  // AD7                        | Reserved
#define AD08   0x08  // AD8                        | Reserved
#define AD09   0x09  // AD9                        | Reserved
#define AD10   0x0A  // AD10                       | Reserved
#define AD11   0x0B  // AD11                       | Reserved
#define AD12   0x0C  // AD12                       | Reserved
#define AD13   0x0D  // AD13                       | Reserved
#define AD14   0x0E  // AD14                       | Reserved
#define AD15   0x0F  // AD15                       | Reserved
#define AD16   0x10  // AD16                       | Reserved
#define AD17   0x11  // AD17                       | Reserved
#define AD18   0x12  // AD18                       | Reserved
#define AD19   0x13  // AD19                       | Reserved
#define AD20   0x14  // AD20                       | Reserved
#define AD21   0x15  // AD21                       | Reserved
#define AD22   0x16  // AD22                       | Reserved
#define AD23   0x17  // AD23                       | Reserved
#define AD24   0x18  // Reserved                   | Reserved
#define AD25   0x19  // Reserved                   | Reserved
#define AD26   0x1A  // Temp Sensor (single ended) | Temp Sensor (differential)
#define AD27   0x1B  // Bandgap     (single ended) | Bandgap     (differential)
#define AD28   0x1C  // Reserved                   | Reserved
#define AD29   0x1D  // VRefSH                     | -VRefSH     (differential)
#define AD30   0x1E  // VRefSL                     | -VRefSL     (differential)
#define AD31   0x1F  // Module disabled            | Module diabled

static void init(void);
static void deInit(void);

/* ADCs are scanned using two DMA channels. Upon ADC read complete, the first DMA channel (Channel 1 for ADC 0, Channel 3 for ADC 1)
 * will write the result of the ADC measurement to the result array. Upon DMA completion the first channel triggers the
 * second DMA channel (Channel 0 for ADC 0, Channel 2 for ADC 1) to write a new MUX value into the ADC's configuration register.
 * This repeats, creating an infinite loop reading out 3 Samples per ADC.
 *
 * This loop gets initialised by an initial DMA request for the second Channel to write the first MUX value.
 */

/* Result buffer indices:
 *          Input Signal, ADC Channel, Pin
 * ADC0[0]: ADC0_DP1,     DAD1,        14
 * ADC0[1]: ADC0_SE12,    AD12,        55
 * ADC0[2]: ADC0_SE13,    AD13,        56
 * ADC1[0]: ADC1_DP0,     DAD0,        20
 * ADC1[1]: ADC1_DP1,     DAD1,        16
 * ADC1[2]: ADC1_DP3,     DAD3,        18
 */

// ADC Result buffers
volatile uint16 adc0_result[3] = { 0 };
volatile uint16 adc1_result[3] = { 0 };
// ADC Multiplexer selection
const uint8  adc0_mux[3] = { DAD1, AD12, AD13 };
const uint8  adc1_mux[3] = { DAD0, DAD1, DAD3 };

ADCTypeDef ADCs =
{
	.AIN0    = &adc1_result[1],
	.AIN1    = &adc1_result[2],
	.AIN2    = &adc1_result[0],
	.DIO4    = &adc0_result[1],
	.DIO5    = &adc0_result[2],
	.VM      = &adc0_result[0],
	.init    = init,
	.deInit  = deInit
};

static void init(void)
{
	// === ADC initialization ===
	// enable clock for ADC0 & ADC1
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;

	HAL.IOs->pins->DIO4.configuration.GPIO_Mode = GPIO_Mode_IN;
	HAL.IOs->pins->DIO5.configuration.GPIO_Mode = GPIO_Mode_IN;

	HAL.IOs->config->set(&HAL.IOs->pins->DIO4);
	HAL.IOs->config->set(&HAL.IOs->pins->DIO5);

	// CFG1: set ADC clocks
	ADC0_CFG1 = ADC_CFG1_MODE(0x03) | ADC_CFG1_ADICLK(1);
	ADC1_CFG1 = ADC_CFG1_MODE(0x03) | ADC_CFG1_ADICLK(1);

	// CFG2: configuration for high speed conversions and selects the long sample time duration
	ADC0_CFG2 = ADC_CFG2_ADLSTS(0);  // use default longest sample time
	ADC1_CFG2 = ADC_CFG2_ADLSTS(0);  // use default longest sample time

	// SC2: conversion
	ADC0_SC2 = ADC_SC2_DMAEN_MASK;  // enable DMA
	ADC1_SC2 = ADC_SC2_DMAEN_MASK;  // enable DMA;

	// average over 4 samples, single measurement (trigger from having DMA write the MUX value to SC1A)
	ADC0_SC3 = 	ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);
	ADC1_SC3 = 	ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);

	ADC0_SC3 |= ADC_SC3_CAL_MASK;
	while(ADC0_SC3 & ADC_SC3_CAL_MASK);

	uint16 cal = 0;
	cal += ADC0_CLP0;
	cal += ADC0_CLP1;
	cal += ADC0_CLP2;
	cal += ADC0_CLP3;
	cal += ADC0_CLP4;
	cal += ADC0_CLPS;
	cal >>= 1;
	cal |= 1<<15;

	ADC0_PG = cal;

	ADC1_SC3 |= ADC_SC3_CAL_MASK;
	while(ADC1_SC3 & ADC_SC3_CAL_MASK);

	cal = 0;
	cal += ADC1_CLP0;
	cal += ADC1_CLP1;
	cal += ADC1_CLP2;
	cal += ADC1_CLP3;
	cal += ADC1_CLP4;
	cal += ADC1_CLPS;
	cal >>= 1;
	cal |= 1<<15;

	ADC1_PG = cal;

	// === DMA initialization ===

	// enable clock for DMA
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// enable clock for DMA mux
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;

	// === setup DMA channels for ADC0 ===

	// DMA channel 0, use for write ADC mux channel, from SRAM to ADC
	DMAMUX_CHCFG0           = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(0x36);                 // DMA source: DMA Mux channel 1 (Source Number 54)
	DMA_TCD0_SADDR          = (uint32) &adc0_mux[0];                                              // Source address: ADC Mux Settings Array
	DMA_TCD0_SOFF           = 0x01;                                                               // Source address increment (Added after each major loop step)
	DMA_TCD0_SLAST          = (uint32) -3;                                                        // Source address decrement (Added on major loop completion)
	DMA_TCD0_DADDR          = (uint32) &ADC0_SC1A;                                                // Destination address: ADC0 control register (containing ADC Mux bitfield)
	DMA_TCD0_DOFF           = 0x00;                                                               // Destination address increment (Added after each major loop step)
	DMA_TCD0_DLASTSGA       = 0x00;                                                               // Destination address decrement (Added on major loop completion)
	DMA_TCD0_NBYTES_MLNO    = 0x01;                                                               // Number of bytes transferred per request (1 Byte ADC Setting)
	DMA_TCD0_BITER_ELINKNO  = 0x03;                                                               // Disable channel link, beginning major iteration count: 3
	DMA_TCD0_CITER_ELINKNO  = 0x03;                                                               // Disable channel link, current major iteration count: 3
	DMA_TCD0_ATTR           = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);                              // Source and destination size: 8 bit
	DMA_TCD0_CSR            = DMA_CSR_START_MASK;                                                 // Request channel start, to initiate our readout loop

	// DMA channel 1, use for read ADC result data, from ADC to SRAM
	DMAMUX_CHCFG1           = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(0x28);                 // DMA source: ADC0 (Source Number 40)
	DMA_TCD1_SADDR          = (uint32) &ADC0_RA;                                                  // Source address: ADC0 result register
	DMA_TCD1_SOFF           = 0x00;                                                               // Source address increment (Added after each major loop step)
	DMA_TCD1_SLAST          = 0x00;                                                               // Source address decrement (Added on major loop completion)
	DMA_TCD1_DADDR          = (uint32) &adc0_result[0];                                           // Destination address: ADC0 result buffer
	DMA_TCD1_DOFF           = 0x02;                                                               // Destination address increment (Added after each major loop step)
	DMA_TCD1_DLASTSGA       = (uint32) -6;                                                        // Destination address decrement (Added on major loop completion)
	DMA_TCD1_NBYTES_MLNO    = 0x02;                                                               // Number of bytes transferred per request (2 Byte ADC Result)
	DMA_TCD1_BITER_ELINKYES = (DMA_BITER_ELINKYES_ELINK_MASK|DMA_BITER_ELINKYES_LINKCH(0)|0x03);  // Enable channel link (to channel 0) on major loop step, beginning major iteration count: 3
	DMA_TCD1_CITER_ELINKYES = (DMA_CITER_ELINKYES_ELINK_MASK|DMA_BITER_ELINKYES_LINKCH(0)|0x03);  // Enable channel link (to channel 0) on major loop step, current major iteration count: 3
	DMA_TCD1_ATTR           = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);                              // Source and destination size: 16 bit
	DMA_TCD1_CSR            = (DMA_CSR_MAJORLINKCH(0) | DMA_CSR_MAJORELINK_MASK);                 // Major loop completion starts request for Channel 0

	// Start the DMA Channel 1
	DMA_SERQ = 1;

	// === setup DMA channels for ADC1 ===

	// DMA channel 2, use for write ADC mux channel, from SRAM to ADC
	DMAMUX_CHCFG2           = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(0x38);                   // DMA source: DMA Mux channel 2 (Source Number 56)
	DMA_TCD2_SADDR          = (uint32) &adc1_mux[0];                                                // Source address: ADC Mux Settings
	DMA_TCD2_SOFF           = 0x01;                                                                 // Source address increment (Added after each major loop step)
	DMA_TCD2_SLAST          = (uint32) -3;                                                          // Source address decrement (Added on major loop completion)
	DMA_TCD2_DADDR          = (uint32) &ADC1_SC1A;                                                  // Destination address: ADC1 control register (containing ADC Mux bitfield)
	DMA_TCD2_DOFF           = 0x00;                                                                 // Destination address increment (Added for each major loop step)
	DMA_TCD2_DLASTSGA       = 0x00;                                                                 // Destination address decrement (Added on major loop completion)
	DMA_TCD2_NBYTES_MLNO    = 0x01;                                                                 // Number of bytes transferred per request (1 Byte ADC Setting)
	DMA_TCD2_BITER_ELINKNO  = 0x03;                                                                 // Disable channel link, beginning major iteration count: 3
	DMA_TCD2_CITER_ELINKNO  = 0x03;                                                                 // Disable channel link, current major iteration count: 3
	DMA_TCD2_ATTR           = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);                                // Source and destination size: 8 bit
	DMA_TCD2_CSR            = DMA_CSR_START_MASK;                                                   // Request channel start, to initiate our readout loop

	// DMA channel 3, use for read ADC result data, from ADC to SRAM
	DMAMUX_CHCFG3           = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(0x29);                   // DMA source: ADC1 (Source Number 41)
	DMA_TCD3_SADDR          = (uint32) &ADC1_RA;                                                    // Source address: ADC1 result register
	DMA_TCD3_SOFF           = 0x00;                                                                 // Source address increment (Added after each major loop step)
	DMA_TCD3_SLAST          = 0x00;                                                                 // Source address decrement (Added on major loop completion)
	DMA_TCD3_DADDR          = (uint32) &adc1_result[0];                                             // Destination address: ADC0 result buffer
	DMA_TCD3_DOFF           = 0x02;                                                                 // Destination address increment (Added after each major loop step)
	DMA_TCD3_DLASTSGA       = (uint32) -6;                                                          // Destination address decrement (Added on major loop completion)
	DMA_TCD3_NBYTES_MLNO    = 0x02;                                                                 // Number of bytes transferred per request (2 Byte ADC Result)
	DMA_TCD3_BITER_ELINKYES = DMA_BITER_ELINKYES_ELINK_MASK | DMA_BITER_ELINKYES_LINKCH(2) | 0x03;  // Enable channel link (to channel 2) on major loop step, beginning major iteration count: 3
	DMA_TCD3_CITER_ELINKYES = DMA_CITER_ELINKYES_ELINK_MASK | DMA_CITER_ELINKYES_LINKCH(2) | 0x03;  // Enable channel link (to channel 2) on major loop step, current major iteration count: 3
	DMA_TCD3_ATTR           = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);                                // Source and destination size: 16 bit
	DMA_TCD3_CSR            = DMA_CSR_MAJORLINKCH(2) | DMA_CSR_MAJORELINK_MASK;                     // Major loop completion starts request for Channel 2

	// Start the DMA Channel 3
	DMA_SERQ = 3;

	EnableInterrupts;
}

static void deInit()
{
	// disable clock for DMA
	SIM_SCGC7 &= ~(SIM_SCGC7_DMA_MASK);

	// disable clock for DMA mux
	SIM_SCGC6 &= ~(SIM_SCGC6_DMAMUX_MASK);

	// disable clock for ADC0/ADC1
	SIM_SCGC6 &= ~(SIM_SCGC6_ADC0_MASK);
	SIM_SCGC3 &= ~(SIM_SCGC3_ADC1_MASK);
}
