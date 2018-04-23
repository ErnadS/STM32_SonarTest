#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

#ifndef BUFFERSIZE
#define BUFFERSIZE 		640000  // 300m (max depth) *2 (smijera) /1500 (brzina zvuka u sec) * 200000 (Hz) *4 (4 samples za svaki herz)
#endif

float convertToVoltage(uint16_t measured);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
void TIM2_Configuration(uint32_t samplFreq);
void NVIC_Configuration2(void);
void DMA2_Stream0_IRQHandler(void);

uint32_t getSamplingFreq(void);

#endif