#include "Adc.h"

#include "stm32f4xx.h"
#include <stdio.h>
#include "stm32f4_discovery.h"
//#include "stm32f429i_discovery.h"
#include "stm32f4xx_dma.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"

#define REF_VOLT  		3.3f
#define MAX_MEASUR   	4095   // ADC is 12 bits. Max number is 4095

#define ADC_BUFFER       ((uint32_t)0xD0000000)

////////////////////////////////////////////////////////////////////////////////
// ADC
char HalfBuff=0; // Koristeni kad sam citao ADC sa DMA (da mi javi kada je pola buffer-a napunjeno)
char FullBuff=0;

uint32_t samplingFreq = 800000; // 4 samples za svaki Herz (200kHz transducer)

char bOn = 0;
// __IO  uint16_t ADCConvertedValue[BUFFERSIZE];
static uint16_t* ADCConvertedValue = (uint16_t*)ADC_BUFFER;
////////////////////////////////////////////////////////////////////////////////

float convertToVoltage(uint16_t measured) {
	return (measured * REF_VOLT) / MAX_MEASUR;
}

void adc_init(void) {
    RCC_Configuration();
    GPIO_Configuration(); 
    NVIC_Configuration2();   
    TIM2_Configuration(48000); // sample sa 48kHz

    DMA_Configuration();
    
    ADC_Configuration();
}

void RCC_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC /*RCC_AHB1Periph_GPIOC*/, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* ADC Channel 11 -> PC1  // PC3
  */
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC /*GPIOC*/, &GPIO_InitStructure);
}

void ADC_Configuration(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;  // APB2=84MHz, prescaler=2, conversion rate =42MHz, adc cycle =15 clock cycles
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;        // brzinom od 20kHz se starta ADC Svakih 50us (interrupt timera2)
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC1 regular channel 11 configuration */
  ADC_RegularChannelConfig(ADC1, /*ADC_Channel_11*/ ADC_Channel_13, 1, ADC_SampleTime_28Cycles); // PC1
  
  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
}

void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValue[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;    // Buffer se puni brzinom od 20kHz, napuni se za 400x50us=20ms
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE; // Count of 16-bit words
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  
  /* Enable DMA Stream Half / Transfer Complete interrupt */
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);
  
  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);
}


uint32_t getSamplingFreq(void) {
	return samplingFreq;
}

void TIM2_Configuration(uint32_t samplingFrequency)
{
	samplingFreq = samplingFrequency;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 1000000/samplingFreq - 1; // Timer clock je 84MHz?  Prescaler = 2 => 21MHz/2/2100 = 5000Hz  (Timer clock je 84MHz, prescaler je 2 
                                               // period je 2100, interrupt se dešava brzinom 20kHz
  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;//84 - 1; // Down to 1 MHz (adjust per your clock)//0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* TIM2 TRGO selection */
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

//void TIM3_Configuration(void)
//{
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//  
//  /* Time base configuration */
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//  TIM_TimeBaseStructure.TIM_Period = 510-1;  // Period is 2kHz  
//                                               
//  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;//84 - 1; // Down to 1 MHz (adjust per your clock)//0;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//  
//  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 
//  
//  /* TIM3 enable counter */  // Enable when detected jump from 0 to 1
//  TIM_Cmd(TIM3, ENABLE);
//}

void NVIC_Configuration2(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the DMA Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
//  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
int zzz= 0;
void DMA2_Stream0_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
  /* Test on DMA Stream Half Transfer interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
  {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
    HalfBuff=1;
  }
  
  /* Test on DMA Stream Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
      
      zzz ++;
      if (zzz == 1)
          zzz = 1;
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    FullBuff=1;
	if (bOn) {
		bOn = 0;
		STM_EVAL_LEDOff(LED3);  //20ms off
	}
	else {
		bOn = 1;
		STM_EVAL_LEDOn(LED3);   // 20ms on          (vrijeme punjenja buffera)
	}
  }
}

void TIM2_IRQHandler (void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
