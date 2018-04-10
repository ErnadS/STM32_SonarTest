//#include "defines.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"

#include "VoltageRegulator.h"

TIM_BDTRInitTypeDef TIM_BDTRInitStructure2;

void TM_LEDS_Init(void);
void TM_TIMER_Init(void);
void TM_PWM_Init(void);
void setHigh();

void initVoltageReg(void) {
    TM_LEDS_Init();
    /* Init timer */
    TM_TIMER_Init();
    /* Init PWM */
    TM_PWM_Init();
}

void voltReg_setOutputLow() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
}

void voltReg_stop() {
    TIM_Cmd(TIM4, DISABLE);
    //setHigh();
    // TIM4->BDTR &= (uint16_t)~TIM_BDTR_MOE;
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    
    //TM_LEDS_Init();
}

void voltReg_start()
{
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);
    /*TIM4->CNT = 0;
    TIM4->BDTR |= TIM_BDTR_MOE;**/
}


 
void TM_LEDS_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* Clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
    /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    /* GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);*/
    
    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; //| GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}
 
void TM_TIMER_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    
    /* Enable clock for TIM4 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
/*    
    TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock                 
    But, timer has internal PLL, which double this frequency for timer, up to 84MHz     
    Remember: Not each timer is connected to APB1, there are also timers connected     
    on APB2, which works at 84MHz by default, and internal PLL increase                 
    this to up to 168MHz                                                             
    
    Set timer prescaller 
    Timer count frequency is set with 
    
    timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)        
    
    In our case, we want a max frequency for timer, so we set prescaller to 0         
    And our timer will have tick frequency        
    
    timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
*/    
    TIM_BaseStruct.TIM_Prescaler = 0;
    /* Count up */
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
/*
    Set timer period when it have reset
    First you have to know max value for timer
    In our case it is 16bit = 65535
    To get your frequency for PWM, equation is simple
    
    PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
    
    If you know your PWM frequency you want to have timer period set correct
    
    TIM_Period = timer_tick_frequency / PWM_frequency - 1
    
    In our case, for 10Khz PWM_frequency, set Period to
    
    TIM_Period = 84000000 / 10000 - 1 = 8399
    
    If you get TIM_Period larger than max timer value (in our case 65535),
    you have to choose larger prescaler and slow down timer tick frequency
*/
    TIM_BaseStruct.TIM_Period = 150;//839;//9;//8399; /* 10kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    /* Initialize TIM4 */
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    /* Start count on TIM4 */
    // TIM_Cmd(TIM4, ENABLE);
}
 
void TM_PWM_Init(void) {
    TIM_OCInitTypeDef TIM_OCStruct;
    
    /* Common settings */
    
    /* PWM mode 2 = Clear on compare match */
    /* PWM mode 1 = Set on compare match */
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCStruct.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM4->CNT = 0;
    
/*
    To get proper duty cycle, you have simple equation
    
    pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
    
    where DutyCycle is in percent, between 0 and 100%
    
    25% duty cycle:     pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
    50% duty cycle:     pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
    75% duty cycle:     pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
    
    Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
*/
    TIM_OCStruct.TIM_Pulse =   40;//419;// 2099; /* 25% duty cycle */
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 40;//209;//4199; /* 50% duty cycle */
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    
    voltReg_start();
    voltReg_stop();
}