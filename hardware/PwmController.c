#include<stdio.h>
#include<math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"

#include "PwmController.h"
#include "VoltageRegulator.h"

#define PWM_PRESCALER 2


void pwm_GPIO_Initialization(void);
void pwm_gpio_Timer_Configuration(void);

void pwm_setDuty(TIM_TypeDef* TIMx, uint8_t Duty, uint8_t OCx);
// void PWM_Handler(void);
void pwm_enable(void);
void pwm_disable(void);

uint32_t pwmTimerTick = 0;
uint32_t pwmMainTimerTick = 0;
uint8_t pwmFlag = 1;

uint16_t uPwm_period;
int nPingLength_us = 200;

void pwm_init(void) {
    // uPwm_period = 16799;   //10KHz
    uPwm_period = 840;   //200KHz
    // uPwm_period = TimerClock/PWM_Freq - 1;
    
    pwm_GPIO_Initialization();
    pwm_gpio_Timer_Configuration();
    
    uint8_t PWM_Duty = 50;
    pwm_setDuty(TIM1, PWM_Duty, 1);
    TIM1->BDTR &= (uint16_t)~TIM_BDTR_MOE; 
}

//GPIO pin initialization
void pwm_GPIO_Initialization(void){
    GPIO_InitTypeDef GPIO_struct_pwm;
    
    //Enable clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
           
    //Enable pins for PWM
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    
    //PWM Pin Initialization
    GPIO_struct_pwm.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_struct_pwm.GPIO_Mode = GPIO_Mode_AF;
    GPIO_struct_pwm.GPIO_OType = GPIO_OType_PP;
    GPIO_struct_pwm.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_struct_pwm.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_struct_pwm);  
}



//Timer initialization for PWM 
void pwm_gpio_Timer_Configuration(void){
  TIM_TimeBaseInitTypeDef TIM_BaseStruct_pwm;
  TIM_OCInitTypeDef TIM_OCStruct_pwm;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure_pwm;

  TIM_BaseStruct_pwm.TIM_Period = uPwm_period;
  TIM_BaseStruct_pwm.TIM_Prescaler = 0;
  TIM_BaseStruct_pwm.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStruct_pwm.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct_pwm.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_BaseStruct_pwm);
  TIM_Cmd(TIM1, ENABLE);
  
  TIM_OCStruct_pwm.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCStruct_pwm.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct_pwm.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCStruct_pwm.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCStruct_pwm.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCStruct_pwm.TIM_Pulse = 0;
  TIM_OCStruct_pwm.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCStruct_pwm.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  
  TIM_OC1Init(TIM1, &TIM_OCStruct_pwm);
  
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
  TIM_BDTRInitStructure_pwm.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure_pwm.TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInitStructure_pwm.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  
  TIM_BDTRInitStructure_pwm.TIM_DeadTime = 0x0F;
  TIM_BDTRInitStructure_pwm.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure_pwm.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStructure_pwm.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure_pwm);
  
  TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void pwm_on_tick(void) {
    if(pwmFlag == 2){
        if(pwmTimerTick == (nPingLength_us -23) * PWM_PRESCALER){ // 65*MY_PRESCALER){
            voltReg_OFF();
        }
        if(pwmTimerTick == nPingLength_us * PWM_PRESCALER){ // 87*MY_PRESCALER){
            pwmFlag = 0;
            TIM1->BDTR &= (uint16_t)~TIM_BDTR_MOE;  // stop PWM
            voltReg_stop();
            GPIO_ResetBits(GPIOE, GPIO_Pin_8);
            GPIO_ResetBits(GPIOE, GPIO_Pin_9);
            //TIM_CtrlPWMOutputs(TIM1, DISABLE);
            pwmTimerTick = 1;
        } 
        else
            pwmTimerTick++;
    } else if(pwmFlag == 1){
        TIM1->CNT = 0;
        TIM1->BDTR |= TIM_BDTR_MOE;              // Start PWM
        //TIM_CtrlPWMOutputs(TIM1, ENABLE);
        pwmFlag = 2;
        pwmTimerTick = 1;
    }
    
    if(pwmMainTimerTick == (1000000 - 1)*PWM_PRESCALER){
        voltReg_ON();
    } else  if(pwmMainTimerTick == 1000000*PWM_PRESCALER){   // Svake sekunde upali
       // voltReg_start();
        // voltReg_ON();
        pwmFlag = 1;
        pwmMainTimerTick = 0;
    } //else
    pwmMainTimerTick++;
}

//Helper function for setting PWM DUTY CYCLE
void pwm_setDuty(TIM_TypeDef* TIMx, uint8_t Duty, uint8_t OCx){
 switch(OCx)
 {
 case 1: TIMx->CCR1 = (Duty * uPwm_period) / 100; break;
 case 2: TIMx->CCR2 = (Duty * uPwm_period) / 100; break;
 case 3: TIMx->CCR3 = (Duty * uPwm_period) / 100; break;
 case 4: TIMx->CCR4 = (Duty * uPwm_period) / 100; break;
 }
}

//Helper function for disabling PWM after impulse time
void pwm_disable(void){
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
//Helper function for enabling PWM after impulse pause
void pwm_enable(void){
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/*
void PWM_Handler(void){
  uTimerTick_PWM = 0;
  uint8_t PWM_Duty = 50;
  PWM_SetDutyCycle(TIM1, PWM_Duty, 1);
} */