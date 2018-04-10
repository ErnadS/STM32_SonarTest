#include<stdio.h>
#include<math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"

#define MAX_STRLEN 20
//#if !defined  (HSE_VALUE) 
 // #define HSE_VALUE    ((uint32_t)8000000) 
// #endif
#define PI 3.1415926535897932384626433832795028841971693993751058209
#define MY_PRESCALER 2

#include "VoltageRegulator.h"


uint32_t multiplier;
uint8_t message_received = 0;
volatile char received_string[MAX_STRLEN+1];
uint32_t mainTimerTick = 0;
uint8_t TimerTick_PWM = 0;

uint16_t pwm_period;
uint8_t Flag = 1;



void Delay_Init(void){
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    multiplier = RCC_Clocks.HCLK_Frequency / 4000000;
}
void Delay(uint32_t ms){
    uint32_t milis = 1000*ms*multiplier - 10;
    while(milis--);
}
void Delay_us(uint32_t us){
  uint32_t milis = us*multiplier-10;
  while(milis--);
}

GPIO_InitTypeDef GPIO_InitStruct_USART, GPIO_InitStruct_PWM;
USART_InitTypeDef USART_InitStruct;
NVIC_InitTypeDef NVIC_InitStruct;

TIM_TimeBaseInitTypeDef TIM_BaseStruct;
TIM_OCInitTypeDef TIM_OCStruct;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

//GPIO pin initialization
void GPIO_Initialization(void){
    //Enable clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
           
    //Enable pins for PWM
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    
    //PWM Pin Initialization
    GPIO_InitStruct_PWM.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct_PWM.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct_PWM.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct_PWM.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct_PWM.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct_PWM);
    
    //Enable pins for Bluetooth
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    
    //USART3 Pin Initialization
    GPIO_InitStruct_USART.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct_USART.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct_USART.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct_USART.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct_USART.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct_USART);
}

//USART pin initialization
void USART_Initialization(uint32_t baudrate){
    //Enable clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    //Set options for USART
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    
    USART_Init(USART3, &USART_InitStruct);
}
//Interrupt initialization for receiving messages
void NVIC_Initialization(void){
    //Enable interrupt needed for recieving data from USART3
    
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 
    USART_Cmd(USART3, ENABLE);
}
//Timer initialization for PWM 
void Timer_Configuration(void){
  TIM_BaseStruct.TIM_Period = pwm_period;
  TIM_BaseStruct.TIM_Prescaler = 0;
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
  TIM_Cmd(TIM1, ENABLE);
  
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCStruct.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCStruct.TIM_Pulse = 0;
  TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  
  TIM_OC1Init(TIM1, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  
  TIM_BDTRInitStructure.TIM_DeadTime = 0x0F;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
  
  TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USART3_IRQHandler(void);
void USART_read(USART_TypeDef* USARTx);
void handleReceivedMessage(void);
void PWM_SetDutyCycle(TIM_TypeDef* TIMx, uint8_t Duty, uint8_t OCx);
void PWM_Handler(void);
void DisablePWM(void);
void EnablePWM(void);

void SysTick_Handler2(void){}

void SysTick_Handler(void){
  if(Flag == 2){
    if(TimerTick_PWM == 87*MY_PRESCALER){ // 43*MY_PRESCALER){
      Flag = 0;
      TIM1->BDTR &= (uint16_t)~TIM_BDTR_MOE;
      voltReg_stop();
      GPIO_ResetBits(GPIOE, GPIO_Pin_8);
      GPIO_ResetBits(GPIOE, GPIO_Pin_9);
      //TIM_CtrlPWMOutputs(TIM1, DISABLE);
      TimerTick_PWM = 1;
    } 
    else
      TimerTick_PWM++;
  }else if(Flag == 1){
    //PWM_Handler();
    TIM1->CNT = 0;
    TIM1->BDTR |= TIM_BDTR_MOE;
    //TIM_CtrlPWMOutputs(TIM1, ENABLE);
    Flag = 2;
    TimerTick_PWM = 1;
  }
   
  
  if(mainTimerTick == 1000000*MY_PRESCALER){
    voltReg_start();
    Flag = 1;
    mainTimerTick = 0;
  } else
    mainTimerTick++;
}

int main()
{
  Delay_Init();
  
  //Enabling clock
  RCC_HSEConfig(RCC_HSE_ON);
  while(!RCC_WaitForHSEStartUp());
  
  //SystemTick Interrupt - 0.25 microsecond
  SysTick_Config(SystemCoreClock/(1000000*MY_PRESCALER));
  
  voltReg_setOutputLow();
  
  //Creating PWM signal
  
  //10KHz
  //pwm_period = 16799;
  
  //200KHz
  pwm_period = 840;
  
  //pwm_period = TimerClock/PWM_Freq - 1; 
  
  GPIO_Initialization();
  Timer_Configuration();
  USART_Initialization(9600);
  NVIC_Initialization();
  
  uint8_t PWM_Duty = 50;
  PWM_SetDutyCycle(TIM1, PWM_Duty, 1);
  TIM1->BDTR &= (uint16_t)~TIM_BDTR_MOE;
  
  int counter = 0, A = 5;
  double Ts = 0.0001, w = 2*PI*100;
  char temp[10];  
  
  initVoltageReg();
  
  //Sending message to bluetooth module
  while(1){}
  
  /*while(1){ 
      Delay(5);
      double temp_value = A*sin(Ts * counter * w);
      sprintf(temp, "%.5f\n", temp_value);
      USART_puts(USART3, temp);
      counter++;
  }*/
  return 0;
}

//Helper function for setting PWM DUTY CYCLE
void PWM_SetDutyCycle(TIM_TypeDef* TIMx, uint8_t Duty, uint8_t OCx){
 switch(OCx)
 {
 case 1: TIMx->CCR1 = (Duty * pwm_period) / 100; break;
 case 2: TIMx->CCR2 = (Duty * pwm_period) / 100; break;
 case 3: TIMx->CCR3 = (Duty * pwm_period) / 100; break;
 case 4: TIMx->CCR4 = (Duty * pwm_period) / 100; break;
 }
}

//Helper function for disabling PWM after impulse time
void DisablePWM(void){
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
//Helper function for enabling PWM after impulse pause
void EnablePWM(void){
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_Handler(void){
  TimerTick_PWM = 0;
  uint8_t PWM_Duty = 50;
  PWM_SetDutyCycle(TIM1, PWM_Duty, 1);
}

//Writing string to USART
void USART_puts(USART_TypeDef* USARTx, volatile char *s){
    while(*s != '\0' && *s){
        // wait until data register is empty
        while( !(USARTx->SR & 0x00000040) );
        USART_SendData(USARTx, *s);
        *s++;
    }
}
//Handling recieved message
void handleReceivedMessage(void){
    USART_puts(USART3, "Primio rijec:\n");
    USART_puts(USART3, received_string);
    USART_puts(USART3, "\n");
    message_received = 0;
}
//Interrupt for USART3
void USART3_IRQHandler(void){
    // check if the USART3 receive interrupt flag was set
    static uint8_t cnt = 0; 
    if(USART_GetITStatus(USART3, USART_IT_RXNE)){
        char t = USART_ReceiveData(USART3);
        if(cnt < MAX_STRLEN)
            received_string[cnt] = t;
        
        if(t == '\n'){
            message_received = 1;
            received_string[cnt] = 0;
            cnt = -1;
        }
        if(cnt == MAX_STRLEN)
            cnt--;
        cnt++;
        USART_ClearFlag(USART3, USART_IT_RXNE);
    }
}

