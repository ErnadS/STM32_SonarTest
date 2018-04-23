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
#include "hardware/PwmController.h"

uint32_t multiplier;
uint8_t message_received = 0;
volatile char received_string[MAX_STRLEN+1];


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

GPIO_InitTypeDef GPIO_InitStruct_USART;
USART_InitTypeDef USART_InitStruct;
NVIC_InitTypeDef NVIC_InitStruct;



//GPIO pin initialization
void uart_GPIO_init(void){
    //Enable clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
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


void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USART3_IRQHandler(void);
void USART_read(USART_TypeDef* USARTx);
void handleReceivedMessage(void);


int main()
{
  Delay_Init();
  
  //Enabling clock
  RCC_HSEConfig(RCC_HSE_ON);
  while(!RCC_WaitForHSEStartUp());
  
  
  adc_init();
  
  uart_GPIO_init();
  pwm_init();
  
  
 
  //SystemTick Interrupt (SysTick_Handler) - 0.25 microsecond
  SysTick_Config(SystemCoreClock/(1000000*MY_PRESCALER));
  
  
  USART_Initialization(9600);
  NVIC_Initialization();
  
  
  int counter = 0, A = 5;
  double Ts = 0.0001, w = 2*PI*100;
  char temp[10];  
  
  initVoltageReg();
  voltReg_initOutputLow();
  
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

void SysTick_Handler2(void){}

void SysTick_Handler(void){
    pwm_on_tick();
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

