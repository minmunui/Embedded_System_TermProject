
#include "init.h"

void myInit(void){
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();
    USART1_Configure();
    USART2_Configure();
    TIM_Configure();
}

void RCC_Configure(void) // stm32f10x_rcc.h ????
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
// -----------------Port-------------------
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

// -----------------Usart-------------------
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

// -----------------ADC-------------------
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

// -----------------Timer-------------------  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h ????
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_2);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_10 | GPIO_Pin_3);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // From LCD
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // to activate motor
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_11 | GPIO_Pin_12 );
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitTypeDef GPIO_InitStructure1;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure1.GPIO_Pin = (GPIO_Pin_13);
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure1);
}

void ADC_Configure() {
  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_Cmd(ADC1 ,ENABLE);
  ADC_ResetCalibration(ADC1);

  while(ADC_GetResetCalibrationStatus(ADC1)) ;
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1)) ;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE) ;
}


void NVIC_Configure(void) { // misc.h
 
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);


  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_EnableIRQ(ADC1_2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
}


void TIM_Configure(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
 
  int prescale = (uint16_t)(SystemCoreClock / 10000);
 
  TIM_TimeBaseStructure.TIM_Period = 10000;
  TIM_TimeBaseStructure.TIM_Prescaler = prescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
 
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
 
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}


void USART1_Configure() {
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1 ,&USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE , ENABLE);
}

void USART2_Configure() {
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2 ,&USART_InitStructure);
  USART_Cmd(USART2, ENABLE);
  USART_ITConfig(USART2, USART_IT_RXNE , ENABLE);
}


