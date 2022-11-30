#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};
uint16_t value;

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void NVIC_Configure(void);

//---------------------------------------------------------------------------------------------------

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
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);


  //From LCD
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_EnableIRQ(ADC1_2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
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


void Delay(void) {
int i;
for (i = 0; i < 2000000; i++) {}
}

void ADC1_2_IRQHandler() {
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET ) {
    value = ADC_GetConversionValue(ADC1);
   
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }
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

char usart1In = 'a';

void USART1_IRQHandler(void){
  if ( USART_GetITStatus(USART1, USART_IT_RXNE) != RESET ) {
    usart1In = USART_ReceiveData(USART1);
    USART_SendData(USART2, usart1In);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

char usart2In = 'a';

void USART2_IRQHandler(void){
  if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET ) {
    usart2In = USART_ReceiveData(USART2);
    USART_SendData(USART1, usart2In);
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

int cnt = 0;
uint16_t sub[2] = {1000, 2000};

void delay() {
  int i;
  for(i = 0; i < 5000000; ++i) {}
}


void change_pulse(uint16_t pulse){
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = pulse; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

int time = 0;
int TICK = 1;
void timeCheck(){
  if(time == 60 ){
    GPIO_SetBits(GPIOD, GPIO_Pin_11);
  }
  if(time == 120){
    GPIO_ResetBits(GPIOD, GPIO_Pin_11);
    time = 0;
  }
  LCD_ShowNum(70, 0, time, 10, color[11], color[0]);
}


uint16_t a = 0;
uint16_t count = 0;

void TIM2_IRQHandler() {
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) {
      count ++;
   
      time+=TICK;
      timeCheck();
      
      change_pulse(sub[cnt++]);
      if (cnt == 2)
        cnt = 0;
     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}



char MSG_TIME[] = "TIME : ";
char MSG_LUMIN[] = "LUMI : ";
char MSG_MODE[] = "MODE : ";
char MSG_AUTO[] = "AUTO  ";
char MSG_MAN[] = "MANUAL";
char MSG_LED[] = "LED : ";
char MSG_BLIND[] = "BLD : ";
char MSG_ON[] = "ON ";
char MSG_OFF[] = "OFF";

uint16_t blind_state = 1;
uint16_t led_state = 1;
uint16_t auto_mode = 1;


uint16_t V_I = 32; // Vertical Interval
uint16_t H_I = 70; // Horizontal Interval

void LCD_indicate(uint16_t input_time, uint16_t input_lumin, uint16_t is_blind, uint16_t is_led, uint16_t auto_mode) {

  // TIME
//  LCD_ShowString(0, 0, MSG_TIME, color[11], color[0]);
//  LCD_ShowNum(H_I, 0, input_time, 10, color[11], color[0]);

  // LUMIN
  LCD_ShowString(0, V_I, MSG_LUMIN, color[11], color[0]);
  LCD_ShowNum(H_I, V_I, input_lumin, 10, color[11], color[0]);

  // MODE
  LCD_ShowString(0, V_I * 2, MSG_MODE, color[11], color[0]);
  if (auto_mode) {
    LCD_ShowString(H_I, V_I * 2, MSG_AUTO, color[11], color[0]);
  }
  else {
    LCD_ShowString(H_I, V_I * 2, MSG_MAN, color[11], color[0]);
  }

  // Blind
  LCD_ShowString(0, V_I * 4, MSG_BLIND, color[11], color[0]);
  if (blind_state == 1) {
    LCD_ShowString(H_I, V_I * 4, MSG_ON, color[11], color[0]);
  }
  else {
    LCD_ShowString(H_I, V_I * 4, MSG_OFF, color[11], color[0]);
  }

  // LED
  LCD_ShowString(H_I * 2, V_I * 4, MSG_LED, color[11], color[0]);
  if ( led_state == 1 ) {
    LCD_ShowString(H_I * 3, V_I * 4, MSG_ON, color[11], color[0]);
  }
  else {
    LCD_ShowString(H_I * 3, V_I * 4, MSG_OFF, color[11], color[0]);
  }
}


uint16_t TOUCH_WIDTH = 20;

void activate_blind() {
  blind_state = 1;
  // TODO 블라인드 치다
}

void remove_blind() {
  blind_state = 0;
  // TODO 블라인드 걷다
}

void control_blind() {
  if ( auto_mode ) {
    auto_mode = !auto_mode;
  }
  if ( blind_state == 0 ) {
    blind_state = 1;
    activate_blind();
  }
  else {
    blind_state = 0;
    remove_blind();
  }
}

void on_LED() {
  led_state = 1;
  GPIO_SetBits(GPIOD, GPIO_Pin_11);
}

void off_LED() {
  led_state = 0;
  GPIO_ResetBits(GPIOD, GPIO_Pin_11);
}

void control_LED() {
  if ( auto_mode ) {
    auto_mode = !auto_mode;
  }
  led_state = !led_state;
  if ( led_state == 0 ) {
    on_LED();
  }
  else {
    off_LED();
  }
}

void process_touch(uint16_t x, uint16_t y) {

  // BLIND sense
  if ( H_I * 0 <= x && x <= H_I * 1 && V_I * 4 - TOUCH_WIDTH <= y && y <= V_I * 4 + TOUCH_WIDTH ) {
    control_blind();
  }

  // LED sense
  if ( H_I * 1 <= x && x <= H_I * 2 && V_I * 4- TOUCH_WIDTH <= y && y <= V_I * 4 + TOUCH_WIDTH ) {
    control_LED();
  }

  // MODE sense
  if ( H_I * 0 <= x && x <= H_I * 1 && V_I * 2 - TOUCH_WIDTH <= y && y <= V_I * 2 + TOUCH_WIDTH ) {
    auto_mode = !auto_mode;
  }

}


int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();
    USART1_Configure();
    USART2_Configure();
    TIM_Configure();

    LCD_Init();
    Touch_Configuration();
    //Touch_Adjust();
    LCD_Clear(WHITE);
    
      LCD_ShowString(0, 0, MSG_TIME, color[11], color[0]);

  /*
    char msg[] = "WED_Team08";
    LCD_ShowString(0, 0, msg, color[11], color[0] );
*/
    uint16_t x, y;
   
    while (1) {
//      GPIO_SetBits(GPIOD, GPIO_Pin_11);
//      GPIOD->BSRR |= GPIO_Pin_11;
//      GPIO_SetBits(GPIOD, GPIO_Pin_12);
//      GPIO_SetBits(GPIOD, GPIO_Pin_13);
//      GPIO_ResetBits(GPIOD, GPIO_Pin_13);

      Touch_GetXY(&x, &y, 1);
      Convert_Pos(x, y, &x, &y);
      process_touch(x, y);
      
      /*
      LCD_ShowNum(0, 32, value, 10,color[11], color[0]);
      LCD_DrawCircle(x, y, 6);
*/
      
      LCD_indicate(time, 1200, auto_mode, blind_state, led_state);
      LCD_ShowNum(0, V_I * 5, x, 10,color[11], color[0]);
      LCD_ShowNum(0, V_I * 6, y, 10,color[11], color[0]);
      

    }
       
    return 0;
}
