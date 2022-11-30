#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "misc.h"
#include "core_cm3.h"
#include "lcd.h"
#include "touch.h"

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

void RCC_Configure() {

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

}

void GPIO_Configure() {

  // From Bluetooth
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

void NVIC_Configure() {

  // From Bluetooth
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
  NVIC_Init(&NVIC_InitStructure);


  //From LCD
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_EnableIRQ(ADC1_2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
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

void delay(void){
  int i = 0;
  for(i=0;i<1000000;i++);
}

void delay2(void){
  int i = 0;
  for(i=0;i<10000;i++);
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

uint16_t value;

void ADC1_2_IRQHandler() {
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET ) {
    value = ADC_GetConversionValue(ADC1);
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
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

  LCD_Init();
  Touch_Configuration();
  Touch_Adjust();
  LCD_Clear(WHITE);

//  char msg[] = "WED_Team08";
//  LCD_ShowString(0, 0, msg, color[11], color[0] );
  uint16_t x, y;

  while (1) {
    Touch_GetXY(&x, &y, 1);
    Convert_Pos(x, y, &x, &y);
    LCD_ShowNum(0, 32, value, 10, color[11], color[0]);
    LCD_ShowNum(0, 64, x, 10, color[11], color[0]);
    LCD_ShowNum(0, 90, y, 10, color[11], color[0]);
    LCD_DrawCircle(x, y, 6);

    // TOUCH INDICATE

  }
  return 0;
}

#define ON = true;
#define OFF = false;

char MSG_TIME[] = "TIME : ";
char MSG_LUMIN[] = "LUMI : ";
char MSG_MODE[] = "MODE : ";
char MSG_AUTO[] = "AUTO";
char MSG_MAN[] = "MANUAL";
char MSG_LED[] = "LED : ";
char MSG_BLIND[] = "BLD : ";
char MSG_ON[] = "ON";
char MSG_OFF[] = "OFF";

bool blind_state = true;
bool led_state = true;
bool auto_mode = true;

uint16_t V_I = 32; // Vertical Interval
uint16_t H_I = 70; // Horizontal Interval


void LCD_indicate(uint16_t input_time, uint16_t input_lumin, bool is_blind, bool is_led, bool auto_mode) {

  // TIME
  LCD_ShowString(0, 0, MSG_TIME, color[11], color[0]);
  LCD_ShowNum(H_I, 0, input_time, color[11], color[0]);

  // LUMIN
  LCD_ShowString(0, V_I, MSG_LUMIN, color[11], color[0]);
  LCD_ShowNum(H_I, V_I, input_lumin, color[11], color[0]);

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
  if (blind_state) {
    LCD_ShowString(H_I, V_I * 4, MSG_ON, color[11], color[0]);
  }
  else {
    LCD_ShowString(H_I, V_I * 4, MSG_OFF, color[11], color[0]);
  }

  // LED
  LCD_ShowString(H_I * 2, V_I * 4, MSG_LED, color[11], color[0]);
  if ( led_state ) {
    LCD_ShowString(H_I * 3, V_I * 4, MSG_ON, color[11], color[0]);
  }
  else {
    LCD_ShowString(H_I * 3, V_I * 4, MSG_OFF, color[11], color[0]);
  }
}

uint16_t TOUCH_WIDTH = 10;

void activate_blind() {
  // TODO 블라인드 치다
}

void remove_blind() {
  // TODO 블라인드 걷다
}

void control_blind() {
  if ( auto_mode ) {
    auto_mode = !auto_mode;
  }
  if ( blind_state == false ) {
    blind_state = true;
    activate_blind();
  }
  else {
    blind_state = false;
    remove_blind();
  }
}

void on_LED() {
  // TODO LED 키다
}

void off_LED() {
  // TODO LED 끄다
}

void control_LED() {
  if ( auto_mode ) {
    auto_mode = !auto_mode;
  }
  led_state = !led_state;
  if ( led_state == false ) {
    on_LED();
  }
  else {
    off_LED();
  }
}

// TODO : 터치 위치 감지하여 Blind LED 조작하기
void process_touch(uint16_t x, uint16_t y) {

  // BLIND sense
  if ( H_I * 0 <= x && x <= H_I * 1 && V_I - TOUCH_WIDTH <= y && y <= V_I + TOUCH_WIDTH ) {
    control_blind();
  }

  // LED sense
  if ( H_I * 2 <= x && x <= H_I * 3 && V_I - TOUCH_WIDTH <= y && y <= V_I + TOUCH_WIDTH ) {
    control_LED();
  }

  // MODE sense
  if ( H_I * 4 <= x && x <= H_I * 5 && V_I - TOUCH_WIDTH <= y && y <= V_I + TOUCH_WIDTH ) {
    auto_mode = !auto_mode;
  }

}

bool is_activity_time() {
  // TODO : This function return boolean according to time
}

uint16_t bright_threshold = 4000;

bool is_bright() {
  if ( value < bright_threshold ) {
    return true;
  }
  else {
    return false;
  }
}

void activate_auto_mode() {
  if ( !is_activity_time() ) {
    off_LED();
    remove_blind();
    flag = false;
  }
  else if ( is_bright() ){
    activate_blind();
    off_LED();
    flag - true;
  }
  else if ( !is_bright() ) {
    if ( flag == false ) {
      on_LED();
    }
  }
}

uint16_t time_coefficient = 1000;

struct Time {
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
} Time;

void convert_int_to_time(uint16_t time_to_convert, Time* converted_time) {
  time_to_convert /= time_coefficient;
  converted_time->second = time_to_convert % 60;
  time_to_convert /= 60;
  converted_time->minute = time_to_convert % 60;
  time_to_convert /= 60;
  converted_time->hour = time_to_convert % 60;
}

uint16_t convert_time_to_int(Time* time_to_convert) {
  uint16_t temp = time_to_convert->hour;
  temp *= 60;
  temp += time_to_convert->minute;
  temp *= 60;
  temp += time_to_convert->second;
  temp *= time_coefficient;
  return temp;
}