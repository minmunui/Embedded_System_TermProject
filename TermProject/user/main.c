#include "lcd.h"
#include "touch.h"
#include "init.h"

//---------------------------LCD--------------------------------------------------------
char MSG_TITLE[] = "SMART BLINDER";
char MSG_TIME[] = "TIME";
char MSG_LUMIN[] = "LUMI";
char MSG_MODE[] = "MODE";
char MSG_AUTO[] = "AUTO  ";
char MSG_MAN[] = "MANUAL";
char MSG_LED[] = "LED";
char MSG_BLIND[] = "BLD";
char MSG_ON[] = "ON ";
char MSG_OFF[] = "OFF";

uint16_t V_I = 32; // Vertical Interval
uint16_t H_I = 60; // Horizontal Interval
uint16_t V_S = 10; // Vertical Shift
uint16_t H_S = 20; // Horizontal Shift
uint16_t H_END = 1000;
uint16_t TOUCH_WIDTH = 20;

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};
//---------------------------------------------------------------------------------------

uint16_t blind_state = 1;
uint16_t led_state = 1;
uint16_t auto_mode = 1;


uint16_t value;

void concate_input_string(char input_char);
void process_command();

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

char usart2In;
void USART2_IRQHandler(void){
  if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET ) {
    usart2In = USART_ReceiveData(USART2);
    if (usart2In == '\0') {
      process_command();
    }
    else {
      concate_input_string(usart2In);
    }
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

/*
// ORIGINAL USART2 HANDLER
void USART2_IRQHandler(void){
  if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET ) {
    usart2In = USART_ReceiveData(USART2);
    USART_SendData(USART1, usart2In);
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}
*/

char usart2In;
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

//uint16_t a = 0;
uint16_t count = 0;

void timeCheck(){
  if(time == 60 ){
    GPIO_SetBits(GPIOD, GPIO_Pin_11);
  }
  if(time == 120){
    GPIO_ResetBits(GPIOD, GPIO_Pin_11);
    time = 0;
  }
  LCD_ShowNum(174, 42, time, 5, color[11], color[0]);
  LCD_ShowNum(174, V_I * 2 + 10, value, 5, color[11], color[0]);
}

void TIM2_IRQHandler() {
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) {
      count ++;
   
      time+=TICK;
      timeCheck();
      
      /*
      change_pulse(sub[cnt++]);
      if (cnt == 2)
        cnt = 0;
      */     
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}


void LCD_indicate(uint16_t input_time, uint16_t input_lumin, uint16_t is_blind, uint16_t is_led, uint16_t auto_mode) {
  // MODE
  if (auto_mode == 1) {
    LCD_ShowString(174, V_I * 3 + 10, MSG_AUTO, color[11], color[0]);
  }
  else {
    LCD_ShowString(174 , V_I * 3 + 10, MSG_MAN, color[11], color[0]);
  }

  // Blind
  if (blind_state == 1) {
    LCD_ShowString(174 , V_I * 4 + 10, MSG_ON, color[11], color[0]);
  }
  else {
    LCD_ShowString(174, V_I * 4 + 10, MSG_OFF, color[11], color[0]);
  }

  // LED
  if ( led_state == 1 ) {
    LCD_ShowString(174, V_I * 5 + 10, MSG_ON, color[11], color[0]);
  }
  else {
    LCD_ShowString(174, V_I * 5 + 10, MSG_OFF, color[11], color[0]);
  }  
}



void motor_delay() {
  for ( int i = 0 ; i < 100000 ; i ++ ) {
  }
}

// LED PIN = 11
// ACTIVATE BLIND PIN = 12
// REMOVE BLIIND PIN = 13
// ALARM PIN = 14

void activate_alarm() {
    
}

void activate_blind() {
  blind_state = 1;
  // TODO 
  GPIO_SetBits(GPIOD, GPIO_Pin_12);
}

void remove_blind() {
  blind_state = 0;
  // TODO 
  GPIO_SetBits(GPIOD, GPIO_Pin_11);
  
}

void control_blind() {
  if ( auto_mode == 0) {
    if( blind_state == 1){
      remove_blind();
    }else{
      activate_blind();
    }
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
  if ( auto_mode == 0) {
    if( led_state == 1){
      off_LED();
    }else{
      on_LED();
    }
  }
}

void control_mode() {
  
}

void check_activity_time() {
  if ( auto_mode == 1 /* && activity time*/ ) {
    
  }
}

void process_touch(uint16_t x, uint16_t y) {

  // BLIND sense
  if ( ( 116<= x && x <=252 ) && ( V_I * 4<= y && y < V_I * 5) ) {
    control_blind();
  }

  // LED sense
  if ( ( 116<= x && x <=252 ) && ( V_I * 5<= y && y < V_I * 6) ) {
    control_LED();
  }

  // MODE sense
  if ( ( 116<= x && x <=252 ) && (V_I * 3 <= y && y < V_I * 4) ) {
    if(auto_mode == 0){
      auto_mode = 1;
    }else{
      auto_mode = 0;
    }
  }
}

void showLCD(void){
  LCD_ShowString(H_S, V_I * 2 + V_S, MSG_LUMIN, color[11], color[0]);
  LCD_ShowString(H_S, V_I * 3 + V_S , MSG_MODE, color[11], color[0]);
  LCD_ShowString(H_S, V_I * 4 + V_S, MSG_BLIND, color[11], color[0]);
  LCD_ShowString(H_S, V_I * 5 + V_S, MSG_LED, color[11], color[0]);
  LCD_ShowString(H_S, V_I * 2 + V_S, MSG_LUMIN, color[11], color[0]);
  LCD_ShowString(60, 10, MSG_TITLE, color[11], color[0]);
  LCD_ShowString(H_S, V_I + V_S, MSG_TIME, color[11], color[0]);

  LCD_DrawLine(0, V_I * 1, H_END , V_I * 1);
  LCD_DrawLine(0, V_I * 2, H_END , V_I * 2);
  LCD_DrawLine(0, V_I * 3, H_END , V_I * 3);
  LCD_DrawLine(0, V_I * 4, H_END , V_I * 4);
  LCD_DrawLine(0, V_I * 5, H_END , V_I * 5);
  LCD_DrawLine(0, V_I * 6, H_END , V_I * 6);
  LCD_DrawLine(116, V_I, 116 , V_I * 6);
}



uint16_t UINT16_T_MAX = 65535;

uint16_t BUFFER_SIZE = 30;
char buffer_input[30] = "\0";
uint16_t buffer_cursor = 0;
char E_M[] = "BUF OVER";

char* TIM = "TIM";
char* LED = "LED";
char* BLI = "BLI";
char* MOD = "MOD";
char* INVALID_ERROR = "Invalid Value\0";
char* TIME_SET_MSG = "Time Set Complete";
char* TIME_REQUEST_MESSAGE = "Please Input TIME Do You Want\0";

uint16_t input_time_flag = 0;

void print_message(char * message) {
    LCD_ShowString(H_S, V_I * 7 + V_S, MSG_LUMIN, color[11], color[0]);
}

void bluetooth_message(char * message) {
  if (message[0] == '\0') {}
  else
    for ( int i = 0 ; message[i] != '\0' ; i ++ )
      USART_SendData(USART2, usart1In);
}

void buffer_flush() {
  buffer_input[0] = "\0";
  buffer_cursor = 0;
}

void concate_input_string(char input_char) {
  if ( buffer_cursor >= BUFFER_SIZE ) {
    print_message( E_M );
  }
  else {
    buffer_input[buffer_cursor] = input_char;
    buffer_cursor += 1;
    buffer_input[buffer_cursor] = '\0';
  }
}

// TODO : I'm implementing strcmp to filter command

uint16_t strcmp(char* str1, char* str2) {
  for ( int i = 0 ;  ; i ++ ) {
    if ( str1[i] == str2[i] ) {
      if ( str2[i] == '\0' && str1[i] == '\0' ) {
        return 1;
      }
    }
    else {
      return 0;
    }
  }
}

uint16_t is_digit(char char_to_test) {
  if ( char_to_test >= '\0' && char_to_test <= '\0') {
    return 1;
  }
  else {
    return 0;
  }
}

uint16_t string_to_decimal(char* input) {
  uint16_t result = 0;
  for ( int i = 0 ; input[0]!='\0' ; i++) {
    result += input[i] - '0';
    if ( is_digit(input[i]) ) {
      return UINT16_T_MAX;
    }
    if ( input[i+1] == '\0') {
      return result;
    }
    else {
      result *= 10;
    }
  }
}

void process_command() {
  if ( input_time_flag == 0 ) {
    if (strcmp(TIM, buffer_input) == 1) {
      input_time_flag = 1;
      bluetooth_message(TIME_REQUEST_MESSAGE);
    } else if (strcmp(LED, buffer_input) == 1) {
      control_LED();
      buffer_flush();
    } else if (strcmp(BLI, buffer_input) == 1) {
      control_blind();
      buffer_flush();
    } else if (strcmp(MOD, buffer_input) == 1) {
      control_mode();
      buffer_flush();
    } else {
      bluetooth_message(INVALID_ERROR);
      buffer_flush();
    }
  }
  else {
    uint16_t temp = string_to_decimal(buffer_input);
    if ( temp == UINT16_T_MAX ) {
      bluetooth_message(INVALID_ERROR);
    }
    else {
      time = temp;
      bluetooth_message(TIME_SET_MSG);
      bluetooth_message(TIME_REQUEST_MESSAGE);
      input_time_flag = 0;
    }
    buffer_flush();
  }
}

int main(void)
{
    myInit();
    LCD_Init();
    Touch_Configuration();
//    Touch_Adjust();
    LCD_Clear(WHITE);
    
    showLCD();

    uint16_t x, y;
    
   while (1) {

      //Touch_GetXY(&x, &y, 1);
      //Convert_Pos(x, y, &x, &y);
      //process_touch(x, y);
      
      //LCD_indicate(time, value, blind_state, led_state, auto_mode);
      
      //activate_blind();
      //remove_blind();
    }
    return 0;
}
