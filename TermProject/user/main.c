#include "lcd.h"
#include "touch.h"
#include "init.h"

//---------------------------LCD--------------------------------------------------------

uint16_t V_I = 32; // Vertical Interval
uint16_t H_I = 60; // Horizontal Interval
uint16_t V_S = 10; // Vertical Shift
uint16_t H_S = 20; // Horizontal Shift
uint16_t H_END = 1000;
uint16_t TOUCH_WIDTH = 20;

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

//-----------------------------USART----------------------------------------------------

char usart2In;
char usart1In;

char cmd[200];
int index = 0;

void Command(void);

//-----------------------------BLIND----------------------------------------

void activate_blind(void);
void remove_blind(void); 
int blindOperation = 0;

enum BlindState{
  BlindUP,
  BlindDOWN
};

enum BlindState blind_state = BlindDOWN;

//-------------------------------TIME-------------------------------------

int TIME      = 0;
int MAX_TIME  = 24;
int AFTERNOON = 6;
int NIGHT     = 18;

int checktime;

//----------------------------LIGHT-------------------------------

uint16_t LIGHT;
void lumi_check(int lumi);

//----------------------------MODE--------------------------

enum Mode{
  AUTO,
  MANUAL
};
enum Mode mode = MANUAL;


enum LED{
  ON,
  OFF
};
enum LED LEDState = OFF;

//---------------------------METHOD-----------------------------
void control_blind (void);
void control_LED   (void);
void control_mode  (void);
void LCD_indicate  (void);
int  setTime       (void);
void UpBlind       (void);
void DownBlind     (void);
void StopBlind     (void);
void timeCheck     (void);
void touch_LED     (void);
//-----------------------------------------------------------


//--------------------------------INTERRUPT HANDLER--------------------------------

void ADC1_2_IRQHandler() {
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET ) {
    LIGHT = ADC_GetConversionValue(ADC1);
    
    if(mode == AUTO && blindOperation == 0){
      if(blindOperation == 0){
        lumi_check(LIGHT);
      }
    }
    
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }
}

void USART2_IRQHandler(void){
  if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET ) {
    usart2In = USART_ReceiveData(USART2);
    USART_SendData(USART1,usart2In);
    
    cmd[index++] = usart2In;
    if(cmd[index-1] == '\n'){
      Command();
      index = 0;
    }

    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

void Command(){
  if(cmd[0] == 'L' && mode == MANUAL){
    touch_LED();
  }
  else if(cmd[0] == 'B' && mode == MANUAL){
    control_blind();
  }
  else if(cmd[0] == 'M'){
    control_mode();
  }
}

void USART1_IRQHandler(void){
  if ( USART_GetITStatus(USART1, USART_IT_RXNE) != RESET ) {
    usart1In = USART_ReceiveData(USART1);
    USART_SendData(USART2, usart1In);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}


void TIM2_IRQHandler() {
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) {
   
      TIME+=1;
      if ( TIME >= MAX_TIME ) {
          TIME = 0;
      }
  
      LCD_ShowNum(174, 42, TIME, 5, color[11], color[0]);
      LCD_ShowNum(174, V_I * 2 + 10, LIGHT, 5, color[11], color[0]);
      
      if(mode == AUTO && blindOperation == 0){
        if(TIME == AFTERNOON || TIME == NIGHT){
          timeCheck();
        }
      }
      
      if(TIME == checktime && blindOperation == 1){
        StopBlind();
      }
      
      control_LED();
      LCD_indicate();
      
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

//---------------------------------------------------------------------------------------


//-------------------------------------LCD-------------------------------------------

void showLCD(void){
  LCD_ShowString(H_S, V_I * 2 + V_S, "LUMI",          color[11], color[0]);
  LCD_ShowString(H_S, V_I * 3 + V_S, "MODE",          color[11], color[0]);
  LCD_ShowString(H_S, V_I * 4 + V_S, "BLD" ,          color[11], color[0]);
  LCD_ShowString(H_S, V_I * 5 + V_S, "LED" ,          color[11], color[0]);
  LCD_ShowString(H_S, V_I * 2 + V_S, "LUMI",          color[11], color[0]);
  LCD_ShowString(60 , 10,            "SMART BLINDER", color[11], color[0]);
  LCD_ShowString(H_S, V_I + V_S,     "TIME",          color[11], color[0]);

  LCD_DrawLine(0, V_I * 1, H_END , V_I * 1);
  LCD_DrawLine(0, V_I * 2, H_END , V_I * 2);
  LCD_DrawLine(0, V_I * 3, H_END , V_I * 3);
  LCD_DrawLine(0, V_I * 4, H_END , V_I * 4);
  LCD_DrawLine(0, V_I * 5, H_END , V_I * 5);
  LCD_DrawLine(0, V_I * 6, H_END , V_I * 6);
  LCD_DrawLine(116, V_I, 116 , V_I * 6);
}


void LCD_indicate() {
  // MODE
  if (mode == AUTO) {
    LCD_ShowString(174, V_I * 3 + 10, "AUTO  ", color[11], color[0]);
  }
  else {
    LCD_ShowString(174, V_I * 3 + 10, "MANUAL", color[11], color[0]);
  }

  // Blind
  if (blind_state == BlindDOWN) {
    LCD_ShowString(174, V_I * 4 + 10, "DOWN",   color[11], color[0]);
  }
  else {
    LCD_ShowString(174, V_I * 4 + 10, "UP  ",   color[11], color[0]);
  }

  // LED
  if ( LEDState == ON ) {
    LCD_ShowString(174, V_I * 5 + 10, "ON ",    color[11], color[0]);
  }
  else {
    LCD_ShowString(174, V_I * 5 + 10, "OFF",    color[11], color[0]);
  }  
}


void process_touch(uint16_t x, uint16_t y) {

  // BLIND sense
  if ( ( 116<= x && x <=252 ) && ( V_I * 4<= y && y < V_I * 5) ) {
    if(blindOperation == 0){
      control_blind();
    }
  }

  // LED sense
  if ( ( 116<= x && x <=252 ) && ( V_I * 5<= y && y < V_I * 6) ) {
    touch_LED();
  }

  // MODE sense
  if ( ( 116<= x && x <=252 ) && (V_I * 3 <= y && y < V_I * 4) ) {
    control_mode();
  }
  LCD_indicate();
}


//---------------------------------------------------------------------------------



void lumi_check(int lumi){
  if(lumi < 2000 && TIME < AFTERNOON ){
    if(blind_state == BlindDOWN){
      UpBlind();
      checktime = setTime();
    }
  }
    
  else if(lumi >=2000 && TIME < NIGHT){
    if(blind_state == BlindUP){
      DownBlind(); 
      checktime = setTime();
    }
  }
  LCD_indicate();
}




void control_blind() {
  if ( mode == MANUAL) {
    if( blind_state == BlindDOWN){
      UpBlind();
      checktime = setTime();
    }else{
       DownBlind();
      checktime = setTime();
    }
  }
}

void touch_LED(){
  if( mode == MANUAL){
      if(LEDState == ON){
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        LEDState = OFF;
      }
      else{
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        LEDState = ON;
      }
  }
}

void control_LED() {
  if(mode == AUTO){
    if(blind_state == BlindDOWN){
      if(LEDState == ON ){
        LEDState = OFF;
        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
      }
    }
    else{
       if(LEDState == OFF ){
        LEDState = ON;
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
       }
    }
  }
}


void control_mode() {
  if ( mode == AUTO ) {
    mode = MANUAL;
  }
  else {
    mode = AUTO;
  }
}
 
void timeCheck(){
    if (TIME == AFTERNOON) {
      UpBlind();
      checktime = setTime();
    }
    else if (TIME == NIGHT) {
      DownBlind();
      checktime = setTime();
    }
}


int setTime(){
  int t = TIME+4;
  if( t >= MAX_TIME ){
    t -= MAX_TIME;
  }
  return t;
}

void UpBlind(){
  if (blind_state == BlindDOWN) { 
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    blind_state = BlindUP;
    blindOperation = 1;
  }
}

void DownBlind(){
   if (blind_state == BlindUP) { 
      GPIO_SetBits(GPIOD, GPIO_Pin_11);
      blind_state = BlindDOWN;
      blindOperation = 1;
   }
}

void StopBlind(){
  if(blind_state == BlindDOWN){
    GPIO_ResetBits(GPIOD, GPIO_Pin_11);
   }
   else{
     GPIO_ResetBits(GPIOD, GPIO_Pin_12);
   }
   blindOperation = 0;
}


 int main(void)
{
    myInit();
    LCD_Init();
    Touch_Configuration();
    
    uint16_t x, y;
    
    Touch_Adjust();
    LCD_Clear(WHITE);
    showLCD();
    
    while (1) {
      Touch_GetXY(&x, &y, 1);
      Convert_Pos(x, y, &x, &y);
      process_touch(x, y);
    }
    return 0;
}
