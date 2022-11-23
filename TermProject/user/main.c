
#include "stm32f10x.h"

#define RCC_APB2ENR (*(volatile unsigned int *)0x40021018)
#define GPIOC_CRL (*(volatile unsigned int *)0x40011000)
#define GPIOC_IDR (*(volatile unsigned int *)0x40011008)
#define GPIOD_ODR (*(volatile unsigned int *)0x4001140C)
#define GPIOD_IDR (*(volatile unsigned int *)0x40011408)
#define GPIOD_CRH (*(volatile unsigned int *)0x40011404)


#define GPIOD_CRL (*(volatile unsigned int *)0x40011400)

#define GPIOC_BSRR (*(volatile unsigned int *)0x40011010)
#define GPIOD_BSRR (*(volatile unsigned int *)0x40011410)
#include "stm32f10x.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "core_cm3.h"

void RCC_Configure() {

RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}
void GPIO_Configure() {

GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_2);
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_10 | GPIO_Pin_3);
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);

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
}


char usart1In = 'a';
char usart2In = 'a';

void delay(void){
int i = 0;
for(i=0;i<1000000;i++);
}

void delay2(void){
int i = 0;
for(i=0;i<10000;i++);
}

void sendDataUART1(uint16_t data) {
	/* Wait till TC is set */
	while ((USART1->SR & USART_SR_TC) == 0);
	USART_SendData(USART1, data);
}

int myflag;
void USART1_IRQHandler(void){
  if ( USART_GetITStatus(USART1, USART_IT_RXNE) != RESET ) {
    usart1In = USART_ReceiveData(USART1);
    USART_SendData(USART2, usart1In);
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
  
  uint16_t word;
  if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
    	// the most recent received data by the USART1 peripheral
    word = USART_ReceiveData(USART1);

        // TODO implement
    if(word == 'a'){
      myflag = 0;
      sendDataUART1('a');
    }else if(word == 'b') {
      myflag =1;
      sendDataUART1('a');
    }
    
        // clear 'Read data register not empty' flag
      USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}


void USART2_IRQHandler(void){
  if ( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET ) {
    usart2In = USART_ReceiveData(USART2);
    USART_SendData(USART1, usart2In);
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}


 int main(void){
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  USART1_Configure();
  USART2_Configure();
  NVIC_Configure();
  
  RCC_APB2ENR |= 0x30;

  
  GPIOC_CRL &= ~0x00F00F00;
  GPIOC_CRL |= 0x00800800;
  
  GPIOD_CRH &= ~0x000FF000;
  GPIOD_CRH |= 0x00033000;
  
  GPIOC_CRL = 0x800800;
  GPIOD_CRH = 0x00011000;

  GPIOC_BSRR |= 0x240000;
  GPIOD_BSRR |= 0x8000000;
  
  //GPIOD_BSRR |= 0x800;

   while(1){
     if(myflag == 0){ //down
       GPIOD_ODR  &= ~0x1800;
       GPIOD_ODR |= 0x800;
       delay();
     }
     else if(myflag == 1){ // up
          GPIOD_ODR  &= ~0x1800;
          GPIOD_ODR |= 0x1000;
          
          delay();
     }
     else if((GPIOD_IDR & 0x800) == 0){ // s1
        GPIOD_ODR  &= ~0x1800;
     }
               
      if((GPIOC_IDR & 0x04) == 0){ //down
                GPIOD_ODR  &= ~0x1800;
        GPIOD_ODR |= 0x800;
        delay();
      }
    
      else if((GPIOC_IDR & 0x20) == 0){ // up
                GPIOD_ODR  &= ~0x1800;
          GPIOD_ODR |= 0x1000;
          
          delay();
      }
      else if((GPIOD_IDR & 0x800) == 0){ // s1
        GPIOD_ODR  &= ~0x1800;
     }
  }
}
