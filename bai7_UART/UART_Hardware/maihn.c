#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM

void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_10; //Chan RX
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;// Neu 2 chan de AF thi bi ngan mach, 2 chan thong nhau
	GPIO_Init(GPIOA, &GPIOInitStruct);
	
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_9; //Chan TX
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}

void TIMER_config(void){
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2; //Chia thanh clock nho hon de cap cho timer default: 72MHz (1s tao duoc 72 trieu dao dong) , DIV1: khong chia
    TIM_InitStruct.TIM_Prescaler = 36000; //Sau bao nhieu dao dong thi se dem len 1 lan.  1s = 72M giao dong, gia tri < 65535, neu lon hon thi doi bo chia
    //VD muon 1ms dem len 1 lan thi (10^-3)/(1/36M) = 36000 dao dong
    TIM_InitStruct.TIM_Period  = 0xFFFF;//Dem bao nhieu lan thi reset
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; //Set mode dem len tu 0
    TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
    
    TIM_Cmd(TIM2, ENABLE); //Cho phep timer2 hoat dong
    
    //chia clock cho 2 de 1s tao duoc 36.000.000 dao dong trong 1s, tuc 1 dao dong mat 1/36.000.000
    //Prescaler = 36.000 tuc la voi moi 36.000 dao dong thi dem len 1 lan
    //tuc la mat (1/36.000.000)*(36.000) = 1/1000s = 1ms thi dem len mot lan
}

void delay_ms(uint16_t timedelay){
    TIM_SetCounter(TIM2, 0);
    while(TIM_GetCounter(TIM2)<timedelay){}
}

void UART_Config(void){
	USART_InitTypeDef UARTInitStruct;
	UARTInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //Cau hinh che do: ca truyen va nhan (song cong)
	UARTInitStruct.USART_BaudRate = 115200; //Cau hinh toc do bit
	UARTInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Cau hinh kiem soat luong truyen du lieu tranh viec tran bo dem
	UARTInitStruct.USART_WordLength = USART_WordLength_8b; //Truyen du lieu 8 hoac 9 bit
	UARTInitStruct.USART_Parity = USART_Parity_No;
	UARTInitStruct.USART_StopBits = USART_StopBits_1;
	
	USART_Init(USART1, &UARTInitStruct);
	
	USART_Cmd(USART1, ENABLE);
	
}


void UART_SendChar(USART_TypeDef *USARTx, char data){
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)==RESET); // Cho khi thanh ghi DR trong de chen du lieu moi de gui
	
	USART_SendData(USARTx, data);
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);//Cho den khi truyen thanh cong
	
	
}


char UART_ReceiveChar(USART_TypeDef *USARTx){
	char tmp = 0x00;
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)==RESET);
	
	tmp = USART_ReceiveData(USARTx);
	
	return tmp;
}

uint8_t DataTrans[] = {'V','A','N','T','U'};//Du lieu duoc truyen di
int main() {
	RCC_Config();
	GPIO_Config();
	UART_Config();
	TIMER_config();	
	for(int i = 0; i<5; ++i){
			delay_ms(1998);
			UART_SendChar(USART1, DataTrans[i]);
			delay_ms(2);
		}
	UART_SendChar(USART1, '\n');

	while(1){
		UART_SendChar(USART1,UART_ReceiveChar(USART1));
	}
}