#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "RTE_Components.h"             // Component selection
#include "RTE_Device.h"                 // Keil::Device:Startup

 unsigned int i;
static void delay(unsigned int timedelay){
			for (i = 0 ; i< timedelay ; i++){}
	}
	

static void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA ,ENABLE);
	// cho phep cap clock cho ngoai vi gpioc (ngoai vi muon cap clock, enable or disable)
	
	}

 static void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStruct;// tao bien de dung struct
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// out push pull
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7; // CAI DAT 4 CHAN CUNG LUC
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	// PA0
	
	}

int main (){
	RCC_Config();
	GPIO_Config();
	while(1)
		{ GPIO_SetBits(GPIOC, GPIO_Pin_13);// off
			delay(10000);
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);//ON
			delay(10000);
			// vi du chay led
			// chaseLed(3);
			//break;
// vi du nut nhan
//if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0){
//while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0);// khi nao nha nut nhan moi hoat dong code
//if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)){// kiem tra trang thai led13 dang la gi
//GPIO_ResetBits(GPIOC, GPIO_Pin_13);
//} else {
//GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
}

		

