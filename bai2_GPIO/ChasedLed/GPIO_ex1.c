#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "RTE_Components.h"             // Component selection
#include "RTE_Device.h"                 // Keil::Device:Startup

static void delay(uint32_t time)
	{for(uint32_t i; i <time; i++);
	}
	static	void chaseLed(uint8_t loop){
	uint16_t Ledval;
	for(int j = 0; j < loop; j++)
{
		Ledval = 0x0010; //0b0 0001 0000
		for( int a = 0; a < 4; a++)
		{
			
			GPIO_Write(GPIOA, Ledval);
			Ledval = Ledval << 1;
			delay(1000000000);
		}
	}
}

static void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA ,ENABLE);
	// cho phep cap clock cho ngoai vi gpioc (ngoai vi muon cap clock, enable or disable)
	
	}

 static void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStruct;// tao bien de dung struct
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;// out push pull
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7; // CAI DAT 4 CHAN CUNG LUC
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	}

int main (){
	RCC_Config();
	GPIO_Config();
	while(1)
		{ 
			// vi du chay led
			chaseLed(3);
			break;

}
}
		

