        // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

#define SPI_SCK_Pin GPIO_Pin_0
#define SPI_MISO_Pin GPIO_Pin_1
#define SPI_MOSI_Pin GPIO_Pin_2
#define SPI_CS_Pin GPIO_Pin_3
#define SPI_GPIO GPIOA
#define SPI_RCC RCC_APB2Periph_GPIOA
static void RCC_Config(){
	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
static void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}


static void TIM_config()
	{
		TIM_TimeBaseInitTypeDef  TIM_InitStruct;
		TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;// 72Mhz
		TIM_InitStruct.TIM_Prescaler = 7200 -1; // 0.1ms count ++
		TIM_InitStruct. TIM_Period  = 0xFFFF;
		TIM_InitStruct. TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_InitStruct); // ghi vao TIM2
		TIM_Cmd (TIM2,ENABLE);
	}
	
		//delay tuyet doi
 static void delay_ms(uint32_t time) {
		TIM_SetCounter(TIM2,0); // dat count dem = 0
	 while ( TIM_GetCounter(TIM2) < time *10){} // cout dem  = time*10 thi dung lai 
		 }// vi 0.1ms count ++ -> muon thanh ms thi time phai *10 de count dem gap 10 lan nua
static	void SPI_Init(){ // trang thai idle (nghi)
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
}

		 static void Clock(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
	delay_ms(4);
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	delay_ms(4);
}
		static void SPI_Master_Transmit(uint8_t u8Data){	//0b10010000
	uint8_t u8Mask = 0x80;	// 0b10000000
	uint8_t tempData;
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_RESET);
	delay_ms(1); // chac chan BIT dc xuong muc 0 roi moi tiep tuc chuong trinh
	for(int i = 0; i < 8; i++){
		tempData = u8Data & u8Mask;
		if(tempData){// neu tempdata la 1 thì set
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_SET);
			delay_ms(1);
		} else{
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
			delay_ms(1);
		}
		u8Data = u8Data << 1; // dich bit sang trai 0b01010000
		Clock();
	}
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	delay_ms(1);
}


  static uint8_t DataTrans[] = {1,3,9,10,15,19,90};//Data
int main(){
	RCC_Config();
	GPIO_Config();
	TIM_config();
	SPI_Init();
	while(1){	
		for(int i = 0; i < 7; i++){
			SPI_Master_Transmit(DataTrans[i]);
			delay_ms(1000); // 1s truyen 1 lan
	
		}
	}
}

