#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_spi.h"              // Keil::Device:StdPeriph Drivers:SPI

#define SPI1_NSS 	GPIO_Pin_4
#define SPI1_SCK	GPIO_Pin_5
#define SPI1_MISO 	GPIO_Pin_6
#define SPI1_MOSI 	GPIO_Pin_7
#define SPI1_GPIO 	GPIOA
#define SPI_RCC RCC_APB2Periph_GPIOA
#include "stm32f10x.h"                  // Device header

static void RCC_Config(){
	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
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
static void delay_ms(uint32_t time) {
		TIM_SetCounter(TIM2,0); // dat count dem = 0
	 while ( TIM_GetCounter(TIM2) < time*10 ){}
}// cout dem  = time*10 thi dung lai 
static void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SPI1_NSS | SPI1_SCK | SPI1_MISO | SPI1_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
}
 static void SPI_Config(){
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;// slave or master
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;// truyen va nhan cung luc 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // cap tan so sung clock de lam viec (16 la he so chia)
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // = 0
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // doc tai canh 1
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // kich thuoc truyen du lieu
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;// chon bit dau tien gui di
	SPI_InitStructure.SPI_CRCPolynomial = 7; // 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // kiem soat chan NSS = phan mem
	
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE); // cho phep spi hoat dong
	 
}
static void SPI_Send1Byte(uint8_t data){
GPIO_ResetBits(GPIOA, SPI1_NSS);// ha chan Cs = 0
while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}// kiemtra thanhghi DR có trong hay chua
SPI_I2S_SendData(SPI1, data); //sendata
while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET){}
	GPIO_SetBits(GPIOA, SPI1_NSS);
}
static uint8_t dataSend[] = {3,1,10,19,20,36,90};
int main () {
	RCC_Config();
	GPIO_Config();
	SPI_Config();
	TIM_config();
	for(uint8_t i=0; i<7; i++)
	{
	SPI_Send1Byte(dataSend[i]);
		delay_ms(1000);
		}
	}

