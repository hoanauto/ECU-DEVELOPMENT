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

int main () {
	RCC_Config();
	GPIO_Config();
	SPI_Config();
	
	}

