#include "stm32f10x.h"                  // Device header
#include "RTE_Device.h"                 // Keil::Device:Startup
#include "RTE_Components.h"             // Component selection

#define RCC_APB2ENR *((unsigned int *) 0x40021018) // derenference = lay gia tri tu dia chi
//0x40021018 o phai là gia tri ma vdk hieu, keilc hieu day la so nguyen
// muon lap trinh thanh ghi (doc,ghi,..) phai truy cap vao dia chi
// phai khai bao thanh ghi la 1 dia chi thi phai khai bao gia tri do la 1 pointer
// thanh ghi la 1 gia tri 32 bit = int trong keilc (ep kieu thanh pointer 32 bit)
// unsigned là k dau , int la 32 bit, * la con tro
// truy cap vao dia chi de lay gia tri trong thanh ghi
#define GPIOC_CRH *((unsigned int *) 0x40011004)// dia chi portc
#define GPIOC_ODR *((unsigned int *) 0x4001100C)
unsigned int i;
static void delay(unsigned int timedelay){
			for (i = 0 ; i< timedelay ; i++){}
	}
int main(){
	// cách 1
//RCC_APB2ENR |= 0x00000010; //1 so 0 la 4 ô 
	// cách 2: chuyen 1 qua 4 lan
 RCC_APB2ENR |= (1<<4);// bit 1 o vi tri thu 4, con lai bang 0 (GPIOC)
	GPIOC_CRH &= ~((1<<23)|(1<<22)); // ~(1:1) = (0:0)
	GPIOC_CRH |= ((1<<21)|(1<<20));
	while(1)
		{
			GPIOC_ODR |= 1<<13;
			delay(100000);
			GPIOC_ODR &=~(1<<13); // SANG LED
			delay(100000);
		}
}
