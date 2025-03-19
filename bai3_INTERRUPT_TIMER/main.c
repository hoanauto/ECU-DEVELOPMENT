#include "RTE_Device.h"                 // Keil::Device:Startup
#include "stm32f10x.h"                  // Device header
#include "misc.h"                       // Keil::Device:StdPeriph Drivers:Framework
#include "RTE_Components.h"             // Component selection
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC

#define TIM2_BASE   0x40000000
#define TIM2_CR1  (*(volatile unsigned int *) (TIM2_BASE + 0x00))
#define TIM2_DIER (*(volatile unsigned int *) (TIM2_BASE + 0x0C))
#define TIM2_SR   (*(volatile unsigned int *) (TIM2_BASE + 0x10))
#define TIM2_PSC  (*(volatile unsigned int *) (TIM2_BASE + 0x28))
#define TIM2_ARR  (*(volatile unsigned int *) (TIM2_BASE + 0x2C))
#define TIM2_CNT   (*(volatile unsigned int *) (TIM2_BASE + 0x24))
	
#define RCC_BASE    0x40021000
#define RCC_APB1ENR (*(volatile unsigned int *) (RCC_BASE + 0x1C))
#define RCC_APB2ENR (*(volatile unsigned int *) (RCC_BASE + 0x18))

#define GPIOC_BASE    0x40011000
#define GPIOC_CRH (*(volatile unsigned int *) (GPIOC_BASE + 0x04))
#define GPIOC_ODR (*(volatile unsigned int *) (GPIOC_BASE + 0x0C))

#define NVIC_ISER0 (*(volatile unsigned int *) 0xE000E100)



static void RCC_Config(){
	RCC_APB1ENR |=1;// tim2 bit dau tien
	RCC_APB2ENR |=(1 << 4); // gpioc bit thu 4
	
}

static void GPIO_Config(){
	GPIOC_CRH &= ~((1 <<23) | (1 <<22));
	GPIOC_CRH |= (1 <<21) | (1 <<20);

}
 
static void TIM_Config(){ // Fclk = 72MHZ
	TIM2_PSC = 7200 -1; // PRESCALE = 7200 -> TIMER CHAY VOI 1KHZ (7200 xung thi se dem len 1)
	TIM2_ARR = 10000 -1; // -1 vi timer bat dau tu so 0
	
	TIM2_DIER |= (1<<0);// CHO PHEP NGAT TIMER 2 (ENABLE)
	TIM2_CR1 |= (1<<0); // BAT TIMER 2, CHO PHEP TIMER 2 HOAT DONG
	
	NVIC_ISER0 |= (1<< 28); //BAT NGAT TIMER2 TRONG NVIC (TIM HIEU THEM) 
}
	static void TIM2_IRQHandler(void)
{if(TIM2_SR &1){// kiem tra bit dau tien thanh ghi SR bat len 1 khi ngat say ra(update Interrupt Flag) 
	TIM2_SR &= ~1;// xoa lai co ngat, reset trang thai, neu k reset se lien tuc bao ngat
	GPIOC_ODR ^= (1<<13);}// bat trang thai chan PC13
}
	
int main(){
	RCC_Config();
	GPIO_Config();
	TIM_Config();
	
	while(1) {
	}
		
}
