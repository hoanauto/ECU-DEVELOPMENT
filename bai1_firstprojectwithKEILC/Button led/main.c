// xây dung cau truc thanh ghi cua cac ngoai vi
#include "RTE_Device.h"                 // Keil::Device:Startup


typedef struct
{
  unsigned int CRL;
  unsigned int CRH;
  unsigned int IDR;
  unsigned int ODR;
  unsigned int BSRR;
  unsigned int BRR;
  unsigned int LCKR;
} GPIO_TypeDef;
typedef struct
{
  unsigned int CR; // 1 phan tu chiem 32 bit, phan tu tiep theo cung chiem 32bit tiep theo
  unsigned int CFGR;
  unsigned int CIR;
  unsigned int APB2RSTR;
  unsigned int APB1RSTR;
  unsigned int AHBENR;
  unsigned int APB2ENR;
  unsigned int APB1ENR;
  unsigned int BDCR;
  unsigned int CSR;
} RCC_TypeDef;

static void delay(unsigned int timedelay){
			for ( unsigned int i = 0 ; i< timedelay ; i++){}
	}

#define RCC ((RCC_TypeDef *) 0x40021000) // dia chi cua struct
#define GPIOC ((GPIO_TypeDef *) 0x40011000) // dia chi port c
	#define GPIOA ((GPIO_TypeDef *) 0x40010800)

int main(){
	// cách 1
//RCC_APB2ENR |= 0x00000010; //1 so 0 la 4 ô 
	// cách 2: chuyen 1 qua 4 lan
	// PC13
 RCC -> APB2ENR |= (1<<4)|(1<<2);// bit 1 o vi tri thu 4 va 2, con lai bang 0 (GPIOC va GPIOA)
	GPIOA->CRL |= (1 << 1); // CNF0 = 10: Input with pull-up/pull-down
    GPIOA->CRL &= ~(1 << 0);	       
    GPIOA->ODR |= (1 << 0); // Set ODR0, PA0 là input pull-up

    GPIOC->CRH |= (1 << 20) | (1 << 21); // MODE13 = 11: Output mode, max speed 50 MHz
    GPIOC->CRH &= ~((1 << 22) | (1 << 23)); // CNF13 = 00: General purpose output push-pull

while(1){
	if((GPIOA->IDR & (1 << 0)) == 0) // Ð?c tr?ng thái nút nh?n
	{
		GPIOC->ODR = 0 << 13;   // N?u PA0 = 0 -> PC13 = 0
	}
	else
	{
		GPIOC->ODR = 1 << 13;   // N?u PA0 = 1 -> PC13 = 1
	}
}

}
