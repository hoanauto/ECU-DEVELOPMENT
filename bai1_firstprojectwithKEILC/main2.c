// xây dung cau truc thanh ghi cua cac ngoai vi
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

void delay(unsigned int timedelay){
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
 RCC -> APB2ENR |= ((1<<4)|(1<<2));// bit 1 o vi tri thu 4 va 2, con lai bang 0 (GPIOC va GPIOA)
	GPIOC -> CRH&= ~((1<<23)|(1<<22)); // ~(1:1) = (0:0)
	GPIOC ->CRH|= ((1<<21)|(1<<20));
	// PA0
	GPIOA-> CRL &= ~((1<<0)|(1<<1)|(1<<2));
	GPIOA ->CRL |=  (1<<3);
	GPIOA -> ODR |= 1; // pull up or pull down
	while(1)
		{
			if ((GPIOA -> IDR & (1<<0)) ==0) // and voi mat na 1
				{
					GPIOC->ODR |= 1<<13;} // =1` thì tat led
				else{
		
			GPIOC->ODR &=~(1<<13); // SANG LED
		
			}
		}
}

