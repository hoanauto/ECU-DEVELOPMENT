#include "stm32f10x_rcc.h"          
#include "stm32f10x.h"       
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"  
#include "stm32f10x.h"
#include "DIO.h"
#include "Dio_Config.h"

void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms * 8000; i++) __NOP();
}

int main(void)
{
    // Bật Clock GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;

    // PC13 → Output Push-Pull (LED nhấp nháy)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PB0 → Output Push-Pull (LED điều khiển bằng nút)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PA0 → Input Pull-Up (Nút nhấn)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    while (1)
    {
        // Nhấp nháy LED tại PC13
        DIO_FlipChannel(Dio_ConfiguredChannels[LED_PC13_INDEX]);
        delay_ms(300);

        // Đọc nút nhấn tại PA0
        if (DIO_ReadChannel(Dio_ConfiguredChannels[BUTTON_A0_INDEX]) == STD_LOW)
        {
            DIO_WriteChannel(Dio_ConfiguredChannels[LED_B0_INDEX], STD_HIGH); // Bật LED PB0
        }
        else
        {
            DIO_WriteChannel(Dio_ConfiguredChannels[LED_B0_INDEX], STD_LOW); // Tắt LED PB0
        }
    }
}
