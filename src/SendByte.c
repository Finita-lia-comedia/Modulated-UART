#include "SendByte.h"

uint8_t tx_data = 0x00;
uint8_t bit_index = 0;
uint8_t tx_busy = 0;

void SendByte(uint8_t data)
{
    tx_data = data;
    bit_index = 0;
    tx_busy = 1;

    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableIT_UPDATE(TIM2);
}

void TIM2_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);

        if (bit_index == 0)  
        {
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
            TIM3->CCR1 = TIM3->ARR / 2;
        }
        else if (bit_index <= 8)  
        {
            if (tx_data & (1 << (bit_index - 1)))
            {
                LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
                TIM3->CCR1 = TIM3->ARR + 1;
            }
            else
            {
                LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
                TIM3->CCR1 = TIM3->ARR / 2;
            }
        }
        else if (bit_index == 9)  
        {
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
            TIM3->CCR1 = TIM3->ARR + 1;
            tx_busy = 0;
            LL_TIM_DisableIT_UPDATE(TIM2);
        }

        bit_index++;
    }
}
