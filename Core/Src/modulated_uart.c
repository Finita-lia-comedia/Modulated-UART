#include "modulated_uart.h"

void irqHandlerTim2(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);

        if (g_bitIndex == 0)
        {
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
            TIM3->CCR1 = TIM3->ARR / 2;
        }
        else if (g_bitIndex <= 8)
        {
            if (g_txData & (1 << (g_bitIndex - 1)))
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
        else if (g_bitIndex == 9)
        {
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
            TIM3->CCR1 = TIM3->ARR + 1;
            g_txBusy = 0;
            LL_TIM_DisableIT_UPDATE(TIM2);
        }

        g_bitIndex++;
    }
}
