#include "sendByte.h"

uint8_t g_txData = 0x00;
uint8_t g_bitIndex = 0;
uint8_t g_txBusy = 0;

void sendByte(uint8_t data)
{
    g_txData = data;
    g_bitIndex = 0;
    g_txBusy = 1;

    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableIT_UPDATE(TIM2);
}