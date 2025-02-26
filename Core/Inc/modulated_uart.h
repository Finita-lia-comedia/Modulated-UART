#ifndef __MODULATED_UART_H
#define __MODULATED_UART_H

#include <stdint.h>
#include "main.h"

extern uint8_t g_txData;
extern uint8_t g_bitIndex;
extern uint8_t g_txBusy;

void irqHandlerTim2(void);

#endif