#ifndef __SENDBYTE_H
#define __SENDBYTE_H

#include <stdint.h>
#include "main.h"

extern uint8_t g_txData;
extern uint8_t g_bitIndex;
extern uint8_t g_txBusy;

void sendByte(uint8_t data);

#endif