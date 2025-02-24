#ifndef __SENDBYTE_H
#define __SENDBYTE_H

extern uint8_t tx_data;
extern uint8_t bit_index;
extern uint8_t tx_busy;

void SendByte(uint8_t data);
void TIM2_IRQHandler(void);

#endif
