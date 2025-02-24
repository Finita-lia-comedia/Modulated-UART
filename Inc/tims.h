#ifndef __TIMS_H
#define __TIMS_H

#define BAUDRATE 2400
#define BIT_TIME (1000000 / BAUDRATE)

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);

#endif