#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

#include <stdint.h>

void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);

#endif