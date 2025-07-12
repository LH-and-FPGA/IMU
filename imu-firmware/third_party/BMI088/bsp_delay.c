#include "bsp_delay.h"
#include "stdint.h"
#include <system_stm32h7xx.h>
#include "stm32h7xx.h"
// By LHandFPGA, switch to different stm lib for DWT delay if you use other STM32 series.

void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}