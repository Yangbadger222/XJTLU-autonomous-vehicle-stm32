//user_delay.c
//2023.10.20
//by gjx

#include "user_delay.h"

static uint8_t dwt_cycle_counter_enabled = 0u;

static void EnableDwtCycleCounter(void)
{
    if (dwt_cycle_counter_enabled != 0u)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_cycle_counter_enabled = 1u;
}

void HAL_Delay_us(uint32_t us)
{
    uint32_t start_cycles;
    uint32_t wait_cycles;
    uint32_t cycles_per_us;

    if (us == 0u)
    {
        return;
    }

    // SysTick is the HAL millisecond time base used by the drive watchdog and
    // PID.  Do not reconfigure it for PS2 bit timing.
    EnableDwtCycleCounter();
    cycles_per_us = SystemCoreClock / 1000000u;
    wait_cycles = us * cycles_per_us;
    start_cycles = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start_cycles) < wait_cycles)
    {
    }
}