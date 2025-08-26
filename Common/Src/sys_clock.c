/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "sys_clock.h"
#include "stm32h7xx_hal.h"

uint32_t getCurrentTime(void) {
    return HAL_GetTick();
}

uint64_t getHighResTime_ns(void)
{
    uint32_t load = SysTick->LOAD + 1;
    uint32_t tick_per_ms = load;
    uint32_t ms, val1, val2, tick_in_ms;
    uint64_t ns;

    do {
        val1 = SysTick->VAL;
        ms = HAL_GetTick();
        val2 = SysTick->VAL;
    } while (val1 < val2); // 如果val1<val2，说明中间SysTick溢出了，重新读取

    tick_in_ms = tick_per_ms - val2;
    uint32_t ns_per_tick = 1000000UL / tick_per_ms; // 1ms = 1,000,000ns
    ns = (uint64_t)ms * 1000000ULL + (uint64_t)tick_in_ms * ns_per_tick;
    return ns;
}

float getFreq(Caculate_Freq_t *freq)
{
    freq->CurrentSampleTime = getHighResTime_ns();
    uint64_t deltaTime_ns = freq->CurrentSampleTime - freq->PreviousSampleTime;
    if(deltaTime_ns > 0)
    {
        freq->Freq = 1/(deltaTime_ns * 1e-9f);
    }
    else
    {
        freq->Freq = 0.0f;
    }
    freq->PreviousSampleTime = freq->CurrentSampleTime;
    return freq->Freq;
}
