#include "valves.h"

// (I stole this from a stackoverflow post)
// Delay in microseconds, useful for timing with the clock line
void DelayUS(uint32_t us) {
    uint32_t start = TIMx->CNT;
    uint32_t duration = us * 16;
    while (TIMx->CNT - start < duration);
}