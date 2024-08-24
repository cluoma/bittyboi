//
// Created by colin on 8/23/24.
//

#ifndef BITTYBOI_TIMER_H
#define BITTYBOI_TIMER_H

#include <cstdint>
#include "mmu.h"

class Timer {
public:
    void tick(uint32_t clocks, mmu &mmu);
private:
    uint16_t div_count = 0;
    uint16_t div_overflow = 64;  // divider increments by 1 every this many clocks

    uint32_t m_clock_freq = 1048576;
    uint32_t clock_count = 0;
};


#endif //BITTYBOI_TIMER_H
