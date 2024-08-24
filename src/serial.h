//
// Created by colin on 8/23/24.
//

#ifndef BITTYBOI_SERIAL_H
#define BITTYBOI_SERIAL_H

#include <cstdint>
#include "mmu.h"

class Serial {
public:
    void tick(uint32_t clocks, mmu &mmu);
private:
    enum State {
        FINISHED,
        WRITING,
        READING
    };
    uint32_t clock_count = 0;
    State state = FINISHED;
    uint32_t speed = 128; // 128 m-clocks per write/read (8192 Hz)
    uint32_t shifts = 0;
};


#endif //BITTYBOI_SERIAL_H
