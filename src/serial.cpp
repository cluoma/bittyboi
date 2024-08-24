//
// Created by colin on 8/23/24.
//

#include "serial.h"

void Serial::tick(uint32_t clocks, mmu &mmu) {
    // hot path, no need to check anything while clocks increase
    if (clock_count + clocks < speed) {
        clock_count += clocks;
        return;
    }

    // increase clock count and modulo to add the leftovers
    // this should be fine for normal gameboy, maybe doesn't work with a colour gameboy
    clock_count = (clock_count + clocks) % speed;

    // shift out 1 bit
    if (mmu.read(0xFF02) & 0x80) {  // check that bit 7 of SB is set
        uint8_t sb = mmu.read(0xFF01);
        mmu.write(0xFF01, sb << 1);
        shifts ++;

        // check if we're done
        if (shifts == 8) {
            mmu.write(0xFF02, mmu.read(0xFF02) & 0x7F);  // reset bit 7 of SB
            mmu.write(0xFF0F, mmu.read(0xFF0F) | 0x08);  // raise serial interrupt flag
            shifts = 0;
        }
    }
}
