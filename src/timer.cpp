//
// Created by colin on 8/23/24.
//

#include <cstdio>
#include "timer.h"

void Timer::tick(uint32_t clocks, mmu &mmu) {
    // increment divider register
    div_count += clocks;
    while (div_count >= div_overflow) {
        div_count -= div_overflow;
        mmu.write(0xFF04, mmu.read(0xFF04)+1); // increment divider register
    }

    uint8_t tac = mmu.read(0xFF07);  // get TAC - timer control register

    if ( !(tac & 0x4) ) return; // timer is disabled

    uint32_t timer_freq;
    switch (tac & 0x3) {
        case 0:
            timer_freq = 256;
            break;
        case 1:
            timer_freq = 4;
            break;
        case 2:
            timer_freq = 16;
            break;
        case 3:
            timer_freq = 64;
            break;
    }

    //printf("Clock Count: %u Freq: %u Clocks: %u\n", clock_count, timer_freq, clocks);
    clock_count += clocks;

    while(clock_count >= timer_freq) {
        clock_count -= timer_freq;
        uint8_t tima = mmu.read(0xFF05);
        uint8_t tma  = mmu.read(0xFF06);
        //printf("TIMA: %02x TMA: %02x\n", tima, tma);
        tima += 1;
        //printf("TIMA: %02x TMA: %02x\n", tima, tma);
        if (tima == 0) {  // TIMA has overflowed
            mmu.write(0xFF05, tma);  // write TMA - timer modulo - to TIMA register
            mmu.write(0xFF0F, mmu.read(0xFF0F) | 0x04);  // raise timer interrupt flag
            //printf("Timer interrupt\n");
        } else {
            mmu.write(0xFF05, tima);
        }
    }
}
