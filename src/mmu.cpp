//
// Created by colin on 7/27/24.
//

#include <cstdio>
#include "mmu.h"

uint8_t mmu::read(uint16_t addr) {
    if (addr == 0x0104) {
        printf("reading Nintendo logo: %02x\n", mem[addr]);
    }

    if (mem[0xFF50] != 1) {
        return bootrom[addr];
    } else {
        return mem[addr];
    }
}

void mmu::write(uint16_t addr, uint8_t val) {
    mem[addr] = val;
}

void mmu::dump_mem(uint16_t addr, uint16_t bytes) {
    for (uint16_t i = 0; i < bytes; i+=16) {
        printf("/*0x%04x*/ ", addr + i);
        for (uint16_t j = 0; j < 16; j++) {
            printf("0x%02x, ", read(addr + i + j));
        }
        printf("\n");
    }
}
