//
// Created by colin on 7/27/24.
//

#ifndef BITTYBOI_MMU_H
#define BITTYBOI_MMU_H

#include <cstdint>

#define MEMSIZE 0xFFFF

class mmu {
public:
    mmu() {
        for (uint16_t i = 0; i < MEMSIZE; i++) {
            mem[i] = 0xFF;
        }
        for (uint16_t i = 0xFF00; i < MEMSIZE; i++) {
            mem[i] = 0;
        }
    }

    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t val);

    void dump_mem(uint16_t addr, uint16_t bytes);
private:
    uint8_t mem[MEMSIZE]{};
};


#endif //BITTYBOI_MMU_H
