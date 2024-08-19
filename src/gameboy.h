//
// Created by colin on 7/27/24.
//

#ifndef BITTYBOI_GAMEBOY_H
#define BITTYBOI_GAMEBOY_H

#include <cstdint>
#include "cpu.h"

#define ROMSIZE 0xFFFF

class gameboy {

public:
    explicit gameboy() : _ppu(gb_screen_buffer) {
    };

    void init_no_bootrom();
    void load_rom(const char * filename);
    int tick();
    bool is_vblank();

    uint8_t * get_screen_buffer() {
        return gb_screen_buffer;
    }

private:
    uint8_t rom[ROMSIZE] = {0};
    cpu _cpu;
    mmu _mmu;
    ppu _ppu;

    uint8_t gb_screen_buffer[160 * 144] = {0};
};


#endif //BITTYBOI_GAMEBOY_H
