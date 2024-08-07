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
    explicit gameboy(uint8_t * screen_buffer) : _ppu(screen_buffer) {
        gb_screen_buffer = screen_buffer;
    };

    void load_rom(const char * filename);
    int run();
    int tick();
    bool is_vblank();

private:
    uint8_t rom[ROMSIZE] = {0};
    cpu _cpu;
    mmu _mmu;
    ppu _ppu;
    uint8_t *gb_screen_buffer;
};


#endif //BITTYBOI_GAMEBOY_H
