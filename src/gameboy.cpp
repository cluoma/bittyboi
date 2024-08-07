//
// Created by colin on 7/27/24.
//

#include <cstdio>
#include "gameboy.h"

void gameboy::load_rom(const char * filename) {
    /* load rom */
    FILE * rom_file = fopen(filename, "r");
    uint8_t next_byte = 0x0;
    uint32_t rom_size = 0;
    for (int i = 0;
         fread(&next_byte, 1, 1, rom_file) == 1 && i < 0xFFFF;
         i++)
    {
        rom[i] = next_byte;
        rom_size++;
    }

    /* load rom into ram */
    for (int i = 0; i < rom_size; i++)
    {
        _mmu.write(i, rom[i]);
    }
}

int gameboy::run() {
    do {} while(_cpu.tick(_mmu, _ppu) != -1);
    return 0;
}

int gameboy::tick() {
    return _cpu.tick(_mmu, _ppu);
}

bool gameboy::is_vblank() {
    if (_ppu.get_state() == VBLANK)
        return true;
    else
        return false;
}
