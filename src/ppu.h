//
// Created by colin on 7/26/24.
//

#ifndef BITTYBOICPP_PPU_H
#define BITTYBOICPP_PPU_H

#include <cstdint>
#include "mmu.h"

typedef enum {
    OAM_SEARCH = 0,
    PIXEL_TRANSFER,
    HBLANK,
    VBLANK
} ppu_state;

class ppu {
public:
    explicit ppu(uint8_t * screen_buffer) {
        gb_screen_buffer = screen_buffer;
    };

    void tick(uint8_t clocks, mmu &mmu);

    ppu_state get_state();
private:

    ppu_state state = OAM_SEARCH;
    uint32_t ticks = 0;
    uint16_t pixels_blitted = 0;
    uint8_t * gb_screen_buffer;

    void fill_pixel_line(mmu &mmu, uint8_t scan_line);
    void fillBackgroundPixels(mmu &mmu, uint8_t scan_line);

    void fillWindowPixels(mmu &mmu, uint8_t scan_line);
};


#endif //BITTYBOICPP_PPU_H
