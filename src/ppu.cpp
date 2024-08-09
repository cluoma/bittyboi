//
// Created by colin on 7/26/24.
//

#include <cstdio>
#include "ppu.h"

#define SCY 0xFF42  // scroll Y
#define LY  0xFF44  // current scanline
#define BGP 0xFF47  // background palette - monochrome

void ppu::fill_pixel_line(mmu &mmu, uint8_t scan_line) {
    uint8_t scan_line_scroll_y = scan_line + mmu.read(SCY);
    uint8_t tile_line = scan_line_scroll_y % 8;
    uint16_t tile_row = (scan_line_scroll_y/8)*32;
    int pixels = 0;
    uint8_t reads = 0;

    uint8_t bg_palette = mmu.read(BGP);
    uint8_t bg_palette_vals[4];
    bg_palette_vals[0] = bg_palette & 0b00000011;
    bg_palette_vals[1] = (bg_palette & 0b00001100) >> 2;
    bg_palette_vals[2] = (bg_palette & 0b00110000) >> 4;
    bg_palette_vals[3] = (bg_palette & 0b11000000) >> 6;

    // keep grabbing pixels until we have got all 160
    while (pixels < 160) {
        uint16_t cur_addr = 0x9800 + tile_row + reads;
        uint16_t tile_nbr = mmu.read(cur_addr);
        uint8_t data0 = mmu.read(0x8000 + (tile_nbr * 16) + (tile_line * 2)    );
        uint8_t data1 = mmu.read(0x8000 + (tile_nbr * 16) + (tile_line * 2) + 1);

//        printf("Addr: %04x Tile: %d D0_addr: %04x D1_addr: %04x\n",
//               cur_addr,
//               tile_row,
//               0x8000 + (tile_nbr * 16) + (tile_line * 2),
//               0x8000 + (tile_nbr * 16) + (tile_line * 2) + 1
//               );

        for (int i = 0; i < 8; i++) {
            uint8_t bitmask = (uint8_t)1 << (7-i);
            if (pixels + (scan_line*160) >= 160*144) {
                printf("pixels: %d scanline: %d\n", pixels, scan_line);
            }
//            if ( (( (uint8_t)((data1 & bitmask) >> (7-i)) << 1 ) | ( (data0 & bitmask) >> (7-i) ) ) > 3) {
//                printf("%08b\n", (( (uint8_t)((data1 & bitmask) >> (7-i)) << 1 ) | ( (data0 & bitmask) >> (7-i) ) ));
//            }
            gb_screen_buffer[pixels + (scan_line*160)] = bg_palette_vals[( (uint8_t)((data1 & bitmask) >> (6-i)) ) | ( (data0 & bitmask) >> (7-i) )];
            if (gb_screen_buffer[pixels + (scan_line*160)] > 3) {
                printf("AAAAAAAAAA\n");
            }
            pixels++;
        }
        reads++;
    }
}

void ppu::tick(uint8_t clocks, mmu &mmu) {
    //printf("Clocks: %d Ticks: %d\n", clocks, ticks);
    while (clocks-- > 0) {
        ticks++;
        switch (state) {
            // OAM memory starts at $FE00 to $FE9C inclusive
            // each entry is 4 bytes: position Y, position X, tile number, flags
            case OAM_SEARCH:
                //printf("OAM Ticks: %d\n", ticks);
                // TODO: collect sprite data here.
                if (ticks == 20) {
                    pixels_blitted = 0;
                    state = PIXEL_TRANSFER;
                }
            case PIXEL_TRANSFER:
                //printf("PIXEL Ticks: %d\n", ticks);
                // TODO: push pixel data to display.
                pixels_blitted += 4;
                // magic number: 160: number of pixels across the gameboy screen
                if (pixels_blitted == 160) {
                    fill_pixel_line(mmu, mmu.read(LY));
                    state = HBLANK;
                }
                break;
            case HBLANK:
                //printf("HBLANK Ticks: %d\n", ticks);
                // TODO: wait, then go back to sprite search for next line, or vblank.
                if (ticks == 114) {
                    ticks = 0;
                    mmu.write(LY, mmu.read(LY) + 1);
                    if (mmu.read(LY) == 144) {
                        state = VBLANK;
                    } else {
                        state = OAM_SEARCH;
                    }
                }
                break;
            case VBLANK:
                //printf("VBLANK Ticks: %d\n", ticks);
                // TODO: wait, then go back to sprite search for the top line.
                if (ticks == 114) {
                    ticks = 0;
                    mmu.write(LY, mmu.read(LY) + 1);
                    if (mmu.read(LY) == 154) {
                        mmu.write(LY, 0);
                        state = OAM_SEARCH;
                    }
                }
                break;
            default:
                break;
        }
    }
}

ppu_state ppu::get_state() {
    return state;
}
