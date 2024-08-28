//
// Created by colin on 7/26/24.
//

#include <cstdio>
#include "ppu.h"

#define LCDC 0xFF40  // LCD control
#define SCY  0xFF42  // scroll Y
#define LY   0xFF44  // current scanline
#define BGP  0xFF47  // background palette - monochrome

void ppu::fillWindowPixels(mmu &mmu, uint8_t scan_line) {
    return;
}

void ppu::fillBackgroundPixels(mmu &mmu, uint8_t scan_line) {
    int pixels = 0;                                             // how many pixels we've processed
    uint8_t reads = 0;                                          // how many tiles (8-pixel groups) we've processed

    uint8_t lcdc = mmu.read(LCDC);

    uint8_t scan_line_scroll_y = scan_line + mmu.read(SCY);     // increment scanline by the Y-scroll offset
    uint8_t tile_line = scan_line_scroll_y % 8;                 // mod 8 to get the line of target tile
    uint16_t tile_row = (scan_line_scroll_y/8)*32;              //

    if ( !(lcdc & 0x1) ) {                                      // LCDC bit 0 - BG is disabled (B/W GB), write all white
        for (int i = 0; i < 160; i++)
            gb_screen_buffer[pixels + (scan_line*160)] = 0;
        return;
    }

    uint16_t bg_map_location = lcdc & 0x8 ? 0x98C0 : 0x9800;    // LCDC bit 3 - where BG map data is stored
    uint16_t bg_win_data_loc = lcdc & 0x10 ? 0x8000 : 0x9000;   // LCDC bit 4 - where BG/Window tile data is stored

    // read and store grayscale palette data
    uint8_t bg_palette = mmu.read(BGP);
    uint8_t bg_palette_vals[4];
    bg_palette_vals[0] = bg_palette & 0b00000011;
    bg_palette_vals[1] = (bg_palette & 0b00001100) >> 2;
    bg_palette_vals[2] = (bg_palette & 0b00110000) >> 4;
    bg_palette_vals[3] = (bg_palette & 0b11000000) >> 6;

    // keep grabbing pixels until we have got all 160
    while (pixels < 160) {
        uint16_t cur_addr = bg_map_location + tile_row + reads;
        uint16_t tile_nbr = mmu.read(cur_addr);

        // get the 2 bytes of tile data
        uint8_t data0, data1;
        if (bg_win_data_loc == 0x8000) {   // LCDC bit 4 is set, use '0x8000 addressing'
            data0 = mmu.read(bg_win_data_loc + (tile_nbr * 16) + (tile_line * 2)    );
            data1 = mmu.read(bg_win_data_loc + (tile_nbr * 16) + (tile_line * 2) + 1);
        } else {                            // LCDC bit 4 NOT set, use '0x9000 addressing'
            data0 = mmu.read(bg_win_data_loc + (int16_t)((tile_nbr * 16) + (tile_line * 2))    );
            data1 = mmu.read(bg_win_data_loc + (int16_t)((tile_nbr * 16) + (tile_line * 2)) + 1);
        }

//        printf("Addr: %04x Tile: %d D0_addr: %04x D1_addr: %04x\n",
//               cur_addr,
//               tile_row,
//               0x8000 + (tile_nbr * 16) + (tile_line * 2),
//               0x8000 + (tile_nbr * 16) + (tile_line * 2) + 1
//               );


        // do some funky bit manipulation to interleave the bits of both tile bytes
        // how does it work?
        // create 8 2-bit numbers where each bit of data1 is in the 2's spot and each bit of data0 is in the 1's spot
        // example for 4 bits, d1 = 0110 and d0 = 1100 -> 01, 11, 10, 00
        for (int i = 0; i < 8; i++) {
            uint8_t bitmask = (uint8_t)1 << (7-i);
//            if (pixels + (scan_line*160) >= 160*144) {
//                printf("pixels: %d scanline: %d\n", pixels, scan_line);
//            }
//            if ( (( (uint8_t)((data1 & bitmask) >> (7-i)) << 1 ) | ( (data0 & bitmask) >> (7-i) ) ) > 3) {
//                printf("%08b\n", (( (uint8_t)((data1 & bitmask) >> (7-i)) << 1 ) | ( (data0 & bitmask) >> (7-i) ) ));
//            }
            gb_screen_buffer[pixels + (scan_line*160)] = bg_palette_vals[
                    ( ((data1 & bitmask) >> (7-i)) << 1 ) |
                    ( (data0 & bitmask) >> (7-i) )
                    ];
//            if (gb_screen_buffer[pixels + (scan_line*160)] > 3) {
//                printf("AAAAAAAAAA\n");
//            }
            pixels++;
        }
        reads++;
    }
}

void ppu::fill_pixel_line(mmu &mmu, uint8_t scan_line) {
    fillBackgroundPixels(mmu, scan_line);
    fillWindowPixels(mmu, scan_line);
}

void ppu::tick(uint8_t clocks, mmu &mmu) {
    //return;
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
                        mmu.write(0xFF0F, mmu.read(0xFF0F) | 0x01);  // raise VBLANK interrupt flag
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
