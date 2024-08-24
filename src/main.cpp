//
// Created by colin on 7/26/24.
//

#include <cstdio>
#include "gameboy.h"
#include <SDL.h>

int SCREEN_WIDTH = 320;
int SCREEN_HEIGHT = 288;


int main () {

    // Init SDL
    // code without checking for errors
    SDL_Init(SDL_INIT_VIDEO);
    // flags
    int32_t WindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
    // init SDL window
    SDL_Window *Window = SDL_CreateWindow(
            "bittyboi",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            SCREEN_WIDTH, SCREEN_HEIGHT,
            WindowFlags
    );
    SDL_SetWindowMouseGrab(Window, SDL_FALSE);
    SDL_SetRelativeMouseMode(SDL_FALSE);

    SDL_Renderer *Renderer = SDL_CreateRenderer(Window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_TARGETTEXTURE);
    if (Renderer == nullptr)
    {
        return -2;
    }
    SDL_SetRenderDrawBlendMode(Renderer,SDL_BLENDMODE_BLEND);



    // render loop
//    float deltaTime = 0.0f;
//    float lastFrame = (float) SDL_GetTicks() / 1000.0f;


    gameboy gb;
    gb.load_rom("tetris.gb");
//    gb.load_rom("cpu_instrs.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/01-special.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/02-interrupts.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/03-op sp,hl.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/04-op r,imm.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/05-op rp.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/06-ld r,r.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/07-jr,jp,call,ret,rst.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/08-misc instrs.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/09-op r,r.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/10-bit ops.gb");
//    gb.load_rom("/home/colin/Documents/gb-test-roms/cpu_instrs/individual/11-op a,(hl).gb");
//    gb.init_no_bootrom();

    bool last_state_is_vblank = false;
    uint64_t rt = 0;
    while (gb.tick() != -1
            //&& rt < 5000000
            ) {
        rt++;
        // only refresh screen once per vblank
        if (gb.is_vblank()) {
            if (!last_state_is_vblank) {
                SDL_SetRenderDrawColor(Renderer, 255, 255, 255, 255);
                SDL_RenderClear(Renderer);

                for (int y = 0; y < 144; y++) {
                    for (int x = 0; x < 160; x++) {
                        int shade = 255;
                        switch (gb.get_screen_buffer()[x + 160 * y]) {
                            case 0:
                                shade = 255;
                                break;
                            case 1:
                                shade = 175;
                                break;
                            case 2:
                                shade = 75;
                                break;
                            case 3:
                                shade = 0;
                                break;
                            default:
                                printf("found a weird number in the framebuffer\n");
                                break;
                        }
                        SDL_SetRenderDrawColor(Renderer, shade, shade, shade, 255);
                        int x1 = x * 2;
                        int y1 = y * 2;
                        SDL_RenderDrawPoint(Renderer, x1, y1);
                        SDL_RenderDrawPoint(Renderer, x1+1, y1);
                        SDL_RenderDrawPoint(Renderer, x1, y1+1);
                        SDL_RenderDrawPoint(Renderer, x1+1, y1+1);
                    }
                }
                SDL_RenderPresent(Renderer);
                last_state_is_vblank = true;
            }
        } else {
            last_state_is_vblank = false;
        }
    }
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}
//
//
//
//    while (Running)
//    {
//        // update frame delta
//        float currentFrame = (float)SDL_GetTicks()/1000.0f;
//        deltaTime = currentFrame - lastFrame;
//        lastFrame = currentFrame;
//
//        // continuous key state
//        const Uint8 * keystate = SDL_GetKeyboardState(NULL);
//        const float cameraSpeed = 5.0f * deltaTime;
//        SDL_PumpEvents();
//
//        // SDL single events
//        SDL_Event Event;
//        while (SDL_PollEvent(&Event))
//        {
//            if (Event.type == SDL_KEYDOWN)
//            {
//                switch (Event.key.keysym.sym)
//                {
//                    case SDLK_ESCAPE:
//                        Running = 0;
//                        break;
//                    case 'f':
//                        FullScreen = !FullScreen;
//                        if (FullScreen)
//                        {
//                            SDL_SetWindowFullscreen(Window, WindowFlags | SDL_WINDOW_FULLSCREEN_DESKTOP);
//                        }
//                        else
//                        {
//                            SDL_SetWindowFullscreen(Window, WindowFlags);
//                        }
//                        break;
//                    default:
//                        break;
//                }
//            }
//            else if (Event.type == SDL_MOUSEBUTTONDOWN)
//            {
//                switch (Event.button.button)
//                {
//                    case SDL_BUTTON_RIGHT:
//                        if (SDL_GetRelativeMouseMode() == SDL_TRUE)
//                            SDL_SetRelativeMouseMode(SDL_FALSE);
//                        else
//                            SDL_SetRelativeMouseMode(SDL_TRUE);
//                        break;
//                    default:
//                        break;
//                }
//            }
//            else if (Event.type == SDL_MOUSEMOTION && SDL_GetRelativeMouseMode() == SDL_TRUE)
//            {
//
//            }
//            else if (Event.type == SDL_MOUSEWHEEL && SDL_GetRelativeMouseMode() == SDL_TRUE)
//            {
//
//            }
//            else if (Event.type == SDL_WINDOWEVENT)
//            {
//                if(Event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
//                    SCREEN_WIDTH = Event.window.data1;
//                    SCREEN_HEIGHT = Event.window.data2;
//                }
//            }
//            else if (Event.type == SDL_QUIT)
//            {
//                Running = 0;
//            }
//        }
//
//        // run gameboy
//
//        int clocks;
//        float start = (float)SDL_GetTicks();
//        if ( (clocks = gb.tick()) == -1)
//            return 0;
//
//        // only refresh screen once per vblank
//        if (gb.is_vblank()) {
//            if (!last_state_is_vblank) {
//                SDL_SetRenderDrawColor(Renderer, 255, 255, 255, 255);
//                SDL_RenderClear(Renderer);
//
//                for (int y = 0; y < 144; y++) {
//                    for (int x = 0; x < 160; x++) {
//                        int shade = 255;
//                        switch (gb.get_screen_buffer()[x + 160*y]) {
//                            case 0:
//                                break;
//                            case 1:
//                                shade = 175;
//                                break;
//                            case 2:
//                                shade = 75;
//                                break;
//                            case 3:
//                                shade = 0;
//                                break;
//                        }
//                        SDL_SetRenderDrawColor(Renderer, shade, shade, shade, 255);
//                        SDL_RenderDrawPoint(Renderer, x, y);
//                    }
//                }
//                SDL_RenderPresent(Renderer);
//                last_state_is_vblank = true;
//            }
//        } else {
//            last_state_is_vblank = false;
//        }
//    }
//
//    SDL_DestroyWindow(Window);
//    SDL_Quit();
//
//    return 0;
//}