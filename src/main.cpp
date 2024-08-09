//
// Created by colin on 7/26/24.
//

#include <cstdio>
#include "gameboy.h"
#include <SDL.h>

int SCREEN_WIDTH = 160;
int SCREEN_HEIGHT = 144;

uint8_t gb_screen_buffer[160 * 144] = {0};

int main () {
    gameboy gb(gb_screen_buffer);
    gb.load_rom("tetris.gb");


    // test
//    uint8_t data0 = 0x7C;
//    uint8_t data1 = 0x56;
//    for (int i = 0; i < 8; i++) {
//        uint8_t mask = (uint8_t)1 << (7-i);
//        uint8_t x = ( (data1 & mask) >> (6-i) ) | ( (data0 & mask) >> (7-i) );
//        printf("bit %d\n", i);
//        printf("data0: %08b\n", data0);
//        printf("data1: %08b\n", data1);
//        printf("data1 bit: %08b\n", (data1 & mask) >> (6-i) );
//        printf("data0 bit: %08b\n", (data0 & mask) >> (7-i) );
//        printf("%02b \n", x);
//    }
//    return 1;

//    int q = 0;
//    while (true) {
//        int clocks = gb.tick();
//        q += clocks;
//        if (gb.tick() == -1)
//            break;
//        //printf("Clocks: %d\n", clocks);
//        //printf("\033[H\033[J");
//        if (q >= 17000) {
//            for (int y = 0; y < 144; y++) {
//                for (int x = 0; x < 160; x++) {
//                    printf("%d%d", gb_screen_buffer[x + 160*y], gb_screen_buffer[x + 160*y]);
//                    //printf("▒");
//                }
//                printf("\n");
//            }
//            q = 0;
//        }
//        fflush(stdout);
//        SDL_Delay(100);
//    }

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
    int32_t Running = 1;
    int32_t FullScreen = 0;

    float deltaTime = 0.0f;
    float lastFrame = (float)SDL_GetTicks()/1000.0f;

    bool last_state_is_vblank = false;


    int clocks;
    while ( (clocks = gb.tick()) != -1) {
        // only refresh screen once per vblank
        if (gb.is_vblank()) {
            if (!last_state_is_vblank) {
                SDL_SetRenderDrawColor(Renderer, 255, 255, 255, 255);
                SDL_RenderClear(Renderer);

                for (int y = 0; y < 144; y++) {
                    for (int x = 0; x < 160; x++) {
                        int shade = 255;
                        switch (gb_screen_buffer[x + 160*y]) {
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
                                //printf("shouldn't happen %d\n", gb_screen_buffer[x + 160*y]);
                                break;
                        }
                        SDL_SetRenderDrawColor(Renderer, shade, shade, shade, 255);
                        SDL_RenderDrawPoint(Renderer, x, y);
                    }
                }
                SDL_RenderPresent(Renderer);
                last_state_is_vblank = true;
            }
        } else {
            last_state_is_vblank = false;
        }
    }
    return 1;



    while (Running)
    {
        // update frame delta
        float currentFrame = (float)SDL_GetTicks()/1000.0f;
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // continuous key state
        const Uint8 * keystate = SDL_GetKeyboardState(NULL);
        const float cameraSpeed = 5.0f * deltaTime;
        SDL_PumpEvents();

        // SDL single events
        SDL_Event Event;
        while (SDL_PollEvent(&Event))
        {
            if (Event.type == SDL_KEYDOWN)
            {
                switch (Event.key.keysym.sym)
                {
                    case SDLK_ESCAPE:
                        Running = 0;
                        break;
                    case 'f':
                        FullScreen = !FullScreen;
                        if (FullScreen)
                        {
                            SDL_SetWindowFullscreen(Window, WindowFlags | SDL_WINDOW_FULLSCREEN_DESKTOP);
                        }
                        else
                        {
                            SDL_SetWindowFullscreen(Window, WindowFlags);
                        }
                        break;
                    default:
                        break;
                }
            }
            else if (Event.type == SDL_MOUSEBUTTONDOWN)
            {
                switch (Event.button.button)
                {
                    case SDL_BUTTON_RIGHT:
                        if (SDL_GetRelativeMouseMode() == SDL_TRUE)
                            SDL_SetRelativeMouseMode(SDL_FALSE);
                        else
                            SDL_SetRelativeMouseMode(SDL_TRUE);
                        break;
                    default:
                        break;
                }
            }
            else if (Event.type == SDL_MOUSEMOTION && SDL_GetRelativeMouseMode() == SDL_TRUE)
            {

            }
            else if (Event.type == SDL_MOUSEWHEEL && SDL_GetRelativeMouseMode() == SDL_TRUE)
            {

            }
            else if (Event.type == SDL_WINDOWEVENT)
            {
                if(Event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    SCREEN_WIDTH = Event.window.data1;
                    SCREEN_HEIGHT = Event.window.data2;
                }
            }
            else if (Event.type == SDL_QUIT)
            {
                Running = 0;
            }
        }

        // run gameboy

        int clocks;
        float start = (float)SDL_GetTicks();
        if ( (clocks = gb.tick()) == -1)
            return 0;

        // only refresh screen once per vblank
        if (gb.is_vblank()) {
            if (!last_state_is_vblank) {
                SDL_SetRenderDrawColor(Renderer, 255, 255, 255, 255);
                SDL_RenderClear(Renderer);

                for (int y = 0; y < 144; y++) {
                    for (int x = 0; x < 160; x++) {
                        int shade = 255;
                        switch (gb_screen_buffer[x + 160*y]) {
                            case 0:
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
                        }
                        SDL_SetRenderDrawColor(Renderer, shade, shade, shade, 255);
                        SDL_RenderDrawPoint(Renderer, x, y);
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