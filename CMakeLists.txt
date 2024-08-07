cmake_minimum_required(VERSION 3.20)
project(bittyboi CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(bittyboi
        src/main.cpp
        src/cpu.h
        src/cpu.cpp
        src/ppu.h
        src/ppu.cpp
        src/gameboy.cpp
        src/gameboy.h
        src/mmu.cpp
        src/mmu.h)
target_link_libraries(bittyboi ${SDL2_LIBRARIES})
