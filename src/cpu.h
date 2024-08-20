//
// Created by colin on 7/26/24.
//

#ifndef BITTYBOICPP_CPU_H
#define BITTYBOICPP_CPU_H

#include <cstdint>
#include "ppu.h"
#include "mmu.h"

#define MEMSIZE 0xFFFF

class cpu{

public:
    cpu()=default;

    int tick(mmu &mmu, ppu &ppu);
    void init_no_bootrom();

private:
    uint8_t fetch_pc(mmu &mmu); // fetch next byte from program counter, increment program counter
    void prefix(mmu &mmu);

    uint16_t sp = 0x0;
    uint16_t pc = 0x0;

    /* registers */
    uint8_t acc = 0;
    uint8_t flag_z = 0;
    uint8_t flag_n = 0;
    uint8_t flag_h = 0;
    uint8_t flag_c = 0;
    uint8_t flag_ime = 0;
    uint8_t b = 0; uint8_t c = 0;
    uint8_t d = 0; uint8_t e = 0;
    uint8_t h = 0; uint8_t l = 0;

    uint8_t  pc_val = 0;
    uint8_t  scratch8 = 0;
    uint16_t scratch16 = 0;

    inline void LD_R8_R8(uint8_t &x, uint8_t &y);
    inline void LD_R8_U8(mmu &mmu, uint8_t *x);
    inline void LD_R8_AR16(mmu &mmu, uint8_t *x, uint8_t upper, uint8_t lower);
    inline void LD_R8_A16(mmu &mmu, uint8_t *x);
    static inline void LD_A_HLP(mmu &mmu, uint8_t &acc, uint8_t &h, uint8_t &l);
    static inline void LD_AR16_A(mmu &mmu, uint8_t &upper, uint8_t &lower, uint8_t &acc);
    inline void LD_A16_U8(mmu &mmu, uint16_t addr, uint8_t val);

    inline void POP_R16(mmu &mmu, uint8_t *upper, uint8_t *lower);
    inline void PUSH_R16(mmu &mmu, uint8_t upper, uint8_t lower);

    inline void INC_R8(uint8_t *x);
    inline void DEC_R8(uint8_t *x);
    static inline void DEC_R16(uint8_t *upper, uint8_t *lower);
    inline void ADD_R8_U8(uint8_t *x, uint8_t y);
    inline void ADC_R8_U8(uint8_t *x, uint8_t y);
    inline void ADD_R16_U16(uint8_t *dest_upper, uint8_t *dest_lower, uint8_t upper, uint8_t lower);

    inline void CPL(uint8_t *x);

    inline void OR_R8_U8(uint8_t *x, uint8_t y);
    inline void AND_R8_U8(uint8_t *x, uint8_t y);
    inline void XOR_R8_U8(uint8_t *x, uint8_t y);

    inline void RST(mmu &mmu, uint8_t x);

    inline void RET(mmu &mmu);

    inline void SUB_R8_U8(uint8_t *x, uint8_t y);


    inline void RR(uint8_t *x);
    inline void RR_addr(mmu &mmu, uint16_t addr);
    inline void RRC(uint8_t *x);
    inline void RRC_addr(mmu &mmu, uint16_t addr);
    inline void RL(uint8_t *x);
    inline void RL_addr(mmu &mmu, uint16_t addr);
    inline void RLC(uint8_t *x);
    inline void RLC_addr(mmu &mmu, uint16_t addr);

    inline void SLA(uint8_t *x);
    inline void SLA_addr(mmu &mmu, uint16_t addr);
    inline void SRA(uint8_t *x);
    inline void SRL(uint8_t *x);

    inline void SWAP(uint8_t *x);
    inline void CB_FUNC_ADDR(mmu &mmu, uint16_t addr, void(cpu::*cb_func)(uint8_t*));

    inline void BIT(uint8_t x, uint8_t bit);
    inline void BIT_addr(mmu &mmu, uint16_t addr, uint8_t bit);

    inline void RES(uint8_t *x, uint8_t bit);
    inline void RES_addr(mmu &mmu, uint16_t addr, uint8_t bit);

    inline void SET(uint8_t *x, uint8_t bit);
    inline void SET_addr(mmu &mmu, uint16_t addr, uint8_t bit);

    inline void CP(uint8_t x, uint8_t y);
};


#endif //BITTYBOICPP_CPU_H
