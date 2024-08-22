//
// Created by colin on 7/26/24.
//

#include <cstdio>
#include <functional>
#include "cpu.h"

// these are the lowest clock counts for each instruction
// e.g. if branching is 3 or 5 clocks, depending on the branch, the number here is 3
uint8_t INSTR_CLOCKS_BASE[] = {
        1,4,2,2,1,1,2,1,5,2,2,2,1,1,2,1, //00
        1,3,2,2,1,1,2,1,3,2,2,2,1,1,2,1, //10
        2,3,2,2,1,1,2,1,2,2,2,2,1,1,2,1, //20
        2,3,2,2,3,3,3,1,2,2,2,2,1,1,2,1, //30

        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //40
        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //50
        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //60
        2,2,2,2,2,2,1,2,1,1,1,1,1,1,2,1, //70

        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //80
        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //90
        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //A0
        1,1,1,1,1,1,2,1,1,1,1,1,1,1,2,1, //B0

        2,3,3,4,3,4,2,4,2,4,3,1,3,6,2,4, //C0
        2,3,3,0,3,4,2,4,2,4,3,0,3,0,2,4, //D0
        3,3,2,0,0,4,2,4,4,1,4,0,0,0,2,4, //E0
        3,3,4,4,0,4,2,4,3,2,4,1,0,0,2,4  //F0
};
uint8_t INSTR_CLOCKS_BRANCH[] = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //00
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //10
        1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, //20
        1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0, //30

        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //40
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //50
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //60
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //70

        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //80
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //90
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //A0
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //B0

        3,0,1,0,3,0,0,0,3,0,1,0,3,0,0,0, //C0
        3,0,1,0,3,0,0,0,3,0,1,0,3,0,0,0, //D0
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, //E0
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0  //F0
};

#define HILO(HIGH, LOW) ((uint16_t)(LOW) | ((uint16_t)(HIGH) << 8))
#define LO(X) (uint8_t)((uint16_t)(X) & 0xFF)
#define HI(X) (uint8_t)((uint16_t)(X) >> 8)

// Use this before doing any operations to A
#define CHECK_HALF_CARRY(A, B) (uint8_t)( ( ( ( ((uint8_t)A) & 0xF ) + ( ((uint8_t)B) & 0xF ) ) & 0x10 ) >> 4 )
#define CHECK_HALF_CARRY_WITH_CARRY(A, B, C) (uint8_t)( ( ( (((uint8_t)A) & 0xF) + (((uint8_t)B) & 0xF) + C ) & 0x10 ) >> 4 )
#define CHECK_HALF_CARRY_DEC(A, B) (uint8_t)( ( ( ( ((uint8_t)A) & 0xF ) - ( ((uint8_t)(B)) & 0xF ) ) & 0x10 ) >> 4 )
#define CHECK_HALF_CARRY_DEC_WITH_CARRY(A, B, C) (uint8_t)( ( ( ( ((uint8_t)A) & 0xF ) - ( ((uint8_t)(B)) & 0xF ) - C ) & 0x10 ) >> 4 )
#define CHECK_CARRY(A, B) (uint8_t)( ( (((uint16_t)A) & 0xFF) + (((uint16_t)B) & 0xFF) ) >> 8)
#define CHECK_CARRY_WITH_CARRY(A, B, C) (uint8_t)( ( (((uint16_t)A) & 0xFF) + (((uint16_t)B) & 0xFF) + C ) >> 8)
#define CHECK_CARRY_DEC(A, B) A < B ? 1 : 0

#define BIT0(X) (((X) & 0b00000001))
#define BIT1(X) (((X) & 0b00000010) >> 1)
#define BIT2(X) (((X) & 0b00000100) >> 2)
#define BIT3(X) (((X) & 0b00001000) >> 3)
#define BIT4(X) (((X) & 0b00010000) >> 4)
#define BIT5(X) (((X) & 0b00100000) >> 5)
#define BIT6(X) (((X) & 0b01000000) >> 6)
#define BIT7(X) (((X) & 0b10000000) >> 7)

#define BIT0_COMPLEMENT(X) (((~X) & 0b00000001))
#define BIT1_COMPLEMENT(X) (((~X) & 0b00000010) >> 1)
#define BIT2_COMPLEMENT(X) (((~X) & 0b00000100) >> 2)
#define BIT3_COMPLEMENT(X) (((~X) & 0b00001000) >> 3)
#define BIT4_COMPLEMENT(X) (((~X) & 0b00010000) >> 4)
#define BIT5_COMPLEMENT(X) (((~X) & 0b00100000) >> 5)
#define BIT6_COMPLEMENT(X) (((~X) & 0b01000000) >> 6)
#define BIT7_COMPLEMENT(X) (((~X) & 0b10000000) >> 7)

#define SUBu8(X, Y) \
    flag_h = CHECK_HALF_CARRY_DEC(X, Y);\
    flag_c = CHECK_CARRY_DEC(X, Y);\
    X -= Y;\
    flag_z = (X) == 0 ? 1 : 0

#define NOT_IMPLEMENTED printf("%02x not implemented\n", pc_val); return(-1);


inline uint8_t cpu::fetch_pc(mmu &mmu) {
    return mmu.read(pc++);
}

/*
 * CPU OPERATIONS
 */

inline void cpu::LD_R8_R8(uint8_t &x, uint8_t &y) {
    /* Store one register into another */
    x = y;
}
inline void cpu::LD_R16_R16(uint8_t *x, uint8_t *y, uint8_t upper, uint8_t lower) {
    (*x) = upper;
    (*y) = lower;
}
inline void cpu::LD_R8_U8(mmu &mmu, uint8_t *x) {
    /* Load next byte into register */
    (*x) = fetch_pc(mmu);
}
inline void cpu::LD_R8_AR16(mmu &mmu, uint8_t *x, uint8_t upper, uint8_t lower) {
    /* Store the value pointed to by a register pair in a register */
    (*x) = mmu.read(HILO(upper, lower));
}
inline void cpu::LD_R8_A16(mmu &mmu, uint8_t *x) {
    /* Use the next two bytes as the address to load into the register */
    uint8_t lower = fetch_pc(mmu);
    uint8_t upper = fetch_pc(mmu);
    (*x) = mmu.read(HILO((upper), (lower)));
}
inline void cpu::LD_A16_U8(mmu &mmu, uint16_t addr, uint8_t val) {
    mmu.write(addr, val);
}
inline void cpu::LD_A_HLP(mmu &mmu, uint8_t &acc, uint8_t &h, uint8_t &l) {
    /* Store the value pointed to by register pair in A, then increment register pair */
    acc = mmu.read(HILO(h, l));
    uint16_t tmp = HILO(h, l) + 1;
    h = HI(tmp);
    l = LO(tmp);
}
inline void cpu::LD_A_HLM(mmu &mmu, uint8_t *x, uint8_t *upper, uint8_t *lower) {
    /* Store the value pointed to by register pair in A, then increment register pair */
    *x = mmu.read(HILO((*upper), (*lower)));
    uint16_t tmp = HILO((*upper), (*lower)) - 1;
    *upper = HI(tmp);
    *lower = LO(tmp);
}
inline void cpu::LD_AR16_A(mmu &mmu, uint8_t &upper, uint8_t &lower, uint8_t &acc) {
    /* Store A into the address pointed to by a register pair */
    mmu.write(HILO(upper, lower), acc);
}

inline void cpu::POP_R16(mmu &mmu, uint8_t *upper, uint8_t *lower) {
    (*lower) = mmu.read(sp++);
    (*upper) = mmu.read(sp++);
}
inline void cpu::PUSH_R16(mmu &mmu, uint8_t upper, uint8_t lower) {
    mmu.write(--sp, upper);
    mmu.write(--sp, lower);
}

inline void cpu::INC_R8(uint8_t *x) {
    flag_h = CHECK_HALF_CARRY((*x), 1);
    (*x)++;
    flag_n = 0;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::INC_A16(mmu &mmu, uint16_t addr) {
    uint8_t x = mmu.read(addr);
    flag_h = CHECK_HALF_CARRY(x, 1);
    x++;
    flag_n = 0;
    flag_z = x == 0 ? 1 : 0;
    mmu.write(addr, x);
}
inline void cpu::DEC_R8(uint8_t *x) {
    flag_h = CHECK_HALF_CARRY_DEC((*x), 1);
    (*x)--;
    flag_n = 1;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::DEC_R16(uint8_t *upper, uint8_t *lower) {
    uint16_t tmp = HILO((*upper), (*lower)) - 1;
    *upper = HI(tmp);
    *lower = LO(tmp);
}
inline void cpu::ADD_R8_U8(uint8_t *x, uint8_t y) {
    flag_c = CHECK_CARRY((*x), y);
    flag_h = CHECK_HALF_CARRY((*x), y);
    flag_n = 0;
    (*x) += y;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::ADC_R8_U8(uint8_t *x, uint8_t y) {
    uint8_t new_val = (*x) + y + flag_c;
    // set carry flags
    flag_h = CHECK_HALF_CARRY_WITH_CARRY((*x), y, flag_c);
    flag_c = CHECK_CARRY_WITH_CARRY((*x), y, flag_c);
    flag_n = 0;
    // set new value
    (*x) = new_val;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::ADD_R16_U16(uint8_t *dest_upper, uint8_t *dest_lower, uint8_t upper, uint8_t lower) {
    /* Add two 8-bit register pairs as 16-bit registers */
    flag_n = 0;
    uint16_t tmp_left  = HILO((*dest_upper), (*dest_lower));
    uint16_t tmp_right = HILO(upper, lower);
    flag_h = (((tmp_left & 0xFFF) + (tmp_right & 0xFFF)) & 0x1000) >> 12;  // half-carry is from the half of upper 8-bit
    flag_c = (uint8_t)( ( (((uint32_t)tmp_left) & 0xFFFF) + (((uint32_t)tmp_right) & 0xFFFF) ) >> 16);
    tmp_left += tmp_right;
    (*dest_upper) = HI(tmp_left);
    (*dest_lower) = LO(tmp_left);
}
inline void cpu::SUB_R8_U8(uint8_t *x, uint8_t y) {
    flag_c = CHECK_CARRY_DEC((*x), y);
    flag_h = CHECK_HALF_CARRY_DEC((*x), y);
    flag_n = 1;
    (*x) -= y;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::SBC_R8_U8(uint8_t *x, uint8_t y) {
    uint8_t new_val = (*x) - y - flag_c;
    // set carry flags
    flag_h = CHECK_HALF_CARRY_DEC_WITH_CARRY((*x), y, flag_c);
    flag_c = CHECK_CARRY_DEC((*x), (y+flag_c));
    flag_n = 1;
    // set new value
    (*x) = new_val;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::OR_R8_U8(uint8_t *x, uint8_t y) {
    /* Logical OR on x and y, store contents in x */
    (*x) = (*x) | y;
    flag_h = 0;
    flag_c = 0;
    flag_n = 0;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::XOR_R8_U8(uint8_t *x, uint8_t y) {
    /* Logical XOR on x and y, store contents in x */
    (*x) = (*x) ^ y;
    flag_c = 0; flag_h = 0; flag_n = 0;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::CPL(uint8_t *x) {
    (*x) = (*x) ^ 0xFF;
    flag_h = 1;
    flag_n = 1;
}
inline void cpu::AND_R8_U8(uint8_t *x, uint8_t y) {
    (*x) = (*x) & y;
    flag_c = 0;
    flag_h = 1;
    flag_n = 0;
    flag_z = (*x) == 0 ? 1 : 0;
}

inline void cpu::RST(mmu &mmu, uint8_t x) {
    mmu.write(--sp, HI(pc));
    mmu.write(--sp, LO(pc));
    pc = x;
}

inline void cpu::RET(mmu &mmu) {
    uint8_t lower = mmu.read(sp++);
    uint8_t upper = mmu.read(sp++);
    pc = HILO(upper, lower);
}

inline void cpu::RR(uint8_t *x) {
    if (*x & 0x1) {
        *x = ((*x) >> 1) | ((flag_c  << 7) & 0b10000000);
        flag_c = 1;
    } else {
        *x = ((*x) >> 1) | ((flag_c  << 7) & 0b10000000);
        flag_c = 0;
    }
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}
inline void cpu::RR_addr(mmu &mmu, uint16_t addr) {
    uint8_t x = mmu.read(addr);
    RR(&x);
    mmu.write(addr, x);
}
inline void cpu::RRC(uint8_t *x) {
    if (*x & 0x1) {
        *x = ((*x) >> 1) | 0b10000000;
        flag_c = 1;
    } else {
        *x = ((*x) >> 1);
        flag_c = 0;
    }
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}
inline void cpu::RRC_addr(mmu &mmu, uint16_t addr) {
    uint8_t x = mmu.read(addr);
    RRC(&x);
    mmu.write(addr, x);
}
inline void cpu::RL(uint8_t *x) {
    if (*x & 0b10000000) {
        *x = ((*x) << 1) | (flag_c & 0x1);
        flag_c = 1;
    } else {
        *x = ((*x) << 1) | (flag_c & 0x1);
        flag_c = 0;
    }
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}
inline void cpu::RL_addr(mmu &mmu, uint16_t addr) {
    uint8_t x = mmu.read(addr);
    RL(&x);
    mmu.write(addr, x);
}
inline void cpu::RLC(uint8_t *x) {
    if (*x & 0b10000000) {
        *x = ((*x) << 1) | 0x1;
        flag_c = 1;
    } else {
        *x = ((*x) << 1);
        flag_c = 0;
    }
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}
inline void cpu::RLC_addr(mmu &mmu, uint16_t addr) {
    uint8_t x = mmu.read(addr);
    RLC(&x);
    mmu.write(addr, x);
}
inline void cpu::SLA(uint8_t *x) {
    // bit 7 into carry
    flag_c = *x & 0b10000000 ? 1 : 0;
    // bit 0 is reset to 0
    (*x) <<= 1;
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}
inline void cpu::SLA_addr(mmu &mmu, uint16_t addr) {
    uint8_t x = mmu.read(addr);
    SLA(&x);
    mmu.write(addr, x);
}
inline void cpu::SRA(uint8_t *x) {
    // bit 0 into carry
    flag_c = *x & 0x1 ? 1 : 0;
    (*x) >>= 1;
    // bit 7 should be unchanged
    if (*x & 0b01000000) {
        *x = (*x) | 0b10000000;
    }
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}
inline void cpu::SRL(uint8_t * x) {
    // bit 0 into carry
    flag_c = *x & 0x1 ? 1 : 0;
    (*x) >>= 1;
    flag_z = (*x) == 0 ? 1 : 0;
    flag_h = 0;
    flag_n = 0;
}

inline void cpu::SWAP(uint8_t *x) {
    (*x) = ((*x) >> 4) | (((*x) & 0x0F) << 4);
    flag_z = (*x) == 0 ? 1 : 0;
    flag_c = 0;
    flag_h = 0;
    flag_n = 0;
}

inline void cpu::CB_FUNC_ADDR(mmu &mmu, uint16_t addr, void(cpu::*cb_func)(uint8_t*)) {
    uint8_t x = mmu.read(addr);
    (this->*cb_func)(&x);
    mmu.write(addr, x);
}

inline void cpu::BIT(uint8_t x, uint8_t bit) {
    // bit should be between 0 and 7
    flag_z = ((~x) >> bit) & 0x1 ;
    flag_h = 1;
    flag_n = 0;
}
inline void cpu::BIT_addr(mmu &mmu, uint16_t addr, uint8_t bit) {
    // bit should be between 0 and 7
    uint8_t x = mmu.read(addr);
    BIT(x, bit);
    mmu.write(addr, x);
}

inline void cpu::RES(uint8_t *x, uint8_t bit) {
    // bit should be between 0 and 7
    (*x) = (*x) & ~((uint8_t)0x1 << bit );
}
inline void cpu::RES_addr(mmu &mmu, uint16_t addr, uint8_t bit) {
    // bit should be between 0 and 7
    uint8_t x = mmu.read(addr);
    RES(&x, bit);
    mmu.write(addr, x);
}
inline void cpu::SET(uint8_t *x, uint8_t bit) {
    // bit should be between 0 and 7
    (*x) = (*x) | (0x1 << bit);
}
inline void cpu::SET_addr(mmu &mmu, uint16_t addr, uint8_t bit) {
    // bit should be between 0 and 7
    uint8_t x = mmu.read(addr);
    SET(&x, bit);
    mmu.write(addr, x);
}

inline void cpu::CP(uint8_t x, uint8_t y) {
    // compare x and y by calculating x - y
    flag_c = CHECK_CARRY_DEC(x, y);
    flag_h = CHECK_HALF_CARRY_DEC(x, y);
    flag_n = 1;
    flag_z = x == y ? 1 : 0;
}

inline void cpu::DAA(uint8_t *x) {
    if(flag_n){
        if(flag_c)
            (*x) -= 0x60;
        if(flag_h)
            (*x) -= 0x6;
    }
    else{
        if(flag_c || (*x) > 0x99){
            (*x) += 0x60;
            flag_c = 1;
        }
        if(flag_h || ((*x) & 0xF) > 0x9)
            (*x) += 0x6;
    }
    flag_h = 0;
    flag_z = (*x) == 0 ? 1 : 0;
}

int cpu::tick(mmu &mmu, ppu &ppu) {
    uint8_t f = (flag_z << 7) | (flag_n << 6) | (flag_h << 5) | (flag_c << 4);
    printf("A:%02x F:%02x B:%02x C:%02x D:%02x E:%02x H:%02x L:%02x SP:%04x PC:%04x PCMEM:%02x,%02x,%02x,%02x\n",
           acc, f, b, c, d, e, h, l, sp, pc,
           mmu.read(pc), mmu.read(pc+1), mmu.read(pc+2), mmu.read(pc+3));

    pc_val = fetch_pc(mmu);

    int clocks = 0;
    clocks += INSTR_CLOCKS_BASE[pc_val];

    switch (pc_val) {
        case 0x00:
            /* NOP */
            break;
        case 0x01:
            /* LD BC,u16 */
            c = fetch_pc(mmu);
            b = fetch_pc(mmu);
            break;
        case 0x02:
            /* LD (BC),A */
            mmu.write(HILO(b, c), acc);
            break;
        case 0x03:
            /* INC BC */
            scratch16 = HILO(b, c) + 1;
            b = HI(scratch16);
            c = LO(scratch16);
            break;
        case 0x04:
            /* INC B */
            INC_R8(&b);
            break;
        case 0x05:
            /* DEC B */
            DEC_R8(&b);
            break;
        case 0x06:
            /* LD B,u8 */
            LD_R8_U8(mmu, &b);
            break;
        case 0x07:
            /* RLCA */
            RLC(&acc);
            // Special case for RLCA
            flag_z = 0;
            break;
        case 0x08:
            /* LD (u16),SP */
            scratch8 = fetch_pc(mmu);
            scratch16 = HILO(fetch_pc(mmu), scratch8);
            mmu.write(scratch16, LO(sp));
            mmu.write(scratch16+1, HI(sp));
            break;
        case 0x09:
            /* ADD HL,BC */
        case 0x0A:
            /* LD A,(BC) */
            LD_R8_AR16(mmu, &acc, b, c); break;
        case 0x0B:
            /* DEC BC */
            DEC_R16(&b, &c); break;
        case 0x0C:
            /* INC C */
            INC_R8(&c); break;
        case 0x0D:
            /* DEC C */
            DEC_R8(&c); break;
        case 0x0E:
            /* LD C,u8 */
            LD_R8_U8(mmu, &c);
            break;
        case 0x0F:
            /* RRCA */
            RRC(&acc);
            flag_z = 0; // Special case for RRCA
            break;
        case 0x10:
            return -1;
        case 0x11:
            /* LD DE,u16 */
            e = fetch_pc(mmu);
            d = fetch_pc(mmu);
            break;
        case 0x12:
            /* LD (DE),A */
            LD_A16_U8(mmu, HILO(d, e), acc); break;
        case 0x13:
            /* INC DE */
            scratch16 = HILO(d, e);
            d = HI(scratch16+1);
            e = LO(scratch16+1);
            break;
        case 0x14:
            /* INC D */
            INC_R8(&d); break;
        case 0x15:
            /* DEC D */
            DEC_R8(&d); break;
        case 0x16:
            /* LD D,u8 */
            LD_R8_U8(mmu, &d); break;
        case 0x17:
            /* RLA */
            RL(&acc);
            // Special case for RLA
            flag_z = 0;
            break;
        case 0x18:
            /* JR i8 */
            pc += (int8_t) fetch_pc(mmu); break;
        case 0x19:
            /* ADD HL,DE */
            ADD_R16_U16(&h, &l, d, e); break;
        case 0x1A:
            /* LD A,(DE) */
            LD_R8_AR16(mmu, &acc, d, e); break;
        case 0x1B:
            /* DEC DE */
            DEC_R16(&d, &e); break;
        case 0x1C:
            /* INC E */
            INC_R8(&e); break;
        case 0x1D:
            /* DEC E */
            DEC_R8(&e); break;
        case 0x1E:
            /* LD E,u8 */
            LD_R8_U8(mmu, &e); break;
        case 0x1F:
            /* RRA */
            RR(&acc);
            flag_z = 0; // special case for RRA opcode, it unsets zero flag
            break;
        case 0x20:
            /* JR NZ, s8 */
            if (flag_z == 0) {
                pc += (int8_t) fetch_pc(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            else
                pc++;
            break;
        case 0x21:
            /* LD HL,u16 */
            l = fetch_pc(mmu);
            h = fetch_pc(mmu);
            break;
        case 0x22:
            /* LD (HL+), A */
            scratch16 = HILO(h, l);
            mmu.write(scratch16, acc);
            h = HI(scratch16+1);
            l = LO(scratch16+1);
            break;
        case 0x23:
            /* INC HL */
            scratch16 = HILO(h, l);
            h = HI(scratch16+1);
            l = LO(scratch16+1);
            break;
        case 0x24:
            /* INC H */
            INC_R8(&h); break;
        case 0x25:
            /* DEC H */
            DEC_R8(&h); break;
        case 0x26:
            /* LD H,u8 */
            LD_R8_U8(mmu, &h); break;
        case 0x27:
            /* DAA */
            DAA(&acc); break;
        case 0x28:
            /* JR Z, s8 */
            if (flag_z == 1) {
                pc += (int8_t) fetch_pc(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            else
                pc++;
            break;
        case 0x29:
            /* ADD HL,HL */
            ADD_R16_U16(&h, &l, h, l); break;
        case 0x2A:
            /* LD A,(HL+) */
            LD_A_HLP(mmu, acc, h, l);
            break;
        case 0x2B:
            /* DEC HL */
            DEC_R16(&h, &l); break;
        case 0x2C:
            /* INC L */
            INC_R8(&l);
            break;
        case 0x2D:
            /* DEC L */
            DEC_R8(&l);
            break;
        case 0x2E:
            /* LD L,u8 */
            LD_R8_U8(mmu, &l);
            break;
        case 0x2F:
            /* CPL */
            CPL(&acc);
            break;
        case 0x30:
            /* JR NC,i8 */
            if (flag_c == 0) {
                pc += (int8_t) fetch_pc(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc++;
            }
            break;
        case 0x31:
            /* LD SP,u16 */
            sp = HILO(mmu.read(pc + 1), mmu.read(pc));
            pc += 2;
            break;
        case 0x32:
            /* LD (HL-),A */
            scratch16 = HILO(h, l);
            mmu.write(scratch16, acc);
            l = LO(scratch16 - 1);
            h = HI(scratch16 - 1);
            break;
        case 0x33:
            /* INC SP */
            sp += 1; break;
        case 0x34:
            /* INC (HL) */
            INC_A16(mmu, HILO(h, l)); break;
        case 0x35:
            /* DEC (HL) */
            scratch8 = mmu.read(HILO(h, l));
            DEC_R8(&scratch8);
            mmu.write(HILO(h, l), scratch8);
            break;
        case 0x36:
            /* LD (HL),u8 */
            mmu.write(HILO(h, l), fetch_pc(mmu));
            break;
        case 0x37:
            /* SCF - set carry flag */
            flag_c = 1;
            flag_n = 0;
            flag_h = 0;
            break;
        case 0x38:
            /* JR C,i8 */
            if (flag_c == 1) {
                pc += (int8_t) fetch_pc(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 1;
            }
            break;
        case 0x39:
            /* ADD HL,SP */
            ADD_R16_U16(&h, &l, HI(sp), LO(sp)); break;
        case 0x3A:
            /* LD A,(HL-) */
            LD_A_HLM(mmu, &acc, &h, &l); break;
        case 0x3B:
            /* DEC SP */
            sp -= 1; break;
        case 0x3C:
            /* INC A */
            INC_R8(&acc); break;
        case 0x3D:
            /* DEC A */
            DEC_R8(&acc); break;
        case 0x3E:
            /* LD A,u8 */
            LD_R8_U8(mmu, &acc); break;
        case 0x3F:
            /* CCF - flip the carry flag*/
            flag_c = flag_c == 0 ? 1 : 0;
            flag_n = 0;
            flag_h = 0;
            break;
        case 0x40:
            /* LD B,B */
            LD_R8_R8(b, b); break;
        case 0x41:
            /* LD B,C */
            LD_R8_R8(b, c); break;
        case 0x42:
            /* LD B,D */
            LD_R8_R8(b, d); break;
        case 0x43:
            /* LD B,E */
            LD_R8_R8(b, e); break;
        case 0x44:
            /* LD B,H */
            LD_R8_R8(b, h); break;
        case 0x45:
            /* LD B,L */
            LD_R8_R8(b, l); break;
        case 0x46:
            /* LD B,(HL) */
            LD_R8_AR16(mmu, &b, h, l); break;
        case 0x47:
            /* LD B,A */
            LD_R8_R8(b, acc); break;
        case 0x48:
            /* LD C,B */
            LD_R8_R8(c, b); break;
        case 0x49:
            /* LD C,C */
            LD_R8_R8(c, c); break;
        case 0x4A:
            /* LD C,D */
            LD_R8_R8(c, d); break;
        case 0x4B:
            /* LD C,E */
            LD_R8_R8(c, e); break;
        case 0x4C:
            /* LD C,H */
            LD_R8_R8(c, h); break;
        case 0x4D:
            /* LD C,L */
            LD_R8_R8(c, l); break;
        case 0x4E:
            /* LD C,(HL) */
            LD_R8_AR16(mmu, &c, h, l); break;
        case 0x4F:
            /* LD C,A */
            LD_R8_R8(c, acc); break;
        case 0x50:
            /* LD D,B */
            LD_R8_R8(d, b); break;
        case 0x51:
            /* LD D,C */
            LD_R8_R8(d, c); break;
        case 0x52:
            /* LD D,D */
            LD_R8_R8(d, d); break;
        case 0x53:
            /* LD D,E */
            LD_R8_R8(d, e); break;
        case 0x54:
            /* LD D,H */
            LD_R8_R8(d, h); break;
        case 0x55:
            /* LD D,L */
            LD_R8_R8(d, l); break;
        case 0x56:
            /* LD D,(HL) */
            LD_R8_AR16(mmu, &d, h, l); break;
        case 0x57:
            /* LD D,A */
            LD_R8_R8(d, acc); break;
        case 0x58:
            /* LD E,B */
            LD_R8_R8(e, b); break;
        case 0x59:
            /* LD E,C */
            LD_R8_R8(e, c); break;
        case 0x5A:
            /* LD E,D */
            LD_R8_R8(e, d); break;
        case 0x5B:
            /* LD E,E */
            LD_R8_R8(e, e); break;
        case 0x5C:
            /* LD E,H */
            LD_R8_R8(e, h); break;
        case 0x5D:
            /* LD E,L */
            LD_R8_R8(e, l); break;
        case 0x5E:
            /* LD E,(HL) */
            LD_R8_AR16(mmu, &e, h, l); break;
        case 0x5F:
            /* LD E,A */
            LD_R8_R8(e, acc); break;
        case 0x60:
            /* LD H,B */
            LD_R8_R8(h, b); break;
        case 0x61:
            /* LD H,C */
            LD_R8_R8(h, c); break;
        case 0x62:
            /* LD H,D */
            LD_R8_R8(h, d); break;
        case 0x63:
            /* LD H,E */
            LD_R8_R8(h, e); break;
        case 0x64:
            /* LD H,H */
            LD_R8_R8(h, h); break;
        case 0x65:
            /* LD H,L */
            LD_R8_R8(h, l); break;
        case 0x66:
            /* LD H,(HL) */
            LD_R8_AR16(mmu, &h, h, l); break;
        case 0x67:
            /* LD H,A */
            LD_R8_R8(h, acc); break;
        case 0x68:
            /* LD L,B */
            LD_R8_R8(l, b); break;
        case 0x69:
            /* LD L,C */
            LD_R8_R8(l, c); break;
        case 0x6A:
            /* LD L,D */
            LD_R8_R8(l , d); break;
        case 0x6B:
            /* LD L,E */
            LD_R8_R8(l, e); break;
        case 0x6C:
            /* LD L,H */
            LD_R8_R8(l, h); break;
        case 0x6D:
            /* LD L,L */
            LD_R8_R8(l, l); break;
        case 0x6E:
            /* LD L,(HL) */
            LD_R8_AR16(mmu, &l, h, l); break;
        case 0x6F:
            /* LD L,A */
            LD_R8_R8(l, acc); break;
        case 0x70:
            /* LD (HL),B */
            LD_A16_U8(mmu, HILO(h, l), b); break;
        case 0x71:
            /* LD (HL),C */
            LD_A16_U8(mmu, HILO(h, l), c); break;
        case 0x72:
            /* LD (HL),D */
            LD_A16_U8(mmu, HILO(h, l), d); break;
        case 0x73:
            /* LD (HL),E */
            LD_A16_U8(mmu, HILO(h, l), e); break;
        case 0x74:
            /* LD (HL),H */
            LD_A16_U8(mmu, HILO(h, l), h); break;
        case 0x75:
            /* LD (HL),L */
            LD_A16_U8(mmu, HILO(h, l), l); break;
        case 0x76:
            /* HALT */
            return -1;
        case 0x77:
            /* LD (HL),A */
            LD_AR16_A(mmu, h, l, acc); break;
        case 0x78:
            /* LD A,B */
            LD_R8_R8(acc, b); break;
        case 0x79:
            /* LD A,C */
            LD_R8_R8(acc, c); break;
        case 0x7A:
            /* LD A,D */
            LD_R8_R8(acc, d); break;
        case 0x7B:
            /* LD A, E */
            LD_R8_R8(acc, e); break;
        case 0x7C:
            /* LD A,H */
            LD_R8_R8(acc, h); break;
        case 0x7D:
            /* LD A,L */
            LD_R8_R8(acc, l); break;
        case 0x7E:
            /* LD A,(HL) */
            LD_R8_AR16(mmu, &acc, h, l); break;
        case 0x7F:
            /* LD A,A */
            LD_R8_R8(acc, acc); break;
        case 0x80:
            /* ADD A,B */
            ADD_R8_U8(&acc, b); break;
        case 0x81:
            /* ADD A,C */
            ADD_R8_U8(&acc, c); break;
        case 0x82:
            /* ADD A,D */
            ADD_R8_U8(&acc, d); break;
        case 0x83:
            /* ADD A,E */
            ADD_R8_U8(&acc, e); break;
        case 0x84:
            /* ADD A,H */
            ADD_R8_U8(&acc, h); break;
        case 0x85:
            /* ADD A,L */
            ADD_R8_U8(&acc, l); break;
        case 0x86:
            /* ADD A,(HL) */
            ADD_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0x87:
            /* ADD A,A */
            ADD_R8_U8(&acc, acc); break;
        case 0x88:
            /* ADC A,B */
            ADC_R8_U8(&acc, b); break;
        case 0x89:
            /* ADC A,C */
            ADC_R8_U8(&acc, c); break;
        case 0x8A:
            /* ADC A,D */
            ADC_R8_U8(&acc, d); break;
        case 0x8B:
            /* ADC A,E */
            ADC_R8_U8(&acc, e); break;
        case 0x8C:
            /* ADC A,H */
            ADC_R8_U8(&acc, h); break;
        case 0x8D:
            /* ADC A,L */
            ADC_R8_U8(&acc, l); break;
        case 0x8E:
            /* ADC A,(HL) */
            ADC_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0x8F:
            /* ADC A,A */
            ADC_R8_U8(&acc, acc); break;
        case 0x90:
            /* SUB A,B */
            SUB_R8_U8(&acc, b); break;
        case 0x91:
            /* SUB A,C */
            SUB_R8_U8(&acc, c); break;
        case 0x92:
            /* SUB A,D */
            SUB_R8_U8(&acc, d); break;
        case 0x93:
            /* SUB A,E */
            SUB_R8_U8(&acc, e); break;
        case 0x94:
            /* SUB A,H */
            SUB_R8_U8(&acc, h); break;
        case 0x95:
            /* SUB A,L */
            SUB_R8_U8(&acc, l); break;
        case 0x96:
            /* SUB A,(HL) */
            SUB_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0x97:
            /* SUB A,A */
            SUB_R8_U8(&acc, acc); break;
        case 0x98:
            /* SBC A,B - subtract with carry flag */
            SBC_R8_U8(&acc, b); break;
        case 0x99:
            /* SBC A,C */
            SBC_R8_U8(&acc, c); break;
        case 0x9A:
            /* SBC A,D */
            SBC_R8_U8(&acc, d); break;
        case 0x9B:
            /* SBC A,E */
            SBC_R8_U8(&acc, e); break;
        case 0x9C:
            /* SBC A,H */
            SBC_R8_U8(&acc, h); break;
        case 0x9D:
            /* SBC A,L */
            SBC_R8_U8(&acc, l); break;
        case 0x9E:
            /* SBC A,(HL) */
            SBC_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0x9F:
            /* SBC A,A */
            SBC_R8_U8(&acc, acc); break;
        case 0xA0:
            /* AND A,B */
            AND_R8_U8(&acc, b); break;
        case 0xA1:
            /* AND A,C */
            AND_R8_U8(&acc, c); break;
        case 0xA2:
            /* AND A,D */
            AND_R8_U8(&acc, d); break;
        case 0xA3:
            /* AND A,E */
            AND_R8_U8(&acc, e); break;
        case 0xA4:
            /* AND A,H */
            AND_R8_U8(&acc, h); break;
        case 0xA5:
            /* AND A,L */
            AND_R8_U8(&acc, l); break;
        case 0xA6:
            /* AND A,(HL) */
            AND_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0xA7:
            /* AND A,A */
            AND_R8_U8(&acc, acc); break;
        case 0xA8:
            /* XOR A,B */
            XOR_R8_U8(&acc, b); break;
        case 0xA9:
            /* XOR A,C */
            XOR_R8_U8(&acc, c); break;
        case 0xAA:
            /* XOR A,D */
            XOR_R8_U8(&acc, d); break;
        case 0xAB:
            /* XOR A,E */
            XOR_R8_U8(&acc, e); break;
        case 0xAC:
            /* XOR A,H */
            XOR_R8_U8(&acc, h); break;
        case 0xAD:
            /* XOR A,L */
            XOR_R8_U8(&acc, l); break;
        case 0xAE:
            /* XOR A,(HL) */
            XOR_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0xAF:
            /* XOR A,A */
            XOR_R8_U8(&acc, acc); break;
        case 0xB0:
            /* OR A,B */
            OR_R8_U8(&acc, b); break;
        case 0xB1:
            /* OR A,C */
            OR_R8_U8(&acc, c); break;
        case 0xB2:
            /* OR A,D */
            OR_R8_U8(&acc, d); break;
        case 0xB3:
            /* OR A,E */
            OR_R8_U8(&acc, e); break;
        case 0xB4:
            /* OR A,H */
            OR_R8_U8(&acc, h); break;
        case 0xB5:
            /* OR A,L */
            OR_R8_U8(&acc, l); break;
        case 0xB6:
            /* OR A,(HL) */
            OR_R8_U8(&acc, mmu.read(HILO(h, l))); break;
        case 0xB7:
            /* OR A,A */
            OR_R8_U8(&acc, acc); break;
        case 0xB8:
            /* CP A,B */
            CP(acc, b); break;
        case 0xB9:
            /* CP A,C */
            CP(acc, c); break;
        case 0xBA:
            /* CP A,D */
            CP(acc, d); break;
        case 0xBB:
            /* CP A,E */
            CP(acc, e); break;
        case 0xBC:
            /* CP A,H */
            CP(acc, h); break;
        case 0xBD:
            /* CP A,L */
            CP(acc, l); break;
        case 0xBE:
            /* CP A,(HL) */
            CP(acc, mmu.read(HILO(h, l))); break;
        case 0xBF:
            /* CP A,A */
            CP(acc, acc); break;
        case 0xC0:
            /* RET NZ */
            if (flag_z == 0) {
                RET(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            break;
        case 0xC1:
            /* POP BC */
            POP_R16(mmu, &b, &c); break;
        case 0xC2:
            /* JP NZ,u16 */
            if (flag_z == 0) {
                scratch8 = fetch_pc(mmu);
                pc = HILO(fetch_pc(mmu), scratch8);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xC3:
            /* JP u16 */
            scratch8 = fetch_pc(mmu);
            pc = HILO(fetch_pc(mmu), scratch8);
            break;
        case 0xC4:
            /* CALL NZ,u16 */
            if (flag_z == 0) {
                PUSH_R16(mmu, HI(pc+2), LO(pc+2));
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xC5:
            /* PUSH BC */
            PUSH_R16(mmu, b, c);
            break;
        case 0xC6:
            /* ADD A,u8 */
            ADD_R8_U8(&acc, fetch_pc(mmu));
            break;
        case 0xC7:
            /* RST 00h */
            RST(mmu, 0x00); break;
        case 0xC8:
            /* RET Z */
            if (flag_z == 1) {
                RET(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            break;
        case 0xC9:
            /* RET */
            RET(mmu); break;
        case 0xCA:
            /* JP Z,u16 */
            scratch8 = fetch_pc(mmu);
            scratch16 = HILO(fetch_pc(mmu), scratch8);
            if (flag_z == 1) {
                pc = scratch16;
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            break;
        case 0xCB:
            prefix(mmu);
            clocks += 2;
            break;
        case 0xCC:
            /* CALL Z,u16 */
            if (flag_z == 1) {
                PUSH_R16(mmu, HI(pc+2), LO(pc+2));
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xCD:
            /* CALL u16 */
            // push pc of next instruction to sp
            mmu.write(--sp, HI(pc + 2));
            mmu.write(--sp, LO(pc + 2));
            // set pc to call routine
            pc = HILO(mmu.read(pc+1), mmu.read(pc));
            break;
        case 0xCE:
            /* ADC A,u8 */
            ADC_R8_U8(&acc, fetch_pc(mmu)); break;
        case 0xCF:
            /* RST 08h */
            RST(mmu, 0x08); break;
        case 0xD0:
            /* RET NC */
            if (flag_c == 0) {
                pc = mmu.read(sp++);
                pc |= (mmu.read(sp++) << 8);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            break;
        case 0xD1:
            /* POP DE */
            POP_R16(mmu, &d, &e); break;
        case 0xD2:
            /* JP NC,u16 */
            if (flag_c == 0) {
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xD3:
            /* ILLEGAL */
            return -1;
        case 0xD4:
            /* CALL NC,u16 */
            if (flag_c == 0) {
                PUSH_R16(mmu, HI(pc+2), LO(pc+2));
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xD5:
            /* PUSH DE */
            PUSH_R16(mmu, d, e); break;
        case 0xD6:
            /* SUB A,u8 */
            SUB_R8_U8(&acc, fetch_pc(mmu)); break;
        case 0xD7:
            /* RST 10h */
            RST(mmu, 0x10); break;
        case 0xD8:
            /* RET C */
            if (flag_c == 1) {
                RET(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            }
            break;
        case 0xD9:
            /* RETI */
        NOT_IMPLEMENTED
        case 0xDA:
            /* JP C,u16 */
            if (flag_c == 1) {
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xDB:
            /* ILLEGAL */
            return -1;
        case 0xDC:
            /* CALL C,u16 */
            if (flag_c == 1) {
                PUSH_R16(mmu, HI(pc+2), LO(pc+2));
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
            } else {
                pc += 2;
            }
            break;
        case 0xDD:
            /* ILLEGAL */
            return -1;
        case 0xDE:
            /* SBC A,u8 */
            SBC_R8_U8(&acc, fetch_pc(mmu)); break;
        case 0xDF:
            /* RST 18h */
            RST(mmu, 0x18); break;
        case 0xE0:
            /* LD (FF00+u8),A */
            mmu.write(0xFF00 + fetch_pc(mmu), acc); break;
        case 0xE1:
            /* POP HL */
            POP_R16(mmu, &h, &l); break;
        case 0xE2:
            /* LD (FF00+C),A */
            mmu.write(0xFF00 + c, acc); break;
        case 0xE3:
            /* ILLEGAL */
            return -1;
        case 0xE4:
            /* ILLEGAL */
            return -1;
        case 0xE5:
            /* PUSH HL */
            PUSH_R16(mmu, h, l); break;
        case 0xE6:
            /* AND A,u8 */
            AND_R8_U8(&acc, fetch_pc(mmu)); break;
        case 0xE7:
            /* RST 20h */
            RST(mmu, 0x20); break;
        case 0xE8:
            /* ADD SP,i8 */
        NOT_IMPLEMENTED
        case 0xE9:
            /* JP HL */
            pc = HILO(h, l); break;
        case 0xEA:
            /* LD (a16), A */
            mmu.write(HILO(mmu.read(pc+1), mmu.read(pc)), acc);
            pc += 2;
            break;
        case 0xEB:
            /* ILLEGAL */
            return -1;
        case 0xEC:
            /* ILLEGAL */
            return -1;
        case 0xED:
            /* ILLEGAL */
            return -1;
        case 0xEE:
            /* XOR A,u8 */
            XOR_R8_U8(&acc, fetch_pc(mmu)); break;
        case 0xEF:
            /* RST 28h */
            RST(mmu, 0x28); break;
        case 0xF0:
            /* LD A,(FF00+u8) */
            acc = mmu.read(0xFF00 + fetch_pc(mmu)); break;
        case 0xF1:
            /* POP AF */
            POP_R16(mmu, &acc, &scratch8);
            flag_c = BIT4(scratch8);
            flag_h = BIT5(scratch8);
            flag_n = BIT6(scratch8);
            flag_z = BIT7(scratch8);
            break;
        case 0xF2:
            /* LD A,(FF00+C) */
            acc = mmu.read(0xFF00 + c); break;
        case 0xF3:
            /* DI */
            flag_ime = 0;
            break;
        case 0xF4:
            /* ILLEGAL */
            return -1;
        case 0xF5:
            /* PUSH AF */
            PUSH_R16(mmu, acc, (flag_z << 7) | (flag_n << 6) | (flag_h << 5) | (flag_c << 4));
            break;
        case 0xF6:
            /* OR A,u8 */
            OR_R8_U8(&acc, fetch_pc(mmu)); break;
        case 0xF7:
            /* RST 30h */
            RST(mmu, 0x30);
        case 0xF8:
            /* LD HL,SP+i8 */
        NOT_IMPLEMENTED
        case 0xF9:
            /* LD SP,HL */
            sp = HILO(h, l); break;
        case 0xFA:
            /* LD A,(u16) */
            LD_R8_A16(mmu, &acc); break;
        case 0xFB:
            /* EI */
            flag_ime = 1; break;
        case 0xFC:
            /* ILLEGAL */
            return -1;
        case 0xFD:
            /* ILLEGAL */
            return -1;
        case 0xFE:
            /* CP A,u8 */
            scratch8 = fetch_pc(mmu);
            flag_z = acc == scratch8 ? 1 : 0;
            flag_h = CHECK_HALF_CARRY_DEC(acc, scratch8);
            flag_c = CHECK_CARRY_DEC(acc, scratch8);
            flag_n = 1;
            break;
        case 0xFF:
            /* RST 38h */
            RST(mmu, 0x38); break;
    }

    // run ppu
    ppu.tick(clocks, mmu);

    return clocks;
}

void cpu::prefix(mmu &mmu) {
    switch (fetch_pc(mmu)) {
        case 0x00:
            /* RLC B */
            RLC(&b); break;
        case 0x01:
            /* RLC C */
            RLC(&c); break;
        case 0x02:
            /* RLC D */
            RLC(&d); break;
        case 0x03:
            /* RLC E */
            RLC(&e); break;
        case 0x04:
            /* RLC H */
            RLC(&h); break;
        case 0x05:
            /* RLC L */
            RLC(&l); break;
        case 0x06:
            /* RLC (HL) */
            RLC_addr(mmu, HILO(h, l)); break;
        case 0x07:
            /* RLC A */
            RLC(&acc); break;
        case 0x08:
            /* RRC B */
            RRC(&b); break;
        case 0x09:
            /* RRC C */
            RRC(&c); break;
        case 0x0A:
            /* RRC D */
            RRC(&d); break;
        case 0x0B:
            /* RRC E */
            RRC(&e); break;
        case 0x0C:
            /* RRC H */
            RRC(&h); break;
        case 0x0D:
            /* RRC L */
            RRC(&l); break;
        case 0x0E:
            /* RRC (HL) */
            RRC_addr(mmu, HILO(h, l)); break;
        case 0x0F:
            /* RRC A */
            RRC(&acc); break;
        case 0x10:
            /* RL B */
            RL(&b); break;
        case 0x11:
            /* RL C */
            RL(&c); break;
        case 0x12:
            /* RL D */
            RL(&d); break;
        case 0x13:
            /* RL E */
            RL(&e); break;
        case 0x14:
            /* RL H */
            RL(&h); break;
        case 0x15:
            RL(&l); break;
        case 0x16:
            /* RL (HL) */
            RL_addr(mmu, HILO(h, l)); break;
        case 0x17:
            /* RL A */
            RL(&acc); break;
        case 0x18:
            /* RR B */
            RR(&b); break;
        case 0x19:
            /* RR C */
            RR(&c); break;
        case 0x1A:
            /* RR D */
            RR(&d); break;
        case 0x1B:
            /* RR E */
            RR(&e); break;
        case 0x1C:
            /* RR H */
            RR(&h); break;
        case 0x1D:
            /* RR L */
            RR(&l); break;
        case 0x1E:
            /* RR (HL) */
            RR_addr(mmu, HILO(h, l)); break;
        case 0x1F:
            /* RR A */
            RR(&acc); break;
        case 0x20:
            /* SLA B */
            SLA(&b); break;
        case 0x21:
            /* SLA C */
            SLA(&c); break;
        case 0x22:
            /* SLA D */
            SLA(&d); break;
        case 0x23:
            /* SLA E */
            SLA(&e); break;
        case 0x24:
            /* SLA H */
            SLA(&h); break;
        case 0x25:
            /* SLA L */
            SLA(&l); break;
        case 0x26:
            /* SLA (HL) */
            SLA_addr(mmu, HILO(h, l)); break;
        case 0x27:
            /* SLA A */
            SLA(&acc); break;
        case 0x28:
            /* SRA B */
            SRA(&b); break;
        case 0x29:
            /* SRA C */
            SRA(&c); break;
        case 0x2A:
            /* SRA D */
            SRA(&d); break;
        case 0x2B:
            /* SRA E */
            SRA(&e); break;
        case 0x2C:
            /* SRA H */
            SRA(&h); break;
        case 0x2D:
            /* SRA L */
            SRA(&l); break;
        case 0x2E:
            /* SRA (HL) */
            CB_FUNC_ADDR(mmu, HILO(h, l), &cpu::SRA); break;
        case 0x2F:
            /* SRA A */
            SRA(&acc); break;
        case 0x30:
            /* SWAP B */
            SWAP(&b); break;
        case 0x31:
            /* SWAP C */
            SWAP(&c); break;
        case 0x32:
            /* SWAP D */
            SWAP(&d); break;
        case 0x33:
            /* SWAP E */
            SWAP(&e); break;
        case 0x34:
            /* SWAP H */
            SWAP(&h); break;
        case 0x35:
            /* SWAP L */
            SWAP(&l); break;
        case 0x36:
            /* SWAP (HL) */
            CB_FUNC_ADDR(mmu, HILO(h, l), &cpu::SWAP); break;
        case 0x37:
            /* SWAP A */
            SWAP(&acc); break;
        case 0x38:
            /* SRL B */
            SRL(&b); break;
        case 0x39:
            /* SRL C */
            SRL(&c); break;
        case 0x3A:
            /* SRL D */
            SRL(&d); break;
        case 0x3B:
            /* SRL E */
            SRL(&e); break;
        case 0x3C:
            /* SRL H */
            SRL(&h); break;
        case 0x3D:
            /* SRL L */
            SRL(&l); break;
        case 0x3E:
            /* SRL (HL) */
            CB_FUNC_ADDR(mmu, HILO(h, l), &cpu::SRL); break;
        case 0x3F:
            /* SRL A */
            SRL(&acc); break;
        case 0x40:
            /* BIT 0,B */
            BIT(b, 0); break;
        case 0x41:
            /* BIT 0,C */
            BIT(c, 0); break;
        case 0x42:
            /* BIT 0,D */
            BIT(d, 0); break;
        case 0x43:
            /* BIT 0,E */
            BIT(e, 0); break;
        case 0x44:
            /* BIT 0,H */
            BIT(h, 0); break;
        case 0x45:
            /* BIT 0,L */
            BIT(l, 0); break;
        case 0x46:
            /* BIT 0,(HL) */
            BIT_addr(mmu, HILO(h, l), 0); break;
        case 0x47:
            /* BIT 0,A */
            BIT(acc, 0); break;
        case 0x48:
            /* BIT 1,B */
            BIT(b, 1); break;
        case 0x49:
            /* BIT 1,C */
            BIT(c, 1); break;
        case 0x4A:
            /* BIT 1,D */
            BIT(d, 1); break;
        case 0x4B:
            /* BIT 1,E */
            BIT(e, 1); break;
        case 0x4C:
            /* BIT 1,H */
            BIT(h, 1); break;
        case 0x4D:
            /* BIT 1,L */
            BIT(l, 1); break;
        case 0x4E:
            /* BIT 1,(HL) */
            BIT_addr(mmu, HILO(h, l), 1); break;
        case 0x4F:
            /* BIT 1,A */
            BIT(acc, 1); break;
        case 0x50:
            /* BIT 2,B */
            BIT(b, 2); break;
        case 0x51:
            /* BIT 2,C */
            BIT(c, 2); break;
        case 0x52:
            /* BIT 2,D */
            BIT(d, 2); break;
        case 0x53:
            /* BIT 2,E */
            BIT(e, 2); break;
        case 0x54:
            /* BIT 2,H */
            BIT(h, 2); break;
        case 0x55:
            /* BIT 2,L */
            BIT(l, 2); break;
        case 0x56:
            /* BIT 2,(HL) */
            BIT_addr(mmu, HILO(h, l), 2); break;
        case 0x57:
            /* BIT 2,A */
            BIT(acc, 2); break;
        case 0x58:
            /* BIT 3,B */
            BIT(b, 3); break;
        case 0x59:
            /* BIT 3,C */
            BIT(c, 3); break;
        case 0x5A:
            /* BIT 3,D */
            BIT(d, 3); break;
        case 0x5B:
            /* BIT 3,E */
            BIT(e, 3); break;
        case 0x5C:
            /* BIT 3,H */
            BIT(h, 3); break;
        case 0x5D:
            /* BIT 3,L */
            BIT(l, 3); break;
        case 0x5E:
            /* BIT 3,(HL) */
            BIT_addr(mmu, HILO(h, l), 3); break;
        case 0x5F:
            /* BIT 3,A */
            BIT(acc, 3); break;
        case 0x60:
            /* BIT 4,B */
            BIT(b, 4); break;
        case 0x61:
            /* BIT 4,C */
            BIT(c, 4); break;
        case 0x62:
            /* BIT 4,D */
            BIT(d, 4); break;
        case 0x63:
            /* BIT 4,E */
            BIT(e, 4); break;
        case 0x64:
            /* BIT 4,H */
            BIT(h, 4); break;
        case 0x65:
            /* BIT 4,L */
            BIT(l, 4); break;
        case 0x66:
            /* BIT 4,(HL) */
            BIT_addr(mmu, HILO(h, l), 4); break;
        case 0x67:
            /* BIT 4,A */
            BIT(acc, 4); break;
        case 0x68:
            /* BIT 5,B */
            BIT(b, 5); break;
        case 0x69:
            /* BIT 5,C */
            BIT(c, 5); break;
        case 0x6A:
            /* BIT 5,D */
            BIT(d, 5); break;
        case 0x6B:
            /* BIT 5,E */
            BIT(e, 5); break;
        case 0x6C:
            /* BIT 5,H */
            BIT(h, 5); break;
        case 0x6D:
            /* BIT 5,L */
            BIT(l, 5); break;
        case 0x6E:
            /* BIT 5,(HL) */
            BIT_addr(mmu, HILO(h, l), 5); break;
        case 0x6F:
            /* BIT 5,A */
            BIT(acc, 5); break;
        case 0x70:
            /* BIT 6,B */
            BIT(b, 6); break;
        case 0x71:
            /* BIT 6,C */
            BIT(c, 6); break;
        case 0x72:
            /* BIT 6,D */
            BIT(d, 6); break;
        case 0x73:
            /* BIT 6,E */
            BIT(e, 6); break;
        case 0x74:
            /* BIT 6,H */
            BIT(h, 6); break;
        case 0x75:
            /* BIT 6,L */
            BIT(l, 6); break;
        case 0x76:
            /* BIT 6,(HL) */
            BIT_addr(mmu, HILO(h, l), 6); break;
        case 0x77:
            /* BIT 6,A */
            BIT(acc, 6); break;
        case 0x78:
            /* BIT 7,B */
            BIT(b, 7); break;
        case 0x79:
            /* BIT 7,C */
            BIT(c, 7); break;
        case 0x7A:
            /* BIT 7,D */
            BIT(d, 7); break;
        case 0x7B:
            /* BIT 7,E */
            BIT(e, 7); break;
        case 0x7C:
            /* BIT 7,H */
            BIT(h, 7); break;
        case 0x7D:
            /* BIT 7,L */
            BIT(l, 7); break;
        case 0x7E:
            /* BIT 7,(HL) */
            BIT_addr(mmu, HILO(h, l), 7); break;
        case 0x7F:
            /* BIT 7,A */
            BIT(acc, 7); break;
        case 0x80:
            /* RES 0,B */
            RES(&b, 0); break;
        case 0x81:
            /* RES 0,C */
            RES(&c, 0); break;
        case 0x82:
            /* RES 0,D */
            RES(&d, 0); break;
        case 0x83:
            /* RES 0,E */
            RES(&e, 0); break;
        case 0x84:
            /* RES 0,H */
            RES(&h, 0); break;
        case 0x85:
            /* RES 0,L */
            RES(&l, 0); break;
        case 0x86:
            /* RES 0,(HL) */
            RES_addr(mmu, HILO(h, l), 0); break;
        case 0x87:
            /* RES 0,A */
            RES(&acc, 0); break;
        case 0x88:
            /* RES 1,B */
            RES(&b, 1); break;
        case 0x89:
            /* RES 1,C */
            RES(&c, 1); break;
        case 0x8A:
            /* RES 1,D */
            RES(&d, 1); break;
        case 0x8B:
            /* RES 1,E */
            RES(&e, 1); break;
        case 0x8C:
            /* RES 1,H */
            RES(&h, 1); break;
        case 0x8D:
            /* RES 1,L */
            RES(&l, 1); break;
        case 0x8E:
            /* RES 1,(HL) */
            RES_addr(mmu, HILO(h, l), 1); break;
        case 0x8F:
            /* RES 1,A */
            RES(&acc, 1); break;
        case 0x90:
            /* RES 2,B */
            RES(&b, 2); break;
        case 0x91:
            /* RES 2,C */
            RES(&c, 2); break;
        case 0x92:
            /* RES 2,D */
            RES(&d, 2); break;
        case 0x93:
            /* RES 2,E */
            RES(&e, 2); break;
        case 0x94:
            /* RES 2,H */
            RES(&h, 2); break;
        case 0x95:
            /* RES 2,L */
            RES(&l, 2); break;
        case 0x96:
            /* RES 2,(HL) */
            RES_addr(mmu, HILO(h, l), 2); break;
        case 0x97:
            /* RES 2,A */
            RES(&acc, 2); break;
        case 0x98:
            /* RES 3,B */
            RES(&b, 3); break;
        case 0x99:
            /* RES 3,C */
            RES(&c, 3); break;
        case 0x9A:
            /* RES 3,D */
            RES(&d, 3); break;
        case 0x9B:
            /* RES 3,E */
            RES(&e, 3); break;
        case 0x9C:
            /* RES 3,H */
            RES(&h, 3); break;
        case 0x9D:
            /* RES 3,L */
            RES(&l, 3); break;
        case 0x9E:
            /* RES 3,(HL) */
            RES_addr(mmu, HILO(h, l), 3); break;
        case 0x9F:
            /* RES 3,A */
            RES(&acc, 3); break;
        case 0xA0:
            /* RES 4,B */
            RES(&b, 4); break;
        case 0xA1:
            /* RES 4,C */
            RES(&c, 4); break;
        case 0xA2:
            /* RES 4,D */
            RES(&d, 4); break;
        case 0xA3:
            /* RES 4,E */
            RES(&e, 4); break;
        case 0xA4:
            /* RES 4,H */
            RES(&h, 4); break;
        case 0xA5:
            /* RES 4,L */
            RES(&l, 4); break;
        case 0xA6:
            /* RES 4,(HL) */
            RES_addr(mmu, HILO(h, l), 4); break;
        case 0xA7:
            /* RES 4,A */
            RES(&acc, 4); break;
        case 0xA8:
            /* RES 5,B */
            RES(&b, 5); break;
        case 0xA9:
            /* RES 5,C */
            RES(&c, 5); break;
        case 0xAA:
            /* RES 5,D */
            RES(&d, 5); break;
        case 0xAB:
            /* RES 5,E */
            RES(&e, 5); break;
        case 0xAC:
            /* RES 5,H */
            RES(&h, 5); break;
        case 0xAD:
            /* RES 5,L */
            RES(&l, 5); break;
        case 0xAE:
            /* RES 5,(HL) */
            RES_addr(mmu, HILO(h, l), 5); break;
        case 0xAF:
            /* RES 5,A */
            RES(&acc, 5); break;
        case 0xB0:
            /* RES 6,B */
            RES(&b, 6); break;
        case 0xB1:
            /* RES 6,C */
            RES(&c, 6); break;
        case 0xB2:
            /* RES 6,D */
            RES(&d, 6); break;
        case 0xB3:
            /* RES 6,E */
            RES(&e, 6); break;
        case 0xB4:
            /* RES 6,H */
            RES(&h, 6); break;
        case 0xB5:
            /* RES 6,L */
            RES(&l, 6); break;
        case 0xB6:
            /* RES 6,(HL) */
            RES_addr(mmu, HILO(h, l), 6); break;
        case 0xB7:
            /* RES 6,A */
            RES(&acc, 6); break;
        case 0xB8:
            /* RES 7,B */
            RES(&b, 7); break;
        case 0xB9:
            /* RES 7,C */
            RES(&c, 7); break;
        case 0xBA:
            /* RES 7,D */
            RES(&d, 7); break;
        case 0xBB:
            /* RES 7,E */
            RES(&e, 7); break;
        case 0xBC:
            /* RES 7,H */
            RES(&h, 7); break;
        case 0xBD:
            /* RES 7,L */
            RES(&l, 7); break;
        case 0xBE:
            /* RES 7,(HL) */
            RES_addr(mmu, HILO(h, l), 7); break;
        case 0xBF:
            /* RES 7,A */
            RES(&acc, 7); break;
        case 0xC0:
            /* SET 0,B */
            SET(&b, 0); break;
        case 0xC1:
            /* SET 0,C */
            SET(&c, 0); break;
        case 0xC2:
            /* SET 0,D */
            SET(&d, 0); break;
        case 0xC3:
            /* SET 0,E */
            SET(&e, 0); break;
        case 0xC4:
            /* SET 0,H */
            SET(&h, 0); break;
        case 0xC5:
            /* SET 0,L */
            SET(&l, 0); break;
        case 0xC6:
            /* SET 0,(HL) */
            SET_addr(mmu, HILO(h, l), 0); break;
        case 0xC7:
            /* SET 0,A */
            SET(&acc, 0); break;
        case 0xC8:
            /* SET 1,B */
            SET(&b, 1); break;
        case 0xC9:
            /* SET 1,C */
            SET(&c, 1); break;
        case 0xCA:
            /* SET 1,D */
            SET(&d, 1); break;
        case 0xCB:
            /* SET 1,E */
            SET(&e, 1); break;
        case 0xCC:
            /* SET 1,H */
            SET(&h, 1); break;
        case 0xCD:
            /* SET 1,L */
            SET(&l, 1); break;
        case 0xCE:
            /* SET 1,(HL) */
            SET_addr(mmu, HILO(h, l), 1); break;
        case 0xCF:
            /* SET 1,A */
            SET(&acc, 1); break;
        case 0xD0:
            /* SET 2,B */
            SET(&b, 2); break;
        case 0xD1:
            /* SET 2,C */
            SET(&c, 2); break;
        case 0xD2:
            /* SET 2,D */
            SET(&d, 2); break;
        case 0xD3:
            /* SET 2,E */
            SET(&e, 2); break;
        case 0xD4:
            /* SET 2,H */
            SET(&h, 2); break;
        case 0xD5:
            /* SET 2,L */
            SET(&l, 2); break;
        case 0xD6:
            /* SET 2,(HL) */
            SET_addr(mmu, HILO(h, l), 2); break;
        case 0xD7:
            /* SET 2,A */
            SET(&acc, 2); break;
        case 0xD8:
            /* SET 3,B */
            SET(&b, 3); break;
        case 0xD9:
            /* SET 3,C */
            SET(&c, 3); break;
        case 0xDA:
            /* SET 3,D */
            SET(&d, 3); break;
        case 0xDB:
            /* SET 3,E */
            SET(&e, 3); break;
        case 0xDC:
            /* SET 3,H */
            SET(&h, 3); break;
        case 0xDD:
            /* SET 3,L */
            SET(&l, 3); break;
        case 0xDE:
            /* SET 3,(HL) */
            SET_addr(mmu, HILO(h, l), 3); break;
        case 0xDF:
            /* SET 3,A */
            SET(&acc, 3); break;
        case 0xE0:
            /* SET 4,B */
            SET(&b, 4); break;
        case 0xE1:
            /* SET 4,C */
            SET(&c, 4); break;
        case 0xE2:
            /* SET 4,D */
            SET(&d, 4); break;
        case 0xE3:
            /* SET 4,E */
            SET(&e, 4); break;
        case 0xE4:
            /* SET 4,H */
            SET(&h, 4); break;
        case 0xE5:
            /* SET 4,L */
            SET(&l, 4); break;
        case 0xE6:
            /* SET 4,(HL) */
            SET_addr(mmu, HILO(h, l), 4); break;
        case 0xE7:
            /* SET 4,A */
            SET(&acc, 4); break;
        case 0xE8:
            /* SET 5,B */
            SET(&b, 5); break;
        case 0xE9:
            /* SET 5,C */
            SET(&c, 5); break;
        case 0xEA:
            /* SET 5,D */
            SET(&d, 5); break;
        case 0xEB:
            /* SET 5,E */
            SET(&e, 5); break;
        case 0xEC:
            /* SET 5,H */
            SET(&h, 5); break;
        case 0xED:
            /* SET 5,L */
            SET(&l, 5); break;
        case 0xEE:
            /* SET 5,(HL) */
            SET_addr(mmu, HILO(h, l), 5); break;
        case 0xEF:
            /* SET 5,A */
            SET(&acc, 5); break;
        case 0xF0:
            /* SET 6,B */
            SET(&b, 6); break;
        case 0xF1:
            /* SET 6,C */
            SET(&c, 6); break;
        case 0xF2:
            /* SET 6,D */
            SET(&d, 6); break;
        case 0xF3:
            /* SET 6,E */
            SET(&e, 6); break;
        case 0xF4:
            /* SET 6,H */
            SET(&h, 6); break;
        case 0xF5:
            /* SET 6,L */
            SET(&l, 6); break;
        case 0xF6:
            /* SET 6,(HL) */
            SET_addr(mmu, HILO(h, l), 6); break;
        case 0xF7:
            /* SET 6,A */
            SET(&acc, 6); break;
        case 0xF8:
            /* SET 7,B */
            SET(&b, 7); break;
        case 0xF9:
            /* SET 7,C */
            SET(&c, 7); break;
        case 0xFA:
            /* SET 7,D */
            SET(&d, 7); break;
        case 0xFB:
            /* SET 7,E */
            SET(&e, 7); break;
        case 0xFC:
            /* SET 7,H */
            SET(&h, 7); break;
        case 0xFD:
            /* SET 7,L */
            SET(&l, 7); break;
        case 0xFE:
            /* SET 7,(HL) */
            SET_addr(mmu, HILO(h, l), 7); break;
        case 0xFF:
            /* SET 7,A */
            SET(&acc, 7); break;
    }
}

void cpu::init_no_bootrom() {
    acc = 0x01;
    flag_c = 1; flag_h = 1; flag_z = 1;
    b = 0x00;
    c = 0x13;
    d = 0x00;
    e = 0xD8;
    h = 0x01;
    l = 0x4D;
    sp = 0xFFFE;
    pc = 0x0100;
}
