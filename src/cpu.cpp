//
// Created by colin on 7/26/24.
//

#include <cstdio>
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
#define CHECK_HALF_CARRY_DEC(A, B) (uint8_t)( ( ( ( ((uint8_t)A) & 0xF ) - ( ((uint8_t)(B)) & 0xF ) ) & 0x10 ) >> 4 )
#define CHECK_CARRY(A, B) (uint8_t)( ( (((uint16_t)A) & 0xFF) + (((uint16_t)B) & 0xFF) ) >> 8)
#define CHECK_CARRY_DEC(A, B) A < B ? 1 : 0
#define CHECK_ZERO(A) (A) == 0 ? 1 : 0

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
    y += flag_c;
    flag_c = CHECK_CARRY((*x), y);
    flag_h = CHECK_HALF_CARRY((*x), y);
    flag_n = 0;
    (*x) += y;
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
    flag_h = 0;
    flag_n = 0;
    flag_z = 0;
}
inline void cpu::RRC(uint8_t *x) {
    if (*x & 0x1) {
        *x = ((*x) >> 1) | 0b10000000;
        flag_c = 1;
    } else {
        *x = ((*x) >> 1);
        flag_c = 0;
    }
    flag_h = 0;
    flag_n = 0;
    flag_z = 0;
}
inline void cpu::RL(uint8_t *x) {
    if (*x & 0b10000000) {
        *x = ((*x) << 1) | (flag_c & 0x1);
        flag_c = 1;
    } else {
        *x = ((*x) << 1) | (flag_c & 0x1);
        flag_c = 0;
    }
    flag_h = 0;
    flag_n = 0;
    flag_z = 0;
}
inline void cpu::RLC(uint8_t *x) {
    if (*x & 0b10000000) {
        *x = ((*x) << 1) | 0x1;
        flag_c = 1;
    } else {
        *x = ((*x) << 1);
        flag_c = 0;
    }
    flag_h = 0;
    flag_n = 0;
    flag_z = 0;
}

int cpu::tick(mmu &mmu, ppu &ppu) {
    uint8_t f = (flag_z << 7) | (flag_n << 6) | (flag_h << 5) | (flag_c << 4);
    printf("A:%02x F:%02x B:%02x C:%02x D:%02x E:%02x H:%02x L:%02x SP:%04x PC:%04x PCMEM:%02x,%02x,%02x,%02x\n",
           acc, f, b, c, d, e, h, l, sp, pc,
           mmu.read(pc), mmu.read(pc+1), mmu.read(pc+2), mmu.read(pc+3));

    int clocks = 0;
    pc_val = fetch_pc(mmu);
    clocks += INSTR_CLOCKS_BASE[pc_val];
    ppu.tick(INSTR_CLOCKS_BASE[pc_val], mmu);

//    if (pc == 0x0055) {
//        mmu.dump_mem(0x8000, 26 * 16);
//    }

//    printf("PC: %04x PC_VAL: %02x\n", pc-1, pc_val);
//    printf("(%04x)->%02x (%04x)->%02x\n", HILO(h,l), mmu.read(HILO(h, l)), HILO(d,e), mmu.read(HILO(d, e)));
//    printf("acc: %02x\n", acc);
    switch (pc_val) {
        case 0x00:
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
            flag_h = 0;
            flag_n = 0;
            flag_z = 0;
            scratch8 = BIT7(acc);
            acc = (acc << 1) | scratch8;
            flag_c = scratch8;
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
            flag_h = CHECK_HALF_CARRY(c, 1);
            c ++;
            flag_z = c == 0 ? 1 : 0;
            flag_n = 0;
            break;
        case 0x0D:
            /* DEC C */
            DEC_R8(&c); break;
        case 0x0E:
            /* LD C,u8 */
            LD_R8_U8(mmu, &c);
            break;
        case 0x0F:
            /* RRCA */
            RRC(&acc); break;
        case 0x10:
            return -1;
        case 0x11:
            /* LD DE,u16 */
            e = fetch_pc(mmu);
            d = fetch_pc(mmu);
            break;
        case 0x12:
            /* LD (DE),A */
            LD_A16_U8(mmu, HILO(d, e), acc);
            break;
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
            RL(&acc); break;
        case 0x18:
            /* JR i8 */
            pc += (int8_t) fetch_pc(mmu); break;
        case 0x19:
            /* ADD HL,DE */
            ADD_R16_U16(&h, &l, d, e); break;
        case 0x1A:
            /* LD A,(DE) */
            LD_R8_AR16(mmu, &acc, d, e);
            break;
        case 0x1B:
            /* DEC DE */
            DEC_R16(&d, &e);
            break;
        case 0x1C:
            /* INC E */
            INC_R8(&e);
            break;
        case 0x1D:
            /* DEC E */
            DEC_R8(&e);
            break;
        case 0x1E:
            /* LD E,u8 */
            LD_R8_U8(mmu, &e);
            break;
        case 0x1F:
            /* RRA */
            RR(&acc);
            break;
        case 0x20:
            /* JR NZ, s8 */
            if (flag_z == 0) {
                pc += (int8_t) fetch_pc(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
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
            INC_R8(&h);
            break;
        case 0x25:
            /* DEC H */
            DEC_R8(&h);
            break;
        case 0x26:
            /* LD H,u8 */
            LD_R8_U8(mmu, &h);
            break;
        case 0x27:
            /* DAA */
        NOT_IMPLEMENTED
        case 0x28:
            /* JR Z, s8 */
            if (flag_z == 1) {
                pc += (int8_t) fetch_pc(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
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
        NOT_IMPLEMENTED
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
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
            } else {
                pc++;
            }
            break;
        case 0x31:
            sp = HILO(mmu.read(pc + 1), mmu.read(pc));
            pc += 2;
            //printf("0x31 :: %04X\n", sp);
            break;
        case 0x32:
            scratch16 = HILO(h, l);
            mmu.write(scratch16, acc);
            l = LO(scratch16 - 1);
            h = HI(scratch16 - 1);
            break;
        case 0x33:
        case 0x34:
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
        case 0x38:
        case 0x39:
        case 0x3A:
        case 0x3B:
        case 0x3C:
        NOT_IMPLEMENTED
        case 0x3D:
            /* DEC A */
            flag_h = CHECK_HALF_CARRY_DEC(acc, 1);
            acc --;
            flag_n = 1;
            flag_z = acc == 0 ? 1 : 0;
            break;
        case 0x3E:
            /* LD A,u8 */
            LD_R8_U8(mmu, &acc);
            break;
        case 0x3F:
        case 0x40:
        case 0x41:
        case 0x42:
        case 0x43:
        case 0x44:
        case 0x45:
        NOT_IMPLEMENTED
        case 0x46:
            /* LD B,(HL) */
            LD_R8_AR16(mmu, &b, h, l);
            break;
        case 0x47:
            /* LD B,A */
            LD_R8_R8(b, acc);
            break;
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0x4B:
        case 0x4C:
        case 0x4D:
        NOT_IMPLEMENTED
        case 0x4E:
            /* LD C,(HL) */
            LD_R8_AR16(mmu, &c, h, l);
            break;
        case 0x4F:
            /* LD C,A */
            LD_R8_R8(c, acc);
            break;
        case 0x50:
            /* LD D,B */
            LD_R8_R8(d, b);
            break;
        case 0x51:
            /* LD D,C */
            LD_R8_R8(d, c);
            break;
        case 0x52:
            /* LD D,D */
            LD_R8_R8(d, d);
            break;
        case 0x53:
            /* LD D,E */
            LD_R8_R8(d, e);
            break;
        case 0x54:
            /* LD D,H */
            LD_R8_R8(d, h);
            break;
        case 0x55:
            /* LD D,L */
            LD_R8_R8(d, l);
            break;
        case 0x56:
            /* LD D,(HL) */
            LD_R8_AR16(mmu, &d, h, l);
            break;
        case 0x57:
            /* LD D,A */
            LD_R8_R8(d, acc);
            break;
        case 0x58:
            /* LD E,B */
            LD_R8_R8(e, b);
            break;
        case 0x59:
            /* LD E,C */
            LD_R8_R8(e, c);
            break;
        case 0x5A:
            /* LD E,D */
            LD_R8_R8(e, d);
            break;
        case 0x5B:
            /* LD E,E */
            LD_R8_R8(e, e);
            break;
        case 0x5C:
            /* LD E,H */
            LD_R8_R8(e, h);
            break;
        case 0x5D:
            /* LD E,L */
            LD_R8_R8(e, l);
            break;
        case 0x5E:
            /* LD E,(HL) */
            LD_R8_AR16(mmu, &e, h, l);
            break;
        case 0x5F:
            /* LD E,A */
            LD_R8_R8(e, acc);
            break;
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
            return(-1);
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
            ADD_R8_U8(&acc, b);
            break;
        case 0x81:
            /* ADD A,C */
            ADD_R8_U8(&acc, c);
            break;
        case 0x82:
            /* ADD A,D */
            ADD_R8_U8(&acc, d);
            break;
        case 0x83:
            /* ADD A,E */
            ADD_R8_U8(&acc, e);
            break;
        case 0x84:
            /* ADD A,H */
            ADD_R8_U8(&acc, h);
            break;
        case 0x85:
            /* ADD A,L */
            ADD_R8_U8(&acc, l);
            break;
        case 0x86:
            /* ADD A,(HL) */
            ADD_R8_U8(&acc, mmu.read(HILO(h, l)));
            break;
        case 0x87:
            /* ADD A,A */
            ADD_R8_U8(&acc, acc);
            break;
        case 0x88:
            /* ADC A,B */
            ADC_R8_U8(&acc, b);
            break;
        case 0x89:
            /* ADC A,C */
            ADC_R8_U8(&acc, c);
            break;
        case 0x8A:
            /* ADC A,D */
            ADC_R8_U8(&acc, d);
            break;
        case 0x8B:
            /* ADC A,E */
            ADC_R8_U8(&acc, e);
            break;
        case 0x8C:
            /* ADC A,H */
            ADC_R8_U8(&acc, h);
            break;
        case 0x8D:
            /* ADC A,L */
            ADC_R8_U8(&acc, l);
            break;
        case 0x8E:
            /* ADC A,(HL) */
            ADC_R8_U8(&acc, mmu.read(HILO(h, l)));
            break;
        case 0x8F:
            /* ADC A,A */
            ADC_R8_U8(&acc, acc);
            break;
        case 0x90:
            /* SUB A,B */
            SUBu8(acc, b);
            break;
        case 0x91:
            /* SUB A,C */
            SUBu8(acc, c);
            break;
        case 0x92:
            /* SUB A,D */
            SUBu8(acc, d);
            break;
        case 0x93:
            /* SUB A,E */
            SUBu8(acc, e);
            break;
        case 0x94:
            /* SUB A,H */
            SUBu8(acc, h);
            break;
        case 0x95:
            /* SUB A,L */
            SUBu8(acc, l);
            break;
        case 0x96:
            /* SUB A,C */
            scratch8 = mmu.read(HILO(h, l));
            SUBu8(acc, scratch8);
            break;
        case 0x97:
            /* SUB A,A */
            SUBu8(acc, acc);
            break;
        case 0x98:
        case 0x99:
        case 0x9A:
        case 0x9B:
        case 0x9C:
        case 0x9D:
        case 0x9E:
        case 0x9F:
        NOT_IMPLEMENTED
        case 0xA0:
            /* AND A,B */
            AND_R8_U8(&acc, b);
            break;
        case 0xA1:
            /* AND A,C */
            AND_R8_U8(&acc, c);
            break;
        case 0xA2:
            /* AND A,D */
            AND_R8_U8(&acc, d);
            break;
        case 0xA3:
            /* AND A,E */
            AND_R8_U8(&acc, e);
            break;
        case 0xA4:
            /* AND A,H */
            AND_R8_U8(&acc, h);
            break;
        case 0xA5:
            /* AND A,L */
            AND_R8_U8(&acc, l);
            break;
        case 0xA6:
            /* AND A,(HL) */
            AND_R8_U8(&acc, mmu.read(HILO(h, l)));
            break;
        case 0xA7:
            /* AND A,A */
            AND_R8_U8(&acc, acc);
            break;
        case 0xA8:
            /* XOR A,B */
            XOR_R8_U8(&acc, b);
            break;
        case 0xA9:
            /* XOR A,C */
            XOR_R8_U8(&acc, c);
            break;
        case 0xAA:
            /* XOR A,D */
            XOR_R8_U8(&acc, d);
            break;
        case 0xAB:
            /* XOR A,E */
            XOR_R8_U8(&acc, e);
            break;
        case 0xAC:
            /* XOR A,H */
            XOR_R8_U8(&acc, h);
            break;
        case 0xAD:
            /* XOR A,L */
            XOR_R8_U8(&acc, l);
            break;
        case 0xAE:
            /* XOR A,(HL) */
            XOR_R8_U8(&acc, mmu.read(HILO(h, l)));
            break;
        case 0xAF:
            /* XOR A,A */
            XOR_R8_U8(&acc, acc);
            break;
        case 0xB0:
            /* OR A,B */
            OR_R8_U8(&acc, b);
            break;
        case 0xB1:
            /* OR A,C */
            OR_R8_U8(&acc, c);
            break;
        case 0xB2:
            /* OR A,D */
            OR_R8_U8(&acc, d);
            break;
        case 0xB3:
            /* OR A,E */
            OR_R8_U8(&acc, e);
            break;
        case 0xB4:
            /* OR A,H */
            OR_R8_U8(&acc, h);
            break;
        case 0xB5:
            /* OR A,L */
            OR_R8_U8(&acc, l);
            break;
        case 0xB6:
            /* OR A,(HL) */
            OR_R8_U8(&acc, mmu.read(HILO(h, l)));
            break;
        case 0xB7:
            /* OR A,A */
            OR_R8_U8(&acc, acc);
            break;
        case 0xB8:
        case 0xB9:
        case 0xBA:
        case 0xBB:
        case 0xBC:
        case 0xBD:
        NOT_IMPLEMENTED
        case 0xBE:
            /* CP A,(HL) */
            scratch8 = mmu.read(HILO(h, l));
            flag_z = acc == scratch8 ? 1 : 0;
            flag_h = CHECK_HALF_CARRY_DEC(acc, scratch8);
            flag_c = CHECK_CARRY_DEC(acc, scratch8);
            flag_n = 1;
            break;
        case 0xBF:
        case 0xC0:
            /* RET NZ */
            if (flag_z == 0) {
                RET(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
            }
            break;
        case 0xC1:
            /* POP BC */
            c = mmu.read(sp++);
            b = mmu.read(sp++);
            break;
        case 0xC2:
        NOT_IMPLEMENTED
        case 0xC3:
            /* JP u16 */
            scratch8 = fetch_pc(mmu);
            pc = HILO(fetch_pc(mmu), scratch8);
            break;
        case 0xC4:
            /* CALL NZ,u16 */
            if (flag_z == 0) {
                PUSH_R16(mmu, HI(pc), LO(pc));
                pc = HILO(mmu.read(pc+1), mmu.read(pc));
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
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
        NOT_IMPLEMENTED
        case 0xC8:
            /* RET Z */
            if (flag_z == 1) {
                RET(mmu);
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
            }
            break;
        case 0xC9:
            /* RET */
            RET(mmu);
            break;
        case 0xCA:
            /* JP Z,u16 */
            scratch8 = fetch_pc(mmu);
            scratch16 = HILO(fetch_pc(mmu), scratch8);
            if (flag_z == 1) {
                pc = scratch16;
                clocks += INSTR_CLOCKS_BRANCH[pc_val];
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
            }
            break;
        case 0xCB:
            prefix(mmu);
            clocks += 2;
            ppu.tick(2, mmu);
            break;
        case 0xCC:
        NOT_IMPLEMENTED
        case 0xCD:
            //printf("CALL\n");
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
                ppu.tick(INSTR_CLOCKS_BRANCH[pc_val], mmu);
            }
            break;
        case 0xD1:
            /* POP DE */
            POP_R16(mmu, &d, &e);
            break;
        case 0xD2:
        case 0xD3:
        case 0xD4:
            NOT_IMPLEMENTED
        case 0xD5:
            /* PUSH DE */
            PUSH_R16(mmu, d, e);
            break;
        case 0xD6:
            /* SUB A,u8 */
            SUB_R8_U8(&acc, fetch_pc(mmu));
            break;
        case 0xD7:
        case 0xD8:
        case 0xD9:
        case 0xDA:
        case 0xDB:
        case 0xDC:
        case 0xDD:
        case 0xDE:
        case 0xDF:
        NOT_IMPLEMENTED
        case 0xE0:
            /* LD (FF00+u8),A */
            mmu.write(0xFF00 + fetch_pc(mmu), acc);
            break;
        case 0xE1:
            /* POP HL */
            POP_R16(mmu, &h, &l);
            break;
        case 0xE2:
            mmu.write(0xFF00 + c, acc);
            break;
        case 0xE3:
        case 0xE4:
        NOT_IMPLEMENTED
        case 0xE5:
            /* PUSH HL */
            PUSH_R16(mmu, h, l);
            break;
        case 0xE6:
            /* AND A,u8 */
            AND_R8_U8(&acc, fetch_pc(mmu));
            break;
        case 0xE7:
        case 0xE8:
        NOT_IMPLEMENTED
        case 0xE9:
            /* JP HL */
            pc = HILO(h, l);
            break;
        case 0xEA:
            /* LD (a16), A */
            mmu.write(HILO(mmu.read(pc+1), mmu.read(pc)), acc);
            pc += 2;
            break;
        case 0xEB:
        case 0xEC:
        case 0xED:
        NOT_IMPLEMENTED
        case 0xEE:
            /* XOR A,u8 */
            XOR_R8_U8(&acc, fetch_pc(mmu));
            break;
        case 0xEF:
            /* RST 28h */
            RST(mmu, 0x28);
            break;
        case 0xF0:
            /* LD A,(FF00+u8) */
            acc = mmu.read(0xFF00 + fetch_pc(mmu));
            break;
        case 0xF1:
            /* POP AF */
            POP_R16(mmu, &acc, &scratch8);
            flag_c = BIT4(scratch8);
            flag_h = BIT5(scratch8);
            flag_n = BIT6(scratch8);
            flag_z = BIT7(scratch8);
            break;
        case 0xF2:
        NOT_IMPLEMENTED
        case 0xF3:
            /* DI */
            flag_ime = 0;
            break;
        case 0xF4:
        NOT_IMPLEMENTED
        case 0xF5:
            /* PUSH AF */
            PUSH_R16(mmu, acc, (flag_z << 7) | (flag_n << 6) | (flag_h << 5) | (flag_c << 4));
            break;
        case 0xF6:
        case 0xF7:
        case 0xF8:
        case 0xF9:
        NOT_IMPLEMENTED
        case 0xFA:
            /* LD A,(u16) */
            LD_R8_A16(mmu, &acc);
            break;
        case 0xFB:
            /* EI */
            flag_ime = 1;
            break;
        case 0xFC:
        case 0xFD:
        NOT_IMPLEMENTED
        case 0xFE:
            /* CP A,u8 */
            scratch8 = fetch_pc(mmu);
            flag_z = acc == scratch8 ? 1 : 0;
            flag_h = CHECK_HALF_CARRY_DEC(acc, scratch8);
            flag_c = CHECK_CARRY_DEC(acc, scratch8);
            flag_n = 1;
            break;
        case 0xFF:
        NOT_IMPLEMENTED
    }

    return clocks;
}

void cpu::prefix(mmu &mmu) {
    switch (fetch_pc(mmu)) {
        case 0x00:
            break;
        case 0x01:
            break;
        case 0x02:
            break;
        case 0x03:
            break;
        case 0x04:
            break;
        case 0x05:
            break;
        case 0x06:
            break;
        case 0x07:
            break;
        case 0x08:
            break;
        case 0x09:
            break;
        case 0x0A:
            break;
        case 0x0B:
            break;
        case 0x0C:
            break;
        case 0x0D:
            break;
        case 0x0E:
            break;
        case 0x0F:
            break;
        case 0x10:
            break;
        case 0x11:
            /* RL C */
            scratch8 = BIT7(c);
            c = (c << 1) | flag_c;
            flag_z = c == 0 ? 1 : 0;
            flag_c = scratch8;
            flag_n = 0;
            flag_h = 0;
            break;
        case 0x12:
            break;
        case 0x13:
            break;
        case 0x14:
            break;
        case 0x15:
            break;
        case 0x16:
            break;
        case 0x17:
            break;
        case 0x18:
            break;
        case 0x19:
            break;
        case 0x1A:
            break;
        case 0x1B:
            break;
        case 0x1C:
            break;
        case 0x1D:
            break;
        case 0x1E:
            break;
        case 0x1F:
            break;
        case 0x20:
            break;
        case 0x21:
            break;
        case 0x22:
            break;
        case 0x23:
            break;
        case 0x24:
            break;
        case 0x25:
            break;
        case 0x26:
            break;
        case 0x27:
            break;
        case 0x28:
            break;
        case 0x29:
            break;
        case 0x2A:
            break;
        case 0x2B:
            break;
        case 0x2C:
            break;
        case 0x2D:
            break;
        case 0x2E:
            break;
        case 0x2F:
            break;
        case 0x30:
            break;
        case 0x31:
            break;
        case 0x32:
            break;
        case 0x33:
            break;
        case 0x34:
            break;
        case 0x35:
            break;
        case 0x36:
            break;
        case 0x37:
            break;
        case 0x38:
            break;
        case 0x39:
            break;
        case 0x3A:
            break;
        case 0x3B:
            break;
        case 0x3C:
            break;
        case 0x3D:
            break;
        case 0x3E:
            break;
        case 0x3F:
            break;
        case 0x40:
            break;
        case 0x41:
            break;
        case 0x42:
            break;
        case 0x43:
            break;
        case 0x44:
            break;
        case 0x45:
            break;
        case 0x46:
            break;
        case 0x47:
            break;
        case 0x48:
            break;
        case 0x49:
            break;
        case 0x4A:
            break;
        case 0x4B:
            break;
        case 0x4C:
            break;
        case 0x4D:
            break;
        case 0x4E:
            break;
        case 0x4F:
            break;
        case 0x50:
            break;
        case 0x51:
            break;
        case 0x52:
            break;
        case 0x53:
            break;
        case 0x54:
            break;
        case 0x55:
            break;
        case 0x56:
            break;
        case 0x57:
            break;
        case 0x58:
            break;
        case 0x59:
            break;
        case 0x5A:
            break;
        case 0x5B:
            break;
        case 0x5C:
            break;
        case 0x5D:
            break;
        case 0x5E:
            break;
        case 0x5F:
            break;
        case 0x60:
            break;
        case 0x61:
            break;
        case 0x62:
            break;
        case 0x63:
            break;
        case 0x64:
            break;
        case 0x65:
            break;
        case 0x66:
            break;
        case 0x67:
            break;
        case 0x68:
            break;
        case 0x69:
            break;
        case 0x6A:
            break;
        case 0x6B:
            break;
        case 0x6C:
            break;
        case 0x6D:
            break;
        case 0x6E:
            break;
        case 0x6F:
            break;
        case 0x70:
            break;
        case 0x71:
            break;
        case 0x72:
            break;
        case 0x73:
            break;
        case 0x74:
            break;
        case 0x75:
            break;
        case 0x76:
            break;
        case 0x77:
            break;
        case 0x78:
            break;
        case 0x79:
            break;
        case 0x7A:
            break;
        case 0x7B:
            break;
        case 0x7C:
            flag_z = BIT7_COMPLEMENT(h);
            flag_h = 1;
            flag_n = 0;
            break;
        case 0x7D:
            break;
        case 0x7E:
            break;
        case 0x7F:
            break;
        case 0x80:
            break;
        case 0x81:
            break;
        case 0x82:
            break;
        case 0x83:
            break;
        case 0x84:
            break;
        case 0x85:
            break;
        case 0x86:
            break;
        case 0x87:
            break;
        case 0x88:
            break;
        case 0x89:
            break;
        case 0x8A:
            break;
        case 0x8B:
            break;
        case 0x8C:
            break;
        case 0x8D:
            break;
        case 0x8E:
            break;
        case 0x8F:
            break;
        case 0x90:
            break;
        case 0x91:
            break;
        case 0x92:
            break;
        case 0x93:
            break;
        case 0x94:
            break;
        case 0x95:
            break;
        case 0x96:
            break;
        case 0x97:
            break;
        case 0x98:
            break;
        case 0x99:
            break;
        case 0x9A:
            break;
        case 0x9B:
            break;
        case 0x9C:
            break;
        case 0x9D:
            break;
        case 0x9E:
            break;
        case 0x9F:
            break;
        case 0xA0:
            break;
        case 0xA1:
            break;
        case 0xA2:
            break;
        case 0xA3:
            break;
        case 0xA4:
            break;
        case 0xA5:
            break;
        case 0xA6:
            break;
        case 0xA7:
            break;
        case 0xA8:
            break;
        case 0xA9:
            break;
        case 0xAA:
            break;
        case 0xAB:
            break;
        case 0xAC:
            break;
        case 0xAD:
            break;
        case 0xAE:
            break;
        case 0xAF:

            break;
        case 0xB0:
            break;
        case 0xB1:
            break;
        case 0xB2:
            break;
        case 0xB3:
            break;
        case 0xB4:
            break;
        case 0xB5:
            break;
        case 0xB6:
            break;
        case 0xB7:
            break;
        case 0xB8:
            break;
        case 0xB9:
            break;
        case 0xBA:
            break;
        case 0xBB:
            break;
        case 0xBC:
            break;
        case 0xBD:
            break;
        case 0xBE:
            break;
        case 0xBF:
            break;
        case 0xC0:
            break;
        case 0xC1:
            break;
        case 0xC2:
            break;
        case 0xC3:
            break;
        case 0xC4:
            break;
        case 0xC5:
            break;
        case 0xC6:
            break;
        case 0xC7:
            break;
        case 0xC8:
            break;
        case 0xC9:
            break;
        case 0xCA:
            break;
        case 0xCB:

            break;
        case 0xCC:
            break;
        case 0xCD:
            break;
        case 0xCE:
            break;
        case 0xCF:
            break;
        case 0xD0:
            break;
        case 0xD1:
            break;
        case 0xD2:
            break;
        case 0xD3:
            break;
        case 0xD4:
            break;
        case 0xD5:
            break;
        case 0xD6:
            break;
        case 0xD7:
            break;
        case 0xD8:
            break;
        case 0xD9:
            break;
        case 0xDA:
            break;
        case 0xDB:
            break;
        case 0xDC:
            break;
        case 0xDD:
            break;
        case 0xDE:
            break;
        case 0xDF:
            break;
        case 0xE0:
            break;
        case 0xE1:
            break;
        case 0xE2:
            break;
        case 0xE3:
            break;
        case 0xE4:
            break;
        case 0xE5:
            break;
        case 0xE6:
            break;
        case 0xE7:
            break;
        case 0xE8:
            break;
        case 0xE9:
            break;
        case 0xEA:
            break;
        case 0xEB:
            break;
        case 0xEC:
            break;
        case 0xED:
            break;
        case 0xEE:
            break;
        case 0xEF:
            break;
        case 0xF0:
            break;
        case 0xF1:
            break;
        case 0xF2:
            break;
        case 0xF3:
            break;
        case 0xF4:
            break;
        case 0xF5:
            break;
        case 0xF6:
            break;
        case 0xF7:
            break;
        case 0xF8:
            break;
        case 0xF9:
            break;
        case 0xFA:
            break;
        case 0xFB:
            break;
        case 0xFC:
            break;
        case 0xFD:
            break;
        case 0xFE:
            break;
        case 0xFF:
            break;
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
