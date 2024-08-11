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
#define CHECK_CARRY(A, B) (uint8_t)( ( (((uint16_t)A) & 0xFF) + (((uint16_t)B) & 0xFF) ) >> 8)
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

#define INCu8(X) \
    flag_h = CHECK_HALF_CARRY((X), 1);\
    X ++;\
    flag_n = 0;\
    flag_z = (X) == 0 ? 1 : 0
#define DECu8(X) \
    flag_h = CHECK_HALF_CARRY((X), -1);\
    X --;\
    flag_n = 1;\
    flag_z = (X) == 0 ? 1 : 0
#define SUBu8(X, Y) \
    flag_h = CHECK_HALF_CARRY(X, -Y);\
    flag_c = CHECK_CARRY(X, -Y);\
    X -= Y;\
    flag_z = (X) == 0 ? 1 : 0
#define ADDu8(X, Y) \
    flag_h = CHECK_HALF_CARRY(X, Y);\
    flag_c = CHECK_CARRY(X, Y);\
    X += Y;\
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
    /* Store the value pointed to by the next two bytes into a register */
    uint8_t lower = fetch_pc(mmu);
    uint8_t upper = fetch_pc(mmu);
    (*x) = mmu.read(HILO((upper), (lower)));
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

inline void cpu::INC_R8(uint8_t *x) {
    flag_h = CHECK_HALF_CARRY((*x), 1);
    (*x)++;
    flag_n = 0;
    flag_z = (*x) == 0 ? 1 : 0;
}
inline void cpu::DEC_R8(uint8_t *x) {
    flag_h = CHECK_HALF_CARRY((*x), -1);
    (*x)--;
    flag_n = 1;
    flag_z = (*x) == 0 ? 1 : 0;
}

int cpu::tick(mmu &mmu, ppu &ppu) {
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
            flag_h, flag_n, flag_z = 0;
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
        case 0x0B:
        NOT_IMPLEMENTED
        case 0x0C:
            flag_h = CHECK_HALF_CARRY(c, 1);
            c ++;
            flag_z = c == 0 ? 1 : 0;
            flag_n = 0;
            break;
        case 0x0D:
            /* DEC C */
            DEC_R8(&c);
            break;
        case 0x0E:
            /* LD C,u8 */
            LD_R8_U8(mmu, &c);
            break;
        case 0x0F:
        NOT_IMPLEMENTED
        case 0x10:
            return -1;
        case 0x11:
            /* LD DE,u16 */
            e = fetch_pc(mmu);
            d = fetch_pc(mmu);
            break;
        case 0x12:
        NOT_IMPLEMENTED
        case 0x13:
            /* INC DE */
            scratch16 = HILO(d, e);
            d = HI(scratch16+1);
            e = LO(scratch16+1);
            break;
        case 0x14:
        NOT_IMPLEMENTED
        case 0x15:
            /* DEC D */
            DEC_R8(&d);
            break;
        case 0x16:
            /* LD D,u8 */
            LD_R8_U8(mmu, &d);
            break;
        case 0x17:
            /* RLA */
            //printf("RLA\n");
            //printf("A: %08b \n", acc);
            scratch8 = BIT7(acc);
            acc = (acc << 1) | flag_c;
            flag_c = scratch8;
            flag_z, flag_h, flag_n = 0;
            //printf("A: %08b flag_c: %01b\n", acc, flag_c);
            break;
        case 0x18:
            /* JR i8 */
            scratch8 = fetch_pc(mmu);
            pc += (int8_t) scratch8;
            break;
        case 0x19:
        NOT_IMPLEMENTED
        case 0x1A:
            /* LD A,(DE) */
            LD_R8_AR16(mmu, &acc, d, e);
            break;
        case 0x1B:
        case 0x1C:
        NOT_IMPLEMENTED
        case 0x1D:
            /* DEC E */
            DEC_R8(&e);
            break;
        case 0x1E:
            /* LD E,u8 */
            LD_R8_U8(mmu, &e);
            break;
        case 0x1F:
        NOT_IMPLEMENTED
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
        case 0x26:
        case 0x27:
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
        NOT_IMPLEMENTED
        case 0x2A:
            /* LD A,(HL+) */
            LD_A_HLP(mmu, acc, h, l);
            break;
        case 0x2B:
        case 0x2C:
        case 0x2D:
        NOT_IMPLEMENTED
        case 0x2E:
            /* LD L,u8 */
            LD_R8_U8(mmu, &l);
            break;
        case 0x2F:
        case 0x30:
        NOT_IMPLEMENTED
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
        NOT_IMPLEMENTED
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
            flag_h = CHECK_HALF_CARRY(acc, -1);
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
        case 0x46:
        case 0x47:
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0x4B:
        case 0x4C:
        case 0x4D:
        case 0x4E:
        NOT_IMPLEMENTED
        case 0x4F:
            /* LD C,A */
            LD_R8_R8(c, acc);
            break;
        case 0x50:
        case 0x51:
        case 0x52:
        case 0x53:
        case 0x54:
        case 0x55:
        case 0x56:
        NOT_IMPLEMENTED
        case 0x57:
            /* LD D,A */
            LD_R8_R8(d, acc);
            break;
        case 0x58:
        case 0x59:
        case 0x5A:
        case 0x5B:
        case 0x5C:
        case 0x5D:
        case 0x5E:
        case 0x5F:
        case 0x60:
        case 0x61:
        case 0x62:
        case 0x63:
        case 0x64:
        case 0x65:
        case 0x66:
        NOT_IMPLEMENTED
        case 0x67:
            /* LD H,A */
            LD_R8_R8(h, acc);
            break;
        case 0x68:
        case 0x69:
        case 0x6A:
        case 0x6B:
        case 0x6C:
        case 0x6D:
        case 0x6E:
        case 0x6F:
        case 0x70:
        case 0x71:
        case 0x72:
        case 0x73:
        case 0x74:
        case 0x75:
        case 0x76:
        NOT_IMPLEMENTED
        case 0x77:
            /* LD (HL),A */
            LD_AR16_A(mmu, h, l, acc);
            break;
        case 0x78:
            /* LD A,B */
            LD_R8_R8(acc, b);
            break;
        case 0x79:
        case 0x7A:
        NOT_IMPLEMENTED
        case 0x7B:
            /* LD A, E */
            LD_R8_R8(acc, e);
            break;
        case 0x7C:
            /* LD A,H */
            LD_R8_R8(acc, h);
            break;
        case 0x7D:
            /* LD A,L */
            LD_R8_R8(acc, l);
            break;
        case 0x7E:
        case 0x7F:
        case 0x80:
        case 0x81:
        case 0x82:
        case 0x83:
        case 0x84:
        case 0x85:
            NOT_IMPLEMENTED
        case 0x86:
            /* ADD A,(HL) */
            scratch8 = mmu.read(HILO(h, l));
            ADDu8(acc, scratch8);
            break;
        case 0x87:
        case 0x88:
        case 0x89:
        case 0x8A:
        case 0x8B:
        case 0x8C:
        case 0x8D:
        case 0x8E:
        case 0x8F:
            NOT_IMPLEMENTED
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
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8:
        case 0xA9:
        case 0xAA:
        case 0xAB:
        case 0xAC:
        case 0xAD:
        case 0xAE:
            NOT_IMPLEMENTED
        case 0xAF:
            acc = 0;
            flag_z = 1;
            flag_c, flag_h, flag_n = 0;
            break;
        case 0xB0:
        case 0xB1:
        case 0xB2:
        case 0xB3:
        case 0xB4:
        case 0xB5:
        case 0xB6:
        case 0xB7:
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
            flag_h = CHECK_HALF_CARRY(acc, -scratch8);
            flag_c = CHECK_CARRY(acc, -scratch8);
            flag_n = 1;
            break;
        case 0xBF:
        case 0xC0:
        NOT_IMPLEMENTED
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
        NOT_IMPLEMENTED
        case 0xC5:
            /* PUSH BC */
            mmu.write(--sp, b);
            mmu.write(--sp, c);
            break;
        case 0xC6:
        case 0xC7:
        case 0xC8:
        NOT_IMPLEMENTED
        case 0xC9:
            /* RET */
            pc = HILO(mmu.read(sp+1), mmu.read(sp));
            sp += 2;
            //printf("RET: %04x\n", pc);
            break;
        case 0xCA:
        NOT_IMPLEMENTED
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
        case 0xCF:
        case 0xD0:
        case 0xD1:
        case 0xD2:
        case 0xD3:
        case 0xD4:
        case 0xD5:
        case 0xD6:
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
        NOT_IMPLEMENTED
        case 0xE2:
            mmu.write(0xFF00 + c, acc);
            break;
        case 0xE3:
        case 0xE4:
        case 0xE5:
        case 0xE6:
        case 0xE7:
        case 0xE8:
        case 0xE9:
        NOT_IMPLEMENTED
        case 0xEA:
            /* LD (a16), A */
            mmu.write(HILO(mmu.read(pc+1), mmu.read(pc)), acc);
            pc += 2;
            break;
        case 0xEB:
        case 0xEC:
        case 0xED:
        case 0xEE:
        case 0xEF:
        NOT_IMPLEMENTED
        case 0xF0:
            /* LD A,(FF00+u8) */
            acc = mmu.read(0xFF00 + fetch_pc(mmu));
            break;
        case 0xF1:
        case 0xF2:
        NOT_IMPLEMENTED
        case 0xF3:
            /* DI */
            flag_ime = 0;
            break;
        case 0xF4:
        case 0xF5:
        case 0xF6:
        case 0xF7:
        case 0xF8:
        case 0xF9:
        case 0xFA:
        case 0xFB:
            /* EI */

        case 0xFC:
        case 0xFD:
        NOT_IMPLEMENTED
        case 0xFE:
            /* CP A,u8 */
            scratch8 = fetch_pc(mmu);
            //printf("CP A,u8 -> %02x,%02x\n", acc, scratch8);
            flag_z = acc == scratch8 ? 1 : 0;
            //printf("acc %02x u8 %02x\n", acc, scratch8);
            flag_h = CHECK_HALF_CARRY(acc, -scratch8);
            flag_c = CHECK_CARRY(acc, -scratch8);
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