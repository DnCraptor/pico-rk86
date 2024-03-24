// Intel 8080 (KR580VM80A) microprocessor core model
//
// Copyright (C) 2012 Alexander Demin <alexander@demin.ws>
//
// Credits
//
// Viacheslav Slavinsky, Vector-06C FPGA Replica
// http://code.google.com/p/vector06cc/
//
// Dmitry Tselikov, Bashrikia-2M and Radio-86RK on Altera DE1
// http://bashkiria-2m.narod.ru/fpga.html
//
// Ian Bartholomew, 8080/8085 CPU Exerciser
// http://www.idb.me.uk/sunhillow/8080.html
//
// Frank Cringle, The original exerciser for the Z80.
//
// Thanks to zx.pk.ru and nedopc.org/forum communities.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

#include "i8080.h"
#include "i8080_hal.h"
#include <inttypes.h>
#include <ets.h>

#ifdef SUPPRES_DISASM
#undef printf
#define printf(...)
#endif

#define RD_BYTE(addr) i8080_hal_memory_read_byte(addr)
#define RD_WORD(addr) i8080_hal_memory_read_word(addr)

#define WR_BYTE(addr, value) i8080_hal_memory_write_byte(addr, value)
#define WR_WORD(addr, value) i8080_hal_memory_write_word(addr, value)

typedef unsigned char           uns8;
typedef unsigned short          uns16;
typedef unsigned long int       uns32;
typedef signed char             sgn8;
typedef signed short            sgn16;
typedef signed long int         sgn32;

typedef union {
    struct {
        uns8 l, h;
    } b;
    uns16 w;
} reg_pair;

typedef struct {
    uns8 carry_flag;
    uns8 unused1;
    uns8 parity_flag;
    uns8 unused3;
    uns8 half_carry_flag;
    uns8 unused5;
    uns8 zero_flag;
    uns8 sign_flag;
} flag_reg;

struct i8080 {
    flag_reg f;
    reg_pair af, bc, de, hl;
    reg_pair sp, pc;
    uns16 iff;
    uns16 last_pc;
};

#define FLAGS           cpu.f
#define AF              cpu.af.w
#define BC              cpu.bc.w
#define DE              cpu.de.w
#define HL              cpu.hl.w
#define SP              cpu.sp.w
#define PC              cpu.pc.w
#define A               cpu.af.b.h
#define F               cpu.af.b.l
#define B               cpu.bc.b.h
#define C               cpu.bc.b.l
#define D               cpu.de.b.h
#define E               cpu.de.b.l
#define H               cpu.hl.b.h
#define L               cpu.hl.b.l
#define HSP             cpu.sp.b.h
#define LSP             cpu.sp.b.l
#define HPC             cpu.pc.b.h
#define LPC             cpu.pc.b.l
#define IFF             cpu.iff

#define F_CARRY         0x01
#define F_UN1           0x02
#define F_PARITY        0x04
#define F_UN3           0x08
#define F_HCARRY        0x10
#define F_UN5           0x20
#define F_ZERO          0x40
#define F_NEG           0x80

#define C_FLAG          FLAGS.carry_flag
#define P_FLAG          FLAGS.parity_flag
#define H_FLAG          FLAGS.half_carry_flag
#define Z_FLAG          FLAGS.zero_flag
#define S_FLAG          FLAGS.sign_flag
#define UN1_FLAG        FLAGS.unused1
#define UN3_FLAG        FLAGS.unused3
#define UN5_FLAG        FLAGS.unused5

#define SET(flag)       (flag = 1)
#define CLR(flag)       (flag = 0)
#define TST(flag)       (flag)
#define CPL(flag)       (flag = !flag)

#define POP(reg)        { (reg) = RD_WORD(SP); SP += 2; }
#define PUSH(reg)       { SP -= 2; WR_WORD(SP, (reg)); }
#define RET()           { POP(PC); }
#define STC()           { SET(C_FLAG); }
#define CMC()           { CPL(C_FLAG); }

#define INR(reg) \
{                                               \
    ++(reg);                                    \
    S_FLAG = (((reg) & 0x80) != 0);             \
    Z_FLAG = ((reg) == 0);                      \
    H_FLAG = (((reg) & 0x0f) == 0);             \
    P_FLAG = PARITY(reg);                       \
}

#define DCR(reg) \
{                                               \
    --(reg);                                    \
    S_FLAG = (((reg) & 0x80) != 0);             \
    Z_FLAG = ((reg) == 0);                      \
    H_FLAG = !(((reg) & 0x0f) == 0x0f);         \
    P_FLAG = PARITY(reg);                       \
}

#define ADD(val) \
{                                               \
    work16 = (uns16)A + (val);                  \
    _index = ((A & 0x88) >> 1) |                 \
            (((val) & 0x88) >> 2) |             \
            ((work16 & 0x88) >> 3);             \
    A = work16 & 0xff;                          \
    S_FLAG = ((A & 0x80) != 0);                 \
    Z_FLAG = (A == 0);                          \
    H_FLAG = half_carry_table[_index & 0x7];     \
    P_FLAG = PARITY(A);                         \
    C_FLAG = ((work16 & 0x0100) != 0);          \
}

#define ADC(val) \
{                                               \
    work16 = (uns16)A + (val) + C_FLAG;         \
    _index = ((A & 0x88) >> 1) |                 \
            (((val) & 0x88) >> 2) |             \
            ((work16 & 0x88) >> 3);             \
    A = work16 & 0xff;                          \
    S_FLAG = ((A & 0x80) != 0);                 \
    Z_FLAG = (A == 0);                          \
    H_FLAG = half_carry_table[_index & 0x7];     \
    P_FLAG = PARITY(A);                         \
    C_FLAG = ((work16 & 0x0100) != 0);          \
}

#define SUB(val) \
{                                                \
    work16 = (uns16)A - (val);                   \
    _index = ((A & 0x88) >> 1) |                  \
            (((val) & 0x88) >> 2) |              \
            ((work16 & 0x88) >> 3);              \
    A = work16 & 0xff;                           \
    S_FLAG = ((A & 0x80) != 0);                  \
    Z_FLAG = (A == 0);                           \
    H_FLAG = !sub_half_carry_table[_index & 0x7]; \
    P_FLAG = PARITY(A);                          \
    C_FLAG = ((work16 & 0x0100) != 0);           \
}

#define SBB(val) \
{                                                \
    work16 = (uns16)A - (val) - C_FLAG;          \
    _index = ((A & 0x88) >> 1) |                  \
            (((val) & 0x88) >> 2) |              \
            ((work16 & 0x88) >> 3);              \
    A = work16 & 0xff;                           \
    S_FLAG = ((A & 0x80) != 0);                  \
    Z_FLAG = (A == 0);                           \
    H_FLAG = !sub_half_carry_table[_index & 0x7]; \
    P_FLAG = PARITY(A);                          \
    C_FLAG = ((work16 & 0x0100) != 0);           \
}

#define CMP(val) \
{                                                \
    work16 = (uns16)A - (val);                   \
    _index = ((A & 0x88) >> 1) |                  \
            (((val) & 0x88) >> 2) |              \
            ((work16 & 0x88) >> 3);              \
    S_FLAG = ((work16 & 0x80) != 0);             \
    Z_FLAG = ((work16 & 0xff) == 0);             \
    H_FLAG = !sub_half_carry_table[_index & 0x7]; \
    C_FLAG = ((work16 & 0x0100) != 0);           \
    P_FLAG = PARITY(work16 & 0xff);              \
}

#define ANA(val) \
{                                               \
    H_FLAG = ((A | val) & 0x08) != 0;           \
    A &= (val);                                 \
    S_FLAG = ((A & 0x80) != 0);                 \
    Z_FLAG = (A == 0);                          \
    P_FLAG = PARITY(A);                         \
    CLR(C_FLAG);                                \
}

#define XRA(val) \
{                                               \
    A ^= (val);                                 \
    S_FLAG = ((A & 0x80) != 0);                 \
    Z_FLAG = (A == 0);                          \
    CLR(H_FLAG);                                \
    P_FLAG = PARITY(A);                         \
    CLR(C_FLAG);                                \
}

#define ORA(val) \
{                                               \
    A |= (val);                                 \
    S_FLAG = ((A & 0x80) != 0);                 \
    Z_FLAG = (A == 0);                          \
    CLR(H_FLAG);                                \
    P_FLAG = PARITY(A);                         \
    CLR(C_FLAG);                                \
}

#define DAD(reg) \
{                                               \
    work32 = (uns32)HL + (reg);                 \
    HL = work32 & 0xffff;                       \
    C_FLAG = ((work32 & 0x10000L) != 0);        \
}

#define CALL \
{                                               \
    PUSH(PC + 2);                               \
    PC = RD_WORD(PC);                           \
}

#define RST(addr) \
{                                               \
    PUSH(PC);                                   \
    PC = (addr);                                \
}

#define PARITY(reg) parity_table[(reg)]

static struct i8080 cpu;

static uns32 work32;
static uns16 work16;
static uns8 work8;
static int _index;
static uns8 carry, add;

const uint8_t parity_table[] = {
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
};

const uint8_t half_carry_table[] = { 0, 0, 1, 0, 1, 0, 1, 1 };
const uint8_t sub_half_carry_table[] = { 0, 1, 1, 1, 0, 0, 0, 1 };

void i8080_init(void) {
    C_FLAG = 0;
    S_FLAG = 0;
    Z_FLAG = 0;
    H_FLAG = 0;
    P_FLAG = 0;
    UN1_FLAG = 1;
    UN3_FLAG = 0;
    UN5_FLAG = 0;

    PC = 0xF800;
}


int i8080_state_size(void)
{
    return sizeof(cpu);
}


void i8080_state_save(unsigned char *buf)
{
    ets_memcpy(buf, &cpu, sizeof(cpu));
}


void i8080_state_load(const unsigned char *buf)
{
    ets_memcpy(&cpu, buf, sizeof(cpu));
}


static void i8080_store_flags(void) {
    if (S_FLAG) F |= F_NEG;      else F &= ~F_NEG;
    if (Z_FLAG) F |= F_ZERO;     else F &= ~F_ZERO;
    if (H_FLAG) F |= F_HCARRY;   else F &= ~F_HCARRY;
    if (P_FLAG) F |= F_PARITY;   else F &= ~F_PARITY;
    if (C_FLAG) F |= F_CARRY;    else F &= ~F_CARRY;
    F |= F_UN1;    // UN1_FLAG is always 1.
    F &= ~F_UN3;   // UN3_FLAG is always 0.
    F &= ~F_UN5;   // UN5_FLAG is always 0.
}

static void i8080_retrieve_flags(void) {
    S_FLAG = F & F_NEG      ? 1 : 0;
    Z_FLAG = F & F_ZERO     ? 1 : 0;
    H_FLAG = F & F_HCARRY   ? 1 : 0;
    P_FLAG = F & F_PARITY   ? 1 : 0;
    C_FLAG = F & F_CARRY    ? 1 : 0;
}

static int i8080_execute(uint8_t opcode) {
    int cpu_cycles;
    uint16_t PC_REF = PC - 1;
    work8 = A;
    switch (opcode) {
        case 0x00:            /* nop */
        // Undocumented NOP.
        case 0x08:            /* nop */
        case 0x10:            /* nop */
        case 0x18:            /* nop */
        case 0x20:            /* nop */
        case 0x28:            /* nop */
        case 0x30:            /* nop */
        case 0x38:            /* nop */
            cpu_cycles = 4;
            printf("%04X: %02X NOP (4)", PC_REF, opcode);
            break;
        case 0x01:            /* lxi b, data16 */
            cpu_cycles = 10;
            BC = RD_WORD(PC);
            printf("%04X: %02X LXI B, %04Xh @ %04Xh+2 (10)", PC_REF, opcode, BC, PC);
            PC += 2;
            break;
        case 0x02:            /* stax b */
            cpu_cycles = 7;
            WR_BYTE(BC, A);
            printf("%04X: %02X STAX B(%04Xh) (A: %02Xh) (7)", PC_REF, opcode, BC, A);
            break;
        case 0x03:            /* inx b */
            cpu_cycles = 5;
            BC++;
            printf("%04X: %02X INX B(%04Xh) (5)", PC_REF, opcode, BC);
            break;
        case 0x04:            /* inr b */
            cpu_cycles = 5;
            INR(B);
            printf("%04X: %02X INR B(%04Xh) (5)", PC_REF, opcode, BC);
            break;
        case 0x05:            /* dcr b */
            cpu_cycles = 5;
            DCR(B);
            printf("%04X: %02X DCR B(%04Xh) (5)", PC_REF, opcode, BC);
            break;
        case 0x06:            /* mvi b, data8 */
            cpu_cycles = 7;
            B = RD_BYTE(PC++);
            printf("%04X: %02X MVI B, %02Xh (7)", PC_REF, opcode, B);
            break;
        case 0x07:            /* rlc */
            cpu_cycles = 4;
            C_FLAG = ((A & 0x80) != 0);
            A = (A << 1) | C_FLAG;
            printf("%04X: %02X RLC A(%02Xh) (4)", PC_REF, opcode, A);
            break;
        case 0x09:            /* dad b */
            cpu_cycles = 10;
            DAD(BC);
            printf("%04X: %02X DAD BC(%04Xh) (10)", PC_REF, opcode, BC);
            break;
        case 0x0A:            /* ldax b */
            cpu_cycles = 7;
            A = RD_BYTE(BC);
            printf("%04X: %02X LDAX BC(%04Xh) A(%02Xh) (7)", PC_REF, opcode, BC, A);
            break;
        case 0x0B:            /* dcx b */
            cpu_cycles = 5;
            BC--;
            printf("%04X: %02X DCX BC(%04Xh) (5)", PC_REF, opcode, BC);
            break;
        case 0x0C:            /* inr c */
            cpu_cycles = 5;
            INR(C);
            printf("%04X: %02X INR C(%02Xh) (5)", PC_REF, opcode, C);
            break;
        case 0x0D:            /* dcr c */
            cpu_cycles = 5;
            DCR(C);
            printf("%04X: %02X DCR C(%02Xh) (5)", PC_REF, opcode, C);
            break;
        case 0x0E:            /* mvi c, data8 */
            cpu_cycles = 7;
            C = RD_BYTE(PC++);
            printf("%04X: %02X MVI C(%02Xh) (7)", PC_REF, opcode, C);
            break;
        case 0x0F:            /* rrc */
            cpu_cycles = 4;
            C_FLAG = A & 0x01;
            A = (A >> 1) | (C_FLAG << 7);
            printf("%04X: %02X RRC A(%02Xh) (4)", PC_REF, opcode, A);
            break;
        case 0x11:            /* lxi d, data16 */
            cpu_cycles = 10;
            DE = RD_WORD(PC);
            PC += 2;
            printf("%04X: %02X LXI DE(%04Xh) (10)", PC_REF, opcode, DE);
            break;
        case 0x12:            /* stax d */
            cpu_cycles = 7;
            WR_BYTE(DE, A);
            printf("%04X: %02X STAX DE(%04Xh) (7)", PC_REF, opcode, DE);
            break;
        case 0x13:            /* inx d */
            cpu_cycles = 5;
            DE++;
            printf("%04X: %02X INX DE(%04Xh) (5)", PC_REF, opcode, DE);
            break;
        case 0x14:            /* inr d */
            cpu_cycles = 5;
            INR(D);
            printf("%04X: %02X INR D(%02Xh) (5)", PC_REF, opcode, D);
            break;
        case 0x15:            /* dcr d */
            cpu_cycles = 5;
            DCR(D);
            printf("%04X: %02X DCR D(%02Xh) (5)", PC_REF, opcode, D);
            break;
        case 0x16:            /* mvi d, data8 */
            cpu_cycles = 7;
            D = RD_BYTE(PC++);
            printf("%04X: %02X MVI D(%02Xh) (7)", PC_REF, opcode, D);
            break;
        case 0x17:            /* ral */
            cpu_cycles = 4;
            work8 = (uns8)C_FLAG;
            C_FLAG = ((A & 0x80) != 0);
            A = (A << 1) | work8;
            printf("%04X: %02X RAL A(%02Xh) (4)", PC_REF, opcode, A);
            break;
        case 0x19:            /* dad d */
            cpu_cycles = 10;
            DAD(DE);
            printf("%04X: %02X DAD DE(%04Xh) (10)", PC_REF, opcode, DE);
            break;
        case 0x1A:            /* ldax d */
            cpu_cycles = 7;
            A = RD_BYTE(DE);
            printf("%04X: %02X LDAX A(%02Xh) DE(%04Xh) (7)", PC_REF, opcode, A, DE);
            break;
        case 0x1B:            /* dcx d */
            cpu_cycles = 5;
            DE--;
            printf("%04X: %02X DCX DE(%04Xh) (5)", PC_REF, opcode, DE);
            break;
        case 0x1C:            /* inr e */
            cpu_cycles = 5;
            INR(E);
            printf("%04X: %02X INR E(%02Xh) (5)", PC_REF, opcode, E);
            break;
        case 0x1D:            /* dcr e */
            cpu_cycles = 5;
            DCR(E);
            printf("%04X: %02X DCR E(%02Xh) (5)", PC_REF, opcode, E);
            break;
        case 0x1E:            /* mvi e, data8 */
            cpu_cycles = 7;
            E = RD_BYTE(PC++);
            printf("%04X: %02X MVI E(%02Xh) (7)", PC_REF, opcode, E);
            break;
        case 0x1F:             /* rar */
            cpu_cycles = 4;
            work8 = (uns8)C_FLAG;
            C_FLAG = A & 0x01;
            A = (A >> 1) | (work8 << 7);
            printf("%04X: %02X RAR A(%02Xh) (4)", PC_REF, opcode, A);
            break;
        case 0x21:             /* lxi h, data16 */
            cpu_cycles = 10;
            HL = RD_WORD(PC);
            PC += 2;
            printf("%04X: %02X LXI HL(%04Xh) (10)", PC_REF, opcode, HL);
            break;
        case 0x22:            /* shld addr */
            cpu_cycles = 16;
            WR_WORD(RD_WORD(PC), HL);
            PC += 2;
            printf("%04X: %02X SHLD HL(%04Xh) (16)", PC_REF, opcode, HL);
            break;
        case 0x23:            /* inx h */
            cpu_cycles = 5;
            HL++;
            printf("%04X: %02X INX HL(%04Xh) (5)", PC_REF, opcode, HL);
            break;
        case 0x24:            /* inr h */
            cpu_cycles = 5;
            INR(H);
            printf("%04X: %02X INR H(%02Xh) (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x25:            /* dcr h */
            cpu_cycles = 5;
            DCR(H);
            printf("%04X: %02X DCR H(%02Xh) (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x26:            /* mvi h, data8 */
            cpu_cycles = 7;
            H = RD_BYTE(PC++);
            printf("%04X: %02X MVI H(%02Xh) (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x27:            /* daa */
            cpu_cycles = 4;
            carry = (uns8)C_FLAG;
            add = 0;
            if (H_FLAG || (A & 0x0f) > 9) {
                add = 0x06;
            }
            if (C_FLAG || (A >> 4) > 9 || ((A >> 4) >= 9 && (A & 0x0f) > 9)) {
                add |= 0x60;
                carry = 1;
            }
            ADD(add);
            P_FLAG = PARITY(A);
            C_FLAG = carry;
            printf("%04X: %02X DAA A(%02Xh) Pf: %d Cf: %d (%d)", PC_REF, opcode, A, P_FLAG, C_FLAG, cpu_cycles);
            break;
        case 0x29:            /* dad hl */
            cpu_cycles = 10;
            DAD(HL);
            printf("%04X: %02X DAD H(%04Xh) (%d)", PC_REF, opcode, HL, cpu_cycles);
            break;
        case 0x2A:            /* ldhl addr */
            cpu_cycles = 16;
            HL = RD_WORD(RD_WORD(PC));
            PC += 2;
            printf("%04X: %02X LDHL H(%04Xh) (%d)", PC_REF, opcode, HL, cpu_cycles);
            break;
        case 0x2B:            /* dcx h */
            cpu_cycles = 5;
            HL--;
            printf("%04X: %02X DCX H(%04Xh) (%d)", PC_REF, opcode, HL, cpu_cycles);
            break;
        case 0x2C:            /* inr l */
            cpu_cycles = 5;
            INR(L);
            printf("%04X: %02X INR L(%02Xh) (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x2D:            /* dcr l */
            cpu_cycles = 5;
            DCR(L);
            printf("%04X: %02X DCR L(%02Xh) (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x2E:            /* mvi l, data8 */
            cpu_cycles = 7;
            L = RD_BYTE(PC++);
            printf("%04X: %02X MVI L(%02Xh) (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x2F:            /* cma */
            cpu_cycles = 4;
            A ^= 0xff;
            printf("%04X: %02X CMA A(%02Xh) (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x31:            /* lxi sp, data16 */
            cpu_cycles = 10;
            SP = RD_WORD(PC);
            PC += 2;
            printf("%04X: %02X LXI SP(%04Xh) (%d)", PC_REF, opcode, SP, cpu_cycles);
            break;
        case 0x32:            /* sta addr */
            cpu_cycles = 13;
            WR_BYTE(RD_WORD(PC), A);
            PC += 2;
            printf("%04X: %02X STA A(%02Xh) (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x33:            /* inx sp */
            cpu_cycles = 5;
            SP++;
            printf("%04X: %02X INX SP(%04Xh) (%d)", PC_REF, opcode, SP, cpu_cycles);
            break;
        case 0x34:            /* inr m */
            cpu_cycles = 10;
            work8 = RD_BYTE(HL);
            INR(work8);
            WR_BYTE(HL, work8);
            printf("%04X: %02X INR HL(%04Xh)(%04Xh) (%d)", PC_REF, opcode, HL, work8, cpu_cycles);
            break;
        case 0x35:            /* dcr m */
            cpu_cycles = 10;
            work8 = RD_BYTE(HL);
            DCR(work8);
            WR_BYTE(HL, work8);
            printf("%04X: %02X DCR HL(%04Xh)(%04Xh) (%d)", PC_REF, opcode, HL, work8, cpu_cycles);
            break;
        case 0x36:            /* mvi m, data8 */
            cpu_cycles = 10;
            work8 = RD_BYTE(PC++);
            WR_BYTE(HL, work8);
            printf("%04X: %02X MVI HL(%04Xh)(%04Xh) (%d)", PC_REF, opcode, HL, work8, cpu_cycles);
            break;
        case 0x37:            /* stc */
            cpu_cycles = 4;
            SET(C_FLAG);
            printf("%04X: %02X STC Cf(%d) (%d)", PC_REF, opcode, C_FLAG, cpu_cycles);
            break;
        case 0x39:            /* dad sp */
            cpu_cycles = 10;
            DAD(SP);
            printf("%04X: %02X DAD SP(%04Xh) (%d)", PC_REF, opcode, SP, cpu_cycles);
            break;
        case 0x3A:            /* lda addr */
            cpu_cycles = 13;
            A = RD_BYTE(RD_WORD(PC));
            PC += 2;
            printf("%04X: %02X LDA A(%02Xh) (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x3B:            /* dcx sp */
            cpu_cycles = 5;
            SP--;
            printf("%04X: %02X DCX SP(%04Xh) (%d)", PC_REF, opcode, SP, cpu_cycles);
            break;
        case 0x3C:            /* inr a */
            cpu_cycles = 5;
            INR(A);
            printf("%04X: %02X INR A(%02Xh) (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x3D:            /* dcr a */
            cpu_cycles = 5;
            DCR(A);
            printf("%04X: %02X DCR A(%02Xh) (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x3E:            /* mvi a, data8 */
            cpu_cycles = 7;
            A = RD_BYTE(PC++);
            printf("%04X: %02X MVI A(%02Xh) (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x3F:            /* cmc */
            cpu_cycles = 4;
            CPL(C_FLAG);
            printf("%04X: %02X CPL Cf(%d) (%d)", PC_REF, opcode, C_FLAG, cpu_cycles);
            break;
        case 0x40:            /* mov b, b */
            cpu_cycles = 4;
            printf("%04X: %02X MOV B, B (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0x41:            /* mov b, c */
            cpu_cycles = 5;
            B = C;
            printf("%04X: %02X MOV B(%02Xh), C (%d)", PC_REF, opcode, B, cpu_cycles);
            break;
        case 0x42:            /* mov b, d */
            cpu_cycles = 5;
            B = D;
            printf("%04X: %02X MOV B(%02Xh), D (%d)", PC_REF, opcode, B, cpu_cycles);
            break;
        case 0x43:            /* mov b, e */
            cpu_cycles = 5;
            B = E;
            printf("%04X: %02X MOV B(%02Xh), E (%d)", PC_REF, opcode, B, cpu_cycles);
            break;
        case 0x44:            /* mov b, h */
            cpu_cycles = 5;
            B = H;
            printf("%04X: %02X MOV B(%02Xh), H (%d)", PC_REF, opcode, B, cpu_cycles);
            break;
        case 0x45:            /* mov b, l */
            cpu_cycles = 5;
            B = L;
            printf("%04X: %02X MOV B(%02Xh), L (%d)", PC_REF, opcode, B, cpu_cycles);
            break;
        case 0x46:            /* mov b, m */
            cpu_cycles = 7;
            B = RD_BYTE(HL);
            printf("%04X: %02X MOV B(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, B, HL, cpu_cycles);
            break;
        case 0x47:            /* mov b, a */
            cpu_cycles = 5;
            B = A;
            printf("%04X: %02X MOV B(%02Xh), A (%d)", PC_REF, opcode, B, cpu_cycles);
            break;
        case 0x48:            /* mov c, b */
            cpu_cycles = 5;
            C = B;
            printf("%04X: %02X MOV C(%02Xh), B (%d)", PC_REF, opcode, C, cpu_cycles);
            break;
        case 0x49:            /* mov c, c */
            cpu_cycles = 5;
            printf("%04X: %02X MOV C(%02Xh), C (%d)", PC_REF, opcode, C, cpu_cycles);
            break;
        case 0x4A:            /* mov c, d */
            cpu_cycles = 5;
            C = D;
            printf("%04X: %02X MOV C(%02Xh), D (%d)", PC_REF, opcode, C, cpu_cycles);
            break;
        case 0x4B:            /* mov c, e */
            cpu_cycles = 5;
            C = E;
            printf("%04X: %02X MOV C(%02Xh), E (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x4C:            /* mov c, h */
            cpu_cycles = 5;
            C = H;
            printf("%04X: %02X MOV C(%02Xh), H (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x4D:            /* mov c, l */
            cpu_cycles = 5;
            C = L;
            printf("%04X: %02X MOV C(%02Xh), L (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x4E:            /* mov c, m */
            cpu_cycles = 7;
            C = RD_BYTE(HL);
            printf("%04X: %02X MOV C(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, C, HL, cpu_cycles);
            break;
        case 0x4F:            /* mov c, a */
            cpu_cycles = 5;
            C = A;
            printf("%04X: %02X MOV C(%02Xh), A (%d)", PC_REF, opcode, C, cpu_cycles);
            break;
        case 0x50:            /* mov d, b */
            cpu_cycles = 5;
            D = B;
            printf("%04X: %02X MOV D(%02Xh), B (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x51:            /* mov d, c */
            cpu_cycles = 5;
            D = C;
            printf("%04X: %02X MOV D(%02Xh), C (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x52:            /* mov d, d */
            cpu_cycles = 5;
            printf("%04X: %02X MOV D(%02Xh), D (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x53:            /* mov d, e */
            cpu_cycles = 5;
            D = E;
            printf("%04X: %02X MOV D(%02Xh), E (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x54:            /* mov d, h */
            cpu_cycles = 5;
            D = H;
            printf("%04X: %02X MOV D(%02Xh), H (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x55:            /* mov d, l */
            cpu_cycles = 5;
            D = L;
            printf("%04X: %02X MOV D(%02Xh), L (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x56:            /* mov d, m */
            cpu_cycles = 7;
            D = RD_BYTE(HL);
            printf("%04X: %02X MOV D(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, D, HL, cpu_cycles);
            break;
        case 0x57:            /* mov d, a */
            cpu_cycles = 5;
            D = A;
            printf("%04X: %02X MOV D(%02Xh), A (%d)", PC_REF, opcode, D, cpu_cycles);
            break;
        case 0x58:            /* mov e, b */
            cpu_cycles = 5;
            E = B;
            printf("%04X: %02X MOV E(%02Xh), B (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x59:            /* mov e, c */
            cpu_cycles = 5;
            E = C;
            printf("%04X: %02X MOV E(%02Xh), C (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x5A:            /* mov e, d */
            cpu_cycles = 5;
            E = D;
            printf("%04X: %02X MOV E(%02Xh), D (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x5B:            /* mov e, e */
            cpu_cycles = 5;
            printf("%04X: %02X MOV E(%02Xh), E (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x5C:            /* mov c, h */
            cpu_cycles = 5;
            E = H;
            printf("%04X: %02X MOV E(%02Xh), H (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x5D:            /* mov c, l */
            cpu_cycles = 5;
            E = L;
            printf("%04X: %02X MOV E(%02Xh), L (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x5E:            /* mov c, m */
            cpu_cycles = 7;
            E = RD_BYTE(HL);
            printf("%04X: %02X MOV E(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, E, HL, cpu_cycles);
            break;
        case 0x5F:            /* mov c, a */
            cpu_cycles = 5;
            E = A;
            printf("%04X: %02X MOV E(%02Xh), A (%d)", PC_REF, opcode, E, cpu_cycles);
            break;
        case 0x60:            /* mov h, b */
            cpu_cycles = 5;
            H = B;
            printf("%04X: %02X MOV H(%02Xh), B (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x61:            /* mov h, c */
            cpu_cycles = 5;
            H = C;
            printf("%04X: %02X MOV H(%02Xh), C (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x62:            /* mov h, d */
            cpu_cycles = 5;
            H = D;
            printf("%04X: %02X MOV H(%02Xh), D (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x63:            /* mov h, e */
            cpu_cycles = 5;
            H = E;
            printf("%04X: %02X MOV H(%02Xh), E (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x64:            /* mov h, h */
            cpu_cycles = 5;
            printf("%04X: %02X MOV H(%02Xh), H (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x65:            /* mov h, l */
            cpu_cycles = 5;
            H = L;
            printf("%04X: %02X MOV H(%02Xh), L (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x66:            /* mov h, m */
            cpu_cycles = 7;
            H = RD_BYTE(HL);
            printf("%04X: %02X MOV H(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, H, HL, cpu_cycles);
            break;
        case 0x67:            /* mov h, a */
            cpu_cycles = 5;
            H = A;
            printf("%04X: %02X MOV H(%02Xh), A (%d)", PC_REF, opcode, H, cpu_cycles);
            break;
        case 0x68:            /* mov l, b */
            cpu_cycles = 5;
            L = B;
            printf("%04X: %02X MOV L(%02Xh), B (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x69:            /* mov l, c */
            cpu_cycles = 5;
            L = C;
            printf("%04X: %02X MOV L(%02Xh), C (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x6A:            /* mov l, d */
            cpu_cycles = 5;
            L = D;
            printf("%04X: %02X MOV L(%02Xh), D (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x6B:            /* mov l, e */
            cpu_cycles = 5;
            L = E;
            printf("%04X: %02X MOV L(%02Xh), E (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x6C:            /* mov l, h */
            cpu_cycles = 5;
            L = H;
            printf("%04X: %02X MOV L(%02Xh), H (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x6D:            /* mov l, l */
            cpu_cycles = 5;
            printf("%04X: %02X MOV L(%02Xh), L (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x6E:            /* mov l, m */
            cpu_cycles = 7;
            L = RD_BYTE(HL);
            printf("%04X: %02X MOV L(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, L, HL, cpu_cycles);
            break;
        case 0x6F:            /* mov l, a */
            cpu_cycles = 5;
            L = A;
            printf("%04X: %02X MOV L(%02Xh), A (%d)", PC_REF, opcode, L, cpu_cycles);
            break;
        case 0x70:            /* mov m, b */
            cpu_cycles = 7;
            WR_BYTE(HL, B);
            printf("%04X: %02X MOV @HL(%04Xh), B(%02Xh) (%d)", PC_REF, opcode, HL, B, cpu_cycles);
            break;
        case 0x71:            /* mov m, c */
            cpu_cycles = 7;
            WR_BYTE(HL, C);
            printf("%04X: %02X MOV @HL(%04Xh), C(%02Xh) (%d)", PC_REF, opcode, HL, C, cpu_cycles);
            break;
        case 0x72:            /* mov m, d */
            cpu_cycles = 7;
            WR_BYTE(HL, D);
            printf("%04X: %02X MOV @HL(%04Xh), D(%02Xh) (%d)", PC_REF, opcode, HL, D, cpu_cycles);
            break;
        case 0x73:            /* mov m, e */
            cpu_cycles = 7;
            WR_BYTE(HL, E);
            printf("%04X: %02X MOV @HL(%04Xh), E(%02Xh) (%d)", PC_REF, opcode, HL, E, cpu_cycles);
            break;
        case 0x74:            /* mov m, h */
            cpu_cycles = 7;
            WR_BYTE(HL, H);
            printf("%04X: %02X MOV @HL(%04Xh), H(%02Xh) (%d)", PC_REF, opcode, HL, H, cpu_cycles);
            break;
        case 0x75:            /* mov m, l */
            cpu_cycles = 7;
            WR_BYTE(HL, L);
            printf("%04X: %02X MOV @HL(%04Xh), L(%02Xh) (%d)", PC_REF, opcode, HL, L, cpu_cycles);
            break;
        case 0x76:            /* hlt */
            cpu_cycles = 4;
            PC--;
            printf("%04X: %02X HLT (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0x77:            /* mov m, a */
            cpu_cycles = 7;
            WR_BYTE(HL, A);
            printf("%04X: %02X MOV @HL(%04Xh), A(%02Xh) (%d)", PC_REF, opcode, HL, A, cpu_cycles);
            break;
        case 0x78:            /* mov a, b */
            cpu_cycles = 5;
            A = B;
            printf("%04X: %02X MOV A(%02Xh), B (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x79:            /* mov a, c */
            cpu_cycles = 5;
            A = C;
            printf("%04X: %02X MOV A(%02Xh), C (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x7A:            /* mov a, d */
            cpu_cycles = 5;
            A = D;
            printf("%04X: %02X MOV A(%02Xh), D (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x7B:            /* mov a, e */
            cpu_cycles = 5;
            A = E;
            printf("%04X: %02X MOV A(%02Xh), E (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x7C:            /* mov a, h */
            cpu_cycles = 5;
            A = H;
            printf("%04X: %02X MOV A(%02Xh), H (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x7D:            /* mov a, l */
            cpu_cycles = 5;
            A = L;
            printf("%04X: %02X MOV A(%02Xh), L (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x7E:            /* mov a, m */
            cpu_cycles = 7;
            A = RD_BYTE(HL);
            printf("%04X: %02X MOV A(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, A, HL, cpu_cycles);
            break;
        case 0x7F:            /* mov a, a */
            cpu_cycles = 5;
            printf("%04X: %02X MOV A(%02Xh), A (%d)", PC_REF, opcode, A, cpu_cycles);
            break;
        case 0x80:            /* add b */
            cpu_cycles = 4;
            ADD(B);
            printf("%04X: %02X ADD B A(%02Xh), B(%02Xh) (%d)", PC_REF, opcode, A, B, cpu_cycles);
            break;
        case 0x81:            /* add c */
            cpu_cycles = 4;
            ADD(C);
            printf("%04X: %02X ADD C A(%02Xh) = A(%02Xh) + C(%02Xh) (%d)", PC_REF, opcode, A, work8, C, cpu_cycles);
            break;
        case 0x82:            /* add d */
            cpu_cycles = 4;
            ADD(D);
            printf("%04X: %02X ADD D A(%02Xh) = A(%02Xh) + D(%02Xh) (%d)", PC_REF, opcode, A, work8, D, cpu_cycles);
            break;
        case 0x83:            /* add e */
            cpu_cycles = 4;
            ADD(E);
            printf("%04X: %02X ADD E A(%02Xh) = A(%02Xh) + E(%02Xh) (%d)", PC_REF, opcode, A, work8, E, cpu_cycles);
            break;
        case 0x84:            /* add h */
            cpu_cycles = 4;
            ADD(H);
            printf("%04X: %02X ADD H A(%02Xh) = A(%02Xh) + H(%02Xh) (%d)", PC_REF, opcode, A, work8, H, cpu_cycles);
            break;
        case 0x85:            /* add l */
            cpu_cycles = 4;
            ADD(L);
            printf("%04X: %02X ADD L A(%02Xh) = A(%02Xh) + L(%02Xh) (%d)", PC_REF, opcode, A, work8, L, cpu_cycles);
            break;
        case 0x86:            /* add m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            ADD(work8);
            printf("%04X: %02X ADD M A(%02Xh), @HL(%04Xh) (%d)", PC_REF, opcode, A, HL, cpu_cycles);
            break;
        case 0x87:            /* add a */
            cpu_cycles = 4;
            ADD(A);
            printf("%04X: %02X ADD A A(%02Xh), A(%02Xh) (%d)", PC_REF, opcode, A, A, cpu_cycles);
            break;
        case 0x88:            /* adc b */
            cpu_cycles = 4;
            ADC(B);
            printf("%04X: %02X ADC B A(%02Xh), B(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, B, C_FLAG, cpu_cycles);
            break;
        case 0x89:            /* adc c */
            cpu_cycles = 4;
            ADC(C);
            printf("%04X: %02X ADC C A(%02Xh), C(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, C, C_FLAG, cpu_cycles);
            break;
        case 0x8A:            /* adc d */
            cpu_cycles = 4;
            ADC(D);
            printf("%04X: %02X ADC D A(%02Xh), D(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, D, C_FLAG, cpu_cycles);
            break;
        case 0x8B:            /* adc e */
            cpu_cycles = 4;
            ADC(E);
            printf("%04X: %02X ADC E A(%02Xh), B(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, E, C_FLAG, cpu_cycles);
            break;
        case 0x8C:            /* adc h */
            cpu_cycles = 4;
            ADC(H);
            printf("%04X: %02X ADC H A(%02Xh), H(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, H, C_FLAG, cpu_cycles);
            break;
        case 0x8D:            /* adc l */
            cpu_cycles = 4;
            ADC(L);
            printf("%04X: %02X ADC L A(%02Xh), L(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, L, C_FLAG, cpu_cycles);
            break;
        case 0x8E:            /* adc m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            ADC(work8);
            printf("%04X: %02X ADC M A(%02Xh), @HL(%04Xh) Cf(%d) (%d)", PC_REF, opcode, A, HL, C_FLAG, cpu_cycles);
            break;
        case 0x8F:            /* adc a */
            cpu_cycles = 4;
            ADC(A);
            printf("%04X: %02X ADC L A(%02Xh), A(%02Xh) Cf(%d) (%d)", PC_REF, opcode, A, A, C_FLAG, cpu_cycles);
            break;
        case 0x90:            /* sub b */
            cpu_cycles = 4;
            SUB(B);
            printf("%04X: %02X SUB B A(%02Xh) = A(%02Xh) - B(%02Xh) (%d)", PC_REF, opcode, A, work8, B, cpu_cycles);
            break;
        case 0x91:            /* sub c */
            cpu_cycles = 4;
            SUB(C);
            printf("%04X: %02X SUB C A(%02Xh) = A(%02Xh) - C(%02Xh) (%d)", PC_REF, opcode, A, work8, C, cpu_cycles);
            break;
        case 0x92:            /* sub d */
            cpu_cycles = 4;
            SUB(D);
            printf("%04X: %02X SUB D A(%02Xh) = A(%02Xh) - D(%02Xh) (%d)", PC_REF, opcode, A, work8, B, cpu_cycles);
            break;
        case 0x93:            /* sub e */
            cpu_cycles = 4;
            SUB(E);
            printf("%04X: %02X SUB D A(%02Xh) = A(%02Xh) - E(%02Xh) (%d)", PC_REF, opcode, A, work8, E, cpu_cycles);
            break;
        case 0x94:            /* sub h */
            cpu_cycles = 4;
            SUB(H);
            printf("%04X: %02X SUB H A(%02Xh) = A(%02Xh) - H(%02Xh) (%d)", PC_REF, opcode, A, work8, H, cpu_cycles);
            break;
        case 0x95:            /* sub l */
            cpu_cycles = 4;
            SUB(L);
            printf("%04X: %02X SUB L A(%02Xh) = A(%02Xh) - L(%02Xh) (%d)", PC_REF, opcode, A, work8, L, cpu_cycles);
            break;
        case 0x96:            /* sub m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            SUB(work8);
            printf("%04X: %02X SUB M A(%02Xh) -= @HL(%04Xh) %02Xh (%d)", PC_REF, opcode, A, HL, work8, cpu_cycles);
            break;
        case 0x97:            /* sub a */
            cpu_cycles = 4;
            SUB(A);
            printf("%04X: %02X SUB A (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0x98:            /* sbb b */
            cpu_cycles = 4;
            SBB(B);
            printf("%04X: %02X SBB B A(%02Xh) = A(%02Xh) - B(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, B, cpu_cycles);
            break;
        case 0x99:            /* sbb c */
            cpu_cycles = 4;
            SBB(C);
            printf("%04X: %02X SBB B A(%02Xh) = A(%02Xh) - C(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, C, cpu_cycles);
            break;
        case 0x9A:            /* sbb d */
            cpu_cycles = 4;
            SBB(D);
            printf("%04X: %02X SBB D A(%02Xh) = A(%02Xh) - D(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, D, cpu_cycles);
            break;
        case 0x9B:            /* sbb e */
            cpu_cycles = 4;
            SBB(E);
            printf("%04X: %02X SBB E A(%02Xh) = A(%02Xh) - E(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, E, cpu_cycles);
            break;
        case 0x9C:            /* sbb h */
            cpu_cycles = 4;
            SBB(H);
            printf("%04X: %02X SBB H A(%02Xh) = A(%02Xh) - H(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, H, cpu_cycles);
            break;
        case 0x9D:            /* sbb l */
            cpu_cycles = 4;
            SBB(L);
            printf("%04X: %02X SBB L A(%02Xh) = A(%02Xh) - L(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, L, cpu_cycles);
            break;
        case 0x9E:            /* sbb m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            SBB(work8);
            printf("%04X: %02X SBB M A(%02Xh) -= @HL(%04Xh) %02Xh - Cf (%d)", PC_REF, opcode, A, HL, work8, cpu_cycles);
            break;
        case 0x9F:            /* sbb a */
            cpu_cycles = 4;
            SBB(A);
            printf("%04X: %02X SBB A A(%02Xh) = A(%02Xh) - A(%02Xh) - Cf (%d)", PC_REF, opcode, A, work8, work8, cpu_cycles);
            break;
        case 0xA0:            /* ana b */
            cpu_cycles = 4;
            ANA(B);
            printf("%04X: %02X ANA B A(%02Xh) = A(%02Xh) & B(%02Xh) (%d)", PC_REF, opcode, A, work8, B, cpu_cycles);
            break;
        case 0xA1:            /* ana c */
            cpu_cycles = 4;
            ANA(C);
            printf("%04X: %02X ANA C A(%02Xh) = A(%02Xh) & C(%02Xh) (%d)", PC_REF, opcode, A, work8, C, cpu_cycles);
            break;
        case 0xA2:            /* ana d */
            cpu_cycles = 4;
            ANA(D);
            printf("%04X: %02X ANA D A(%02Xh) = A(%02Xh) & D(%02Xh) (%d)", PC_REF, opcode, A, work8, D, cpu_cycles);
            break;
        case 0xA3:            /* ana e */
            cpu_cycles = 4;
            ANA(E);
            printf("%04X: %02X ANA E A(%02Xh) = A(%02Xh) & E(%02Xh) (%d)", PC_REF, opcode, A, work8, E, cpu_cycles);
            break;
        case 0xA4:            /* ana h */
            cpu_cycles = 4;
            ANA(H);
            printf("%04X: %02X ANA H A(%02Xh) = A(%02Xh) & H(%02Xh) (%d)", PC_REF, opcode, A, work8, H, cpu_cycles);
            break;
        case 0xA5:            /* ana l */
            cpu_cycles = 4;
            ANA(L);
            printf("%04X: %02X ANA L A(%02Xh) = A(%02Xh) & L(%02Xh) (%d)", PC_REF, opcode, A, work8, L, cpu_cycles);
            break;
        case 0xA6:            /* ana m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            ANA(work8);
            printf("%04X: %02X ANA M A(%02Xh) &= @HL(%04Xh) %02Xh (%d)", PC_REF, opcode, A, HL, work8, cpu_cycles);
            break;
        case 0xA7:            /* ana a */
            cpu_cycles = 4;
            ANA(A);
            printf("%04X: %02X ANA A A(%02Xh) = A(%02Xh) & A(%02Xh) (%d)", PC_REF, opcode, A, work8, work8, cpu_cycles);
            break;
        case 0xA8:            /* xra b */
            cpu_cycles = 4;
            XRA(B);
            printf("%04X: %02X XRA B A(%02Xh) = A(%02Xh) ^ B(%02Xh) (%d)", PC_REF, opcode, A, work8, B, cpu_cycles);
            break;
        case 0xA9:            /* xra c */
            cpu_cycles = 4;
            XRA(C);
            printf("%04X: %02X XRA C A(%02Xh) = A(%02Xh) ^ C(%02Xh) (%d)", PC_REF, opcode, A, work8, C, cpu_cycles);
            break;
        case 0xAA:            /* xra d */
            cpu_cycles = 4;
            XRA(D);
            printf("%04X: %02X XRA D A(%02Xh) = A(%02Xh) ^ D(%02Xh) (%d)", PC_REF, opcode, A, work8, D, cpu_cycles);
            break;
        case 0xAB:            /* xra e */
            cpu_cycles = 4;
            XRA(E);
            printf("%04X: %02X XRA E A(%02Xh) = A(%02Xh) ^ E(%02Xh) (%d)", PC_REF, opcode, A, work8, E, cpu_cycles);
            break;
        case 0xAC:            /* xra h */
            cpu_cycles = 4;
            XRA(H);
            printf("%04X: %02X XRA H A(%02Xh) = A(%02Xh) ^ H(%02Xh) (%d)", PC_REF, opcode, A, work8, H, cpu_cycles);
            break;
        case 0xAD:            /* xra l */
            cpu_cycles = 4;
            XRA(L);
            printf("%04X: %02X XRA L A(%02Xh) = A(%02Xh) ^ L(%02Xh) (%d)", PC_REF, opcode, A, work8, L, cpu_cycles);
            break;
        case 0xAE:            /* xra m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            XRA(work8);
            printf("%04X: %02X XRA M A(%02Xh) ^= @HL(%04Xh) %02Xh - Cf (%d)", PC_REF, opcode, A, HL, work8, cpu_cycles);
            break;
        case 0xAF:            /* xra a */
            cpu_cycles = 4;
            XRA(A);
            printf("%04X: %02X XRA A A(%02Xh) = A(%02Xh) ^ A(%02Xh) (%d)", PC_REF, opcode, A, work8, work8, cpu_cycles);
            break;
        case 0xB0:            /* ora b */
            cpu_cycles = 4;
            ORA(B);
            printf("%04X: %02X ORA B A(%02Xh) = A(%02Xh) | B(%02Xh) (%d)", PC_REF, opcode, A, work8, B, cpu_cycles);
            break;
        case 0xB1:            /* ora c */
            cpu_cycles = 4;
            ORA(C);
            printf("%04X: %02X ORA C A(%02Xh) = A(%02Xh) | C(%02Xh) (%d)", PC_REF, opcode, A, work8, C, cpu_cycles);
            break;
        case 0xB2:            /* ora d */
            cpu_cycles = 4;
            ORA(D);
            printf("%04X: %02X ORA D A(%02Xh) = A(%02Xh) | D(%02Xh) (%d)", PC_REF, opcode, A, work8, D, cpu_cycles);
            break;
        case 0xB3:            /* ora e */
            cpu_cycles = 4;
            ORA(E);
            printf("%04X: %02X ORA E A(%02Xh) = A(%02Xh) | E(%02Xh) (%d)", PC_REF, opcode, A, work8, E, cpu_cycles);
            break;
        case 0xB4:            /* ora h */
            cpu_cycles = 4;
            ORA(H);
            printf("%04X: %02X ORA H A(%02Xh) = A(%02Xh) | H(%02Xh) (%d)", PC_REF, opcode, A, work8, H, cpu_cycles);
            break;
        case 0xB5:            /* ora l */
            cpu_cycles = 4;
            ORA(L);
            printf("%04X: %02X ORA L A(%02Xh) = A(%02Xh) | L(%02Xh) (%d)", PC_REF, opcode, A, work8, L, cpu_cycles);
            break;
        case 0xB6:            /* ora m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            ORA(work8);
            printf("%04X: %02X ORA M A(%02Xh) |= @HL(%04Xh) %02Xh - Cf (%d)", PC_REF, opcode, A, HL, work8, cpu_cycles);
            break;
        case 0xB7:            /* ora a */
            cpu_cycles = 4;
            ORA(A);
            printf("%04X: %02X ORA A A(%02Xh) = A(%02Xh) | A(%02Xh) (%d)", PC_REF, opcode, A, work8, work8, cpu_cycles);
            break;
        case 0xB8:            /* cmp b */
            cpu_cycles = 4;
            CMP(B);
            printf("%04X: %02X CMP B A(%02Xh) <> B(%02Xh) (%d)", PC_REF, opcode, A, B, cpu_cycles);
            break;
        case 0xB9:            /* cmp c */
            cpu_cycles = 4;
            CMP(C);
            printf("%04X: %02X CMP C A(%02Xh) <> C(%02Xh) (%d)", PC_REF, opcode, A, C, cpu_cycles);
            break;
        case 0xBA:            /* cmp d */
            cpu_cycles = 4;
            CMP(D);
            printf("%04X: %02X CMP D A(%02Xh) <> D(%02Xh) (%d)", PC_REF, opcode, A, D, cpu_cycles);
            break;
        case 0xBB:            /* cmp e */
            cpu_cycles = 4;
            CMP(E);
            printf("%04X: %02X CMP E A(%02Xh) <> E(%02Xh) (%d)", PC_REF, opcode, A, E, cpu_cycles);
            break;
        case 0xBC:            /* cmp h */
            cpu_cycles = 4;
            CMP(H);
            printf("%04X: %02X CMP H A(%02Xh) <> H(%02Xh) (%d)", PC_REF, opcode, A, H, cpu_cycles);
            break;
        case 0xBD:            /* cmp l */
            cpu_cycles = 4;
            CMP(L);
            printf("%04X: %02X CMP L A(%02Xh) <> L(%02Xh) (%d)", PC_REF, opcode, A, L, cpu_cycles);
            break;
        case 0xBE:            /* cmp m */
            cpu_cycles = 7;
            work8 = RD_BYTE(HL);
            CMP(work8);
            printf("%04X: %02X CMP M A(%02Xh) <> @HL(%04Xh) %02Xh - Cf (%d)", PC_REF, opcode, A, HL, work8, cpu_cycles);
            break;
        case 0xBF:            /* cmp a */
            cpu_cycles = 4;
            CMP(A);
            printf("%04X: %02X CMP L A(%02Xh) <> A(%02Xh) (%d)", PC_REF, opcode, A, A, cpu_cycles);
            break;
        case 0xC0:            /* rnz */
            cpu_cycles = 5;
            if (!TST(Z_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RNZ (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xC1:            /* pop b */
            cpu_cycles = 11;
            POP(BC);
            printf("%04X: %02X POP BC(%04Xh) (%d)", PC_REF, opcode, BC, cpu_cycles);
            break;
        case 0xC2:            /* jnz addr */
            cpu_cycles = 10;
            if (!TST(Z_FLAG)) {
                PC = RD_WORD(PC);
            }
            else {
                PC += 2;
            }
            printf("%04X: %02X JNZ (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xC3:            /* jmp addr */
        case 0xCB:            /* jmp addr, undocumented */
            cpu_cycles = 10;
            PC = RD_WORD(PC);
            printf("%04X: %02X JMP %04Xh (%d)", PC_REF, opcode, PC, cpu_cycles);
            break;
        case 0xC4:            /* cnz addr */
            if (!TST(Z_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CNZ (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xC5:            /* push b */
            cpu_cycles = 11;
            PUSH(BC);
            printf("%04X: %02X PUSH B (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xC6:            /* adi data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            ADD(work8);
            printf("%04X: %02X ADI A(%02X) += %02Xh (%d)", PC_REF, opcode, A, work8, cpu_cycles);
            break;
        case 0xC7:            /* rst 0 */
            cpu_cycles = 11;
            RST(0x0000);
            printf("%04X: %02X RST 0 (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xC8:            /* rz */
            cpu_cycles = 5;
            if (TST(Z_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RZ (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xC9:            /* ret */
        case 0xD9:            /* ret, undocumented */
            cpu_cycles = 10;
            POP(PC);
            printf("%04X: %02X RET (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xCA:            /* jz addr */
            cpu_cycles = 10;
            if (TST(Z_FLAG)) {
                PC = RD_WORD(PC);
            } else {
                PC += 2;
            }
            printf("%04X: %02X JZ (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xCC:            /* cz addr */
            if (TST(Z_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CZ (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xCD:            /* call addr */
        case 0xDD:            /* call, undocumented */
        case 0xED:
        case 0xFD:
            cpu_cycles = 17;
            CALL;
            printf("%04X: %02X CALL (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xCE:            /* aci data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            ADC(work8);
            printf("%04X: %02X ACI A(%02X) += %02Xh + Cf (%d)", PC_REF, opcode, A, work8, cpu_cycles);
            break;
        case 0xCF:            /* rst 1 */
            cpu_cycles = 11;
            RST(0x0008);
            printf("%04X: %02X RST 1 (8h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xD0:            /* rnc */
            cpu_cycles = 5;
            if (!TST(C_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RNC (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xD1:            /* pop d */
            cpu_cycles = 11;
            POP(DE);
            printf("%04X: %02X POP D DE(%04Xh) (%d)", PC_REF, opcode, DE, cpu_cycles);
            break;
        case 0xD2:            /* jnc addr */
            cpu_cycles = 10;
            if (!TST(C_FLAG)) {
                PC = RD_WORD(PC);
            } else {
                PC += 2;
            }
            printf("%04X: %02X JNC (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xD3:            /* out port8 */
            cpu_cycles = 10;
            printf("%04X: %02X OUT PORT8 (%d)", PC_REF, opcode, cpu_cycles);
            i8080_hal_io_output(RD_BYTE(PC++), A);
            break;
        case 0xD4:            /* cnc addr */
            if (!TST(C_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CNC (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xD5:            /* push d */
            cpu_cycles = 11;
            PUSH(DE);
            printf("%04X: %02X PUSH D (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xD6:            /* sui data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            SUB(work8);
            printf("%04X: %02X SUI A(%02X) -= %02Xh (%d)", PC_REF, A, work8, opcode, cpu_cycles);
            break;
        case 0xD7:            /* rst 2 */
            cpu_cycles = 11;
            RST(0x0010);
            printf("%04X: %02X RST 2 (10h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xD8:            /* rc */
            cpu_cycles = 5;
            if (TST(C_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RC (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xDA:            /* jc addr */
            cpu_cycles = 10;
            if (TST(C_FLAG)) {
                PC = RD_WORD(PC);
            } else {
                PC += 2;
            }
            printf("%04X: %02X JC (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xDB:            /* in port8 */
            cpu_cycles = 10;
            A = i8080_hal_io_input(RD_BYTE(PC++));
            printf("%04X: %02X IN PORT8 (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xDC:            /* cc addr */
            if (TST(C_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CC (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xDE:            /* sbi data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            SBB(work8);
            printf("%04X: %02X SBI A(%02X) -= %02Xh - Cf (%d)", PC_REF, A, work8, opcode, cpu_cycles);
            break;
        case 0xDF:            /* rst 3 */
            cpu_cycles = 11;
            RST(0x0018);
            printf("%04X: %02X RST 3 (18h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE0:            /* rpo */
            cpu_cycles = 5;
            if (!TST(P_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RPO (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE1:            /* pop h */
            cpu_cycles = 11;
            POP(HL);
            printf("%04X: %02X POP H HL(%04Xh) (%d)", PC_REF, opcode, HL, cpu_cycles);
            break;
        case 0xE2:            /* jpo addr */
            cpu_cycles = 10;
            if (!TST(P_FLAG)) {
                PC = RD_WORD(PC);
            }
            else {
                PC += 2;
            }
            printf("%04X: %02X JPO (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE3:            /* xthl */
            cpu_cycles = 18;
            work16 = RD_WORD(SP);
            WR_WORD(SP, HL);
            HL = work16;
            printf("%04X: %02X XTHL HL(%04Xh) (%d)", PC_REF, opcode, HL, cpu_cycles);
            break;
        case 0xE4:            /* cpo addr */
            if (!TST(P_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CPO (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE5:            /* push h */
            cpu_cycles = 11;
            PUSH(HL);
            printf("%04X: %02X PUSH H (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE6:            /* ani data8 */
            cpu_cycles = 7;
            work16 = A;
            work8 = RD_BYTE(PC++);
            ANA(work8);
            printf("%04X: %02X ANI A(%02Xh) = A(%02Xh) & @PC(%02Xh) Zf(%d) (%d)", PC_REF, opcode, A, (uint8_t)work16, work8, Z_FLAG, cpu_cycles);
            break;
        case 0xE7:            /* rst 4 */
            cpu_cycles = 11;
            RST(0x0020);
            printf("%04X: %02X RST 4 (20h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE8:            /* rpe */
            cpu_cycles = 5;
            if (TST(P_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RPE (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xE9:            /* pchl */
            cpu_cycles = 5;
            PC = HL;
            printf("%04X: %02X PCHL (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xEA:            /* jpe addr */
            cpu_cycles = 10;
            if (TST(P_FLAG)) {
                PC = RD_WORD(PC);
            } else {
                PC += 2;
            }
            printf("%04X: %02X JPE (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xEB:            /* xchg */
            cpu_cycles = 4;
            work16 = DE;
            DE = HL;
            HL = work16;
            printf("%04X: %02X XCHG DE(%04Xh) HL(%04Xh) (%d)", PC_REF, opcode, DE, HL, cpu_cycles);
            break;
        case 0xEC:            /* cpe addr */
            if (TST(P_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CPE (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xEE:            /* xri data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            XRA(work8);
            printf("%04X: %02X XRI A(%02X) ^= %02Xh (%d)", PC_REF, A, work8, opcode, cpu_cycles);
            break;
        case 0xEF:            /* rst 5 */
            cpu_cycles = 11;
            RST(0x0028);
            printf("%04X: %02X RST 5 (28h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF0:            /* rp */
            cpu_cycles = 5;
            if (!TST(S_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RP (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF1:            /* pop psw */
            cpu_cycles = 10;
            POP(AF);
            i8080_retrieve_flags();
            printf("%04X: %02X POP PSW (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF2:            /* jp addr */
            cpu_cycles = 10;
            if (!TST(S_FLAG)) {
                PC = RD_WORD(PC);
            } else {
                PC += 2;
            }
            printf("%04X: %02X JP (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF3:            /* di */
            cpu_cycles = 4;
            IFF = 0;
            i8080_hal_iff(IFF);
            printf("%04X: %02X DI (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF4:            /* cp addr */
            if (!TST(S_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CP (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF5:            /* push psw */
            cpu_cycles = 11;
            i8080_store_flags();
            PUSH(AF);
            printf("%04X: %02X PUSH PSW (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF6:            /* ori data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            ORA(work8);
            printf("%04X: %02X ORI A(%02X) |= %02Xh (%d)", PC_REF, A, work8, opcode, cpu_cycles);
            break;
        case 0xF7:            /* rst 6 */
            cpu_cycles = 11;
            RST(0x0030);
            printf("%04X: %02X RST 6 (30h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF8:            /* rm */
            cpu_cycles = 5;
            if (TST(S_FLAG)) {
                cpu_cycles = 11;
                POP(PC);
            }
            printf("%04X: %02X RM (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xF9:            /* sphl */
            cpu_cycles = 5;
            SP = HL;
            printf("%04X: %02X SPHL (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xFA:            /* jm addr */
            cpu_cycles = 10;
            if (TST(S_FLAG)) {
                PC = RD_WORD(PC);
            } else {
                PC += 2;
            }
            printf("%04X: %02X JM (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xFB:            /* ei */
            cpu_cycles = 4;
            IFF = 1;
            i8080_hal_iff(IFF);
            printf("%04X: %02X EI (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xFC:            /* cm addr */
            if (TST(S_FLAG)) {
                cpu_cycles = 17;
                CALL;
            } else {
                cpu_cycles = 11;
                PC += 2;
            }
            printf("%04X: %02X CM (%d)", PC_REF, opcode, cpu_cycles);
            break;
        case 0xFE:            /* cpi data8 */
            cpu_cycles = 7;
            work8 = RD_BYTE(PC++);
            CMP(work8);
            printf("%04X: %02X CPI A(%02X) <> %02Xh (%d)", PC_REF, A, work8, opcode, cpu_cycles);
            break;
        case 0xFF:            /* rst 7 */
            cpu_cycles = 11;
            RST(0x0038);
            printf("%04X: %02X RST 7 (38h) (%d)", PC_REF, opcode, cpu_cycles);
            break;
        default:
            cpu_cycles = -1;  /* Shouldn't be really here. */
            printf("%04X: %02X ??? (%d)", PC_REF, opcode, cpu_cycles);
            break;
    }
    return cpu_cycles;
}

int i8080_instruction(void) {
    return i8080_execute(RD_BYTE(PC++));
}

void i8080_jump(int addr) {
    PC = addr & 0xffff;
}

int i8080_pc(void) {
    return PC;
}

int i8080_regs_bc(void) {
    return BC;
}

int i8080_regs_de(void) {
    return DE;
}

int i8080_regs_hl(void) {
    return HL;
}

int i8080_regs_sp(void) {
    return SP;
}

int i8080_regs_a(void) {
    return A;
}

int i8080_regs_b(void) {
    return B;
}

int i8080_regs_c(void) {
    return C;
}

int i8080_regs_d(void) {
    return D;
}

int i8080_regs_e(void) {
    return E;
}

int i8080_regs_h(void) {
    return H;
}

int i8080_regs_l(void) {
    return L;
}
