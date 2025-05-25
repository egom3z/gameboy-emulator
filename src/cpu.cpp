/**
 * @file cpu.cpp
 * @brief Implements the CPU class: registers, opcode, decoding, ALU, flags, interrupts.
 *
 * This file implements the FETCH/DECODE/EXECUTE pipeline logic for the GameBoy's
 * SMLR35902 CPU which follows the SM83 instruction set.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#include "cpu.hpp"
#include "utils.hpp"

namespace gb {

  CPU::CPU(Memory& mem) : mem_(mem) {
    initInstructionTable();
  }

  void CPU::initInstructionTable() {
    using Fn = int (CPU::*)();

    instruction_table_ = {
        /* 0x00 */ &CPU::NOP,
        /* 0x01 */ &CPU::LD_BC_NN,
        /* 0x02 */ &CPU::LD_BC_A,
        /* 0x03 */ &CPU::INC_BC,
        /* 0x04 */ &CPU::INC_B,
        /* 0x05 */ &CPU::DEC_B,
        /* 0x06 */ &CPU::LD_B_N,
        /* 0x07 */ &CPU::RLCA,
        /* 0x08 */ &CPU::LD_NN_SP,
        /* 0x09 */ &CPU::ADD_HL_BC,
        /* 0x0A */ &CPU::LD_A_BC,
        /* 0x0B */ &CPU::DEC_BC,
        /* 0x0C */ &CPU::INC_C,
        /* 0x0D */ &CPU::DEC_C,
        /* 0x0E */ &CPU::LD_C_N,
        /* 0x0F */ &CPU::RRCA,

        /* 0x10 */ &CPU::STOP,
        /* 0x11 */ &CPU::LD_DE_NN,
        /* 0x12 */ &CPU::LD_DE_A,
        /* 0x13 */ &CPU::INC_DE,
        /* 0x14 */ &CPU::INC_D,
        /* 0x15 */ &CPU::DEC_D,
        /* 0x16 */ &CPU::LD_D_N,
        /* 0x17 */ &CPU::RLA,
        /* 0x18 */ &CPU::JR_N,
        /* 0x19 */ &CPU::ADD_HL_DE,
        /* 0x1A */ &CPU::LD_A_DE,
        /* 0x1B */ &CPU::DEC_DE,
        /* 0x1C */ &CPU::INC_E,
        /* 0x1D */ &CPU::DEC_E,
        /* 0x1E */ &CPU::LD_E_N,
        /* 0x1F */ &CPU::RRA,

        /* 0x20 */ &CPU::JR_NZ,
        /* 0x21 */ &CPU::LD_HL_NN,
        /* 0x22 */ &CPU::LD_HLI_A,
        /* 0x23 */ &CPU::INC_HL,
        /* 0x24 */ &CPU::INC_H,
        /* 0x25 */ &CPU::DEC_H,
        /* 0x26 */ &CPU::LD_H_N,
        /* 0x27 */ &CPU::DAA,
        /* 0x28 */ &CPU::JR_Z,
        /* 0x29 */ &CPU::ADD_HL_HL,
        /* 0x2A */ &CPU::LD_A_HLI,
        /* 0x2B */ &CPU::DEC_HL,
        /* 0x2C */ &CPU::INC_L,
        /* 0x2D */ &CPU::DEC_L,
        /* 0x2E */ &CPU::LD_L_N,
        /* 0x2F */ &CPU::CPL,

        /* 0x30 */ &CPU::JR_NC,
        /* 0x31 */ &CPU::LD_SP_NN,
        /* 0x32 */ &CPU::LD_HLD_A,
        /* 0x33 */ &CPU::INC_SP,
        /* 0x34 */ &CPU::INC_HL_MEM,
        /* 0x35 */ &CPU::DEC_HL_MEM,
        /* 0x36 */ &CPU::LD_HL_N,
        /* 0x37 */ &CPU::SCF,
        /* 0x38 */ &CPU::JR_C,
        /* 0x39 */ &CPU::ADD_HL_SP,
        /* 0x3A */ &CPU::LD_A_HLD,
        /* 0x3B */ &CPU::DEC_SP,
        /* 0x3C */ &CPU::INC_A,
        /* 0x3D */ &CPU::DEC_A,
        /* 0x3E */ &CPU::LD_A_N,
        /* 0x3F */ &CPU::CCF,

        /* 0x40–0x7F: LD r1, r2 */
        &CPU::LD_B_B,  &CPU::LD_B_C,  &CPU::LD_B_D,  &CPU::LD_B_E,
        &CPU::LD_B_H,  &CPU::LD_B_L,  &CPU::LD_B_HL, &CPU::LD_B_A,
        &CPU::LD_C_B,  &CPU::LD_C_C,  &CPU::LD_C_D,  &CPU::LD_C_E,
        &CPU::LD_C_H,  &CPU::LD_C_L,  &CPU::LD_C_HL, &CPU::LD_C_A,
        &CPU::LD_D_B,  &CPU::LD_D_C,  &CPU::LD_D_D,  &CPU::LD_D_E,
        &CPU::LD_D_H,  &CPU::LD_D_L,  &CPU::LD_D_HL, &CPU::LD_D_A,
        &CPU::LD_E_B,  &CPU::LD_E_C,  &CPU::LD_E_D,  &CPU::LD_E_E,
        &CPU::LD_E_H,  &CPU::LD_E_L,  &CPU::LD_E_HL, &CPU::LD_E_A,
        &CPU::LD_H_B,  &CPU::LD_H_C,  &CPU::LD_H_D,  &CPU::LD_H_E,
        &CPU::LD_H_H,  &CPU::LD_H_L,  &CPU::LD_H_HL, &CPU::LD_H_A,
        &CPU::LD_L_B,  &CPU::LD_L_C,  &CPU::LD_L_D,  &CPU::LD_L_E,
        &CPU::LD_L_H,  &CPU::LD_L_L,  &CPU::LD_L_HL, &CPU::LD_L_A,
        &CPU::LD_HL_B, &CPU::LD_HL_C, &CPU::LD_HL_D, &CPU::LD_HL_E,
        &CPU::LD_HL_H, &CPU::LD_HL_L, &CPU::HALT,    &CPU::LD_HL_A,
        &CPU::LD_A_B,  &CPU::LD_A_C,  &CPU::LD_A_D,  &CPU::LD_A_E,
        &CPU::LD_A_H,  &CPU::LD_A_L,  &CPU::LD_A_HL, &CPU::LD_A_A,

        /* 0x80–0xBF: ALU ops */
        &CPU::ADD_A_B, &CPU::ADD_A_C, &CPU::ADD_A_D,  &CPU::ADD_A_E,
        &CPU::ADD_A_H, &CPU::ADD_A_L, &CPU::ADD_A_HL, &CPU::ADD_A_A,
        &CPU::ADC_A_B, &CPU::ADC_A_C, &CPU::ADC_A_D,  &CPU::ADC_A_E,
        &CPU::ADC_A_H, &CPU::ADC_A_L, &CPU::ADC_A_HL, &CPU::ADC_A_A,
        &CPU::SUB_A_B, &CPU::SUB_A_C, &CPU::SUB_A_D,  &CPU::SUB_A_E,
        &CPU::SUB_A_H, &CPU::SUB_A_L, &CPU::SUB_A_HL, &CPU::SUB_A_A,
        &CPU::SBC_A_B, &CPU::SBC_A_C, &CPU::SBC_A_D,  &CPU::SBC_A_E,
        &CPU::SBC_A_H, &CPU::SBC_A_L, &CPU::SBC_A_HL, &CPU::SBC_A_A,
        &CPU::AND_A_B, &CPU::AND_A_C, &CPU::AND_A_D,  &CPU::AND_A_E,
        &CPU::AND_A_H, &CPU::AND_A_L, &CPU::AND_A_HL, &CPU::AND_A_A,
        &CPU::XOR_A_B, &CPU::XOR_A_C, &CPU::XOR_A_D,  &CPU::XOR_A_E,
        &CPU::XOR_A_H, &CPU::XOR_A_L, &CPU::XOR_A_HL, &CPU::XOR_A_A,
        &CPU::OR_A_B,  &CPU::OR_A_C,  &CPU::OR_A_D,   &CPU::OR_A_E,
        &CPU::OR_A_H,  &CPU::OR_A_L,  &CPU::OR_A_HL,  &CPU::OR_A_A,
        &CPU::CP_A_B,  &CPU::CP_A_C,  &CPU::CP_A_D,   &CPU::CP_A_E,
        &CPU::CP_A_H,  &CPU::CP_A_L,  &CPU::CP_A_HL,  &CPU::CP_A_A,

        /* 0xC0–0xFF: Calls, jumps, rets, misc */
        &CPU::RET_NZ,      &CPU::POP_BC,    &CPU::JP_NZ,   &CPU::JP,
        &CPU::CALL_NZ,     &CPU::PUSH_BC,   &CPU::ADD_A_N, &CPU::RST_00,
        &CPU::RET_Z,       &CPU::RET,       &CPU::JP_Z,    &CPU::UNUSED_CB,
        &CPU::CALL_Z,      &CPU::CALL,      &CPU::ADC_A_N, &CPU::RST_08,
        &CPU::RET_NC,      &CPU::POP_DE,    &CPU::JP_NC,   &CPU::UNUSED_D3,
        &CPU::CALL_NC,     &CPU::PUSH_DE,   &CPU::SUB_A_N, &CPU::RST_10,
        &CPU::RET_C,       &CPU::RETI,      &CPU::JP_C,    &CPU::UNUSED_DB,
        &CPU::CALL_C,      &CPU::UNUSED_DD, &CPU::SBC_A_N, &CPU::RST_18,
        &CPU::LDH_N_A,     &CPU::POP_HL,    &CPU::LDH_C_A, &CPU::UNUSED_E3,
        &CPU::UNUSED_E4,   &CPU::PUSH_HL,   &CPU::AND_A_N, &CPU::RST_20,
        &CPU::ADD_SP_I8,   &CPU::JP_HL,     &CPU::LD_NN_A, &CPU::UNUSED_EB,
        &CPU::UNUSED_EC,   &CPU::UNUSED_ED, &CPU::XOR_A_N, &CPU::RST_28,
        &CPU::LDH_A_N,     &CPU::POP_AF,    &CPU::LDH_A_C, &CPU::DI,
        &CPU::UNUSED_F4,   &CPU::PUSH_AF,   &CPU::OR_A_N,  &CPU::RST_30,
        &CPU::LD_HL_SP_I8, &CPU::LD_SP_HL,  &CPU::LD_A_NN, &CPU::EI,
        &CPU::UNUSED_FC,   &CPU::UNUSED_FD, &CPU::CP_A_N,  &CPU::RST_38
    };
  }

  auto CPU::NOP() -> int {
    return 4;
  }

  int CPU::LD_BC_NN() {
    bc_ = fetch16();
    return 12;
  }

  int CPU::LD_BC_A() {
    mem_.write8(bc_, a_);
    return 8;
  }

  int CPU::INC_BC() {
    bc_++;
    return 8;
  }

  int CPU::INC_B() {
    u8 result = b_ + 1;

    f_ &= FLAG_C;
    if (result == 0)            f_ |= FLAG_Z; // Z
    if ((b_ & 0x0F) + 1 > 0x0F) f_ |= FLAG_H; // H
    // N flag cleared automatically since it's 0
    // C flag is not affected

    b_ = result;
    return 4;
  }

  int CPU::DEC_B() {
    u8 result = b_ - 1;

    f_ = (f_ & FLAG_C) | FLAG_N; // set N, preserve C
    if (result == 0)          f_ |= FLAG_Z; // Z
    if ((b_ & 0x0F) == 0x00)  f_ |= FLAG_H; // H (borrow from bit 4)

    b_ = result;
    return 4;
  }

  int CPU::LD_B_N() {
    b_ = fetch8();
    return 8;
  }

  int CPU::RLCA() {
    u8 carry = (a_ & FLAG_Z) >> 7;
    a_ = (a_ << 1) | carry;

    f_ = 0;
    if (carry) f_ |= FLAG_C; // set C if needed

    return 4;
  }

  int CPU::LD_NN_SP() {
    mem_.write16(fetch16(), sp_);
    return 20;
  }

  int CPU::ADD_HL_BC() {
    u32 result = hl_ + bc_; // 32-bit to detect overflow

    f_ &= ~0x70; // Preserve Z

    if (((hl_ & 0x0FFF) + (bc_ & 0x0FFF)) > 0x0FFF) {
      f_ |= FLAG_H; // set H
    }
    if (result > 0xFFFF) {
      f_ |= FLAG_C; // set C
    }

    hl_ = static_cast<u16>(result);
    return 8;
  }

  int CPU::LD_A_BC() {
    a_ = mem_.read8(bc_);
    return 8;
  }

  int CPU::DEC_BC() {
    bc_--;
    return 8;
  }

  int CPU::INC_C() {
    u8 result = c_ + 1;

    f_ &= ~0xE0; // clear N 

    if (result == 0)            f_ |= FLAG_Z; // Z
    if ((c_ & 0x0F) + 1 > 0x0F) f_ |= FLAG_H; // H

    c_ = result;

    return 4;
  }

  int CPU::DEC_C() {
    u8 result = c_ - 1;


    f_ = (f_ & FLAG_C) | FLAG_N;
    if (result == 0)      f_ |= FLAG_Z; // Z
    if ((c_ & 0x0F) == 0) f_ |= FLAG_H; // H

    c_ = result;

    return 4;
  }

  int CPU::LD_C_N() {
    c_ = fetch8();
    return 8;
  }

  int CPU::RRCA() {
    u8 carry = a_ & 0x01;
    a_ = (a_ >> 1) | (carry << 7);

    f_ = 0;
    if (carry) f_ |= FLAG_C;

    return 4;
  }

  int CPU::STOP() { UNIMPLEMENTED(); }
  int CPU::LD_DE_NN() { UNIMPLEMENTED(); }
  int CPU::LD_DE_A() { UNIMPLEMENTED(); }
  int CPU::INC_DE() { UNIMPLEMENTED(); }
  int CPU::INC_D() { UNIMPLEMENTED(); }
  int CPU::DEC_D() { UNIMPLEMENTED(); }
  int CPU::LD_D_N() { UNIMPLEMENTED(); }
  int CPU::RLA() { UNIMPLEMENTED(); }
  int CPU::JR_N() { UNIMPLEMENTED(); }
  int CPU::ADD_HL_DE() { UNIMPLEMENTED(); }
  int CPU::LD_A_DE() { UNIMPLEMENTED(); }
  int CPU::DEC_DE() { UNIMPLEMENTED(); }
  int CPU::INC_E() { UNIMPLEMENTED(); }
  int CPU::DEC_E() { UNIMPLEMENTED(); }
  int CPU::LD_E_N() { UNIMPLEMENTED(); }
  int CPU::RRA() { UNIMPLEMENTED(); }

  int CPU::JR_NZ() { UNIMPLEMENTED(); }
  int CPU::LD_HL_NN() { UNIMPLEMENTED(); }
  int CPU::LD_HLI_A() { UNIMPLEMENTED(); }
  int CPU::INC_HL() { UNIMPLEMENTED(); }
  int CPU::INC_H() { UNIMPLEMENTED(); }
  int CPU::DEC_H() { UNIMPLEMENTED(); }
  int CPU::LD_H_N() { UNIMPLEMENTED(); }
  int CPU::DAA() { UNIMPLEMENTED(); }
  int CPU::JR_Z() { UNIMPLEMENTED(); }
  int CPU::ADD_HL_HL() { UNIMPLEMENTED(); }
  int CPU::LD_A_HLI() { UNIMPLEMENTED(); }
  int CPU::DEC_HL() { UNIMPLEMENTED(); }
  int CPU::INC_L() { UNIMPLEMENTED(); }
  int CPU::DEC_L() { UNIMPLEMENTED(); }
  int CPU::LD_L_N() { UNIMPLEMENTED(); }
  int CPU::CPL() { UNIMPLEMENTED(); }

  int CPU::JR_NC() { UNIMPLEMENTED(); }
  int CPU::LD_SP_NN() { UNIMPLEMENTED(); }
  int CPU::LD_HLD_A() { UNIMPLEMENTED(); }
  int CPU::INC_SP() { UNIMPLEMENTED(); }
  int CPU::INC_HL_MEM() { UNIMPLEMENTED(); }
  int CPU::DEC_HL_MEM() { UNIMPLEMENTED(); }
  int CPU::LD_HL_N() { UNIMPLEMENTED(); }
  int CPU::SCF() { UNIMPLEMENTED(); }
  int CPU::JR_C() { UNIMPLEMENTED(); }
  int CPU::ADD_HL_SP() { UNIMPLEMENTED(); }
  int CPU::LD_A_HLD() { UNIMPLEMENTED(); }
  int CPU::DEC_SP() { UNIMPLEMENTED(); }
  int CPU::INC_A() { UNIMPLEMENTED(); }
  int CPU::DEC_A() { UNIMPLEMENTED(); }
  int CPU::LD_A_N() { UNIMPLEMENTED(); }
  int CPU::CCF() { UNIMPLEMENTED(); }

  int CPU::LD_B_B() {
    return 4;
  }

  int CPU::LD_B_C() {
    b_ = c_;
    return 4;
  }

  int CPU::LD_B_D() {
    b_ = d_;
    return 4;
  }

  int CPU::LD_B_E() {
    b_ = e_;
    return 4;
  }

  int CPU::LD_B_H() {
    b_ = h_;
    return 4;
  }

  int CPU::LD_B_L() {
    b_ = l_;
    return 4;
  }

  int CPU::LD_B_HL() {
    b_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_B_A() {
    b_ = a_;
    return 4;
  }

  int CPU::LD_C_B() {
    c_ = b_;
    return 4;
  }

  int CPU::LD_C_C() {
    return 4;
  }

  int CPU::LD_C_D() {
    c_ = d_;
    return 4;
  }

  int CPU::LD_C_E() {
    c_ = e_;
    return 4;
  }

  int CPU::LD_C_H() {
    c_ = h_;
    return 4;
  }

  int CPU::LD_C_L() {
    c_ = l_;
    return 4;
  }

  int CPU::LD_C_HL() {
    c_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_C_A() {
    c_ = a_;
    return 4;
  }

  int CPU::LD_D_B() {
    d_ = b_;
    return 4;
  }

  int CPU::LD_D_C() {
    d_ = c_;
    return 4;
  }

  int CPU::LD_D_D() {
    return 4;
  }

  int CPU::LD_D_E() {
    d_ = e_;
    return 4;
  }

  int CPU::LD_D_H() {
    d_ = h_;
    return 4;
  }

  int CPU::LD_D_L() {
    d_ = l_;
    return 4;
  }

  int CPU::LD_D_HL() {
    d_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_D_A() {
    d_ = a_;
    return 4;
  }

  int CPU::LD_E_B() {
    e_ = b_;
    return 4;
  }

  int CPU::LD_E_C() {
    e_ = c_;
    return 4;
  }

  int CPU::LD_E_D() {
    e_ = d_;
    return 4;
  }

  int CPU::LD_E_E() {
    e_ = e_;
    return 4;
  }

  int CPU::LD_E_H() {
    e_ = h_;
    return 4;
  }

  int CPU::LD_E_L() {
    e_ = l_;
    return 4;
  }

  int CPU::LD_E_HL() {
    e_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_E_A() {
    e_ = a_;
    return 4;
  }

  int CPU::LD_H_B() {
    h_ = b_;
    return 4;
  }

  int CPU::LD_H_C() {
    h_ = c_;
    return 4;
  }

  int CPU::LD_H_D() {
    h_ = d_;
    return 4;
  }

  int CPU::LD_H_E() {
    h_ = e_;
    return 4;
  }

  int CPU::LD_H_H() {
    return 4;
  }

  int CPU::LD_H_L() {
    h_ = l_;
    return 4;
  }

  int CPU::LD_H_HL() {
    h_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_H_A() {
    h_ = a_;
    return 4;
  }

  int CPU::LD_L_B() {
    l_ = b_;
    return 4;
  }

  int CPU::LD_L_C() {
    l_ = c_;
    return 4;
  }

  int CPU::LD_L_D() {
    l_ = d_;
    return 4;
  }

  int CPU::LD_L_E() {
    l_ = e_;
    return 4;
  }

  int CPU::LD_L_H() {
    l_ = h_;
    return 4;
  }

  int CPU::LD_L_L() {
    return 4;
  }

  int CPU::LD_L_HL() {
    l_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_L_A() {
    l_ = a_;
    return 4;
  }

  int CPU::LD_HL_B() {
    mem_.write8(hl_, b_);
    return 8;
  }

  int CPU::LD_HL_C() {
    mem_.write8(hl_, c_);
    return 8;
  }

  int CPU::LD_HL_D() {
    mem_.write8(hl_, d_);
    return 8;
  }

  int CPU::LD_HL_E() {
    mem_.write8(hl_, e_);
    return 8;
  }

  int CPU::LD_HL_H() {
    mem_.write8(hl_, h_);
    return 8;
  }

  int CPU::LD_HL_L() {
    mem_.write8(hl_, l_);
    return 8;
  }

  int CPU::HALT() {
    halted_ = true;
    return 4;
  }

  int CPU::LD_HL_A() {
    mem_.write8(hl_, a_);
    return 8;
  }

  int CPU::LD_A_B() {
    a_ = b_;
    return 4;
  }

  int CPU::LD_A_C() {
    a_ = c_;
    return 4;
  }

  int CPU::LD_A_D() {
    a_ = d_;
    return 4;
  }

  int CPU::LD_A_E() {
    a_ = e_;
    return 4;
  }

  int CPU::LD_A_H() {
    a_ = h_;
    return 4;
  }

  int CPU::LD_A_L() {
    a_ = l_;
    return 4;
  }

  int CPU::LD_A_HL() {
    a_ = mem_.read8(hl_);
    return 8;
  }

  int CPU::LD_A_A() {
    return 4;
  }

  int CPU::ADD_A_B() { UNIMPLEMENTED(); }
  int CPU::ADD_A_C() { UNIMPLEMENTED(); }
  int CPU::ADD_A_D() { UNIMPLEMENTED(); }
  int CPU::ADD_A_E() { UNIMPLEMENTED(); }
  int CPU::ADD_A_H() { UNIMPLEMENTED(); }
  int CPU::ADD_A_L() { UNIMPLEMENTED(); }
  int CPU::ADD_A_HL() { UNIMPLEMENTED(); }
  int CPU::ADD_A_A() { UNIMPLEMENTED(); }
  int CPU::ADC_A_B() { UNIMPLEMENTED(); }
  int CPU::ADC_A_C() { UNIMPLEMENTED(); }
  int CPU::ADC_A_D() { UNIMPLEMENTED(); }
  int CPU::ADC_A_E() { UNIMPLEMENTED(); }
  int CPU::ADC_A_H() { UNIMPLEMENTED(); }
  int CPU::ADC_A_L() { UNIMPLEMENTED(); }
  int CPU::ADC_A_HL() { UNIMPLEMENTED(); }
  int CPU::ADC_A_A() { UNIMPLEMENTED(); }
  int CPU::SUB_A_B() { UNIMPLEMENTED(); }
  int CPU::SUB_A_C() { UNIMPLEMENTED(); }
  int CPU::SUB_A_D() { UNIMPLEMENTED(); }
  int CPU::SUB_A_E() { UNIMPLEMENTED(); }
  int CPU::SUB_A_H() { UNIMPLEMENTED(); }
  int CPU::SUB_A_L() { UNIMPLEMENTED(); }
  int CPU::SUB_A_HL() { UNIMPLEMENTED(); }
  int CPU::SUB_A_A() { UNIMPLEMENTED(); }
  int CPU::SBC_A_B() { UNIMPLEMENTED(); }
  int CPU::SBC_A_C() { UNIMPLEMENTED(); }
  int CPU::SBC_A_D() { UNIMPLEMENTED(); }
  int CPU::SBC_A_E() { UNIMPLEMENTED(); }
  int CPU::SBC_A_H() { UNIMPLEMENTED(); }
  int CPU::SBC_A_L() { UNIMPLEMENTED(); }
  int CPU::SBC_A_HL() { UNIMPLEMENTED(); }
  int CPU::SBC_A_A() { UNIMPLEMENTED(); }
  int CPU::AND_A_B() { UNIMPLEMENTED(); }
  int CPU::AND_A_C() { UNIMPLEMENTED(); }
  int CPU::AND_A_D() { UNIMPLEMENTED(); }
  int CPU::AND_A_E() { UNIMPLEMENTED(); }
  int CPU::AND_A_H() { UNIMPLEMENTED(); }
  int CPU::AND_A_L() { UNIMPLEMENTED(); }
  int CPU::AND_A_HL() { UNIMPLEMENTED(); }
  int CPU::AND_A_A() { UNIMPLEMENTED(); }
  int CPU::XOR_A_B() { UNIMPLEMENTED(); }
  int CPU::XOR_A_C() { UNIMPLEMENTED(); }
  int CPU::XOR_A_D() { UNIMPLEMENTED(); }
  int CPU::XOR_A_E() { UNIMPLEMENTED(); }
  int CPU::XOR_A_H() { UNIMPLEMENTED(); }
  int CPU::XOR_A_L() { UNIMPLEMENTED(); }
  int CPU::XOR_A_HL() { UNIMPLEMENTED(); }
  int CPU::XOR_A_A() { UNIMPLEMENTED(); }
  int CPU::OR_A_B() { UNIMPLEMENTED(); }
  int CPU::OR_A_C() { UNIMPLEMENTED(); }
  int CPU::OR_A_D() { UNIMPLEMENTED(); }
  int CPU::OR_A_E() { UNIMPLEMENTED(); }
  int CPU::OR_A_H() { UNIMPLEMENTED(); }
  int CPU::OR_A_L() { UNIMPLEMENTED(); }
  int CPU::OR_A_HL() { UNIMPLEMENTED(); }
  int CPU::OR_A_A() { UNIMPLEMENTED(); }
  int CPU::CP_A_B() { UNIMPLEMENTED(); }
  int CPU::CP_A_C() { UNIMPLEMENTED(); }
  int CPU::CP_A_D() { UNIMPLEMENTED(); }
  int CPU::CP_A_E() { UNIMPLEMENTED(); }
  int CPU::CP_A_H() { UNIMPLEMENTED(); }
  int CPU::CP_A_L() { UNIMPLEMENTED(); }
  int CPU::CP_A_HL() { UNIMPLEMENTED(); }
  int CPU::CP_A_A() { UNIMPLEMENTED(); }

  int CPU::RET_NZ() { UNIMPLEMENTED(); }
  int CPU::POP_BC() { UNIMPLEMENTED(); }
  int CPU::JP_NZ() { UNIMPLEMENTED(); }
  int CPU::JP() { UNIMPLEMENTED(); }
  int CPU::CALL_NZ() { UNIMPLEMENTED(); }
  int CPU::PUSH_BC() { UNIMPLEMENTED(); }
  int CPU::ADD_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_00() { UNIMPLEMENTED(); }
  int CPU::RET_Z() { UNIMPLEMENTED(); }
  int CPU::RET() { UNIMPLEMENTED(); }
  int CPU::JP_Z() { UNIMPLEMENTED(); }
  int CPU::UNUSED_CB() { UNIMPLEMENTED(); }
  int CPU::CALL_Z() { UNIMPLEMENTED(); }
  int CPU::CALL() { UNIMPLEMENTED(); }
  int CPU::ADC_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_08() { UNIMPLEMENTED(); }
  int CPU::RET_NC() { UNIMPLEMENTED(); }
  int CPU::POP_DE() { UNIMPLEMENTED(); }
  int CPU::JP_NC() { UNIMPLEMENTED(); }
  int CPU::UNUSED_D3() { UNIMPLEMENTED(); }
  int CPU::CALL_NC() { UNIMPLEMENTED(); }
  int CPU::PUSH_DE() { UNIMPLEMENTED(); }
  int CPU::SUB_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_10() { UNIMPLEMENTED(); }
  int CPU::RET_C() { UNIMPLEMENTED(); }
  int CPU::RETI() { UNIMPLEMENTED(); }
  int CPU::JP_C() { UNIMPLEMENTED(); }
  int CPU::UNUSED_DB() { UNIMPLEMENTED(); }
  int CPU::CALL_C() { UNIMPLEMENTED(); }
  int CPU::UNUSED_DD() { UNIMPLEMENTED(); }
  int CPU::SBC_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_18() { UNIMPLEMENTED(); }
  int CPU::LDH_N_A() { UNIMPLEMENTED(); }
  int CPU::POP_HL() { UNIMPLEMENTED(); }
  int CPU::LDH_C_A() { UNIMPLEMENTED(); }
  int CPU::UNUSED_E3() { UNIMPLEMENTED(); }
  int CPU::UNUSED_E4() { UNIMPLEMENTED(); }
  int CPU::PUSH_HL() { UNIMPLEMENTED(); }
  int CPU::AND_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_20() { UNIMPLEMENTED(); }
  int CPU::ADD_SP_I8() { UNIMPLEMENTED(); }
  int CPU::JP_HL() { UNIMPLEMENTED(); }
  int CPU::LD_NN_A() { UNIMPLEMENTED(); }
  int CPU::UNUSED_EB() { UNIMPLEMENTED(); }
  int CPU::UNUSED_EC() { UNIMPLEMENTED(); }
  int CPU::UNUSED_ED() { UNIMPLEMENTED(); }
  int CPU::XOR_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_28() { UNIMPLEMENTED(); }
  int CPU::LDH_A_N() { UNIMPLEMENTED(); }
  int CPU::POP_AF() { UNIMPLEMENTED(); }
  int CPU::LDH_A_C() { UNIMPLEMENTED(); }
  int CPU::DI() { UNIMPLEMENTED(); }
  int CPU::UNUSED_F4() { UNIMPLEMENTED(); }
  int CPU::PUSH_AF() { UNIMPLEMENTED(); }
  int CPU::OR_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_30() { UNIMPLEMENTED(); }
  int CPU::LD_HL_SP_I8() { UNIMPLEMENTED(); }
  int CPU::LD_SP_HL() { UNIMPLEMENTED(); }
  int CPU::LD_A_NN() { UNIMPLEMENTED(); }
  int CPU::EI() { UNIMPLEMENTED(); }
  int CPU::UNUSED_FC() { UNIMPLEMENTED(); }
  int CPU::UNUSED_FD() { UNIMPLEMENTED(); }
  int CPU::CP_A_N() { UNIMPLEMENTED(); }
  int CPU::RST_38() { UNIMPLEMENTED(); }



  void CPU::reset() {
    af_ = 0x01B0;
    bc_ = 0x0013;
    de_ = 0x00D8;
    hl_ = 0x014D;
    pc_ = 0x0100;
    sp_ = 0xFFFE;
  }

  auto CPU::step() -> int {
    UNIMPLEMENTED();
  }

  auto CPU::pc() -> const u16 { return pc_; }
  
  auto CPU::sp() -> const u16 { return sp_; }

  auto CPU::ra() -> const u8 { UNIMPLEMENTED(); }

  auto CPU::fetch8() -> u8 { return mem_.read8(pc_++); }

  auto CPU::fetch16() -> u16 { return mem_.read16(pc_++); }

  void CPU::executeOpcode(u8 opcode) { UNIMPLEMENTED(); }

  void CPU::handleInterrupts() { UNIMPLEMENTED(); }

} // namespace gb