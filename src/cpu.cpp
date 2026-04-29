/**
 * @file cpu.cpp
 * @brief Implements the CPU class: registers, opcode, decoding, ALU, flags, interrupts.
 *
 * This file implements the FETCH/DECODE/EXECUTE pipeline logic for the GameBoy's
 * SMLR35902 CPU which follows the SM83 instruction set.
 */

#include "cpu.hpp"
#include "utils.hpp"

namespace gb {

  // r8 index: 0=B 1=C 2=D 3=E 4=H 5=L 6=[HL] 7=A
  // Index 6 is [HL] — a memory access through HL, not a real register.
  static constexpr int kR8_B     = 0;
  static constexpr int kR8_C     = 1;
  static constexpr int kR8_D     = 2;
  static constexpr int kR8_E     = 3;
  static constexpr int kR8_H     = 4;
  static constexpr int kR8_L     = 5;
  static constexpr int kR8_IndHL = 6;
  static constexpr int kR8_A     = 7;

  // r16 index: 0=BC 1=DE 2=HL 3=SP  (used by LD r16,n16 / INC / DEC / ADD HL,r16)
  static constexpr int kR16_BC = 0;
  static constexpr int kR16_DE = 1;
  static constexpr int kR16_HL = 2;
  static constexpr int kR16_SP = 3;

  // r16stk index: 0=BC 1=DE 2=HL 3=AF  (used by PUSH / POP; index 3 is AF, not SP)
  static constexpr int kR16Stk_BC = 0;
  static constexpr int kR16Stk_DE = 1;
  static constexpr int kR16Stk_HL = 2;
  static constexpr int kR16Stk_AF = 3;

  CPU::CPU(Memory& mem) : mem_(mem) {}

  // Returns a reference to the actual register variable. Never call with kR8_IndHL.
  auto CPU::getR8Ref(int idx) -> u8& {
    switch (idx) {
      case kR8_B: return b_;
      case kR8_C: return c_;
      case kR8_D: return d_;
      case kR8_E: return e_;
      case kR8_H: return h_;
      case kR8_L: return l_;
      case kR8_A: return a_;
      default: {
        static u8 dummy = 0;
        return dummy;
      }
    }
  }

  // Reads an r8 operand. idx==kR8_IndHL reads from mem[HL].
  auto CPU::getR8(int idx) -> u8 {
    if (idx == kR8_IndHL) return mem_.read8(hl_);
    return getR8Ref(idx);
  }

  // Writes an r8 operand. idx==kR8_IndHL writes to mem[HL].
  void CPU::setR8(int idx, u8 val) {
    if (idx == kR8_IndHL) {
      mem_.write8(hl_, val);
    } else {
      getR8Ref(idx) = val;
    }
  }

  // r16 encoding: BC/DE/HL/SP. Used by LD r16,n16 / INC / DEC / ADD HL,r16.
  auto CPU::getR16(int idx) -> u16& {
    switch (idx) {
      case kR16_BC: return bc_;
      case kR16_DE: return de_;
      case kR16_HL: return hl_;
      case kR16_SP: return sp_;
      default: {
        static u16 dummy = 0;
        return dummy;
      }
    }
  }

  // r16stk encoding: BC/DE/HL/AF. Used by PUSH / POP.
  auto CPU::getR16stk(int idx) -> u16& {
    switch (idx) {
      case kR16Stk_BC: return bc_;
      case kR16Stk_DE: return de_;
      case kR16Stk_HL: return hl_;
      case kR16Stk_AF: return af_;
      default: {
        static u16 dummy = 0;
        return dummy;
      }
    }
  }

  auto CPU::getCondition(int cc) -> bool {
    switch (cc) {
      case 0: return (f_ & FLAG_Z) == 0; // NZ
      case 1: return (f_ & FLAG_Z) != 0; // Z
      case 2: return (f_ & FLAG_C) == 0; // NC
      case 3: return (f_ & FLAG_C) != 0; // C
      default:
        UNIMPLEMENTED();
    }
  }

  auto CPU::ADC_A_n8() -> int {
    u8 carry_in = (f_ & FLAG_C) ? 1 : 0;
    u8 value = fetch8();
    u16 sum = a_ + value + carry_in;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) + carry_in > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return 8;
  }

  auto CPU::ADC_A_r8(int src) -> int {
    u8 carry_in = (f_ & FLAG_C) ? 1 : 0;
    u8 value = getR8(src);
    u16 sum = a_ + value + carry_in;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) + carry_in > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::ADD_A_n8() -> int {
    u8 value = fetch8();
    u16 sum = a_ + value;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return 8;
  }

  auto CPU::ADD_A_r8(int src) -> int {
    u8 value = getR8(src);
    u16 sum = a_ + value;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::ADD_HL_r16(int reg) -> int {
    u16 value = getR16(reg);
    u32 result = hl_ + value;

    f_ &= FLAG_Z;

    if ((hl_ & 0x0FFF) + (value & 0x0FFF) > 0x0FFF) {
      f_ |= FLAG_H;
    }

    if (result > 0xFFFF) {
      f_ |= FLAG_C;
    }

    hl_ = static_cast<u16>(result);
    return 8;
  }

  auto CPU::AND_A_n8() -> int {
    u8 value = fetch8();
    u8 result = a_ & value;

    f_ = FLAG_H;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return 8;
  }

  auto CPU::AND_A_r8(int src) -> int {
    u8 value = getR8(src);
    u8 result = a_ & value;

    f_ = FLAG_H;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::CALL() -> int {
    u16 addr = fetch16();
    sp_ -= 2;
    mem_.write16(sp_, pc_);
    pc_ = addr;
    return 24;
  }

  auto CPU::CALL_cc(bool condition) -> int {
    u16 addr = fetch16();
    if (condition) {
      // Push PC onto stack
      sp_ -= 2;
      mem_.write16(sp_, pc_);
      pc_ = addr;
      return 24; // 6 cycles if called
    }

    return 12; // 3 cycles if not called
  }

  auto CPU::CP_A_n8() -> int {
    u8 value = fetch8();
    u16 diff = a_ - value;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    return 8;
  }

  auto CPU::CP_A_r8(int src) -> int {
    u8 value = getR8(src);
    u16 diff = a_ - value;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::DEC_r8(int reg) -> int {
    u8 oldValue = getR8(reg);
    u8 newValue = oldValue - 1;

    f_ = (f_ & FLAG_C) | FLAG_N;
    if (newValue == 0) f_ |= FLAG_Z;
    if ((oldValue & 0x0F) == 0) f_ |= FLAG_H;

    setR8(reg, newValue);
    return (reg == kR8_IndHL) ? 12 : 4;
  }

  auto CPU::DEC_r16(int reg) -> int {
    getR16(reg)--;
    return 8;
  }

  auto CPU::INC_r8(int reg) -> int {
    u8 oldValue = getR8(reg);
    u8 newValue = oldValue + 1;

    f_ &= FLAG_C;
    if (newValue == 0) f_ |= FLAG_Z;
    if ((oldValue & 0x0F) + 1 > 0x0F) f_ |= FLAG_H;

    setR8(reg, newValue);
    return (reg == kR8_IndHL) ? 12 : 4;
  }

  auto CPU::INC_r16(int reg) -> int {
    getR16(reg)++;
    return 8;
  }

  auto CPU::JP() -> int {
    u16 addr = fetch16();
    pc_ = addr;
    return 16;
  }

  auto CPU::JP_cc(bool condition) -> int {
    u16 addr = fetch16();
    if (condition) {
      pc_ = addr;
      return 16;
    }

    return 12;
  }

  auto CPU::JR() -> int {
    i8 offset = static_cast<i8>(fetch8());
    pc_ += offset;
    return 12;
  }
      
  auto CPU::JR_cc(bool condition) -> int {
    i8 offset = static_cast<i8>(fetch8());
    if (condition) {
      pc_ += offset;
      return 12;
    }

    return 8;
  }

  auto CPU::LD_A_IndBC() -> int { a_ = mem_.read8(bc_); return 8; }
  auto CPU::LD_A_IndDE() -> int { a_ = mem_.read8(de_); return 8; }
  auto CPU::LD_A_HLD()   -> int { a_ = mem_.read8(hl_--); return 8; }
  auto CPU::LD_A_HLI()   -> int { a_ = mem_.read8(hl_++); return 8; }

  auto CPU::LD_IndBC_A() -> int { mem_.write8(bc_, a_); return 8; }
  auto CPU::LD_IndDE_A() -> int { mem_.write8(de_, a_); return 8; }
  auto CPU::LD_HLD_A()   -> int { mem_.write8(hl_--, a_); return 8; }
  auto CPU::LD_HLI_A()   -> int { mem_.write8(hl_++, a_); return 8; }

  auto CPU::LD_r8_n8(int dst) -> int {
    setR8(dst, fetch8());
    return (dst == kR8_IndHL) ? 12 : 8;
  }

  auto CPU::LD_r8_r8(int dst, int src) -> int {
    setR8(dst, getR8(src));
    return (dst == kR8_IndHL || src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::LD_r16_n16(int dst) -> int {
    getR16(dst) = fetch16();
    return 12;
  }

  auto CPU::LD_n16_SP() -> int {
    mem_.write16(fetch16(), sp_);
    return 20;
  }

  // High-memory loads: LDH uses a 1-byte offset into 0xFF00 page.
  // LD (without H) uses a full 16-bit address.
  auto CPU::LDH_n8_A()  -> int { mem_.write8(0xFF00 + fetch8(), a_); return 12; }
  auto CPU::LD_IndC_A() -> int { mem_.write8(0xFF00 + c_, a_);       return 8;  }
  auto CPU::LD_n16_A()  -> int { mem_.write8(fetch16(), a_);         return 16; }
  auto CPU::LDH_A_n8()  -> int { a_ = mem_.read8(0xFF00 + fetch8()); return 12; }
  auto CPU::LD_A_IndC() -> int { a_ = mem_.read8(0xFF00 + c_);       return 8;  }
  auto CPU::LD_A_n16()  -> int { a_ = mem_.read8(fetch16());         return 16; }

  auto CPU::RLCA() -> int {
    const u8 old = a_;
    const bool carry = (old & 0x80) != 0;

    a_ = static_cast<u8>((old << 1) | (carry ? 1 : 0));

    f_ = 0;
    if (carry) { f_ |= FLAG_C; }

    return 4;
  }

  auto CPU::RRCA() -> int {
    const u8 old = a_;
    const bool carry = (old & 0x1) != 0;

    a_ = static_cast<u8>((old >> 1) | (carry ? 0x80 : 0));

    f_ = 0;
    if (carry) { f_ |= FLAG_C; }

    return 4;
  }

  auto CPU::RLA() -> int {
    const u8 old = a_;
    const bool oldC = (f_ & FLAG_C) != 0;
    const bool carry = (old & 0x80) != 0;

    a_ = static_cast<u8>((old << 1) | (oldC ? 1 : 0));

    f_ = 0;
    if (carry) { f_ |= FLAG_C; }

    return 4;
  }

  auto CPU::RRA() -> int {
    const u8 old = a_;
    const bool oldC = (f_ & FLAG_C) != 0;
    const bool oldBit0 = (old & 0x1) != 0;

    a_ = static_cast<u8>((old >> 1) | (oldC ? 0x80 : 0));

    f_ = 0;
    if (oldBit0) { f_ |= FLAG_C; }

    return 4;
  }

  auto CPU::DAA() -> int {
    u8 adjustment = 0;

    bool c = (f_ & FLAG_C) != 0;
    if (f_ & FLAG_N) { // operation was a subtraction (N == 1)
      if (f_ & FLAG_H) {
        adjustment += static_cast<u8>(0x06);
      }

      if (f_ & FLAG_C) {
        adjustment += static_cast<u8>(0x60);
      }

      a_ -= adjustment;
    }
    else { // operation was an addition (N == 0)
      if ((f_ & FLAG_H) || ((a_ & 0xF) > 0x9)) {
        adjustment += static_cast<u8>(0x06);
      }

      if ((f_ & FLAG_C) || (a_ > 0x99)) {
        adjustment += static_cast<u8>(0x60);
        c = true;
      }

      a_ += adjustment;
    }

    const bool z = a_ == 0;
    const bool n = (f_ & FLAG_N) != 0;

    f_ = (n ? FLAG_N : 0) | (z ? FLAG_Z : 0) | (c ? FLAG_C : 0);

    return 4;
  }

  auto CPU::SCF() -> int {
    const bool z = (f_ & FLAG_Z) != 0;
    f_ = (z ? FLAG_Z : 0) | FLAG_C;

    return 4;
  }

  auto CPU::CPL() -> int {
    a_ = ~a_;
    f_ |= FLAG_N | FLAG_H;

    return 4;
  }

  auto CPU::CCF() -> int {
    const bool z = (f_ & FLAG_Z) != 0;
    const bool c = (f_ & FLAG_C) != 0;
    f_ = (z ? FLAG_Z : 0) | (c ? 0 : FLAG_C);

    return 4;
  }

  auto CPU::OR_A_n8() -> int {
    u8 value = fetch8();
    u8 result = a_ | value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return 8;
  }

  auto CPU::OR_A_r8(int src) -> int {
    u8 value = getR8(src);
    u8 result = a_ | value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::POP_r16stk(int idx) -> int {
    getR16stk(idx) = mem_.read16(sp_);
    sp_ += 2;
    if (idx == kR16Stk_AF) f_ &= 0xF0; // low nibble of F is always 0
    return 12;
  }

  auto CPU::PUSH_r16stk(u16 value) -> int {
    sp_ -= 2;
    mem_.write16(sp_, value);
    return 16;
  }

  auto CPU::RET() -> int {
    pc_ = mem_.read16(sp_);
    sp_ += 2;
    return 20;
  }

  auto CPU::RET_cc(bool condition) -> int {
    if (condition) {
      pc_ = mem_.read16(sp_);
      sp_ += 2;
      return 20;
    }

    return 8;
  }

  auto CPU::RST(u8 vector) -> int {
    sp_ -= 2;
    mem_.write16(sp_, pc_);
    pc_ = vector;
    return 16;
  }

  auto CPU::SBC_A_n8() -> int {
    u8 carry_in = (f_ & FLAG_C) ? 1 : 0;
    u8 value = fetch8();
    u16 total_sub = static_cast<u16>(value) + carry_in;
    u8 diff = a_ - value - carry_in;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < ((value & 0x0F) + carry_in)) f_ |= FLAG_H;
    if (a_ < total_sub) f_ |= FLAG_C;

    a_ = diff & 0xFF;
    return 8;
  }

  auto CPU::SBC_A_r8(int src) -> int {
    u8 carry_in = (f_ & FLAG_C) ? 1 : 0;
    u8 value = getR8(src);
    u16 total_sub = static_cast<u16>(value) + carry_in;
    u8 diff = a_ - value - carry_in;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < ((value & 0x0F) + carry_in)) f_ |= FLAG_H;
    if (a_ < total_sub) f_ |= FLAG_C;

    a_ = diff & 0xFF;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::SUB_A_n8() -> int {
    u8 value = fetch8();
    u8 result = a_ - value;

    f_ = FLAG_N;
    if (result == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    a_ = result;
    return 8;
  }

  auto CPU::SUB_A_r8(int src) -> int {
    u8 value = getR8(src);
    u16 diff = a_ - value;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    a_ = diff & 0xFF;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::XOR_A_n8() -> int {
    u8 value = fetch8();
    u8 result = a_ ^ value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return 8;
  }

  auto CPU::XOR_A_r8(int src) -> int {
    u8 value = getR8(src);
    u8 result = a_ ^ value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return (src == kR8_IndHL) ? 8 : 4;
  }

  auto CPU::CB_RLC(int reg) -> int {
    const u8 val = getR8(reg);
    const bool bit7 = (val & 0x80) != 0;
    const u8 result = (val << 1) | (bit7 ? 1 : 0);
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit7 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_RRC(int reg) -> int {
    const u8 val = getR8(reg);
    const bool bit0 = (val & 0x01) != 0;
    const u8 result = (val >> 1) | (bit0 ? 0x80 : 0);
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit0 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_RL(int reg) -> int {
    const u8 val = getR8(reg);
    const bool old_C = (f_ & FLAG_C) != 0;
    const bool bit7 = (val & 0x80) != 0;
    const u8 result = (val << 1) | (old_C ? 1 : 0);
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit7 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_RR(int reg) -> int {
    const u8 val = getR8(reg);
    const bool old_C = (f_ & FLAG_C) != 0;
    const bool bit0 = (val & 0x01) != 0;
    const u8 result = (val >> 1) | (old_C ? 0x80 : 0);
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit0 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_SLA(int reg) -> int {
    const u8 val = getR8(reg);
    const bool bit7 = (val & 0x80) != 0;
    const u8 result = val << 1;
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit7 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_SRA(int reg) -> int {
    const u8 val = getR8(reg);
    const bool bit0 = (val & 0x01) != 0;
    const u8 result = (val >> 1) | (val & 0x80);
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit0 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_SWAP(int reg) -> int {
    const u8 val = getR8(reg);
    const u8 result = ((val & 0x0F) << 4) | (val >> 4);
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::CB_SRL(int reg) -> int {
    const u8 val = getR8(reg);
    const bool bit0 = (val & 0x01) != 0;
    const u8 result = val >> 1;
    setR8(reg, result);
    f_ = (result == 0 ? FLAG_Z : 0) | (bit0 ? FLAG_C : 0);
    return (reg == kR8_IndHL) ? 16 : 8;
  }

  auto CPU::execute_CB(u8 op) -> int {
    const int reg = op & 0x07;
    const int operand = (op >> 3) & 0x07;
    const int group = op >> 6;

    if (group == 1) { // BIT b, r8
      const u8 val = getR8(reg);
      f_ &= FLAG_C;
      f_ |= FLAG_H;
      if (((val >> operand) & 1) == 0) { f_ |= FLAG_Z; }
      return (reg == kR8_IndHL) ? 12 : 8;
    }

    if (group == 2) { // RES b, r8
      setR8(reg, getR8(reg) & ~(1 << operand));
      return (reg == kR8_IndHL) ? 16 : 8;
    }

    if (group == 3) { // SET b, r8
      setR8(reg, getR8(reg) | (1 << operand));
      return (reg == kR8_IndHL) ? 16 : 8;
    }

    switch (operand) {
      case 0: return CB_RLC(reg);
      case 1: return CB_RRC(reg);
      case 2: return CB_RL(reg);
      case 3: return CB_RR(reg);
      case 4: return CB_SLA(reg);
      case 5: return CB_SRA(reg);
      case 6: return CB_SWAP(reg);
      case 7: return CB_SRL(reg);
      default: UNIMPLEMENTED();
    }
  }

  void CPU::reset() {
    af_ = 0x01B0;
    bc_ = 0x0013;
    de_ = 0x00D8;
    hl_ = 0x014D;
    pc_ = 0x0100;
    sp_ = 0xFFFE;
    halted_ = false;
    stopped_ = false;
  }

  auto CPU::step() -> int {
    if (halted_ || stopped_) {
      // Minimal behavior: HALT stops instruction execution until an interrupt.
      // Interrupt wakeup handling is not implemented yet.
      return 4;
    }

    const u8 opcode = fetch8();

    switch (opcode) {
      // 0x00-0x0F
      case 0x00: return 4; // NOP
      case 0x01: return LD_r16_n16(kR16_BC); // LD BC, n16
      case 0x02: return LD_IndBC_A(); // LD [BC], A
      case 0x03: return INC_r16(kR16_BC); // INC BC
      case 0x04: return INC_r8(kR8_B); // INC B
      case 0x05: return DEC_r8(kR8_B); // DEC B
      case 0x06: return LD_r8_n8(kR8_B); // LD B, n8
      case 0x07: return RLCA(); // RLCA
      case 0x08: return LD_n16_SP(); // LD [n16], SP
      case 0x09: return ADD_HL_r16(kR16_BC); // ADD HL, BC
      case 0x0A: return LD_A_IndBC(); // LD A, [BC]
      case 0x0B: return DEC_r16(kR16_BC); // DEC BC
      case 0x0C: return INC_r8(kR8_C); // INC C
      case 0x0D: return DEC_r8(kR8_C); // DEC C
      case 0x0E: return LD_r8_n8(kR8_C); // LD C, n8
      case 0x0F: return RRCA(); // RRCA
      // 0x10-0x1F
      case 0x10: return STOP(); // STOP
      case 0x11: return LD_r16_n16(kR16_DE); // LD DE, n16
      case 0x12: return LD_IndDE_A(); // LD [DE], A
      case 0x13: return INC_r16(kR16_DE); // INC DE
      case 0x14: return INC_r8(kR8_D); // INC D
      case 0x15: return DEC_r8(kR8_D); // DEC D
      case 0x16: return LD_r8_n8(kR8_D); // LD D, n8
      case 0x17: return RLA(); // RLA
      case 0x18: return JR(); // JR e8
      case 0x19: return ADD_HL_r16(kR16_DE); // ADD HL, DE
      case 0x1A: return LD_A_IndDE(); // LD A, [DE]
      case 0x1B: return DEC_r16(kR16_DE); // DEC DE
      case 0x1C: return INC_r8(kR8_E); // INC E
      case 0x1D: return DEC_r8(kR8_E); // DEC E
      case 0x1E: return LD_r8_n8(kR8_E); // LD E, n8
      case 0x1F: return RRA(); // RRA
      // 0x20-0x2F
      case 0x20: return JR_cc((f_ & FLAG_Z) == 0); // JR NZ, e8
      case 0x21: return LD_r16_n16(kR16_HL); // LD HL, n16
      case 0x22: return LD_HLI_A(); // LD [HL+], A
      case 0x23: return INC_r16(kR16_HL); // INC HL
      case 0x24: return INC_r8(kR8_H); // INC H
      case 0x25: return DEC_r8(kR8_H); // DEC H
      case 0x26: return LD_r8_n8(kR8_H); // LD H, n8
      case 0x27: return DAA(); // DAA
      case 0x28: return JR_cc((f_ & FLAG_Z) != 0); // JR Z, e8
      case 0x29: return ADD_HL_r16(kR16_HL); // ADD HL, HL
      case 0x2A: return LD_A_HLI(); // LD A, [HL+]
      case 0x2B: return DEC_r16(kR16_HL); // DEC HL
      case 0x2C: return INC_r8(kR8_L); // INC L
      case 0x2D: return DEC_r8(kR8_L); // DEC L
      case 0x2E: return LD_r8_n8(kR8_L); // LD L, n8
      case 0x2F: return CPL(); // CPL
      // 0x30-0x3F
      case 0x30: return JR_cc((f_ & FLAG_C) == 0); // JR NC, e8
      case 0x31: return LD_r16_n16(kR16_SP); // LD SP, n16
      case 0x32: return LD_HLD_A(); // LD [HL-], A
      case 0x33: return INC_r16(kR16_SP); // INC SP
      case 0x34: return INC_r8(kR8_IndHL); // INC [HL]
      case 0x35: return DEC_r8(kR8_IndHL); // DEC [HL]
      case 0x36: return LD_r8_n8(kR8_IndHL); // LD [HL], n8
      case 0x37: return SCF(); // SCF
      case 0x38: return JR_cc((f_ & FLAG_C) != 0); // JR C, e8
      case 0x39: return ADD_HL_r16(kR16_SP); // ADD HL, SP
      case 0x3A: return LD_A_HLD(); // LD A, [HL-]
      case 0x3B: return DEC_r16(kR16_SP); // DEC SP
      case 0x3C: return INC_r8(kR8_A); // INC A
      case 0x3D: return DEC_r8(kR8_A); // DEC A
      case 0x3E: return LD_r8_n8(kR8_A); // LD A, n8
      case 0x3F: return CCF(); // CCF
      // 0x40-0x7F: LD r8, r8  (dst = bits[5:3], src = bits[2:0])
      // 0x76 is HALT — it sits where LD [HL],[HL] would be.
      case 0x40: return LD_r8_r8(kR8_B, kR8_B);
      case 0x41: return LD_r8_r8(kR8_B, kR8_C);
      case 0x42: return LD_r8_r8(kR8_B, kR8_D);
      case 0x43: return LD_r8_r8(kR8_B, kR8_E);
      case 0x44: return LD_r8_r8(kR8_B, kR8_H);
      case 0x45: return LD_r8_r8(kR8_B, kR8_L);
      case 0x46: return LD_r8_r8(kR8_B, kR8_IndHL);
      case 0x47: return LD_r8_r8(kR8_B, kR8_A);
      case 0x48: return LD_r8_r8(kR8_C, kR8_B);
      case 0x49: return LD_r8_r8(kR8_C, kR8_C);
      case 0x4A: return LD_r8_r8(kR8_C, kR8_D);
      case 0x4B: return LD_r8_r8(kR8_C, kR8_E);
      case 0x4C: return LD_r8_r8(kR8_C, kR8_H);
      case 0x4D: return LD_r8_r8(kR8_C, kR8_L);
      case 0x4E: return LD_r8_r8(kR8_C, kR8_IndHL);
      case 0x4F: return LD_r8_r8(kR8_C, kR8_A);
      case 0x50: return LD_r8_r8(kR8_D, kR8_B);
      case 0x51: return LD_r8_r8(kR8_D, kR8_C);
      case 0x52: return LD_r8_r8(kR8_D, kR8_D);
      case 0x53: return LD_r8_r8(kR8_D, kR8_E);
      case 0x54: return LD_r8_r8(kR8_D, kR8_H);
      case 0x55: return LD_r8_r8(kR8_D, kR8_L);
      case 0x56: return LD_r8_r8(kR8_D, kR8_IndHL);
      case 0x57: return LD_r8_r8(kR8_D, kR8_A);
      case 0x58: return LD_r8_r8(kR8_E, kR8_B);
      case 0x59: return LD_r8_r8(kR8_E, kR8_C);
      case 0x5A: return LD_r8_r8(kR8_E, kR8_D);
      case 0x5B: return LD_r8_r8(kR8_E, kR8_E);
      case 0x5C: return LD_r8_r8(kR8_E, kR8_H);
      case 0x5D: return LD_r8_r8(kR8_E, kR8_L);
      case 0x5E: return LD_r8_r8(kR8_E, kR8_IndHL);
      case 0x5F: return LD_r8_r8(kR8_E, kR8_A);
      case 0x60: return LD_r8_r8(kR8_H, kR8_B);
      case 0x61: return LD_r8_r8(kR8_H, kR8_C);
      case 0x62: return LD_r8_r8(kR8_H, kR8_D);
      case 0x63: return LD_r8_r8(kR8_H, kR8_E);
      case 0x64: return LD_r8_r8(kR8_H, kR8_H);
      case 0x65: return LD_r8_r8(kR8_H, kR8_L);
      case 0x66: return LD_r8_r8(kR8_H, kR8_IndHL);
      case 0x67: return LD_r8_r8(kR8_H, kR8_A);
      case 0x68: return LD_r8_r8(kR8_L, kR8_B);
      case 0x69: return LD_r8_r8(kR8_L, kR8_C);
      case 0x6A: return LD_r8_r8(kR8_L, kR8_D);
      case 0x6B: return LD_r8_r8(kR8_L, kR8_E);
      case 0x6C: return LD_r8_r8(kR8_L, kR8_H);
      case 0x6D: return LD_r8_r8(kR8_L, kR8_L);
      case 0x6E: return LD_r8_r8(kR8_L, kR8_IndHL);
      case 0x6F: return LD_r8_r8(kR8_L, kR8_A);
      case 0x70: return LD_r8_r8(kR8_IndHL, kR8_B);
      case 0x71: return LD_r8_r8(kR8_IndHL, kR8_C);
      case 0x72: return LD_r8_r8(kR8_IndHL, kR8_D);
      case 0x73: return LD_r8_r8(kR8_IndHL, kR8_E);
      case 0x74: return LD_r8_r8(kR8_IndHL, kR8_H);
      case 0x75: return LD_r8_r8(kR8_IndHL, kR8_L);
      case 0x76: return HALT();
      case 0x77: return LD_r8_r8(kR8_IndHL, kR8_A);
      case 0x78: return LD_r8_r8(kR8_A, kR8_B);
      case 0x79: return LD_r8_r8(kR8_A, kR8_C);
      case 0x7A: return LD_r8_r8(kR8_A, kR8_D);
      case 0x7B: return LD_r8_r8(kR8_A, kR8_E);
      case 0x7C: return LD_r8_r8(kR8_A, kR8_H);
      case 0x7D: return LD_r8_r8(kR8_A, kR8_L);
      case 0x7E: return LD_r8_r8(kR8_A, kR8_IndHL);
      case 0x7F: return LD_r8_r8(kR8_A, kR8_A);
      // 0x80-0xBF: ALU A, r8  (op = bits[5:3], src = bits[2:0])
      case 0x80: return ADD_A_r8(kR8_B);
      case 0x81: return ADD_A_r8(kR8_C);
      case 0x82: return ADD_A_r8(kR8_D);
      case 0x83: return ADD_A_r8(kR8_E);
      case 0x84: return ADD_A_r8(kR8_H);
      case 0x85: return ADD_A_r8(kR8_L);
      case 0x86: return ADD_A_r8(kR8_IndHL);
      case 0x87: return ADD_A_r8(kR8_A);
      case 0x88: return ADC_A_r8(kR8_B);
      case 0x89: return ADC_A_r8(kR8_C);
      case 0x8A: return ADC_A_r8(kR8_D);
      case 0x8B: return ADC_A_r8(kR8_E);
      case 0x8C: return ADC_A_r8(kR8_H);
      case 0x8D: return ADC_A_r8(kR8_L);
      case 0x8E: return ADC_A_r8(kR8_IndHL);
      case 0x8F: return ADC_A_r8(kR8_A);
      case 0x90: return SUB_A_r8(kR8_B);
      case 0x91: return SUB_A_r8(kR8_C);
      case 0x92: return SUB_A_r8(kR8_D);
      case 0x93: return SUB_A_r8(kR8_E);
      case 0x94: return SUB_A_r8(kR8_H);
      case 0x95: return SUB_A_r8(kR8_L);
      case 0x96: return SUB_A_r8(kR8_IndHL);
      case 0x97: return SUB_A_r8(kR8_A);
      case 0x98: return SBC_A_r8(kR8_B);
      case 0x99: return SBC_A_r8(kR8_C);
      case 0x9A: return SBC_A_r8(kR8_D);
      case 0x9B: return SBC_A_r8(kR8_E);
      case 0x9C: return SBC_A_r8(kR8_H);
      case 0x9D: return SBC_A_r8(kR8_L);
      case 0x9E: return SBC_A_r8(kR8_IndHL);
      case 0x9F: return SBC_A_r8(kR8_A);
      case 0xA0: return AND_A_r8(kR8_B);
      case 0xA1: return AND_A_r8(kR8_C);
      case 0xA2: return AND_A_r8(kR8_D);
      case 0xA3: return AND_A_r8(kR8_E);
      case 0xA4: return AND_A_r8(kR8_H);
      case 0xA5: return AND_A_r8(kR8_L);
      case 0xA6: return AND_A_r8(kR8_IndHL);
      case 0xA7: return AND_A_r8(kR8_A);
      case 0xA8: return XOR_A_r8(kR8_B);
      case 0xA9: return XOR_A_r8(kR8_C);
      case 0xAA: return XOR_A_r8(kR8_D);
      case 0xAB: return XOR_A_r8(kR8_E);
      case 0xAC: return XOR_A_r8(kR8_H);
      case 0xAD: return XOR_A_r8(kR8_L);
      case 0xAE: return XOR_A_r8(kR8_IndHL);
      case 0xAF: return XOR_A_r8(kR8_A);
      case 0xB0: return OR_A_r8(kR8_B);
      case 0xB1: return OR_A_r8(kR8_C);
      case 0xB2: return OR_A_r8(kR8_D);
      case 0xB3: return OR_A_r8(kR8_E);
      case 0xB4: return OR_A_r8(kR8_H);
      case 0xB5: return OR_A_r8(kR8_L);
      case 0xB6: return OR_A_r8(kR8_IndHL);
      case 0xB7: return OR_A_r8(kR8_A);
      case 0xB8: return CP_A_r8(kR8_B);
      case 0xB9: return CP_A_r8(kR8_C);
      case 0xBA: return CP_A_r8(kR8_D);
      case 0xBB: return CP_A_r8(kR8_E);
      case 0xBC: return CP_A_r8(kR8_H);
      case 0xBD: return CP_A_r8(kR8_L);
      case 0xBE: return CP_A_r8(kR8_IndHL);
      case 0xBF: return CP_A_r8(kR8_A);
      // 0xC0-0xFF
      case 0xC0: return RET_cc((f_ & FLAG_Z) == 0); // RET NZ
      case 0xC1: return POP_r16stk(kR16Stk_BC); // POP BC
      case 0xC2: return JP_cc((f_ & FLAG_Z) == 0); // JP NZ, n16
      case 0xC3: return JP(); // JP n16
      case 0xC4: return CALL_cc((f_ & FLAG_Z) == 0); // CALL NZ, n16
      case 0xC5: return PUSH_r16stk(bc_); // PUSH BC
      case 0xC6: return ADD_A_n8(); // ADD A, n8
      case 0xC7: return RST(0x00); // RST $00
      case 0xC8: return RET_cc((f_ & FLAG_Z) != 0); // RET Z
      case 0xC9: return RET(); // RET
      case 0xCA: return JP_cc((f_ & FLAG_Z) != 0); // JP Z, n16
      case 0xCB: execute_CB(fetch8()); // CB prefix (bit ops)
      case 0xCC: return CALL_cc((f_ & FLAG_Z) != 0); // CALL Z, n16
      case 0xCD: return CALL(); // CALL n16
      case 0xCE: return ADC_A_n8(); // ADC A, n8
      case 0xCF: return RST(0x08); // RST $08
      case 0xD0: return RET_cc((f_ & FLAG_C) == 0); // RET NC
      case 0xD1: return POP_r16stk(kR16Stk_DE); // POP DE
      case 0xD2: return JP_cc((f_ & FLAG_C) == 0); // JP NC, n16
      case 0xD4: return CALL_cc((f_ & FLAG_C) == 0); // CALL NC, n16
      case 0xD5: return PUSH_r16stk(de_); // PUSH DE
      case 0xD6: return SUB_A_n8(); // SUB A, n8
      case 0xD7: return RST(0x10); // RST $10
      case 0xD8: return RET_cc((f_ & FLAG_C) != 0); // RET C
      case 0xD9: return RETI(); // RETI
      case 0xDA: return JP_cc((f_ & FLAG_C) != 0); // JP C, n16
      case 0xDC: return CALL_cc((f_ & FLAG_C) != 0); // CALL C, n16
      case 0xDE: return SBC_A_n8(); // SBC A, n8
      case 0xDF: return RST(0x18); // RST $18
      case 0xE0: return LDH_n8_A(); // LDH [n8], A
      case 0xE1: return POP_r16stk(kR16Stk_HL); // POP HL
      case 0xE2: return LD_IndC_A(); // LD [C], A
      case 0xE5: return PUSH_r16stk(hl_); // PUSH HL
      case 0xE6: return AND_A_n8(); // AND A, n8
      case 0xE7: return RST(0x20); // RST $20
      case 0xE8: UNIMPLEMENTED(); // ADD SP, e8 - TODO
      case 0xE9: pc_ = hl_; return 4; // JP HL
      case 0xEA: return LD_n16_A(); // LD [n16], A
      case 0xEE: return XOR_A_n8(); // XOR A, n8
      case 0xEF: return RST(0x28); // RST $28
      case 0xF0: return LDH_A_n8(); // LDH A, [n8]
      case 0xF1: return POP_r16stk(kR16Stk_AF); // POP AF
      case 0xF2: return LD_A_IndC(); // LD A, [C]
      case 0xF3: UNIMPLEMENTED(); // DI - TODO
      case 0xF5: return PUSH_r16stk(af_); // PUSH AF
      case 0xF6: return OR_A_n8(); // OR A, n8
      case 0xF7: return RST(0x30); // RST $30
      case 0xF8: UNIMPLEMENTED(); // LD HL, SP+e8 - TODO
      case 0xF9: sp_ = hl_; return 8; // LD SP, HL
      case 0xFA: return LD_A_n16(); // LD A, [n16]
      case 0xFB: UNIMPLEMENTED(); // EI - TODO
      case 0xFE: return CP_A_n8(); // CP A, n8
      case 0xFF: return RST(0x38); // RST $38
      default:
        UNIMPLEMENTED();
    }
  }

  auto CPU::HALT() -> int {
    halted_ = true;
    return 4;
  }

  auto CPU::RETI() -> int {
    pc_ = mem_.read16(sp_);
    sp_ += 2;
    // TODO: re-enable IME when interrupt handling is implemented
    return 16;
  }

  auto CPU::STOP() -> int {
    // STOP on DMG is encoded as 0x10 0x00. Consume the padding byte.
    // On CGB, STOP also triggers a double-speed switch — not handled yet.
    const u8 padding = fetch8();
    if (padding != 0x00) { UNIMPLEMENTED(); }
    stopped_ = true;
    return 4;
  }

  auto CPU::fetch8() -> u8 { return mem_.read8(pc_++); }

  auto CPU::fetch16() -> u16 {
    const u16 value = mem_.read16(pc_);
    pc_ += 2;
    return value;
  }

  auto CPU::pc() -> const u16 { return pc_; }
  auto CPU::sp() -> const u16 { return sp_; }
  auto CPU::a()  -> const u8  { return a_;  }

  void CPU::handleInterrupts() { UNIMPLEMENTED(); }

} // namespace gb