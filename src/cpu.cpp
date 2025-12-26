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

  CPU::CPU(Memory& mem) : mem_(mem) {}

  auto CPU::getRegister8(int index) -> u8& {
    switch(index) {
      case 0: return b_;
      case 1: return c_;
      case 2: return d_;
      case 3: return e_;
      case 4: return h_;
      case 5: return l_;
      case 7: return a_;
      default: {
        static u8 dummy = 0;
        return dummy; // For (HL) case, not used
      }
    }
  }

  auto CPU::getRegisterValue(int index) -> u8 {
    if (index == 6) return mem_.read8(hl_); // (HL)
    return getRegister8(index);
  }

  void CPU::setRegister8(int index, u8 value) {
    if (index == 6) {
      mem_.write8(hl_, value);
    } else {
      getRegister8(index) = value;
    }
  }

  auto CPU::getRegister16_v1(int index) -> u16& {
    switch(index) {
      case 0: return bc_;
      case 1: return de_;
      case 2: return hl_;
      case 3: return sp_;
      default: {
        static u16 dummy = 0;
        return dummy;
      }
    }
  }

  auto CPU::getRegister16_v2(int index) -> u16& {
    switch(index) {
      case 0: return bc_;
      case 1: return de_;
      case 2: return hl_;
      case 3: return af_;
      default: {
        static u16 dummy = 0;
        return dummy;
      }
    }
  }

  auto CPU::getCondition(int cc) -> bool {
    switch(cc) {
      case 0: return !(f_ & FLAG_Z); // NZ
      case 1: return f_ & FLAG_Z; // Z
      case 2: return !(f_ & FLAG_C); // NC
      case 3: return f_ & FLAG_C; // C
      default:
        UNIMPLEMENTED(); // invalid condition code
    }
  }

  auto CPU::ADC_A_n() -> int {
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

  auto CPU::ADC_A_r(int src) -> int {
    u8 carry_in = (f_ & FLAG_C) ? 1 : 0;
    u8 value = getRegisterValue(src);
    u16 sum = a_ + value + carry_in;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) + carry_in > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return (src == 6) ? 8 : 4;
  }

  auto CPU::ADD_A_n() -> int {
    u8 value = fetch8();
    u16 sum = a_ + value;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return 8;
  }

  auto CPU::ADD_A_r(int src) -> int {
    u8 value = getRegisterValue(src);
    u16 sum = a_ + value;

    f_ = 0;
    if ((sum & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) + (value & 0x0F) > 0x0F) f_ |= FLAG_H;
    if (sum > 0xFF) f_ |= FLAG_C;

    a_ = sum & 0xFF;
    return (src == 6) ? 8 : 4;
  }

  auto CPU::ADD_HL_rr(int reg) -> int {
    u16 value = getRegister16_v1(reg);
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

  auto CPU::AND_A_n() -> int {
    u8 value = fetch8();
    u8 result = a_ & value;

    f_ = FLAG_H;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return 8;
  }

  auto CPU::AND_A_r(int src) -> int {
    u8 value = getRegisterValue(src);
    u8 result = a_ & value;

    f_ = FLAG_H;
    if (result == 0) f_ |= FLAG_Z;
    
    a_ = result;
    return (src == 6) ? 8 : 4;
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

  auto CPU::CP_A_n() -> int {
    u8 value = fetch8();
    u16 diff = a_ - value;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    return 8;
  }

  auto CPU::CP_A_r(int src) -> int {
    u8 value = getRegisterValue(src);
    u16 diff = a_ - value;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    return (src == 6) ? 8 : 4;
  }

  auto CPU::DEC_r(int reg) -> int {
    u8 oldValue = getRegisterValue(reg);
    u8 newValue = oldValue - 1;

    f_ = (f_ & FLAG_C) | FLAG_N;
    if (newValue == 0) f_ |= FLAG_Z;
    if ((oldValue & 0x0F) == 0) f_ |= FLAG_H;

    setRegister8(reg, newValue);
    return (reg == 6) ? 12 : 4;
  }

  auto CPU::DEC_rr(int reg) -> int {
    getRegister16_v1(reg)--;
    return 8;
  }

  auto CPU::INC_r(int reg) -> int {
    u8 oldValue = getRegisterValue(reg);
    u8 newValue = oldValue + 1;

    f_ &= FLAG_C;
    if (newValue == 0) f_ |= FLAG_Z;
    if ((oldValue & 0x0F) + 1 > 0x0F) f_ |= FLAG_H;

    setRegister8(reg, newValue);
    return (reg == 6) ? 12 : 4;
  }

  auto CPU::INC_rr(int reg) -> int {
    getRegister16_v1(reg)++;
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

  auto CPU::LD_A_n() -> int { UNIMPLEMENTED(); }

  auto CPU::LD_A_BC() -> int {
    a_ = mem_.read8(bc_);
    return 8;
  }

  auto CPU::LD_A_DE() -> int {
    a_ = mem_.read8(de_);
    return 8;
  }

  auto CPU::LD_A_HLD() -> int {
    a_ = mem_.read8(hl_--);
    return 8;
  }

  auto CPU::LD_A_HLI() -> int { UNIMPLEMENTED(); }

  auto CPU::LD_BC_A() -> int {
    mem_.write8(bc_, a_);
    return 8;
  }

  auto CPU::LD_DE_A() -> int {
    mem_.write8(de_, a_);
    return 8;
  }

  auto CPU::LD_HLD_A() -> int {
    mem_.write8(hl_--, a_);
    return 8;
  }

  auto CPU::LD_HLI_A() -> int {
    mem_.write8(hl_++, a_);
    return 8;
  }

  auto CPU::LD_r_n(int dst) -> int {
    u8 value = fetch8();
    setRegister8(dst, value);
    return (dst == 6) ? 12 : 8;
  }

  auto CPU::LD_r_r(int dst, int src) -> int {
    u8 value = getRegisterValue(src);
    setRegister8(dst, value);
    return (dst == 6 || src == 6) ? 8 : 4;
  }

  auto CPU::LD_rr_nn(int dst) -> int { UNIMPLEMENTED(); }

  auto CPU::LDH_A_nn() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_A_C() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_C_A() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_n_A() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_nn_A() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_nn_SP() -> int { UNIMPLEMENTED(); }

  auto CPU::OR_A_n() -> int {
    u8 value = fetch8();
    u8 result = a_ | value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return 8;
  }

  auto CPU::OR_A_r(int src) -> int {
    u8 value = getRegisterValue(src);
    u8 result = a_ | value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return (src == 6) ? 8 : 4;
  }

  auto CPU::POP_rr(int idx) -> int {
    u16& reg = getRegister16_v2(idx);
    reg = mem_.read16(sp_);
    sp_ += 2;

    // Special case for POP AF
    if (idx == 3) {
      // TODO: check correctness
      f_ &= 0xF0;
    }

    return 12;
  }

  auto CPU::PUSH_rr(u16 value) -> int {
    sp_ -= 2;
    mem_.write16(sp_, value); // TODO: are we supposed to fetch16() the value instead?
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

  auto CPU::SBC_A_n() -> int {
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

  auto CPU::SBC_A_r(int src) -> int {
    u8 carry_in = (f_ & FLAG_C) ? 1 : 0;
    u8 value = getRegisterValue(src);
    u16 total_sub = static_cast<u16>(value) + carry_in;
    u8 diff = a_ - value - carry_in;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < ((value & 0x0F) + carry_in)) f_ |= FLAG_H;
    if (a_ < total_sub) f_ |= FLAG_C;

    a_ = diff & 0xFF;
    return (src == 6) ? 8 : 4;
  }

  auto CPU::SUB_A_n() -> int {
    u8 value = fetch8();
    u8 result = a_ - value;

    f_ = FLAG_N;
    if (result == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    a_ = result;
    return 8;
  }

  auto CPU::SUB_A_r(int src) -> int {
    u8 value = getRegisterValue(src);
    u16 diff = a_ - value;

    f_ = FLAG_N;
    if ((diff & 0xFF) == 0) f_ |= FLAG_Z;
    if ((a_ & 0x0F) < (value & 0x0F)) f_ |= FLAG_H;
    if (a_ < value) f_ |= FLAG_C;

    a_ = diff & 0xFF;
    return (src == 6) ? 8 : 4;
  }

  auto CPU::XOR_A_n() -> int {
    u8 value = fetch8();
    u8 result = a_ ^ value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return 8;
  }

  auto CPU::XOR_A_r(int src) -> int {
    u8 value = getRegisterValue(src);
    u8 result = a_ ^ value;

    f_ = 0;
    if (result == 0) f_ |= FLAG_Z;

    a_ = result;
    return (src == 6) ? 8 : 4;
  }


  void CPU::reset() {
    af_ = 0x01B0;
    bc_ = 0x0013;
    de_ = 0x00D8;
    hl_ = 0x014D;
    pc_ = 0x0100;
    sp_ = 0xFFFE;
  }

  auto CPU::step() -> int {
    if (halted_) {
      // Minimal behavior: HALT stops instruction execution until an interrupt.
      // Interrupt wakeup handling is not implemented yet.
      return 4;
    }

    const u8 opcode = fetch8();

    switch (opcode) {
      case 0x00: // NOP
        // NOP does nothing; fetch8() already advanced PC by 1.
        return 4;
      case 0xC3: // JP nn
        return JP();

      default:
        UNIMPLEMENTED();
    }
  }

  auto CPU::pc() -> const u16 { return pc_; }
  
  auto CPU::sp() -> const u16 { return sp_; }

  auto CPU::ra() -> const u8 { UNIMPLEMENTED(); }

  auto CPU::fetch8() -> u8 { return mem_.read8(pc_++); }

  auto CPU::fetch16() -> u16 {
    const u16 value = mem_.read16(pc_);
    pc_ += 2;
    return value;
  }

  void CPU::executeOpcode(u8 opcode) { UNIMPLEMENTED(); }

  void CPU::handleInterrupts() { UNIMPLEMENTED(); }

} // namespace gb