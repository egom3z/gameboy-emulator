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

  // 16-bit register pair indexes for "v1" encoding (rr): 0=BC, 1=DE, 2=HL, 3=SP
  static constexpr int RR_BC = 0;
  static constexpr int RR_DE = 1;
  static constexpr int RR_HL = 2;
  static constexpr int RR_SP = 3;

  // 8-bit register indexes for "r" encoding: 0=B,1=C,2=D,3=E,4=H,5=L,6=(HL),7=A
  static constexpr int R_B  = 0;
  static constexpr int R_C  = 1;
  static constexpr int R_D  = 2;
  static constexpr int R_E  = 3;
  static constexpr int R_H  = 4;
  static constexpr int R_L  = 5;
  static constexpr int R_HL = 6; // (HL) memory indirection, not a real register
  static constexpr int R_A  = 7;

  CPU::CPU(Memory& mem) : mem_(mem) {}

  auto CPU::getRegister8(int index) -> u8& {
    switch(index) {
      case R_B: return b_;
      case R_C: return c_;
      case R_D: return d_;
      case R_E: return e_;
      case R_H: return h_;
      case R_L: return l_;
      case R_A: return a_;
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

  // Used for instructions like LD rr,nn or INC/DEC rr or ADD HL,rr
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

  // Used for instructions like PUSH qq or POP qq
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

  auto CPU::LD_A_HLI() -> int {
    a_ = mem_.read8(hl_++);
    return 8;
  }

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

  auto CPU::LD_rr_nn(int dst) -> int {
    u16 value = fetch16();
    getRegister16_v1(dst) = value;
    return 12;
  }

  auto CPU::LD_nn_SP() -> int {
    u16 addr = fetch16();
    mem_.write16(addr, sp_);

    return 20;
  }

  auto CPU::LDH_A_nn() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_A_C() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_C_A() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_n_A() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_nn_A() -> int { UNIMPLEMENTED(); }
  auto CPU::LDH_nn_SP() -> int { UNIMPLEMENTED(); }

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
      case 0x00: // NOP
        return 4;
      case 0x01: // LD BC,d16
        return LD_rr_nn(RR_BC);
      case 0x02: // LD (BC),A
        return LD_BC_A();
      case 0x03: // INC BC
        return INC_rr(RR_BC);
      case 0x04: // INC B
        return INC_r(R_B);
      case 0x05: // DEC B
        return DEC_r(R_B);
      case 0x06: // LD B,d8
        return LD_r_n(R_B);
      case 0x07: // RLCA
        return RLCA();
      case 0x08: // LD (a16),SP
        return LD_nn_SP();
      case 0x09: // ADD HL,BC
        return ADD_HL_rr(RR_BC);
      case 0x0A: // LD A,(BC)
        return LD_A_BC();
      case 0x0B: // DEC BC
        return DEC_rr(RR_BC);
      case 0x0C: // INC C
        return INC_r(R_C);
      case 0x0D: // DEC C
        return DEC_r(R_C);
      case 0x0E: // LD C,d8
        return LD_r_n(R_C);
      case 0x0F: // RRCA
        return RRCA();
      case 0x10: // STOP 0
        return STOP();
      case 0x11: // LD DE,d16
        return LD_rr_nn(RR_DE);
      case 0x12: // LD (DE),A
        return LD_DE_A();
      case 0x13: // INC DE
        return INC_rr(RR_DE);
      case 0x14: // INC D
        return INC_r(R_D);
      case 0x15: // DEC D
        return DEC_r(R_D);
      case 0x16: // LD D,d8
        return LD_r_n(R_D);
      case 0x17: // RLA
        return RLA();
      case 0x18: // JR r8
        return JR();
      case 0x19: // ADD HL,DE
        return ADD_HL_rr(RR_DE);
      case 0x1A: // LD A,(DE)
        return LD_A_DE();
      case 0x1B: // DEC DE
        return DEC_rr(RR_DE);
      case 0x1C: // INC E
        return INC_r(R_E);
      case 0x1D: // DEC E
        return DEC_r(R_E);
      case 0x1E: // LD E,d8
        return LD_r_n(R_E);
      case 0x1F: // RRA
        return RRA();
      case 0x20: // JR NZ,r8
        return JR_cc((f_ & FLAG_Z) == 0);
      case 0x21: // LD HL,d16
        return LD_rr_nn(RR_HL);
      case 0x22: // LD (HL+),A
        return LD_HLI_A();
      case 0x23: // INC HL
        return INC_rr(RR_HL);
      case 0x24: // INC H
        return INC_r(R_H);
      case 0x25: // DEC H
        return DEC_r(R_H);
      case 0x26: // LD H,d8
        return LD_r_n(R_H);
      case 0x27: // DAA
        return DAA();
      case 0x28: // JR Z,r8
        return JR_cc(f_ & FLAG_Z);
      case 0x29: // ADD HL,HL
        return ADD_HL_rr(RR_HL);
      case 0x2A: // LD A,(HL+)
        return LD_A_HLI();
      case 0x2B: // DEC HL
        return DEC_rr(RR_HL);
      case 0x2C: // INC L
        return INC_r(R_L);
      case 0x2D: // DEC L
        return DEC_r(R_L);
      case 0x2E: // LD L,d8
        return LD_r_n(R_L);
      case 0x2F: // CPL
        return CPL();
      case 0x30: // JR NC,r8
        return JR_cc((f_ & FLAG_C) == 0);
      case 0x31: // LD SP,d16
        return LD_rr_nn(RR_SP);
      case 0x32: // LD (HL-),A
        return LD_HLD_A();
      case 0x33: // INC SP
        return INC_rr(RR_SP);
      case 0x34: // INC HL
        return INC_r(R_HL);
      case 0x35: // DEC HL
        return DEC_r(R_HL);
      case 0x36: // LD (HL),d8
        return LD_r_n(R_HL);
      case 0x37: // SCF
        return SCF();
      case 0x38: // JR C,r8
        return JR_cc(f_ & FLAG_C);
      case 0x39: // ADD HL,SP
        return ADD_HL_rr(RR_SP);
      case 0x3A: // LD A,(HL-)
        return LD_A_HLD();
      case 0x3B: // DEC SP
        return DEC_rr(RR_SP);
      case 0x3C: // INC A
        return INC_r(R_A);
      case 0x3D: // DEC A
        return DEC_r(R_A);
      case 0x3E: // LD A,d8
        return LD_r_n(R_A);
      case 0x3F: // CCF
        return CCF();
      case 0xC3: // JP nn
        return JP();

      default:
        UNIMPLEMENTED();
    }
  }

  auto CPU::STOP() -> int {
    // STOP on DMG is encoded as 0x10 0x00. Consume the padding byte.
    // (On CGB, STOP is also used for speed switching; not handled yet.)
    const u8 padding = fetch8();
    if (padding != 0x00) { UNIMPLEMENTED(); }

    stopped_ = true;
    return 4;
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