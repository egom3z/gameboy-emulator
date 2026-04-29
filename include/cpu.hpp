/**
 * @file cpu.hpp
 * @brief Declares the CPU class, registers, flags, and the step() function.
 *
 * Naming follows Pan Docs conventions:
 *   r8     – 8-bit register operand (B,C,D,E,H,L,[HL],A; index 0-7)
 *   r16    – 16-bit register operand, SP variant (BC,DE,HL,SP; index 0-3)
 *   r16stk – 16-bit register operand, AF variant (BC,DE,HL,AF; index 0-3)
 *   n8     – unsigned 8-bit immediate
 *   n16    – unsigned 16-bit immediate
 *   e8     – signed 8-bit offset (JR, ADD SP,e8, LD HL,SP+e8)
 *   cc     – condition code (NZ=0, Z=1, NC=2, C=3)
 */

#pragma once
#include "memory.hpp"
#include "types.hpp"

namespace gb {

  class Memory;

  class CPU {
    public:
      explicit CPU(Memory& mem);

      void reset();
      auto step() -> int; // execute one instruction, return T-cycles used

      // Expose key registers for debugging / testing
      [[nodiscard]] auto pc() -> const u16;
      [[nodiscard]] auto sp() -> const u16;
      [[nodiscard]] auto a()  -> const u8;

    private:
      // Register pairs as unions so r8 and r16 accesses share storage.
      // Struct member order is low-byte first (little-endian layout):
      //   af_: f_ is bits 0-7, a_ is bits 8-15 -> af_ = 0x01B0 -> A=0x01, F=0xB0
      union { struct { u8 f_, a_; }; u16 af_; };
      union { struct { u8 c_, b_; }; u16 bc_; };
      union { struct { u8 e_, d_; }; u16 de_; };
      union { struct { u8 l_, h_; }; u16 hl_; };

      u16 pc_{0x0100};
      u16 sp_{0xFFFE};

      bool halted_{false};
      bool stopped_{false};

      Memory& mem_;

      // ── Fetch helpers ────────────────────────────────────────────────────────
      auto fetch8()  -> u8;
      auto fetch16() -> u16;

      // ── Flag constants (bits of F register) ─────────────────────────────────
      static constexpr u8 FLAG_Z = 0x80; // Zero
      static constexpr u8 FLAG_N = 0x40; // Subtract
      static constexpr u8 FLAG_H = 0x20; // Half-carry
      static constexpr u8 FLAG_C = 0x10; // Carry

      // ── Register access helpers ──────────────────────────────────────────────
      // r8 index: 0=B 1=C 2=D 3=E 4=H 5=L 6=[HL] 7=A
      auto getR8Ref(int idx) -> u8&; // reference to actual register variable
      auto getR8(int idx) -> u8;     // value; idx==6 reads mem[HL]
      void setR8(int idx, u8 val);   // value; idx==6 writes mem[HL]

      // r16 index: 0=BC 1=DE 2=HL 3=SP
      auto getR16(int idx) -> u16&;

      // r16stk index: 0=BC 1=DE 2=HL 3=AF  (used by PUSH / POP)
      auto getR16stk(int idx) -> u16&;

      // Condition code: 0=NZ 1=Z 2=NC 3=C
      auto getCondition(int cc) -> bool;

      // ── Interrupt handling ───────────────────────────────────────────────────
      void handleInterrupts();

      // ── Instruction handlers (Pan Docs mnemonic + operand-type suffix) ───────

      auto ADC_A_n8() -> int;
      auto ADC_A_r8(int src) -> int;

      auto ADD_A_n8() -> int;
      auto ADD_A_r8(int src) -> int;
      auto ADD_HL_r16(int reg) -> int;

      auto AND_A_n8() -> int;
      auto AND_A_r8(int src) -> int;

      auto CALL() -> int;
      auto CALL_cc(bool cond) -> int;

      auto CCF() -> int;
      auto CPL() -> int;

      auto CP_A_n8() -> int;
      auto CP_A_r8(int src) -> int;

      auto DAA() -> int;

      auto DEC_r8(int reg) -> int;
      auto DEC_r16(int reg) -> int;

      auto HALT() -> int;

      auto INC_r8(int reg) -> int;
      auto INC_r16(int reg) -> int;

      auto JP() -> int;             // JP n16
      auto JP_cc(bool cond) -> int; // JP cc, n16
      auto JP_HL() -> int;          // JP HL

      auto JR() -> int;             // JR e8
      auto JR_cc(bool cond) -> int; // JR cc, e8

      // LD r8/r16 – register <-> immediate / memory
      auto LD_r8_n8(int dst) -> int;
      auto LD_r8_r8(int dst, int src) -> int;
      auto LD_r16_n16(int dst) -> int;

      // LD A <- [r16mem]
      auto LD_A_IndBC() -> int;
      auto LD_A_IndDE() -> int;
      auto LD_A_HLI() -> int; // LD A, [HL+]
      auto LD_A_HLD() -> int; // LD A, [HL-]

      // LD [r16mem] <- A
      auto LD_IndBC_A() -> int;
      auto LD_IndDE_A() -> int;
      auto LD_HLI_A() -> int; // LD [HL+], A
      auto LD_HLD_A() -> int; // LD [HL-], A

      // LD [n16] <- SP  (opcode 0x08)
      auto LD_n16_SP() -> int;

      // High-memory loads (LDH = 0xFF00 + offset)
      auto LDH_n8_A() -> int;  // LDH [n8], A  (0xE0)
      auto LD_IndC_A() -> int; // LD  [C],  A  (0xE2)
      auto LD_n16_A() -> int;  // LD  [n16],A  (0xEA)
      auto LDH_A_n8() -> int;  // LDH A, [n8]  (0xF0)
      auto LD_A_IndC() -> int; // LD  A, [C]   (0xF2)
      auto LD_A_n16() -> int;  // LD  A, [n16] (0xFA)

      auto OR_A_n8() -> int;
      auto OR_A_r8(int src) -> int;

      auto POP_r16stk(int idx) -> int;
      auto PUSH_r16stk(u16 value) -> int;

      auto RET() -> int;
      auto RET_cc(bool cond) -> int;
      auto RETI() -> int;

      auto RLA()  -> int;
      auto RLCA() -> int;
      auto RRA()  -> int;
      auto RRCA() -> int;

      auto RST(u8 vec) -> int;

      auto SBC_A_n8() -> int;
      auto SBC_A_r8(int src) -> int;

      auto SCF() -> int;

      auto STOP() -> int;

      auto SUB_A_n8() -> int;
      auto SUB_A_r8(int src) -> int;

      auto XOR_A_n8() -> int;
      auto XOR_A_r8(int src) -> int;

      // CB prefix dispatcher
      auto execute_CB(u8 op) -> int;

      // CB rotate / shift helpers (operate on any r8; set Z unlike accumulator versions)
      auto CB_RLC(int reg) -> int;  // rotate left, bit 7 -> carry and -> bit 0
      auto CB_RRC(int reg) -> int;  // rotate right, bit 0 -> carry and -> bit 7
      auto CB_RL(int reg)  -> int;  // rotate left through carry
      auto CB_RR(int reg)  -> int;  // rotate right through carry
      auto CB_SLA(int reg) -> int;  // shift left arithmetic (bit 0 = 0)
      auto CB_SRA(int reg) -> int;  // shift right arithmetic (bit 7 preserved)
      auto CB_SWAP(int reg) -> int; // swap high and low nibbles
      auto CB_SRL(int reg) -> int;  // shift right logical (bit 7 = 0)
      // BIT / RES / SET are handled inline in execute_CB
  };

} // namespace gb
