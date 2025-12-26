/**
 * @file cpu.hpp
 * @brief Declares the CPU class, registers, flags, and the step() function.
 */

#pragma once
#include "memory.hpp"
#include "types.hpp"

#include <array>

namespace gb {

  class Memory;

  class CPU {
    public:
      explicit CPU(Memory& mem);

      void reset();
      auto step() -> int; // execute one instruction, return cycles used
      void initInstructionTable();

      // expose key registers for debugging
      [[nodiscard]] auto pc() -> const u16;
      [[nodiscard]] auto sp() -> const u16;
      [[nodiscard]] auto ra() -> const u8;

    private:
      // 8-bit registers
      union { struct {u8 f_, a_; }; u16 af_; }; // AF = Accumulator + Flags (ZNHC)
      union { struct {u8 c_, b_; }; u16 bc_; }; // BC = General-purpose
      union { struct {u8 e_, d_; }; u16 de_; }; // DE = General-purpose
      union { struct {u8 l_, h_; }; u16 hl_; }; // HL = Pointer to memory (address reg)

      u16 pc_{0x0100}; // Program Counter
      u16 sp_{0xFFFE}; // Stack Pointer

      bool halted_{false};

      Memory& mem_;
      std::array<int (CPU::*)(), 256> instruction_table_{};

      // internal helpers (only declarations)
      auto fetch8()  -> u8;
      auto fetch16() -> u16;
      void executeOpcode(u8 opcode);
      void handleInterrupts();

      // Flag constants
      static constexpr u8 FLAG_Z = 0x80;
      static constexpr u8 FLAG_N = 0x40;
      static constexpr u8 FLAG_H = 0x20;
      static constexpr u8 FLAG_C = 0x10;

      // Register indexing: 0=B, 1=C, 2=D, 3=E, 4=H, 5=L, 6=(HL), 7=A
      auto getRegister8(int index) -> u8&;
      auto getRegisterValue(int index) -> u8;
      void setRegister8(int index, u8 value);

      // 16-bit register access: 0=BC, 1=DE, 2=HL, 3=SP
      auto getRegister16_v1(int index) -> u16&;

      // Alternative 16-bit mapping for PUSH/POP: 0=BC, 1=DE, 2=HL, 3=AF
      auto getRegister16_v2(int index) -> u16&;

      // Condition checking helper
      auto getCondition(int cc) -> bool;

      /// Method handlers for SM83 instructions (alphabetically ordered) ///

      auto ADC_A_n() -> int;

      auto ADC_A_r(int src) -> int;

      auto ADD_A_n() -> int;

      auto ADD_A_r(int src) -> int;

      auto ADD_HL_rr(int reg) -> int;

      auto AND_A_n() -> int;

      auto AND_A_r(int src) -> int;

      auto CALL() -> int;

      auto CALL_cc(bool condition) -> int;

      auto CP_A_n() -> int;

      auto CP_A_r(int src) -> int;

      auto DEC_r(int reg) -> int;

      auto DEC_rr(int reg) -> int;

      auto INC_r(int reg) -> int;

      auto INC_rr(int reg) -> int;

      auto JP() -> int;

      auto JP_cc(bool condition) -> int;

      auto JR() -> int;
      
      auto JR_cc(bool condition) -> int;

      auto LD_A_n() -> int;

      auto LD_A_BC() -> int;

      auto LD_A_DE() -> int;

      auto LD_A_HLD() -> int;

      auto LD_A_HLI() -> int;

      auto LD_BC_A() -> int;

      auto LD_DE_A() -> int;

      auto LD_HLD_A() -> int;

      auto LD_HLI_A() -> int;
      
      auto LD_r_n(int dst) -> int;

      auto LD_r_r(int dst, int src) -> int;

      auto LD_rr_nn(int dst) -> int;

      auto LDH_A_nn() -> int;

      auto LDH_A_C() -> int;

      auto LDH_C_A() -> int;

      auto LDH_n_A() -> int;

      auto LDH_nn_A() -> int;

      auto LDH_nn_SP() -> int;

      auto OR_A_n() -> int;

      auto OR_A_r(int src) -> int;

      auto POP_rr(int idx) -> int;

      auto PUSH_rr(u16 value) -> int;

      auto RET() -> int;

      auto RET_cc(bool condition) -> int;

      auto RST(u8 vector) -> int;

      auto SBC_A_n() -> int;

      auto SBC_A_r(int src) -> int;

      auto SUB_A_n() -> int;

      auto SUB_A_r(int src) -> int;

      auto XOR_A_n() -> int;

      auto XOR_A_r(int src) -> int;

  };

} // namespace gb
