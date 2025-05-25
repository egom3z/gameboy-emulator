/**
 * @file cpu.hpp
 * @brief Declares the CPU class, registers, flags, and the step() function.
 *
 * @author Enrique Gomez and Jeshua Linder
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

      // CPU instructions
      int NOP();
      int LD_BC_NN();
      int LD_BC_A();
      int INC_BC();
      int INC_B();
      int DEC_B();
      int LD_B_N();
      int RLCA();
      int LD_NN_SP();
      int ADD_HL_BC();
      int LD_A_BC();
      int DEC_BC();
      int INC_C();
      int DEC_C();
      int LD_C_N();
      int RRCA();

      int STOP();
      int LD_DE_NN();
      int LD_DE_A();
      int INC_DE();
      int INC_D();
      int DEC_D();
      int LD_D_N();
      int RLA();
      int JR_N();
      int ADD_HL_DE();
      int LD_A_DE();
      int DEC_DE();
      int INC_E();
      int DEC_E();
      int LD_E_N();
      int RRA();

      int JR_NZ();
      int LD_HL_NN();
      int LD_HLI_A();
      int INC_HL();
      int INC_H();
      int DEC_H();
      int LD_H_N();
      int DAA();
      int JR_Z();
      int ADD_HL_HL();
      int LD_A_HLI();
      int DEC_HL();
      int INC_L();
      int DEC_L();
      int LD_L_N();
      int CPL();

      int JR_NC();
      int LD_SP_NN();
      int LD_HLD_A();
      int INC_SP();
      int INC_HL_MEM();
      int DEC_HL_MEM();
      int LD_HL_N();
      int SCF();
      int JR_C();
      int ADD_HL_SP();
      int LD_A_HLD();
      int DEC_SP();
      int INC_A();
      int DEC_A();
      int LD_A_N();
      int CCF();

      // LD r1, r2 handlers 0x40–0x7F
      int LD_B_B();  int LD_B_C();  int LD_B_D();  int LD_B_E();  int LD_B_H();  int LD_B_L();  int LD_B_HL(); int LD_B_A();
      int LD_C_B();  int LD_C_C();  int LD_C_D();  int LD_C_E();  int LD_C_H();  int LD_C_L();  int LD_C_HL(); int LD_C_A();
      int LD_D_B();  int LD_D_C();  int LD_D_D();  int LD_D_E();  int LD_D_H();  int LD_D_L();  int LD_D_HL(); int LD_D_A();
      int LD_E_B();  int LD_E_C();  int LD_E_D();  int LD_E_E();  int LD_E_H();  int LD_E_L();  int LD_E_HL(); int LD_E_A();
      int LD_H_B();  int LD_H_C();  int LD_H_D();  int LD_H_E();  int LD_H_H();  int LD_H_L();  int LD_H_HL(); int LD_H_A();
      int LD_L_B();  int LD_L_C();  int LD_L_D();  int LD_L_E();  int LD_L_H();  int LD_L_L();  int LD_L_HL(); int LD_L_A();
      int LD_HL_B(); int LD_HL_C(); int LD_HL_D(); int LD_HL_E(); int LD_HL_H(); int LD_HL_L(); int HALT();    int LD_HL_A();
      int LD_A_B();  int LD_A_C();  int LD_A_D();  int LD_A_E();  int LD_A_H();  int LD_A_L();  int LD_A_HL(); int LD_A_A();

      // ALU ops 0x80–0xBF
      int ADD_A_B(); int ADD_A_C(); int ADD_A_D(); int ADD_A_E(); int ADD_A_H(); int ADD_A_L(); int ADD_A_HL(); int ADD_A_A();
      int ADC_A_B(); int ADC_A_C(); int ADC_A_D(); int ADC_A_E(); int ADC_A_H(); int ADC_A_L(); int ADC_A_HL(); int ADC_A_A();
      int SUB_A_B(); int SUB_A_C(); int SUB_A_D(); int SUB_A_E(); int SUB_A_H(); int SUB_A_L(); int SUB_A_HL(); int SUB_A_A();
      int SBC_A_B(); int SBC_A_C(); int SBC_A_D(); int SBC_A_E(); int SBC_A_H(); int SBC_A_L(); int SBC_A_HL(); int SBC_A_A();
      int AND_A_B(); int AND_A_C(); int AND_A_D(); int AND_A_E(); int AND_A_H(); int AND_A_L(); int AND_A_HL(); int AND_A_A();
      int XOR_A_B(); int XOR_A_C(); int XOR_A_D(); int XOR_A_E(); int XOR_A_H(); int XOR_A_L(); int XOR_A_HL(); int XOR_A_A();
      int OR_A_B();  int OR_A_C();  int OR_A_D();  int OR_A_E();  int OR_A_H();  int OR_A_L();  int OR_A_HL();  int OR_A_A();
      int CP_A_B();  int CP_A_C();  int CP_A_D();  int CP_A_E();  int CP_A_H();  int CP_A_L();  int CP_A_HL();  int CP_A_A();

      // Calls, jumps, rets, and misc 0xC0–0xFF
      int RET_NZ();      int POP_BC();    int JP_NZ();   int JP();
      int CALL_NZ();     int PUSH_BC();   int ADD_A_N(); int RST_00();
      int RET_Z();       int RET();       int JP_Z();    int UNUSED_CB();
      int CALL_Z();      int CALL();      int ADC_A_N(); int RST_08();
      int RET_NC();      int POP_DE();    int JP_NC();   int UNUSED_D3();
      int CALL_NC();     int PUSH_DE();   int SUB_A_N(); int RST_10();
      int RET_C();       int RETI();      int JP_C();    int UNUSED_DB();
      int CALL_C();      int UNUSED_DD(); int SBC_A_N(); int RST_18();
      int LDH_N_A();     int POP_HL();    int LDH_C_A(); int UNUSED_E3();
      int UNUSED_E4();   int PUSH_HL();   int AND_A_N(); int RST_20();
      int ADD_SP_I8();   int JP_HL();     int LD_NN_A(); int UNUSED_EB();
      int UNUSED_EC();   int UNUSED_ED(); int XOR_A_N(); int RST_28();
      int LDH_A_N();     int POP_AF();    int LDH_A_C(); int DI();
      int UNUSED_F4();   int PUSH_AF();   int OR_A_N();  int RST_30();
      int LD_HL_SP_I8(); int LD_SP_HL();  int LD_A_NN(); int EI();
      int UNUSED_FC();   int UNUSED_FD(); int CP_A_N();  int RST_38();
  };

} // namespace gb
