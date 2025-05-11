/**
 * @file cpu.hpp
 * @brief Declares the CPU class, registers, flags, and the step() function.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#pragma once
#include "types.hpp"

namespace gb {

  class Memory;

  class CPU {
    public:
      explicit CPU(Memory& mem);

      void reset();
      auto step() -> int; // execute one instruction, return cycles used

      // expose key registers for debugging
      [[nodiscard]] auto pc() -> const u16;
      [[nodiscard]] auto sp() -> const u16;
      [[nodiscard]] auto ra() -> const u8;

    private:
      // 8-bit registers (F = flags)
      union { struct {u8 f, a; }; u16 af; };
      union { struct {u8 c, b; }; u16 bc; };
      union { struct {u8 e, d; }; u16 de; };
      union { struct {u8 l, h; }; u16 hl; };

      u16 pc_{0x0100};
      u16 sp_{0xFFFE};

      Memory& mem_;

      // internal helpers (only declarations)
      auto fetch8()  -> u8;
      auto fetch16() -> u16;
      void executeOpcode(u8 opcode);
      void handleInterrupts();
  };

} // namespace gb