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

  CPU::CPU(Memory& mem) : mem_(mem) { UNIMPLEMENTED(); }

  void CPU::reset() { UNIMPLEMENTED(); }

  auto CPU::step() -> int { UNIMPLEMENTED(); }

  auto CPU::pc() -> const u16 { UNIMPLEMENTED(); }
  
  auto CPU::sp() -> const u16 { UNIMPLEMENTED(); }

  auto CPU::ra() -> const u8 { UNIMPLEMENTED(); }

  auto CPU::fetch8() -> u8 { UNIMPLEMENTED(); }

  auto CPU::fetch16() -> u16 { UNIMPLEMENTED(); }

  void CPU::executeOpcode(u8 opcode) { UNIMPLEMENTED(); }

  void CPU::handleInterrupts() { UNIMPLEMENTED(); }

} // namespace gb