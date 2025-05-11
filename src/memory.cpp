/**
 * @file memory.cpp
 * @brief Implements memory access and memory-mapped I/O; maps ROM, VRAM, RAM, etc.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#include "memory.hpp"
#include "utils.hpp"

namespace gb {

  Memory::Memory(const ROM& rom) : rom_(rom), ppu_(nullptr) { UNIMPLEMENTED(); }

  auto Memory::read8(u16 addr) -> const u8 { UNIMPLEMENTED(); }

  auto Memory::read16(u16 addr) -> const u16 { UNIMPLEMENTED(); }

  void Memory::write8(u16 addr, u8 value) { UNIMPLEMENTED(); }

  void Memory::write16(u16 addr, u16 value) { UNIMPLEMENTED(); }

  auto Memory::isIO(u16 addr) -> const bool { UNIMPLEMENTED(); }

} // namespace gb