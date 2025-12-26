/**
 * @file memory.cpp
 * @brief Implements memory access and memory-mapped I/O; maps ROM, VRAM, RAM, etc.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#include "memory.hpp"
#include "rom.hpp"
#include "utils.hpp"

namespace gb {

  Memory::Memory(const ROM& rom) : rom_(rom), ppu_(nullptr) {}

  auto Memory::read8(u16 addr) -> const u8 {
    if (addr < 0x8000) {
      const auto i = static_cast<std::size_t>(addr);
      if (i < rom_.size()) return rom_.data()[i];
      return 0xFF; // open bus / unmapped
    }

    UNIMPLEMENTED();
  }

  auto Memory::read16(u16 addr) -> const u16 {
    // Game Boy is little-endian
    const u8 lo = read8(addr);
    const u8 hi = read8(static_cast<u16>(addr + 1));
    return static_cast<u16>(static_cast<u16>(lo) | (static_cast<u16>(hi) << 8));
  }

  void Memory::write8(u16 addr, u8 value) { UNIMPLEMENTED(); }

  void Memory::write16(u16 addr, u16 value) { UNIMPLEMENTED(); }

  auto Memory::isIO(u16 addr) -> const bool { UNIMPLEMENTED(); }

} // namespace gb