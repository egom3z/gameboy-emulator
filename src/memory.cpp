/**
 * @file memory.cpp
 * @brief Implements memory access and memory-mapped I/O; maps ROM, VRAM, RAM, etc.
 */

#include "memory.hpp"
#include "rom.hpp"
#include "utils.hpp"

namespace gb {

  Memory::Memory(const ROM& rom) : rom_(rom), ppu_(nullptr) {}

  auto Memory::read8(u16 addr) -> const u8 {
    // 0x0000-0x7FFF: ROM (no MBC yet)
    if (addr < 0x8000) {
      const auto i = static_cast<std::size_t>(addr);
      if (i < rom_.size()) return rom_.data()[i];
      return 0xFF; // open bus / unmapped
    }

    // 0x8000-0x9FFF: VRAM
    if (0x8000 <= addr && addr <= 0x9FFF) {
      return vram_[addr - 0x8000];
    }

    // 0xA000-0xBFFF: External RAM (cartridge RAM)
    if (0xA000 <= addr && addr <= 0xBFFF) {
      return eram_[addr - 0xA000];
    }

    // 0xC000-0xDFFF: WRAM
    if (0xC000 <= addr && addr <= 0xDFFF) {
      return wram_[addr - 0xC000];
    }

    // 0xE000-0xFDFF: Echo RAM (mirror of 0xC000-0xDDFF)
    if (0xE000 <= addr && addr <= 0xFDFF) {
      return wram_[addr - 0xE000];
    }

    // 0xFE00-0xFE9F: OAM
    if (0xFE00 <= addr && addr <= 0xFE9F) {
      return oam_[addr - 0xFE00];
    }

    // 0xFEA0-0xFEFF: Not usable
    if (0xFEA0 <= addr && addr <= 0xFEFF) {
      return 0xFF;
    }

    // 0xFF00-0xFF7F: I/O registers
    if (0xFF00 <= addr && addr <= 0xFF7F) {
      const u8 reg = io_[addr - 0xFF00];
      if (addr == 0xFF0F) { return static_cast<u8>(reg | 0xE0); }
      return reg;
    }

    // 0xFF80-0xFFFE: HRAM
    if (0xFF80 <= addr && addr <= 0xFFFE) {
      return hram_[addr - 0xFF80];
    }

    // 0xFFFF: Interrupt Enable
    if (addr == 0xFFFF) {
      return ie_;
    }

    UNIMPLEMENTED();
  }

  auto Memory::read16(u16 addr) -> const u16 {
    // Game Boy is little-endian
    const u8 lo = read8(addr);
    const u8 hi = read8(static_cast<u16>(addr + 1));
    return static_cast<u16>(static_cast<u16>(lo) | (static_cast<u16>(hi) << 8));
  }

  void Memory::write8(u16 addr, u8 value) {
    // 0x0000-0x7FFF: ROM region
    if (addr < 0x8000) {
      return;
    }

    if (0x8000 <= addr && addr <= 0x9FFF) {
      vram_[addr - 0x8000] = value;
      return;
    }
    else if (0xA000 <= addr && addr <= 0xBFFF) {
      // External (cartridge) RAM
      eram_[addr - 0xA000] = value;
      return;
    }
    else if (0xC000 <= addr && addr <= 0xDFFF) {
      wram_[addr - 0xC000] = value;
      return;
    }
    else if (0xE000 <= addr && addr <= 0xFDFF) {
      // Echo RAM mirrors 0xC000-0xDDFF
      wram_[addr - 0xE000] = value;
      return;
    }
    else if (0xFE00 <= addr && addr <= 0xFE9F) {
      oam_[addr - 0xFE00] = value;
      return;
    }
    else if (0xFEA0 <= addr && addr <= 0xFEFF) {
      // Not usable: ignore writes
      return;
    }
    else if (0xFF00 <= addr && addr <= 0xFF7F) {
      // A few registers have important side effects that many ROMs rely on.
      const u16 off = static_cast<u16>(addr - 0xFF00);

      switch (addr) {
        case 0xFF04: // DIV: writing any value resets DIV to 0
          io_[off] = 0;
          return;

        case 0xFF41: // STAT: lower 3 bits are read-only (mode + coincidence)
          io_[off] = static_cast<u8>((value & 0xF8) | (io_[off] & 0x07));
          return;

        case 0xFF44: // LY: read-only; writing resets to 0 on DMG
          io_[off] = 0;
          return;

        case 0xFF46: { // DMA: start OAM DMA transfer from (value << 8)
          io_[off] = value;
          const u16 src_base = static_cast<u16>(static_cast<u16>(value) << 8);
          for (u16 i = 0; i < 0x00A0; i++) {
            oam_[i] = read8(static_cast<u16>(src_base + i));
          }
          return;
        }

        case 0xFF0F: // IF: only low 5 bits used; upper bits read as 1
          io_[off] = static_cast<u8>(value | 0xE0);
          return;

        default:
          io_[off] = value;
          return;
      }
    }
    else if (0xFF80 <= addr && addr <= 0xFFFE) {
      hram_[addr - 0xFF80] = value;
      return;
    }
    else if (addr == 0xFFFF) {
      ie_ = value;
      return;
    }

    UNIMPLEMENTED();
  }

  void Memory::write16(u16 addr, u16 value) {
    write8(addr, static_cast<u8>(value & 0xFF));
    write8(addr + 1, static_cast<u8>(value >> 8));
  }

  auto Memory::isIO(u16 addr) -> const bool { return 0xFF00 <= addr && addr <= 0xFF7F; }

} // namespace gb