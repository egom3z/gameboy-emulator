/**
 * @file memory.hpp
 * @brief Declares the Memory class, with read8, write8, and region mapping.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#pragma once
#include "ppu.hpp"
#include "types.hpp"

namespace gb {
  
  class PPU;
  class ROM;

  class Memory {
    public:
      explicit Memory(const ROM& rom);

      auto read8(u16 addr)  -> const u8;
      auto read16(u16 addr) -> const u16;
      void write8(u16 addr, u8 value);
      void write16(u16 addr, u16 value);

    private:
      const ROM& rom_;
      PPU* ppu_;

      // 8 KiB VRAM, 8 KiB WRAM, etc.
      u8 vram_[0x2000]{};
      u8 wram_[0x2000]{};
      u8 oam_ [0x00A0]{};
      u8 hram_[0x007F]{};

      // helpers
      auto isIO(u16 addr) -> const bool;
  };

} // namespace gb