/**
 * @file emulator.hpp
 * @brief Declares a high-level Emulator class that wraps CPU, Memory, and PPU into one interface.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#pragma once
#include "cpu.hpp"
#include "memory.hpp"
#include "ppu.hpp"
#include "rom.hpp"
#include <string_view>

namespace gb {

  class Emulator {
    public:
      explicit Emulator();

      auto loadROM(std::string_view path) -> bool;
      void reset();
      void runFrame();

      [[nodiscard]] auto framebuffer() -> const PPU::FrameBuffer;
    
    private:
      ROM    rom_;
      Memory mem_;
      CPU    cpu_;
      PPU    ppu_;
  };

} // namespace gb