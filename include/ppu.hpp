/**
 * @file ppu.hpp
 * @brief Declares the PPU class, modes (OAM scan, draw, HBlank), and scanline rendering logic.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#pragma once
#include <array>
#include "types.hpp"

namespace gb {

  class Memory;

  class PPU {
    public:
      static constexpr int SCREEN_W = 160;
      static constexpr int SCREEN_H = 144;

      explicit PPU(Memory& mem);

      // advance PPU by the given CPU cycle count
      void step(int cpuCycles);

      // true when a complete frame is ready
      [[nodiscard]] auto frameRead() -> const bool;

      // 32-bit RGBA framebuffer (one frame worth of pixels)
      using FrameBuffer = std::array<u32, SCREEN_W * SCREEN_H>;
      [[nodiscard]] auto framebuffer() -> FrameBuffer&&;

      void reset();
    
    private:
      Memory& mem_;

      int dotCounter_{0};
      int scanline_{0};
      bool frameReady{false};
      FrameBuffer fb_{};

      // mode helpers
      void modeOAM();
      void modeDraw();
      void modeHBlank();
      void enterVBlank();
  };

}