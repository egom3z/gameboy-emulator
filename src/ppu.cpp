/**
 * @file ppu.cpp
 * @brief Implements the PPU: tile fetch, scanline rendering, mode transitions, VBlank
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#include "ppu.hpp"
#include "utils.hpp"

namespace gb {

  PPU::PPU(Memory& mem) : mem_(mem) { UNIMPLEMENTED(); }

  void PPU::step(int cpuCycles) { UNIMPLEMENTED(); }

  auto PPU::frameRead() -> const bool { UNIMPLEMENTED(); }

  auto PPU::framebuffer() -> FrameBuffer&& { UNIMPLEMENTED(); }

  void PPU::reset() { UNIMPLEMENTED(); }

  void PPU::modeOAM() { UNIMPLEMENTED(); }

  void PPU::modeDraw() { UNIMPLEMENTED(); }

  void PPU::modeHBlank() { UNIMPLEMENTED(); }

  void PPU::enterVBlank() { UNIMPLEMENTED(); }

} // namespace gb