/**
 * @file emulator.cpp
 * @brief Integrates CPU, Memory, and PPU and drives their coordination per cycle/frame.
 */

#include "emulator.hpp"
#include "utils.hpp"

namespace gb {

  Emulator::Emulator()
    : rom_(),
      mem_(rom_),
      cpu_(mem_),
      ppu_(mem_)
  {
    cpu_.reset();
    ppu_.reset();
    
    UNIMPLEMENTED();
  }

  auto Emulator::loadROM(std::string_view path) -> bool { UNIMPLEMENTED(); }

  void Emulator::reset() { UNIMPLEMENTED(); }

  void Emulator::runFrame() { UNIMPLEMENTED(); }

  auto Emulator::framebuffer() -> const PPU::FrameBuffer { UNIMPLEMENTED(); }

} // namespace gb