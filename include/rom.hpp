/**
 * @file rom.hpp
 * @brief Declares a ROM loader class or utility function to load .gb files into memory.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#pragma once
#include <iostream>
#include <fstream>
#include <string_view>
#include "types.hpp"
#include <vector>

namespace gb {
  
  class ROM {
    public:
      explicit ROM() = default;

      // Loaders
      auto load(std::string_view path) -> bool;      // from .gb file on disk
      auto load(std::vector<u8>&& rawBytes) -> bool; // from an in-memory blob

      [[nodiscard]] auto data() -> const u8*;
      [[nodiscard]] auto size() -> const size_t;

      // Cartridge header helpers
      [[nodiscard]] auto titleChar(size_t i) -> const u8;
      [[nodiscard]] auto isColorGB() -> const bool;
      [[nodiscard]] auto mbcType() -> const u8;

    private:
      std::vector<u8> buffer_;
  };
  
} // namespace gb
