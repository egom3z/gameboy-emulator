/**
 * @file rom.hpp
 * @brief Declares a ROM loader class or utility function to load .gb files into memory.
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#pragma once
#include <string_view>
#include "types.hpp"
#include <vector>

namespace gb {
  
  class ROM {
    public:
      explicit ROM() = default;

      bool load(std::string_view path);
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