/**
 * @file rom.cpp
 * @brief Implements ROM file loading and header parsing 
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#include "rom.hpp"
#include "utils.hpp"

namespace gb {

  bool ROM::load(std::string_view path) { UNIMPLEMENTED(); }

  auto ROM::data() -> const u8* { UNIMPLEMENTED(); }

  auto ROM::size() -> const size_t { UNIMPLEMENTED(); }

  auto ROM::titleChar(size_t i) -> const u8 { UNIMPLEMENTED(); }

  auto ROM::isColorGB() -> const bool { UNIMPLEMENTED(); }

  auto ROM::mbcType() -> const u8 { UNIMPLEMENTED(); }

} // namespace gb