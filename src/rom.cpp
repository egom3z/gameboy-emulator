/**
 * @file rom.cpp
 * @brief Implements ROM file loading and header parsing 
 *
 * @author Enrique Gomez and Jeshua Linder
 */

#include "rom.hpp"
#include "utils.hpp"
#include <ios>
#include <stdexcept>

namespace gb {

  auto ROM::load(std::string_view path) -> bool {
    std::cout << "Resolved path: " << path << "\n";

    // Open ROM file
    std::ifstream rom_file(path, std::ios::binary | std::ios::ate);
    if (!rom_file.is_open()) {
      std::cerr << "Error: Could not open ROM file: " << path << "\n";
  
      #ifdef __APPLE__
        std::cerr << "Tip: On macOS, remove quarantine with:\n"
                  << "     xattr -d com.apple.quarantine \"" << path << "\"\n";
      #endif
  
      return false;
    }
  
    std::cout << "ROM opened successfully!\n";

    // Get size in bytes
    const std::streamsize size = rom_file.tellg();
    if (size <= 0) {
      throw std::runtime_error("ROM::load: tellg() failed");
    }

    std::cout << "ROM size " << size << " B" << std::endl;

    // Reserve bytes for buffer
    buffer_.resize(static_cast<std::size_t>(size));

    // Read
    rom_file.seekg(0, std::ios::beg); // rewind
    if (!rom_file.read(reinterpret_cast<char*>(buffer_.data()), size)) {
      throw std::runtime_error("ROM::load: read failed");
    }

    std::cout << "ROM contents read into buffer." << std::endl;

    return true;
  }

  auto ROM::load(std::vector<u8>&& rawBytes) -> bool {
    buffer_ = std::move(rawBytes);
    return !buffer_.empty();
  }

  auto ROM::data() -> const u8* { UNIMPLEMENTED(); }

  auto ROM::size() -> const size_t { UNIMPLEMENTED(); }

  auto ROM::titleChar(size_t i) -> const u8 { UNIMPLEMENTED(); }

  auto ROM::isColorGB() -> const bool { UNIMPLEMENTED(); }

  auto ROM::mbcType() -> const u8 { UNIMPLEMENTED(); }

} // namespace gb
