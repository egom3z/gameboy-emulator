/**
 * @file main.cpp
 * @brief Entry point. Loads ROM, instantiates Emulator, runs emulation loop.
 */

#include "cpu.hpp"
#include "emulator.hpp"
#include "memory.hpp"
#include "ppu.hpp"
#include "rom.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
 
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  std::cout << "Initializing GameBoy emulator...\n";

  std::cout << "CWD: " << fs::current_path() << "\n";

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path/to/file.gb(c)>\n";
    return 1;
  }

  gb::ROM rom;
  bool load_success = rom.load(argv[1]);
  if (!load_success) {
    return 1;
  }

  while (true);

  return 0;
}
