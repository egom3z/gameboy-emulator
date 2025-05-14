# Game Boy Emulator (WIP)

> A clean-room, C++17 emulator for Nintendo’s original Game Boy, written for fun and learning.  
> **Status:** CPU core compiling; PPU & APU under active development.

---

## Features (Currently Unimplemented)

- **Sharp LR35902 / SM83** CPU emulation  
- Accurate memory-map (ROM, VRAM, WRAM, OAM, I/O, HRAM)  
- Scanline-based **PPU** (LCD) renderer → 160 × 144 framebuffer  
- SDL 2 window, input, and (eventually) audio  
- Passes blargg’s CPU instruction tests (goal)  
- Runs on **macOS (Apple Silicon & Intel)**, **Linux**, and **Windows**

---

## 📂 Project Layout

```
gameboy-emulator/
├── include/          # public headers (cpu.hpp, memory.hpp, ppu.hpp…)
├── src/              # implementation (.cpp) files
├── docs/             # architecture notes, opcode tables, etc.
├── test/             # GoogleTest unit files & tiny ROMs
├── CMakeLists.txt    # top-level build script
└── .gitignore
```
---

## Building & Running

### 1. Prerequisites

| Platform | Packages |
|----------|----------|
| **macOS** (ARM/Intel) | `brew install cmake sdl2` |
| **Ubuntu 22.04** | `sudo apt install build-essential cmake libsdl2-dev` |
| **Windows 10+** | Install **Visual Studio Build Tools** → `winget install cmake`, `vcpkg install sdl2` |

> **No global install of GoogleTest needed**—CMake fetches it automatically if you enable tests.

### 2. Configure & build

```bash
git clone https://github.com/<your-user>/gameboy-emulator.git
cd gameboy-emulator
mkdir build && cd build

# Configure:
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DGBEMU_BUILD_TESTS=ON      # OFF to skip GoogleTest

# Compile emulator + (optionally) unit_tests
cmake --build .
```

### 3. Run the emulator

```bash
./gbemu /path/to/roms/<rom_name/>.gb # currently defaults to entering main
```

### 4. Run unit tests
```bash
ctest        # all tests
ctest -N     # list all discovered tests
ctest -R CPU # run only tests whose gtest name contains "CPU"
```