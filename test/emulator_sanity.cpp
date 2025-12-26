#include "emulator.hpp"
#include "gtest/gtest.h"
#include "rom.hpp"


inline auto make_endless_nop_rom() -> gb::ROM
{
  //  ________________________
  // |    HEX   | ASSEMBLY    |
  // |----------|-------------|
  // | 00 00    |  NOP NOP    |
  // | C3 00 01 |  JP  0x0100 |
  // |________________________|

  // The Game Boy CPU starts executing at address 0x0100 after reset.
  // Pad the ROM so our tiny program actually lives at 0x0100.
  std::vector<gb::u8> bytes(0x0100 + 5, 0x00);
  bytes[0x0100 + 0] = 0x00; // NOP
  bytes[0x0100 + 1] = 0x00; // NOP
  bytes[0x0100 + 2] = 0xC3; // JP nn
  bytes[0x0100 + 3] = 0x00; // low byte of 0x0100
  bytes[0x0100 + 4] = 0x01; // high byte of 0x0100
  gb::ROM rom;
  rom.load(std::move(bytes));
  return rom;
}

TEST(CPU, EndlessNopLoop)
{
  // Build a custom emulator instance with the tiny ROM
  auto rom = make_endless_nop_rom();
  auto mem = gb::Memory(rom);
  auto cpu = gb::CPU(mem);

  cpu.reset(); // should set PC = 0x0100

  constexpr int kSteps = 1000;
  for (int i = 0; i < kSteps; i++) {
    cpu.step();
  }

  // After any multiple of 3 instructions (NOP, NOP, JP) PC is back at 0x0100
  EXPECT_EQ(cpu.pc(), 0x0101);
}