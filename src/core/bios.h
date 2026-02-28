#pragma once
#include "types.h"
#include <array>
#include <string>


// ── BIOS ROM (512KB) ───────────────────────────────────────────────
// Loads and provides read-only access to the PS1 BIOS.
// Supports all known BIOS versions (SCPH-1001, 5501, 7001, etc.)

class Bios {
public:
  bool load(const std::string &path);
  bool is_loaded() const { return loaded_; }

  u8 read8(u32 offset) const;
  u16 read16(u32 offset) const;
  u32 read32(u32 offset) const;

  const std::string &get_info() const { return info_; }

private:
  std::array<u8, psx::BIOS_SIZE> data_{};
  bool loaded_ = false;
  std::string info_;

  void identify();
};
