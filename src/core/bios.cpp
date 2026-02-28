#include "bios.h"
#include <fstream>

bool Bios::load(const std::string &path) {
  loaded_ = false;
  info_.clear();

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    LOG_ERROR("BIOS: Failed to open file: %s", path.c_str());
    return false;
  }

  auto size = file.tellg();
  if (size != psx::BIOS_SIZE) {
    LOG_ERROR("BIOS: Invalid size %lld (expected %u)", (long long)size,
              psx::BIOS_SIZE);
    return false;
  }

  file.seekg(0);
  file.read(reinterpret_cast<char *>(data_.data()), psx::BIOS_SIZE);
  if (!file || file.gcount() != psx::BIOS_SIZE) {
    LOG_ERROR("BIOS: Failed to read full image (%lld/%u bytes)",
              (long long)file.gcount(), psx::BIOS_SIZE);
    return false;
  }

  loaded_ = true;

  identify();
  LOG_INFO("BIOS: Loaded successfully — %s", info_.c_str());
  return true;
}

void Bios::identify() {
  // Try to identify the BIOS version by scanning for known strings
  std::string content(reinterpret_cast<const char *>(data_.data()),
                      psx::BIOS_SIZE);

  struct BiosInfo {
    const char *search;
    const char *name;
  };
  static const BiosInfo known_bioses[] = {
      {"SCPH-1000", "SCPH-1000 (Japan v1.0)"},
      {"SCPH-1001", "SCPH-1001 (North America v2.0)"},
      {"SCPH-1002", "SCPH-1002 (Europe v2.0)"},
      {"SCPH-3000", "SCPH-3000 (Japan v1.1)"},
      {"SCPH-3500", "SCPH-3500 (Japan v2.1)"},
      {"SCPH-5000", "SCPH-5000 (Japan v3.0)"},
      {"SCPH-5500", "SCPH-5500 (Japan v3.0)"},
      {"SCPH-5501", "SCPH-5501 (North America v3.0)"},
      {"SCPH-5502", "SCPH-5502 (Europe v3.0)"},
      {"SCPH-7000", "SCPH-7000 (Japan v4.0)"},
      {"SCPH-7001", "SCPH-7001 (North America v4.1)"},
      {"SCPH-7002", "SCPH-7002 (Europe v4.1)"},
      {"SCPH-7501", "SCPH-7501 (North America v4.4)"},
      {"SCPH-7502", "SCPH-7502 (Europe v4.4)"},
      {"SCPH-9001", "SCPH-9001 (North America v4.5)"},
      {"SCPH-9002", "SCPH-9002 (Europe v4.5)"},
      {"SCPH-100", "PSone SCPH-100 (Japan v4.3)"},
      {"SCPH-101", "PSone SCPH-101 (North America v4.5)"},
      {"SCPH-102", "PSone SCPH-102 (Europe v4.5)"},
  };

  for (const auto &bios : known_bioses) {
    if (content.find(bios.search) != std::string::npos) {
      info_ = bios.name;
      return;
    }
  }

  // Many retail dumps do not include SCPH strings; fall back to ROM version.
  const char *ver_tag = "System ROM Version ";
  std::string best_ver;
  size_t pos = 0;
  while ((pos = content.find(ver_tag, pos)) != std::string::npos) {
    size_t ver_start = pos + std::strlen(ver_tag);
    std::string ver;
    for (size_t i = ver_start; i < content.size() && ver.size() < 48; ++i) {
      const unsigned char ch = static_cast<unsigned char>(content[i]);
      if (ch < 32 || ch > 126) {
        break;
      }
      ver.push_back(static_cast<char>(ch));
    }
    if (!ver.empty()) {
      // Prefer entries with date stamps (e.g. "2.2 12/04/95 A").
      if (ver.find('/') != std::string::npos) {
        best_ver = ver;
      } else if (best_ver.empty()) {
        best_ver = ver;
      }
    }
    pos = ver_start;
  }

  if (!best_ver.empty()) {
    if (best_ver.find("2.2 12/04/95 A") != std::string::npos) {
      info_ = "SCPH-1001 compatible (ROM " + best_ver + ")";
    } else {
      info_ = "PS1 ROM " + best_ver;
    }
    return;
  }

  info_ = "Unknown BIOS (valid size, unknown signature)";
}

u8 Bios::read8(u32 offset) const {
  return data_[offset & (psx::BIOS_SIZE - 1)];
}

u16 Bios::read16(u32 offset) const {
  u32 off = offset & (psx::BIOS_SIZE - 1);
  u16 val;
  std::memcpy(&val, &data_[off], sizeof(u16));
  return val;
}

u32 Bios::read32(u32 offset) const {
  u32 off = offset & (psx::BIOS_SIZE - 1);
  u32 val;
  std::memcpy(&val, &data_[off], sizeof(u32));
  return val;
}
