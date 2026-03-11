#pragma once

#include "types.h"

#include <array>
#include <cstddef>
#include <string>

class MemoryCard {
public:
  struct TransferResult {
    u8 data_out = 0xFF;
    bool ack = false;
  };

  static constexpr size_t kSizeBytes = 128 * 1024;
  static constexpr size_t kFrameSize = 128;
  static constexpr u16 kNumFrames =
      static_cast<u16>(kSizeBytes / kFrameSize);

  MemoryCard();

  bool load_or_create(const std::string &path);
  void eject();
  bool save_if_dirty();
  void reset();

  void reset_transfer_state();
  TransferResult transfer(u8 data_in);

  bool inserted() const { return inserted_; }
  bool dirty() const { return dirty_; }
  const std::string &path() const { return path_; }

private:
  enum class State : u8 {
    Idle,
    Command,

    ReadCardID1,
    ReadCardID2,
    ReadAddressMSB,
    ReadAddressLSB,
    ReadACK1,
    ReadACK2,
    ReadConfirmAddressMSB,
    ReadConfirmAddressLSB,
    ReadData,
    ReadChecksum,
    ReadEnd,

    WriteCardID1,
    WriteCardID2,
    WriteAddressMSB,
    WriteAddressLSB,
    WriteData,
    WriteChecksum,
    WriteACK1,
    WriteACK2,
    WriteEnd,

    GetIDCardID1,
    GetIDCardID2,
    GetIDACK1,
    GetIDACK2,
    GetID1,
    GetID2,
    GetID3,
    GetID4,
  };

  static constexpr u8 kFlagWriteError = 0x04;
  static constexpr u8 kFlagNoWriteYet = 0x08;

  static void format_data(std::array<u8, kSizeBytes> &data);
  static u8 frame_checksum(const u8 *frame);

  bool load_raw_file(const std::string &path, std::array<u8, kSizeBytes> &out_data);
  bool save_raw_file(const std::string &path, const std::array<u8, kSizeBytes> &data);
  bool write_byte(u16 address, u8 offset, u8 value);
  u8 read_byte(u16 address, u8 offset) const;

  State state_ = State::Idle;
  u8 flags_ = kFlagNoWriteYet;
  u16 address_ = 0;
  u8 sector_offset_ = 0;
  u8 checksum_ = 0;
  u8 last_byte_ = 0;

  bool inserted_ = false;
  bool dirty_ = false;
  std::string path_;
  std::array<u8, kSizeBytes> data_{};
};
