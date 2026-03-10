#include "mdec.h"

#include <algorithm>
#include <array>

namespace {

constexpr std::array<s16, 64> kDefaultScaleTable = {
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0x5A82),
    static_cast<s16>(0x7D8A), static_cast<s16>(0x6A6D),
    static_cast<s16>(0x471C), static_cast<s16>(0x18F8),
    static_cast<s16>(0xE707), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x9592), static_cast<s16>(0x8275),
    static_cast<s16>(0x7641), static_cast<s16>(0x30FB),
    static_cast<s16>(0xCF04), static_cast<s16>(0x89BE),
    static_cast<s16>(0x89BE), static_cast<s16>(0xCF04),
    static_cast<s16>(0x30FB), static_cast<s16>(0x7641),
    static_cast<s16>(0x6A6D), static_cast<s16>(0xE707),
    static_cast<s16>(0x8275), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x471C), static_cast<s16>(0x7D8A),
    static_cast<s16>(0x18F8), static_cast<s16>(0x9592),
    static_cast<s16>(0x5A82), static_cast<s16>(0xA57D),
    static_cast<s16>(0xA57D), static_cast<s16>(0x5A82),
    static_cast<s16>(0x5A82), static_cast<s16>(0xA57D),
    static_cast<s16>(0xA57D), static_cast<s16>(0x5A82),
    static_cast<s16>(0x471C), static_cast<s16>(0x8275),
    static_cast<s16>(0x18F8), static_cast<s16>(0x6A6D),
    static_cast<s16>(0x9592), static_cast<s16>(0xE707),
    static_cast<s16>(0x7D8A), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x30FB), static_cast<s16>(0x89BE),
    static_cast<s16>(0x7641), static_cast<s16>(0xCF04),
    static_cast<s16>(0xCF04), static_cast<s16>(0x7641),
    static_cast<s16>(0x89BE), static_cast<s16>(0x30FB),
    static_cast<s16>(0x18F8), static_cast<s16>(0xB8E3),
    static_cast<s16>(0x6A6D), static_cast<s16>(0x8275),
    static_cast<s16>(0x7D8A), static_cast<s16>(0x9592),
    static_cast<s16>(0x471C), static_cast<s16>(0xE707),
};

// Row-major zigzag order, matching the reference MDEC decode path.
constexpr std::array<u8, 64> kDecodeZigZag = {
    0,  8,  1,  2,  9,  16, 24, 17, 10, 3,  4,  11, 18, 25, 32, 40,
    33, 26, 19, 12, 5,  6,  13, 20, 27, 34, 41, 48, 56, 49, 42, 35,
    28, 21, 14, 7,  15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30,
    23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63,
};

constexpr u32 pack_rgb(u8 r, u8 g, u8 b) {
  return static_cast<u32>(r) | (static_cast<u32>(g) << 8) |
         (static_cast<u32>(b) << 16);
}

} // namespace

void Mdec::reset() {
  control_ = 0;
  state_ = State::Idle;
  remaining_halfwords_ = 0;
  in_fifo_.clear();
  out_fifo_.clear();
  iq_uv_.fill(0);
  iq_y_.fill(0);
  scale_table_ = kDefaultScaleTable;
  for (auto &block : blocks_) {
    block.fill(0);
  }
  block_rgb_.fill(0);
  current_block_ = 0;
  current_coefficient_ = kBlockSize;
  current_q_scale_ = 0;
  command_id_ = 0;
  command_word_ = 0;
  output_depth_ = 2;
  output_signed_ = false;
  output_set_bit15_ = false;
  debug_stats_ = {};
}

void Mdec::write_command(u32 value) {
  if (in_fifo_.space() < 2u) {
    return;
  }

  in_fifo_.push(static_cast<u16>(value & 0xFFFFu));
  in_fifo_.push(static_cast<u16>(value >> 16));
  if (state_ == State::DecodingMacroblock || state_ == State::WritingMacroblock) {
    ++debug_stats_.decode_data_words;
  }
  execute();
}

void Mdec::write_control(u32 value) {
  if ((value & 0x80000000u) != 0u) {
    reset();
    return;
  }

  control_ = value;
  execute();
}

void Mdec::dma_write(const u32 *words, u32 word_count) {
  if (words == nullptr || word_count == 0u) {
    return;
  }

  const u32 words_to_write =
      std::min<u32>(word_count, static_cast<u32>(in_fifo_.space() / 2u));
  if (words_to_write == 0u) {
    return;
  }

  for (u32 i = 0; i < words_to_write; ++i) {
    in_fifo_.push(static_cast<u16>(words[i] & 0xFFFFu));
    in_fifo_.push(static_cast<u16>(words[i] >> 16));
  }

  if (state_ == State::DecodingMacroblock || state_ == State::WritingMacroblock) {
    debug_stats_.decode_data_words += words_to_write;
  }
  execute();
}

u32 Mdec::read_data() {
  if (out_fifo_.empty()) {
    return 0xFFFFFFFFu;
  }

  const u32 value = out_fifo_.pop();
  ++debug_stats_.out_words_read;

  if (out_fifo_.empty() && state_ == State::WritingMacroblock) {
    state_ = (remaining_halfwords_ == 0u) ? State::Idle : State::DecodingMacroblock;
    execute();
  }

  return value;
}

void Mdec::dma_read(u32 *words, u32 word_count) {
  if (words == nullptr || word_count == 0u) {
    return;
  }

  const u32 words_to_read =
      std::min<u32>(word_count, static_cast<u32>(out_fifo_.size()));
  out_fifo_.pop_range(words, words_to_read);
  for (u32 i = words_to_read; i < word_count; ++i) {
    words[i] = 0xFFFFFFFFu;
  }
  debug_stats_.out_words_read += words_to_read;

  if (out_fifo_.empty() && state_ == State::WritingMacroblock) {
    state_ = (remaining_halfwords_ == 0u) ? State::Idle : State::DecodingMacroblock;
    execute();
  }
}

u32 Mdec::read_status() const {
  u32 status = 0;
  if (out_fifo_.empty()) {
    status |= (1u << 31);
  }
  if (in_fifo_.full()) {
    status |= (1u << 30);
  }
  if (state_ != State::Idle) {
    status |= (1u << 29);
  }
  if (dma_in_request()) {
    status |= (1u << 28);
  }
  if (dma_out_request()) {
    status |= (1u << 27);
  }
  status |= (static_cast<u32>(output_depth_ & 0x3u) << 25);
  if (output_signed_) {
    status |= (1u << 24);
  }
  if (output_set_bit15_) {
    status |= (1u << 23);
  }
  status |= (((current_block_ + 4u) % static_cast<u32>(kNumBlocks)) & 0x7u)
            << 16;
  status |= ((remaining_halfwords_ / 2u) - 1u) & 0xFFFFu;
  return status;
}

bool Mdec::dma_in_request() const {
  return ((control_ & 0x40000000u) != 0u) &&
         (in_fifo_.space() >= kDmaInRequestThresholdHalfwords);
}

bool Mdec::dma_out_request() const {
  return ((control_ & 0x20000000u) != 0u) && !out_fifo_.empty();
}

u32 Mdec::dma_in_words_capacity() const {
  return static_cast<u32>(in_fifo_.space() / 2u);
}

u32 Mdec::dma_out_words_available() const {
  return static_cast<u32>(out_fifo_.size());
}

void Mdec::begin_command(u32 value) {
  ++debug_stats_.command_words;
  command_word_ = value;
  command_id_ = static_cast<u8>((value >> 29) & 0x7u);
  debug_stats_.last_command_word = value;
  debug_stats_.last_command_id = command_id_;

  switch (command_id_) {
  case 0:
    ++debug_stats_.command_id0;
    break;
  case 1:
    ++debug_stats_.command_id1;
    break;
  case 2:
    ++debug_stats_.command_id2;
    break;
  case 3:
    ++debug_stats_.command_id3;
    break;
  default:
    ++debug_stats_.command_id_other;
    break;
  }

  output_depth_ = static_cast<u8>((value >> 27) & 0x3u);
  output_signed_ = (value & (1u << 26)) != 0u;
  output_set_bit15_ = (value & (1u << 25)) != 0u;
  out_fifo_.clear();

  switch (command_id_) {
  case 1:
  case 4:
    ++debug_stats_.decode_commands;
    remaining_halfwords_ = (value & 0xFFFFu) * 2u;
    reset_decoder();
    state_ = State::DecodingMacroblock;
    break;

  case 2:
    remaining_halfwords_ = ((value & 0x1u) != 0u) ? 64u : 32u;
    state_ = State::SetIqTable;
    break;

  case 3:
    remaining_halfwords_ = 64u;
    state_ = State::SetScaleTable;
    break;

  default:
    remaining_halfwords_ = (value & 0xFFFFu) * 2u;
    state_ = (remaining_halfwords_ == 0u) ? State::Idle : State::NoCommand;
    break;
  }
}

void Mdec::execute() {
  for (;;) {
    switch (state_) {
    case State::Idle: {
      if (in_fifo_.size() < 2u) {
        return;
      }

      const u32 command = static_cast<u32>(in_fifo_.peek(0)) |
                          (static_cast<u32>(in_fifo_.peek(1)) << 16);
      in_fifo_.remove(2u);
      begin_command(command);
      continue;
    }

    case State::DecodingMacroblock:
      ++debug_stats_.decode_execute_calls;
      if (handle_decode_macroblock_command()) {
        return;
      }
      if (remaining_halfwords_ == 0u &&
          current_block_ != static_cast<u32>(kNumBlocks)) {
        reset_decoder();
        state_ = State::Idle;
        continue;
      }
      ++debug_stats_.decode_wait_breaks;
      return;

    case State::WritingMacroblock:
      return;

    case State::SetIqTable:
      if (in_fifo_.size() < remaining_halfwords_) {
        return;
      }
      handle_set_quant_table_command();
      state_ = State::Idle;
      continue;

    case State::SetScaleTable:
      if (in_fifo_.size() < remaining_halfwords_) {
        return;
      }
      handle_set_scale_command();
      state_ = State::Idle;
      continue;

    case State::NoCommand: {
      const size_t halfwords_to_consume =
          std::min<size_t>(remaining_halfwords_, in_fifo_.size());
      in_fifo_.remove(halfwords_to_consume);
      remaining_halfwords_ -= static_cast<u32>(halfwords_to_consume);
      if (remaining_halfwords_ != 0u) {
        return;
      }
      state_ = State::Idle;
      continue;
    }
    }
  }
}

void Mdec::reset_decoder() {
  current_block_ = 0;
  current_coefficient_ = kBlockSize;
  current_q_scale_ = 0;
}

bool Mdec::handle_decode_macroblock_command() {
  return (output_depth_ <= 1u) ? decode_mono_macroblock()
                               : decode_colored_macroblock();
}

bool Mdec::decode_mono_macroblock() {
  if (!out_fifo_.empty()) {
    return false;
  }

  if (!decode_rle_new(blocks_[0].data(), iq_y_)) {
    return false;
  }

  idct_new(blocks_[0].data());
  yuv_to_mono(blocks_[0]);
  flush_output_macroblock();
  ++debug_stats_.decode_macroblocks;
  reset_decoder();
  state_ = State::WritingMacroblock;
  return true;
}

bool Mdec::decode_colored_macroblock() {
  if (!out_fifo_.empty()) {
    return false;
  }

  for (; current_block_ < static_cast<u32>(kNumBlocks); ++current_block_) {
    const auto &qt = (current_block_ >= 2u) ? iq_y_ : iq_uv_;
    if (!decode_rle_new(blocks_[current_block_].data(), qt)) {
      return false;
    }
    idct_new(blocks_[current_block_].data());
  }

  yuv_to_rgb(0, 0, blocks_[0], blocks_[1], blocks_[2]);
  yuv_to_rgb(8, 0, blocks_[0], blocks_[1], blocks_[3]);
  yuv_to_rgb(0, 8, blocks_[0], blocks_[1], blocks_[4]);
  yuv_to_rgb(8, 8, blocks_[0], blocks_[1], blocks_[5]);
  flush_output_macroblock();
  ++debug_stats_.decode_macroblocks;
  reset_decoder();
  state_ = State::WritingMacroblock;
  return true;
}

void Mdec::handle_set_quant_table_command() {
  for (size_t i = 0; i < iq_y_.size(); i += 2u) {
    const u16 value = in_fifo_.pop();
    iq_y_[i + 0u] = static_cast<u8>(value & 0xFFu);
    iq_y_[i + 1u] = static_cast<u8>(value >> 8);
  }
  remaining_halfwords_ -= 32u;

  if (remaining_halfwords_ != 0u) {
    for (size_t i = 0; i < iq_uv_.size(); i += 2u) {
      const u16 value = in_fifo_.pop();
      iq_uv_[i + 0u] = static_cast<u8>(value & 0xFFu);
      iq_uv_[i + 1u] = static_cast<u8>(value >> 8);
    }
    remaining_halfwords_ -= 32u;
  }
}

void Mdec::handle_set_scale_command() {
  std::array<u16, kBlockSize> packed{};
  for (size_t i = 0; i < packed.size(); ++i) {
    packed[i] = in_fifo_.pop();
  }
  remaining_halfwords_ = 0;

  for (size_t y = 0; y < 8u; ++y) {
    for (size_t x = 0; x < 8u; ++x) {
      scale_table_[y * 8u + x] = static_cast<s16>(packed[x * 8u + y]);
    }
  }
}

bool Mdec::decode_rle_new(s16 *blk, const std::array<u8, kBlockSize> &qt) {
  if (current_coefficient_ == kBlockSize) {
    std::fill_n(blk, static_cast<int>(kBlockSize), static_cast<s16>(0));

    u16 value = 0;
    for (;;) {
      if (in_fifo_.empty() || remaining_halfwords_ == 0u) {
        return false;
      }

      value = in_fifo_.pop();
      --remaining_halfwords_;
      if (value != 0xFE00u) {
        break;
      }
    }

    current_coefficient_ = 0;
    current_q_scale_ = static_cast<u16>(value >> 10);

    const s32 level = sign_extend_10(value);
    const s32 coeff =
        (current_q_scale_ == 0u)
            ? (level << 5)
            : (((level * static_cast<s32>(qt[0])) << 4) +
               (level == 0 ? 0 : ((level < 0) ? 8 : -8)));
    blk[kDecodeZigZag[0]] =
        static_cast<s16>(std::clamp(coeff, -0x4000, 0x3FFF));
  }

  while (!in_fifo_.empty() && remaining_halfwords_ > 0u) {
    const u16 value = in_fifo_.pop();
    --remaining_halfwords_;

    current_coefficient_ += ((value >> 10) & 0x3Fu) + 1u;
    if (current_coefficient_ < kBlockSize) {
      const s32 level = sign_extend_10(value);
      const s32 scale =
          static_cast<s32>(current_q_scale_) *
          static_cast<s32>(qt[current_coefficient_]);
      const s32 coeff =
          (scale == 0)
              ? (level << 5)
              : ((((level * scale) >> 3) << 4) +
                 (level == 0 ? 0 : ((level < 0) ? 8 : -8)));
      blk[kDecodeZigZag[current_coefficient_]] =
          static_cast<s16>(std::clamp(coeff, -0x4000, 0x3FFF));
    }

    if (current_coefficient_ >= (kBlockSize - 1u)) {
      current_coefficient_ = kBlockSize;
      return true;
    }
  }

  return false;
}

void Mdec::idct_new(s16 *blk) const {
  auto idct_row = [](const s16 *src, const s16 *matrix) -> s16 {
    s64 sum = 0;
    for (int i = 0; i < 8; ++i) {
      sum += static_cast<s64>(src[i]) * static_cast<s64>(matrix[i]);
    }
    return static_cast<s16>((sum + 0x20000) >> 18);
  };

  std::array<s16, kBlockSize> temp{};
  for (u32 x = 0; x < 8u; ++x) {
    for (u32 y = 0; y < 8u; ++y) {
      temp[y * 8u + x] = idct_row(&blk[x * 8u], &scale_table_[y * 8u]);
    }
  }

  for (u32 x = 0; x < 8u; ++x) {
    for (u32 y = 0; y < 8u; ++y) {
      const int value = idct_row(&temp[x * 8u], &scale_table_[y * 8u]);
      blk[x * 8u + y] =
          static_cast<s16>(std::clamp(sign_extend_9(value), -128, 127));
    }
  }
}

void Mdec::yuv_to_rgb(u32 xx, u32 yy, const Block &cr, const Block &cb,
                      const Block &y) {
  for (u32 py = 0; py < 8u; ++py) {
    for (u32 px = 0; px < 8u; ++px) {
      const int chroma_index =
          static_cast<int>(((px + xx) / 2u) + (((py + yy) / 2u) * 8u));
      const int crv = cr[chroma_index];
      const int cbv = cb[chroma_index];
      const int yv = y[py * 8u + px];

      const int r = sign_extend_9(yv + (((359 * crv) + 0x80) >> 8));
      const int g = sign_extend_9(
          yv + ((((-88 * cbv) & ~0x1F) + ((-183 * crv) & ~0x07) + 0x80) >> 8));
      const int b = sign_extend_9(yv + (((454 * cbv) + 0x80) >> 8));

      block_rgb_[(px + xx) + ((py + yy) * 16u)] =
          pack_rgb(encode_component(std::clamp(r, -128, 127)),
                   encode_component(std::clamp(g, -128, 127)),
                   encode_component(std::clamp(b, -128, 127)));
    }
  }
}

void Mdec::yuv_to_mono(const Block &y) {
  for (size_t i = 0; i < y.size(); ++i) {
    const u8 luma =
        encode_component(std::clamp(sign_extend_9(y[i]), -128, 127));
    block_rgb_[i] = pack_rgb(luma, luma, luma);
  }
}

void Mdec::flush_output_macroblock() {
  switch (output_depth_) {
  case 0: {
    for (u32 i = 0; i < 64u; i += 8u) {
      u32 value = (block_rgb_[i + 0u] >> 4);
      value |= (block_rgb_[i + 1u] >> 4) << 4;
      value |= (block_rgb_[i + 2u] >> 4) << 8;
      value |= (block_rgb_[i + 3u] >> 4) << 12;
      value |= (block_rgb_[i + 4u] >> 4) << 16;
      value |= (block_rgb_[i + 5u] >> 4) << 20;
      value |= (block_rgb_[i + 6u] >> 4) << 24;
      value |= (block_rgb_[i + 7u] >> 4) << 28;
      out_fifo_.push(value);
      ++debug_stats_.out_words_pushed;
    }
    break;
  }

  case 1: {
    for (u32 i = 0; i < 64u; i += 4u) {
      u32 value = block_rgb_[i + 0u];
      value |= block_rgb_[i + 1u] << 8;
      value |= block_rgb_[i + 2u] << 16;
      value |= block_rgb_[i + 3u] << 24;
      out_fifo_.push(value);
      ++debug_stats_.out_words_pushed;
    }
    break;
  }

  case 2: {
    u32 index = 0;
    u32 pack_state = 0;
    u32 rgb = 0;
    while (index < block_rgb_.size()) {
      switch (pack_state) {
      case 0:
        rgb = block_rgb_[index++];
        pack_state = 1;
        break;
      case 1:
        rgb |= (block_rgb_[index] & 0xFFu) << 24;
        out_fifo_.push(rgb);
        ++debug_stats_.out_words_pushed;
        rgb = block_rgb_[index] >> 8;
        ++index;
        pack_state = 2;
        break;
      case 2:
        rgb |= block_rgb_[index] << 16;
        out_fifo_.push(rgb);
        ++debug_stats_.out_words_pushed;
        rgb = block_rgb_[index] >> 16;
        ++index;
        pack_state = 3;
        break;
      default:
        rgb |= block_rgb_[index] << 8;
        out_fifo_.push(rgb);
        ++debug_stats_.out_words_pushed;
        ++index;
        pack_state = 0;
        break;
      }
    }
    break;
  }

  case 3: {
    const u32 alpha = output_set_bit15_ ? 0x8000u : 0u;
    for (u32 i = 0; i < block_rgb_.size(); i += 2u) {
      const auto pack_15bit = [&](u32 color) -> u32 {
        const u32 r = std::min<u32>((((color >> 0) & 0xFFu) + 4u) >> 3, 0x1Fu);
        const u32 g = std::min<u32>((((color >> 8) & 0xFFu) + 4u) >> 3, 0x1Fu);
        const u32 b =
            std::min<u32>((((color >> 16) & 0xFFu) + 4u) >> 3, 0x1Fu);
        return r | (g << 5) | (b << 10) | alpha;
      };

      out_fifo_.push(pack_15bit(block_rgb_[i + 0u]) |
                     (pack_15bit(block_rgb_[i + 1u]) << 16));
      ++debug_stats_.out_words_pushed;
    }
    break;
  }
  }
}

int Mdec::sign_extend_10(u16 value) {
  int result = static_cast<int>(value & 0x03FFu);
  if ((result & 0x0200) != 0) {
    result -= 0x0400;
  }
  return result;
}

int Mdec::sign_extend_9(int value) {
  value &= 0x1FF;
  if ((value & 0x100) != 0) {
    value -= 0x200;
  }
  return value;
}

u8 Mdec::encode_component(int value) const {
  const int clamped = std::clamp(value, -128, 127);
  if (output_signed_) {
    return static_cast<u8>(static_cast<s8>(clamped));
  }
  return static_cast<u8>(clamped + 128);
}
