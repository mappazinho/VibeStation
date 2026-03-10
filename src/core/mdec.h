#pragma once

#include "types.h"

#include <array>

class Mdec {
public:
  struct DebugStats {
    u64 command_words = 0;
    u64 command_id0 = 0;
    u64 command_id1 = 0;
    u64 command_id2 = 0;
    u64 command_id3 = 0;
    u64 command_id_other = 0;
    u32 last_command_word = 0;
    u8 last_command_id = 0;
    u64 decode_commands = 0;
    u64 decode_data_words = 0;
    u64 decode_execute_calls = 0;
    u64 decode_macroblocks = 0;
    u64 out_words_pushed = 0;
    u64 out_words_read = 0;
    u64 decode_wait_breaks = 0;
  };

  void reset();

  void write_command(u32 value);
  void write_control(u32 value);
  u32 read_data();
  u32 read_status() const;

  void dma_write(u32 value) { write_command(value); }
  void dma_write(const u32 *words, u32 word_count);
  u32 dma_read() { return read_data(); }
  void dma_read(u32 *words, u32 word_count);
  bool dma_in_request() const;
  bool dma_out_request() const;
  u32 dma_in_words_capacity() const;
  u32 dma_out_words_available() const;
  const DebugStats &debug_stats() const { return debug_stats_; }

private:
  template <typename T, size_t Capacity> class InlineFifo {
  public:
    void clear() {
      head_ = 0;
      size_ = 0;
    }

    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == Capacity; }
    size_t size() const { return size_; }
    size_t space() const { return Capacity - size_; }

    void push(const T &value) {
      if (full()) {
        return;
      }
      data_[(head_ + size_) % Capacity] = value;
      ++size_;
    }

    T pop() {
      const T value = data_[head_];
      head_ = (head_ + 1) % Capacity;
      --size_;
      return value;
    }

    const T &peek(size_t index = 0) const {
      return data_[(head_ + index) % Capacity];
    }

    void remove(size_t count) {
      if (count >= size_) {
        clear();
        return;
      }
      head_ = (head_ + count) % Capacity;
      size_ -= count;
    }

    void push_range(const T *values, size_t count) {
      const size_t count_to_write = (count < space()) ? count : space();
      for (size_t i = 0; i < count_to_write; ++i) {
        push(values[i]);
      }
    }

    void pop_range(T *values, size_t count) {
      const size_t count_to_read = (count < size_) ? count : size_;
      for (size_t i = 0; i < count_to_read; ++i) {
        values[i] = pop();
      }
    }

  private:
    std::array<T, Capacity> data_{};
    size_t head_ = 0;
    size_t size_ = 0;
  };

  static constexpr size_t kBlockSize = 64;
  static constexpr size_t kNumBlocks = 6;
  static constexpr size_t kDataInFifoHalfwords = 1024u / sizeof(u16);
  static constexpr size_t kDataOutFifoWords = 768u;
  static constexpr size_t kDmaInRequestThresholdHalfwords = 32u * 2u;

  enum class State : u8 {
    Idle = 0,
    DecodingMacroblock,
    WritingMacroblock,
    SetIqTable,
    SetScaleTable,
    NoCommand,
  };

  using Block = std::array<s16, kBlockSize>;
  using MacroblockPixels = std::array<u32, 16u * 16u>;

  void begin_command(u32 value);
  void execute();
  void reset_decoder();
  bool handle_decode_macroblock_command();
  bool decode_mono_macroblock();
  bool decode_colored_macroblock();
  void handle_set_quant_table_command();
  void handle_set_scale_command();
  bool decode_rle_new(s16 *blk, const std::array<u8, kBlockSize> &qt);
  void idct_new(s16 *blk) const;
  void yuv_to_rgb(u32 xx, u32 yy, const Block &cr, const Block &cb,
                  const Block &y);
  void yuv_to_mono(const Block &y);
  void flush_output_macroblock();

  static int sign_extend_10(u16 value);
  static int sign_extend_9(int value);
  u8 encode_component(int value) const;

  u32 control_ = 0;
  State state_ = State::Idle;
  u32 remaining_halfwords_ = 0;

  InlineFifo<u16, kDataInFifoHalfwords> in_fifo_{};
  InlineFifo<u32, kDataOutFifoWords> out_fifo_{};

  std::array<u8, kBlockSize> iq_uv_{};
  std::array<u8, kBlockSize> iq_y_{};
  std::array<s16, kBlockSize> scale_table_{};
  std::array<Block, kNumBlocks> blocks_{};
  MacroblockPixels block_rgb_{};

  u32 current_block_ = 0;
  u32 current_coefficient_ = kBlockSize;
  u16 current_q_scale_ = 0;

  u8 command_id_ = 0;
  u32 command_word_ = 0;
  u8 output_depth_ = 2;
  bool output_signed_ = false;
  bool output_set_bit15_ = false;

  DebugStats debug_stats_{};
};
