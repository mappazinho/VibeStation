#include "dma.h"
#include "system.h"
#include <algorithm>
#include <chrono>
#include <vector>

void DmaController::recompute_dicr_master(bool request_irq_on_rise) {
  const bool old_master = (dicr_ & 0x80000000u) != 0;
  const bool force_irq = ((dicr_ >> 15) & 1u) != 0;
  const bool master_enable = ((dicr_ >> 23) & 1u) != 0;
  const u32 channel_flags = (dicr_ >> 24) & 0x7Fu;
  const u32 channel_enables = (dicr_ >> 16) & 0x7Fu;
  const bool any_triggered = (channel_flags & channel_enables) != 0;
  const bool master = force_irq || (master_enable && any_triggered);

  if (master) {
    dicr_ |= 0x80000000u;
  } else {
    dicr_ &= ~0x80000000u;
  }

  if (request_irq_on_rise && !old_master && master) {
    sys_->irq().request(Interrupt::DMA);
  }
}

void DmaController::reset() {
  for (auto &ch : channels_) {
    ch = {};
  }
  for (auto &dbg : last_debug_) {
    dbg = {};
  }
  dpcr_ = 0x07654321;
  dicr_ = 0;
}

u32 DmaController::read(u32 offset) const {
  if (offset < 0x70) {
    int channel = (offset >> 4) & 0x7;
    int reg = offset & 0xF;

    if (channel >= 7) {
      return 0;
    }

    const auto &ch = channels_[channel];
    switch (reg) {
    case 0x0:
      return ch.base_addr;
    case 0x4:
      return ch.block_ctrl;
    case 0x8:
      return ch.channel_ctrl;
    default:
      return 0;
    }
  }

  switch (offset) {
  case 0x70:
    return dpcr_;
  case 0x74:
    return dicr_;
  default:
    return 0;
  }
}

void DmaController::write(u32 offset, u32 value) {
  if (offset < 0x70) {
    int channel = (offset >> 4) & 0x7;
    int reg = offset & 0xF;

    if (channel >= 7) {
      return;
    }

    auto &ch = channels_[channel];
    switch (reg) {
    case 0x0:
      ch.base_addr = value & 0x00FFFFFF;
      break;
    case 0x4:
      ch.block_ctrl = value;
      ch.block_words_remaining = 0;
      break;
    case 0x8:
      ch.block_words_remaining = 0;
      ch.channel_ctrl = value;
      if (ch.is_active() && channel_enabled(channel) && request_active(channel)) {
        execute_dma(channel);
      }
      break;
    }
    return;
  }

  switch (offset) {
  case 0x70:
    dpcr_ = value;
    break;
  case 0x74: {
    const u32 old_flags = (dicr_ >> 24) & 0x7Fu;
    const u32 ack_flags = (value >> 24) & 0x7Fu;
    const u32 new_flags = old_flags & ~ack_flags;
    const u32 writable_control = value & 0x00FF803Fu;
    const u32 old_master = dicr_ & 0x80000000u;
    dicr_ = old_master | writable_control | (new_flags << 24);
    recompute_dicr_master(true);
    break;
  }
  }
}

void DmaController::execute_dma(int channel) {
  auto &ch = channels_[channel];
  switch (ch.sync_mode()) {
  case DmaChannel::SyncMode::Immediate:
    dma_block(channel);
    transfer_complete(channel);
    break;
  case DmaChannel::SyncMode::Block:
    dma_block(channel);
    if (ch.block_count() == 0 && ch.block_words_remaining == 0) {
      transfer_complete(channel);
    }
    break;
  case DmaChannel::SyncMode::LinkedList:
    dma_linked_list(channel);
    transfer_complete(channel);
    break;
  }
}

void DmaController::dma_block(int channel) {
  auto &ch = channels_[channel];

  u32 addr = ch.base_addr;
  u32 word_count = 0;

  if (ch.sync_mode() == DmaChannel::SyncMode::Immediate) {
    word_count = ch.block_size();
    if (word_count == 0)
      word_count = 0x10000;
  } else {
    if (ch.block_words_remaining == 0) {
      word_count = ch.block_size();
      if (word_count == 0) {
        word_count = 0x10000;
      }
      ch.block_words_remaining = word_count;
    }
    word_count = ch.block_words_remaining;
  }

  u32 transfer_words = word_count;

  s32 step;
  if ((ch.channel_ctrl >> 1) & 1) {
    step = -4;
  } else {
    step = 4;
  }

  bool from_ram = ch.from_ram();
  auto &dbg = last_debug_[channel];
  dbg.base_addr = addr & 0x00FFFFFFu;
  dbg.block_ctrl = ch.block_ctrl;
  dbg.channel_ctrl = ch.channel_ctrl;
  dbg.transfer_words = transfer_words;
  dbg.cpu_cycle = sys_ ? sys_->cpu().cycle_count() : 0;
  dbg.from_ram = from_ram;
  dbg.first_addr = addr & 0x001FFFFCu;
  if (transfer_words != 0) {
    const s32 span = step * static_cast<s32>(transfer_words - 1);
    dbg.last_addr =
        static_cast<u32>(static_cast<s32>(dbg.first_addr) + span) & 0x001FFFFCu;
  } else {
    dbg.last_addr = dbg.first_addr;
  }

  if (!from_ram && ch.sync_mode() == DmaChannel::SyncMode::Block) {
    if (channel == 1 && sys_->mdec_dma_out_words_available() < transfer_words) {
       transfer_words = sys_->mdec_dma_out_words_available();
    } else if (channel == 3 && sys_->cdrom_dma_words_available() < transfer_words) {
       transfer_words = sys_->cdrom_dma_words_available();
    }
    if (transfer_words == 0) {
      return;
    }
  }

  // Only truncate MDEC in if it is block mode.
  if (from_ram && channel == 0 && ch.sync_mode() == DmaChannel::SyncMode::Block) {
    transfer_words = std::min(transfer_words, sys_->mdec_dma_in_words_capacity());
    if (transfer_words == 0) {
      return;
    }
  }

  if (channel == 6) {
    u32 current = addr;
    for (u32 i = 0; i < transfer_words; i++) {
      u32 val;
      if (i == transfer_words - 1) {
        val = 0x00FFFFFF;
      } else {
        val = (current - 4) & 0x001FFFFC;
      }
      sys_->write32(current, val);
      current -= 4;
    }
    if (ch.sync_mode() == DmaChannel::SyncMode::Block) {
      ch.base_addr = current & 0x00FFFFFF;
      if (ch.block_words_remaining > transfer_words) {
        ch.block_words_remaining -= transfer_words;
      } else {
        ch.block_words_remaining = 0;
      }
      if (ch.block_words_remaining == 0) {
        const u16 remaining = ch.block_count();
        const u16 next = (remaining > 0) ? static_cast<u16>(remaining - 1) : 0;
        ch.block_ctrl =
            (ch.block_ctrl & 0x0000FFFFu) | (static_cast<u32>(next) << 16);
      }
    }
    return;
  }

  if (from_ram && channel == 0 && transfer_words != 0u) {
    std::vector<u32> words(transfer_words);
    u32 current = ch.base_addr;
    for (u32 i = 0; i < transfer_words; ++i) {
      words[i] = sys_->read32(current & 0x001FFFFCu);
      current = static_cast<u32>(static_cast<s32>(current) + step);
    }
    sys_->mdec_dma_write(words.data(), transfer_words);
    addr = current;
  } else if (!from_ram && channel == 1 && transfer_words != 0u) {
    std::vector<u32> words(transfer_words);
    sys_->mdec_dma_read(words.data(), transfer_words);
    u32 current = ch.base_addr;
    for (u32 i = 0; i < transfer_words; ++i) {
      sys_->write32(current & 0x001FFFFCu, words[i]);
      current = static_cast<u32>(static_cast<s32>(current) + step);
    }
    addr = current;
  } else {
    for (u32 i = 0; i < transfer_words; ++i) {
      const u32 current_addr = addr & 0x001FFFFCu;
      if (from_ram) {
        const u32 data = sys_->read32(current_addr);
        switch (channel) {
        case 2:
          sys_->gpu_gp0(data);
          break;
        case 4:
          sys_->spu_dma_write(data);
          break;
        default:
          break;
        }
      } else {
        u32 data = 0;
        switch (channel) {
        case 2:
          data = sys_->gpu_read();
          break;
        case 3:
          data = sys_->cdrom_dma_read();
          break;
        case 4:
          data = sys_->spu_dma_read();
          break;
        default:
          break;
        }
        sys_->write32(current_addr, data);
      }
      addr = static_cast<u32>(static_cast<s32>(addr) + step);
    }
  }

  ch.base_addr = addr & 0x00FFFFFF;
  if (ch.sync_mode() == DmaChannel::SyncMode::Block) {
    if (ch.block_words_remaining > transfer_words) {
      ch.block_words_remaining -= transfer_words;
    } else {
      ch.block_words_remaining = 0;
    }
    if (ch.block_words_remaining == 0) {
      const u16 remaining = ch.block_count();
      const u16 next = (remaining > 0) ? static_cast<u16>(remaining - 1) : 0;
      ch.block_ctrl =
          (ch.block_ctrl & 0x0000FFFFu) | (static_cast<u32>(next) << 16);
    }
  }
}

void DmaController::dma_linked_list(int channel) {
  if (channel != 2) return;
  u32 addr = channels_[channel].base_addr & 0x001FFFFC;
  u32 safety = 0;
  while (safety < 0x100000) {
    u32 header = sys_->read32(addr);
    u32 word_count = header >> 24;
    for (u32 i = 0; i < word_count; i++) {
      addr = (addr + 4) & 0x001FFFFC;
      sys_->gpu_gp0(sys_->read32(addr));
    }
    if (header & 0x00800000) break;
    addr = header & 0x001FFFFC;
    safety++;
  }
}

void DmaController::transfer_complete(int channel) {
  auto &ch = channels_[channel];
  ch.channel_ctrl &= ~(1u << 24);
  ch.channel_ctrl &= ~(1u << 28);
  ch.block_words_remaining = 0;
  dicr_ |= (1u << (24 + channel));
  recompute_dicr_master(true);
}

void DmaController::tick() {
  for (int i = 0; i < 7; i++) {
    if (channels_[i].is_active() && channel_enabled(i) && request_active(i)) {
      execute_dma(i);
    }
  }
}

bool DmaController::request_active(int channel) const {
  const DmaChannel &ch = channels_[channel];
  if (ch.sync_mode() == DmaChannel::SyncMode::Immediate) {
    if (channel == 0) return sys_->mdec_dma_in_request();
    if (channel == 3) {
      u32 words = ch.block_size();
      if (words == 0u) words = 0x10000u;
      return sys_->cdrom_dma_request() && (sys_->cdrom_dma_words_available() >= words);
    }
    return true;
  }
  if (ch.sync_mode() == DmaChannel::SyncMode::LinkedList) return true;

  switch (channel) {
  case 0: return sys_->mdec_dma_in_request();
  case 1: return sys_->mdec_dma_out_request();
  case 2: return sys_->gpu_dma_request();
  case 3: return sys_->cdrom_dma_request();
  case 4: return sys_->spu_dma_request();
  default: return true;
  }
}
