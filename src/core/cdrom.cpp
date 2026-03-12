  case 0x10:
    cmd_getloc_l();
    break;
  case 0x11:
    cmd_getloc_p();
    break;
  case 0x12:
    cmd_setsession();
    break;
  case 0x13:
    cmd_gettn();
    break;
  case 0x14:
    cmd_gettd();
    break;
  case 0x15:
  case 0x16:
    cmd_seekl();
    break;
  case 0x19:
    cmd_test();
    break;
  case 0x1A:
    cmd_getid();
    break;
  case 0x1B:
    cmd_reads();
    break;
  case 0x1C:
    enqueue_irq(3, {stat_byte()});
    break;
  case 0x1E:
    cmd_readtoc();
    break;
  default:
    enqueue_irq(5, {0x11u, 0x40u});
    break;
  }
}

u8 CdRom::read8(u32 offset) {
  offset &= 0x3u;
  if (g_trace_cdrom) {
    static u64 cdrom_read_counter = 0;
    if (trace_should_log(cdrom_read_counter, g_trace_burst_cdrom,
                         g_trace_stride_cdrom)) {
      LOG_DEBUG("CDROM: read8 off=%u bank=%u", static_cast<unsigned>(offset),
                static_cast<unsigned>(index_reg_));
    }
  }

  switch (offset) {
  case 0: {
    u8 status = static_cast<u8>(index_reg_ & 0x03u);
    if (adpcm_busy_cycles_ > 0) {
      status |= 0x04u;
    }
    if (param_fifo_.empty()) {
      status |= 0x08u;
    }
    if (param_fifo_.size() < kFifoCapacity) {
      status |= 0x10u;
    }
    if (response_index_ < static_cast<int>(response_fifo_.size())) {
      status |= 0x20u;
    }
    if (data_ready_ && data_request_) {
      status |= 0x40u;
    }
    if (command_busy_) {
      status |= 0x80u;
    }

    if (status == 0xE0u) {
      ++status_e0_poll_count_;
      ++status_e0_streak_current_;
      status_e0_streak_max_ =
          std::max(status_e0_streak_max_, status_e0_streak_current_);
    } else {
      status_e0_streak_current_ = 0;
    }
    return status;
  }

  case 1:
    if (response_index_ < static_cast<int>(response_fifo_.size())) {
      const u8 value = response_fifo_[response_index_++];
      if (response_index_ >= static_cast<int>(response_fifo_.size())) {
        response_fifo_.clear();
        response_index_ = 0;
        service_pending_irq();
      }
      return value;
    }
    return 0;

  case 2: {
    if (data_ready_ && data_index_ < static_cast<int>(data_buffer_.size())) {
      const u8 value = data_buffer_[static_cast<size_t>(data_index_++)];
      if (data_index_ >= static_cast<int>(data_buffer_.size())) {
        data_ready_ = false;
      }
      return value;
    }
    return 0;
  }

  case 3:
    if (index_reg_ == 0 || index_reg_ == 2) {
      return static_cast<u8>(interrupt_enable_ | 0xE0u);
    }
    return static_cast<u8>(interrupt_flag_ | 0xE0u);
  }

  return 0xFFu;
}

void CdRom::write8(u32 offset, u8 value) {
  // FIX: offset masking was incorrectly commented out. Without this mask,
  // writes with stray high bits in the offset fall through all switch cases
  // silently, losing commands and parameters entirely.
  offset &= 0x3u;
  if (g_trace_cdrom) {
    static u64 cdrom_write_counter = 0;
    if (trace_should_log(cdrom_write_counter, g_trace_burst_cdrom,
                         g_trace_stride_cdrom)) {
      LOG_DEBUG("CDROM: write8 off=%u bank=%u value=%02X",
                static_cast<unsigned>(offset), static_cast<unsigned>(index_reg_),
                value);
    }
  }

  switch (offset) {
  case 0:
    index_reg_ = value & 0x3;
    return;

  case 1:
    switch (index_reg_) {
    case 0:
      last_command_ = value;
      ++command_counter_;
      ++command_hist_[static_cast<size_t>(value)];
      command_busy_cycles_ = command_busy_for(value);
      command_busy_ = command_busy_cycles_ > 0;
      execute_command(value);
      param_fifo_.clear();
      return;
    case 1:
      host_audio_regs_[6] = value;
      return;
    case 2:
      host_audio_regs_[1] = value;
      return;
    case 3:
      atv_pending_[2] = value;
      return;
    default:
      LOG_WARN("CDROM: Write to port 1 index %u = 0x%02X", index_reg_, value);
      return;
    }

  case 2:
    switch (index_reg_) {
    case 0:
      if (param_fifo_.size() < kFifoCapacity) {
        param_fifo_.push_back(value);
      }
      return;
    case 1:
      interrupt_enable_ = static_cast<u8>(value & kHintMaskAll);
      refresh_irq_line();
      return;
    case 2:
      atv_pending_[0] = value;
      return;
    case 3:
      atv_pending_[3] = value;
      return;
    default:
      return;
    }

  case 3:
    switch (index_reg_) {
    case 0:
      host_audio_regs_[0] = value;
      data_request_ = (value & 0x80u) != 0;
      // FIX D: clearing data_request_ (BFRD=0) must reset the read position
      // so that a subsequent set re-delivers the sector from the beginning.
      // Metal Gear Solid: Special Missions clears BFRD between two DMAs and
      // needs the buffer pointer reset; matches DuckStation's sb.position = 0.
      if (!data_request_) {
        data_index_ = 0;
      }
      if ((value & 0x20u) != 0) {
        host_audio_regs_[7] = 1;
      }
      return;

    case 1:
      if ((value & 0x40u) != 0) {
        param_fifo_.clear();
      }
      if ((value & 0x20u) != 0) {
        adpcm_busy_cycles_ = 0;
      }
      if ((value & 0x1Fu) != 0) {
        const u8 ack_mask = static_cast<u8>(value & 0x1Fu);
        const u8 old_type = static_cast<u8>(interrupt_flag_ & kHintTypeMask);
        const u8 old_mask = hint_to_mask(old_type);
        if ((old_mask & ack_mask) != 0u) {
          interrupt_flag_ =
              static_cast<u8>(interrupt_flag_ & static_cast<u8>(~kHintTypeMask));
          response_fifo_.clear();
          response_index_ = 0;
        }
      }
      if ((value & 0x80u) != 0) {
        response_fifo_.clear();
        response_index_ = 0;
      }
      refresh_irq_line();
      service_pending_irq();
      return;

    case 2:
      atv_pending_[1] = value;
      return;

    case 3:
      host_audio_apply_ = value;
      cdda_adp_muted_ = (value & 0x01u) != 0;
      if ((value & 0x20u) != 0) {
        atv_active_ = atv_pending_;
      }
      return;
    default:
      return;
    }
  }
}

void CdRom::tick(u32 cycles) {
  const bool profile_detailed = g_profile_detailed_timing;
  std::chrono::high_resolution_clock::time_point t0{};
  if (profile_detailed) {
    t0 = std::chrono::high_resolution_clock::now();
  }

  const int step = static_cast<int>(cycles);

  if (command_busy_) {
    command_busy_cycles_ -= step;
    if (command_busy_cycles_ <= 0) {
      command_busy_cycles_ = 0;
      command_busy_ = false;
      service_pending_irq();
    }
  }

  if (adpcm_busy_cycles_ > 0) {
    adpcm_busy_cycles_ -= step;
    if (adpcm_busy_cycles_ < 0) {
      adpcm_busy_cycles_ = 0;
    }
  }

  if (pending_second_.active) {
    pending_second_.delay -= step;
    if (pending_second_.delay <= 0) {
      pending_second_.active = false;
      enqueue_irq(pending_second_.irq, std::move(pending_second_.response), false);
      pending_second_.response.clear();
    }
  }

  if (insert_probe_active_) {
    insert_probe_delay_cycles_ -= step;
    if (insert_probe_delay_cycles_ <= 0) {
      switch (insert_probe_stage_) {
      case 0:
        execute_internal_command(0x19u, {0x20u});
        break;
      case 1:
        execute_internal_command(0x01u, {});
        break;
      case 2:
        execute_internal_command(0x01u, {});
        break;
      default:
        insert_probe_active_ = false;
        break;
      }

      if (insert_probe_active_) {
        ++insert_probe_stage_;
        insert_probe_delay_cycles_ = 4000;
        if (insert_probe_stage_ > 2) {
          insert_probe_active_ = false;
        }
      }
    }
  }

  if (state_ == State::Seeking) {
    pending_cycles_ -= step;
    if (pending_cycles_ <= 0) {
      read_lba_ = seek_target_valid_ ? seek_target_lba_ : read_lba_;
      seek_complete_ = true;
      if (pending_read_start_) {
        pending_read_start_ = false;
        state_ = State::Reading;
        pending_cycles_ = std::max(1, read_period_cycles_);
      } else {
        state_ = State::Idle;
      }
    }
  }

  if (state_ == State::Reading) {
    int remaining = step;
    while (remaining > 0 && state_ == State::Reading) {
      if (pending_cycles_ > remaining) {
        pending_cycles_ -= remaining;
        break;
      }
      remaining -= std::max(0, pending_cycles_);
      pending_cycles_ = std::max(1, read_period_cycles_);

      bool ok = cdda_playing_ ? stream_cdda_sector() : read_sector();
      if (!ok) {
        seek_error_ = true;
        state_ = State::Idle;
        cdda_playing_ = false;
        enqueue_irq(5, make_error_response(stat_byte(), 0x04u), false);
        break;
      }

      ++sector_counter_;
      if (!cdda_playing_) {
        // Only fire INT1 when a real host-data sector landed in the buffer.
        // XA ADPCM sectors are consumed silently by maybe_decode_xa_audio()
        // and leave data_ready_ false. On real hardware those sectors do NOT
        // generate INT1; firing it with an empty buffer corrupts the game's
        // sector count and causes DMA reads to return zeros.
        if (data_ready_) {
          enqueue_irq(1, {stat_byte()}, false);
        }
      } else if ((mode_ & 0x04u) != 0) {
        const int abs_lba = read_lba_ + 150;
        if ((abs_lba % 10) == 0) {
          const CdTrack *track = track_for_lba(read_lba_);
          const int track_start = (track != nullptr) ? track->index01_abs_lba : 150;
          const int rel_lba = std::max(0, abs_lba - track_start);
          u8 rel_m = 0;
          u8 rel_s = 0;
          u8 rel_f = 0;
          u8 abs_m = 0;
          u8 abs_s = 0;
          u8 abs_f = 0;
          lba_to_msf_bcd(rel_lba, rel_m, rel_s, rel_f);
          lba_to_msf_bcd(abs_lba, abs_m, abs_s, abs_f);
          const bool rel = (abs_lba % 32) >= 16;
          const u8 track_no = to_bcd(clip_u8(
              std::clamp((track != nullptr) ? track->number : 1, 1, 99)));
          const u8 ss = rel ? static_cast<u8>(rel_s | 0x80u) : abs_s;
          const u8 mm = rel ? rel_m : abs_m;
          const u8 ff = rel ? rel_f : abs_f;
          enqueue_irq(1, {stat_byte(), track_no, 0x01u, mm, ss, ff, 0x00u, 0x00u},
                      false);
        }
      }

      const CdTrack *track = track_for_lba(read_lba_);
      const int end_lba = track_end_lba(track);
      if (end_lba >= 0 && read_lba_ >= end_lba) {
        if (cdda_playing_) {
          cdda_playing_ = false;
          state_ = State::Idle;
          if ((mode_ & 0x02u) == 0) {
            motor_on_ = false;
          }
          enqueue_irq(4, {stat_byte()}, false);
        } else {
          state_ = State::Idle;
        }
        break;
      }

      ++read_lba_;
    }
  }

  service_pending_irq();

  if (profile_detailed && sys_ != nullptr) {
    const auto t1 = std::chrono::high_resolution_clock::now();
    sys_->add_cdrom_time(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
}

u32 CdRom::dma_read() {
  if (!data_ready_ || !data_request_ ||
      data_index_ >= static_cast<int>(data_buffer_.size())) {
    return 0;
  }

  u32 value = 0;
  for (int i = 0; i < 4; ++i) {
    u8 byte = 0;
    if (data_index_ < static_cast<int>(data_buffer_.size())) {
      byte = data_buffer_[static_cast<size_t>(data_index_++)];
    }
    value |= static_cast<u32>(byte) << (i * 8u);
  }

  if (data_index_ >= static_cast<int>(data_buffer_.size())) {
    data_ready_ = false;
  }
  return value;
}

