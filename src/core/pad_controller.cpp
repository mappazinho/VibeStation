#include "pad_controller.h"

namespace {
u64 g_pad_trace_counter = 0;
constexpr u8 kOpenBusByte = 0xFF;
} // namespace

void PadController::set_analog_state(u8 lx, u8 ly, u8 rx, u8 ry) {
  analog_lx_ = lx;
  analog_ly_ = ly;
  analog_rx_ = rx;
  analog_ry_ = ry;
}

void PadController::clear_command_buffers() {
  rx_buffer_.fill(0x00);
  tx_buffer_.fill(0x00);
  command_step_ = 0;
  response_length_ = 0;
}

void PadController::set_analog_mode(bool enabled) { analog_mode_ = enabled; }

u8 PadController::response_halfwords() const {
  if (config_mode_ || analog_mode_) {
    return 0x03;
  }
  return 0x01;
}

u8 PadController::mode_id() const {
  if (config_mode_) {
    return 0x0F;
  }
  if (analog_mode_) {
    return 0x07;
  }
  return 0x04;
}

u8 PadController::id_byte() const {
  return static_cast<u8>((mode_id() << 4) | response_halfwords());
}

void PadController::poll_controller() {
  tx_buffer_[0] = id_byte();
  tx_buffer_[1] = status_byte_;
  tx_buffer_[2] = static_cast<u8>(button_state_ & 0xFFu);
  tx_buffer_[3] = static_cast<u8>((button_state_ >> 8) & 0xFFu);

  if (analog_mode_ || config_mode_) {
    tx_buffer_[4] = analog_rx_;
    tx_buffer_[5] = analog_ry_;
    tx_buffer_[6] = analog_lx_;
    tx_buffer_[7] = analog_ly_;
  } else {
    tx_buffer_[4] = 0x00;
    tx_buffer_[5] = 0x00;
    tx_buffer_[6] = 0x00;
    tx_buffer_[7] = 0x00;
  }
}

void PadController::reset_transfer_state() {
  command_ = Command::Idle;
  clear_command_buffers();
}

void PadController::reset() {
  button_state_ = 0xFFFF;
  analog_lx_ = 0x80;
  analog_ly_ = 0x80;
  analog_rx_ = 0x80;
  analog_ry_ = 0x80;
  analog_mode_ = false;
  config_mode_ = false;
  analog_locked_ = false;
  dualshock_enabled_ = false;
  status_byte_ = 0x5A;
  rumble_config_.fill(0xFF);
  reset_transfer_state();
}

PadController::TransferResult PadController::transfer(u8 data_in) {
  TransferResult result{};
  if (command_step_ < rx_buffer_.size()) {
    rx_buffer_[command_step_] = data_in;
  }

  switch (command_) {
  case Command::Idle: {
    result.data_out = kOpenBusByte;
    if (data_in == 0x01) {
      command_ = Command::Ready;
      result.ack = true;
    } else if (data_in != kOpenBusByte) {
      result.invalid_sequences = 1;
    }
    if (g_trace_sio &&
        trace_should_log(g_pad_trace_counter, g_trace_burst_sio,
                         g_trace_stride_sio)) {
      LOG_DEBUG("SIO: PAD idle in=0x%02X out=0x%02X ack=%d", data_in,
                result.data_out, result.ack ? 1 : 0);
    }
    return result;
  }

  case Command::Ready: {
    command_step_ = 0;
    response_length_ = static_cast<u8>((response_halfwords() + 1u) * 2u);

    if (data_in == 0x42) {
      command_ = Command::ReadPad;
      poll_controller();
      result.saw_id = true;
      result.saw_cmd42 = true;
    } else if (data_in == 0x43) {
      command_ = Command::ConfigModeSetMode;
      if (!config_mode_) {
        poll_controller();
      } else {
        tx_buffer_[0] = id_byte();
        tx_buffer_[1] = status_byte_;
        tx_buffer_[2] = 0x00;
        tx_buffer_[3] = 0x00;
        tx_buffer_[4] = 0x00;
        tx_buffer_[5] = 0x00;
        tx_buffer_[6] = 0x00;
        tx_buffer_[7] = 0x00;
      }
      result.saw_id = true;
    } else if (config_mode_ && data_in == 0x44) {
      command_ = Command::SetAnalogMode;
      tx_buffer_[0] = id_byte();
      tx_buffer_[1] = status_byte_;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x00;
      tx_buffer_[5] = 0x00;
      tx_buffer_[6] = 0x00;
      tx_buffer_[7] = 0x00;
      rumble_config_.fill(0xFF);
      result.saw_id = true;
    } else if (config_mode_ && data_in == 0x45) {
      command_ = Command::GetAnalogMode;
      tx_buffer_[0] = id_byte();
      tx_buffer_[1] = status_byte_;
      tx_buffer_[2] = 0x01;
      tx_buffer_[3] = 0x02;
      tx_buffer_[4] = analog_mode_ ? static_cast<u8>(0x01) : static_cast<u8>(0x00);
      tx_buffer_[5] = 0x02;
      tx_buffer_[6] = 0x01;
      tx_buffer_[7] = 0x00;
      result.saw_id = true;
    } else if (config_mode_ && data_in == 0x46) {
      command_ = Command::Command46;
      tx_buffer_[0] = id_byte();
      tx_buffer_[1] = status_byte_;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x00;
      tx_buffer_[5] = 0x00;
      tx_buffer_[6] = 0x00;
      tx_buffer_[7] = 0x00;
      result.saw_id = true;
    } else if (config_mode_ && data_in == 0x47) {
      command_ = Command::Command47;
      tx_buffer_[0] = id_byte();
      tx_buffer_[1] = status_byte_;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x02;
      tx_buffer_[5] = 0x00;
      tx_buffer_[6] = 0x01;
      tx_buffer_[7] = 0x00;
      result.saw_id = true;
    } else if (config_mode_ && data_in == 0x4C) {
      command_ = Command::Command4C;
      tx_buffer_[0] = id_byte();
      tx_buffer_[1] = status_byte_;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x00;
      tx_buffer_[5] = 0x00;
      tx_buffer_[6] = 0x00;
      tx_buffer_[7] = 0x00;
      result.saw_id = true;
    } else if (config_mode_ && data_in == 0x4D) {
      command_ = Command::GetSetRumble;
      tx_buffer_[0] = id_byte();
      tx_buffer_[1] = status_byte_;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x00;
      tx_buffer_[5] = 0x00;
      tx_buffer_[6] = 0x00;
      tx_buffer_[7] = 0x00;
      result.saw_id = true;
    } else {
      result.data_out = kOpenBusByte;
      if (data_in != kOpenBusByte) {
        result.invalid_sequences = 1;
      }
      if (g_trace_sio &&
          trace_should_log(g_pad_trace_counter, g_trace_burst_sio,
                           g_trace_stride_sio)) {
        LOG_DEBUG("SIO: PAD unhandled cmd in=0x%02X cfg=%d", data_in,
                  config_mode_ ? 1 : 0);
      }
      return result;
    }
    break;
  }

  case Command::ReadPad:
    break;

  case Command::ConfigModeSetMode:
    if (command_step_ == static_cast<u8>(response_length_ - 1u)) {
      config_mode_ = (rx_buffer_[2] == 0x01);
      if (config_mode_) {
        dualshock_enabled_ = true;
        status_byte_ = 0x5A;
      }
    }
    break;

  case Command::SetAnalogMode:
    if (command_step_ == 2) {
      const u8 mode = rx_buffer_[2];
      if (mode == 0x00 || mode == 0x01) {
        set_analog_mode(mode == 0x01);
      }
    } else if (command_step_ == 3) {
      const u8 lock = rx_buffer_[3];
      if (lock == 0x02 || lock == 0x03) {
        analog_locked_ = (lock == 0x03);
      }
    }
    break;

  case Command::GetAnalogMode:
    break;

  case Command::Command46:
    if (command_step_ == 2) {
      if (data_in == 0x00) {
        tx_buffer_[4] = 0x01;
        tx_buffer_[5] = 0x02;
        tx_buffer_[6] = 0x00;
        tx_buffer_[7] = 0x0A;
      } else if (data_in == 0x01) {
        tx_buffer_[4] = 0x01;
        tx_buffer_[5] = 0x01;
        tx_buffer_[6] = 0x01;
        tx_buffer_[7] = 0x14;
      }
    }
    break;

  case Command::Command47:
    if (command_step_ == 2 && data_in != 0x00) {
      tx_buffer_[4] = 0x00;
      tx_buffer_[5] = 0x00;
      tx_buffer_[6] = 0x00;
      tx_buffer_[7] = 0x00;
    }
    break;

  case Command::Command4C:
    if (command_step_ == 2) {
      if (data_in == 0x00) {
        tx_buffer_[5] = 0x04;
      } else if (data_in == 0x01) {
        tx_buffer_[5] = 0x07;
      }
    }
    break;

  case Command::GetSetRumble:
    if (command_step_ >= 2 && command_step_ < 7) {
      const u8 index = static_cast<u8>(command_step_ - 2u);
      tx_buffer_[command_step_] = rumble_config_[index];
      rumble_config_[index] = data_in;
    }
    break;
  }

  if (response_length_ == 0 || command_step_ >= tx_buffer_.size()) {
    result.data_out = kOpenBusByte;
    result.invalid_sequences = 1;
    command_ = Command::Idle;
    clear_command_buffers();
    return result;
  }

  if (command_ == Command::ReadPad &&
      (command_step_ == 2 || command_step_ == 3) &&
      tx_buffer_[command_step_] != kOpenBusByte) {
    result.non_ff_button = true;
  }

  result.data_out = tx_buffer_[command_step_];
  command_step_ = static_cast<u8>((command_step_ + 1u) % response_length_);
  result.ack = (command_step_ != 0);

  if (!result.ack) {
    if (command_ == Command::ReadPad) {
      result.poll_complete = true;
      result.polled_buttons = button_state_;
    }
    command_ = Command::Idle;
    clear_command_buffers();
  }

  if (g_trace_sio &&
      trace_should_log(g_pad_trace_counter, g_trace_burst_sio,
                       g_trace_stride_sio)) {
    LOG_DEBUG(
        "SIO: PAD xfer in=0x%02X out=0x%02X ack=%d cmd=%d step=%u len=%u cfg=%d "
        "ana=%d",
        data_in, result.data_out, result.ack ? 1 : 0, static_cast<int>(command_),
        static_cast<unsigned>(command_step_), static_cast<unsigned>(response_length_),
        config_mode_ ? 1 : 0, analog_mode_ ? 1 : 0);
  }

  return result;
}
