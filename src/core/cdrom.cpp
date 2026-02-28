#include "cdrom.h"
#include "system.h"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace {
u8 irq_code_to_enable_bit(u8 irq_code) {
  if (irq_code >= 1 && irq_code <= 5) {
    return static_cast<u8>(1u << (irq_code - 1));
  }
  return 0;
}

std::string trim_copy(const std::string &text) {
  const size_t begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const size_t end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string lower_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

std::string upper_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return text;
}

bool starts_with_ci(const std::string &line, const char *token) {
  const size_t n = std::strlen(token);
  if (line.size() < n) {
    return false;
  }
  for (size_t i = 0; i < n; ++i) {
    if (std::tolower(static_cast<unsigned char>(line[i])) !=
        std::tolower(static_cast<unsigned char>(token[i]))) {
      return false;
    }
  }
  return true;
}

int parse_msf(const std::string &text) {
  const std::string msf = trim_copy(text);
  int mm = 0;
  int ss = 0;
  int ff = 0;
  char c1 = '\0';
  char c2 = '\0';
  std::istringstream iss(msf);
  if (!(iss >> mm >> c1 >> ss >> c2 >> ff) || c1 != ':' || c2 != ':') {
    return -1;
  }
  return mm * 60 * 75 + ss * 75 + ff;
}

std::string parse_file_name(const std::string &line) {
  const size_t q1 = line.find('"');
  if (q1 != std::string::npos) {
    const size_t q2 = line.find('"', q1 + 1);
    if (q2 != std::string::npos && q2 > q1 + 1) {
      return line.substr(q1 + 1, q2 - q1 - 1);
    }
  }

  std::istringstream iss(line);
  std::string token;
  std::string name;
  iss >> token >> name;
  return name;
}

} // namespace

// -- Disc Loading --------------------------------------------------------------

bool CdRom::load_bin_cue(const std::string &bin_path,
                         const std::string &cue_path) {
  if (bin_file_.is_open()) {
    bin_file_.close();
  }
  tracks_.clear();
  disc_loaded_ = false;
  resolved_disc_path_.clear();
  track_map_valid_ = false;
  state_ = State::Idle;
  response_fifo_.clear();
  response_index_ = 0;
  data_buffer_.clear();
  data_index_ = 0;
  data_ready_ = false;
  data_request_ = false;
  motor_on_ = false;
  shell_open_ = false;
  seek_error_ = false;
  id_error_ = false;
  interrupt_enable_ = 0;
  interrupt_flag_ = 0;
  pending_second_ = {};
  pending_irqs_.clear();
  bin_size_ = 0;
  read_period_cycles_ = 0;
  command_busy_ = false;
  command_busy_cycles_ = 0;
  last_command_ = 0;
  last_irq_code_ = 0;
  command_counter_ = 0;
  sector_counter_ = 0;
  saw_read_command_ = false;
  saw_getid_ = false;
  saw_setloc_ = false;
  saw_seekl_ = false;
  saw_readn_or_reads_ = false;
  saw_sector_visible_ = false;
  read_command_count_ = 0;
  irq_int1_count_ = 0;
  irq_int3_count_ = 0;
  read_buffer_stall_count_ = 0;
  response_promotion_count_ = 0;
  status_e0_poll_count_ = 0;
  status_e0_streak_max_ = 0;
  status_e0_streak_current_ = 0;
  seek_target_lba_ = 0;
  seek_target_valid_ = false;
  seek_complete_ = false;
  read_whole_sector_ = false;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  host_audio_regs_.fill(0);
  host_audio_apply_ = 0;

  const std::filesystem::path cue_fs(cue_path);
  const std::filesystem::path cue_dir = cue_fs.parent_path();
  if (!parse_cue(cue_path, cue_dir.string())) {
    LOG_ERROR("CDROM: Failed to parse CUE file: %s", cue_path.c_str());
    return false;
  }

  std::filesystem::path resolved_bin_path = bin_path;
  if (resolved_bin_path.empty() && !tracks_.empty() &&
      !tracks_.front().filename.empty()) {
    resolved_bin_path = cue_dir / tracks_.front().filename;
  }

  if (resolved_bin_path.empty()) {
    LOG_ERROR("CDROM: No BIN file could be resolved from CUE");
    return false;
  }

  bin_file_.clear();
  bin_file_.open(resolved_bin_path, std::ios::binary);
  if (!bin_file_.is_open() && !tracks_.empty() &&
      !tracks_.front().filename.empty()) {
    const std::filesystem::path cue_referenced = cue_dir / tracks_.front().filename;
    if (cue_referenced != resolved_bin_path) {
      bin_file_.clear();
      bin_file_.open(cue_referenced, std::ios::binary);
      if (bin_file_.is_open()) {
        resolved_bin_path = cue_referenced;
      }
    }
  }
  if (!bin_file_.is_open()) {
    LOG_ERROR("CDROM: Failed to open BIN file: %s",
              resolved_bin_path.string().c_str());
    return false;
  }

  std::error_code ec;
  bin_size_ = std::filesystem::file_size(resolved_bin_path, ec);
  if (ec) {
    bin_size_ = 0;
  }

  bool single_file = true;
  bool monotonic = true;
  int first_index01 = 0;
  int prev_index01 = 0;
  if (!tracks_.empty()) {
    first_index01 = tracks_.front().index01_file_lba;
    prev_index01 = first_index01;
  }
  const std::string first_track_file =
      tracks_.empty() ? std::string() : lower_copy(tracks_.front().filename);
  for (CdTrack &track : tracks_) {
    if (!first_track_file.empty() &&
        lower_copy(track.filename) != first_track_file) {
      single_file = false;
    }
    if (track.index01_file_lba < prev_index01) {
      monotonic = false;
    }
    prev_index01 = track.index01_file_lba;
    track.index01_file_offset = static_cast<u64>(std::max(track.index01_file_lba, 0)) *
                                static_cast<u64>(std::max(track.sector_size, 1));
  }

  track_map_valid_ = !tracks_.empty() && single_file && monotonic;
  for (CdTrack &track : tracks_) {
    const int delta = std::max(0, track.index01_file_lba - first_index01);
    track.index01_abs_lba = 150 + delta;
  }

  disc_loaded_ = true;
  // Boot-disc flow should start from a closed-lid, inserted-disc state.
  motor_on_ = true;
  shell_open_ = false;
  seek_error_ = false;
  id_error_ = false;
  resolved_disc_path_ = resolved_bin_path.string();

  LOG_INFO("CDROM: Loaded disc - %zu track(s) from %s", tracks_.size(),
           resolved_disc_path_.c_str());
  if (!track_map_valid_) {
    LOG_WARN("CDROM: Track map is not fully deterministic (multi-file or non-monotonic CUE)");
  }
  return true;
}

void CdRom::reset() {
  index_reg_ = 0;
  interrupt_enable_ = 0;
  interrupt_flag_ = 0;
  param_fifo_.clear();
  response_fifo_.clear();
  response_index_ = 0;
  data_buffer_.clear();
  data_index_ = 0;
  data_ready_ = false;
  data_request_ = false;
  motor_on_ = disc_loaded_;
  shell_open_ = false;
  seek_error_ = false;
  id_error_ = false;
  state_ = State::Idle;
  pending_second_ = {};
  pending_irqs_.clear();
  seek_mm_ = seek_ss_ = seek_ff_ = 0;
  read_lba_ = 0;
  pending_cycles_ = 0;
  read_period_cycles_ = 0;
  command_busy_ = false;
  command_busy_cycles_ = 0;
  last_command_ = 0;
  last_irq_code_ = 0;
  mode_ = 0;
  filter_file_ = 0;
  filter_channel_ = 0;
  command_counter_ = 0;
  sector_counter_ = 0;
  saw_read_command_ = false;
  saw_getid_ = false;
  saw_setloc_ = false;
  saw_seekl_ = false;
  saw_readn_or_reads_ = false;
  saw_sector_visible_ = false;
  read_command_count_ = 0;
  irq_int1_count_ = 0;
  irq_int3_count_ = 0;
  read_buffer_stall_count_ = 0;
  response_promotion_count_ = 0;
  status_e0_poll_count_ = 0;
  status_e0_streak_max_ = 0;
  status_e0_streak_current_ = 0;
  seek_target_lba_ = 0;
  seek_target_valid_ = false;
  seek_complete_ = false;
  read_whole_sector_ = false;
  pending_read_start_ = false;
  pending_reads_mode_ = false;
  host_audio_regs_.fill(0);
  host_audio_apply_ = 0;
}

bool CdRom::parse_cue(const std::string &cue_path,
                      const std::string & /*bin_dir*/) {
  std::ifstream cue(cue_path);
  if (!cue.is_open()) {
    return false;
  }

  tracks_.clear();
  std::string current_file;
  CdTrack *current_track = nullptr;
  std::string line;

  while (std::getline(cue, line)) {
    line = trim_copy(line);
    if (line.empty() || line[0] == ';') {
      continue;
    }

    if (starts_with_ci(line, "FILE")) {
      current_file = parse_file_name(line);
      continue;
    }

    if (starts_with_ci(line, "TRACK")) {
      std::istringstream iss(line);
      std::string token;
      std::string type;
      int number = 0;
      if (!(iss >> token >> number >> type)) {
        continue;
      }

      CdTrack track{};
      track.number = number;
      track.type = upper_copy(type);
      track.filename = current_file;
      if (track.type == "MODE1/2048") {
        track.sector_size = 2048;
      } else {
        track.sector_size = 2352;
      }
      tracks_.push_back(track);
      current_track = &tracks_.back();
      continue;
    }

    if (starts_with_ci(line, "PREGAP")) {
      if (current_track == nullptr) {
        continue;
      }
      std::istringstream iss(line);
      std::string token;
      std::string msf;
      iss >> token >> msf;
      const int sectors = parse_msf(msf);
      if (sectors >= 0) {
        current_track->pregap_sectors = sectors;
      }
      continue;
    }

    if (starts_with_ci(line, "INDEX")) {
      if (current_track == nullptr) {
        continue;
      }
      std::istringstream iss(line);
      std::string token;
      std::string index_no;
      std::string msf;
      if (!(iss >> token >> index_no >> msf)) {
        continue;
      }
      if (index_no != "01") {
        continue;
      }
      const int sectors = parse_msf(msf);
      if (sectors >= 0) {
        current_track->index01_file_lba = sectors;
        current_track->index01_file_offset =
            static_cast<u64>(sectors) * static_cast<u64>(current_track->sector_size);
      }
    }
  }

  return !tracks_.empty();
}

// -- Register Access -----------------------------------------------------------

u8 CdRom::read8(u32 offset) {
  auto trace_read = [&](u8 value) {
    if (g_trace_cdrom) {
      static u64 io_read_count = 0;
      if (trace_should_log(io_read_count, g_trace_burst_cdrom, g_trace_stride_cdrom)) {
        LOG_INFO("CDROM: R8 off=%u idx=%u -> 0x%02X", offset,
                 static_cast<unsigned>(index_reg_),
                 static_cast<unsigned>(value));
      }
    }
    return value;
  };

  switch (offset) {
  case 0: {
    u8 status = static_cast<u8>(index_reg_ & 0x3);
    // Bit2 ADPBUSY is intentionally left clear in this model.
    // Bit3 PRMEMPT: parameter FIFO empty.
    if (param_fifo_.empty()) {
      status |= (1u << 3);
    }
    // Bit4 PRMWRDY: parameter FIFO can accept writes.
    if (param_fifo_.size() < 16) {
      status |= (1u << 4);
    }
    // Bit5 RSLRRDY: response FIFO has unread data.
    if (response_index_ < static_cast<int>(response_fifo_.size())) {
      status |= (1u << 5);
    }
    // Bit6 DRQSTS: data buffer readable by host when requested.
    if (data_ready_ && data_request_) {
      status |= (1u << 6);
    }
    // Bit7 BUSYSTS: command processing busy window active.
    if (command_busy_ || command_busy_cycles_ > 0) {
      status |= (1u << 7);
    }
    if (status == 0xE0u) {
      ++status_e0_poll_count_;
      ++status_e0_streak_current_;
      status_e0_streak_max_ =
          std::max(status_e0_streak_max_, status_e0_streak_current_);
    } else {
      status_e0_streak_current_ = 0;
    }
    return trace_read(status);
  }
  case 1:
    if (response_index_ < static_cast<int>(response_fifo_.size())) {
      const u8 value = response_fifo_[response_index_++];
      if (response_index_ >= static_cast<int>(response_fifo_.size())) {
        response_fifo_.clear();
        response_index_ = 0;
        service_pending_irq();
      }
      return trace_read(value);
    }
    return trace_read(0);
  case 2:
    if (data_index_ < static_cast<int>(data_buffer_.size())) {
      const u8 value = data_buffer_[data_index_++];
      if (data_index_ >= static_cast<int>(data_buffer_.size())) {
        data_ready_ = false;
      }
      return trace_read(value);
    }
    return trace_read(0);
  case 3:
    if (index_reg_ == 0 || index_reg_ == 2) {
      return trace_read(static_cast<u8>(interrupt_enable_ | 0xE0));
    }
    return trace_read(static_cast<u8>(interrupt_flag_ | 0xE0));
  default:
    LOG_WARN("CDROM: Unhandled read8 at offset %u index %u", offset, index_reg_);
    return trace_read(0);
  }
}

void CdRom::write8(u32 offset, u8 value) {
  if (g_trace_cdrom) {
    static u64 io_write_count = 0;
    if (trace_should_log(io_write_count, g_trace_burst_cdrom, g_trace_stride_cdrom)) {
      LOG_INFO("CDROM: W8 off=%u idx=%u val=0x%02X", offset,
               static_cast<unsigned>(index_reg_),
               static_cast<unsigned>(value));
    }
  }

  switch (offset) {
  case 0:
    index_reg_ = value & 0x3;
    break;
  case 1:
    switch (index_reg_) {
    case 0:
      last_command_ = value;
      command_busy_cycles_ = command_busy_for(value);
      command_busy_ = command_busy_cycles_ > 0;
      execute_command(value);
      param_fifo_.clear();
      break;
    case 1:
      host_audio_regs_[0] = value;
      break;
    case 2:
      host_audio_regs_[1] = value;
      break;
    case 3:
      host_audio_apply_ = value;
      break;
    default:
      LOG_WARN("CDROM: Write to port 1 index %u = 0x%02X", index_reg_, value);
      break;
    }
    break;
  case 2:
    switch (index_reg_) {
    case 0:
      param_fifo_.push_back(value);
      break;
    case 1:
      interrupt_enable_ = value & 0x1F;
      refresh_irq_line();
      break;
    case 2:
      host_audio_regs_[2] = value;
      break;
    case 3:
      host_audio_regs_[3] = value;
      break;
    default:
      LOG_WARN("CDROM: Write to port 2 index %u = 0x%02X", index_reg_, value);
      break;
    }
    break;
  case 3:
    switch (index_reg_) {
    case 0:
      data_request_ = (value & 0x80) != 0;
      if (data_request_) {
        data_index_ = 0;
        data_ready_ = !data_buffer_.empty();
      } else {
        data_index_ = 0;
      }
      break;
    case 1: {
      const u8 ack_mask = static_cast<u8>(value & 0x1F);
      bool irq_cleared = false;
      if (interrupt_flag_ >= 1 && interrupt_flag_ <= 5) {
        const u8 active_bit = irq_code_to_enable_bit(interrupt_flag_);
        if ((ack_mask & active_bit) != 0) {
          irq_cleared = true;
        }
      }
      if (irq_cleared) {
        // HCLRCTL bits0..4 acknowledge pending HINTSTS flags.
        // On real hardware this also drains the current result FIFO, allowing
        // queued responses/IRQs to promote.
        interrupt_flag_ = 0;
        response_fifo_.clear();
        response_index_ = 0;
      }
      if (value & 0x40) {
        // CLRPRM: clear parameter FIFO.
        param_fifo_.clear();
      }
      if (value & 0x80) {
        // CHPRST/host reset control: keep this limited to host FIFOs so active
        // read/seek state isn't disrupted mid-boot.
        response_fifo_.clear();
        response_index_ = 0;
      }
      refresh_irq_line();
      service_pending_irq();
      break;
    }
    case 2:
      host_audio_regs_[4] = value;
      break;
    case 3:
      host_audio_regs_[5] = value;
      host_audio_apply_ = value;
      break;
    default:
      LOG_WARN("CDROM: Write to port 3 index %u = 0x%02X", index_reg_, value);
      break;
    }
    break;
  }
}

// -- Commands -----------------------------------------------------------------

void CdRom::execute_command(u8 cmd) {
  ++command_counter_;
  if (g_trace_cdrom &&
      (command_counter_ <= static_cast<u64>(g_trace_burst_cdrom) ||
       (g_trace_stride_cdrom > 0 &&
        (command_counter_ % static_cast<u64>(g_trace_stride_cdrom)) == 0))) {
    std::ostringstream oss;
    oss << "CDROM: CMD#" << command_counter_ << " 0x" << std::hex
        << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(cmd) << " params=[";
    for (size_t i = 0; i < param_fifo_.size(); ++i) {
      if (i)
        oss << ' ';
      oss << "0x" << std::setw(2) << static_cast<unsigned>(param_fifo_[i]);
    }
    oss << "]";
    LOG_INFO("%s", oss.str().c_str());
  }

  switch (cmd) {
  case 0x01:
    cmd_getstat();
    break;
  case 0x02:
    saw_setloc_ = true;
    cmd_setloc();
    break;
  case 0x06:
    saw_readn_or_reads_ = true;
    cmd_readn();
    break;
  case 0x07:
    cmd_standby();
    break;
  case 0x08:
    cmd_stop();
    break;
  case 0x09:
    cmd_pause();
    break;
  case 0x0B:
    cmd_mute();
    break;
  case 0x0C:
    cmd_demute();
    break;
  case 0x0A:
    cmd_init();
    break;
  case 0x0E:
    cmd_setmode();
    break;
  case 0x0D:
    cmd_setfilter();
    break;
  case 0x10:
    cmd_setsession();
    break;
  case 0x11:
    cmd_getloc_l();
    break;
  case 0x12:
    cmd_getloc_p();
    break;
  case 0x15:
    saw_seekl_ = true;
    cmd_seekl();
    break;
  case 0x1A:
    saw_getid_ = true;
    cmd_getid();
    break;
  case 0x1B:
    saw_readn_or_reads_ = true;
    cmd_reads();
    break;
  case 0x13:
    cmd_gettn();
    break;
  case 0x14:
    cmd_gettd();
    break;
  case 0x19:
    cmd_test();
    break;
  case 0x1E:
    cmd_readtoc();
    break;
  default:
    LOG_WARN("CDROM: Unhandled command 0x%02X", cmd);
    enqueue_irq(3, {static_cast<u8>(stat_byte() | 0x01)});
    break;
  }
}

u8 CdRom::stat_byte() const {
  u8 stat = 0;
  if (motor_on_)
    stat |= 0x02;
  if (seek_error_)
    stat |= 0x04;
  if (id_error_)
    stat |= 0x08;
  if (shell_open_ || !disc_loaded_)
    stat |= 0x10;
  if (state_ == State::Reading)
    stat |= 0x20;
  if (state_ == State::Seeking)
    stat |= 0x40;
  return stat;
}

void CdRom::cmd_getstat() {
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_setloc() {
  if (param_fifo_.size() >= 3) {
    seek_mm_ = from_bcd(param_fifo_[0]);
    seek_ss_ = from_bcd(param_fifo_[1]);
    seek_ff_ = from_bcd(param_fifo_[2]);
    seek_target_lba_ = msf_to_lba(seek_mm_, seek_ss_, seek_ff_);
    seek_target_valid_ = true;
    seek_complete_ = false;
  }
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_readn() {
  start_read_stream(false);
}

void CdRom::cmd_reads() {
  start_read_stream(true);
}

void CdRom::cmd_pause() {
  state_ = State::Idle;
  read_period_cycles_ = 0;
  pending_read_start_ = false;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(25000, 2, {stat_byte()});
}

void CdRom::cmd_init() {
  state_ = State::Idle;
  read_period_cycles_ = 0;
  pending_read_start_ = false;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(50000, 2, {stat_byte()});
}

void CdRom::cmd_setmode() {
  if (!param_fifo_.empty()) {
    mode_ = param_fifo_[0];
  }
  if (state_ == State::Reading) {
    refresh_read_period();
    pending_cycles_ = std::max(1, read_period_cycles_);
  }
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_stop() {
  state_ = State::Idle;
  motor_on_ = false;
  read_period_cycles_ = 0;
  pending_read_start_ = false;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

void CdRom::cmd_standby() {
  state_ = State::Idle;
  motor_on_ = true;
  read_period_cycles_ = 0;
  pending_read_start_ = false;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(40000, 2, {stat_byte()});
}

void CdRom::cmd_mute() { enqueue_irq(3, {stat_byte()}); }

void CdRom::cmd_demute() { enqueue_irq(3, {stat_byte()}); }

void CdRom::cmd_setfilter() {
  if (param_fifo_.size() >= 2) {
    filter_file_ = param_fifo_[0];
    filter_channel_ = param_fifo_[1];
  }
  enqueue_irq(3, {stat_byte()});
}

void CdRom::cmd_setsession() {
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

void CdRom::cmd_getloc_l() {
  const int abs_lba = std::max(read_lba_, 0) + 150;
  const int mm = abs_lba / (60 * 75);
  const int ss = (abs_lba / 75) % 60;
  const int ff = abs_lba % 75;
  enqueue_irq(3, {stat_byte(), to_bcd(static_cast<u8>(mm)),
                  to_bcd(static_cast<u8>(ss)), to_bcd(static_cast<u8>(ff)),
                  0x02, 0x00, 0x00, 0x00});
}

void CdRom::cmd_getloc_p() {
  const int abs_lba = std::max(read_lba_, 0) + 150;
  const int mm = abs_lba / (60 * 75);
  const int ss = (abs_lba / 75) % 60;
  const int ff = abs_lba % 75;
  u8 track = 1;
  if (const CdTrack *t = track_for_lba(read_lba_)) {
    track = static_cast<u8>(std::max(1, t->number));
  }
  enqueue_irq(3, {stat_byte(), to_bcd(track), 0x01,
                  to_bcd(static_cast<u8>(mm)), to_bcd(static_cast<u8>(ss)),
                  to_bcd(static_cast<u8>(ff)), 0x00, 0x00});
}

void CdRom::cmd_seekl() {
  motor_on_ = true;
  if (seek_target_valid_) {
    read_lba_ = seek_target_lba_;
  } else {
    read_lba_ = msf_to_lba(seek_mm_, seek_ss_, seek_ff_);
  }
  state_ = State::Seeking;
  seek_complete_ = false;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

void CdRom::cmd_getid() {
  if (!disc_loaded_) {
    enqueue_irq(5, {0x11, 0x80});
    return;
  }
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2,
                           {stat_byte(), 0x00, 0x20, 0x00, 'S', 'C', 'E', 'A'});
  if (g_trace_cdrom) {
    LOG_INFO("CDROM: GetID queued region=SCEA type=0x20");
  }
}

void CdRom::cmd_gettn() {
  const u8 first = 1;
  const u8 last = static_cast<u8>(tracks_.size());
  enqueue_irq(3, {stat_byte(), to_bcd(first), to_bcd(last)});
}

void CdRom::cmd_gettd() {
  u8 track = 0;
  if (!param_fifo_.empty()) {
    track = from_bcd(param_fifo_[0]);
  }

  int lba = 0;
  if (track == 0) {
    if (!tracks_.empty() && bin_size_ > 0) {
      const CdTrack &t = tracks_.front();
      const int sectors =
          static_cast<int>(bin_size_ / static_cast<u64>(std::max(1, t.sector_size)));
      const int leadout_abs =
          t.index01_abs_lba + std::max(0, sectors - t.index01_file_lba);
      lba = std::max(0, leadout_abs - 150);
    }
  } else if (track <= static_cast<int>(tracks_.size())) {
    lba = std::max(0, tracks_[track - 1].index01_abs_lba - 150);
  }

  const int mm = lba / (60 * 75);
  const int ss = (lba / 75) % 60;
  enqueue_irq(3, {stat_byte(), to_bcd(static_cast<u8>(mm)),
                  to_bcd(static_cast<u8>(ss))});
}

void CdRom::cmd_test() {
  if (!param_fifo_.empty() && param_fifo_[0] == 0x20) {
    // CDROM BIOS date/version tuple used by common SCPH-100x boot flows.
    enqueue_irq(3, {0x95, 0x07, 0x24, 0xC1});
  } else {
    enqueue_irq(3, {stat_byte()});
  }
}

void CdRom::cmd_readtoc() {
  motor_on_ = true;
  state_ = State::Idle;
  enqueue_irq(3, {stat_byte()});
  schedule_second_response(33868, 2, {stat_byte()});
}

// -- Helpers ------------------------------------------------------------------

int CdRom::msf_to_lba(u8 mm, u8 ss, u8 ff) const {
  return (mm * 60 * 75) + (ss * 75) + ff - 150;
}

int CdRom::read_period_for_mode() const {
  const bool double_speed = (mode_ & 0x80) != 0;
  const int sectors_per_second = double_speed ? 150 : 75;
  return std::max(1, static_cast<int>(psx::CPU_CLOCK_HZ / sectors_per_second));
}

int CdRom::command_busy_for(u8 cmd) const {
  switch (cmd) {
  case 0x01: // Getstat
    return 1200;
  case 0x02: // Setloc
  case 0x0E: // Setmode
    return 2000;
  case 0x06: // ReadN
  case 0x1B: // ReadS
    return 2400;
  case 0x15: // SeekL
    return 10000;
  case 0x1A: // GetID
    return 4000;
  case 0x1E: // ReadTOC
    return 4000;
  case 0x0A: // Init
    return 6000;
  case 0x09: // Pause
    return 4000;
  default:
    return 2000;
  }
}

void CdRom::refresh_read_period() { read_period_cycles_ = read_period_for_mode(); }

void CdRom::schedule_second_response(int delay_cycles, u8 irq,
                                     std::vector<u8> response) {
  pending_second_.active = true;
  pending_second_.delay = std::max(1, delay_cycles);
  pending_second_.irq = irq;
  pending_second_.response = std::move(response);
}

void CdRom::start_read_stream(bool reads_mode) {
  if (!pending_read_start_) {
    saw_read_command_ = true;
    saw_readn_or_reads_ = true;
    ++read_command_count_;
  }

  // ReadN/ReadS can be issued after Setloc without an explicit SeekL command.
  // Only defer if a real SeekL operation is currently in progress.
  if (state_ == State::Seeking && !seek_complete_) {
    pending_read_start_ = true;
    pending_reads_mode_ = reads_mode;
    enqueue_irq(3, {stat_byte()});
    return;
  }

  motor_on_ = true;
  if (seek_target_valid_) {
    read_lba_ = seek_target_lba_;
  } else {
    read_lba_ = msf_to_lba(seek_mm_, seek_ss_, seek_ff_);
  }
  seek_complete_ = true;

  read_whole_sector_ = reads_mode;
  state_ = State::Reading;
  data_buffer_.clear();
  data_index_ = 0;
  data_ready_ = false;
  refresh_read_period();
  pending_cycles_ = std::max(1, read_period_cycles_);
  enqueue_irq(3, {stat_byte()});
}

bool CdRom::read_sector() {
  if (!disc_loaded_ || !bin_file_.is_open()) {
    return false;
  }

  const CdTrack *track = track_for_lba(read_lba_);
  const std::string track_type = (track != nullptr) ? track->type : "MODE2/2352";
  const int sector_size = (track != nullptr) ? track->sector_size : 2352;

  int payload_offset = 24;
  int payload_size = 2048;
  if (track_type == "MODE1/2352") {
    payload_offset = 16;
    payload_size = 2048;
  } else if (track_type == "MODE1/2048") {
    payload_offset = 0;
    payload_size = 2048;
  }

  if (sector_size == 2048) {
    payload_offset = 0;
    payload_size = 2048;
  } else if ((mode_ & 0x20) || read_whole_sector_) {
    payload_offset = 12;
    payload_size = 2340;
  }

  const int psx_lba = std::max(read_lba_, 0);
  const int abs_lba = psx_lba + 150;

  u64 raw_sector_offset =
      static_cast<u64>(psx_lba) * static_cast<u64>(std::max(sector_size, 1));
  if (track != nullptr) {
    const int track_relative_lba = std::max(0, abs_lba - track->index01_abs_lba);

    const u64 file_sector = static_cast<u64>(
        std::max(0, track->index01_file_lba + track_relative_lba));
    raw_sector_offset = file_sector * static_cast<u64>(std::max(track->sector_size, 1));
  }

  const u64 payload_offset_bytes =
      raw_sector_offset + static_cast<u64>(std::max(payload_offset, 0));
  if (bin_size_ > 0 &&
      payload_offset_bytes + static_cast<u64>(payload_size) > bin_size_) {
    data_buffer_.clear();
    data_index_ = 0;
    data_ready_ = false;
    state_ = State::Idle;
    return false;
  }

  bin_file_.clear();
  bin_file_.seekg(static_cast<std::streamoff>(payload_offset_bytes),
                  std::ios::beg);

  data_buffer_.resize(static_cast<size_t>(payload_size));
  bin_file_.read(reinterpret_cast<char *>(data_buffer_.data()), payload_size);
  if (bin_file_.gcount() != payload_size) {
    data_buffer_.clear();
    data_index_ = 0;
    data_ready_ = false;
    state_ = State::Idle;
    return false;
  }

  data_index_ = 0;
  data_ready_ = true;
  saw_sector_visible_ = true;
  ++sector_counter_;
  if (g_trace_cdrom &&
      (sector_counter_ <= static_cast<u64>(g_trace_burst_cdrom) ||
       (g_trace_stride_cdrom > 0 &&
        (sector_counter_ % static_cast<u64>(g_trace_stride_cdrom)) == 0))) {
    LOG_INFO(
        "CDROM: Sector#%llu LBA=%d abs=%d mode=0x%02X first4=%02X %02X %02X %02X",
        static_cast<unsigned long long>(sector_counter_), read_lba_, abs_lba,
        static_cast<unsigned>(mode_), static_cast<unsigned>(data_buffer_[0]),
        static_cast<unsigned>(data_buffer_[1]),
        static_cast<unsigned>(data_buffer_[2]),
        static_cast<unsigned>(data_buffer_[3]));
  }

  ++read_lba_;
  return true;
}

u32 CdRom::dma_read() {
  if (data_index_ + 3 < static_cast<int>(data_buffer_.size())) {
    u32 val = 0;
    val |= data_buffer_[data_index_++];
    val |= data_buffer_[data_index_++] << 8;
    val |= data_buffer_[data_index_++] << 16;
    val |= data_buffer_[data_index_++] << 24;
    if (data_index_ >= static_cast<int>(data_buffer_.size())) {
      data_ready_ = false;
      if (state_ != State::Reading) {
        data_request_ = false;
      }
    }
    return val;
  }
  data_ready_ = false;
  return 0;
}

void CdRom::fire_irq(u8 irq_num) {
  if (irq_num < 1 || irq_num > 5) {
    return;
  }
  // HINTSTS bits 0-2 are the interrupt type value (INT1..INT5), not one-hot.
  interrupt_flag_ = static_cast<u8>(irq_num & 0x07);
  last_irq_code_ = static_cast<u8>(irq_num & 0x07);
  if (irq_num == 1) {
    ++irq_int1_count_;
  } else if (irq_num == 3) {
    ++irq_int3_count_;
  }
  refresh_irq_line();
  if (g_trace_cdrom &&
      (command_counter_ <= static_cast<u64>(g_trace_burst_cdrom) ||
       (g_trace_stride_cdrom > 0 &&
        (command_counter_ % static_cast<u64>(g_trace_stride_cdrom)) == 0))) {
    LOG_INFO("CDROM: IRQ INT%u flag=0x%02X enable=0x%02X",
             static_cast<unsigned>(irq_num), static_cast<unsigned>(interrupt_flag_),
             static_cast<unsigned>(interrupt_enable_));
  }
}

void CdRom::enqueue_irq(u8 irq_num, std::vector<u8> response,
                        bool wait_for_command_idle) {
  const bool response_active =
      response_index_ < static_cast<int>(response_fifo_.size());
  const bool must_wait_for_busy =
      wait_for_command_idle && (command_busy_ || command_busy_cycles_ > 0);
  if (must_wait_for_busy || (interrupt_flag_ & 0x1F) != 0 || response_active) {
    pending_irqs_.push_back(
        PendingIrq{irq_num, wait_for_command_idle, std::move(response)});
    return;
  }
  response_fifo_ = std::move(response);
  response_index_ = 0;
  fire_irq(irq_num);
}

void CdRom::service_pending_irq() {
  if ((interrupt_flag_ & 0x1F) != 0) {
    return;
  }
  if (response_index_ < static_cast<int>(response_fifo_.size())) {
    return;
  }
  if (pending_irqs_.empty()) {
    return;
  }
  if (pending_irqs_.front().wait_for_command_idle &&
      (command_busy_ || command_busy_cycles_ > 0)) {
    return;
  }
  PendingIrq next = std::move(pending_irqs_.front());
  pending_irqs_.pop_front();
  response_fifo_ = std::move(next.response);
  response_index_ = 0;
  ++response_promotion_count_;
  fire_irq(next.irq);
}

void CdRom::refresh_irq_line() {
  const u8 pending_bit = irq_code_to_enable_bit(interrupt_flag_);
  if ((pending_bit & interrupt_enable_ & 0x1F) != 0 && sys_ != nullptr) {
    sys_->irq().request(Interrupt::CDROM);
  }
}

const CdTrack *CdRom::track_for_lba(int lba) const {
  if (tracks_.empty()) {
    return nullptr;
  }
  const int abs_lba = std::max(lba, 0) + 150;
  const CdTrack *result = &tracks_.front();
  for (const CdTrack &track : tracks_) {
    if (track.index01_abs_lba <= abs_lba) {
      result = &track;
    } else {
      break;
    }
  }
  return result;
}

void CdRom::tick(u32 cycles) {
  if (command_busy_cycles_ > 0) {
    command_busy_cycles_ -= static_cast<int>(cycles);
    if (command_busy_cycles_ <= 0) {
      command_busy_cycles_ = 0;
      command_busy_ = false;
    } else {
      command_busy_ = true;
    }
  }

  service_pending_irq();

  if (pending_second_.active) {
    pending_second_.delay -= static_cast<int>(cycles);
    if (pending_second_.delay <= 0 && (interrupt_flag_ & 0x1F) == 0) {
      enqueue_irq(pending_second_.irq, pending_second_.response);
      pending_second_.active = false;
      if (state_ == State::Seeking) {
        state_ = State::Idle;
        seek_complete_ = true;
        if (pending_read_start_) {
          const bool reads_mode = pending_reads_mode_;
          start_read_stream(reads_mode);
          pending_read_start_ = false;
          pending_reads_mode_ = false;
        }
      }
    }
  }

  if (state_ != State::Reading) {
    return;
  }

  pending_cycles_ -= static_cast<int>(cycles);
  if (pending_cycles_ <= 0) {
    const bool irq_backpressure =
        ((interrupt_flag_ & 0x1F) != 0) ||
        (response_index_ < static_cast<int>(response_fifo_.size())) ||
        !pending_irqs_.empty();
    if (irq_backpressure) {
      ++read_buffer_stall_count_;
      refresh_read_period();
      pending_cycles_ = std::max(1, read_period_cycles_);
      return;
    }

    if (data_ready_ && data_index_ < static_cast<int>(data_buffer_.size())) {
      ++read_buffer_stall_count_;
      refresh_read_period();
      pending_cycles_ = std::max(1, read_period_cycles_);
      return;
    }

    if (read_sector()) {
      enqueue_irq(1, {stat_byte()}, false);
    }

    refresh_read_period();
    pending_cycles_ = std::max(1, read_period_cycles_);
  }
}
