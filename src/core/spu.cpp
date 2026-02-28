#include "spu.h"
#include <algorithm>
#include <cmath>

namespace {
u64 g_spu_trace_reg_counter = 0;
u64 g_spu_trace_dma_write_counter = 0;
u64 g_spu_trace_dma_read_counter = 0;
constexpr s32 kEnvMax = 0x7FFF;

// PSX Gaussian interpolation table from PSX-SPX.
constexpr std::array<s16, 512> kGaussTable = {
#include "spu_gauss_table.inc"
};

inline s32 adsr_base_step(u8 step, bool decreasing) {
  s32 v = 7 - static_cast<s32>(step & 0x3);
  if (decreasing) {
    v = ~v; // +7..+4 -> -8..-5
  }
  return v;
}

inline s32 clamp_env_level(s32 v) {
  if (v < 0) {
    return 0;
  }
  if (v > kEnvMax) {
    return kEnvMax;
  }
  return v;
}

inline float q15_to_float(s16 value) {
  return static_cast<float>(value) / 32768.0f;
}

inline s16 decode_fixed_volume_q15(u16 raw) {
  // Fixed volume uses bits14..0 as signed "volume/2" in range -0x4000..+0x3FFF.
  s32 v = static_cast<s32>(raw & 0x7FFFu);
  if (v & 0x4000) {
    v -= 0x8000;
  }
  v *= 2;
  if (v < -32768) {
    return -32768;
  }
  if (v > 32767) {
    return 32767;
  }
  return static_cast<s16>(v);
}
} // namespace

void Spu::init(System *sys) {
  sys_ = sys;

  if (audio_device_ != 0) {
    return;
  }
  if ((SDL_WasInit(SDL_INIT_AUDIO) & SDL_INIT_AUDIO) == 0) {
    return;
  }

  SDL_AudioSpec desired{};
  desired.freq = SAMPLE_RATE;
  desired.format = AUDIO_S16SYS;
  desired.channels = 2;
  desired.samples = 1024;
  desired.callback = nullptr;

  SDL_AudioSpec obtained{};
  audio_device_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, 0);
  if (audio_device_ == 0) {
    LOG_WARN("SPU: Failed to open audio device: %s", SDL_GetError());
    audio_enabled_ = false;
    return;
  }
  if (obtained.format != AUDIO_S16SYS || obtained.channels != 2) {
    LOG_WARN("SPU: Unsupported audio format; disabling SPU audio");
    SDL_CloseAudioDevice(audio_device_);
    audio_device_ = 0;
    audio_enabled_ = false;
    return;
  }

  audio_enabled_ = true;
  SDL_PauseAudioDevice(audio_device_, 0);
  LOG_INFO("SPU: Audio device initialized at %d Hz", obtained.freq);
}

void Spu::shutdown() {
  if (audio_device_ != 0) {
    SDL_CloseAudioDevice(audio_device_);
    audio_device_ = 0;
  }
  audio_enabled_ = false;
}

void Spu::clear_audio_capture() { capture_samples_.clear(); }

void Spu::reset() {
  if (audio_device_ != 0) {
    SDL_ClearQueuedAudio(audio_device_);
  }

  sample_accum_ = 0.0;
  for (int v = 0; v < NUM_VOICES; ++v) {
    voices_[v] = VoiceState{};
  }

  regs_.fill(0);
  spucnt_ = 0;
  spustat_ = 0;
  transfer_addr_ = 0;
  transfer_busy_cycles_ = 0;
  capture_half_ = 0;
  endx_mask_ = 0;
  reverb_on_mask_ = 0;
  master_vol_l_ = 0x3FFF;
  master_vol_r_ = 0x3FFF;
  reverb_depth_l_ = 0;
  reverb_depth_r_ = 0;
  cd_vol_l_ = 0;
  cd_vol_r_ = 0;
  ext_vol_l_ = 0;
  ext_vol_r_ = 0;
  reverb_base_addr_ = 0;
  reverb_cursor_ = 0;
  reverb_half_rate_phase_ = false;
  reverb_hold_l_ = 0.0f;
  reverb_hold_r_ = 0.0f;
  reverb_regs_ = ReverbRegs{};
  audio_diag_ = AudioDiag{};
  host_staging_samples_.clear();
  capture_samples_.clear();
  spu_ram_.fill(0);
}

u16 Spu::read16(u32 offset) const {
  switch (offset) {
  case 0x19C:
    return static_cast<u16>(endx_mask_ & 0xFFFFu);
  case 0x19E:
    return static_cast<u16>((endx_mask_ >> 16) & 0x00FFu);
  case 0x1AA:
    return spucnt_;
  case 0x1AE: {
    // PSX-SPX SPUSTAT:
    //  5-0 current SPU mode (mirrors SPUCNT bits 5-0)
    //  6   IRQ9 flag (not fully modeled here)
    //  7   transfer DMA R/W request
    //  8   transfer DMA write request
    //  9   transfer DMA read request
    // 10   transfer busy
    // 11   capture half toggle
    u16 stat = spucnt_ & 0x3F;
    const u16 transfer_mode = static_cast<u16>((spucnt_ >> 4) & 0x3);
    stat |= ((spucnt_ >> 5) & 1) << 7;
    stat |= static_cast<u16>(transfer_mode == 2 ? 1 : 0) << 8;
    stat |= static_cast<u16>(transfer_mode == 3 ? 1 : 0) << 9;
    stat |= static_cast<u16>(transfer_busy_cycles_ > 0 ? 1 : 0) << 10;
    stat |= static_cast<u16>(capture_half_ & 1) << 11;
    return stat;
  }
  default:
    if (offset < 0x200) {
      return regs_[offset / 2];
    }
    LOG_WARN("SPU: Unhandled read16 at offset 0x%X", offset);
    return 0;
  }
}

void Spu::write16(u32 offset, u16 value) {
  if (g_trace_spu) {
    if ((offset >= 0x180 ||
         (offset < 0x180 && (offset % VOICE_REG_STRIDE) == 0)) &&
        trace_should_log(g_spu_trace_reg_counter, g_trace_burst_spu,
                         g_trace_stride_spu)) {
      LOG_DEBUG("SPU: W16 off=0x%03X val=0x%04X", offset, value);
    }
  }

  if (offset < 0x400) {
    regs_[offset / 2] = value;
  }

  auto key_on_voice = [&](int v) {
    if (v < 0 || v >= NUM_VOICES) {
      return;
    }
    VoiceState &vs = voices_[v];
    vs.key_on = true;
    vs.hist1 = 0;
    vs.hist2 = 0;
    vs.gauss_hist = {};
    vs.gauss_ready = false;
    vs.sample_index = 28;
    vs.pitch_counter = 0;
    vs.adsr_counter = 0;
    vs.addr = static_cast<u32>(regs_[(v * VOICE_REG_STRIDE + 0x6) / 2]) * 8u;
    vs.repeat_addr =
        static_cast<u32>(regs_[(v * VOICE_REG_STRIDE + 0xE) / 2]) * 8u;

    // ADSR registers and Attack start.
    const u16 adsr_lo = regs_[(v * VOICE_REG_STRIDE + 0x8) / 2];
    const u16 adsr_hi = regs_[(v * VOICE_REG_STRIDE + 0xA) / 2];
    vs.attack_exp = (adsr_lo & 0x8000u) != 0;
    vs.attack_shift = static_cast<u8>((adsr_lo >> 10) & 0x1F);
    vs.attack_step = static_cast<u8>((adsr_lo >> 8) & 0x3);
    vs.decay_shift = static_cast<u8>((adsr_lo >> 4) & 0xF);
    const int sustain_raw = adsr_lo & 0xF;
    vs.sustain_level =
        static_cast<u16>(std::min<s32>((sustain_raw + 1) * 0x800, kEnvMax));
    vs.sustain_exp = (adsr_hi & 0x8000u) != 0;
    vs.sustain_decrease = (adsr_hi & 0x4000u) != 0;
    vs.sustain_shift = static_cast<u8>((adsr_hi >> 8) & 0x3F);
    vs.sustain_step = static_cast<u8>((adsr_hi >> 6) & 0x3);
    vs.release_exp = (adsr_hi & 0x20u) != 0;
    vs.release_shift = static_cast<u8>(adsr_hi & 0x1F);

    vs.phase = VoiceState::AdsrPhase::Attack;
    vs.env_level = 0;
    endx_mask_ &= ~(1u << v);
  };

  auto key_off_voice = [&](int v) {
    if (v < 0 || v >= NUM_VOICES) {
      return;
    }
    VoiceState &vs = voices_[v];
    if (vs.phase != VoiceState::AdsrPhase::Off) {
      vs.phase = VoiceState::AdsrPhase::Release;
      vs.adsr_counter = 0;
    }
  };

  switch (offset) {
  case 0x180: // Main volume left
    master_vol_l_ = decode_fixed_volume_q15(value);
    break;
  case 0x182: // Main volume right
    master_vol_r_ = decode_fixed_volume_q15(value);
    break;
  case 0x184: // Reverb depth left
    reverb_depth_l_ = decode_fixed_volume_q15(value);
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x186: // Reverb depth right
    reverb_depth_r_ = decode_fixed_volume_q15(value);
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x188: // Key On low (voices 0-15)
    for (int i = 0; i < 16; ++i) {
      if (value & (1u << i)) {
        key_on_voice(i);
      }
    }
    break;
  case 0x18A: // Key On high (voices 16-23)
    for (int i = 0; i < 8; ++i) {
      if (value & (1u << i)) {
        key_on_voice(16 + i);
      }
    }
    break;
  case 0x18C: // Key Off low
    for (int i = 0; i < 16; ++i) {
      if (value & (1u << i)) {
        key_off_voice(i);
      }
    }
    break;
  case 0x18E: // Key Off high
    for (int i = 0; i < 8; ++i) {
      if (value & (1u << i)) {
        key_off_voice(16 + i);
      }
    }
    break;
  case 0x198: // Reverb on low
    reverb_on_mask_ = (reverb_on_mask_ & 0xFF0000u) | value;
    break;
  case 0x19A: // Reverb on high
    reverb_on_mask_ = (reverb_on_mask_ & 0x00FFFFu) |
                      (static_cast<u32>(value & 0x00FF) << 16);
    break;
  case 0x1A2: // Reverb work area start
    reverb_base_addr_ = (static_cast<u32>(value) * 8u) & SPU_RAM_WORD_MASK;
    reverb_cursor_ = 0;
    audio_diag_.saw_reverb_config_write = true;
    break;
  case 0x1A6: // Transfer address
    transfer_addr_ = static_cast<u32>(value) * 8u;
    transfer_busy_cycles_ = 0;
    break;
  case 0x1A8: // Data FIFO
    if (transfer_addr_ + 1 < spu_ram_.size()) {
      spu_ram_[transfer_addr_] = static_cast<u8>(value & 0xFF);
      spu_ram_[transfer_addr_ + 1] = static_cast<u8>(value >> 8);
      transfer_addr_ += 2;
      transfer_busy_cycles_ = 64;
    }
    break;
  case 0x1AA: // SPU control
    spucnt_ = value;
    spustat_ = (spustat_ & ~0x3F) | (value & 0x3F);
    break;
  case 0x1B0: // CD input volume left
    cd_vol_l_ = static_cast<s16>(value);
    break;
  case 0x1B2: // CD input volume right
    cd_vol_r_ = static_cast<s16>(value);
    break;
  case 0x1B4: // External input volume left
    ext_vol_l_ = static_cast<s16>(value);
    break;
  case 0x1B6: // External input volume right
    ext_vol_r_ = static_cast<s16>(value);
    break;
  default:
    if (offset >= 0x1C0 && offset <= 0x1FE && (offset & 1u) == 0) {
      write_reverb_reg(offset, value);
    }
    break;
  }
}

void Spu::dma_write(u32 value) {
  if (g_trace_spu && trace_should_log(g_spu_trace_dma_write_counter,
                                      g_trace_burst_spu, g_trace_stride_spu)) {
    LOG_DEBUG("SPU: DMA write addr=0x%05X val=0x%08X", transfer_addr_, value);
  }
  if (transfer_addr_ + 3 >= spu_ram_.size()) {
    return;
  }
  spu_ram_[transfer_addr_] = static_cast<u8>(value & 0xFF);
  spu_ram_[transfer_addr_ + 1] = static_cast<u8>((value >> 8) & 0xFF);
  spu_ram_[transfer_addr_ + 2] = static_cast<u8>((value >> 16) & 0xFF);
  spu_ram_[transfer_addr_ + 3] = static_cast<u8>((value >> 24) & 0xFF);
  transfer_addr_ += 4;
  transfer_busy_cycles_ = 64;
}

u32 Spu::dma_read() {
  if (g_trace_spu && trace_should_log(g_spu_trace_dma_read_counter,
                                      g_trace_burst_spu, g_trace_stride_spu)) {
    LOG_DEBUG("SPU: DMA read addr=0x%05X", transfer_addr_);
  }
  if (transfer_addr_ + 3 >= spu_ram_.size()) {
    return 0;
  }
  u32 value = 0;
  value |= static_cast<u32>(spu_ram_[transfer_addr_]);
  value |= static_cast<u32>(spu_ram_[transfer_addr_ + 1]) << 8;
  value |= static_cast<u32>(spu_ram_[transfer_addr_ + 2]) << 16;
  value |= static_cast<u32>(spu_ram_[transfer_addr_ + 3]) << 24;
  transfer_addr_ += 4;
  transfer_busy_cycles_ = 64;
  return value;
}

float Spu::decode_volume(u16 raw) const {
  if ((raw & 0x8000u) == 0) {
    return static_cast<float>(decode_fixed_volume_q15(raw)) / 32768.0f;
  }

  // Sweep mode decode path (register-bit driven approximation of hardware
  // sweep envelope timing and exponent handling).
  const s16 base = static_cast<s16>((raw & 0x7FFFu) << 1);
  const s32 stepped = decode_sweep_step(raw, base);
  return static_cast<float>(sat16(stepped)) / 32767.0f;
}

s32 Spu::decode_sweep_step(u16 raw, s16 current) const {
  const bool exponential = (raw & 0x4000u) != 0;
  const bool decreasing = (raw & 0x2000u) != 0;
  const u8 shift = static_cast<u8>((raw >> 2) & 0x1F);
  const u8 step = static_cast<u8>(raw & 0x3);
  s32 delta = adsr_base_step(step, decreasing);
  delta <<= std::max(0, 11 - static_cast<int>(shift));
  if (exponential && !decreasing && current > 0x6000) {
    delta >>= 2;
  }
  if (exponential && decreasing) {
    delta = (delta * static_cast<s32>(current)) / 0x8000;
  }
  return static_cast<s32>(current) + delta;
}

bool Spu::decode_block(int voice) {
  static constexpr int kFilterA[5] = {0, 60, 115, 98, 122};
  static constexpr int kFilterB[5] = {0, 0, -52, -55, -60};

  VoiceState &vs = voices_[voice];
  if (vs.addr + 16 > spu_ram_.size()) {
    vs.key_on = false;
    vs.phase = VoiceState::AdsrPhase::Off;
    return false;
  }

  const u8 predict_shift = spu_ram_[vs.addr];
  const u8 flags = spu_ram_[vs.addr + 1];
  const int shift = predict_shift & 0x0F;
  const int filter = (predict_shift >> 4) & 0x07;
  const int f = (filter <= 4) ? filter : 0;

  for (int i = 0; i < 28; ++i) {
    const u8 packed = spu_ram_[vs.addr + 2 + (i >> 1)];
    int nibble = (i & 1) ? (packed >> 4) : (packed & 0x0F);
    if (nibble & 0x8) {
      nibble -= 16;
    }
    int sample = (nibble << 12);
    sample >>= shift;
    sample += (kFilterA[f] * vs.hist1 + kFilterB[f] * vs.hist2 + 32) >> 6;
    sample = std::clamp(sample, -32768, 32767);

    vs.hist2 = vs.hist1;
    vs.hist1 = static_cast<s16>(sample);
    vs.decoded[i] = static_cast<s16>(sample);
  }

  vs.sample_index = 0;
  vs.addr += 16;

  // ADPCM flags: bit2 loop start, bit1 loop repeat, bit0 end.
  if (flags & 0x04) {
    vs.repeat_addr = vs.addr - 16;
  }
  if (flags & 0x01) {
    endx_mask_ |= (1u << voice);
    vs.addr = vs.repeat_addr;
    if ((flags & 0x02) == 0) {
      vs.phase = VoiceState::AdsrPhase::Release;
      vs.env_level = 0;
    }
  }

  return true;
}

bool Spu::fetch_decoded_sample(int voice, s16 &sample) {
  VoiceState &vs = voices_[voice];
  if (vs.phase == VoiceState::AdsrPhase::Off) {
    sample = 0;
    return false;
  }
  if (vs.sample_index >= 28 && !decode_block(voice)) {
    sample = 0;
    return false;
  }
  if (vs.sample_index >= 28) {
    sample = 0;
    return false;
  }
  sample = vs.decoded[vs.sample_index++];
  return true;
}

void Spu::prime_gaussian_history(int voice) {
  VoiceState &vs = voices_[voice];
  if (vs.gauss_ready) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    s16 s = 0;
    fetch_decoded_sample(voice, s);
    vs.gauss_hist[static_cast<size_t>(i)] = s;
  }
  vs.gauss_ready = true;
}

void Spu::advance_gaussian_history(int voice) {
  VoiceState &vs = voices_[voice];
  s16 next = 0;
  fetch_decoded_sample(voice, next);
  vs.gauss_hist[0] = vs.gauss_hist[1];
  vs.gauss_hist[1] = vs.gauss_hist[2];
  vs.gauss_hist[2] = vs.gauss_hist[3];
  vs.gauss_hist[3] = next;
}

s16 Spu::gaussian_interpolate(const VoiceState &vs) const {
  if (!vs.gauss_ready) {
    return 0;
  }
  const u32 n = (vs.pitch_counter >> 4) & 0xFFu;
  const s32 g0 = kGaussTable[0x0FFu - n];
  const s32 g1 = kGaussTable[0x1FFu - n];
  const s32 g2 = kGaussTable[0x100u + n];
  const s32 g3 = kGaussTable[0x000u + n];
  const s32 s0 = static_cast<s32>(vs.gauss_hist[0]);
  const s32 s1 = static_cast<s32>(vs.gauss_hist[1]);
  const s32 s2 = static_cast<s32>(vs.gauss_hist[2]);
  const s32 s3 = static_cast<s32>(vs.gauss_hist[3]);
  const s32 out = (s0 * g0 + s1 * g1 + s2 * g2 + s3 * g3 + 0x4000) >> 15;
  return sat16(out);
}

s16 Spu::apply_envelope(s16 sample, const VoiceState &vs) const {
  const s32 env = static_cast<s32>(vs.env_level);
  const s32 mixed = (static_cast<s32>(sample) * env) / kEnvMax;
  return sat16(mixed);
}

void Spu::write_reverb_reg(u32 offset, u16 value) {
  audio_diag_.saw_reverb_config_write = true;
  switch (offset) {
  case 0x1C0:
    reverb_regs_.dAPF1 = value;
    break;
  case 0x1C2:
    reverb_regs_.dAPF2 = value;
    break;
  case 0x1C4:
    reverb_regs_.vIIR = static_cast<s16>(value);
    break;
  case 0x1C6:
    reverb_regs_.vCOMB1 = static_cast<s16>(value);
    break;
  case 0x1C8:
    reverb_regs_.vCOMB2 = static_cast<s16>(value);
    break;
  case 0x1CA:
    reverb_regs_.vCOMB3 = static_cast<s16>(value);
    break;
  case 0x1CC:
    reverb_regs_.vCOMB4 = static_cast<s16>(value);
    break;
  case 0x1CE:
    reverb_regs_.vWALL = static_cast<s16>(value);
    break;
  case 0x1D0:
    reverb_regs_.vAPF1 = static_cast<s16>(value);
    break;
  case 0x1D2:
    reverb_regs_.vAPF2 = static_cast<s16>(value);
    break;
  case 0x1D4:
    reverb_regs_.mLSAME = static_cast<s16>(value);
    break;
  case 0x1D6:
    reverb_regs_.mRSAME = static_cast<s16>(value);
    break;
  case 0x1D8:
    reverb_regs_.mLCOMB1 = static_cast<s16>(value);
    break;
  case 0x1DA:
    reverb_regs_.mRCOMB1 = static_cast<s16>(value);
    break;
  case 0x1DC:
    reverb_regs_.mLCOMB2 = static_cast<s16>(value);
    break;
  case 0x1DE:
    reverb_regs_.mRCOMB2 = static_cast<s16>(value);
    break;
  case 0x1E0:
    reverb_regs_.dLSAME = static_cast<s16>(value);
    break;
  case 0x1E2:
    reverb_regs_.dRSAME = static_cast<s16>(value);
    break;
  case 0x1E4:
    reverb_regs_.mLDIFF = static_cast<s16>(value);
    break;
  case 0x1E6:
    reverb_regs_.mRDIFF = static_cast<s16>(value);
    break;
  case 0x1E8:
    reverb_regs_.mLCOMB3 = static_cast<s16>(value);
    break;
  case 0x1EA:
    reverb_regs_.mRCOMB3 = static_cast<s16>(value);
    break;
  case 0x1EC:
    reverb_regs_.mLCOMB4 = static_cast<s16>(value);
    break;
  case 0x1EE:
    reverb_regs_.mRCOMB4 = static_cast<s16>(value);
    break;
  case 0x1F0:
    reverb_regs_.dLDIFF = static_cast<s16>(value);
    break;
  case 0x1F2:
    reverb_regs_.dRDIFF = static_cast<s16>(value);
    break;
  case 0x1F4:
    reverb_regs_.mLAPF1 = static_cast<s16>(value);
    break;
  case 0x1F6:
    reverb_regs_.mRAPF1 = static_cast<s16>(value);
    break;
  case 0x1F8:
    reverb_regs_.mLAPF2 = static_cast<s16>(value);
    break;
  case 0x1FA:
    reverb_regs_.mRAPF2 = static_cast<s16>(value);
    break;
  case 0x1FC:
    reverb_regs_.vLIN = static_cast<s16>(value);
    break;
  case 0x1FE:
    reverb_regs_.vRIN = static_cast<s16>(value);
    break;
  default:
    break;
  }
}

u32 Spu::reverb_work_area_span() const {
  const u32 base = reverb_base_addr_ & SPU_RAM_WORD_MASK;
  return (SPU_RAM_WORD_MASK + 2u) - base;
}

u32 Spu::reverb_wrap_addr(s32 addr) const {
  const s32 base = static_cast<s32>(reverb_base_addr_ & SPU_RAM_WORD_MASK);
  const s32 span = static_cast<s32>(reverb_work_area_span());
  if (span <= 0) {
    return static_cast<u32>(addr) & SPU_RAM_WORD_MASK;
  }
  s32 wrapped = addr;
  while (wrapped < base) {
    wrapped += span;
  }
  while (wrapped >= base + span) {
    wrapped -= span;
  }
  return static_cast<u32>(wrapped) & SPU_RAM_WORD_MASK;
}

u32 Spu::reverb_addr_from_reg(s16 reg, s32 extra) const {
  const s32 base = static_cast<s32>(reverb_base_addr_ & SPU_RAM_WORD_MASK);
  const s32 cursor = static_cast<s32>(reverb_cursor_);
  const s32 offset = static_cast<s32>(reg) * 8 + extra;
  return reverb_wrap_addr(base + cursor + offset);
}

s16 Spu::reverb_read_s16(u32 addr) const {
  const u32 a = addr & SPU_RAM_WORD_MASK;
  return static_cast<s16>(
      static_cast<u16>(spu_ram_[a]) |
      (static_cast<u16>(spu_ram_[(a + 1) & SPU_RAM_MASK]) << 8));
}

void Spu::reverb_write_s16(u32 addr, s16 value) {
  const u32 a = addr & SPU_RAM_WORD_MASK;
  spu_ram_[a] = static_cast<u8>(value & 0xFF);
  spu_ram_[(a + 1) & SPU_RAM_MASK] = static_cast<u8>((value >> 8) & 0xFF);
  ++audio_diag_.reverb_ram_writes;
}

s16 Spu::sat16(s32 value) {
  if (value < -32768) {
    return -32768;
  }
  if (value > 32767) {
    return 32767;
  }
  return static_cast<s16>(value);
}

s32 Spu::mul_q15(s32 a, s32 b) {
  const s64 p = static_cast<s64>(a) * static_cast<s64>(b);
  if (p >= 0) {
    return static_cast<s32>((p + 0x4000) >> 15);
  }
  return static_cast<s32>((p - 0x4000) >> 15);
}

float Spu::soft_clip(float value) {
  const float a = std::abs(value);
  return value / (1.0f + 0.5f * a);
}

static s16 clamp_fb_coef(s16 value, s16 limit = 0x6000) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return static_cast<s16>(-limit);
  }
  return value;
}

void Spu::mix_reverb(float in_l, float in_r, float &wet_l, float &wet_r) {
  wet_l = 0.0f;
  wet_r = 0.0f;
  if ((spucnt_ & 0x0080u) == 0) {
    reverb_half_rate_phase_ = false;
    reverb_hold_l_ = 0.0f;
    reverb_hold_r_ = 0.0f;
    return;
  }
  // Hardware reverb core runs at 22.05kHz and output is held for the next slot.
  reverb_half_rate_phase_ = !reverb_half_rate_phase_;
  if (!reverb_half_rate_phase_) {
    wet_l = reverb_hold_l_;
    wet_r = reverb_hold_r_;
    return;
  }

  const s32 send_l =
      static_cast<s32>(std::clamp(in_l, -1.0f, 1.0f) * 32767.0f);
  const s32 send_r =
      static_cast<s32>(std::clamp(in_r, -1.0f, 1.0f) * 32767.0f);
  // Reverb matrix input gain is configured via 0x1FC/0x1FE.
  const s32 in_l_q = mul_q15(reverb_regs_.vLIN, send_l);
  const s32 in_r_q = mul_q15(reverb_regs_.vRIN, send_r);
  const s32 d_apf1 = static_cast<s32>(reverb_regs_.dAPF1) * 8;
  const s32 d_apf2 = static_cast<s32>(reverb_regs_.dAPF2) * 8;
  const s32 lsame_tap = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.dLSAME));
  const s32 rsame_tap = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.dRSAME));
  const s32 ldiff_tap = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.dLDIFF));
  const s32 rdiff_tap = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.dRDIFF));
  const s32 lsame_hist = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLSAME, -2));
  const s32 rsame_hist = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRSAME, -2));
  const s32 ldiff_hist = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLDIFF, -2));
  const s32 rdiff_hist = reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRDIFF, -2));

  const s16 v_wall = clamp_fb_coef(reverb_regs_.vWALL);
  const s16 v_iir = clamp_fb_coef(reverb_regs_.vIIR);
  const s32 same_l_in =
      in_l_q + mul_q15(v_wall, lsame_tap) - lsame_hist;
  const s32 same_r_in =
      in_r_q + mul_q15(v_wall, rsame_tap) - rsame_hist;
  const s32 diff_l_in =
      in_l_q + mul_q15(v_wall, rdiff_tap) - ldiff_hist;
  const s32 diff_r_in =
      in_r_q + mul_q15(v_wall, ldiff_tap) - rdiff_hist;

  const s32 lsame_new = mul_q15(v_iir, same_l_in) + lsame_hist;
  const s32 rsame_new = mul_q15(v_iir, same_r_in) + rsame_hist;
  const s32 ldiff_new = mul_q15(v_iir, diff_l_in) + ldiff_hist;
  const s32 rdiff_new = mul_q15(v_iir, diff_r_in) + rdiff_hist;

  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mLSAME), sat16(lsame_new));
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mRSAME), sat16(rsame_new));
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mLDIFF), sat16(ldiff_new));
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mRDIFF), sat16(rdiff_new));

  s32 comb_l = 0;
  s32 comb_r = 0;
  const s16 v_comb1 = clamp_fb_coef(reverb_regs_.vCOMB1, 0x5000);
  const s16 v_comb2 = clamp_fb_coef(reverb_regs_.vCOMB2, 0x5000);
  const s16 v_comb3 = clamp_fb_coef(reverb_regs_.vCOMB3, 0x5000);
  const s16 v_comb4 = clamp_fb_coef(reverb_regs_.vCOMB4, 0x5000);
  comb_l += mul_q15(v_comb1,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLCOMB1)));
  comb_l += mul_q15(v_comb2,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLCOMB2)));
  comb_l += mul_q15(v_comb3,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLCOMB3)));
  comb_l += mul_q15(v_comb4,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLCOMB4)));
  comb_r += mul_q15(v_comb1,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRCOMB1)));
  comb_r += mul_q15(v_comb2,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRCOMB2)));
  comb_r += mul_q15(v_comb3,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRCOMB3)));
  comb_r += mul_q15(v_comb4,
                    reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRCOMB4)));
  comb_l = static_cast<s32>(sat16(comb_l));
  comb_r = static_cast<s32>(sat16(comb_r));

  const s32 lapf1_hist =
      reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLAPF1, -d_apf1));
  const s32 rapf1_hist =
      reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRAPF1, -d_apf1));
  const s16 v_apf1 = clamp_fb_coef(reverb_regs_.vAPF1, 0x5000);
  const s16 v_apf2 = clamp_fb_coef(reverb_regs_.vAPF2, 0x5000);
  const s32 lapf1_w = comb_l - mul_q15(v_apf1, lapf1_hist);
  const s32 rapf1_w = comb_r - mul_q15(v_apf1, rapf1_hist);
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mLAPF1), sat16(lapf1_w));
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mRAPF1), sat16(rapf1_w));
  const s32 lapf1_out = mul_q15(v_apf1, lapf1_w) + lapf1_hist;
  const s32 rapf1_out = mul_q15(v_apf1, rapf1_w) + rapf1_hist;

  const s32 lapf2_hist =
      reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mLAPF2, -d_apf2));
  const s32 rapf2_hist =
      reverb_read_s16(reverb_addr_from_reg(reverb_regs_.mRAPF2, -d_apf2));
  const s32 lapf2_w = lapf1_out - mul_q15(v_apf2, lapf2_hist);
  const s32 rapf2_w = rapf1_out - mul_q15(v_apf2, rapf2_hist);
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mLAPF2), sat16(lapf2_w));
  reverb_write_s16(reverb_addr_from_reg(reverb_regs_.mRAPF2), sat16(rapf2_w));
  const s32 lapf2_out = mul_q15(v_apf2, lapf2_w) + lapf2_hist;
  const s32 rapf2_out = mul_q15(v_apf2, rapf2_w) + rapf2_hist;

  // Reverb output gain (0x184/0x186) applied after core.
  const s32 wet_l_q = mul_q15(reverb_depth_l_, lapf2_out);
  const s32 wet_r_q = mul_q15(reverb_depth_r_, rapf2_out);
  float wet_l_f = static_cast<float>(sat16(wet_l_q)) / 32768.0f;
  float wet_r_f = static_cast<float>(sat16(wet_r_q)) / 32768.0f;
  if (std::abs(wet_l_f) > 0.75f || std::abs(wet_r_f) > 0.75f) {
    ++audio_diag_.reverb_guard_events;
  }
  wet_l = soft_clip(wet_l_f);
  wet_r = soft_clip(wet_r_f);
  reverb_hold_l_ = wet_l;
  reverb_hold_r_ = wet_r;

  reverb_cursor_ += 2;
  const u32 span = reverb_work_area_span();
  if (span != 0 && reverb_cursor_ >= span) {
    reverb_cursor_ = 0;
  }
  ++audio_diag_.reverb_mix_frames;
}

void Spu::tick_volume_sweep() {
  if (regs_[0x180 / 2] & 0x8000u) {
    master_vol_l_ = sat16(decode_sweep_step(regs_[0x180 / 2], master_vol_l_));
  }
  if (regs_[0x182 / 2] & 0x8000u) {
    master_vol_r_ = sat16(decode_sweep_step(regs_[0x182 / 2], master_vol_r_));
  }
  if (regs_[0x184 / 2] & 0x8000u) {
    reverb_depth_l_ = sat16(decode_sweep_step(regs_[0x184 / 2], reverb_depth_l_));
  }
  if (regs_[0x186 / 2] & 0x8000u) {
    reverb_depth_r_ = sat16(decode_sweep_step(regs_[0x186 / 2], reverb_depth_r_));
  }
}

void Spu::queue_host_audio(const std::vector<s16> &samples) {
  if (samples.empty()) {
    return;
  }

  if (capture_enabled_) {
    if (capture_samples_.size() + samples.size() > CAPTURE_MAX_SAMPLES) {
      size_t drop = capture_samples_.size() + samples.size() - CAPTURE_MAX_SAMPLES;
      drop &= ~static_cast<size_t>(1);
      if (drop > 0 && drop < capture_samples_.size()) {
        capture_samples_.erase(capture_samples_.begin(),
                               capture_samples_.begin() + static_cast<s64>(drop));
      } else if (drop >= capture_samples_.size()) {
        capture_samples_.clear();
      }
    }
    capture_samples_.insert(capture_samples_.end(), samples.begin(), samples.end());
    audio_diag_.capture_frames += samples.size() / 2;
    // Deterministic offline capture mode: avoid SDL queue coupling.
    return;
  }

  if (!audio_enabled_ || audio_device_ == 0) {
    return;
  }

  host_staging_samples_.insert(host_staging_samples_.end(), samples.begin(),
                               samples.end());

  if (host_staging_samples_.size() > HOST_STAGING_MAX_SAMPLES) {
    size_t drop = host_staging_samples_.size() - HOST_STAGING_MAX_SAMPLES;
    drop &= ~static_cast<size_t>(1);
    if (drop > 0) {
      host_staging_samples_.erase(
          host_staging_samples_.begin(),
          host_staging_samples_.begin() + static_cast<s64>(drop));
      audio_diag_.dropped_frames += drop / 2;
      ++audio_diag_.overrun_events;
    }
  }

  while (!host_staging_samples_.empty()) {
    const u32 queued = SDL_GetQueuedAudioSize(audio_device_);
    audio_diag_.queue_last_bytes = queued;
    audio_diag_.queue_peak_bytes = std::max(audio_diag_.queue_peak_bytes, queued);

    if (queued >= HOST_MAX_QUEUE_BYTES) {
      break;
    }

    u32 room = 0;
    if (queued < HOST_TARGET_QUEUE_BYTES) {
      room = HOST_TARGET_QUEUE_BYTES - queued;
    } else {
      room = HOST_MAX_QUEUE_BYTES - queued;
    }
    if (room < sizeof(s16) * 2u) {
      break;
    }

    size_t samples_room = room / sizeof(s16);
    samples_room &= ~static_cast<size_t>(1);
    size_t to_queue = std::min(samples_room, host_staging_samples_.size());
    to_queue &= ~static_cast<size_t>(1);
    if (to_queue == 0) {
      break;
    }

    SDL_QueueAudio(audio_device_, host_staging_samples_.data(),
                   static_cast<Uint32>(to_queue * sizeof(s16)));
    audio_diag_.queued_frames += to_queue / 2;
    host_staging_samples_.erase(
        host_staging_samples_.begin(),
        host_staging_samples_.begin() + static_cast<s64>(to_queue));
  }
}

void Spu::tick(u32 cycles) {
  if (transfer_busy_cycles_ > 0) {
    transfer_busy_cycles_ =
        (transfer_busy_cycles_ > cycles) ? (transfer_busy_cycles_ - cycles) : 0;
  }

  if ((spucnt_ & 0x8000u) == 0) {
    return;
  }

  const bool muted = (spucnt_ & 0x4000u) == 0;
  audio_diag_.reverb_enabled = (spucnt_ & 0x0080u) != 0;
  audio_diag_.gaussian_active = true;

  sample_accum_ +=
      (static_cast<double>(cycles) * static_cast<double>(SAMPLE_RATE)) /
      static_cast<double>(psx::CPU_CLOCK_HZ);

  const int samples_to_gen = static_cast<int>(sample_accum_);
  if (samples_to_gen <= 0) {
    return;
  }
  sample_accum_ -= samples_to_gen;
  audio_diag_.generated_frames += static_cast<u64>(samples_to_gen);

  std::vector<s16> out(static_cast<size_t>(samples_to_gen) * 2);
  for (int s = 0; s < samples_to_gen; ++s) {
    capture_half_ ^= 1u;
    tick_volume_sweep();
    float mix_l = 0.0f;
    float mix_r = 0.0f;
    float wet_l = 0.0f;
    float wet_r = 0.0f;
    float rev_send_l = 0.0f;
    float rev_send_r = 0.0f;

    for (int v = 0; v < NUM_VOICES; ++v) {
      VoiceState &vs = voices_[v];
      if (vs.phase == VoiceState::AdsrPhase::Off) {
        continue;
      }
      prime_gaussian_history(v);

      const u32 base = static_cast<u32>(v) * VOICE_REG_STRIDE;
      const u16 pitch = regs_[(base + 0x4) / 2];
      const float vol_l = decode_volume(regs_[(base + 0x0) / 2]);
      const float vol_r = decode_volume(regs_[(base + 0x2) / 2]);
      const u32 step = std::min<u32>(pitch, 0x4000u);

      tick_adsr(vs);
      const s16 raw = gaussian_interpolate(vs);
      const s16 env = apply_envelope(raw, vs);
      const float smp = q15_to_float(env);

      const float v_l = smp * vol_l;
      const float v_r = smp * vol_r;
      mix_l += v_l;
      mix_r += v_r;
      if (reverb_on_mask_ & (1u << v)) {
        rev_send_l += v_l;
        rev_send_r += v_r;
      }

      vs.pitch_counter += step;
      while (vs.pitch_counter >= 0x1000u) {
        vs.pitch_counter -= 0x1000u;
        advance_gaussian_history(v);
      }
    }

    audio_diag_.peak_dry_l =
        std::max(audio_diag_.peak_dry_l, std::abs(mix_l));
    audio_diag_.peak_dry_r =
        std::max(audio_diag_.peak_dry_r, std::abs(mix_r));
    if (std::abs(mix_l) > 1.0f || std::abs(mix_r) > 1.0f) {
      ++audio_diag_.clip_events_dry;
    }

    mix_reverb(rev_send_l, rev_send_r, wet_l, wet_r);
    audio_diag_.peak_wet_l =
        std::max(audio_diag_.peak_wet_l, std::abs(wet_l));
    audio_diag_.peak_wet_r =
        std::max(audio_diag_.peak_wet_r, std::abs(wet_r));
    if (std::abs(wet_l) > 0.98f || std::abs(wet_r) > 0.98f) {
      ++audio_diag_.clip_events_wet;
    }

    mix_l += wet_l;
    mix_r += wet_r;

    // Master volume.
    const float mvl = q15_to_float(master_vol_l_);
    const float mvr = q15_to_float(master_vol_r_);
    mix_l *= mvl;
    mix_r *= mvr;

    audio_diag_.peak_mix_l =
        std::max(audio_diag_.peak_mix_l, std::abs(mix_l));
    audio_diag_.peak_mix_r =
        std::max(audio_diag_.peak_mix_r, std::abs(mix_r));
    if (std::abs(mix_l) > 1.0f || std::abs(mix_r) > 1.0f) {
      ++audio_diag_.clip_events_out;
    }

    mix_l = std::clamp(mix_l, -1.0f, 1.0f);
    mix_r = std::clamp(mix_r, -1.0f, 1.0f);
    if (muted) {
      out[static_cast<size_t>(s) * 2] = 0;
      out[static_cast<size_t>(s) * 2 + 1] = 0;
    } else {
      out[static_cast<size_t>(s) * 2] = static_cast<s16>(mix_l * 30000.0f);
      out[static_cast<size_t>(s) * 2 + 1] = static_cast<s16>(mix_r * 30000.0f);
    }
  }

  queue_host_audio(out);
}

void Spu::tick_adsr(VoiceState &vs) {
  auto run_adsr = [&](u8 shift, u8 step, bool decreasing, bool exponential) {
    s32 adsr_step = adsr_base_step(step, decreasing);
    adsr_step <<= std::max(0, 11 - static_cast<int>(shift));

    const u32 counter_inc =
        0x8000u >> std::max(0, static_cast<int>(shift) - 11);
    vs.adsr_counter = static_cast<u16>(vs.adsr_counter + counter_inc);
    if ((vs.adsr_counter & 0x8000u) == 0) {
      return;
    }

    if (exponential && !decreasing && vs.env_level > 0x6000) {
      adsr_step /= 4;
    }
    if (exponential && decreasing) {
      adsr_step =
          (adsr_step * static_cast<s32>(vs.env_level)) / static_cast<s32>(0x8000);
    }

    const s32 next = clamp_env_level(static_cast<s32>(vs.env_level) + adsr_step);
    vs.env_level = static_cast<u16>(next);
    vs.adsr_counter &= 0x7FFFu;
  };

  switch (vs.phase) {
  case VoiceState::AdsrPhase::Attack:
    run_adsr(vs.attack_shift, vs.attack_step, false, vs.attack_exp);
    if (vs.env_level >= kEnvMax) {
      vs.env_level = kEnvMax;
      vs.phase = VoiceState::AdsrPhase::Decay;
    }
    break;
  case VoiceState::AdsrPhase::Decay:
    run_adsr(static_cast<u8>(vs.decay_shift + 2), 0, true, true);
    if (vs.env_level <= vs.sustain_level) {
      vs.env_level = vs.sustain_level;
      vs.phase = VoiceState::AdsrPhase::Sustain;
    }
    break;
  case VoiceState::AdsrPhase::Sustain:
    run_adsr(vs.sustain_shift, vs.sustain_step, vs.sustain_decrease,
             vs.sustain_exp);
    break;
  case VoiceState::AdsrPhase::Release:
    run_adsr(vs.release_shift, 0, true, vs.release_exp);
    if (vs.env_level == 0) {
      vs.phase = VoiceState::AdsrPhase::Off;
      vs.key_on = false;
    }
    break;
  case VoiceState::AdsrPhase::Off:
  default:
    break;
  }
}
