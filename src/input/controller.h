#pragma once
#include "../core/types.h"
#include <string>

// ── PS1 Controller Emulation ───────────────────────────────────────
// Maps abstract button states to the PS1 controller protocol.

// PS1 button bits (active LOW — 0 = pressed, 1 = released)
enum class PsxButton : u16 {
  Select = 1 << 0,
  L3 = 1 << 1, // Analog stick press
  R3 = 1 << 2,
  Start = 1 << 3,
  Up = 1 << 4,
  Right = 1 << 5,
  Down = 1 << 6,
  Left = 1 << 7,
  L2 = 1 << 8,
  R2 = 1 << 9,
  L1 = 1 << 10,
  R1 = 1 << 11,
  Triangle = 1 << 12,
  Circle = 1 << 13,
  Cross = 1 << 14,
  Square = 1 << 15,
};

class Controller {
public:
  void press(PsxButton btn) { buttons_ &= ~static_cast<u16>(btn); }
  void release(PsxButton btn) { buttons_ |= static_cast<u16>(btn); }
  void set_button_state(u16 buttons) { buttons_ = buttons; }

  void set_left_stick(u8 x, u8 y) {
    lx_ = x;
    ly_ = y;
  }
  void set_right_stick(u8 x, u8 y) {
    rx_ = x;
    ry_ = y;
  }

  u16 button_state() const { return buttons_; }
  u8 lx() const { return lx_; }
  u8 ly() const { return ly_; }
  u8 rx() const { return rx_; }
  u8 ry() const { return ry_; }

  void reset() {
    buttons_ = 0xFFFF;
    lx_ = ly_ = rx_ = ry_ = 0x80;
  }

private:
  u16 buttons_ = 0xFFFF; // All released
  u8 lx_ = 0x80, ly_ = 0x80;
  u8 rx_ = 0x80, ry_ = 0x80;
};
