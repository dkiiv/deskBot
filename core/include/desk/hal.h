// deskbot core - hardware abstraction seam.
//
// The controller talks to the world only through this interface. That is what
// lets the exact same motion/safety logic run (a) on an ESP32 against a real
// bench and (b) on a laptop inside a fake-bench simulation with a worm that
// slips, a binding spot, and a heavy top. Keep every method here something a
// bare-metal board can implement.
#pragma once
#include "desk/machine_profile.h"

namespace desk {

struct Inputs {
  bool limit_low[kMaxAxis] = {};     // mechanical hard-stop / reed switch
  bool limit_high[kMaxAxis] = {};
  bool button_up = false;            // held-state, already debounced by HAL
  bool button_down = false;
  bool button_stop = false;          // physical e-stop / enable bar: MUST be NC
  bool enable_closed = true;         // series "deadman" chain closed
  float driver_temp_c = 25.0f;
  i64  encoder_counts[kMaxAxis] = {};
  bool encoder_valid = false;        // false => no feedback fitted: open loop
};

struct Outputs {
  // Signed motor rate in microsteps/second. The HAL owns pulse generation;
  // core never counts individual steps.
  rate_t axis_rate_steps_s[kMaxAxis] = {};
  bool   motor_power = false;        // driver ENABLE, true = coils energised
  bool   brake = false;              // optional powered holding brake
  u8     led_pattern = 0;            // 0 off, 1 moving, 2 fault, 3 need-home
  bool   buzzer = false;
  // Diagnostics snapshot for the UI/CLI.
  mm_t   cmd_mm[kMaxAxis] = {};
  mm_t   enc_mm[kMaxAxis] = {};
};

class Hal {
 public:
  virtual ~Hal() = default;

  virtual us_t now_us() const = 0;
  virtual void read_inputs(Inputs& out) = 0;

  // Rate output. Sign = direction, magnitude = microsteps/s. Zero stops.
  virtual void drive_rate(u8 axis, rate_t steps_per_s) = 0;
  virtual void set_motor_power(bool on) = 0;
  virtual void set_brake(bool on) = 0;

  virtual void set_led(u8 pattern) = 0;
  virtual void set_buzzer(bool on) = 0;
  virtual void log(const char* msg) = 0;

  // Non-volatile storage for calibration + presets. Implementations must be
  // wear-aware (the controller does not throttle writes).
  virtual bool nv_read(u8 key, void* buf, u32 len) = 0;
  virtual bool nv_write(u8 key, const void* buf, u32 len) = 0;
};

// Storage keys. Keep stable: bumping profile_version is cheap, losing a
// user's saved heights is not.
enum NvsKey : u8 {
  kNvsCalibration = 1,
  kNvsPresets     = 2,
  kNvsProfile     = 3,
  kNvsCounters    = 4,
};

}  // namespace desk
