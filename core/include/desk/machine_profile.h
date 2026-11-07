// deskbot core - the machine descriptor.
//
// THIS FILE IS THE SEAM THAT MAKES THE SOFTWARE GENERIC. Everything below the
// controller is written against MachineProfile; nothing in core/ knows it is
// talking to a Husky work bench, an IKEA SKARSTA, or a two-motor office desk.
// A new desk is a JSON file (see machines/<name>.machine.json) plus a mount.
#pragma once
#include "desk/types.h"

namespace desk {

// How the controller's velocity command reaches the mechanics. Retrofit
// projects differ here more than anywhere else, so it is data, not code.
enum class DriveKind : u8 {
  StepperPulse = 0,  // STEP/DIR to a chopper driver (this project's default)
  DcHBridge,         // PWM + direction on a geared DC motor
  KeypadSim,         // opto-isolated taps across an existing desk keypad
};

// Where position feedback comes from. Open loop is allowed but the controller
// then refuses to trust its own position after a stall.
enum class FeedbackKind : u8 {
  None = 0,          // pure open-loop stepper
  MotorEncoder,      // magnetic encoder on the motor shaft (cheap, before gears)
  ShaftEncoder,      // encoder on the crank/torque tube (after gears: best)
};

// How the two legs of a desk relate to each other. Critical correctness
// detail: a hand-crank bench like the Husky has ONE shaft through both legs,
// so the legs cannot desync no matter what - skew is then a *health alarm*
// (baked/broken worm gear), not a correction target. A dual-motor desk must
// actively equalise. Getting this wrong produces software that fights itself.
enum class AxisCoupling : u8 {
  Single = 0,        // one drive, both legs (torque tube / single screw)
  MasterFollower,    // two drives, software keeps them together
  Independent,       // two drives, no hard requirement to stay level
};

struct AxisProfile {
  // --- the only three numbers you must measure on a real desk --------------
  steps_t steps_per_motor_rev = 3200;  // full steps x microstep setting
  float   gear_ratio = 9.0f;           // motor revs per drive-shaft rev
  mm_t    mm_per_shaft_rev = 6.77f;    // top travel per one revolution

  // Soft limits, in desk millimetres, measured with a ruler at the top
  // surface. Mechanical hard stops must sit outside these.
  mm_t travel_low_mm = 660.0f;
  mm_t travel_high_mm = 1057.0f;

  // Rate/acceleration ceilings for *this* machine. Keep accel low: a desk
  // with a monitor arm on it is a cantilevered load and a stepper that
  // accelerates harder than the mechanism can carry just loses steps.
  rate_t vmax_up_mm_s = 12.0f;
  rate_t vmax_down_mm_s = 14.0f;
  rate_t amax_mm_s2 = 45.0f;

  // Sign convention: +1 if driving the motor positive makes the top rise.
  float direction = 1.0f;

  // Per-axis encoder pulses per motor revolution (4x quadrature counts).
  u32 encoder_counts_per_motor_rev = 2000;
};

struct SafetyProfile {
  // Stall: how far the mechanism may lag the command before we call it.
  // Expressed in desk millimetres because that is what a human can reason
  // about, and it survives a gear-ratio change.
  float  stall_lag_mm_moving = 1.6f;
  float  stall_lag_mm_stopped = 0.35f;
  // Parked-position tolerance: more than this between encoder and model while
  // nothing is driving means the top was moved by hand or the gears slipped.
  float  position_loss_mm = 2.0f;
  us_t   stall_hold_us = 220'000;      // lag must persist before tripping

  // When a stall trips while descending we back up (up) this far: a heavy top
  // on a nearly-self-locking worm will keep drifting if we just cut power.
  mm_t   stall_backoff_up_mm = 4.0f;

  // Homing hunt speed and how far ahead a held button aims the planner, so a
  // jog ramps up but never runs away if the input line sticks.
  rate_t home_rate_mm_s = 3.0f;
  // Homing must finish in bounded time. Without this, an obstruction during
  // the downward home hunt leaves the motor driving into it indefinitely: the
  // position-lag detector cannot help, because before the first anchor there
  // is no trusted position to compare against. Margin covers the back-off and
  // ramp. This is the only thing standing between a blocked home and a
  // cooked driver.
  float  home_timeout_margin = 1.6f;
  us_t   home_max_us = 0;   // 0 = derive from travel span / home rate
  mm_t   jog_lookahead_mm = 12.0f;

  // Obstacle / human-in-the-way handling for descending moves.
  bool   obstacle_backoff = true;
  // Only meaningful when coupling != Single.
  float  max_skew_mm = 8.0f;          // hard stop: unequalise and halt
  float  skew_alarm_mm = 4.0f;        // warn the operator
  // Proportional unequalise gain: per mm of skew, how much to bias each leg's
  // rate as a fraction. 0.10 means 1 mm of skew adds 10% to the lagging leg.
  float  skew_trim_per_mm = 0.10f;
  float  skew_trim_max = 0.25f;       // never command more than +25% on a leg

  // Driver/driver-board thermal cut-out (deg C at the sensed heatsink).
  float  temp_c_stop = 85.0f;
  float  temp_c_reenter = 65.0f;

  // A remote (Wi-Fi/CLI) command stops mattering after this long unless the
  // host keeps asserting it. Buttons are never subject to the deadman.
  us_t   remote_deadman_us = 4'000'000;

  // Duty limiting: a crank bench is not meant to be cycled like a CNC axis.
  us_t   move_window_us = 120'000'000;   // 2 min
  us_t   move_max_us   = 60'000'000;     // 1 min of the 2 min window
  us_t   cool_us = 300'000'000;          // forced rest after over-run
};

struct MachineProfile {
  // Identity, surfaced over USB/Wi-Fi so a controller board knows what it is
  // bolted to and the CLI can refuse a mismatched profile.
  const char* name = "generic";
  const char* url = "";
  u32         profile_version = 1;

  DriveKind    drive = DriveKind::StepperPulse;
  FeedbackKind feedback = FeedbackKind::MotorEncoder;
  AxisCoupling coupling = AxisCoupling::Single;
  u8           axis_count = 1;

  AxisProfile  axis[kMaxAxis] = {};
  SafetyProfile safety = {};

  // --- derived helpers ---------------------------------------------------
  // Desk millimetres -> motor microsteps. One function, used everywhere, so
  // a mis-scaled encoder is impossible to hide in three places.
  steps_t mm_to_steps(const AxisProfile& a, mm_t mm) const {
    const float revs = mm / a.mm_per_shaft_rev;
    const float msteps = revs * a.gear_ratio * static_cast<float>(a.steps_per_motor_rev);
    return static_cast<steps_t>(std::lroundf(a.direction * msteps));
  }

  mm_t steps_to_mm(const AxisProfile& a, steps_t s) const {
    const float msteps = static_cast<float>(a.direction) * static_cast<float>(s);
    const float revs = msteps /
        (static_cast<float>(a.steps_per_motor_rev) * a.gear_ratio);
    return revs * a.mm_per_shaft_rev;
  }

  // Encoder counts -> millimetres of top travel. Note this is MOTOR counts:
  // with FeedbackKind::MotorEncoder the ratio must include the printed gears,
  // so `gear_ratio` belongs in the measurement, not here. With ShaftEncoder
  // (encoder on the crank) pass gear_ratio = 1 in the axis profile.
  mm_t counts_to_mm(const AxisProfile& a, i64 counts) const {
    const float motor_revs =
        static_cast<float>(counts) / static_cast<float>(a.encoder_counts_per_motor_rev);
    const float shaft_revs = motor_revs / (a.gear_ratio > 0.0f ? a.gear_ratio : 1.0f);
    return shaft_revs * a.mm_per_shaft_rev;
  }

  rate_t vmax_for(const AxisProfile& a, mm_t target_mm, mm_t current_mm) const {
    return target_mm >= current_mm ? a.vmax_up_mm_s : a.vmax_down_mm_s;
  }

  bool in_travel(mm_t mm, u8 ax = 0) const {
    const AxisProfile& a = axis[ax < axis_count ? ax : 0];
    return mm >= a.travel_low_mm && mm <= a.travel_high_mm;
  }
};

}  // namespace desk
