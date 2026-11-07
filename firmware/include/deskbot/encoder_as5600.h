// deskbot firmware - AS5600 absolute angle -> cumulative counts.
//
// This file is the highest-risk 40 lines in firmware/, which is why it is a
// header shared between the target and a host unit test rather than private to
// main.cpp.
//
// THE MISMATCH
// desk::Inputs::encoder_counts is a cumulative int64: monotonic while the shaft
// turns, and the core feeds it through Calibration::counts_to_mm() and then
// into the stall / position_lost detectors, which are all *differences* against
// a modelled position. The AS5600 is a 12-bit absolute angle sensor: it reports
// 0..4095 for one revolution and silently wraps at both ends. Hand the core the
// raw register and every crossing of 4095 -> 0 looks like the desk jumped back
// ~4096 counts (~1.4 mm on the Husky gearbox), once per motor revolution, and
// stall detection fires on a healthy machine - or worse, at exactly the moment
// it matters.
//
// THE FIX
// Track the revolution count. On each sample, take the shortest arc from the
// last angle to the new one in raw counts; if that arc is more than half a turn
// the sensor must have wrapped, so add/subtract a full turn instead.
//
// How big can a legitimate single-tick movement be? At the profile's fastest
// descent (14 mm/s), with gear_ratio 5.06 and 5.5 mm per shaft rev, the MOTOR
// turns 14/5.5*5.06 = 12.9 rev/s, i.e. 12.9*4096*0.002 = ~106 raw counts per
// 2 ms sample. Half a turn is 2048. So the shortest-arc rule has ~19x margin
// in normal operation, and a sanity threshold of 512 counts/tick sits
// comfortably between "real motion" and "bogus read" while staying below the
// ambiguity limit.
//
// THE BOUNDARY, EXPLICITLY
// A delta of exactly +kHalfTurn or -kHalfTurn is ambiguous by construction: it
// is equally 2048 counts up or 2048 counts down. This code resolves it as "no
// wrap" (the strict > and < below). Reaching that boundary requires ~20x the
// physically possible shaft speed, i.e. a hand yanking the top down or a sensor
// already misbehaving; either way the kMaxDeltaCounts check below latches
// encoder_valid=false before the boundary case can silently corrupt the datum.
//
// DIRECTION
// The AS5600 angle increases in one physical direction; whether that is "up"
// depends on magnet orientation and mount. The profile's `direction` field
// already signs the *motor* steps; the encoder sign is separate and is set at
// compile time (see kEncoderInvert in main.cpp) and must be verified at
// bring-up: raise the desk by hand and confirm the reported counts increase.
#pragma once
#include <stdint.h>

namespace deskbot {

// --- the unwrap, in absolute-count-space, as pure integer math ---------------
// The math lives once, here, and the target wraps it. (It is not templated on
// the bit width: keeping one spelling means the host test cannot drift from
// what the ESP32 executes.)
struct AngleUnwrap {
  static constexpr uint16_t kResolution = 12;
  static constexpr int32_t kTurn = 1 << kResolution;        // 4096 raw counts
  static constexpr int32_t kHalfTurn = kTurn / 2;           // 2048

  // Refuse any single-tick movement above this as implausible rather than
  // silently accumulating a wrong revolution. ~5x the physical worst case
  // (~106 counts/tick, computed above): enough margin for real motion and
  // noise, still far below the kHalfTurn ambiguity limit, so a glitchy I2C
  // read or a spinning-away shaft is reported instead of integrated.
  static constexpr int32_t kMaxDeltaCounts = 512;

  bool started = false;
  uint16_t last_raw = 0;
  int32_t rev = 0;               // completed revolutions, signed
  bool valid = false;            // sticky: false after an implausible jump
  uint32_t fault_count = 0;

  // Feed one 12-bit register value. Returns false (and latches invalid) if the
  // jump was not physically plausible. On success, *out is cumulative counts.
  //
  // ORDER MATTERS: unwrap first, then sanity-check. Checking the raw delta
  // before unwrapping would reject every legitimate 4095 -> 0 crossing (raw
  // delta -4095) as a fault, i.e. the sensor would break once per revolution.
  bool update(uint16_t raw, int64_t* out) {
    raw &= static_cast<uint16_t>(kTurn - 1);
    if (!started) {
      started = true;
      last_raw = raw;
      rev = 0;
      valid = true;
      *out = static_cast<int32_t>(raw);
      return true;
    }
    // Invalid is sticky until reset(). Once the datum is lost, no later sample
    // can be trusted with it - recovery is a re-home, not a good read. Without
    // this, a caller that looked at the return value instead of valid() could
    // resume feedback with a revolution count off by an unknown amount.
    if (!valid) {
      *out = cumulative();
      return false;
    }
    const int32_t raw_delta = static_cast<int32_t>(raw) - static_cast<int32_t>(last_raw);

    // Shortest arc. A raw jump past +half a turn means the sensor actually went
    // down through zero; past -half a turn it went up through zero (see the
    // boundary discussion at the top of this file, which explains why exactly
    // +-half a turn resolves as "no wrap" and why that is only reachable if
    // something is already badly wrong).
    int32_t delta = raw_delta;
    int32_t rev_step = 0;
    if (raw_delta > kHalfTurn) { delta -= kTurn; rev_step = -1; }
    else if (raw_delta < -kHalfTurn) { delta += kTurn; rev_step = +1; }

    // Now the per-tick movement is in ordinary signed counts and must be small.
    // If it is not, the sensor, the wiring, or the tick rate is lying: do not
    // guess a revolution count out of a bad sample - report it and let the
    // caller drop feedback and demand a re-home.
    if (delta > kMaxDeltaCounts || delta < -kMaxDeltaCounts) {
      valid = false;
      ++fault_count;
      *out = cumulative();      // last known value; caller must ignore it
      return false;
    }

    // Commit the datum only for an accepted sample, so a rejected one cannot
    // shift the revolution count out from under the core.
    rev += rev_step;
    last_raw = raw;
    *out = cumulative();
    return true;
  }

  int64_t cumulative() const {
    return static_cast<int64_t>(rev) * kTurn + static_cast<int32_t>(last_raw);
  }

  void reset() { *this = AngleUnwrap{}; }
};

// Raw AS5600 register -> cumulative counts -> core units.
//
// The core expects `encoder_counts_per_motor_rev` counts per motor revolution
// (documented in machine_profile.h as "4x quadrature counts"). The AS5600
// gives 4096. Scale so the profile's number stays meaningful and the one true
// conversion lives here:
//
//     counts = cumulative_raw * counts_per_motor_rev / 4096
//
// IMPORTANT: at 4096 raw counts/rev against a profile expecting 2000, rounding
// to integers introduces a sawtooth of up to +-0.5 core counts. On the Husky
// one core count is 5.5/(5.06*2000) = 0.00054 mm, so the sawtooth is under
// +-0.3 um - six thousand times under the 1.6 mm stall threshold and
// invisible. The same scaling with a *lower* resolution absolute encoder
// (say a 10-bit AS5048 at 1024) would quantise the other way: the profile
// could not be more precise than the sensor, and encoder_counts_per_motor_rev
// should then equal the raw resolution to keep the mapping 1:1. See
// docs/firmware-design-notes.md.
class As5600Track {
 public:
  void configure(int32_t counts_per_motor_rev) {
    counts_per_motor_rev_ = counts_per_motor_rev > 0 ? counts_per_motor_rev : 4096;
  }

  bool update(uint16_t raw, int64_t* counts_out) {
    int64_t raw_cum = 0;
    const bool ok = unwrap_.update(raw, &raw_cum);
    // Round-to-nearest so the sawtooth is centred on zero instead of biased
    // low: a one-sided bias is exactly what a stall detector notices over a
    // long descent.
    const int64_t num = raw_cum * counts_per_motor_rev_ + AngleUnwrap::kTurn / 2;
    *counts_out = floor_div(num, AngleUnwrap::kTurn);
    return ok && unwrap_.valid;
  }

  bool valid() const { return unwrap_.valid; }
  uint32_t faults() const { return unwrap_.fault_count; }
  int64_t raw_cumulative() const { return unwrap_.cumulative(); }
  void reset() { unwrap_.reset(); }

 private:
  // Arithmetic-shift-style floor division (C++ / truncates toward zero).
  static int64_t floor_div(int64_t a, int64_t b) {
    int64_t q = a / b;
    if ((a % b) != 0 && ((a < 0) != (b < 0))) --q;
    return q;
  }

  AngleUnwrap unwrap_;
  int32_t counts_per_motor_rev_ = 4096;
};

}  // namespace deskbot
