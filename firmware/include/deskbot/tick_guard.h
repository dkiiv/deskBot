// deskbot firmware - "sample hardware at most once per control period".
//
// THE PROBLEM
// control_task reads inputs twice per 2 ms period: once inside
// Controller::tick() via the HAL, and once more immediately afterwards for the
// button policy (home gesture, e-stop acknowledge). Both must see the *same*
// snapshot - if they disagree, the core acts on one reading of the world and
// the operator gestures are interpreted against another. So the HAL caches its
// sample for the duration of a tick.
//
// WHY IT IS A SEPARATE HEADER
// The original guard was `if (tick_ == cache_tick_)` inside ESPHal::
// read_inputs(), and cache_tick_ was never assigned anywhere: the comparison
// was always false, the cache never served, and the AS5600 was read twice per
// control period. That is invisible to a firmware build (it compiles fine) and
// invisible to every host test (it is not firmware-only logic, it is firmware
// *plumbing*). Putting the counter pair here makes the invariant testable
// without an ESP32 - the same trick encoder_as5600.h uses for the unwrap.
//
// WHAT GETS BROKEN WHEN IT FAILS SILENTLY
//   * The encoder unwrap advances once per *sample*, and its plausibility
//     bound (AngleUnwrap::kMaxDeltaCounts) is written in counts per control
//     period. Two samples per period means that bound is checked against half a
//     period of motion - the margin documented in encoder_as5600.h is then
//     computed against a premise that is no longer true.
//   * Every I2C/unwrap counter in the status line reads double during bring-up,
//     which is exactly the moment firmware/README.md says to watch them.
//   * Each AS5600 read takes the bus mutex with a 5 ms timeout, inside a 2 ms
//     period: doing it twice doubles the odds of the timeout that reports
//     "no feedback this tick".
#pragma once
#include <stdint.h>

namespace deskbot {

// One sequence counter pair: `begin()` at the top of a control period,
// `already_sampled()` to decide whether to touch hardware, `mark_sampled()`
// once the sample is taken (and cached).
//
// Deliberately not a template, not virtual, no atomics: it is used from one
// task (control_task) on one core. The status task never samples - it reads the
// seqlock-published snapshot instead. If sampling ever moves to two tasks, this
// needs a lock, and that is a design change, not a bug to paper over.
class TickGuard {
 public:
  // Start a new control period. Call once per period, before anything reads
  // inputs.
  void begin() { ++tick_; }

  // True when this period's hardware sample has already been taken; the caller
  // must serve its cached copy instead of reading again.
  //
  // A call before the first begin() also returns true. That is deliberate:
  // reading hardware outside a control period is the bug this whole type exists
  // to prevent, so the pre-period state fails closed to "serve the cache"
  // rather than "touch the bus". If a future HAL ever initialises inputs before
  // the control task starts, it must call begin() first - loud, not implicit.
  bool already_sampled() const { return tick_ == sampled_tick_; }

  // Record that the sample for this period has been taken.
  void mark_sampled() { sampled_tick_ = tick_; }

  // Diagnostics for tests and the status line.
  uint32_t tick() const { return tick_; }
  uint32_t sampled_tick() const { return sampled_tick_; }

 private:
  // begin() pre-increments, so real periods are numbered 1, 2, 3... and the
  // initial (0, 0) means "no period has begun". The first begin() makes tick_ 1
  // != sampled_tick_ 0, so the first read of the first period samples. An
  // off-by-one here - e.g. a sentinel equal to the first period number - leaves
  // the desk blind to its own encoder for the whole first tick after power-on,
  // which is why a test asserts exactly one sample in the first period rather
  // than merely "not two".
  //
  // Wrap (tick_ back to 0 after 2^32 periods, ~161 days at 2 ms) does NOT skip
  // a sample: by then sampled_tick_ holds 0xFFFFFFFF from the previous period's
  // mark, so 0 != 0xFFFFFFFF still forces a read. Checked by stepping the
  // equivalent arithmetic across the boundary in a scratch harness; the class
  // itself offers no way to preset the counter, so this stays a reasoning +
  // scratch result rather than a committed test. A stale 2 ms frame once every
  // five months, in the "do not touch hardware" direction, is not worth a
  // wider counter on a 2 ms path.
  uint32_t tick_ = 0;
  uint32_t sampled_tick_ = 0;
};

}  // namespace deskbot
