// deskbot core - platform-independent types and units.
// No Arduino/ESP-IDF includes in core/: everything here must compile with a
// host compiler so it can be unit-tested without hardware.
#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

namespace desk {

using i8 = int8_t;
using u8 = uint8_t;
using i32 = int32_t;
using i64 = int64_t;
using u32 = uint32_t;
using u64 = uint64_t;

// --- Unit aliases -----------------------------------------------------------
// Distances on the *desk* (what the user sees) are millimetres of top height.
using mm_t = float;
// Distances on the *motor* are whole microsteps. int64 so a lifetime of
// microstepping never wraps.
using steps_t = i64;
// Rates follow the unit of their name: mm/s or steps/s.
using rate_t = float;

// Time is microseconds from an monotonic source supplied by the HAL.
using us_t = u64;

static constexpr mm_t kNoLimit = std::numeric_limits<mm_t>::infinity();
static constexpr i32  kMaxAxis = 2;         // single-shaft or dual-leg desks
static constexpr i32  kMaxPresets = 8;

// Every core routine reports failure through this instead of exceptions
// (exceptions are disabled in the firmware build).
enum class Status : u8 {
  Ok = 0,
  NotCalibrated,
  OutOfRange,
  Busy,
  Stall,
  Skew,
  Blocked,          // obstacle detected, backed off
  LimitLow,
  LimitHigh,
  OverTemp,
  StaleCommand,     // deadman expired (remote control)
  Disabled,         // physical enable / e-stop open
  Storage,
  BadArgument,
  UnsafeToMove,
};

[[nodiscard]] inline const char* name(Status s) {
  switch (s) {
    case Status::Ok:            return "ok";
    case Status::NotCalibrated: return "not-calibrated";
    case Status::OutOfRange:    return "out-of-range";
    case Status::Busy:          return "busy";
    case Status::Stall:         return "stall";
    case Status::Skew:          return "skew";
    case Status::Blocked:       return "blocked";
    case Status::LimitLow:      return "limit-low";
    case Status::LimitHigh:     return "limit-high";
    case Status::OverTemp:      return "over-temp";
    case Status::StaleCommand:  return "stale-command";
    case Status::Disabled:      return "disabled";
    case Status::Storage:       return "storage";
    case Status::BadArgument:   return "bad-argument";
    case Status::UnsafeToMove:  return "unsafe-to-move";
  }
  return "unknown";
}

// A height plus the confidence we have in it. `trusted` is false after a power
// cycle until the mechanism is re-indexed, because a stepper that lost steps
// while parked is indistinguishable from one that did not.
struct Height {
  mm_t mm = 0.0f;
  bool trusted = false;
};

inline bool finite_(float v) { return std::isfinite(v); }

inline mm_t clamp_(mm_t v, mm_t lo, mm_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace desk
