// deskbot core - online trapezoidal velocity planner.
//
// Deliberately recomputes from the *current* state every tick rather than
// committing to a fixed profile: a desk gets new targets pressed mid-move and
// must react instantly without ever exceeding acceleration. This is the same
// velocity-predictor approach used by commercial desk controllers, and it
// handles asymmetric up/down speed limits for free (descending, gravity is on
// your side; ascending, the worm is).
//
// Why not "move N microsteps at 2 kHz": a bench top with a monitor arm on it
// is a cantilevered load. Jerky starts make the whole bench wobble and make
// the worm gear chatter, which is exactly how a retrofit loses steps and
// decides it is a bad idea. Smooth ramps are the feature.
#pragma once
#include "desk/types.h"

namespace desk {

class Trapezoid {
 public:
  struct Limits {
    rate_t vmax = 10.0f;     // mm/s, magnitude, toward target
    rate_t amax = 40.0f;     // mm/s^2
    rate_t dmax = 40.0f;     // mm/s^2, may be tighter than amax downhill
  };

  // dt_s in seconds.
  // `snap_mm` is what makes a move actually finish: as the gap closes, the
  // stop-rate floor sqrt(2*dmax*remaining) approaches a small but nonzero
  // velocity, so without an explicit snap the state limit-cycles around the
  // target forever and the machine never reports itself settled.
  void plan(rate_t& vel_mm_s, mm_t& pos_mm, mm_t target_mm, const Limits& lim,
            float dt_s, mm_t snap_mm = 0.08f) const {
    if (dt_s <= 0.0f) return;

    if (std::fabs(target_mm - pos_mm) <= snap_mm) {
      pos_mm = target_mm;
      vel_mm_s = 0.0f;
      return;
    }

    const float dir = (target_mm >= pos_mm) ? 1.0f : -1.0f;
    const mm_t remaining = std::fabs(target_mm - pos_mm);

    // Speed we may have at this point and still stop exactly at target.
    const rate_t stop_rate = std::sqrt(2.0f * lim.dmax * remaining);
    const rate_t want = dir * std::min(lim.vmax, stop_rate);

    // Slew toward the *signed* target velocity. Using the signed target is
    // what makes descending work: ramping toward an unsigned magnitude drives
    // the desk upward whenever the target is below the current height, which
    // reads as "it will not come down" and is exactly the sort of bug a
    // descent-only safety path (obstacle detection) hides forever if nothing
    // ever descends in a test.
    const bool speed_up = std::fabs(want) > std::fabs(vel_mm_s) ||
                          (vel_mm_s * want) < 0.0f;
    const rate_t slew = (speed_up ? lim.amax : lim.dmax) * dt_s;
    if (vel_mm_s < want) vel_mm_s = std::min(want, vel_mm_s + slew);
    else if (vel_mm_s > want) vel_mm_s = std::max(want, vel_mm_s - slew);

    // Integrate exactly what we will command, so state and hardware agree.
    pos_mm += vel_mm_s * dt_s;
  }

  // True when the move is finished and settled (velocity bled to zero).
  static bool settled(rate_t vel_mm_s, mm_t pos_mm, mm_t target_mm,
                      mm_t pos_tol_mm, rate_t vel_tol) {
    return std::fabs(pos_mm - target_mm) <= pos_tol_mm &&
           std::fabs(vel_mm_s) <= vel_tol;
  }

  // Time to travel a distance at full speed with trapezoidal ramps. Used to
  // check duty budget before committing to a long move, and to warn a user
  // that "sitting -> 40 cm" will take 90 seconds.
  static float eta_s(mm_t from_mm, mm_t to_mm, const Limits& lim) {
    const mm_t d = std::fabs(to_mm - from_mm);
    if (lim.vmax <= 0.0f || lim.amax <= 0.0f) return 0.0f;
    const mm_t d_ramp = (lim.vmax * lim.vmax) / lim.amax;   // up+down ramps
    if (d_ramp >= d) return 2.0f * std::sqrt(d / lim.amax);
    return d_ramp / lim.vmax + (d - d_ramp) / lim.vmax;
  }
};

}  // namespace desk
