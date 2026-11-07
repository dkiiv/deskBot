// deskbot core - calibration model + safety supervisor.
//
// Two ideas do all the work here:
//
// 1) CALIBRATION IS AN OFFSET, NOT A RECALCULATION. steps_per_rev / gear_ratio
//    / mm_per_rev are fixed once, measured, and frozen. Homing stores which
//    microstep the bottom hard-stop is at. Every position after that is a
//    subtraction. This is why a bench that has been re-homed once stays
//    accurate to a millimetre for years without ever touching the gearing.
//
// 2) A RETROFIT LOSES STEPS. A motor spinning its printed gears, a worm that
//    binds on a dry spot, someone shoving the top by hand: all of them make
//    commanded position and reality diverge silently. So the supervisor
//    continuously compares them and treats divergence as a fault class of its
//    own. On a self-locking worm the desk does not run away when that happens
//    - it just stops being where you think it is - which is precisely when you
//    must not trust soft limits any more.
#pragma once
#include "desk/hal.h"
#include "desk/machine_profile.h"

namespace desk {

struct Calibration {
  u32 magic = 0x44454B31;              // 'DEK1'
  u32 profile_version = 0;             // reject stale cal after a ratio change
  // Height and encoder counts captured at the LOW hard stop during homing.
  // The step counter is zeroed there, so every later position is measured
  // relative to this pair; without them, zeroing the counter would also zero
  // the height and the soft limits would clamp the machine to the floor.
  mm_t    home_mm = 0.0f;
  i64     home_counts = 0;
  steps_t home_steps = 0;
  bool    homed = false;

  // Last height the machine believed, written when a move settles. Distinct
  // from home_mm: home_mm is the DATUM (bottom of travel) and last_height_mm
  // is where the top actually was. Resuming a boot from home_mm instead puts
  // the model at the bottom of travel no matter where the desk really is, and
  // every soft limit is then computed from a fiction.
  mm_t  last_height_mm = 0.0f;
  bool  last_height_valid = false;

  // Ruler-entered anchors: at two known heights, the encoder counts seen.
  // Scaling error is then measurable rather than assumed away.
  mm_t ref_low_mm = 0.0f;  i64 ref_low_counts = 0;
  mm_t ref_high_mm = 0.0f; i64 ref_high_counts = 0;
  bool has_refs = false;

  // Encoder counts -> height in desk millimetres. The controller anchors the
  // encoder to zero at the low hard stop during homing, so counts are already
  // home-relative; add travel_low to get an absolute height.
  mm_t counts_to_mm(const AxisProfile& a, const MachineProfile& m, i64 counts) const {
    // Two-point ruler calibration wins when present: it corrects for a printed
    // gear ratio that is slightly off, which no amount of CAD says so.
    const float span_mm = ref_high_mm - ref_low_mm;
    const double span_ct = static_cast<double>(ref_high_counts - ref_low_counts);
    if (has_refs && std::fabs(span_mm) > 1.0f && std::fabs(span_ct) > 1.0) {
      const double per_mm = span_ct / static_cast<double>(span_mm);
      return ref_low_mm + static_cast<mm_t>((static_cast<double>(counts) -
                                             static_cast<double>(ref_low_counts)) / per_mm);
    }
    return a.travel_low_mm + m.counts_to_mm(a, counts - home_counts);
  }
};

enum class MachineState : u8 {
  Uncalibrated = 0,   // never homed this power cycle: height unknown
  Ready,              // knows where it is, may move
  Moving,
  Settling,
  Retreating,        // deliberate post-stall backing-off
  Fault,              // needs an operator action
  Protect,            // stopped by a hard limit / e-stop, may recover
};

[[nodiscard]] inline const char* name(MachineState s) {
  switch (s) {
    case MachineState::Uncalibrated: return "uncalibrated";
    case MachineState::Ready:        return "ready";
    case MachineState::Moving:       return "moving";
    case MachineState::Settling:     return "settling";
    case MachineState::Retreating:   return "retreating";
    case MachineState::Fault:        return "fault";
    case MachineState::Protect:      return "protect";
  }
  return "?";
}

// One latch per fault class. Sticky until clear_faults() so a user cannot
// unknowingly re-run into the condition that just stopped the desk, but a
// momentary e-stop bounce does not wedge the machine forever either (see
// Supervisor::tick, which auto-clears Protect once the cause is gone).
struct Faults {
  bool stall = false;
  bool skew = false;
  bool blocked = false;
  bool limit_low = false;
  bool limit_high = false;
  bool over_temp = false;
  bool estop = false;
  bool position_lost = false;   // encoder disagrees with commanded, can't trust
  bool home_timeout = false;    // homing ran too long: something is in the way

  bool any_hard() const {
    return stall || skew || blocked || over_temp || estop || position_lost || home_timeout;
  }
  // Being parked against a hard stop is a normal resting state, not a fault:
  // every desk spends its life on one of them until it is told otherwise.
  // That is why limit flags are reported separately from any_hard().
  bool any() const { return any_hard() || limit_low || limit_high; }
  void clear() { *this = {}; }
};

class Supervisor {
 public:
  // Call once per control tick with fresh state. Returns any new fault raised.
  // `in_retreat` widens (never disables) the position-lag threshold while the
  // controller is deliberately driving away from whatever it stalled against:
  // during a retreat the mechanism is expected to lag, and suppressing the
  // detector outright would hide a second, unrelated failure.
  //
  // `position_trusted` MUST be false until the machine has been homed. Before
  // the first anchor the commanded model has no relationship at all to the
  // encoder reading (one says 0, the other says "somewhere in the travel"), so
  // comparing them reports a stall on every machine on the first tick after
  // boot. Pre-home protection is then honestly limited to what still works
  // without a position model: the hard stops, the slow home rate, and the duty
  // budget. Claiming more than that is how a retrofit ends up trusting a
  // number it never measured.
  Status tick(const MachineProfile& m, const Inputs& in, mm_t cmd_mm,
              rate_t cmd_rate, mm_t enc_mm, bool have_encoder,
              Faults& f, us_t now_us, bool in_retreat = false,
              bool position_trusted = true, bool homing = false) {
    // --- e-stop / enable chain: absolute first, never debounced away -------
    if (!in.enable_closed || in.button_stop) {
      if (!f.estop) { f.estop = true; return Status::Disabled; }
      return Status::Disabled;
    }
    if (f.estop && in.enable_closed && !in.button_stop) f.estop = false;

    // --- thermal ----------------------------------------------------------------
    if (in.driver_temp_c >= m.safety.temp_c_stop) {
      if (!f.over_temp) { f.over_temp = true; return Status::OverTemp; }
      return Status::OverTemp;
    }
    if (f.over_temp && in.driver_temp_c <= m.safety.temp_c_reenter) f.over_temp = false;


    // --- position coherence -----------------------------------------------------
    // This detector answers "did the mechanism follow the command?", which is
    // only a meaningful question while the mechanism is being driven. Once the
    // desk is parked and reported settled, drift between encoder and model is a
    // *different* question ("has something moved the top?") and belongs to the
    // controller's integrity check, which uses the looser position_loss_mm.
    // Running the tight stopped-lag threshold on an idle desk turns normal
    // sub-millimetre quantisation into a stall fault a few seconds after every
    // move, which is how a retrofit ends up looking unreliable for no reason.
    if (!position_trusted) lag_us_ = 0;
    const bool driving = std::fabs(cmd_rate) > 0.15f || verifying_stop_;
    if (have_encoder && position_trusted && driving) {
      const float lag = std::fabs(enc_mm - cmd_mm);
      const bool moving = std::fabs(cmd_rate) > 0.15f;
      float thresh = moving ? m.safety.stall_lag_mm_moving
                            : m.safety.stall_lag_mm_stopped;
      if (in_retreat) thresh *= 5.0f;
      if (lag > thresh) {
        if (lag_us_ == 0) lag_us_ = now_us;
        if (now_us - lag_us_ > m.safety.stall_hold_us) {
          lag_us_ = 0;
          if (!f.stall) { f.stall = true; return Status::Stall; }
          return Status::Stall;
        }
      } else {
        lag_us_ = 0;
      }
    }
    if (verifying_stop_ && now_us > verify_until_us_) verifying_stop_ = false;
    if (!driving) lag_us_ = 0;

    // --- hard limits: evaluated LAST, after stall and skew --------------------
    // They used to return early, which silently disabled both the stall and
    // skew checks whenever the desk was parked on a stop - and a desk spends
    // most of its life parked on a stop. Order by severity instead: a tilted
    // frame against a stop is still a tilted frame.
    auto at_limit = [&](Faults& ff, const Inputs& in) {
      if (in.limit_low[0])  { ff.limit_low  = true; return Status::LimitLow; }
      if (in.limit_high[0]) { ff.limit_high = true; return Status::LimitHigh; }
      ff.limit_low = ff.limit_high = false;
      return Status::Ok;
    };

    // --- skew, only when the legs are not mechanically tied together -----------
    // Skipped while homing: each leg is independently hunting its own stop, so
    // the difference between them measures two separate hunts, not a tilted
    // frame. Latching a skew fault during homing is a false alarm that also
    // wedges the machine before it can ever be used.
    if (!homing && m.coupling != AxisCoupling::Single && m.axis_count > 1 &&
        have_encoder) {
      const float skew = std::fabs(enc_mm - enc_other_);
      if (skew > m.safety.max_skew_mm && !f.skew) {
        f.skew = true;
        return Status::Skew;
      }
    }

    // Skew outranks a limit only as a *report*; both can be true, and the
    // caller acts on the latches, not on this return value.
    const Status lim = at_limit(f, in);
    return lim != Status::Ok ? lim : Status::Ok;
  }

  void set_other_axis_mm(mm_t v) { enc_other_ = v; }
  void clear_faults() { f_.clear(); lag_us_ = 0; }

  // Called by the controller when a move finishes: hold the tight threshold
  // open for a short window so a mechanism that cannot actually reach the
  // target (jammed at the end of travel, for instance) is still caught.
  void begin_stop_verification(us_t now_us) {
    verifying_stop_ = true;
    verify_until_us_ = now_us + 500000;   // 500 ms
  }

  // Clear ONLY the stall latch so a controlled retreat can run. Deliberately
  // narrow: after a stall we want to move, but we do not want to forget that
  // something else was wrong.
  void clear_stall() { f_.stall = false; lag_us_ = 0; }
  Faults& faults() { return f_; }
  const Faults& faults() const { return f_; }

  // Whether the supervisor allows motion right now.
  bool can_move() const { return !f_.any_hard(); }

 private:
  Faults f_{};
  us_t lag_us_ = 0;
  bool verifying_stop_ = false;
  us_t verify_until_us_ = 0;
  mm_t enc_other_ = 0.0f;
};

}  // namespace desk
