// deskbot core - headless fake bench for tests and simulation.
//
// Models the things that actually break retrofits, so safety logic can be
// proven before there is a desk to plug into:
//   * lost steps when the commanded torque exceeds what the gears can carry
//   * a binding spot in the travel where the mechanism fights back
//   * somebody pushing the top by hand (position lies, no current signature)
//   * an obstacle under the top on the way down
//   * a bounced e-stop contact
//
// The real position is tracked separately from what the controller commands,
// which is the entire point: a controller that only ever reads its own model
// cannot fail, and therefore tests nothing.
#pragma once
#include <cmath>
#include <cstring>

#include "desk/controller.h"

namespace desk {

class FakeBench : public Hal {
 public:
  MachineProfile machine{};
  Calibration cal{};
  Controller ctl{};

  // --- world state (truth) --------------------------------------------------
  mm_t true_mm = 700.0f;
  i64  true_counts = 0;
  // Second leg, used only when machine.axis_count > 1. `leg2_drag` scales the
  // rate leg 2 actually achieves, which is how a sticky leg, an unequal load,
  // or a worn nut on one side shows up in real life.
  mm_t true_mm_b = 700.0f;
  float leg2_drag = 1.0f;
  bool at_low = false, at_high = false;
  bool estop_open = false;
  bool obstacle = false;
  float temp_c = 25.0f;

  // --- knobs ----------------------------------------------------------------
  float slip_threshold_mm_s = 999.0f;  // command rate above which we lose steps
  float slip_rate = 0.0f;             // mm of truth lost per mm commanded
  float binding_low_mm = -1e9f;        // below this, mechanism binds
  float binding_high_mm = 1e9f;
  bool  motor_power_seen = false;
  bool  brake_seen = false;
  rate_t last_rate_steps_s = 0.0f;
  int   log_lines = 0;

  us_t now_us_ = 0;
  Inputs in{};

  us_t now_us() const override { return now_us_; }

  void read_inputs(Inputs& out) override {
    out = in;
    out.limit_low[0] = at_low;
    out.limit_high[0] = at_high;
    out.enable_closed = !estop_open;
    out.driver_temp_c = temp_c;
    out.encoder_counts[0] = true_counts;
    out.encoder_counts[1] = counts_for_(true_mm_b);
    out.encoder_valid = machine.feedback != FeedbackKind::None;
  }

  void drive_rate(u8 axis, rate_t steps_per_s) override {
    if (axis == 0) last_rate_steps_s = steps_per_s;
  }
  void set_motor_power(bool on) override { motor_power_seen = on; }
  void set_brake(bool on) override { brake_seen = on; }
  void set_led(u8 p) override { (void)p; }
  void set_buzzer(bool b) override { (void)b; }
  void log(const char* m) override { (void)m; ++log_lines; }

  bool nv_read(u8 key, void* buf, u32 len) override {
    if (key == NvsKey::kNvsCalibration && cal_store_open_) {
      std::memcpy(buf, &cal_store, len < sizeof(cal_store) ? len : sizeof(cal_store));
      return true;
    }
    return false;
  }
  bool nv_write(u8 key, const void* buf, u32 len) override {
    if (key == NvsKey::kNvsCalibration) {
      std::memcpy(&cal_store, buf, len < sizeof(cal_store) ? len : sizeof(cal_store));
      cal_store_open_ = true;
      return true;
    }
    return true;
  }
  double microstep_acc_ = 0.0;
  Calibration cal_store{};
  bool cal_store_open_ = false;

  // Advance the simulated world by dt seconds, then run one control tick.
  void step(float dt_s) {
    now_us_ += static_cast<us_t>(dt_s * 1e6f);

    // Control first, then physics responds to the rate this tick chose.
    // Running physics first makes the mechanism lag its own command by a
    // tick, and on a long ramp that residue reads as real position error.
    ctl.tick(dt_s);

    // 1. physics: commanded rate moves the *truth*, with losses.
    // Accumulate whole microsteps rather than integrating the rate, because
    // that is what a stepper does: a driver cannot emit a fraction of a step,
    // so at low rates the real mechanism advances in discrete lurches. A fake
    // bench that integrates analytically would disagree with the controller's
    // own integer model by up to a step every tick, and that residue looks
    // exactly like a stall. Same reason the controller quantises once.
    const AxisProfile& a = machine.axis[0];
    microstep_acc_ += static_cast<double>(last_rate_steps_s) * dt_s;
    steps_t whole = static_cast<steps_t>(microstep_acc_);
    if (microstep_acc_ < 0 && static_cast<double>(whole) > microstep_acc_) --whole;
    microstep_acc_ -= static_cast<double>(whole);
    // Distance actually moved this tick (integer microsteps), and the rate it
    // corresponds to. Fault-injection knobs compare against the RATE, so keep
    // both: comparing a per-tick distance to a mm/s threshold silently disables
    // the knob, which is worse than not having it because it tests nothing.
    const float applied_mm = steps_to_mm_pure_(whole, dt_s);
    const float mm_s = (dt_s > 0.0f) ? applied_mm / dt_s : 0.0f;
    float applied = applied_mm;

    const bool binding = true_mm <= binding_low_mm || true_mm >= binding_high_mm;
    if (binding && std::fabs(mm_s) > 0.001f) {
      applied = applied_mm * 0.05f;           // 95% of effort goes nowhere
    }
    if (std::fabs(mm_s) > slip_threshold_mm_s) {
      applied -= applied_mm * slip_rate;      // gears skip teeth
    }
    if (obstacle && applied < 0.0f) {
      applied = (true_mm > obstacle_stop_mm_) ? applied : 0.0f;
      if (true_mm <= obstacle_stop_mm_) applied = 0.0f;
    }
    true_mm += applied;

    // Hard stops are absolute, and sit exactly at the travel limits so that a
    // homed offset lands on the same number the ruler says. Leaving the stop
    // clears the switch; that is the behaviour the controller relies on.
    if (true_mm <= machine.axis[0].travel_low_mm) {
      true_mm = machine.axis[0].travel_low_mm;
      if (applied <= 0.0f) at_low = true;
    } else {
      at_low = false;
    }
    if (true_mm >= machine.axis[0].travel_high_mm) {
      true_mm = machine.axis[0].travel_high_mm;
      if (applied >= 0.0f) at_high = true;
    } else {
      at_high = false;
    }

    // Second leg follows its own rate, scaled by drag: this is what makes the
    // legs drift apart and gives the equaliser something to correct.
    if (machine.axis_count > 1) {
      true_mm_b += applied_mm * leg2_drag;
      if (true_mm_b < machine.axis[0].travel_low_mm)
        true_mm_b = machine.axis[0].travel_low_mm;
      if (true_mm_b > machine.axis[0].travel_high_mm)
        true_mm_b = machine.axis[0].travel_high_mm;
    }

    // encoder counts follow truth
    true_counts = counts_for_(true_mm);
  }

  // Move the mechanism to an absolute height while keeping every piece of
  // simulated truth consistent. Assigning true_mm directly leaves the encoder
  // counts describing the OLD position, so the first tick after a teleport
  // reads a height the bench is not at - which looks exactly like a controller
  // bug and is only a test bug. Use this for power-cycle scenarios.
  void teleport(mm_t mm) {
    true_mm = mm;
    true_mm_b = mm;
    true_counts = counts_for_(mm);
    microstep_acc_ = 0.0;
  }

  // Human shoves the top: no motor involved, position silently wrong.
  void push_by_hand(mm_t delta_mm) {
    true_mm += delta_mm;
    true_counts = counts_for_(true_mm);
  }

  // The two legs of a real desk can also diverge from a hand push on one side.
  void push_corner(mm_t delta_a, mm_t delta_b) {
    true_mm += delta_a;
    true_mm_b += delta_b;
    true_counts = counts_for_(true_mm);
  }

  void setup(const MachineProfile& m) {
    machine = m;
    cal = Calibration{};
    ctl.init(this, {&machine, &cal});
    true_counts = counts_for_(true_mm);
  }

  void run(us_t microseconds, float dt_s = 0.002f) {
    for (us_t t = 0; t < microseconds; t += static_cast<us_t>(dt_s * 1e6f)) step(dt_s);
  }

  float obstacle_stop_mm_ = 750.0f;

 private:
  // Distance travelled this tick from an integer count of microsteps.
  float steps_to_mm_pure_(steps_t steps, float) const {
    return machine.steps_to_mm(machine.axis[0], steps);
  }

  static float mmlIKELY_(rate_t steps_s, const AxisProfile& a) {
    // inverse of Controller::command_axis
    if (steps_s == 0.0f) return 0.0f;
    const float revs_s = static_cast<float>(steps_s) /
        (static_cast<float>(a.steps_per_motor_rev) * a.gear_ratio);
    return revs_s * a.mm_per_shaft_rev;
  }
  // Counts are referenced to the low hard stop, which is where the controller
  // anchors the encoder during homing; below the stop they go negative.
  i64 counts_for_(mm_t mm) const {
    const AxisProfile& a = machine.axis[0];
    const double per_mm = static_cast<double>(a.encoder_counts_per_motor_rev) *
                          a.gear_ratio / a.mm_per_shaft_rev;
    return static_cast<i64>(std::llround((static_cast<double>(mm) -
                                          static_cast<double>(a.travel_low_mm)) * per_mm));
  }
};

}  // namespace desk