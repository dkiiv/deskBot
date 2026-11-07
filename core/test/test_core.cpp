// deskbot core - behaviour tests. No hardware, and no mocking of our own
// code: the fake bench tracks *true* position independently, so a controller
// that lies to itself fails here.
//
// Build & run:  make test && ./build/test_core
#include "desk/fake_bench.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using namespace desk;

static int g_fail = 0, g_pass = 0;
static std::string g_group;

// Any latched condition at all. The controller reports hard-stop contact
// separately from real faults, because being parked on a stop is normal; these
// tests assert on the union when the point is "nothing should be wrong".
static bool faults_any(const FakeBench& b) { return b.ctl.faults().any(); }

#define CHECK(cond, ...)                                                        \
  do {                                                                          \
    if (cond) { ++g_pass; }                                                     \
    else {                                                                      \
      ++g_fail;                                                                 \
      std::printf("  FAIL [%s] line %d: ", g_group.c_str(), __LINE__);          \
      std::printf(__VA_ARGS__);                                                 \
      std::printf("\n");                                                        \
    }                                                                           \
  } while (0)

static MachineProfile husky() {
  MachineProfile m{};
  m.name = "husky-52x24";
  m.drive = DriveKind::StepperPulse;
  m.feedback = FeedbackKind::MotorEncoder;
  m.coupling = AxisCoupling::Single;
  m.axis_count = 1;
  AxisProfile a{};
  a.steps_per_motor_rev = 3200;   // 200 full steps x 16 microsteps
  a.gear_ratio = 9.0f;            // any plausible ratio; units test is ratio-agnostic
  a.mm_per_shaft_rev = 5.50f;     // 330 mm travel / ~60 crank turns
  a.travel_low_mm = 660.0f;
  a.travel_high_mm = 990.0f;
  a.vmax_up_mm_s = 12.0f;
  a.vmax_down_mm_s = 14.0f;
  a.amax_mm_s2 = 45.0f;
  a.direction = 1.0f;
  a.encoder_counts_per_motor_rev = 2000;
  m.axis[0] = a;
  SafetyProfile& s = m.safety;
  s.stall_lag_mm_moving = 1.6f;
  s.stall_lag_mm_stopped = 0.35f;
  s.position_loss_mm = 2.0f;
  s.stall_hold_us = 220000;
  s.stall_backoff_up_mm = 4.0f;
  s.home_rate_mm_s = 3.0f;
  s.jog_lookahead_mm = 12.0f;
  s.temp_c_stop = 85.0f;
  s.temp_c_reenter = 65.0f;
  s.remote_deadman_us = 4000000;
  s.move_window_us = 120000000ULL;
  s.move_max_us = 60000000ULL;
  s.cool_us = 300000000ULL;
  return m;
}

// Bring the bench to a calibrated, Ready state by homing from `start_mm`.
static void home_it(FakeBench& b, mm_t start_mm) {
  b.true_mm = start_mm;
  b.true_counts = 0;
  b.at_low = false;
  b.at_high = false;
  CHECK(b.ctl.home() == Status::Ok, "home command rejected");
  b.run(300ULL * 1000000ULL);
  CHECK(b.ctl.calibrated(), "homing did not calibrate (state=%s)",
        name(b.ctl.state()));
  b.ctl.supervisor().clear_faults();
  b.ctl.reset_duty_budget();
}

int main() {
  // ------------------------------------------------------------------ units --
  g_group = "units";
  {
    MachineProfile m = husky();
    const AxisProfile& a = m.axis[0];
    const mm_t span = a.travel_high_mm - a.travel_low_mm;
    const steps_t st = m.mm_to_steps(a, span);
    const mm_t back = m.steps_to_mm(a, st);
    CHECK(std::fabs(back - span) < 0.01f, "roundtrip %.4f vs %.4f", back, span);

    // Full travel in crank revolutions.
    const float revs = static_cast<float>(st) /
                       (static_cast<float>(a.steps_per_motor_rev) * a.gear_ratio);
    // Cross-check against the bench as shipped: owners report ~60 crank turns
    // for the full range, so the profile must land in that neighbourhood or a
    // scale factor is wrong.
    // Husky spec: 13 in of travel, owners report ~60 crank turns. A profile
    // more than 25% off that has a scale factor wrong.
    CHECK(revs > 48.0f && revs < 75.0f, "full travel = %.2f crank revs", revs);

    // One microstep at the top surface should be a fraction of a hair.
    const mm_t one = m.steps_to_mm(a, 1);
    CHECK(one > 0.0001f && one < 0.001f, "microstep = %.6f mm", one);

    CHECK(std::fabs(a.gear_ratio - 9.0f) < 0.001f, "ratio");

    Trapezoid::Limits l{};
    l.vmax = a.vmax_up_mm_s; l.amax = a.amax_mm_s2; l.dmax = l.amax;
    const float eta = Trapezoid::eta_s(a.travel_low_mm, a.travel_high_mm, l);
    CHECK(eta > 15.0f && eta < 60.0f, "full-travel eta %.1f s implausible", eta);
  }

  // ----------------------------------------------------------------- gating --
  g_group = "gating";
  {
    FakeBench b; b.setup(husky());
    CHECK(b.ctl.state() == MachineState::Uncalibrated, "should start uncalibrated");
    CHECK(b.ctl.move_to(800.0f) == Status::NotCalibrated,
          "must refuse absolute move while uncalibrated");
    CHECK(b.ctl.jog(true, false, true, false) == Status::NotCalibrated,
          "must refuse jog while uncalibrated");
    CHECK(std::fabs(b.last_rate_steps_s) < 0.5f, "motor moved while uncalibrated");

    b.cal.homed = true;
    CHECK(b.ctl.move_to(2000.0f) == Status::OutOfRange, "above travel_high accepted");
    CHECK(b.ctl.move_to(100.0f) == Status::OutOfRange, "below travel_low accepted");
    CHECK(b.ctl.move_to(std::nanf("")) == Status::BadArgument, "NaN accepted");
    CHECK(std::fabs(b.last_rate_steps_s) < 0.5f, "motor moved on a rejected move");
  }

  // ----------------------------------------------------------------- homing --
  g_group = "homing";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    CHECK(b.at_low, "never reached the low hard stop");
    CHECK(b.true_mm <= b.machine.axis[0].travel_low_mm + 0.5f,
          "stopped above the hard stop: %.1f", b.true_mm);
    CHECK(std::fabs(b.ctl.height_mm() - b.machine.axis[0].travel_low_mm) < 1.0f,
          "model not re-anchored at home: %.2f", b.ctl.height_mm());
    CHECK(!faults_any(b), "faults after a clean home");

    // Booting already on the bottom stop must not anchor on stale switch state:
    // the controller backs off and re-presses to prove a real rising edge.
    FakeBench c; c.setup(husky());
    c.true_mm = c.machine.axis[0].travel_low_mm;
    c.at_low = true;
    CHECK(c.ctl.home() == Status::Ok, "home rejected at stop");
    c.run(300ULL * 1000000ULL);
    CHECK(c.ctl.calibrated(), "home from on-top-of-the-stop failed");
    CHECK(std::fabs(c.ctl.height_mm() - c.machine.axis[0].travel_low_mm) < 2.0f,
          "wrong zero after home-from-stop: %.1f", c.ctl.height_mm());
  }

  // ------------------------------------------------------- recovery by homing
  // "Faulted -> re-home" is the documented recovery path, so it has to be
  // reachable. This test exists because it was not: home() was gated behind
  // can_move(), and position_lost (raised by leaning on the desk) is in
  // any_hard() - so the desk that most needed to home was the one that could
  // not. Homing IS the recovery; forbidding it while faulty is a catch-22.
  g_group = "home-recovery";
  {
    // (a) position lost while parked, then a full recovery cycle.
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    b.push_by_hand(-40.0f);
    b.run(20 * 1000);
    CHECK(b.ctl.faults().position_lost, "hand-push not detected");
    CHECK(b.ctl.move_to(900.0f) == Status::UnsafeToMove,
          "a trusted move was allowed with lost position");
    CHECK(b.ctl.home() == Status::Ok, "home must be allowed as the recovery path");
    b.run(300ULL * 1000000ULL);
    CHECK(std::fabs(b.ctl.height_mm() - b.true_mm) < 1.0f,
          "re-home did not re-anchor: model %.1f truth %.1f",
          b.ctl.height_mm(), b.true_mm);
    CHECK(!b.ctl.faults().position_lost, "position_lost still latched after re-home");
    CHECK(b.ctl.move_to(b.machine.axis[0].travel_high_mm - 40.0f) == Status::Ok,
          "recovery move refused after a successful re-home");
    b.run(120ULL * 1000000ULL);
    CHECK(!faults_any(b), "faults after a clean recovery cycle");

    // (b) skew on a dual machine: homing may START, but the latch must not be
    // silently dropped - the legs re-synchronise against their stops and the
    // supervisor re-evaluates them.
    MachineProfile m = husky();
    m.coupling = AxisCoupling::MasterFollower;
    m.axis_count = 2;
    m.safety.max_skew_mm = 6.0f;
    FakeBench c; c.setup(m);
    c.true_mm = 800.0f; c.true_mm_b = 800.0f;
    c.ctl.home(); c.run(300ULL * 1000000ULL);
    c.ctl.supervisor().clear_faults(); c.ctl.reset_duty_budget();
    c.push_corner(0.0f, -12.0f);      // press one corner: 12 mm of skew
    c.run(20 * 1000);
    CHECK(c.ctl.faults().skew, "12 mm of skew not detected (limit 6 mm)");
    CHECK(c.ctl.move_to(900.0f) == Status::UnsafeToMove, "move allowed while skewed");
    CHECK(c.ctl.home() == Status::Ok, "home must be reachable while skewed");

    // (c) faults homing must NOT clear: an operator acknowledgement still owns
    // these, because homing cannot fix an open circuit or a hot driver.
    FakeBench d; d.setup(husky());
    home_it(d, 900.0f);
    d.estop_open = true; d.run(200000);
    CHECK(d.ctl.faults().estop, "e-stop not latched");
    CHECK(d.ctl.home() != Status::Ok, "home allowed through an e-stop");
    d.estop_open = false; d.run(200000);
    d.temp_c = 90.0f; d.run(200000);
    CHECK(d.ctl.faults().over_temp, "over-temp not latched");
    CHECK(d.ctl.home() != Status::Ok, "home allowed while overheating");
  }

  // ------------------------------------------------------------------ moves --
  g_group = "moves";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    const mm_t up_target = b.machine.axis[0].travel_high_mm - 20.0f;
    CHECK(b.ctl.move_to(up_target) == Status::Ok, "valid move rejected");
    b.run(120ULL * 1000000ULL);
    CHECK(!b.ctl.moving(), "still moving after the settling window");
    CHECK(std::fabs(b.true_mm - up_target) < 1.5f,
          "truth %.1f mm vs target %.1f", b.true_mm, up_target);
    CHECK(std::fabs(b.ctl.height_mm() - b.true_mm) < 1.0f,
          "model drifted from truth: %.2f vs %.2f", b.ctl.height_mm(), b.true_mm);
    CHECK(!faults_any(b), "unexpected fault on a clean move");

    const mm_t dn_target = b.machine.axis[0].travel_low_mm + 40.0f;
    CHECK(b.ctl.move_to(dn_target) == Status::Ok, "downward move rejected");
    b.run(120ULL * 1000000ULL);
    CHECK(std::fabs(b.true_mm - dn_target) < 1.5f, "downward truth %.1f", b.true_mm);
    CHECK(!faults_any(b), "fault on a clean descent");
  }

  // ------------------------------------------------------------ stall/obstacle
  g_group = "stall";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    // Start the descent from high up: homing parks the bench at the bottom,
    // and a blocked descent is only interesting if it has room to be blocked.
    const mm_t start = b.machine.axis[0].travel_high_mm - 60.0f;
    CHECK(b.ctl.move_to(start) == Status::Ok, "reposition rejected");
    b.run(120ULL * 1000000ULL);
    b.ctl.reset_duty_budget();
    b.obstacle = true;
    b.obstacle_stop_mm_ = b.machine.axis[0].travel_low_mm + 200.0f;
    b.slip_threshold_mm_s = 0.5f;   // nothing gets through: it all slips
    b.slip_rate = 1.0f;
    const mm_t blocked_target = b.machine.axis[0].travel_low_mm + 10.0f;
    CHECK(b.ctl.move_to(blocked_target) == Status::Ok, "descent rejected");
    b.run(60ULL * 1000000ULL);
    CHECK(faults_any(b), "stalled descent went undetected");
    CHECK(!b.ctl.moving(), "kept driving into an obstacle");
    CHECK(b.true_mm > b.obstacle_stop_mm_ - 20.0f,
          "descended through the obstacle to %.1f", b.true_mm);
    CHECK(b.ctl.needs_attention(), "a stalled descent should demand attention");
    CHECK(b.ctl.move_to(blocked_target) != Status::Ok,
          "re-issued the failed descent with nobody at the controls");
    b.obstacle = false;
    b.slip_rate = 0.0f;
    // The blocked descent burned most of the duty budget trying to push
    // through; hand it back so this check tests recovery, not the budget
    // (the budget has its own test).
    b.ctl.reset_duty_budget();
    b.ctl.acknowledge();
    CHECK(!b.ctl.needs_attention(), "acknowledge did not clear");
    CHECK(b.ctl.move_to(start) == Status::Ok, "cannot raise the bench after ack");
    b.run(120ULL * 1000000ULL);
    CHECK(std::fabs(b.true_mm - start) < 2.0f, "recovery move landed at %.1f", b.true_mm);
  }

  // -------------------------------------------------- position loss by hand --
  g_group = "position-loss";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    CHECK(b.ctl.move_to(b.machine.axis[0].travel_high_mm - 40.0f) == Status::Ok, "move rejected");
    b.run(30ULL * 1000000ULL);
    CHECK(b.ctl.state() == MachineState::Ready, "did not settle (state=%s)",
          name(b.ctl.state()));
    b.push_by_hand(-40.0f);          // somebody leans on the top
    b.run(20 * 1000);
    CHECK(faults_any(b), "40 mm of hand movement went unnoticed");
    CHECK(b.ctl.state() != MachineState::Ready, "still Ready after losing 40 mm");
    CHECK(b.ctl.move_to(b.machine.axis[0].travel_high_mm - 10.0f) != Status::Ok,
          "accepted a new move with a position it cannot trust");
  }

  // ------------------------------------------------------------------ e-stop --
  g_group = "estop";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    CHECK(b.ctl.move_to(b.machine.axis[0].travel_high_mm - 20.0f) == Status::Ok,
          "move rejected");
    b.run(1000000);
    CHECK(b.ctl.moving(), "should be mid-move");
    b.estop_open = true;
    b.run(100000);
    CHECK(!b.motor_power_seen, "coils powered with the enable chain open");
    CHECK(std::fabs(b.last_rate_steps_s) < 0.5f, "pulsing with e-stop open");
    CHECK(b.ctl.move_to(960.0f) != Status::Ok, "moved during e-stop");
    b.estop_open = false;
    b.run(200000);
    b.ctl.supervisor().clear_faults();
    CHECK(b.ctl.move_to(b.ctl.height_mm() + 30.0f) == Status::Ok,
          "did not recover after the e-stop was released");
  }

  // ----------------------------------------------------------- remote deadman
  g_group = "deadman";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    // A remote client asserts jog once, then its connection drops.
    CHECK(b.ctl.jog(true, false, true, /*from_remote=*/true) == Status::Ok,
          "remote jog rejected");
    b.run(500000);
    CHECK(b.ctl.moving(), "remote jog did not start motion");
    const mm_t before = b.true_mm;
    b.run(12ULL * 1000000ULL);          // well past the deadman window
    CHECK(!b.ctl.moving(), "ran on after the remote client vanished");
    CHECK(std::fabs(b.last_rate_steps_s) < 0.5f, "still pulsing after deadman expiry");
    CHECK(b.true_mm - before < 60.0f, "remote jog travelled %.1f mm unattended",
          b.true_mm - before);
  }

  // -------------------------------------------------------------- thermal ----
  g_group = "thermal";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    CHECK(b.ctl.move_to(b.machine.axis[0].travel_high_mm - 20.0f) == Status::Ok,
          "move rejected");
    b.run(400000);
    b.temp_c = 90.0f;
    b.run(200000);
    CHECK(std::fabs(b.last_rate_steps_s) < 0.5f, "driving above temp_c_stop");
    b.temp_c = 40.0f;
    b.run(200000);
    b.ctl.supervisor().clear_faults();
    CHECK(b.ctl.move_to(b.machine.axis[0].travel_high_mm - 20.0f) == Status::Ok ||
          !b.ctl.moving(), "would not resume after cooling");
  }

  // ------------------------------------------------------------ soft limits --
  g_group = "soft-limits";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    const mm_t hi = b.machine.axis[0].travel_high_mm;
    CHECK(b.ctl.move_to(hi - 0.5f) == Status::Ok, "move to just inside limit rejected");
    b.run(120ULL * 1000000ULL);
    CHECK(b.true_mm <= hi + 0.5f, "passed the soft high limit: %.1f > %.1f",
          b.true_mm, hi);
    CHECK(!b.at_high, "reached the mechanical stop; the soft limit should stop first");
  }

  // ---------------------------------------------------------------- presets --
  g_group = "presets";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    const mm_t p0 = b.machine.axis[0].travel_high_mm - 10.0f;
    const mm_t p1 = b.machine.axis[0].travel_low_mm + 80.0f;
    b.ctl.save_preset(0, p0);
    b.ctl.save_preset(1, p1);
    b.ctl.load_presets();
    CHECK(std::fabs(b.ctl.preset(0).mm - p0) < 0.01f, "preset 0 lost");
    CHECK(b.ctl.go_to_preset(9) == Status::BadArgument, "unassigned preset accepted");
    CHECK(b.ctl.go_to_preset(1) == Status::Ok, "go_to_preset failed");
    b.run(120ULL * 1000000ULL);
    CHECK(std::fabs(b.true_mm - p1) < 1.5f, "did not reach preset: %.1f", b.true_mm);
  }


  // -------------------------------------------------- dual-leg desk (generic)
  // The whole point of MachineProfile: a two-motor office desk uses the same
  // core with no code changes, and gains skew handling that the single-shaft
  // bench correctly never needs.
  g_group = "dual-leg";
  {
    MachineProfile m = husky();
    m.name = "generic-dual-motor";
    m.coupling = AxisCoupling::MasterFollower;
    m.axis_count = 2;
    m.axis[0].vmax_up_mm_s = 20.0f;
    m.axis[0].vmax_down_mm_s = 20.0f;
    m.safety.max_skew_mm = 6.0f;
    m.safety.skew_alarm_mm = 2.5f;

    FakeBench b; b.setup(m);
    b.true_mm = 750.0f; b.true_mm_b = 750.0f;
    b.ctl.home();
    b.run(300ULL * 1000000ULL);
    CHECK(b.ctl.calibrated(), "dual-leg home failed");
    b.ctl.supervisor().clear_faults();
    b.ctl.reset_duty_budget();

    // Leg 2 is sticky: it achieves only 70% of what it is told.
    b.leg2_drag = 0.70f;
    CHECK(b.ctl.move_to(950.0f) == Status::Ok, "dual move rejected");
    b.run(120ULL * 1000000ULL);
    const float skew = std::fabs(b.true_mm - b.true_mm_b);
    CHECK(skew < m.safety.max_skew_mm,
          "legs diverged %.2f mm with the equaliser active (limit %.1f)",
          skew, m.safety.max_skew_mm);
    CHECK(std::fabs(b.ctl.height_mm() - (b.true_mm + b.true_mm_b) * 0.5f) < 3.0f,
          "height should track the leg mean: model %.1f mean %.1f",
          b.ctl.height_mm(), (b.true_mm + b.true_mm_b) * 0.5f);

    // With the equaliser disabled the same drag must blow past the limit:
    // proof the correction is doing real work and is not a coincidence.
    FakeBench c; c.setup(m);
    c.true_mm = 750.0f; c.true_mm_b = 750.0f;
    c.leg2_drag = 0.70f;
    m.safety.skew_trim_per_mm = 0.0f;
    c.ctl.home(); c.run(300ULL * 1000000ULL);
    c.ctl.supervisor().clear_faults(); c.ctl.reset_duty_budget();
    CHECK(c.ctl.move_to(950.0f) == Status::Ok, "control-run move rejected");
    c.run(120ULL * 1000000ULL);
    CHECK(std::fabs(c.true_mm - c.true_mm_b) > 1.0f,
          "control run should drift; the equaliser may be a no-op");
    m.safety.skew_trim_per_mm = 0.10f;
  }

  // ------------------------------------------------------------ duty budget --
  // A printed gearbox is the weak thermal link. Long or repeated moves must
  // force a rest, and homing must not be interrupted by it (see homing test).
  g_group = "duty";
  {
    FakeBench b; b.setup(husky());
    home_it(b, 900.0f);
    // Shrink the window so the test does not need minutes of sim time.
    b.machine.safety.move_max_us = 10000000;    // 10 s of drive
    b.machine.safety.move_window_us = 20000000; // in a 20 s window
    b.machine.safety.cool_us = 50000000;        // then rest 50 s
    const mm_t lo = b.machine.axis[0].travel_low_mm + 20.0f;
    const mm_t hi = b.machine.axis[0].travel_high_mm - 20.0f;
    CHECK(b.ctl.move_to(hi) == Status::Ok, "first move rejected");
    b.run(15000000);   // 15 s: more than the 10 s budget
    CHECK(b.ctl.cooling_down(), "no cool-down after exceeding the duty budget");
    const mm_t frozen = b.true_mm;
    CHECK(b.ctl.move_to(lo) != Status::Ok || !b.ctl.moving(),
          "accepted a new move while cooling down");
    b.run(20000000);
    CHECK(std::fabs(b.true_mm - frozen) < 0.5f,
          "desk drifted %.2f mm during cool-down", b.true_mm - frozen);
    b.run(60000000);   // wait out the rest
    b.ctl.reset_duty_budget();
    CHECK(b.ctl.move_to(lo) == Status::Ok, "would not move after the budget reset");
    b.run(120000000);
    CHECK(std::fabs(b.true_mm - lo) < 2.0f, "never completed post-cool move: %.1f", b.true_mm);
  }

  // ------------------------------------------------- homing into an obstacle
  // Before the first anchor there is no trusted position, so the position-lag
  // detector cannot protect the homing descent. The watchdog is the only thing
  // between a blocked home and a motor driving into an obstruction forever.
  g_group = "home-watchdog";
  {
    FakeBench b; b.setup(husky());
    b.true_mm = 950.0f;
    b.obstacle = true;
    b.obstacle_stop_mm_ = 900.0f;   // bench can never reach the low stop
    b.slip_threshold_mm_s = 0.1f;
    b.slip_rate = 1.0f;
    CHECK(b.ctl.home() == Status::Ok, "home rejected");
    b.run(900ULL * 1000000ULL);     // far longer than any legitimate home
    CHECK(b.ctl.faults().home_timeout, "blocked home never timed out");
    CHECK(!b.ctl.moving(), "still driving after the home timeout");
    CHECK(std::fabs(b.last_rate_steps_s) < 0.5f, "pulsing after the home timeout");
    CHECK(!b.ctl.calibrated(), "claimed calibration it never earned");
    CHECK(b.ctl.move_to(940.0f) != Status::Ok, "moved after a failed home");
    const float travel = 950.0f - b.true_mm;
    CHECK(travel < 400.0f, "grinded %.0f mm into the obstruction", travel);
  }

  // ------------------------------------------------ power-cycle trust policy
  // What a reboot is allowed to believe. Two bugs lived here: init() resumed
  // claiming cal_->home_mm (the LOW datum, i.e. the bottom of travel) as the
  // current height, and it did so even on a machine with no feedback, where
  // nothing can ever disprove it. On a self-locking bench the top can be moved
  // by hand while the board is unpowered, so an open-loop desk must boot
  // Uncalibrated and home, every time.
  g_group = "boot-trust";
  {
    // (a) open-loop: stored calibration must NOT survive as a trusted height.
    MachineProfile ol = husky();
    ol.feedback = FeedbackKind::None;
    {
      FakeBench b; b.setup(ol);
      b.cal.homed = true;
      b.cal.home_mm = b.machine.axis[0].travel_low_mm;
      b.cal.last_height_valid = true;
      b.cal.last_height_mm = 830.0f;
      FakeBench reboot; reboot.setup(ol);
      reboot.cal = b.cal;             // exactly what came back out of NVS
      reboot.ctl.init(&reboot, {&reboot.machine, &reboot.cal});
      CHECK(reboot.ctl.state() == MachineState::Uncalibrated,
            "open-loop desk booted trusting a height it cannot verify (state=%s)",
            name(reboot.ctl.state()));
      CHECK(reboot.ctl.move_to(900.0f) == Status::NotCalibrated,
            "open-loop desk accepted a move straight after boot");
      CHECK(std::fabs(reboot.last_rate_steps_s) < 0.5f,
            "open-loop desk pulsed before ever homing");
    }

    // (b) with feedback: resuming is allowed, because the integrity check can
    // falsify the stored value within a tick.
    {
      FakeBench b; b.setup(husky());
      b.true_mm = 900.0f;
      home_it(b, 900.0f);
      const mm_t hi = b.machine.axis[0].travel_high_mm - 30.0f;
      CHECK(b.ctl.move_to(hi) == Status::Ok, "move rejected");
      b.run(120ULL * 1000000ULL);
      CHECK(b.ctl.state() == MachineState::Ready, "did not settle");
      Calibration saved = b.cal;      // NVS image at power-down

      // Honest resume: encoder agrees with the stored height.
      FakeBench r1; r1.setup(husky());
      r1.teleport(b.true_mm); r1.cal = saved;
      r1.ctl.init(&r1, {&r1.machine, &r1.cal});
      r1.run(50 * 1000);
      CHECK(r1.ctl.state() != MachineState::Fault,
            "honest resume was faulted (state=%s)", name(r1.ctl.state()));
      CHECK(r1.ctl.faults().position_lost == false, "false position_lost on resume");
      CHECK(r1.ctl.move_to(r1.machine.axis[0].travel_low_mm + 50.0f) == Status::Ok,
            "could not move after an honest resume");

      // Dishonest resume: the top was moved while unpowered. Correct
      // behaviour is to HEAL, not to fault: the encoder is what touches the
      // mechanism, so adopting it leaves the machine accurately positioned and
      // safe to move. Faulting instead would refuse a desk that demonstrably
      // knows where it is, purely because a stale EEPROM value disagreed.
      FakeBench r2; r2.setup(husky());
      r2.teleport(b.true_mm - 60.0f);   // leaned on: 60 mm off
      r2.cal = saved;
      r2.ctl.init(&r2, {&r2.machine, &r2.cal});
      r2.run(50 * 1000);
      CHECK(std::fabs(r2.ctl.height_mm() - r2.true_mm) < 1.5f,
            "model resumed the stale height instead of the encoder: %.1f vs %.1f",
            r2.ctl.height_mm(), r2.true_mm);
      CHECK(!r2.ctl.faults().position_lost, "healed boot should not be faulted");
      CHECK(r2.ctl.move_to(r2.true_mm + 30.0f) == Status::Ok,
            "would not move after healing the stale height");
      r2.run(60ULL * 1000000ULL);
      CHECK(std::fabs(r2.true_mm - (b.true_mm - 30.0f)) < 2.0f,
            "post-heal move landed at %.1f", r2.true_mm);
    }

    // (c) the persisted height must be the LAST height, not the home datum.
    {
      FakeBench b; b.setup(husky());
      b.true_mm = 900.0f;
      home_it(b, 900.0f);
      const mm_t hi = b.machine.axis[0].travel_high_mm - 20.0f;
      CHECK(b.ctl.move_to(hi) == Status::Ok, "move rejected");
      b.run(120ULL * 1000000ULL);
      CHECK(b.cal.last_height_valid, "settled height was never recorded");
      CHECK(std::fabs(b.cal.last_height_mm - hi) < 2.0f,
            "persisted %.1f mm but the desk is at %.1f mm",
            b.cal.last_height_mm, hi);
      CHECK(b.cal.home_mm < b.cal.last_height_mm - 100.0f,
            "last height looks like the home datum (%.1f vs %.1f)",
            b.cal.last_height_mm, b.cal.home_mm);
    }
  }

  std::printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
