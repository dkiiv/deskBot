// deskbot core - the motion controller. Pure logic: no hardware, no threads,
// no exceptions, no dynamic allocation.
//
// One function, tick(dt_s), is the whole machine. Call it at a fixed rate
// (2 ms on the ESP32, anything at all in simulation). Everything else -
// buttons, Wi-Fi, CLI - only sets a target and reads state.
//
// Three decisions worth defending:
//
//  1. Commanded position is integrated from the commanded *rate*, never from a
//     step counter. If the driver misses pulses, model and hardware stay
//     consistent with each other, which is what lets the encoder detect the lie.
//
//  2. Refuse while uncalibrated. A desk that does not know where its bottom
//     hard stop is has no business trusting a soft top limit, so the only
//     motion allowed before homing is the slow limit hunt itself.
//
//  3. A retrofit loses steps. Slipped gears, a dry spot in the worm, somebody
//     leaning on the top: all make commanded and actual diverge silently. So
//     divergence is its own fault class, and after it, position is untrusted
//     until re-homed. On a self-locking worm the desk never runs away - it just
//     quietly stops being where you think - which is exactly when soft limits
//     must stop being trusted.
//
// Order within one tick:
//   inputs -> feedback -> integrity -> supervisor -> fault response (e-stop)
//   -> retreat -> deadman -> button priority -> limit direction blocking
//   -> duty budget -> homing | normal move
#pragma once
#include "desk/hal.h"
#include "desk/machine_profile.h"
#include "desk/safety.h"
#include "desk/trapezoid.h"

namespace desk {

enum class Command : u8 {
  Stop = 0,        // decelerate to a halt here
  GoTo,            // absolute millimetres
  JogUp, JogDown,  // held: target is a refreshed lookahead
  Home,            // hunt the low hard stop and anchor there
};

struct Preset {
  mm_t mm = 0.0f;
  bool used = false;
};

class Controller {
 public:
  struct Config {
    const MachineProfile* machine = nullptr;
    Calibration* cal = nullptr;
  };

  void init(Hal* hal, const Config& cfg) {
    hal_ = hal;
    m_ = cfg.machine;
    cal_ = cfg.cal;
    cmd_ = Command::Stop;
    vel_mm_s_ = 0.0f;
    cmd_mm_[0] = cmd_mm_[1] = 0.0f;
    enc_mm_[0] = enc_mm_[1] = 0.0f;
    cmd_steps_[0] = cmd_steps_[1] = 0;
    step_carry_ = 0.0;
    retreat_dir_ = 0;
    retreat_left_mm_ = 0.0f;
    home_phase_ = HomePhase::Init;
    if (cal_ && cal_->homed) {
      // Resume from the stored datum so a power cut mid-travel does not
      // forget the height it was at; still re-home if the profile changed.
      cmd_mm_[0] = cal_->home_mm;
      cmd_steps_[0] = 0;
    }
    state_ = (cal_ && cal_->homed) ? MachineState::Ready : MachineState::Uncalibrated;
    target_mm_ = cmd_mm_[0];
  }

  // ---------------------------------------------------------------- commands
  Status move_to(mm_t target_mm) {
    if (!m_ || !cal_) return Status::BadArgument;
    if (!cal_->homed) return Status::NotCalibrated;
    if (!sup_.can_move()) return Status::UnsafeToMove;
    if (state_ == MachineState::Moving) return Status::Busy;
    if (!finite_(target_mm)) return Status::BadArgument;
    if (!m_->in_travel(target_mm)) return Status::OutOfRange;
    target_mm_ = target_mm;
    cmd_ = Command::GoTo;
    remote_deadline_us_ = 0;   // a committed absolute move is not remote-gated
    return Status::Ok;
  }

  Status go_to_preset(u8 idx) {
    if (idx >= kMaxPresets || !preset_[idx].used) return Status::BadArgument;
    return move_to(preset_[idx].mm);
  }

  // Held-button or held-remote jogging. `asserted` must be refreshed every
  // tick by remote callers so the deadman can expire; a button driver passes
  // true for as long as the input is physically held.
  Status jog(bool up, bool down, bool asserted, bool from_remote) {
    if (!m_ || !cal_) return Status::BadArgument;
    if (!cal_->homed) return Status::NotCalibrated;
    if (!sup_.can_move()) return Status::UnsafeToMove;
    if (!up && !down) {
      if (cmd_ == Command::JogUp || cmd_ == Command::JogDown) stop();
      return Status::Ok;
    }
    if (from_remote) {
      if (!asserted) return Status::StaleCommand;
      remote_deadline_us_ = hal_->now_us() + m_->safety.remote_deadman_us;
    }
    cmd_ = up ? Command::JogUp : Command::JogDown;
    state_ = MachineState::Moving;
    return Status::Ok;
  }

  // Homing is the recovery path for a lost position model, so it must not be
  // gated behind can_move() the way a trusted move is - that would mean a desk
  // that lost its datum could never regain it (the latch that blocks movement
  // is the same latch only homing can clear).
  //
  // The latches homing is allowed to clear on a successful anchor are exactly
  // the ones homing can actually FIX: position_lost and skew are claims about
  // where the mechanism is, and an anchor against a proven hard stop settles
  // them. The rest are not, and homing cannot make them true: an e-stop chain
  // is still open, the driver is still hot, a stalled gearbox may still be
  // jammed, and a timeout already proved the path is blocked. Those stay
  // latched and still refuse the home outright.
  Status home() {
    if (!m_) return Status::BadArgument;
    Faults& f = sup_.faults();
    if (f.estop || f.over_temp || f.stall || f.blocked || f.home_timeout)
      return Status::UnsafeToMove;
    if (state_ == MachineState::Moving || state_ == MachineState::Retreating)
      return Status::Busy;
    // Recovery latches are dropped only for the duration of the hunt: the
    // supervisor re-evaluates both from live feedback, and if the machine
    // cannot actually re-synchronise the fault simply returns.
    f.position_lost = false;
    f.skew = false;
    cmd_ = Command::Home;
    home_phase_ = HomePhase::Init;
    home_deadline_us_ = 0;
    return Status::Ok;
  }

  void stop() {
    cmd_ = Command::Stop;
    target_mm_ = cmd_mm_[0];
  }

  void save_preset(u8 idx, mm_t mm) {
    if (idx >= kMaxPresets || !finite_(mm)) return;
    preset_[idx] = {mm, true};
    if (hal_) hal_->nv_write(NvsKey::kNvsPresets, preset_, sizeof(preset_));
  }

  // Returns true if saved presets were found. An empty first-boot EEPROM must
  // not wipe presets the caller already set, and must not look like an error
  // worth retrying - so the bool is advisory, not a Status.
  bool load_presets() {
    if (!hal_) return false;
    return hal_->nv_read(NvsKey::kNvsPresets, preset_, sizeof(preset_));
  }

  // Operator acknowledgement: the only way out of a latched stall. Deliberately
  // explicit rather than automatic - see finish_retreat(). The position model
  // was already re-anchored during the retreat, so moving up afterwards is
  // safe, and being able to raise the bench is usually how you free whatever
  // was under it.
  void acknowledge() {
    sup_.clear_stall();
    sup_.faults().home_timeout = false;
    home_deadline_us_ = 0;
    if (state_ == MachineState::Protect && sup_.can_move())
      state_ = calibrated() ? MachineState::Ready : MachineState::Uncalibrated;
  }

  // After a duty-budget cool-down the operator can hand the budget back.
  void reset_duty_budget() {
    run_us_ = 0;
    window_start_us_ = 0;
    window_open_ = false;
    move_cool_until_us_ = 0;
  }

  // ------------------------------------------------------------------- tick
  void tick(float dt_s) {
    if (!m_ || !cal_ || dt_s <= 0.0f) return;
    Inputs in{};
    hal_->read_inputs(in);
    const us_t now = hal_->now_us();
    const AxisProfile& a = m_->axis[0];

    // --- feedback: encoder counts -> desk millimetres ----------------------
    have_enc_ = in.encoder_valid && m_->feedback != FeedbackKind::None;
    if (have_enc_) {
      enc_mm_[0] = cal_->counts_to_mm(a, *m_, in.encoder_counts[0]);
      if (m_->axis_count > 1)
        enc_mm_[1] = cal_->counts_to_mm(m_->axis[1], *m_, in.encoder_counts[1]);
    }

    // --- integrity: parked but reality disagrees => someone moved the top ---
    // Skipped while moving (lag there is a stall, handled differently) and
    // during a retreat (the mechanism is expected to lag on purpose).
    const bool parked = state_ == MachineState::Ready ||
                        state_ == MachineState::Settling ||
                        state_ == MachineState::Protect;
    if (have_enc_ && parked && cal_->homed &&
        std::fabs(enc_mm_[0] - cmd_mm_[0]) > m_->safety.position_loss_mm) {
      sup_.faults().position_lost = true;
      state_ = MachineState::Fault;
      hal_->log("integrity: position lost, re-home required");
    }

    // --- supervisor ---------------------------------------------------------
    // The enable chain is evaluated first inside the supervisor, so cutting
    // power here (not before it) still happens within this 2 ms tick. Doing it
    // before the supervisor instead would unpowered the motors but never latch
    // why, and move_to() consults the latches: an e-stop that silently permits
    // new commands is worse than a slow one.
    const bool retreating = (state_ == MachineState::Retreating);
    sup_.set_other_axis_mm(enc_mm_[1]);
    sup_.tick(*m_, in, cmd_mm_[0], vel_mm_s_, enc_mm_[0], have_enc_,
              sup_.faults(), now, retreating, cal_->homed,
              /*homing=*/cmd_ == Command::Home);
    const Faults& f = sup_.faults();

    // --- fault response -----------------------------------------------------
    if (f.estop || f.over_temp || f.position_lost || f.skew) {
      kill_all();
      was_low_ = in.limit_low[0];
      return;
    }
    if (retreating) {
      run_retreat(in, dt_s, now);
      was_low_ = in.limit_low[0];
      return;
    }
    if (f.stall) {
      enter_retreat(vel_mm_s_);
      was_low_ = in.limit_low[0];
      return;
    }
    if (!sup_.can_move()) {
      hal_->drive_rate(0, 0.0f);
      hal_->set_motor_power(false);
      state_ = MachineState::Fault;
      was_low_ = in.limit_low[0];
      return;
    }

    // --- remote deadman: a dropped client must not leave the desk running ---
    if (remote_deadline_us_ && now > remote_deadline_us_) {
      remote_deadline_us_ = 0;
      if (cmd_ == Command::JogUp || cmd_ == Command::JogDown) stop();
      hal_->log("deadman: remote control expired");
    }

    // --- physical buttons outrank any remote target ------------------------
    if (in.button_up || in.button_down) {
      const bool up = in.button_up && !in.button_down;
      jog(up, !up, true, false);
    } else if ((cmd_ == Command::JogUp || cmd_ == Command::JogDown) &&
               !remote_deadline_us_) {
      stop();
    }

    // A hard stop is a normal resting place, not a fault, so a pressed switch
    // only forbids driving further into it. Homing is exempt: it hunts a stop
    // deliberately and re-anchors on a verified rising edge.
    const bool at_low = in.limit_low[0];
    const bool at_high = in.limit_high[0];
    if (cmd_ != Command::Home) {
      if (at_low && vel_mm_s_ < 0.0f) vel_mm_s_ = 0.0f;
      if (at_high && vel_mm_s_ > 0.0f) vel_mm_s_ = 0.0f;
    }

    // --- duty budget --------------------------------------------------------
    // Homing is deliberately exempt (see track_duty): refusing to finish a
    // home leaves the desk with no trusted position, which is a worse failure
    // than a warm gearbox. Full-travel homing at 3 mm/s over 400 mm takes
    // ~130 s, so a budget sized for normal moves would always abort it.
    if (cmd_ != Command::Home && move_cool_until_us_ && now < move_cool_until_us_) {
      hal_->drive_rate(0, 0.0f);
      hal_->set_motor_power(false);
      vel_mm_s_ = 0.0f;
      target_mm_ = cmd_mm_[0];
      cmd_ = Command::Stop;
      // Never report Ready when there is no position model to be ready with.
      state_ = cal_->homed ? MachineState::Ready : MachineState::Uncalibrated;
      was_low_ = at_low;
      return;
    }

    // --- homing -------------------------------------------------------------
    if (cmd_ == Command::Home) {
      run_homing(in, a, dt_s);
      last_us_ = now;   // keep the duty clock continuous, but bank no runtime
      was_low_ = in.limit_low[0];
      return;
    }

    // --- jogging: refresh a lookahead target so the ramp has somewhere to go
    if (cmd_ == Command::JogUp || cmd_ == Command::JogDown) {
      const mm_t dir = (cmd_ == Command::JogUp) ? 1.0f : -1.0f;
      target_mm_ = clamp_(cmd_mm_[0] + dir * m_->safety.jog_lookahead_mm,
                          a.travel_low_mm, a.travel_high_mm);
    }

    // --- normal move --------------------------------------------------------
    Trapezoid::Limits lim{};
    lim.vmax = m_->vmax_for(a, target_mm_, cmd_mm_[0]);
    lim.amax = a.amax_mm_s2;
    // Descending gets gentler deceleration: gravity is assisting and an
    // obstacle must be met slowly, not with momentum.
    lim.dmax = (target_mm_ < cmd_mm_[0]) ? a.amax_mm_s2 * 0.7f : a.amax_mm_s2;

    trapezoid_.plan(vel_mm_s_, cmd_mm_[0], target_mm_, lim, dt_s);

    // Soft limits clamp rather than fault, then we settle against them.
    if (cmd_mm_[0] <= a.travel_low_mm) {
      cmd_mm_[0] = a.travel_low_mm;
      if (vel_mm_s_ < 0) vel_mm_s_ = 0;
    }
    if (cmd_mm_[0] >= a.travel_high_mm) {
      cmd_mm_[0] = a.travel_high_mm;
      if (vel_mm_s_ > 0) vel_mm_s_ = 0;
    }

    if (Trapezoid::settled(vel_mm_s_, cmd_mm_[0], target_mm_, 0.15f, 0.02f)) {
      vel_mm_s_ = 0.0f;
      cmd_mm_[0] = target_mm_;
      cmd_ = Command::Stop;
      state_ = MachineState::Ready;
      sup_.begin_stop_verification(now);
      // Hold at reduced current: enough to stop creep on a loaded single-shaft
      // bench, without the heat of full run current parked for hours.
      hold_current_ = true;
    } else {
      state_ = MachineState::Moving;
      hold_current_ = false;
    }

    command_axis(vel_mm_s_, dt_s);
    track_duty(vel_mm_s_, now);
    was_low_ = in.limit_low[0];
  }

  // ------------------------------------------------------------------ state
  MachineState state() const { return state_; }
  mm_t height_mm() const { return cmd_mm_[0]; }
  mm_t target_mm() const { return target_mm_; }
  rate_t rate_mm_s() const { return vel_mm_s_; }
  const Faults& faults() const { return sup_.faults(); }
  Supervisor& supervisor() { return sup_; }
  bool calibrated() const { return cal_ && cal_->homed; }
  bool moving() const {
    return state_ == MachineState::Moving || state_ == MachineState::Retreating;
  }
  // Needs a human: latched fault or an unintentional stop.
  bool needs_attention() const {
    return state_ == MachineState::Fault || state_ == MachineState::Protect ||
           faults().any_hard();
  }
  const Preset& preset(u8 i) const { return i < kMaxPresets ? preset_[i] : preset_[0]; }
  bool cooling_down() const { return move_cool_until_us_ != 0; }
  us_t cool_until_us() const { return move_cool_until_us_; }
  float eta_s() const {
    if (!m_) return 0.0f;
    const AxisProfile& a = m_->axis[0];
    Trapezoid::Limits l{};
    l.vmax = m_->vmax_for(a, target_mm_, cmd_mm_[0]);
    l.amax = a.amax_mm_s2;
    l.dmax = l.amax;
    return Trapezoid::eta_s(cmd_mm_[0], target_mm_, l);
  }

 private:
  enum class HomePhase : u8 { Init, Backoff, Search, Done };

  static rate_t approach(rate_t v, rate_t goal, rate_t slew_per_s, float dt) {
    const float d = slew_per_s * dt;
    if (v < goal) return std::min(goal, v + d);
    if (v > goal) return std::max(goal, v - d);
    return v;
  }

  // Desk-space rate -> motor microsteps. The model integrates the *rounded*
  // integer delta, exactly what the pulse generator emits, so model and
  // hardware cannot drift apart by accumulating fractional steps.
  void command_axis(rate_t mm_s, float dt_s) {
    const AxisProfile& a = m_->axis[0];

    // Quantise ONCE, here. The value published to the driver and the value
    // integrated into the model must be the same number: rounding the rate to
    // whole microsteps/second and separately rounding a per-tick distance
    // disagree, and the disagreement shows up exactly where it hurts - the
    // decelerating tail, where the commanded rate is small enough that the
    // driver's rounding sends it to zero while the model keeps crediting
    // itself with motion. That reads as a phantom ~1 mm of lag, and after
    // enough moves it reads as a stall on a desk that is working perfectly.
    steps_t rate_steps_s = m_->mm_to_steps(a, mm_s);
    if (std::fabs(rate_steps_s) < 1) rate_steps_s = 0;
    const rate_t steps_s = static_cast<rate_t>(rate_steps_s);

    // Carry the fractional step between ticks. At 12 mm/s and 16 microstepping
    // a 2 ms tick is 125.67 microsteps; rounding that to 126 every tick biases
    // the model ~0.5% ahead of the pulses actually emitted, and the error grows
    // with the length of the move. A real pulse train has this remainder
    // implicitly; the model needs it explicitly or the encoder eventually
    // reports a disagreement that the software invented.
    step_carry_ += static_cast<double>(rate_steps_s) * dt_s;
    steps_t delta = static_cast<steps_t>(step_carry_);
    if (step_carry_ < 0 && delta > step_carry_) --delta;   // floor, not truncate
    step_carry_ -= static_cast<double>(delta);
    cmd_steps_[0] += delta;
    cmd_mm_[0] = cal_->home_mm + m_->steps_to_mm(a, cmd_steps_[0]);

    // Leg equalisation, dual-drive machines only. The frame wants the mean of
    // the two legs to follow the commanded profile, and the *difference* to go
    // to zero, so the mean rate is split with a proportional bias toward the
    // lagging leg. Rate-domain trimming (rather than a position servo on each
    // leg) keeps the ramp smooth and cannot fight the trapezoid.
    rate_t s0 = steps_s, s1 = steps_s;
    if (m_->axis_count > 1 && have_enc_ &&
        m_->coupling != AxisCoupling::Single) {
      const mm_t skew = enc_mm_[0] - enc_mm_[1];        // + => leg 2 lags
      float trim = skew * m_->safety.skew_trim_per_mm;
      trim = clamp_(trim, -m_->safety.skew_trim_max, m_->safety.skew_trim_max);
      // Keep the mean intact: bias one way, the other way by the same amount.
      s0 = steps_s * (1.0f - trim);
      s1 = steps_s * (1.0f + trim);
    }

    hal_->drive_rate(0, s0);
    if (m_->axis_count > 1) hal_->drive_rate(1, s1);
    hal_->set_motor_power(vel_mm_s_ != 0.0f || hold_current_);
  }

  void run_homing(const Inputs& in, const AxisProfile& a, float dt_s) {
    const bool at_low = in.limit_low[0];

    // Homing watchdog: see SafetyProfile::home_timeout_margin. Derived from
    // the geometry so it adapts to any machine, with a floor for the back-off.
    if (home_phase_ != HomePhase::Done && home_deadline_us_ == 0) {
      us_t budget = m_->safety.home_max_us;
      if (budget == 0) {
        const mm_t span = a.travel_high_mm - a.travel_low_mm;
        const float secs = (span / std::max(0.1f, m_->safety.home_rate_mm_s)) *
                           m_->safety.home_timeout_margin + 10.0f;
        budget = static_cast<us_t>(secs * 1e6f);
      }
      home_deadline_us_ = hal_->now_us() + budget;
      home_budget_us_ = budget;
    }
    if (home_phase_ != HomePhase::Done && home_deadline_us_ &&
        hal_->now_us() > home_deadline_us_) {
      vel_mm_s_ = 0.0f;
      hal_->drive_rate(0, 0.0f);
      hal_->set_motor_power(false);
      cmd_ = Command::Stop;
      home_phase_ = HomePhase::Done;
      sup_.faults().home_timeout = true;
      state_ = MachineState::Fault;
      hal_->log("home: TIMEOUT - path blocked?");
      return;
    }
    const rate_t vhunt = -std::min(m_->safety.home_rate_mm_s, a.vmax_down_mm_s);
    const rate_t vback = std::min(m_->safety.home_rate_mm_s, a.vmax_up_mm_s);
    const rate_t slew = a.amax_mm_s2 * 0.35f;   // homing is gentle on purpose

    switch (home_phase_) {
      case HomePhase::Init:
        // Anchoring on a switch we cannot prove we just pressed is how a desk
        // boots with a wrong zero, so if we start already on the stop, back off
        // and re-approach to manufacture a real rising edge.
        home_phase_ = at_low ? HomePhase::Backoff : HomePhase::Search;
        break;

      case HomePhase::Backoff:
        vel_mm_s_ = approach(vel_mm_s_, vback, slew, dt_s);
        command_axis(vel_mm_s_, dt_s);
        state_ = MachineState::Moving;
        if (!at_low && std::fabs(vel_mm_s_) >= vback * 0.9f) home_phase_ = HomePhase::Search;
        return;

      case HomePhase::Search:
        vel_mm_s_ = approach(vel_mm_s_, vhunt, slew, dt_s);
        command_axis(vel_mm_s_, dt_s);
        state_ = MachineState::Moving;
        if (at_low && !was_low_) {
          // Verified rising edge: anchor here and now.
          vel_mm_s_ = 0.0f;
          hal_->drive_rate(0, 0.0f);
          // Zero the counter AND record what height that zero means. Both
          // halves are required: a zeroed counter with an unknown datum is
          // how a retrofit ends up driving into its own soft limit.
          cmd_steps_[0] = 0;
          cmd_mm_[0] = a.travel_low_mm;
          cal_->home_mm = a.travel_low_mm;
          cal_->home_steps = 0;
          cal_->homed = true;
          if (have_enc_) cal_->home_counts = in.encoder_counts[0];
          hal_->nv_write(NvsKey::kNvsCalibration, cal_, sizeof(Calibration));
          target_mm_ = cmd_mm_[0];
          cmd_ = Command::Stop;
          home_phase_ = HomePhase::Done;
          state_ = MachineState::Ready;
          hold_current_ = false;
          hal_->log("home: anchored at low stop");
        }
        return;

      case HomePhase::Done:
        return;
    }
  }

  // A stall means the mechanism is not going where it is told. Stop, then back
  // off away from whatever it hit. A heavy top on a worn worm keeps sinking if
  // the coils are simply dropped, and "stopped, but nobody knows where" is a
  // worse place to leave an operator than a slow, deliberate retreat.
  void enter_retreat(rate_t vel_at_stall) {
    vel_mm_s_ = 0.0f;
    hal_->drive_rate(0, 0.0f);
    hal_->set_motor_power(true);
    retreat_dir_ = (vel_at_stall > 0.0f) ? -1 : (vel_at_stall < 0.0f ? 1 : 0);
    retreat_left_mm_ = m_->safety.stall_backoff_up_mm;
    cmd_ = Command::Stop;      // do not resume the failed move
    target_mm_ = cmd_mm_[0];
    if (retreat_dir_ == 0) {   // stalled while nominally stopped: nowhere to back off to
      retreat_left_mm_ = 0.0f;
      finish_retreat(false);
      return;
    }
    state_ = MachineState::Retreating;
    hal_->log("stall: retreating");
  }

  void run_retreat(const Inputs& in, float dt_s, us_t now) {
    const AxisProfile& a = m_->axis[0];
    const bool hit_stop = (retreat_dir_ < 0 && in.limit_low[0]) ||
                          (retreat_dir_ > 0 && in.limit_high[0]);
    if (retreat_left_mm_ <= 0.0f || hit_stop) {
      // Only trust the encoder if the retreat actually ran; a zeroed encoder
      // would anchor the model at the wrong height.
      finish_retreat(have_enc_ && !hit_stop);
      return;
    }
    const rate_t v = retreat_dir_ * std::min(2.0f, a.vmax_up_mm_s);
    vel_mm_s_ = approach(vel_mm_s_, v, a.amax_mm_s2 * 0.3f, dt_s);
    command_axis(vel_mm_s_, dt_s);
    retreat_left_mm_ -= std::fabs(vel_mm_s_) * dt_s;
    track_duty(vel_mm_s_, now);
  }

  // Re-anchor the model to the encoder, then clear *only* the stall latch: if
  // something else was wrong it is still wrong and still latched.
  void finish_retreat(bool trust_encoder) {
    vel_mm_s_ = 0.0f;
    hal_->drive_rate(0, 0.0f);
    retreat_left_mm_ = 0.0f;
    retreat_dir_ = 0;
    if (trust_encoder && cal_->homed) {
      cmd_mm_[0] = enc_mm_[0];
      cmd_steps_[0] = m_->mm_to_steps(m_->axis[0], cmd_mm_[0] - cal_->home_mm);
    }
    target_mm_ = cmd_mm_[0];
    // The stall latch deliberately stays set. The retreat made the machine
    // safe and re-anchored its position, but a desk that quietly looks healthy
    // again right after it hit something invites a second, harder attempt at
    // whatever was under the top. One acknowledge() releases it.
    state_ = MachineState::Protect;
    hold_current_ = true;
    hal_->log("retreat complete: acknowledge to continue");
  }

  void kill_all() {
    vel_mm_s_ = 0.0f;
    cmd_ = Command::Stop;
    target_mm_ = cmd_mm_[0];
    retreat_left_mm_ = 0.0f;
    retreat_dir_ = 0;
    hold_current_ = false;
    if (hal_) {
      hal_->drive_rate(0, 0.0f);
      if (m_ && m_->axis_count > 1) hal_->drive_rate(1, 0.0f);
      hal_->set_motor_power(false);
    }
    state_ = MachineState::Protect;
  }

  // A crank bench is not a CNC axis: the printed gearbox is the weak thermal
  // link, so cap how much of any window is spent driving. Only called for
  // user-initiated moves; homing runs at a gentle rate and must complete.
  void track_duty(rate_t v, us_t now) {
    const bool is_moving = std::fabs(v) > 0.05f;  // duty bookkeeping
    if (last_us_ && now > last_us_) {
      if (is_moving) {
        if (!window_open_) { window_start_us_ = now; window_open_ = true; }
        run_us_ += now - last_us_;
        if (run_us_ >= m_->safety.move_max_us) {
          move_cool_until_us_ = now + m_->safety.cool_us;
          run_us_ = 0;
          window_open_ = false;
          hal_->log("duty: cooling down");
        }
      }
      if (window_open_ && now - window_start_us_ > m_->safety.move_window_us) {
        run_us_ = 0;
        window_open_ = false;
      }
    }
    last_us_ = now;
  }

  Hal* hal_ = nullptr;
  const MachineProfile* m_ = nullptr;
  Calibration* cal_ = nullptr;
  Supervisor sup_{};
  Trapezoid trapezoid_{};

  Command cmd_ = Command::Stop;
  MachineState state_ = MachineState::Uncalibrated;
  HomePhase home_phase_ = HomePhase::Init;
  us_t home_deadline_us_ = 0;
  us_t home_budget_us_ = 0;
  mm_t target_mm_ = 0.0f;
  rate_t vel_mm_s_ = 0.0f;
  bool hold_current_ = false;
  bool have_enc_ = false;
  bool was_low_ = false;
  int  log_burst_ = 0;  // reserved for rate-limited logging

  steps_t cmd_steps_[kMaxAxis] = {};
  double step_carry_ = 0.0;   // fractional microsteps between ticks
  mm_t enc_mm_[kMaxAxis] = {};
  mm_t cmd_mm_[kMaxAxis] = {};

  i8 retreat_dir_ = 0;
  mm_t retreat_left_mm_ = 0.0f;

  us_t remote_deadline_us_ = 0;
  us_t last_us_ = 0, run_us_ = 0, window_start_us_ = 0, move_cool_until_us_ = 0;
  bool window_open_ = false;

  Preset preset_[kMaxPresets] = {};
};

}  // namespace desk
