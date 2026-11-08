// deskbot cli - machine profile loader: JSON -> MachineProfile.
//
// This file is the boundary between "generic software" and "a particular desk".
// Everything a new machine needs lives in one JSON document; nothing about a
// specific bench is compiled in.
#pragma once
#include <string>
#include "desk/machine_profile.h"
#include "deskbot/json_mini.h"

namespace desk {

inline DriveKind drive_from(const std::string& s) {
  if (s == "stepper_pulse") return DriveKind::StepperPulse;
  if (s == "dc_hbridge")    return DriveKind::DcHBridge;
  if (s == "keypad_sim")    return DriveKind::KeypadSim;
  return DriveKind::StepperPulse;
}
inline FeedbackKind feedback_from(const std::string& s) {
  if (s == "none")          return FeedbackKind::None;
  if (s == "motor_encoder") return FeedbackKind::MotorEncoder;
  if (s == "shaft_encoder") return FeedbackKind::ShaftEncoder;
  return FeedbackKind::None;
}
inline AxisCoupling coupling_from(const std::string& s) {
  if (s == "single")           return AxisCoupling::Single;
  if (s == "master_follower")  return AxisCoupling::MasterFollower;
  if (s == "independent")      return AxisCoupling::Independent;
  return AxisCoupling::Single;
}

// Load a profile. `err` explains the first missing required key, because a
// silently defaulted scale factor is the most expensive mistake a retrofit can
// make: it looks like it works and then lies about the height.
inline bool load_machine(const std::string& path, MachineProfile& m, std::string& err) {
  json::Value root;
  if (!json::parse_file(path, root, err)) return false;
  if (root.kind != json::Value::Kind::Obj) { err = "profile root must be an object"; return false; }

  const json::Value& id = root.at("identity");
  m.name = strdup(id.at("name").as_str("generic").c_str());
  m.url = strdup(id.at("url").as_str("").c_str());
  m.profile_version = static_cast<u32>(id.at("profile_version").as_num(1));

  m.drive = drive_from(root.at("drive").at("kind").as_str("stepper_pulse"));
  m.feedback = feedback_from(root.at("feedback").at("kind").as_str("none"));
  m.coupling = coupling_from(root.at("mechanism").at("coupling").as_str("single"));

  const json::Value& axes = root.at("axes");
  if (axes.kind != json::Value::Kind::Arr || axes.arr.empty()) {
    err = "profile needs a non-empty 'axes' array";
    return false;
  }
  if (axes.arr.size() > kMaxAxis) { err = "too many axes"; return false; }
  m.axis_count = static_cast<u8>(axes.arr.size());

  for (size_t i = 0; i < axes.arr.size(); ++i) {
    const json::Value& ax = axes.arr[i];
    AxisProfile& a = m.axis[i];
    for (const char* req : {"steps_per_motor_rev", "gear_ratio", "mm_per_shaft_rev",
                            "travel_low_mm", "travel_high_mm"}) {
      if (!ax.has(req)) {
        err = "axis " + std::to_string(i) + " missing required '" + req + "'";
        return false;
      }
    }
    a.steps_per_motor_rev = static_cast<steps_t>(ax.at("steps_per_motor_rev").as_num());
    a.gear_ratio = static_cast<float>(ax.at("gear_ratio").as_num());
    a.mm_per_shaft_rev = static_cast<float>(ax.at("mm_per_shaft_rev").as_num());
    a.travel_low_mm = static_cast<float>(ax.at("travel_low_mm").as_num());
    a.travel_high_mm = static_cast<float>(ax.at("travel_high_mm").as_num());
    a.vmax_up_mm_s = static_cast<float>(ax.at("vmax_up_mm_s").as_num(12.0));
    a.vmax_down_mm_s = static_cast<float>(ax.at("vmax_down_mm_s").as_num(14.0));
    a.amax_mm_s2 = static_cast<float>(ax.at("amax_mm_s2").as_num(45.0));
    a.direction = static_cast<float>(ax.at("direction").as_num(1.0));
    a.encoder_counts_per_motor_rev =
        static_cast<u32>(ax.at("encoder_counts_per_motor_rev").as_num(2000));
    if (a.mm_per_shaft_rev <= 0.0f || a.gear_ratio <= 0.0f ||
        a.steps_per_motor_rev <= 0 || a.travel_high_mm <= a.travel_low_mm) {
      err = "axis " + std::to_string(i) + " has a non-physical scale or travel";
      return false;
    }
  }

  const json::Value& sf = root.at("safety");
  SafetyProfile& s = m.safety;
  s.stall_lag_mm_moving   = sf.at("stall_lag_mm_moving").as_flt(s.stall_lag_mm_moving);
  s.stall_lag_mm_stopped  = sf.at("stall_lag_mm_stopped").as_flt(s.stall_lag_mm_stopped);
  s.position_loss_mm      = sf.at("position_loss_mm").as_flt(s.position_loss_mm);
  s.stall_backoff_up_mm   = sf.at("stall_backoff_up_mm").as_flt(s.stall_backoff_up_mm);
  s.home_rate_mm_s        = sf.at("home_rate_mm_s").as_flt(s.home_rate_mm_s);
  // Home watchdog tuning. These two were once read only by the firmware loader,
  // so a profile that shortened the home timeout changed the desk and did
  // nothing in the simulator - the divergence test_loaders_agree_on_* guards
  // against. Keys, defaults and units must match profile_embed.h exactly.
  s.home_timeout_margin   = sf.at("home_timeout_margin").as_flt(s.home_timeout_margin);
  s.home_max_us           = static_cast<us_t>(sf.at("home_max_ms").as_num(0.0) * 1e3);
  s.jog_lookahead_mm      = sf.at("jog_lookahead_mm").as_flt(s.jog_lookahead_mm);
  s.max_skew_mm           = sf.at("max_skew_mm").as_flt(s.max_skew_mm);
  s.skew_alarm_mm         = sf.at("skew_alarm_mm").as_flt(s.skew_alarm_mm);
  s.skew_trim_per_mm      = sf.at("skew_trim_per_mm").as_flt(s.skew_trim_per_mm);
  s.skew_trim_max         = sf.at("skew_trim_max").as_flt(s.skew_trim_max);
  s.temp_c_stop           = sf.at("temp_c_stop").as_flt(s.temp_c_stop);
  s.temp_c_reenter        = sf.at("temp_c_reenter").as_flt(s.temp_c_reenter);
  s.stall_hold_us         = static_cast<us_t>(sf.at("stall_hold_ms").as_num(220.0) * 1e3);
  s.remote_deadman_us     = static_cast<us_t>(sf.at("remote_deadman_ms").as_num(4000.0) * 1e3);
  s.move_window_us        = static_cast<us_t>(sf.at("duty_window_s").as_num(120.0) * 1e6);
  s.move_max_us           = static_cast<us_t>(sf.at("duty_max_run_s").as_num(60.0) * 1e6);
  s.cool_us               = static_cast<us_t>(sf.at("duty_cool_s").as_num(300.0) * 1e6);
  s.obstacle_backoff      = sf.at("obstacle_backoff").as_bool(true);
  return true;
}

}  // namespace desk
