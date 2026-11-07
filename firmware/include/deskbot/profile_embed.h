// deskbot firmware - MachineProfile loader for the embedded JSON DOM.
//
// Firmware twin of tests/include/deskbot/profile_loader.h. Keys, defaults and
// validation order mirror the host loader exactly; where they differ this file
// says so in a comment. A difference in defaults between the two loaders is a
// firmware-vs-simulator behaviour bug that only shows up on the real machine,
// so treat the two as a single unit when editing.
//
// `name`/`url` in MachineProfile are const char*: the host loader strdups them,
// here they point into the parser's interned string pool, which lives for the
// lifetime of the Dom (static). No allocation, no lifetime hazard, as long as
// the Dom outlives the profile - which load_machine() enforces by taking it by
// reference.
#pragma once
#include "desk/machine_profile.h"
#include "deskbot/json_embed.h"

namespace desk {

inline DriveKind drive_from(const char* s) {
  if (!s) return DriveKind::StepperPulse;
  if (strcmp(s, "dc_hbridge") == 0) return DriveKind::DcHBridge;
  if (strcmp(s, "keypad_sim") == 0) return DriveKind::KeypadSim;
  return DriveKind::StepperPulse;
}
inline FeedbackKind feedback_from(const char* s) {
  if (!s) return FeedbackKind::None;
  if (strcmp(s, "motor_encoder") == 0) return FeedbackKind::MotorEncoder;
  if (strcmp(s, "shaft_encoder") == 0) return FeedbackKind::ShaftEncoder;
  return FeedbackKind::None;
}
inline AxisCoupling coupling_from(const char* s) {
  if (!s) return AxisCoupling::Single;
  if (strcmp(s, "master_follower") == 0) return AxisCoupling::MasterFollower;
  if (strcmp(s, "independent") == 0) return AxisCoupling::Independent;
  return AxisCoupling::Single;
}

// Parse `json_text` into `dom` and fill `m`. Returns false and sets `err` to a
// static string on the first problem. All numbers are taken as written: a
// missing optional key gets the same default as the host loader, and the axes
// are range-checked the same way, because a non-physical scale factor must stop
// the boot rather than be "helpfully" corrected.
inline bool load_machine(const char* json_text, size_t len, json::Dom& dom,
                         MachineProfile& m, const char** err) {
  *err = nullptr;
  dom.reset();
  json::Parser p(json_text, len, &dom);
  int root_idx = -1;
  if (!p.parse(&root_idx, err)) return false;
  const json::Value& root = dom.nodes[root_idx];
  if (root.kind != json::Kind::Obj) { *err = "profile root must be an object"; return false; }

  const json::Value& id = json::member(dom, root, "identity");
  m.name = json::as_str(json::member(dom, id, "name"), "generic");
  m.url = json::as_str(json::member(dom, id, "url"), "");
  m.profile_version = static_cast<u32>(json::as_num(json::member(dom, id, "profile_version"), 1));

  m.drive = drive_from(json::as_str(json::member(dom, json::member(dom, root, "drive"), "kind"),
                                    "stepper_pulse"));
  m.feedback = feedback_from(json::as_str(
      json::member(dom, json::member(dom, root, "feedback"), "kind"), "none"));
  m.coupling = coupling_from(json::as_str(
      json::member(dom, json::member(dom, root, "mechanism"), "coupling"), "single"));

  const json::Value& axes = json::member(dom, root, "axes");
  if (axes.kind != json::Kind::Arr || axes.count == 0) {
    *err = "profile needs a non-empty 'axes' array";
    return false;
  }
  if (axes.count > kMaxAxis) { *err = "too many axes"; return false; }
  m.axis_count = static_cast<u8>(axes.count);

  for (int i = 0; i < axes.count; ++i) {
    const json::Value& ax = json::item(dom, axes, i);
    AxisProfile& a = m.axis[i];
    a = AxisProfile{};
    for (const char* req : {"steps_per_motor_rev", "gear_ratio", "mm_per_shaft_rev",
                            "travel_low_mm", "travel_high_mm"}) {
      if (!json::has(dom, ax, req)) {
        *err = req;   // caller prints which key was missing
        return false;
      }
    }
    a.steps_per_motor_rev =
        static_cast<steps_t>(json::as_num(json::member(dom, ax, "steps_per_motor_rev")));
    a.gear_ratio = static_cast<float>(json::as_num(json::member(dom, ax, "gear_ratio")));
    a.mm_per_shaft_rev = static_cast<float>(json::as_num(json::member(dom, ax, "mm_per_shaft_rev")));
    a.travel_low_mm = static_cast<float>(json::as_num(json::member(dom, ax, "travel_low_mm")));
    a.travel_high_mm = static_cast<float>(json::as_num(json::member(dom, ax, "travel_high_mm")));
    a.vmax_up_mm_s = json::as_flt(json::member(dom, ax, "vmax_up_mm_s"), 12.0f);
    a.vmax_down_mm_s = json::as_flt(json::member(dom, ax, "vmax_down_mm_s"), 14.0f);
    a.amax_mm_s2 = json::as_flt(json::member(dom, ax, "amax_mm_s2"), 45.0f);
    a.direction = json::as_flt(json::member(dom, ax, "direction"), 1.0f);
    a.encoder_counts_per_motor_rev =
        static_cast<u32>(json::as_num(json::member(dom, ax, "encoder_counts_per_motor_rev"), 2000.0));
    if (!finite_(a.mm_per_shaft_rev) || a.mm_per_shaft_rev <= 0.0f ||
        !finite_(a.gear_ratio) || a.gear_ratio <= 0.0f ||
        a.steps_per_motor_rev <= 0 || a.travel_high_mm <= a.travel_low_mm) {
      *err = "axis has a non-physical scale or travel";
      return false;
    }
  }

  const json::Value& sf = json::member(dom, root, "safety");
  SafetyProfile& s = m.safety;
  s.stall_lag_mm_moving = json::as_flt(json::member(dom, sf, "stall_lag_mm_moving"), s.stall_lag_mm_moving);
  s.stall_lag_mm_stopped = json::as_flt(json::member(dom, sf, "stall_lag_mm_stopped"), s.stall_lag_mm_stopped);
  s.position_loss_mm = json::as_flt(json::member(dom, sf, "position_loss_mm"), s.position_loss_mm);
  s.stall_backoff_up_mm = json::as_flt(json::member(dom, sf, "stall_backoff_up_mm"), s.stall_backoff_up_mm);
  s.home_rate_mm_s = json::as_flt(json::member(dom, sf, "home_rate_mm_s"), s.home_rate_mm_s);
  s.home_timeout_margin = json::as_flt(json::member(dom, sf, "home_timeout_margin"), s.home_timeout_margin);
  s.home_max_us = static_cast<us_t>(json::as_num(json::member(dom, sf, "home_max_ms"), 0.0) * 1e3);
  s.jog_lookahead_mm = json::as_flt(json::member(dom, sf, "jog_lookahead_mm"), s.jog_lookahead_mm);
  s.max_skew_mm = json::as_flt(json::member(dom, sf, "max_skew_mm"), s.max_skew_mm);
  s.skew_alarm_mm = json::as_flt(json::member(dom, sf, "skew_alarm_mm"), s.skew_alarm_mm);
  s.skew_trim_per_mm = json::as_flt(json::member(dom, sf, "skew_trim_per_mm"), s.skew_trim_per_mm);
  s.skew_trim_max = json::as_flt(json::member(dom, sf, "skew_trim_max"), s.skew_trim_max);
  s.temp_c_stop = json::as_flt(json::member(dom, sf, "temp_c_stop"), s.temp_c_stop);
  s.temp_c_reenter = json::as_flt(json::member(dom, sf, "temp_c_reenter"), s.temp_c_reenter);
  s.obstacle_backoff = json::as_bool(json::member(dom, sf, "obstacle_backoff"), true);
  // ms/s -> us, same conversions as the host loader.
  s.stall_hold_us = static_cast<us_t>(json::as_num(json::member(dom, sf, "stall_hold_ms"), 220.0) * 1e3);
  s.remote_deadman_us = static_cast<us_t>(json::as_num(json::member(dom, sf, "remote_deadman_ms"), 4000.0) * 1e3);
  s.move_window_us = static_cast<us_t>(json::as_num(json::member(dom, sf, "duty_window_s"), 120.0) * 1e6);
  s.move_max_us = static_cast<us_t>(json::as_num(json::member(dom, sf, "duty_max_run_s"), 60.0) * 1e6);
  s.cool_us = static_cast<us_t>(json::as_num(json::member(dom, sf, "duty_cool_s"), 300.0) * 1e6);
  return true;
}

}  // namespace desk
