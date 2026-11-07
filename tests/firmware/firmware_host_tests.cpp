// deskbot firmware - host tests for the firmware-only headers.
//
// json_embed.h, profile_embed.h, encoder_as5600.h and generated profile
// header compile on the host too, and two of them contain logic where a
// silent bug costs the most (a profile parsed wrong => a wrong soft limit;
// an unwrap that loses a revolution => stall detection lies). Neither can be
// caught by a firmware build, so they are caught here.
//
// Build + run (from firmware/):
//   g++ -std=c++17 -Wall -Wextra -fsanitize=undefined,address
//       -Iinclude -I../core/include tests/firmware/firmware_host_tests.cpp -o /tmp/fwtest && /tmp/fwtest
#include <cstdio>
#include <cstring>
#include <string>

#include "desk/controller.h"
#include "deskbot/encoder_as5600.h"
#include "deskbot/json_embed.h"
#include "deskbot/profile_embed.h"
#include "generated/machine_profile_json.h"

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);          \
      ++g_fail;                                                            \
    } else {                                                               \
      ++g_pass;                                                            \
    }                                                                      \
  } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

using namespace desk;

// ---------------------------------------------------------------- JSON parser
static bool load_str(const char* text, MachineProfile& m, const char** err,
                     json::Dom* keep = nullptr) {
  static json::Dom owned;
  json::Dom& dom = keep ? *keep : owned;
  return load_machine(text, strlen(text), dom, m, err);
}

static void test_parser_basics() {
  json::Dom dom;
  dom.reset();
  const char* text = R"({"a": 1, "b": [1, 2, 3], "c": {"d": "x"}, "e": true, "f": null})";
  json::Parser p(text, strlen(text), &dom);
  const char* err = nullptr;
  int root = -1;
  CHECK(p.parse(&root, &err));
  const json::Value& r = dom.nodes[root];
  CHECK(r.kind == json::Kind::Obj);
  CHECK(json::as_num(json::member(dom, r, "a"), -1.0) == 1.0);
  const json::Value& arr = json::member(dom, r, "b");
  CHECK(arr.kind == json::Kind::Arr);
  CHECK(arr.count == 3);
  CHECK(json::as_num(json::item(dom, arr, 0), -1) == 1);
  CHECK(json::as_num(json::item(dom, arr, 2), -1) == 3);
  CHECK(json::member(dom, arr, "b").kind == json::Kind::Null);  // not an object
  const json::Value& c = json::member(dom, r, "c");
  CHECK(std::strcmp(json::as_str(json::member(dom, c, "d")), "x") == 0);
  CHECK(json::as_bool(json::member(dom, r, "e"), false) == true);
  CHECK(json::member(dom, r, "f").kind == json::Kind::Null);
  CHECK(!json::has(dom, r, "nope"));

  // Missing keys must fall through to the caller's default, not to 0.
  CHECK(json::as_num(json::member(dom, r, "nope"), 42.0) == 42.0);
  CHECK(json::as_flt(json::member(dom, r, "nope"), 7.5f) == 7.5f);
  CHECK(std::strcmp(json::as_str(json::member(dom, r, "nope"), "dflt"), "dflt") == 0);
}

// The bug this test exists for: with children in a shared pool, an object's
// members get interleaved with the members of nested objects parsed in between,
// and "member i" silently returns a cousin. Numbers here are all distinct so
// any mix-up is visible.
static void test_nested_lookup_integrity() {
  json::Dom dom;
  const char* text =
      R"({
        "identity": {"name": "n", "profile_version": 3},
        "axes": [{"steps_per_motor_rev": 10, "gear_ratio": 11.0,
                  "notes": ["a", "b"]},
                 {"steps_per_motor_rev": 20, "gear_ratio": 21.0}],
        "safety": {"stall_hold_ms": 220, "nested": {"deep": {"deeper": 99}}},
        "trailer": 12345
      })";
  json::Parser p(text, strlen(text), &dom);
  const char* err = nullptr;
  int root = -1;
  CHECK(p.parse(&root, &err));
  const json::Value& r = dom.nodes[root];
  const json::Value& id = json::member(dom, r, "identity");
  CHECK(json::as_num(json::member(dom, id, "profile_version"), -1) == 3);
  const json::Value& axes = json::member(dom, r, "axes");
  CHECK(axes.count == 2);
  const json::Value& a0 = json::item(dom, axes, 0);
  CHECK(json::as_num(json::member(dom, a0, "steps_per_motor_rev"), -1) == 10);
  CHECK(json::as_num(json::member(dom, a0, "gear_ratio"), -1) == 11.0);
  const json::Value& a1 = json::item(dom, axes, 1);
  CHECK(json::as_num(json::member(dom, a1, "steps_per_motor_rev"), -1) == 20);
  const json::Value& notes = json::member(dom, a0, "notes");
  CHECK(notes.count == 2);
  CHECK(std::strcmp(json::as_str(json::item(dom, notes, 1)), "b") == 0);
  const json::Value& sf = json::member(dom, r, "safety");
  CHECK(json::as_num(json::member(dom, sf, "stall_hold_ms"), -1) == 220);
  const json::Value& deep = json::member(dom, json::member(dom, sf, "nested"), "deep");
  CHECK(json::as_num(json::member(dom, deep, "deeper"), -1) == 99);
  // The member parsed last, after all that nesting, must still be reachable:
  // this is the exact lookup that a shared-edge-pool breaks.
  CHECK(json::as_num(json::member(dom, r, "trailer"), -1) == 12345);
}

static void test_parser_errors() {
  json::Dom dom;
  MachineProfile m;
  const char* err = nullptr;
  CHECK(!load_str(R"({"a": 1) ", m, &err));            // unterminated
  CHECK(!load_str(R"({"a": 1} x)", m, &err));          // trailing characters
  CHECK(!load_str(R"({"a" 1})", m, &err));             // missing ':'
  CHECK(!load_str(R"({"a": 1,})", m, &err));           // trailing comma
  CHECK(!load_str(R"([1, 2,])", m, &err));             // trailing comma in array
  CHECK(!load_str(R"({"a": tru})", m, &err));          // bad literal
  CHECK(!load_str("", m, &err));                       // empty
  CHECK(!load_str(R"({"axes":[{}]})", m, &err));       // parses; loader rejects
  CHECK(load_str(R"({"axes":[{}], "extra": {"k": [1,{"deep": true}]}})", m, &err) == false);
}

// ------------------------------------------------------------------- loader
static void test_loader_matches_host_defaults() {
  MachineProfile m;
  const char* err = nullptr;
  // A minimal profile: everything optional must take the documented default
  // (mirrors tests/include/deskbot/profile_loader.h).
  const char* min =
      R"({"identity":{"name":"t","profile_version":2},
          "axes":[{"steps_per_motor_rev":3200,"gear_ratio":5.0,"mm_per_shaft_rev":5.5,
                   "travel_low_mm":660,"travel_high_mm":990}]})";
  CHECK(load_str(min, m, &err));
  CHECK(m.profile_version == 2);
  CHECK(m.axis_count == 1);
  CHECK(m.coupling == AxisCoupling::Single);
  CHECK(m.feedback == FeedbackKind::None);   // host default when key absent
  CHECK(m.drive == DriveKind::StepperPulse);
  CHECK_NEAR(m.axis[0].vmax_up_mm_s, 12.0f, 1e-6f);
  CHECK_NEAR(m.axis[0].vmax_down_mm_s, 14.0f, 1e-6f);
  CHECK_NEAR(m.axis[0].amax_mm_s2, 45.0f, 1e-6f);
  CHECK_NEAR(m.axis[0].direction, 1.0f, 1e-6f);
  CHECK(m.axis[0].encoder_counts_per_motor_rev == 2000);
  CHECK(m.safety.stall_hold_us == 220000);
  CHECK(m.safety.remote_deadman_us == 4000000);
  CHECK(m.safety.move_window_us == 120000000);
  CHECK(m.safety.move_max_us == 60000000);
  CHECK(m.safety.cool_us == 300000000);
  CHECK(m.safety.temp_c_stop > 80.0f);

  // Non-physical scales must be refused, not tolerated.
  const char* bad[] = {
      R"({"axes":[{"steps_per_motor_rev":0,"gear_ratio":5,"mm_per_shaft_rev":5,
                   "travel_low_mm":0,"travel_high_mm":10}]})",
      R"({"axes":[{"steps_per_motor_rev":3200,"gear_ratio":0,"mm_per_shaft_rev":5,
                   "travel_low_mm":0,"travel_high_mm":10}]})",
      R"({"axes":[{"steps_per_motor_rev":3200,"gear_ratio":5,"mm_per_shaft_rev":-1,
                   "travel_low_mm":0,"travel_high_mm":10}]})",
      R"({"axes":[{"steps_per_motor_rev":3200,"gear_ratio":5,"mm_per_shaft_rev":5,
                   "travel_low_mm":990,"travel_high_mm":660}]})",
      R"({"axes":[]})",
      R"({"axes":[{},{},{}]})",
  };
  for (const char* s : bad) {
    MachineProfile tmp;
    CHECK(!load_str(s, tmp, &err));
  }

  // Three axes is more than the core supports.
  MachineProfile three;
  CHECK(!load_str(R"({"axes":[
      {"steps_per_motor_rev":1,"gear_ratio":1,"mm_per_shaft_rev":1,
       "travel_low_mm":0,"travel_high_mm":1},
      {"steps_per_motor_rev":1,"gear_ratio":1,"mm_per_shaft_rev":1,
       "travel_low_mm":0,"travel_high_mm":1},
      {"steps_per_motor_rev":1,"gear_ratio":1,"mm_per_shaft_rev":1,
       "travel_low_mm":0,"travel_high_mm":1}]})", three, &err));
}

// The committed generated header must load to the numbers the Husky profile
// actually declares. If someone edits machines/*.json and forgets to regenerate
// this fails loudly (and so does the sync check in main.cpp at boot, which
// compares profile_version against the header's macro).
static void test_embedded_profile() {
  static json::Dom dom;
  MachineProfile m;
  const char* err = nullptr;
  CHECK(load_machine(kMachineJson, sizeof(kMachineJson) - 1, dom, m, &err));
  CHECK(std::strcmp(m.name, DESKBOT_MACHINE_NAME) == 0);
  CHECK(m.profile_version == DESKBOT_MACHINE_PROFILE_VERSION);
  CHECK(m.axis_count == 1);
  CHECK(m.coupling == AxisCoupling::Single);
  CHECK(m.feedback == FeedbackKind::MotorEncoder);
  CHECK(m.drive == DriveKind::StepperPulse);
  CHECK(m.axis[0].steps_per_motor_rev == 3200);
  CHECK_NEAR(m.axis[0].gear_ratio, 5.06f, 1e-6f);
  CHECK_NEAR(m.axis[0].mm_per_shaft_rev, 5.5f, 1e-6f);
  CHECK_NEAR(m.axis[0].travel_low_mm, 660.0f, 1e-6f);
  CHECK_NEAR(m.axis[0].travel_high_mm, 990.0f, 1e-6f);
  CHECK(m.axis[0].encoder_counts_per_motor_rev == 2000);
  CHECK_NEAR(m.safety.stall_lag_mm_moving, 1.6f, 1e-6f);
  CHECK_NEAR(m.safety.home_rate_mm_s, 3.0f, 1e-6f);
  CHECK(m.safety.stall_hold_us == 220000);
  CHECK(m.safety.move_max_us == 60000000);

  // Sanity-check the unit conversions the firmware relies on, using the core's
  // own functions: 1 motor rev must be a sane sliver of desk travel, and the
  // encoder scale must agree with the step scale (both are the same mechanism).
  const float steps_per_mm =
      static_cast<float>(m.axis[0].steps_per_motor_rev) * m.axis[0].gear_ratio /
      m.axis[0].mm_per_shaft_rev;
  CHECK_NEAR(steps_per_mm, 2944.0f, 5.0f);       // 3200*5.06/5.5
  const mm_t one_rev_mm = m.counts_to_mm(m.axis[0], m.axis[0].encoder_counts_per_motor_rev);
  CHECK_NEAR(one_rev_mm, m.axis[0].mm_per_shaft_rev / m.axis[0].gear_ratio, 1e-4f);
}

// ------------------------------------------------------------------ encoder
static void test_unwrap_monotonic_up() {
  deskbot::AngleUnwrap u;
  int64_t last = -1;
  // Sweep 5 full revolutions upward through every wrap point.
  for (int i = 0; i < 4096 * 5; ++i) {
    int64_t out = 0;
    CHECK(u.update(static_cast<uint16_t>(i & 0xFFF), &out));
    CHECK(out == i);
    CHECK(out > last);
    last = out;
  }
  CHECK(u.valid);
  CHECK(u.fault_count == 0);
}

static void test_unwrap_monotonic_down() {
  deskbot::AngleUnwrap u;
  int64_t expect = 0;
  int64_t out = 0;
  CHECK(u.update(0, &out));
  expect = 0;
  for (int i = 1; i <= 4096 * 3; ++i) {
    expect -= 1;
    CHECK(u.update(static_cast<uint16_t>((-(i))& 0xFFF), &out));
    CHECK(out == expect);
  }
  CHECK(u.valid);
}

static void test_unwrap_wrap_points_both_directions() {
  deskbot::AngleUnwrap u;
  int64_t out = 0;
  CHECK(u.update(4094, &out));  CHECK(out == 4094);
  CHECK(u.update(4095, &out));  CHECK(out == 4095);
  CHECK(u.update(0, &out));     CHECK(out == 4096);   // 4095 -> 0 forward wrap
  CHECK(u.update(1, &out));     CHECK(out == 4097);
  CHECK(u.update(0, &out));     CHECK(out == 4096);   // back down
  CHECK(u.update(4095, &out));  CHECK(out == 4095);   // backward wrap
  CHECK(u.valid);
}

static void test_unwrap_fast_motion_still_works() {
  // ~106 counts/tick is the profile's physical maximum. It must pass.
  deskbot::AngleUnwrap u;
  int64_t out = 0;
  CHECK(u.update(0, &out));
  for (int i = 1; i <= 100; ++i) {
    CHECK(u.update(static_cast<uint16_t>((i * 106) & 0xFFF), &out));
  }
  CHECK(u.valid);
  CHECK(out == 100 * 106);
}

static void test_unwrap_rejects_glitch() {
  deskbot::AngleUnwrap u;
  int64_t out = 0;
  CHECK(u.update(100, &out));
  // +512 is exactly at the threshold: accepted, not a fault (boundary check).
  CHECK(u.update(100 + 512, &out));
  CHECK(u.valid);
  CHECK(u.fault_count == 0);
  // 513 counts beyond the *last accepted sample* (612) is rejected and latches
  // invalid. Compute targets against 612, not against the original 100 - a
  // first-draft of this test probed 100+513 == 613, which is delta +1.
  const uint16_t last = 100 + 512;
  CHECK(!u.update(last + 513, &out));
  CHECK(!u.valid);
  CHECK(u.fault_count == 1);
  // A subsequent *small* sample still cannot re-enable it: the datum was lost,
  // and only a re-home may be trusted (the sticky-valid design). The early
  // return does not re-count a fault; fault_count counts anomalies detected.
  CHECK(!u.update(last + 1, &out));
  CHECK(u.fault_count == 1);
}

static void test_unwrap_rejects_half_turn_ambiguity() {
  deskbot::AngleUnwrap u;
  int64_t out = 0;
  CHECK(u.update(0, &out));
  CHECK(!u.update(2048, &out));   // exactly half a turn: ambiguous -> fault
  CHECK(!u.valid);
}

static void test_glitch_does_not_shift_datum() {
  // After a rejected sample the revolution count must be untouched: the
  // rejection path may not consume the wrap it tentatively counted.
  deskbot::AngleUnwrap u;
  int64_t out = 0;
  CHECK(u.update(4090, &out));
  CHECK(!u.update(1000, &out));   // raw delta -3090 -> unwraps to +1006 -> fault
  CHECK(!u.valid);
  // The last published cumulative is still the pre-fault value.
  CHECK(u.cumulative() == 4090);
}

static void test_track_scaling_and_sign() {
  deskbot::As5600Track t;
  t.configure(2000);              // the Husky's encoder_counts_per_motor_rev
  int64_t counts = 0;
  // One full raw revolution == 4096 raw counts == 2000 core counts.
  int64_t out = 0;
  for (int i = 1; i <= 4096; ++i) {
    CHECK(t.update(static_cast<uint16_t>(i & 0xFFF), &counts));
    out = counts;
  }
  // Starting sample was 1 (i=1), so cumulative raw = 1 + 4095 = 4096 -> 2000.
  CHECK(out == 2000);
  CHECK(t.valid());
  // Downward motion must produce non-increasing core counts that actually
  // fall (no unsigned wrap). Non-increasing, not strictly decreasing: the
  // 4096 -> 2000 scaling means a single raw count is only ~0.49 core counts,
  // so consecutive samples legitimately repeat half the time.
  const int64_t start = counts;
  int64_t prev = counts;
  for (int i = 1; i <= 500; ++i) {
    CHECK(t.update(static_cast<uint16_t>((4096 - i) & 0xFFF), &counts));
    CHECK(counts <= prev);
    prev = counts;
  }
  CHECK(counts < start);
  CHECK(start - counts >= 200);   // 500 raw counts * 2000/4096 ~= 244
}

// ----------------------------------------------------- rate sign convention
// The sign convention the HAL must honour: core passes mm_to_steps(direction,
// mm), so a positive drive_rate() is the direction the profile calls "up".
// This pins down the mapping the pin table in main.cpp documents.
static void test_sign_convention_helper() {
  static json::Dom dom;
  MachineProfile m;
  const char* err = nullptr;
  CHECK(load_machine(kMachineJson, sizeof(kMachineJson) - 1, dom, m, &err));
  CHECK(m.mm_to_steps(m.axis[0], 10.0f) > 0);    // direction = +1 in the JSON
  CHECK(m.mm_to_steps(m.axis[0], -10.0f) < 0);
  MachineProfile flipped = m;
  flipped.axis[0].direction = -1.0f;
  CHECK(flipped.mm_to_steps(flipped.axis[0], 10.0f) < 0);
  // And the round trip that makes the encoder/model comparison meaningful.
  const steps_t s = m.mm_to_steps(m.axis[0], 123.0f);
  CHECK_NEAR(m.steps_to_mm(m.axis[0], s), 123.0f, 1e-3f);
}

int main() {
  test_parser_basics();
  test_nested_lookup_integrity();
  test_parser_errors();
  test_loader_matches_host_defaults();
  test_embedded_profile();
  test_unwrap_monotonic_up();
  test_unwrap_monotonic_down();
  test_unwrap_wrap_points_both_directions();
  test_unwrap_fast_motion_still_works();
  test_unwrap_rejects_glitch();
  test_unwrap_rejects_half_turn_ambiguity();
  test_glitch_does_not_shift_datum();
  test_track_scaling_and_sign();
  test_sign_convention_helper();

  std::printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
