// deskbot sim - drive the real controller from a machine JSON with no hardware.
//
// This is the tool that makes "the software is generic" checkable rather than
// aspirational: point it at any machine profile and it runs the same core the
// ESP32 runs. It is also the fastest way to see why a set of numbers is wrong
// before anything is printed or soldered.
//
//   ./build/deskbot-sim machines/husky-46x24.machine.json
//   ./build/deskbot-sim <profile> --script home,go=900,go=700,status
//   ./build/deskbot-sim <profile> --inject stall_at=880
//   ./build/deskbot-sim <profile> --profile-only     # validate + echo numbers
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "desk/fake_bench.h"
#include "deskbot/profile_loader.h"

using namespace desk;

namespace {

struct Opts {
  std::string profile;
  std::vector<std::string> script;
  float slip_at = 0.0f;        // mm/s above which the sim loses steps
  float slip_rate = 0.0f;
  mm_t stall_at = 0.0f;        // simulate an obstacle stopping descent here
  float leg2_drag = 1.0f;
  bool profile_only = false;
  bool quiet = false;
};

void usage() {
  std::printf(
      "deskbot-sim - run the desk controller with no hardware\n\n"
      "usage: deskbot-sim <machine.json> [options]\n\n"
      "  --script a,b,c       commands: home, go=MM, up, down, stop, status,\n"
      "                       ack, preset=MM, preset_go=N, wait=SECONDS\n"
      "  --slip-at MM_S       lose steps above this commanded rate\n"
      "  --slip-rate F        fraction of motion lost when slipping\n"
      "  --stall-at MM        obstacle: descending stops making progress here\n"
      "  --leg2-drag F        second leg achieves only F of its rate (dual)\n"
      "  --profile-only       load, validate, print derived numbers, exit\n"
      "  --quiet              no per-tick trace\n");
}

bool parse(int argc, char** argv, Opts& o) {
  std::vector<std::string> args(argv + 1, argv + argc);
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    auto next = [&](const char* what) -> const char* {
      if (i + 1 >= args.size()) { std::fprintf(stderr, "%s needs a value\n", what); return nullptr; }
      return args[++i].c_str();
    };
    if (a[0] != '-') { if (o.profile.empty()) o.profile = a; continue; }
    if (a == "--script") {
      const char* v = next("--script");
      if (!v) return false;
      std::string s(v), part;
      for (char c : s) {
        if (c == ',') { if (!part.empty()) o.script.push_back(part); part.clear(); }
        else part += c;
      }
      if (!part.empty()) o.script.push_back(part);
    } else if (a == "--slip-at")      { auto* v = next("--slip-at"); if (!v) return false; o.slip_at = std::strtof(v, nullptr); }
    else if (a == "--slip-rate")      { auto* v = next("--slip-rate"); if (!v) return false; o.slip_rate = std::strtof(v, nullptr); }
    else if (a == "--stall-at")       { auto* v = next("--stall-at"); if (!v) return false; o.stall_at = std::strtof(v, nullptr); }
    else if (a == "--leg2-drag")      { auto* v = next("--leg2-drag"); if (!v) return false; o.leg2_drag = std::strtof(v, nullptr); }
    else if (a == "--profile-only")   o.profile_only = true;
    else if (a == "--quiet")          o.quiet = true;
    else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return false; }
  }
  if (o.profile.empty()) { usage(); return false; }
  return true;
}

void print_profile(const MachineProfile& m) {
  std::printf("profile   : %s (v%u)\n", m.name, m.profile_version);
  std::printf("drive     : %d   feedback: %d   coupling: %d   axes: %u\n",
              (int)m.drive, (int)m.feedback, (int)m.coupling, m.axis_count);
  for (u8 i = 0; i < m.axis_count; ++i) {
    const AxisProfile& a = m.axis[i];
    const float span = a.travel_high_mm - a.travel_low_mm;
    const float revs = span / a.mm_per_shaft_rev;
    std::printf("axis %u    : %.0f mm travel (%.1f mm/s up, %.1f mm/s down)\n",
                i, span, a.vmax_up_mm_s, a.vmax_down_mm_s);
    std::printf("  scale     : %d steps/rev x ratio %.2f x %.3f mm/rev\n",
                (int)a.steps_per_motor_rev, a.gear_ratio, a.mm_per_shaft_rev);
    std::printf("  => one motor rev moves the top %.4f mm\n",
                a.mm_per_shaft_rev / a.gear_ratio);
    std::printf("  => full travel = %.1f shaft revs, %.1f motor revs\n", revs,
                revs * a.gear_ratio);
    Trapezoid::Limits l{};
    l.vmax = a.vmax_up_mm_s; l.amax = a.amax_mm_s2; l.dmax = l.amax;
    std::printf("  => full travel takes %.0f s, ramping for %.2f s\n",
                Trapezoid::eta_s(a.travel_low_mm, a.travel_high_mm, l),
                a.vmax_up_mm_s / a.amax_mm_s2);
  }
}

}  // namespace

int main(int argc, char** argv) {
  Opts o;
  if (!parse(argc, argv, o)) return 2;

  MachineProfile m;
  std::string err;
  if (!load_machine(o.profile, m, err)) {
    std::fprintf(stderr, "profile error: %s\n", err.c_str());
    return 1;
  }
  print_profile(m);
  if (o.profile_only) return 0;

  FakeBench b;
  b.setup(m);
  b.true_mm = m.axis[0].travel_low_mm + (m.axis[0].travel_high_mm - m.axis[0].travel_low_mm) * 0.5f;
  b.true_mm_b = b.true_mm;
  if (o.slip_at > 0.0f) b.slip_threshold_mm_s = o.slip_at;
  b.slip_rate = o.slip_rate;
  b.leg2_drag = o.leg2_drag;
  if (o.stall_at > 0.0f) { b.obstacle_stop_mm_ = o.stall_at; }

  std::printf("\nstart: %.1f mm (uncalibrated, like powering up a real desk)\n\n",
              b.true_mm);

  auto status = [&](const char* tag) {
    std::printf("%-12s h=%8.2f mm  target=%8.2f  %7.3f mm/s  %-12s%s%s%s%s\n",
                tag, b.ctl.height_mm(), b.ctl.target_mm(), b.ctl.rate_mm_s(),
                name(b.ctl.state()),
                b.ctl.faults().stall ? " STALL" : "",
                b.ctl.faults().position_lost ? " LOST" : "",
                b.ctl.faults().over_temp ? " HOT" : "",
                b.ctl.faults().estop ? " E-STOP" : "");
  };

  auto run_for = [&](us_t us) {
    const float dt = 0.002f;
    for (us_t t = 0; t < us; t += (us_t)(dt * 1e6f)) {
      b.step(dt);
      if (o.stall_at > 0.0f && b.ctl.rate_mm_s() < 0.0f &&
          b.true_mm <= o.stall_at + 0.5f)
        b.obstacle = true;
      if (!o.quiet && (t % (us_t)(2'000'000)) == 0 && b.ctl.moving())
        status("  ...");
    }
  };

  if (o.script.empty()) {
    // Default demo: the sequence a builder should trust first.
    o.script = {"home", "status", "go=950", "status", "go=700", "status"};
    if (m.axis[0].travel_high_mm - 20.0f < 950.0f)
      o.script = {"home", "status",
                  "go=" + std::to_string((int)(m.axis[0].travel_high_mm - 20)),
                  "status",
                  "go=" + std::to_string((int)(m.axis[0].travel_low_mm + 40)),
                  "status"};
  }

  for (const std::string& cmd : o.script) {
    std::string c = cmd, val;
    auto eq = c.find('=');
    if (eq != std::string::npos) { val = c.substr(eq + 1); c = c.substr(0, eq); }

    if (c == "home") {
      std::printf("[home] hunting the low hard stop at %.1f mm/s...\n",
                  m.safety.home_rate_mm_s);
      b.ctl.home();
      run_for(600ULL * 1000000ULL);
      status("home done");
    } else if (c == "go") {
      mm_t mm = (mm_t)std::strtof(val.c_str(), nullptr);
      Status s = b.ctl.move_to(mm);
      std::printf("[go %8.1f] -> %s", mm, name(s));
      if (s == Status::Ok) std::printf("  eta %.0f s", b.ctl.eta_s());
      std::printf("\n");
      if (s == Status::Ok) { run_for(600ULL * 1000000ULL); status("  settled"); }
    } else if (c == "up" || c == "down") {
      bool up = (c == "up");
      std::printf("[%s] holding...\n", c.c_str());
      for (int i = 0; i < 100; ++i) {   // 200 ms of held button
        b.in.button_up = up; b.in.button_down = !up;
        b.step(0.002f);
      }
      b.in.button_up = b.in.button_down = false;
      run_for(20ULL * 1000000ULL);
      status("jog done");
    } else if (c == "stop") {
      b.ctl.stop(); run_for(10ULL * 1000000ULL); status("stopped");
    } else if (c == "ack") {
      b.ctl.acknowledge(); status("acked");
    } else if (c == "preset") {
      mm_t mm = (mm_t)std::strtof(val.c_str(), nullptr);
      b.ctl.save_preset(0, mm); std::printf("[preset] saved %.1f mm\n", mm);
    } else if (c == "preset_go") {
      b.ctl.load_presets();
      Status s = b.ctl.go_to_preset((u8)std::strtol(val.c_str(), nullptr, 10));
      std::printf("[preset_go] -> %s\n", name(s));
      if (s == Status::Ok) { run_for(600ULL * 1000000ULL); status("  settled"); }
    } else if (c == "wait") {
      run_for((us_t)(std::strtof(val.c_str(), nullptr) * 1e6f));
      status("waited");
    } else if (c == "status") {
      status("status");
      std::printf("             truth=%.2f mm  calibrated=%s  cooling=%s\n",
                  b.true_mm, b.ctl.calibrated() ? "yes" : "NO",
                  b.ctl.cooling_down() ? "yes" : "no");
    } else {
      std::fprintf(stderr, "unknown command '%s'\n", c.c_str());
    }
  }

  const bool clean = !b.ctl.faults().any_hard();
  std::printf("\n%s\n", clean ? "run finished clean" : "run ended with a latched fault");
  return clean ? 0 : 3;
}
