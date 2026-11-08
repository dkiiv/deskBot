// deskbot firmware - ESP32-S3 target implementing desk::Hal over deskbot's
// hardware: TMC5160 (STEP/DIR), AS5600 (I2C absolute encoder), 2 limit
// switches, 3 buttons, an NC enable chain, a TMP36 on the driver heatsink,
// and a status LED.
//
// Structure:
//   setup()        pins -> embedded profile -> calibration -> controller -> tasks
//   control_task   fixed 2 ms period, core1, priority 4: controller.tick(0.002)
//                  + button policy (home gesture, e-stop acknowledge)
//   status_task    core0, priority 1: 500 ms status line, serial commands,
//                  draining the HAL log ring
//
// Nothing in core/ is modified. If something below looks like it compensates
// for core behaviour, it is written up in docs/firmware-design-notes.md instead of patched.
//
// Safety stance (read before touching):
//   * Every stub is marked `// STUB:` and listed in README.md. No stub
//     silently pretends a peripheral is alive.
//   * Any sensor read failure disables the *feature it guards*, never
//     re-enables it optimistically. A failed encoder read reports
//     encoder_valid=false for that tick (never a repeated last count), and a
//     lost revolution datum latches until a re-home.
//   * Motor power and STEP are the two outputs whose wrong state can hurt
//     someone; both fail to "off" on reset: GPIO defaults are floating, the
//     ENABLE line is board-pulled inactive, and LEDC duty resets to 0.
#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <math.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <atomic>

#include "desk/controller.h"
#include "desk/hal.h"

#include "deskbot/encoder_as5600.h"
#include "deskbot/json_embed.h"
#include "deskbot/profile_embed.h"
#include "deskbot/tick_guard.h"
#include "generated/machine_profile_json.h"

// core/ aliases its integer types inside desk:: (u8, u32, us_t...) and keeps
// its vocabulary types in namespace desk. Pull in exactly what this file
// touches: `using namespace desk` would also drag name(Status) /
// name(MachineState) into overload resolution next to Arduino symbols, which
// produces very confusing errors far from here.
using desk::i64;
using desk::mm_t;
using desk::rate_t;
using desk::u8;
using desk::u32;
using desk::us_t;
using desk::kMaxAxis;
using desk::Controller;
using desk::MachineProfile;
using desk::AxisProfile;
using desk::SafetyProfile;
using desk::Calibration;
using desk::Inputs;
using desk::Faults;
using desk::Status;
using desk::DriveKind;
using desk::FeedbackKind;
using desk::load_machine;
using desk::name;   // the two overloads arrive as a set

// ===========================================================================
// PIN MAP - one block, every pin tied to the HAL method it serves.
// Numbers are GPIOs on an ESP32-S3-DevKitC-1 (8 MB quad flash). Avoided:
// 0/3/45/46 (strapping), 19/20 (native USB - this board's console), 43/44
// (UART0, flashing), 26-32 (flash). Temperature sits on ADC1 deliberately:
// ADC2 (GPIO11-20) is unusable while WiFi runs, and this board has WiFi.
// Re-check against your board's variant before ordering a PCB.
//
//   GPIO 12  STEP0    drive_rate(0, ...)  LEDC ch0, 50% duty square wave
//   GPIO 13  DIR0     drive_rate(0, ...)  level DIR_LEVEL_POSITIVE = the
//                                         profile's positive direction
//   GPIO 16  STEP1    drive_rate(1, ...)  LEDC ch2 (own timer - see init)
//   GPIO 17  DIR1     drive_rate(1, ...)
//   GPIO 14  EN       set_motor_power()   TMC5160 EN; ENABLE_ACTIVE_LOW says
//                                         which level means "drives". The
//                                         board pulls this line to the
//                                         *disabled* level so an unpowered or
//                                         floating MCU = no motor power.
//   GPIO 21  BRAKE    set_brake()         HIGH = brake engaged (via relay).
//   GPIO 4   LIM_LO0  read_inputs()       reed to GND, internal pull-up,
//                                         LOW = at low hard stop. Wire NC if
//                                         you can: a broken wire then reads
//                                         as "at limit" = the safe direction.
//   GPIO 5   LIM_HI0  read_inputs()       same convention, high stop.
//   GPIO 6   LIM_LO1  read_inputs()       second-leg limits, used only when
//   GPIO 7   LIM_HI1  read_inputs()       the profile declares 2 axes.
//   GPIO 10  BTN_UP   read_inputs()       to GND, pull-up, LOW = pressed.
//   GPIO 11  BTN_DOWN read_inputs()
//   GPIO 18  BTN_STOP read_inputs()       e-stop *tap*: the e-stop itself is
//                                         an NC switch in series with motor
//                                         power; this GPIO only reports it.
//   GPIO 15  ENSYNC   read_inputs()       enable-chain sense (chain closed =>
//                                         LOW). NOT debounced, deliberately:
//                                         see read_inputs().
//   GPIO 1   TEMP     read_inputs()       TMP36 on ADC1_CH0 w/ 11 dB pad:
//                                         0 degC = 0.5 V, 10 mV/degC. Below
//                                         ~60 mV = sensor missing => reported
//                                         as scorching so the core refuses
//                                         to move, not as cold.
//   GPIO 8   I2C_SDA  read_inputs()       AS5600 (and future OLED) shared bus,
//   GPIO 9   I2C_SCL  read_inputs()       external 4k7 pull-ups required.
//   GPIO 48  LED      set_led()           onboard WS2812 data pin, driven as
//                                         plain GPIO here - see set_led().
//   GPIO 2   BUZZ     set_buzzer()        active buzzer driver, HIGH = on.
// ===========================================================================
static constexpr uint8_t PIN_STEP[kMaxAxis]       = {12, 16};
static constexpr uint8_t PIN_DIR[kMaxAxis]        = {13, 17};
static constexpr uint8_t PIN_LIMIT_LOW[kMaxAxis]  = {4, 6};
static constexpr uint8_t PIN_LIMIT_HIGH[kMaxAxis] = {5, 7};
static constexpr uint8_t PIN_BTN_UP               = 10;
static constexpr uint8_t PIN_BTN_DOWN             = 11;
static constexpr uint8_t PIN_BTN_STOP             = 18;
static constexpr uint8_t PIN_ENABLE_CHAIN         = 15;
static constexpr uint8_t PIN_TEMP                 = 1;
static constexpr uint8_t PIN_I2C_SDA              = 8;
static constexpr uint8_t PIN_I2C_SCL              = 9;
static constexpr uint8_t PIN_ENABLE               = 14;
static constexpr uint8_t PIN_BRAKE                = 21;
static constexpr uint8_t PIN_LED                  = 48;
static constexpr uint8_t PIN_BUZZER               = 2;

// Sign convention (matches machine_profile.h and Controller::command_axis):
// the core signs every rate through mm_to_steps(), which multiplies by
// axis.direction. So drive_rate(+x) already means "the direction this profile
// calls rising" and drive_rate(-x) means falling, whatever the wiring does.
// DIR_LEVEL_POSITIVE is the electrical level for that positive direction.
// If the desk rises when DIR is LOW, set this to LOW here - or better, set
// `direction: -1` in the machine JSON and leave the wiring boring. Never do
// both: two inversions that each "look right" are the worst kind of position
// bug. The core also integrates its position model from this same signed
// rate, so a disagreement here does not just invert a button - it guarantees
// a phantom stall fault on the first move.
static constexpr uint8_t DIR_LEVEL_POSITIVE = HIGH;

// TMC5160 EN is active-high = disabled. The control board inverts, so
// "motor power on" = GPIO HIGH:
static constexpr bool ENABLE_ACTIVE_LOW = false;  // true if the board has no inverter

// ===========================================================================
// Control-loop and sensor configuration
// ===========================================================================
static constexpr uint32_t kTickUs = 2000;            // 2 ms, fixed
static constexpr float kTickDtS = kTickUs / 1000000.0f;
static constexpr uint32_t kStatusMs = 500;
static constexpr uint32_t kDebounceLimitMs = 20;     // limit switches
static constexpr uint32_t kDebounceButtonMs = 30;    // buttons
// Homing gesture: UP+DOWN held this long (buttons are otherwise pure jogs -
// the core has no "home" button, and adding silent home-on-boot would move a
// heavy top nobody asked to move).
static constexpr uint32_t kHomeGestureMs = 2000;

// STEP pulse ceiling. 14 mm/s on the Husky profile is ~41 kHz; 120 kHz leaves
// room for a faster desk at 48 V. Above this the HAL *clamps and counts*:
// the model then runs ahead of reality and the encoder's stall detector
// catches it, which is the designed failure path, not a silent one.
static constexpr uint32_t kMaxStepHz = 120000;

// Below this many steps/s we stop pulsing rather than clamp up (clamping
// would move the motor FASTER than commanded at the tail of every ramp -
// worse than momentarily dropping it). 10 Hz sits just above the hardware
// floor computed below (9.5 Hz), so in practice nothing is ever dropped.
static constexpr float kMinStepHz = 10.0f;

// LEDC band derivation, from this framework's own source (verified against
// cores/esp32/esp32-hal-ledc.c and the S3 SDK headers, not guessed):
//   source clock: esp32-hal-ledc.c uses LEDC_USE_XTAL_CLK because the S3's
//     soc/ledc_caps.h defines SOC_LEDC_SUPPORT_XTAL_CLOCK=1  ->  40 MHz
//   divider field (ledc_reg.h LSTIMERx, bits [21:4]): 18-bit integer + 4-bit
//     fraction, so the effective divider runs ~1.94..262143 and
//     output freq = 40 MHz / (div * 2^res)
//   at res = 4:  band = [9.5 Hz .. 1.25 MHz], 50% duty = 8/16 unambiguous;
//     kMinStepHz sits just above the floor so nothing is ever silently lost
//   rate quantum: the divider steps in 1/16, so the relative frequency error
//     is <= 1/(16*div); at our ceiling (120 kHz -> div 21) that is ~0.3% and
//     it improves toward low rates. Against the core's stall margin (1.6 mm
//     sustained 220 ms) even 0.3% is two orders of magnitude below trip -
//     which is why this was checked against the registers and not assumed.
//   out-of-band requests are NOT clamped silently: the IDF driver returns
//   ESP_FAIL when no pre-divider exists for the requested frequency
//   (driver/ledc.h documents it), the wrapper turns that into 0, and both
//   call sites below treat 0 as an error.
// If a future SDK flips the source to 80 MHz APB the band shifts up 2x
// (floor 19 Hz, ceiling 2.5 MHz): still fine for [kMinStepHz, kMaxStepHz],
// and init() prints ledcReadFreq() of the init point so a surprise shows up
// at boot rather than as a scale error on the bench.
static constexpr uint8_t kPwmResBits = 4;
static constexpr uint32_t kPwmInitHz = 1000;   // always representable; duty 0
static constexpr uint32_t kPwmDuty50 = 1u << (kPwmResBits - 1);

static constexpr uint32_t kI2cHz = 100000;  // AS5600 tolerates 400k on short
                                            // wires; 100k is the noise-
                                            // tolerant choice for a bench.
static constexpr uint8_t kAs5600Addr[kMaxAxis] = {0x36, 0x37};
static constexpr uint8_t kAs5600RegAngle = 0x0C;  // RAWANGLE (read mode 3)

// The AS5600 angle increases with CCW magnet rotation (datasheet). Whether
// that is "desk rising" is a mount decision. Set at bring-up: raise the top
// by hand and watch `cnt=` in the status line - it must increase.
static constexpr bool kEncoderInvert = false;

// ===========================================================================
// Globals (single instances; no heap on the control path)
// ===========================================================================
namespace {

json::Dom g_dom;
MachineProfile g_profile;
Calibration g_cal;
Controller g_ctrl;

// HAL log ring: log() runs on the control task; the status task drains it so
// a Serial.print with USB flow control can never stall the 2 ms loop.
constexpr int kLogLines = 16;
constexpr int kLogLen = 110;
char g_log_ring[kLogLines][kLogLen];
int g_log_head = 0;   // next write slot
int g_log_tail = 0;   // next read slot
portMUX_TYPE g_log_mux = portMUX_INITIALIZER_UNLOCKED;

enum CmdKind : uint8_t {
  kCmdHome = 1, kCmdAck, kCmdResetDuty, kCmdGoto, kCmdStop, kCmdClearEnc,
};
struct Cmd {
  uint8_t kind;
  float arg;
};
QueueHandle_t g_cmd_q = nullptr;

// Encoder datum latch (see deskbot/encoder_as5600.h). Cleared only by an
// explicit operator action, never by a good-looking sample.
bool g_enc_latched = false;

void ring_log(const char* fmt, ...) {
  char buf[kLogLen];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  taskENTER_CRITICAL(&g_log_mux);
  const int next = (g_log_head + 1) % kLogLines;
  if (next != g_log_tail) {   // full => drop the newest line, keep draining
    memcpy(g_log_ring[g_log_head], buf, sizeof(g_log_ring[0]));
    g_log_head = next;
  }
  taskEXIT_CRITICAL(&g_log_mux);
}

}  // namespace

// ===========================================================================
// Debounced digital input (non-blocking, tick-sampled; shared clock per tick)
// ===========================================================================
namespace {
class DebouncedPin {
 public:
  void begin(uint8_t p, bool active_low, uint32_t window_ms) {
    pin_ = p;
    active_low_ = active_low;
    window_ms_ = window_ms;
    pinMode(p, INPUT_PULLUP);
    stable_ = read_raw();
    last_change_ms_ = millis();
  }
  // Call once per tick with the same `now_ms` for every pin.
  void sample(uint32_t now_ms) {
    const bool r = read_raw();
    if (r != stable_) {
      if (now_ms - last_change_ms_ >= window_ms_) stable_ = r;
    } else {
      last_change_ms_ = now_ms;
    }
  }
  bool stable() const { return stable_; }

 private:
  bool read_raw() const {
    const bool v = digitalRead(pin_) == HIGH;
    return active_low_ ? !v : v;
  }
  uint8_t pin_ = 0;
  bool active_low_ = true;
  uint32_t window_ms_ = 20;
  bool stable_ = false;
  uint32_t last_change_ms_ = 0;
};
}  // namespace

// ===========================================================================
// ESPHal - the desk::Hal implementation
// ===========================================================================
class ESPHal final : public desk::Hal {
 public:
  // ------------------------------------------------------------- lifecycle
  // Returns false on a hard init failure (LEDC exhausted). Caller must not
  // start the control task on false: half a HAL is worse than a dead one.
  bool init(uint8_t axis_count, bool wants_encoder) {
    for (u8 a = 0; a < kMaxAxis; ++a) {
      axis_[a] = AxisPwm{};
      pinMode(PIN_DIR[a], OUTPUT);
      digitalWrite(PIN_DIR[a], DIR_LEVEL_POSITIVE == HIGH ? LOW : HIGH);
      pinMode(PIN_STEP[a], OUTPUT);
      digitalWrite(PIN_STEP[a], LOW);
      if (a < axis_count) {
        // Channel/timer choice is not cosmetic. ledcSetup() derives the timer
        // from the channel (timer = (chan/2)%4), so axes on channels 0 and 1
        // would SHARE a timer and every rate change of one axis would silently
        // retune the other. Channels 0 and 2 get independent timers.
        //
        // kPwmResBits / kPwmInitHz and the band they imply are derived (with
        // sources) next to their definitions above; the init frequency is
        // deliberately representable because ledcSetup() returns 0 for an
        // impossible request, and a 0 here would abort boot as "HAL init
        // failed" - the worst possible error to get from a pin that is about
        // to be held at duty 0 anyway.
        axis_[a].channel = static_cast<uint8_t>(a * 2);
        const uint32_t got = ledcSetup(axis_[a].channel, kPwmInitHz, kPwmResBits);
        ledcAttachPin(PIN_STEP[a], axis_[a].channel);
        ledcWrite(axis_[a].channel, 0);
        if (got == 0) return false;
        axis_[a].init_hz = got;
        axis_[a].ready = true;
        axis_[a].hz = 0;
      }
    }
    pinMode(PIN_ENABLE, OUTPUT);
    apply_motor_power(false);
    pinMode(PIN_BRAKE, OUTPUT);
    digitalWrite(PIN_BRAKE, LOW);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    lim_low_[0].begin(PIN_LIMIT_LOW[0], true, kDebounceLimitMs);
    lim_high_[0].begin(PIN_LIMIT_HIGH[0], true, kDebounceLimitMs);
    lim_low_[1].begin(PIN_LIMIT_LOW[1], true, kDebounceLimitMs);
    lim_high_[1].begin(PIN_LIMIT_HIGH[1], true, kDebounceLimitMs);
    btn_up_.begin(PIN_BTN_UP, true, kDebounceButtonMs);
    btn_down_.begin(PIN_BTN_DOWN, true, kDebounceButtonMs);
    btn_stop_.begin(PIN_BTN_STOP, true, kDebounceButtonMs);
    // Enable chain: deliberately NOT debounced. The core evaluates it before
    // anything else in the supervisor and cuts power within the tick; a
    // debounce window here would be a window where a tripped chain is
    // ignored. An NC chain fails safe against noise (a bounce reads "open"
    // = stop, never "closed" = run).
    pinMode(PIN_ENABLE_CHAIN, INPUT_PULLUP);

    analogSetPinAttenuation(PIN_TEMP, ADC_11db);

    if (wants_encoder) {
      if (bus_lock_ == nullptr) bus_lock_ = xSemaphoreCreateMutex();
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, kI2cHz);
      encoder_wanted_ = true;
      magnet_ok_ = probe_magnet();
      for (u8 a = 0; a < kMaxAxis; ++a) {
        const u32 cpr = g_profile.axis[a].encoder_counts_per_motor_rev;
        enc_track_[a].configure(static_cast<int32_t>(cpr ? cpr : 4096));
      }
    }

    if (nvs_open("deskbot", NVS_READWRITE, &nvs_) != ESP_OK) nvs_ = 0;

    // Print what the pulse generator actually negotiated, so a source-clock
    // or SDK change that moves the frequency band is visible at boot instead
    // of becoming an unexplained scale error on the bench.
    for (u8 a = 0; a < kMaxAxis; ++a) {
      if (axis_[a].ready) {
        // ledcSetup() returns the frequency the timer actually negotiated
        // (ledcReadFreq() cannot be used here: it reports 0 while duty is 0).
        Serial.printf("ledc: axis %u ch=%u init=%lu Hz (asked %lu, res=%u); "
                      "stop below %.0f/s, clamp above %lu/s, both counted\n",
                      a, axis_[a].channel,
                      static_cast<unsigned long>(axis_[a].init_hz),
                      static_cast<unsigned long>(kPwmInitHz),
                      static_cast<unsigned>(kPwmResBits),
                      static_cast<double>(kMinStepHz),
                      static_cast<unsigned long>(kMaxStepHz));
      }
    }
    return true;
  }

  // ------------------------------------------------------------- desk::Hal
  us_t now_us() const override { return static_cast<us_t>(esp_timer_get_time()); }

  // The control task calls begin_tick() once per 2 ms period, before tick().
  // read_inputs() then samples hardware at most once per period: a second
  // caller in the same period gets the cached snapshot. That matters because
  // the encoder unwrap must advance exactly once per control period (its
  // plausibility bound is counts-per-period) and I2C is the most expensive
  // thing in the loop. The counter pair lives in deskbot/tick_guard.h so the
  // invariant is host-tested; it used to be two bare members here, one of which
  // (cache_tick_) was never assigned, so the guard never engaged and the
  // encoder was read twice per tick. See that header for the full story.
  void begin_tick() { guard_.begin(); }

  void read_inputs(Inputs& out) override {
    if (guard_.already_sampled()) {   // already sampled this period
      out = same_tick_copy_;
      return;
    }
    Inputs in{};
    const uint32_t ms = millis();
    lim_low_[0].sample(ms);
    lim_high_[0].sample(ms);
    lim_low_[1].sample(ms);
    lim_high_[1].sample(ms);
    btn_up_.sample(ms);
    btn_down_.sample(ms);
    btn_stop_.sample(ms);

    in.limit_low[0] = lim_low_[0].stable();
    in.limit_high[0] = lim_high_[0].stable();
    in.limit_low[1] = lim_low_[1].stable();
    in.limit_high[1] = lim_high_[1].stable();
    in.button_up = btn_up_.stable();
    in.button_down = btn_down_.stable();
    in.button_stop = btn_stop_.stable();
    // Open chain reads HIGH with the pull-up. Unknown must mean do-not-move,
    // and "HIGH or floating" is exactly what a broken wire looks like:
    // enable_closed=false. The safe direction.
    in.enable_closed = (digitalRead(PIN_ENABLE_CHAIN) == LOW);

    in.driver_temp_c = read_temp_c();

    // --- encoder: see deskbot/encoder_as5600.h for the unwrap rationale ----
    // Any failure sets encoder_valid for *this tick* only (never repeats the
    // last count: a frozen encoder reads as "desk holds position" to the stall
    // detector, which is precisely the lie this system exists to catch).
    bool enc_ok = encoder_wanted_ && magnet_ok_ && !g_enc_latched;
    if (enc_ok) {
      // The bus is shared with the status task (and later anything else on
      // I2C). Wire's internal lock covers one transaction, not the
      // pointer-write + read pair an AS5600 angle read is, and interleaving
      // there would corrupt the unwrap datum - so the bus is held explicitly.
      if (xSemaphoreTake(bus_lock_, pdMS_TO_TICKS(5)) != pdTRUE) {
        ++bus_lock_timeouts_;
        enc_ok = false;                 // fail this tick, never fake a count
      } else {
        for (u8 a = 0; a < g_profile.axis_count && a < kMaxAxis; ++a) {
          uint16_t raw = 0;
          if (!read_angle(kAs5600Addr[a], &raw)) {
            ++i2c_errors_;
            enc_ok = false;
            break;
          }
          int64_t counts = 0;
          if (!enc_track_[a].update(raw, &counts)) {
            ++enc_faults_;
            if (!g_enc_latched)
              ring_log("encoder: axis %u datum lost - HOME REQUIRED", a);
            g_enc_latched = true;   // datum lost: sticky until ack + re-home
            enc_ok = false;
            break;
          }
          in.encoder_counts[a] = kEncoderInvert ? -counts : counts;
        }
        xSemaphoreGive(bus_lock_);
      }
    }
    in.encoder_valid = enc_ok;
    last_enc_ok_ = enc_ok;

    // publish() fills same_tick_copy_, so from this line the period's snapshot
    // exists and a second caller can be served it. Marking before publish would
    // let the second reader see a stale cache; marking after out = in would work
    // but reads like the copy is the trigger. It is the publish.
    publish(in);
    guard_.mark_sampled();
    out = in;
  }

  // Most recent published snapshot. Safe from any task: no hardware access,
  // and the seqlock hands a racing reader the *previous* complete frame
  // instead of a mix of two (one tick of staleness on a 500 ms status line
  // is nothing; a torn frame printed as fact is exactly what this file
  // promises not to do).
  Inputs snapshot() const {
    Inputs a;
    for (int attempt = 0; attempt < 4; ++attempt) {
      const uint32_t s1 = seq_;
      std::atomic_thread_fence(std::memory_order_acquire);
      a = buf_[(s1 ^ 1u) & 1u];
      if (seq_ == s1) return a;
    }
    return a;   // pathological contention: still a whole frame, just old
  }

  // Called from the control task only (the core's contract). LEDC writes are
  // register-level and safe, but two callers racing setMotorPower would not
  // be, so this is a task-only method by discipline, not by locking.
  void drive_rate(u8 axis, rate_t steps_per_s) override {
    if (axis >= kMaxAxis) return;
    AxisPwm& a = axis_[axis];
    if (!a.ready) return;   // axis not in this profile: keep silent

    if (!std::isfinite(steps_per_s)) steps_per_s = 0.0f;
    const rate_t mag = steps_per_s < 0.0f ? -steps_per_s : steps_per_s;
    if (mag < kMinStepHz) {
      // Zero (or sub-quantum) rate: stop pulsing. Duty 0 parks STEP low; the
      // driver sees no edges. The channel stays attached: detaching costs
      // more than it saves and could glitch the line mid-pulse, and a
      // glitched STEP is a phantom microstep.
      if (a.hz != 0) {
        ledcWrite(a.channel, 0);
        a.hz = 0;
      }
      return;
    }
    const bool positive = steps_per_s > 0.0f;
    digitalWrite(PIN_DIR[axis],
                 positive ? DIR_LEVEL_POSITIVE
                          : static_cast<uint8_t>(DIR_LEVEL_POSITIVE == HIGH ? LOW : HIGH));
    uint32_t hz = static_cast<uint32_t>(mag + 0.5f);
    if (hz > kMaxStepHz) {
      ++rate_clamps_;   // model runs ahead; encoder catches it. Counted.
      hz = kMaxStepHz;
    }
    if (hz != a.hz) {
      // Park the pin before re-tuning: ledc's timer restart while the channel
      // is live can stretch or squash one edge, and a fat half-pulse on STEP
      // is a phantom microstep. With duty 0 first, the worst case across the
      // retune is a *missed* step on the same tick - same detectability via
      // the encoder, but it cannot make the mechanism move further than the
      // model thinks it already moved.
      ledcWrite(a.channel, 0);
      // ledcChangeFrequency re-tunes at the given resolution and returns 0 if
      // the request cannot be represented. On 0, stay stopped and count it:
      // silently continuing at last tick's rate while commanding another one
      // is exactly the "told the core something that is not true" failure.
      const uint32_t got = ledcChangeFrequency(a.channel, hz, kPwmResBits);
      if (got == 0) {
        ++rate_rejects_;
        a.hz = 0;    // we are stopped now, whatever the old rate was
        return;
      }
      ledcWrite(a.channel, kPwmDuty50);   // 50% square wave at `hz`
      a.hz = hz;
    }
  }

  void set_motor_power(bool on) override { apply_motor_power(on); }

  void set_brake(bool on) override {
    // A powered holding brake releases only when explicitly commanded; the
    // core does that (Outputs::brake), and de-energising on every fault path
    // is the brake's own fail-safe (springs clamp when power drops).
    digitalWrite(PIN_BRAKE, on ? HIGH : LOW);
  }

  // Core pattern semantics: 0 off, 1 moving, 2 fault, 3 need-home.
  // STUB: the devkit LED is a WS2812 on GPIO48; driving the data line as a
  // plain GPIO yields a dim single-colour blink, enough to read state during
  // bring-up. Real colour/pattern work (NeoPixel/RMT) is deliberately not in
  // the control path. If you need patterns, do it in the status task.
  void set_led(u8 pattern) override {
    bool on = false;
    switch (pattern) {
      case 1: on = true; break;                          // moving: solid
      case 2: on = ((millis() / 120) & 1) != 0; break;   // fault: fast blink
      case 3: on = ((millis() / 500) & 1) != 0; break;   // need-home: slow
      default: on = false; break;
    }
    digitalWrite(PIN_LED, on ? HIGH : LOW);
  }

  void set_buzzer(bool on) override { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }

  // Never print from here: this runs on the 2 ms control task and HWCDC
  // writes can block up to their tx timeout when no host is attached. The
  // status task drains the ring and owns the wire.
  void log(const char* msg) override { ring_log("%s", msg); }

  // -------------------------------------------------------------- NVS
  // Calibration is written once per home, presets on save - low rate, but
  // nvs_set_blob is a wear-levelled flash write and the core does not
  // throttle, so any new caller of nv_write must think first.
  bool nv_read(u8 key, void* buf, u32 len) override {
    if (nvs_ == 0) return false;
    const char* k = nvs_key(key);
    if (!k) return false;
    size_t want = len;
    if (nvs_get_blob(nvs_, k, nullptr, &want) != ESP_OK) return false;
    if (want != len) return false;   // struct changed: no data, never a
                                     // half-filled Calibration
    return nvs_get_blob(nvs_, k, buf, &want) == ESP_OK;
  }

  bool nv_write(u8 key, const void* buf, u32 len) override {
    if (nvs_ == 0) return false;
    const char* k = nvs_key(key);
    if (!k) return false;
    if (nvs_set_blob(nvs_, k, buf, len) != ESP_OK) return false;
    return nvs_commit(nvs_) == ESP_OK;
  }

  // -------------------------------------------------------------- extras
  // Publishes an input snapshot without ever blocking the writer: the writer
  // bumps seq then writes buf_[seq & 1]; readers take buf_[~seq & 1] - the
  // previous complete frame - and retry if seq moved under them.
  void publish(const Inputs& in) {
    same_tick_copy_ = in;
    ++seq_;
    std::atomic_thread_fence(std::memory_order_release);
    buf_[seq_ & 1u] = in;
  }

  float read_temp_c() {
    const uint32_t mv = analogReadMilliVolts(PIN_TEMP);
    if (mv < 60) {
      // Near-zero rail = sensor absent, cut, or shorted to GND. Report far
      // above temp_c_stop so the supervisor latches over_temp and the desk
      // refuses to move, instead of believing "cold" from an open wire.
      temp_sensor_fault_ = true;
      return 999.0f;
    }
    temp_sensor_fault_ = false;
    // TMP36: 0 degC at 500 mV, 10 mV/degC. If an NTC divider is fitted
    // instead, replace this whole function - do not "tune" these two numbers.
    return (static_cast<float>(mv) - 500.0f) / 10.0f;
  }

  uint32_t i2c_errors() const { return i2c_errors_; }
  uint32_t rate_clamps() const { return rate_clamps_; }
  uint32_t rate_rejects() const { return rate_rejects_; }
  uint32_t enc_faults() const { return enc_faults_; }
  uint32_t bus_timeouts() const { return bus_lock_timeouts_; }
  bool temp_sensor_fault() const { return temp_sensor_fault_; }
  bool encoder_ok_now() const { return last_enc_ok_; }
  bool encoder_latched() const { return g_enc_latched; }
  void clear_encoder_latch() {
    g_enc_latched = false;
    for (u8 a = 0; a < kMaxAxis; ++a) enc_track_[a].reset();
  }

 private:
  struct AxisPwm {
    uint8_t channel = 0;
    uint32_t hz = 0;        // current pulse rate, 0 = not pulsing
    uint32_t init_hz = 0;   // what ledcSetup() actually produced at init
    bool ready = false;
  };

  void apply_motor_power(bool on) {
    // Written even when unchanged (init path); idempotent for the core.
    // Ordering note: the core only raises motor power while it also commands
    // a rate, and drops it to zero first, so ENABLE never asserts against a
    // parked shaft with charge in the windings.
    digitalWrite(PIN_ENABLE, (on ^ ENABLE_ACTIVE_LOW) ? HIGH : LOW);
    motor_power_on_ = on;
  }

  static const char* nvs_key(u8 k) {
    switch (k) {
      case desk::kNvsCalibration: return "cal";
      case desk::kNvsPresets: return "pre";
      case desk::kNvsProfile: return "pro";
      case desk::kNvsCounters: return "cnt";
      default: return nullptr;   // unknown key: refuse, do not invent a slot
    }
  }

  // RAWANGLE read: write pointer, re-issue START, read 2 bytes. A NACK at
  // any point is a failed read - never a zero-angle read (zero is a legal
  // angle; failure is not).
  bool read_angle(uint8_t addr, uint16_t* out) {
    Wire.beginTransmission(addr);
    Wire.write(kAs5600RegAngle);
    if (Wire.endTransmission(true) != 0) return false;
    if (Wire.requestFrom(static_cast<int>(addr), 2) != 2) return false;
    if (Wire.available() < 2) return false;
    const uint16_t hi = Wire.read();
    const uint16_t lo = Wire.read();
    *out = static_cast<uint16_t>(((hi & 0x0F) << 8) | lo);
    return true;
  }

  bool probe_magnet() {
    uint16_t r0 = 0, r1 = 0;
    if (!read_angle(kAs5600Addr[0], &r0)) return false;
    delay(5);   // init-only; the bus is quiet until the control task runs
    if (!read_angle(kAs5600Addr[0], &r1)) return false;
    // AGC=0xFFF in status/raw means "no magnet detected". Both reads equal
    // 0xFFF is the documented signature; a single 0xFFF between valid reads
    // is a legal angle near the top of range.
    return !(r0 == 0x0FFF && r1 == 0x0FFF);
  }

  AxisPwm axis_[kMaxAxis] = {};
  DebouncedPin lim_low_[kMaxAxis];
  DebouncedPin lim_high_[kMaxAxis];
  DebouncedPin btn_up_, btn_down_, btn_stop_;
  deskbot::As5600Track enc_track_[kMaxAxis];
  // Input snapshot state. guard_ gives "sample once per control period" (see
  // deskbot/tick_guard.h); seq_/buf_ give a lock-free, torn-read-free handoff to
  // the status task (see publish()/snapshot()).
  deskbot::TickGuard guard_;
  Inputs same_tick_copy_{};
  volatile uint32_t seq_ = 0;
  Inputs buf_[2] = {};
  SemaphoreHandle_t bus_lock_ = nullptr;
  uint32_t bus_lock_timeouts_ = 0;
  bool motor_power_on_ = false;
  bool encoder_wanted_ = false;
  bool magnet_ok_ = false;
  bool last_enc_ok_ = false;
  bool temp_sensor_fault_ = false;
  uint32_t i2c_errors_ = 0;
  uint32_t rate_clamps_ = 0;
  uint32_t rate_rejects_ = 0;
  uint32_t enc_faults_ = 0;
  nvs_handle_t nvs_ = 0;
};

static ESPHal g_hal;

// ===========================================================================
// Boot steps (setup, before the control task exists)
// ===========================================================================
namespace {

[[noreturn]] void fatal_loop(const char* reason) {
  // The desk must not move, ever, until the reason is fixed. Kill everything
  // that can move, blink, and repeat the reason over USB at 1 Hz.
  for (;;) {
    digitalWrite(PIN_ENABLE, ENABLE_ACTIVE_LOW ? HIGH : LOW);
    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_LED, ((millis() / 200) & 1) ? HIGH : LOW);
    Serial.printf("FATAL: %s\n", reason);
    delay(1000);
  }
}

void load_profile() {
  const char* err = nullptr;
  if (!load_machine(kMachineJson, sizeof(kMachineJson) - 1, g_dom, g_profile, &err)) {
    char why[192];
    snprintf(why, sizeof(why),
             "embedded machine profile rejected: %s (regenerate "
             "include/generated/machine_profile_json.h from machines/*.json)",
             err ? err : "?");
    fatal_loop(why);
  }
  if (g_profile.drive != DriveKind::StepperPulse)
    fatal_loop("this firmware drives STEP/DIR only (DriveKind::StepperPulse)");
  if (g_profile.axis_count < 1 || g_profile.axis_count > kMaxAxis)
    fatal_loop("profile axis_count out of range");
}

void load_calibration() {
  Calibration fresh{};
  if (g_hal.nv_read(desk::kNvsCalibration, &g_cal, sizeof(g_cal)) &&
      g_cal.magic == fresh.magic &&
      g_cal.profile_version == g_profile.profile_version) {
    return;   // good calibration for this exact profile version
  }
  // Missing, corrupt, or calibrated against different gearing: none of these
  // may borrow the old datum. Presets are deliberately not wiped - they are
  // heights, not offsets, and stay useful across a re-home.
  g_cal = fresh;
  g_cal.profile_version = g_profile.profile_version;
}

}  // namespace

// ===========================================================================
// Tasks
// ===========================================================================
namespace {

// Physical gestures the core does not own. The core handles jog priority,
// e-stop, limits and stalls internally; what is left is (1) a way to start
// homing from the only three buttons we have, and (2) an acknowledge.
void post_cmd(uint8_t kind, float arg);   // fwd (defined below)

struct ControlUI {
  uint32_t gesture_ms = 0;
  bool gesturing = false;
  bool stop_was = false;

  void tick(const Inputs& in) {
    const bool both = in.button_up && in.button_down;
    if (both) {
      if (!gesturing) {
        gesturing = true;
        gesture_ms = millis();
      } else if (millis() - gesture_ms >= kHomeGestureMs) {
        gesturing = false;   // fire once per gesture; release to re-arm
        post_cmd(kCmdHome, 0.0f);
      }
    } else if (!in.button_up && !in.button_down) {
      gesturing = false;
    }

    // STOP = acknowledge on a latched fault. Motion stop itself is already
    // covered: the supervisor treats button_stop as e-stop, so a pressed STOP
    // kills power within one tick without any help from here.
    if (in.button_stop && !stop_was && g_ctrl.needs_attention())
      post_cmd(kCmdAck, 0.0f);
    stop_was = in.button_stop;
  }
} g_ui;

void post_cmd(uint8_t kind, float arg) {
  if (!g_cmd_q) return;
  const Cmd c{kind, arg};
  xQueueSend(g_cmd_q, &c, 0);
}

void apply_cmd(const Cmd& c) {
  Status s{};
  switch (c.kind) {
    case kCmdHome:
      // No latch pre-clearing here any more. This used to work around a core
      // bug (home() gated behind can_move(), so position_lost was a dead end)
      // by calling clear_faults() first - which also dropped estop/over_temp
      // for the interval before the next supervisor tick. The core now treats
      // homing as the sanctioned recovery: it clears only position_lost and
      // skew, and still refuses while estop/over_temp/stall/home_timeout are
      // latched. That policy belongs in core/ and is covered by the
      // home-recovery tests in tests/core/test_core.cpp.
      s = g_ctrl.home();
      ring_log("cmd home -> %s", name(s));
      break;
    case kCmdAck:
      g_ctrl.acknowledge();
      if (g_enc_latched) {
        // Operator ack clears the encoder latch too: the machine is still
        // untrusted (integrity check) and will fault again until re-homed,
        // but a lost encoder datum must not be a brick - the physical e-stop
        // and limits are what protect a person, not the latch.
        g_hal.clear_encoder_latch();
        ring_log("encoder latch cleared - HOME REQUIRED");
      }
      ring_log("cmd ack");
      break;
    case kCmdResetDuty:
      g_ctrl.reset_duty_budget();
      ring_log("cmd duty-reset");
      break;
    case kCmdGoto:
      s = g_ctrl.move_to(c.arg);
      ring_log("cmd goto %.1f -> %s", c.arg, name(s));
      break;
    case kCmdStop:
      g_ctrl.stop();
      ring_log("cmd stop");
      break;
    case kCmdClearEnc:
      g_hal.clear_encoder_latch();
      ring_log("encoder latch cleared - HOME REQUIRED");
      break;
    default: break;
  }
}

void control_task(void*) {
  // If this task dies, every software protection dies with it. The Arduino
  // build ships CONFIG_ESP_TASK_WDT enabled with panic and a 5 s timeout, and
  // loopTask/feed-dog behaviour around idle is not something this firmware
  // should depend on: reconfigure the singleton explicitly (4 s, panic) and
  // subscribe this task only. Any *other* task that stops feeding then also
  // trips it, which is the conservative direction.
  esp_task_wdt_deinit();
  esp_task_wdt_init(4, true);
  esp_task_wdt_add(nullptr);

  vTaskDelay(2);
  TickType_t next = xTaskGetTickCount();

  for (;;) {
    esp_task_wdt_reset();

    // Commands first: a stale remote jog must not gate a button-initiated
    // stop/home behind the planner.
    Cmd c{};
    while (xQueueReceive(g_cmd_q, &c, 0) == pdTRUE) apply_cmd(c);

    g_hal.begin_tick();
    g_ctrl.tick(kTickDtS);

    // Button policy sees the same snapshot the core saw (read_inputs samples
    // hardware once per tick and serves the copy).
    Inputs in{};
    g_hal.read_inputs(in);
    g_ui.tick(in);

    // Cosmetic outputs driven from controller state (Outputs::led_pattern is
    // core-side bookkeeping; the core does not push Outputs to the HAL).
    const bool attn = g_ctrl.needs_attention();
    const u8 pattern = attn ? 2 : (!g_ctrl.calibrated() ? 3 :
                                   (g_ctrl.moving() ? 1 : 0));
    g_hal.set_led(pattern);
    g_hal.set_buzzer(attn);

    // Fixed 2 ms period. Note vTaskDelayUntil is not hard real-time: the
    // core integrates with the *assumed* dt (0.002 s), not measured time, so
    // a scheduler hiccup slows the desk slightly rather than lying about
    // position. The encoder remains the arbiter of reality either way.
    vTaskDelayUntil(&next, pdMS_TO_TICKS(kTickUs / 1000));
  }
}

void drain_log_ring() {
  for (;;) {
    char line[kLogLen];
    bool have = false;
    taskENTER_CRITICAL(&g_log_mux);
    if (g_log_tail != g_log_head) {
      memcpy(line, g_log_ring[g_log_tail], sizeof(line));
      g_log_tail = (g_log_tail + 1) % kLogLines;
      have = true;
    }
    taskEXIT_CRITICAL(&g_log_mux);
    if (!have) return;
    Serial.printf("[%6lums] %s\n", static_cast<unsigned long>(millis()), line);
  }
}

void print_status() {
  const Faults& f = g_ctrl.faults();
  char flags[110];
  flags[0] = '\0';
  auto cat = [&](const char* s) {
    if (strlen(flags) + strlen(s) + 2 < sizeof(flags)) {
      if (flags[0]) strcat(flags, "|");
      strcat(flags, s);
    }
  };
  if (f.stall) cat("stall");
  if (f.skew) cat("skew");
  if (f.blocked) cat("blocked");
  if (f.over_temp) cat("otemp");
  if (f.estop) cat("estop");
  if (f.position_lost) cat("poslost");
  if (f.home_timeout) cat("hometo");
  if (g_enc_latched) cat("enclatch");
  if (g_hal.temp_sensor_fault()) cat("tempsens");
  if (!flags[0]) strcpy(flags, "-");

  Inputs in = g_hal.snapshot();   // last complete published frame; no I2C cost

  Serial.printf(
      "h=%7.2f tgt=%7.2f v=%+6.2f st=%-12s enc=%s cnt=%lld T=%5.1f "
      "f=%s i2c=%lu/%lu clamp=%lu rej=%lu\n",
      static_cast<double>(g_ctrl.height_mm()),
      static_cast<double>(g_ctrl.target_mm()),
      static_cast<double>(g_ctrl.rate_mm_s()),
      name(g_ctrl.state()),
      in.encoder_valid ? "ok " : "OFF",
      static_cast<long long>(in.encoder_counts[0]),
      static_cast<double>(in.driver_temp_c),
      flags,
      static_cast<unsigned long>(g_hal.i2c_errors()),
      static_cast<unsigned long>(g_hal.bus_timeouts()),
      static_cast<unsigned long>(g_hal.rate_clamps()),
      static_cast<unsigned long>(g_hal.rate_rejects()));
  if (g_ctrl.cooling_down()) {
    const us_t now = static_cast<us_t>(esp_timer_get_time());
    const us_t until = g_ctrl.cool_until_us();
    Serial.printf("  cooling %.0fs remaining\n",
                  until > now ? static_cast<double>(until - now) / 1e6 : 0.0);
  }
}

// Tiny line parser: h=home  a=ack  d=duty-reset  e=clear-encoder-latch
//                         s=stop  g<mm>=goto  ?=status now
void feed_serial() {
  static char buf[48];
  static int n = 0;
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (n == 0) continue;
      buf[n] = '\0';
      n = 0;
      switch (buf[0]) {
        case 'h': post_cmd(kCmdHome, 0.0f); break;
        case 'a': post_cmd(kCmdAck, 0.0f); break;
        case 'd': post_cmd(kCmdResetDuty, 0.0f); break;
        case 'e': post_cmd(kCmdClearEnc, 0.0f); break;
        case 's': post_cmd(kCmdStop, 0.0f); break;
        case 'g': post_cmd(kCmdGoto, static_cast<float>(atof(buf + 1))); break;
        case '?': print_status(); break;
        default: Serial.println("cmds: h a d e s g<mm> ?"); break;
      }
    } else if (n < static_cast<int>(sizeof(buf)) - 1) {
      buf[n++] = ch;
    }
  }
}

void status_task(void*) {
  uint32_t last_print = 0;
  for (;;) {
    drain_log_ring();
    feed_serial();
    const uint32_t ms = millis();
    if (ms - last_print >= kStatusMs) {
      last_print = ms;
      print_status();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

}  // namespace

// ===========================================================================
// Arduino entry points
// ===========================================================================
void setup() {
  // Console is USB-CDC via the S3 native USB-Serial-JTAG (board flags
  // ARDUINO_USB_MODE=1 + ARDUINO_USB_CDC_ON_BOOT=1). Cap the write timeout:
  // with no host attached the ring buffer fills and default write() would
  // stall the status task for its whole timeout.
  Serial.begin(115200);
  Serial.setTxTimeoutMs(20);
  delay(100);   // let the host open the port; cosmetic only

  // Safe pin state BEFORE anything else can request motion.
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, ENABLE_ACTIVE_LOW ? HIGH : LOW);
  for (u8 a = 0; a < kMaxAxis; ++a) {
    pinMode(PIN_STEP[a], OUTPUT);
    digitalWrite(PIN_STEP[a], LOW);
    pinMode(PIN_DIR[a], OUTPUT);
    digitalWrite(PIN_DIR[a], LOW);
  }

  Serial.println();
  Serial.printf("deskbot firmware - machine: %s v%lu (built %s %s)\n",
                DESKBOT_MACHINE_NAME,
                static_cast<unsigned long>(DESKBOT_MACHINE_PROFILE_VERSION),
                __DATE__, __TIME__);

  load_profile();
  if (!g_hal.init(g_profile.axis_count,
                  g_profile.feedback != FeedbackKind::None)) {
    fatal_loop("HAL init failed (LEDC channels / pins)");
  }
  load_calibration();

  Serial.printf("profile: %s axes=%u coupling=%d feedback=%d steps/rev=%lld "
                "ratio=%.3f mm/rev=%.3f travel=[%.1f..%.1f]\n",
                g_profile.name ? g_profile.name : "?", g_profile.axis_count,
                static_cast<int>(g_profile.coupling),
                static_cast<int>(g_profile.feedback),
                static_cast<long long>(g_profile.axis[0].steps_per_motor_rev),
                static_cast<double>(g_profile.axis[0].gear_ratio),
                static_cast<double>(g_profile.axis[0].mm_per_shaft_rev),
                static_cast<double>(g_profile.axis[0].travel_low_mm),
                static_cast<double>(g_profile.axis[0].travel_high_mm));
  if (g_profile.feedback != FeedbackKind::None)
    Serial.printf("encoder: %s\n", "probed - see enc= in status line");
  else
    Serial.println("encoder: profile declares feedback=none");

  // STUB: TMC5160 SPI configuration (MRES microstepping, coil current,
  // stallGuard, stealthChop, VSENSE) is NOT implemented. The profile's
  // steps_per_motor_rev=3200 assumes 16 microsteps; many 5160 boards POR to
  // 256u, so until this is fixed *in firmware or by PCB straps*, the desk
  // moves at a 16x wrong scale. That is loud on purpose, and it is why
  // bring-up step 2 says "watch STEP count with a scope before trusting mm".
  Serial.println("WARNING: TMC5160 registers NOT configured (SPI stub) - "
                 "verify driver microstep setting matches profile "
                 "steps_per_motor_rev before loading the mechanism!");

  // No open-loop workaround here. This used to drop g_cal when
  // feedback==None && homed, because an open-loop machine cannot disprove a
  // stale stored datum (the top can be pushed by hand while powered off).
  // Controller::init() now applies exactly that policy itself - it clears
  // homed/last_height_valid and logs "stored height dropped - home required" -
  // so the decision lives in one place. A frontend re-implementing core safety
  // policy is the failure mode docs/firmware-design-notes.md concern #2 was
  // raised about; the fix landed in core, so the copy is deleted, not kept
  // "belt-and-braces".

  Controller::Config cfg;
  cfg.machine = &g_profile;
  cfg.cal = &g_cal;
  g_ctrl.init(&g_hal, cfg);
  if (!g_ctrl.load_presets()) Serial.println("no stored presets yet");

  g_cmd_q = xQueueCreate(8, sizeof(Cmd));
  if (!g_cmd_q) fatal_loop("command queue alloc");

  xTaskCreatePinnedToCore(control_task, "ctrl", 8192, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(status_task, "stat", 8192, nullptr, 1, nullptr, 0);
}

void loop() {
  // All work runs in pinned tasks; loopTask must not compete with the
  // control task for core 1.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
