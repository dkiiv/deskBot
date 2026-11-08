# Firmware design notes and core concerns

Written against `core/` as found at the time (unmodified, 103 checks passing
then; 141 now). Ordered by how much I'd want a human to look at them.

**Status note:** concerns 1 and 2 below were later fixed in `core/` and are kept
here because the reasoning is the useful part. Concern 4 is not a bug, just a
documented margin. Do not read this file as an open issue list - AGENTS.md
"Current state" is where open work lives.

## Concerns about core/ (not edited, per instructions)

### 1. `position_lost` is a dead end: the core's own prescribed recovery is
### refused by the supervisor (highest priority)

`safety.h` documents `position_lost` as "refuse all motion until re-homed"
(and README.md repeats it). But `Faults::any_hard()` includes
`position_lost`, `Supervisor::can_move()` is `!any_hard()`, and
`Controller::home()` returns `UnsafeToMove` when `!can_move()`.
`acknowledge()` deliberately clears only the stall latch and
`home_timeout`. So once the integrity check in `tick()` sets
`position_lost` (it writes `sup_.faults().position_lost = true` directly,
bypassing the supervisor's return paths), **no Controller call can ever
clear it** — the machine is bricked against the core's own documented
recovery. `skew` on a dual-motor desk is the same trap.

This is reachable in normal use: encoder fitted, someone leans on the parked
top (the exact scenario the check is *for*), desk refuses to move *and*
refuses to re-home. A firmware workaround exists (see `apply_cmd`'s
`kCmdHome`: the firmware calls `supervisor().clear_faults()` only when
starting a home and only for position_lost/skew) but a workaround living in
one of N future frontends is how it goes wrong. Suggested core fix (one
line): allow `home()` through `can_move()` when the only latches are
position_lost/skew — homing is exactly re-establishing the datum those
faults say is lost — or add `Controller::require_rehome()` that clears just
those two. I did not make the change because the HARD RULE said not to; this
needs an owner decision and a test.

### 2. `Controller::init()` trusts a stored datum it just said is unsafe on
### an open-loop machine

`controller.h` resumes `cmd_mm_[0] = cal_->home_mm` (state Ready) whenever
`cal_->homed`, but the file's own header comment argues position is
untrustworthy after a power cycle, and with `FeedbackKind::None` nothing can
ever disprove the stored value (self-locking worm: the top can be moved by
hand with the board unpowered; the machine wakes believing a height nobody
can verify and will happily drive to a soft limit computed from it).
The firmware works around it by dropping `Calibration{}` at boot when
feedback==None && homed (see `setup()`), forcing a home every power cycle —
which contradicts the persistence the core went out of its way to build, so
this is worth deciding in one place. Options: gate the resume on
`have_encoder` at first tick; or accept it as policy and delete the
firmware's belt-and-braces. My vote: the core should not start Ready when it
has no way to notice a lie.

### 3. `Status` names missing from `name()` are fine, but two `Outputs`
### fields have no transport

The HAL has no "here are your Outputs" method; the core calls
`drive_rate`/`set_motor_power` directly and fills a private `Outputs`-shaped
diagnostic state nobody can read. `led_pattern` is computed nowhere I could
find outside `Outputs` docs — the firmware derives LED/buzzer from
controller state instead. If Outputs is meant to be the contract (it reads
that way: `u8 led_pattern` with documented values), a `set_outputs(const
Outputs&)` on the HAL (or making the core's snapshot readable) would let
firmware and future frontends stop re-deriving policy the core already has.
Not a bug; a seam that stopped short.

### 4. Minor: `home_max_us` derived budget ignores that the desk may start
### far below `travel_low`

`run_homing` sizes the watchdog from `(travel_high - travel_low) /
home_rate`, but a desk booted at an unknown height may have to travel *down*
from above `travel_high` (mechanical stops sit outside soft limits — the
profile's own comment says so). The margin (1.6× + 10 s) probably covers it;
a bench with a seized worm near the top will hit `home_timeout` (correct
behaviour, just for a slightly wrong reason). Flagging because the number's
derivation reads like it assumes the worst case is full travel *down from
travel_high*, which is what it computes. No action requested.

### 5. Trivia

* `controller.h:615 log_burst_` — reserved and unused; fine, but it is the
  kind of member that accretes a lie. Delete or use.
* `Inputs::encoder_counts[kMaxAxis]` is filled by the firmware per
  `profile.axis_count`; core reads [1] for skew even when axis_count==1
  (guarded). OK.

## Firmware decisions worth knowing

* **Profile embedding.** JSON text → generated header (`scripts/
  gen_machine_header.py`, committed output) → parsed at boot by a
  no-allocation parser. One source of truth with the simulator; the parser
  is host-tested (91k checks) including the nested-lookup bug class it
  replaces (children linked in-node, not in a shared edge pool — the first
  draft had that bug and the test caught it; see the comment in
  `json_embed.h`). Alternative rejected: hard-coded struct (drifts from the
  JSON; sibling revision did exactly this), or LittleFS on flash (a
  filesystem in the boot path of a safety device for ~2 KB of constants).
* **Why not `strdup`/`std::string` in the loader:** the host loader's
  `strdup` leaks by design on repeated loads; the embedded version points
  into the static `Dom`'s interned pool — lifetime is program lifetime,
  zero allocation.
* **Encoder unwrap** lives in `include/deskbot/encoder_as5600.h` (pure,
  host-tested) rather than inline in main.cpp, because it is the single
  highest-risk transform in the firmware (full write-up in that header).
  Sticky-invalid: a lost revolution datum cannot be re-trusted by a good
  sample, only by ack + re-home.
* **Failure policy everywhere:** a failed read disables its feature for that
  tick (never repeats the last value); a failed *actuation* (LEDC reject,
  rate clamp) is counted and shown, never silent. Temperature sensor below
  60 mV ⇒ reported hot (refuse motion), not cold. Enable chain and limits
  are pull-up/NC so open = stop.
* **Watchdog:** task watchdog 4 s panic (the Arduino sdkconfig already ships
  5 s/panic; reconfiguring explicitly avoids depending on whether loopTask
  feeds the legacy idle dog). The *real* fail-safe is hardware: EN line
  board-pulled to disabled, NC e-stop, self-locking worm.
* **I2C bus lock:** AS5600 reads hold a mutex across the pointer-write +
  read pair (Wire's internal lock covers one transaction; an interleaved
  pair would corrupt the unwrap datum). Status task never touches the bus —
  it reads a seqlock-published snapshot instead.
* **`ledcSetup(…, 1 Hz, …)` fails**: the divider is 18+4 bits
  (ledc_reg.h LSTIMERx [21:4]), so the frequency floor at 4-bit resolution is
  40 MHz / (262143 × 16) ≈ 9.5 Hz and the IDF driver returns ESP_FAIL (the
  wrapper, 0) for anything below it. The first revision initialised at 1 Hz
  and would have bricked boot in `fatal_loop`. Init is now 1 kHz (duty 0
  keeps the pin low); init prints the negotiated frequency.
* **NVS** uses raw `nvs_*` with strict length matching; a struct-size change
  reads as "no data" (refuse to half-fill a `Calibration`). `profile_version`
  + magic checked at boot; presets survive a discarded calibration.
* **Stub inventory** lives in README.md ("What is real and what is not"),
  the largest item being TMC5160 SPI configuration — until then the PCB must
  strap the driver to match the profile's `steps_per_motor_rev`, and the
  boot banner says so.

## Open questions for the owner / hardware

1. TMC5160: strap microstepping for 16 µ (matching the profile), or accept
   firmware SPI config as the next firmware task? (I'd do SPI config next:
   it also buys stallGuard as a second opinion on the core's stall logic.)
2. `direction`: which way does the printed gearbox wind the crank for DIR
   HIGH? Decide at bring-up step 2, encode in JSON, never in wiring.
3. Second AS5600 at 0x37 — real boards must strap the address; single-shaft
   machines use only #0. Fine to leave #1 unpopulated: probing it is skipped
   when axis_count==1.
4. Brake: is one actually fitted? `set_brake()` is wired; the core's
   `Outputs::brake` has no transport (concern #3), so today the brake would
   be engaged at rest by firmware policy. Right now it idles LOW (released).
5. OLED on the I2C bus (address collision check needed if fitted; 0x3C is
   the usual, clear of 0x36/0x37).
6. Serial command `g` is unthrottled; a remote client can spam `g` and each
   accepted move writes NVS once (at home time only — fine). Presets have no
   save key yet (`save_preset` exists in core, no command bound).
