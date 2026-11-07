# deskbot firmware

ESP32-S3 implementation of `desk::Hal`. Nothing here thinks about motion: the
planner, homing, stall detection and presets all live in `../core`, which is
host-compiled and unit-tested (141 checks). This directory is a hardware
window, a control loop, and a small JSON reader.

## Build

```sh
cd firmware
pio run                 # -> .pio/build/esp32s3/firmware.bin   (verified: 0 errors)
pio run -t upload       # flash over USB (hold BOOT + tap RESET on a virgin board)
pio device monitor      # 115200, USB-CDC on the native USB connector
```

Requires PlatformIO Core 6.x and `espressif32` 6.10.0 (auto-installed; the
toolchain download needs registry access on the first build).

There are two environments: `esp32s3` (the firmware) and `esp32s3_smoke`
(minimal Arduino loop, proves the toolchain when the firmware is red for
unrelated reasons).

### Machine profile: one JSON, generated header

The machine definition is `../machines/husky-52x24.machine.json` — the same
file the simulator loads. It is embedded as **verbatim JSON text** in a
generated header and parsed at boot:

```sh
python3 scripts/gen_machine_header.py ../machines/husky-52x24.machine.json
# -> include/generated/machine_profile_json.h   (committed; keep it in sync)
```

The generated header is committed deliberately: `pio run` then needs no
python step, and a stale copy cannot pass silently — the host test below
recompiles the embedded text and checks every number against the declared
profile, and the boot log prints name/version so a mismatch is visible.
The parser (`include/deskbot/json_embed.h`, `profile_embed.h`) is a
no-std, no-allocation, no-exception twin of `cli/include/deskbot/` with the
same keys, defaults and validation.

### Host tests (firmware-only logic, no hardware)

```sh
g++ -std=c++17 -Wall -Wextra -fsanitize=undefined,address \
    -Iinclude -I../core/include test/firmware_host_tests.cpp -o /tmp/fwtest && /tmp/fwtest
```

91 334 checks: JSON parse/load integrity (including the nested-lookup class
of bug), the embedded Husky profile, and the AS5600 revolution unwrap across
wrap points, glitch rejection and datum-loss latching. The unwrap and the
parser are the two places a silent bug costs the most, so they are tested on
the host where bugs are cheap.

## Pin map

Defined once, at the top of `src/main.cpp`, each line commented with the HAL
method it serves. This table and that block must stay identical — and both
must match `../hardware/` when the PCB exists. ESP32-S3-DevKitC-1 numbering;
avoided pins: 0/3/45/46 (strapping), 19/20 (native USB console), 43/44
(UART0/flashing), 26–32 (flash). TEMP is on **ADC1** on purpose (ADC2 is
unusable while Wi-Fi runs).

| Function | GPIO | HAL method | Notes |
|---|---|---|---|
| STEP0 | 12 | `drive_rate(0)` | LEDC ch0, 50% duty |
| DIR0 | 13 | `drive_rate(0)` | HIGH = profile's positive direction |
| STEP1 | 16 | `drive_rate(1)` | LEDC ch2 (own timer) |
| DIR1 | 17 | `drive_rate(1)` | |
| ENABLE | 14 | `set_motor_power()` | TMC5160 EN via board-level polarity (see `ENABLE_ACTIVE_LOW`) |
| BRAKE | 21 | `set_brake()` | HIGH = engaged; de-energised = spring brake |
| LIM_LOW0 | 4 | `read_inputs()` | reed to GND, pull-up; LOW = at low stop; wire **NC** if you can |
| LIM_HIGH0 | 5 | `read_inputs()` | same, high stop |
| LIM_LOW1 / LIM_HIGH1 | 6 / 7 | `read_inputs()` | used only if the profile declares 2 axes |
| BTN_UP | 10 | `read_inputs()` | to GND, debounced 30 ms |
| BTN_DOWN | 11 | `read_inputs()` | |
| BTN_STOP | 18 | `read_inputs()` | **tap only** — the e-stop itself is an NC switch in motor power |
| EN_CHAIN | 15 | `read_inputs()` | enable-chain sense, closed→LOW, **not debounced** (see below) |
| TEMP | 1 | `read_inputs()` | TMP36 on ADC1_CH0, 11 dB; <60 mV = sensor missing = reported hot |
| I2C SDA / SCL | 8 / 9 | `read_inputs()` | AS5600 (+future OLED), external 4k7 pull-ups |
| LED | 48 | `set_led()` | onboard WS2812, driven as plain GPIO (STUB) |
| BUZZER | 2 | `set_buzzer()` | HIGH = on |

Why the enable chain is not debounced: the core's supervisor evaluates it
before everything else and cuts power within the tick. A debounce window
there would be a window in which a tripped chain is ignored. An NC chain
fails safe against contact bounce (a bounce reads "open" = stop).

The two safety-relevant facts about the buttons/limits, because neither is
visible in the firmware:

* `BTN_STOP`, `EN_CHAIN` and both limits use internal pull-ups, so an **open
  wire reads as "stop"**. That only holds if the physical switch is
  normally-closed. An NO e-stop wired to this pin silently becomes a button
  that does nothing.
* The e-stop function that works with the MCU dead lives on the PCB, not
  here. If you are tempted to make a GPIO the only thing that cuts motor
  power, read `../docs/pcb-agent.md` first: watchdog reset, brown-out, and
  "the software checked" all end with the desk falling.

## Sign conventions (the two that bite)

1. **Rate sign.** The core signs every commanded rate through
   `AxisProfile::direction` before calling `drive_rate()`. So
   `drive_rate(+x)` means "the direction the profile calls rising", and
   `DIR_LEVEL_POSITIVE` maps that to a level. If the desk rises when DIR is
   LOW, set `direction: -1` in the machine JSON (preferred) or
   `DIR_LEVEL_POSITIVE = LOW` (here) — never both. The core also *integrates
   its position model* from this same signed rate, so a disagreement does not
   just invert a button; it guarantees a phantom stall on the first move.
2. **Encoder sign.** The AS5600 angle increases CCW. Set `kEncoderInvert` at
   bring-up: raise the top by hand, `cnt=` in the status line must increase.

## Serial console

Commands (single letter + Enter): `h` home · `g<mm>` go to height ·
`s` stop · `a` acknowledge a latched fault · `d` reset duty budget ·
`e` clear a latched encoder fault (after a re-home) · `?` status now.

Status line, every 500 ms:

```
h= 949.92 tgt= 950.00 v= +0.00 st=ready         enc=ok  cnt=  845122 T= 31.2 f=- i2c=0/0 clamp=0 rej=0
```

`enc=OFF` means no usable feedback this tick (never fitted, I2C failed, or a
lost revolution datum latched) and the core is running open loop — the desk
still homes, still obeys limits, but stall detection is off by construction.
A never-homed desk shows `st=uncalibrated` and refuses to move; that is the
expected state after power-on.

Buttons: UP/DOWN jog while held (core's own logic). **UP+DOWN held 2 s
starts homing** (the only gesture three buttons allow without inventing a
move nobody asked for). STOP doubles as acknowledge for a latched fault.

## What is real and what is not

Read this before trusting anything. Every `STUB:` in the source is listed
here too. Nothing stubbed returns "Ok" silently — failures disable the
feature they guard and show up in the status line or boot log.

| Area | Status |
|---|---|
| Motion planning, homing, stall/limit/thermal supervision, presets | **real** — `../core`, 141 host tests |
| 2 ms control task (core 1, priority 4, task watchdog 4 s panic) | real |
| STEP/DIR via LEDC | real — band [10 Hz, 120 kHz], sub-floor stops, over-ceiling clamps, both counted |
| Limit switches / buttons, debounced 20/30 ms | real, non-blocking |
| Enable-chain sense | real, deliberately un-debounced |
| AS5600 I2C read + shortest-arc unwrap | real code, **untested on hardware**; host-tested logic |
| Encoder failure → `encoder_valid=false` (never a repeated count), datum-loss latch | real |
| Temperature (TMP36: 500 mV @ 0 °C, 10 mV/°C; <60 mV ⇒ reports 999 °C) | real math, **untested calibration** |
| NVS storage (raw `nvs_*`, strict length check, magic + profile_version) | real |
| Machine JSON embedded + parsed at boot | real, host-tested |
| HAL logging (ring buffer, status task drains) | real |
| **TMC5160 SPI configuration** | **MISSING (STUB)** — see below |
| WS2812 colour patterns | **STUB** — GPIO blink on the data line |
| Dual-leg (`axis_count > 1`) | wired (STEP1/DIR1/encoder #2) but **untested**; core skew logic unexercised |
| Second limit pair / second encoder | wired, untested |
| Wi-Fi / MQTT / Home Assistant | **not implemented.** Buttons and serial only |
| Outputs::led_pattern / cmd_mm / enc_mm | core-internal bookkeeping; firmware derives LED from controller state instead (the HAL has no output push channel — see DESIGN-NOTES) |

### The driver is not configured — the biggest gap before hardware

`main.cpp` emits STEP/DIR but never talks to the TMC5160 over SPI. The
driver's power-on defaults therefore apply: microstep resolution (MRES) of
many boards is 256 µ, while the profile's `steps_per_motor_rev = 3200`
assumes 16 µ. Until this is fixed **in firmware or by PCB straps**, the desk
moves at a 16× wrong scale: 1 mm reported ≠ 1 mm moved, and the soft limits
are fiction at the wrong end. The boot banner says so loudly. The AS5600
does *not* save you here: it reads the motor shaft, and it cannot tell 16 µ
from 256 µ. Current limit and ChopStim/StealthChop are also at defaults.

This is listed as an STUB rather than a silent default because it is the one
gap that changes the *mechanism's* relationship to the numbers.

## Bring-up order

Skipping steps here is how a bench gets broken, and the top is heavy enough
to hurt you. Every step: `pio device monitor` open, status line in view.

1. **Power without motor, without load.** 36–48 V in (per `docs/pcb-agent.md`;
   a 24 V brick also works for this test). Check 5 V and 3.3 V rails. Check
   VS with a scope during an enabled-but-motorless jog: no spikes past 80 %
   of the driver's absolute max. Confirm the E-stop cuts VS **with the MCU
   unpowered** — that test belongs to the PCB, do it now, not later.
2. **Motor connected, mechanism disconnected** (gearbox off the shaft, or
   shaft loose). Home it (`h` — it will hunt; that is the point). Verify:
   DIR flips between up/down jog, STEP toggles at the expected rate (scope:
   ~41 kHz at full speed — or count with a counter IC), encoder counts
   **increase** when you turn the shaft toward rising (set `kEncoderInvert`
   if not), `enc=ok` in the status line. `i2c=`, `clamp=`, `rej=` must stay
   0 during all of this.
3. **Gearbox engaged, top unloaded** (bench top removed or supported by
   blocks). Full home, then `g` moves of 20–50 mm. Reported height must
   match a tape measure within ~1 mm. If it moves 16× too far, see "The
   driver is not configured" above.
4. **Loaded, nothing under the top.** Full-travel moves. Cycle it repeatedly
   and confirm the duty budget engages (`cooling ...s remaining` in the
   status line) — that is the printed gearbox's thermal protection, much
   cheaper to test now than to discover as a melted worm.
5. **Deliberate obstruction, once.** A block of wood under the descending
   top. Expect: stop, `f=stall`, retreat up ~4 mm, `st=protect`; press STOP
   (acknowledge), then re-home. You want to have seen this work once before
   something softer is in that position.
6. **Sensor-fault drills.** Unplug the TMP36 → `f=tempsens|otemp`, motion
   refused. Pull the encoder → `enc=OFF`. Hold the e-stop → `f=estop` and
   VS dead. Do these on the bench, not during week one of use.

## Known limits, stated

**LEDC pulse quantisation at low rates.** At 4-bit resolution (a deliberate
choice — see the derivation next to `kPwmResBits` in `main.cpp`) the emitted
frequency is `f_src / (divider × 2^res)` with an 8-bit fractional divider;
the worst rate error across our band is well under 1 % and two to three
orders of magnitude below the core's stall margin. At the very bottom
(single-digit steps/s) it is worst-in-relative-terms, which is why rates
below `kMinStepHz` stop pulsing rather than clamp up. The model integrates
what the core *commanded*, so any of this that ever lies is caught by the
encoder — that is what it is there for.

**Rate changes are re-tuned every tick they change.** LEDC frequency changes
while running can emit one stretched edge; we park duty before re-tuning so
the worst case is a missed step (model ahead → encoder catches it) rather
than an extra one (model behind → desk further than you think). A future
move to the TMC5160's ramp generator or RMT removes the issue entirely.

**Button/limit debounce uses `millis()`,** so inputs are read at most once
per millisecond; at a 2 ms tick that costs nothing and avoids a blocking
debounce in the control loop.

**The console is USB-CDC.** Writes are bounded (`setTxTimeoutMs(20)`) and
never happen on the control task — logging goes through a ring the status
task drains. Plugging/unplugging USB while running cannot stall the 2 ms
loop, but expect the status stream to stop and resume.
