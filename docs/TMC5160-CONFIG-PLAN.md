# TMC5160 SPI configuration — plan

The firmware stub (`firmware/src/main.cpp`, `// STUB: TMC5160 SPI
configuration`) means the driver currently runs at power-on defaults:
microstepping per straps/POR, uncontrolled coil current, chopper per POR. The
boot banner warns; this doc is how we make it real.

Datasheet: **TMC5160A rev 1.18+** (Analog Devices / Trinamic).

> **Deliberate policy: this plan names registers and fields but carries no hex
> addresses.** The first draft of this file quoted addresses from memory and at
> least two were wrong. Whoever implements this transcribes the address table
> from the datasheet register map (and the field layouts, same caution) into a
> single header — `firmware/include/deskbot/tmc5160_regs.h` — and the code
> thereafter uses named constants only. A readback check (see §6) then catches
> any remaining transcription error at boot instead of in the field.

## 0. The part-selection catch (read before PCB layout)

**TMC5160 = controller + MOSFET *driver*. The power MOSFETs are external.**
Datasheet: "Using external transistors… 1 to several 10A coil current." This
changes the board effort and was understated in the first draft of
`pcb-agent.md` (now fixed there):

| option | what it means |
|---|---|
| **A. Discrete 5160** (current plan) | 4× N-channel power MOSFETs + gate drive + sense resistors per the datasheet application circuit. Real layout work; correct for a ~2 A NEMA 23 at 48 V. |
| B. 5160 module carrier | Design around an off-the-shelf 5160 module (e.g. BTT TMC5160). Faster bring-up, one more mechanical part, less control over sense-R and thermal design. |
| C. TMC5130 (FETs integrated, 46 V, ~2 A RMS) | Simplest silicon, but 46 V absolute-ish ceiling fights the 48 V decision and a 2 A/phase N23 exceeds it. Only viable if MOTOR-DECISION lands on ≤1.4 A motor **and** the rail drops to 36 V. |

Decision owner: Curtis, with the PCB agent. Everything below assumes A.

## 1. SPI basics (datasheet "Serial Data Transfer" section)

- Mode 3 (CPOL=1, CPHA=1); ≤ ~24 MHz spec, run the ESP32-S3 at 1–5 MHz.
- 40-bit datagrams: 8-bit header + 32-bit data. **Reads have one datagram of
  latency**: send a write (or dummy access) whose reply carries the *previous*
  read's data. Plan the driver as: transaction returns next read, never current.
- CS per driver; data latches on rising CS. Every config write is sent
  **twice** (datasheet note: first write after wake can be lost).
- One `readback_check()` after bring-up: re-read every written register,
  compare, log mismatches loudly. Silent SPI wiring problems are the #1
  reason "the driver ignores my settings."

## 2. Bring-up configuration sequence

Order matters where noted; addresses come from the transcribed header.

1. **CHOPCONF** — the workhorse register:
   - `TOFF ≥ 1` — **TOFF=0 disables the driver**; start TOFF=3.
   - `TBL` = 2 (blank time default).
   - `MRES` — logarithmic, 0 = 256 µstep. For **16 µsteps set MRES = 4**
     (256 >> 4); cross-check against the datasheet's MRES table and the
     readback, because *this field is what makes the profile's
     `steps_per_motor_rev=3200` true*. Set it from the JSON value, never a
     literal.
   - `HSTRT`/`HEND`/`HDEC` — datasheet defaults.
   - `TPFD` — default.
2. **IHOLD_IRUN** — the safety-critical register; see §3. Also set
   `IHOLDDELAY` so hold current doesn't snap down mid-ramp.
3. **GLOBAL_SCALER** — 255 (100%). Set currents down from here, never rely on
   >100%.
4. **COOLCONF** — CoolStep **off** (`SEMIN=0`). Load-adaptive current on a
   worm drive surprises you (draws down when lifting = stalls mid-lift).
5. **DRV_CONF** — enable temperature prewarning if the part has it (read the
   OT.S bit later as an extra thermal signal for `Inputs::driver_temp_c`
   alongside the NTC).
6. **RAMPMODE = soft stop** and leave the ramp engine idle — we drive by
   STEP/DIR (§5).
7. StallGuard config (§4) — **log-only initially**.

After all writes: `readback_check()`, then the boot banner prints the truth:

```
TMC5160: MRES=4(16us) IRUN=1400mA IHOLD=500mA RSENSE=0.050 GLOBALSC=255 readback=OK
```

## 3. Current — where motors and egos die

`I_peak = I_rms × √2` of the motor nameplate. The CS (current scale) field maps
to amperes as a function of **the board's actual sense resistor** and
VREF/global scaler — compute mA→CS from constants that live with the board
definition (one `board_constants.h` generated alongside the profile), not
forum numbers.

Procedure:
1. First-ever run: IRUN ≈ 30 % of nameplate. Confirm direction/steps by
   counting, feel motor temp after a minute.
2. Step to 60 %, repeat. Only then nameplate.
3. `IHOLD` ≈ 30–40 % of run (matches the core's reduced hold-current intent;
   the core's `hold_current_` concept and IHOLD should tell the same story —
   when implementing, have the HAL map `set_motor_power(false)` to full
   disable and let IHOLD handle the "parked" case, per DESIGN-NOTES discussion).
4. Log computed mA at every boot. If someone later changes Rsense on the PCB
   and forgets the constant, the banner lies on purpose and the AC-clamp test
   (§6.4) catches it.

## 4. StallGuard — second opinion, never primary

The core's stall detection is encoder-vs-model: positional truth. StallGuard2
is torque-based and sees a binding worm *earlier* than position lag. Integration
plan (deliberately gentle):

1. Read `SG_RESULT` into the 500 ms status line. Tune `SGT` (threshold) on the
   bench until clean air-moves read high and a hand-braked shaft reads low.
2. Log only, until a few hundred clean moves correlate.
3. Only then: propose the core change `Inputs::stall_guard_load` (u16) so
   `Supervisor` can trip `Faults::blocked` on a sustained zero-load-on-descent
   or saturated-load reading. **Do not overload encoder fields to fake it** —
   that corrupts the primary detector. (Open core change request; tracked
   here until added.)

## 5. Deliberately unused

- **On-chip ramp generator / XTARGET / VMAX**: two planners in one loop is a
  bug factory. The core trapezoid is the tested, host-verified source of
  truth; the 5160 stays a dumb STEP/DIR driver with good current control.
- **MicroPlyer interpolation**: 16 µsteps already exceeds what the mechanism
  resolves; 256-interpolation adds bookkeeping for no physical gain.
- **DcStep**: auto-slowdown under load would silently change velocity and
  lie to the core's rate assumptions. If we ever want obstacle-aware descent,
  it goes through StallGuard (§4) into the supervisor, where it is visible.
- **Encoder inputs on the 5160**: encoder lives on the ESP32 side (AS5600).
  If that later moves to the 5160's ABN inputs (a legitimate board revision),
  revisit ENC_CONST + this section as a unit.

## 6. Acceptance (hardware bench, after PCB option A/B lands)

- [ ] `tmc5160_regs.h` transcribed from datasheet rev ≥1.18, PR'd separately,
      reviewed line-by-line against the PDF.
- [ ] Boot: all writes read back equal; banner shows MRES/IRUN/IHOLD from
      JSON + board constants.
- [ ] **Open-loop step audit** (catches MRES mismatch without an encoder):
      command exactly 3200 core-steps (one motor rev per profile), crank must
      turn exactly one gear-ratio revolution: `mark shaft with tape, verify
      shaft angle = 360°/ratio ± 2°`. Wrong MRES shows up here as a factor-of-16
      error, not a slow drift.
- [ ] Encoder cross-check: 1 m of commanded travel → AS5600 counts within
      0.5 % of `mm_to_steps` prediction.
- [ ] Current honesty: variac-free check — measure DC supply current during a
      lift, compare to expected from IRUN × duty sanity; and motor case ≤ 45 °C
      after 10 full-travel cycles at nameplate IRUN, 25 °C ambient.
- [ ] Removal of the boot WARNING line (stub banner) and README stub table
      updated; `make test` + firmware host tests green.
- [ ] One weekend of log-only StallGuard data reviewed before §4 step 3.
