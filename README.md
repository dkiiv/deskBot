# deskbot

Turn a mechanical height-adjustable desk into a motorised one, with software that
does not care which desk it is bolted to.

The motion and safety logic is a platform-independent C++17 library, compiled on
your laptop and driven against a fake bench before anything is printed, soldered
or plugged in. A specific desk is one JSON file plus some printed plastic.

[![CI](https://github.com/dkiiv/deskBot/actions/workflows/ci.yml/badge.svg)](https://github.com/dkiiv/deskBot/actions/workflows/ci.yml)

**Agents and humans new here: read [AGENTS.md](AGENTS.md) first** — lanes,
rules, what's left. The map below is just the directories.

```
core/          motion planner, stall detection, homing, presets  <- generic
machines/      one .machine.json per desk                         <- the seam
firmware/      ESP32-S3 firmware (PlatformIO) — compiles, unflashed
tests/         host suites: core/ firmware/ sim/ (+ include/ for sim helpers)
scripts/       calibrate.py (measure your desk), sizing.py (motor math)
husky-bench/   (planned) prints + build guide for the specific bench
docs/          decisions, handoff specs, config plans
.github/       CI: make test + gen-header sync + sim run + pio build
```

## Status

| Area | State |
|---|---|
| Core motion + safety library | **done, 141 host checks passing** |
| Simulator + machine-profile loader | **done** (`tests/sim`, `make run`) |
| CI (host tests, header sync, firmware build) | **done** — `.github/workflows/ci.yml` |
| Husky machine profile | numbers present, **three must be measured** |
| Motor/gearbox sizing model | `scripts/sizing.py`, validated against published desk speeds |
| ESP32 firmware | compiling scaffold, no hardware yet — `firmware/` |
| Motor & supply decision | recommended, awaiting 2 measurements — `docs/MOTOR-DECISION.md` |
| TMC5160 SPI config (biggest firmware gap) | planned — `docs/TMC5160-CONFIG-PLAN.md` |
| Printed gearbox / mount | not started — see `docs/cad-agent.md` |
| Control PCB | not started — see `docs/pcb-agent.md` (note: 5160 needs external MOSFETs) |

## Why this shape

Retrofitting a crank desk fails in a specific, predictable way: the mechanism is
older and sloppier than the software expects, and the software has no way to
notice. A stepper that loses steps sounds identical to one that is not. So the
core is built around noticing.

*Commanded position is integrated from the commanded rate, and the encoder
disagrees with it on purpose.* Every fault in the system is some flavour of
"model and reality diverged":

| Fault | What it means | What the desk does |
|---|---|---|
| `stall` | mechanism lagged the command while driving | stop, back off, re-anchor, latch |
| `position_lost` | parked, but reality moved | refuse all motion until re-homed |
| `home_timeout` | homing ran too long | cut power, latch (blocked path) |
| `over_temp` | driver heatsink hot | stop, resume when cool |
| `skew` | legs diverged (dual-motor only) | unequalise, then stop |
| duty budget | gearbox run too long | forced rest |

Two of those are the ones that make or break a retrofit:

**`position_lost`.** A self-locking worm means the desk never runs away when the
gears slip — it just quietly stops being where you think it is. That is exactly
when soft limits become fiction. So if the encoder and the model disagree while
nothing is driving (someone leaned on the top, a gear skipped), the machine
refuses to move at all until it is re-homed. On a bench that can hold a monitor
arm and a 40 lb tool chest, "I don't know where I am, so I won't move" is the
correct behaviour.

**`home_timeout`.** Before the first home there is *no* trusted position, so the
lag detector that protects every other move cannot exist yet. A home sequence
whose path is blocked would otherwise drive forever. The watchdog is derived from
the machine geometry, so it adapts to any desk.

## The three numbers you must measure

Everything else is engineering. These three are physics, and no datasheet has
them for your bench:

| Key | Meaning | How to measure |
|---|---|---|
| `mm_per_shaft_rev` | top travel per one turn of the crank | tape measure, count turns |
| `gear_ratio` | motor revolutions per drive-shaft revolution | it is your printed gearbox |
| `travel_low_mm` / `travel_high_mm` | height of the *top surface* at each mechanical hard stop | tape measure |

`steps_per_motor_rev` = full steps × microsteps (3200 for a 1.8° NEMA 17 at 16µ).

The scale factor is deliberately split so the encoder measures the mechanism as
built, including gear slop, rather than the mechanism as modelled.

## Quick start

No hardware needed:

```sh
make test          # 141 behaviour checks against a fake bench
python3 scripts/sizing.py --load-kg 50   # can it actually be fast enough?
make run           # simulate the Husky profile end to end

# drive it by hand
./build/deskbot-sim machines/husky-46x24.machine.json \
    --script 'home,go=950,go=700,status'

# break things on purpose - do this before you break a real desk
./build/deskbot-sim machines/husky-46x24.machine.json --stall-at 800 --script 'home'
./build/deskbot-sim machines/husky-46x24.machine.json --slip-at 2 --slip-rate 0.6 \
    --script 'home,go=950'
```

Requirements: g++ or clang with C++17. Nothing else.

## Adding a desk

Copy `machines/husky-46x24.machine.json`, change the numbers, run
`make test`-equivalent against it. The `coupling` field is the one that actually
matters:

- `"single"` — one shaft through both legs (crank benches, torque tubes). The legs
  *cannot* tilt apart, so skew is not a correction target, it is a
  broken-mechanism alarm. Software that fights this fights itself.
- `"master_follower"` — two motors, software keeps them level.
- `"independent"` — two motors, no levelness requirement.

A second machine is validated in CI by the dual-leg test in
`tests/core/test_core.cpp`, which uses a sticky leg to prove the equaliser is
doing real work rather than getting lucky.

## Layout

```
core/include/desk/
  types.h             units, Status, limits
  machine_profile.h   the machine descriptor - this is the generic seam
  hal.h               hardware interface (7 methods)
  trapezoid.h         online velocity planner, asymmetric up/down
  safety.h            calibration model, supervisor, fault latches
  controller.h        the machine: one tick() call
  fake_bench.h        simulated desk that lies to you on purpose
cli/
  src/sim_main.cpp    simulator CLI
  include/deskbot/    JSON parser + profile loader (no dependencies)
firmware/
  README.md           HAL implementation, pin map, build + bring-up
husky-bench/          prints + build guide for one specific bench
docs/                 agent handoff specs + decisions record
```

## Safety

This moves a heavy object in three dimensions, near people, using printed plastic
and a motor you sized from a datasheet.

- A **physical e-stop or enable switch in series with the motor supply** is not
  optional, and it is not software. `enable_closed` exists so the software can
  know; it must not be the thing that decides.
- Mechanical hard stops must stay. Soft limits are convenience; the worm's own
  end-stops are what stop the desk when software is wrong.
- Printed gears are the weak link **by design** — slipping before stripping is
  preferable to a jam. But that also means the gearbox is a wear item and the
  duty budget is not paranoia.
- Do not put your hands under a motorised top. Descending is the dangerous
  direction, which is why descent gets gentler deceleration and obstacle back-off.
- The Husky's crank should stay usable. Keep the hand crank or a removable
  coupling so a dead microcontroller or a dead 24 V supply does not mean
  disassembling the bench to lower the top in an emergency.

## License

MIT — see `LICENSE`.
