# Decisions

Design choices that were genuinely contested, what was chosen, and what it cost.
Written so a future contributor can disagree with the reasoning rather than
guessing at it. Numbers quoted here were produced by `make test` and
`scripts/sizing.py` in this repo, not remembered.

---

## Software lives in a header-only C++17 core, not in the firmware

**Chosen.** `core/` compiles with `g++ -std=c++17 -Wall -Wextra` and has no
Arduino, ESP-IDF, or FreeRTOS includes anywhere in it.

**Why.** The interesting failure modes of this project — losing steps, a blocked
home, a hand-pushed top, a dropped Wi-Fi client mid-descent — cannot be
reproduced on demand on hardware, and must not be discovered on hardware at all.
A host-compiled core means every one of them is a 30-second deterministic test.
141 checks currently run in under two seconds with no hardware present.

**Cost.** A HAL interface with seven methods instead of direct register pokes,
and a fake desk that is more work to write than a test harness. The fake desk has
already paid for itself twice (below).

---

## `MachineProfile` is data, loaded from JSON

**Chosen.** One struct in `core/include/desk/machine_profile.h`, populated from
`machines/*.machine.json`. `coupling`, `drive`, `feedback` are enums in that data.

**Why.** The owner's constraint was that the software should apply to *any*
mechanical desk, with only the prints being bench-specific. That is only true if
"new desk" means "new JSON", not "new fork".

**Payoff already realised:** the dual-leg desk test in `core/test/test_core.cpp`
exercises `AxisCoupling::MasterFollower` on the identical controller object the
single-shaft tests use. That is the generic claim being checked, not asserted.

**Corollary that mattered:** `AxisCoupling::Single` is not cosmetic. On a crank
bench one shaft drives both legs, so the legs *cannot* tilt apart. Treating skew
as a correction target there makes the software fight a mechanism that is
physically incapable of the error — it would unequalise a straight bench. On that
machine skew is a broken-mechanism alarm and nothing else.

---

## Commanded position integrates the *quantised* rate, with a carried remainder

**Chosen.** The controller computes microsteps/s once, publishes that integer,
and integrates the same integer with a fractional remainder carried between ticks.

**Why this is a real bug and not pedantry.** The first version rounded a per-tick
delta: at 12 mm/s and 16 microstepping, one 2 ms tick is 125.67 microsteps,
rounded to 126. That biases the model ~0.5% ahead of the pulses actually emitted,
which grows with move length and reaches ~1 mm over the full travel — enough to
trip the stall detector on a desk that is working perfectly. The fix was found by
simulation, and the phantom-stall symptom was confusing enough that an earlier
attempt at it *caused* a worse bug: an editing slip during that fix deleted one
line (`cmd_steps_[0] += delta`) and froze the model entirely. Both were caught
only because a fake desk tracked truth independently.

---

## Faults latch until acknowledged; hard stops do not

**Chosen.** Stall, position-lost, skew, over-temp, e-stop and home-timeout latch.
A desk resting against a mechanical end stop is a normal state, reported
separately from faults.

**Why.** A desk that silently looks healthy right after it hit something invites
a second, harder attempt at whatever was under the top. Being parked on a hard
stop, conversely, is where a desk spends most of its life; latching on it would
make the machine unusable.

**Cost, paid twice.** Both halves of this were initially wrong in the opposite
direction. The retreat path auto-cleared the stall latch, so the bench looked
fine immediately after an obstacle stop. And limit contact was modelled as a
fault, which meant a successful home — which ends pressed against a stop —
reported as an error. The distinction is `Faults::any_hard()` vs
`Faults::any()`.

---

## Position is *untrusted* until homed, and pre-home protection is honestly limited

**Chosen.** Before the first anchor, only hard stops, a slow home rate, and a
watchdog protect the mechanism. The lag detector is explicitly disabled.

**Why.** Two reasons, both found by the code failing. First, before homing the
model says 0 mm while the encoder says "somewhere mid-travel"; comparing them
fires a phantom stall on the very first move of every machine, on every boot.
Second, and more important: inventing a protection that does not exist is worse
than admitting the gap. Pre-home, the honest safety story is "slow, short,
watchdogged, and the mechanical stops are still there."

**This exposed the missing protection:** a home sequence whose path is blocked had
*no* timeout at all, so it drove forever — model reached −1800 mm in simulation
while the desk was stuck against an obstruction. That is now `home_timeout`,
derived from machine geometry, with its own test.

---

## Descending is a powered move

**Chosen/assumed.** A self-locking worm (efficiency ≲0.5) cannot be over-driven by
the load, so the motor pushes down as well as up.

**Why.** The alternative — trusting gravity plus the worm's margin — is what
actually makes a desk a guillotine. It also means descent current is not free,
which is part of why the duty budget exists.

---

## Sizing: power is the constraint, and the gear ratio cannot help

**Chosen.** `scripts/sizing.py` computes `P = mgv / (η_worm · η_gear)` and reports
what each motor frame can actually deliver, cross-checked against published
commercial desk speeds.

**Why, and what it corrected.** This is where the project was most wrong and only
the arithmetic caught it. The first plan was NEMA 17 with a 9:1 printed gearbox
at 300 rpm. That yields **3.1 mm/s** — 108 s for full travel. For comparison, a
budget single-motor FlexiSpot is ~25 mm/s and a Linak 3-stage column ~38 mm/s,
so the design was roughly 8x too slow, and would have felt broken on the first
test.

The important part is *why* it could not be fixed by re-gearing: the required
power contains no ratio term. Higher ratio buys torque and spends speed at the
same exchange rate. At 50 kg, 25 mm/s needs ~36 W at the motor after the worm has
eaten most of it, which no NEMA 17 supplies. The levers are motor frame, supply
voltage, and friction — and only one of those is free.

Two further constraints fell out: never park the operating point at 200–400 rpm
(mid-band resonance stalls a stepper with torque to spare on paper — the original
300 rpm guess sat exactly there), and higher supply voltage is the cheap lever on
torque-at-speed, which is a second, independent argument for the 60 V-rated
driver chosen in the PCB spec.

**Honesty note.** The torque-speed curve in that script is fitted to published
NEMA 17 data, not to the motor you will buy. It is a comparison tool for choosing
a frame size and a supply voltage. The datasheet plot of the actual motor is the
last word, and the script says so in its own docstring.

---

## A reboot may only believe what it can still check

**Chosen.** On boot with feedback: resume the stored height as a *hypothesis*,
adopt the encoder on the first trustworthy reading, and refuse motion until that
reconciliation has happened. On boot with `FeedbackKind::None`: drop `homed`
entirely and require a home every power cycle.

**Why.** `init()` originally resumed from `cal_->home_mm` — which is the
*datum* (bottom of travel), not the last height. Any power cut mid-travel woke
the desk claiming to be at its lowest point, so every soft limit was computed
from a fiction. Separately, resuming at all was wrong on an open-loop machine:
a self-locking worm can be pushed by hand with the board unpowered, and nothing
in an open-loop build can ever disprove a stale EEPROM value.

Found by the firmware workstream's design note, not by me — both were my bugs.

**The interesting part: a stale height should HEAL, not FAULT.** The first test
asserted that a desk whose stored height the encoder contradicts must latch
`position_lost`. That was wrong. The encoder is what touches the mechanism; the
stored value is what a previous process remembered. Adopting the encoder leaves
the machine accurately positioned and safe, so faulting it would refuse a desk
that demonstrably knows where it is, purely because a stale number disagreed.
The rule is "refuse to move when you do not know where you are" — after
reconciliation it does know. The log line exists because a large discrepancy
means something was resting on the top, and the operator is about to call a
preset computed from where they *think* the desk is.

**Cost.** An open-loop desk is now more annoying: it homes on every power cycle
(~2 min at the low end). That is the honest price of having no way to know, and
it is why the encoder is worth the $3 in the PCB spec.

---

## `mm_per_shaft_rev` is measured, never inferred from gearing

**Chosen.** `scripts/calibrate.py` takes it from a tape measure and a turn count,
asks for a second reading, and rejects the profile if the two disagree by >10%.

**Why.** The bench's worm pitch, its wear, and the slop in a 10 mm hex coupling
are not in any datasheet. Inferring travel from gear math gives a number that is
wrong in exactly the way nobody notices: the desk moves, reports a height, and
the height is a lie. The 10% rejection exists because a miscounted turn is the
most likely single error in the whole project, and it propagates into every
soft limit and every preset.

**Consequence.** The encoder deliberately reads the *motor*, so it measures the
mechanism as built including slop, and any slip between motor and output shows up
as a stall the software can act on. Putting the encoder on the output shaft
instead is supported (`feedback.kind: shaft_encoder`) but then `gear_ratio` must
become 1.0 in the profile, because the ratio moves from the measurement into the
mechanism.
