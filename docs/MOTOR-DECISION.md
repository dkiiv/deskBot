# Motor & supply decision

Driven by `scripts/sizing.py`; the arithmetic, not vibes. Status: **recommendation
below — final call after the two measurements in the checklist.**

## Load inventory (owner's actual bench, Husky 46×24 adjustable-height)

| Item | Mass | Source |
|---|---|---|
| Odyssey OLED G9 49" **with stand** | 12.6–12.9 kg | Samsung spec sheets (model-year variance) |
| Mid-tower ATX gaming PC | 12–18 kg | typical for GPU-equipped mid-tower; **weigh it** |
| Desk's own wooden top (lifted too) | ~15–18 kg | 1.2 in solid hardwood 46×24; estimated — **weigh it** |
| Keyboard, mouse, cables, small stuff | ~5 kg | estimate |
| **Working total** | **~45–55 kg** | |

Bench rated capacity is 300 lb / 136 kg (manual), so the bench itself is not a
concern; the concern is what the *motor* sees through the worm.

Note: monitor on a desk arm clamped to the top does not reduce the load — the
top still carries it. Only wall-monitoring removes it.

## Sizing runs (330 mm travel, 5.5 mm/crank-rev placeholder)

| config | worm η=0.3 | η=0.4 | η=0.6 |
|---|---|---|---|
| NEMA 17 (0.6 Nm), 24 V | ~11 mm/s | ~14 mm/s | ~22 mm/s |
| NEMA 17 (0.6 Nm), 48 V | ~17 mm/s | ~22 mm/s | ~33 mm/s |
| **NEMA 23 (1.9 Nm), 48 V** | **~44 mm/s** | ~44 mm/s | ~59 mm/s |

Industry: budget desk 25 mm/s, Linak 3-stage 38 mm/s. Under ~15 mm/s feels broken.

**Reading:** the NEMA 17 is never *wrong*, but it is only *good* when the worm
cooperates, and worm efficiency in a 3 mm-steel-leg bench nobody publishes is
the single least trustworthy number in this project (assumed 0.4, plausibly
0.3–0.6, dry/greasy changes it). Every row where N17 is fine is also a row where
N23 is comfortable. The price difference is ~$10 and the gearbox size
difference is one frame. The one real argument for N17 — a lighter
cantilevered package under the top — matters less on a steel bench than on an
IKEA tabletop.

## Recommendation

**NEMA 23, 1.5–2.1 Nm holding, N+M rated ≥2 A, on 48 V.** Ratio chosen last
by `sizing.py` after the measurements; expect ~2.5:1, possibly single-stage.
Fallback if envelope or budget forbids N23: big NEMA 17 (≥0.6 Nm, low
inductance) at 48 V with the ratio the script picks, accepting ~15–20 mm/s.

48 V is load-bearing logic, not excess: it is what keeps torque alive at speed
(chopper drive pushes current through inductance), and it is the same reason
the driver must be a 60 V part (TMC5160 class), not a 29 V one.

## Checklist before the print workstream cuts a gearbox

- [ ] Weigh PC + peripherals on a bathroom scale (tare with a box).
- [ ] Measure travel span with tape: full down to full up, mm.
- [ ] Measure mm per crank rev: count turns over a measured rise, twice.
- [ ] Rough-test worm friction (best-effort, drives conservatism): note crank
      effort empty vs loaded, up vs down. If cranking up fully loaded is real
      effort, assume η = 0.3.
- [ ] Measure clearance envelope under the top (motor hangs there): max
      diameter and length the gearbox+motor can occupy.
- [ ] Re-run: `python3 scripts/sizing.py --load-kg <measured>
      --per-rev-mm <measured> --travel-mm <measured> --voltage 48`
- [ ] Record the chosen motor + ratio in this file and the machine JSON.

Until then every printed gear dimension is provisional.
