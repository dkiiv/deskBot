#!/usr/bin/env python3
"""Measure your desk and print the numbers to put in the machine profile.

Three things have to come from a tape measure, not a datasheet. This script
makes that a five-minute job instead of a guessing game, and - importantly -
refuses to produce a profile when the inputs do not hang together, because a
silently wrong scale factor is the failure mode that makes a retrofit look
cursed: it moves, it reports a height, and the height is a lie.

usage:
    python3 scripts/calibrate.py                 # interactive
    python3 scripts/calibrate.py --json out.json # also writes a profile
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

TEMPLATE = {
    "identity": {"name": "my-desk", "url": "", "profile_version": 1},
    "drive": {"kind": "stepper_pulse"},
    "feedback": {"kind": "motor_encoder"},
    "mechanism": {"coupling": "single", "self_locking": True},
    "axes": [{
        "label": "crank",
        "steps_per_motor_rev": 3200,
        "gear_ratio": 0.0,          # from the gearbox you printed
        "mm_per_shaft_rev": 0.0,    # measured here
        "travel_low_mm": 0.0,       # measured here
        "travel_high_mm": 0.0,      # measured here
        "vmax_up_mm_s": 12.0,
        "vmax_down_mm_s": 14.0,
        "amax_mm_s2": 45.0,
        "direction": 1.0,
        "encoder_counts_per_motor_rev": 2000,
    }],
    "safety": {},
}


def ask(prompt: str, lo: float, hi: float, unit: str) -> float:
    """Numeric input with a sanity band. Ranges here are deliberately generous:
    they exist to catch a decimal point in the wrong place, not to police
    unusual desks."""
    while True:
        raw = input(f"{prompt} [{lo:g}-{hi:g} {unit}]: ").strip()
        try:
            v = float(raw)
        except ValueError:
            print(f"  not a number: {raw!r}")
            continue
        if not math.isfinite(v):
            print("  not finite")
            continue
        if lo <= v <= hi:
            return v
        print(f"  {v:g} is outside {lo:g}-{hi:g}; re-measure rather than override")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, help="write the resulting profile here")
    ap.add_argument("--assume-ratio", type=float, default=None,
                    help="gear ratio if you already know it (skips the question)")
    args = ap.parse_args()

    print("deskbot calibration\n" + "=" * 46)
    print("You need: a tape measure, and the desk empty (no monitors on it).\n")

    prof = json.loads(json.dumps(TEMPLATE))
    ax = prof["axes"][0]

    print("[1/4] travel limits")
    print("  Lower the desk until it physically cannot go further, and measure")
    print("  the floor to the TOP SURFACE. Same at the top of travel.")
    low = ask("  lowest height, top surface", 300, 1500, "mm")
    high = ask("  highest height, top surface", 300, 1500, "mm")
    if high - low < 50:
        print(f"\n  only {high - low:.0f} mm of travel? That is implausibly small - "
              "check you measured the same surface both times.", file=sys.stderr)
        return 1
    ax["travel_low_mm"] = round(low, 1)
    ax["travel_high_mm"] = round(high, 1)

    print("\n[2/4] travel per crank revolution")
    print("  This is the number that decides whether the desk is usable, and it")
    print("  cannot be looked up. Raise the desk a measurable amount, counting")
    print("  whole turns of the crank.")
    span = ask("  height gained", 20, 600, "mm")
    turns = ask("  crank turns that took", 0.25, 200, "turns")
    per_rev = span / turns
    print(f"  => {per_rev:.3f} mm per crank revolution")
    if not 0.5 <= per_rev <= 40:
        print(f"\n  {per_rev:.2f} mm/rev is outside the plausible band for a "
              "crank desk; re-measure.", file=sys.stderr)
        return 1
    # A second reading catches the classic miscount, and costs ten seconds.
    if input("  measure again for confidence? [Y/n] ").strip().lower() != "n":
        span2 = ask("  height gained (2nd)", 20, 600, "mm")
        turns2 = ask("  crank turns (2nd)", 0.25, 200, "turns")
        per_rev2 = span2 / turns2
        drift = abs(per_rev2 - per_rev) / per_rev
        print(f"  => {per_rev2:.3f} mm/rev, disagreement {drift * 100:.1f}%")
        if drift > 0.10:
            print("\n  readings differ by more than 10%. Something is loose, or "
                  "the turns were miscounted. Fix that first: a 10% scale error "
                  "means the desk stops 30 mm short of where it says.",
                  file=sys.stderr)
            return 1
        per_rev = 0.5 * (per_rev + per_rev2)
    ax["mm_per_shaft_rev"] = round(per_rev, 3)

    print("\n[3/4] gearing")
    print("  steps_per_motor_rev = full steps x microsteps (1.8 deg NEMA 17:")
    print("  200 x 16 = 3200). gear_ratio = motor revs per crank rev, which is")
    print("  your printed gearbox - ask the CAD workstream if it is not built.")
    if args.assume_ratio is not None:
        ratio = args.assume_ratio
        print(f"  gear ratio: {ratio:g} (from --assume-ratio)")
    else:
        ratio = ask("  gear ratio", 1, 100, ":1")
    steps = ask("  steps per motor revolution", 200, 51200, "steps")
    ax["gear_ratio"] = round(ratio, 4)
    ax["steps_per_motor_rev"] = int(steps)

    print("\n[4/4] cross-checks")
    travel = high - low
    crank_revs = travel / per_rev
    motor_revs = crank_revs * ratio
    print(f"  full travel      : {travel:.0f} mm = {crank_revs:.1f} crank revs")
    print(f"  hand crank today : ~{crank_revs:.0f} turns (owners of common benches")
    print(f"                     report 30-70, so this should look familiar)")
    print(f"  motor workload   : {motor_revs:.0f} motor revs per full move")
    for rpm, label in ((300, "quiet, 24 V"), (600, "fast, loud")):
        crank_rpm = rpm / ratio
        rate = crank_rpm / 60.0 * per_rev
        print(f"  at {rpm} rpm ({label:15s}): {rate:5.1f} mm/s -> "
              f"{travel / rate:4.0f} s end to end")
    if crank_revs > 200:
        print("\n  WARNING: >200 crank revs of travel. Either the measurement is"
              "\n  off or this bench is unusually fine-pitched; you will need a"
              "\n  lower gear ratio or a faster motor to get usable speed.")
    if crank_revs * ratio > 40000:
        print("\n  WARNING: >40k motor revs per full move. At 16 microsteps that"
              "\n  is a lot of pulses for continuous duty - consider 8 microsteps"
              "\n  (still ~0.01 mm resolution here) to halve the ISR load.")

    print(f"\n  => suggested vmax_up_mm_s = 12.0 (raise only if the desk stops,"
          f"\n     cools, and reports no stalls for a week)")

    if args.json:
        args.json.write_text(json.dumps(prof, indent=2) + "\n")
        print(f"\nwrote {args.json}")
        print("next: ./build/deskbot-sim " + str(args.json) + " --profile-only")
    else:
        print("\n" + json.dumps(prof, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
