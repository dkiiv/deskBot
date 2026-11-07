#!/usr/bin/env python3
"""Motor sizing for a crank-desk retrofit: can it be fast *and* strong enough?

Why this exists: the first pass at this project quoted a speed without checking
whether the torque-speed curve allows it, which is the classic way a retrofit
project dies - the motor has the holding torque, the gearbox has the ratio, and
somehow it still cannot lift the desk. Holding torque is the number on the box.
Torque *while turning* is the number that moves the desk, and on a stepper those
differ by a factor of 4-10.

The binding constraint is power at speed, not torque at rest:

    P_motor = (load_weight x g x desired_speed) / (worm_efficiency x gear_eff)

which does not care what your ratio is. The ratio only decides whether the
motor happens to be at a point on its curve where it can produce that power.

Model note: the torque decay below is fitted to published NEMA 17 curves
(~0.45 Nm holding -> ~0.12 Nm at 600 rpm -> ~0.05 Nm at 1200 rpm, 24 V chopper
drive). It is an order-of-magnitude guide for *comparison*, not a substitute for
the torque-speed plot in the datasheet of the motor you actually buy. Verify
against that plot before committing a design.

usage: python3 scripts/sizing.py [--load-kg 50] [--travel-mm 330]
                                 [--per-rev-mm 5.5] [--voltage 24]
"""
from __future__ import annotations

import argparse
import math

G = 9.80665

# Candidate motors: (holding Nm, mechanical power ceiling W). Power ceilings are
# rough continuous-output limits for the frame size at a sane supply.
MOTORS = {
    "NEMA 17 (0.44 Nm)": (0.44, 25.0),
    "NEMA 17 (0.60 Nm)": (0.60, 35.0),
    "NEMA 23 (1.9 Nm)":  (1.90, 90.0),
    "NEMA 24 (3.1 Nm)":  (3.10, 140.0),
}

# Published NEMA 17 curve anchors, 24 V chopper drive, fitted as an exponential
# decay above a low-speed plateau. Holding torque is useless above the knee.
KNEE_RPM = 200.0
TAU_RPM = 670.0

# Mid-band resonance: steppers lose torque badly here. Avoid parking an
# operating point in this window, which an early draft of this project did.
RESONANCE = (200.0, 400.0)


def dyn_torque(holding_nm: float, rpm: float, voltage_scale: float = 1.0) -> float:
    """Available torque while running. voltage_scale > 1 lifts the whole curve
    because a chopper drive pushes current through the winding faster."""
    if rpm <= KNEE_RPM:
        return holding_nm * voltage_scale
    decay = math.exp(-(rpm - KNEE_RPM) / TAU_RPM)
    return holding_nm * voltage_scale * decay


def worm_torque_nm(load_kg: float, per_rev_mm: float, worm_eff: float) -> float:
    """Torque the worm needs at the crank shaft to raise the load.

    Screw-jack relation, losses on the driving side: T = F*p / (2*pi*eta).
    The worm is the reason this is hard - at eta ~0.4 you pay 2.5x. It is also
    the reason the desk holds position unpowered, so it stays.
    """
    f = load_kg * G
    p = per_rev_mm / 1000.0
    return f * p / (2.0 * math.pi * worm_eff)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--load-kg", type=float, default=50.0,
                    help="mass on the top (bench top + monitors + tools)")
    ap.add_argument("--travel-mm", type=float, default=330.0)
    ap.add_argument("--per-rev-mm", type=float, default=5.5,
                    help="measured: top travel per crank revolution")
    ap.add_argument("--worm-eff", type=float, default=0.40,
                    help="0.3-0.6 for a loaded worm; low efficiency is what "
                         "makes it self-locking")
    ap.add_argument("--gear-eff", type=float, default=0.85, help="printed spur pair")
    ap.add_argument("--voltage", type=float, default=24.0)
    args = ap.parse_args()

    v_scale = {12.0: 0.65, 24.0: 1.0, 36.0: 1.25, 48.0: 1.45}.get(
        args.voltage, max(0.4, args.voltage / 24.0))

    t_shaft = worm_torque_nm(args.load_kg, args.per_rev_mm, args.worm_eff)
    crank_rps_for = lambda mm_s: mm_s / args.per_rev_mm
    print(f"load {args.load_kg:g} kg | {args.travel_mm:g} mm travel | "
          f"{args.per_rev_mm:g} mm per crank rev")
    print(f"worm torque needed to RAISE that: {t_shaft:.2f} N.m at the crank shaft")
    print(f"supply {args.voltage:g} V -> torque curve x{v_scale:.2f} vs 24 V baseline\n")

    # The ratio-independent truth first, because it settles the argument.
    print("Power is what actually constrains this, and the gear ratio cannot "
          "help:\n")
    print(f"  {'rate':>7} {'time':>7} {'crank rpm':>10} {'power @load':>12} {'power @motor':>13}")
    for rate in (3.0, 6.0, 11.0, 20.0, 25.0, 38.0):
        p_load = args.load_kg * G * rate / 1000.0
        p_motor = p_load / (args.worm_eff * args.gear_eff)
        secs = args.travel_mm / rate
        print(f"  {rate:6.1f}m/s {secs:6.0f} s {crank_rps_for(rate)*60:10.0f} "
              f"{p_load:10.1f} W {p_motor:11.1f} W")
    print("\n  Industry reference: budget single-motor desks ~25 mm/s, Linak")
    print("  3-stage columns ~38 mm/s, premium dual-motor up to ~80 mm/s.")
    print("  Anything under ~15 mm/s will feel sluggish next to those.\n")

    print("What each motor can actually deliver at this bench's geometry:\n")
    hdr = f"  {'motor':<20} {'ratio':>6} {'motor rpm':>10} {'rate':>8} {'T avail':>9} {'T need':>8}  verdict"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for name, (hold, pmax) in MOTORS.items():
        best = None
        for ratio in (2.5, 3.0, 4.0, 5.0, 6.0, 9.0, 12.0):
            for rpm in (250, 400, 600, 900, 1200, 1600):
                rate = (rpm / 60.0) * args.per_rev_mm / ratio
                t_avail = dyn_torque(hold, rpm, v_scale)
                t_need = t_shaft / (ratio * args.gear_eff)
                # A stepper may be overloaded briefly; this is not a servo and
                # the desk is not cycled like a machine axis.
                if t_avail < t_need:
                    continue
                if (rpm / 60.0) * 2 * math.pi * t_avail > pmax * 1.2:
                    continue
                resonant = RESONANCE[0] <= rpm <= RESONANCE[1]
                score = rate * (0.45 if resonant else 1.0)
                if best is None or score > best[0]:
                    best = (score, rate, rpm, ratio, t_avail, t_need, resonant)
        if best is None:
            print(f"  {name:<20} {'-':>6} {'-':>10} {'-':>8} {'-':>9} {'-':>8}  "
                  f"cannot lift {args.load_kg:g} kg at any usable speed")
            continue
        _, rate, rpm, ratio, ta, tn, resonant = best
        note = f"{rate:5.1f} mm/s"
        if resonant:
            note += " (in resonance band - derate)"
        if rate >= 25.0:
            note += "  commercial-grade"
        elif rate >= 12.0:
            note += "  acceptable"
        else:
            note += "  sluggish"
        print(f"  {name:<20} {ratio:6.1f} {rpm:10.0f} {rate:7.1f}m/s "
              f"{ta:8.3f}N {tn:7.3f}N  {note}")

    print("\nReading this: for a light load a NEMA 17 is genuinely enough. As")
    print("the load climbs, the binding term stops being torque and becomes")
    print("watts - and raising the supply voltage is the cheapest way to buy")
    print("both torque-at-speed and headroom, which is also why the PCB note")
    print("specifies a 60 V-rated driver instead of one topping out at 29 V.\n")
    print("Also: never park the operating point at 200-400 rpm. Mid-band")
    print("resonance makes a stepper stall there with plenty of torque on paper.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
