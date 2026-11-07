# CAD agent handoff — 3D printed gearbox and mounts

You are working this half. The software is done and will not need to change for
whatever you produce, **as long as the interface contract below holds**.

Read `../README.md` first. The relevant contract is three numbers in the machine
profile: `gear_ratio`, `steps_per_motor_rev`, `mm_per_shaft_rev`. You own the
first. The second is firmware config. The third is the bench's physics.

## Where to run this

**Do not run CAD on the agent server.** It is a headless LXC with no display and
no CAD tools installed. Run on the owner's desktop under WSL2, where the printer
toolchain and the slicer already live. This document is written so that an agent
meeting CAD for the first time on that machine can proceed without asking.

### Option A — FreeCAD + MCP (recommended start)

FreeCAD is the only CAD package with a mature, screenshot-returning MCP bridge
and it runs in WSL2 with `WSLg` (`sudo apt install freecad`, then `freecad`
opens a window on Windows).

```sh
uv tool install freecad-mcp        # or: pipx install freecad-mcp
```

In FreeCAD: Extensions → Add-on Manager → install the FreeCAD MCP add-on, then
start its RPC server from the add-on's menu (it listens on localhost XML-RPC).
Register the server with the agent client:

```json
{ "mcpServers": { "freecad": { "command": "uvx", "args": ["freecad-mcp"] } } }
```

The value of the MCP bridge over scripting is that it returns **viewports**: you
can see what you built, which is the whole reason CAD agents fail less often
than blind code generation. If screenshots are not coming back, you are working
blind — stop and fix that before modelling anything.

### Option B — CadQuery / build123d (headless, script-first)

Better for parametric parts you will iterate, because the model is code and the
diff is reviewable. Verified working headless (needs `libgl1`):

```sh
sudo apt install -y libgl1 libglib2.0-0t64
python3 -m venv ~/venv-cad && ~/venv-cad/bin/pip install build123d
```

Then export STEP for FreeCAD/slicer review and STL for printing. Keep the
scripts in `scripts/` so a dimension change is a one-line edit, not a modelling
session.

**API pitfalls already paid for** (build123d 0.13):
- The vector type is `Vector`; there is no `Vec2`/`Vector2`, and it has no
  `.rotated()`. Use plain math for polar points.
- `extrude()` must be called **after** the `BuildSketch` context closes, inside
  `with BuildPart()`. Calling it inside `BuildSketch` raises
  `extrude doesn't apply to ['BuildPart']`.
- The functional form `extrude(Face(...), amount=...)` returns an **empty** Part
  that reports volume 0. Silent. Do not use it.
- Holes: second sketch + `extrude(amount=..., dir=(0,0,-1), mode=Mode.SUBTRACT)`.
- `.moved()` needs `Location(Vector(x,y,0))`. Floats and tuples both raise.
- `is_valid` is a **property**, not a method.
- Export with the module-level `export_stl(part, path)` / `export_step(part, path)`.
  There is no `part.export_stl()`.
- Put every shape you measure through `part.volume`, `part.is_valid`
  and a bounding-box assertion. A silently-empty solid prints as nothing.

### Option C — Fusion 360

Autodesk ships a built-in MCP server (Fusion ≥ 2026-04: Preferences → General →
API → "Fusion MCP Server", listens on `127.0.0.1:27182/mcp`), plus cloud-hosted
`https://developer.api.autodesk.com/da/v3/mcp/fusion`. Use it if the owner wants
the .f3d in their Autodesk account. Caveats: Fusion is Windows-native (fine — run
Fusion on Windows, point the agent at it over localhost), and its licence for
commercial use is restricted; personal-use licence is fine for this.

Whichever you pick: **commit the parametric source, not just STL.**

## What to design

### Interface contract (fixed — everything else follows)

| Item | Value | Why |
|---|---|---|
| Bench drive socket | **10 mm hex** — MEASURE | owner-confirmed across sources; verify with calipers before printing |
| Motor | NEMA 17, 5 mm D-shaped shaft | 42×42 mm bolt pattern, 4× M3 on 31 mm |
| Motor mounting | 4× M3 through the 42 mm face | standard |
| Target reduction | **choose last** | motor + voltage + rate pick it; see Sizing |
| Encoder | magnetic, on the motor shaft | e.g. AS5600 (I²C) or ABN quadrature |
| Total mass budget | ≤ ~1.2 kg including motor | it hangs under a bench top |

Nothing in `core/` knows or cares what the gearbox looks like. `gear_ratio` in
the machine JSON is the only coupling. Print 9:1, put 9.0 in the JSON, done.

### Design decisions already made (and why)

**1. Keep the hand crank usable.** The original crank handle pushes in and out
and is held by one screw. Design the motor coupling as a **parallel** drive on the
same hex shaft — a one-way/overrunning coupling, or simply a printed clamp that
leaves the outer crank socket free — not a replacement of the shaft. Two reasons:
emergency lowering when the electronics are dead, and the bench keeps its
warranty-friendly reversibility. If you cannot find an off-the-shelf
overrunning clutch that fits, a sprag-less alternative is a printed dog clutch
the operator disengages to hand-crank, with a positive stop so it cannot
half-engage.

**2. Do not print a worm.** The bench's own worm and gear in each leg stay. You
are adding torque to an existing mechanism, not rebuilding it. The printed parts
carry only the ratio you add, and the bench's worm remains what makes the desk
self-locking. Losing self-locking is how a desk becomes a guillotine.

**3. Fewer stages than you think.** Size the ratio with the script below first;
only split it into stages if one wheel will not fit the clearance envelope. For
reference, at 5.06:1 a practical pair is 20T→45T twice (module 2, 10 mm face:
94 mm OD wheels, 65 mm centre distances, train roughly 130 x 70 mm), while a
single 9:1 stage would need a 148 mm wheel. Measure the gap between the bench
underside and the top's cross-members before committing to a wheel diameter, and
note that if the script lands near 2.5:1 a *single* stage may be all you need -
one less interface to print, align and lose efficiency to.

**4. Module 2 (0.66 mm tooth) minimum.** Module 1 is easier on the printer and
will strip. This gearbox's job is to slip before it jams.

**5. Backlash: 0.3–0.5 mm of circular thickness on printed teeth.** Confirmed
worth modelling in the prototype: a mesh sweep at working centre distance with
0.35 mm backlash produced positive flank clearance across a full tooth pitch and
no interference. Mesh checks must be done **in the mesh zone only** (clip both
gears to a wedge around the line of centres); a global min-distance between two
gears finds tip-to-root contact far from the pitch point and reports a confident
lie.

**6. The encoder reads the motor, not the output.** Deliberate. It then measures
the mechanism *as built* — including gear slop — and any slip between motor and
output shows up as a stall the software can act on. If you instead put the
encoder on the hex shaft after the gearbox, set `gear_ratio: 1.0` in the axis
profile and `feedback.kind` to `shaft_encoder`, because the ratio then lives in
the mechanism, not the measurement.

**7. Plan for the driver's heat, not the motor's.** A NEMA 17 at 1.5 A in an
enclosed printed housing under a bench is a small heater, and the gearbox is the
part that softens first (PLA glass-transitions ~60 °C). Choose PETG, ASA or PA.
PLA near a driver board on a bench that gets moved into sunlight is how the
gearbox slowly deforms. Add vents and give the driver a metal path.

### Sizing: run the model before you model anything

```sh
python3 scripts/sizing.py --load-kg 50 --per-rev-mm 5.5 --travel-mm 330
```

That script is the authoritative answer for this bench and it will stay more
current than any table pasted into a doc, because it recomputes against your real
load and measured pitch. What it says, in the order that matters:

**1. The gear ratio is not the constraint. Watts are.** Power required at the
motor is

    P = (load_mass x g x desired_rate) / (worm_efficiency x gear_efficiency)

and the ratio does not appear in it. Raising the ratio buys torque and spends
speed at the same exchange rate, so no ratio buys power. To go faster you need a
bigger motor, a higher supply voltage, or less friction.

**2. The worm is the whole story - and it makes descending a powered move too.**
Its low efficiency is exactly what makes the desk self-locking (good) and it costs
about 2.5x in required torque (less good). A consequence people get surprised by:
a truly self-locking worm (efficiency below ~0.5) cannot be over-driven by the
load, so the motor must *push the desk down* as well as up. Descent is not free,
which is inconvenient for sizing and excellent for safety - it is why a power loss
leaves the bench exactly where it was instead of dropping it. Size the gearbox for
the lift case and it covers both.

Hand-cranking feels trivial because a person is content with ~60 turns over a
minute, which is a couple of watts. Doing the same 330 mm in 13 seconds at 50 kg
is ~36 W *at the motor*, after the worm has eaten most of it. No NEMA 17 does
that comfortably.

**3. Speed targets, from published commercial desks:**

| class | rate | 330 mm takes |
|---|---|---|
| budget single-motor (FlexiSpot EN1/EC1 class) | ~25 mm/s | 13 s |
| Linak 3-stage column (DL4S class) | ~38 mm/s | 9 s |
| premium dual-motor | up to ~80 mm/s | 4 s |
| this project, first pass at 9:1 / 300 rpm | 3.1 mm/s | 108 s |

Under ~15 mm/s will feel broken next to anything with a keypad. The 20s are
genuinely comparable to a bought desk.

**4. Do not run a stepper at 200-400 rpm.** Mid-band resonance: torque collapses
and it stalls with plenty of torque on paper. An earlier draft of this project
parked the operating point at 300 rpm, which is the worst place in the range to
put it. Run above the band (600+) or below it, deliberately.

**5. Voltage is the cheapest performance lever.** The same motor on a 48 V chopper
drive holds markedly more torque at speed than at 24 V, because current reaches
the winding faster. That is a second reason `pcb-agent.md` specifies a 60 V-rated
driver (TMC5160 class) instead of one topping out at 29 V: at 36-48 V the 29 V
parts are not tight, they are unusable.

**Consequence for this workstream:** choose the ratio *last*, after the sizing
script has chosen motor, supply voltage and target rate. The right number is
usually lower than intuition suggests - the bench's own worm is already doing the
torque multiplication, and every extra stage is more printed plastic, more
reflected inertia and more loss. If the script says 2.5:1, print 2.5:1.

Re-run it after you change anything: `--load-kg` is the mass actually on your top,
and `--worm-eff 0.3` vs `0.6` is the difference between a design that works and
one that was modelled on a lucky gearbox.


## Deliverables

1. `husky-bench/cad/parametric/` — the generator scripts (gear train, housing, motor mount,
   shaft coupling) with every dimension derived from the interface table.
2. `husky-bench/cad/parts/` — exported STEP + STL, one per part, plus a printed test coupon:
   a 1:1 gear pair segment and a test fit of the hex bore. Print the coupon
   **before** the housing.
3. `husky-bench/cad/drawing/` — one PDF/HTML assembly view with dimensions that a human can
   check against their actual bench with a ruler.
4. Update `../machines/husky-46x24.machine.json` with the real `gear_ratio` and
   mark the two measured values as measured (date + method in a comment field).
5. `../husky-bench/BUILD.md` — print settings that worked, orientation, insert
   strategy, assembly order, and what failed.

## Acceptance criteria

Not "looks right in the viewport":

- [ ] Gear pair meshes with positive flank clearance across a full tooth pitch,
      measured in the mesh zone (report the number and the phase sweep).
- [ ] Centre distance matches `module × (T1 + T2) / 2` to within your tolerance.
- [ ] Every part reports `is_valid` true and non-zero volume, asserted in script.
- [ ] Hex bore test-fits the bench shaft: press fit if the intent is torque
      transfer without a pin, or a designed-in M4 grub screw / roll pin. Do not
      rely on printed hex alone for a reversible mod.
- [ ] Static load check: gearbox holds the bench at full travel with motor
      unpowered (self-locking preserved, no tooth deflection at the root).
- [ ] A printed tooth fails **slipping**, not shattering — verify by driving it
      into a hard stop and watching which gives.
- [ ] Housing tolerances: motor register fits, driver board sits, and the whole
      thing assembles with the top at its lowest position.
- [ ] `make test` still passes with your `gear_ratio` in the profile.

## Non-goals for this workstream

- Do not modify `core/`. If the software cannot express what you need, that is a
  bug worth raising, but the fix is usually one more field in `MachineProfile`.
- Do not remove or bypass the bench's worm drive.
- Do not optimise for speed over the duty budget — a faster gearbox that cannot
  rest is a worse gearbox.
