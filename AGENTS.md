# AGENTS.md — read this first

**deskbot**: software for motorising a mechanical height-adjustable desk
(currently the owner's Husky 46×24 work bench: hand crank → stepper + printed
gearbox + ESP32 controller). Started October 2026. The motion/safety core is
complete and host-tested; the physical build has not started.

If you're an agent picking this repo up: find your lane below, do the
**Current state → What is left** work for it, and obey **Rules**.

## The one idea everything hangs off

`core/` contains no hardware and no hardware headers. It is header-only C++17
that compiles and runs on a laptop against a fake desk (`core/include/desk/
fake_bench.h`) that tracks *true* position independently of the controller.
Every behaviour — including fault behaviour like blocked descent or a hand-
pushed top — is a host test before it is ever firmware. Keep it that way:

- **Never** put Arduino/ESP-IDF includes in `core/`.
- A new desk = a new `machines/*.machine.json`, not a code fork.

```
core/       the product: motion planner, homing, stall/thermal/skew supervision
            (header-only C++17 — see core/include/desk/*.h; controller.h is
            "the machine": one tick() call drives everything)
machines/   machine profiles = THE desk-specific seam. JSON only.
firmware/   ESP32-S3 firmware (PlatformIO). Implements desk::Hal. May not
            change core/ — concerns go to docs/firmware-design-notes.md.
tests/      everything host-compiled that proves the above:
              core/     behaviour suite (141 checks) incl. fake_bench
              firmware/ host tests for firmware-only logic (91,334 checks)
              sim/ + include/  CLI simulator that drives the real controller
scripts/    calibrate.py (measure your desk), sizing.py (motor math)
husky-bench/  (planned) prints + build guide for the specific bench
docs/       decisions and agent handoffs — see lanes below
```

## Verify before you claim anything

```sh
make test          # both host suites + machine JSON validity. Must stay green.
make run           # end-to-end simulation of the Husky profile
cd firmware && pio run -e esp32s3   # firmware must keep compiling
python3 firmware/scripts/gen_machine_header.py machines/<name>.machine.json
                   # after ANY machines/*.json edit — CI enforces sync
```

CI (.github/workflows/ci.yml) runs all of this on every push. Nothing has run
on real hardware yet — never claim hardware behaviour.

## Lanes (who does what, and what's left)

### Owner (Curtis) — blocking decisions
1. Motor + supply + final gear ratio: complete the checklist in
   `docs/MOTOR-DECISION.md` (two weigh-ins, two tape measurements, then
   re-run `scripts/sizing.py`). Everything else waits on this.
2. `python3 scripts/calibrate.py` results → commit updated
   `machines/husky-46x24.machine.json` (three placeholder scale numbers are
   in there; they are marked).

### CAD/print lane — `docs/cad-agent.md`
Printed gearbox, motor mount, shaft coupling (keep the hand crank usable),
encoder mount. Runs on the **owner's desktop/WSL2** (FreeCAD-MCP or
build123d; the doc has toolchain setup + hard-won API pitfalls).
Blocked on: sizing decision above. Deliverables + acceptance in the doc.

### PCB lane — `docs/pcb-agent.md` + `docs/TMC5160-CONFIG-PLAN.md`
KiCad on owner's desktop/WSL2. Critical catch already documented: the TMC5160
needs **external power MOSFETs**. Register-level SPI plan in TMC5160-CONFIG-PLAN.
Blocked on: sizing decision (rail voltage, motor current).

### Firmware lane — `firmware/`, notes to `docs/firmware-design-notes.md`
Compiles; never flashed (no board). Biggest gap: **TMC5160 SPI config is a
stub** (driver runs POR defaults — see plan §2 for what to write, and the
no-memory-addresses rule: transcribe `tmc5160_regs.h` from the datasheet).
Next: serial command handling exists; Wi-Fi/HaaS deliberately absent.

### Software lane (core) — mostly done; open items in
`docs/firmware-design-notes.md` §"concerns": one accepted core change proposed
(`Inputs::stall_guard_load`) — needs owner sign-off, not unilateral core
edits. Add features via machine JSON fields where possible.

## Rules for agents

1. `make test` green before any commit. Red CI blocks everything else.
2. `core/` and `machines/` edits: software lane only, with a test added in
   `tests/core/` first (failing test → fix → green, this repo has caught real
   bugs that way; the home-recovery and boot-trust tests are the model).
3. If you edit `machines/*.json`, regenerate the firmware header (command
   above) — CI fails otherwise.
4. CAD/EDA and anything graphical: on the owner's desktop, not the agent
   server. The server is headless; the lanes say so.
5. Safety-critical behaviour (fault handling, limits, e-stop semantics) needs
   a test AND a line in `docs/DECISIONS.md` explaining the trade.
6. Subagent reports of builds/tests/files are **claims, not facts** — verify
   the artifact yourself before relaying. (This repo has been burned; it's in
   DECISIONS for a reason.)
7. Commits: `dkiiv <dkiiv@users.noreply.github.com>`. No real email in git
   history — the repo is public; so is every commit author field forever.
8. Don't commit: toolchain installs, generated STL/STEP without sources,
   `.pio/`, anything secret (there are none; keep it that way — this is a
   public repo).

## Where the bodies are buried (read before "fixing" something)

`docs/DECISIONS.md` — the bugs this project actually had and the reasoning:
phantom stalls from per-tick step rounding; the pre-home trust gap; the
home-recovery catch-22 (a desk that lost its position could not home to
recover — gates were circular); boot-time trust; and why the first sizing
attempt was 8× too slow (power, not gear ratio). The firmware design notes
list what the firmware agent found in core — two of those became the tests
above.

## Status snapshot (October 2026)

Done: core+tests (141), simulator, sizing model, firmware scaffold
(compiles, unflashed, SPI-stubbed), CI, decision docs, agent handoffs.
Not done: hardware sizing decisions, any CAD, any PCB, any flashing, any
physical install, docs/husky-bench BUILD guide. Nothing here has moved a real
desk.
