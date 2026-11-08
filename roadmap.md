# Roadmap — honest audit of deskBot (2026-11-07)

> **Progress (same day, after approval).** Phase 1 and Phase 2 are complete and
> pushed; §7 has a ✅ on each landed step, with what was *not* done and why.
> Phase 3 items are the ones this audit deliberately left for the owner, because
> they change safety semantics and the repo's own rule 5 wants the trade
> recorded with a human behind it.

Audited at `b46c23d` on macOS (Apple clang 17, C++17). At audit time **no work
was done inside the repo**: `git status` was clean and `build/` was the only
thing created. Every claim below
is backed by something I ran — the commands are in §6 so you can re-check me.

Read this as: the architecture is good and should not be restructured. The problems are
concentrated in **(a) the firmware path that no test can reach**, **(b) two leaks in the
`make test` gate this repo treats as its source of truth**, and **(c) documentation drift
in a repo that is unusually docs-driven.**

---

## 0. Verdict

This is a well-above-average embedded codebase. The central idea — a header-only
motion/safety core that is compiled and *behaviourally* tested on a laptop against a fake
desk that tracks truth independently of the controller — is the right architecture for a
retrofit project, it is enforced mechanically rather than by convention, and it has
already caught real bugs (the phantom-stall step-rounding pair, the pre-home trust gap,
the home-recovery catch-22). `docs/DECISIONS.md` is the best artifact in the repo: it
records *why*, including where the author was wrong, which is exactly what a future
contributor needs in order to disagree with the reasoning instead of re-breaking it.

That said, three things need fixing before any hardware is plugged in, and one of them is
a live firmware bug:

| | |
|---|---|
| **Must fix before hardware** | A1 (dead guard → encoder sampled 2×/tick), B1 (stale firmware test binaries), B2 (CI sim step cannot fail) |
| **Should fix before hardware** | A2 (lost feedback is not a fault), E1 (two loaders, silently diverging keys), B3 (sanitizers documented but never run) |
| **Decide, then act** | E2 (delete the firmware JSON parser?), A3/E4 (is `axis_count: 2` actually supported?), C2 (two-point calibration is unreachable) |
| **Owner-blocked, highest leverage overall** | E6: the three tape measurements. Not code, and half the roadmap waits on it. |

The single most important sentence in the repo is AGENTS.md's "Nothing here has moved a
real desk." I did not find anything that contradicts it, and I looked for overclaiming
specifically.

---

## 1. What the layout gets right (do not change this)

- **`core/` has no hardware includes, and that is *testable*.** `make test` compiling core
  with zero Arduino headers is the enforcement of "portable engine". AGENTS.md's argument
  for why core is not nested under `firmware/` is correct and the dependency arrows only
  point one way (`firmware/ → core/`, `tests/ → core/`).
- **`MachineProfile` is a real seam, not an aspirational one.** The dual-leg test builds a
  `MasterFollower` machine out of the *same* controller object and no core code changes —
  that's the generic claim being checked rather than asserted. `AxisCoupling::Single`
  being load-bearing (skew = alarm, not correction target) is the kind of detail that
  separates a design from a translation of one.
- **`FakeBench` is the reason this repo works.** Truth is tracked separately from the
  model, so a self-deluding controller fails. The microstep-quantisation parity between
  fake bench and controller (both floor-cumulate a remainder) is a subtle thing to get
  right and is commented with the reason.
- **Fail-safe direction is chosen consistently and correctly.** Sensor missing → reports
  *hot* (refuse motion) not cold. Open wire on NC chain → reads "stop". LEDC cannot
  represent a frequency → stay stopped and *count it*. Rate above the pulse ceiling →
  clamp but count, so the encoder catches the lie. Lost revolution datum → sticky latch,
  recoverable only by ack + re-home, never by one good-looking sample.
- **Comments are engineering comments.** They say why, they name the failure mode
  (`// Descending gets gentler deceleration: gravity is assisting`), and several quote the
  measured numbers that forced the design. The `kPwmResBits` block in `main.cpp` derives
  the LEDC band from framework source with file/line citations instead of guessing.

---

## 2. Findings — correctness

### A1 · `read_inputs()`'s sample-once-per-tick guard is dead code — and the encoder is read twice per control tick
**Severity: high · firmware · verified by grep + verbatim replica**

`firmware/src/main.cpp:404`:
```cpp
void begin_tick() { ++tick_; }
void read_inputs(Inputs& out) override {
  if (tick_ == cache_tick_) { out = same_tick_copy_; return; }   // never taken
```
`cache_tick_` is **never assigned anywhere** (only its initializer at `:699`;
`grep -c 'cache_tick_ = ' firmware/src/main.cpp` → 1, which *is* the initializer). So the
guard is always false and `control_task` — which calls `read_inputs()` twice per period
(via `g_ctrl.tick()` at `:882`, then again for button policy at `:887`) — re-reads
hardware on both calls. I confirmed the arithmetic with a replica of the exact guard
(§6, probe 3): **10 hardware samples for 5 control periods = 2.0×**.

Why it matters, worst first:
1. **It breaks the invariant the encoder correctness argument rests on.** The comment at
   `:397-400` says sampling once "matters because the encoder unwrap must advance exactly
   once per control period." `deskbot::AngleUnwrap::update()` runs **twice** per period, so
   the per-tick delta the unwrap's `kMaxDeltaCounts = 512` sanity bound is designed around
   is no longer one control period of motion — it's half one. The 19× margin documented in
   `encoder_as5600.h:23-29` is real but the *premise* is now wrong.
2. **Double I2C inside a 2 ms budget.** Each AS5600 read takes a bus mutex with a
   5 ms timeout (`:442`) — longer than the control period. Two per period doubles the
   odds of the bus timeout that sets `enc_ok = false`, i.e. self-inflicted
   "no feedback this tick".
3. **The bring-up counters lie by 2×.** `i2c_errors_`, `bus_lock_timeouts_`, `enc_faults_`
   all advance at twice the rate the docs say they should, during the exact bring-up where
   the README tells you "i2c=, clamp=, rej= must stay 0".
4. `publish()` runs twice, so the seqlock's "one tick of staleness" contract is two;
   `DebouncedPin::sample()` is fed two identical samples per period (benign today, but the
   state machine is written for one).

**Fix (one line, no core change):** assign `cache_tick_ = tick_;` after sampling in
`read_inputs()`. Then add a cheap guard against it regressing: the guard's whole purpose is
"at most one hardware read per period", so give `ESPHal` a `samples_` counter and assert it
in a host test — see A1-test below.

---

### A2 · Losing the encoder on a machine that declares one is not a fault
**Severity: high (safety semantics) · core · verified in simulation**

`machines/*.json` declares `feedback.kind: motor_encoder`. If the encoder goes bad at
runtime (I2C failure, `g_enc_latched`, cable pulled), the HAL reports
`encoder_valid = false` — and the core continues with **full authority and no latch**.

Probe 2 (a profile with feedback, feedback then removed mid-life):
```
move_to after feedback lost   -> ok
ended at 799.9 mm; state=ready faults.any_hard=0 (blocked=0 stall=0 pos_lost=0)
after -120 mm hand push with no encoder: state=ready pos_lost=0; move_to(950) -> ok
```
A 120 mm hand-push on a desk that has silently become open-loop goes undetected, and the
desk then drives to an absolute height computed from a model nothing is checking.

The inconsistency is with the core's *own* boot policy. `controller.h:82-86` decides that
an open-loop machine may believe **nothing** across a power cycle ("boot: open-loop machine,
stored height dropped — home required") because nothing can disprove a stale height. At
runtime, the same machine loses its disproof and is simply not told. Boot says open-loop is
untrustworthy; runtime says it's fine. `firmware/README.md` is honest that `enc=OFF` means
"the desk still homes, still obeys limits, but stall detection is off by construction" —
which is a *description* of the gap, not a mitigation.

**Proposed fix (needs owner sign-off + a test + a DECISIONS.md line, per AGENTS.md rule 5):**
in `Controller::tick()`, if `m_->feedback != FeedbackKind::None` and `!in.encoder_valid`
persists beyond N ticks (I'd take N from the profile, default ~250 ms), set
`Faults::position_lost`. That reuses the existing, already-tested recovery semantics —
homing is the sanctioned way out (`home()` deliberately clears `position_lost`) — so it is
a small change with no new state machine. Add the "cable pulled mid-life" case to
`tests/core/test_core.cpp`.

---

### A3 · Dual-leg machines only ever home leg 1
**Severity: medium-high (only if you ever build one) · core · verified**

`Inputs::limit_low[kMaxAxis]` exists, the firmware wires and samples GPIO 6/7 into
`limit_low[1]`/`limit_high[1]`, and **no core code ever indexes past 0** — every one of the
11 limit references in `controller.h` is `[0]` (grep in §6). `run_homing()` anchors on
`in.limit_low[0]` and infers leg 2 from the step model. `skew` is *correctly* disarmed
during homing (`safety.h:198`, well-reasoned for a tied shaft), so during the one phase
where leg 2 is provably unanchored, the only per-leg check is switched off. A leg-2 that
meets its own stop early has no detector; afterwards its position is a guess.

Related, and measured: the rate-domain equaliser and the stall detector disagree.
`command_axis()` splits the **mean** rate with a per-leg trim, integrates the **mean** into
`cmd_mm_[0]`, and the supervisor compares that mean against **leg 1's** encoder. Probe 5,
a legal `MasterFollower` move with a sticky leg 2 (`leg2_drag = 0.55`, well inside what the
code calls "an unequal load, or a worn nut on one side"):
```
STALL LATCHED at t=1.1s lag=2.27 mm   (stall trips at 1.6 mm sustained 220 ms)
peak trim on leg 1 = 25% (saturated), state=retreating, skew=0
```
So on a dual machine a sticky leg is indistinguishable from a jam. The existing test only
runs `leg2_drag = 0.70`, which sits under the trip — so the equaliser's margin is
**unproven**, not proven. This is not a bug in the equaliser; it is the detector measuring
a quantity that the equaliser intentionally moves.

**Recommendation (decision, not a patch):** either (a) declare `axis_count > 1`
unsupported-for-safety in the profile schema and have the firmware `fatal_loop` on it
until per-leg models exist, or (b) give each axis its own model + encoder comparison, which
is the real fix and a substantial one. Today `firmware/README.md` says "wired but
untested", which I'd upgrade to "the core's primary fault detector is not valid for this
topology" — that difference changes what a future agent is allowed to build.

---

### A4 · Redundant open-loop workaround still in `setup()`
**Severity: low · firmware · verified**

`main.cpp:1071-1078` drops `g_cal` when `feedback == None && homed`. `controller.h:82-86`
now does exactly that, with a log line. Both are RAM-only, they do not conflict, and
`docs/firmware-design-notes.md` concern #2 asked for this to be decided *in one place*.
It has been decided — delete the firmware copy and update the note. (Note #2 itself is now
stale: the core fixed it.)

---

## 3. Findings — the verification gate leaks

The repo's discipline is "make test green before any commit" and "subagent reports of
builds/tests are claims, not facts". These four items weaken that gate. None of them are
hard.

### B1 · `make test-firmware` does not depend on the firmware headers it tests
**Severity: high (for a repo that trusts this gate) · Makefile · verified**

`HEADERS = $(wildcard core/include/desk/*.h) $(wildcard tests/include/deskbot/*.h)`. The
`build/test_firmware` rule adds `-Ifirmware/include` but never lists
`firmware/include/deskbot/*.h` or `firmware/include/generated/*.h` as prerequisites.
Verified: `touch firmware/include/deskbot/encoder_as5600.h && make test-firmware` →
re-runs the **stale binary**, no compile line. The files whose staleness this hides are
`encoder_as5600.h` ("the highest-risk 40 lines in firmware/"), `json_embed.h` and
`profile_embed.h`. The Makefile carries a comment explaining that it solved this exact
problem for core headers — the same class was left open one directory over.

**Fix:** `FW_HEADERS = $(wildcard firmware/include/deskbot/*.h firmware/include/generated/*.h)`
and add it to the `test_firmware` prerequisites.

### B2 · `make run`'s pipeline masks the simulator's exit code, so CI's fault check cannot fail
**Severity: high (CI) · Makefile · verified**

`Makefile:50` pipes the sim through `| tail -3`; make sees `tail`. Proof:
```
./build/deskbot-sim … --stall-at 800 --script 'home'          -> exit 3   ("run ended with a latched fault")
./build/deskbot-sim … --stall-at 800 --script 'home' | tail -3 -> exit 0   <- what make/CI sees
```
CI's step is literally named *"End-to-end simulation (home + full travel, **fault-free
exit**)"*. It can currently only fail on a crash, not on a fault. **Fix:** `SHELL :=
/bin/bash` + `.SHELLFLAGS := -o pipefail -c`, or drop the pipe (`--quiet` already trims
output) and post-process only in a debug target. One line either way.

### B3 · Sanitizers are documented as the build but are not the build
`firmware/README.md` and the header of `tests/firmware/firmware_host_tests.cpp` both show
`-fsanitize=undefined,address`. The Makefile (hence CI) uses `-O2 -g` only. The two things
those tests guard are a fixed-pool hand-written parser with manual index arithmetic and
`memcpy` into a shared char arena, plus integer unwrap math — the highest-yield UBSan/ASan
targets in the repo. **Fix:** a `make test-asan` target and a CI job running it. While
there, add a test that a profile *larger* than `kMaxNodes = 256` / `kMaxText = 4096` is
rejected cleanly (the exhaustion paths exist and are ~untouched; husky uses ~120 nodes).

### B4 · Warnings are tolerated on a green run
`make test` at `-Wall -Wextra -Wshadow` currently emits 2 warnings and passes:
`fake_bench.h:111 unused variable 'a'` (in the *test harness* — the code that decides
whether everything else works) and `controller.h:699 private field 'log_burst_' is not
used` (already flagged in the design notes as "the kind of member that accretes a lie").
**Fix:** delete both, add `-Werror` to CI as a ratchet.

### B5 · Two of the referenced test files do not exist
`test_json_embed.cpp` is cited by `firmware/include/generated/machine_profile_json.h:8`
**and re-generated on every run by `gen_machine_header.py:78`** — the generator
remanufactures the stale claim. `test/json_embed_test.cpp` is cited by `json_embed.h:22`.
Neither is in `git ls-files`; the real file is `tests/firmware/firmware_host_tests.cpp`.
Also worth knowing when reading the headline numbers: the 91,334 count comes from 123
`CHECK` sites wrapped in sweeps (4096×5 unwrap steps, etc.). The number is honest — it is
what ran — but next to "141 checks" in the same README table it invites a comparison that
isn't meaningful. 141 *behaviours* is the coverage figure; 91,334 is iterations.

### B6 · One `main()`, 14 sequential groups, no isolation
`tests/core/test_core.cpp` has no per-group isolation: a crash in group 3 silently loses
groups 4-14, and no group can be run or skipped alone. Cheap fix: a `--group=` filter or a
small registry of `void(*)()` + name, matching what the firmware suite already does with
named functions.

---

## 4. Findings — dead, unreachable, or unenforced

### C1 · Config knobs that do nothing
| Item | Evidence |
|---|---|
| `safety.obstacle_backoff` | Loaded by **both** loaders, documented in README's fault table, read by nothing in `core/` (`grep obstacle_backoff core/` → the one declaration). The sim's obstacle behaviour is a `FakeBench` property, not a policy. An operator-facing knob that silently does nothing. |
| `safety.skew_alarm_mm` ("warn the operator") | Loaded, never read in core. |
| `Faults::blocked` / `Status::Blocked` | `blocked = true` is assigned **nowhere**. So `any_hard()` includes a latch no code can raise, `home()` refuses a fault that cannot exist, and README lists `blocked` as a live fault. Wire it to StallGuard per `TMC5160-CONFIG-PLAN.md` §4, or remove it and stop describing it. |
| `struct Outputs` (`hal.h:27-38`) | No HAL method takes it; core never fills it. Design notes concern #3 already calls this "a seam that stopped short". Fine as a sketch — but in a header it reads as contract, which is how the firmware ends up re-deriving LED policy. |
| `types.h::Height`, `types.h::kNoLimit` | Zero references. |
| `json_mini.h::as_str_flag()`, `as_str_ref()` | Zero references. |
| `fake_bench.h::mmlIKELY_()` | Zero references, and the name looks like an editing slip (sits beside `steps_to_mm_pure_`). Its comment claims it's "inverse of Controller::command_axis" — the thing the file's own long comments argue about. Delete or fix the name and use it. |
| `fake_bench.h::steps_to_mm_pure_(steps_t, float)` | Ignores the `float` parameter. |
| `controller.h::home_budget_us_` | Written (`:518`), never read. |
| `NvsKey::kNvsProfile`, `kNvsCounters` | Mapped to NVS slots in firmware, never read or written. |
| `MachineState::Settling` | In `parked` (`:255`) and `name()`; **never assigned** by any code path. So a whole state in the enum, and a whole branch of the integrity check, is unreachable. |

### C2 · Two-point ruler calibration is implemented and unreachable
`Calibration::counts_to_mm()` prefers `ref_low_*`/`ref_high_*` when `has_refs` — the feature
`DECISIONS.md` uses to absorb printed-gear error ("scaling error is then measurable rather
than assumed away"). **Nothing writes it**: no loader, no serial command, no CLI command,
and `scripts/calibrate.py` never asks for the two reference readings. So the mechanism that
makes a printed ratio forgiving is dormant, and a real builder never exercises it. This is
the highest-value *feature* item in the roadmap and it is mostly script + one serial
command, not core surgery.

### C3 · `persist_height()` can rewrite a `Calibration` with a stale `profile_version`
The firmware boot check (`load_calibration()`) correctly rejects magic/version mismatch, but
the core's `persist_height()` writes the struct back with whatever version it was loaded
with and never re-stamps `magic`/`profile_version`. Benign today because boot rejects
first; it's a latent trap the moment calibration can be reloaded without a restart (which is
what `NvsKey::kNvsProfile` hints at). One `cal_->profile_version = m_->profile_version;` in
`Controller::init()`, plus a test.

### C4 · `Controller::supervisor()` hands out a mutable reference, and the tests lean on it
`tests/core/test_core.cpp` calls `ctl.supervisor().clear_faults()` 8 times, including
immediately after homing. That is precisely the firmware workaround design-notes §1
identified as the wrong place for policy ("a workaround living in one of N future frontends
is how it goes wrong"). The core fixed `home()`; the tests still blanket-clear whatever a
home leaves latched, which makes the suite weaker at proving "a clean home leaves no
latches" than its green count suggests. `acknowledge()` is the sanctioned narrow path —
prefer it, and add one test that a clean home leaves `faults().any_hard() == false`
*without* any `clear_faults()` call.

---

## 5. Architecture: two decisions worth having, and the rest is drift

### E1 · The two profile loaders can diverge silently, and one already has — in CI's blind spot
`json_embed.h`'s header comment states the invariant perfectly: *"If you change a key or a
default here, change profile_loader.h too… a silently diverging default shows up as a desk
that stops at a different height than the simulator says it will."* The invariant is
enforced by prose. I diffed the two loaders' key sets:

- embedded reads `home_timeout_margin` and `home_max_ms`;
- **the host loader ignores both.**

So a profile that tunes the home watchdog changes firmware behaviour and does *nothing* in
the simulator — the exact failure the comment warns about, already present, invisible to
`make test`. **Fix I'd actually adopt: one host test that parses the same JSON text through
both loaders and asserts every `MachineProfile` field is equal.** ~30 lines, closes the
class permanently, and it is the only thing that makes "the simulator runs the same core the
ESP32 runs" true for the *config* path as well as the logic path.

### E2 · Consider deleting the firmware JSON parser entirely
One machine profile, two hand-written parsers (`json_mini.h` 183 lines + host loader;
`json_embed.h` 327 lines + embedded loader), both individually well tested, plus ~120 CHECK
sites maintaining the parity that E1 shows can still break. The embedded parser exists for
good reasons (no `std`, no allocation, `-fno-exceptions`) and it is written carefully. But
the alternative isn't the one `DECISIONS.md` rejected — that entry rejects a **hand-maintained**
struct, and this repo already has a generator (`gen_machine_header.py`) that is *already*
CI-sync-checked. A **generated** struct would: reuse the host loader's validation once, in
CI, where a bad schema fails loudly; delete ~460 lines of boot-path char-level parsing from
a safety device; and make E1 structurally impossible.

Keep the counter-argument on the record, because it's the reason to say no: verbatim JSON in
flash means a human with a serial console can read the machine's actual numbers back, and
`docs/firmware-design-notes.md` prices a filesystem in the boot path at "no thank you" (I
agree). Neither of those requires a *parser* at boot — a generated struct plus a printed
summary gets both. Flagging as a decision, not a unilateral change.

### E3 · `Controller::tick()` maintains a cross-return invariant by hand
185 lines, 7 early returns, and 6 separate `was_low_ = …` assignments that each must
remember to run or homing's rising-edge detection silently breaks. New exit paths that
forget it produce no compile error and no failing test (this is the same bug class the repo
already caught once with `cmd_steps_[0] += delta` being deleted). Cheapest structural fix:
stop latching `was_low_` — derive it at the top of the next tick from a stored previous
`Inputs`, or hoist the assignment to a single exit. Same applies to `step_carry_`.

### E4 · Per-machine-mean model vs per-leg feedback
The root of A3: `cmd_mm_[0]` is the machine mean, `enc_mm_[0..1]` are legs, and the primary
fault detector compares the mean against leg 1. Exactly right for `coupling: single` (the
legs *cannot* differ). Not right for `master_follower`, on the machine most likely to need
it. Worth writing down as a rule in `machine_profile.h`: "the model is per-machine, so
per-leg safety checks require per-leg models", so the next reader doesn't re-discover it
from a phantom stall.

### E5 · No command authority concept in the core
The remote **deadman** is good design (jogs must be re-asserted; buttons outrank remote).
But `move_to()` has no liveness, no source, no notion of authority, and `g` is unthrottled
(design-notes open question 6). Harmless with two frontends (buttons, serial) and Wi-Fi
deliberately absent. It stops being harmless at the first network frontend, and at that
point whoever adds it will be re-deriving policy the core should own. Note it now, build it
before the first remote client, not during.

### E6 · Nothing here is calibrated, and that's the critical path
Every number in `machines/husky-46x24.machine.json` is an estimate (correctly marked
in-file), and half the safety case is expressed in millimetres derived from
`mm_per_shaft_rev` — a number `DECISIONS.md` insists must come from a tape measure and
which nobody has taken. AGENTS.md already says "everything else waits on this," and the
CAD/PCB lanes are both marked blocked on it. So the highest-leverage roadmap item is not a
code change at all: run `scripts/calibrate.py` and `scripts/sizing.py`, then re-derive the
mm thresholds. I'd add one guard while there: `make test` should refuse a machine profile
whose scale numbers are still the template defaults, so "placeholder" can't quietly ship as
"calibrated".

### E7 · Documentation drift (cheap; the repo is docs-driven so this matters more here than most)
- **README "Layout" block describes a tree that no longer exists**: lists `cli/src/sim_main.cpp`
  and `cli/include/deskbot/` — both moved under `tests/` in `d5999a5`. The top-level map at
  README:20-27 was updated; the detailed block wasn't.
- `docs/firmware-design-notes.md:3` still says "103 checks passing" (now 141).
- `firmware/README.md` (and the header of `tests/firmware/firmware_host_tests.cpp`) documents
  the host-test build as running "from `firmware/`" while the source path in the command
  (`tests/firmware/firmware_host_tests.cpp`) only resolves from the repo root — `ls` from
  `firmware/` fails. Copy-paste as written does not work.
- `firmware/include/README` is the **untouched PlatformIO template** — generic prose about
  what a header file is, with a `src/main.c` example. Delete.
- Planned paths are cited inconsistently: `AGENTS.md:142` says `docs/husky-bench BUILD
  guide` while README and the tree map say top-level `husky-bench/`; `docs/husky-bench`
  doesn't exist either way. `hardware/bom.csv`, `hardware/NETLIST-NOTES.md` are cited as if
  present. Mark planned artifacts `(planned)` at every mention, not just the first.
- No tags and no `CHANGELOG`. The claim "core complete + 141 checks green at this commit" is
  load-bearing across lanes; freeze it with a `v0.1-core` tag.
- `.gitignore` lacks `.DS_Store` (this is a macOS clone and Finder is already doing it).
- `docs/MOTOR-DECISION.md`'s sizing table quotes a `gear_ratio` 9:1 as the rejected baseline
  while the machine JSON says 5.06 from the CAD workstream; both are placeholders, but the
  doc reads as settled. One "superseded by" line each.

---

## 6. How I verified this

```sh
make test            # 141 core + 91,334 firmware checks, 0 failed — green, as documented
make run             # clean sim run
```
- **A1**: `grep -n 'cache_tick_' firmware/src/main.cpp` (only the initializer) + a replica
  of the exact guard with two `read_inputs()` calls per period → 10 samples / 5 periods.
- **A2**: compiled harness — home, remove feedback, `move_to` → `ok`, 120 mm hand-push →
  no fault, next `move_to` → `ok`.
- **A3**: `grep -n 'limit_(low|high)\[' core/include/desk/controller.h` → 11 hits, all
  `[0]`. Plus: `MasterFollower`, `leg2_drag = 0.55`, `move_to(950)` → **stall latched at
  t = 1.1 s**, peak |model − leg-1 encoder| = 2.27 mm vs the 1.6 mm trip, trim saturated at 25%.
- **B1**: `touch firmware/include/deskbot/encoder_as5600.h && make test-firmware` → ran the
  stale binary (no compile line). Core headers do rebuild, correctly.
- **B2**: sim with `--stall-at 800` → exit **3**; identical line `| tail -3` → exit **0**.
- **B4**: the 2 warnings above are from the `make test` output I ran.
- **B5/C1/E1/E7**: `git ls-files` + reference-counted greps, and a key-set diff of the two
  loaders.
- Probes lived outside the repo (`~/.hermes/cache/scratch/dbaudit/`). Repo left clean:
  `git status --porcelain | wc -l` → 0.
- **Not verified at audit time:** anything requiring an ESP32 (no board; `pio` was not
  installed on this machine, so `pio run -e esp32s3` was not run — CI was the only
  evidence the firmware compiles, and A1 lived on exactly that unverifiable path) and
  anything requiring real hardware or network (no `gh` auth, so whether CI was
  currently green was not checked).
- **Since approval:** `pio` was installed in a scratch venv and `pio run -e esp32s3`
  builds clean for the touched firmware (RAM 11.2% / Flash 16.4%), so the firmware half
  of A1 is now compiled-and-checked rather than CI-only. **Nothing has run on hardware**
  — no board exists, and that limit is unchanged.

---

## 7. Proposed sequence — Phase 1 & 2 landed, Phase 3 pending owner

Ordered so each step is independently landable and the two CI lies go first. ✅ =
committed and pushed; the commit is named where a claim needs an address.

**Phase 1 — make the gate tell the truth** ✅
1. ✅ `B1` firmware headers added to `test_firmware`'s prerequisites. Verified both
   directions: a firmware-header touch now rebuilds, and an untouched tree does not.
2. ✅ `B2` `make run` no longer pipes, and `.SHELLFLAGS` adds pipefail. Verified with a
   profile-only fault (injected `home_max_ms=1`): sim exits 3 and `make run` now exits 2
   (`Error 3`) where it exited 0 before the fix — so the "fault-free exit" CI step can
   fail for the reason it names.
3. ✅ `B4` both warnings removed, `-Werror` added as a ratchet (override `WEXTRA=`).
4. ✅ `B3` `make test-asan` (ASan+UBSan, UB fatal) + CI job. UBSan clean locally; **ASan
   is first proven in CI**, because Apple clang's ASan runtime hangs even on
   `int main(){return 0;}` here. `SAN=undefined` is the documented local subset.
5. ✅ `B5` the two phantom test filenames fixed *in the generator* (not just the header,
   which it regenerates); README distinguishes behaviours from iterations. Regenerated
   header is byte-identical, so CI's sync check passes.
   ➕ **went beyond the roadmap:** the phase is worthless if the hazard assertions
   can't fail, so `tests/run_scenarios.sh` asserts *which latch trips* (not just the exit code),
   and was validated by mutation — neutering the stall latch reddens only the stall
   scenario, neutering the home watchdog reddens only that one. First draft asserted only
   "exit 3" and passed a core that cannot detect a stall; that failure is why it asserts
   names now.

**Phase 2 — the firmware bug and the parity hole** ✅
6. ✅ `A1` the guard was not just reassigned — the counter pair moved to
   `tick_guard.h` and is host-tested (the firmware build cannot catch this bug and the
   old code proves a bare member rots silently). `pio run -e esp32s3` builds clean.
7. ✅ `E1` cross-loader equality test. It could not be one TU — both parsers define
   `namespace json`, a genuine redefinition error — so it spans two TUs meeting at the
   filled `MachineProfile`. TDD red first: it caught the `home_timeout_margin`/`home_max_ms`
   divergence as three failing host-side assertions, then green after the host loader read
   both. No reverse divergence found.
8. ✅ `A4` redundant open-loop workaround deleted from `setup()`; design-notes status note
   added (kept concerns #1/#2 as history, not an open list).
9. ✅ `D/E7` README layout tree (verified `d5999a5` moved `cli/`→`tests/`), check counts,
   firmware build command (was uncopy-pasteable), `firmware/include/README` deleted,
   planned paths marked, `.DS_Store`/`build-san` ignored.
- ⏭️ **`v0.1-core` tag not created** — pushing a tag to a public repo is the kind of
  semi-irreversible act left for the owner.
- ⏭️ **AGENTS.md:31 still says "91,334"** (now 91,395) — protected file, edit was declined
  at approval; owner one-liner.

**Phase 3 — safety semantics: deliberately NOT started (owner lane).** Each needs a sign-off
call this audit is not authorised to make: `A2` (lost feedback → fault), `A3`/`E4` (is
`axis_count > 1` supported?), `C1` (wire or delete each dead knob), `C3`/`C4`. Rule 5
requires a test *and* a DECISIONS line per item; doing those unilaterally would be exactly
the "software lane edits core" the repo's rules forbid without the owner.

**Phase 4** not started (features + the `E2` parser decision).

**Phase 3 — safety semantics (needs owner sign-off + test + DECISIONS line each)**
10. `A2` lost-feedback-on-a-feedback-machine → `position_lost`, with a mid-life-cable-pull test.
11. `A3`/`E4` decide: gate `axis_count > 1` in firmware, or design per-leg models. Do not
    leave it ambiguous in both directions.
12. `C1` resolve each dead knob (wire `obstacle_backoff` into the retreat decision, or delete
    it; same for `skew_alarm_mm`, `Faults::blocked`, `MachineState::Settling`, `Outputs`).
13. `C3` re-stamp `profile_version` in `init()`; `C4` prefer `acknowledge()` over
    `clear_faults()` in tests and add the no-`clear_faults()` clean-home assertion.

**Phase 4 — features that unlock the build**
14. `C2` two-point calibration: `calibrate.py` prompts for the two ruler anchors + a serial
    command to write them, so `has_refs` stops being dead. This is the item that makes the
    printed gearbox's dimensional error measurable instead of hoped for.
15. `E3` `tick()` single-exit refactor; `B6` per-group test selection.
16. `E2` the parser decision, taken deliberately, recorded in `DECISIONS.md` either way.

**Blocked on the owner (not code):** `E6` — the tape measures and weigh-ins in
`docs/MOTOR-DECISION.md`. Until those land, the mm-denominated thresholds in the profile
are estimates and the CAD/PCB lanes stay correctly parked.

---

## 8. Explicitly *not* proposing

- **No restructuring.** The core/HAL/tests split, header-only core, generated committed
  header, `firmware/` as consumer of `core/`: all justified, and AGENTS.md defends each with
  the dependency arrows to prove it. I'd only be adding a way to get this wrong.
- **No "just use FreeRTOS timers / an RTOS scheduler / a motion library."** The
  recompute-from-current-state trapezoid and the fixed 2 ms tick with assumed `dt` (a hiccup
  slows the desk rather than lying about position — encoder stays arbiter of reality) are
  deliberate and commented. That's a considered design, not an accident.
- **No new abstractions to "clean up" `main.cpp`.** 1,097 lines of one-file board glue with
  every pin tied to the HAL method it serves and every stub marked `// STUB:` and listed in
  a table is *good* embedded layout. Splitting it would hurt the auditability that makes it
  trustworthy.
- **No Wi-Fi/MQTT/Home Assistant.** Deliberately absent per AGENTS.md, and E5 says the
  reason it should stay absent for now is that the authority model isn't there yet.
- **No weakening of the stub disclosure.** "The driver is not configured — the biggest gap
  before hardware" being a loud boot banner, a README row, and a doc rather than a shrug is
  the standard I'd want to hold, not edit.
