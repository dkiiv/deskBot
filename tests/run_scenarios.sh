#!/usr/bin/env bash
# Fault-injection scenarios for the simulator - the safety half of the gate.
#
# `make run` proves the happy path homes cleanly. It proves nothing about the
# cases the core exists to handle, and those are the ones that must not regress
# silently: a stall detector that stops tripping looks exactly like a desk that
# stopped being dangerous.
#
# ASSERT ON THE FAULT NAME, NOT THE EXIT CODE. This is not stylistic: an earlier
# revision of this file asserted only "exit 3", and when validated against a
# deliberately sabotaged core (lag computed, Faults::stall never latched) it
# still passed - the desk had walked into its own hard stop and some *other*
# latch tripped. "Something was wrong" is not "the hazard I injected was
# detected". Validation procedure is at the bottom; run it whenever a scenario
# is added.
#
# The sim's contract (tests/sim/sim_main.cpp): 0 = finished clean,
# 1 = profile/load error, 2 = bad usage, 3 = ended with a latched fault, plus a
# `f=<names>` field naming every latch that is set.
#
# Usage: tests/run_scenarios.sh [path/to/deskbot-sim] [machine.json]
set -uo pipefail

SIM="${1:-build/deskbot-sim}"
PROFILE="${2:-machines/husky-46x24.machine.json}"

if [[ ! -x "$SIM" ]]; then
  echo "run_scenarios: simulator not built at '$SIM' - run 'make sim' first" >&2
  exit 1
fi
if [[ ! -f "$PROFILE" ]]; then
  echo "run_scenarios: no machine profile at '$PROFILE'" >&2
  exit 1
fi

pass=0
fail=0

# check <label> <clean|fault:name> <sim args...>
check() {
  local label="$1" want="$2"; shift 2
  local out rc need fault
  out="$("$SIM" "$PROFILE" --quiet "$@" 2>&1)"; rc=$?
  # The latch set at the end of the run, read from the sim's own f= field.
  fault="$(printf '%s\n' "$out" | grep -o 'f=.*' | tail -1 | cut -d= -f2-)"

  if [[ "$want" == clean ]]; then
    if [[ $rc -eq 0 && "$fault" == none ]]; then
      printf '  ok    %-44s (clean)\n' "$label"; pass=$((pass+1))
    else
      printf '  FAIL  %-44s expected clean, got exit %d f=%s\n' "$label" "$rc" "$fault"
      fail=$((fail+1))
    fi
    return
  fi

  need="${want#fault:}"
  if [[ $rc -eq 0 ]]; then
    printf '  FAIL  %-44s HAZARD UNDETECTED (exit 0, clean)\n' "$label"
    fail=$((fail+1))
  elif [[ $rc -ne 3 ]]; then
    printf '  FAIL  %-44s expected latched fault (3), got exit %d f=%s\n' \
           "$label" "$rc" "$fault"
    fail=$((fail+1))
  elif [[ " $fault " != *" $need "* ]]; then
    # Right outcome, wrong reason: the desk stopped, but not because the
    # injected hazard was recognised. Asserting the name is what catches this.
    printf '  FAIL  %-44s latched "%s"; this scenario injects %s\n' \
           "$label" "$fault" "$need"
    fail=$((fail+1))
  else
    printf '  ok    %-44s (f=%s)\n' "$label" "$fault"; pass=$((pass+1))
  fi
}

echo "simulator scenarios: $SIM against $PROFILE"

echo "[1] the reference path must stay fault-free"
check "home, full travel up, full travel down" clean \
    --script 'home,go=950,go=700'

echo "[2] a blocked path must not be driven forever"
# What this injects, precisely: --stall-at arms the obstacle on any descent
# below the given height, so it blocks the *home hunt* coming down from mid
# travel. Before the first anchor there is no trusted position, so the lag
# detector cannot exist and the geometry-derived watchdog is the only thing
# between a blocked home and a cooked driver. That watchdog is the protection
# being asserted here - not stall detection.
check "blocked home hunt trips the watchdog" fault:home_timeout \
    --stall-at 800 --script 'home,go=950'

echo "[3] slipping gears must be caught while driving"
# The retrofit failure mode the whole design is built around: the mechanism
# quietly stops matching the command. At these settings the model runs ahead of
# the encoder by more than stall_lag_mm_moving, and the stall detector must be
# what notices - not an incidental hard-stop contact.
check "gears slip while driving" fault:stall \
    --slip-at 2 --slip-rate 0.6 --script 'home,go=950'

# Known gap, deliberately not papered over: an obstacle under a *descending,
# already-homed* top - the README's headline hazard - is covered by the C++
# suite (tests/core/test_core.cpp, "stall" group) but not reachably from the
# CLI, because --stall-at also arms during homing's own descent, so homing
# times out before the move under test ever starts. Closing it needs a sim
# switch to arm the obstacle after the first home; tracked in roadmap.md rather
# than smuggled into a gate fix.

echo
if [[ $fail -ne 0 ]]; then
  echo "scenarios: $pass passed, $fail FAILED"
  exit 1
fi
echo "scenarios: $pass passed, 0 failed"

# VALIDATION - do this whenever a scenario is added, because a hazard assertion
# that cannot fail is theatre:
#   1. cp -r core /tmp/broken/ && edit /tmp/broken/core/include/desk/safety.h:
#      keep the lag arithmetic, replace the two `return Status::Stall` lines in
#      the trip branch so Faults::stall is never set.
#   2. c++ -std=c++17 -I/tmp/broken/core/include -Itests/include \
#         tests/sim/sim_main.cpp -o /tmp/sim_broken
#   3. ./tests/run_scenarios.sh /tmp/sim_broken
#      -> scenario [3] MUST go red. If it passes, it is not testing the stall
#         detector. (Scenario [2] is expected to stay green here: it asserts a
#         different latch, which is the point of asserting names.)
