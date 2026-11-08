# deskbot - host-side build (tests, simulator, and CI entry points).
#
# Layout logic: core/ is the product (header-only, board-independent).
# Everything that RUNS code on a host to prove the product works lives under
# tests/. The ESP32 firmware builds with PlatformIO in firmware/; this Makefile
# covers everything that must pass on a laptop (and in CI) first.
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wshadow -O2 -g
INC       = -Icore/include -Itests/include
BUILD     = build

# Recipes run through bash with pipefail. Without it, a recipe that pipes a
# program into `tail` reports *tail's* exit status, and a faulting simulator
# looked like a passing end-to-end run. Belt and braces: `run` no longer pipes
# at all, but any future piped recipe inherits the correct rule from here.
SHELL       := /bin/bash
.SHELLFLAGS := -o pipefail -c

# -Werror is a ratchet, not pedantry: the repo's gate is "`make test` green", so
# a warning that survives a green run is a noticed bug nobody acted on. Override
# with `make WEXTRA=` when a newer compiler introduces a warning you have not
# triaged yet.
WEXTRA ?= -Werror
FLAGS   = $(CXXFLAGS) $(WEXTRA)

TEST_SRC = tests/core/test_core.cpp
SIM_SRC  = tests/sim/sim_main.cpp
# Two TUs: firmware_host_tests.cpp includes the EMBEDDED parser; host_loader_tu.cpp
# includes the HOST parser. They cannot share a TU (both define `namespace json`
# with incompatible types) - see the comment at the top of host_loader_tu.cpp.
FW_SRC   = tests/firmware/firmware_host_tests.cpp tests/firmware/host_loader_tu.cpp

# Header-only: EVERY header a binary includes is a dependency, otherwise an edit
# silently leaves a stale binary behind, which is a very confusing way to "fix a
# bug" - or to watch a fixed bug stay fixed forever. This once covered only
# core/ and tests/include/ while the firmware host suite compiled
# -Ifirmware/include without declaring it, so edits to the encoder unwrap (the
# highest-risk 40 lines in firmware/) did not trigger a rebuild.
HEADERS    = $(wildcard core/include/desk/*.h) $(wildcard tests/include/deskbot/*.h)
FW_HEADERS = $(wildcard firmware/include/deskbot/*.h) \
             $(wildcard firmware/include/generated/*.h)

.PHONY: all test test-core test-firmware test-asan test-core-asan \
        test-firmware-asan sim run scenarios clean

all: test sim

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/test_core: $(TEST_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(FLAGS) $(INC) $(TEST_SRC) -o $@

$(BUILD)/test_firmware: $(FW_SRC) $(HEADERS) $(FW_HEADERS) | $(BUILD)
	$(CXX) $(FLAGS) $(INC) -Ifirmware/include $(FW_SRC) -o $@

$(BUILD)/deskbot-sim: $(SIM_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(FLAGS) $(INC) $(SIM_SRC) -o $@

# The gate: both suites plus every machine profile parses as valid JSON.
test: test-core test-firmware
	@python3 -c "import json,glob,sys; ps=glob.glob('machines/*.machine.json'); [json.load(open(p)) for p in ps]; sys.exit(0 if ps else 'no machine profiles found')" \
	  && echo "machine profiles: json valid"

test-core: $(BUILD)/test_core
	./$(BUILD)/test_core

test-firmware: $(BUILD)/test_firmware
	./$(BUILD)/test_firmware

# ---------------------------------------------------------------- sanitizers --
# The two most fragile things in the repo are hand-written parsers doing index
# arithmetic over fixed pools (firmware/include/deskbot/json_embed.h) and
# integer unwrap math (encoder_as5600.h). Both are exercised on the host, so
# both can and should run under ASan/UBSan. The firmware README documented
# exactly this command for a while without anything ever running it; CI runs it
# now. -fno-sanitize-recover=all makes undefined behaviour fatal rather than
# reported-and-continued, so a UB trip fails the build instead of printing and
# passing.
#
# Separate build dir: sanitiser binaries must not clobber the optimised ones.
#
# SAN picks the sanitiser set. CI uses the default (address + undefined). On
# macOS with Apple clang the AddressSanitizer runtime can hang even on
# `int main(){return 0;}` - a known interaction, not a bug in this code - so
# local runs should narrow it:
#     make test-asan SAN=undefined
SANBUILD ?= build-san
SAN      ?= address,undefined
SANFLAGS  = -std=c++17 -Wall -Wextra -Wshadow -g -O1 \
            -fsanitize=$(SAN) -fno-omit-frame-pointer \
            -fno-sanitize-recover=all
ASAN_OPTIONS ?= detect_leaks=1

$(SANBUILD):
	@mkdir -p $(SANBUILD)

$(SANBUILD)/test_core: $(TEST_SRC) $(HEADERS) | $(SANBUILD)
	$(CXX) $(SANFLAGS) $(INC) $(TEST_SRC) -o $@

$(SANBUILD)/test_firmware: $(FW_SRC) $(HEADERS) $(FW_HEADERS) | $(SANBUILD)
	$(CXX) $(SANFLAGS) $(INC) -Ifirmware/include $(FW_SRC) -o $@

test-core-asan: $(SANBUILD)/test_core
	ASAN_OPTIONS=$(ASAN_OPTIONS) ./$<

test-firmware-asan: $(SANBUILD)/test_firmware
	ASAN_OPTIONS=$(ASAN_OPTIONS) ./$<

# The message reports $(SAN), because a build that prints "address + undefined"
# after running `SAN=undefined` is a gate lying about what it checked.
test-asan: test-core-asan test-firmware-asan
	@echo "sanitizers: clean (-fsanitize=$(SAN), UB is fatal)"

sim: $(BUILD)/deskbot-sim

# End-to-end sanity: simulate the Husky profile through a home sequence.
# The sim exits non-zero on a latched fault, so this recipe must not pipe it
# into anything - the pipe would decide whether CI passes, not the desk.
run: sim
	./$(BUILD)/deskbot-sim machines/husky-46x24.machine.json --quiet \
	  --script 'home,status'

# Safety scenarios: fault injection, asserting WHICH latch trips. `run` proves
# the happy path; this proves the hazards the core exists for still bite.
scenarios: sim
	./tests/run_scenarios.sh

clean:
	rm -rf $(BUILD) $(SANBUILD)
