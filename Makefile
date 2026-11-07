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

TEST_SRC = tests/core/test_core.cpp
SIM_SRC  = tests/sim/sim_main.cpp
FW_SRC   = tests/firmware/firmware_host_tests.cpp
# Header-only core: every .h is a dependency, otherwise edits to core/include
# silently leave stale binaries, which is a very confusing way to "fix a bug".
HEADERS  = $(wildcard core/include/desk/*.h) $(wildcard tests/include/deskbot/*.h)

.PHONY: all test test-core test-firmware sim run clean

all: test sim

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/test_core: $(TEST_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) $(TEST_SRC) -o $@

$(BUILD)/test_firmware: $(FW_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) -Ifirmware/include $(FW_SRC) -o $@

$(BUILD)/deskbot-sim: $(SIM_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) $(SIM_SRC) -o $@

# The gate: both suites plus every machine profile parses as valid JSON.
test: test-core test-firmware
	@python3 -c "import json,glob,sys; ps=glob.glob('machines/*.machine.json'); [json.load(open(p)) for p in ps]; sys.exit(0 if ps else 'no machine profiles found')" \
	  && echo "machine profiles: json valid"

test-core: $(BUILD)/test_core
	./$(BUILD)/test_core

test-firmware: $(BUILD)/test_firmware
	./$(BUILD)/test_firmware

sim: $(BUILD)/deskbot-sim

# End-to-end sanity: simulate the Husky profile through a home sequence.
run: sim
	./$(BUILD)/deskbot-sim machines/husky-46x24.machine.json --quiet \
	  --script 'home,status' | tail -3

clean:
	rm -rf $(BUILD)
