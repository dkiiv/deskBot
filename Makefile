# deskbot - host-side build (tests + simulator).
#
# The firmware is built with PlatformIO in firmware/; this Makefile covers the
# platform-independent core, which is deliberately host-compilable so motion
# and safety logic can be tested with no hardware attached.
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wshadow -O2 -g
INC       = -Icore/include -Icli/include
BUILD     = build

TEST_SRC  = core/test/test_core.cpp
SIM_SRC   = cli/src/sim_main.cpp
# Header-only core: every .h is a dependency, otherwise edits to core/include
# silently leave stale binaries, which is a very confusing way to "fix a bug".
HEADERS   = $(wildcard core/include/desk/*.h) $(wildcard cli/include/deskbot/*.h)

.PHONY: all test sim clean run

all: test sim

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/test_core: $(TEST_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) $(TEST_SRC) -o $@

$(BUILD)/deskbot-sim: $(SIM_SRC) $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(INC) $(SIM_SRC) -o $@

test: $(BUILD)/test_core
	./$(BUILD)/test_core

sim: $(BUILD)/deskbot-sim

# A quick end-to-end sanity run against the Husky profile.
run: sim
	./$(BUILD)/deskbot-sim machines/husky-52x24.machine.json

clean:
	rm -rf $(BUILD)
