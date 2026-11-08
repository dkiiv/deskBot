// One half of the loader-parity check: the HOST (simulator) loader, isolated in
// its own translation unit.
//
// WHY A SEPARATE .cpp
// The firmware loader (firmware/include/deskbot/profile_embed.h + json_embed.h)
// and the host loader (tests/include/deskbot/profile_loader.h + json_mini.h)
// both define `namespace json` with different Value/Parser types. They cannot be
// included in the same translation unit - it is a redefinition error, not an
// inconvenience. That collision is exactly why the two loaders have been allowed
// to drift: nothing could hold both and compare them. This file plus
// firmware_host_tests.cpp give the comparison somewhere to live: each TU
// includes one parser, they meet only at the desk::MachineProfile they fill.
#include <cstdio>
#include <string>

#include "desk/machine_profile.h"
#include "deskbot/profile_loader.h"

namespace desk_test {

// Parse a machine profile with the host loader. `json_text` is written to a
// temporary file because the host loader reads from disk (the simulator's real
// use); returns false and sets `err` on any parse/validation failure.
bool load_host(const std::string& json_text, desk::MachineProfile& out,
               std::string& err) {
  const char* path = "/tmp/deskbot_loader_parity.json";
  {
    FILE* f = std::fopen(path, "wb");
    if (!f) { err = "cannot open temp profile"; return false; }
    std::fwrite(json_text.data(), 1, json_text.size(), f);
    std::fclose(f);
  }
  return desk::load_machine(std::string(path), out, err);
}

}  // namespace desk_test
