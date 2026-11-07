# PCB agent handoff — ESP32 control board

You are working this half. The firmware logic already exists in `../core/` and
will not need to change for your design, as long as the HAL contract below (`../core/include/desk/hal.h`) is met.

Read `../README.md` first. **Run this on the owner's desktop/WSL2, not the agent
server** — that box has no display, and KiCad's IPC API needs a running GUI
session. `kicad-cli` alone is not enough for anything interactive.

## Tooling on the desktop (WSL2)

**KiCad 9 + MCP** is the path with the most working tooling.

```sh
sudo apt install -y kicad kicad-python       # or the AppImage / ppa:kicad/kicad-9.0-releases
```

WSLg gives you the GUI (`freecad`/`kicad` open windows on Windows). In KiCad:
Preferences → API → tick "Enable IPC API" and copy the token. The API talks
protobuf over a Unix socket inside WSL; from a Windows-side MCP client you need
to bridge that socket, so **run the MCP server inside WSL2 too** and connect the
agent over TCP:

```sh
pip install kicad-mcp-pro          # or: git clone https://github.com/blwfish/kicad-mcp
```

Capability you should expect from any of these bridges: schematic symbol/label
placement, footprint placement, track/via routing with A\* pathfinding, ratsnest
and clearance queries, ERC/DRC via `kicad-cli`, and Gerbers/BOM/CPL export.

Two that are worth the install specifically because they check work instead of
trusting it:

- `lucia-h/connections-visualizer-plugin` — renders flying leads over the live
  PCB. The single most useful thing for an agent, because it turns "did I route
  this right" into a picture you can look at.
- `laton-it/pcb-tools` — export board state for external analysis.

**EasyEDA.** Its API is a proprietary cloud JS plugin (`editor-api`) that only
runs inside the editor and cannot attach to an external process, so an agent
driving EasyEDA means driving a GUI, not an API. If the owner insists on EasyEDA
for the final layout, do the work in KiCad and use EasyEDA only for the JLCPCB
handoff, or hand the netlist + placement notes over. Do not fight this — KiCad
exports what JLCPCB needs directly.

**Headless verification works without any of the above** and you should be
running it continuously regardless of which editor you use:

```sh
kicad-cli sch erc board.kicad_pro -o erc.rpt
kicad-cli pcb drc board.kicad_pro -o drc.rpt --unconnected-pads-ok
kicad-cli pcb export gerbers board.kicad_pcb -o fab/
```

If `apt install kicad` fails with 404s on unrelated packages, the image has a
stale index: `sudo apt-get update` first. That cost this project one confused
debugging session.

## Board contract

Derived from `core/include/desk/hal.h`. These are the wires the software needs;
everything else is yours.

| HAL method | What the board must provide |
|---|---|
| `read_inputs()` | 2 limit switches, up/down/stop buttons, enable chain, temperature |
| `drive_rate(ax, sps)` | STEP/DIR (or SPI/UART) to one or two stepper drivers |
| `set_motor_power()` | driver ENABLE, and the motor supply switch |
| `set_brake()` | optional |
| `set_led()` / `set_buzzer()` | status |
| `nv_read/nv_write` | calibration + presets (ESP32 NVS is fine) |

### Reference architecture

```
                 ┌──────────────────────────────────────────┐
 36-48 V PSU ─────┤ XT60/DC jack ── TVS ── E-stop(NC) ── P-MOS┤── VM to driver
                 │                                │         │
                 │  buck 24→5 (3 A min) ──┬───────┤ ESP32-S3├── WiFi + BLE
                 │  LDO 5→3.3 ────────────┘       │         │
                 │                                │         │
  AS5600 ──I2C───┤                                │  SPI───┼── TMC5160 ──4 wire── motor
  2× reed ──GPIO─┤                                │        │
  3× button GPIO─┤                                │  UART──┼── debug/CAN future
  TMP36/NTC GPIO─┤                                │        │
  OLED    ──I2C──┤                                └────────┘
                 └──────────────────────────────────────────┘
```

**Parts, with the reasoning that matters:**

- **Driver: TMC5160 (SPI), not TMC2209.** Three independent reasons, all load-
  bearing, plus one part-selection catch below. Register-level plan:
  `TMC5160-CONFIG-PLAN.md`.
  1. *Voltage headroom.* TMC2209's absolute maximum on VS is 33 V with 29 V
     operating. A 24 V supply is already at 83 % of absolute maximum, and a
     bench gets bumped, dragged across a garage floor, and plugged into whatever
     supply is nearest. Regenerative spikes from stopping an inertial load during
     descent are exactly how you find out. TMC5160 runs to 60 V. If you must use
     a 2209 for availability, add a clamp (TL431-referenced or a zener rail) and
     treat 24 V as the ceiling, not the nominal.
  2. *Integrated ramp generator + encoder input.* The 5160 does the trapezoid in
     silicon and can take the ABN encoder directly, which means the ESP32 can be
     slept on and the step timing stops depending on WiFi interrupts. The core
     still supervises position in software either way — that is where the safety
     logic lives — but having the hardware ramp means no EMI-ghost-step surprise
     becomes a stall you cannot distinguish from a jam. (Note: `TMC5160-
     CONFIG-PLAN.md` §5 decides NOT to use the on-chip ramp — two planners is
     a bug factory — but having the option, and the SPI + encoder inputs,
     still distinguishes the part.)
  3. **Part-selection catch: the 5160 has NO output stage.** It is a gate
     driver — the four power MOSFETs and current-sense resistors are external,
     per the datasheet application circuit. Budget for 4× N-channel power
     MOSFETs (logic-level gate, ≥60 V, ≥ the IRUN you size), 4 gate drive
     paths, and 2 sense resistors (0.05 Ω class) as real BOM and layout work,
     or deliberately pick the discrete-module carrier route instead. Do not
     discover this during layout. (If integrated FETs feel mandatory, the
     fallback is TMC5130 — but that caps the rail at ~36 V and coil at 2 A
     RMS, which constrains the motor decision; see MOTOR-DECISION.md.)
  4. **VMTB / heatsink:** these packages need their exposed pad soldered to real
     copper with vias to an internal plane. A "soldered-down module" without
     that is how you get a driver that works on the bench and thermally trips
     after 40 seconds of lifting. Budget copper, not a heatsink you glue on.
- **MCU: ESP32-S3.** USB-OTG/CDC for the CLI with no extra chip, and enough
  UART/SPI/I2C. If the owner prefers bare-metal reliability over WiFi, an
  RP2350 or STM32 works — the core is portable, only `firmware/` changes.
- **Encoder: AS5600 (I²C, magnetic, contactless, £2).** Mounts to the motor
  back with a diametric magnet on the shaft. 4096 counts/rev. Alternative if
  you want quadrature: AS5047 (SPI, 14-bit). Note the core expects counts, not
  an absolute angle — handle the wrap in the HAL, and see `firmware/README.md`.
- **Buttons: 3, not 6.** Up, Down, Stop/acknowledge. That matches how commercial
  desk keypads work and it constrains the UI honestly.
- **Limits: 2 reed switches + magnets** on the leg travel. They are *reference*,
  not protection — the mechanical stops are protection. Wire them NC if you can
  source them; a broken wire then reads as a limit, which is the safe direction.
- **E-stop: physically separate from the MCU.** A normally-closed switch in
  series with the motor power, and a *tap* into a GPIO so the software can
  report it. Do not route the only e-stop function through a GPIO-driven MOSFET:
  firmware bugs, watchdog resets, and brown-outs all end with the desk falling.
  This is the one place where "the software checked" is not an acceptable answer.
- **Motor power switch: P-MOS high side with a pull-up that fails open**
  (= motor off) when the GPIO is unpowered/floating, plus a bleed resistor on
  the gate. An N-MOS low side is wrong here because it would disconnect the
  motor's common.
- **TVS on 24 V** (SMBJ26A or similar) plus bulk capacitance ≥ 470 µF at the
  driver. Descending an inertial load dumps energy back; without the cap the
  supply pumps and the MCU browns out mid-descent, which is the worst possible
  time to reset.
- **Flyback/clamp on the motor outputs only if the driver does not have internal
  clamps.** TMC parts do — body diodes rated for full motor current rectify
  back-drive into VM, so external Schottkys are usually unnecessary and can make
  regen worse. Check the datasheet you are laying out from, not this file.
- **Debug: UART header + SWIM/JTAG pads + a spare UART.** Also put a 4-pin
  header on I2C and one on the motor connector. You will want all of them.
- **CAN or RS-485 pads ( populated or not ):** `AxisCoupling::MasterFollower`
  exists in the core for dual-leg desks. If the owner ever moves this software
  to a two-motor desk, a bus is how you do it without running six wires across
  moving machinery. Cheap to add as pads now, annoying to retrofit.

### Power

**Run 36-48 V, not 24 V, unless the sizing says 24 is enough.** `scripts/sizing.py
--voltage 48` shows why: with a chopper driver the same motor holds substantially
more torque at speed on a higher rail, because current reaches the winding faster.
On this bench the binding constraint is power-at-speed (the worm eats ~2.5x), and
voltage is the cheapest way to buy it - often the difference between a NEMA 17
that is adequate and one that is 3x too slow.

Design the board for a **48 V input** even if the first build runs at 24 V; that
is the decision the driver choice above is already assuming. Current: size for
motor + ~1 A of margin (a NEMA 17 at 1.5 A/phase draws well under 40 W, plus
inrush and regen absorption).

Do **not** use a supply with a latching shutdown - a driver fault would latch it
off and hang the desk mid-travel. Use one that current-limits and auto-recovers.
Bulk capacitance at the driver matters more here than amp-hours; see the regen
note above.

## Deliverables

1. KiCad project in `hardware/` (schematic, PCB, footprints, 3D).
2. BOM as `hardware/bom.csv` with **LCSC part numbers**, JLC-ready.
3. `hardware/NETLIST-NOTES.md` — net names and the mapping to HAL methods.
4. ERC clean, DRC clean at JLC 2-layer minimums, Gerbers + CPL + pick-place
   exported.
5. A one-page bring-up procedure: what to check with a multimeter before first
   power, what to check with the motor *unconnected*, then loaded.

## Acceptance criteria

- [ ] ERC clean, zero unconnected pins (including the ones you intentionally left).
- [ ] DRC clean at the fab's stated minimums for the process you're ordering.
- [ ] Every net in the schematic traces to a HAL method or is explicitly
      labelled unused. No mystery nets.
- [ ] Driver VS worst-case (supply tolerance + regen spike + your trace
      inductance) sits below 80 % of the IC's absolute maximum, computed not
      assumed. At 48 V nominal this is the criterion that rules parts out.
- [ ]Exposed pads with thermal vias; document copper area used for the driver.
- [ ] E-stop function verified to work with the MCU **unpowered**. Test this on
      hardware and write down the result; it is the criterion most likely to be
      quietly skipped.
- [ ] Connector pinouts documented in the silkscreen or a printed card.
- [ ] Board does not require a prototype spin to run the firmware — i.e. the
      UART header and a way to re-flash exist without jtagpondering.

## Non-goals

- Do not change `core/` to fit your board. The HAL is seven methods; adapt to
  them. If you genuinely cannot, write up why in `docs/` and raise it.
- No mains-voltage anything. 24 V DC from a brick, full stop. If the owner wants
  an internal PSU, that's a separate board with separate rules.
- Do not add a battery/UPS "feature". It creates a second way for the desk to
  move when nobody expects it.
