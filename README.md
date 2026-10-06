# USBrepeater

A Teensy 4.1 USB-CDC repeater/controller built on the
[GAACE](https://github.com/GordonAnderson) (GAA Custom Electronics Core
Extensions) framework — [GAACE_Core](https://github.com/GordonAnderson/GAACE_Core)
and [ArduinoThread](https://github.com/GordonAnderson/ArduinoThread), following
the same layout as [GordonAnderson/Template](https://github.com/GordonAnderson/Template).

At its core it transparently relays a USB CDC connection from a PC through the
Teensy's USB host port to a downstream USB CDC device, exactly like a plain
USB-to-USB serial repeater. On top of that it adds a GAACE command processor,
a mode framework, a cooperative thread scheduler, persistent settings, and a
periodic ADC-driven control feature — all running alongside the passthrough
without disturbing it.

## Hardware

- Teensy 4.1
- Downstream USB CDC device connected to the Teensy's host port (5-pin header)
- PC connected to the Teensy's micro-USB (device) port

## USB port layout

Built with `-D USB_DUAL_SERIAL`, so the Teensy enumerates as **two** USB CDC
ports:

| Port | Purpose |
| --- | --- |
| `Serial` | Repeater byte pump: PC \<-\> downstream USB CDC device. Raw, transparent passthrough — nothing else writes to this port. |
| `SerialUSB1` | Command/control port: the GAACE `commandProcessor` lives here (ASCII line protocol, ACK `0x06` / NAK `0x15` responses). |

They're independent. A PC application that only needs the repeater can use
`Serial` exactly as if this were a single-port device; a separate tool
(terminal, script, MIPS host) can use `SerialUSB1` to query status or change
settings without ever touching the data stream.

## Ethernet

[QNEthernet](https://github.com/ssilverman/QNEthernet) drives the Teensy
4.1's built-in Ethernet MAC/PHY, adding two more ports — this time TCP, not
USB — that mirror the two USB ones:

| Port | Purpose |
| --- | --- |
| `EthCtrlPort` (default 5000) | Control port: every `SerialUSB1` command works verbatim over this TCP connection — same commandProcessor, same ASCII protocol. |
| `EthDataPort` (default 5001) | Passthrough port: a second source for the repeater byte pump, alongside `Serial`. |

Both are single-session — a new connection takes over from whatever was
previously connected, same policy every commercial serial-to-Ethernet device
server uses.

**Passthrough arbitration — "last speaker wins".** `Serial` and the Ethernet
data client can both drive the downstream device; whichever one most
recently sent bytes becomes `pcSide` (`src/USBrepeater.cpp`, Section 6), and
the downstream device's replies are routed back only to it — the same
pattern `commandProcessor::processStreams()` already uses internally
(`serial = streams[i];`). This resolves *who gets the reply*; it does not
prevent both sources writing to the device in the same loop pass if both
have data available at once — accepted as a known edge case under sustained
concurrent traffic from both sides, not solved with hard exclusivity. See
Section 6c's header comment for the full reasoning.

**Baud rate**: `Serial.baud()` auto-follows the PC's chosen baud for the USB
side; there's no TCP equivalent, so `GBAUD`/`SBAUD` exist for a pure-Ethernet
client to set it explicitly.

**Configuration**: `?ETHDHCP` (DHCP vs. static), `SETHIP`/`SETHMASK`/`SETHGW`
(static IP config, dotted-quad), and `?ETHCTRLPORT`/`?ETHDATAPORT` all take
effect on the **next boot** (`SAVE` first to persist), not live — same
convention as other reboot-required settings in this codebase. `GETHSTAT`
always reflects live state (link up/down, DHCP vs. static, current IP,
whether each TCP port has a connected client) regardless of pending config
changes.

**Hardware status (verified 2026-10-05)**: tested on real hardware with the
magjack installed — `GETHSTAT` reported `LINKUP,DHCP,192.168.68.61,FALSE,FALSE`
after boot, confirming link detection, DHCP, and `Ethernet.begin()` all work
as implemented, with no crash anywhere in setup() (GAACE_Script's preload and
EthernetSetup() both survived first contact with real silicon). `GVER`/
`GLINK`/`GBAUD`/`GSCRIPTST,0` all round-tripped correctly over `SerialUSB1`
in the same session. **Not yet verified**: a TCP client actually reaching
`EthCtrlPort`/`EthDataPort` (blocked by the test machine's own LAN routing,
not a device issue — see TODO.md) and passthrough to a real downstream
device (none was connected during this test).

**Licensing**: QNEthernet is AGPL-3.0-or-later, unlike `GAACE_Core` (GPLv3)
or `ArduinoThread` (Public Domain) — AGPL carries network-use
source-disclosure obligations GPL doesn't. Worth a deliberate decision
before shipping a product built on this, not just carrying the dependency
forward by default.

The USB Manufacturer string (`GAA Custom Electronics, LLC`) and Product string
(`USBrepeater`) are shared by both ports — see
`board_vendor` / `board_build.usb_product` in `platformio.ini` and
[scripts/usb_name.py](scripts/usb_name.py) (Teensyduino's `USB_DUAL_SERIAL`
descriptor has one Manufacturer/Product string for the whole composite device,
not one per port; the script applies the values to the Teensy core at build time).

## Architecture

### Modes

The firmware runs one active mode at a time (`include/USBrepeater.h`'s
`SystemMode` enum + `modeTable[]` in `src/USBrepeater.cpp`). Today there is
one mode:

- **REPEATER** (index 0, default) — the passthrough behavior described above.

Adding a mode: append to the `SystemMode` enum, write a
`<Mode>Setup()`/`<Mode>Loop()` pair, add a row to `modeTable[]`. See
[TODO.md](TODO.md) for planned additions.

### Threading

A `ThreadController` runs two threads today, independent of the active mode:

- **Housekeeping** — 250 ms placeholder for future low-frequency work (link
  supervision, status polling, etc.). Currently a no-op.
- **ADCUpdate** — the periodic ADC-driven downstream command feature (below).

`threadCommands` (GAACE_Core, enabled via `-D GAACE_THREAD_CMDS`) exposes the
thread pool over the command processor: `TLIST`, `?TENA`, `?TINT`, `TTRIG`,
`TDELAY`, `TSTOPALL`, `TSTARTALL`, `TREM`.

### Periodic ADC -> downstream-device command update

`ADCThread` reads an analog pin on its own configurable interval, applies a
linear scale, and sends the result to the downstream device as a normal GAACE
command — independent of the active mode, so it keeps running during REPEATER
mode:

```
counts      = readAdcCounts(AdcPin)   // 12-bit, hardware + software averaged
scaledValue = AdcScaleM * counts + AdcScaleB
downstream device <- "<AdcCmdName>,<scaledValue>\n"
```

`readAdcCounts()` (`src/USBrepeater.cpp`, Section 5) is the single point of
truth for reading the ADC: 12-bit resolution and 32x hardware oversampling
(`analogReadResolution()`/`analogReadAveraging()`, configured once in
`setup()`), plus a 16-sample software average on top — all three tunable
via `#define`s (`ADC_RESOLUTION_BITS`/`ADC_HW_AVERAGING`/`ADC_SW_SAMPLES`).
Every reader — `ADCUpdate()`, `GADC`, and the script `read_adc()` syscall —
goes through it, so there's exactly one averaging behavior to reason about.

**Upgrading from an older build**: earlier firmware read the ADC at
Teensy's default 10-bit resolution (0–1023 counts) with no averaging at
all. The same physical input now reads roughly **4x higher** in raw counts
at 12-bit resolution (0–4095) — any `AdcScaleM`/`AdcScaleB` calibrated and
saved under that older firmware is 4x off and needs recalibrating, not
just reusing as-is.

Because this writes to the same link the repeater pumps PC<->device traffic
over, two guards keep it from colliding with ordinary passthrough traffic
(see Section 6a in `src/USBrepeater.cpp` for the full explanation):

- **Quiet guard** — skips a cycle if the PC forwarded bytes to the device in
  the last 5 ms.
- **Swallow window** — discards device-\>PC bytes for 100 ms after injecting,
  so the downstream device's reply to the injected command doesn't show up as
  unsolicited bytes in the PC application's stream.

Both are heuristics sized for short ASCII command/reply traffic, not a real
bus-arbitration protocol — widen them if your downstream device replies
slowly.

### GAACE_Script ADC demo

[GAACE_Script](https://github.com/GordonAnderson/GAACE_Script) — a minimal
bytecode VM (plus a standard N-slot runtime, `GAACEScript::ScriptRuntime`)
built alongside this project for host-downloadable control scripts (see
[TODO.md](TODO.md) for the design discussion) — runs a real example in
slot 0 ("Script0") of `scripts`, alongside `ADCThread`. It does the same
conceptual job as the ADC feature above (read the ADC, send a scaled value
downstream), but the decision of *when* to send lives in
[gaace_scripts/adc_demo.gs](gaace_scripts/adc_demo.gs) instead of being
hardcoded in C++: it only sends when the reading has moved more than 20 raw
counts since the last send (20, not the original 5, because the ADC read
is now 12-bit instead of 10-bit — see above), using a script-level variable
that persists across ticks.

This is a demonstration running in parallel, not a replacement, so
`Script0` starts **disabled**. Enable it with `STENA,Script0,TRUE` — but
not at the same time as `ADCEN,TRUE` against the same downstream command,
which would double-send. It reuses the same `AdcPin`/`AdcScaleM`/
`AdcScaleB`/`AdcCmdName` settings and the same guard/swallow state as
`ADCUpdate()`. Change its rate with `STINT,Script0,<mS>` — starting,
stopping, and rate all come from GAACE_Core's `threadCommands`
(`?TENA`/`?TINT`), not from anything specific to this feature; see
[GAACE_Script's README](https://github.com/GordonAnderson/GAACE_Script)
for why.

Editing the script requires recompiling it (see
[gaace_scripts/adc_demo.gs](gaace_scripts/adc_demo.gs)'s header comment for
the exact command) and committing the regenerated
[include/ScriptBytecode.h](include/ScriptBytecode.h) — there's no build-time
codegen step yet.

### Persistence

Settings live in the `Data` struct (`include/USBrepeater.h`), saved to and
restored from Teensy 4.1's emulated EEPROM (survives firmware uploads) via
`SAVE`/`RESTORE`. A `Signature` field detects uninitialized/invalid storage
and falls back to defaults; it's bumped whenever the struct's layout changes.

## Command reference

All commands are sent as ASCII lines on `SerialUSB1`, 115200 baud:
`CMD,arg1,arg2\n`. A `?CMD` table entry means `GCMD` reads, `SCMD,<value>`
writes. Responses are ACK (`0x06`) or NAK (`0x15?`); built-in `GCMDS` lists
every registered command and its help string, `HELP,<cmd>` looks up one.

| Command | Description |
| --- | --- |
| `GVER` | Firmware version |
| `GNAME` / `SNAME,<str>` | Device name |
| `SAVE` | Save settings to EEPROM |
| `RESTORE` | Restore settings from EEPROM |
| `GMODE` | Active mode, index and name |
| `SMODE,<index>` | Switch active mode (not persisted until `SAVE`) |
| `GLINK` | Downstream USB device connection state (`TRUE`/`FALSE`) |
| `GBAUD` / `SBAUD,<baud>` | Baud rate to the downstream device; set explicitly for Ethernet-only use |
| `GETHDHCP` / `SETHDHCP,TRUE\|FALSE` | DHCP vs. static IP; reboot to apply |
| `GETHIP` / `SETHIP,<a.b.c.d>` | Static IP address; reboot to apply |
| `GETHMASK` / `SETHMASK,<a.b.c.d>` | Static subnet mask; reboot to apply |
| `GETHGW` / `SETHGW,<a.b.c.d>` | Static gateway; reboot to apply |
| `GETHCTRLPORT` / `SETHCTRLPORT,<port>` | TCP port for the network control port; reboot to apply |
| `GETHDATAPORT` / `SETHDATAPORT,<port>` | TCP port for network passthrough; reboot to apply |
| `GETHSTAT` | Live: `link,mode,ip,ctrlConnected,dataConnected` |
| `GADCEN` / `SADCEN,TRUE\|FALSE` | ADC-update master enable |
| `GADCPIN` / `SADCPIN,<pin>` | ADC input pin |
| `GADCINT` / `SADCINT,<mS>` | ADC update interval |
| `GADCCMD` / `SADCCMD,<name>` | Command name sent to the downstream device |
| `GADCM` / `SADCM,<m>` | ADC scale slope |
| `GADCB` / `SADCB,<b>` | ADC scale offset |
| `GADC` | Read ADC now: averaged counts (2 decimals) + scaled value (4 decimals), no send |
| `SCRIPTLOAD,<slot>,<hex>` | Load a compiled script (`gsc.py --format hex`) into a `GAACE_Script` slot |
| `GSCRIPTLIMITS` | `slots,maxCodeLen,stackSize,varSlots,maxSyscalls` for the script runtime |
| `GSCRIPTST,<slot>` | `loaded(0\|1),lastStatus` for one script slot |
| `GCMDS` | List all commands and help strings |
| `HELP,<cmd>` | Help for one command |
| `TLIST` / `?TENA` / `?TINT` / `TTRIG` / `TDELAY` / `TSTOPALL` / `TSTARTALL` / `TREM` | Thread introspection/control (see GAACE_Core's `threadCommands`) — also how the ADC demo's `Script0` is started/stopped/re-timed: `?TENA,Script0,...` / `?TINT,Script0,...` |

## Building

```
pio run                # build
pio run -t upload      # build and flash (upload_protocol = teensy-cli)
```

If `pio run -t upload` can't find the board automatically (times out looking
for a running app to signal, or reports a write error mid-programming), a
physical double-tap of the Teensy's reset button to force HalfKay bootloader
mode before running the command works reliably — this was needed during
initial hardware bring-up on at least one dev machine/cable combination.

Dependencies (`GAACE_Core`, `ArduinoThread`, `GAACE_Script`, `QNEthernet`) are fetched automatically by
PlatformIO from their GitHub repos per `platformio.ini`'s `lib_deps` — no
manual vendoring required.

## Planned work

Single-device Ethernet is implemented (above). See [TODO.md](TODO.md) for
the remaining open work: real-hardware network validation, and the settled
design for expanding to multiple downstream devices via a USB hub (which
also generalizes Ethernet to one TCP port per device).
