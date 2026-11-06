# GAACE Gateway

**Product name: GAACE Gateway.** This repository is its firmware. The
repository, source files and some internal identifiers keep the original
working name, *USBrepeater* (for example `src/USBrepeater.cpp`);
"USBrepeater" in these docs means this firmware or the GAACE Gateway
itself. The firmware identifies itself as `GAACEgateway`: `GVER` replies
`GAACEgateway, version ...` and the factory-default `GNAME` is
`GAACEgateway`. A board that already has a saved name keeps it until
`SNAME,GAACEgateway` + `SAVE` (or a settings reset).

A Teensy 4.1 USB-CDC repeater/controller built on the
[GAACE](https://github.com/GordonAnderson) (GAA Custom Electronics Core
Extensions) framework — [GAACE_Core](https://github.com/GordonAnderson/GAACE_Core)
and [ArduinoThread](https://github.com/GordonAnderson/ArduinoThread), following
the same layout as [GordonAnderson/Template](https://github.com/GordonAnderson/Template).

At its core it transparently relays a USB CDC connection from a PC through the
Teensy's USB host port to one of up to 8 downstream USB serial devices (on 4 physical connectors), exactly
like a plain USB-to-USB serial repeater with a switch on the downstream side.
On top of that it adds a GAACE command processor, a mode framework, a
cooperative thread scheduler, persistent settings, Ethernet, and a periodic
ADC-driven control feature — all running alongside the passthrough without
disturbing it.

## Hardware

- Teensy 4.1
- Either a single downstream USB CDC device wired directly to the Teensy's
  host port (5-pin header), or the **GAACE Gateway board** (originally the
  USBrepeater Hub Carrier) — a
  Microchip USB2514B 4-port hub on that same host port, driving 4 downstream
  USB-A ports plus an analog input and digital I/O (see
  [TODO.md](TODO.md)'s "Hardware-verified development plan" for the full
  schematic review)
- PC connected to the Teensy's micro-USB (device) port

## USB port layout

Built with `-D USB_TRIPLE_SERIAL`, so the Teensy enumerates as **three** USB
CDC ports — the hardware's full endpoint budget (see "Downstream ports"
below for why there isn't room for a 4th):

| Port | Purpose |
| --- | --- |
| `Serial` | Repeater byte pump #1: PC \<-\> downstream USB CDC device (`SDEV`/`GDEV` selects which one). Raw, transparent passthrough — nothing else writes to this port. |
| `SerialUSB1` | Command/control port: the GAACE `commandProcessor` lives here (ASCII line protocol, ACK `0x06` / NAK `0x15` responses). |
| `SerialUSB2` | Repeater byte pump #2: PC \<-\> a **different** downstream USB CDC device (`SDEV2`/`GDEV2`). Independent of `Serial` — a second simultaneous passthrough session, not a fallback. |

They're independent. A PC application that only needs the repeater can use
`Serial` exactly as if this were a single-port device; a separate tool
(terminal, script, MIPS host) can use `SerialUSB1` to query status or change
settings without ever touching the data stream. `SerialUSB2` is a second,
equally "plain" passthrough port — a second PC application (or a second
connection from the same one) can use it exactly like `Serial`, against
whichever downstream device `SDEV2` currently points it at.

## Ethernet

[QNEthernet](https://github.com/ssilverman/QNEthernet) drives the Teensy
4.1's built-in Ethernet MAC/PHY, adding 5 TCP ports — one control port plus
one dedicated passthrough port per downstream device:

| Port | Purpose |
| --- | --- |
| `EthCtrlPort` (default 5000) | Control port: every `SerialUSB1` command works verbatim over this TCP connection — same commandProcessor, same ASCII protocol. |
| `EthDataPortBase` (default 5001) | Device 1's dedicated passthrough port. Slots 2-8 get `EthDataPortBase`+1 through +7 (5002-5008 by default; slot N = connector N, see "Downstream ports") — always reachable regardless of which device `SDEV` currently has Serial pointed at, since Ethernet isn't endpoint-limited the way USB is. |

All 5 are single-session — a new connection on a given port takes over from
whatever was previously connected on that same port, same policy every
commercial serial-to-Ethernet device server uses. Each device's port is
independent of the others.

**Passthrough arbitration — one command at a time per device.** Several
requesters can want the same device at once: the USB data port bridged to
it, its own TCP data client, and the repeater itself (ADC and script
sends). Each device has an arbiter (`src/USBrepeater.cpp`, Section 6): each
requester's input is collected one line at a time, the arbiter sends one
line, routes the reply back to whoever sent it, and only then serves the
next requester. An exchange ends once the reply's newline has arrived and
the device has been quiet for 20 ms (so a multi-line reply isn't cut off),
or after 1 s with no complete reply (`GARBSTAT` counts these). When the
same requester sends its next command and nobody else is waiting, it goes
out straight away, so a single host app sees no extra delay. Output the
device sends on its own goes to whoever last talked to it. Lines are
forwarded when complete, not byte by byte: right for the ASCII command
traffic hosts send, not for binary transfers (none are used).

Measured on real hardware (2026-11-05): about 6 ms per command over USB
and 10 ms over TCP for a single host; with four requesters on two devices
at once, 600 exchanges with no misrouted replies and no timeouts (about
25 ms per command each).

**Baud rate**: `Serial.baud()` auto-follows the PC's chosen baud for the USB
side; there's no TCP equivalent, so `GBAUD`/`SBAUD` exist for a pure-Ethernet
client to set it explicitly.

**Configuration**: `?ETHDHCP` (DHCP vs. static), `SETHIP`/`SETHMASK`/`SETHGW`
(static IP config, dotted-quad), and `?ETHCTRLPORT`/`?ETHDATAPORT` (the
latter being the per-device *base* port — see the table above) all take
effect on the **next boot** (`SAVE` first to persist), not live — same
convention as other reboot-required settings in this codebase. `GETHSTAT`
always reflects live state (link up/down, DHCP vs. static, current IP,
whether the control port and each of the 4 per-device data ports has a
connected client) regardless of pending config changes.

**Hardware status (fully verified 2026-10-06)**: tested end-to-end on real
hardware with the magjack installed, a real downstream MIPS device attached,
and a second machine actually reachable on the Teensy's network:

- Link detection, DHCP lease acquisition, and `setup()` completing without a
  crash (confirmed 2026-10-05 — `GAACE_Script`'s preload and
  `EthernetSetup()` both survived first contact with real silicon).
- **USB passthrough**: `GVER` sent into `Serial` was relayed to the
  downstream MIPS device over `userial` and its reply (`Version
  1.267,Sept 1,2026`) came back correctly.
- **TCP control port** (`EthCtrlPort`): `GVER` over a plain TCP connection
  got the same `commandProcessor` response as `SerialUSB1` — the
  Stream-based design works exactly as intended, no special-casing needed.
- **TCP passthrough port** (`EthDataPort`): `GVER` sent over TCP was
  relayed to the same downstream device and its reply came back over that
  TCP connection, confirming the Ethernet-to-serial-converter use case this
  was built for.
- **Reply routing (`pcSide`, "last speaker wins")**: after the Ethernet
  passthrough test, a follow-up USB request correctly got its reply back on
  USB, not on the now-closed Ethernet connection — the arbitration switches
  cleanly in both directions, not just once.

Nothing left unverified from the original design discussion for a single
downstream device. **The 4-dedicated-port expansion above (2026-10-27) is
compile-verified only** — real hardware for the 4-port hub isn't built yet
(see [TODO.md](TODO.md)); the single-device Ethernet mechanics it reuses
(TCP servers, `cp.registerStream()`, "last speaker wins") are the
hardware-proven parts, generalized to 4 independent data ports rather than
rewritten.

**Licensing**: QNEthernet is AGPL-3.0-or-later, unlike `GAACE_Core` (GPLv3)
or `ArduinoThread` (Public Domain) — AGPL carries network-use
source-disclosure obligations GPL doesn't. **Decision (2026-10-27, per the
user): keep QNEthernet as-is for now** — noted here rather than acted on;
see TODO.md for the alternative considered (`NativeEthernet`, MIT, built
on an Apache-2.0 TCP/IP stack) if this needs revisiting before a closed-
source commercial release.

The USB Manufacturer string (`GAA Custom Electronics, LLC`) and Product string
(`GAACE Gateway`, what the PC shows for the device) are shared by all three
ports — see
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

### Downstream ports

4 physical downstream USB-A connectors (via the Hub Carrier's USB2514B),
feeding **8 serial-device slots** (`DEVPORT_COUNT`), each with its own
ring buffer. Nested hubs are supported (one extra `USBHub` driver per
connector), and both high-speed (512-byte) and full-speed (64-byte, e.g.
SAMD boards, FTDI/CP210x/CH340 adapters) serial devices are claimed.

**Slots are numbered by connector.** Any device can go in any connector;
the connector decides its number, regardless of boot or plug order:

| Slot | What | TCP data port |
|---|---|---|
| 1-4 | The instrument on connector 1-4 | 5001-5004 |
| 5-8 | A USB-serial bridge behind connector 1-4's instrument's own internal hub | 5005-5008 |

Slots 5-8 are for a MIPS with an internal temperature controller on a
USB-RS485 adapter (FTDI, behind the MIPS's own hub): that adapter is
always 4 slots above its MIPS. A recognised bridge chip (FTDI, CP210x,
CH340, PL2303) behind an inner hub takes slot N+4; anything else takes
slot N. A bridge-chip adapter plugged straight into a connector is the
instrument itself and gets slot N. A device that finds both of its
connector's slots taken is left unassigned and shown as slot 0 in
`GDEVPATH`. `GDEVPATH` shows each slot's connector, inner-hub port and
VID/PID.

All slots stay connected and auto-reconnecting continuously, and each one
**keeps capturing incoming
bytes into its own buffer regardless of whether it's currently
selected** — so a device that sends unsolicited chatter while not in focus
is never silently lost.

On the USB side, two ports are each bridged to one device at a time:
`SDEV,<n>`/`GDEV` for `Serial`, `SDEV2,<n>`/`GDEV2` for `SerialUSB2` — the
hardware's 3-CDC-port ceiling (control + 2 data) means two switchable USB
paths is the most available, not four.

**By default both are automatic (`SDEV,0`/`SDEV2,0`)**: data port 1 takes
the lowest occupied slot and data port 2 the next one. With two
instruments in any two connectors, the two data ports reach them with no
setup, transparent to the host software. Adding a device on a lower
connector later shifts the assignment (a device added on connector 1 takes
data port 1). `SDEV,<n>`/`SDEV2,<n>` with n = 1-8 pins a port to that
slot instead; the other port, if automatic, skips it.

**Or by name:** `SDEV,QUAD` / `SDEV2,IsoElectrometer` (an instrument's
`GNAME`, case-insensitive) pins a data port to that instrument wherever
it's plugged in, for setups where a given COM port must always reach a
given system. While it's unplugged that data port has nothing to bridge.
`SDEV,0` (automatic) or `SDEV,<n>` clears the name. `SAVE` keeps it.

`GDEV`/`GDEV2` reply `<setting>,<effective slot>`: `0,4` = automatic,
currently slot 4; `QUAD,1` = named, QUAD is in slot 1; effective 0 =
nothing to bridge. Don't switch a data port while a host app has it open:
the app would suddenly be talking to a different instrument. **The two
data ports never bridge the same device** — `SDEV`/`SDEV2` refuse (NAK) a
setting that would collide right now: the same name on both, the same
pinned slot, or a name whose instrument is in the other port's pinned
slot. If a name and a pinned slot only collide later (the instrument was
moved), data port 1 keeps it and data port 2 goes idle.

Over Ethernet, `GDEVPORT,<name>` returns an instrument's TCP data port
(e.g. `GDEVPORT,QUAD` -> `5001`), so host software can find a system by
name without knowing slots.
Switching either delivers that port's buffered backlog on the very next
tick, with no separate "flush" step — the firmware always flushes a
selected port's backlog to whoever last spoke to it, so what built up
while it wasn't selected goes out immediately once it is.

On the Ethernet side, every port is independent of `SDEV`/`SDEV2`: each of
the 8 slots has its own dedicated, always-reachable TCP data
port (see the Ethernet section above), so a client connected to device 3's
port gets device 3's traffic whether or not `SDEV`/`SDEV2` currently points
either USB port at device 3. A device reachable from both a USB data port
and its TCP port is shared through its arbiter (see "Passthrough
arbitration" above), so both can be used at the same time.

**The firmware learns each instrument's name.** When a device lands in
slot 1-4, the repeater sends it `GNAME` (every GAACE device answers with
its system name) and retries for about 8 s while a MIPS boots. The reply
is swallowed, never forwarded. While that's in progress (typically well
under a second) the slot is held out of passthrough, ADC and script sends;
host bytes simply wait. Slots 5-8 (a MIPS's RS-485 temperature-controller
adapter) are not queried, so `GNAME` never goes out onto that RS-485 bus.

```
GDEVLIST          -> #slot,connector,role,vid,pid,state,name
                     1,1,MAIN,2341,003E,NAMED,QUAD
                     2,2,MAIN,239A,801E,NAMED,IsoElectrometer
GDEVSLOT,QUAD     -> 1          (case-insensitive; NAK if no such name)
```

`state` is `NAMED`, `PENDING` (still asking), `NONAME` (NAKed `GNAME` or
never answered: not a GAACE device) or `NOTQUERIED` (slot 5-8).
`DEVIDENT,<slot>` asks again, for example after renaming an instrument
with `SNAME` through the passthrough; on a slot 5-8 it forces a query.
If two devices report the same name, `GDEVSLOT` returns the lower slot.
You can also check a data port directly by sending `GNAME` through it.

`ADCThread` and the script demo (below) target a separate, independent
selection — `SADCDEV` / `GADCDEV` — so switching which device the host
is interactively looking at never interrupts background monitoring of a
different one. The target can be a slot (`SADCDEV,3`) or, better, an
instrument's name (`SADCDEV,QUAD`, its `GNAME`): a name follows the
instrument to whatever connector it's in and sends nothing while it's
absent. `GADCDEV` replies `<setting>,<effective slot>`, e.g. `QUAD,1`
(effective 0 = not connected). `SAVE` keeps the name.

`GDEVSTAT` reports one line per port: `dev,connected,bytesWaiting,
bytesDropped`. `bytesDropped` is a monotonic since-boot counter — if a
port's buffer genuinely fills (new bytes are dropped, not oldest-overwritten,
so order is preserved for whatever's already queued), this is how you'd
know, even if nobody was watching at the time.

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
counts      = readAdcCounts(AdcPin)   // 12-bit, hardware + software averaged, then low-pass filtered
scaledValue = scaleAdcValue(counts)   // AdcScaleM * counts + AdcScaleB, then REF-corrected if RefCal is set
downstream device <- "<AdcCmdName>,<scaledValue>\n"
```

`readAdcCounts()` (`src/USBrepeater.cpp`, Section 5) is the single point of
truth for reading the ADC: 12-bit resolution and 32x hardware oversampling
(`analogReadResolution()`/`analogReadAveraging()`, configured once in
`setup()`), plus a 16-sample software average on top — all three tunable
via `#define`s (`ADC_RESOLUTION_BITS`/`ADC_HW_AVERAGING`/`ADC_SW_SAMPLES`),
then the tunable low-pass filter below. `scaleAdcValue()` is the matching
single point of truth for turning counts into a real value. Every reader —
`ADCUpdate()`, `GADC`, and the script `read_adc()`/`scale_send()` syscalls —
goes through both, so there's exactly one averaging/filtering/scaling
behavior to reason about.

**Upgrading from an older build**: earlier firmware read the ADC at
Teensy's default 10-bit resolution (0–1023 counts) with no averaging at
all. The same physical input now reads roughly **4x higher** in raw counts
at 12-bit resolution (0–4095) — any `AdcScaleM`/`AdcScaleB` calibrated and
saved under that older firmware is 4x off and needs recalibrating, not
just reusing as-is.

**AIN range** (0–10V vs ±10V) is a hardware population option, selected by
a build-time `#define` (`AIN_RANGE_UNIPOLAR`/`AIN_RANGE_BIPOLAR` in
`platformio.ini`, default `BIPOLAR`) rather than anything firmware can
detect at runtime — it only changes the factory-default `AdcScaleM`/
`AdcScaleB`, which `SADCM`/`SADCB` can always override live afterward.

**AIN low-pass filter** — a dt-corrected exponential moving average
(`applyAdcLowPass()`) applied on top of the averaging above, to smooth
noise *between* reads (which oversampling within a single read can't).
`GADCFILTER`/`SADCFILTER,<mS>` sets its time constant live, no reflash
needed; `0` disables it (today's original raw-averaged behavior). dt-
corrected rather than a fixed per-call coefficient because `GADC` can be
called on demand at any time, not just on `ADCThread`'s own schedule — the
filter's real-world time constant stays correct regardless of which
caller triggers a given read.

**REF-channel ratiometric correction** — the board's 1.25 V precision
reference (on `A1`; changed from 2.5 V in the original design) lets
firmware cancel AIN gain drift against the Teensy's 3.3 V rail. The
correction is a ratio of two readings of the same reference
(`RefCal`/`REF_now`), so the firmware doesn't depend on the reference
voltage; the 1.25 V reference reads about 1550 counts. `RefCal` must be
captured on a board with the reference it actually has: a value captured
against a 2.5 V reference is wrong for a 1.25 V board. Run `CALREF` once, with the REF input in its
known-good state, to capture the current reading into `RefCal` (`SAVE` to
persist); from then on, `scaleAdcValue()` multiplies by
`RefCal`/`REF_now` on every read. `GREFCAL` reads back the captured value;
`0` means `CALREF` has never been run and the correction stays disabled
(plain `AdcScaleM`/`AdcScaleB` passthrough). Deliberately *not* run
through the low-pass filter above — REF is a slowly-drifting calibration
factor, not a noisy signal to track quickly, so plain averaging is the
right amount of smoothing for it.

The command goes through the device's arbiter as the repeater's own
request: it waits for any host command in progress to finish, never splices
into one, and the device's reply is discarded so it never reaches a host
app. If the previous value hasn't been sent yet, the newer one replaces it.

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
`AdcScaleB`/`AdcCmdName` settings and the same arbiter path as
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

### Loading scripts at runtime (no reflash)

`SCRIPTLOAD,<slot>,<hex>` sends a compiled script over the control port
without touching firmware — but it's decoded through `commandProcessor`'s
shared 512-byte scratch arena, so one call can only ever deliver roughly
250 decoded bytes, independent of `GAACE_SCRIPT_MAX_CODE_LEN` (bumped to
2048 here to give scripts real room to grow). Anything larger goes through
`SCRIPTAPPEND,<slot>,<offset>,<hexchunk>` (one or more calls, each well
under the arena's ceiling) followed by `SCRIPTCOMMIT,<slot>,<totalLen>` to
finalize it — `tools/gsc.py --upload` in
[GAACE_Script](https://github.com/GordonAnderson/GAACE_Script) picks
single-`SCRIPTLOAD` vs. chunked automatically based on size, so this is
usually invisible day to day. See that repo's `WORKFLOW.md` for the full
walkthrough.

### `cmd()` with float arguments

Scripts can pass floats to any control-port command: `cmd("SETV", v)` with
`v` a float sends `SETV,23.76` (ints are sent as integers). This comes from
the `GAACE_Script` library, so it needs a build against its current `main`
(`lib_deps` tracks the branch head; after an update, `pio run` re-fetches
it). Scripts that use it need that firmware; int-only scripts are
unchanged. Not yet run on hardware.

### Scripts that talk to instruments by name

A script can read from and control any connected instrument by its name
(its `GNAME`), including several at once:

```
q = device("QUAD");
e = device("IsoElectrometer");
if (ready(q) && ready(e)) {
    v = dcmdf(q, "GRFADRV", 1);    // read from one instrument
    dcmd(q, "SRFADRV", 2, v);      // set another value
}
```

- A handle stands for the name, not a slot or connector: move the
  instrument to another connector and the script keeps working. `ready()`
  is 0 while it's absent.
- `dcmd()`/`dcmdf()` go through the instrument's arbiter like any host
  command: they wait their turn, never interleave with a host app using
  the same instrument, and the reply goes only to the script. The script
  pauses until the reply arrives; passthrough and everything else keep
  running.
- Results follow `cmd()`'s rules: ACK + number -> that number, a bare ACK
  (a set command that worked) -> 1, `TRUE`/`FALSE` -> 1/0; a NAK, no reply
  within 1 s, or an absent instrument -> 0. So `ok = dcmd(q, "SRFADRV", 1,
  v);` tells you whether the set worked. Use single-line commands.
- Up to 8 device commands can be in flight across all script slots.
- Measured: about 6 ms per command to a MIPS; a script doing 3 device
  commands per run keeps up with a 100 ms (10 Hz) interval.
- `GSCRIPTVAR,<slot>,<var>` shows a script's variable, for debugging
  (`gsc.py --disasm` shows which number each variable has).

These are GAACE_Script built-ins, not syscalls, so the syscall table hash
(`GSYSCALLS`) is unchanged and saved scripts still restore. The language
reference is GAACE_Script's `tools/README.md`.

### `cmd()` with bool (`TRUE`/`FALSE`) commands

Scripts can read and set bool commands (2026-11-05):

```
cmd("SADCEN", true);        // sends SADCEN,1
cmd("SDIODIR", false);      // sends SDIODIR,0
on = cmd("GADCEN");         // TRUE -> 1, FALSE -> 0
```

`true`/`false` are just the numbers `1`/`0`. The repeater's bool
commands (`SADCEN`, `SETHDHCP`, `SDIODIR`, `SDOUT`, and `STENA`) accept
`1`/`0` as well as `TRUE`/`FALSE`, from a script or from a person typing
them. (`STENA` also needs a thread name, which a script can't pass, so
it's still control-port only.) Replies are still `TRUE`/`FALSE`. A `FALSE` reply and a NAK both
read as `0` in a script. A set command that succeeds (a bare ACK) returns
`1` (since 2026-11-05; it used to read as `0`, like a NAK). Needs current `GAACE_Core` and `GAACE_Script`
`main`; a script using `true`/`false` must be compiled with the current
`gsc.py`. No bytecode or syscall-hash change, so saved scripts still
restore.

### Syscalls: one table, no declarations in scripts

The script syscalls (`link_ready`, `read_adc`, `scale_send`, `dio_set_dir`,
`dio_write`, `dio_read`) are described once, in the `scriptSyscalls[]` table
in `setup()` ([src/USBrepeater.cpp](src/USBrepeater.cpp), Section 8). A
syscall's id is its index there; **add new ones at the end** (inserting or
reordering renumbers the later ones and changes the table's hash, so scripts
compiled earlier are then refused until recompiled). `GSYSCALLS` reports the
table, so `gsc.py --upload` compiles a script against it directly — a script
just calls `v = read_adc();` with no `syscall ...;` lines. List what this
board offers with:

```
python3 ../GAACE_Script/tools/gsc.py --list-syscalls /dev/cu.usbmodemXXXX
```

Uploads of scripts that call syscalls carry the table's hash, and the board
NAKs a script compiled against a different table. `gaace_scripts/adc_demo.gs`
is the exception: its bytecode is compiled into the firmware at build time
(there's no device to ask), so it keeps explicit declarations that must match
the table's order — it works because the first three entries haven't moved.

### Script persistence across reboot

A `SCRIPTLOAD`/`SCRIPTAPPEND`-loaded script is RAM-only as far as
`GAACE_Script` itself is concerned. `SCRIPTSAVE,<slot>` writes that slot's
exact current buffer — the same `[codeLen][code][pool]` payload
`SCRIPTLOAD` uses, verbatim, no new format — to a file on a small
`LittleFS_Program` filesystem reserved in on-chip flash (`/script<N>.bin`,
one per slot). `SCRIPTRESTORE,<slot>` reloads it on demand; **every slot
is also attempted automatically at boot** (`setup()`, Section 8), so a
saved script resumes running on its own after a power cycle or reflash —
no manual re-load needed.

A script that calls syscalls is saved with the hash of the syscall table it
was compiled against (`/script<N>.sig`, 4 bytes, next to its `.bin`). On
restore — at boot or via `SCRIPTRESTORE` — the hash is compared with this
firmware's own table: if a firmware update changed the syscalls, the saved
script is **not** run (it would call different functions than it was written
for); the slot stays unloaded (at boot, slot 0 falls back to the compiled-in
demo), and `SCRIPTRESTORE` NAKs with `ERR_BADARG` rather than the
`ERR_NOSDCARD` of "nothing saved". Recompile and re-upload it. Scripts saved
before this existed, or loaded unchecked, have no `.sig` and restore as
before.

A restored script is left **enabled**, at its default 1000 mS interval
(`?TENA`/`?TINT,Script<n>` to change either) — deliberately different from
a freshly-`SCRIPTLOAD`-ed script, which always starts disabled. The point
of persistence is a script that configures itself and starts working
immediately on boot (e.g. setting up its own DIO direction, per the design
discussion that motivated this feature) without the host having to
re-enable it every time. Slot 0 falls back to the compiled-in ADC demo
(below) only if it has no saved file to restore instead — a persisted
slot 0 script takes priority over the demo.

This is implemented entirely in USBrepeater (`src/USBrepeater.cpp`,
"Script slot persistence" in Section 6b), not inside `GAACE_Script`
itself — that library stays hardware-agnostic (no Teensy-specific code in
its core), and `LittleFS_Program` is Teensy-only. `ScriptSlot::code[]` and
`VM::codeLen`/`VM::poolLen` being public fields was enough to build this
with zero changes to the library.

Hardware-tested (2026-11-05) on the bare Teensy 4.1 over the control port:
`SCRIPTSAVE`/`SCRIPTRESTORE` round-trip correctly, and a script survives a
real reflash-triggered reboot, re-restoring with `loaded=TRUE` and running
unattended with no commands sent after boot. This testing caught and fixed
a real bug — `saveScriptSlotToFlash()` had assumed a loaded slot's `code[]`
buffer always starts with the `[codeLen]` wire-format header, which isn't
true for a slot preloaded directly via `vmInit()` (the compiled-in ADC
demo); it now rebuilds the saved payload from the VM's own
`code`/`codeLen`/`pool`/`poolLen` instead of trusting that layout. See
TODO.md for the full writeup.

### Digital I/O (DIO, SMA)

One external line, independent of the 4 downstream ports — a board-level
feature like the AIN input, not per-device. Switchable between input and
output at runtime, from **both** the control port (`GDIODIR`/`SDIODIR`,
`GDOUT`/`SDOUT`, `GDIN`) and scripts (`dio_set_dir()`/`dio_write()`/
`dio_read()` syscalls) through the same underlying functions — same pattern
as the ADC's `readAdcCounts()`/`sys_read_adc()`, so the two paths can never
see it differently.

Direction is deliberately scriptable, not just control-port-settable: a
script loaded at startup can configure its own direction without depending
on the host having set it first via `SDIODIR`. Only direction is persisted
(`SAVE`/`RESTORE`); the output level itself resets to `FALSE` every boot
until the host or a script sets it.

**Hardware** (Teensy pins 2/3/4, confirmed via a `kicad-cli`-exported
netlist of the real schematic): pin 2 drives a 74AHCT1G125 tri-state
buffer's data input (only reaches the external line while enabled); pin 3
is that buffer's active-low output enable, pulled up on-board (R15) so it
defaults disabled/input with no firmware involvement at all, including the
boot window before `setup()` runs; pin 4 reads a 74LVC1G17 Schmitt buffer's
output — valid in either direction, so `GDIN` while output is enabled can
reveal a shorted or contended line. `dioInit()` (`src/USBrepeater.cpp`,
Section 6d) still explicitly drives the enable pin disabled as the very
first thing in `setup()`, before touching the output pin at all, as a
deliberate belt-and-suspenders on top of the hardware pull-up.

Compile-verified only — no DIO hardware here yet to test against (same
caveat as the rest of the Hub Carrier work).

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
| `GLINK` | `ActiveDevice`'s USB connection state (`TRUE`/`FALSE`) |
| `GLINK2` | `ActiveDevice2`'s USB connection state (`TRUE`/`FALSE`) |
| `GBAUD` / `SBAUD,<baud>` | Baud rate to the downstream device; set explicitly for Ethernet-only use |
| `GDEV` / `SDEV,<n\|name>` | Get/set what data port 1 (`Serial`) is bridged to: 0 = automatic (lowest occupied slot, the default), 1-8 = a pinned slot, or an instrument name (its `GNAME`; follows it between connectors). `GDEV` replies `<setting>,<effective slot>`. `SDEV` refuses a setting that would put both data ports on the same device |
| `GDEV2` / `SDEV2,<n\|name>` | Same for data port 2 (`SerialUSB2`); automatic = the next occupied slot after data port 1's |
| `GADCDEV` / `SADCDEV,<n\|name>` | Get/set what `ADCThread`/the script demo targets: a slot (1-8) or an instrument name (its `GNAME`; follows it between connectors). `GADCDEV` replies `<setting>,<effective slot>`, e.g. `QUAD,1` (effective 0 = not connected). Setting a slot clears the name |
| `GDEVSTAT` | One row per slot (1-8): `dev,connected,bytesWaiting,bytesDropped` |
| `GARBSTAT` | Per-slot arbiter: `slot,busy,owner,exchanges,timeouts`. `owner` is who has the open exchange (`USB1`, `USB2`, `ETH`, `INT` = the repeater's ADC sends, `SCRIPT` = a script's `dcmd()`, or `-`); `exchanges`/`timeouts` count since boot. A rising `timeouts` means the device isn't answering some commands within 1 s |
| `GDEVPATH` | Diagnostic: one row per slot (`slot,connected,connector,innerPort,vid,pid,speed,addr,hubAddr,hubPort`), then one per hub driver. `connector` is the board connector (1-4, matches the labels); `innerPort` is the port on a device's own internal hub, 0 if none. Also lists any attached device left without a slot, as slot 0 |
| `GDEVLIST` | One row per occupied slot: `slot,connector,role,vid,pid,state,name`. `role` `MAIN` (slot 1-4) or `EXTRA` (5-8); `state` `NAMED`/`PENDING`/`NONAME`/`NOTQUERIED`; `name` from the device's `GNAME`, asked on attach |
| `GDEVSLOT,<name>` | Slot (1-8) of the device whose `GNAME` is `<name>` (case-insensitive); NAK if none |
| `GDEVPORT,<name>` | TCP data port of the device whose `GNAME` is `<name>` (`EthDataPortBase` + slot - 1); NAK if none |
| `DEVIDENT,<slot>` | Ask that slot's device its `GNAME` again (also forces a query on a slot 5-8) |
| `GETHDHCP` / `SETHDHCP,TRUE\|FALSE` | DHCP vs. static IP; reboot to apply |
| `GETHIP` / `SETHIP,<a.b.c.d>` | Static IP address; reboot to apply |
| `GETHMASK` / `SETHMASK,<a.b.c.d>` | Static subnet mask; reboot to apply |
| `GETHGW` / `SETHGW,<a.b.c.d>` | Static gateway; reboot to apply |
| `GETHCTRLPORT` / `SETHCTRLPORT,<port>` | TCP port for the network control port; reboot to apply |
| `GETHDATAPORT` / `SETHDATAPORT,<port>` | TCP port for network passthrough; reboot to apply |
| `GETHSTAT` | Live: `link,mode,ip,ctrlConnected,` then one `dataConnected` flag per slot (1-8) |
| `GADCEN` / `SADCEN,TRUE\|FALSE` | ADC-update master enable |
| `GADCPIN` / `SADCPIN,<pin>` | ADC input pin |
| `GADCINT` / `SADCINT,<mS>` | ADC update interval |
| `GADCCMD` / `SADCCMD,<name>[,<fixed args>...]` | Command prefix sent to the downstream device as `<prefix>,<value>`; extra arguments are joined with commas, e.g. `SADCCMD,SRFADRV,1` sends `SRFADRV,1,<value>` (19 characters max) |
| `GADCM` / `SADCM,<m>` | ADC scale slope |
| `GADCB` / `SADCB,<b>` | ADC scale offset |
| `GADCFILTER` / `SADCFILTER,<mS>` | AIN low-pass filter time constant, mS (`0`=disabled); tunable live, no reflash |
| `CALREF` | Capture the current REF reading into `RefCal`, enabling ratiometric correction (`SAVE` to persist) |
| `GREFCAL` | Read back `RefCal`; `0` means the REF correction is disabled |
| `GADC` | Read ADC now: averaged+filtered counts (2 decimals) + scaled, REF-corrected value (4 decimals), no send |
| `GDIODIR` / `SDIODIR,TRUE\|FALSE` | Get/set the DIO line's direction: `TRUE`=output, `FALSE`=input (power-up default) |
| `GDOUT` / `SDOUT,TRUE\|FALSE` | Get/set the DIO output level; only reaches the external line while direction is output |
| `GDIN` | Read the DIO line's actual level now (valid in either direction) |
| `GSYSCALLS` | The script syscall table: `count,hash`, then `index,name,argc,int\|float` per syscall (what `gsc.py --upload` compiles against) |
| `SCRIPTLOAD,<slot>,<hex>[,<hash>]` | Load a compiled script (`gsc.py --format hex`) into a `GAACE_Script` slot in one call — capped at roughly 250 decoded bytes by `commandProcessor`'s shared 512-byte scratch arena, regardless of `GAACE_SCRIPT_MAX_CODE_LEN` |
| `SCRIPTAPPEND,<slot>,<offset>,<hexchunk>` | Write one chunk of a larger script into a slot's buffer at `offset`; marks the slot not-loaded until `SCRIPTCOMMIT` finalizes it |
| `SCRIPTCOMMIT,<slot>,<totalLen>[,<hash>]` | Finalize a buffer assembled via `SCRIPTAPPEND` calls and make the slot runnable |
| `GSCRIPTLIMITS` | `slots,maxCodeLen,stackSize,varSlots,maxSyscalls` for the script runtime |
| `GSCRIPTST,<slot>` | `loaded(0\|1),lastStatus` for one script slot |
| `SCRIPTSAVE,<slot>` | Save a loaded slot's buffer to on-chip flash (survives reboot/power loss) |
| `SCRIPTRESTORE,<slot>` | Reload a slot's buffer from flash now (also attempted automatically at boot); refused if it was compiled against a different syscall table than this firmware has |
| `SCRIPTERASE,<slot>` | Delete a slot's saved copy so it is no longer restored (and auto-enabled) at boot; the copy loaded in RAM keeps running until disabled. ACKs if nothing was saved |
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

Single-device Ethernet, hub bring-up, 4-device downstream support,
per-device Ethernet data ports, the second switchable USB data port,
digital I/O, script persistence across reboot, and the AIN range/REF-
correction/low-pass-filter work are all implemented (above) — see
[TODO.md](TODO.md) for exactly what's real-hardware-verified versus
compile-verified-only so far. As of 2026-11-05, the first Hub Carrier
board exists and most of this is now real-hardware-verified: hub bring-up,
all 4 connectors with 4 real instruments at once (5 serial devices,
including one behind a nested hub and one full-speed), both switchable
USB paths and the per-slot Ethernet data ports, with the per-device
arbiter verified under concurrent USB/Ethernet/ADC traffic, script persistence
across a real reboot, and DIO (control-port and script-syscall paths
both). Remaining open work: slots 5-8 (untested), simultaneous multi-port TCP
load, and the AIN REF-correction/filter work (needs a known calibration
signal, not yet applied).
