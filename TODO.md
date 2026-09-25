# TODO

Design notes and open decisions for planned USBrepeater features. The
architecture below (4-device hub + Ethernet) is a **settled design as of
2026-09-25** — captured from design discussion so implementation can start
without re-deriving the reasoning. Not implemented yet: blocked on sourcing
the hardware (4-port hub, Ethernet magjack module). The current single-device
build works as-is and remains usable for testing in the meantime.

## Settled design: 4 downstream devices via hub + Ethernet

**Goal**: support 4 downstream USB devices connected through a hub on the
Teensy's host port, reachable from the PC over both USB and Ethernet, with
enough spare Teensy headroom to eventually run control algorithms against all
4 devices — not just the current single-device ADC-injection feature.

### Device (downstream) side

- All 4 downstream devices stay connected continuously via a hub on the
  Teensy's USB **host** port — one `USBSerial_BigBuffer` object per device
  (`userial1..userial4`), each claimed and alive all the time, regardless of
  which device currently has interactive focus.
- Each device gets its own application-level **ring buffer** capturing
  incoming bytes at all times, including while not the currently-selected
  device. This is needed because a device can keep sending unsolicited bytes
  (status chatter, replies) even when nothing is actively reading it right
  now; without a buffer those bytes are lost or (worse) show up stale/mixed
  in once the device is reselected. Size generously — Teensy 4.1 has RAM to
  spare (~135 KB / 512 KB used today).
- Powered/self-powered hub strongly recommended — the Teensy's own host-port
  power budget can't reliably supply 4 downstream USB-serial devices.

### Host (PC) side — USB

- **Hardware ceiling: 3 total CDC ports.** Checked `usb_desc.h` in the
  installed Teensyduino core (`framework-arduinoteensy/cores/teensy4`): the
  i.MXRT1062's USB device controller gives Teensyduino 7 usable endpoint
  numbers beyond EP0, and each independent CDC-ACM port costs 2 (one ACM
  notification endpoint, one bulk data endpoint). `USB_TRIPLE_SERIAL` (3
  ports) uses 6 of 7 endpoints — one port short of what a 4th would need.
  There is no `USB_QUAD_SERIAL`; this is a real hardware limit, not a missing
  Tools-menu option. More physical CDC ports than 3 would require an add-on
  USB-to-serial bridge chip (separate BOM item), not attempted here.
- Decided allocation: **1 control port + 1 switchable data port.** Since
  host-side access to the 4 devices is sequential, not simultaneous, one
  data port that can be pointed at any of the 4 devices is sufficient.
  (Budget headroom exists for a 2nd data port later if simultaneous
  dual-device USB access is ever needed — 3-port ceiling allows control + 2
  data.)
- Device selection: a control-port command (e.g. `SDEV,<n>`) picks which of
  `userial1..userial4` the switchable data port is currently bridged to.
  Selecting a different device does not disconnect/reconnect the downstream
  USB device — it's a pointer change in firmware, same pattern as the
  existing `pcSide`/reply-routing logic.
- Persistence: active-device selection has a power-up default, stored in
  `Data` and saved/restored the same way as everything else today (`SAVE` /
  `RESTORE`).

### Host (PC) side — Ethernet

- One IP address, one TCP port per downstream device (4 total), always
  mapped 1:1 — no switching needed since Ethernet isn't endpoint-limited the
  way USB is. Control port is also reachable as its own TCP port.
- **Hardware**: Teensy 4.1 has a built-in Ethernet MAC (i.MXRT1062 ENET) and
  onboard PHY — no shield needed, just the RJ45 magjack module (sold
  separately by PJRC) soldered onto the board's bottom-side pads.
- **Library**: [QNEthernet](https://github.com/ssilverman/QNEthernet)
  (recommended — modern, actively maintained, lwIP-based) or
  [NativeEthernet](https://github.com/vjmuzik/NativeEthernet) (classic
  Arduino `Ethernet` API).
- `EthernetClient`/`EthernetServer` implement the Arduino `Stream` interface,
  same as `Serial`/`SerialUSB1` — each TCP connection can be handed to
  `cp.registerStream()` exactly like the control port is today, and to the
  same byte-pump logic used for USB passthrough. `GVER`, `GMODE`, `SADCxxx`,
  etc. work verbatim over the network with no command-processor changes.
- DHCP lease maintenance / link-state polling is a natural fit for another
  `Thread` on the existing `ThreadController` — no new scheduling mechanism
  needed. IP configuration (static vs. DHCP, address) is a few more fields
  on `Data`, persisted through the existing `SAVE`/`RESTORE` mechanism.

### Reply routing / arbitration

- Reuses the already-decided "last speaker wins" pattern (mirrors
  `commandProcessor::processStreams()`'s existing
  `serial = streams[i]; // route replies back to the stream that sent the command`)
  — a per-device `Stream *` pointer updated to whichever source (the
  switchable USB data port, or that device's dedicated Ethernet TCP port)
  most recently forwarded bytes to it. This only matters when both the USB
  data port and a device's Ethernet port are used against the *same* device
  concurrently; each device's Ethernet port is otherwise independent of the
  others.
- Same caveat as before: this solves who gets the reply, not simultaneous
  writes. Sustained concurrent traffic from both USB and Ethernet to the
  same device in the same loop iteration can still interleave on the wire.
  Accepted as a known edge case, as it was for the single-device design.
- Baud rate wrinkle also carries over per-device: `Serial.baud()` follows
  the PC's chosen baud for USB; TCP has no equivalent, so Ethernet-driven
  sessions need either a fixed configured baud per device or an explicit
  `SBAUD,<n>,<baud>` command.

### Background/control features (decoupled from interactive selection)

- The existing `ADCThread` feature (periodic ADC read -> command sent to a
  downstream device) must gain its own target-device setting (e.g.
  `SADCDEV,<n>` / `GADCDEV`), **independent of** whichever device currently
  has interactive focus via `SDEV`. Confirmed direction: independent, not
  coupled — the Teensy has plenty of spare headroom, and coupling it to the
  interactive selection would break background monitoring every time the
  operator switches devices.
- This generalizes: any future background feature (closed-loop control
  algorithms, per-device monitoring, etc.) follows the same pattern — its
  own configurable target device(s), running on the existing
  `ThreadController`, independent of interactive USB/Ethernet selection.
  This is the actual long-term motivation for the 4-device architecture —
  not just multiplexed passthrough, but a platform for running control
  algorithms against all 4 devices from the Teensy itself.

### Open implementation details (not yet decided)

- [ ] Exact ring buffer size per device
- [ ] Exact command names/args (`SDEV`, `SADCDEV`, `SBAUD`, Ethernet IP config
      commands)
- [ ] `Data` struct layout additions (active device, per-device baud, IP
      config, ADC target device) and the resulting `Signature` bump
- [ ] Whether the 2nd USB data-port headroom is worth using now or left for
      later
- [ ] Confirm power budget / recommend a specific powered hub part in
      hardware docs once hardware is sourced

## Future option (not committed): uLisp scripting layer

**Not part of the settled design above.** Captured here because it came up
in design discussion and the research is worth keeping, but it is a later
phase, contingent on the 4-device hub/Ethernet work above landing first —
not something to build alongside it.

**Motivation**: today's `ADCThread` is a single fixed pattern (read ADC ->
scale -> send one command to one device). With 4 persistently-connected
devices and per-device ring buffers already in place, the Teensy has real
spare headroom to run actual host-downloaded control logic against multiple
devices — conditionals, loops, math — instead of only ever the one hardcoded
ADC-injection behavior. A small Lisp interpreter is one way to get that
without shipping raw native machine code to the device (see below for why
raw code download was rejected: no MPU/OS process isolation, so malformed
downloaded machine code can corrupt or hang the whole device).

**Library**: [uLisp](http://www.ulisp.com/), specifically
[technoblogy/ulisp-arm](https://github.com/technoblogy/ulisp-arm) — its
README explicitly lists Teensy 4.0/4.1 as a supported board, confirmed by
downloading and inspecting `ulisp-arm.ino` directly.

**RAM finding (checked against the actual source, not assumed)**: for
`ARDUINO_TEENSY40`/`ARDUINO_TEENSY41`, uLisp defaults to
`WORKSPACESIZE 60000` objects at 8 bytes each — a ~469 KB heap (`Workspace[]`
array) — placed in the same default RAM bank as everything else on this
board (the `MEMBANK` override to `DMAMEM` that other boards in the same file
use is not enabled for Teensy 4.x). Added to this project's current ~135 KB
usage in that same bank, the stock configuration would overflow it before
counting ring buffers, Ethernet, or anything else. Two independent fixes,
either or both:
- Shrink `WORKSPACESIZE` — it's a compile-time `#define`; a control-script
  use case (not general-purpose Lisp programs) likely only needs a few
  thousand objects (e.g. 4,000-8,000 objects = 32-64 KB).
- Relocate `Workspace[]` into Teensy 4.1's second, mostly-unused 512 KB RAM
  bank via `#define MEMBANK DMAMEM` — the same pattern the file already uses
  for other boards, just not wired up for Teensy 4.x by default.

**Extending it with GAACE/device-specific functions**: uLisp's dispatch is a
`tbl_entry_t *tables[] = {lookup_table, NULL};` — the built-in functions are
`tables[0]`; the `NULL` second slot exists specifically so a project can
plug in its own function table without editing uLisp's own source. Each
custom function needs: a C function
`object *fn_name(object *args, object *env)` that unpacks Lisp args and
calls into existing GAACE code (e.g. send a command to `userialN`, read a
device's ring buffer, read the ADC), a name string, a doc string, and one
`tbl_entry_t` row referencing them (name, function pointer, packed min/max
arg-count byte, doc pointer) — e.g. a `(send-dev n cmd)` function exposed to
host-downloaded scripts. New primitives are added by appending rows to this
project's own table, never touching uLisp's core file.

**Rough effort estimate** (from a standing start, after the 4-device
hub/Ethernet design above is already implemented and working — not a
guarantee, just a planning-level order of magnitude as of 2026-09-25):
- Bare integration (vendor `ulisp-arm`, fix `WORKSPACESIZE`/`MEMBANK`,
  confirm it builds and a basic script runs): ~1 day.
- A useful set of GAACE-specific primitives (send/read per device, read ADC,
  persistence hooks) plus a control-port command to load/store/run a script:
  ~3-5 days.
- Hardening/testing against real hardware and the existing timing-sensitive
  guards (quiet guard / swallow window, cooperative `ThreadController`
  scheduling): hard to size without the hardware in hand; budget at least a
  few more days once the hub/Ethernet hardware exists to test against.
- Total: roughly **1-2 weeks** of focused firmware work for a solid first
  version, on top of (not overlapping) the settled 4-device design above.
