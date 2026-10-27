# TODO

Design notes and open decisions for planned USBrepeater features. The
architecture below (4-device hub + Ethernet) is a **settled design as of
2026-09-25** — captured from design discussion so implementation can start
without re-deriving the reasoning.

**Status update (2026-10-05)**: the Ethernet *half* of this design is now
implemented for the current single downstream device — see
`src/USBrepeater.cpp` Section 6c and the README's Ethernet section. That
covers: QNEthernet bring-up (DHCP or static), a control-port TCP server
mirroring `SerialUSB1`, a data-port TCP server as a second passthrough
source, "last speaker wins" reply routing shared with `Serial` via `pcSide`,
and `GBAUD`/`SBAUD` for the baud-rate gap called out below. **Not yet
implemented**: the 4-device hub expansion itself (still blocked on sourcing
the 4-port hub) and everything below that's specifically about *multiple*
devices — per-device ring buffers, `SDEV` USB switching, one TCP port per
device instead of the single pair that exists now.

**Real-hardware validation (2026-10-05)**: tested on the actual Teensy 4.1
with the magjack installed. Confirmed working: link detection, DHCP lease
acquisition (`GETHSTAT` -> `LINKUP,DHCP,192.168.68.61,FALSE,FALSE`), and —
just as importantly — `setup()` completing without a crash on first contact
with real silicon (this was the real risk; `EthernetSetup()` and
`GAACE_Script`'s slot-0 preload had only ever been build-verified before
this). `GVER`/`GLINK`/`GBAUD`/`GSCRIPTST,0` all round-tripped correctly over
`SerialUSB1` in the same session.

Found one real, unrelated hardware/workflow issue along the way:
`pio run -t upload` intermittently failed to find the board automatically
(timeout, then one "error writing to Teensy" mid-programming on this dev
machine/cable) — resolved by manually double-tapping the Teensy's reset
button to force HalfKay bootloader mode before uploading, which then
succeeded reliably. Documented in the README's Building section. Not
investigated further (cable/hub-side, not a firmware issue).

**Full end-to-end validation (2026-10-06)**: both items above are now
resolved — a real downstream MIPS device was connected and a second machine
with an actual route to the Teensy's subnet was available. All four paths
confirmed on real hardware in one session:

- [x] USB passthrough — `GVER` sent into `Serial` was relayed to the MIPS
      device over `userial`; its reply (`Version 1.267,Sept 1,2026`) came
      back correctly.
- [x] TCP control port (`EthCtrlPort`) — `GVER` over TCP got the identical
      `commandProcessor` response as `SerialUSB1`.
- [x] TCP passthrough port (`EthDataPort`) — `GVER` over TCP was relayed to
      the same MIPS device and its reply came back over that TCP
      connection — the Ethernet-to-serial-converter use case this was
      built for, confirmed working.
- [x] Reply routing (`pcSide`, "last speaker wins") — switches correctly in
      both directions: after the Ethernet test, a follow-up USB request
      correctly got its reply back on USB, not the (by then closed)
      Ethernet connection.

Nothing left unverified from the original single-device design discussion.
Remaining open work in this file is specifically the multi-device
generalization (per-device ring buffers, `SDEV`, one TCP port per device).

## Hardware-verified development plan (2026-10-27) — USBrepeater Hub Carrier

Real hardware is designed, boards are on order. This section reviews the
actual schematic (`/Users/gordonanderson/GAACE_Local/Products/USBrepeater/KiCad/USBrepeater`,
exported and verified pin-by-pin via `kicad-cli sch export netlist`, not just
read visually), `UsbHubBoot.h`/`.cpp` (the hub boot helper, lives alongside
the schematic, not yet in this firmware repo), and
`USBrepeater_Hub_KiCad_Design_Guide_4.docx` (the design narrative) — and
replaces the speculative parts of "Settled design" below with what's
actually built. The multi-device *topology* decisions in that section
(per-device ring buffers, `SDEV`, one TCP port per device) still stand; this
section is the hardware ground truth they need to build against.

### Correction: the design guide's V1/V2 hub-strapping scheme is superseded

The docx (dated 2026-09-27) describes a two-variant plan: V1 = pure resistor
straps with the Teensy physically disconnected from the hub's SDA/SCL pins,
V2 = add population options later for real SMBus control. **The schematic
that was actually built does neither.** Verified via the exported netlist:
the Teensy drives all four hub strap pins directly as GPIO, every time, on
every board — there is no resistor-strap-only variant and no DNP population
option scheme in the real design:

| Teensy pin | Net (verified) | Hub pin (U6) |
| --- | --- | --- |
| 18 | `U4-18_A4_SDA` | 22 — SDA/SMBDATA/NON_REM1 |
| 19 | `U4-19_A5_SCL` | 24 — SCL/SMBCLK/CFG_SEL0 |
| 34 | `U4-34_RX8` | 25 — HS_IND/CFG_SEL1 |
| 35 | `U4-35_TX8` | 28 — SUSP_IND/LOCAL_PWR/NON_REM0 |
| 36 | via R12 (100k) | 26 — RESET_N (C8 0.1µF to GND at the node) |
| 32 | via R7 (4.7k) | 27 — VBUS_DET (R8 100k pull-down at the node) |

This is exactly what `UsbHubBoot.h`'s header comment and `UsbHubBoot.cpp`'s
`run()`/`attach()`/`detach()` implement — drive the straps while holding
reset, release reset, hold the strap levels past the hold time, then
tri-state the strap pins (which become the hub's `HS_IND`/`SUSP_IND` status
outputs on pins 34/35, and real SDA/SCL on 18/19 if a future build ever
wants actual SMBus). Treat `UsbHubBoot.h`/`.cpp` and the netlist as the
source of truth over the docx for anything hub-strapping-related.

**Standing constraint, not just a one-time setup step**: pins 18/19 and
34/35 are permanently committed to the hub interface on this board. Current
firmware uses neither (`Wire`/`Serial8` are never initialized anywhere in
`src/USBrepeater.cpp`), so there's no conflict today — but any future
feature must not claim `Wire` (default pins 18/19) or `Serial8` (pins
34/35). `UsbHubBoot.h`'s comment ("do not enable Serial8 or Wire before
run() returns") understates this: it's not just before `run()` returns,
it's never, for the life of this firmware, unless a real SMBus hub-control
feature is deliberately added later using those same pins for their real
function.

**Also confirmed not wired on the real board** (present in the docx's
§5.8 pin table, absent from the netlist — `unconnected-(U4-...)` entries
for every one of these): the Board-ID strap (pins 28/29) and all four
per-port `FLG` over-current flags (pins 37–40). Two consequences for
firmware: there's no way to detect hardware revision at boot (don't build
a feature assuming one exists), and over-current protection is fully
autonomous at the hub+switch level — firmware has no visibility into a
per-port fault, only `GLINK`-style "is something enumerated" after the
fact.

### Phase 0 — Hub bring-up (do this first; blocks everything else)

**Done (2026-10-27), compile-verified only — no hub board to test against
yet.**

- [x] Copied `UsbHubBoot.h`/`UsbHubBoot.cpp` verbatim from
      `/Users/gordonanderson/GAACE_Local/Products/USBrepeater/` into
      `include/`/`src/` of this repo.
- [x] Wired the exact sequence from `UsbHubBoot_example.ino` into
      `RepeaterModeSetup()` (`src/USBrepeater.cpp`, Section 6):
      `UsbHubBoot::run();` **before** `myusb.begin()`,
      `UsbHubBoot::attach();` **after**.
- [x] Confirmed no other use of pins 18/19/32/34/35/36 anywhere in
      firmware (`Wire`/`Serial8` are never initialized) — no conflict today.
- [x] Builds clean, no new warnings.
- [ ] `UsbHubBoot::Options.nonRem` is left at its default (`0`, all 4 ports
      removable/hot-pluggable) — not overridden, since that seems correct
      for 4 USB-A connectors meant for pluggable downstream devices. Still
      needs confirming against actual intent, not just left unchallenged.
- [ ] **Not yet tested on real hardware** — no hub board here yet. First
      real test is Phase 1 below, once boards arrive.

### Phase 1 — Validate the hub itself before going to 4 devices

Bring-up order from the docx's own §9 is worth following literally: with
hub bring-up (Phase 0) done, plug in **one** downstream device to any one
of the 4 ports and run **today's existing single-`userial` firmware**
unchanged. This isolates "does the hub attach and pass traffic at all"
from "does 4-device multiplexing work" — don't debug both at once.

- [ ] Pass criterion: `GLINK` reports `TRUE`, and `GVER` through the
      repeater passthrough reaches the downstream device and gets a reply —
      literally the same test already done end-to-end on 2026-10-06, just
      with the hub now in the path instead of a direct connection.

### Phase 2 — Multi-device support (4× `USBSerial_BigBuffer`)

**Implemented (2026-10-27), compile-verified only — no 4-device hardware to
test against yet.** `src/USBrepeater.cpp` Section 6 now has `userial1..4`,
one `DevPort` (ring buffer + connection state) per port, and `SDEV`/`GDEV`
(active device) plus `SADCDEV`/`GADCDEV` (independent `ADCThread`/script
target) commands. `GDEVSTAT` reports `dev,connected,bytesWaiting,
bytesDropped` per port.

**The device-identification question is resolved — by decision, not by
testing.** The `hub_port`-is-protected problem below turned out not to
matter: it's explicitly **the host's job**, not firmware's, to map port
number to device identity/function. The host switches `SDEV` through 1-4
and asks each one `GNAME`/`GVER` itself over the ordinary passthrough path,
building its own table. Firmware never tries to identify what's plugged
in — no injected identify-query, no caching, no parsing. This is simpler
than anything on the table before and sidesteps the `hub_port` limitation
entirely. (Original finding, kept for context: `Device_t` has a `hub_port`
field but it's `protected`, not reachable through `USBSerial_BigBuffer`;
the only identification it exposes publicly is `idVendor()`/`idProduct()`/
`manufacturer()`/`product()`/`serialNumber()` — moot now, but would matter
again if firmware-side identification is ever revisited.)

**Ring buffer / overflow design (decided):**
- Every port captures incoming bytes continuously into its own ring buffer
  (`DEVPORT_RINGBUF_SIZE`, `#define`, currently 2048 bytes — tunable),
  regardless of whether it's selected. Only the selected port's buffer gets
  flushed to the upstream, every tick — which naturally delivers backlog
  the moment `SDEV` switches to a port, with no special "flush on switch"
  code needed.
- Overflow policy: **drop incoming bytes once full**, preserving the order
  of what's already queued (rejected the drop-oldest/overwrite
  alternative — explicitly "this is bad no matter what," so correctness
  under pressure (preserve order) was prioritized over always having the
  freshest sample). `GDEVSTAT`'s `bytesDropped` is a monotonic since-boot
  counter, never auto-cleared (including on flush/select), so the loss is
  never silently hidden.
- The injection swallow-window (ADC/script replies that must never reach
  the host) happens at buffer-*push* time now, not flush time — a byte
  arriving during a port's swallow window is discarded before it can enter
  that port's ring buffer at all, so it can never leak out later even if
  that port becomes selected afterward.

- [ ] **Not yet tested on real hardware** — no 4-port hub/4 downstream
      devices here yet. First real test, once hardware arrives: the docx's
      own bring-up pass criterion is a good target — all four enumerate
      after power-up and after hot-plugging each individually, `GDEVSTAT`
      shows sane `bytesWaiting`/`bytesDropped` for a port left unselected
      while its device talks, and `ADCThread` keeps running throughout.
- [ ] `SDEV`'s reply-delivery-on-switch behavior (the core of the "nothing
      is lost" design) only really proves itself with a device that
      actually sends meaningful unsolicited chatter while not selected —
      worth a deliberate test for this specifically, not just "does it
      enumerate."

### Phase 3 — Analog input (AIN, SMA) hardware alignment

**Already done**, confirmed in `src/USBrepeater.cpp` (commit `8bc42ec`):
12-bit resolution, 32× hardware averaging, 16× software averaging,
one shared `readAdcCounts()` read path — exactly what the design guide's
§5.6.2 recommends. No firmware change needed for this part.

**Still needed**:
- [ ] On-hardware calibration once boards arrive: apply known voltages at
      the AIN SMA and fit `SADCM`/`SADCB` from the real board, per the
      docx's bring-up plan. Starting points (12-bit-adjusted from the
      docx's §5.6/§5.6.2 transfer functions — confirm against the actual
      populated range before trusting them):
      - ±10 V range: `SADCM,0.005348` `SADCB,-10.96`
      - 0–10 V range: `SADCM,0.002705` `SADCB,-0.496`
- [ ] **REF channel (A1/pin 15) ratiometric correction — not implemented
      at all yet.** The board has a 2.5 V precision reference
      (`ISL21070CIH325Z`, confirmed on A1 via the netlist) specifically so
      firmware can cancel gain drift from the Teensy's 3.3 V rail (which is
      both the ADC reference and the AIN summing network's offset
      reference). Per the docx's §5.6.1 formula:
      `value_corrected = (REF_cal / REF_now) × (AdcScaleM × AIN_counts + AdcScaleB)`.
      Concrete firmware work: add a `RefCal` field to `Data` (`Signature`
      bump), read A1 through the same averaged-read path as AIN, and a way
      to capture `REF_cal` at calibration time (e.g. a `CALREF` command
      that reads the current A1 average into `data.RefCal`, then `SAVE`).
      Until this exists, the channel is wired but unused — the board works
      exactly as it does today without it.
- [ ] `AinRange` (±10 V vs. 0–10 V) is a **population option, not a
      firmware feature** — confirmed by the docx itself ("gain, offset and
      sign are all absorbed by `AdcScaleM`/`AdcScaleB`, so no firmware
      change is needed"). Just make sure the calibration constants above
      match whichever range was actually populated on the boards that were
      ordered.

### Phase 4 — Digital I/O (DIO, SMA) — new feature, nothing exists yet

Confirmed via the netlist: logical pin 2 → `74AHCT1G125` tri-state driver
(`DIO_OUT`), pin 3 → that driver's `OE_N` (`DIO_OE_N`, pulled up on-board —
defaults to input/disabled at power-up, before firmware ever runs), pin 4 →
`74LVC1G17` Schmitt input (`DIO_IN`). Matches the docx's §5.8 proposal
exactly for this one connector (unlike the hub-strap and Board-ID pins,
which didn't make it into the real board as proposed).

- [ ] New `Data` fields: at minimum a persisted `DioOutputEnabled`
      (`Signature` bump).
- [ ] New commands, roughly per the docx's own suggestion: `?DIODIR`
      (`TRUE`=output enabled, drives `OE_N` low; `FALSE`=input/high-Z,
      the safe default) and `?DOUT` (drives `DIO_OUT` when direction is
      output) as bidirectional `CMDbool`s; `GDIN` (reads `DIO_IN`,
      meaningful in both directions — with output enabled it reads back
      the actual line level, so firmware can detect a shorted or contended
      output).
- [ ] Boot safety: set `DIO_OE_N` `OUTPUT`/`HIGH` (disabled) *before*
      touching `DIO_OUT` at all, so the driver can never glitch into an
      unintended output state during `setup()`.
- [ ] Not needed for V1: pin 2 is PWM-capable and pin 4 is
      interrupt/capture-capable specifically so a later feature (pulse
      output, edge-triggered ADC sampling — the docx explicitly raises "DIO
      edge triggers an ADC sample/send" as a future idea) doesn't need a
      board respin. Keep the V1 command handlers simple (static levels
      only); don't build the PWM/capture path now.

### Phase 5 — Documentation follow-up

- [ ] `USBrepeater_Hub_KiCad_Design_Guide_4.docx` should get a revision (or
      at least an errata note) reflecting the hub-strapping correction
      above, so nobody builds a future SMBus/Board-ID feature against the
      stale V1/V2 plan without checking the real schematic first. Low
      urgency — this TODO.md section is the correction of record in the
      meantime.

## Settled design: 4 downstream devices via hub + Ethernet

**Real hardware now exists for this** — see "Hardware-verified development
plan" above for the actual schematic facts (hub strapping, AIN/REF/DIO
connectors, what's and isn't wired to the Teensy) and the phased work order.
This section is the topology/architecture design; that one is the hardware
ground truth it needs to build against.

**Goal**: support 4 downstream USB devices connected through a hub on the
Teensy's host port, reachable from the PC over both USB and Ethernet, with
enough spare Teensy headroom to eventually run control algorithms against all
4 devices — not just the current single-device ADC-injection feature.

**Note**: the Ethernet mechanics described below (QNEthernet, Stream-based
`cp.registerStream()`, last-speaker-wins reply routing, the baud-rate gap)
are no longer purely prospective — they're implemented for one device today
(see the status update above). What's left here is specifically the
multi-device generalization: one TCP port *per device* instead of today's
single data port, and the USB-side `SDEV` switching since USB can't give
every device its own port the way Ethernet can.

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

## Future option (not committed): custom bytecode VM scripting layer

**Not part of the settled design above.** Captured here because it came up
in design discussion and the design is worth keeping, but it is a later
phase, contingent on the 4-device hub/Ethernet work above landing first —
not something to build alongside it.

**Scaffolded as its own library, now wired into this firmware as a real
(parallel) demo, not just an isolated example**: local sibling project
`../GAACE_Script`, pushed to
[GitHub](https://github.com/GordonAnderson/GAACE_Script). The VM core,
opcode encoding, syscall boundary, host-side compiler (`tools/gsc.py` —
variables, `if`/`else`, bounded `while`, `var` declarations for state that
persists across `vmRun()` calls), and a standardized N-slot runtime
(`GAACEScript::ScriptRuntime`, see below) are implemented and tested there.

**Standardized runtime, not a one-off** — decided this should become the
default way *any* GAACE project embeds scripting, not a per-project
reinvention each time (see design discussion: N script slots and per-VM
limits are per-project `#define`s with defaults, since resources vary
system to system; starting/stopping a script and changing its rate reuse
GAACE_Core's existing `threadCommands` rather than inventing new commands,
because each script slot is just another named `Thread`; the only
genuinely new commands are loading a script and querying limits/status).
Implemented in `GAACE_Script` as `ScriptRuntime` (constructor takes
`(&cp, &control)`, mirrors `threadCommands`'s own shape exactly):
- `SCRIPTLOAD,<slot>,<hex>` / `GSCRIPTLIMITS` / `GSCRIPTST,<slot>` — the new
  commands. Bytecode travels as hex over the existing ASCII line protocol
  (`gsc.py --format hex` already produces exactly this).
- `?TENA,Script<n>,...` / `?TINT,Script<n>,...` — start/stop/rate, free
  from `threadCommands`, since each slot is a named `Thread`
  ("Script0".."Script<N-1>") added to the project's `ThreadController`.
- `GAACE_SCRIPT_SLOTS`, `GAACE_SCRIPT_MAX_CODE_LEN`,
  `GAACE_SCRIPT_STACK_SIZE`, `GAACE_SCRIPT_VAR_SLOTS`,
  `GAACE_SCRIPT_MAX_SYSCALLS` — the per-project defines, all with defaults.

Found and fixed a real bug along the way: `commandProcessor::getValue(int*,
ll, ul)` treats `ll==0 && ul==0` as "skip the range check" by convention,
which would have silently disabled slot-index bounds checking whenever
`GAACE_SCRIPT_SLOTS==1` — a very plausible configuration on a tight board.
Both `SCRIPTLOAD` and `GSCRIPTST` validate the slot index explicitly
instead of relying on that convention.

**USBrepeater migrated onto it (Section 6b/8 in `src/USBrepeater.cpp`)**:
`gaace_scripts/adc_demo.gs` (compiled to `include/ScriptBytecode.h`) is
preloaded into slot 0 ("Script0") at boot, alongside `ADCThread`. The old
bespoke `?SCRIPTEN`/`GSCRIPTINT`/`SSCRIPTINT`/`GSCRIPTST` commands and the
hand-rolled `ScriptThread`/`scriptVm` are gone — replaced by the
standardized commands above, net removing code rather than adding it. The
three syscalls (`link_ready`, `read_adc`, `scale_send`) are unchanged: they
still drive the real downstream link, calibration fields, and guard/swallow
state `ADCUpdate()` uses. Slot 0 starts disabled (`Thread::enabled`, not a
persisted `Data` field anymore — see the open item below); enable with
`STENA,Script0,TRUE`. Not at the same time as `ADCEN,TRUE` against the same
downstream command — still double-sends.

Verified: builds successfully for `teensy41`; `GAACE_Script`'s own
`ScriptRuntime` was proven to actually link (not dead-code-eliminated) by
building its `BasicScript` example for both Teensy 4.1 and an Adafruit
Feather M0 (SAMD21) with real memory-usage deltas. The demo script's
compiled bytecode was dry-run against a simulated reading sequence through
the real VM before being embedded, confirming the threshold/persistence
logic behaves as intended. Not yet tested on real hardware — no downstream
device attached during this work.

**Motivation**: today's `ADCThread` is a single fixed pattern (read ADC ->
scale -> send one command to one device). With 4 persistently-connected
devices and per-device ring buffers already in place, the Teensy has real
spare headroom to run actual host-downloaded control logic against multiple
devices — conditionals, loops, math — instead of only ever the one hardcoded
ADC-injection behavior.

**Superseded direction: uLisp.** [uLisp](http://www.ulisp.com/)
(`technoblogy/ulisp-arm`) was researched first and does run on Teensy 4.0/4.1
(and, for what it's worth, already has ready-made support for common
SAMD21/SAMD51 Arduino boards too). But its default Teensy 4.x heap
(`WORKSPACESIZE 60000` objects x 8 bytes = ~469 KB) would overflow this
project's primary RAM bank even after shrinking it, its GC introduces
non-deterministic pauses that sit awkwardly next to this firmware's existing
timing-sensitive guards (ADC quiet guard / swallow window), and it's a much
bigger, more general piece of machinery than a fixed control-script use case
actually needs. Decided against it in favor of a minimal custom VM, sized to
exactly what's needed and small enough to be a reusable library rather than
a per-project integration exercise.

**Core design — a small stack machine, not an interpreter with a heap**:

```c
struct VM {
  int32_t stack[32];      // operand stack
  int32_t vars[16];       // local variable slots
  const uint8_t *code;    // script bytecode
  uint16_t pc, sp;
};
```

No dynamic allocation, no garbage collector — total footprint per running
script is under ~1 KB, vs. uLisp's ~469 KB default heap. This is the "not a
big footprint solution" requirement.

**Opcode set** (~20 opcodes; `IF`/`WHILE` are not opcodes — they compile down
to plain jumps, same as any real compiler's control-flow lowering):

| Category | Opcodes |
| --- | --- |
| Stack | `PUSH_I32 <i32>`, `DUP`, `POP` |
| Variables | `LOAD <slot>`, `STORE <slot>` |
| Arithmetic | `ADD SUB MUL DIV MOD NEG` |
| Compare | `EQ NE LT LE GT GE` (push 1/0) |
| Logic | `AND OR NOT` |
| Control flow | `JMP <offset>`, `JZ <offset>`, `JNZ <offset>` |
| Extension | `CALL <syscall_id> <argc>`, `HALT` |

Example lowering (confirms loop and if/then/else are covered, as required):

```
if (a > b) { send(0, X) } else { send(1, Y) }       while (cond) { body }
---------------------------------------------        --------------------
LOAD a                                                loop:
LOAD b                                                  <cond>
GT                                                      JZ  end
JZ  else                                                <body>
  <then body>                                           JMP loop
  JMP end                                              end:
else:
  <else body>
end:
```

**The syscall boundary — this is what makes it a reusable library, not a
one-off**: `CALL` invokes a function pointer from a table the *embedding
project* supplies, not anything the VM core knows about:

```c
typedef int32_t (*syscall_fn)(int32_t *args, uint8_t argc);
```

The VM core (opcode enum, `struct VM`, `vmRun()`) has zero hardware-specific
code in it — no `Serial`, no `USBHost_t36`, no Teensy headers anywhere.
USBrepeater registers its own syscall table (`read_adc`, `send_device`,
`read_ring_buffer`, `get_setting`, `set_setting`, ...); a future SAM-based
GAACE project would register a completely different table suited to its own
hardware, against the identical VM core. Ship it as its own repo
(`GAACE_Script` or similar) pulled in via `lib_deps`, exactly like
`GAACE_Core`/`ArduinoThread` are shared across projects today.

**Compiler lives on the host, not the device** — the Teensy only ever needs
the ~20-opcode interpreter loop; there's no parser on-device. A small
PC-side tool compiles a readable script into the bytecode array and sends it
over the existing control port with a new command (e.g.
`LOADPROG,<n>,<hexbytes>`) — the same ASCII-line transport already used for
everything else.

**Constraint carried over from the existing cooperative-threading design**:
`vmRun()` must return promptly each time it's invoked (called periodically
from a `Thread`, same as `ADCThread` today) — loops inside a script must be
bounded (fixed iteration counts), not "loop forever." Same no-blocking
discipline the rest of the codebase already follows for `ArduinoThread`, not
a new constraint.

### Open implementation details (not yet decided)

- [x] Full opcode encoding — settled in `GAACE_Script`: 1 opcode byte +
      little-endian operand bytes (`PUSH_I32`=5B, `LOAD`/`STORE`=2B,
      `JMP`/`JZ`/`JNZ`=3B absolute target, `CALL`=3B, everything else 1B)
- [x] Number of variable slots / stack depth — 16 vars, 32-deep operand
      stack in the current scaffold; revisit only if a real script needs
      more
- [x] Repo/library structure and naming — `GAACE_Script`, laid out like
      `GAACE_Core`/`ArduinoThread` (`library.json` + `src/` at repo root),
      plus `examples/`, `test/test_vm/` (native Unity tests), and a
      `lib/GAACE_Script_core -> ../src` symlink so PlatformIO's Library
      Dependency Finder picks up the core under every environment,
      including `native` (a self-referential `lib_deps = symlink://.`
      — the pattern `ArduinoThread`/`GAACE_Core` use for their hardware
      examples — did not reliably expose headers/sources to the `native`
      test env; the `lib/` symlink works uniformly for all environments)
- [x] Syscall table contents for USBrepeater specifically — decided and
      implemented for the ADC demo (`link_ready`, `read_adc`, `scale_send`);
      a different feature would still need its own table designed
- [x] Host-side compiler — `tools/gsc.py` in `GAACE_Script`: variables,
      `if`/`else`, bounded `while`, `var` declarations, syscall
      declarations, bin/hex/carray output, a disassembler
- [x] Push `GAACE_Script` to GitHub — done,
      [github.com/GordonAnderson/GAACE_Script](https://github.com/GordonAnderson/GAACE_Script)
- [x] Runtime host-downloadable scripts — `SCRIPTLOAD,<slot>,<hex>` now
      exists and works (RAM-only; see the still-open persistence item
      below). USBrepeater's own ADC demo still preloads slot 0 at compile
      time (`gaace_scripts/adc_demo.gs` -> `include/ScriptBytecode.h`)
      rather than exercising `SCRIPTLOAD` live, but the capability itself —
      the original "download without reflashing" motivation — is real now,
      not just proposed
- [x] **Resolved (2026-10-27): `SCRIPTLOAD`'s single-call size ceiling.**
      `SCRIPTLOAD`'s hex argument is decoded through `commandProcessor`'s
      shared 512-byte `charAllocate` scratch arena, so one call can only
      ever deliver roughly 250 decoded bytes — raising
      `GAACE_SCRIPT_MAX_CODE_LEN` alone doesn't help, since the payload
      can't arrive in one piece regardless of the destination buffer size.
      `GAACE_Script` gained `SCRIPTAPPEND,<slot>,<offset>,<hexchunk>` +
      `SCRIPTCOMMIT,<slot>,<totalLen>` to assemble a slot's buffer across
      many small calls instead, each individually under the arena's
      ceiling regardless of total script size; a partially-written buffer
      is marked not-loaded until `SCRIPTCOMMIT` succeeds. `gsc.py --upload`
      picks single-`SCRIPTLOAD` vs. chunked automatically based on size.
      This firmware's `platformio.ini` now sets
      `GAACE_SCRIPT_MAX_CODE_LEN=2048` (up from the library default 128) —
      meaningfully raisable now that it bounds what `SCRIPTAPPEND` can
      assemble, not what one command can transfer. Verified: fresh
      `GAACE_Script` pull (commit `24a5135`), full build with the new
      define, RAM delta exactly matching the expected `+7680` bytes
      (4 slots x (2048-128)). **Not yet hardware-tested** — real-hardware
      validation is on hold until the first Hub Carrier boards are
      built (expected ~1-2 weeks from 2026-10-27); a flash attempt this
      session got stuck waiting for the Teensy to enter bootloader mode
      and was abandoned rather than chased further, since hardware
      validation overall is blocked on boards anyway.
- [x] Standard command set for load/start/stop/rate/limits, and whether N
      and per-VM limits should be per-project defines — all decided and
      implemented in `GAACE_Script`'s `ScriptRuntime` (see above): `N`
      (`GAACE_SCRIPT_SLOTS`), max script size, stack/var/syscall counts are
      all `#define`s with defaults; start/stop/rate reuse `threadCommands`
      rather than adding new commands
- [ ] Script *persistence* across reboot — still RAM-only. The
      filesystem-based save/load/load-on-boot idea (for platforms that have
      one, e.g. Teensy 4.1's LittleFS) is still just an idea, not built.
      Explicitly the next piece of work after `SCRIPTAPPEND`/`SCRIPTCOMMIT`
      above (per the user, 2026-10-27) — scoped as a following step, not
      part of that round. Composes cleanly once started: saving/restoring a
      slot's buffer to/from a file is plain buffer<->file I/O with no size
      ceiling of its own, unlike the serial transfer that just got fixed
- [ ] A general "device just rebooted" notification — raised during design
      discussion as a real gap (a host has no way to know a script's state
      was lost to an unexpected reset), but it's a GAACE_Core-level concern
      independent of scripting specifically, not started
- [ ] License for the new `GAACE_Script` repo — placeholder `MIT` in
      `library.json` for now; existing GAACE libraries aren't consistent
      (`GAACE_Core` is GPLv3, `ArduinoThread` is Public Domain), so this
      needs an actual decision, not just carrying the placeholder forward
- [ ] Real-hardware validation against an actual downstream device — on
      2026-10-05, slot 0 was confirmed loaded and alive on real Teensy 4.1
      hardware (`GSCRIPTST,0` -> `TRUE,0`), which is further than the
      dry-run-only verification this item originally described. Still not
      tested with the script actually enabled and sending to a real
      downstream device — none was connected during that session — so the
      threshold/persistence logic itself remains dry-run-verified only, not
      hardware-verified.
