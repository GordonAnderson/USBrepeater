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
- [x] Standard command set for load/start/stop/rate/limits, and whether N
      and per-VM limits should be per-project defines — all decided and
      implemented in `GAACE_Script`'s `ScriptRuntime` (see above): `N`
      (`GAACE_SCRIPT_SLOTS`), max script size, stack/var/syscall counts are
      all `#define`s with defaults; start/stop/rate reuse `threadCommands`
      rather than adding new commands
- [ ] Script *persistence* across reboot — still RAM-only. The
      filesystem-based save/load/load-on-boot idea (for platforms that have
      one, e.g. Teensy 4.1's LittleFS) is still just an idea, not built
- [ ] A general "device just rebooted" notification — raised during design
      discussion as a real gap (a host has no way to know a script's state
      was lost to an unexpected reset), but it's a GAACE_Core-level concern
      independent of scripting specifically, not started
- [ ] License for the new `GAACE_Script` repo — placeholder `MIT` in
      `library.json` for now; existing GAACE libraries aren't consistent
      (`GAACE_Core` is GPLv3, `ArduinoThread` is Public Domain), so this
      needs an actual decision, not just carrying the placeholder forward
- [ ] Real-hardware validation — the ADC demo (Section 6b) only builds
      cleanly so far; no downstream device was attached during this work,
      so the threshold/persistence logic is only verified by dry-running
      the compiled bytecode through the VM against simulated readings, not
      against a real downstream device
