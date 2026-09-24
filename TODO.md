# TODO

Design notes and open decisions for planned USBrepeater features. Nothing
here is implemented yet — captured from design discussion so it can be picked
up later without re-deriving the reasoning.

## Ethernet support

- [ ] Add Ethernet hardware/library support
- [ ] Decide: Ethernet as a *second* control-plane stream alongside
      `SerialUSB1`, or a *replacement* for it?
- [ ] Decide: raw TCP command socket, or something richer (e.g. a small web
      page)?
- [ ] If passthrough-over-Ethernet is wanted too, decide the source
      arbitration model (see below)

**Hardware**: Teensy 4.1 has a built-in Ethernet MAC (i.MXRT1062 ENET) and
onboard PHY — no shield needed, just the RJ45 magjack module (sold separately
by PJRC) soldered onto the board's bottom-side pads.

**Library**: [QNEthernet](https://github.com/ssilverman/QNEthernet)
(recommended — modern, actively maintained, lwIP-based) or
[NativeEthernet](https://github.com/vjmuzik/NativeEthernet) (classic Arduino
`Ethernet` API).

**Why it fits the existing framework**:
- `EthernetClient`/`EthernetServer` implement the Arduino `Stream` interface,
  same as `Serial`/`SerialUSB1` — a TCP connection can be handed to
  `cp.registerStream()` exactly like the control port is today. `GVER`,
  `GMODE`, `SADCxxx`, etc. would work verbatim over the network with no
  command-processor changes.
- GAACE_Core / Template's own README already anticipates modules talking to
  the MIPS host "over USB serial and optional Ethernet" — this would extend
  USBrepeater to match a pattern the rest of the GAACE ecosystem assumes.
- DHCP lease maintenance / link-state polling is a natural fit for another
  `Thread` on the existing `ThreadController` — no new scheduling mechanism
  needed.
- IP configuration (static vs. DHCP, address) would be a few more fields on
  `Data`, persisted through the existing `SAVE`/`RESTORE` mechanism.
- Flash/RAM headroom is not a concern (current build: ~86 KB / ~8 MB flash,
  ~135 KB / ~512 KB RAM).

### Ethernet as an Ethernet-to-serial converter (feeding the passthrough)

Could generalize `RepeaterModeLoop()`'s hardcoded `Serial` reference into a
selectable `Stream *pcSide` — whichever endpoint currently counts as "the PC"
(USB `Serial`, or a connected `EthernetClient` session) — while the actual
byte-pump logic between that pointer and `userial` stays as-is. Small
refactor, not a rewrite.

**Core constraint**: only one "PC" can safely drive the downstream device at
a time. Unlike the ADC-injection feature (occasional, short, framed
commands, protectable with guard/swallow windows), full passthrough is
continuous, unframed binary traffic — two live sources writing to `userial`
concurrently will interleave and corrupt the downstream device's input for
both. Every commercial serial-to-Ethernet device server enforces single-
session access for this exact reason.

**Reply routing — "last speaker wins"** (decided direction): mirrors
`commandProcessor::processStreams()`'s existing pattern
(`serial = streams[i]; // route replies back to the stream that sent the command`).
Keep a `Stream *pcSide` pointer, updated to whichever source's bytes were
most recently forwarded to `userial`. Downstream device -> PC bytes are
written only to `pcSide`. This fully solves *who gets the reply*.

**What it does NOT solve**: simultaneous writes. If USB and Ethernet both
have bytes available in the same loop iteration, both still get forwarded to
`userial` that pass and can interleave on the wire. This is only a real risk
under sustained concurrent traffic from both sides (not "one connected while
the other is idle"), so it's being accepted as a known edge case for now
rather than solved with hard exclusivity/locking. Revisit if it turns out to
matter in practice — a possible follow-up is a short "ownership" window after
a write, similar in spirit to the ADC swallow-guard.

**Baud rate wrinkle**: the current code follows the PC's chosen baud via
`Serial.baud()` to reconfigure `userial`. TCP has no equivalent signal. Needs
either a fixed configured baud for `userial` when Ethernet is the active
source, or an explicit `SBAUD` command over the control port. (RFC 2217
"Telnet COM Port Control" is the standard solution commercial converters use,
but is almost certainly more than needed here.)

Does not affect `ADCThread` either way — that logic is independent of which
source is driving passthrough.

## Multiple downstream devices via a USB hub

- [ ] Decide the upstream-to-downstream topology model (see options below)
- [ ] Add one `USBSerial_BigBuffer` object per downstream device to be
      supported
- [ ] Confirm power budget / recommend a powered hub in hardware docs

**Hardware**: a *powered/self-powered* hub is strongly recommended — the
Teensy 4.1 host port's current budget from its own regulator can easily be
exceeded by several downstream USB-serial devices.

**Software starting point**: `USBHub hub1(myusb);` is already declared in
`RepeaterModeSetup()`'s scope (carried over from the original single-device
repeater), so hub *recognition* already works at the USBHost_t36 level.
What's missing is device binding: USBHost_t36's model is "one driver object
claims one matching device," so only one `USBSerial_BigBuffer` today means
only one downstream device actually gets serviced even with a hub connected
— anything else enumerates at the USB level but has nothing in the sketch to
claim it. Supporting N devices means declaring N `USBSerial_BigBuffer`
objects (`userial1, userial2, ... userialN`).

**Open design question — topology model** (blocks implementation until
decided):

1. **N independent tunnels** — each downstream device gets its own upstream
   channel (e.g. Triple/Quad Serial USB type, or a TCP port per device once
   Ethernet exists). Full 1:1 passthrough per device, N of them in parallel.
2. **One upstream channel, addressed/selected** — a single PC-facing
   connection with a command (e.g. `SDEV,<n>`) to select which downstream
   device is currently patched through. Only one active at a time, switched
   on demand.
3. **Broadcast/fan-out** — the same upstream traffic mirrored to all N
   downstream devices simultaneously. Only makes sense if the devices are
   meant to receive identical commands.

Which one applies depends on what the downstream devices are and how the
host application needs to address them — needs a decision before this is
buildable.
