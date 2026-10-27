// =============================================================================
//  USB REPEATER
//  Built on the GAACE (Gordon Anderson Arduino Core Extensions) framework
// =============================================================================
//
//  PURPOSE
//  -------
//  Teensy 4.1 USB CDC repeater with a GAACE command-processor control plane
//  bolted on top, following the same layout as github.com/GordonAnderson/Template.
//
//  USB PORT LAYOUT  (Tools > USB Type: "Dual Serial" / build_flags -D USB_DUAL_SERIAL)
//  --------------------------------------------------------------------------
//    Serial      — dedicated to the repeater's raw byte pump (PC <-> downstream
//                  device). Nothing else may write to this port: the command
//                  processor would otherwise inject ACK/NAK bytes into a stream
//                  the PC application expects to be a transparent passthrough.
//    SerialUSB1  — the command/control port. The commandProcessor (GCMDS, GVER,
//                  ?NAME, GMODE/SMODE, SAVE/RESTORE, thread introspection, ...)
//                  lives here so it never collides with repeater traffic.
//
//  DOWNSTREAM PORTS  (USBrepeater Hub Carrier board — USB2514B 4-port hub)
//  -------------------------------------------------------------------------
//  4 downstream USB-A ports, all connected/auto-reconnecting continuously
//  and each capturing incoming bytes into its own ring buffer regardless of
//  selection (see Section 6). Only one is bridged to the upstream data path
//  (Serial + EthDataPort) at a time — data.ActiveDevice, set via SDEV/GDEV —
//  because the hardware doesn't have upstream USB endpoints to spare for
//  more. The host is responsible for mapping port number to device identity:
//  it switches SDEV through 1-4 and asks each one GNAME/GVER itself over the
//  ordinary passthrough path; firmware never tries to identify what's
//  plugged in. ADCThread/the script demo (below) target a second,
//  independent selection (data.AdcTargetDevice, SADCDEV/GADCDEV) so
//  background monitoring of one device isn't disrupted by the host switching
//  interactive focus to another.
//
//  MODES
//  -----
//  The system runs one active SystemMode at a time (see USBrepeater.h). Mode 0
//  is REPEATER — the behavior this file originally implemented unconditionally.
//  Future modes are added by: (1) appending to the SystemMode enum, (2) writing
//  a <Mode>Setup()/<Mode>Loop() pair, (3) adding a row to modeTable[] below.
//
//  THREADING
//  ---------
//  A ThreadController is wired up (GAACE_Core's threadCommands module exposes
//  it over the command processor via TLIST/TENA/TINT/... when built with
//  -D GAACE_THREAD_CMDS). Housekeeping is a placeholder thread for future
//  low-frequency tasks. ADCThread is real work: on its own configurable
//  interval, it reads an ADC pin, applies a linear scale (m, b), and sends
//  "<name>,<value>\n" to the downstream device to drive one of its
//  parameters — running independently of the active mode so it keeps working
//  while REPEATER mode is active. See Section 6a for the injection/swallow
//  logic and why both are needed on a shared raw byte stream.
//
//  PERSISTENCE
//  -----------
//  Data{} (see USBrepeater.h) is saved to / restored from Teensy 4.1's
//  emulated EEPROM via SAVE / RESTORE, following the same SIGNATURE-validated
//  pattern used across GAACE modules.
// =============================================================================

#include "USBrepeater.h"

// =============================================================================
//  SECTION 1 — INCLUDES
// =============================================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <USBHost_t36.h>
#include <stdio.h>   // snprintf() — used by ADCUpdate() (Section 6a)
#include <string.h>  // memcpy() — used by setup() to preload a default script
#include <math.h>    // lroundf() — used by sys_read_adc() (Section 6b)

//  UsbHubBoot — USB2514B strap/reset sequencing for the USBrepeater Hub
//  Carrier board (Section 6). Required any time this firmware runs on that
//  board: without it the hub comes up in an unknown strap configuration.
//  See TODO.md's "Hardware-verified development plan" for the pin mapping
//  this was verified against (kicad-cli-exported netlist, not just the
//  design guide, which is stale on this specific point).
#include "UsbHubBoot.h"

//  GAACE_Core — command processor, ring buffer, arena allocator, error codes.
//  Fetched by PlatformIO from https://github.com/GordonAnderson/GAACE_Core.git
//  (see platformio.ini lib_deps). All of it depends only on Arduino.h.
#include <commandProcessor.h>
#include <Errors.h>

//  ArduinoThread — cooperative scheduler used for future periodic work.
//  Fetched from https://github.com/GordonAnderson/ArduinoThread.git
#include <Thread.h>
#include <ThreadController.h>

//  threadCommands — optional GAACE_Core module that exposes the thread pool
//  over the command processor (TLIST, ?TENA, ?TINT, TTRIG, TDELAY, TSTOPALL,
//  TSTARTALL, TREM, ...). Compiles to nothing unless GAACE_THREAD_CMDS is
//  defined (see platformio.ini build_flags).
#include <threadCommands.h>

//  GAACE_Script — standard N-slot scripting runtime (Section 6b). Each slot
//  is its own named Thread added to `control` below, so ?TENA/?TINT above
//  already give start/stop/rate control per script; this module only adds
//  SCRIPTLOAD/GSCRIPTLIMITS/GSCRIPTST. Fetched from
//  https://github.com/GordonAnderson/GAACE_Script.git
#include <GAACEScriptRuntime.h>

//  QNEthernet — lwIP-based Ethernet for Teensy 4.1's built-in MAC/PHY
//  (Section 6c). EthernetClient/EthernetServer are Stream/Client subclasses,
//  same as SerialUSB1, so they plug into cp.registerStream() and
//  RepeaterModeLoop() with no command-processor changes. Fetched from
//  https://github.com/ssilverman/QNEthernet.git
//
//  LICENSING NOTE: QNEthernet is AGPL-3.0-or-later (GAACE_Core is GPLv3,
//  ArduinoThread is Public Domain) — AGPL has network-use source-disclosure
//  obligations GPL doesn't. Worth a deliberate decision before shipping a
//  product that embeds it, not just carrying it forward by default; see
//  TODO.md's existing (separate) GAACE_Script license item for the same
//  category of open question.
//
//  SYMBOL COLLISION: QNEthernet.h pulls in raw lwIP C headers (lwip/err.h)
//  at global scope, which declare their own ERR_TIMEOUT and ERR_ARG — the
//  same two names GAACE_Core already uses (Errors.h's ErrorCode::ERR_TIMEOUT
//  enum member, and commandProcessor.h's ERR_ARG #define). Neither library
//  can be edited to avoid this, so this is the standard preprocessor
//  workaround for an unavoidable third-party name collision, confined to
//  this one include:
//    - ERR_ARG is already a macro (commandProcessor.h, above): undef it so
//      lwIP's enum sees a plain identifier, then restore the same value
//      afterward.
//    - ERR_TIMEOUT is a genuine enum member (Errors.h, above), which #undef
//      can't touch — alias it to a throwaway name for the duration of this
//      one include so lwIP's own ERR_TIMEOUT enum member gets renamed to the
//      alias instead of colliding with GAACE's.
#undef ERR_ARG
#define ERR_TIMEOUT QNE_LWIP_ERR_TIMEOUT
#include <QNEthernet.h>
#undef ERR_TIMEOUT
#define ERR_ARG 2   // restore commandProcessor.h's definition
using namespace qindesign::network;

// =============================================================================
//  SECTION 2 — GLOBAL STATE
// =============================================================================

// Per-downstream-port capture ring buffer size (Section 6). Tunable: start
// small (2-4 KB; RAM isn't the constraint here, reasonable backlog is), bump
// up if a real device's unsolicited chatter between host visits needs more.
#define DEVPORT_RINGBUF_SIZE 2048

// Active runtime data. Loaded from EEPROM on boot; written back by SAVE.
Data data;

// Default (factory) values written whenever EEPROM is un-initialised or its
// signature is invalid (first boot, or a blank chip).
Data DefaultData =
{
  sizeof(Data),   // Size — always first so future code can detect struct growth
  "USBrepeater",  // Name
  1,              // Rev  — increment when hardware changes
  MODE_REPEATER,  // Mode — boot into repeater mode by default

  1,              // ActiveDevice    — downstream port 1, boots selected by default
  1,              // AdcTargetDevice — same, independent selection

  false,          // AdcEnabled  — off until a pin/command/scale is configured
  A0,             // AdcPin
  1000,           // AdcInterval — 1000 mS
  "SETVAL",       // AdcCmdName  — placeholder; set to match the downstream command
  1.0f,           // AdcScaleM   — value = counts (identity scale) until calibrated
  0.0f,           // AdcScaleB

  true,           // EthDHCP      — DHCP by default, zero-config on a typical LAN
  0,              // EthIP        — 0.0.0.0; set explicitly before switching off DHCP
  0x00FFFFFF,     // EthSubnet    — 255.255.255.0 (packed as IPAddress(255,255,255,0))
  0,              // EthGateway   — 0.0.0.0; set explicitly before switching off DHCP
  5000,           // EthCtrlPort  — TCP control port (same commands as SerialUSB1)
  5001,           // EthDataPort  — TCP passthrough port

  SIGNATURE       // Signature — always last
};

// Version string returned by the GVER command.
const char *Version = "USBrepeater, version 1.1 Sept 27, 2026";

// Currently active mode. Mirrors data.Mode but kept as a separate runtime
// variable so a mode change can be applied live without requiring SAVE.
uint8_t currentMode = MODE_REPEATER;

// =============================================================================
//  SECTION 3 — COMMAND PROCESSOR / THREADING SETUP
// =============================================================================

commandProcessor cp;
ThreadController control;               // Root scheduler

#if defined(GAACE_THREAD_CMDS)
threadCommands   tcmds(&cp, &control);  // TLIST / ?TENA / ?TINT / ... over cp
#endif

// GAACE_Script runtime (Section 6b) — GAACE_SCRIPT_SLOTS slots (default 4),
// each its own named Thread ("Script0".."Script<N-1>") added to `control`.
// Must be declared after cp/control above (constructor takes their address
// and calls control.add()); C++ constructs same-file globals in declaration
// order, same as tcmds above.
GAACEScript::ScriptRuntime scripts(&cp, &control);

// Housekeeping — placeholder periodic thread for future low-frequency work
// (link supervision, status polling, etc.). Currently a no-op: fill in
// HousekeepingUpdate() when a real periodic task is needed. Kept slow (250 ms)
// and off the byte-pump path so it can never stall repeater throughput.
Thread HousekeepingThread = Thread();

void HousekeepingUpdate(void)
{
  // Reserved for future periodic work.
}

// ADCUpdate — periodic ADC-driven downstream command injector. Implemented in
// Section 6a (below), alongside the per-port state it shares with
// RepeaterModeLoop() (userialPtr[]/devPort[], Section 6). ADCThread.onRun(
// ADCUpdate) is wired up in setup() (Section 8), which comes after Section 6a
// in this file, so no forward declaration is needed.
Thread ADCThread = Thread();

// EthernetUpdate — periodic Ethernet.loop() pump + connection accept-loop for
// the control/data TCP servers. Implemented in Section 6c (below), same
// deferred-definition pattern as ADCUpdate above.
Thread EthernetThread = Thread();

// =============================================================================
//  SECTION 4 — MODE DISPATCH TABLE
// =============================================================================
//
//  Add a new mode by: appending an entry to SystemMode in USBrepeater.h,
//  writing <Mode>Setup()/<Mode>Loop() functions, and adding a row here at the
//  same index as the enum value.

typedef void (*ModeFn)(void);

typedef struct
{
  const char *name;
  ModeFn      setup;
  ModeFn      loop;
} ModeDescriptor;

ModeDescriptor modeTable[NUM_MODES] =
{
  { "REPEATER", RepeaterModeSetup, RepeaterModeLoop },
};

// =============================================================================
//  SECTION 5 — COMMAND TABLE
// =============================================================================
//
//  Each entry is: { "CMD", CmdType, nargs, pointer, options, "help string" }
//  See commandProcessor.h for the full field reference.

static void cmdGetMode(void);
static void cmdSetMode(void);
static void cmdGetLink(void);
static void cmdGetAdcInterval(void);
static void cmdSetAdcInterval(void);
static void cmdGetAdc(void);
static void cmdGetBaud(void);
static void cmdSetBaud(void);
static void cmdGetEthIp(void);
static void cmdSetEthIp(void);
static void cmdGetEthMask(void);
static void cmdSetEthMask(void);
static void cmdGetEthGw(void);
static void cmdSetEthGw(void);
static void cmdGetEthStat(void);
static void cmdGetDev(void);
static void cmdSetDev(void);
static void cmdGetAdcDev(void);
static void cmdSetAdcDev(void);
static void cmdGetDevStat(void);

static int portLimits[] = {1, 65535};  // range check for ?ETHCTRLPORT / ?ETHDATAPORT

// Per-port accessors — implemented in Section 6, where userial1..userial4
// and their ring buffers actually live. Keeps these command handlers (and
// GLINK/SBAUD above) from needing extern access to the raw per-port arrays.
// dev1based is in [1, DEVPORT_COUNT]; callers here always pass a value
// already range-checked by cp.getValue() or a persisted Data field.
bool     isDeviceConnected(uint8_t dev1based);
uint16_t deviceBytesWaiting(uint8_t dev1based);
uint32_t deviceBytesDropped(uint8_t dev1based);
void     reinitDeviceBaud(uint8_t dev1based, uint32_t baud);

Command cmds[] =
{
  // ── Identification & persistence ──────────────────────────────────────────
  {"GVER",     CMDstr,      -1, (void *)Version,           NULL, "Firmware version"},
  {"?NAME",    CMDstr,      -1, (void *)&data.Name,        NULL, "Device name"},
  {"SAVE",     CMDfunction,  0, (void *)SaveSettings,      NULL, "Save system settings to EEPROM"},
  {"RESTORE",  CMDfunction,  0, (void *)RestoreSettings,   NULL, "Restore system settings from EEPROM"},

  // ── Mode control ───────────────────────────────────────────────────────────
  {"GMODE",    CMDfunction, -1, (void *)cmdGetMode,        NULL, "Get active mode, index and name"},
  {"SMODE",    CMDfunction, -1, (void *)cmdSetMode,        NULL, "Set active mode by index"},

  // ── Repeater diagnostics ───────────────────────────────────────────────────
  {"GLINK",    CMDfunction,  0, (void *)cmdGetLink,        NULL, "ActiveDevice's USB connection state"},
  {"GBAUD",    CMDfunction,  0, (void *)cmdGetBaud,        NULL, "Baud rate used to talk to the downstream device"},
  {"SBAUD",    CMDfunction,  1, (void *)cmdSetBaud,        NULL, "Set downstream baud rate; needed for Ethernet-only use (no Serial.baud() to follow)"},

  // ── Downstream device selection (Hub Carrier board, 4 ports) ────────────────
  //  SDEV switches which port the upstream data path (Serial + EthDataPort)
  //  bridges to; each port keeps capturing into its own ring buffer whether
  //  selected or not, so switching never loses what a non-selected device
  //  said — see Section 6. SADCDEV is a completely independent selection for
  //  ADCThread/the script demo, so background monitoring of one device isn't
  //  interrupted by switching interactive focus to another.
  {"GDEV",     CMDfunction, -1, (void *)cmdGetDev,         NULL, "Get the active downstream device, 1-4"},
  {"SDEV",     CMDfunction, -1, (void *)cmdSetDev,         NULL, "Set the active downstream device, 1-4"},
  {"GADCDEV",  CMDfunction, -1, (void *)cmdGetAdcDev,      NULL, "Get ADCThread/script target device, 1-4"},
  {"SADCDEV",  CMDfunction, -1, (void *)cmdSetAdcDev,      NULL, "Set ADCThread/script target device, 1-4"},
  {"GDEVSTAT", CMDfunction,  0, (void *)cmdGetDevStat,     NULL, "Per-port status: dev,connected,bytesWaiting,bytesDropped"},

  // ── Ethernet (single downstream device; see Section 6c) ────────────────────
  //  EthCtrlPort mirrors every SerialUSB1 command over TCP. EthDataPort is a
  //  second passthrough source for RepeaterModeLoop() alongside Serial — see
  //  `pcSide` in Section 6 for how replies are routed back to whichever one
  //  spoke most recently. EthDHCP/EthIP/EthMask/EthGw take effect on the next
  //  boot, not live; GETHSTAT always reflects live state.
  {"?ETHDHCP",     CMDbool,     -1, (void *)&data.EthDHCP,      NULL, "Use DHCP (TRUE) or static IP (FALSE); reboot to apply"},
  {"GETHIP",       CMDfunction, -1, (void *)cmdGetEthIp,        NULL, "Static IP address a.b.c.d; reboot to apply"},
  {"SETHIP",       CMDfunction, -1, (void *)cmdSetEthIp,        NULL, "Set static IP address a.b.c.d; reboot to apply"},
  {"GETHMASK",     CMDfunction, -1, (void *)cmdGetEthMask,      NULL, "Static subnet mask a.b.c.d; reboot to apply"},
  {"SETHMASK",     CMDfunction, -1, (void *)cmdSetEthMask,      NULL, "Set static subnet mask a.b.c.d; reboot to apply"},
  {"GETHGW",       CMDfunction, -1, (void *)cmdGetEthGw,        NULL, "Static gateway a.b.c.d; reboot to apply"},
  {"SETHGW",       CMDfunction, -1, (void *)cmdSetEthGw,        NULL, "Set static gateway a.b.c.d; reboot to apply"},
  {"?ETHCTRLPORT", CMDint,      -1, (void *)&data.EthCtrlPort,  portLimits, "TCP port for the network control port; reboot to apply"},
  {"?ETHDATAPORT", CMDint,      -1, (void *)&data.EthDataPort,  portLimits, "TCP port for network passthrough; reboot to apply"},
  {"GETHSTAT",     CMDfunction,  0, (void *)cmdGetEthStat,      NULL, "Live status: link,mode,ip,ctrlConnected,dataConnected"},

  // ── Periodic ADC -> downstream-device command update ───────────────────────
  //  Runs on ADCThread regardless of the active mode. Each cycle sends
  //  "<AdcCmdName>,<AdcScaleM * counts + AdcScaleB>\n" to the downstream
  //  device. Configure AdcCmdName to match a command the downstream device
  //  actually accepts before enabling with SADCEN,TRUE.
  {"?ADCEN",   CMDbool,     -1, (void *)&data.AdcEnabled,  NULL, "ADC update enabled, TRUE or FALSE"},
  {"?ADCPIN",  CMDbyte,     -1, (void *)&data.AdcPin,      NULL, "ADC input pin (e.g. 14 for A0)"},
  {"GADCINT",  CMDfunction, -1, (void *)cmdGetAdcInterval, NULL, "Get ADC update interval, mS"},
  {"SADCINT",  CMDfunction, -1, (void *)cmdSetAdcInterval, NULL, "Set ADC update interval, mS"},
  {"?ADCCMD",  CMDstr,      -1, (void *)&data.AdcCmdName,  NULL, "Command name sent to the downstream device"},
  {"?ADCM",    CMDfloat,    -1, (void *)&data.AdcScaleM,   NULL, "ADC scale slope: value = M * counts + B"},
  {"?ADCB",    CMDfloat,    -1, (void *)&data.AdcScaleB,   NULL, "ADC scale offset: value = M * counts + B"},
  {"GADC",     CMDfunction,  0, (void *)cmdGetAdc,         NULL, "Read ADC now: averaged counts (2dp) and scaled value (4dp)"},

  // ── GAACE_Script ADC demo (Section 6b) ──────────────────────────────────
  //  SCRIPTLOAD/GSCRIPTLIMITS/GSCRIPTST come from GAACEScriptRuntime.h
  //  (registered in setup() via cp.registerCommands(scripts.scriptCmdList())).
  //  Starting/stopping the demo (slot "Script0") and changing its rate use
  //  GAACE_Core's own threadCommands instead of a dedicated command here:
  //  GTENA,Script0 / STENA,Script0,TRUE|FALSE and GTINT,Script0 /
  //  STINT,Script0,<mS>. Reuses ?ADCPIN/?ADCM/?ADCB/?ADCCMD above. Don't
  //  enable both Script0 and ADCEN against the same downstream command —
  //  they'd double-send.

  {NULL}  // Sentinel — must remain as the last entry
};
static CommandList cmdList = {cmds, NULL};

// -----------------------------------------------------------------------------
// GMODE — print the active mode's index and name.
// -----------------------------------------------------------------------------
static void cmdGetMode(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.print((int)currentMode);
  cp.print(",");
  cp.println(modeTable[currentMode].name);
}

// -----------------------------------------------------------------------------
// SMODE,<index> — switch the active mode and re-run its setup().
// The change is applied immediately but not persisted; call SAVE to make it
// survive a reboot.
// -----------------------------------------------------------------------------
static void cmdSetMode(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int m;
  if (!cp.getValue(&m, 0, NUM_MODES - 1))
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  currentMode = (uint8_t)m;
  data.Mode   = currentMode;
  modeTable[currentMode].setup();
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GLINK — TRUE if the active downstream device (data.ActiveDevice) is
// currently enumerated. isDeviceConnected() etc. are declared earlier in this
// section (forward declarations, implemented in Section 6).
// -----------------------------------------------------------------------------
extern uint32_t       curBaud;         // defined in Section 6
extern EthernetClient ethCtrlClient;    // defined in Section 6c
extern EthernetClient ethDataClient;    // defined in Section 6c

static void cmdGetLink(void)
{
  cp.sendACK(false);
  cp.println(isDeviceConnected(data.ActiveDevice));
}

// -----------------------------------------------------------------------------
// GDEV / SDEV,<n> — get/set which downstream port (1-4) the upstream data
// path (Serial + EthDataPort) is bridged to. The newly-selected port's ring
// buffer (Section 6) starts flushing to the upstream on the very next
// RepeaterModeLoop() pass — no separate "flush on switch" step is needed,
// since the loop always flushes whichever port is currently selected.
// -----------------------------------------------------------------------------
static void cmdGetDev(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println((int)data.ActiveDevice);
}

static void cmdSetDev(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int n;
  if (!cp.getValue(&n, 1, DEVPORT_COUNT))
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  data.ActiveDevice = (uint8_t)n;
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GADCDEV / SADCDEV,<n> — get/set which downstream port (1-4) ADCThread and
// the GAACE_Script demo send to. Deliberately independent of GDEV/SDEV: see
// the command table comment above.
// -----------------------------------------------------------------------------
static void cmdGetAdcDev(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println((int)data.AdcTargetDevice);
}

static void cmdSetAdcDev(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int n;
  if (!cp.getValue(&n, 1, DEVPORT_COUNT))
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  data.AdcTargetDevice = (uint8_t)n;
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GDEVSTAT — one row per downstream port: dev, connected, bytesWaiting,
// bytesDropped. bytesDropped is a monotonic since-boot counter (Section 6
// never auto-clears it, including on flush) so a host that wasn't watching
// can still tell whether a port's "nothing is lost" promise actually held.
// -----------------------------------------------------------------------------
static void cmdGetDevStat(void)
{
  if (!cp.checkExpectedArgs(0)) return;

  cp.sendACK(false);
  cp.println("#dev,connected,bytesWaiting,bytesDropped");
  for (uint8_t i = 1; i <= DEVPORT_COUNT; i++)
  {
    cp.print((int)i);                            cp.print(",");
    cp.print(isDeviceConnected(i));               cp.print(",");
    cp.print((int)deviceBytesWaiting(i));         cp.print(",");
    cp.println(deviceBytesDropped(i));
  }
}

// -----------------------------------------------------------------------------
// GBAUD / SBAUD,<baud> — get/set the baud rate used to talk to the downstream
// device. Serial.baud() auto-follows the PC's chosen baud when Serial drives
// the link (see RepeaterModeLoop(), Section 6); that signal doesn't exist
// over Ethernet, so a network-only client needs this to configure it.
// -----------------------------------------------------------------------------
static void cmdGetBaud(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println(curBaud);
}

static void cmdSetBaud(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  uint32_t b;
  if (!cp.getValue(&b, 300, 2000000))   // sane serial baud range
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  curBaud = b;
  reinitDeviceBaud(data.ActiveDevice, curBaud);
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GETHIP/SETHIP, GETHMASK/SETHMASK, GETHGW/SETHGW — dotted-quad get/set for
// the static-IP config fields. Function pairs rather than a bidirectional
// CMDstr/CMDint field because there's no built-in CmdType that formats or
// parses a.b.c.d. Changes take effect on the next boot (EthernetSetup(),
// Section 6c/8), not live.
// -----------------------------------------------------------------------------
static void printIpAddr(IPAddress ip)
{
  cp.print((int)ip[0]); cp.print(".");
  cp.print((int)ip[1]); cp.print(".");
  cp.print((int)ip[2]); cp.print(".");
  cp.print((int)ip[3]);
}

static bool parseIpArg(uint32_t *out)
{
  char *tok;
  if (!cp.getValue(&tok)) return false;

  int a, b, c, d;
  bool ok = (sscanf(tok, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) &&
            a >= 0 && a <= 255 && b >= 0 && b <= 255 &&
            c >= 0 && c <= 255 && d >= 0 && d <= 255;
  cp.ca->free(tok);
  if (!ok) return false;

  *out = (uint32_t)IPAddress(a, b, c, d);
  return true;
}

static void cmdGetEthIp(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  printIpAddr(IPAddress(data.EthIP));
  cp.print();
}

static void cmdSetEthIp(void)
{
  if (!cp.checkExpectedArgs(1)) return;
  if (!parseIpArg(&data.EthIP)) { cp.sendNAK(ERR_BADARG); return; }
  cp.sendACK();
}

static void cmdGetEthMask(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  printIpAddr(IPAddress(data.EthSubnet));
  cp.print();
}

static void cmdSetEthMask(void)
{
  if (!cp.checkExpectedArgs(1)) return;
  if (!parseIpArg(&data.EthSubnet)) { cp.sendNAK(ERR_BADARG); return; }
  cp.sendACK();
}

static void cmdGetEthGw(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  printIpAddr(IPAddress(data.EthGateway));
  cp.print();
}

static void cmdSetEthGw(void)
{
  if (!cp.checkExpectedArgs(1)) return;
  if (!parseIpArg(&data.EthGateway)) { cp.sendNAK(ERR_BADARG); return; }
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GETHSTAT — live network status: link,mode,ip,ctrlConnected,dataConnected.
// Always reflects current runtime state (unlike the config getters above,
// which read back the stored, possibly-not-yet-applied, EthIP/EthMask/EthGw).
// -----------------------------------------------------------------------------
static void cmdGetEthStat(void)
{
  if (!cp.checkExpectedArgs(0)) return;

  cp.sendACK(false);
  cp.print(Ethernet.linkState() ? "LINKUP" : "LINKDOWN"); cp.print(",");
  cp.print(data.EthDHCP ? "DHCP" : "STATIC");             cp.print(",");
  printIpAddr(Ethernet.localIP());                        cp.print(",");
  cp.print((bool)ethCtrlClient && ethCtrlClient.connected()); cp.print(",");
  cp.println((bool)ethDataClient && ethDataClient.connected());
}

// -----------------------------------------------------------------------------
// GADCINT / SADCINT,<mS> — get/set the ADC update interval.
// A plain bidirectional CMDint field can't also update ADCThread's own
// interval, so this is a function pair (same pattern as GMODE/SMODE) rather
// than a "?ADCINT" entry.
// -----------------------------------------------------------------------------
static void cmdGetAdcInterval(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println((uint32_t)data.AdcInterval);
}

static void cmdSetAdcInterval(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  uint32_t ms;
  if (!cp.getValue(&ms, 1, 3600000))   // 1 mS .. 1 hour
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  data.AdcInterval = ms;
  ADCThread.setInterval((unsigned long)ms);
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// readAdcCounts — single point of truth for reading the ADC, per the
// carrier-board design guide §5.6.2. Hardware oversampling
// (analogReadAveraging(), configured once in setup() alongside
// analogReadResolution()) plus a software average on top smooths out noise
// further than either alone. Every reader — ADCUpdate(), GADC, and the
// script read_adc() syscall — goes through this, so there's exactly one
// averaging behavior to reason about. Returns counts as a float (not
// rounded) so AdcScaleM*counts+AdcScaleB keeps full precision; a caller
// that needs an integer (sys_read_adc(), Section 6b — the VM is int32)
// rounds it itself.
//
// NOTE: this runs at 12-bit resolution (0-4095 counts). Earlier builds read
// the ADC at Teensy's default 10-bit resolution (0-1023) with no averaging
// at all — the same physical input now reads ~4x higher in raw counts, so
// any AdcScaleM/AdcScaleB calibrated and saved under that older firmware is
// 4x off until recalibrated, not just reusable as-is.
//
// Tuning values below are #defines, not hardcoded in the function body, so
// they can be adjusted without touching the averaging logic itself.
// -----------------------------------------------------------------------------
#define ADC_RESOLUTION_BITS  12   // analogReadResolution() — 0..4095 counts
#define ADC_HW_AVERAGING     32   // analogReadAveraging() — hardware oversampling
#define ADC_SW_SAMPLES       16   // additional software average on top of the above

static float readAdcCounts(void)
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SW_SAMPLES; i++)
  {
    sum += analogRead(data.AdcPin);
  }
  return (float)sum / (float)ADC_SW_SAMPLES;
}

// -----------------------------------------------------------------------------
// GADC — read the ADC pin right now and report both the averaged counts and
// the scaled value, without waiting for the next ADCThread cycle or sending
// anything to the downstream device. Useful for checking AdcScaleM/AdcScaleB
// before enabling ADCEN.
// -----------------------------------------------------------------------------
static void cmdGetAdc(void)
{
  float counts = readAdcCounts();
  float value  = data.AdcScaleM * counts + data.AdcScaleB;

  cp.sendACK(false);
  cp.print(counts, 2);
  cp.print(",");
  cp.println(value, 4);
}

// =============================================================================
//  SECTION 6 — REPEATER MODE (4 downstream ports, Hub Carrier board)
// =============================================================================
//
//  PC <-> Teensy device port (micro-USB, SerialUSB1 shares the connector) <->
//  Teensy host port (5-pin header) <-> USB2514B hub <-> one of 4 downstream
//  USB-A ports. As of Section 6c, a connected Ethernet client on EthDataPort
//  is a second possible "PC" for this same link — see `pcSide` below.
//
//  TOPOLOGY
//  --------
//  All 4 downstream ports stay connected/auto-reconnecting continuously,
//  independent of which one the host is looking at (devPortUpdateConnection()
//  below runs on all 4 every tick). Each port has its own ring buffer
//  (devPort[i].rxbuf) that captures incoming bytes continuously too — so a
//  device that chatters unsolicited while not selected is never silently
//  lost. Only ONE port at a time can be bridged to the upstream data path
//  (Serial + EthDataPort): the hardware doesn't have more upstream USB
//  endpoints to spare (see TODO.md), and giving every port its own Ethernet
//  port is still future work. data.ActiveDevice (1-4, SDEV/GDEV) selects it.
//
//  The host is responsible for mapping port number to device identity/
//  function — it does this itself, by switching SDEV through 1-4 and asking
//  each one GNAME/GVER over the ordinary passthrough path. Firmware doesn't
//  try to identify what's plugged into a port; that would require injecting
//  and parsing a command on every connect, which is real complexity this
//  design deliberately avoids by pushing the decision to the host instead.
//
//  RING BUFFER / FLUSH MODEL
//  -------------------------
//  Every tick: drain available bytes from ALL 4 ports into their own ring
//  buffers (devPortDrainIncoming()), unconditionally. Then flush ONLY the
//  currently-selected port's buffer out to the upstream (devPortFlush()).
//  This one mechanism covers both live forwarding (the selected port's
//  buffer drains about as fast as it fills, so latency in steady state is
//  the same as direct passthrough) and "deliver the backlog" when SDEV
//  switches to a port that's been accumulating in the background — nothing
//  special has to happen at the moment of switching, because the flush step
//  doesn't care whether the backlog built up over one tick or a thousand.
//
//  Overflow policy (decided): drop incoming bytes once a port's buffer is
//  full, preserving the order of what's already queued, rather than
//  overwriting old data with new. Either way data gets lost once genuinely
//  full — devPort[i].rxDropped is a monotonic since-boot counter (GDEVSTAT)
//  so that loss is at least visible, never silent.
//
//  Runs unthrottled from loop() — do not add commandProcessor traffic or
//  other blocking work to this path; it must stay fast enough to keep up
//  with the downstream devices' baud rate.

USBHost             myusb;
USBHub              hub1(myusb);
USBSerial_BigBuffer userial1(myusb);
USBSerial_BigBuffer userial2(myusb);
USBSerial_BigBuffer userial3(myusb);
USBSerial_BigBuffer userial4(myusb);

// Indexed 0..DEVPORT_COUNT-1; devPort[]/the command handlers above use
// 1-based device numbers throughout (dev1based - 1 to index in here).
USBSerial_BigBuffer *userialPtr[DEVPORT_COUNT] = { &userial1, &userial2, &userial3, &userial4 };

uint32_t curBaud = 115200;   // shared across all 4 ports; set via SBAUD or Serial.baud() auto-follow
uint8_t  repeaterBuf[512];   // shared scratch buffer for all per-port USB reads

struct DevPort
{
  bool     wasConnected       = false;
  uint16_t rxHead             = 0;  // ring buffer write index
  uint16_t rxTail             = 0;  // ring buffer read index
  uint16_t rxCount            = 0;  // bytes currently buffered
  uint32_t rxDropped          = 0;  // bytes dropped because the buffer was full, since boot
  uint32_t swallowUntilMs     = 0;  // while millis() < this, incoming bytes are discarded (ADC/script injection reply — Section 6a/6b)
  uint32_t lastHostToDeviceMs = 0;  // last time host traffic was forwarded TO this port (ADC/script quiet guard)
  uint8_t  rxbuf[DEVPORT_RINGBUF_SIZE];
};

DevPort devPort[DEVPORT_COUNT];

// Whichever source (Serial, or the Ethernet data client — Section 6c) most
// recently sent bytes to whichever device is currently selected. Downstream
// device -> PC bytes are written only here: "last speaker wins", the same
// pattern commandProcessor::processStreams() already uses
// (`serial = streams[i]; // route replies back to the stream that sent the command`).
// This settles *who gets the reply*; it does not stop two sources from
// writing to the same device in the same loop pass if both have data
// available at once — see Section 6c for why that's accepted as a known
// edge case rather than solved with hard exclusivity. Not per-port: it's
// about which upstream *channel* the host is using, independent of which
// device SDEV currently points at (switching devices over the control port
// doesn't change which passthrough channel the host is using for data).
Stream *pcSide = &Serial;

// -----------------------------------------------------------------------------
// Per-port accessors — forward-declared in Section 5 for the command
// handlers there (GLINK, GDEVSTAT, SBAUD, ...).
// -----------------------------------------------------------------------------
bool isDeviceConnected(uint8_t dev1based)
{
  if (dev1based < 1 || dev1based > DEVPORT_COUNT) return false;
  return (bool)(*userialPtr[dev1based - 1]);
}

uint16_t deviceBytesWaiting(uint8_t dev1based)
{
  if (dev1based < 1 || dev1based > DEVPORT_COUNT) return 0;
  return devPort[dev1based - 1].rxCount;
}

uint32_t deviceBytesDropped(uint8_t dev1based)
{
  if (dev1based < 1 || dev1based > DEVPORT_COUNT) return 0;
  return devPort[dev1based - 1].rxDropped;
}

void reinitDeviceBaud(uint8_t dev1based, uint32_t baud)
{
  if (dev1based < 1 || dev1based > DEVPORT_COUNT) return;
  if (isDeviceConnected(dev1based)) userialPtr[dev1based - 1]->begin(baud);
}

// -----------------------------------------------------------------------------
// devPortPush — accept one incoming byte into port idx's ring buffer.
// Swallow check happens here (not at flush time): a byte arriving during
// idx's swallow window is the reply to an ADCUpdate()/script-injected
// command and is never meant for the host, on this port or any other, so it
// must never enter the buffer at all — not dropped-and-counted (that's for
// genuine overflow only), just discarded.
// -----------------------------------------------------------------------------
static void devPortPush(uint8_t idx, uint8_t b)
{
  DevPort &p = devPort[idx];
  if (millis() < p.swallowUntilMs) return;

  if (p.rxCount >= DEVPORT_RINGBUF_SIZE)
  {
    p.rxDropped++;   // overflow: drop incoming, keep what's already queued
    return;
  }

  p.rxbuf[p.rxHead] = b;
  p.rxHead = (uint16_t)((p.rxHead + 1) % DEVPORT_RINGBUF_SIZE);
  p.rxCount++;
}

// Drain whatever's available from userialPtr[idx] into devPort[idx]'s ring
// buffer. Runs for all 4 ports every tick, regardless of selection.
static void devPortDrainIncoming(uint8_t idx)
{
  int n = userialPtr[idx]->available();
  if (n <= 0) return;

  n = userialPtr[idx]->readBytes((char *)repeaterBuf, min(n, (int)sizeof(repeaterBuf)));
  for (int i = 0; i < n; i++) devPortPush(idx, repeaterBuf[i]);
}

// Write out everything currently buffered for port idx, in order, to `out`.
// At most two write() calls (handles the ring's wraparound in two
// contiguous runs) rather than one per byte.
static void devPortFlush(uint8_t idx, Stream *out)
{
  DevPort &p = devPort[idx];
  while (p.rxCount > 0)
  {
    uint16_t contiguous = (uint16_t)min((int)p.rxCount, (int)(DEVPORT_RINGBUF_SIZE - p.rxTail));
    out->write(&p.rxbuf[p.rxTail], contiguous);
    p.rxTail   = (uint16_t)((p.rxTail + contiguous) % DEVPORT_RINGBUF_SIZE);
    p.rxCount -= contiguous;
  }
}

// Attach/detach + auto-reconnect for one port. Runs on all 4 every tick,
// independent of selection — a port reconnecting must not depend on whether
// it's the one currently bridged to the upstream.
static void devPortUpdateConnection(uint8_t idx)
{
  DevPort &p = devPort[idx];
  bool connected = (bool)(*userialPtr[idx]);

  if (connected && !p.wasConnected) userialPtr[idx]->begin(curBaud);
  p.wasConnected = connected;
}

void RepeaterModeSetup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);       // LED = ActiveDevice connected
  Serial.begin(115200);               // native USB, baud value ignored

  // Strap + reset the USB2514B hub (no hardware straps on this board — see
  // UsbHubBoot.h). Must happen before myusb.begin(): run() leaves the hub
  // deliberately detached (VBUS_DET low) so it never attaches mid-strap;
  // attach() is what lets the now-running host stack actually see it.
  UsbHubBoot::run();
  myusb.begin();                      // enables host port VBUS
  UsbHubBoot::attach();
}

void RepeaterModeLoop(void)
{
  myusb.Task();

  // Connection management + incoming capture: all 4 ports, every tick,
  // unconditionally. This is what makes "nothing is lost" true regardless
  // of what's selected.
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    devPortUpdateConnection(i);
    devPortDrainIncoming(i);
  }

  uint8_t selIdx = (uint8_t)(data.ActiveDevice - 1);
  if (selIdx >= DEVPORT_COUNT) selIdx = 0;   // defensive; ActiveDevice is range-checked at SDEV time
  USBSerial_BigBuffer *sel = userialPtr[selIdx];
  bool connected = (bool)(*sel);

  digitalWriteFast(LED_BUILTIN, connected ? HIGH : LOW);

  // Follow the baud rate the PC application sets (matters for FTDI/CP210x/CH340 bridges).
  // Applies only to the selected device — the one actually receiving PC traffic right now.
  uint32_t b = Serial.baud();
  if (b != 0 && b != curBaud)
  {
    curBaud = b;
    if (connected) sel->begin(curBaud);
  }

  // PC -> device: only meaningful if the selected device is actually
  // connected. Both Serial and a connected Ethernet data client can drive
  // it; whichever one actually sends bytes this pass becomes pcSide, and
  // also marks this port as "just heard from the host" for ADCUpdate()'s
  // quiet guard (Section 6a). If both have data in the same pass, both
  // still get forwarded here (in this order) and can interleave on the
  // wire — see Section 6c.
  if (connected)
  {
    int n = Serial.available();
    if (n > 0)
    {
      n = Serial.readBytes((char *)repeaterBuf, min(n, (int)sizeof(repeaterBuf)));
      sel->write(repeaterBuf, n);
      devPort[selIdx].lastHostToDeviceMs = millis();
      pcSide = &Serial;
    }

    if (ethDataClient && ethDataClient.connected())
    {
      int n2 = ethDataClient.available();
      if (n2 > 0)
      {
        n2 = ethDataClient.read(repeaterBuf, min(n2, (int)sizeof(repeaterBuf)));
        sel->write(repeaterBuf, n2);
        devPort[selIdx].lastHostToDeviceMs = millis();
        pcSide = &ethDataClient;
      }
    }
  }

  // Device -> PC: flush whatever's buffered for the selected port, to
  // whoever last spoke. Flushed even if the device isn't connected right
  // now — a backlog captured before a disconnect is still worth delivering.
  devPortFlush(selIdx, pcSide);
}

// =============================================================================
//  SECTION 6a — PERIODIC ADC -> DOWNSTREAM-DEVICE COMMAND UPDATE
// =============================================================================
//
//  ADCUpdate() runs on ADCThread (registered in setup()), independent of the
//  active mode, so it keeps working while REPEATER mode is active. It writes
//  directly to data.AdcTargetDevice's userial — a specific downstream port,
//  independent of whatever data.ActiveDevice (SDEV) the host currently has
//  selected — because that IS the connection to the device being
//  controlled. There is no framing on that raw byte stream, so two guards,
//  now tracked per-port (DevPort::lastHostToDeviceMs/swallowUntilMs, Section
//  6), keep injected commands from colliding with ordinary repeater traffic:
//
//   1. QUIET GUARD (before sending): skip this cycle if the host forwarded
//      bytes to THIS port within the last ADC_QUIET_GUARD_MS. Only actually
//      possible if the ADC target happens to also be the selected device —
//      a non-selected port's timestamp just sits stale, which correctly
//      never blocks an injection. Reduces the chance of splicing our own
//      bytes into the middle of a host-initiated command that's still
//      arriving.
//
//   2. SWALLOW WINDOW (after sending): for ADC_SWALLOW_WINDOW_MS after
//      injecting, devPortPush() (Section 6) discards this port's incoming
//      bytes instead of buffering them, so the downstream device's reply to
//      our command can never reach the host — on this port or any other —
//      whether or not it happens to be selected right now.
//
//  Neither guard is a hard guarantee — a downstream device that replies
//  unusually slowly could see its reply forwarded anyway (swallow window
//  expired), and a host command that happens to start in the few mS after an
//  injection could still overlap it. They are heuristics sized for typical
//  short ASCII command/reply traffic, not a real bus arbitration protocol.
//  Widen the windows here if your downstream device is slower to reply.

#define ADC_QUIET_GUARD_MS     5   // skip this cycle if the link was busy this recently
#define ADC_SWALLOW_WINDOW_MS  100 // discard downstream traffic for this long after injecting

void ADCUpdate(void)
{
  if (!data.AdcEnabled) return;

  uint8_t idx = (uint8_t)(data.AdcTargetDevice - 1);
  if (idx >= DEVPORT_COUNT) return;   // defensive; AdcTargetDevice is range-checked at SADCDEV time
  if (!(bool)(*userialPtr[idx])) return;   // nothing connected to control

  if ((millis() - devPort[idx].lastHostToDeviceMs) < ADC_QUIET_GUARD_MS) return;  // link busy, try next cycle

  float counts = readAdcCounts();
  float value  = data.AdcScaleM * counts + data.AdcScaleB;

  char cmdLine[48];
  int  len = snprintf(cmdLine, sizeof(cmdLine), "%s,%.3f\n", data.AdcCmdName, value);
  if (len <= 0) return;
  if (len >= (int)sizeof(cmdLine)) len = sizeof(cmdLine) - 1;  // truncated; still send what fits

  userialPtr[idx]->write((uint8_t *)cmdLine, len);
  devPort[idx].swallowUntilMs = millis() + ADC_SWALLOW_WINDOW_MS;
}

// =============================================================================
//  SECTION 6b — GAACE_SCRIPT ADC DEMO
// =============================================================================
//
//  Runs gaace_scripts/adc_demo.gs (compiled to include/ScriptBytecode.h — see
//  that file's header for the regeneration command) in slot 0 of `scripts`
//  (the GAACEScript::ScriptRuntime declared in Section 3), alongside
//  ADCThread. This is a real integration, not a standalone toy: the syscalls
//  below drive the actual downstream link for data.AdcTargetDevice (the same
//  target ADCThread uses), the actual calibration fields (AdcPin/AdcScaleM/
//  AdcScaleB/AdcCmdName), and that port's own guard/swallow state
//  (DevPort::lastHostToDeviceMs/swallowUntilMs, Section 6) — so the two
//  features can't collide with ordinary repeater traffic any differently
//  than ADCUpdate() already doesn't.
//
//  This is a demonstration, not a replacement for ADCThread — the two do the
//  same conceptual job (read the ADC, send a scaled value downstream), so
//  slot 0 ("Script0") starts disabled (see setup(), Section 8). Enabling it
//  (STENA,Script0,TRUE) at the same time as ADCEN, against the same
//  downstream command, would double-send. What the script demonstrates that
//  ADCUpdate() doesn't: conditional logic living in the script instead of
//  hardcoded in C++ — it only sends when the reading has moved more than a
//  threshold since the last send, using a `var`-declared slot (see
//  gaace_scripts/adc_demo.gs) that persists across ticks because
//  ScriptSlot::run() (GAACE_Script's runtime module) only resets pc/sp
//  between calls, not vmInit()'s full reset.
//
//  Syscall ids (must match scripts.registerSyscall() call order in setup(),
//  and the `syscall NAME(...) = ID;` declarations in gaace_scripts/adc_demo.gs):
//    0 = link_ready()      — data.AdcTargetDevice connected AND quiet-guard elapsed
//    1 = read_adc()         — readAdcCounts() rounded to int32 via lroundf()
//                             (the VM is int32-only; see readAdcCounts()'s
//                             own comment in Section 5 for the averaging)
//    2 = scale_send(counts) — AdcScaleM*counts+AdcScaleB, sent downstream

#include "ScriptBytecode.h"

using namespace GAACEScript;

static int32_t sys_link_ready(int32_t *args, uint8_t argc)
{
  (void)args; (void)argc;

  uint8_t idx = (uint8_t)(data.AdcTargetDevice - 1);
  if (idx >= DEVPORT_COUNT) return 0;
  if (!(bool)(*userialPtr[idx])) return 0;                                      // nothing connected
  if ((millis() - devPort[idx].lastHostToDeviceMs) < ADC_QUIET_GUARD_MS) return 0; // link busy
  return 1;
}

static int32_t sys_read_adc(int32_t *args, uint8_t argc)
{
  (void)args; (void)argc;
  return lroundf(readAdcCounts());
}

static int32_t sys_scale_send(int32_t *args, uint8_t argc)
{
  (void)argc;

  uint8_t idx = (uint8_t)(data.AdcTargetDevice - 1);
  if (idx >= DEVPORT_COUNT) return 0;

  float value = data.AdcScaleM * (float)args[0] + data.AdcScaleB;

  char cmdLine[48];
  int  len = snprintf(cmdLine, sizeof(cmdLine), "%s,%.3f\n", data.AdcCmdName, value);
  if (len <= 0) return 0;
  if (len >= (int)sizeof(cmdLine)) len = sizeof(cmdLine) - 1;  // truncated; still send what fits

  userialPtr[idx]->write((uint8_t *)cmdLine, len);
  devPort[idx].swallowUntilMs = millis() + ADC_SWALLOW_WINDOW_MS;
  return 1;
}

// =============================================================================
//  SECTION 6c — ETHERNET
// =============================================================================
//
//  Two independent TCP servers, both single-session ("latest connection
//  wins" — same policy every commercial serial-to-Ethernet device server
//  uses, because two live sessions driving the same downstream link would
//  corrupt each other's traffic regardless of arbitration):
//
//    ethCtrlServer / ethCtrlClient — mirrors SerialUSB1: registered with
//      cp.registerStream() once in EthernetSetup(), so every command
//      (GVER, GMODE, SDEV, SADCxxx, ...) works verbatim over this TCP port
//      with no command-processor changes.
//
//    ethDataServer / ethDataClient — a second passthrough source for
//      RepeaterModeLoop() (Section 6), alongside Serial, bridged to
//      whichever port data.ActiveDevice currently selects. See `pcSide`
//      there for how replies are routed back to whichever source spoke most
//      recently. One data port shared across all 4 downstream devices for
//      now (matching Serial's single switchable upstream path) — a
//      dedicated TCP port per device is still future work (see TODO.md).
//
//  Both client globals are reused across reconnects (`ethXClient = newClient`
//  inside EthernetUpdate() below) rather than replaced, because
//  cp.registerStream() has no matching "unregister"/"replace" call — it
//  records a Stream* once, so that address must stay the same object for the
//  life of the program. QNEthernet's EthernetClient supports this: it's a
//  copyable handle around a connection, and calling available()/connected()
//  on one that was never connected (or has since disconnected) safely
//  returns 0/false rather than crashing or blocking.
//
//  KNOWN LIMITATION (carried over from design discussion, not solved here):
//  pcSide (Section 6) resolves who gets a reply, not simultaneous writes. If
//  Serial and ethDataClient both have bytes available in the same
//  RepeaterModeLoop() pass, both still get forwarded to the selected
//  device's userial that pass and can interleave on the wire. This only bites
//  under sustained
//  concurrent traffic from both sides at once, not "one connected while the
//  other is idle" — accepted as a known edge case, same as the equivalent
//  ADC-injection guards are heuristics rather than a real bus-arbitration
//  protocol.
//
//  BAUD RATE: Serial.baud() (Section 6) only exists for a USB CDC host; a
//  pure-Ethernet client has no equivalent way to signal a baud rate, so
//  GBAUD/SBAUD (Section 5) exist for that case — set it once after boot if
//  you're not also using the USB side.

EthernetServer ethCtrlServer;   // port bound in EthernetSetup() from data.EthCtrlPort
EthernetServer ethDataServer;   // port bound in EthernetSetup() from data.EthDataPort
EthernetClient ethCtrlClient;
EthernetClient ethDataClient;

static void EthernetSetup(void)
{
  if (data.EthDHCP)
  {
    Ethernet.begin();   // DHCP; uses the Teensy's system MAC automatically
  }
  else
  {
    Ethernet.begin(IPAddress(data.EthIP), IPAddress(data.EthSubnet), IPAddress(data.EthGateway));
  }

  ethCtrlServer.begin(data.EthCtrlPort);
  ethDataServer.begin(data.EthDataPort);

  // Registered once; see the comment above for why the same object is
  // reused across reconnects instead of re-registering a new one.
  cp.registerStream(&ethCtrlClient);
}

static void EthernetUpdate(void)
{
  Ethernet.loop();   // "call often" per QNEthernet's docs — DHCP renewal etc.

  // accept() is non-blocking: it returns an unconnected client immediately
  // if nobody is trying to connect right now.
  EthernetClient nc = ethCtrlServer.accept();
  if (nc) { if (ethCtrlClient) ethCtrlClient.stop(); ethCtrlClient = nc; }

  EthernetClient nd = ethDataServer.accept();
  if (nd) { if (ethDataClient) ethDataClient.stop(); ethDataClient = nd; }
}

// =============================================================================
//  SECTION 7 — SETTINGS PERSISTENCE (EEPROM)
// =============================================================================

#define EEPROM_ADDR 0

// SaveSettings — write the current Data struct to EEPROM.
// The Signature is set here (not at load time) so that a partial write caused
// by power loss does not create an apparently-valid but incomplete record.
void SaveSettings(void)
{
  data.Signature = SIGNATURE;
  EEPROM.put(EEPROM_ADDR, data);
  cp.sendACK();
}

// RestoreSettings — reload Data from EEPROM without rebooting.
// Validates the signature before overwriting the live data struct.
void RestoreSettings(void)
{
  Data td;
  EEPROM.get(EEPROM_ADDR, td);
  if (td.Signature == SIGNATURE)
  {
    data        = td;
    currentMode = data.Mode;
    cp.sendACK();
  }
  else
  {
    // No valid record has ever been saved.
    cp.sendNAK(ERR_NOSDCARD);
  }
}

// =============================================================================
//  SECTION 8 — SETUP
// =============================================================================

void setup()
{
  // ── 1. Restore persistent data ──────────────────────────────────────────────
  EEPROM.get(EEPROM_ADDR, data);
  if (data.Signature != SIGNATURE) data = DefaultData;
  currentMode = data.Mode;
  if (currentMode >= NUM_MODES) currentMode = MODE_REPEATER;

  // ── 2. ADC hardware configuration ───────────────────────────────────────────
  //  Once, here — not per-read. See readAdcCounts() (Section 5) for the full
  //  averaging story (hardware oversampling here, plus a software average on
  //  top in readAdcCounts() itself) and the 10-bit-vs-12-bit calibration note.
  analogReadResolution(ADC_RESOLUTION_BITS);
  analogReadAveraging(ADC_HW_AVERAGING);

  // ── 3. Command / control USB port ───────────────────────────────────────────
  //  Kept separate from Serial (see the port-layout note at the top of this
  //  file) so the two never collide.
  SerialUSB1.begin(115200);
  cp.registerStream(&SerialUSB1);

  // ── 4. Command registration ─────────────────────────────────────────────────
  cp.registerCommands(&cmdList);
  #if defined(GAACE_THREAD_CMDS)
  cp.registerCommands(tcmds.threadCmdList());  // TLIST / ?TENA / ?TINT / ...
  #endif
  cp.registerCommands(scripts.scriptCmdList()); // SCRIPTLOAD / GSCRIPTLIMITS / GSCRIPTST

  // ── 5. Thread scheduler ─────────────────────────────────────────────────────
  //  scripts' own slot threads ("Script0".."Script<N-1>") were already added
  //  to `control` by ScriptRuntime's constructor (Section 3).
  HousekeepingThread.setName("Housekeeping");
  HousekeepingThread.onRun(HousekeepingUpdate);
  HousekeepingThread.setInterval(250);   // ms between HousekeepingUpdate() calls
  control.add(&HousekeepingThread);

  ADCThread.setName("ADCUpdate");
  ADCThread.onRun(ADCUpdate);
  ADCThread.setInterval(data.AdcInterval);
  control.add(&ADCThread);

  // ── 5a. Ethernet (Section 6c) ───────────────────────────────────────────────
  //  Bring up the network and the two TCP servers, then poll for new
  //  connections / pump lwIP on its own thread. Independent of the active
  //  mode, same as ADCThread — keeps working during REPEATER mode.
  EthernetSetup();
  EthernetThread.setName("Ethernet");
  EthernetThread.onRun(EthernetUpdate);
  EthernetThread.setInterval(20);   // ms; keeps Ethernet.loop() ticking promptly
  control.add(&EthernetThread);

  // Preload the ADC demo (Section 6b) into slot 0, the same way a project
  // preloads any default script: fill code[]/codeLen, vmInit(), loaded=true.
  // Starts disabled -- see Section 6b for why (double-send risk with
  // ADCThread); enable with STENA,Script0,TRUE once ready to compare them.
  scripts.registerSyscall(sys_link_ready);   // id 0 — see Section 6b
  scripts.registerSyscall(sys_read_adc);     // id 1
  scripts.registerSyscall(sys_scale_send);   // id 2

  memcpy(scripts.slots[0].code, scriptDemo, scriptDemo_len);
  scripts.slots[0].codeLen = scriptDemo_len;
  vmInit(scripts.slots[0].vm, scripts.slots[0].code, scripts.slots[0].codeLen);
  scripts.slots[0].loaded  = true;
  scripts.slots[0].enabled = false;

  // ── 6. Active mode ───────────────────────────────────────────────────────────
  modeTable[currentMode].setup();
}

// =============================================================================
//  SECTION 9 — MAIN LOOP
// =============================================================================
//
//  processStreams()/processCommands() service SerialUSB1 only, so they never
//  compete with the repeater's Serial <-> downstream-device byte pump.
//  control.run() drives the thread pool, including ADCThread — which runs
//  regardless of the active mode, so it keeps working during REPEATER mode.
//  The active mode's loop() runs last and unconditionally, at full speed,
//  every iteration.

void loop()
{
  cp.processStreams();
  cp.processCommands();
  control.run();
  modeTable[currentMode].loop();
}
