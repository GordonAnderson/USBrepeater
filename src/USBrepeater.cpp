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
// Section 6a (below), alongside the repeater state it shares (userial, and
// the guard/swallow timestamps). ADCThread.onRun(ADCUpdate) is wired up in
// setup() (Section 8), which comes after Section 6a in this file, so no
// forward declaration is needed.
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

static int portLimits[] = {1, 65535};  // range check for ?ETHCTRLPORT / ?ETHDATAPORT

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
  {"GLINK",    CMDfunction,  0, (void *)cmdGetLink,        NULL, "Downstream USB device connection state"},
  {"GBAUD",    CMDfunction,  0, (void *)cmdGetBaud,        NULL, "Baud rate used to talk to the downstream device"},
  {"SBAUD",    CMDfunction,  1, (void *)cmdSetBaud,        NULL, "Set downstream baud rate; needed for Ethernet-only use (no Serial.baud() to follow)"},

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
// GLINK — TRUE if a downstream USB CDC device is currently enumerated.
// -----------------------------------------------------------------------------
extern USBSerial_BigBuffer userial;    // defined in Section 6
extern uint32_t            curBaud;    // defined in Section 6
extern EthernetClient       ethCtrlClient;  // defined in Section 6c
extern EthernetClient       ethDataClient;  // defined in Section 6c

static void cmdGetLink(void)
{
  cp.sendACK(false);
  cp.println((bool)userial);
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
  if ((bool)userial) userial.begin(curBaud);
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
//  SECTION 6 — REPEATER MODE
// =============================================================================
//
//  PC <-> Teensy device port (micro-USB, SerialUSB1 shares the connector) <->
//  Teensy host port (5-pin header) <-> downstream VCP device. As of Section
//  6c, a connected Ethernet client on EthDataPort is a second possible "PC"
//  for this same link — see `pcSide` below.
//
//  Runs unthrottled from loop() — do not add commandProcessor traffic or
//  other blocking work to this path; it must stay fast enough to keep up
//  with the downstream device's baud rate.

USBHost             myusb;
USBHub              hub1(myusb);
USBSerial_BigBuffer userial(myusb);   // handles FS (64 B) and HS (512 B) CDC devices

uint32_t curBaud      = 115200;
bool     wasConnected = false;
uint8_t  repeaterBuf[512];

// Whichever source (Serial, or the Ethernet data client — Section 6c) most
// recently sent bytes to the device. Downstream device -> PC bytes are
// written only here: "last speaker wins", the same pattern
// commandProcessor::processStreams() already uses
// (`serial = streams[i]; // route replies back to the stream that sent the command`).
// This settles *who gets the reply*; it does not stop two sources from
// writing to `userial` in the same loop pass if both have data available at
// once — see Section 6c for why that's accepted as a known edge case rather
// than solved with hard exclusivity.
Stream *pcSide = &Serial;

// State shared with ADCUpdate() (Section 6a) so an injected command doesn't
// collide with in-flight PC<->device traffic. See Section 6a for the full
// explanation of both guards.
uint32_t lastPcToDeviceMs = 0;   // updated whenever PC bytes are forwarded to the device
uint32_t adcSwallowUntilMs = 0;  // while millis() < this, discard device->PC bytes

void RepeaterModeSetup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);       // LED = downstream device connected
  Serial.begin(115200);               // native USB, baud value ignored
  myusb.begin();                      // enables host port VBUS
}

void RepeaterModeLoop(void)
{
  myusb.Task();

  bool connected = userial;

  // Downstream device attach / detach
  if (connected && !wasConnected)
  {
    userial.begin(curBaud);
    digitalWriteFast(LED_BUILTIN, HIGH);
  }
  else if (!connected && wasConnected)
  {
    digitalWriteFast(LED_BUILTIN, LOW);
  }
  wasConnected = connected;

  // Follow the baud rate the PC application sets (matters for FTDI/CP210x/CH340 bridges)
  uint32_t b = Serial.baud();
  if (b != 0 && b != curBaud)
  {
    curBaud = b;
    if (connected) userial.begin(curBaud);
  }

  if (!connected) return;

  // PC -> device. Both Serial and a connected Ethernet data client can drive
  // the link; whichever one actually sends bytes this pass becomes pcSide,
  // so the reply below goes back to whoever just spoke. If both have data in
  // the same pass, both still get forwarded here (in this order) and can
  // interleave on the wire to the downstream device — see Section 6c.
  int n = Serial.available();
  if (n > 0)
  {
    n = Serial.readBytes((char *)repeaterBuf, min(n, (int)sizeof(repeaterBuf)));
    userial.write(repeaterBuf, n);
    lastPcToDeviceMs = millis();   // tells ADCUpdate() the link was just busy
    pcSide = &Serial;
  }

  if (ethDataClient && ethDataClient.connected())
  {
    int n2 = ethDataClient.available();
    if (n2 > 0)
    {
      n2 = ethDataClient.read(repeaterBuf, min(n2, (int)sizeof(repeaterBuf)));
      userial.write(repeaterBuf, n2);
      lastPcToDeviceMs = millis();
      pcSide = &ethDataClient;
    }
  }

  // Device -> PC — written only to pcSide, not broadcast to both.
  n = userial.available();
  if (n > 0)
  {
    n = userial.readBytes((char *)repeaterBuf, min(n, (int)sizeof(repeaterBuf)));
    // Discard bytes that are (most likely) the downstream device's reply to
    // an ADCUpdate()-injected command instead of forwarding them to the PC —
    // see Section 6a. Outside that short window, behavior is unchanged.
    if (millis() >= adcSwallowUntilMs) pcSide->write(repeaterBuf, n);
  }
}

// =============================================================================
//  SECTION 6a — PERIODIC ADC -> DOWNSTREAM-DEVICE COMMAND UPDATE
// =============================================================================
//
//  ADCUpdate() runs on ADCThread (registered in setup()), independent of the
//  active mode, so it keeps working while REPEATER mode is active. It writes
//  directly to `userial` — the same link RepeaterModeLoop() pumps PC<->device
//  traffic over — because that IS the connection to the device being
//  controlled. There is no framing on that raw byte stream, so two guards
//  keep injected commands from colliding with ordinary repeater traffic:
//
//   1. QUIET GUARD (before sending): skip this cycle if the PC forwarded
//      bytes to the device within the last ADC_QUIET_GUARD_MS. Reduces the
//      chance of splicing our own bytes into the middle of a PC-initiated
//      command that's still arriving.
//
//   2. SWALLOW WINDOW (after sending): for ADC_SWALLOW_WINDOW_MS after
//      injecting, RepeaterModeLoop() discards device->PC bytes instead of
//      forwarding them, so the downstream device's reply to our command
//      doesn't show up as unsolicited bytes in the PC application's stream.
//
//  Neither guard is a hard guarantee — a downstream device that replies
//  unusually slowly could see its reply forwarded anyway (swallow window
//  expired), and a PC command that happens to start in the few mS after an
//  injection could still overlap it. They are heuristics sized for typical
//  short ASCII command/reply traffic, not a real bus arbitration protocol.
//  Widen the windows here if your downstream device is slower to reply.

#define ADC_QUIET_GUARD_MS     5   // skip this cycle if the link was busy this recently
#define ADC_SWALLOW_WINDOW_MS  100 // discard downstream traffic for this long after injecting

void ADCUpdate(void)
{
  if (!data.AdcEnabled) return;
  if (!(bool)userial)   return;   // nothing connected to control

  if ((millis() - lastPcToDeviceMs) < ADC_QUIET_GUARD_MS) return;  // link busy, try next cycle

  float counts = readAdcCounts();
  float value  = data.AdcScaleM * counts + data.AdcScaleB;

  char cmdLine[48];
  int  len = snprintf(cmdLine, sizeof(cmdLine), "%s,%.3f\n", data.AdcCmdName, value);
  if (len <= 0) return;
  if (len >= (int)sizeof(cmdLine)) len = sizeof(cmdLine) - 1;  // truncated; still send what fits

  userial.write((uint8_t *)cmdLine, len);
  adcSwallowUntilMs = millis() + ADC_SWALLOW_WINDOW_MS;
}

// =============================================================================
//  SECTION 6b — GAACE_SCRIPT ADC DEMO
// =============================================================================
//
//  Runs gaace_scripts/adc_demo.gs (compiled to include/ScriptBytecode.h — see
//  that file's header for the regeneration command) in slot 0 of `scripts`
//  (the GAACEScript::ScriptRuntime declared in Section 3), alongside
//  ADCThread. This is a real integration, not a standalone toy: the syscalls
//  below drive the actual downstream link (userial), the actual calibration
//  fields (AdcPin/AdcScaleM/AdcScaleB/AdcCmdName), and the same guard/swallow
//  state (lastPcToDeviceMs/adcSwallowUntilMs) ADCUpdate() uses, so the two
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
//    0 = link_ready()      — userial connected AND quiet-guard elapsed
//    1 = read_adc()         — readAdcCounts() rounded to int32 via lroundf()
//                             (the VM is int32-only; see readAdcCounts()'s
//                             own comment in Section 5 for the averaging)
//    2 = scale_send(counts) — AdcScaleM*counts+AdcScaleB, sent downstream

#include "ScriptBytecode.h"

using namespace GAACEScript;

static int32_t sys_link_ready(int32_t *args, uint8_t argc)
{
  (void)args; (void)argc;
  if (!(bool)userial) return 0;                                    // nothing connected
  if ((millis() - lastPcToDeviceMs) < ADC_QUIET_GUARD_MS) return 0; // link busy
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
  float value = data.AdcScaleM * (float)args[0] + data.AdcScaleB;

  char cmdLine[48];
  int  len = snprintf(cmdLine, sizeof(cmdLine), "%s,%.3f\n", data.AdcCmdName, value);
  if (len <= 0) return 0;
  if (len >= (int)sizeof(cmdLine)) len = sizeof(cmdLine) - 1;  // truncated; still send what fits

  userial.write((uint8_t *)cmdLine, len);
  adcSwallowUntilMs = millis() + ADC_SWALLOW_WINDOW_MS;
  return 1;
}

// =============================================================================
//  SECTION 6c — ETHERNET (single downstream device)
// =============================================================================
//
//  Two independent TCP servers, both single-session ("latest connection
//  wins" — same policy every commercial serial-to-Ethernet device server
//  uses, because two live sessions driving the same downstream link would
//  corrupt each other's traffic regardless of arbitration):
//
//    ethCtrlServer / ethCtrlClient — mirrors SerialUSB1: registered with
//      cp.registerStream() once in EthernetSetup(), so every command
//      (GVER, GMODE, SADCxxx, ...) works verbatim over this TCP port with no
//      command-processor changes.
//
//    ethDataServer / ethDataClient — a second passthrough source for
//      RepeaterModeLoop() (Section 6), alongside Serial. See `pcSide` there
//      for how replies are routed back to whichever source spoke most
//      recently.
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
//  RepeaterModeLoop() pass, both still get forwarded to userial that pass
//  and can interleave on the wire. This only bites under sustained
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
//  compete with the repeater's Serial <-> userial byte pump. control.run()
//  drives the thread pool, including ADCThread — which runs regardless of the
//  active mode, so it keeps working during REPEATER mode. The active mode's
//  loop() runs
//  last and unconditionally, at full speed, every iteration.

void loop()
{
  cp.processStreams();
  cp.processCommands();
  control.run();
  modeTable[currentMode].loop();
}
