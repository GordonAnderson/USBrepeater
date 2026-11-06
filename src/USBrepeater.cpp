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
//  selection (see Section 6). On the USB side only one is bridged to the
//  single switchable upstream path (Serial) at a time — data.ActiveDevice,
//  set via SDEV/GDEV — because the hardware doesn't have upstream USB
//  endpoints to spare for more. Ethernet isn't endpoint-limited the same
//  way: each slot also has its own dedicated, always-reachable TCP data
//  port (EthDataPortBase + slot - 1, Section 6c), independent of SDEV.
//  Slots are numbered by board connector (slot N = connector N; a MIPS's
//  RS-485 adapter behind its own hub = N+4), so a slot's number, data port
//  and TCP port stay the same across boots and replugs as long as the
//  cabling does (Section 6, updateSlotMap()). ADCThread/the script demo (below) target a second,
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
#include <strings.h> // strcasecmp() — used to parse TRUE/FALSE args (Section 5, DIO)
#include <LittleFS.h> // LittleFS_Program — script slot persistence on-chip flash (Section 6b)

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

// AIN range — a build-time hardware decision (which input divider/offset
// network is populated on the board), not something firmware can detect or
// change at runtime. Selects the factory-default AdcScaleM/AdcScaleB below
// so a fresh board reads real volts out of the box instead of raw counts
// until someone manually calibrates it; SADCM/SADCB can always override
// live afterward, same as today. Starting-point values per the carrier-
// board design guide's §5.6/§5.6.2 transfer functions (12-bit-adjusted) —
// confirm against the actual populated range before trusting them on a
// specific board. Define exactly one of these two in platformio.ini
// build_flags; BIPOLAR is the default if neither is defined, since it's
// the one consulted most often in this project's hardware docs so far —
// not a claim about which variant any particular board actually has.
#if defined(AIN_RANGE_UNIPOLAR)
  #define AIN_DEFAULT_SCALE_M  0.002705f   // 0-10V range
  #define AIN_DEFAULT_SCALE_B -0.496f
#else
  #define AIN_DEFAULT_SCALE_M  0.005348f   // +/-10V range (default)
  #define AIN_DEFAULT_SCALE_B -10.96f
#endif

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

  0,              // ActiveDevice    — 0 = automatic: lowest occupied slot (see effectiveDev())
  0,              // ActiveDevice2   — 0 = automatic: next occupied slot after data port 1's
  1,              // AdcTargetDevice — independent selection, no exclusivity constraint

  false,          // AdcEnabled  — off until a pin/command/scale is configured
  A0,             // AdcPin
  1000,           // AdcInterval — 1000 mS
  "SETVAL",       // AdcCmdName  — placeholder; set to match the downstream command
  AIN_DEFAULT_SCALE_M,  // AdcScaleM — range-dependent default (AIN_RANGE_UNIPOLAR/BIPOLAR above)
  AIN_DEFAULT_SCALE_B,  // AdcScaleB

  0.0f,           // RefCal          — correction disabled until CALREF is run
  0.0f,           // AdcFilterTauMs  — low-pass disabled; raw averaged passthrough

  true,           // EthDHCP      — DHCP by default, zero-config on a typical LAN
  0,              // EthIP        — 0.0.0.0; set explicitly before switching off DHCP
  0x00FFFFFF,     // EthSubnet    — 255.255.255.0 (packed as IPAddress(255,255,255,0))
  0,              // EthGateway   — 0.0.0.0; set explicitly before switching off DHCP
  5000,           // EthCtrlPort     — TCP control port (same commands as SerialUSB1)
  5001,           // EthDataPortBase — device 1's TCP passthrough port; devices 2-4 get 5002-5004

  false,          // DioOutputEnabled — input/high-Z by default (hardware already pulls OE_N disabled on its own)

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
// RepeaterModeLoop() (slotDrv[]/devPort[], Section 6). ADCThread.onRun(
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
static void cmdGetLink2(void);
static void cmdGetAdcInterval(void);
static void cmdSetAdcInterval(void);
static void cmdGetAdc(void);
static void cmdCalRef(void);
static void cmdGetRefCal(void);
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
static void cmdGetDev2(void);
static void cmdSetDev2(void);
static void cmdGetAdcDev(void);
static void cmdSetAdcDev(void);
static void cmdGetDevStat(void);
void        cmdGetDevPath(void);   // Section 6 (needs the USB driver objects)
void        cmdGetDevList(void);   // Section 6 (device names, GNAME on attach)
void        cmdGetDevSlot(void);   // Section 6
void        cmdDevIdent(void);     // Section 6
void        cmdGetArbStat(void);   // Section 6 (per-slot arbiter)
static void cmdGetDioDir(void);
static void cmdSetDioDir(void);
static void cmdGetDout(void);
static void cmdSetDout(void);
static void cmdScriptSave(void);
static void cmdScriptRestore(void);
static void cmdScriptErase(void);
static void cmdGetAdcCmd(void);
static void cmdSetAdcCmd(void);
static void cmdGetDin(void);

static int portLimits[] = {1, 65535};  // range check for ?ETHCTRLPORT / ?ETHDATAPORT

// Per-port accessors — implemented in Section 6, where userial1..userial6
// and their ring buffers actually live. Keeps these command handlers (and
// GLINK/SBAUD above) from needing extern access to the raw per-port arrays.
// dev1based is in [1, DEVPORT_COUNT]; callers here always pass a value
// already range-checked by cp.getValue() or a persisted Data field.
bool     isDeviceConnected(uint8_t dev1based);
uint8_t  effectiveDev(uint8_t which);   // data port 1 or 2 -> slot it's bridged to now (1-based, 0 = none)
uint16_t deviceBytesWaiting(uint8_t dev1based);
uint32_t deviceBytesDropped(uint8_t dev1based);
void     reinitDeviceBaud(uint8_t dev1based, uint32_t baud);

// DIO accessors — implemented in Section 6d, shared by the control-port
// handlers below and the script syscalls (sys_dio_*). dioOutLevel mirrors
// curBaud's extern-declared-here/defined-there pattern just above.
void     dioSetDirection(bool outputEnabled);
void     dioWrite(bool level);
bool     dioRead(void);
extern bool dioOutLevel;

// Script-slot flash persistence — implemented in Section 6b, where the
// GAACE_Script runtime (`scripts`) lives. Plain integer signatures here (no
// GAACEScript:: types) so these can be forward-declared before Section 6b's
// `using namespace GAACEScript;` takes effect.
enum ScriptRestoreResult : uint8_t
{
  SCRIPT_RESTORE_OK,      // slot restored and loaded
  SCRIPT_RESTORE_NONE,    // nothing saved for this slot, or the file didn't validate
  SCRIPT_RESTORE_STALE    // saved against different syscalls than this firmware has
};
bool                saveScriptSlotToFlash(uint8_t slot);
ScriptRestoreResult restoreScriptSlotFromFlash(uint8_t slot);
bool                eraseScriptSlotFromFlash(uint8_t slot);

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
  //  SDEV switches which port Serial (the first switchable USB upstream
  //  path) bridges to; SDEV2 is the identical thing for SerialUSB2 (the
  //  second switchable path). Each port also has its own always-reachable
  //  Ethernet data port, unaffected by either — see Section 6c. Every port
  //  keeps capturing into its own ring buffer whether selected or not, so
  //  switching never loses what a non-selected device said — see Section 6.
  //  SDEV and SDEV2 are kept mutually exclusive: setting one to the other's
  //  current value is rejected (NAK) — see Section 6's header comment for
  //  why. SADCDEV is a completely independent selection for ADCThread/the
  //  script demo, so background monitoring of one device isn't interrupted
  //  by switching either port's interactive focus.
  {"GDEV",     CMDfunction, -1, (void *)cmdGetDev,         NULL, "Get Serial's active downstream device, " DEVPORT_RANGE_STR},
  {"SDEV",     CMDfunction, -1, (void *)cmdSetDev,         NULL, "Set Serial's active downstream device, " DEVPORT_RANGE_STR " (must differ from ActiveDevice2)"},
  {"GDEV2",    CMDfunction, -1, (void *)cmdGetDev2,        NULL, "Get SerialUSB2's active downstream device, " DEVPORT_RANGE_STR},
  {"SDEV2",    CMDfunction, -1, (void *)cmdSetDev2,        NULL, "Set SerialUSB2's active downstream device, " DEVPORT_RANGE_STR " (must differ from ActiveDevice)"},
  {"GADCDEV",  CMDfunction, -1, (void *)cmdGetAdcDev,      NULL, "Get ADCThread/script target device, " DEVPORT_RANGE_STR},
  {"SADCDEV",  CMDfunction, -1, (void *)cmdSetAdcDev,      NULL, "Set ADCThread/script target device, " DEVPORT_RANGE_STR},
  {"GDEVSTAT", CMDfunction,  0, (void *)cmdGetDevStat,     NULL, "Per-port status: dev,connected,bytesWaiting,bytesDropped"},
  {"GDEVPATH", CMDfunction,  0, (void *)cmdGetDevPath,     NULL, "Diagnostic: each slot's board connector, inner-hub port, VID/PID, USB addresses; then each hub"},
  {"GDEVLIST", CMDfunction,  0, (void *)cmdGetDevList,     NULL, "Occupied slots: slot,connector,MAIN|EXTRA,vid,pid,state,name (name from GNAME on attach)"},
  {"GDEVSLOT", CMDfunction,  1, (void *)cmdGetDevSlot,     NULL, "Slot (" DEVPORT_RANGE_STR ") of the device whose GNAME is <name>, NAK if none"},
  {"DEVIDENT", CMDfunction,  1, (void *)cmdDevIdent,       NULL, "Ask slot <n>'s device its GNAME again (also queries an extra slot 5-8)"},
  {"GARBSTAT", CMDfunction,  0, (void *)cmdGetArbStat,     NULL, "Per-slot arbiter: slot,busy,owner,exchanges,timeouts"},
  {"GLINK2",   CMDfunction,  0, (void *)cmdGetLink2,       NULL, "ActiveDevice2's USB connection state"},

  // ── Ethernet (control port + one dedicated data port per device; Section 6c) ─
  //  EthCtrlPort mirrors every SerialUSB1 command over TCP. EthDataPortBase
  //  is slot 1's own dedicated passthrough TCP port; slots 2-8 get
  //  EthDataPortBase+1..+7 — independent, always-reachable data ports, not
  //  one switchable port like Serial. A device reachable from both a USB
  //  data port and its TCP port is shared through its per-slot arbiter
  //  (Section 6): one command at a time, each reply back to its sender.
  //  EthDHCP/EthIP/EthMask/EthGw/EthDataPortBase take
  //  effect on the next boot, not live; GETHSTAT always reflects live state.
  {"?ETHDHCP",     CMDbool,     -1, (void *)&data.EthDHCP,         NULL, "Use DHCP (TRUE) or static IP (FALSE); reboot to apply"},
  {"GETHIP",       CMDfunction, -1, (void *)cmdGetEthIp,           NULL, "Static IP address a.b.c.d; reboot to apply"},
  {"SETHIP",       CMDfunction, -1, (void *)cmdSetEthIp,           NULL, "Set static IP address a.b.c.d; reboot to apply"},
  {"GETHMASK",     CMDfunction, -1, (void *)cmdGetEthMask,         NULL, "Static subnet mask a.b.c.d; reboot to apply"},
  {"SETHMASK",     CMDfunction, -1, (void *)cmdSetEthMask,         NULL, "Set static subnet mask a.b.c.d; reboot to apply"},
  {"GETHGW",       CMDfunction, -1, (void *)cmdGetEthGw,           NULL, "Static gateway a.b.c.d; reboot to apply"},
  {"SETHGW",       CMDfunction, -1, (void *)cmdSetEthGw,           NULL, "Set static gateway a.b.c.d; reboot to apply"},
  {"?ETHCTRLPORT", CMDint,      -1, (void *)&data.EthCtrlPort,     portLimits, "TCP port for the network control port; reboot to apply"},
  {"?ETHDATAPORT", CMDint,      -1, (void *)&data.EthDataPortBase, portLimits, "TCP base port for per-device passthrough: device N uses base+(N-1), N=" DEVPORT_RANGE_STR "; reboot to apply"},
  {"GETHSTAT",     CMDfunction,  0, (void *)cmdGetEthStat,         NULL, "Live status: link,mode,ip,ctrlConnected,dataConnected1,dataConnected2,dataConnected3,dataConnected4"},

  // ── Periodic ADC -> downstream-device command update ───────────────────────
  //  Runs on ADCThread regardless of the active mode. Each cycle sends
  //  "<AdcCmdName>,<AdcScaleM * counts + AdcScaleB>\n" to the downstream
  //  device. Configure AdcCmdName to match a command the downstream device
  //  actually accepts before enabling with SADCEN,TRUE.
  {"?ADCEN",   CMDbool,     -1, (void *)&data.AdcEnabled,  NULL, "ADC update enabled, TRUE or FALSE"},
  {"?ADCPIN",  CMDbyte,     -1, (void *)&data.AdcPin,      NULL, "ADC input pin (e.g. 14 for A0)"},
  {"GADCINT",  CMDfunction, -1, (void *)cmdGetAdcInterval, NULL, "Get ADC update interval, mS"},
  {"SADCINT",  CMDfunction, -1, (void *)cmdSetAdcInterval, NULL, "Set ADC update interval, mS"},
  {"GADCCMD",  CMDfunction,  0, (void *)cmdGetAdcCmd,      NULL, "Get the command prefix sent to the downstream device"},
  {"SADCCMD",  CMDfunction, -1, (void *)cmdSetAdcCmd,      NULL, "Set the command prefix, name[,fixed args...]; e.g. SADCCMD,SRFADRV,1 sends SRFADRV,1,<value>"},
  {"?ADCM",    CMDfloat,    -1, (void *)&data.AdcScaleM,   NULL, "ADC scale slope: value = M * counts + B"},
  {"?ADCB",    CMDfloat,    -1, (void *)&data.AdcScaleB,   NULL, "ADC scale offset: value = M * counts + B"},
  {"?ADCFILTER", CMDfloat,  -1, (void *)&data.AdcFilterTauMs, NULL, "AIN low-pass time constant, mS (0 = disabled); tunable live, no reflash"},
  {"CALREF",   CMDfunction,  0, (void *)cmdCalRef,        NULL, "Capture the current REF reading into RefCal (then SAVE to persist)"},
  {"GREFCAL",  CMDfunction,  0, (void *)cmdGetRefCal,      NULL, "Get RefCal; 0 = REF ratiometric correction disabled"},
  {"GADC",     CMDfunction,  0, (void *)cmdGetAdc,         NULL, "Read ADC now: averaged+filtered counts (2dp) and scaled value (4dp)"},

  // ── Digital I/O (DIO, SMA connector; see Section 6d) ────────────────────
  //  One external line, independent of the 4 downstream ports. Direction
  //  and level are function pairs, not plain CMDbool fields, because
  //  setting either has an immediate hardware side effect (driving
  //  DIO_OE_N / DIO_OUT) a plain field write can't do on its own. Scripts
  //  get the identical operations via dio_set_dir()/dio_write()/dio_read()
  //  syscalls (Section 6b/8) through the same underlying functions, so the
  //  control port and scripts can never see it differently.
  {"GDIODIR",  CMDfunction, -1, (void *)cmdGetDioDir,      NULL, "Get DIO direction: TRUE=output, FALSE=input (power-up default)"},
  {"SDIODIR",  CMDfunction, -1, (void *)cmdSetDioDir,      NULL, "Set DIO direction, TRUE|FALSE"},
  {"GDOUT",    CMDfunction, -1, (void *)cmdGetDout,        NULL, "Get DIO output level (meaningful only while direction is output)"},
  {"SDOUT",    CMDfunction, -1, (void *)cmdSetDout,        NULL, "Set DIO output level, TRUE|FALSE"},
  {"GDIN",     CMDfunction,  0, (void *)cmdGetDin,         NULL, "Read DIO line level now (valid in either direction)"},

  // ── GAACE_Script ADC demo (Section 6b) ──────────────────────────────────
  //  SCRIPTLOAD/GSCRIPTLIMITS/GSCRIPTST come from GAACEScriptRuntime.h
  //  (registered in setup() via cp.registerCommands(scripts.scriptCmdList())).
  //  Starting/stopping the demo (slot "Script0") and changing its rate use
  //  GAACE_Core's own threadCommands instead of a dedicated command here:
  //  GTENA,Script0 / STENA,Script0,TRUE|FALSE and GTINT,Script0 /
  //  STINT,Script0,<mS>. Reuses ?ADCPIN/?ADCM/?ADCB/?ADCCMD above. Don't
  //  enable both Script0 and ADCEN against the same downstream command —
  //  they'd double-send.

  // ── Script slot persistence (flash; see Section 6b) ────────────────────
  //  USBrepeater-specific, not part of GAACE_Script itself (that library
  //  stays hardware-agnostic — LittleFS_Program is Teensy-only). Every
  //  slot is attempted at boot (setup(), Section 8), overriding the
  //  compiled-in ADC demo for slot 0 if slot 0 has a saved file.
  {"SCRIPTSAVE",    CMDfunction, -1, (void *)cmdScriptSave,    NULL, "SCRIPTSAVE,<slot> -- save a loaded script's buffer to flash"},
  {"SCRIPTRESTORE", CMDfunction, -1, (void *)cmdScriptRestore, NULL, "SCRIPTRESTORE,<slot> -- reload a slot's buffer from flash"},
  {"SCRIPTERASE",   CMDfunction, -1, (void *)cmdScriptErase,   NULL, "SCRIPTERASE,<slot> -- delete a slot's saved copy from flash (RAM copy untouched)"},

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
extern uint32_t       curBaud;                        // defined in Section 6
extern EthernetClient ethCtrlClient;                  // defined in Section 6c
extern EthernetClient ethDataClient[DEVPORT_COUNT];   // defined in Section 6c

static void cmdGetLink(void)
{
  cp.sendACK(false);
  cp.println(isDeviceConnected(effectiveDev(1)));
}

// GLINK2 — same as GLINK, for SerialUSB2's selection (ActiveDevice2).
static void cmdGetLink2(void)
{
  cp.sendACK(false);
  cp.println(isDeviceConnected(effectiveDev(2)));
}

// -----------------------------------------------------------------------------
// GDEV / SDEV,<n> and GDEV2 / SDEV2,<n> — get/set which downstream port
// (1-DEVPORT_COUNT) Serial / SerialUSB2 (the two switchable USB upstream paths) are
// each bridged to. Does not affect Ethernet: each port has its own
// dedicated, always-reachable data port there (Section 6c), independent of
// either selection. The newly-selected port's ring buffer (Section 6)
// starts flushing to Serial/SerialUSB2 on the very next RepeaterModeLoop()
// pass — no separate "flush on switch" step is needed, since the loop
// always flushes each selected port's backlog to whoever last spoke to it.
//
// ActiveDevice and ActiveDevice2 are kept mutually exclusive — SDEV/SDEV2
// NAK a value that would make them equal. See Section 6's header comment
// for why: it keeps each device's upstream arbitration to at most one
// switchable USB port plus its own Ethernet client, never both switchable
// ports at once.
//
// 0 = automatic (the factory default): data port 1 follows the lowest
// occupied slot, data port 2 the next one (see effectiveDev(), Section 6).
// Slots are numbered by connector, so with two devices plugged in the two
// data ports reach them with no setup, whichever connectors they're in.
// A non-zero value pins the port to that slot. GDEV/GDEV2 reply
// "<setting>,<effective slot>" (effective 0 = nothing to talk to).
// -----------------------------------------------------------------------------
static void printDevSelection(uint8_t setting, uint8_t which)
{
  cp.sendACK(false);
  cp.print((int)setting);
  cp.print(",");
  cp.println((int)effectiveDev(which));
}

static void cmdGetDev(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  printDevSelection(data.ActiveDevice, 1);
}

static void cmdSetDev(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int n;
  if (!cp.getValue(&n, 0, DEVPORT_COUNT) || (n != 0 && n == (int)data.ActiveDevice2))
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  data.ActiveDevice = (uint8_t)n;
  cp.sendACK();
}

static void cmdGetDev2(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  printDevSelection(data.ActiveDevice2, 2);
}

static void cmdSetDev2(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int n;
  if (!cp.getValue(&n, 0, DEVPORT_COUNT) || (n != 0 && n == (int)data.ActiveDevice))
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  data.ActiveDevice2 = (uint8_t)n;
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GADCDEV / SADCDEV,<n> — get/set which downstream port (1-DEVPORT_COUNT) ADCThread and
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
  reinitDeviceBaud(effectiveDev(1), curBaud);
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
// GETHSTAT — live network status: link,mode,ip,ctrlConnected, then one
// dataConnected flag per downstream device (1-DEVPORT_COUNT, each its own dedicated
// Ethernet data port — Section 6c). Always reflects current runtime state
// (unlike the config getters above, which read back the stored, possibly-
// not-yet-applied, EthIP/EthMask/EthGw).
// -----------------------------------------------------------------------------
static void cmdGetEthStat(void)
{
  if (!cp.checkExpectedArgs(0)) return;

  cp.sendACK(false);
  cp.print(Ethernet.linkState() ? "LINKUP" : "LINKDOWN"); cp.print(",");
  cp.print(data.EthDHCP ? "DHCP" : "STATIC");             cp.print(",");
  printIpAddr(Ethernet.localIP());                        cp.print(",");
  cp.print((bool)ethCtrlClient && ethCtrlClient.connected());
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    cp.print(",");
    cp.print((bool)ethDataClient[i] && ethDataClient[i].connected());
  }
  cp.print();   // bare newline — see commandProcessor.h; there's no println(void)
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
// readAveragedCounts — hardware oversampling (analogReadAveraging(),
// configured once in setup() alongside analogReadResolution()) plus a
// software average on top, per the carrier-board design guide §5.6.2.
// Shared by readAdcCounts() (AIN, data.AdcPin) and readRefCounts() (REF,
// AIN_REF_PIN below) so both channels get identical averaging behavior.
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

#define AIN_REF_PIN  A1   // ISL21070CIH325Z 2.5V precision reference (confirmed via netlist, TODO.md)

static float readAveragedCounts(uint8_t pin)
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SW_SAMPLES; i++)
  {
    sum += analogRead(pin);
  }
  return (float)sum / (float)ADC_SW_SAMPLES;
}

// -----------------------------------------------------------------------------
// applyAdcLowPass — a dt-corrected exponential moving average, applied on
// top of readAveragedCounts() to smooth noise *between* reads (which
// oversampling within a single read cannot). data.AdcFilterTauMs is the
// time constant in mS, tunable live via ?ADCFILTER — no reflash needed.
// <= 0 disables it (raw averaged passthrough, today's original behavior).
//
// dt-corrected rather than a fixed per-call alpha because readAdcCounts()
// is called both periodically (ADCThread's own interval) and on demand
// (GADC, whenever a host asks) — computing alpha from the actual elapsed
// time keeps the filter's real-world time constant correct regardless of
// which caller triggers a given read, rather than drifting with call rate.
// -----------------------------------------------------------------------------
static float  adcFilterState   = 0.0f;
static bool   adcFilterInit    = false;
static uint32_t adcFilterLastMs = 0;

static float applyAdcLowPass(float raw)
{
  if (data.AdcFilterTauMs <= 0.0f)
  {
    adcFilterInit = false;   // disabled; re-seed cleanly from raw if re-enabled later
    return raw;
  }

  uint32_t now = millis();
  if (!adcFilterInit)
  {
    adcFilterState  = raw;   // seed from the first sample -- no startup transient from 0
    adcFilterLastMs = now;
    adcFilterInit   = true;
    return raw;
  }

  uint32_t dtMs = now - adcFilterLastMs;
  if (dtMs == 0) return adcFilterState;   // called again within the same mS; nothing new to incorporate

  adcFilterLastMs = now;
  float alpha = (float)dtMs / (data.AdcFilterTauMs + (float)dtMs);
  adcFilterState += alpha * (raw - adcFilterState);
  return adcFilterState;
}

// readAdcCounts — single point of truth for reading the AIN channel.
// Every reader — ADCUpdate(), GADC, and the script read_adc() syscall —
// goes through this, so there's exactly one averaging+filtering behavior
// to reason about. Returns counts as a float (not rounded) so
// scaleAdcValue() keeps full precision; a caller that needs an integer
// (sys_read_adc(), Section 6b — the VM is int32) rounds it itself.
static float readAdcCounts(void)
{
  return applyAdcLowPass(readAveragedCounts(data.AdcPin));
}

// readRefCounts — same averaging as readAdcCounts(), on the REF channel.
// Deliberately NOT run through applyAdcLowPass(): REF is a slowly-drifting
// calibration correction factor, not a noisy signal to track quickly, so
// plain averaging is the right amount of smoothing for it.
static float readRefCounts(void)
{
  return readAveragedCounts(AIN_REF_PIN);
}

// -----------------------------------------------------------------------------
// scaleAdcValue — single point of truth for turning AIN counts into a real
// value: value = AdcScaleM*counts + AdcScaleB, then the REF-channel
// ratiometric correction (carrier-board design guide §5.6.1) if a RefCal
// has been captured: value *= (RefCal / REF_now). RefCal <= 0 means CALREF
// has never been run — correction stays disabled, plain M/B passthrough.
// Used by cmdGetAdc() (GADC), ADCUpdate(), and sys_scale_send() (Section
// 6b) so all three apply the correction identically once it's configured.
// -----------------------------------------------------------------------------
static float scaleAdcValue(float counts)
{
  float value = data.AdcScaleM * counts + data.AdcScaleB;

  if (data.RefCal > 0.0f)
  {
    float refNow = readRefCounts();
    if (refNow > 0.0f) value *= (data.RefCal / refNow);
  }

  return value;
}

// -----------------------------------------------------------------------------
// GADC — read the ADC pin right now and report both the averaged+filtered
// counts and the scaled (and, if configured, REF-corrected) value, without
// waiting for the next ADCThread cycle or sending anything to the
// downstream device. Useful for checking AdcScaleM/AdcScaleB before
// enabling ADCEN.
// -----------------------------------------------------------------------------
static void cmdGetAdc(void)
{
  float counts = readAdcCounts();
  float value  = scaleAdcValue(counts);

  cp.sendACK(false);
  cp.print(counts, 2);
  cp.print(",");
  cp.println(value, 4);
}

// -----------------------------------------------------------------------------
// CALREF — capture the REF channel's current averaged reading into
// data.RefCal, enabling the ratiometric correction in scaleAdcValue() from
// this point on (live; SAVE to persist). Run this once, with the REF input
// in its known-good state, as part of initial calibration.
// -----------------------------------------------------------------------------
static void cmdCalRef(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  data.RefCal = readRefCounts();
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GREFCAL — read back the captured RefCal. 0 means CALREF has never been
// run (or was explicitly cleared via RESTORE to factory defaults) and the
// REF correction is disabled.
// -----------------------------------------------------------------------------
static void cmdGetRefCal(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println(data.RefCal, 2);
}

// -----------------------------------------------------------------------------
// parseBoolArg — shared TRUE/FALSE token parser for the DIO function-pair
// commands below. Mirrors commandProcessor.cpp's own CMDbool case
// (tokenS == "TRUE"/"FALSE") but case-insensitively and via a plain
// cp.getValue(char**) token, since these are CMDfunction handlers (a
// side-effecting direction/level change, not a bare field write) rather than
// CMDbool fields.
// -----------------------------------------------------------------------------
static bool parseBoolArg(bool *out)
{
  char *tok;
  if (!cp.getValue(&tok)) return false;

  bool ok = true;
  // "1"/"0" accepted too, matching GAACE_Core's CMDbool, so a script's
  // cmd("SDIODIR", 1) works (cmd() can only send numbers).
  if      (strcasecmp(tok, "TRUE")  == 0 || strcmp(tok, "1") == 0) *out = true;
  else if (strcasecmp(tok, "FALSE") == 0 || strcmp(tok, "0") == 0) *out = false;
  else ok = false;

  cp.ca->free(tok);
  return ok;
}

// -----------------------------------------------------------------------------
// GDIODIR / SDIODIR,<TRUE|FALSE> — get/set the DIO line's direction.
// dioSetDirection() (Section 6d) is the single point of truth, shared with
// the dio_set_dir() script syscall, so the control port and a script can
// never leave the hardware and data.DioOutputEnabled disagreeing about which
// direction is actually active.
// -----------------------------------------------------------------------------
static void cmdGetDioDir(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println(data.DioOutputEnabled);
}

static void cmdSetDioDir(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  bool v;
  if (!parseBoolArg(&v)) { cp.sendNAK(ERR_BADARG); return; }

  dioSetDirection(v);
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GDOUT / SDOUT,<TRUE|FALSE> — get/set the DIO output level. Only reaches
// the external line while direction is output (GDIODIR TRUE); setting it
// while direction is input is accepted but has no external effect until the
// direction is switched.
// -----------------------------------------------------------------------------
static void cmdGetDout(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println(dioOutLevel);
}

static void cmdSetDout(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  bool v;
  if (!parseBoolArg(&v)) { cp.sendNAK(ERR_BADARG); return; }

  dioWrite(v);
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// GDIN — read the DIO line's actual level right now. Valid in either
// direction: with output enabled, comparing this against GDOUT's last-set
// value can reveal a shorted or contended line.
// -----------------------------------------------------------------------------
static void cmdGetDin(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println(dioRead());
}

// -----------------------------------------------------------------------------
// SCRIPTSAVE,<slot> / SCRIPTRESTORE,<slot> — save/reload a script slot's
// buffer to/from on-chip flash (Section 6b). ERR_BADARG for a bad slot
// index or (SCRIPTSAVE only) a slot that isn't currently loaded —
// consistent with SCRIPTLOAD/SCRIPTCOMMIT's own range checks above.
// ERR_NOSDCARD for "no persisted file exists to restore" (SCRIPTRESTORE
// only), same reuse RestoreSettings() (Section 7) already makes of that
// code for "no valid record has ever been saved".
// -----------------------------------------------------------------------------
static void cmdScriptSave(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  // Not cp.getValue(&slot, 0, GAACE_SCRIPT_SLOTS - 1): commandProcessor
  // treats ll==0 && ul==0 as "no range check" (see GAACEScriptRuntime.cpp's
  // identical note), which would silently disable bounds checking whenever
  // GAACE_SCRIPT_SLOTS==1.
  int slot;
  if (!cp.getValue(&slot) || slot < 0 || slot >= GAACE_SCRIPT_SLOTS)
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  if (!saveScriptSlotToFlash((uint8_t)slot))
  {
    cp.sendNAK(ERR_BADARG);   // not currently loaded, or the write itself failed
    return;
  }
  cp.sendACK();
}

static void cmdScriptRestore(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int slot;   // see cmdScriptSave()'s comment above for why this isn't cp.getValue(&slot, 0, ...)
  if (!cp.getValue(&slot) || slot < 0 || slot >= GAACE_SCRIPT_SLOTS)
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }

  ScriptRestoreResult result = restoreScriptSlotFromFlash((uint8_t)slot);
  if (result == SCRIPT_RESTORE_STALE)
  {
    cp.sendNAK(ERR_BADARG);     // saved against different syscalls: recompile + re-upload
    return;
  }
  if (result != SCRIPT_RESTORE_OK)
  {
    cp.sendNAK(ERR_NOSDCARD);   // no saved file for this slot (or it failed to parse)
    return;
  }
  cp.sendACK();
}

// GADCCMD / SADCCMD,<name>[,<fixed arg>...] — the prefix ADCUpdate() and the
// scale_send() syscall put before ",<value>". Extra arguments are joined back
// with commas, so a channel-indexed downstream command works:
// SADCCMD,SRFADRV,1 -> "SRFADRV,1,<value>". (A plain CMDstr accepts exactly
// one token, which made such commands impossible to configure.)
static void cmdGetAdcCmd(void)
{
  cp.sendACK(false);
  cp.println(data.AdcCmdName);
}

static void cmdSetAdcCmd(void)
{
  int numArgs = cp.getNumArgs();
  if (numArgs < 1) { cp.sendNAK(ERR_BADARG); return; }

  char   buf[sizeof(data.AdcCmdName)];
  size_t used = 0;
  bool   ok   = true;
  for (int i = 0; i < numArgs && ok; i++)
  {
    char *tok;
    if (!cp.getValue(&tok)) { ok = false; break; }
    size_t len = strlen(tok);
    size_t sep = (i > 0) ? 1 : 0;
    if (len == 0 || used + sep + len >= sizeof(buf)) ok = false;
    else
    {
      if (sep) buf[used++] = ',';
      memcpy(buf + used, tok, len);
      used += len;
    }
    cp.ca->free(tok);
  }
  if (!ok) { cp.sendNAK(ERR_BADARG); return; }

  buf[used] = '\0';
  strcpy(data.AdcCmdName, buf);
  cp.sendACK();
}

// SCRIPTERASE,<slot> — delete a slot's saved copy so it is no longer
// restored (and auto-enabled) at boot. Only the flash copy: whatever is
// loaded in RAM keeps running until disabled or overwritten. ACKs when there
// was nothing saved too, so it's safe to call unconditionally.
static void cmdScriptErase(void)
{
  if (!cp.checkExpectedArgs(1)) return;

  int slot;   // see cmdScriptSave()'s comment above for why this isn't cp.getValue(&slot, 0, ...)
  if (!cp.getValue(&slot) || slot < 0 || slot >= GAACE_SCRIPT_SLOTS)
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }
  if (!eraseScriptSlotFromFlash((uint8_t)slot))
  {
    cp.sendNAK(ERR_NOSDCARD);   // filesystem not mounted, or a file couldn't be removed
    return;
  }
  cp.sendACK();
}

// =============================================================================
//  SECTION 6 — REPEATER MODE (DEVPORT_COUNT device slots, 4-connector Hub Carrier board)
// =============================================================================
//
//  PC <-> Teensy device port (micro-USB, SerialUSB1 shares the connector) <->
//  Teensy host port (5-pin header) <-> USB2514B hub <-> one of 4 downstream
//  USB-A ports. There are now TWO independently switchable USB upstream
//  paths (Serial/SDEV and SerialUSB2/SDEV2 — USB_TRIPLE_SERIAL), plus, as of
//  Section 6c, each slot's own dedicated Ethernet data port. Everything
//  that talks to a device (a USB data port, its TCP client, the repeater's
//  own ADC/script sends) goes through that slot's arbiter: one command line
//  at a time, its reply routed back to whoever sent it (see PER-SLOT
//  ARBITER below). ActiveDevice and ActiveDevice2 are still kept mutually
//  exclusive (enforced at SDEV/SDEV2 time, Section 5).
//
//  TOPOLOGY
//  --------
//  All 4 downstream ports stay connected/auto-reconnecting continuously,
//  independent of which one(s) the host is looking at
//  (devPortUpdateConnection() below runs on all DEVPORT_COUNT every tick). Each port has
//  its own ring buffer (devPort[i].rxbuf) that captures incoming bytes
//  continuously too — so a device that chatters unsolicited while not
//  selected by anything is never silently lost. Only ONE port at a time can
//  be bridged to Serial, and only one (a different one) to SerialUSB2 — the
//  hardware's 3-CDC-port ceiling means two switchable USB paths is the most
//  available (see TODO.md). data.ActiveDevice/ActiveDevice2 (1-DEVPORT_COUNT, SDEV/GDEV,
//  SDEV2/GDEV2) select them. Ethernet isn't endpoint-limited the same way —
//  every port also has its own dedicated, always-reachable Ethernet data
//  port (Section 6c), independent of either USB selection.
//
//  SLOTS FOLLOW CONNECTORS
//  -----------------------
//  USBHost_t36 hands attached devices to the serial driver objects
//  (userial1..8) in its own claim order. updateSlotMap() then files each
//  one under the slot its board connector dictates (devicePath() reads the
//  connector from the device's hub chain), and everything else here works
//  in slots via slotSerial(). The data ports default to automatic (SDEV/
//  SDEV2 = 0): lowest occupied slot, then the next (resolveDataPorts()).
//  This reverses the earlier "the host maps slots itself" design — claim
//  order made slot numbers depend on boot timing (see TODO.md).
//
//  RING BUFFER / FLUSH MODEL
//  -------------------------
//  Every tick: drain available bytes from ALL DEVPORT_COUNT ports into their own ring
//  buffers (devPortDrainIncoming()), unconditionally. Every port is then
//  also flushed every tick (devPortFlush()) to whatever arbOutput() says:
//  the owner of the open exchange, or for unsolicited output the last
//  owner, else the USB data port bridged to the slot, else its TCP client,
//  else it stays buffered (connecting to a slot's TCP port is this design's
//  equivalent of "SDEV switches to it" for delivering backlog). This one
//  mechanism covers both live forwarding
//  (a connected port's buffer drains about as fast as it fills, so latency
//  in steady state is the same as direct passthrough) and "deliver the
//  backlog" the moment a port starts actually being read — nothing special
//  has to happen at that moment, because the flush step doesn't care
//  whether the backlog built up over one tick or a thousand.
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

// USBHost_t36 keeps each driver's Device_t (which records the hub address
// and hub port a device was enumerated on) protected. These two thin
// subclasses only expose it read-only, so devicePath() below can work out
// which board connector a device is plugged into. No behavior change.
class TrackedHub : public USBHub
{
public:
  TrackedHub(USBHost &host) : USBHub(host) {}
  const Device_t *dev() { return *(Device_t * volatile *)&device; }
};

class SlotSerial : public USBSerial_BigBuffer
{
public:
  SlotSerial(USBHost &host, uint16_t minRxTx) : USBSerial_BigBuffer(host, minRxTx) {}
  const Device_t *dev() { return *(Device_t * volatile *)&device; }
};

// One USBHub driver instance is needed per physical hub in the chain, not
// just the board's own USB2514B (hub1): a downstream device with its own
// internal hub (e.g. MIPS units built around an FE1.1s, VID 1A40) is
// otherwise left unclaimed and nothing behind it ever enumerates. hub2-hub5
// cover one nested hub per downstream port.
TrackedHub          hub1(myusb);
TrackedHub          hub2(myusb);
TrackedHub          hub3(myusb);
TrackedHub          hub4(myusb);
TrackedHub          hub5(myusb);
TrackedHub         *hubPtr[] = { &hub1, &hub2, &hub3, &hub4, &hub5 };
#define HUB_COUNT (sizeof(hubPtr) / sizeof(hubPtr[0]))
// min_rxtx = 1, not the library default of 65: the default makes BigBuffer
// claim only high-speed devices (>64-byte bulk endpoints), silently
// rejecting every full-speed one -- SAMD/TinyUSB boards like the
// IsoElectrometer, and most FTDI/CP210x/CH340 adapters.
//
// These are USB *driver objects*, handed out by USBHost_t36 in its own claim
// order — not slots. updateSlotMap() (below) files each attached one under
// the slot its connector dictates; everything else in this file works in
// slots, via slotSerial(), never on these objects directly.
SlotSerial          userial1(myusb, 1);
SlotSerial          userial2(myusb, 1);
SlotSerial          userial3(myusb, 1);
SlotSerial          userial4(myusb, 1);
SlotSerial          userial5(myusb, 1);
SlotSerial          userial6(myusb, 1);
SlotSerial          userial7(myusb, 1);
SlotSerial          userial8(myusb, 1);

SlotSerial *drvPtr[] = { &userial1, &userial2, &userial3, &userial4,
                         &userial5, &userial6, &userial7, &userial8 };
static_assert(sizeof(drvPtr) / sizeof(drvPtr[0]) == DEVPORT_COUNT,
              "declare one serial driver per DEVPORT_COUNT slot");

// Slot (0-based) -> the driver object currently assigned to it, NULL =
// empty. devPort[]/the command handlers above use 1-based slot numbers
// (dev1based - 1 to index in here).
SlotSerial *slotDrv[DEVPORT_COUNT] = { NULL };

static inline SlotSerial *slotSerial(uint8_t idx)
{
  return (idx < DEVPORT_COUNT) ? slotDrv[idx] : NULL;
}

static inline bool slotConnected(uint8_t idx)
{
  SlotSerial *s = slotSerial(idx);
  return s != NULL && (bool)(*s);
}

// -----------------------------------------------------------------------------
// devicePath — which board connector (USB2514B downstream port, 1-4) a
// device is on, and, if it sits behind its own internal hub (a MIPS with a
// USB-RS485 temperature controller), which port of that inner hub.
//
// Every Device_t records the USB address of the hub it hangs off
// (hub_address) and the port on that hub (hub_port). The board hub is the
// one hanging off the Teensy's root port (hub_address 0), so: walk up
// through the hubs until the parent is the board hub; the hub_port at
// that point is the connector. Returns false if the device isn't behind
// the board hub at all (no hub fitted / plugged straight into the root).
// The connector numbers are the USB2514B's own port numbers: whether they
// match the connector labels on the board is confirmed on hardware.
// -----------------------------------------------------------------------------
static const Device_t *hubDeviceByAddress(uint8_t addr)
{
  for (uint8_t i = 0; i < HUB_COUNT; i++)
  {
    const Device_t *d = hubPtr[i]->dev();
    if (d != NULL && d->address == addr) return d;
  }
  return NULL;
}

bool devicePath(const Device_t *d, uint8_t *connector, uint8_t *innerPort)
{
  *connector = 0;
  *innerPort = 0;
  if (d == NULL) return false;

  uint8_t inner = 0;
  for (uint8_t depth = 0; depth < 4; depth++)
  {
    const Device_t *parent = hubDeviceByAddress(d->hub_address);
    if (parent == NULL) return false;
    if (parent->hub_address == 0)          // parent is the board hub
    {
      *connector = d->hub_port;
      *innerPort = inner;
      return true;
    }
    inner = d->hub_port;                   // port on an inner hub; keep climbing
    d = parent;
  }
  return false;
}

// -----------------------------------------------------------------------------
// Slot assignment — slots follow connectors, not USB claim order.
//
//   connector N, the instrument itself           -> slot N      (1-4)
//   connector N, a USB-serial bridge chip behind
//   that instrument's own internal hub            -> slot N + 4  (5-8)
//
// The second case is a MIPS with its internal temperature controller on a
// USB-RS485 adapter (FTDI) — the only known nested-hub setup — so that
// adapter is always 4 slots above its MIPS. A bridge chip plugged straight
// into a connector (no inner hub) is the instrument itself and gets slot N.
// If the preferred slot is already taken (an unexpected second device
// behind one connector), the other slot of that connector is used; if both
// are taken, the device stays unassigned (GDEVPATH shows it as slot 0). A
// device that isn't behind the board hub at all (no hub fitted) takes the
// first free slot, so the firmware still works on a bare Teensy.
// -----------------------------------------------------------------------------
static bool isUsbSerialBridgeChip(uint16_t vid)
{
  return vid == 0x0403    // FTDI
      || vid == 0x10C4    // Silicon Labs CP210x
      || vid == 0x1A86    // WCH CH340/CH341
      || vid == 0x067B;   // Prolific PL2303
}

static bool driverAssigned(SlotSerial *drv)
{
  for (uint8_t s = 0; s < DEVPORT_COUNT; s++)
    if (slotDrv[s] == drv) return true;
  return false;
}

static int8_t chooseSlot(SlotSerial *drv)
{
  uint8_t conn = 0, inner = 0;
  if (!devicePath(drv->dev(), &conn, &inner) || conn < 1 || conn > 4)
  {
    for (uint8_t s = 0; s < DEVPORT_COUNT; s++)
      if (slotDrv[s] == NULL) return (int8_t)s;
    return -1;
  }

  uint8_t mainSlot  = (uint8_t)(conn - 1);
  uint8_t extraSlot = (uint8_t)(conn - 1 + 4);
  bool preferExtra  = (inner != 0) && isUsbSerialBridgeChip(drv->idVendor());

  uint8_t first  = preferExtra ? extraSlot : mainSlot;
  uint8_t second = preferExtra ? mainSlot  : extraSlot;
  if (slotDrv[first]  == NULL) return (int8_t)first;
  if (slotDrv[second] == NULL) return (int8_t)second;
  return -1;
}

// Runs every RepeaterModeLoop() pass, right after myusb.Task(): drop slots
// whose device has gone, then file any newly attached driver.
static void updateSlotMap(void)
{
  for (uint8_t s = 0; s < DEVPORT_COUNT; s++)
    if (slotDrv[s] != NULL && !(bool)(*slotDrv[s])) slotDrv[s] = NULL;

  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    SlotSerial *drv = drvPtr[i];
    if (!(bool)(*drv) || driverAssigned(drv)) continue;
    int8_t s = chooseSlot(drv);
    if (s >= 0) slotDrv[s] = drv;
  }
}

// -----------------------------------------------------------------------------
// effectiveDev — which slot data port 1 (which=1, Serial) or data port 2
// (which=2, SerialUSB2) is bridged to right now. 1-based; 0 = none.
//
// A non-zero ActiveDevice/ActiveDevice2 pins that port to that slot. 0 is
// automatic: data port 1 takes the lowest occupied slot, data port 2 the
// next occupied one, skipping whatever the other port uses. Because slots
// follow connectors, two devices in any two connectors reach the two data
// ports with no setup. Adding a device on a lower connector later shifts
// the automatic assignment.
// -----------------------------------------------------------------------------
static void resolveDataPorts(uint8_t *sel1, uint8_t *sel2)   // 0-based; DEVPORT_COUNT = none
{
  uint8_t a = data.ActiveDevice, b = data.ActiveDevice2;
  *sel1 = (a >= 1 && a <= DEVPORT_COUNT) ? (uint8_t)(a - 1) : DEVPORT_COUNT;
  *sel2 = (b >= 1 && b <= DEVPORT_COUNT) ? (uint8_t)(b - 1) : DEVPORT_COUNT;

  if (a == 0)
    for (uint8_t s = 0; s < DEVPORT_COUNT; s++)
      if (slotConnected(s) && s != *sel2) { *sel1 = s; break; }

  if (b == 0)
    for (uint8_t s = 0; s < DEVPORT_COUNT; s++)
      if (slotConnected(s) && s != *sel1) { *sel2 = s; break; }

  if (*sel2 == *sel1) *sel2 = DEVPORT_COUNT;   // defensive: SDEV/SDEV2 keep pinned values distinct
}

uint8_t effectiveDev(uint8_t which)
{
  uint8_t s1, s2;
  resolveDataPorts(&s1, &s2);
  uint8_t s = (which == 2) ? s2 : s1;
  return (s < DEVPORT_COUNT) ? (uint8_t)(s + 1) : 0;
}

// -----------------------------------------------------------------------------
// GDEVPATH — diagnostic: for every serial driver slot, what's attached and
// where (connector, inner-hub port, VID/PID, speed, USB address/hub
// address/hub port), then every hub driver. Lets the connector mapping be
// checked on real hardware before slots are tied to connectors.
// -----------------------------------------------------------------------------
static void printHex4(uint16_t v)
{
  char buf[5];
  snprintf(buf, sizeof(buf), "%04X", v);
  cp.print(buf);
}

static void printDevPathRow(int slot, const Device_t *d)
{
  uint8_t conn = 0, inner = 0;
  devicePath(d, &conn, &inner);
  cp.print(slot);                 cp.print(",");
  cp.print(d != NULL);            cp.print(",");
  cp.print((int)conn);            cp.print(",");
  cp.print((int)inner);           cp.print(",");
  printHex4(d ? d->idVendor : 0); cp.print(",");
  printHex4(d ? d->idProduct : 0); cp.print(",");
  cp.print((int)(d ? d->speed : 0));       cp.print(",");
  cp.print((int)(d ? d->address : 0));     cp.print(",");
  cp.print((int)(d ? d->hub_address : 0)); cp.print(",");
  cp.println((int)(d ? d->hub_port : 0));
}

void cmdGetDevPath(void)
{
  if (!cp.checkExpectedArgs(0)) return;

  cp.sendACK(false);
  cp.println("#slot,connected,connector,innerPort,vid,pid,speed,addr,hubAddr,hubPort");
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    SlotSerial *s = slotSerial(i);
    printDevPathRow(i + 1, s ? s->dev() : NULL);
  }
  // Attached but not given a slot (both of its connector's slots taken):
  // reported as slot 0 so the problem is visible, not silent.
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    if ((bool)(*drvPtr[i]) && !driverAssigned(drvPtr[i])) printDevPathRow(0, drvPtr[i]->dev());
  }
  cp.println("#hub,connected,vid,pid,addr,hubAddr,hubPort");
  for (uint8_t i = 0; i < HUB_COUNT; i++)
  {
    const Device_t *d = hubPtr[i]->dev();
    cp.print((int)(i + 1));         cp.print(",");
    cp.print(d != NULL);            cp.print(",");
    printHex4(d ? d->idVendor : 0); cp.print(",");
    printHex4(d ? d->idProduct : 0); cp.print(",");
    cp.print((int)(d ? d->address : 0));     cp.print(",");
    cp.print((int)(d ? d->hub_address : 0)); cp.print(",");
    cp.println((int)(d ? d->hub_port : 0));
  }
}

// Shared across all DEVPORT_COUNT ports; set via SBAUD, or Serial.baud() auto-follow
// (RepeaterModeLoop(), below). SerialUSB2 does NOT get its own auto-follow —
// a known limitation, not an oversight: curBaud is one value for the whole
// system, so if Serial and SerialUSB2 are both in use against two different
// devices that genuinely need two different baud rates, SBAUD is the only
// way to set the second one (Serial's auto-follow will fight it on its own
// next change). Fine for today's primarily-ASCII-command-at-one-baud
// downstream devices; revisit with a per-port baud if that stops being true.
uint32_t curBaud = 115200;
uint8_t  repeaterBuf[512];   // shared scratch buffer for all per-port USB reads

struct DevPort
{
  SlotSerial *startedDrv      = NULL;  // driver begin()'d for this slot; differs from slotDrv[] => (re)start it
  uint16_t rxHead             = 0;  // ring buffer write index
  uint16_t rxTail             = 0;  // ring buffer read index
  uint16_t rxCount            = 0;  // bytes currently buffered
  uint32_t rxDropped          = 0;  // bytes dropped because the buffer was full, since boot
  uint8_t  rxbuf[DEVPORT_RINGBUF_SIZE];
};

DevPort devPort[DEVPORT_COUNT];

// -----------------------------------------------------------------------------
// Device identification — ask each newly attached device its GNAME.
//
// Every GAACE device answers GNAME with its (unique, by convention) system
// name. When a device lands in slot 1-4, the firmware sends "GNAME\n" and
// waits for one reply line: ACK + name -> NAMED; NAK -> NONAME (answers, but
// isn't a GAACE device); no reply -> retry every IDENT_RETRY_MS (a MIPS may
// still be booting) up to IDENT_MAX_TRIES, then NONAME. Lines that aren't an
// ACK/NAK reply (a boot banner, an echoed command) are ignored.
//
// While identification is PENDING the slot is held out of everything else:
// its incoming bytes go to the reply parser instead of its ring buffer, and
// host passthrough, ADCUpdate() and script sends to it wait (slotReady()).
// The data-port assignment (resolveDataPorts()) still counts it as
// occupied, so the automatic layout doesn't shift while it's identified.
//
// Slots 5-8 (a USB-serial bridge behind a MIPS's own hub — its RS-485
// temperature controller) are NOT queried automatically: "GNAME" would go
// out onto that controller's RS-485 bus. DEVIDENT,<slot> forces a query.
// -----------------------------------------------------------------------------
#define IDENT_FIRST_DELAY_MS   300   // after attach, before the first GNAME
#define IDENT_REPLY_TIMEOUT_MS 300   // wait this long for each reply
#define IDENT_RETRY_MS         700   // gap before retrying a query that got no reply
#define IDENT_MAX_TRIES        8     // ~8 s in total, enough for a MIPS to boot
#define DEV_NAME_LEN           24

enum IdentState : uint8_t
{
  IDENT_EMPTY = 0,   // no device in the slot
  IDENT_PENDING,     // querying GNAME
  IDENT_NAMED,       // answered; name[] valid
  IDENT_NONAME,      // NAK or no reply: not a GAACE device
  IDENT_SKIPPED      // extra slot (5-8), not queried
};

struct DevIdent
{
  IdentState state      = IDENT_EMPTY;
  uint8_t    tries      = 0;
  bool       awaiting   = false;   // a GNAME is outstanding
  bool       eatLF      = false;   // reply ended on '\r': drop the '\n' that follows it
  uint32_t   nextMs     = 0;       // when to send the next GNAME
  uint32_t   deadlineMs = 0;       // when the outstanding one times out
  uint8_t    lineLen    = 0;
  char       line[DEV_NAME_LEN + 8];
  char       name[DEV_NAME_LEN];
};

DevIdent devIdent[DEVPORT_COUNT];

// Connected and not being identified: safe for passthrough, ADC and scripts.
static inline bool slotReady(uint8_t idx)
{
  return slotConnected(idx) && devIdent[idx].state != IDENT_PENDING;
}

static void identStart(uint8_t idx, bool force)
{
  DevIdent &id = devIdent[idx];
  id.name[0]  = '\0';
  id.tries    = 0;
  id.awaiting = false;
  id.eatLF    = false;
  id.lineLen  = 0;
  if (!force && idx >= 4) { id.state = IDENT_SKIPPED; return; }
  id.state  = IDENT_PENDING;
  id.nextMs = millis() + IDENT_FIRST_DELAY_MS;
}

static bool arbIdle(uint8_t idx);   // per-slot arbiter, defined below

// Send / time out the GNAME query. Runs every loop pass for every slot.
static void identPoll(uint8_t idx)
{
  DevIdent &id = devIdent[idx];
  if (id.state != IDENT_PENDING) return;

  SlotSerial *s = slotSerial(idx);
  if (s == NULL || !(bool)(*s)) return;   // detach is handled by devPortUpdateConnection()

  uint32_t now = millis();
  if (!id.awaiting)
  {
    if ((int32_t)(now - id.nextMs) < 0) return;
    if (!arbIdle(idx)) return;   // DEVIDENT during an exchange: wait for it to end
    s->write((const uint8_t *)"GNAME\n", 6);
    id.awaiting   = true;
    id.deadlineMs = now + IDENT_REPLY_TIMEOUT_MS;
    id.lineLen    = 0;
    id.tries++;
  }
  else if ((int32_t)(now - id.deadlineMs) >= 0)
  {
    id.awaiting = false;
    if (id.tries >= IDENT_MAX_TRIES) id.state = IDENT_NONAME;
    else id.nextMs = now + IDENT_RETRY_MS;
  }
}

// One incoming byte while PENDING. Only a complete ACK/NAK line settles it.
static void identFeed(uint8_t idx, uint8_t b)
{
  DevIdent &id = devIdent[idx];
  if (b != '\n' && b != '\r')
  {
    if (id.lineLen < sizeof(id.line) - 1) id.line[id.lineLen++] = (char)b;
    return;
  }
  if (id.lineLen == 0) return;
  id.line[id.lineLen] = '\0';
  id.lineLen = 0;
  if (!id.awaiting) return;   // e.g. a boot banner before the first query

  // GAACE replies end "\r\n". If this '\r' settles the reply, the '\n' still
  // to come must not leak through to the host as a stray blank line.
  id.eatLF = (b == '\r');

  if ((uint8_t)id.line[0] == 0x06)
  {
    const char *n = id.line + 1;
    while (*n == ' ') n++;
    strncpy(id.name, n, DEV_NAME_LEN - 1);
    id.name[DEV_NAME_LEN - 1] = '\0';
    for (int i = (int)strlen(id.name) - 1; i >= 0 && id.name[i] == ' '; i--) id.name[i] = '\0';
    id.state = (id.name[0] != '\0') ? IDENT_NAMED : IDENT_NONAME;
    id.awaiting = false;
  }
  else if ((uint8_t)id.line[0] == 0x15)
  {
    id.state = IDENT_NONAME;
    id.awaiting = false;
  }
  // anything else (unsolicited output, echoed "GNAME"): keep waiting
}

static const char *identStateName(IdentState s)
{
  switch (s)
  {
    case IDENT_PENDING: return "PENDING";
    case IDENT_NAMED:   return "NAMED";
    case IDENT_NONAME:  return "NONAME";
    case IDENT_SKIPPED: return "NOTQUERIED";
    default:            return "EMPTY";
  }
}

// Slot (0-based) whose device answered GNAME with `name` (case-insensitive),
// or -1. Lowest slot wins if two devices share a name.
int8_t slotByName(const char *name)
{
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
    if (slotConnected(i) && devIdent[i].state == IDENT_NAMED && strcasecmp(devIdent[i].name, name) == 0)
      return (int8_t)i;
  return -1;
}

// -----------------------------------------------------------------------------
// GDEVLIST — one row per occupied slot:
//   slot,connector,role,vid,pid,state,name
// role MAIN (slot 1-4, the instrument) or EXTRA (5-8, a bridge behind its
// hub); state NAMED / PENDING / NONAME / NOTQUERIED; name empty unless NAMED.
// GDEVSLOT,<name> — the slot (1-8) of the device with that name, NAK if none.
// DEVIDENT,<slot> — ask that slot's device its GNAME again (also forces a
// query on an extra slot 5-8).
// -----------------------------------------------------------------------------
void cmdGetDevList(void)
{
  if (!cp.checkExpectedArgs(0)) return;

  cp.sendACK(false);
  cp.println("#slot,connector,role,vid,pid,state,name");
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    if (!slotConnected(i)) continue;
    const Device_t *d = slotSerial(i)->dev();
    uint8_t conn = 0, inner = 0;
    devicePath(d, &conn, &inner);
    char row[24];
    cp.print((int)(i + 1));                   cp.print(",");
    cp.print((int)conn);                      cp.print(",");
    cp.print(i < 4 ? "MAIN" : "EXTRA");       cp.print(",");
    snprintf(row, sizeof(row), "%04X,%04X", d ? d->idVendor : 0, d ? d->idProduct : 0);
    cp.print(row);                            cp.print(",");
    cp.print(identStateName(devIdent[i].state)); cp.print(",");
    cp.println(devIdent[i].state == IDENT_NAMED ? devIdent[i].name : "");
  }
}

void cmdGetDevSlot(void)
{
  if (!cp.checkExpectedArgs(1)) return;
  char *tok;
  if (!cp.getValue(&tok)) { cp.sendNAK(ERR_BADARG); return; }
  int8_t s = slotByName(tok);
  cp.ca->free(tok);
  if (s < 0) { cp.sendNAK(ERR_BADARG); return; }
  cp.sendACK(false);
  cp.println((int)(s + 1));
}

void cmdDevIdent(void)
{
  if (!cp.checkExpectedArgs(1)) return;
  int n;
  if (!cp.getValue(&n, 1, DEVPORT_COUNT) || !slotConnected((uint8_t)(n - 1)))
  {
    cp.sendNAK(ERR_BADARG);
    return;
  }
  identStart((uint8_t)(n - 1), true);
  cp.sendACK();
}

// -----------------------------------------------------------------------------
// PER-SLOT ARBITER — one command in progress per device.
//
// GAACE devices are strictly request/reply: a command line in, one reply
// (ACK, ACK+value or NAK, ending in a newline) out. Several requesters can
// want the same device: the USB data port bridged to it (Serial or
// SerialUSB2), its own TCP data client, and the repeater itself (ADCUpdate,
// script sends). Each requester is a "line source" that collects one
// complete line; each slot's arbiter sends one line at a time, routes the
// reply bytes to whoever sent it, and treats the exchange as finished once
// the reply's newline has arrived and the device has then been quiet for
// ARB_QUIET_MS (covers multi-line replies), or after ARB_TIMEOUT_MS with no
// complete reply. Waiting lines simply stay in their source; a requester
// that waits for its reply before sending again (as GAACE hosts do) never
// needs more than one queued line, so the queue can't grow.
//
// Fast path: if the requester that owns the open exchange already has its
// next line ready and nobody else is waiting, it's sent as soon as the reply
// newline arrives, without the quiet gap, so a single host app sees no
// added latency per command. Lines are forwarded once complete (newline,
// ARB_LINE_MAX bytes, or ARB_PARTIAL_MS of silence for a host that doesn't
// end its lines), never byte by byte: fine for the ASCII command traffic
// the host apps send (per the user, no binary/firmware transfers).
//
// Bytes the device sends with no exchange open (unsolicited output) go to
// whoever last owned an exchange on it, if that requester still targets the
// slot, else to the USB data port bridged to it, else its TCP client, else
// they stay buffered. A whitespace-only line (a stray "\r\n") is forwarded
// without opening an exchange, since the device won't reply to it.
//
// Replaces the old "last speaker wins" pcSide/pcSide2 routing and the ADC
// quiet-guard/swallow-window timing heuristics.
// -----------------------------------------------------------------------------
#define ARB_LINE_MAX     256    // longest line a source collects before forwarding anyway
#define ARB_QUIET_MS     20     // after a reply's newline, quiet this long = exchange over
#define ARB_TIMEOUT_MS   1000   // no complete reply this long after sending = give up
#define ARB_PARTIAL_MS   50     // a partial line idle this long is forwarded as-is

enum : uint8_t
{
  SRC_USB1  = 0,                          // Serial (data port 1)
  SRC_USB2  = 1,                          // SerialUSB2 (data port 2)
  SRC_ETH0  = 2,                          // + slot: that slot's TCP data client
  SRC_INT0  = SRC_ETH0 + DEVPORT_COUNT,   // + slot: the repeater itself (ADC, scripts)
  SRC_COUNT = SRC_INT0 + DEVPORT_COUNT,
  SRC_NONE  = 0xFF
};

struct LineSrc
{
  uint16_t len        = 0;
  bool     ready      = false;   // a complete line is waiting to be sent
  uint32_t lastByteMs = 0;
  char     buf[ARB_LINE_MAX];
};

struct Arb
{
  uint8_t  owner     = SRC_NONE;   // requester whose exchange is open
  uint8_t  lastOwner = SRC_NONE;   // last non-internal owner: gets unsolicited output
  uint8_t  rr        = 0;          // round-robin start among the slot's requesters
  bool     sawNL     = false;      // the open exchange's reply newline has arrived
  uint32_t sentMs    = 0;
  uint32_t lastRxMs  = 0;
  uint32_t exchanges = 0;          // since boot
  uint32_t timeouts  = 0;          // since boot
};

LineSrc lineSrc[SRC_COUNT];
Arb     arb[DEVPORT_COUNT];
uint8_t srcTarget[SRC_COUNT];      // slot each source addresses this pass (DEVPORT_COUNT = none)

static inline bool isIntSrc(uint8_t src) { return src >= SRC_INT0 && src < SRC_COUNT; }

// The stream a requester talks over, or NULL (internal, or a TCP client
// that isn't connected).
static Stream *srcStream(uint8_t src)
{
  if (src == SRC_USB1) return &Serial;
  if (src == SRC_USB2) return &SerialUSB2;
  if (src >= SRC_ETH0 && src < SRC_INT0)
  {
    uint8_t i = (uint8_t)(src - SRC_ETH0);
    return (ethDataClient[i] && ethDataClient[i].connected()) ? &ethDataClient[i] : NULL;
  }
  return NULL;
}

// Queue a line from the repeater itself for slot idx (ADCUpdate, script
// sends). Replaces any internal line still waiting: for a periodic value
// the newest one is the one worth sending. The reply is discarded.
static bool arbSubmitInternal(uint8_t idx, const char *line, int len)
{
  if (idx >= DEVPORT_COUNT || len <= 0) return false;
  LineSrc &L = lineSrc[SRC_INT0 + idx];
  if (len > ARB_LINE_MAX) len = ARB_LINE_MAX;
  memcpy(L.buf, line, (size_t)len);
  L.len   = (uint16_t)len;
  L.ready = true;
  return true;
}

static bool arbIdle(uint8_t idx) { return arb[idx].owner == SRC_NONE; }

// True while the repeater's own line for slot idx is waiting or in progress.
static bool arbInternalBusy(uint8_t idx)
{
  return lineSrc[SRC_INT0 + idx].ready || arb[idx].owner == (uint8_t)(SRC_INT0 + idx);
}

// -----------------------------------------------------------------------------
// Per-port accessors — forward-declared in Section 5 for the command
// handlers there (GLINK, GDEVSTAT, SBAUD, ...).
// -----------------------------------------------------------------------------
bool isDeviceConnected(uint8_t dev1based)
{
  if (dev1based < 1 || dev1based > DEVPORT_COUNT) return false;
  return slotConnected((uint8_t)(dev1based - 1));
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
  if (isDeviceConnected(dev1based)) slotSerial((uint8_t)(dev1based - 1))->begin(baud);
}

// -----------------------------------------------------------------------------
// devPortPush — accept one incoming byte into port idx's ring buffer.
// -----------------------------------------------------------------------------
static void devPortPush(uint8_t idx, uint8_t b)
{
  DevPort &p = devPort[idx];

  if (p.rxCount >= DEVPORT_RINGBUF_SIZE)
  {
    p.rxDropped++;   // overflow: drop incoming, keep what's already queued
    return;
  }

  p.rxbuf[p.rxHead] = b;
  p.rxHead = (uint16_t)((p.rxHead + 1) % DEVPORT_RINGBUF_SIZE);
  p.rxCount++;
}

// Drain whatever's available from slot idx's device into devPort[idx]'s ring
// buffer. Runs for all DEVPORT_COUNT ports every tick, regardless of selection.
static void devPortDrainIncoming(uint8_t idx)
{
  SlotSerial *s = slotSerial(idx);
  if (s == NULL) return;
  int n = s->available();
  if (n <= 0) return;

  n = s->readBytes((char *)repeaterBuf, min(n, (int)sizeof(repeaterBuf)));
  for (int i = 0; i < n; i++)
  {
    // While its GNAME is outstanding a slot's bytes are the reply, not
    // host-bound traffic; once settled, the rest of the chunk flows normally.
    DevIdent &id = devIdent[idx];
    if (id.state == IDENT_PENDING) { identFeed(idx, repeaterBuf[i]); continue; }
    if (id.eatLF)
    {
      id.eatLF = false;
      if (repeaterBuf[i] == '\n') continue;   // tail of the settled GNAME reply
    }

    // Track the open exchange's reply for the arbiter (end = newline + quiet).
    Arb &a = arb[idx];
    if (a.owner != SRC_NONE)
    {
      a.lastRxMs = millis();
      if (repeaterBuf[i] == '\n') a.sawNL = true;
      if (isIntSrc(a.owner)) continue;   // reply to the repeater's own command: not for any host
    }
    devPortPush(idx, repeaterBuf[i]);
  }
}

static void devPortDiscard(uint8_t idx)
{
  DevPort &p = devPort[idx];
  p.rxTail = p.rxHead;
  p.rxCount = 0;
}

// Write out everything currently buffered for port idx, in order, to `out`.
// At most two write() calls (handles the ring's wraparound in two
// contiguous runs) rather than one per byte.
static void devPortFlush(uint8_t idx, Stream *out)
{
  DevPort &p = devPort[idx];
  if (p.rxCount == 0) return;
  while (p.rxCount > 0)
  {
    uint16_t contiguous = (uint16_t)min((int)p.rxCount, (int)(DEVPORT_RINGBUF_SIZE - p.rxTail));
    out->write(&p.rxbuf[p.rxTail], contiguous);
    p.rxTail   = (uint16_t)((p.rxTail + contiguous) % DEVPORT_RINGBUF_SIZE);
    p.rxCount -= contiguous;
  }
  // QNEthernet queues written data until flush() or lwIP's 250 ms TCP timer,
  // so without this every reply over TCP waited up to a quarter second
  // (measured ~250 ms/command). Only for TCP clients: USB Serial's flush()
  // blocks until the host has read the data.
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
    if (out == (Stream *)&ethDataClient[i]) { out->flush(); break; }
}

// Attach/detach + auto-reconnect for one port. Runs on all DEVPORT_COUNT every tick,
// independent of selection — a port reconnecting must not depend on whether
// it's the one currently bridged to the upstream.
static void devPortUpdateConnection(uint8_t idx)
{
  DevPort &p = devPort[idx];
  SlotSerial *s = slotConnected(idx) ? slotSerial(idx) : NULL;

  // Start the driver when a device lands in this slot, including a
  // different driver object replacing the previous one in the same pass.
  if (s != NULL && s != p.startedDrv)
  {
    s->begin(curBaud);
    arb[idx].owner = SRC_NONE;   // a new device: no exchange carries over
    arb[idx].sawNL = false;
    identStart(idx, false);   // new device in this slot: find out its name
  }
  if (s == NULL)
  {
    devIdent[idx].state = IDENT_EMPTY;
    arb[idx].owner = SRC_NONE;
  }
  p.startedDrv = s;
}

// -----------------------------------------------------------------------------
// Arbiter helpers (see the PER-SLOT ARBITER comment above).
// -----------------------------------------------------------------------------

// Collect bytes from a host-side source into its line buffer, only while it
// addresses a ready slot and has no complete line already waiting (so a
// waiting line holds back the rest of that host's input in its own stream).
static void srcFill(uint8_t src)
{
  LineSrc &L = lineSrc[src];
  if (L.ready) return;
  uint32_t now = millis();

  uint8_t target = srcTarget[src];
  Stream *st = srcStream(src);
  if (target < DEVPORT_COUNT && slotReady(target) && st != NULL)
  {
    while (st->available() > 0)
    {
      int c = st->read();
      if (c < 0) break;
      L.buf[L.len++] = (char)c;
      L.lastByteMs = now;
      if (c == '\n' || L.len >= ARB_LINE_MAX) { L.ready = true; break; }
    }
  }
  if (!L.ready && L.len > 0 && (now - L.lastByteMs) >= ARB_PARTIAL_MS) L.ready = true;
}

static void arbSend(uint8_t idx, uint8_t src)
{
  LineSrc &L = lineSrc[src];
  Arb &a = arb[idx];
  SlotSerial *s = slotSerial(idx);
  if (s != NULL) s->write((const uint8_t *)L.buf, L.len);

  bool blank = true;
  for (uint16_t k = 0; k < L.len; k++)
    if (L.buf[k] != '\r' && L.buf[k] != '\n' && L.buf[k] != ' ' && L.buf[k] != '\t') { blank = false; break; }

  if (blank)
  {
    if (!isIntSrc(src)) a.lastOwner = src;   // nothing to wait for
  }
  else
  {
    a.owner    = src;
    a.sentMs   = millis();
    a.lastRxMs = a.sentMs;
    a.sawNL    = false;
    a.exchanges++;
  }
  L.len   = 0;
  L.ready = false;
}

// Where slot idx's buffered output goes right now. *discard = true means it
// belongs to a requester that's gone (a closed TCP client) or to the
// repeater itself, and must not reach anyone else.
static Stream *arbOutput(uint8_t idx, bool *discard)
{
  Arb &a = arb[idx];
  *discard = false;
  if (a.owner != SRC_NONE)
  {
    Stream *st = srcStream(a.owner);
    if (st == NULL) *discard = true;
    return st;
  }
  uint8_t o = a.lastOwner;
  if (o != SRC_NONE && srcTarget[o] == idx && srcStream(o) != NULL) return srcStream(o);
  if (srcTarget[SRC_USB1] == idx) return &Serial;
  if (srcTarget[SRC_USB2] == idx) return &SerialUSB2;
  return srcStream((uint8_t)(SRC_ETH0 + idx));   // NULL: no listener, stays buffered
}

static void arbService(uint8_t idx)
{
  Arb &a = arb[idx];
  if (!slotConnected(idx)) { a.owner = SRC_NONE; return; }
  uint32_t now = millis();

  const uint8_t cand[4] = { SRC_USB1, SRC_USB2, (uint8_t)(SRC_ETH0 + idx), (uint8_t)(SRC_INT0 + idx) };
  auto waiting = [&](uint8_t src) { return srcTarget[src] == idx && lineSrc[src].ready; };

  if (a.owner != SRC_NONE)
  {
    uint8_t owner = a.owner;
    bool othersWaiting = false;
    for (uint8_t k = 0; k < 4; k++)
      if (cand[k] != owner && waiting(cand[k])) othersWaiting = true;

    bool done = false, fast = false;
    if (a.sawNL)
    {
      if ((now - a.lastRxMs) >= ARB_QUIET_MS) done = true;
      else if (waiting(owner) && !othersWaiting) { done = true; fast = true; }
    }
    if (!done && (now - a.sentMs) >= ARB_TIMEOUT_MS) { done = true; a.timeouts++; }
    if (!done) return;

    if (!isIntSrc(owner)) a.lastOwner = owner;
    a.owner = SRC_NONE;
    if (fast) { arbSend(idx, owner); return; }   // same requester, nobody else waiting
  }

  if (!slotReady(idx)) return;   // GNAME being asked: hold everything
  for (uint8_t k = 0; k < 4 && a.owner == SRC_NONE; k++)
  {
    uint8_t pick = (uint8_t)((a.rr + k) % 4);
    if (!waiting(cand[pick])) continue;
    arbSend(idx, cand[pick]);
    a.rr = (uint8_t)((pick + 1) % 4);
    // a blank line opens no exchange: keep going so the next one is served too
  }
}

// GARBSTAT — per slot: slot,busy,owner,exchanges,timeouts. owner is who has
// the open exchange (USB1, USB2, ETH, INT or -); exchanges/timeouts are
// since-boot counters. A rising timeouts count means the device isn't
// answering some commands within ARB_TIMEOUT_MS.
static const char *srcName(uint8_t src)
{
  if (src == SRC_NONE) return "-";
  if (src == SRC_USB1) return "USB1";
  if (src == SRC_USB2) return "USB2";
  if (src < SRC_INT0)  return "ETH";
  return "INT";
}

void cmdGetArbStat(void)
{
  if (!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  cp.println("#slot,busy,owner,exchanges,timeouts");
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    cp.print((int)(i + 1));                  cp.print(",");
    cp.print(arb[i].owner != SRC_NONE);      cp.print(",");
    cp.print(srcName(arb[i].owner));         cp.print(",");
    cp.print(arb[i].exchanges);              cp.print(",");
    cp.println(arb[i].timeouts);
  }
}

void RepeaterModeSetup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);       // LED = ActiveDevice connected
  Serial.begin(115200);               // native USB, baud value ignored
  SerialUSB2.begin(115200);           // native USB, baud value ignored — second switchable data port

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
  updateSlotMap();   // file newly attached devices under their connector's slot

  // Connection management + incoming capture: all DEVPORT_COUNT ports, every tick,
  // unconditionally. This is what makes "nothing is lost" true regardless
  // of what's selected.
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    devPortUpdateConnection(i);
    identPoll(i);
    devPortDrainIncoming(i);
  }

  // Which slot each source addresses this pass. Data ports: pinned by
  // SDEV/SDEV2 or automatic (effectiveDev()); DEVPORT_COUNT = nothing to
  // bridge, and that data port's input simply waits. TCP clients and the
  // repeater's own lines always address their own slot.
  uint8_t selIdx, selIdx2;
  resolveDataPorts(&selIdx, &selIdx2);
  srcTarget[SRC_USB1] = selIdx;
  srcTarget[SRC_USB2] = selIdx2;
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    srcTarget[SRC_ETH0 + i] = i;
    srcTarget[SRC_INT0 + i] = i;
  }

  bool connected = slotReady(selIdx);
  digitalWriteFast(LED_BUILTIN, connected ? HIGH : LOW);   // reflects data port 1's device only

  // Follow the baud rate the PC application sets (matters for FTDI/CP210x/CH340 bridges).
  // Applies only to Serial's selected device — see curBaud's own comment
  // above for why SerialUSB2 doesn't get an equivalent auto-follow.
  uint32_t b = Serial.baud();
  if (b != 0 && b != curBaud)
  {
    curBaud = b;
    if (connected) slotSerial(selIdx)->begin(curBaud);
  }

  // Host -> device: collect complete lines from every host-side source.
  srcFill(SRC_USB1);
  srcFill(SRC_USB2);
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++) srcFill((uint8_t)(SRC_ETH0 + i));

  // Device -> host first, then arbitration: everything drained this pass
  // belongs to the exchange that was open while it arrived, so it's
  // delivered before the arbiter can hand the device to someone else.
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    bool discard;
    Stream *out = arbOutput(i, &discard);
    if (discard)    devPortDiscard(i);
    else if (out)   devPortFlush(i, out);
    arbService(i);
  }
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
//  controlled. The command goes through that slot's arbiter (Section 6,
//  arbSubmitInternal()) as the repeater's own requester: it waits its turn
//  behind any host command in progress, never splices into one, and its
//  reply is discarded so it never reaches a host. If the previous value is
//  still waiting to be sent, the new one replaces it.

void ADCUpdate(void)
{
  if (!data.AdcEnabled) return;

  uint8_t idx = (uint8_t)(data.AdcTargetDevice - 1);
  if (idx >= DEVPORT_COUNT) return;   // defensive; AdcTargetDevice is range-checked at SADCDEV time
  if (!slotReady(idx)) return;   // nothing connected to control, or GNAME pending

  float counts = readAdcCounts();
  float value  = scaleAdcValue(counts);

  char cmdLine[48];
  int  len = snprintf(cmdLine, sizeof(cmdLine), "%s,%.3f\n", data.AdcCmdName, value);
  if (len <= 0) return;
  if (len >= (int)sizeof(cmdLine)) len = sizeof(cmdLine) - 1;  // truncated; still send what fits

  arbSubmitInternal(idx, cmdLine, len);
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
//  AdcScaleB/AdcCmdName), and the same per-slot arbiter path ADCUpdate()
//  uses (Section 6) — so the two features can't collide with ordinary
//  repeater traffic any differently than ADCUpdate() already doesn't.
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
//  Syscalls are described once, in the `scriptSyscalls[]` table in setup()
//  (Section 8): that table is the single source of truth for each syscall's
//  name, argument count, return type and id (its index). The device reports
//  it to gsc.py (GSYSCALLS), so scripts need no hand-written declarations;
//  gaace_scripts/adc_demo.gs keeps explicit ones only because its bytecode
//  is compiled in at build time (include/ScriptBytecode.h) without a device
//  to ask. Keep these ids (the current table order) if that file is not
//  regenerated:
//    0 = link_ready()      — data.AdcTargetDevice connected, identified, and the
//                             repeater's previous send to it already done
//    1 = read_adc()         — readAdcCounts() rounded to int32 via lroundf()
//                             (the VM is int32-only; see readAdcCounts()'s
//                             own comment in Section 5 for the averaging)
//    2 = scale_send(counts) — scaleAdcValue(counts) (Section 5: AdcScaleM*
//                             counts+AdcScaleB, plus the REF correction if
//                             RefCal is set), sent downstream

#include "ScriptBytecode.h"

using namespace GAACEScript;

static int32_t sys_link_ready(int32_t *args, uint8_t argc)
{
  (void)args; (void)argc;

  uint8_t idx = (uint8_t)(data.AdcTargetDevice - 1);
  if (idx >= DEVPORT_COUNT) return 0;
  if (!slotReady(idx)) return 0;         // nothing connected, or GNAME pending
  if (arbInternalBusy(idx)) return 0;    // our previous send hasn't gone out / been answered yet
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
  if (!slotReady(idx)) return 0;   // empty slot (no driver object to write to), or GNAME pending

  float value = scaleAdcValue((float)args[0]);

  char cmdLine[48];
  int  len = snprintf(cmdLine, sizeof(cmdLine), "%s,%.3f\n", data.AdcCmdName, value);
  if (len <= 0) return 0;
  if (len >= (int)sizeof(cmdLine)) len = sizeof(cmdLine) - 1;  // truncated; still send what fits

  return arbSubmitInternal(idx, cmdLine, len) ? 1 : 0;
}

// -----------------------------------------------------------------------------
// Script slot persistence (flash) — USBrepeater-specific, not part of
// GAACE_Script itself: that library stays hardware-agnostic (no Serial, no
// Teensy headers in its core), and LittleFS_Program is Teensy-only. A
// slot's buffer (ScriptSlot::code[]) is exactly the wire-format payload
// SCRIPTLOAD/SCRIPTAPPEND+SCRIPTCOMMIT already use — [codeLen:u16 LE][code
// bytes][pool bytes] — so saving/restoring it is plain whole-buffer file
// I/O; no new format to invent. codeLen/poolLen aren't stored directly on
// ScriptSlot after a load, but VM::codeLen/VM::poolLen (public fields,
// GAACEScript.h) are exactly those values, so totalLen = 2 + vm.codeLen +
// vm.poolLen is recoverable without any GAACE_Script changes.
//
// One file per slot ("/script<N>.bin") on a small LittleFS_Program flash
// filesystem — reserved program flash, not the EEPROM Data{} uses, and not
// a microSD card (there isn't one). SCRIPT_FS_RESERVE_BYTES is generous
// for how little this needs: GAACE_SCRIPT_SLOTS slots x
// GAACE_SCRIPT_MAX_CODE_LEN bytes each is the absolute ceiling (today,
// 4 x 2048 = 8 KB), and Teensy 4.1's program flash has megabytes to spare
// (see any `pio run` size report's "free for files" line).
// -----------------------------------------------------------------------------
#define SCRIPT_FS_RESERVE_BYTES (256UL * 1024UL)

LittleFS_Program scriptFS;
bool scriptFSReady = false;   // set in setup() (Section 8); every function below checks it first

static void scriptSlotPath(uint8_t slot, char *buf, size_t bufLen)
{
  snprintf(buf, bufLen, "/script%u.bin", (unsigned)slot);
}

// A slot's syscall-table hash (ScriptSlot::syscallHash) is saved beside its
// buffer in "/script<N>.sig" (4 bytes, little-endian), so a script compiled
// against one set of syscalls is never silently restored into firmware whose
// syscalls have since changed (e.g. after a firmware update that added or
// reordered one) -- see restoreScriptSlotFromFlash(). A slot loaded unchecked
// (hash 0) has no .sig file, and a .bin saved before this existed has none
// either; both restore as before, unchecked.
static void scriptSigPath(uint8_t slot, char *buf, size_t bufLen)
{
  snprintf(buf, bufLen, "/script%u.sig", (unsigned)slot);
}

// finalizeScriptSlot — identical validation/finalization logic to
// GAACEScriptRuntime.cpp's cmdScriptCommit(), applied directly to a slot
// whose buffer was just filled by restoreScriptSlotFromFlash() below
// (reading raw bytes from a file, not decoding hex from a command line, so
// SCRIPTCOMMIT itself can't be called/reused as-is).
static bool finalizeScriptSlot(uint8_t slot, uint16_t totalLen)
{
  if (totalLen < 2 || totalLen > SCRIPT_MAX_CODE_LEN) return false;

  ScriptSlot &s = scripts.slots[slot];
  uint16_t codeLen = (uint16_t)((uint8_t)s.code[0] | ((uint8_t)s.code[1] << 8));
  if (codeLen > (uint16_t)totalLen - 2) return false;
  uint16_t poolLen = (uint16_t)((uint16_t)totalLen - 2 - codeLen);

  s.codeLen = codeLen;
  vmInit(s.vm, s.code + 2, codeLen);
  vmSetPool(s.vm, s.code + 2 + codeLen, poolLen);
  s.loaded = true;
  s.lastStatus = VM_OK;
  return true;
}

// saveScriptSlotToFlash — rebuild the [codeLen][code][pool] wire-format
// payload from the VM's own authoritative code/pool pointers (vm.code,
// vm.pool — not s.code[] directly) and write it to the slot's file.
// s.code[] only actually starts with that 2-byte header for a slot loaded
// via SCRIPTLOAD/SCRIPTAPPEND+SCRIPTCOMMIT; a slot preloaded by a project
// via a direct vmInit() call (e.g. this file's compiled-in ADC demo,
// Section 8) has vm.code pointing at s.code+0 with no header at all, so
// reading s.code[0..totalLen) verbatim silently saved garbage for that
// case -- caught by hardware testing (SCRIPTSAVE,0/SCRIPTRESTORE,0 round-
// trip on the booted demo), fixed by writing the header/code/pool pieces
// separately instead of assuming the in-RAM buffer's layout.
// Fails (false, no file written/touched) if the slot isn't loaded or the
// filesystem isn't mounted; a slot mid-SCRIPTAPPEND (loaded==false) must
// never be persisted as if it were a complete, runnable script.
bool saveScriptSlotToFlash(uint8_t slot)
{
  if (!scriptFSReady || slot >= SCRIPT_SLOTS) return false;

  ScriptSlot &s = scripts.slots[slot];
  if (!s.loaded) return false;

  uint16_t codeLen = s.vm.codeLen;
  uint16_t poolLen  = s.vm.poolLen;
  uint16_t totalLen = (uint16_t)(2 + codeLen + poolLen);
  if (totalLen > SCRIPT_MAX_CODE_LEN) return false;   // defensive; should be unreachable

  char path[20];
  scriptSlotPath(slot, path, sizeof(path));

  scriptFS.remove(path);   // start from empty so a shorter script doesn't leave trailing garbage
  File f = scriptFS.open(path, FILE_WRITE_BEGIN);
  if (!f) return false;

  uint8_t header[2] = {(uint8_t)codeLen, (uint8_t)(codeLen >> 8)};
  size_t written = f.write(header, sizeof(header));
  written += f.write(s.vm.code, codeLen);
  if (poolLen > 0) written += f.write(s.vm.pool, poolLen);
  f.close();
  if (written != totalLen) return false;

  // The .sig is removed first so a stale one from the slot's previous
  // script can never outlive it; only a checked script gets a new one.
  char sigPath[20];
  scriptSigPath(slot, sigPath, sizeof(sigPath));
  scriptFS.remove(sigPath);
  if (s.syscallHash != 0)
  {
    File sig = scriptFS.open(sigPath, FILE_WRITE_BEGIN);
    if (!sig) return false;
    uint8_t hashBytes[4] = {(uint8_t)(s.syscallHash), (uint8_t)(s.syscallHash >> 8),
                            (uint8_t)(s.syscallHash >> 16), (uint8_t)(s.syscallHash >> 24)};
    size_t sigWritten = sig.write(hashBytes, sizeof(hashBytes));
    sig.close();
    if (sigWritten != sizeof(hashBytes)) return false;
  }
  return true;
}

// restoreScriptSlotFromFlash — read a slot's file back into its buffer and
// finalize it exactly like SCRIPTCOMMIT would. Fails (slot untouched) if the
// filesystem isn't mounted, no file exists for this slot, or the file's own
// [codeLen][code][pool] layout doesn't validate — a corrupt/foreign file
// must not leave the slot in a half-updated state. SCRIPT_RESTORE_STALE: the
// saved script was compiled against a different syscall table than this
// firmware has (its .sig hash differs from scripts.syscallHash()); it is
// refused, not run, since its CALL ids would now mean different functions.
// Recompile and re-upload it.
ScriptRestoreResult restoreScriptSlotFromFlash(uint8_t slot)
{
  if (!scriptFSReady || slot >= SCRIPT_SLOTS) return SCRIPT_RESTORE_NONE;

  char path[20];
  scriptSlotPath(slot, path, sizeof(path));
  if (!scriptFS.exists(path)) return SCRIPT_RESTORE_NONE;

  // Check the syscall hash before touching the slot's buffer.
  uint32_t savedHash = 0;
  char sigPath[20];
  scriptSigPath(slot, sigPath, sizeof(sigPath));
  if (scriptFS.exists(sigPath))
  {
    File sig = scriptFS.open(sigPath, FILE_READ);
    uint8_t hashBytes[4];
    if (!sig || sig.readBytes((char *)hashBytes, 4) != 4)
    {
      if (sig) sig.close();
      return SCRIPT_RESTORE_STALE;   // unreadable signature: can't prove it's safe
    }
    sig.close();
    savedHash = (uint32_t)hashBytes[0] | ((uint32_t)hashBytes[1] << 8) |
                ((uint32_t)hashBytes[2] << 16) | ((uint32_t)hashBytes[3] << 24);
    if (savedHash != scripts.syscallHash()) return SCRIPT_RESTORE_STALE;
  }

  File f = scriptFS.open(path, FILE_READ);
  if (!f) return SCRIPT_RESTORE_NONE;

  uint64_t fileSize = f.size();
  if (fileSize < 2 || fileSize > SCRIPT_MAX_CODE_LEN)
  {
    f.close();
    return SCRIPT_RESTORE_NONE;
  }

  ScriptSlot &s = scripts.slots[slot];
  size_t readBytesN = f.readBytes((char *)s.code, (size_t)fileSize);
  f.close();

  if (readBytesN != (size_t)fileSize) return SCRIPT_RESTORE_NONE;
  if (!finalizeScriptSlot(slot, (uint16_t)fileSize)) return SCRIPT_RESTORE_NONE;
  s.syscallHash = savedHash;
  return SCRIPT_RESTORE_OK;
}

// eraseScriptSlotFromFlash — remove a slot's .bin and .sig. True if neither
// exists afterwards (including "nothing was saved"); false only if the
// filesystem isn't mounted or a removal failed.
bool eraseScriptSlotFromFlash(uint8_t slot)
{
  if (!scriptFSReady || slot >= SCRIPT_SLOTS) return false;

  char path[20];
  char sigPath[20];
  scriptSlotPath(slot, path, sizeof(path));
  scriptSigPath(slot, sigPath, sizeof(sigPath));
  if (scriptFS.exists(path))    scriptFS.remove(path);
  if (scriptFS.exists(sigPath)) scriptFS.remove(sigPath);
  return !scriptFS.exists(path) && !scriptFS.exists(sigPath);
}

// =============================================================================
//  SECTION 6c — ETHERNET
// =============================================================================
//
//  5 TCP servers total, every one single-session ("latest connection wins" —
//  same policy every commercial serial-to-Ethernet device server uses,
//  because two live sessions driving the same downstream link would corrupt
//  each other's traffic regardless of arbitration):
//
//    ethCtrlServer / ethCtrlClient — mirrors SerialUSB1: registered with
//      cp.registerStream() once in EthernetSetup(), so every command
//      (GVER, GMODE, SDEV, SADCxxx, ...) works verbatim over this TCP port
//      with no command-processor changes. One port, data.EthCtrlPort.
//
//    ethDataServer[i] / ethDataClient[i] — one dedicated passthrough port per
//      downstream device (data.EthDataPortBase+0..3), always reachable
//      regardless of which device data.ActiveDevice/SDEV currently selects —
//      unlike Serial's single switchable path, Ethernet isn't endpoint-
//      limited, so every device gets its own port rather than sharing one.
//      A device's TCP client and a USB data port bridged to it share it
//      through the slot's arbiter (Section 6).
//
//  All client globals are reused across reconnects (`ethXClient = newClient`
//  inside EthernetUpdate() below) rather than replaced, because
//  cp.registerStream() has no matching "unregister"/"replace" call — it
//  records a Stream* once, so that address must stay the same object for the
//  life of the program (true of ethCtrlClient specifically; the per-device
//  ethDataClient[] array elements follow the same reuse pattern for
//  consistency even though nothing registers them with cp). QNEthernet's
//  EthernetClient supports this: it's a copyable handle around a connection,
//  and calling available()/connected() on one that was never connected (or
//  has since disconnected) safely returns 0/false rather than crashing or
//  blocking.
//
//  Simultaneous USB and TCP traffic to one device used to be able to
//  interleave on the wire ("last speaker wins" only routed replies). The
//  per-slot arbiter (Section 6) replaced that: lines are sent one at a time.
//
//  FLUSHING: QNEthernet queues written data until flush() or lwIP's 250 ms
//  TCP timer, so devPortFlush() and loop() flush TCP clients after writing
//  (measured ~250 ms -> ~10 ms per command); setNoDelay(true) on accept.
//
//  BAUD RATE: Serial.baud() (Section 6) only exists for a USB CDC host; a
//  pure-Ethernet client has no equivalent way to signal a baud rate, so
//  GBAUD/SBAUD (Section 5) exist for that case — set it once after boot if
//  you're not also using the USB side.

EthernetServer ethCtrlServer;                      // port bound in EthernetSetup() from data.EthCtrlPort
EthernetServer ethDataServer[DEVPORT_COUNT];        // port i bound from data.EthDataPortBase + i
EthernetClient ethCtrlClient;
EthernetClient ethDataClient[DEVPORT_COUNT];

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
  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    ethDataServer[i].begin(data.EthDataPortBase + i);
  }

  // Registered once; see the comment above for why the same object is
  // reused across reconnects instead of re-registering a new one.
  cp.registerStream(&ethCtrlClient);
}

static void EthernetUpdate(void)
{
  Ethernet.loop();   // "call often" per QNEthernet's docs — DHCP renewal etc.

  // accept() is non-blocking: it returns an unconnected client immediately
  // if nobody is trying to connect right now.
  // setNoDelay(true) turns off Nagle's algorithm: a reply often goes out in
  // two small writes (the ACK byte, then the value), and with Nagle the
  // second waits for the host to acknowledge the first, which a host's
  // delayed-ACK timer holds back ~200 ms. Measured: ~240 ms per command over
  // TCP with Nagle on, against ~6 ms over USB.
  EthernetClient nc = ethCtrlServer.accept();
  if (nc) { if (ethCtrlClient) ethCtrlClient.stop(); nc.setNoDelay(true); ethCtrlClient = nc; }

  for (uint8_t i = 0; i < DEVPORT_COUNT; i++)
  {
    EthernetClient nd = ethDataServer[i].accept();
    if (nd) { if (ethDataClient[i]) ethDataClient[i].stop(); nd.setNoDelay(true); ethDataClient[i] = nd; }
  }
}

// =============================================================================
//  SECTION 6d — DIGITAL I/O (DIO, SMA)
// =============================================================================
//
//  One external line (SMA connector), independent of the 4 downstream
//  ports — a board-level feature like the AIN input, not per-device. Can be
//  switched between input and output at runtime, from both the control
//  port (GDIODIR/SDIODIR, GDOUT/SDOUT, GDIN above) and scripts
//  (dio_set_dir()/dio_write()/dio_read() syscalls, registered in setup())
//  via the identical functions below — same pattern as
//  readAdcCounts()/sys_read_adc() (Section 5/6a/6b).
//
//  HARDWARE (confirmed via a kicad-cli-exported netlist of the real
//  schematic, same method used for the hub-strap pins in TODO.md):
//    Teensy pin 2  — DIO_OUT: data input ('A') to a 74AHCT1G125 tri-state
//      buffer. Only actually reaches the external line while DIO_OE_N is
//      driven low.
//    Teensy pin 3  — DIO_OE_N: that buffer's active-low output enable.
//      Pulled up to 3.3V on-board via R15 — defaults HIGH (disabled/input)
//      on its own, with no firmware involvement at all, including the boot
//      window before setup() runs. Firmware only ever needs to drive it
//      LOW to enable output; there's no undefined floating state to race.
//    Teensy pin 4  — DIO_IN: output ('Y') of a 74LVC1G17 Schmitt buffer
//      reading the external line — valid in both directions, so reading
//      it back while output is enabled can reveal a shorted/contended line
//      (see GDIN's comment above).
//
//  PERSISTENCE: only direction (data.DioOutputEnabled) is persisted. The
//  output level itself is not — it resets to LOW every boot until the host
//  or a script sets it. Direction is deliberately settable from scripts,
//  not just the control port: a script loaded at startup needs to be able
//  to configure its own direction without depending on the host having
//  already set it via SDIODIR.
//
//  BOOT SAFETY: dioInit() (called from setup(), Section 8, independent of
//  mode) still explicitly drives DIO_OE_N HIGH before touching DIO_OUT at
//  all, even though R15's pull-up already makes that the hardware default —
//  belt-and-suspenders, and it's what actually re-asserts the disabled
//  state before deciding whether to re-enable it per whatever direction was
//  restored from EEPROM.

#define DIO_OUT_PIN   2   // 74AHCT1G125 'A' — SDOUT / dio_write() drive this
#define DIO_OE_N_PIN  3   // 74AHCT1G125 OE-bar, active low, pulled up via R15 — SDIODIR / dio_set_dir() drive this
#define DIO_IN_PIN    4   // 74LVC1G17 'Y' — GDIN / dio_read() read this

bool dioOutLevel = false;   // last level SDOUT/dio_write() set; not persisted (see above)

// dioSetDirection — single point of truth for changing DIO direction, used
// by both SDIODIR (Section 5) and the dio_set_dir() script syscall (below).
void dioSetDirection(bool outputEnabled)
{
  data.DioOutputEnabled = outputEnabled;
  digitalWriteFast(DIO_OE_N_PIN, outputEnabled ? LOW : HIGH);   // active low
}

// dioWrite — drives DIO_OUT. Only reaches the external line while direction
// is output; harmless to call while direction is input.
void dioWrite(bool level)
{
  dioOutLevel = level;
  digitalWriteFast(DIO_OUT_PIN, level ? HIGH : LOW);
}

// dioRead — reads DIO_IN. Valid in both directions — see the header comment
// above for why that matters with output enabled.
bool dioRead(void)
{
  return digitalReadFast(DIO_IN_PIN) != 0;
}

// dioInit — boot-time setup, independent of mode (called once from setup(),
// Section 8, before command registration). Establishes the disabled/input
// state explicitly before anything else touches these pins, then re-applies
// whatever direction was persisted.
static void dioInit(void)
{
  pinMode(DIO_OE_N_PIN, OUTPUT);
  digitalWriteFast(DIO_OE_N_PIN, HIGH);   // disabled, before DIO_OUT is touched at all
  pinMode(DIO_OUT_PIN, OUTPUT);
  digitalWriteFast(DIO_OUT_PIN, LOW);
  pinMode(DIO_IN_PIN, INPUT);

  dioSetDirection(data.DioOutputEnabled);   // re-apply whatever was persisted
}

// Script syscalls — listed in the scriptSyscalls[] table in setup()
// (Section 8) alongside the ADC demo's (Section 6b). Direction is deliberately included here (not
// just read/write) per the design decision above: a script needs to be
// able to set up its own direction on load.
static int32_t sys_dio_set_dir(int32_t *args, uint8_t argc)
{
  (void)argc;
  dioSetDirection(args[0] != 0);
  return 1;
}

static int32_t sys_dio_write(int32_t *args, uint8_t argc)
{
  (void)argc;
  dioWrite(args[0] != 0);
  return 1;
}

static int32_t sys_dio_read(int32_t *args, uint8_t argc)
{
  (void)args; (void)argc;
  return dioRead() ? 1 : 0;
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

  // ── 2a. Digital I/O boot safety (Section 6d) ────────────────────────────────
  //  Independent of mode, same as the ADC config above. Must run before
  //  anything else so DIO_OE_N is explicitly driven disabled before DIO_OUT
  //  is touched at all — see Section 6d's header comment.
  dioInit();

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

  //  Script syscalls. THE table: each syscall's id is its index here, and the
  //  names/arg counts/return types are what GSYSCALLS reports to gsc.py, so a
  //  script written as `set = dio_read();` compiles against exactly this list
  //  with no declarations to keep in sync. Append new syscalls at the END —
  //  inserting or reordering renumbers the ones after it (and changes the
  //  table hash, so scripts compiled before are refused rather than run
  //  wrong; recompile them).  Sections 6b (ADC demo) and 6d (DIO) have the
  //  implementations.
  static const GAACEScript::SyscallDesc scriptSyscalls[] = {
    {"link_ready",  sys_link_ready,  0, false},   // id 0 — see Section 6b
    {"read_adc",    sys_read_adc,    0, false},   // id 1
    {"scale_send",  sys_scale_send,  1, false},   // id 2
    {"dio_set_dir", sys_dio_set_dir, 1, false},   // id 3 — see Section 6d
    {"dio_write",   sys_dio_write,   1, false},   // id 4
    {"dio_read",    sys_dio_read,    0, false},   // id 5
  };
  scripts.registerSyscalls(scriptSyscalls,
                           sizeof(scriptSyscalls) / sizeof(scriptSyscalls[0]));

  // ── 5b. Script slot persistence (flash; Section 6b) ─────────────────────────
  //  Mount the on-chip flash filesystem, then try restoring every slot from
  //  its saved file. A slot with no saved file (or a filesystem that fails
  //  to mount) just stays unloaded — except slot 0, which falls back to the
  //  compiled-in ADC demo below, exactly as before this feature existed.
  //  Restored scripts are left at Thread's own default (enabled=true,
  //  1000 mS interval, from ScriptRuntime's constructor) so a persisted
  //  script resumes running on its own at boot — the actual point of
  //  persistence (e.g. a script that sets up DIO direction on load, per the
  //  user's original motivating use case). This differs deliberately from
  //  the compiled-in demo's explicit enabled=false below, which exists for
  //  an unrelated, demo-specific reason (avoiding a double-send with
  //  ADCThread by default).
  scriptFSReady = scriptFS.begin(SCRIPT_FS_RESERVE_BYTES);
  bool slot0Restored = false;
  for (uint8_t i = 0; i < GAACE_SCRIPT_SLOTS; i++)
  {
    bool restored = (restoreScriptSlotFromFlash(i) == SCRIPT_RESTORE_OK);
    if (i == 0) slot0Restored = restored;
  }

  // Preload the ADC demo (Section 6b) into slot 0, the same way a project
  // preloads any default script: fill code[]/codeLen, vmInit(), loaded=true.
  // Only if slot 0 had no persisted script to restore instead (above).
  // Starts disabled -- see Section 6b for why (double-send risk with
  // ADCThread); enable with STENA,Script0,TRUE once ready to compare them.
  if (!slot0Restored)
  {
    memcpy(scripts.slots[0].code, scriptDemo, scriptDemo_len);
    scripts.slots[0].codeLen = scriptDemo_len;
    vmInit(scripts.slots[0].vm, scripts.slots[0].code, scripts.slots[0].codeLen);
    scripts.slots[0].loaded  = true;
    scripts.slots[0].enabled = false;
  }

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
  // Push any command reply to the TCP control client out now: QNEthernet
  // otherwise holds it for lwIP's 250 ms TCP timer (measured ~250 ms per
  // command on port 5000). Cheap when there's nothing queued.
  if (ethCtrlClient && ethCtrlClient.connected()) ethCtrlClient.flush();
  control.run();
  modeTable[currentMode].loop();
}
