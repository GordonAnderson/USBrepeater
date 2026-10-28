// =============================================================================
//  USB REPEATER — MODULE HEADER
//  Built on the GAACE (Gordon Anderson Arduino Core Extensions) framework
// =============================================================================
//
//  PlatformIO placement: include/USBrepeater.h
//  Included by:          src/USBrepeater.cpp
//
//  Contains: SIGNATURE constant, the SystemMode enum, the persistent Data
//  struct, and forward declarations for functions implemented in
//  src/USBrepeater.cpp.
// =============================================================================
#pragma once
#include <Arduino.h>

// SIGNATURE is written into EEPROM so we can detect un-initialised storage.
// Change this value if you ever change the layout of the Data struct — doing
// so forces a reset to defaults on the next boot rather than reading garbage.
// Bumped from 0xAA55A5A9 when ActiveDevice/AdcTargetDevice were added below
// for the 4-downstream-port USBrepeater Hub Carrier board.
#define SIGNATURE  0xAA55A5AA

// Number of downstream USB ports on the Hub Carrier board (one USB2514B
// 4-port hub). ActiveDevice/AdcTargetDevice below are 1-based indices into
// this range. See src/USBrepeater.cpp Section 6 for the per-port state this
// drives (userial1..userial4, one ring buffer each).
#define DEVPORT_COUNT 4

// ── Operating modes ───────────────────────────────────────────────────────
//  Add new modes by appending to this enum (before NUM_MODES) and adding a
//  matching entry to modeTable[] in USBrepeater.cpp.  Each mode supplies a
//  setup() and loop() function; the dispatcher in loop() calls whichever
//  pair is active.
enum SystemMode
{
  MODE_REPEATER = 0,   // PC <-> Teensy host port <-> downstream USB CDC device
  NUM_MODES
};

// ── Persistent data structure ─────────────────────────────────────────────
//  Every field in Data{} is saved to and restored from EEPROM (Teensy 4.1's
//  emulated EEPROM survives firmware uploads). The Signature field is always
//  written last so that a power-loss during a write does not cause a
//  partially-updated record to be accepted on the next boot.
typedef struct
{
  int16_t       Size;           // Byte count of this struct — used for version detection
  char          Name[20];       // Human-readable board name, settable via SNAME
  int8_t        Rev;            // Firmware/hardware revision number
  uint8_t       Mode;           // Active SystemMode, persisted across reboots

  // ── Downstream device selection (Hub Carrier board, 4 ports) ─────────────
  //  See USBrepeater.cpp Section 6. ActiveDevice is which downstream port the
  //  single switchable Serial upstream path is bridged to, settable via
  //  SDEV,<n> (each port also has its own always-reachable dedicated
  //  Ethernet data port, independent of this selection — see EthDataPortBase
  //  below). AdcTargetDevice is which port ADCThread/the
  //  GAACE_Script demo sends to — independent of ActiveDevice on purpose:
  //  switching which device the host is interactively looking at must not
  //  interrupt a background ADC feed to a different one. Both are 1-based,
  //  range [1, DEVPORT_COUNT].
  uint8_t       ActiveDevice;
  uint8_t       AdcTargetDevice;

  // ── Periodic ADC -> downstream-device command update ────────────────────
  //  Runs on its own thread (see ADCThread in USBrepeater.cpp), independent
  //  of the active mode, so it keeps running while REPEATER mode is active.
  //  Each cycle: scaledValue = AdcScaleM * analogRead(AdcPin) + AdcScaleB,
  //  sent to the downstream device as "<AdcCmdName>,<scaledValue>\n" — the
  //  same "CMD,arg\n" convention every other GAACE command uses.
  bool          AdcEnabled;     // Master enable, settable via SADCEN,TRUE|FALSE
  uint8_t       AdcPin;         // Analog input pin (e.g. A0), settable via SADCPIN
  uint32_t      AdcInterval;    // Update period in mS, settable via SADCINT
  char          AdcCmdName[20]; // Command name sent to the downstream device
  float         AdcScaleM;      // value = AdcScaleM * counts + AdcScaleB
  float         AdcScaleB;

  // Note: the GAACE_Script ADC demo (USBrepeater.cpp Section 6b) has no
  // fields here. Its enable state and update rate are GAACEScript::
  // ScriptRuntime's own Thread::enabled and interval for slot "Script0"
  // (GTENA/STENA,Script0 and GTINT/STINT,Script0) rather than persisted
  // Data fields — see TODO.md for why that's RAM-only for now.

  // ── Ethernet (control port + one dedicated data port per downstream device) ──
  //  See USBrepeater.cpp Section 6c. EthIP/EthSubnet/EthGateway are packed
  //  IPAddress values (uint32_t), used only when EthDHCP is false. All five
  //  fields take effect on the next boot (EthernetSetup() in setup()), not
  //  live — same "change now, SAVE, reboot to apply" convention as the
  //  Template's STWIADD. EthCtrlPort is a second command-processor stream
  //  (same commands as SerialUSB1, over TCP). EthDataPortBase is device 1's
  //  dedicated passthrough TCP port; devices 2-4 use EthDataPortBase+1,
  //  +2, +3 — 4 independent, always-reachable ports, not one switchable
  //  port like Serial/SDEV, since Ethernet isn't endpoint-limited the way
  //  USB is. See `pcSide`/ethDataClient[], Section 6/6c.
  bool          EthDHCP;        // TRUE = DHCP, FALSE = the three static fields below
  uint32_t      EthIP;          // Static IP (only used when EthDHCP == FALSE)
  uint32_t      EthSubnet;      // Static subnet mask
  uint32_t      EthGateway;     // Static gateway
  int           EthCtrlPort;    // TCP port for the network control port (commandProcessor)
  int           EthDataPortBase; // TCP port for device 1's passthrough; devices 2-4 use +1..+3

  unsigned int  Signature;      // Must equal SIGNATURE for the struct to be considered valid
} Data;

// Forward declarations — implementations are in src/USBrepeater.cpp.
void SaveSettings(void);
void RestoreSettings(void);

void RepeaterModeSetup(void);
void RepeaterModeLoop(void);

void ADCUpdate(void);
