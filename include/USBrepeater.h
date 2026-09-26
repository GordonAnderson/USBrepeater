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
// Bumped from 0xAA55A5A6 when the script-demo fields were added below.
#define SIGNATURE  0xAA55A5A7

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

  // ── GAACE_Script ADC demo (see USBrepeater.cpp Section 6b) ──────────────
  //  Runs gaace_scripts/adc_demo.gs on its own thread, alongside ADCThread.
  //  Reuses AdcPin/AdcScaleM/AdcScaleB/AdcCmdName above — it's the same
  //  conceptual feature, demonstrated via script instead of hardcoded C++.
  //  Defaults off: enabling both this and AdcEnabled against the same
  //  downstream command would double-send.
  bool          ScriptEnabled;  // Master enable, settable via SSCRIPTEN,TRUE|FALSE
  uint32_t      ScriptInterval; // Update period in mS, settable via SSCRIPTINT

  unsigned int  Signature;      // Must equal SIGNATURE for the struct to be considered valid
} Data;

// Forward declarations — implementations are in src/USBrepeater.cpp.
void SaveSettings(void);
void RestoreSettings(void);

void RepeaterModeSetup(void);
void RepeaterModeLoop(void);

void ADCUpdate(void);
void ScriptUpdate(void);
