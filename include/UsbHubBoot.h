// UsbHubBoot.h - USB2514B boot/strap sequencing for the USBrepeater board.
//
// The hub is run in "default configuration, strap options enabled":
//   CFG_SEL1 = 0, CFG_SEL0 = 0 at RESET_N negation
//   NON_REM1/NON_REM0 = 00 (all ports removable) unless changed below.
// There are no hardware strap resistors, so the Teensy drives the strap pins
// while it holds the hub in reset, releases reset, then lets go of the straps.
//
// Pin map (from the USBrepeater schematic):
//   Teensy 18  -> U6.22 SDA/SMBDATA/NON_REM1   (R11 4.7k pull-up to 3V3)
//   Teensy 19  -> U6.24 SCL/SMBCLK/CFG_SEL0    (R10 4.7k pull-up to 3V3)
//   Teensy 34  -> U6.25 HS_IND/CFG_SEL1        (becomes a hub OUTPUT after reset)
//   Teensy 35  -> U6.28 SUSP_IND/NON_REM0      (becomes a hub OUTPUT after reset)
//   Teensy 36  -> R12 (100k) -> U6.26 RESET_N, with C8 0.1uF to GND
//   Teensy 32  -> R7 (4.7k) -> U6.27 VBUS_DET  (R8 100k pull-down keeps it low)
//     VBUS_DET high = hub attaches to the Teensy host port; low = detached.
//
// Do not enable Serial8 (pins 34/35) or Wire (pins 18/19) before run() returns.
#pragma once
#include <Arduino.h>

namespace UsbHubBoot {

constexpr uint8_t PIN_NON_REM1 = 18;
constexpr uint8_t PIN_CFG_SEL0 = 19;
constexpr uint8_t PIN_CFG_SEL1 = 34;
constexpr uint8_t PIN_NON_REM0 = 35;
constexpr uint8_t PIN_RESET_N  = 36;
constexpr uint8_t PIN_VBUS_DET = 32;

struct Options {
  // NON_REM[1:0]: 0 = all ports removable, 1 = port 1 non-removable,
  //               2 = ports 1-2, 3 = ports 1-3.
  uint8_t  nonRem        = 0;
  // RESET_N is filtered by R12*C8 (tau ~= 10 ms). Keep it low well past 5*tau.
  uint16_t resetLowMs    = 100;
  // Time to keep the straps valid after RESET_N is driven high. RESET_N crosses
  // its input threshold roughly 10 ms after the drive goes high (RC), so this
  // must cover that plus the datasheet strap hold time (CHECK its RESET_N
  // timing figure for default configuration and adjust if needed).
  uint16_t strapHoldMs   = 50;
};

// Runs the full sequence. Blocks for about resetLowMs + strapHoldMs.
// Leaves RESET_N driven high, the four strap pins as high-impedance inputs,
// and VBUS_DET low (hub detached). Call attach() afterwards.
void run(const Options &opt = Options());

// Attach / detach the hub on the upstream (Teensy host) port by driving
// VBUS_DET. run() leaves the hub detached; call attach() once the USB host
// stack is up. detach() followed by attach() forces a clean re-enumeration
// without re-strapping the hub.
void attach();
void detach();

// Pulse the hub through a full re-strap/reset (call this any time the Teensy
// wants to restart the hub; the hub keeps its old config otherwise).
inline void restart(const Options &opt = Options()) { run(opt); }

}  // namespace UsbHubBoot
