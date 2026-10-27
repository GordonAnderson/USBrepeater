#include "UsbHubBoot.h"

namespace UsbHubBoot {

static void driveStrap(uint8_t pin, bool level) {
  digitalWrite(pin, level ? HIGH : LOW);
  pinMode(pin, OUTPUT);
}

static void releaseStrap(uint8_t pin) {
  pinMode(pin, INPUT);  // no internal pull; R10/R11 hold 18/19 high afterwards
}

void attach() {
  digitalWrite(PIN_VBUS_DET, HIGH);
  pinMode(PIN_VBUS_DET, OUTPUT);
}

void detach() {
  digitalWrite(PIN_VBUS_DET, LOW);
  pinMode(PIN_VBUS_DET, OUTPUT);
}

void run(const Options &opt) {
  // 0. Keep the hub detached from the upstream port during reset/strapping.
  detach();

  // 1. Put the hub in reset first so it cannot sample partial strap levels.
  digitalWrite(PIN_RESET_N, LOW);
  pinMode(PIN_RESET_N, OUTPUT);

  // 2. Present the strap levels while reset is asserted.
  driveStrap(PIN_CFG_SEL0, false);                 // CFG_SEL0 = 0 -> strap options enabled
  driveStrap(PIN_CFG_SEL1, false);                 // CFG_SEL1 = 0 -> default configuration
  driveStrap(PIN_NON_REM1, (opt.nonRem >> 1) & 1);
  driveStrap(PIN_NON_REM0, opt.nonRem & 1);

  // 3. Hold reset (RC-filtered through R12/C8) with straps stable.
  delay(opt.resetLowMs);

  // 4. Release reset. Straps are sampled at RESET_N negation.
  digitalWrite(PIN_RESET_N, HIGH);

  // 5. Keep the straps valid past negation plus hold time.
  delay(opt.strapHoldMs);

  // 6. Let go of the straps. 34/35 turn into hub outputs (HS_IND, SUSP_IND),
  //    so they must not stay driven by the Teensy. RESET_N stays driven high.
  releaseStrap(PIN_CFG_SEL1);
  releaseStrap(PIN_NON_REM0);
  releaseStrap(PIN_CFG_SEL0);
  releaseStrap(PIN_NON_REM1);
}

}  // namespace UsbHubBoot
