// adc_demo.gs — GAACE_Script demo for USBrepeater's ADC feature.
//
// Compiled with (from the GAACE_Script repo checked out alongside this one):
//   python3 ../GAACE_Script/tools/gsc.py gaace_scripts/adc_demo.gs \
//       --format carray --carray-name scriptDemo -o include/ScriptBytecode.h
//
// Re-run that command and commit the regenerated include/ScriptBytecode.h
// whenever this file changes — there is no build-time codegen step yet
// (see USBrepeater/TODO.md, "Host-side compiler" open item).
//
// Syscall ids below must match the vmRegisterSyscall() call order in
// src/USBrepeater.cpp, Section 6b:
//   0 = link_ready()     -- userial connected AND quiet-guard elapsed
//   1 = read_adc()        -- readAdcCounts() rounded to int32 (12-bit, hardware
//                             + software averaged -- see src/USBrepeater.cpp)
//   2 = scale_send(counts) -- data.AdcScaleM*counts+data.AdcScaleB, sent to
//                             the downstream device exactly like ADCUpdate()
//
// Unlike ADCUpdate() (Section 6a), which sends unconditionally every tick,
// this only sends when the reading has moved more than 20 counts since the
// last send -- real conditional logic living in the script, not hardcoded
// in C++. (20, not the original 5, because src/USBrepeater.cpp's ADC read
// is now 12-bit instead of 10-bit -- see readAdcCounts()'s comment there.)
// It's a demonstration running alongside ADCThread, not a replacement --
// see the Section 6b comment for why both shouldn't be enabled against the
// same downstream command at once.

syscall link_ready() = 0;
syscall read_adc() = 1;
syscall scale_send(counts) = 2;

var prev;

counts = read_adc();
delta = counts - prev;
if (delta < 0) { delta = -delta; }

if (link_ready() && delta > 20) {
  scale_send(counts);
  prev = counts;
}
