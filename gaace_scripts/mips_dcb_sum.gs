// mips_dcb_sum.gs: reads DC bias channels 1 and 5 on the MIPS and sets channel 2 to their sum.
//
// Runs on the GAACE Gateway (USBrepeater) and talks to the MIPS by its name (GNAME, "MIPS" by
// default; change the string below if the box has another name). Each run (one thread tick):
//   ch2 = GDCBV,1 + GDCBV,5      (readbacks, in volts)
// Channel 2 is only written when the sum has moved more than 0.25 V since the last write, so
// readback noise does not cause a write every tick.
//
// Upload to slot 0 through the gateway's control port, then start it:
//   python3 ../GAACE_Script/tools/gsc.py gaace_scripts/mips_dcb_sum.gs --upload <control port> --slot 0
//   STINT,Script0,200
//   STENA,Script0,TRUE
// SCRIPTSAVE,0 keeps it across a reboot.

var last: float;

m = device("MIPS");
if (ready(m)) {
  v1 = dcmdf(m, "GDCBV", 1);
  v5 = dcmdf(m, "GDCBV", 5);
  sum = v1 + v5;
  d = sum - last;
  if (d < 0.0) { d = -d; }
  if (d > 0.25) {
    dcmd(m, "SDCB", 2, sum);
    last = sum;
  }
}
