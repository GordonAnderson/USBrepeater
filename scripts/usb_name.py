"""PlatformIO pre-build script: set the USB manufacturer/product strings, and
optionally a name for each serial port.

The Teensy core hard-codes these in cores/teensy4/usb_desc.h, one block per USB
type. This script rewrites the block for the active USB type with the values from
platformio.ini:

    board_vendor           = <manufacturer string>
    board_build.usb_product = <product string>

It runs on every build and sets the values explicitly, so several projects with
different names can share one PlatformIO package install. The patched file lives in
the PlatformIO package folder, not in this repository.

Port names (USB_TRIPLE_SERIAL / USB_DUAL_SERIAL / USB_SERIAL):

    custom_usb_interface_names = <Serial name>, <SerialUSB1 name>, <SerialUSB2 name>

All the ports share one product string, so a host can't tell them apart. With
this option each CDC port also gets its own name (the USB interface string, and
the interface-association "function" string Windows uses), so a host sees e.g.
"GAACE Gateway Control". This patches cores/teensy4/usb_desc.c: the iFunction /
iInterface fields of each CDC port become GAACE_CDC_STR_<n>, and the strings are
added as descriptors 4, 5, 6. All of it is inside #ifdef GAACE_CDC_NAMES, which
this script defines only for a project that sets the option, so other projects
sharing the package build exactly as before (their ports stay unnamed).
"""
import re
Import("env")

vendor  = env.GetProjectOption("board_vendor", None)
product = env.GetProjectOption("board_build.usb_product", None)

def usb_type():
    # USB type comes from "-D USB_xxx" in build_flags, Teensy default is USB_SERIAL
    for flag in env.GetProjectOption("build_flags", []):
        m = re.search(r"-D\s*(USB_[A-Z0-9_]+)", flag)
        if m:
            return m.group(1)
    return "USB_SERIAL"

def c_chars(text):
    return "{" + ",".join("'%s'" % ("\\'" if c == "'" else c) for c in text) + "}"

def patch(path, utype):
    lines = open(path).read().split("\n")
    # Find the block for this USB type, then edit the first matching defines after it
    start = next((i for i, l in enumerate(lines) if re.match(r"\s*#\s*(el)?if defined\(%s\)" % utype, l)), None)
    if start is None:
        print("usb_name: no block for %s in usb_desc.h, not patched" % utype)
        return
    wanted = {"MANUFACTURER_NAME": vendor, "PRODUCT_NAME": product}
    done = set()
    for i in range(start + 1, len(lines)):
        if re.match(r"\s*#\s*(el)?if defined\(USB_", lines[i]):
            break                                   # reached the next USB type
        for name, text in wanted.items():
            if text is None or name in done:
                continue
            m = re.match(r"(\s*#define\s+%s\s+)\{.*?\}(.*)$" % name, lines[i])
            if m:
                lines[i] = m.group(1) + c_chars(text) + m.group(2)
                lines[i + 1] = re.sub(r"\d+\s*$", str(len(text)), lines[i + 1].rstrip())
                done.add(name)
    new = "\n".join(lines)
    if new != open(path).read():
        open(path, "w").write(new)
        print("usb_name: patched %s for %s" % (path, utype))

if vendor or product:
    core = env.PioPlatform().get_package_dir("framework-arduinoteensy")
    patch(core + "/cores/teensy4/usb_desc.h", usb_type())


# ── Per-port names ──────────────────────────────────────────────────────────
DEFS_BEGIN = "/* GAACE_CDC_NAMES defs begin (written by a project's usb_name.py) */"
DEFS_END   = "/* GAACE_CDC_NAMES defs end */"
LIST_BEGIN = "/* GAACE_CDC_NAMES list begin */"
LIST_END   = "/* GAACE_CDC_NAMES list end */"
PORTS = ["CDC", "CDC2", "CDC3"]   # Serial, SerialUSB1, SerialUSB2

def wide_chars(text):
    return "{" + ",".join("'%s'" % ("\\'" if c == "'" else c) for c in text) + "}"

def replace_block(text, begin, end, block, anchor, after=False):
    """Remove any earlier copy of the marked block, then put it on its own lines
    just before (or just after) the anchor line."""
    if begin in text:
        a = text.index(begin); b = text.index(end, a) + len(end)
        if text[b:b + 1] == "\n":
            b += 1
        text = text[:a] + text[b:]
    i = text.index(anchor)
    if after:
        i = text.index("\n", i) + 1
    else:
        i = text.rfind("\n", 0, i) + 1
    return text[:i] + block + "\n" + text[i:]

def patch_names(path, names):
    text = open(path).read()
    defs = [DEFS_BEGIN, "#ifdef GAACE_CDC_NAMES"]
    for k, port in enumerate(PORTS):
        name = names[k] if k < len(names) else ""
        if name:
            defs.append("#define GAACE_CDC_STR_%s %d" % (port, 4 + k))
            defs.append("PROGMEM const struct usb_string_descriptor_struct gaace_cdc_name_%s = {2 + %d * 2, 3, %s};"
                        % (port, len(name), wide_chars(name)))
        else:
            defs.append("#define GAACE_CDC_STR_%s 0" % port)
    defs += ["#else"] + ["#define GAACE_CDC_STR_%s 0" % p for p in PORTS] + ["#endif", DEFS_END]
    # after the includes: PROGMEM comes from avr/pgmspace.h
    text = replace_block(text, DEFS_BEGIN, DEFS_END, "\n".join(defs), '#include "avr/pgmspace.h"', after=True)

    entries = [LIST_BEGIN, "#ifdef GAACE_CDC_NAMES"]
    for k, port in enumerate(PORTS):
        if k < len(names) and names[k]:
            entries.append("        {0x%04X, 0x0409, (const uint8_t *)&gaace_cdc_name_%s, 0}," % (0x0304 + k, port))
    entries += ["#endif", LIST_END]
    text = replace_block(text, LIST_BEGIN, LIST_END, "\n".join(entries),
                         "{0x0300, 0x0000, (const uint8_t *)&string0, 0},")

    # iFunction (interface association) and iInterface (status + data) of each port
    lines = text.split("\n"); pending = None; fields = 0
    for i, l in enumerate(lines):
        m = re.search(r"\b(CDC[23]?)_(STATUS|DATA)_INTERFACE,\s*// b(FirstInterface|InterfaceNumber)", l)
        if m:
            pending = m.group(1); continue
        if pending and re.search(r"//\s*i(Function|Interface)\b", l):
            lines[i] = re.sub(r"^(\s*)(0|GAACE_CDC_STR_CDC[23]?),", r"\g<1>GAACE_CDC_STR_%s," % pending, l)
            fields += 1; pending = None
    new = "\n".join(lines)
    if new != open(path).read():
        open(path, "w").write(new)
        print("usb_name: port names patched into %s (%d descriptor fields)" % (path, fields))

port_names = env.GetProjectOption("custom_usb_interface_names", None)
if port_names:
    names = [n.strip() for n in port_names.split(",")]
    core = env.PioPlatform().get_package_dir("framework-arduinoteensy")
    patch_names(core + "/cores/teensy4/usb_desc.c", names)
    env.Append(CPPDEFINES=["GAACE_CDC_NAMES"])
