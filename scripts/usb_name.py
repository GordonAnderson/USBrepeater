"""PlatformIO pre-build script: set the USB manufacturer/product strings.

The Teensy core hard-codes these in cores/teensy4/usb_desc.h, one block per USB
type. This script rewrites the block for the active USB type with the values from
platformio.ini:

    board_vendor           = <manufacturer string>
    board_build.usb_product = <product string>

It runs on every build and sets the values explicitly, so several projects with
different names can share one PlatformIO package install. The patched file lives in
the PlatformIO package folder, not in this repository.
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
