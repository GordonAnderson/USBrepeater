"""
PlatformIO pre-build script: overrides the USB Manufacturer/Product strings
Teensyduino's core reports for the USB_DUAL_SERIAL descriptor.

WHY THIS EXISTS
----------------
Unlike PlatformIO's SAMD platform (board_build.usb_product / board_vendor),
the "teensy" platform has no project-level hook for these strings. They are
hardcoded in framework-arduinoteensy's cores/teensy4/usb_desc.h, one
Manufacturer + one Product string shared by the whole composite USB device
(both CDC ports show the same name; Teensyduino's dual/triple-serial
descriptors do not support distinct per-interface names — every iInterface/
iFunction field is hardcoded to 0).

This script patches that shared framework file in place, in the
USB_DUAL_SERIAL branch only, every time PlatformIO builds. It is idempotent
(checks for its own marker before touching the file) so re-running it, or
running it against an already-patched file, is a no-op. Because it runs from
this project via extra_scripts, a fresh `git clone` + `pio run` reproduces the
patch automatically — no manual toolchain edits required.

CAVEAT: this edits a file inside the shared PlatformIO package cache
(~/.platformio/packages/framework-arduinoteensy), not a copy scoped to this
project. If that package is reinstalled/updated, this script re-applies the
patch on the next build; but a *different* project sharing that same cached
package install would also see this Manufacturer/Product string while it
builds with USB_DUAL_SERIAL, until that project's own build overwrites it
back (or not, if it doesn't carry this script).
"""

import re

Import("env")

MARKER = "GAACE USBrepeater build patch"

MANUFACTURER = "GAA Custom Electronics, LLC"
PRODUCT = "USBrepeater"


def c_char_array(s):
    return "{" + ",".join("'%s'" % ch for ch in s) + "}"


def patch(path):
    with open(path, "r") as f:
        text = f.read()

    if MARKER in text:
        return  # already patched

    block_re = re.compile(
        r"(#elif defined\(USB_DUAL_SERIAL\)\n)"
        r"(.*?)"
        r"(\n#elif defined\(USB_TRIPLE_SERIAL\))",
        re.DOTALL,
    )

    m = block_re.search(text)
    if not m:
        print("patch_usb_desc.py: USB_DUAL_SERIAL block not found, skipping")
        return

    block = m.group(2)
    block = re.sub(
        r"#define MANUFACTURER_NAME\s+\{.*?\}",
        "#define MANUFACTURER_NAME     %s  // %s" % (c_char_array(MANUFACTURER), MARKER),
        block,
    )
    block = re.sub(
        r"#define MANUFACTURER_NAME_LEN\s+\d+",
        "#define MANUFACTURER_NAME_LEN %d" % len(MANUFACTURER),
        block,
    )
    block = re.sub(
        r"#define PRODUCT_NAME\s+\{.*?\}",
        "#define PRODUCT_NAME          %s" % c_char_array(PRODUCT),
        block,
    )
    block = re.sub(
        r"#define PRODUCT_NAME_LEN\s+\d+",
        "#define PRODUCT_NAME_LEN      %d" % len(PRODUCT),
        block,
    )

    new_text = text[: m.start(2)] + block + text[m.end(2) :]

    with open(path, "w") as f:
        f.write(new_text)

    print("patch_usb_desc.py: patched USB_DUAL_SERIAL Manufacturer/Product strings")


platform = env.PioPlatform()
framework_dir = platform.get_package_dir("framework-arduinoteensy")
if framework_dir:
    usb_desc_h = framework_dir + "/cores/teensy4/usb_desc.h"
    try:
        patch(usb_desc_h)
    except IOError as e:
        print("patch_usb_desc.py: could not patch %s: %s" % (usb_desc_h, e))
else:
    print("patch_usb_desc.py: framework-arduinoteensy package not found, skipping")
