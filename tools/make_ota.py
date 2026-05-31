#!/usr/bin/env python3
"""
Wrap an ESP-IDF app .bin into a Zigbee OTA (.ota) file for Zigbee2MQTT.

Usage:
  python tools/make_ota.py build/firebeetle_tank_zigbee.bin tank2_v1.0.1.ota --version 0x01000001

The --version MUST be higher than the version the device is currently running
(OTA_FW_VERSION in main/app_config.h) or Z2M will not offer the update.
--manuf and --image-type must match OTA_MANUF_CODE / OTA_IMAGE_TYPE in the firmware.

Prints the metadata you need for the Z2M override index (size + sha512).
"""
import argparse
import hashlib
import json
import struct

OTA_FILE_IDENTIFIER = 0x0BEEF11E
HEADER_LENGTH = 56
ZIGBEE_STACK_VERSION = 0x0002


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="app .bin")
    ap.add_argument("output", help="output .ota")
    ap.add_argument("--manuf", type=lambda x: int(x, 0), default=0x1224)
    ap.add_argument("--image-type", type=lambda x: int(x, 0), default=0x1011)
    ap.add_argument("--version", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--desc", default="FB2-C5-TANK")
    args = ap.parse_args()

    with open(args.input, "rb") as f:
        app = f.read()

    total = HEADER_LENGTH + 6 + len(app)          # header + subelement(2+4) + image
    desc = args.desc.encode()[:32].ljust(32, b"\x00")
    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_FILE_IDENTIFIER, 0x0100, HEADER_LENGTH, 0x0000,
        args.manuf, args.image_type, args.version, ZIGBEE_STACK_VERSION,
        desc, total,
    )
    subelement = struct.pack("<HI", 0x0000, len(app)) + app
    ota = header + subelement

    with open(args.output, "wb") as f:
        f.write(ota)

    sha = hashlib.sha512(ota).hexdigest()
    print(f"wrote {args.output}: {len(ota)} bytes (image {len(app)} bytes)")
    print("Z2M override-index entry:")
    print(json.dumps([{
        "fileVersion": args.version,
        "fileSize": len(ota),
        "manufacturerCode": args.manuf,
        "imageType": args.image_type,
        "sha512": sha,
        "url": args.output,
    }], indent=2))


if __name__ == "__main__":
    main()
