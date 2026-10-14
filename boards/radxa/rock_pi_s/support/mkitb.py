#!/usr/bin/env python3
# Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
# SPDX-License-Identifier: Apache-2.0
"""
Build a U-Boot SPL FIT image (u-boot.itb replacement) that starts Zephyr as
BL33 straight from TF-A BL31 -- no OP-TEE, no U-Boot proper.

Layout matches U-Boot's rockchip-u-boot.dtsi (binman) so the SPL loads it
unchanged:

  firmware  = atf-1          (BL31 entry, SPL jumps here)
  loadables = zephyr, atf-2..  (rest of BL31)

spl_invoke_atf() takes the BL33 entry from the image with os = "u-boot",
hence the Zephyr image carries that os type.

BL31 (bl31.elf) is split into one image per PT_LOAD segment, like binman's
"split-elf". Data is stored externally, 512-byte aligned, and every image
gets a sha256 hash (RK3308 SPL implies SPL_FIT_SIGNATURE).

A BL33 FDT is NOT included by default: our Rockchip BL31 does not read its
console from an FDT, and Zephyr carries its own devicetree. Pass --fdt only
if your SPL build insists on an fdt image in the config (any valid dtb, e.g.
the built zephyr.dtb, works -- Zephyr ignores the runtime dtb).
"""

import argparse
import os
import subprocess
import sys
import tempfile

from elftools.elf.elffile import ELFFile


def elf_segments(path, outdir, prefix):
    """Write every loadable segment to its own file; return
    [(file, load)] and the ELF entry point."""
    out = []
    with open(path, "rb") as f:
        elf = ELFFile(f)
        entry = elf.header["e_entry"]
        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD" or seg["p_filesz"] == 0:
                continue
            name = os.path.join(outdir, f"{prefix}-{len(out) + 1}.bin")
            with open(name, "wb") as o:
                o.write(seg.data())
            out.append((name, seg["p_paddr"]))
    if not out:
        sys.exit(f"{path}: no loadable segments")
    return out, entry


def image_node(name, desc, typ, os_, path, load, entry=None):
    lines = [
        f"\t\t{name} {{",
        f'\t\t\tdescription = "{desc}";',
        f'\t\t\tdata = /incbin/("{path}");',
        f'\t\t\ttype = "{typ}";',
        '\t\t\tarch = "arm64";',
        f'\t\t\tos = "{os_}";',
        '\t\t\tcompression = "none";',
        f"\t\t\tload = <0x{load:08x}>;",
    ]
    if entry is not None:
        lines.append(f"\t\t\tentry = <0x{entry:08x}>;")
    lines += [
        "\t\t\thash {",
        '\t\t\t\talgo = "sha256";',
        "\t\t\t};",
        "\t\t};",
    ]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zephyr", required=True, help="zephyr.bin")
    ap.add_argument("--load", type=lambda x: int(x, 0), required=True,
                    help="Zephyr load/entry address")
    ap.add_argument("--bl31", required=True, help="TF-A bl31.elf")
    ap.add_argument("--fdt", help="optional DTB image for BL33 (usually not needed)")
    ap.add_argument("--mkimage", default="mkimage", help="mkimage binary")
    ap.add_argument("-o", "--output", required=True, help="output .itb")
    ap.add_argument("--its", help="also keep the generated .its here")
    args = ap.parse_args()

    for p in (args.zephyr, args.bl31, args.fdt):
        if p and not os.path.isfile(p):
            sys.exit(f"{p}: not found")

    with tempfile.TemporaryDirectory() as tmp:
        atf, atf_entry = elf_segments(args.bl31, tmp, "atf")

        nodes = [image_node("zephyr", "Zephyr (BL33)", "standalone", "u-boot",
                            os.path.abspath(args.zephyr), args.load, args.load)]
        atf_names = []
        for i, (path, load) in enumerate(atf, 1):
            atf_names.append(f"atf-{i}")
            nodes.append(image_node(f"atf-{i}", "ARM Trusted Firmware", "firmware",
                                    "arm-trusted-firmware", path, load,
                                    atf_entry if i == 1 else None))
        if args.fdt:
            nodes.append("\n".join([
                "\t\tfdt-1 {",
                '\t\t\tdescription = "fdt for BL31";',
                f'\t\t\tdata = /incbin/("{os.path.abspath(args.fdt)}");',
                '\t\t\ttype = "flat_dt";',
                '\t\t\tcompression = "none";',
                "\t\t\thash {",
                '\t\t\t\talgo = "sha256";',
                "\t\t\t};",
                "\t\t};",
            ]))

        loadables = ["zephyr"] + atf_names[1:]
        cfg = [
            "\t\tconfig-1 {",
            '\t\t\tdescription = "zephyr";',
            '\t\t\tfirmware = "atf-1";',
            "\t\t\tloadables = " + ", ".join(f'"{n}"' for n in loadables) + ";",
        ]
        if args.fdt:
            cfg.append('\t\t\tfdt = "fdt-1";')
        cfg.append("\t\t};")

        its = "\n".join([
            "/dts-v1/;",
            "",
            "/ {",
            '\tdescription = "Zephyr as BL33 behind TF-A (RK3308)";',
            "\t#address-cells = <1>;",
            "",
            "\timages {",
            "\n\n".join(nodes),
            "\t};",
            "",
            "\tconfigurations {",
            '\t\tdefault = "config-1";',
            "\n".join(cfg),
            "\t};",
            "};",
            "",
        ])

        its_path = os.path.join(tmp, "zephyr.its")
        with open(its_path, "w") as f:
            f.write(its)
        if args.its:
            with open(args.its, "w") as f:
                f.write(its)

        # external data, 512-byte aligned like binman's fit,align = <512>
        r = subprocess.run([args.mkimage, "-E", "-B", "0x200", "-f", its_path,
                            args.output], capture_output=True, text=True)
        if r.returncode != 0:
            sys.stderr.write(r.stdout + r.stderr)
            sys.exit(f"mkimage failed ({r.returncode})")


if __name__ == "__main__":
    main()
