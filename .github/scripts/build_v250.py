#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import shutil
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "brWheel_my" / "Config.h"
SKETCH_DIR = ROOT / "brWheel_my"
LEO_HEX_DIR = ROOT / "brWheel_my" / "leonardo hex" / "fw_v250"
PRO_HEX_DIR = ROOT / "brWheel_my" / "promicro hex" / "fw_v250"
DIST_DIR = ROOT / "dist"

LIB_DIR = Path(os.environ.get("ARDUINO_LIB_DIR", Path.home() / "Arduino" / "libraries"))

DEFINE_MAP = {
    "a": "USE_AUTOCALIB",
    "b": "USE_TWOFFBAXIS",
    "w": "USE_AS5600",
    "z": "USE_ZINDEX",
    "h": "USE_HATSWITCH",
    "s": "USE_ADS1015",
    "t": "USE_BTNMATRIX",
    "f": "USE_XY_SHIFTER",
    "i": "AVG_INPUTS",
    "e": "USE_EXTRABTN",
    "x": "USE_ANALOGFFBAXIS",
    "l": "USE_LOAD_CELL",
    "g": "USE_MCP4725",
    "u": "USE_TCA9548",
    "c": "USE_CENTERBTN",
    "k": "USE_SPLITAXIS",
}

SPECIAL_DEFINES = {
    "USE_SHIFT_REGISTER",
    "USE_SN74ALS166N",
    "USE_PROMICRO",
    "USE_EEPROM",
    "USE_QUADRATURE_ENCODER",
}


def set_define(lines: list[str], name: str, enabled: bool) -> list[str]:
    pattern = re.compile(rf"^(\s*)(//\s*)?#\s*define\s+{re.escape(name)}(\b.*)$")
    found = False
    updated = []
    for line in lines:
        match = pattern.match(line)
        if match:
            indent, _comment, rest = match.group(1), match.group(2), match.group(3)
            if enabled:
                updated.append(f"{indent}#define {name}{rest}\n")
            else:
                updated.append(f"{indent}//#define {name}{rest}\n")
            found = True
        else:
            updated.append(line)
    if not found:
        raise RuntimeError(f"Define not found in Config.h: {name}")
    return updated


def apply_options(base_config: list[str], options: str, promicro: bool) -> list[str]:
    letters = set(options)

    enabled_defines = {DEFINE_MAP[ch] for ch in letters if ch in DEFINE_MAP}

    use_shift_register = "n" in letters or "r" in letters
    use_sn74 = "r" in letters

    use_eeprom = "p" not in letters
    use_promicro = promicro
    use_quadrature = "d" not in letters and "w" not in letters

    updated = base_config[:]
    for define in enabled_defines:
        updated = set_define(updated, define, True)

    # Disable any mapped defines not explicitly enabled
    for define in DEFINE_MAP.values():
        if define not in enabled_defines:
            updated = set_define(updated, define, False)

    updated = set_define(updated, "USE_SHIFT_REGISTER", use_shift_register)
    updated = set_define(updated, "USE_SN74ALS166N", use_sn74)
    updated = set_define(updated, "USE_EEPROM", use_eeprom)
    updated = set_define(updated, "USE_PROMICRO", use_promicro)
    updated = set_define(updated, "USE_QUADRATURE_ENCODER", use_quadrature)

    return updated


def parse_variants(hex_dir: Path) -> list[tuple[str, str]]:
    variants: list[tuple[str, str]] = []
    for hex_path in sorted(hex_dir.glob("*.hex")):
        name = hex_path.name
        match = re.search(r"v250([a-z]*)", name, flags=re.IGNORECASE)
        if not match:
            continue
        options = match.group(1).lower()
        variants.append((name, options))
    return variants


def compile_variant(
    fqbn: str,
    options: str,
    output_name: str,
    promicro: bool,
    output_dir: Path,
) -> None:
    build_dir = ROOT / "build" / fqbn.replace(":", "_") / output_name.replace(".hex", "")
    build_dir.mkdir(parents=True, exist_ok=True)

    base_config = CONFIG_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    updated = apply_options(base_config, options, promicro)
    CONFIG_PATH.write_text("".join(updated), encoding="utf-8")

    cmd = [
        "arduino-cli",
        "compile",
        "--fqbn",
        fqbn,
        str(SKETCH_DIR),
        "--build-path",
        str(build_dir),
        "--libraries",
        str(LIB_DIR),
    ]
    try:
        subprocess.check_call(cmd)
    finally:
        CONFIG_PATH.write_text("".join(base_config), encoding="utf-8")

    hex_candidates = sorted(build_dir.glob("*.hex"))
    if not hex_candidates:
        raise RuntimeError(f"No HEX produced for {output_name}")
    preferred = [p for p in hex_candidates if "with_bootloader" not in p.name]
    hex_path = preferred[0] if preferred else hex_candidates[0]

    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(hex_path, output_dir / output_name)
    DIST_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy2(hex_path, DIST_DIR / output_name)


def main() -> None:
    leo_variants = parse_variants(LEO_HEX_DIR)
    pro_variants = parse_variants(PRO_HEX_DIR)

    if not leo_variants:
        raise RuntimeError("No Leonardo v250 variants found")
    if not pro_variants:
        raise RuntimeError("No ProMicro v250 variants found")

    failures: list[str] = []

    # Build Leonardo variants
    for output_name, options in leo_variants:
        try:
            compile_variant(
                "arduino:avr:leonardo",
                options,
                output_name,
                promicro=False,
                output_dir=LEO_HEX_DIR,
            )
        except subprocess.CalledProcessError:
            failures.append(output_name)

    # Build ProMicro variants (use Arduino Micro board definition + USE_PROMICRO)
    for output_name, options in pro_variants:
        if options.endswith("m"):
            options = options[:-1]
        try:
            compile_variant(
                "arduino:avr:micro",
                options,
                output_name,
                promicro=True,
                output_dir=PRO_HEX_DIR,
            )
        except subprocess.CalledProcessError:
            failures.append(output_name)

    DIST_DIR.mkdir(parents=True, exist_ok=True)
    zip_path = DIST_DIR / "v250_hex.zip"
    if zip_path.exists():
        zip_path.unlink()

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zipf:
        for hex_path in sorted(LEO_HEX_DIR.glob("*.hex")):
            zipf.write(hex_path, hex_path.relative_to(ROOT))
        for hex_path in sorted(PRO_HEX_DIR.glob("*.hex")):
            zipf.write(hex_path, hex_path.relative_to(ROOT))

    if failures:
        print("Build failed for: " + ", ".join(sorted(failures)))


if __name__ == "__main__":
    main()
