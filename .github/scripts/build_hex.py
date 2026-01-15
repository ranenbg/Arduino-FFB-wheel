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
LEO_BASE_DIR = ROOT / "brWheel_my" / "leonardo hex"
PRO_BASE_DIR = ROOT / "brWheel_my" / "promicro hex"
ZIP_PATH = Path(os.environ.get("ZIP_PATH", "build.zip"))
VERSIONS = os.environ.get("VERSIONS")
VERSIONS_FILE = ROOT / ".github" / "versions-to-build.txt"

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

    for define in DEFINE_MAP.values():
        if define not in enabled_defines:
            updated = set_define(updated, define, False)

    updated = set_define(updated, "USE_SHIFT_REGISTER", use_shift_register)
    updated = set_define(updated, "USE_SN74ALS166N", use_sn74)
    updated = set_define(updated, "USE_EEPROM", use_eeprom)
    updated = set_define(updated, "USE_PROMICRO", use_promicro)
    updated = set_define(updated, "USE_QUADRATURE_ENCODER", use_quadrature)

    return updated


def parse_variants(hex_dir: Path, version: str) -> list[tuple[str, str]]:
    variants: list[tuple[str, str]] = []
    for hex_path in sorted(hex_dir.glob("*.hex")):
        name = hex_path.name
        match = re.search(rf"{re.escape(version)}([a-z]*)", name, flags=re.IGNORECASE)
        if not match:
            continue
        options = match.group(1).lower()
        variants.append((name, options))
    return variants


def parse_versions(value: str) -> list[str]:
    parts = re.split(r"[\s,]+", value.strip())
    return [p for p in parts if p]


def read_versions_file(path: Path) -> str | None:
    if not path.exists():
        return None
    lines = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        lines.append(line)
    return " ".join(lines) if lines else None


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


def main() -> None:
    versions_value = VERSIONS or read_versions_file(VERSIONS_FILE)
    if not versions_value:
        raise RuntimeError(
            "No versions configured. Set VERSIONS env or add .github/versions-to-build.txt"
        )
    versions = parse_versions(versions_value)
    failures: list[str] = []
    hex_dirs: list[Path] = []

    for version in versions:
        leo_hex_dir = LEO_BASE_DIR / f"fw_{version}"
        pro_hex_dir = PRO_BASE_DIR / f"fw_{version}"

        if not leo_hex_dir.exists() and not pro_hex_dir.exists():
            print(f"Skipping {version}: no fw_{version} folders found")
            continue

        if leo_hex_dir.exists():
            hex_dirs.append(leo_hex_dir)
        if pro_hex_dir.exists():
            hex_dirs.append(pro_hex_dir)

        leo_variants = parse_variants(leo_hex_dir, version) if leo_hex_dir.exists() else []
        pro_variants = parse_variants(pro_hex_dir, version) if pro_hex_dir.exists() else []

        for output_name, options in leo_variants:
            try:
                compile_variant(
                    "arduino:avr:leonardo",
                    options,
                    output_name,
                    promicro=False,
                    output_dir=leo_hex_dir,
                )
            except subprocess.CalledProcessError:
                failures.append(output_name)

        for output_name, options in pro_variants:
            if options.endswith("m"):
                options = options[:-1]
            try:
                compile_variant(
                    "arduino:avr:micro",
                    options,
                    output_name,
                    promicro=True,
                    output_dir=pro_hex_dir,
                )
            except subprocess.CalledProcessError:
                failures.append(output_name)

    if not hex_dirs:
        raise RuntimeError("No fw_<version> folders found for configured VERSIONS")

    ZIP_PATH.parent.mkdir(parents=True, exist_ok=True)
    if ZIP_PATH.exists():
        ZIP_PATH.unlink()

    with zipfile.ZipFile(ZIP_PATH, "w", compression=zipfile.ZIP_DEFLATED) as zipf:
        for hex_dir in hex_dirs:
            for hex_path in sorted(hex_dir.glob("*.hex")):
                zipf.write(hex_path, hex_path.relative_to(ROOT))

    if failures:
        print("Build failed for: " + ", ".join(sorted(failures)))


if __name__ == "__main__":
    main()