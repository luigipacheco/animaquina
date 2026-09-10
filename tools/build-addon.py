# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Pack the add-on into an installable Blender zip.
#
#   python tools/build-addon.py
#   python tools/build-addon.py --suffix uitest
#   python tools/build-addon.py --vendor-py ../vendor_py_cp313
#
# vendor_py holds compiled native dependencies (the patched ur_rtde, paramiko
# and friends) and is deliberately not in the repository - it is built per
# Python version, see tools/vendor-build/README.md. Without --vendor-py the
# build is pure Python: KUKA and xArm work fully, UR works on the bundled urx
# backend, and ur_rtde features are unavailable.

from __future__ import annotations

import argparse
import ast
import pathlib
import re
import shutil
import sys
import tempfile
import zipfile

REPO = pathlib.Path(__file__).resolve().parent.parent
PKG = REPO / "animaquina"
DOCS = ("LICENSE", "SAFETY.md", "THIRD-PARTY-NOTICES.md")
ROBOTS = REPO / "assets" / "robots"
# Shipped beside the rig files: the rig work is CC-BY-4.0 over manufacturer CAD.
ROBOT_DOCS = ("LICENSES/CC-BY-4.0.txt", "THIRD-PARTY-NOTICES.md")

# Blender ships a fixed Python per release, and the compiled .pyd files in
# vendor_py only load on the matching one. Mismatches fail at import with
# "bad magic number", which is a confusing thing to hand a user.
BLENDER_PYTHON = {"4.2": "cp311", "4.3": "cp311", "4.4": "cp311", "4.5": "cp311",
                  "5.0": "cp313", "5.1": "cp313", "5.2": "cp313", "5.3": "cp313"}

REQUIRED_MANIFEST_KEYS = ("schema_version", "id", "version", "name", "tagline",
                          "maintainer", "type", "license", "blender_version_min")


def fail(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    raise SystemExit(1)


def read_manifest() -> dict:
    path = PKG / "blender_manifest.toml"
    if not path.is_file():
        fail(f"no manifest at {path}")
    try:
        import tomllib
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except ModuleNotFoundError:  # Python < 3.11
        text = path.read_text(encoding="utf-8")
        data = {k: v.strip('"') for k, v in re.findall(r'^(\w+)\s*=\s*(".*?")', text, re.M)}
    missing = [k for k in REQUIRED_MANIFEST_KEYS if k not in data]
    if missing:
        fail(f"manifest is missing required keys: {', '.join(missing)}")
    tagline = str(data.get("tagline", ""))
    if len(tagline) > 64:
        fail(f"tagline is {len(tagline)} chars, Blender allows 64")
    if tagline and tagline[-1] in ".!?,;:":
        fail("tagline must not end with punctuation")
    if not re.fullmatch(r"\d+\.\d+\.\d+", str(data["version"])):
        fail(f"version {data['version']!r} is not x.y.z")
    return data


def check_syntax(root: pathlib.Path) -> int:
    bad, count = [], 0
    for f in sorted(root.rglob("*.py")):
        count += 1
        try:
            ast.parse(f.read_text(encoding="utf-8"), filename=str(f))
        except SyntaxError as exc:
            bad.append(f"  {f.relative_to(root)}:{exc.lineno}: {exc.msg}")
    if bad:
        fail("syntax errors in the staged tree:\n" + "\n".join(bad))
    return count


def check_vendor_abi(vendor: pathlib.Path, blender_min: str) -> None:
    """Compare vendor_py's compiled ABI tag against the manifest's Blender floor."""
    series = ".".join(str(blender_min).split(".")[:2])
    want = BLENDER_PYTHON.get(series)
    tags = sorted({m.group(1) for f in vendor.rglob("*.pyd")
                   for m in [re.search(r"\.(cp\d{3})-", f.name)] if m})
    if not tags:
        print("  vendor_py: no compiled extensions found (pure Python?)")
        return
    print(f"  vendor_py ABI: {', '.join(tags)}")
    if want is None:
        print(f"  ! unknown Python for Blender {series}; ABI not verified")
    elif tags != [want]:
        fail(f"vendor_py is {'/'.join(tags)} but Blender {series} needs {want}.\n"
             f"       These .pyd files will fail to import with 'bad magic number'.\n"
             f"       Build a matching vendor_py (tools/vendor-build/README.md), or omit\n"
             f"       --vendor-py for a pure-Python build.")


def write_zip(out_path: pathlib.Path, members: list[tuple[pathlib.Path, str]]) -> list[str]:
    """Write members as (source file, arcname) and verify the result."""
    if out_path.exists():
        out_path.unlink()
    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as z:
        for src, arc in members:
            z.write(src, arc)
    with zipfile.ZipFile(out_path) as z:
        if z.testzip() is not None:
            fail(f"{out_path.name} failed its integrity check")
        return z.namelist()


def build_robot_library(out_dir: pathlib.Path, version: str, suffix: str) -> None:
    """Pack assets/robots into the separate robot-library release asset.

    Kept out of the add-on zip deliberately: ~70 MB that would ride along on
    every patch release, under different licence terms from the code.
    """
    if not ROBOTS.is_dir():
        fail(f"no robot assets at {ROBOTS}")
    members = []
    for f in sorted(ROBOTS.rglob("*")):
        # .blend1/.blend2 are Blender's own backups, never ship them
        if f.is_file() and f.suffix not in {".blend1", ".blend2"} and not f.name.endswith("~"):
            members.append((f, ("robots" / f.relative_to(ROBOTS)).as_posix()))
    if not any(a.endswith(".blend") for _, a in members):
        fail("no .blend found in assets/robots - nothing to ship")
    for rel in ROBOT_DOCS:
        src = REPO / rel
        if src.is_file():
            members.append((src, pathlib.Path(rel).name))
        else:
            print(f"  ! {rel} missing, not bundled with the robot library")

    stem = f"animaquina-robots-{version}" + (f"-{suffix}" if suffix else "")
    out = out_dir / f"{stem}.zip"
    names = write_zip(out, members)
    print(f"
  {out}")
    print(f"  {out.stat().st_size / 1048576:.2f} MB, {len(names)} files")


def main() -> int:
    ap = argparse.ArgumentParser(description="Pack the Animaquina add-on into a Blender zip.")
    ap.add_argument("--vendor-py", type=pathlib.Path, default=None,
                    help="folder of compiled native deps to bundle as animaquina/vendor_py")
    ap.add_argument("--suffix", default="", help="appended to the filename, e.g. 'uitest'")
    ap.add_argument("--blender-min", default=None, metavar="X.Y.Z",
                    help="override blender_version_min in the built zip. The repo manifest "
                         "targets the ABI of the vendor_py you normally ship; use this to "
                         "produce a build for a different Blender/Python series")
    ap.add_argument("--out", type=pathlib.Path, default=REPO / "dist", help="output directory")
    ap.add_argument("--robots", action="store_true",
                    help="also pack assets/robots into the separate robot-library asset")
    ap.add_argument("--only-robots", action="store_true",
                    help="pack only the robot library, skipping the add-on")
    args = ap.parse_args()

    manifest = read_manifest()
    version = str(manifest["version"])
    if args.only_robots:
        args.out.mkdir(parents=True, exist_ok=True)
        build_robot_library(args.out, version, args.suffix)
        return 0
    blender_min = str(args.blender_min or manifest["blender_version_min"])
    note = "" if not args.blender_min else f"  (manifest says {manifest['blender_version_min']})"
    print(f"Animaquina {version}  (Blender {blender_min}+){note}")

    with tempfile.TemporaryDirectory() as tmp:
        stage = pathlib.Path(tmp) / "animaquina"
        shutil.copytree(PKG, stage, ignore=shutil.ignore_patterns(
            "__pycache__", "*.pyc", "*.pyo", "vendor_py"))

        if args.blender_min:
            mf = stage / "blender_manifest.toml"
            mf.write_text(
                re.sub(r'^blender_version_min = "[^"]*"',
                       f'blender_version_min = "{blender_min}"',
                       mf.read_text(encoding="utf-8"), count=1, flags=re.M),
                encoding="utf-8")

        for name in DOCS:
            src = REPO / name
            if src.is_file():
                shutil.copy2(src, stage / name)
            else:
                print(f"  ! {name} missing from the repo root, not bundled")

        if args.vendor_py:
            vendor = args.vendor_py.resolve()
            if not vendor.is_dir():
                fail(f"--vendor-py {vendor} is not a directory")
            check_vendor_abi(vendor, blender_min)
            shutil.copytree(vendor, stage / "vendor_py",
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        else:
            print("  vendor_py: omitted (pure Python build - ur_rtde unavailable)")

        n = check_syntax(stage)
        print(f"  {n} modules parse")

        args.out.mkdir(parents=True, exist_ok=True)
        stem = f"animaquina-{version}" + (f"-{args.suffix}" if args.suffix else "")
        zip_path = args.out / f"{stem}.zip"
        if zip_path.exists():
            zip_path.unlink()
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(stage.rglob("*")):
                if f.is_file():
                    z.write(f, ("animaquina" / f.relative_to(stage)).as_posix())

        with zipfile.ZipFile(zip_path) as z:
            names = z.namelist()
        if any("__pycache__" in n or n.endswith(".pyc") for n in names):
            fail("compiled caches leaked into the zip")
        if "animaquina/blender_manifest.toml" not in names:
            fail("manifest is not at animaquina/blender_manifest.toml in the zip")

    print(f"\n  {zip_path}")
    print(f"  {zip_path.stat().st_size / 1048576:.2f} MB, {len(names)} files")
    print("\nInstall: Blender > Preferences > Add-ons > Install from Disk")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
