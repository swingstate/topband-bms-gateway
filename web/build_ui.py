"""
PlatformIO pre-build script: packages web/ui/ into web/littlefs_ui.tar,
then pre-generates the IDF embedding assembly file so PlatformIO's SCons
build finds it ready (avoiding a race with CMake's CUSTOM_COMMAND).

Plain tar (no gzip) so the firmware extracts directly with microtar.

UI_VERSION in src/storage/ui_provisioner.h is kept in sync with a
content-hash of web/ui/ source files (excluding ui_version.txt itself).
The file is only written when the hash actually changes, so repeated
builds with no UI changes leave the git tree clean.
"""

import gzip
import hashlib
import re
import subprocess
import tarfile
import io
import shutil
import sys
from pathlib import Path

Import("env")  # noqa: F821 — injected by PlatformIO

PROJECT_DIR  = Path(env["PROJECT_DIR"])   # noqa: F821
WEB_DIR      = PROJECT_DIR / "web"
UI_DIR       = WEB_DIR / "ui"
OUT_TAR      = WEB_DIR / "littlefs_ui.tar"
PROV_HEADER  = PROJECT_DIR / "src" / "storage" / "ui_provisioner.h"

# IDF tooling paths — use env.subst() to expand SCons $VARIABLES.
BUILD_DIR    = Path(env.subst("$BUILD_DIR"))  # .pio/build/esp32s3
_PKG_DIR     = Path(env.subst("$PROJECT_PACKAGES_DIR"))
CMAKE_SCRIPT = _PKG_DIR / "framework-espidf/tools/cmake/scripts/data_file_embed_asm.cmake"
CMAKE_BIN    = _PKG_DIR / "tool-cmake/bin/cmake"


def read_ui_version():
    """Extract UI_VERSION constant from ui_provisioner.h."""
    try:
        text = PROV_HEADER.read_text(encoding="utf-8")
        m = re.search(r'UI_VERSION\s*=\s*"([^"]+)"', text)
        if m:
            return m.group(1)
    except OSError:
        pass
    return "unknown"


def compute_ui_content_hash():
    """SHA-256 over all UI source files, stable sorted order.

    Excludes ui_version.txt (derived output) to avoid a chicken-and-egg
    cycle where updating the version would change the hash on the next run.
    """
    h = hashlib.sha256()
    for src in sorted(UI_DIR.rglob("*")):
        if src.is_file() and src.name != "ui_version.txt":
            rel = src.relative_to(UI_DIR).as_posix()
            h.update(rel.encode())
            h.update(b"\0")
            h.update(src.read_bytes())
            h.update(b"\0")
    return h.hexdigest()[:8]


def sync_ui_version():
    """Write ui_provisioner.h only when UI content actually changed.

    Uses a content-hash suffix (e.g. 3.1.0-dev.a3f9c2b1) instead of an
    opaque counter so repeated no-op builds leave the tracked file unmodified.
    Release versions (no -dev. suffix) are left untouched.
    """
    try:
        text = PROV_HEADER.read_text(encoding="utf-8")
        m = re.search(r'UI_VERSION\s*=\s*"([^"]+)"', text)
        if not m:
            print("[build_ui] Warning: UI_VERSION not found in ui_provisioner.h",
                  file=sys.stderr)
            return

        current = m.group(1)

        # Release versions are set manually — don't touch them.
        dev_m = re.match(r'^(.+)-dev\.\S+$', current)
        if not dev_m:
            print(f"[build_ui] Release version {current!r} — skipping hash sync")
            return

        base = dev_m.group(1)
        content_hash = compute_ui_content_hash()
        new_version = f"{base}-dev.{content_hash}"

        if new_version == current:
            print(f"[build_ui] UI unchanged (hash={content_hash}) — "
                  f"version stays {current!r}")
            return

        new_text = re.sub(
            r'(UI_VERSION\s*=\s*")[^"]+(")',
            lambda mo: f'{mo.group(1)}{new_version}{mo.group(2)}',
            text
        )
        PROV_HEADER.write_text(new_text, encoding="utf-8")
        print(f"[build_ui] UI content changed → {current!r} → {new_version!r}")

    except OSError as exc:
        print(f"[build_ui] Warning: could not sync UI version: {exc}",
              file=sys.stderr)


def build_tarball():
    if not UI_DIR.exists():
        print(f"[build_ui] ERROR: {UI_DIR} not found", file=sys.stderr)
        env.Exit(1)  # noqa: F821

    ui_version = read_ui_version()
    (UI_DIR / "ui_version.txt").write_text(ui_version, encoding="utf-8")

    # Build plain tar in memory, then write atomically.
    # GNU_FORMAT avoids PAX extended headers (././@PaxHeader type 'x') that
    # Python 3.8+ emits by default; microtar cannot iterate past them.
    #
    # For text assets (JS, CSS, HTML, SVG) we also add a pre-compressed
    # <name>.gz companion so the HTTP server can serve gzip to capable
    # browsers without compressing on the fly.
    COMPRESS_EXTS = {".js", ".css", ".html", ".svg"}
    gz_count = 0
    gz_saved = 0

    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.GNU_FORMAT) as tar:
        for src in sorted(UI_DIR.rglob("*")):
            if src.is_file():
                arcname = src.relative_to(UI_DIR).as_posix()
                tar.add(str(src), arcname=arcname)
                if src.suffix.lower() in COMPRESS_EXTS:
                    data    = src.read_bytes()
                    gz_data = gzip.compress(data, compresslevel=9)
                    if len(gz_data) < len(data):
                        info      = tarfile.TarInfo(name=arcname + ".gz")
                        info.size = len(gz_data)
                        tar.addfile(info, io.BytesIO(gz_data))
                        gz_count += 1
                        gz_saved += len(data) - len(gz_data)

    raw = buf.getvalue()
    tmp = OUT_TAR.with_suffix(".tar.tmp")
    tmp.write_bytes(raw)
    shutil.move(str(tmp), str(OUT_TAR))

    n_files = sum(1 for f in UI_DIR.rglob("*") if f.is_file())
    size_kb  = len(raw) / 1024
    print(f"[build_ui] Packed {n_files} files + {gz_count} .gz companions "
          f"({gz_saved//1024} KB saved by gzip) → {OUT_TAR.name} "
          f"({size_kb:.1f} KB, version={ui_version})")

    return raw


def pre_generate_embed_s(raw_tar_bytes):
    """
    Pre-generate the IDF embedding .S file before SCons tries to compile it.
    PlatformIO's SCons and IDF's CMake CUSTOM_COMMAND both target the same file;
    generating it early avoids the 'source not found' error.
    """
    if not BUILD_DIR or not CMAKE_SCRIPT.exists() or not CMAKE_BIN.exists():
        print("[build_ui] Warning: Cannot pre-generate .S file "
              "(cmake or script not found). Build may fail.", file=sys.stderr)
        return

    embed_s = BUILD_DIR / "littlefs_ui.tar.S"
    embed_s.parent.mkdir(parents=True, exist_ok=True)

    # Always regenerate: mtime comparison can be unreliable when tar and .S
    # are written within the same filesystem timestamp granularity (same second),
    # causing the staleness check to incorrectly skip embedding fresh UI assets.

    try:
        result = subprocess.run(
            [str(CMAKE_BIN),
             "-D", f"DATA_FILE={OUT_TAR}",
             "-D", f"SOURCE_FILE={embed_s}",
             "-D", "FILE_TYPE=TEXT",
             "-P", str(CMAKE_SCRIPT)],
            capture_output=True, text=True, timeout=30
        )
        if result.returncode != 0:
            print(f"[build_ui] Warning: .S generation failed:\n{result.stderr}",
                  file=sys.stderr)
        else:
            print(f"[build_ui] Pre-generated {embed_s.name} "
                  f"({embed_s.stat().st_size} B)")
    except Exception as exc:
        print(f"[build_ui] Warning: .S generation exception: {exc}", file=sys.stderr)


sync_ui_version()
raw = build_tarball()
pre_generate_embed_s(raw)
