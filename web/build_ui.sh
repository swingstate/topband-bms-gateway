#!/usr/bin/env bash
# Manually build web/littlefs_ui.tar from web/ui/.
# Usage: bash web/build_ui.sh
# Run from the project root. Equivalent to the PlatformIO pre-build hook.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$SCRIPT_DIR/ui"
OUT="$SCRIPT_DIR/littlefs_ui.tar"

if [ ! -d "$UI_DIR" ]; then
  echo "ERROR: $UI_DIR not found" >&2
  exit 1
fi

# Extract UI_VERSION from the provisioner header.
PROV_H="$SCRIPT_DIR/../src/storage/ui_provisioner.h"
UI_VERSION=$(sed -n 's/.*UI_VERSION[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' "$PROV_H" 2>/dev/null | head -1)
UI_VERSION=${UI_VERSION:-unknown}

echo "$UI_VERSION" > "$UI_DIR/ui_version.txt"

tar --create --file="$OUT" --directory="$UI_DIR" .

N=$(find "$UI_DIR" -type f | wc -l)
SIZE=$(du -sh "$OUT" | cut -f1)
echo "[build_ui] Packed $N files → $(basename "$OUT") ($SIZE, version=$UI_VERSION)"
