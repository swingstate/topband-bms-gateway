#!/usr/bin/env bash
# tools/build-demo.sh
#
# Refresh docs/demo/ from the live web/ui/ assets.
#
# Run this whenever web/ui/ changes (new JS, CSS, added icons, etc.).
# It copies the real UI files into docs/demo/ while preserving the
# demo-specific files (index.html, mock.js) that are tracked separately.
#
# Usage:
#   bash tools/build-demo.sh
#
# After running, review the diff, then commit docs/demo/.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$REPO_ROOT/web/ui"
DST="$REPO_ROOT/docs/demo"

echo "[build-demo] Copying UI assets from web/ui/ to docs/demo/ ..."

# Core assets — copied verbatim
cp "$SRC/style.css"            "$DST/style.css"
cp "$SRC/uplot.min.css"        "$DST/uplot.min.css"
cp "$SRC/uplot.min.js"         "$DST/uplot.min.js"
cp "$SRC/favicon.svg"          "$DST/favicon.svg"
cp "$SRC/lib/sha256.min.js"    "$DST/lib/sha256.min.js"

# app.js — copied verbatim (mock.js intercepts fetch; app.js needs no changes)
cp "$SRC/app.js"               "$DST/app.js"

echo "[build-demo] Done."
echo ""
echo "Files preserved (not overwritten):"
echo "  docs/demo/index.html   — demo HTML with badge + relative paths"
echo "  docs/demo/mock.js      — in-browser mock backend"
echo ""
echo "If web/ui/index.html changed (new scripts, new meta tags, etc.),"
echo "manually merge the change into docs/demo/index.html."
echo ""
echo "Review diff, then commit:"
echo "  git add docs/demo/"
echo "  git commit -m 'docs: refresh UI demo from web/ui'"
