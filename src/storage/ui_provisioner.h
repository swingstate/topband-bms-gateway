#pragma once

// ── UI Provisioner ───────────────────────────────────────────────────────────
// Checks /lfs/meta/ui_version.txt. If missing or contents != UI_VERSION,
// extracts the embedded littlefs_ui.tar.gz (microtar + IDF miniz) into /lfs/ui/.
// Idempotent: a matching version string is a no-op.

namespace storage::ui_provisioner {

// UI bundle version. Bump whenever web/ui/ files change.
// Must stay in sync with web/ui/ui_version.txt (written by web/build_ui.py).
constexpr const char* UI_VERSION = "3.1.0-dev.6";

// Call once during boot after lfs::init(). Extracts if needed.
// Logs extraction progress at INFO level.
void provision_ui_if_needed();

}  // namespace storage::ui_provisioner
