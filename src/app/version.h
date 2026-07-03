#pragma once

// FW_VERSION is injected at compile time via -DFW_VERSION in platformio.ini.
// That is the SINGLE source of truth — do NOT define it here.
// If you see a stale version at runtime, update platformio.ini and do a clean build:
//   rm sdkconfig.esp32s3 && pio run

// GIT_SHA is injected by tools/git_sha_gen.py into include/git_sha.h before
// each PlatformIO build. Fall back to "unknown" if the file is absent
// (e.g. when compiling outside PlatformIO).
#ifdef __has_include
#  if __has_include("git_sha.h")
#    include "git_sha.h"
#  endif
#endif
#ifndef GIT_SHA
#  define GIT_SHA "unknown"
#endif

// Composite version string shown in health endpoint, boot log, and /api/diag.
// Example: "3.1.0-dev.6 (a750b8d)" or "3.1.0-dev.6 (a750b8d-dirty)"
#define FW_VERSION_FULL FW_VERSION " (" GIT_SHA ")"

#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__
