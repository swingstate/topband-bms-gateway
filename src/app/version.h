#pragma once

#ifndef FW_VERSION
#  define FW_VERSION "3.1.0-dev.5"
#endif

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

// Composite version string shown in health endpoint and boot log.
// Example: "3.0.0 (a750b8d)" or "3.0.0 (a750b8d-dirty)"
#define FW_VERSION_FULL FW_VERSION " (" GIT_SHA ")"

#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__
