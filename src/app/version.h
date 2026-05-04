#pragma once

#define FW_VERSION "3.0.0-dev"

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

#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__
