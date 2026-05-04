#pragma once
#include "config.h"

namespace storage {
  // Reads cfg_v1 blob from NVS namespace "gateway". Verifies CRC-32.
  // On any failure (missing keys, CRC mismatch, schema mismatch), sets out =
  // DEFAULT_CONFIG and returns false. Caller should call saveConfig() to persist
  // defaults when this returns false.
  bool loadConfig(Config& out);

  // Serializes cfg, writes blob + CRC to NVS, then reads back and re-verifies.
  // On readback CRC failure, the old cfg_v1 (if any) is left in place.
  // Returns true only when write AND readback succeed.
  bool saveConfig(const Config& cfg);
}
