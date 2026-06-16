#pragma once
#include "storage/config.h"
#include <cstdint>

// ── Notification provider interface ──────────────────────────────────────────
//
// To add a provider (e.g. ntfy):
//   1. Add src/notify/ntfy.{h,cpp} implementing INotifyProvider.
//   2. Add its config fields to Config (schema bump).
//   3. Add its UI subsection + test button in the settings JS.
//   4. Register an instance in the k_providers table in notify.cpp.
// That is the complete list — no changes to the dispatcher or this header.

namespace notify {

enum class Severity : uint8_t { Info = 0, Warning = 1, Critical = 2 };

struct NotifyMessage {
  Severity    severity;
  const char* title;
  const char* body;
};

class INotifyProvider {
public:
  virtual ~INotifyProvider() = default;

  // Short lowercase identifier, e.g. "telegram", "ntfy".
  virtual const char* id() const = 0;

  // True if this provider is enabled and has enough config to attempt a send.
  virtual bool is_enabled(const Config& cfg) const = 0;

  // Format and send msg using the provider's API.
  // cfg:     the active (or test) config — provider reads its own fields.
  // err_out: populated with a human-readable reason on failure.
  // Returns true on success.
  virtual bool send(const NotifyMessage& msg,
                    const Config&        cfg,
                    char*                err_out,
                    size_t               err_out_size) const = 0;
};

}  // namespace notify
