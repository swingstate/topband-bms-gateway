#pragma once

namespace app {
  // Runs the full boot sequence: NVS init, config load, queue creation,
  // then enters the 5-second heartbeat loop. Never returns.
  void run_boot();
}
