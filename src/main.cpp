#include "app/boot.h"

extern "C" void app_main() {
  app::run_boot();
  // run_boot() never returns; it ends in an infinite heartbeat loop.
}
