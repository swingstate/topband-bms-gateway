// Regression coverage for the V3.2 alerts-persistence bug: end-user report
// that alerts visible on the Alerts page disappeared after every power-cycle,
// despite HousekeepingTask flushing the ring to LittleFS every 5 minutes.
//
// Root cause: storage::lfs::read_file() is a text-file helper — it reserves
// one byte of the caller's buffer for a null terminator, so it can only ever
// return buf_size - 1 bytes. alerts_store.cpp's read_from_disk() asked for
// exactly FILE_SIZE bytes and rejected anything less
// ("if (n < FILE_SIZE) return false;"), so that check was unsatisfiable —
// read_from_disk() failed on *every* boot, regardless of whether the write
// had succeeded or how long the device had been running. init() then treated
// this as "not found or corrupt" and overwrote the ring file with a blank one
// before the caller ever got a chance to look at the old data. The write
// path (write_to_disk() / write_file_atomic()) was never at fault; this was a
// read-path bug reachable at every single restart, not a 5-minute timing
// window.
//
// This compiles the REAL alerts_store.cpp + lfs_store.cpp against the real
// host filesystem (a relative path confined to the test binary's CWD, since
// NATIVE_BUILD skips the ESP LittleFS VFS mount) so the actual buggy
// read/write code path is exercised end-to-end, the same way test_nvs_store
// exercises the real nvs_store.cpp against a fake backing store for the
// analogous V3.2 config-migration bug.
#include "storage/alerts_store.h"
#include "bus/queues.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <cstring>

namespace {

constexpr const char* kRingFile = "./alerts_store_test_ring.bin";

void remove_ring_files() {
  std::remove(kRingFile);
  std::remove("./alerts_store_test_ring.bin.tmp");
}

AlertEntry make_alert(uint32_t ts_epoch, uint8_t severity, const char* message) {
  AlertEntry a{};
  a.ts_epoch  = ts_epoch;
  a.uptime_s  = ts_epoch;
  a.severity  = severity;
  a.category  = 0;
  a.flags     = 0;
  std::strncpy(a.message, message, sizeof(a.message) - 1);
  return a;
}

}  // namespace

TEST_CASE("alerts_store: fresh boot with no ring file starts empty and creates one", "[alerts_store]") {
  remove_ring_files();

  REQUIRE(storage::alerts_store::init());
  REQUIRE(storage::alerts_store::stored_count() == 0);

  // init() must have written a fresh (empty) ring file rather than leaving
  // nothing on disk — otherwise the very first flush later has nothing to
  // atomically replace.
  FILE* f = std::fopen(kRingFile, "rb");
  REQUIRE(f != nullptr);
  std::fclose(f);

  storage::alerts_store::test_simulate_reboot();
  remove_ring_files();
}

TEST_CASE("alerts_store: persisted alerts survive a simulated power cycle", "[alerts_store]") {
  remove_ring_files();

  REQUIRE(storage::alerts_store::init());
  REQUIRE(storage::alerts_store::append(make_alert(1000, 1, "wifi disconnected")));
  REQUIRE(storage::alerts_store::append(make_alert(1001, 3, "5x power-cycle reset triggered")));
  REQUIRE(storage::alerts_store::persist());  // == the 5-min HousekeepingTask flush

  // Simulate an actual power-cycle: drop all in-RAM state, re-run init() as
  // a fresh boot would, and confirm the alerts are read back from disk.
  storage::alerts_store::test_simulate_reboot();
  REQUIRE(storage::alerts_store::init());

  REQUIRE(storage::alerts_store::stored_count() == 2);

  AlertEntry out[4];
  size_t n = storage::alerts_store::read(out, 4);
  REQUIRE(n == 2);
  // read() walks newest-to-oldest.
  REQUIRE(out[0].ts_epoch == 1001);
  REQUIRE(std::strcmp(out[0].message, "5x power-cycle reset triggered") == 0);
  REQUIRE(out[1].ts_epoch == 1000);
  REQUIRE(std::strcmp(out[1].message, "wifi disconnected") == 0);

  storage::alerts_store::test_simulate_reboot();
  remove_ring_files();
}

TEST_CASE("alerts_store: alerts emitted but never flushed do not survive a power cycle", "[alerts_store]") {
  // This is the legitimate timing-window gap (not a bug): an alert appended
  // to the in-RAM ring but never handed to persist() (i.e. the 5-min flush
  // never ran before power was pulled) is expected to be lost. Documented
  // here so it isn't mistaken for a regression of the fix above.
  remove_ring_files();

  REQUIRE(storage::alerts_store::init());
  REQUIRE(storage::alerts_store::append(make_alert(2000, 1, "never flushed")));
  // No persist() call.

  storage::alerts_store::test_simulate_reboot();
  REQUIRE(storage::alerts_store::init());

  REQUIRE(storage::alerts_store::stored_count() == 0);

  storage::alerts_store::test_simulate_reboot();
  remove_ring_files();
}

TEST_CASE("alerts_store: multiple flush/reboot cycles keep accumulating history", "[alerts_store]") {
  remove_ring_files();

  REQUIRE(storage::alerts_store::init());
  REQUIRE(storage::alerts_store::append(make_alert(1, 0, "first")));
  REQUIRE(storage::alerts_store::persist());
  storage::alerts_store::test_simulate_reboot();

  REQUIRE(storage::alerts_store::init());
  REQUIRE(storage::alerts_store::stored_count() == 1);
  REQUIRE(storage::alerts_store::append(make_alert(2, 0, "second")));
  REQUIRE(storage::alerts_store::persist());
  storage::alerts_store::test_simulate_reboot();

  REQUIRE(storage::alerts_store::init());
  REQUIRE(storage::alerts_store::stored_count() == 2);

  storage::alerts_store::test_simulate_reboot();
  remove_ring_files();
}
