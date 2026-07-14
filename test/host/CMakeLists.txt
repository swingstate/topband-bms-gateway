cmake_minimum_required(VERSION 3.16)
project(topband_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Catch2 v3 via FetchContent ─────────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

# ── nlohmann/json v3 via FetchContent (header-only) ──────────────────────────
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# ── ArduinoJson v7 via FetchContent (header-only) ─────────────────────────────
FetchContent_Declare(
  ArduinoJson
  GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
  GIT_TAG        v7.3.1
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(ArduinoJson)

# Pthreads — needed by test_snapshot_bus multi-thread fuzz
find_package(Threads REQUIRED)

# ── Common compile options ────────────────────────────────────────────────────
set(HOST_COMPILE_OPTS -Wall -Wextra -Wno-unused-parameter)
set(HOST_COMPILE_DEFS NATIVE_BUILD SMOKE_READER_ENABLED=0)

# ── test_config_serde ─────────────────────────────────────────────────────────
# config.cpp is host-portable: no IDF headers, no FreeRTOS, no nvs_flash.
add_executable(test_config_serde
  test_config_serde.cpp
  ../../src/storage/config.cpp
)

target_include_directories(test_config_serde PRIVATE
  ../../src        # makes #include "storage/config.h" resolve correctly
  ../../include    # for git_sha.h (if present)
)

target_compile_definitions(test_config_serde PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_config_serde PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_config_serde PRIVATE Catch2::Catch2WithMain)

# ── CTest integration ─────────────────────────────────────────────────────────
include(CTest)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(test_config_serde)

# ── test_protocol ─────────────────────────────────────────────────────────────
# protocol.cpp is host-portable: no IDF headers, no FreeRTOS.
add_executable(test_protocol
  test_protocol.cpp
  ../../src/bms/protocol.cpp
)

target_include_directories(test_protocol PRIVATE
  ../../src
)

target_compile_definitions(test_protocol PRIVATE
  ${HOST_COMPILE_DEFS}
  "FIXTURE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/fixtures\""
)

target_compile_options(test_protocol PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_protocol PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_protocol)

# ── test_snapshot_bus ─────────────────────────────────────────────────────────
# snapshot_bus.cpp uses #ifdef NATIVE_BUILD to switch to plain malloc().
# The multi-thread fuzz test requires pthreads.
add_executable(test_snapshot_bus
  test_snapshot_bus.cpp
  ../../src/bus/snapshot_bus.cpp
)

target_include_directories(test_snapshot_bus PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_snapshot_bus PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_snapshot_bus PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_snapshot_bus PRIVATE
  Catch2::Catch2WithMain
  Threads::Threads
)
catch_discover_tests(test_snapshot_bus
  PROPERTIES TIMEOUT 30   # fuzz test runs for 5 s; give it 30 s margin
)

# ── test_snapshot_filler ──────────────────────────────────────────────────────
# snapshot.cpp and protocol.cpp are both host-portable.
add_executable(test_snapshot_filler
  test_snapshot_filler.cpp
  ../../src/bms/snapshot.cpp
  ../../src/bms/protocol.cpp
)

target_include_directories(test_snapshot_filler PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_snapshot_filler PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_snapshot_filler PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_snapshot_filler PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_snapshot_filler)

# ── test_safety ───────────────────────────────────────────────────────────────
# filters.cpp and runSafety.cpp are host-portable (no IDF, no FreeRTOS).
# Reads JSON fixtures from test/host/fixtures/v267/ for V2.67 regression.
add_executable(test_safety
  test_safety.cpp
  ../../src/safety/filters.cpp
  ../../src/safety/runSafety.cpp
)

target_include_directories(test_safety PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_safety PRIVATE
  ${HOST_COMPILE_DEFS}
  "FIXTURE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/fixtures\""
)

target_compile_options(test_safety PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_safety PRIVATE
  Catch2::Catch2WithMain
  nlohmann_json::nlohmann_json
)
catch_discover_tests(test_safety)

# ── test_can_victron ──────────────────────────────────────────────────────────
# victron.cpp, pylontech.cpp, sma.cpp, tx.cpp (NATIVE_BUILD stub), busoff.cpp.
# tx.cpp dispatches to all three protocol builders; all must be linked together.
# Reads JSON fixtures from test/host/fixtures/v267/ (safety) and
# test/host/fixtures/v267_can/ (CAN expected frames).
add_executable(test_can_victron
  test_can_victron.cpp
  ../../src/can/victron.cpp
  ../../src/can/pylontech.cpp
  ../../src/can/sma.cpp
  ../../src/can/tx.cpp
  ../../src/can/busoff.cpp
)

target_include_directories(test_can_victron PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_can_victron PRIVATE
  ${HOST_COMPILE_DEFS}
  "FIXTURE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/fixtures\""
)

target_compile_options(test_can_victron PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_can_victron PRIVATE
  Catch2::Catch2WithMain
  nlohmann_json::nlohmann_json
)
catch_discover_tests(test_can_victron)

# ── test_can_pylontech ────────────────────────────────────────────────────────
# All three protocol builders + tx.cpp (NATIVE_BUILD stub) + busoff.cpp.
# tx.cpp references all three builders; they must all be linked together.
add_executable(test_can_pylontech
  test_can_pylontech.cpp
  ../../src/can/pylontech.cpp
  ../../src/can/victron.cpp
  ../../src/can/sma.cpp
  ../../src/can/tx.cpp
  ../../src/can/busoff.cpp
)

target_include_directories(test_can_pylontech PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_can_pylontech PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_can_pylontech PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_can_pylontech PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_can_pylontech)

# ── test_can_sma ──────────────────────────────────────────────────────────────
# All three protocol builders + tx.cpp (NATIVE_BUILD stub) + busoff.cpp.
add_executable(test_can_sma
  test_can_sma.cpp
  ../../src/can/sma.cpp
  ../../src/can/victron.cpp
  ../../src/can/pylontech.cpp
  ../../src/can/tx.cpp
  ../../src/can/busoff.cpp
)

target_include_directories(test_can_sma PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_can_sma PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_can_sma PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_can_sma PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_can_sma)

# ── test_mqtt_payloads ────────────────────────────────────────────────────────
# payloads.cpp and topics.cpp are host-portable (ArduinoJson only, no IDF).
add_executable(test_mqtt_payloads
  test_mqtt_payloads.cpp
  ../../src/mqtt/payloads.cpp
  ../../src/mqtt/topics.cpp
)

target_include_directories(test_mqtt_payloads PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_mqtt_payloads PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_mqtt_payloads PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_mqtt_payloads PRIVATE
  Catch2::Catch2WithMain
  ArduinoJson
)
catch_discover_tests(test_mqtt_payloads)

# ── test_runtime_estimator ────────────────────────────────────────────────────
# runtime_estimator.cpp is host-portable: pure function, <cmath> only.
add_executable(test_runtime_estimator
  test_runtime_estimator.cpp
  ../../src/bms/runtime_estimator.cpp
)

target_include_directories(test_runtime_estimator PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_runtime_estimator PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_runtime_estimator PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_runtime_estimator PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_runtime_estimator)

# ── test_energy_integrator ────────────────────────────────────────────────────
# energy_integrator.cpp is host-portable; its energy_store dependencies are
# stubbed inline in the test file (no LittleFS or IDF headers needed).
add_executable(test_energy_integrator
  test_energy_integrator.cpp
  ../../src/bms/energy_integrator.cpp
)

target_include_directories(test_energy_integrator PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_energy_integrator PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_energy_integrator PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_energy_integrator PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_energy_integrator)

# ── test_log_ring ─────────────────────────────────────────────────────────────
# log_ring.cpp uses #ifdef NATIVE_BUILD to skip PSRAM alloc and vprintf hook.
add_executable(test_log_ring
  test_log_ring.cpp
  ../../src/diag/log_ring.cpp
)

target_include_directories(test_log_ring PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_log_ring PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_log_ring PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_log_ring PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_log_ring)

# ── test_alerts ───────────────────────────────────────────────────────────────
# alerts.cpp uses #ifdef NATIVE_BUILD to skip FreeRTOS queue and IDF timer.
# bus/queues.cpp provides queue handles; stubs defined in test file.
add_executable(test_alerts
  test_alerts.cpp
  ../../src/diag/alerts.cpp
  # queues.cpp omitted: test_alerts.cpp provides the extern globals, and
  # queues.cpp drags in IDF headers (esp_log.h, freertos/queue.h) that
  # are not available in the host build.
)

target_include_directories(test_alerts PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_alerts PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_alerts PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_alerts PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_alerts)

# ── test_config_json ──────────────────────────────────────────────────────────
# Tests for config_to_json / json_to_config (web layer on top of Config).
# Verifies the leave-blank-to-keep pattern for notify_telegram_token and
# notify_telegram_chat_id, and that config_to_json redacts the token.
# config_json.cpp has no IDF dependencies — only storage/config.h + ArduinoJson.
add_executable(test_config_json
  test_config_json.cpp
  ../../src/storage/config.cpp
  ../../src/web/config_json.cpp
)

target_include_directories(test_config_json PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_config_json PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_config_json PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_config_json PRIVATE
  Catch2::Catch2WithMain
  ArduinoJson
)
catch_discover_tests(test_config_json)

# ── test_aggregator ───────────────────────────────────────────────────────────
# aggregator.cpp is pure logic, but its header pulls in shunt_source.h /
# mppt_source.h which declare a portMUX_TYPE member — stub out just that IDF
# symbol (stubs/freertos/FreeRTOS.h) so the headers parse. Only the static
# pure functions (select / select_bank_value) are exercised; shunt_source.cpp,
# mppt_source.cpp and bms_source.cpp are never linked or instantiated.
add_executable(test_aggregator
  test_aggregator.cpp
  ../../src/sources/aggregator.cpp
  stubs/sources_link_stubs.cpp
)

target_include_directories(test_aggregator PRIVATE
  stubs
  ../../src
  ../../include
)

target_compile_definitions(test_aggregator PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_aggregator PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_aggregator PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_aggregator)

# ── test_victron_shunt_decode ─────────────────────────────────────────────────
# victron_shunt_decode.cpp is a pure bit-math module: no IDF, no FreeRTOS,
# no mbedTLS dependency (it operates on already-decrypted plaintext).
add_executable(test_victron_shunt_decode
  test_victron_shunt_decode.cpp
  ../../src/sources/victron_shunt_decode.cpp
)

target_include_directories(test_victron_shunt_decode PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_victron_shunt_decode PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_victron_shunt_decode PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_victron_shunt_decode PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_victron_shunt_decode)

# ── test_wifi_backoff ─────────────────────────────────────────────────────────
# wifi_backoff.h is a header-only pure policy (no IDF, no FreeRTOS) extracted
# from net/wifi.cpp's event-driven reconnect state machine specifically so the
# backoff/never-gives-up algorithm can be verified on host — wifi.cpp itself
# depends on esp_wifi/esp_netif/esp_event and cannot be built here.
add_executable(test_wifi_backoff
  test_wifi_backoff.cpp
)

target_include_directories(test_wifi_backoff PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_wifi_backoff PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_wifi_backoff PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_wifi_backoff PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_wifi_backoff)

# ── test_nvs_store ─────────────────────────────────────────────────────────────
# Compiles the REAL nvs_store.cpp (normally ESP-IDF-only: esp_log.h, nvs.h,
# nvs_flash.h) against a fake in-memory NVS backing store
# (stubs/esp_idf/fake_nvs.cpp) instead of stubbing it out of the link like
# test_aggregator does for FreeRTOS. This is what makes it possible to
# reproduce the V3.2 config-migration data-loss regression end-to-end: the
# bug was in nvs_store.cpp's read-buffer sizing, a file no other host test
# target links.
add_executable(test_nvs_store
  test_nvs_store.cpp
  ../../src/storage/nvs_store.cpp
  ../../src/storage/config.cpp
  stubs/esp_idf/fake_nvs.cpp
)

target_include_directories(test_nvs_store PRIVATE
  stubs/esp_idf
  ../../src
  ../../include
)

target_compile_definitions(test_nvs_store PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_nvs_store PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_nvs_store PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_nvs_store)

# ── test_alerts_store ─────────────────────────────────────────────────────────
# Compiles the REAL alerts_store.cpp + lfs_store.cpp against the real host
# filesystem (NATIVE_BUILD skips the ESP LittleFS VFS mount and uses a
# relative path under the test binary's CWD instead). This is what makes it
# possible to reproduce the V3.2 alerts-persistence bug end-to-end: the bug
# was in lfs_store's read_file() being reused for a fixed-size binary read
# (it always reserves a null-terminator byte), so no in-memory-only test of
# alerts_store's ring logic would have caught it.
add_executable(test_alerts_store
  test_alerts_store.cpp
  ../../src/storage/alerts_store.cpp
  ../../src/storage/lfs_store.cpp
)

target_include_directories(test_alerts_store PRIVATE
  ../../src
  ../../include
)

target_compile_definitions(test_alerts_store PRIVATE ${HOST_COMPILE_DEFS})
target_compile_options(test_alerts_store PRIVATE ${HOST_COMPILE_OPTS})
target_link_libraries(test_alerts_store PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_alerts_store)
