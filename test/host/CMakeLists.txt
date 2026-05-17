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
# victron.cpp, tx.cpp (NATIVE_BUILD stub), and busoff.cpp (NATIVE_BUILD mock).
# All three are host-portable when compiled with NATIVE_BUILD defined.
# Reads JSON fixtures from test/host/fixtures/v267/ (safety) and
# test/host/fixtures/v267_can/ (CAN expected frames).
add_executable(test_can_victron
  test_can_victron.cpp
  ../../src/can/victron.cpp
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
