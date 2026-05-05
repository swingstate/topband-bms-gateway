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
