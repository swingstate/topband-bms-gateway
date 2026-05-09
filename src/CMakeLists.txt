# IDF component registration for standalone `idf.py build`.
# PlatformIO (`pio run`) ignores this file and uses its own __pio_env mechanism.
#
# Component names are per IDF 5.5.x. If building against IDF 5.3.x, component
# names are identical for the ones listed here.

idf_component_register(
  SRCS
    "main.cpp"
    "app/boot.cpp"
    "app/smoke_reader.cpp"
    "storage/config.cpp"
    "storage/nvs_store.cpp"
    "bus/queues.cpp"
    "bus/snapshot_bus.cpp"
    "bms/protocol.cpp"
    "bms/snapshot.cpp"
    "bms/poller.cpp"
    "safety/filters.cpp"
    "safety/runSafety.cpp"
    "can/victron.cpp"
    "can/tx.cpp"
    "can/busoff.cpp"
  INCLUDE_DIRS
    "."
    "../include"
  REQUIRES
    log
    nvs_flash
    freertos
    esp_timer
    esp_system
    heap
    driver
    esp_hw_support
)
