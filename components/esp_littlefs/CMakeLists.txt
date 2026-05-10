# Vendored esp_littlefs v1.21.1 (joltwallet/esp_littlefs) + littlefs-project/littlefs
# Pinned at: joltwallet tag v1.21.1, littlefs SHA 6cb4e86540eca0d9ba62500a298385c9d863c8be
# SDMMC support excluded; IDF-6+ block-device layer excluded (IDF 5.5.3 build).
# Fetched from GitHub and vendored because IDF 5.5.3 PlatformIO bundle
# does not include esp_littlefs as a built-in component.

set(SOURCES
    src/esp_littlefs.c
    src/lfs_config.c
    src/littlefs_esp_part.c
    src/littlefs/lfs.c
    src/littlefs/lfs_util.c
)

idf_component_register(
    SRCS            ${SOURCES}
    INCLUDE_DIRS    include
    PRIV_INCLUDE_DIRS src src/littlefs
    REQUIRES        sdmmc esp_partition
    PRIV_REQUIRES   spi_flash vfs
)

target_compile_options(${COMPONENT_LIB} PRIVATE -DLFS_CONFIG=lfs_config.h)
