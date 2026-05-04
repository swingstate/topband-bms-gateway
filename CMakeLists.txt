cmake_minimum_required(VERSION 3.16)

# This file is used by `idf.py build` (CI) and by PlatformIO as the IDF
# project root. PlatformIO with framework=espidf handles source compilation
# through its own __pio_env component; do not add EXTRA_COMPONENT_DIRS here
# or source files will be compiled twice.
#
# For standalone `idf.py build`:
#   export IDF_PATH=<path-to-idf-5.3>
#   idf.py build
# Note: src/ contains idf_component_register() for standalone builds only.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(topband-bms-gateway)
