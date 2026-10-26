cmake_minimum_required(VERSION 3.21)
# C is needed for libserialport (comapi's dependency for COM support) -
# its sources are plain .c, unlike every other library here.
project(jack_qt LANGUAGES CXX C)

# This CMake build is additive: it exists for the new Qt port and does not
# touch jack.sln / the .vcxproj files, which remain the build of record for
# the MFC app until the Qt app reaches parity.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED FW_SHARE_DIR)
    set(FW_SHARE_DIR "${CMAKE_SOURCE_DIR}/../../nios/de0_fw/share" CACHE PATH
        "Shared NIOS firmware / GUI protocol headers (cm_msg.h, cp_msg.h, daq_msg.h, ...)")
endif()

add_subdirectory(cmapi)
add_subdirectory(fifoapi)
add_subdirectory(libserialport)
add_subdirectory(comapi)

# Shared by third_party/qcustomplot and qt-app - found once here so both can
# just use the Qt6::* targets without repeating/guarding the lookup.
find_package(Qt6 COMPONENTS Widgets PrintSupport)

if(Qt6_FOUND)
    add_subdirectory(third_party/qcustomplot)
    add_subdirectory(qt-app)
else()
    message(WARNING
        "Qt6 (MSVC kit) not found - skipping jack-qt and qcustomplot targets. "
        "Set -DCMAKE_PREFIX_PATH=<Qt 6.12 LTS MSVC install dir> to build them.")
endif()
