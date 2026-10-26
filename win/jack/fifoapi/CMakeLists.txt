# CMake build for fifoapi, parallel to fifoapi.vcxproj (see cmapi/CMakeLists.txt
# for why this exists alongside the untouched MSBuild project).
#
# fifo.cpp reads BUILD_MAJOR/MINOR/NUM/INC from build.h, normally generated
# from build.inc by a prebuild step (..\..\utils\version.exe) outside this
# repo. We don't have that tool here, so just copy build.inc's #defines
# verbatim into the build tree instead of reimplementing its auto-increment.
configure_file(build.inc "${CMAKE_CURRENT_BINARY_DIR}/build.h" COPYONLY)

add_library(fifoapi SHARED
    fifo.cpp
    dllmain.cpp
    stdafx.cpp
    trace.cpp
)

target_include_directories(fifoapi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/cmapi
    ${FW_SHARE_DIR}
)

target_compile_definitions(fifoapi PRIVATE
    WIN32
    _WINDOWS
    FIFO_EXPORTS
    _CRT_SECURE_NO_WARNINGS
    UNICODE
    _UNICODE
)

target_link_directories(fifoapi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/x64)

target_link_libraries(fifoapi PRIVATE
    cmapi
    ftd2xx
    user32
)
