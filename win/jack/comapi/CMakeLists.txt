# CMake build for comapi, parallel to comapi.vcxproj (see cmapi/CMakeLists.txt
# for why this exists alongside the untouched MSBuild project).
#
# com.cpp reads BUILD_MAJOR/MINOR/NUM/INC from build.h, normally generated
# from build.inc by a prebuild step outside this repo - see fifoapi's
# CMakeLists.txt for the same situation.
configure_file(build.inc "${CMAKE_CURRENT_BINARY_DIR}/build.h" COPYONLY)

add_library(comapi SHARED
    com.cpp
    dllmain.cpp
    stdafx.cpp
    trace.cpp
)

target_include_directories(comapi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/cmapi
    ${FW_SHARE_DIR}
)

target_compile_definitions(comapi PRIVATE
    WIN32
    _WINDOWS
    COM_EXPORTS
    _CRT_SECURE_NO_WARNINGS
    UNICODE
    _UNICODE
)

target_link_libraries(comapi PRIVATE
    cmapi
    libserialport
    user32
)
