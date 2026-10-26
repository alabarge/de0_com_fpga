# CMake build for cmapi, parallel to cmapi.vcxproj (MSBuild remains the build
# of record for the MFC app; this target exists for the new Qt app).
add_library(cmapi SHARED
    cmapi.cpp
    dllmain.cpp
    stdafx.cpp
    trace.cpp
)

target_include_directories(cmapi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${FW_SHARE_DIR}
)

target_compile_definitions(cmapi PRIVATE
    WIN32
    _WINDOWS
    CMAPI_EXPORTS
    _CRT_SECURE_NO_WARNINGS
    UNICODE
    _UNICODE
)

target_link_libraries(cmapi PRIVATE user32)
