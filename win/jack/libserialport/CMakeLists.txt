# CMake build for libserialport, parallel to libserialport.vcxproj (see
# cmapi/CMakeLists.txt for why this exists alongside the untouched MSBuild
# project). Only the three Windows sources are compiled, matching the
# vcxproj's own ClCompile list - freebsd.c/linux*.c/macosx.c are other
# platforms' backends.
add_library(libserialport SHARED
    serialport.c
    timing.c
    windows.c
)

target_include_directories(libserialport PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# debug.props/release.props: LIBSERIALPORT_MSBUILD selects dllexport (via
# libserialport.h's SP_API macro) when building the library itself: see
# libserialport.h ~line 284.
target_compile_definitions(libserialport PRIVATE LIBSERIALPORT_MSBUILD)

# common.props's Link/AdditionalDependencies.
target_link_libraries(libserialport PRIVATE setupapi)
