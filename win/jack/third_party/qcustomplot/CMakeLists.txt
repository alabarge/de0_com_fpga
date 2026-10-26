# QCustomPlot 2.1.1, vendored (see README.md for licensing - GPLv3 for now,
# commercial license required before any customer-facing build). Qt6 is
# found once by the root CMakeLists.txt before this subdirectory is added.
add_library(qcustomplot STATIC
    qcustomplot.cpp
    qcustomplot.h
)

target_include_directories(qcustomplot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(qcustomplot PUBLIC Qt6::Widgets Qt6::PrintSupport)

set_target_properties(qcustomplot PROPERTIES AUTOMOC ON)
