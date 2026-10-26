# Qt shell for the jack port. Links against cmapi (built from the same
# sources as cmapi.vcxproj, see ../cmapi/CMakeLists.txt). Qt6 is found once
# by the root CMakeLists.txt before this subdirectory is added.
qt_standard_project_setup()

qt_add_executable(jack-qt
    main.cpp
    MainWindow.cpp
    MainWindow.h
    DeviceBridge.cpp
    DeviceBridge.h
    OutputPane.cpp
    OutputPane.h
    ProjectPane.cpp
    ProjectPane.h
    ConnectPane.cpp
    ConnectPane.h
    CpPane.cpp
    CpPane.h
    DAQPane.cpp
    DAQPane.h
    CpMemPane.cpp
    CpMemPane.h
    GraphPane.cpp
    GraphPane.h
    PlotData.h
    ParmsPane.cpp
    ParmsPane.h
    ImagePane.cpp
    ImagePane.h
)

target_include_directories(jack-qt PRIVATE
    ${CMAKE_SOURCE_DIR}/cmapi
    ${CMAKE_SOURCE_DIR}/fifoapi
    ${CMAKE_SOURCE_DIR}/comapi
    ${CMAKE_SOURCE_DIR}/app # appCfg.h (APP_CON_*/APP_DEVID/... constants)
    ${FW_SHARE_DIR}
)

target_link_libraries(jack-qt PRIVATE
    Qt6::Widgets
    cmapi
    fifoapi
    comapi
    qcustomplot
)

set_target_properties(jack-qt PROPERTIES
    WIN32_EXECUTABLE ON
)

# Make the build output double-clickable from Explorer: cmapi.dll/fifoapi.dll/
# comapi.dll/libserialport.dll all live in different output directories,
# ftd2xx64.dll (FTDI's D2XX driver, checked into fifoapi/x64/) is a runtime
# dependency of fifoapi.dll, and the Qt runtime DLLs/plugins aren't on PATH
# outside a shell with vcvars/Qt bin already set up.
add_custom_command(TARGET jack-qt POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:cmapi> $<TARGET_FILE_DIR:jack-qt>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:fifoapi> $<TARGET_FILE_DIR:jack-qt>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:comapi> $<TARGET_FILE_DIR:jack-qt>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:libserialport> $<TARGET_FILE_DIR:jack-qt>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/fifoapi/x64/ftd2xx64.dll" $<TARGET_FILE_DIR:jack-qt>
    COMMENT "Copying cmapi.dll/fifoapi.dll/comapi.dll/libserialport.dll/ftd2xx64.dll next to jack-qt.exe"
)

find_program(WINDEPLOYQT_EXECUTABLE windeployqt6 HINTS "${QT6_INSTALL_PREFIX}/bin")
if(WINDEPLOYQT_EXECUTABLE)
    add_custom_command(TARGET jack-qt POST_BUILD
        COMMAND "${WINDEPLOYQT_EXECUTABLE}" --no-compiler-runtime $<TARGET_FILE:jack-qt>
        COMMENT "Deploying Qt runtime DLLs/plugins next to jack-qt.exe"
    )
else()
    message(WARNING "windeployqt6 not found - jack-qt.exe won't be double-clickable until it's run manually")
endif()
