cmake_minimum_required(VERSION 3.16)

project(PISTON_GUI_V1 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS
    Quick
    Bluetooth
)

qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(appPISTON_GUI_V1
    main.cpp

    PistonBackend.cpp
    PistonBackend.h

    LiveDataModel.cpp
    LiveDataModel.h

    DiagnosticHistoryModel.cpp
    DiagnosticHistoryModel.h
)

qt_add_qml_module(appPISTON_GUI_V1
    URI PISTON_GUI_V1

    QML_FILES
        Main.qml

    RESOURCES
        images/logo.png
)

set_target_properties(appPISTON_GUI_V1 PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appPISTON_GUI_V1
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appPISTON_GUI_V1
    PRIVATE
        Qt6::Quick
        Qt6::Bluetooth
)

if(ANDROID)

    set_property(
        TARGET appPISTON_GUI_V1
        PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/android"
    )

    qt_add_android_permission(
        appPISTON_GUI_V1
        NAME android.permission.BLUETOOTH_SCAN
        ATTRIBUTES
            minSdkVersion 31
            usesPermissionFlags neverForLocation
    )

    qt_add_android_permission(
        appPISTON_GUI_V1
        NAME android.permission.BLUETOOTH_CONNECT
    )

    qt_add_android_permission(
        appPISTON_GUI_V1
        NAME android.permission.BLUETOOTH_ADVERTISE
    )
endif()

include(GNUInstallDirs)

install(TARGETS appPISTON_GUI_V1
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
