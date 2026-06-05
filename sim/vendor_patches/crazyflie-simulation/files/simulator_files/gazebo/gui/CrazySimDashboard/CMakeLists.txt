cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(CrazySimDashboard VERSION 0.1.0)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(gz-plugin REQUIRED COMPONENTS register NAMES gz-plugin2)
set(GZ_PLUGIN_VER ${gz-plugin_VERSION_MAJOR})

find_package(gz-gui REQUIRED NAMES gz-gui8)
set(GZ_GUI_VER ${gz-gui_VERSION_MAJOR})

find_package(gz-transport REQUIRED NAMES gz-transport13)
set(GZ_TRANSPORT_VER ${gz-transport_VERSION_MAJOR})

find_package(gz-msgs REQUIRED NAMES gz-msgs10)
set(GZ_MSGS_VER ${gz-msgs_VERSION_MAJOR})

find_package(Qt5 REQUIRED COMPONENTS Core Quick Qml)

add_library(CrazySimDashboard SHARED
  CrazySimDashboard.hh
  CrazySimDashboard.cc
  CrazySimDashboard.qrc
)

set_property(TARGET CrazySimDashboard PROPERTY CXX_STANDARD 17)

target_link_libraries(CrazySimDashboard
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
    gz-transport${GZ_TRANSPORT_VER}::core
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    Qt5::Core
    Qt5::Quick
    Qt5::Qml
)

include(GNUInstallDirs)
install(TARGETS CrazySimDashboard
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
