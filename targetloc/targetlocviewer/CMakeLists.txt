cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(TargetLocViewer
	DESCRIPTION "QT app to visulalise the 3D detection"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall")

find_package(PkgConfig)
find_package(Qt6 COMPONENTS Widgets REQUIRED)
find_package(QCustomPlot)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include(GNUInstallDirs)

add_executable(targetlocviewer
  window.cpp
  main.cpp
)

target_link_libraries(targetlocviewer Qt6::Widgets QCustomPlotQt6)
target_link_libraries(targetlocviewer targetloc)
target_link_libraries(targetlocviewer ${OpenCV_LIBS})

if(NOT TARGETLOC_USE_ROCK5_V4L_CAMERA)
  target_link_libraries(targetlocviewer PkgConfig::LIBCAMERA)
  target_link_libraries(targetlocviewer cam2opencv)
  target_link_libraries(targetlocviewer ${TURBOJPEG_LINK_LIBRARIES})
endif()
