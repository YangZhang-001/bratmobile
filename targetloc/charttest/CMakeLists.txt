cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(CamViewer
	DESCRIPTION "Example QT application which displays the camera image"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall")

find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(Qt6 COMPONENTS Widgets Charts REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
	${CMAKE_SOURCE_DIR}
)

include(GNUInstallDirs)

add_executable(targetlocviewer
  window.cpp
  main.cpp
)

target_link_libraries(targetlocviewer Qt6::Widgets Qt6::Charts)

