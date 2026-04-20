cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(CamViewer
	DESCRIPTION "Example QT application which displays the camera image"
	LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall -Winvalid-pch -Wnon-virtual-dtor -Wextra -Wno-unused-parameter")

find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(Qt6 COMPONENTS Widgets REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
find_package(OpenMP REQUIRED)
include_directories(${OpenMP_INCLUDE_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

message(STATUS "libcamera library found:")
message(STATUS "    version: ${LIBCAMERA_VERSION}")
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

message(STATUS "opencv:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "turbojpeg:")
message(STATUS "    version: ${TURBOJPEG_VERSION}")
message(STATUS "    libraries: ${TURBOJPEG_LINK_LIBRARIES}")
message(STATUS "    include path: ${TURBOJPEG_INCLUDE_DIRS}")

include_directories(
	${CMAKE_SOURCE_DIR}
	${TURBOJPEG_INCLUDE_DIRS}
	${LIBCAMERA_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	..
)
include(GNUInstallDirs)

add_executable(stereoviewer
  window.cpp
  stereo.cpp
  main.cpp
)

target_link_directories(stereoviewer PUBLIC ..)
target_link_libraries(stereoviewer PkgConfig::LIBCAMERA)
target_link_libraries(stereoviewer Qt6::Widgets)
target_link_libraries(stereoviewer ${OpenCV_LIBS})
target_link_libraries(stereoviewer cam2opencv)
target_link_libraries(stereoviewer ${TURBOJPEG_LINK_LIBRARIES})
target_link_libraries(stereoviewer prime_stereo_match)
