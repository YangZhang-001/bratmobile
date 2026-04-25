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
find_package(Qt6 COMPONENTS Widgets REQUIRED)
find_package(QCustomPlot)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

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
)

set(LIBSRC stereo.cpp targetloc.cpp targetdet.cpp)
set(LIBINCLUDE stereo.h targetloc.h targetdet.h)

add_library(targetloc
  STATIC
  ${LIBSRC}
  )

target_link_libraries(targetloc PkgConfig::LIBCAMERA)
target_link_libraries(targetloc ${OpenCV_LIBS})
target_link_libraries(targetloc cam2opencv)
target_link_libraries(targetloc ${TURBOJPEG_LINK_LIBRARIES})
target_link_libraries(targetloc c1lidarrpi)

include(GNUInstallDirs)

add_executable(targetlocviewer
  window.cpp
  main.cpp
)

target_link_libraries(targetlocviewer Qt6::Widgets QCustomPlotQt6)
target_link_libraries(targetlocviewer targetloc)
target_link_libraries(targetlocviewer PkgConfig::LIBCAMERA)
target_link_libraries(targetlocviewer ${OpenCV_LIBS})
target_link_libraries(targetlocviewer cam2opencv)
target_link_libraries(targetlocviewer ${TURBOJPEG_LINK_LIBRARIES})
