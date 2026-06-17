cmake_minimum_required(VERSION 3.25)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(TargetLocator
	DESCRIPTION "A library which detects an object in 3D using stereo and LIDAR"
	LANGUAGES CXX)
option(TARGETLOC_USE_ROCK5_V4L_CAMERA "Use Rock 5 OpenCV/V4L camera backend" OFF)

set (CMAKE_CXX_STANDARD 17)

set (CMAKE_CXX_FLAGS "-Wall")

set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(PkgConfig)
find_package(OpenCV REQUIRED)

if (NOT TARGETLOC_USE_ROCK5_V4L_CAMERA)
   pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
   pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

   message(STATUS "libcamera library found:")
   message(STATUS "    version: ${LIBCAMERA_VERSION}")
   message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
   message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

   message(STATUS "turbojpeg:")
   message(STATUS "    version: ${TURBOJPEG_VERSION}")
   message(STATUS "    libraries: ${TURBOJPEG_LINK_LIBRARIES}")
   message(STATUS "    include path: ${TURBOJPEG_INCLUDE_DIRS}")
endif()

message(STATUS "opencv:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
	${CMAKE_SOURCE_DIR}
	${TURBOJPEG_INCLUDE_DIRS}
	${LIBCAMERA_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

set(LIBSRC stereo.cpp targetloc.cpp targetdet.cpp)
set(LIBINCLUDE stereo.h targetloc.h targetdet.h)

if(TARGETLOC_USE_ROCK5_V4L_CAMERA)
	list(APPEND LIBSRC rock5_V4Lcamera_backend.cpp)
	list(APPEND LIBINCLUDE rock5_V4Lcamera_backend.h)
endif()

add_library(targetloc STATIC ${LIBSRC})

if(TARGETLOC_USE_ROCK5_V4L_CAMERA)
  target_compile_definitions(targetloc PUBLIC TARGETLOC_USE_ROCK5_V4L_CAMERA=1)
endif()

target_link_libraries(targetloc ${OpenCV_LIBS})
target_link_libraries(targetloc c1lidarrpi)

if(TARGETLOC_USE_ROCK5_V4L_CAMERA)
	target_link_libraries(targetloc v4l2)
else()
	target_link_libraries(targetloc PkgConfig::LIBCAMERA)
	target_link_libraries(targetloc cam2opencv)
	target_link_libraries(targetloc ${TURBOJPEG_LINK_LIBRARIES})
endif()

include(GNUInstallDirs)

add_subdirectory(targetlocviewer)
