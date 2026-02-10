cmake_minimum_required(VERSION 3.16)

project(brat LANGUAGES CXX)

include(CTest)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)


add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

option(BUILD_FASTDDS "Building FastDDS directory" ON)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
 # find_package(iir)
#  pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
  include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/.. ${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
  include(GNUInstallDirs)
  add_executable(brat2_target custom_robot.h brat2_target.cpp)
  add_executable(brat2_targetless custom_robot.h brat2_targetless.cpp)
  add_executable(brat3_target custom_robot.h brat3_target.cpp)
  add_executable(brat3_targetless custom_robot.h brat3_targetless.cpp)
  add_executable(reactive custom_robot.h reactive.cpp)


  target_link_libraries(brat2_targetless ${OpenCV_LIBS}  box2d alphabot a1lidarrpi brat pigpio)
  target_link_libraries(brat2_target ${OpenCV_LIBS} box2d alphabot a1lidarrpi brat pigpio)
  target_link_libraries(brat3_targetless ${OpenCV_LIBS}  box2d alphabot a1lidarrpi brat pigpio)
  target_link_libraries(brat3_target ${OpenCV_LIBS} box2d alphabot a1lidarrpi brat pigpio)
  target_link_libraries(reactive ${OpenCV_LIBS} box2d alphabot a1lidarrpi brat pigpio)
  
  add_subdirectory(test)
  message("detected architecture aarch64, building executables")
else()
  add_subdirectory(test/unit_test "${CMAKE_CURRENT_BINARY_DIR}/test/unit_test")
  add_subdirectory(test "${CMAKE_CURRENT_BINARY_DIR}/test")

	message("not building any executable")
endif()

if(BUILD_FASTDDS)
  add_subdirectory(fastdds "${CMAKE_CURRENT_BINARY_DIR}/fastdds")
endif()




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)

enable_testing()
