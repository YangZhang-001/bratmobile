# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

include(CTest)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	add_executable(closed_loop_task ../custom.h cl_task.cpp)
	target_link_libraries(closed_loop_task box2d ${OpenCV_LIBS} CloCK a1lidarrpi alphabot pthread pigpio)
	message("build test cases")
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
