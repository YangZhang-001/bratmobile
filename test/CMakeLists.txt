 PROJECT(test)
 
cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

find_package(fastcdr REQUIRED)

include(CTest)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	add_executable(closedLoopExecution realWorldTestHeaders.h unit_test/test_classes.h closedLoopExecution.cpp)
	target_link_libraries(closedLoopExecution gtest gtest_main box2d ${OpenCV_LIBS} brat c1lidarrpi zetabot pthread gpiod)
	add_executable(openLoopExecution ../custom_robot.h realWorldTestHeaders.h unit_test/test_classes.h openLoopExecution.cpp)
	target_link_libraries(openLoopExecution gtest gtest_main box2d ${OpenCV_LIBS} brat c1lidarrpi zetabot pthread gpiod)

	message("build test cases")
endif()
add_executable(worldBuildingSpeed worldBuildingSpeed.cpp)
target_link_libraries(worldBuildingSpeed box2d ${OpenCV_LIBS} brat pthread)

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
