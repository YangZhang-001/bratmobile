# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

include(CTest)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	# add_executable(closed_loop_task ../custom_robot.h cl_task.cpp)
	# add_executable(test_tracking ../custom_robot.h test_tracking.cpp)
	# target_link_libraries(closed_loop_task box2d ${OpenCV_LIBS} brat a1lidarrpi alphabot pthread pigpio)
	# target_link_libraries(test_tracking box2d ${OpenCV_LIBS} brat a1lidarrpi alphabot pthread pigpio)
	add_executable(closedLoopExecution realWorldTestHeaders.h closedLoopExecution.cpp)
	target_link_libraries(closedLoopExecution box2d ${OpenCV_LIBS} brat a1lidarrpi alphabot pthread pigpio)
	message("build test cases")
else()
	# #add_subdirectory(simulations)
	# find_package(GTest CONFIG REQUIRED)

	# include(CTest)
	# include(FetchContent)

	# include_directories(${GTEST_INCLUDE_DIRS})  # Var managed by CMake
	# include(GoogleTest)
	# add_executable(execution_unit execution_unit.cpp)
	# target_link_libraries(execution_unit gtest gtest_main)
	# target_link_libraries(execution_unit brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

	gtest_discover_tests(execution_unit)

endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
