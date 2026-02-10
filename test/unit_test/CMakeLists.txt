 
cmake_minimum_required(VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)
project(brat_test LANGUAGES CXX)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)


add_library(test_classes test_classes.h ../test_essentials.h ../callbacks.h)
target_link_libraries(test_classes gtest gtest_main)
target_link_libraries( test_classes brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)


if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64|AMD64") # for x86
    find_package(GTest CONFIG REQUIRED)

    include(CTest)
    include(FetchContent)

    include_directories(${GTEST_INCLUDE_DIRS})  # Var managed by CMake
    include(GoogleTest)
    message(STATUS "GTest include dirs: ${GTEST_INCLUDE_DIRS}")

    add_executable(GTest_HighLevel GTest_HighLevel.cpp)
    target_link_libraries(GTest_HighLevel gtest gtest_main)
    target_link_libraries(GTest_HighLevel  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Math GTest_Math.cpp)
    target_link_libraries(GTest_Math gtest gtest_main)
    target_link_libraries(GTest_Math  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Configurator GTest_Configurator.cpp)
    target_link_libraries(GTest_Configurator gtest gtest_main)
    target_link_libraries(GTest_Configurator  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_B2B GTest_B2B.cpp)
    target_link_libraries(GTest_B2B gtest gtest_main)
    target_link_libraries(GTest_B2B  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_B2BTmp GTest_B2BTmp.cpp)
    target_link_libraries(GTest_B2BTmp gtest gtest_main)
    target_link_libraries(GTest_B2BTmp  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)


    add_executable(GTest_Worldbuilding GTest_Worldbuilding.cpp)
    target_link_libraries(GTest_Worldbuilding gtest gtest_main)
    target_link_libraries(GTest_Worldbuilding  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Logger GTest_Logger.cpp)
    target_link_libraries(GTest_Logger gtest gtest_main)
    target_link_libraries(GTest_Logger  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Matcher GTest_Matcher.cpp)
    target_link_libraries(GTest_Matcher gtest gtest_main)
    target_link_libraries(GTest_Matcher  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Graph GTest_Graph.cpp)
    target_link_libraries(GTest_Graph gtest gtest_main)
    target_link_libraries(GTest_Graph  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Planner GTest_Planner.cpp)
    target_link_libraries(GTest_Planner gtest gtest_main)
    target_link_libraries(GTest_Planner  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Task GTest_Task.cpp)
    target_link_libraries(GTest_Task gtest gtest_main)
    target_link_libraries(GTest_Task  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Controller GTest_Controller.cpp)
    target_link_libraries(GTest_Controller gtest gtest_main)
    target_link_libraries(GTest_Controller  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Tracker GTest_Tracker.cpp)
    target_link_libraries(GTest_Tracker gtest gtest_main)
    target_link_libraries(GTest_Tracker  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_FocusedConfig GTest_FocusedConfig.cpp)
    target_link_libraries(GTest_FocusedConfig gtest gtest_main)
    target_link_libraries(GTest_FocusedConfig  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    add_executable(GTest_Discrete GTest_Discrete.cpp)
    target_link_libraries(GTest_Discrete gtest gtest_main)
    target_link_libraries(GTest_Discrete  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)


    add_executable(GTest_ForFigures GTest_ForFigures.cpp)
    target_link_libraries(GTest_ForFigures gtest gtest_main)
    target_link_libraries(GTest_ForFigures  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

    gtest_discover_tests(GTest_HighLevel)
    gtest_discover_tests(GTest_Math)
    gtest_discover_tests(GTest_Configurator)
    gtest_discover_tests(GTest_B2B)
    gtest_discover_tests(GTest_B2BTmp)
    gtest_discover_tests(GTest_Worldbuilding)
    gtest_discover_tests(GTest_Controller)
    gtest_discover_tests(GTest_Logger)
    gtest_discover_tests(GTest_Matcher)
    gtest_discover_tests(GTest_Graph)
    gtest_discover_tests(GTest_Planner)
    gtest_discover_tests(GTest_Task)
    gtest_discover_tests(GTest_Tracker)
    gtest_discover_tests(GTest_FocusedConfig)
    gtest_discover_tests(GTest_Discrete)
else()
    enable_testing()
    include(FetchContent)

    FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    )

    # This line avoids weird platform defaults
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(googletest)
endif()




add_executable(GTest_Recycle_Exp GTest_Recycle_Exp.cpp)
target_link_libraries(GTest_Recycle_Exp gtest gtest_main)
target_link_libraries(GTest_Recycle_Exp  brat ${OpenCV_LIBS} box2d rt GTest::gtest_main GTest::gtest pthread)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64|AMD64") # for x86
    gtest_discover_tests(GTest_Recycle_Exp)
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)

