cmake_minimum_required(VERSION 3.25.0)

project(brat LANGUAGES CXX)

include(CTest)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

option(BUILD_FASTDDS "Building FastDDS directory" 0)

option(BRAT_BUILD_TARGETLOC_NAVIGATION
       "Link TargetLoc into the brat3 target executable"
       OFF)

# optinal mode for rock5 motor backend
option(BRAT_USE_ROCK5_WHEELEDDRIVE
       "Use Rock 5 sysfs PWM wheel backend"
       OFF)

option(BRAT_ROCK5_MOTOR_DRY_RUN
       "Print Rock 5 navigation motor commands without driving PWM"
       ON)

if(BRAT_USE_ROCK5_WHEELEDDRIVE)
  add_subdirectory(wheeleddrive)
  set(BRAT_MOTOR_BACKEND deltabotdrive)
  # use the same macro in the code to conditionally compile the motor backend
  add_compile_definitions(BRAT_USE_ROCK5_WHEELEDDRIVE=1)

  if(BRAT_ROCK5_MOTOR_DRY_RUN)
    add_compile_definitions(BRAT_ROCK5_MOTOR_DRY_RUN=1)
  endif()
else()
  set(BRAT_MOTOR_BACKEND zetabot)
endif()

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
 # find_package(iir)
#  pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
  include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/.. ${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
  include(GNUInstallDirs)
  add_executable(brat2_target custom_robot.h brat2_target.cpp)
  add_executable(brat2_targetless custom_robot.h brat2_targetless.cpp)
  add_executable(brat3_target custom_robot.h brat3_target.cpp)
  add_executable(brat3_target_wb custom_robot.h brat3_target_wb.cpp)
  add_executable(brat3_targetless custom_robot.h brat3_targetless.cpp)
  add_executable(reactive custom_robot.h reactive.cpp)

  if(BRAT_BUILD_TARGETLOC_NAVIGATION)
    set(TARGETLOC_BUILD_VIEWER OFF CACHE BOOL
        "Build the TargetLoc Qt viewer")

    add_subdirectory(targetloc)

    target_link_libraries(brat3_target targetloc)

    target_compile_definitions(
      brat3_target
      PRIVATE
      BRAT_BUILD_TARGETLOC_NAVIGATION=1
    )
    
  endif()
  target_link_libraries(brat2_targetless ${OpenCV_LIBS}  box2d ${BRAT_MOTOR_BACKEND} gpiodcxx c1lidarrpi brat gpiod)
  target_link_libraries(brat2_target ${OpenCV_LIBS} box2d ${BRAT_MOTOR_BACKEND} c1lidarrpi brat gpiod)
  target_link_libraries(brat3_targetless ${OpenCV_LIBS}  box2d ${BRAT_MOTOR_BACKEND} c1lidarrpi brat gpiod)
  target_link_libraries(brat3_target ${OpenCV_LIBS} box2d ${BRAT_MOTOR_BACKEND} c1lidarrpi brat gpiod)
  target_link_libraries(brat3_target_wb ${OpenCV_LIBS} box2d ${BRAT_MOTOR_BACKEND} c1lidarrpi brat gpiod)
  target_link_libraries(reactive ${OpenCV_LIBS} box2d ${BRAT_MOTOR_BACKEND} c1lidarrpi brat gpiod)
  
  if (BUILD_TESTING)
  add_subdirectory(test)
  endif()
  message("detected architecture aarch64, building executables")
else()
  if (BUILD_TESTING)
  add_subdirectory(test/unit_test "${CMAKE_CURRENT_BINARY_DIR}/test/unit_test")
  add_subdirectory(test "${CMAKE_CURRENT_BINARY_DIR}/test")
  endif()

	message("not building any executable")
endif()

if(BUILD_FASTDDS)
  add_subdirectory(fastdds "${CMAKE_CURRENT_BINARY_DIR}/fastdds")
endif()




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)

enable_testing()
