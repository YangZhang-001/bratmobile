cmake_minimum_required(VERSION 3.20)
project(deltabot LANGUAGES CXX)


find_package(Threads REQUIRED)

add_library(deltabotdrive
  STATIC
  Pwm.cpp
  Driving.cpp
  ServoMotorSetting.cpp
  )

target_include_directories(deltabotdrive PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(deltabotdrive PUBLIC cxx_std_17)

target_link_libraries(deltabotdrive PUBLIC Threads::Threads)

target_compile_options(deltabotdrive PRIVATE -Wall -Wextra -pedantic)


add_executable(TestServoMotor
  TestServoMotor.cpp
  )

target_link_libraries(TestServoMotor PRIVATE deltabotdrive)

target_compile_options(TestServoMotor
  PRIVATE
  -Wall
  -Wextra
  -pedantic
  )