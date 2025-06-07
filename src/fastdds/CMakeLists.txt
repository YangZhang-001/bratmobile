cmake_minimum_required(VERSION 3.6)

project(DDSDraw)


set (CMAKE_CXX_STANDARD 17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED )
# Create a library for the message / topic
add_custom_target(
    objects
    COMMAND fastddsgen ObjectPackage.idl
)
add_library(ObjectPackage STATIC ObjectPackage.cxx ObjectPackagePubSubTypes.cxx)
add_dependencies(ObjectPackage objects)

# Publisher
add_executable(publisher publisher.cpp)
target_link_libraries(publisher fastcdr fastrtps ObjectPackage)


# Subscriber
add_executable(subscriber subscriber.cpp print_helpers.h)
target_link_libraries(subscriber fastcdr fastrtps ObjectPackage)

##Qt Window
add_library(window window.h window.cpp)
target_link_libraries(window Qt5::Widgets fastcdr fastrtps ObjectPackage)
add_executable(qt_window qt_main.cpp window.h window.cpp )
target_link_libraries(qt_window  PRIVATE Qt5::Widgets fastcdr fastrtps ObjectPackage)
target_link_libraries(qt_window PRIVATE qwt-qt5)

