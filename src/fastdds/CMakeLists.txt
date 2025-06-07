cmake_minimum_required(VERSION 3.6)

project(DDSDraw)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Gui)

set (CMAKE_CXX_STANDARD 17)

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

##Qt Window
# add_executable(visualise visualise.cpp)
# target_link_libraries(visualise PRIVATE Qt5::Gui)


# Subscriber
add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber fastcdr fastrtps ObjectPackage)