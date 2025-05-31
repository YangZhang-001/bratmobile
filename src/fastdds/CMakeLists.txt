cmake_minimum_required(VERSION 3.6)

project(DDSDraw)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

set (CMAKE_CXX_STANDARD 17)

# Create a library for the message / topic
add_custom_target(
    objects
    COMMAND fastddsgen Object.idl
)
add_library(Object STATIC Object.cxx ObjectPubSubTypes.cxx)
add_dependencies(Object objects)

# Publisher
add_executable(publisher publisher.cpp)
target_link_libraries(publisher fastcdr fastrtps Object)


# Subscriber
add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber fastcdr fastrtps Object)