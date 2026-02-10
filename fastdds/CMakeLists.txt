cmake_minimum_required(VERSION 3.6)

project(DDSDraw)

find_package(OpenCV REQUIRED)
find_package(box2d REQUIRED)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core )
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

# Pub/Sub library
add_library(fastdds-tracker publisher.h subscriber.h)
install(TARGETS fastdds-tracker
	EXPORT fastdds-tracker-targets 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

##Qt Window
add_library(brat-gui window.h window.cpp publisher.h subscriber.h)
target_link_libraries(brat-gui Qt5::Core Qt5::Widgets fastcdr fastrtps ObjectPackage)
add_executable(qt_window qt_main.cpp window.h window.cpp topics.h)
target_link_libraries(qt_window PRIVATE brat-gui Qt5::Widgets Qt5::Core fastcdr fastrtps ObjectPackage)
target_link_libraries(qt_window PRIVATE qwt-qt5)

#TESTS
add_executable(test_tracker topics.h test_tracker.cpp gui.h publisher.h )
target_link_libraries(test_tracker fastcdr fastrtps ObjectPackage brat ${OpenCV_LIBS} box2d )

add_executable(test_obstacleQt topics.h test_obstacleQt.cpp gui.h publisher.h )
target_link_libraries(test_obstacleQt fastcdr fastrtps ObjectPackage brat ${OpenCV_LIBS} box2d )


if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
 # find_package(iir)
#  pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
  include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/.. ${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
  include(GNUInstallDirs)
  add_executable(brat3_targetGUI ../custom_robot.h gui.h brat3_targetGUI.cpp)
  target_link_libraries(brat3_targetGUI fastcdr fastrtps ObjectPackage ${OpenCV_LIBS}  alphabot a1lidarrpi brat box2d pigpio pthread)

  message("detected architecture aarch64, building executables")
else()
	message("not building robot programs")
endif()
