cmake_minimum_required(VERSION 3.0.2)
project(message_send_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Qt5Core)
find_package(Qt5SerialPort)
find_package(Qt5Network)
find_package(OpenCV REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  yolov5_ros_msgs
  cv_bridge
  image_transport
)

# set(loggerFactory_DIR /usr/local/KdrobotCppLibs/lib/cmake/loggerFactory)
# find_package(loggerFactory)
# set(Qt_Util_DIR /usr/local/KdrobotCppLibs/lib/cmake/Qt_Util)
# find_package(Qt_Util)
# set(VCOMCOMM_DIR /usr/local/KdrobotCppLibs/lib/cmake/VCOMCOMM)
# find_package(VCOMCOMM)
find_package(spdlog)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs yolov5_ros_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(message_send src/message_send.cpp) 
add_executable(message_handle src/message_handle.cpp) 
add_executable(video_handle src/video_handle.cpp) 
add_executable(message_test src/message_test.cpp)
add_executable(depth_message src/depth_message.cpp)

target_link_libraries(
  message_send 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # KdrobotCppLibs::loggerFactory
  # KdrobotCppLibs::Qt_Util
  # KdrobotCppLibs::VCOMCOMM
)
target_link_libraries(
  message_handle 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # KdrobotCppLibs::loggerFactory
  # KdrobotCppLibs::Qt_Util
  # KdrobotCppLibs::VCOMCOMM
)
target_link_libraries(
  message_test 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # KdrobotCppLibs::loggerFactory
  # KdrobotCppLibs::Qt_Util
  # KdrobotCppLibs::VCOMCOMM
)
target_link_libraries(
  video_handle 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # KdrobotCppLibs::loggerFactory
  # KdrobotCppLibs::Qt_Util
  # KdrobotCppLibs::VCOMCOMM
)
target_link_libraries(
  depth_message 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # KdrobotCppLibs::loggerFactory
  # KdrobotCppLibs::Qt_Util
  # KdrobotCppLibs::VCOMCOMM
)
add_dependencies(message_send yolov5_ros_msgs_generate_messages_cpp)
add_dependencies(message_handle yolov5_ros_msgs_generate_messages_cpp)
add_dependencies(video_handle yolov5_ros_msgs_generate_messages_cpp)
add_dependencies(depth_message yolov5_ros_msgs_generate_messages_cpp)


catkin_install_python(PROGRAMS
  scripts/video_generate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
