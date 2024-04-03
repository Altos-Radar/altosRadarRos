cmake_minimum_required(VERSION 3.0.2)
project(altosradar)
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)  #填你自己的路径
find_package(catkin REQUIRED COMPONENTS
  message_generation pcl_ros roscpp rospy sensor_msgs std_msgs OpenCV cv_bridge image_transport)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  )
add_executable(altosRadarParse
  src/altosRadarParse.cpp
)
add_dependencies(altosRadarParse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(altosRadarParse
  ${catkin_LIBRARIES}
)
