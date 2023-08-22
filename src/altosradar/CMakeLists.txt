cmake_minimum_required(VERSION 3.0.2)
project(altosradar)

find_package(catkin REQUIRED COMPONENTS
  message_generation pcl_ros roscpp rospy sensor_msgs std_msgs)

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
