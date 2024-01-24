cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mti
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/demo.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

