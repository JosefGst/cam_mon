cmake_minimum_required(VERSION 3.0.2)
project(cam_mon)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

roslint_cpp()

generate_dynamic_reconfigure_options(
  config/reconf.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cam_mon
  CATKIN_DEPENDS roscpp rospy std_msgs

  # DEPENDS system_lib
)

add_library(cam_mon_lib
  src/cam_mon_ros.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cam_mon_node src/cam_mon_node.cpp)
add_dependencies(cam_mon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_mon_node cam_mon_lib ${catkin_LIBRARIES})

add_dependencies(cam_mon_node ${PROJECT_NAME}_gencfg)
