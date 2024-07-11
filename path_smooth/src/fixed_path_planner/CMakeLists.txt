cmake_minimum_required(VERSION 3.0.2)
project(fixed_path_planner)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  roslib
)

add_message_files(
          FILES
          way_point.msg
          way_points.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS  roscpp rospy std_msgs tf message_runtime 
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(my_global_planner
  src/my_global_planner.cpp
)
add_dependencies(my_global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_global_planner
  ${catkin_LIBRARIES}
)
