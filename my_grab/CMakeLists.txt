cmake_minimum_required(VERSION 2.8.3)
project(my_grab)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)




find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf
  tf2_ros
  #tf2_eigen
  tf2_geometry_msgs
)
    




###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_grab
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ executable
## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(MoveGroupInterface_To_Kinetic src/MoveGroupInterface_To_Kinetic.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(MoveGroupInterface_To_Kinetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})







