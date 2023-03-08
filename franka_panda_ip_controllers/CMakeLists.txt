cmake_minimum_required(VERSION 2.8.3)
project(franka_panda_ip_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/joint_controller_params.cfg
  cfg/joint_position_controller_ip_params.cfg
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(${PROJECT_NAME}_robot_motion_generation
  src/robot_motion_generation/CDDynamics.cpp
)

add_library(${PROJECT_NAME}
  src/franka_robot_joint_position_trapezoid_controller.cpp
  src/position_joint_position_controller.cpp
  src/joint_position_controller_ip.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}  
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_robot_motion_generation
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
   ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_robot_motion_generation PUBLIC
  include
   ${catkin_INCLUDE_DIRS}
)

add_executable(collision_behavior_ip_node src/services_collision_behavior_ip.cpp)
add_dependencies(collision_behavior_ip_node ${PROJECT_NAME}_gencpp)
target_link_libraries(collision_behavior_ip_node PUBLIC ${catkin_LIBRARIES} )
