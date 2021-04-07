cmake_minimum_required(VERSION 2.8.3)
project(panda_hw_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  gazebo_ros_control
  kdl_parser
  orocos_kdl
  tf_conversions
  franka_hw
  franka_msgs
)

find_package(Franka 0.7.0 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
    gazebo_ros_control
    kdl_parser
    tf_conversions
    franka_hw
    franka_msgs
    DEPENDS Franka

  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_plugin ${PROJECT_NAME}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME}_plugin src/panda_hw_gazebo_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} 
 src/panda_hw_gazebo.cpp
 src/kdl_methods.cpp
 #src/kinematics.cpp
 #src/MassMatrix.cpp
 #src/franka_model_handle_impl.cpp
)
target_link_libraries(${PROJECT_NAME} 
 ${catkin_LIBRARIES} 
 ${Franka_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
