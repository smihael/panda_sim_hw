# Gazebo ros_control interfaces for simulated Franka Emika Panda robot

This is a ROS package for that allows to use most features of `franka_state_interface` and `franka_model_interface` with the [Gazebo](http://gazebosim.org/) simulator.

This package provides a Gazebo plugin, based on [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) ([documentation](http://gazebosim.org/tutorials?tut=ros_control)), which instantiates a ros_control controller manager with the default joint interfaces (position, velocity, effort, state) and connects it to a Gazebo model. Additionally, Franka specific interfaces are registered. 

## Usage and dependencies

This is meant to be used along with [gazebo_panda](https://github.com/justagist/gazebo_panda/) and [franka_panda_description](https://github.com/justagist/franka_panda_description). 
Note that, instead of the default gazebo_ros_control plugin (`gazebo_ros_control/DefaultRobotHWSim`), the custom version `panda_hw_gazebo/PandaRobotHWSim` provided by this package has to be loaded in xacro ([example in Reconcycle project](https://github.com/ReconCycle/franka_panda_description/blob/d8cfb29d28b6e5e076a7bbe4a6cbcdde5f6c7acd/robots/panda.transmission.xacro#L24-L29).

## Credits

* Plugin structure: [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/kinetic-devel/gazebo_ros_control) by osrfoundation
* Franka [Model](https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_hw/include/franka_hw/franka_model_interface.h) & [State](https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_hw/include/franka_hw/franka_state_interface.h) Interface specification from [franka_ros](https://github.com/frankaemika/franka_ros/) by Franka Emika GmbH
* [kdl_methods]((https://github.com/justagist/panda_simulator/blob/melodic-devel/panda_gazebo/src/kdl_methods.cpp)) from [panda_gazebo](https://github.com/justagist/panda_simulator/blob/kinetic-devel/panda_gazebo) by Saif Sidhik

## Useful resources for further development

* [ROS hardware_interface documentation](http://docs.ros.org/en/melodic/api/hardware_interface/html/c++/).
