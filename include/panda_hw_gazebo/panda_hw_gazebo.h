/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#ifndef _PANDA_ROBOT_HW_SIM_H_
#define _PANDA_ROBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>

// URDF
#include <urdf/model.h>

// Custom model interface
#include <panda_hw_gazebo/franka_model_interface.h>
#include <panda_hw_gazebo/franka_state_interface.h>

// libfranka robotstate struct
#include <franka/robot_state.h>

// franka state publisher
#include <franka_msgs/FrankaState.h>
#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>

// KDL
#include <Eigen/Geometry>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl/chainfksolver.hpp>
#include <panda_hw_gazebo/kdl_methods.h>
#include <panda_hw_gazebo/franka_model.h>

namespace panda_hw_gazebo
{

  struct Kinematics
  {
    KDL::Chain chain;
    std::vector<std::string> joint_names;
  };

  class PandaRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
  {
  public:
    virtual bool initSim(
        const std::string &robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions);

    virtual void readSim(ros::Time time, ros::Duration period);

  protected:
    franka_hw::FrankaModelInterface franka_model_interface_;
    franka_hw::FrankaStateInterface franka_state_interface_;

  private:
    // states
    franka::RobotState robot_state_;
    std::array<double, 7> gravity_, coriolis_;
    std::array<double, 49> mass_matrix_;
    std::array<double, 42> jacobian_;

    std::string root_name_, tip_name_, gravity_tip_name_;
    urdf::Model robot_model_;
    KDL::Tree tree_;
    std::map<std::string, Kinematics> kinematic_chain_map_;
    KDLMethods *kdl_;

    size_t num_jnts;

    bool initKDL(const ros::NodeHandle &nh);
    bool createKinematicChain(std::string tip_name);

    //Method to get joint  positions, velocities, and efforts and add it to the corresponding KDL Joint Arrays
    void updateRobotStateJoints(KDL::JntArray &jnt_pos, KDL::JntArray &jnt_vel, KDL::JntArray &jnt_eff);

    // Compute Jacobian and add it to Robot State
    void updateJacobian(const KDL::JntArray &jnt_pos, const KDL::JntArray &jnt_vel);

    // Compute coriolis and gravity, and add to Robot State
    void updateCoriolisVec(const KDL::JntArray &jnt_pos, const KDL::JntArray &jnt_vel);
    void updateGravityVec(const KDL::JntArray &jnt_pos);

    // Different methods for inertia calculation
    void updateMassMatrixKDL(const KDL::JntArray& jnt_pos);
    void updateMassMatrixModel();


    // Method to publish the endpoint state message
    void updateRobotStateEndpoint();

    /* Method to calculate the position FK for the required joint configuration in rad
     * with the result stored in geometry_msgs::Pose
     * @returns true if successful
     */
    bool computePositionFK(const Kinematics &kin, const KDL::JntArray &jnt_pos, geometry_msgs::Pose &result);

    // Publish robot state as franka message
    void publishRobotStateMsg();
    realtime_tools::RealtimePublisher<franka_msgs::FrankaState> publisher_franka_states_;
    franka_hw::TriggerRate rate_trigger_{1.0};

    bool mass_calculation_needed = true;
    bool coriolis_calculation_needed = true;
    bool gravity_calculation_needed = true;
    bool robot_state_needed = true;

  };

  typedef boost::shared_ptr<PandaRobotHWSim> PandaRobotHWSimPtr;

}

#endif // #ifndef __PANDA_ROBOT_HW_SIM_H_
