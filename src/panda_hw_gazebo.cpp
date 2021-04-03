#include <panda_hw_gazebo/panda_hw_gazebo.h>
#include <urdf/model.h>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace panda_hw_gazebo
{


bool PandaRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace,model_nh,parent_model,urdf_model,transmissions);

    //Register model handle & interface
    franka_hw::FrankaModelHandle model_handle("panda_model");
    franka_model_interface_.registerHandle(model_handle);
    registerInterface(&franka_model_interface_);

  return true;
}

void PandaRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time,period);
}


}

PLUGINLIB_EXPORT_CLASS(panda_hw_gazebo::PandaRobotHWSim, gazebo_ros_control::RobotHWSim)
