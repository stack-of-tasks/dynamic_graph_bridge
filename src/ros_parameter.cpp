#include <sot/core/robot-utils.hh>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <stdexcept>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <urdf_parser/urdf_parser.h>

#include <ros/ros.h>
#include "dynamic_graph_bridge/ros_parameter.hh"

namespace dynamicgraph {
bool parameter_server_read_robot_description()
{
  ros::NodeHandle nh;
  if (!nh.hasParam("/robot_description"))
  {
    ROS_ERROR("No /robot_description parameter");
    return false;
  }

  std::string robot_description;
  std::string parameter_name("/robot_description");
  nh.getParam(parameter_name,robot_description);

  std::string model_name("robot");

  // Search for the robot util related to robot_name.
  sot::RobotUtilShrPtr aRobotUtil = sot::getRobotUtil(model_name);
  // If does not exist then it is created.
  if (aRobotUtil == sot::RefVoidRobotUtil())
    aRobotUtil = sot::createRobotUtil(model_name);

  // If the creation is fine
  if (aRobotUtil != sot::RefVoidRobotUtil())
  {
    // Then set the robot model.
    aRobotUtil->set_parameter(parameter_name,robot_description);
    ROS_INFO("Set parameter_name : %s.",parameter_name.c_str());
    // Everything went fine.
    return true;
  }
  ROS_ERROR("Wrong initialization of parameter_name %s",
            parameter_name.c_str());

  // Otherwise something went wrong.
  return false;

}

}
