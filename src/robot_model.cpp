#include "robot_model.hh"

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <pinocchio/multibody/parser/urdf.hpp>
#include <pinocchio/multibody/model.hpp>

#include "dynamic_graph_bridge/ros_init.hh"
#pragma GCC diagnostic push
#pragma GCC system_header
#include <ros/package.h>
#pragma GCC diagnostic pop

namespace dynamicgraph {

RosRobotModel::RosRobotModel(const std::string& name)
    : Dynamic(name), jointsParameterName_("jrl_map"), ns_("sot_controller") {
  std::string docstring;

  docstring =
      "\n"
      "  Load the robot model from the parameter server.\n"
      "\n"
      "  This is the recommended method.\n"
      "\n";
  addCommand("loadFromParameterServer",
             command::makeCommandVoid0(
                 *this, &RosRobotModel::loadFromParameterServer, docstring));

  docstring =
      "\n"
      "  Load the robot model from an URDF file.\n"
      "\n";
  addCommand("loadUrdf", command::makeCommandVoid1(
                             *this, &RosRobotModel::loadUrdf, docstring));

  docstring =
      "\n"
      "  Set the controller namespace."
      "\n";
  addCommand("setNamespace",
             command::makeCommandVoid1(*this, &RosRobotModel::setNamespace,
                                       docstring));

  docstring =
      "\n"
      "  Get current configuration of the robot.\n"
      "\n";
  addCommand("curConf", new command::Getter<RosRobotModel, Vector>(
                            *this, &RosRobotModel::curConf, docstring));

  docstring =
      "\n"
      "  Maps a link name in the URDF parser to actual robot link name.\n"
      "\n";
  addCommand("addJointMapping",
             command::makeCommandVoid2(*this, &RosRobotModel::addJointMapping,
                                       docstring));
}

RosRobotModel::~RosRobotModel() {}

void RosRobotModel::loadUrdf(const std::string& filename) {
  rosInit(false);
  m_model = se3::urdf::buildModel(filename);
  this->m_urdfPath = filename;
  if (m_data) delete m_data;
  m_data = new se3::Data(m_model);
  init = true;

  //  m_HDR = parser.parse(filename);
  ros::NodeHandle nh(ns_);

  XmlRpc::XmlRpcValue JointsNamesByRank_;
  JointsNamesByRank_.setSize(m_model.names.size());
  std::vector<std::string>::const_iterator it =
      m_model.names.begin() +
      2;  // first joint is universe, second is freeflyer
  for (int i = 0; it != m_model.names.end(); ++it, ++i)
    JointsNamesByRank_[i] = (*it);
  nh.setParam(jointsParameterName_, JointsNamesByRank_);
}

void RosRobotModel::setNamespace(const std::string& ns) { ns_ = ns; }

void RosRobotModel::loadFromParameterServer() {
  rosInit(false);
  std::string robotDescription;
  ros::param::param<std::string>("/robot_description", robotDescription, "");
  if (robotDescription.empty())
    throw std::runtime_error("No model available as ROS parameter. Fail.");
  ::urdf::ModelInterfacePtr urdfTree = ::urdf::parseURDF(robotDescription);
  if (urdfTree)
    se3::urdf::parseTree(urdfTree->getRoot(), this->m_model,
                         se3::SE3::Identity(), false);
  else {
    const std::string exception_message(
        "robot_description not parsed correctly.");
    throw std::invalid_argument(exception_message);
  }

  this->m_urdfPath = "";
  if (m_data) delete m_data;
  m_data = new se3::Data(m_model);
  init = true;
  ros::NodeHandle nh(ns_);

  XmlRpc::XmlRpcValue JointsNamesByRank_;
  JointsNamesByRank_.setSize(m_model.names.size());
  // first joint is universe, second is freeflyer
  std::vector<std::string>::const_iterator it = m_model.names.begin() + 2;
  for (int i = 0; it != m_model.names.end(); ++it, ++i)
    JointsNamesByRank_[i] = (*it);
  nh.setParam(jointsParameterName_, JointsNamesByRank_);
}

Vector RosRobotModel::curConf() const {
  // The first 6 dofs are associated to the Freeflyer frame
  // Freeflyer reference frame should be the same as global
  // frame so that operational point positions correspond to
  // position in freeflyer frame.
  XmlRpc::XmlRpcValue ffpose;
  ros::NodeHandle nh(ns_);
  std::string param_name = "ffpose";
  if (nh.hasParam(param_name)) {
    nh.getParam(param_name, ffpose);
    ROS_ASSERT(ffpose.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(ffpose.size() == 6);
    for (int32_t i = 0; i < ffpose.size(); ++i) {
      ROS_ASSERT(ffpose[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
  } else {
    ffpose.setSize(6);
    for (int32_t i = 0; i < ffpose.size(); ++i) ffpose[i] = 0.0;
  }

  if (!m_data)
    throw std::runtime_error("no robot loaded");
  else {
    // TODO: confirm accesscopy is for asynchronous commands
    Vector currConf = jointPositionSIN.accessCopy();
    for (int32_t i = 0; i < ffpose.size(); ++i)
      currConf(i) = static_cast<double>(ffpose[i]);

    return currConf;
  }
}

void RosRobotModel::addJointMapping(const std::string& link,
                                    const std::string& repName) {
  specialJoints_[link] = repName;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosRobotModel, "RosRobotModel");
}  // end of namespace dynamicgraph.
