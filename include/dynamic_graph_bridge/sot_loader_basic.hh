/*
 * Copyright 2016,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#ifndef _SOT_LOADER_BASIC_HH_
#define _SOT_LOADER_BASIC_HH_

// System includes
#include <cassert>

// STL includes
#include <map>

// Boost includes
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "std_srvs/srv/empty.hpp"

// Sot Framework includes
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/debug.hh>

#include "dynamic_graph_bridge/ros.hpp"

namespace po = boost::program_options;
namespace dgs = dynamicgraph::sot;

class SotLoaderBasic : public rclcpp::Node {
 protected:
  // Dynamic graph is stopped.
  bool dynamic_graph_stopped_;

  /// \brief the sot-hrp2 controller
  dgs::AbstractSotExternalInterface* sotController_;

  po::variables_map vm_;
  std::string dynamicLibraryName_;

  /// \brief Handle on the SoT library.
  void* sotRobotControllerLibrary_;

  /// \brief Map between SoT state vector and some joint_state_links
  // XmlRpc::XmlRpcValue stateVectorMap_;

  /// \brief List of parallel joints from the state vector.
  typedef std::vector<int> parallel_joints_to_state_vector_t;
  parallel_joints_to_state_vector_t parallel_joints_to_state_vector_;

  /// \brief Coefficient between parallel joints and the state vector.
  std::vector<double> coefficient_parallel_joints_;
  /// Advertises start_dynamic_graph services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_start_;

  /// Advertises stop_dynamic_graph services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_stop_;

  // Joint state publication.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  // Joint state to be published.
  sensor_msgs::msg::JointState joint_state_;

  // Number of DOFs according to KDL.
  int nbOfJoints_;
  parallel_joints_to_state_vector_t::size_type nbOfParallelJoints_;

  // Ordered list of joint names describing the robot state.
  std::vector<std::string> stateVectorMap_;

 public:
  SotLoaderBasic(const std::string& aNodeName = std::string("SotLoaderBasic"));
  virtual ~SotLoaderBasic();

  // \brief Read user input to extract the path of the SoT dynamic library.
  int parseOptions(int argc, char* argv[]);

  /// \brief Load the SoT device corresponding to the robot.
  void loadController();

  // Initialize ROS Context
  void initialize();

  // Returns nodeHandle
  rclcpp::Node::SharedPtr returnsNodeHandle();

  /// \brief Unload the library which handles the robot device.
  void CleanUp();

  // \brief Create ROS services start_dg and stop_dg.
  virtual void initializeServices();

  // \brief Callback function when starting dynamic graph.
  void start_dg(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // \brief Callback function when stopping dynamic graph.
  void stop_dg(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
               std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // \brief Read the state vector description based upon the robot links.
  int readSotVectorStateParam();

  // \brief Init publication of joint states.
  int initPublication();

  // \brief Get Status of dg.
  bool isDynamicGraphStopped() { return dynamic_graph_stopped_; }

  // \brief Specify the name of the dynamic library.
  void setDynamicLibraryName(std::string& afilename);

  // \brief Read the parameters of the node
  bool parameter_server_read_robot_description();
};

#endif /* _SOT_LOADER_BASIC_HH_ */
