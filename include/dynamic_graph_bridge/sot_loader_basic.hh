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

// Pinocchio includes
#include <pinocchio/fwd.hpp>

// Boost includes
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

// ROS includes
#include <sensor_msgs/JointState.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"

// Sot Framework includes
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/debug.hh>

namespace po = boost::program_options;
namespace dgs = dynamicgraph::sot;

class SotLoaderBasic {
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
  XmlRpc::XmlRpcValue stateVectorMap_;

  /// \brief List of parallel joints from the state vector.
  typedef std::vector<int> parallel_joints_to_state_vector_t;
  parallel_joints_to_state_vector_t parallel_joints_to_state_vector_;

  /// \brief Coefficient between parallel joints and the state vector.
  std::vector<double> coefficient_parallel_joints_;
  /// Advertises start_dynamic_graph services
  ros::ServiceServer service_start_;

  /// Advertises stop_dynamic_graph services
  ros::ServiceServer service_stop_;

  // Joint state publication.
  ros::Publisher joint_pub_;

  // Joint state to be published.
  sensor_msgs::JointState joint_state_;

  // Number of DOFs according to KDL.
  int nbOfJoints_;
  parallel_joints_to_state_vector_t::size_type nbOfParallelJoints_;

 public:
  SotLoaderBasic();
  ~SotLoaderBasic() {};

  // \brief Read user input to extract the path of the SoT dynamic library.
  int parseOptions(int argc, char* argv[]);

  /// \brief Load the SoT device corresponding to the robot.
  void Initialization();

  /// \brief Unload the library which handles the robot device.
  void CleanUp();

  // \brief Create a thread for ROS.
  virtual void initializeRosNode(int argc, char* argv[]);

  // \brief Callback function when starting dynamic graph.
  bool start_dg(std_srvs::Empty::Request& request,
                std_srvs::Empty::Response& response);

  // \brief Callback function when stopping dynamic graph.
  bool stop_dg(std_srvs::Empty::Request& request,
               std_srvs::Empty::Response& response);

  // \brief Read the state vector description based upon the robot links.
  int readSotVectorStateParam();

  // \brief Init publication of joint states.
  int initPublication();

  // \brief Get Status of dg.
  bool isDynamicGraphStopped() { return dynamic_graph_stopped_; }

  // \brief Specify the name of the dynamic library.
  void setDynamicLibraryName(std::string& afilename);
};

#endif /* _SOT_LOADER_BASIC_HH_ */
