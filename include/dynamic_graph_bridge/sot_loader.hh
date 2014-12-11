/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

// System includes
#include <iostream>
#include <cassert>

// STL includes
#include <map>

// Boost includes
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

// ROS includes
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <sensor_msgs/JointState.h>

// Sot Framework includes 
#include <sot/core/debug.hh>
#include <sot/core/abstract-sot-external-interface.hh>

namespace po = boost::program_options;
namespace dgs = dynamicgraph::sot;


class SotLoader {

protected:

  // Dynamic graph is stopped.
  bool dynamic_graph_stopped_;

  /// \brief the sot-hrp2 controller
  dgs::AbstractSotExternalInterface * sotController_;

  po::variables_map vm_;
  std::string dynamicLibraryName_;

  /// Map of sensor readings
  std::map <std::string,dgs::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string,dgs::ControlValues> controlValues_;

  /// Angular values read by encoders
  std::vector <double> angleEncoder_;
  /// Angular values sent to motors
  std::vector<double> angleControl_;
  /// Forces read by force sensors
  std::vector<double> forces_;
  /// Torques
  std::vector<double> torques_;
  /// Attitude of the robot computed by extended Kalman filter.
  std::vector<double> baseAtt_;
  /// Accelerations read by Accelerometers
  std::vector <double> accelerometer_;
  /// Angular velocity read by gyrometers
  std::vector <double> gyrometer_;

  /// URDF string description of the robot.
  std::string robot_desc_string_;
  
  /// \brief Map between SoT state vector and some joint_state_links
  XmlRpc::XmlRpcValue stateVectorMap_;

  /// \brief List of parallel joints from the state vector.
  std::vector<int> parallel_joints_to_state_vector_;

  /// \brief Coefficient between parallel joints and the state vector.
  std::vector<double> coefficient_parallel_joints_;

  // Joint state publication.
  ros::Publisher joint_pub_;
  
  // Joint state to be published.
  sensor_msgs::JointState joint_state_;

  // \brief Start control loop.
  virtual void startControlLoop();

  // Number of DOFs according to KDL.
  int nbOfJoints_;
  int nbOfParallelJoints_;


public:
  SotLoader(int argc, char *argv[]);
  ~SotLoader() {};

  // \brief Read user input to extract the path of the SoT dynamic library.
  int parseOptions(int argc, char *argv[]);

  // \brief Load the SoT
  void Initialization();

  // \brief Create a thread for ros
  void initializeRosNode(int argc, char *argv[]);

  // \brief Compute one iteration of control.
  // Basically calls fillSensors, the SoT and the readControl.
  void oneIteration();

  // \brief Fill the sensors value for the SoT.
  void fillSensors(std::map<std::string, 
                   dgs::SensorValues> & sensorsIn);

  // \brief Read the control computed by the SoT framework.
  void readControl(std::map<std::string, 
                   dgs::ControlValues> & controlValues);

  // \brief Prepare the SoT framework.
  void setup();

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
  bool isDynamicGraphStopped()
  { return dynamic_graph_stopped_; }

};

