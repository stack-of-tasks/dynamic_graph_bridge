/*
 * Copyright 2016,
 * Olivier Stasse,
 *
 * CNRS
 *
 * This file is part of dynamic-graph-bridge.
 * dynamic-graph-bridge is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic-graph-bridge is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic-graph-bridge.  If not, see <http://www.gnu.org/licenses/>.
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#ifndef _SOT_LOADER_HH_
#define _SOT_LOADER_HH_

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
#include <tf/transform_broadcaster.h>

// Sot Framework includes 
#include <sot/core/debug.hh>
#include <sot/core/abstract-sot-external-interface.hh>

// Dynamic-graph-bridge includes.
#include <dynamic_graph_bridge/sot_loader_basic.hh>

namespace po = boost::program_options;
namespace dgs = dynamicgraph::sot;


class SotLoader: public SotLoaderBasic 
{

protected:


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
  

  // \brief Start control loop
  virtual void startControlLoop();


  //Robot Pose Publisher
  tf::TransformBroadcaster freeFlyerPublisher_;
  tf::Transform freeFlyerPose_;

public:
  SotLoader();
  ~SotLoader() {};

  // \brief Create a thread for ROS and start the control loop.
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


};

#endif /* SOT_LOADER_HH_ */
