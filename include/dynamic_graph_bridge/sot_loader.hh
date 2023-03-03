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

#ifndef _SOT_LOADER_HH_
#define _SOT_LOADER_HH_

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
#include <boost/thread/thread.hpp>

// ROS includes
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ros/ros.h"
#include "std_srvs/Empty.h"

// Sot Framework includes
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/debug.hh>

// Dynamic-graph-bridge includes.
#include <dynamic_graph_bridge/sot_loader_basic.hh>

namespace po = boost::program_options;
namespace dgs = dynamicgraph::sot;

class SotLoader : public SotLoaderBasic {
 protected:
  /// Map of sensor readings
  std::map<std::string, dgs::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string, dgs::ControlValues> controlValues_;

  /// Angular values read by encoders
  std::vector<double> angleEncoder_;
  /// Angular values sent to motors
  std::vector<double> angleControl_;
  /// Forces read by force sensors
  std::vector<double> forces_;
  /// Torques
  std::vector<double> torques_;
  /// Attitude of the robot computed by extended Kalman filter.
  std::vector<double> baseAtt_;
  /// Accelerations read by Accelerometers
  std::vector<double> accelerometer_;
  /// Angular velocity read by gyrometers
  std::vector<double> gyrometer_;

  /// URDF string description of the robot.
  std::string robot_desc_string_;

  /// The thread running dynamic graph
  boost::thread thread_;

  // \brief Start control loop
  virtual void startControlLoop();

  // Robot Pose Publisher
  tf2_ros::TransformBroadcaster freeFlyerPublisher_;
  geometry_msgs::TransformStamped freeFlyerPose_;

 public:
  SotLoader();
  ~SotLoader();

  // \brief Create a thread for ROS and start the control loop.
  void initializeRosNode(int argc, char *argv[]);

  // \brief Compute one iteration of control.
  // Basically calls fillSensors, the SoT and the readControl.
  void oneIteration(const double &period);

  // \brief Fill the sensors value for the SoT.
  void fillSensors(std::map<std::string, dgs::SensorValues> &sensorsIn);

  // \brief Read the control computed by the SoT framework.
  void readControl(std::map<std::string, dgs::ControlValues> &controlValues);

  // \brief Prepare the SoT framework.
  void setup();
};

#endif /* SOT_LOADER_HH_ */
