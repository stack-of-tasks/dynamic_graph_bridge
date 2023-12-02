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

// Boost includes
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <thread>

// ROS includes
// #include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include "std_srvs/srv/empty.hpp"

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
  std::vector<double> control_;
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
  std::shared_ptr<std::thread> thread_;

  // \brief Start control loop
  virtual void startControlLoop();

  // Robot Pose Publisher
  std::shared_ptr<tf2_ros::TransformBroadcaster> freeFlyerPublisher_;

  // Free flyer pose
  geometry_msgs::msg::TransformStamped freeFlyerPose_;

 public:
  SotLoader(const std::string &aNodeName=std::string("sot_loader"));
  virtual ~SotLoader();

  // \brief Create a thread for ROS and start the control loop.
  void initializeServices();

  // \brief Compute one iteration of control.
  // Basically calls fillSensors, the SoT and the readControl.
  void oneIteration();

  // \brief Fill the sensors value for the SoT.
  void fillSensors(std::map<std::string, dgs::SensorValues> &sensorsIn);

  // \brief Read the control computed by the SoT framework.
  void readControl(std::map<std::string, dgs::ControlValues> &controlValues);

  // \brief Prepare the SoT framework.
  void setup();

  // \brief Method for the thread implementing the starting and stopping part of
  // dynamic_graph
  void workThreadLoader();

  // \brief Join the thread.
  void lthread_join();
  typedef std::shared_ptr<SotLoader> SharedPtr;
};

#endif /* SOT_LOADER_HH_ */
