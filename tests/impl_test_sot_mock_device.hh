/*
 * Copyright 2023, LAAS, CNRS
 *
 * Author:Olivier Stasse
 *
 * This file is part of dynamic_graph_bridge.
 * This file is under the APACHE license 2.0
 *
 */
#ifndef _Impl_Test_SoT_Mock_Device_H_
#define _Impl_Test_SoT_Mock_Device_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal.h>

#include <dynamic-graph/fwd.hh>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/device.hh>
#include <sot/core/matrix-geometry.hh>

namespace dgsot = dynamicgraph::sot;
namespace dg = dynamicgraph;

class ImplTestSotMockDevice : public dgsot::Device {
 public:
  static const std::string CLASS_NAME;
  static const double TIMESTEP_DEFAULT;

  virtual const std::string &getClassName() const { return CLASS_NAME; }

  ImplTestSotMockDevice(std::string RobotName);
  virtual ~ImplTestSotMockDevice();

  void setSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void setupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut,
                  const double &period);

  void timeStep(double ts) { timestep_ = ts; }

 protected:
  /// \brief Previous robot configuration.
  dg::Vector previousState_;

  /// Intermediate variables to avoid allocation during control
  std::vector<double> baseff_;

  /// Accelerations measured by accelerometers
  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> accelerometerSOUT_;
  /// Rotation velocity measured by gyrometers
  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> gyrometerSOUT_;
  /// motor currents
  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> currentsSOUT_;
  /// joint angles
  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> joint_anglesSOUT_;
  /// motor angles
  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> motor_anglesSOUT_;

  /// proportional and derivative position-control gains
  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> p_gainsSOUT_;

  dynamicgraph::Signal<dg::Vector, dg::sigtime_t> d_gainsSOUT_;

  /// Protected methods for internal variables filling
  void setSensorsForce(std::map<std::string, dgsot::SensorValues> &SensorsIn,
                       dg::sigtime_t t);
  void setSensorsIMU(std::map<std::string, dgsot::SensorValues> &SensorsIn,
                     dg::sigtime_t t);
  void setSensorsEncoders(std::map<std::string, dgsot::SensorValues> &SensorsIn,
                          dg::sigtime_t t);
  void setSensorsVelocities(
      std::map<std::string, dgsot::SensorValues> &SensorsIn, dg::sigtime_t t);
  void setSensorsTorquesCurrents(
      std::map<std::string, dgsot::SensorValues> &SensorsIn, dg::sigtime_t t);
  void setSensorsGains(std::map<std::string, dgsot::SensorValues> &SensorsIn,
                       dg::sigtime_t t);

  /// Intermediate variables to avoid allocation during control
  dg::Vector dgforces_;
  dg::Vector dgRobotState_;     // motor-angles
  dg::Vector joint_angles_;     // joint-angles
  dg::Vector motor_angles_;     // motor-angles
  dg::Vector dgRobotVelocity_;  // motor velocities
  dg::Vector velocities_;       // motor velocities
  dgsot::MatrixRotation pose;
  dg::Vector accelerometer_;
  dg::Vector gyrometer_;
  dg::Vector torques_;
  dg::Vector currents_;
  dg::Vector p_gains_;
  dg::Vector d_gains_;
};
#endif /* _Impl_Test_SoT_Mock_Device_H_*/
