/*
 * Copyright 2023, LAAS, CNRS
 *
 * Author:Olivier Stasse
 *
 * This file is part of dynamic_graph_bridge.
 * This file is under the APACHE license 2.0
 *
 */
#include <fstream>
#include <map>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/impl_test_sot_mock_device.txt"

#if 0
#define RESETDEBUG5()                            \
  {                                              \
    std::ofstream DebugFile;                     \
    DebugFile.open(DBGFILE, std::ofstream::out); \
    DebugFile.close();                           \
  }
#define ODEBUG5FULL(x)                                               \
  {                                                                  \
    std::ofstream DebugFile;                                         \
    DebugFile.open(DBGFILE, std::ofstream::app);                     \
    DebugFile << __FILE__ << ":" << __FUNCTION__ << "(#" << __LINE__ \
              << "):" << x << std::endl;                             \
    DebugFile.close();                                               \
  }
#define ODEBUG5(x)                               \
  {                                              \
    std::ofstream DebugFile;                     \
    DebugFile.open(DBGFILE, std::ofstream::app); \
    DebugFile << x << std::endl;                 \
    DebugFile.close();                           \
  }

#else
// Void the macro
#define RESETDEBUG5()
#define ODEBUG5FULL(x)
#define ODEBUG5(x)
#endif

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>

#include "impl_test_sot_mock_device.hh"

using namespace std;

const double ImplTestSotMockDevice::TIMESTEP_DEFAULT = 0.001;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImplTestSotMockDevice,
                                   "ImplTestSotMockDevice");

ImplTestSotMockDevice::ImplTestSotMockDevice(std::string RobotName)
    : dgsot::Device(RobotName),
      previousState_(),
      baseff_(),
      accelerometerSOUT_("Device(" + RobotName +
                         ")::output(vector)::accelerometer"),
      gyrometerSOUT_("Device(" + RobotName + ")::output(vector)::gyrometer"),
      currentsSOUT_("Device(" + RobotName + ")::output(vector)::currents"),
      joint_anglesSOUT_("Device(" + RobotName +
                        ")::output(vector)::joint_angles"),
      motor_anglesSOUT_("Device(" + RobotName +
                        ")::output(vector)::motor_angles"),
      p_gainsSOUT_("Device(" + RobotName + ")::output(vector)::p_gains"),
      d_gainsSOUT_("Device(" + RobotName + ")::output(vector)::d_gains"),
      dgforces_(6),
      accelerometer_(3),
      gyrometer_(3) {
  RESETDEBUG5();
  timestep_ = TIMESTEP_DEFAULT;
  sotDEBUGIN(25);

  for (int i = 0; i < 4; ++i) {
    withForceSignals[i] = true;
  }
  signalRegistration(accelerometerSOUT_
                     << gyrometerSOUT_ << currentsSOUT_ << joint_anglesSOUT_
                     << motor_anglesSOUT_ << p_gainsSOUT_ << d_gainsSOUT_);
  dg::Vector data(3);
  data.setZero();
  accelerometerSOUT_.setConstant(data);
  gyrometerSOUT_.setConstant(data);
  baseff_.resize(7);
  dg::Vector dataForces(6);
  dataForces.setZero();
  for (int i = 0; i < 4; i++) forcesSOUT[i]->setConstant(dataForces);

  sotDEBUGOUT(25);
}

ImplTestSotMockDevice::~ImplTestSotMockDevice() {}

void ImplTestSotMockDevice::setSensorsForce(
    map<string, dgsot::SensorValues>& SensorsIn, dg::sigtime_t t) {
  int map_sot_2_urdf[4] = {2, 0, 3, 1};
  sotDEBUGIN(15);
  map<string, dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("forces");
  if (it != SensorsIn.end()) {
    // Implements force recollection.
    const vector<double>& forcesIn = it->second.getValues();
    for (int i = 0; i < 4; ++i) {
      sotDEBUG(15) << "Force sensor " << i << std::endl;
      int idx_sensor = map_sot_2_urdf[i];
      for (std::vector<double>::size_type j = 0; j < 6; ++j) {
        dgforces_(static_cast<Eigen::Index>(j)) =
            forcesIn[static_cast<std::vector<double>::size_type>(idx_sensor * 6) + j];
        sotDEBUG(15) << "Force value " << j << ":"
                     << dgforces_(static_cast<Eigen::Index>(j)) << std::endl;
      }
      forcesSOUT[i]->setConstant(dgforces_);
      forcesSOUT[i]->setTime(t);
    }
  }
  sotDEBUGIN(15);
}

void ImplTestSotMockDevice::setSensorsIMU(
    map<string, dgsot::SensorValues>& SensorsIn, dg::sigtime_t t) {
  map<string, dgsot::SensorValues>::iterator it;
  // TODO: Confirm if this can be made quaternion
  it = SensorsIn.find("attitude");
  if (it != SensorsIn.end()) {
    const vector<double>& attitude = it->second.getValues();
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j) pose(i, j) = attitude[i * 3 + j];
    attitudeSOUT.setConstant(pose);
    attitudeSOUT.setTime(t);
  }

  it = SensorsIn.find("accelerometer_0");
  if (it != SensorsIn.end()) {
    const vector<double>& accelerometer =
        SensorsIn["accelerometer_0"].getValues();
    for (std::size_t i = 0; i < 3; ++i)
      accelerometer_(static_cast<Eigen::Index>(i)) = accelerometer[i];
    accelerometerSOUT_.setConstant(accelerometer_);
    accelerometerSOUT_.setTime(t);
  }

  it = SensorsIn.find("gyrometer_0");
  if (it != SensorsIn.end()) {
    const vector<double>& gyrometer = SensorsIn["gyrometer_0"].getValues();
    for (std::size_t i = 0; i < 3; ++i)
      gyrometer_(static_cast<Eigen::Index>(i)) = gyrometer[i];
    gyrometerSOUT_.setConstant(gyrometer_);
    gyrometerSOUT_.setTime(t);
  }
}

void ImplTestSotMockDevice::setSensorsEncoders(
    map<string, dgsot::SensorValues>& SensorsIn, dg::sigtime_t t) {
  map<string, dgsot::SensorValues>::iterator it;

  it = SensorsIn.find("motor-angles");
  if (it != SensorsIn.end()) {
    const vector<double>& anglesIn = it->second.getValues();
    dgRobotState_.resize(static_cast<Eigen::Index>(anglesIn.size()) + 6);
    motor_angles_.resize(static_cast<Eigen::Index>(anglesIn.size()));
    for (unsigned i = 0; i < 6; ++i) dgRobotState_(i) = 0.;
    for (unsigned i = 0; i < anglesIn.size(); ++i) {
      dgRobotState_(i + 6) = anglesIn[i];
      motor_angles_(i) = anglesIn[i];
    }
    robotState_.setConstant(dgRobotState_);
    robotState_.setTime(t);
    motor_anglesSOUT_.setConstant(motor_angles_);
    motor_anglesSOUT_.setTime(t);
  }

  it = SensorsIn.find("joint-angles");
  if (it != SensorsIn.end()) {
    const vector<double>& joint_anglesIn = it->second.getValues();
    joint_angles_.resize(static_cast<Eigen::Index>(joint_anglesIn.size()));
    for (unsigned i = 0; i < joint_anglesIn.size(); ++i)
      joint_angles_(i) = joint_anglesIn[i];
    joint_anglesSOUT_.setConstant(joint_angles_);
    joint_anglesSOUT_.setTime(t);
  }
}

void ImplTestSotMockDevice::setSensorsVelocities(
    map<string, dgsot::SensorValues>& SensorsIn, dg::sigtime_t t) {
  map<string, dgsot::SensorValues>::iterator it;

  it = SensorsIn.find("velocities");
  if (it != SensorsIn.end()) {
    const vector<double>& velocitiesIn = it->second.getValues();
    dgRobotVelocity_.resize(static_cast<Eigen::Index>(velocitiesIn.size()) + 6);
    for (unsigned i = 0; i < 6; ++i) dgRobotVelocity_(i) = 0.;
    for (unsigned i = 0; i < static_cast<Eigen::Index>(velocitiesIn.size()); ++i) {
      dgRobotVelocity_(i + 6) = velocitiesIn[i];
    }
    robotVelocity_.setConstant(dgRobotVelocity_);
    robotVelocity_.setTime(t);
  }
}

void ImplTestSotMockDevice::setSensorsTorquesCurrents(
    map<string, dgsot::SensorValues>& SensorsIn, dg::sigtime_t t) {
  map<string, dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("torques");
  if (it != SensorsIn.end()) {
    const std::vector<double>& torques = SensorsIn["torques"].getValues();
    torques_.resize(static_cast<Eigen::Index>(torques.size()));
    for (std::size_t i = 0; i < torques.size(); ++i)
      torques_(static_cast<Eigen::Index>(i)) = torques[i];
    pseudoTorqueSOUT.setConstant(torques_);
    pseudoTorqueSOUT.setTime(t);
  }

  it = SensorsIn.find("currents");
  if (it != SensorsIn.end()) {
    const std::vector<double>& currents = SensorsIn["currents"].getValues();
    currents_.resize(static_cast<Eigen::Index>(currents.size()));
    for (std::size_t i = 0; i < currents.size(); ++i)
      currents_(static_cast<Eigen::Index>(i)) = currents[i];
    currentsSOUT_.setConstant(currents_);
    currentsSOUT_.setTime(t);
  }
}

void ImplTestSotMockDevice::setSensorsGains(
    map<string, dgsot::SensorValues>& SensorsIn, dg::sigtime_t t) {
  map<string, dgsot::SensorValues>::iterator it;
  it = SensorsIn.find("p_gains");
  if (it != SensorsIn.end()) {
    const std::vector<double>& p_gains = SensorsIn["p_gains"].getValues();
    p_gains_.resize(static_cast<Eigen::Index>(p_gains.size()));
    for (std::size_t i = 0; i < p_gains.size(); ++i)
      p_gains_(static_cast<Eigen::Index>(i)) = p_gains[i];
    p_gainsSOUT_.setConstant(p_gains_);
    p_gainsSOUT_.setTime(t);
  }

  it = SensorsIn.find("d_gains");
  if (it != SensorsIn.end()) {
    const std::vector<double>& d_gains = SensorsIn["d_gains"].getValues();
    d_gains_.resize(static_cast<Eigen::Index>(d_gains.size()));
    for (std::size_t i = 0; i < d_gains.size(); ++i)
      d_gains_(static_cast<Eigen::Index>(i)) = d_gains[i];
    d_gainsSOUT_.setConstant(d_gains_);
    d_gainsSOUT_.setTime(t);
  }
}

void ImplTestSotMockDevice::setSensors(
    map<string, dgsot::SensorValues>& SensorsIn) {
  sotDEBUGIN(25);
  map<string, dgsot::SensorValues>::iterator it;
  dg::sigtime_t t = stateSOUT.getTime() + 1;

  setSensorsForce(SensorsIn, t);
  setSensorsIMU(SensorsIn, t);
  setSensorsEncoders(SensorsIn, t);
  setSensorsVelocities(SensorsIn, t);
  setSensorsTorquesCurrents(SensorsIn, t);
  setSensorsGains(SensorsIn, t);

  sotDEBUGOUT(25);
}

void ImplTestSotMockDevice::setupSetSensors(
    map<string, dgsot::SensorValues>& SensorsIn) {
  // The first time we read the sensors, we need to copy the state of the
  // robot into the signal device.state.
  setSensors(SensorsIn);
  setState(robotState_(0));
}

void ImplTestSotMockDevice::nominalSetSensors(
    map<string, dgsot::SensorValues>& SensorsIn) {
  setSensors(SensorsIn);
}

void ImplTestSotMockDevice::cleanupSetSensors(
    map<string, dgsot::SensorValues>& SensorsIn) {
  setSensors(SensorsIn);
}

void ImplTestSotMockDevice::getControl(
    map<string, dgsot::ControlValues>& controlOut, const double&) {
  ODEBUG5FULL("start");
  sotDEBUGIN(25);
  vector<double> anglesOut, velocityOut;
  anglesOut.resize(static_cast<std::vector<double>::size_type>(state_.size()));
  velocityOut.resize(static_cast<std::vector<double>::size_type>(state_.size()));

  // Integrate control
  sotDEBUG(25) << "state = " << state_ << std::endl;
  sotDEBUG(25) << "diff  = "
               << ((previousState_.size() == state_.size())
                       ? (state_ - previousState_)
                       : state_)
               << std::endl;
  ODEBUG5FULL("state = " << state_);
  ODEBUG5FULL("diff  = " << ((previousState_.size() == state_.size())
                                 ? (state_ - previousState_)
                                 : state_));
  previousState_ = state_;

  // Specify the joint values for the controller.
  // warning: we make here the asumption that the control signal contains the
  // velocity of the freeflyer joint. This may change in the future.
  if ((int)anglesOut.size() != state_.size() - 6) {
    anglesOut.resize(static_cast<std::vector<double>::size_type>(state_.size() - 6));
    velocityOut.resize(static_cast<std::vector<double>::size_type>(state_.size() - 6));
  }

  dg::sigtime_t time = controlSIN.getTime();
  for (unsigned int i = 6; i < state_.size(); ++i) {
    anglesOut[i - 6] = state_(i);
    velocityOut[i - 6] = controlSIN(time)(i);
  }
  // Store in "control" the joint values
  controlOut["control"].setValues(anglesOut);
  // Store in "velocity" the joint velocity values
  controlOut["velocity"].setValues(velocityOut);
  // Read zmp reference from input signal if plugged
  zmpSIN.recompute(time + 1);
  // Express ZMP in free flyer reference frame
  dg::Vector zmpGlobal(4);
  for (unsigned int i = 0; i < 3; ++i) zmpGlobal(i) = zmpSIN(time + 1)(i);
  zmpGlobal(3) = 1.;

  ODEBUG5FULL("end");
  sotDEBUGOUT(25);
}

using namespace dynamicgraph::sot;

namespace dynamicgraph {
namespace sot {
#ifdef WIN32
const char* DebugTrace::DEBUG_FILENAME_DEFAULT = "c:/tmp/sot-core-traces.txt";
#else   // WIN32
const char* DebugTrace::DEBUG_FILENAME_DEFAULT = "/tmp/sot-core-traces.txt";
#endif  // WIN32

#ifdef VP_DEBUG
#ifdef WIN32
std::ofstream debugfile("C:/tmp/sot-core-traces.txt",
                        std::ios::trunc& std::ios::out);
#else   // WIN32
std::ofstream debugfile("/tmp/sot-core-traces.txt",
                        std::ios::trunc& std::ios::out);
#endif  // WIN32
#else   // VP_DEBUG

std::ofstream debugfile;

#endif  // VP_DEBUG

} /* namespace sot */
} /* namespace dynamicgraph */
