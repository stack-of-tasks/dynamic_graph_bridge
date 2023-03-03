/*
 * Copyright 2011,
 * Olivier Stasse,
 *
 * CNRS
 *
 */
/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <ros/rate.h>

#include <dynamic_graph_bridge/sot_loader.hh>

#include "dynamic_graph_bridge/ros_init.hh"

// POSIX.1-2001
#include <dlfcn.h>

using namespace std;
using namespace dynamicgraph::sot;
namespace po = boost::program_options;

struct DataToLog {
  const std::size_t N;
  std::size_t idx, iter;

  std::vector<std::size_t> iters;
  std::vector<double> times;
  std::vector<double> ittimes;

  DataToLog(std::size_t N_)
      : N(N_), idx(0), iter(0), iters(N, 0), times(N, 0), ittimes(N, 0) {}

  void record(const double t, const double itt) {
    iters[idx] = iter;
    times[idx] = t;
    ittimes[idx] = itt;
    ++idx;
    ++iter;
    if (idx == N) idx = 0;
  }

  void save(const char *prefix) {
    std::ostringstream oss;
    oss << prefix << "-times.log";

    std::ofstream aof(oss.str().c_str());
    if (aof.is_open()) {
      for (std::size_t k = 0; k < N; ++k) {
        aof << iters[(idx + k) % N] << ' ' << times[(idx + k) % N] << ' '
            << ittimes[(idx + k) % N] << '\n';
      }
    }
    aof.close();
  }
};

void workThreadLoader(SotLoader *aSotLoader) {
  ros::Rate rate(1000);  // 1 kHz
  double periodd (1e-3);

  if (ros::param::has("/sot_controller/dt")) {
    ros::param::get("/sot_controller/dt", periodd);
    rate = ros::Rate(1 / periodd);
  }
  DataToLog dataToLog(5000);

  rate.reset();
  while (ros::ok() && aSotLoader->isDynamicGraphStopped()) {
    rate.sleep();
  }

  ros::NodeHandle nh("/geometric_simu");
  bool paused;
  ros::Time timeOrigin = ros::Time::now();
  ros::Time time;
  while (ros::ok() && !aSotLoader->isDynamicGraphStopped()) {
    nh.param<bool>("paused", paused, false);

    if (!paused) {
      time = ros::Time::now();
      aSotLoader->oneIteration(periodd);

      ros::Duration d = ros::Time::now() - time;
      dataToLog.record((time - timeOrigin).toSec(), d.toSec());
    }
    rate.sleep();
  }
  dataToLog.save("/tmp/geometric_simu");
  ros::waitForShutdown();
}

SotLoader::SotLoader()
    : sensorsIn_(),
      controlValues_(),
      angleEncoder_(),
      angleControl_(),
      forces_(),
      torques_(),
      baseAtt_(),
      accelerometer_(3),
      gyrometer_(3),
      thread_() {
  readSotVectorStateParam();
  initPublication();

  freeFlyerPose_.header.frame_id = "odom";
  freeFlyerPose_.child_frame_id = "base_link";
  if (ros::param::get("/sot/tf_base_link", freeFlyerPose_.child_frame_id)) {
    ROS_INFO_STREAM("Publishing " << freeFlyerPose_.child_frame_id << " wrt "
                                  << freeFlyerPose_.header.frame_id);
  }
}

SotLoader::~SotLoader() {
  dynamic_graph_stopped_ = true;
  thread_.join();
}

void SotLoader::startControlLoop() {
  thread_ = boost::thread(workThreadLoader, this);
}

void SotLoader::initializeRosNode(int argc, char *argv[]) {
  SotLoaderBasic::initializeRosNode(argc, argv);
  // Temporary fix. TODO: where should nbOfJoints_ be initialized from?
  if (ros::param::has("/sot/state_vector_map")) {
    angleEncoder_.resize(nbOfJoints_);
    angleControl_.resize(nbOfJoints_);
  }

  startControlLoop();
}

void SotLoader::fillSensors(map<string, dgs::SensorValues> &sensorsIn) {
  // Update joint values.w
  assert(angleControl_.size() == angleEncoder_.size());

  sensorsIn["joints"].setName("angle");
  for (unsigned int i = 0; i < angleControl_.size(); i++)
    angleEncoder_[i] = angleControl_[i];
  sensorsIn["joints"].setValues(angleEncoder_);
}

void SotLoader::readControl(map<string, dgs::ControlValues> &controlValues) {
  // Update joint values.
  angleControl_ = controlValues["control"].getValues();

  // Debug
  std::map<std::string, dgs::ControlValues>::iterator it =
      controlValues.begin();
  sotDEBUG(30) << "ControlValues to be broadcasted:" << std::endl;
  for (; it != controlValues.end(); it++) {
    sotDEBUG(30) << it->first << ":";
    std::vector<double> ctrlValues_ = it->second.getValues();
    std::vector<double>::iterator it_d = ctrlValues_.begin();
    for (; it_d != ctrlValues_.end(); it_d++) sotDEBUG(30) << *it_d << " ";
    sotDEBUG(30) << std::endl;
  }
  sotDEBUG(30) << "End ControlValues" << std::endl;

  // Check if the size if coherent with the robot description.
  if (angleControl_.size() != (unsigned int)nbOfJoints_) {
    std::cerr << " angleControl_" << angleControl_.size() << " and nbOfJoints"
              << (unsigned int)nbOfJoints_ << " are different !" << std::endl;
    exit(-1);
  }
  // Publish the data.
  joint_state_.header.stamp = ros::Time::now();
  for (int i = 0; i < nbOfJoints_; i++) {
    joint_state_.position[i] = angleControl_[i];
  }
  for (unsigned int i = 0; i < parallel_joints_to_state_vector_.size(); i++) {
    joint_state_.position[i + nbOfJoints_] =
        coefficient_parallel_joints_[i] *
        angleControl_[parallel_joints_to_state_vector_[i]];
  }

  joint_pub_.publish(joint_state_);

  // Publish robot pose
  // get the robot pose values
  std::vector<double> poseValue_ = controlValues["baseff"].getValues();

  freeFlyerPose_.transform.translation.x = poseValue_[0];
  freeFlyerPose_.transform.translation.y = poseValue_[1];
  freeFlyerPose_.transform.translation.z = poseValue_[2];

  freeFlyerPose_.transform.rotation.w = poseValue_[3];
  freeFlyerPose_.transform.rotation.x = poseValue_[4];
  freeFlyerPose_.transform.rotation.y = poseValue_[5];
  freeFlyerPose_.transform.rotation.z = poseValue_[6];

  freeFlyerPose_.header.stamp = joint_state_.header.stamp;
  // Publish
  freeFlyerPublisher_.sendTransform(freeFlyerPose_);
}

void SotLoader::setup() {
  fillSensors(sensorsIn_);
  sotController_->setupSetSensors(sensorsIn_);
  sotController_->getControl(controlValues_, 0);
  readControl(controlValues_);
}

void SotLoader::oneIteration(const double& period) {
  fillSensors(sensorsIn_);
  try {
    sotController_->nominalSetSensors(sensorsIn_);
    sotController_->getControl(controlValues_, period);
  } catch (std::exception &) {
    throw;
  }

  readControl(controlValues_);
}
