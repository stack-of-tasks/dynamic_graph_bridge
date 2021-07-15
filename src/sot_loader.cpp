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

#include <dynamic_graph_bridge/sot_loader.hh>
#include "dynamic_graph_bridge/ros2_init.hh"

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
    : N(N_)
    , idx(0)
    , iter(0)
    , iters(N, 0)
    , times(N, 0)
    , ittimes(N, 0)
  {}

  void record(const double t, const double itt) {
    iters  [idx] = iter;
    times  [idx] = t;
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
        aof
          << iters  [(idx + k) % N] << ' '
          << times  [(idx + k) % N] << ' '
          << ittimes[(idx + k) % N] << '\n';
      }
    }
    aof.close();
  }
};

void workThreadLoader(SotLoader::SharedPtr aSotLoader) {
  double periodd;
  std::shared_ptr<rclcpp::Node> nh = aSotLoader->returnsNodeHandle();

  /// Declare parameters
  if (not nh->has_parameter("dt"))
    nh->declare_parameter<double>("dt",0.01);
  if (not nh->has_parameter("paused"))
    nh->declare_parameter<bool>("paused",false);
  
  // 
  nh->get_parameter_or("dt",periodd,0.001);
  rclcpp::Rate rate(1/periodd); // 1 kHz
  
  DataToLog dataToLog(5000);

  rate.reset();
  while (rclcpp::ok() && aSotLoader->isDynamicGraphStopped()) {
    rate.sleep();
  }

  bool paused;
  rclcpp::Clock aClock;
  rclcpp::Time timeOrigin = aClock.now();
  rclcpp::Time time;
  while (rclcpp::ok() && !aSotLoader->isDynamicGraphStopped()) {
    
    // Check if the user did not paused geometric_simu
    nh->get_parameter_or("paused", paused, false);

    if (!paused) {
      time = aClock.now();
      aSotLoader->oneIteration();

      rclcpp::Duration d = aClock.now() - time;
      dataToLog.record((time - timeOrigin).nanoseconds()*1.0e9, d.nanoseconds()*1.0e9);
    }
    rate.sleep();
  }
  dataToLog.save("/tmp/geometric_simu");
  rclcpp::spin(nh);
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
  if (not nh_->has_parameter("tf_base_link"))
    nh_->declare_parameter("tf_base_link",std::string("base_link"));
  
  if (nh_->get_parameter("tf_base_link",
                      freeFlyerPose_.child_frame_id)) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dynamic_graph_bridge"),
                       "Publishing " << freeFlyerPose_.child_frame_id <<
                       " wrt " << freeFlyerPose_.header.frame_id);
  }
}

SotLoader::~SotLoader() {
  dynamic_graph_stopped_ = true;
  thread_.join();
}

void SotLoader::startControlLoop() { thread_ = std::thread(workThreadLoader,
                                                           std::shared_ptr<SotLoader>(this)); }

void SotLoader::initializeServices() {
  SotLoaderBasic::initializeServices();
  // Temporary fix. TODO: where should nbOfJoints_ be initialized from?
  
  angleEncoder_.resize(static_cast<std::size_t>(nbOfJoints_));
  angleControl_.resize(static_cast<std::size_t>(nbOfJoints_));

  startControlLoop();
}

void SotLoader::fillSensors(map<string, dgs::SensorValues> &sensorsIn) {
  // Update joint values.w
  assert(angleControl_.size() == angleEncoder_.size());

  sensorsIn["joints"].setName("angle");
  for (unsigned int i = 0; i < angleControl_.size(); i++) angleEncoder_[i] = angleControl_[i];
  sensorsIn["joints"].setValues(angleEncoder_);
}

void SotLoader::readControl(map<string, dgs::ControlValues> &controlValues) {
  // Update joint values.
  angleControl_ = controlValues["control"].getValues();

  // Debug
  std::map<std::string, dgs::ControlValues>::iterator it = controlValues.begin();
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
    std::cerr << " angleControl_" << angleControl_.size() << " and nbOfJoints" << (unsigned int)nbOfJoints_
              << " are different !" << std::endl;
    exit(-1);
  }
  // Publish the data.
  joint_state_.header.stamp = rclcpp::Clock().now();
  for (int i = 0; i < nbOfJoints_; i++) {
    joint_state_.position[i] = angleControl_[i];
  }
  for (unsigned int i = 0; i < parallel_joints_to_state_vector_.size(); i++) {
    joint_state_.position[i + nbOfJoints_] =
        coefficient_parallel_joints_[i] * angleControl_[parallel_joints_to_state_vector_[i]];
  }

  joint_pub_->publish(joint_state_);

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
  freeFlyerPublisher_->sendTransform(freeFlyerPose_);
}

void SotLoader::setup() {
  fillSensors(sensorsIn_);
  sotController_->setupSetSensors(sensorsIn_);
  sotController_->getControl(controlValues_);
  readControl(controlValues_);
}

void SotLoader::oneIteration() {
  fillSensors(sensorsIn_);
  try {
    sotController_->nominalSetSensors(sensorsIn_);
    sotController_->getControl(controlValues_);
  } catch (std::exception &) {
    throw;
  }

  readControl(controlValues_);
}
