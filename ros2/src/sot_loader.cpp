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
#include "dynamic_graph_bridge/ros.hpp"
#include <std_msgs/msg/string.hpp>

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


SotLoader::SotLoader()
    : SotLoaderBasic(),
      sensorsIn_(),
      controlValues_(),
      angleEncoder_(),
      control_(),
      forces_(),
      torques_(),
      baseAtt_(),
      accelerometer_(3),
      gyrometer_(3) {
}


SotLoader::~SotLoader() {
  dynamic_graph_stopped_ = true;
  lthread_join();
}

void SotLoader::startControlLoop() {
  thread_ = std::make_shared<std::thread>(&SotLoader::workThreadLoader,this);
}

void SotLoader::initializeServices() {
  SotLoaderBasic::initializeServices();

  freeFlyerPose_.header.frame_id = "odom";
  freeFlyerPose_.child_frame_id = "base_link";

  if (nh_==0) {
    logic_error aLogicError("SotLoaderBasic::initializeFromRosContext aRosCtxt is empty !");
    throw aLogicError;
  }

  if (not nh_->has_parameter("tf_base_link"))
    nh_->declare_parameter("tf_base_link",std::string("base_link"));

  if (nh_->get_parameter("tf_base_link",
                      freeFlyerPose_.child_frame_id)) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dynamic_graph_bridge"),
                       "Publishing " << freeFlyerPose_.child_frame_id <<
                       " wrt " << freeFlyerPose_.header.frame_id);
  }

  // Temporary fix. TODO: where should nbOfJoints_ be initialized from?

  angleEncoder_.resize(static_cast<std::size_t>(nbOfJoints_));
  control_.resize(static_cast<std::size_t>(nbOfJoints_));

  // Creates a publisher for the free flyer transform.
  freeFlyerPublisher_ = std::make_shared<tf2_ros::TransformBroadcaster>
    (nh_);
}

void SotLoader::fillSensors(map<string, dgs::SensorValues> &sensorsIn) {
  // Update joint values.w
  assert(control_.size() == angleEncoder_.size());

  sensorsIn["joints"].setName("angle");
  for (unsigned int i = 0; i < control_.size(); i++) angleEncoder_[i] = control_[i];
  sensorsIn["joints"].setValues(angleEncoder_);
}

void SotLoader::readControl(map<string, dgs::ControlValues> &controlValues) {
  // Update joint values.
  std::map<std::string, dgs::ControlValues>::iterator it_ctrl_map;
  it_ctrl_map = controlValues.find("control");
  if (it_ctrl_map == controlValues.end())
    {
      invalid_argument anInvalidArgument("control is not present in controlValues !");
      throw anInvalidArgument;
    }
  control_ = controlValues["control"].getValues();

#ifdef NDEBUG
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
#endif
  // Check if the size if coherent with the robot description.
  if (control_.size() != (unsigned int)nbOfJoints_) {
    nbOfJoints_ = control_.size();
    angleEncoder_.resize(nbOfJoints_);
  }

  // Publish the data.
  std::vector<string> joint_state_names;
  joint_state_.name = joint_state_names;
  joint_state_.header.stamp = rclcpp::Clock().now();

  if (joint_state_.position.size()!=nbOfJoints_+parallel_joints_to_state_vector_.size())
    {
      joint_state_.position.resize(nbOfJoints_+parallel_joints_to_state_vector_.size());
      joint_state_.velocity.resize(nbOfJoints_+parallel_joints_to_state_vector_.size());
      joint_state_.effort.resize(nbOfJoints_+parallel_joints_to_state_vector_.size());
    }

  for (int i = 0; i < nbOfJoints_; i++) {
    joint_state_.position[i] = angleEncoder_[i];
  }

  std::cerr << "parallel_joints_to_state_vector_.size(): "
            << parallel_joints_to_state_vector_.size()
            << std::endl;

  for (unsigned int i = 0; i < parallel_joints_to_state_vector_.size(); i++) {
    joint_state_.position[i + nbOfJoints_] =
        coefficient_parallel_joints_[i] * angleEncoder_[parallel_joints_to_state_vector_[i]];
  }

  joint_pub_->publish(joint_state_);

  // Get the estimation of the robot base.
  it_ctrl_map = controlValues.find("baseff");
  if (it_ctrl_map == controlValues.end())
    {
      invalid_argument anInvalidArgument("baseff is not present in controlValues !");
      throw anInvalidArgument;
    }

  std::cerr << "Reached poseValue_" << std::endl;
  std::vector<double> poseValue = controlValues["baseff"].getValues();

  freeFlyerPose_.transform.translation.x = poseValue[0];
  freeFlyerPose_.transform.translation.y = poseValue[1];
  freeFlyerPose_.transform.translation.z = poseValue[2];

  freeFlyerPose_.transform.rotation.w = poseValue[3];
  freeFlyerPose_.transform.rotation.x = poseValue[4];
  freeFlyerPose_.transform.rotation.y = poseValue[5];
  freeFlyerPose_.transform.rotation.z = poseValue[6];

  freeFlyerPose_.header.stamp = joint_state_.header.stamp;
  // Publish
  freeFlyerPublisher_->sendTransform(freeFlyerPose_);
  std::cerr << "end of readControl" << std::endl;
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

void SotLoader::lthread_join()  {
  if (thread_!=0)
    if (thread_->joinable())
      thread_->join();
}

void SotLoader::workThreadLoader() {
  double periodd;

  /// Declare parameters
  if (not nh_->has_parameter("dt"))
    nh_->declare_parameter<double>("dt",0.01);
  if (not nh_->has_parameter("paused"))
    nh_->declare_parameter<bool>("paused",false);

  //
  nh_->get_parameter_or("dt",periodd,0.001);
  rclcpp::Rate rate(1/periodd); // 1 kHz

  DataToLog dataToLog(5000);

  rate.reset();
  while (rclcpp::ok() && isDynamicGraphStopped()) {
    rate.sleep();
  }

  bool paused;
  rclcpp::Clock aClock;
  rclcpp::Time timeOrigin = aClock.now();
  rclcpp::Time time;
  while (rclcpp::ok() && !isDynamicGraphStopped()) {

    // Check if the user did not paused geometric_simu
    nh_->get_parameter_or("paused", paused, false);

    if (!paused) {
      time = aClock.now();
      oneIteration();

      rclcpp::Duration d = aClock.now() - time;
      dataToLog.record((time - timeOrigin).nanoseconds()*1.0e9, d.nanoseconds()*1.0e9);
    }
    rate.sleep();
  }
  dataToLog.save("/tmp/geometric_simu");
  std::cerr << "End of this thread: "
            << std::this_thread::get_id()
            << std::endl;
    
}
